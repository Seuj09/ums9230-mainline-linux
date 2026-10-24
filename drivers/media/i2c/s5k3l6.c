// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2020-2021 Purism SPC

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define S5K3L6_MCLK_FREQ		(24 * HZ_PER_MHZ)
#define S5K3L6_DATA_LANES		4

/*
 * Link frequencies are the DDR clock on each lane. They are the values the
 * vendor HAL programs through S5K3L6_REG_REQUESTED_LINK_RATE (0x0820, twice
 * the DDR clock in MHz) for the same three modes.
 */
#define S5K3L6_LINK_FREQ_153MHZ		(153ULL * HZ_PER_MHZ)
#define S5K3L6_LINK_FREQ_284MHZ		(284ULL * HZ_PER_MHZ)
#define S5K3L6_LINK_FREQ_600MHZ		(600ULL * HZ_PER_MHZ)

/* The vendor tables run the pixel pipeline at a fixed 480 MHz. */
#define S5K3L6_PIXEL_RATE		(480ULL * HZ_PER_MHZ)

#define S5K3L6_PIXEL_ARRAY_WIDTH	4224
#define S5K3L6_PIXEL_ARRAY_HEIGHT	3136
#define S5K3L6_ACTIVE_LEFT		8
#define S5K3L6_ACTIVE_TOP		8
#define S5K3L6_ACTIVE_WIDTH		4208
#define S5K3L6_ACTIVE_HEIGHT		3120

#define S5K3L6_CHIP_ID		0x30c6
#define S5K3L6_REVISION		0xb0

#define S5K3L6_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K3L6_REG_REVISION		CCI_REG8(0x0002)
#define S5K3L6_REG_MODE_SELECT	CCI_REG8(0x0100)
#define S5K3L6_MODE_STANDBY		0x00
#define S5K3L6_MODE_STREAMING		0x01

#define S5K3L6_REG_LANE_MODE		CCI_REG8(0x0114)
#define S5K3L6_REG_DATA_FORMAT	CCI_REG16(0x0112)
#define S5K3L6_DATA_FORMAT_RAW8	0x0808
#define S5K3L6_DATA_FORMAT_RAW10	0x0a0a

#define S5K3L6_REG_OP_PLL_MULTIPLIER	CCI_REG16(0x030e)
#define S5K3L6_REG_REQUESTED_LINK_RATE	CCI_REG16(0x0820)
#define S5K3L6_REG_FRAME_LENGTH	CCI_REG16(0x0340)
#define S5K3L6_REG_LINE_LENGTH	CCI_REG16(0x0342)
#define S5K3L6_REG_X_ADDR_START	CCI_REG16(0x0344)
#define S5K3L6_REG_Y_ADDR_START	CCI_REG16(0x0346)
#define S5K3L6_REG_X_ADDR_END		CCI_REG16(0x0348)
#define S5K3L6_REG_Y_ADDR_END		CCI_REG16(0x034a)
#define S5K3L6_REG_X_OUTPUT_SIZE	CCI_REG16(0x034c)
#define S5K3L6_REG_Y_OUTPUT_SIZE	CCI_REG16(0x034e)

#define S5K3L6_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K3L6_EXPOSURE_MIN		2
#define S5K3L6_EXPOSURE_MARGIN	2

#define S5K3L6_REG_ANALOG_GAIN	CCI_REG16(0x0204)
#define S5K3L6_ANALOG_GAIN_MIN	0x20
#define S5K3L6_ANALOG_GAIN_MAX	0x200
#define S5K3L6_ANALOG_GAIN_DEFAULT	0x20

#define S5K3L6_REG_DIGITAL_GAIN	CCI_REG16(0x020e)
#define S5K3L6_DIGITAL_GAIN_MIN	0x100
#define S5K3L6_DIGITAL_GAIN_MAX	0x300
#define S5K3L6_DIGITAL_GAIN_DEFAULT	0x100

#define S5K3L6_REG_TEST_PATTERN	CCI_REG8(0x0601)

#define S5K3L6_REG_BPC		CCI_REG8(0x3403)
#define S5K3L6_BPC_FILTER		BIT(0)
#define S5K3L6_BPC_AF_FILTER		BIT(2)

#define S5K3L6_REG_PLL_PD		CCI_REG8(0x3c1e)
#define S5K3L6_REG_PLL_CTRL		CCI_REG16(0x3c1e)
#define S5K3L6_REG_MODE_SELECT16	CCI_REG16(0x0100)
#define S5K3L6_REG_MIPI_CONTINUOUS	CCI_REG8(0x38a1)

#define to_s5k3l6(_sd)		container_of(_sd, struct s5k3l6, sd)

static const s64 s5k3l6_link_freq_menu[] = {
	S5K3L6_LINK_FREQ_153MHZ,
	S5K3L6_LINK_FREQ_284MHZ,
	S5K3L6_LINK_FREQ_600MHZ,
};

static const char *const s5k3l6_supply_names[] = {
	"vddio",
	"vdda",
	"vddd",
};

#define S5K3L6_NUM_SUPPLIES	ARRAY_SIZE(s5k3l6_supply_names)

struct s5k3l6_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k3l6_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hts;
	u32 vts;
	u8 link_freq_index;

	const struct s5k3l6_reg_list reg_list;
};

struct s5k3l6 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	struct regulator_bulk_data supplies[S5K3L6_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	const struct s5k3l6_mode *mode;
};

static const struct cci_reg_sequence s5k3l6_mode_1052x780[] = {
	/* 4x4 binning, 1052x780, from the vendor HAL (mclk 24 MHz, 4 lanes) */
	{ CCI_REG16(0x0344), 0x0008 }, { CCI_REG16(0x0346), 0x0008 }, { CCI_REG16(0x0348), 0x1077 }, { CCI_REG16(0x034a), 0x0c37 },
	{ CCI_REG16(0x034c), 0x041c }, { CCI_REG16(0x034e), 0x030c }, { CCI_REG16(0x0900), 0x0144 }, { CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 }, { CCI_REG16(0x0384), 0x0001 }, { CCI_REG16(0x0386), 0x0007 }, { CCI_REG16(0x0114), 0x0330 },
	{ CCI_REG16(0x0110), 0x0002 }, { CCI_REG16(0x0136), 0x1800 }, { CCI_REG16(0x0304), 0x0004 }, { CCI_REG16(0x0306), 0x0078 },
	{ CCI_REG16(0x3c1e), 0x0000 }, { CCI_REG16(0x030c), 0x0003 }, { CCI_REG16(0x030e), 0x004d }, { CCI_REG16(0x3c16), 0x0002 },
	{ CCI_REG16(0x0300), 0x0006 }, { CCI_REG16(0x0342), 0x1270 }, { CCI_REG16(0x0340), 0x0350 }, { CCI_REG16(0x38c4), 0x0002 },
	{ CCI_REG16(0x38d8), 0x0006 }, { CCI_REG16(0x38da), 0x0003 }, { CCI_REG16(0x38dc), 0x0003 }, { CCI_REG16(0x38c2), 0x0003 },
	{ CCI_REG16(0x38c0), 0x0000 }, { CCI_REG16(0x38d6), 0x0002 }, { CCI_REG16(0x38d4), 0x0002 }, { CCI_REG16(0x38b0), 0x0002 },
	{ CCI_REG16(0x3932), 0x1800 }, { CCI_REG16(0x3938), 0x000c }, { CCI_REG16(0x0820), 0x0132 }, { CCI_REG16(0x380c), 0x0045 },
	{ CCI_REG16(0x3064), 0xffcf }, { CCI_REG16(0x309c), 0x0640 }, { CCI_REG16(0x3090), 0x8000 }, { CCI_REG16(0x3238), 0x000a },
	{ CCI_REG16(0x314a), 0x5f00 }, { CCI_REG16(0x3300), 0x0000 }, { CCI_REG16(0x3400), 0x0000 }, { CCI_REG16(0x3402), 0x4e46 },
	{ CCI_REG16(0x32b2), 0x000a }, { CCI_REG16(0x32b4), 0x000a }, { CCI_REG16(0x32b6), 0x000a }, { CCI_REG16(0x32b8), 0x000a },
	{ CCI_REG16(0x3c34), 0x0048 }, { CCI_REG16(0x3c36), 0x3000 }, { CCI_REG16(0x3c38), 0x0020 }, { CCI_REG16(0x393e), 0x4000 },
	{ CCI_REG16(0x303a), 0x0202 }, { CCI_REG16(0x3034), 0x4b00 }, { CCI_REG16(0x3036), 0xf729 }, { CCI_REG16(0x3032), 0x3500 },
	{ CCI_REG16(0x320e), 0x0480 },
};

static const struct cci_reg_sequence s5k3l6_mode_2104x1560[] = {
	/* 2x2 binning, 2104x1560 */
	{ CCI_REG16(0x0344), 0x0008 }, { CCI_REG16(0x0346), 0x0008 }, { CCI_REG16(0x0348), 0x1077 }, { CCI_REG16(0x034a), 0x0c37 },
	{ CCI_REG16(0x034c), 0x0838 }, { CCI_REG16(0x034e), 0x0618 }, { CCI_REG16(0x0900), 0x0122 }, { CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 }, { CCI_REG16(0x0384), 0x0001 }, { CCI_REG16(0x0386), 0x0003 }, { CCI_REG16(0x0114), 0x0330 },
	{ CCI_REG16(0x0110), 0x0002 }, { CCI_REG16(0x0136), 0x1800 }, { CCI_REG16(0x0304), 0x0004 }, { CCI_REG16(0x0306), 0x0078 },
	{ CCI_REG16(0x3c1e), 0x0000 }, { CCI_REG16(0x030c), 0x0003 }, { CCI_REG16(0x030e), 0x0047 }, { CCI_REG16(0x3c16), 0x0001 },
	{ CCI_REG16(0x0300), 0x0006 }, { CCI_REG16(0x0342), 0x1320 }, { CCI_REG16(0x0340), 0x0cbc }, { CCI_REG16(0x38c4), 0x0004 },
	{ CCI_REG16(0x38d8), 0x0011 }, { CCI_REG16(0x38da), 0x0005 }, { CCI_REG16(0x38dc), 0x0005 }, { CCI_REG16(0x38c2), 0x0005 },
	{ CCI_REG16(0x38c0), 0x0004 }, { CCI_REG16(0x38d6), 0x0004 }, { CCI_REG16(0x38d4), 0x0004 }, { CCI_REG16(0x38b0), 0x0007 },
	{ CCI_REG16(0x3932), 0x1000 }, { CCI_REG16(0x3938), 0x000c }, { CCI_REG16(0x0820), 0x0238 }, { CCI_REG16(0x380c), 0x0049 },
	{ CCI_REG16(0x3064), 0xebcf }, { CCI_REG16(0x309c), 0x0600 }, { CCI_REG16(0x3090), 0x8000 }, { CCI_REG16(0x3238), 0x000b },
	{ CCI_REG16(0x314a), 0x5f02 }, { CCI_REG16(0x3300), 0x0000 }, { CCI_REG16(0x3400), 0x0000 }, { CCI_REG16(0x3402), 0x4e46 },
	{ CCI_REG16(0x32b2), 0x0008 }, { CCI_REG16(0x32b4), 0x0008 }, { CCI_REG16(0x32b6), 0x0008 }, { CCI_REG16(0x32b8), 0x0008 },
	{ CCI_REG16(0x3c34), 0x0048 }, { CCI_REG16(0x3c36), 0x3000 }, { CCI_REG16(0x3c38), 0x0020 }, { CCI_REG16(0x393e), 0x4000 },
	{ CCI_REG16(0x303a), 0x0204 }, { CCI_REG16(0x3034), 0x4b01 }, { CCI_REG16(0x3036), 0x0029 }, { CCI_REG16(0x3032), 0x4800 },
	{ CCI_REG16(0x320e), 0x049e },
};

static const struct cci_reg_sequence s5k3l6_mode_4208x3120[] = {
	/* full resolution, 4208x3120 */
	{ CCI_REG16(0x0344), 0x0008 }, { CCI_REG16(0x0346), 0x0008 }, { CCI_REG16(0x0348), 0x1077 }, { CCI_REG16(0x034a), 0x0c37 },
	{ CCI_REG16(0x034c), 0x1070 }, { CCI_REG16(0x034e), 0x0c30 }, { CCI_REG16(0x0900), 0x0000 }, { CCI_REG16(0x0380), 0x0001 },
	{ CCI_REG16(0x0382), 0x0001 }, { CCI_REG16(0x0384), 0x0001 }, { CCI_REG16(0x0386), 0x0001 }, { CCI_REG16(0x0114), 0x0330 },
	{ CCI_REG16(0x0110), 0x0002 }, { CCI_REG16(0x0136), 0x1800 }, { CCI_REG16(0x0304), 0x0004 }, { CCI_REG16(0x0306), 0x0078 },
	{ CCI_REG16(0x3c1e), 0x0000 }, { CCI_REG16(0x030c), 0x0003 }, { CCI_REG16(0x030e), 0x004b }, { CCI_REG16(0x3c16), 0x0000 },
	{ CCI_REG16(0x0300), 0x0006 }, { CCI_REG16(0x0342), 0x1320 }, { CCI_REG16(0x0340), 0x0cbc }, { CCI_REG16(0x38c4), 0x0009 },
	{ CCI_REG16(0x38d8), 0x002a }, { CCI_REG16(0x38da), 0x000a }, { CCI_REG16(0x38dc), 0x000b }, { CCI_REG16(0x38c2), 0x000a },
	{ CCI_REG16(0x38c0), 0x000f }, { CCI_REG16(0x38d6), 0x000a }, { CCI_REG16(0x38d4), 0x0009 }, { CCI_REG16(0x38b0), 0x000f },
	{ CCI_REG16(0x3932), 0x1800 }, { CCI_REG16(0x3938), 0x000c }, { CCI_REG16(0x0820), 0x04b0 }, { CCI_REG16(0x380c), 0x0090 },
	{ CCI_REG16(0x3064), 0xefcf }, { CCI_REG16(0x309c), 0x0640 }, { CCI_REG16(0x3090), 0x8800 }, { CCI_REG16(0x3238), 0x000c },
	{ CCI_REG16(0x314a), 0x5f00 }, { CCI_REG16(0x3300), 0x0000 }, { CCI_REG16(0x3400), 0x0000 }, { CCI_REG16(0x3402), 0x4e42 },
	{ CCI_REG16(0x32b2), 0x0006 }, { CCI_REG16(0x32b4), 0x0006 }, { CCI_REG16(0x32b6), 0x0006 }, { CCI_REG16(0x32b8), 0x0006 },
	{ CCI_REG16(0x3c34), 0x0048 }, { CCI_REG16(0x3c36), 0x3000 }, { CCI_REG16(0x3c38), 0x0020 }, { CCI_REG16(0x393e), 0x4000 },
	{ CCI_REG16(0x303a), 0x0204 }, { CCI_REG16(0x3034), 0x4b01 }, { CCI_REG16(0x3036), 0x0029 }, { CCI_REG16(0x3032), 0x4800 },
	{ CCI_REG16(0x320e), 0x049e },
};

static const struct s5k3l6_mode s5k3l6_supported_modes[] = {
	{
		.width = 1052,
		.height = 780,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.hts = 0x1270,
		.vts = 0x0350,
		.link_freq_index = 0,
		.reg_list = {
			.regs = s5k3l6_mode_1052x780,
			.num_regs = ARRAY_SIZE(s5k3l6_mode_1052x780),
		},
	}, {
		.width = 2104,
		.height = 1560,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.hts = 0x1320,
		.vts = 0x0cbc,
		.link_freq_index = 1,
		.reg_list = {
			.regs = s5k3l6_mode_2104x1560,
			.num_regs = ARRAY_SIZE(s5k3l6_mode_2104x1560),
		},
	}, {
		.width = 4208,
		.height = 3120,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.hts = 0x1320,
		.vts = 0x0cbc,
		.link_freq_index = 2,
		.reg_list = {
			.regs = s5k3l6_mode_4208x3120,
			.num_regs = ARRAY_SIZE(s5k3l6_mode_4208x3120),
		},
	},
};

static const char *const s5k3l6_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"Color bars",
};

static bool s5k3l6_code_supported(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5k3l6_supported_modes); i++) {
		if (s5k3l6_supported_modes[i].code == code)
			return true;
	}

	return false;
}

static const struct s5k3l6_mode *s5k3l6_find_mode(u32 code, u32 width,
						  u32 height)
{
	const struct s5k3l6_mode *best = NULL;
	u32 best_delta = U32_MAX;
	unsigned int i;

	if (!s5k3l6_code_supported(code))
		code = s5k3l6_supported_modes[0].code;

	for (i = 0; i < ARRAY_SIZE(s5k3l6_supported_modes); i++) {
		const struct s5k3l6_mode *mode = &s5k3l6_supported_modes[i];
		u32 delta;

		if (mode->code != code)
			continue;

		delta = abs((int)mode->width - (int)width) +
			abs((int)mode->height - (int)height);
		if (!best || delta < best_delta) {
			best = mode;
			best_delta = delta;
		}
	}

	return best;
}

static void s5k3l6_update_pad_format(const struct s5k3l6_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = mode->code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static void s5k3l6_update_controls(struct s5k3l6 *s5k3l6,
				   const struct s5k3l6_mode *mode)
{
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - S5K3L6_EXPOSURE_MARGIN;
	u32 exposure_default = min_t(u32, 0x03de, exposure_max);

	__v4l2_ctrl_s_ctrl(s5k3l6->link_freq, mode->link_freq_index);
	__v4l2_ctrl_modify_range(s5k3l6->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(s5k3l6->vblank, vblank,
				 0xffff - mode->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s5k3l6->vblank, vblank);
	__v4l2_ctrl_modify_range(s5k3l6->exposure, S5K3L6_EXPOSURE_MIN,
				 exposure_max, 1, exposure_default);
	__v4l2_ctrl_s_ctrl(s5k3l6->exposure, exposure_default);
}

static int s5k3l6_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k3l6 *s5k3l6 =
		container_of(ctrl->handler, struct s5k3l6, ctrl_handler);
	const struct s5k3l6_mode *mode = s5k3l6->mode;
	u32 frame_length;
	u32 exposure_max;
	int active;
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max =
			mode->height + ctrl->val - S5K3L6_EXPOSURE_MARGIN;
		__v4l2_ctrl_modify_range(s5k3l6->exposure,
					 S5K3L6_EXPOSURE_MIN, exposure_max, 1,
					 s5k3l6->exposure->default_value);
	}

	active = pm_runtime_get_if_active(s5k3l6->dev);
	if (!active)
		return 0;
	if (active < 0)
		return active;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(s5k3l6->regmap, S5K3L6_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(s5k3l6->regmap, S5K3L6_REG_DIGITAL_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(s5k3l6->regmap, S5K3L6_REG_EXPOSURE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		frame_length = mode->height + ctrl->val;
		ret = cci_write(s5k3l6->regmap, S5K3L6_REG_FRAME_LENGTH,
				frame_length, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(s5k3l6->regmap, S5K3L6_REG_TEST_PATTERN,
				ctrl->val, NULL);
		break;
	default:
		ret = 0;
		break;
	}

	pm_runtime_put(s5k3l6->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k3l6_ctrl_ops = {
	.s_ctrl = s5k3l6_set_ctrl,
};

static int s5k3l6_init_controls(struct s5k3l6 *s5k3l6)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &s5k3l6->ctrl_handler;
	const struct v4l2_ctrl_ops *ops = &s5k3l6_ctrl_ops;
	const struct s5k3l6_mode *mode = s5k3l6->mode;
	struct v4l2_fwnode_device_properties props;
	u32 hblank = mode->hts - mode->width;
	u32 vblank = mode->vts - mode->height;
	u32 exposure_max = mode->vts - S5K3L6_EXPOSURE_MARGIN;
	u32 exposure_default = min_t(u32, 0x03de, exposure_max);
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 10);

	s5k3l6->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &s5k3l6_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(s5k3l6_link_freq_menu) - 1,
				       mode->link_freq_index,
				       s5k3l6_link_freq_menu);
	if (s5k3l6->link_freq)
		s5k3l6->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k3l6->pixel_rate =
		v4l2_ctrl_new_std(ctrl_hdlr, &s5k3l6_ctrl_ops,
				  V4L2_CID_PIXEL_RATE, S5K3L6_PIXEL_RATE,
				  S5K3L6_PIXEL_RATE, 1, S5K3L6_PIXEL_RATE);
	if (s5k3l6->pixel_rate)
		s5k3l6->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k3l6->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3l6_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (s5k3l6->hblank)
		s5k3l6->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	s5k3l6->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &s5k3l6_ctrl_ops,
					   V4L2_CID_VBLANK, vblank,
					   0xffff - mode->height, 1, vblank);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k3l6_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K3L6_ANALOG_GAIN_MIN, S5K3L6_ANALOG_GAIN_MAX, 1,
			  S5K3L6_ANALOG_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &s5k3l6_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  S5K3L6_DIGITAL_GAIN_MIN, S5K3L6_DIGITAL_GAIN_MAX,
			  1, S5K3L6_DIGITAL_GAIN_DEFAULT);

	s5k3l6->exposure =
		v4l2_ctrl_new_std(ctrl_hdlr, ops, V4L2_CID_EXPOSURE,
				  S5K3L6_EXPOSURE_MIN, exposure_max, 1,
				  exposure_default);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &s5k3l6_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3l6_test_pattern_menu) - 1,
				     0, 0, s5k3l6_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		goto error_free_hdlr;
	}

	ret = v4l2_fwnode_device_parse(s5k3l6->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &s5k3l6_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	s5k3l6->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int s5k3l6_write_mode(struct s5k3l6 *s5k3l6)
{
	const struct s5k3l6_mode *mode = s5k3l6->mode;
	int ret = 0;

	cci_multi_reg_write(s5k3l6->regmap, mode->reg_list.regs,
			    mode->reg_list.num_regs, &ret);
	cci_write(s5k3l6->regmap, S5K3L6_REG_DATA_FORMAT,
		  S5K3L6_DATA_FORMAT_RAW10, &ret);

	return ret;
}

static int s5k3l6_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	int ret;

	ret = pm_runtime_resume_and_get(s5k3l6->dev);
	if (ret)
		return ret;

	/* The vendor driver powers up the PLL before loading the mode table. */
	cci_write(s5k3l6->regmap, S5K3L6_REG_PLL_CTRL, 0x0100, &ret);
	if (ret)
		goto error;

	ret = s5k3l6_write_mode(s5k3l6);
	if (ret)
		goto error;

	ret = __v4l2_ctrl_handler_setup(s5k3l6->sd.ctrl_handler);
	if (ret)
		goto error;

	/*
	 * 0x0100 is written as one 16-bit value so that the stream-on bit and
	 * the second field (0x0101 = 0x03) are programmed together; the vendor
	 * driver then clears 0x0101 and releases the PLL again.
	 */
	cci_write(s5k3l6->regmap, S5K3L6_REG_MODE_SELECT16, 0x0103, &ret);
	cci_write(s5k3l6->regmap, S5K3L6_REG_PLL_CTRL, 0x0000, &ret);
	cci_write(s5k3l6->regmap, CCI_REG8(0x0101), 0x0000, &ret);
	if (ret)
		goto error;

	return 0;

error:
	dev_err(s5k3l6->dev, "failed to start streaming: %d\n", ret);
	pm_runtime_put_autosuspend(s5k3l6->dev);

	return ret;
}

static int s5k3l6_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	int ret;

	ret = cci_write(s5k3l6->regmap, S5K3L6_REG_MODE_SELECT,
			S5K3L6_MODE_STANDBY, NULL);
	if (ret)
		dev_err(s5k3l6->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(s5k3l6->dev);

	return ret;
}

static int s5k3l6_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	const struct s5k3l6_mode *mode;

	mode = s5k3l6_find_mode(fmt->format.code, fmt->format.width,
				fmt->format.height);
	s5k3l6_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && s5k3l6->mode != mode) {
		s5k3l6->mode = mode;
		s5k3l6_update_controls(s5k3l6, mode);
	}

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k3l6_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	static const u32 codes[] = {
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SGRBG10_1X10,
	};

	if (code->index >= ARRAY_SIZE(codes))
		return -EINVAL;

	code->code = codes[code->index];

	return 0;
}

static int s5k3l6_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index = 0;
	unsigned int i;

	if (!s5k3l6_code_supported(fse->code))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(s5k3l6_supported_modes); i++) {
		const struct s5k3l6_mode *mode = &s5k3l6_supported_modes[i];

		if (mode->code != fse->code)
			continue;

		if (index++ != fse->index)
			continue;

		fse->min_width = mode->width;
		fse->max_width = mode->width;
		fse->min_height = mode->height;
		fse->max_height = mode->height;

		return 0;
	}

	return -EINVAL;
}

static int s5k3l6_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K3L6_PIXEL_ARRAY_WIDTH;
		sel->r.height = S5K3L6_PIXEL_ARRAY_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r.left = S5K3L6_ACTIVE_LEFT;
		sel->r.top = S5K3L6_ACTIVE_TOP;
		sel->r.width = S5K3L6_ACTIVE_WIDTH;
		sel->r.height = S5K3L6_ACTIVE_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k3l6_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	const struct s5k3l6_mode *mode = s5k3l6->mode;

	if (pad)
		return -EINVAL;

	*config = (struct v4l2_mbus_config) {
		.type = V4L2_MBUS_CSI2_DPHY,
		.link_freq = s5k3l6_link_freq_menu[mode->link_freq_index],
	};
	config->bus.mipi_csi2.num_data_lanes = S5K3L6_DATA_LANES;

	return 0;
}

static int s5k3l6_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = s5k3l6->mode->code,
			.width = s5k3l6->mode->width,
			.height = s5k3l6->mode->height,
		},
	};

	s5k3l6_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k3l6_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k3l6_pad_ops = {
	.set_fmt = s5k3l6_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.get_selection = s5k3l6_get_selection,
	.get_mbus_config = s5k3l6_get_mbus_config,
	.enum_mbus_code = s5k3l6_enum_mbus_code,
	.enum_frame_size = s5k3l6_enum_frame_size,
	.enable_streams = s5k3l6_enable_streams,
	.disable_streams = s5k3l6_disable_streams,
};

static const struct v4l2_subdev_ops s5k3l6_subdev_ops = {
	.video = &s5k3l6_video_ops,
	.pad = &s5k3l6_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k3l6_internal_ops = {
	.init_state = s5k3l6_init_state,
};

static const struct media_entity_operations s5k3l6_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int s5k3l6_identify_sensor(struct s5k3l6 *s5k3l6)
{
	u64 val;
	int ret;

	ret = cci_read(s5k3l6->regmap, S5K3L6_REG_CHIP_ID, &val, NULL);
	if (ret)
		return dev_err_probe(s5k3l6->dev, ret,
				     "failed to read chip id\n");

	if (val != S5K3L6_CHIP_ID)
		return dev_err_probe(s5k3l6->dev, -ENODEV,
				     "chip id mismatch: %x != %llx\n",
				     S5K3L6_CHIP_ID, val);

	ret = cci_read(s5k3l6->regmap, S5K3L6_REG_REVISION, &val, NULL);
	if (ret)
		return dev_err_probe(s5k3l6->dev, ret,
				     "failed to read revision\n");

	if (val != S5K3L6_REVISION)
		dev_warn(s5k3l6->dev, "unexpected revision: %x != %x\n",
			 S5K3L6_REVISION, (u32)val);

	return 0;
}

static int s5k3l6_check_hwcfg(struct s5k3l6 *s5k3l6)
{
	struct fwnode_handle *fwnode = dev_fwnode(s5k3l6->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus = {
			.mipi_csi2 = {
				.num_data_lanes = S5K3L6_DATA_LANES,
			},
		},
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long freq_bitmap;
	unsigned long expected_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K3L6_DATA_LANES) {
		dev_err(s5k3l6->dev, "only %u data lanes are supported\n",
			S5K3L6_DATA_LANES);
		ret = -EINVAL;
		goto endpoint_free;
	}

	ret = v4l2_link_freq_to_bitmap(s5k3l6->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k3l6_link_freq_menu,
				       ARRAY_SIZE(s5k3l6_link_freq_menu),
				       &freq_bitmap);
	if (ret)
		goto endpoint_free;

	expected_bitmap = GENMASK(ARRAY_SIZE(s5k3l6_link_freq_menu) - 1, 0);
	if (freq_bitmap != expected_bitmap) {
		dev_err(s5k3l6->dev, "not all link frequencies are listed\n");
		ret = -EINVAL;
	}

endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int s5k3l6_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);
	int ret;

	ret = regulator_bulk_enable(S5K3L6_NUM_SUPPLIES, s5k3l6->supplies);
	if (ret)
		return ret;

	usleep_range(10, 20);

	ret = clk_prepare_enable(s5k3l6->mclk);
	if (ret)
		goto disable_regulators;

	gpiod_set_value_cansleep(s5k3l6->powerdown_gpio, 0);
	usleep_range(USEC_PER_MSEC, 1200);

	gpiod_set_value_cansleep(s5k3l6->reset_gpio, 0);
	usleep_range(USEC_PER_MSEC, 1200);

	gpiod_set_value_cansleep(s5k3l6->reset_gpio, 1);
	usleep_range(400, 800);
	gpiod_set_value_cansleep(s5k3l6->reset_gpio, 0);

	usleep_range(10 * USEC_PER_MSEC, 11 * USEC_PER_MSEC);

	return 0;

disable_regulators:
	regulator_bulk_disable(S5K3L6_NUM_SUPPLIES, s5k3l6->supplies);

	return ret;
}

static int s5k3l6_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);

	gpiod_set_value_cansleep(s5k3l6->reset_gpio, 1);
	gpiod_set_value_cansleep(s5k3l6->powerdown_gpio, 1);
	clk_disable_unprepare(s5k3l6->mclk);

	return regulator_bulk_disable(S5K3L6_NUM_SUPPLIES,
				      s5k3l6->supplies);
}

static int s5k3l6_probe(struct i2c_client *client)
{
	struct s5k3l6 *s5k3l6;
	unsigned long freq;
	unsigned int i;
	int ret;

	s5k3l6 = devm_kzalloc(&client->dev, sizeof(*s5k3l6), GFP_KERNEL);
	if (!s5k3l6)
		return -ENOMEM;

	s5k3l6->dev = &client->dev;
	v4l2_i2c_subdev_init(&s5k3l6->sd, client, &s5k3l6_subdev_ops);

	s5k3l6->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k3l6->regmap))
		return dev_err_probe(s5k3l6->dev, PTR_ERR(s5k3l6->regmap),
				     "failed to init CCI\n");

	s5k3l6->mclk = devm_v4l2_sensor_clk_get(s5k3l6->dev, NULL);
	if (IS_ERR(s5k3l6->mclk))
		return dev_err_probe(s5k3l6->dev, PTR_ERR(s5k3l6->mclk),
				     "failed to get clock\n");

	freq = clk_get_rate(s5k3l6->mclk);
	if (freq != S5K3L6_MCLK_FREQ)
		return dev_err_probe(s5k3l6->dev, -EINVAL,
				     "clock frequency %lu is not supported\n",
				     freq);

	ret = s5k3l6_check_hwcfg(s5k3l6);
	if (ret)
		return dev_err_probe(s5k3l6->dev, ret,
				     "failed to check HW configuration\n");

	s5k3l6->reset_gpio =
		devm_gpiod_get_optional(s5k3l6->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k3l6->reset_gpio))
		return dev_err_probe(s5k3l6->dev,
				     PTR_ERR(s5k3l6->reset_gpio),
				     "failed to get reset GPIO\n");

	s5k3l6->powerdown_gpio =
		devm_gpiod_get_optional(s5k3l6->dev, "power-down", GPIOD_OUT_LOW);
	if (IS_ERR(s5k3l6->powerdown_gpio))
		return dev_err_probe(s5k3l6->dev,
				     PTR_ERR(s5k3l6->powerdown_gpio),
				     "failed to get power-down GPIO\n");

	for (i = 0; i < S5K3L6_NUM_SUPPLIES; i++)
		s5k3l6->supplies[i].supply = s5k3l6_supply_names[i];

	ret = devm_regulator_bulk_get(s5k3l6->dev, S5K3L6_NUM_SUPPLIES,
				      s5k3l6->supplies);
	if (ret)
		return dev_err_probe(s5k3l6->dev, ret,
				     "failed to get supplies\n");

	ret = s5k3l6_power_on(s5k3l6->dev);
	if (ret)
		return ret;

	ret = s5k3l6_identify_sensor(s5k3l6);
	if (ret)
		goto power_off;

	s5k3l6->mode = &s5k3l6_supported_modes[0];
	ret = s5k3l6_init_controls(s5k3l6);
	if (ret)
		goto power_off;

	s5k3l6->sd.state_lock = s5k3l6->ctrl_handler.lock;
	s5k3l6->sd.internal_ops = &s5k3l6_internal_ops;
	s5k3l6->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k3l6->sd.entity.ops = &s5k3l6_entity_ops;
	s5k3l6->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k3l6->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k3l6->sd.entity, 1, &s5k3l6->pad);
	if (ret)
		goto free_ctrls;

	ret = v4l2_subdev_init_finalize(&s5k3l6->sd);
	if (ret)
		goto cleanup_entity;

	pm_runtime_set_active(s5k3l6->dev);
	pm_runtime_enable(s5k3l6->dev);

	ret = v4l2_async_register_subdev_sensor(&s5k3l6->sd);
	if (ret)
		goto cleanup_subdev;

	pm_runtime_set_autosuspend_delay(s5k3l6->dev, 1000);
	pm_runtime_use_autosuspend(s5k3l6->dev);
	pm_runtime_idle(s5k3l6->dev);

	return 0;

cleanup_subdev:
	v4l2_subdev_cleanup(&s5k3l6->sd);
	pm_runtime_disable(s5k3l6->dev);
	pm_runtime_set_suspended(s5k3l6->dev);

cleanup_entity:
	media_entity_cleanup(&s5k3l6->sd.entity);

free_ctrls:
	v4l2_ctrl_handler_free(s5k3l6->sd.ctrl_handler);

power_off:
	s5k3l6_power_off(s5k3l6->dev);

	return ret;
}

static void s5k3l6_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3l6 *s5k3l6 = to_s5k3l6(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(s5k3l6->dev);

	if (!pm_runtime_status_suspended(s5k3l6->dev)) {
		s5k3l6_power_off(s5k3l6->dev);
		pm_runtime_set_suspended(s5k3l6->dev);
	}
}

static const struct dev_pm_ops s5k3l6_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k3l6_power_off, s5k3l6_power_on, NULL)
};

static const struct of_device_id s5k3l6_of_match[] = {
	{ .compatible = "samsung,s5k3l6" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k3l6_of_match);

static struct i2c_driver s5k3l6_i2c_driver = {
	.driver = {
		.name = "s5k3l6",
		.pm = &s5k3l6_pm_ops,
		.of_match_table = s5k3l6_of_match,
	},
	.probe = s5k3l6_probe,
	.remove = s5k3l6_remove,
};

module_i2c_driver(s5k3l6_i2c_driver);

MODULE_AUTHOR("Purism SPC");
MODULE_AUTHOR("Vincent Cloutier <vincent@cloutier.co>");
MODULE_DESCRIPTION("Samsung S5K3L6 camera sensor driver");
MODULE_LICENSE("GPL");
