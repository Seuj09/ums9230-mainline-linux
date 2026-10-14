// SPDX-License-Identifier: GPL-2.0-only
/*
 * OmniVision TCM touchscreen driver (SPI, ROM-boot "host download" parts)
 *
 * Not yet run on hardware.
 *
 * Written from the behaviour of the stock 5.4 driver
 * (drivers/input/touchscreen/omnivision_tcm/ in the Helix kernel) for the
 * TD4160 touch/display controller of the Infinix X6525, not copied from it.
 *
 * The chip has no flash. After every reset it comes up in its ROM bootloader
 * and the host must download the application firmware and its configuration
 * from an image file (the stock tree's hdl_firmware_*.img):
 *
 *   reset -> IDENTIFY report (mode ROMBOOTLOADER)
 *         -> CMD_ROMBOOT_DOWNLOAD (the ROMBOOT_APP_CODE area, 14 byte header)
 *         -> CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE -> IDENTIFY report
 *         -> STATUS report: firmware asks for APP_CONFIG / DISPLAY / OPENSHORT
 *         -> CMD_DOWNLOAD_CONFIG for each of them
 *         -> application firmware is running: read app info, set the touch
 *            report layout, then parse TOUCH reports.
 *
 * Message framing (identical to Synaptics TCM):
 *   device->host  A5 code len_lo len_hi payload... 5A
 *   host->device  cmd len_lo len_hi payload...   (long writes: 01 + more)
 *
 * Not implemented: the F35 / RMI download path (older parts), flash update,
 * diagnostics, gestures, the stock tree's sysfs and watchdog.
 *
 * Open points, see drivers/README.md:
 *   - which hdl_firmware_*.img belongs to this panel is not known; the stock
 *     driver picks one from lcd_name, and "lcd_td4160_hx_*" matches none of
 *     its four names. It is a DT property here (firmware-name).
 *   - a panel reset resets the touch IC as well; the driver notices the new
 *     ROM-boot IDENTIFY report and downloads again by itself.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define MSG_HEADER_SIZE		4
#define MSG_MARKER		0xa5
#define MSG_PADDING		0x5a
#define MIN_READ_LEN		9
#define RD_CHUNK_SIZE		256
#define WR_CHUNK_SIZE		256
#define RESPONSE_TIMEOUT_MS	3000

/* commands */
#define CMD_CONTINUE_WRITE		0x01
#define CMD_IDENTIFY			0x02
#define CMD_ENTER_DEEP_SLEEP		0x2c
#define CMD_EXIT_DEEP_SLEEP		0x2d
#define CMD_GET_APPLICATION_INFO	0x20
#define CMD_GET_TOUCH_REPORT_CONFIG	0x25
#define CMD_SET_TOUCH_REPORT_CONFIG	0x26
#define CMD_DOWNLOAD_CONFIG		0x30
#define CMD_ROMBOOT_RUN_BOOTLOADER_FW	0x42
#define CMD_ROMBOOT_DOWNLOAD		0x45

/* status codes (device->host, code <= 0x0f) */
#define STATUS_IDLE			0x00
#define STATUS_OK			0x01
#define STATUS_BUSY			0x02
#define STATUS_CONTINUED_READ		0x03
#define STATUS_ERROR			0x0f
#define STATUS_INVALID			0xff

/* report types (code >= 0x10) */
#define REPORT_IDENTIFY			0x10
#define REPORT_TOUCH			0x11
#define REPORT_STATUS			0x1b

#define MODE_APPLICATION_FIRMWARE	0x01
#define MODE_HOSTDOWNLOAD_FIRMWARE	0x02
#define MODE_ROMBOOTLOADER		0x04

#define APP_STATUS_OK			0x00

/* touch report layout codes */
#define TOUCH_END			0
#define TOUCH_FOREACH_ACTIVE_OBJECT	1
#define TOUCH_FOREACH_OBJECT		2
#define TOUCH_FOREACH_END		3
#define TOUCH_PAD_TO_NEXT_BYTE		4
#define TOUCH_TIMESTAMP			5
#define TOUCH_OBJECT_N_INDEX		6
#define TOUCH_OBJECT_N_CLASSIFICATION	7
#define TOUCH_OBJECT_N_X_POSITION	8
#define TOUCH_OBJECT_N_Y_POSITION	9
#define TOUCH_OBJECT_N_Z		10
#define TOUCH_OBJECT_N_X_WIDTH		11
#define TOUCH_OBJECT_N_Y_WIDTH		12

#define TOUCH_REPORT_CONFIG_MIN		128
#define MAX_Z_VALUE			1000
#define MAX_MAJOR_VALUE			255

/* image file */
#define IMAGE_FILE_MAGIC		0x4818472b
#define FLASH_AREA_MAGIC		0x7c05e516
#define ROMBOOT_HDR_BYTES		14

/* HDL_* config types of CMD_DOWNLOAD_CONFIG */
#define HDL_TOUCH_CONFIG		1
#define HDL_DISPLAY_CONFIG		2
#define HDL_OPEN_SHORT_CONFIG		3

struct area_descriptor {
	u8 magic[4];
	u8 id[16];
	u8 flags[4];
	u8 flash_addr_words[4];
	u8 length[4];
	u8 checksum[4];
} __packed;

struct fw_area {
	const u8 *data;
	u32 size;
};

struct identify {
	u8 version;
	u8 mode;
	u8 part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct app_info {
	u8 version[2];
	u8 status[2];
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	u8 customer_config_id[16];
	u8 max_x[2];
	u8 max_y[2];
	u8 max_objects[2];
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
	u8 num_of_force_elecs[2];
} __packed;

/* what the firmware asks for in REPORT_STATUS / DOWNLOAD_CONFIG responses */
struct fw_status {
	u16 invalid_static_config:1;
	u16 need_disp_config:1;
	u16 need_app_config:1;
	u16 hdl_version:4;
	u16 need_open_short_config:1;
	u16 reserved:8;
} __packed;

struct object_data {
	u8 status;
	u32 x, y, z, x_width, y_width;
};

struct ovt_tcm {
	struct spi_device *spi;
	struct gpio_desc *reset;
	struct input_dev *input;
	struct workqueue_struct *wq;
	struct work_struct hdl_work;	/* ROM-boot firmware download */
	struct work_struct cfg_work;	/* config download + touch bring-up */
	struct work_struct init_work;	/* first reset */
	const char *fw_name;

	struct mutex spi_lock;		/* one SPI transaction at a time */
	struct mutex cmd_lock;		/* one command in flight */
	struct completion resp_done;

	/* command in flight, filled in by the IRQ thread */
	u8 pending_cmd;
	u8 resp_code;
	bool resp_err;
	u8 *resp_buf;
	unsigned int resp_len;

	struct identify id;
	bool got_identify;
	unsigned int wr_chunk;
	bool hdl_busy;			/* download sequence in progress */

	const struct firmware *fw;
	struct fw_area app_code, app_cfg, disp_cfg, os_cfg;
	struct fw_status fw_status;

	struct app_info app;
	unsigned int max_x, max_y, max_objects, objs_alloc;
	u8 *report_cfg;
	unsigned int report_cfg_len;
	struct object_data *objs;
	u8 *prev_status;
	bool touch_ready;
	bool suspended;
};

/* ---------------------------------------------------------------- SPI --- */

static int tcm_spi_read(struct ovt_tcm *t, u8 *rx, unsigned int len)
{
	u8 *tx, *rxb;
	struct spi_transfer xfer = { .len = len };
	int ret;

	/* both buffers heap-allocated: callers may pass a stack buffer, which
	 * must not be handed to a DMA-capable SPI controller */
	tx = kmalloc(len, GFP_KERNEL);
	rxb = kmalloc(len, GFP_KERNEL);
	if (!tx || !rxb) {
		kfree(tx);
		kfree(rxb);
		return -ENOMEM;
	}
	memset(tx, 0xff, len);
	xfer.tx_buf = tx;
	xfer.rx_buf = rxb;

	mutex_lock(&t->spi_lock);
	ret = spi_sync_transfer(t->spi, &xfer, 1);
	mutex_unlock(&t->spi_lock);
	if (!ret)
		memcpy(rx, rxb, len);
	kfree(tx);
	kfree(rxb);
	return ret;
}

static int tcm_spi_write(struct ovt_tcm *t, const u8 *tx, unsigned int len)
{
	struct spi_transfer xfer = { .len = len, .tx_buf = tx };
	int ret;

	mutex_lock(&t->spi_lock);
	ret = spi_sync_transfer(t->spi, &xfer, 1);
	mutex_unlock(&t->spi_lock);
	return ret;
}

/*
 * Read one message. Returns the payload in *payload (kmalloc'd, caller
 * frees) and the code in *code. A message that is shorter than
 * MIN_READ_LEN is complete after the first transfer, a longer one is
 * fetched by "continued reads" that each start with A5 03.
 */
static int tcm_read_message(struct ovt_tcm *t, u8 *code, u8 **payload,
			    unsigned int *plen)
{
	u8 first[MIN_READ_LEN], *msg = NULL, *tmp = NULL;
	unsigned int len, total, off, chunk_space, remaining, xfer;
	bool retry = true;
	int ret;

	*payload = NULL;
	*plen = 0;
retry:
	ret = tcm_spi_read(t, first, MIN_READ_LEN);
	if (!ret && first[0] != MSG_MARKER)
		ret = -ENXIO;
	if (ret) {
		if (retry) {
			retry = false;
			usleep_range(5000, 10000);
			goto retry;
		}
		return ret;
	}

	*code = first[1];
	len = get_unaligned_le16(&first[2]);

	if (*code == STATUS_IDLE || *code == STATUS_BUSY ||
	    *code == STATUS_CONTINUED_READ)
		return 0;
	if (*code == STATUS_INVALID && retry) {
		retry = false;
		usleep_range(5000, 10000);
		goto retry;
	}

	total = MSG_HEADER_SIZE + len + 1;
	msg = kmalloc(total + 2, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	memcpy(msg, first, min_t(unsigned int, total, MIN_READ_LEN));

	if (total > MIN_READ_LEN) {
		/* the padding byte alone is not worth another transfer */
		if (total - 1 == MIN_READ_LEN) {
			msg[total - 1] = MSG_PADDING;
		} else {
			remaining = total - MIN_READ_LEN;
			if (t->id.mode == MODE_ROMBOOTLOADER)
				chunk_space = remaining;
			else
				chunk_space = RD_CHUNK_SIZE - 2;
			off = MIN_READ_LEN;

			tmp = kmalloc(chunk_space + 2, GFP_KERNEL);
			if (!tmp) {
				ret = -ENOMEM;
				goto err;
			}
			while (remaining) {
				xfer = min(remaining, chunk_space);
				if (xfer == 1) {
					msg[off++] = MSG_PADDING;
					remaining--;
					continue;
				}
				ret = tcm_spi_read(t, tmp, xfer + 2);
				if (ret)
					goto err;
				if (tmp[0] != MSG_MARKER ||
				    tmp[1] != STATUS_CONTINUED_READ) {
					ret = -EIO;
					goto err;
				}
				memcpy(&msg[off], &tmp[2], xfer);
				off += xfer;
				remaining -= xfer;
			}
			kfree(tmp);
			tmp = NULL;
		}
	}

	if (msg[total - 1] != MSG_PADDING) {
		dev_err(&t->spi->dev, "bad message padding 0x%02x\n",
			msg[total - 1]);
		ret = -EIO;
		goto err;
	}

	*payload = msg;
	*plen = len;
	/* payload starts after the header; hand back an offset-adjusted copy */
	memmove(msg, msg + MSG_HEADER_SIZE, len);
	return 0;
err:
	kfree(tmp);
	kfree(msg);
	return ret;
}

/*
 * Write a command. The packet is cmd, len_lo, len_hi, payload; if the device
 * has a write size limit the remainder follows in CMD_CONTINUE_WRITE chunks.
 */
static int tcm_write_command(struct ovt_tcm *t, u8 cmd, const u8 *payload,
			     unsigned int len)
{
	unsigned int remaining = len + 2, chunk_space, xfer, idx = 0, i;
	u8 *buf;
	int ret = 0;

	/* a first chunk carries cmd + 2 length bytes + data: anything the device
	 * reports below that would never make progress (or overrun buf) */
	if (t->wr_chunk && t->wr_chunk < 4)
		return -EINVAL;
	chunk_space = t->wr_chunk ? t->wr_chunk - 1 : remaining;
	if (cmd == CMD_ROMBOOT_DOWNLOAD)
		chunk_space = remaining;
	buf = kmalloc(chunk_space + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (i = 0; remaining; i++) {
		xfer = min(remaining, chunk_space);
		if (i == 0) {
			buf[0] = cmd;
			buf[1] = len & 0xff;
			buf[2] = len >> 8;
			if (xfer > 2)
				memcpy(&buf[3], payload, xfer - 2);
			idx = xfer - 2;
		} else {
			buf[0] = CMD_CONTINUE_WRITE;
			memcpy(&buf[1], payload + idx, xfer);
			idx += xfer;
		}
		ret = tcm_spi_write(t, buf, xfer + 1);
		if (ret)
			break;
		remaining -= xfer;
		if (remaining)
			usleep_range(500, 1000);
	}
	kfree(buf);
	return ret;
}

/*
 * Send a command and wait for the IRQ thread to deliver the response.
 * *resp (if not NULL) receives a kmalloc'd copy of the response payload.
 */
static int tcm_command(struct ovt_tcm *t, u8 cmd, const u8 *payload,
		       unsigned int len, u8 **resp, unsigned int *resp_len)
{
	int ret;

	mutex_lock(&t->cmd_lock);
	reinit_completion(&t->resp_done);
	t->resp_err = false;
	t->resp_buf = NULL;
	t->resp_len = 0;
	WRITE_ONCE(t->pending_cmd, cmd);

	ret = tcm_write_command(t, cmd, payload, len);
	if (ret)
		goto out;

	if (!wait_for_completion_timeout(&t->resp_done,
					 msecs_to_jiffies(RESPONSE_TIMEOUT_MS))) {
		dev_err(&t->spi->dev, "command 0x%02x timed out\n", cmd);
		ret = -ETIMEDOUT;
		goto out;
	}
	if (t->resp_err) {
		dev_err(&t->spi->dev, "command 0x%02x failed, code 0x%02x\n",
			cmd, t->resp_code);
		ret = -EIO;
		kfree(t->resp_buf);
		t->resp_buf = NULL;
		goto out;
	}
	if (resp) {
		*resp = t->resp_buf;
		if (resp_len)
			*resp_len = t->resp_len;
	} else {
		kfree(t->resp_buf);
	}
	t->resp_buf = NULL;
out:
	WRITE_ONCE(t->pending_cmd, 0);
	mutex_unlock(&t->cmd_lock);
	return ret;
}

/* ---------------------------------------------------------- firmware ---- */

static int tcm_parse_image(struct ovt_tcm *t)
{
	const u8 *img = t->fw->data;
	size_t size = t->fw->size;
	u32 areas, addr, length, i;
	const struct area_descriptor *d;

	memset(&t->app_code, 0, sizeof(t->app_code));
	memset(&t->app_cfg, 0, sizeof(t->app_cfg));
	memset(&t->disp_cfg, 0, sizeof(t->disp_cfg));
	memset(&t->os_cfg, 0, sizeof(t->os_cfg));

	if (size < 8 || get_unaligned_le32(img) != IMAGE_FILE_MAGIC)
		return -EINVAL;
	areas = get_unaligned_le32(img + 4);
	if (areas > (size - 8) / 4)
		return -EINVAL;

	for (i = 0; i < areas; i++) {
		struct fw_area *dst = NULL;

		addr = get_unaligned_le32(img + 8 + 4 * i);
		if (addr > size || size - addr < sizeof(*d))
			return -EINVAL;
		d = (const void *)(img + addr);
		if (get_unaligned_le32(d->magic) != FLASH_AREA_MAGIC)
			continue;
		length = get_unaligned_le32(d->length);
		if (length > size - addr - sizeof(*d))
			return -EINVAL;

		if (!strncmp(d->id, "ROMBOOT_APP_CODE", 16))
			dst = &t->app_code;
		else if (!strncmp(d->id, "APP_CONFIG", 10))
			dst = &t->app_cfg;
		else if (!strncmp(d->id, "DISPLAY", 7))
			dst = &t->disp_cfg;
		else if (!strncmp(d->id, "OPENSHORT", 9))
			dst = &t->os_cfg;
		else
			continue;

		if (get_unaligned_le32(d->checksum) !=
		    (crc32_le(~0, (const u8 *)(d + 1), length) ^ ~0)) {
			dev_err(&t->spi->dev, "checksum error in area %.16s\n",
				d->id);
			return -EINVAL;
		}
		dst->data = (const u8 *)(d + 1);
		dst->size = length;
	}

	if (!t->app_code.size || !t->app_cfg.size)
		return -EINVAL;
	return 0;
}

static int tcm_load_image(struct ovt_tcm *t)
{
	int ret;

	if (t->fw)
		return 0;
	ret = request_firmware(&t->fw, t->fw_name, &t->spi->dev);
	if (ret) {
		dev_err(&t->spi->dev, "cannot load %s: %d\n", t->fw_name, ret);
		return ret;
	}
	ret = tcm_parse_image(t);
	if (ret) {
		dev_err(&t->spi->dev, "%s is not a usable HDL image\n",
			t->fw_name);
		release_firmware(t->fw);
		t->fw = NULL;
	}
	return ret;
}

static void tcm_release_image(struct ovt_tcm *t)
{
	release_firmware(t->fw);
	t->fw = NULL;
}

/* ----------------------------------------------------- touch reporting -- */

static int tcm_report_bits(const u8 *rep, unsigned int rep_len,
			   unsigned int offset, unsigned int bits, u32 *out)
{
	unsigned int bit_off = offset % 8, byte_off = offset / 8, left = bits;
	u32 val = 0;

	if (!bits || bits > 32)
		return -EINVAL;
	if (offset + bits > rep_len * 8) {
		*out = 0;
		return 0;
	}
	while (left) {
		unsigned int n = min(8 - bit_off, left);
		u8 b = (rep[byte_off] >> bit_off) & (0xff >> (8 - n));

		val |= (u32)b << (bits - left);
		bit_off = 0;
		byte_off++;
		left -= n;
	}
	*out = val;
	return 0;
}

static int tcm_parse_touch(struct ovt_tcm *t, const u8 *rep, unsigned int len)
{
	const u8 *cfg = t->report_cfg;
	unsigned int idx = 0, offset = 0, obj = 0, next = 0, loop_start = 0;
	bool active_only = false;
	u32 data;

	memset(t->objs, 0, sizeof(*t->objs) * t->max_objects);

	while (idx < t->report_cfg_len) {
		u8 code = cfg[idx++];
		unsigned int bits;

		switch (code) {
		case TOUCH_END:
			return 0;
		case TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			loop_start = offset;
			active_only = true;
			break;
		case TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;
		case TOUCH_FOREACH_END:
			if (active_only) {
				/* repeat while the report still has bits */
				if (offset < len * 8 && offset > loop_start) {
					loop_start = offset;
					idx = next;
				}
			} else {
				obj++;
				if (obj < t->max_objects)
					idx = next;
			}
			break;
		case TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;
		case TOUCH_TIMESTAMP:
		case TOUCH_OBJECT_N_INDEX:
		case TOUCH_OBJECT_N_CLASSIFICATION:
		case TOUCH_OBJECT_N_X_POSITION:
		case TOUCH_OBJECT_N_Y_POSITION:
		case TOUCH_OBJECT_N_Z:
		case TOUCH_OBJECT_N_X_WIDTH:
		case TOUCH_OBJECT_N_Y_WIDTH:
			if (idx >= t->report_cfg_len)
				return -EINVAL;
			bits = cfg[idx++];
			if (tcm_report_bits(rep, len, offset, bits, &data))
				return -EINVAL;
			offset += bits;
			if (code == TOUCH_OBJECT_N_INDEX) {
				obj = data;
				break;
			}
			if (code == TOUCH_TIMESTAMP)
				break;
			if (obj >= t->max_objects)
				break;
			switch (code) {
			case TOUCH_OBJECT_N_CLASSIFICATION:
				t->objs[obj].status = data;
				break;
			case TOUCH_OBJECT_N_X_POSITION:
				t->objs[obj].x = data;
				break;
			case TOUCH_OBJECT_N_Y_POSITION:
				t->objs[obj].y = data;
				break;
			case TOUCH_OBJECT_N_Z:
				t->objs[obj].z = data;
				break;
			case TOUCH_OBJECT_N_X_WIDTH:
				t->objs[obj].x_width = data;
				break;
			case TOUCH_OBJECT_N_Y_WIDTH:
				t->objs[obj].y_width = data;
				break;
			}
			break;
		default:
			/* unknown field: it carries a bit count, skip it */
			if (idx >= t->report_cfg_len)
				return -EINVAL;
			offset += cfg[idx++];
			break;
		}
	}
	return 0;
}

static void tcm_report_touch(struct ovt_tcm *t, const u8 *rep, unsigned int len)
{
	unsigned int i;

	if (!t->touch_ready || t->suspended)
		return;
	if (tcm_parse_touch(t, rep, len))
		return;

	for (i = 0; i < t->max_objects; i++) {
		u8 st = t->objs[i].status;
		bool down = st == 1 || st == 2;	/* finger, gloved finger */

		if (!down && !t->prev_status[i])
			continue;
		input_mt_slot(t->input, i);
		input_mt_report_slot_state(t->input, MT_TOOL_FINGER, down);
		if (down) {
			input_report_abs(t->input, ABS_MT_POSITION_X, t->objs[i].x);
			input_report_abs(t->input, ABS_MT_POSITION_Y, t->objs[i].y);
			input_report_abs(t->input, ABS_MT_PRESSURE, t->objs[i].z);
			input_report_abs(t->input, ABS_MT_TOUCH_MAJOR,
					 (t->objs[i].x_width + t->objs[i].y_width) / 2);
		}
		t->prev_status[i] = down;
	}
	input_mt_sync_frame(t->input);
	input_sync(t->input);
}

static void tcm_release_all(struct ovt_tcm *t)
{
	unsigned int i;

	if (!t->input || !t->prev_status)
		return;
	for (i = 0; i < t->max_objects; i++) {
		input_mt_slot(t->input, i);
		input_mt_report_slot_state(t->input, MT_TOOL_FINGER, false);
		t->prev_status[i] = 0;
	}
	input_mt_sync_frame(t->input);
	input_sync(t->input);
}

/* Fixed layout, same as the stock driver: 4+4+12+12+16+16+16 bits/object. */
static int tcm_set_report_config(struct ovt_tcm *t)
{
	static const u8 layout[] = {
		TOUCH_FOREACH_ACTIVE_OBJECT,
		TOUCH_OBJECT_N_INDEX, 4,
		TOUCH_OBJECT_N_CLASSIFICATION, 4,
		TOUCH_OBJECT_N_X_POSITION, 12,
		TOUCH_OBJECT_N_Y_POSITION, 12,
		TOUCH_OBJECT_N_X_WIDTH, 16,
		TOUCH_OBJECT_N_Y_WIDTH, 16,
		TOUCH_OBJECT_N_Z, 16,
		TOUCH_FOREACH_END,
		TOUCH_END,
	};
	unsigned int max = get_unaligned_le16(t->app.max_touch_report_config_size);
	u8 *buf;
	int ret;

	if (max < TOUCH_REPORT_CONFIG_MIN)
		return -EINVAL;
	buf = kzalloc(max, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy(buf, layout, sizeof(layout));
	ret = tcm_command(t, CMD_SET_TOUCH_REPORT_CONFIG, buf, max, NULL, NULL);
	kfree(buf);
	return ret;
}

static int tcm_register_input(struct ovt_tcm *t)
{
	struct input_dev *in;
	int ret;

	if (t->input)
		return 0;
	in = devm_input_allocate_device(&t->spi->dev);
	if (!in)
		return -ENOMEM;
	in->name = "omnivision_tcm_touch";
	in->id.bustype = BUS_SPI;
	input_set_capability(in, EV_KEY, BTN_TOUCH);
	input_set_abs_params(in, ABS_MT_POSITION_X, 0, t->max_x, 0, 0);
	input_set_abs_params(in, ABS_MT_POSITION_Y, 0, t->max_y, 0, 0);
	input_set_abs_params(in, ABS_MT_PRESSURE, 0, MAX_Z_VALUE, 0, 0);
	input_set_abs_params(in, ABS_MT_TOUCH_MAJOR, 0, MAX_MAJOR_VALUE, 0, 0);
	ret = input_mt_init_slots(in, t->max_objects, INPUT_MT_DIRECT);
	if (ret)
		return ret;
	ret = input_register_device(in);
	if (ret)
		return ret;
	t->input = in;
	return 0;
}

/* Application firmware is running: read what it can do, set up reporting. */
static int tcm_bring_up_touch(struct ovt_tcm *t)
{
	u8 *resp = NULL;
	unsigned int rlen, tries;
	int ret;

	ret = tcm_command(t, CMD_IDENTIFY, NULL, 0, &resp, &rlen);
	if (ret)
		return ret;
	memcpy(&t->id, resp, min_t(unsigned int, rlen, sizeof(t->id)));
	t->wr_chunk = min_t(unsigned int, get_unaligned_le16(t->id.max_write_size),
			    WR_CHUNK_SIZE);
	kfree(resp);
	if (t->id.mode != MODE_APPLICATION_FIRMWARE &&
	    t->id.mode != MODE_HOSTDOWNLOAD_FIRMWARE) {
		dev_err(&t->spi->dev, "unexpected mode 0x%02x after download\n",
			t->id.mode);
		return -EIO;
	}

	for (tries = 0; tries < 20; tries++) {
		resp = NULL;
		ret = tcm_command(t, CMD_GET_APPLICATION_INFO, NULL, 0,
				  &resp, &rlen);
		if (ret)
			return ret;
		memset(&t->app, 0, sizeof(t->app));
		memcpy(&t->app, resp, min_t(unsigned int, rlen, sizeof(t->app)));
		kfree(resp);
		if (get_unaligned_le16(t->app.status) != 0x01 &&	/* booting */
		    get_unaligned_le16(t->app.status) != 0x02)		/* updating */
			break;
		msleep(100);
	}
	if (get_unaligned_le16(t->app.status) != APP_STATUS_OK) {
		dev_err(&t->spi->dev, "application status 0x%04x\n",
			get_unaligned_le16(t->app.status));
		return -EIO;
	}

	t->max_x = get_unaligned_le16(t->app.max_x);
	t->max_y = get_unaligned_le16(t->app.max_y);
	t->max_objects = get_unaligned_le16(t->app.max_objects);
	if (!t->max_x || !t->max_y || !t->max_objects)
		return -EINVAL;
	/* the arrays and input slots are sized once; a later bring-up must agree */
	if (t->objs && t->max_objects != t->objs_alloc) {
		dev_err(&t->spi->dev, "max_objects changed from %u to %u\n",
			t->objs_alloc, t->max_objects);
		return -EINVAL;
	}

	ret = tcm_set_report_config(t);
	if (ret)
		return ret;

	resp = NULL;
	ret = tcm_command(t, CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0, &resp, &rlen);
	if (ret)
		return ret;

	t->touch_ready = false;
	/* let a touch report being parsed finish before its config is freed */
	synchronize_irq(t->spi->irq);
	kfree(t->report_cfg);
	t->report_cfg = resp;
	t->report_cfg_len = rlen;

	if (!t->objs) {
		t->objs = devm_kcalloc(&t->spi->dev, t->max_objects,
				       sizeof(*t->objs), GFP_KERNEL);
		t->prev_status = devm_kzalloc(&t->spi->dev, t->max_objects,
					      GFP_KERNEL);
		if (!t->objs || !t->prev_status)
			return -ENOMEM;
		t->objs_alloc = t->max_objects;
	}
	ret = tcm_register_input(t);
	if (ret)
		return ret;

	t->touch_ready = true;
	dev_info(&t->spi->dev, "touch ready: %ux%u, %u objects, config id %.16s\n",
		 t->max_x, t->max_y, t->max_objects, t->app.customer_config_id);
	return 0;
}

/* ------------------------------------------------------ download work --- */

static int tcm_send_config(struct ovt_tcm *t, u8 type, const struct fw_area *a)
{
	unsigned int pad = 0, len;
	u8 *buf, *resp = NULL;
	unsigned int rlen;
	int ret;

	if (!a->size)
		return -EINVAL;
	if (type == HDL_TOUCH_CONFIG)
		pad = round_up(a->size, 8) - a->size;
	len = a->size + 2 + pad;
	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	/* byte 0 is the HDL protocol version the firmware asked for */
	buf[0] = t->fw_status.hdl_version == 2 ? 2 : 1;
	buf[1] = type;
	memcpy(&buf[2], a->data, a->size);

	ret = tcm_command(t, CMD_DOWNLOAD_CONFIG, buf, len, &resp, &rlen);
	kfree(buf);
	if (ret)
		return ret;
	if (rlen >= sizeof(t->fw_status))
		memcpy(&t->fw_status, resp, sizeof(t->fw_status));
	kfree(resp);
	return 0;
}

static void tcm_cfg_work(struct work_struct *work)
{
	struct ovt_tcm *t = container_of(work, struct ovt_tcm, cfg_work);
	int ret = 0, guard = 0;

	if (!t->hdl_busy)
		return;
	ret = tcm_load_image(t);
	if (ret)
		goto fail;

	while ((t->fw_status.need_app_config || t->fw_status.need_disp_config ||
		(t->fw_status.need_open_short_config && t->os_cfg.size)) &&
	       guard++ < 10) {
		if (t->fw_status.need_app_config)
			ret = tcm_send_config(t, HDL_TOUCH_CONFIG, &t->app_cfg);
		else if (t->fw_status.need_disp_config)
			ret = tcm_send_config(t, HDL_DISPLAY_CONFIG, &t->disp_cfg);
		else
			ret = tcm_send_config(t, HDL_OPEN_SHORT_CONFIG, &t->os_cfg);
		if (ret)
			goto fail;
	}

	/* nothing more requested: the application firmware is up */
	ret = tcm_bring_up_touch(t);
fail:
	if (ret)
		dev_err(&t->spi->dev, "bring-up after download failed: %d\n", ret);
	tcm_release_image(t);
	t->hdl_busy = false;
}

static void tcm_hdl_work(struct work_struct *work)
{
	struct ovt_tcm *t = container_of(work, struct ovt_tcm, hdl_work);
	u8 *buf;
	int ret;

	t->touch_ready = false;
	tcm_release_all(t);

	ret = tcm_load_image(t);
	if (ret)
		goto fail;

	buf = kzalloc(t->app_code.size + ROMBOOT_HDR_BYTES, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto fail;
	}
	buf[0] = t->app_code.size >> 16;
	memcpy(buf + ROMBOOT_HDR_BYTES, t->app_code.data, t->app_code.size);

	t->wr_chunk = 0;	/* unlimited in ROM boot */
	ret = tcm_command(t, CMD_ROMBOOT_DOWNLOAD, buf,
			  t->app_code.size + ROMBOOT_HDR_BYTES, NULL, NULL);
	kfree(buf);
	if (ret)
		goto fail;

	/* completes when the new firmware's IDENTIFY report arrives */
	ret = tcm_command(t, CMD_ROMBOOT_RUN_BOOTLOADER_FW, NULL, 0, NULL, NULL);
	if (ret)
		goto fail;
	/* from here the firmware sends a STATUS report asking for configs */
	return;
fail:
	dev_err(&t->spi->dev, "firmware download failed: %d\n", ret);
	tcm_release_image(t);
	t->hdl_busy = false;
}

/* ------------------------------------------------------------- IRQ ------ */

static irqreturn_t tcm_irq_thread(int irq, void *data)
{
	struct ovt_tcm *t = data;
	u8 code, *payload = NULL, pending;
	unsigned int len;
	int ret;

	ret = tcm_read_message(t, &code, &payload, &len);
	if (ret) {
		/* the part is simply not talking, e.g. in reset */
		if (READ_ONCE(t->pending_cmd)) {
			t->resp_err = true;
			t->resp_code = STATUS_INVALID;
			complete(&t->resp_done);
		}
		return IRQ_HANDLED;
	}
	if (code == STATUS_IDLE || code == STATUS_BUSY ||
	    code == STATUS_CONTINUED_READ)
		return IRQ_HANDLED;

	pending = READ_ONCE(t->pending_cmd);

	if (code < REPORT_IDENTIFY) {
		/* command response */
		if (pending) {
			t->resp_code = code;
			t->resp_err = code != STATUS_OK;
			t->resp_buf = payload;
			t->resp_len = len;
			payload = NULL;
			complete(&t->resp_done);
		}
		goto out;
	}

	switch (code) {
	case REPORT_IDENTIFY:
		if (len >= 2) {
			memset(&t->id, 0, sizeof(t->id));
			memcpy(&t->id, payload, min_t(unsigned int, len, sizeof(t->id)));
			t->got_identify = true;
		}
		if (pending == CMD_IDENTIFY) {
			/* the reply to CMD_IDENTIFY is itself an identify report */
			t->resp_err = false;
			t->resp_code = STATUS_OK;
			t->resp_buf = payload;
			t->resp_len = len;
			payload = NULL;
			complete(&t->resp_done);
			break;
		}
		if (pending == CMD_ROMBOOT_RUN_BOOTLOADER_FW) {
			/* the "response" to these is the device coming back */
			t->resp_err = false;
			t->resp_code = STATUS_OK;
			complete(&t->resp_done);
			break;
		}
		if (t->id.mode == MODE_ROMBOOTLOADER && !t->hdl_busy) {
			t->hdl_busy = true;
			t->fw_status = (struct fw_status){ 0 };
			queue_work(t->wq, &t->hdl_work);
		}
		break;
	case REPORT_STATUS:
		if (len >= sizeof(t->fw_status) && t->hdl_busy) {
			memcpy(&t->fw_status, payload, sizeof(t->fw_status));
			queue_work(t->wq, &t->cfg_work);
		}
		break;
	case REPORT_TOUCH:
		tcm_report_touch(t, payload, len);
		break;
	default:
		break;
	}
out:
	kfree(payload);
	return IRQ_HANDLED;
}

/* --------------------------------------------------------- probe etc. --- */

static void tcm_hw_reset(struct ovt_tcm *t)
{
	gpiod_set_value_cansleep(t->reset, 1);
	msleep(20);
	gpiod_set_value_cansleep(t->reset, 0);
}

static void tcm_init_work(struct work_struct *work)
{
	struct ovt_tcm *t = container_of(work, struct ovt_tcm, init_work);
	u8 code, *payload;
	unsigned int len;

	tcm_hw_reset(t);
	msleep(200);
	/*
	 * With a level-triggered interrupt the identify report has already
	 * been picked up by the IRQ thread. With an edge, it may have been
	 * missed while the handler was not requested: look once by hand.
	 */
	if (!t->got_identify && !tcm_read_message(t, &code, &payload, &len)) {
		if (code == REPORT_IDENTIFY && len >= 2) {
			memcpy(&t->id, payload,
			       min_t(unsigned int, len, sizeof(t->id)));
			t->got_identify = true;
			if (t->id.mode == MODE_ROMBOOTLOADER && !t->hdl_busy) {
				t->hdl_busy = true;
				queue_work(t->wq, &t->hdl_work);
			}
		}
		kfree(payload);
	}
	if (!t->got_identify)
		dev_warn(&t->spi->dev, "no identify report after reset\n");
}

static int ovt_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ovt_tcm *t;
	int ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;
	t->spi = spi;
	mutex_init(&t->spi_lock);
	mutex_init(&t->cmd_lock);
	init_completion(&t->resp_done);
	INIT_WORK(&t->hdl_work, tcm_hdl_work);
	INIT_WORK(&t->cfg_work, tcm_cfg_work);
	INIT_WORK(&t->init_work, tcm_init_work);
	spi_set_drvdata(spi, t);

	ret = device_property_read_string(dev, "firmware-name", &t->fw_name);
	if (ret) {
		dev_err(dev, "firmware-name is required\n");
		return ret;
	}

	t->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(t->reset))
		return dev_err_probe(dev, PTR_ERR(t->reset), "reset gpio\n");

	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	t->wq = devm_alloc_ordered_workqueue(dev, "ovt-tcm", 0);
	if (!t->wq)
		return -ENOMEM;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, tcm_irq_thread,
					IRQF_ONESHOT, "ovt-tcm", t);
	if (ret)
		return dev_err_probe(dev, ret, "irq\n");

	queue_work(t->wq, &t->init_work);
	return 0;
}

static void ovt_tcm_remove(struct spi_device *spi)
{
	struct ovt_tcm *t = spi_get_drvdata(spi);

	disable_irq(spi->irq);
	cancel_work_sync(&t->init_work);
	cancel_work_sync(&t->hdl_work);
	cancel_work_sync(&t->cfg_work);
	tcm_release_image(t);
	kfree(t->report_cfg);
}

static int ovt_tcm_suspend(struct device *dev)
{
	struct ovt_tcm *t = dev_get_drvdata(dev);

	t->suspended = true;
	tcm_release_all(t);
	if (t->touch_ready)
		tcm_command(t, CMD_ENTER_DEEP_SLEEP, NULL, 0, NULL, NULL);
	return 0;
}

static int ovt_tcm_resume(struct device *dev)
{
	struct ovt_tcm *t = dev_get_drvdata(dev);

	if (t->touch_ready)
		tcm_command(t, CMD_EXIT_DEEP_SLEEP, NULL, 0, NULL, NULL);
	t->suspended = false;
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ovt_tcm_pm, ovt_tcm_suspend, ovt_tcm_resume);

static const struct of_device_id ovt_tcm_of_match[] = {
	{ .compatible = "omnivision,tcm-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match);

static const struct spi_device_id ovt_tcm_id[] = {
	{ "omnivision_tcm_spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_id);

static struct spi_driver ovt_tcm_driver = {
	.driver = {
		.name = "omnivision_tcm_spi",
		.of_match_table = ovt_tcm_of_match,
		.pm = pm_sleep_ptr(&ovt_tcm_pm),
	},
	.id_table = ovt_tcm_id,
	.probe = ovt_tcm_probe,
	.remove = ovt_tcm_remove,
};
module_spi_driver(ovt_tcm_driver);

MODULE_DESCRIPTION("OmniVision TCM touchscreen (SPI, host download)");
MODULE_LICENSE("GPL");
