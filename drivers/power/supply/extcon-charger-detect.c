// SPDX-License-Identifier: GPL-2.0-only
/*
 * Power supply that reports a USB charger as seen on an extcon device.
 *
 * Not yet run on hardware.
 *
 * Some chargers (bq2415x is one) learn what is plugged in from a power
 * supply that reports POWER_SUPPLY_PROP_CURRENT_MAX and sends a change event
 * when it moves, see "ti,usb-charger-detection". Boards without a BC1.2
 * detector driver can still get VBUS presence from an extcon (for example
 * linux,extcon-usb-gpio on a VBUS pin). This turns what the extcon reports
 * into that power supply:
 *
 *   EXTCON_CHG_USB_DCP  -> dedicated-charger-microamp (default 2000000)
 *   EXTCON_CHG_USB_CDP  -> cdp-microamp               (default 1500000)
 *   EXTCON_CHG_USB_SDP or EXTCON_USB
 *                       -> default-microamp           (default  500000)
 *   nothing             -> 0, offline
 *
 * A plain VBUS extcon only ever says EXTCON_USB, so such a board is limited
 * to default-microamp whatever is plugged in: conservative, never too high.
 */

#include <linux/extcon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>

struct ecd {
	struct device *dev;
	struct extcon_dev *edev;
	struct power_supply *psy;
	struct notifier_block nb;
	u32 default_ua, cdp_ua, dcp_ua;
};

static int ecd_current(struct ecd *d)
{
	if (extcon_get_state(d->edev, EXTCON_CHG_USB_DCP) > 0)
		return d->dcp_ua;
	if (extcon_get_state(d->edev, EXTCON_CHG_USB_CDP) > 0)
		return d->cdp_ua;
	if (extcon_get_state(d->edev, EXTCON_CHG_USB_SDP) > 0 ||
	    extcon_get_state(d->edev, EXTCON_USB) > 0)
		return d->default_ua;
	return 0;
}

static int ecd_notify(struct notifier_block *nb, unsigned long state, void *p)
{
	struct ecd *d = container_of(nb, struct ecd, nb);

	power_supply_changed(d->psy);
	return NOTIFY_OK;
}

static int ecd_get_property(struct power_supply *psy,
			    enum power_supply_property prop,
			    union power_supply_propval *val)
{
	struct ecd *d = power_supply_get_drvdata(psy);
	int ua = ecd_current(d);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ua > 0;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = ua;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property ecd_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc ecd_desc = {
	.name = "usb-charger-detect",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ecd_props,
	.num_properties = ARRAY_SIZE(ecd_props),
	.get_property = ecd_get_property,
};

static int ecd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { .fwnode = dev_fwnode(dev) };
	struct ecd *d;
	int ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = dev;
	cfg.drv_data = d;

	d->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(d->edev))
		return dev_err_probe(dev, PTR_ERR(d->edev), "extcon\n");

	d->default_ua = 500000;
	d->cdp_ua = 1500000;
	d->dcp_ua = 2000000;
	device_property_read_u32(dev, "default-microamp", &d->default_ua);
	device_property_read_u32(dev, "cdp-microamp", &d->cdp_ua);
	device_property_read_u32(dev, "dedicated-charger-microamp", &d->dcp_ua);

	d->psy = devm_power_supply_register(dev, &ecd_desc, &cfg);
	if (IS_ERR(d->psy))
		return dev_err_probe(dev, PTR_ERR(d->psy), "power supply\n");

	d->nb.notifier_call = ecd_notify;
	ret = devm_extcon_register_notifier_all(dev, d->edev, &d->nb);
	if (ret)
		return dev_err_probe(dev, ret, "extcon notifier\n");

	power_supply_changed(d->psy);
	return 0;
}

static const struct of_device_id ecd_of_match[] = {
	{ .compatible = "linux,extcon-charger-detect" },
	{ }
};
MODULE_DEVICE_TABLE(of, ecd_of_match);

static struct platform_driver ecd_driver = {
	.probe = ecd_probe,
	.driver = {
		.name = "extcon-charger-detect",
		.of_match_table = ecd_of_match,
	},
};
module_platform_driver(ecd_driver);

MODULE_DESCRIPTION("USB charger power supply backed by an extcon");
MODULE_LICENSE("GPL");
