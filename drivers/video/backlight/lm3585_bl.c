/*
 * Simple driver for Texas Instruments lm3585 Backlight driver chip
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: Daniel Jeong  <gshark.jeong@gmail.com>
 *		  Ldd Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_data/lm3585_bl.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define REG_CTRL				0x00
#define REG_ENABLE				0x01
#define REG_LED_EN_LO			0x02
#define REG_LED_EN_HI			0x03
#define REG_FAULT				0x04
#define REG_BBOOST_MODE			0x06
#define REG_BBOOST_OUTV			0x07
#define REG_BBOOST_CTRL			0x08
#define REG_BOOSTV				0x0a
#define REG_RAMP_DURATION		0x0f
#define REG_RAMP_TAR_LO			0x10
#define REG_RAMP_TAR_HI			0x11
#define REG_RAMP_TAR_CTRL		0x12
#define REG_READ_RAMP_LO		0x14
#define REG_READ_RAMP_HI		0x15
#define REG_READ_CABC_PWM_LO	0x16
#define REG_READ_CABC_PWM_HI	0x17
#define REG_READ_LED_DC_LO		0x18
#define REG_READ_LED_DC_HI		0x19
#define REG_REVISION			0x1f
#define REG_BBOOST_TRIM			0x25
#define REG_MAX	0xff

#define LM3585_ENABLED 1
#define LM3585_DISABLED 0

struct lm3585_chip {
	struct device *dev;
	struct lm3585_platform_data *pdata;
	struct backlight_device *bled;
	struct regmap *regmap;
	int is_enabled;
};

static int lm3585_startup(struct lm3585_chip *pchip)
{
	unsigned int rev1, rev2;
	int ret;

	/* POR wait */
	msleep(1);
	/* sw reset */
	ret = regmap_write(pchip->regmap, REG_CTRL, 0x01);
	msleep(1);
	/* LED enable */
	ret |= regmap_write(pchip->regmap,
			    REG_LED_EN_LO, pchip->pdata->led_enable);
	ret |= regmap_update_bits(pchip->regmap,
				  REG_LED_EN_HI, 0x03,
				  pchip->pdata->led_enable >> 8);

	/* Vpos 5.1V */
	ret |= regmap_write(pchip->regmap, REG_BOOSTV, 0x07);

	/*
	 * revision info and set buck/boost current limit
	 * A0 - 0x00
	 * A1 - 0x01
	 * B0 - 0x04 & R25[1:0] : 11b
	 * B1 - 0x04 & R25[1:0] : 10b
	 * B2 - 0x05
	 */
	ret |= regmap_read(pchip->regmap, REG_REVISION, &rev1);
	switch (rev1) {
	case 0x00:
	case 0x01:
		ret |= regmap_write(pchip->regmap, REG_BBOOST_CTRL, 0x4A);
		break;
	case 0x04:
		ret |= regmap_read(pchip->regmap, REG_BBOOST_TRIM, &rev2);
		switch (rev2 & 0x03) {
		case 0x2:
			ret |=
			    regmap_write(pchip->regmap, REG_BBOOST_CTRL, 0x4A);
			break;
		case 0x3:
			ret |=
			    regmap_write(pchip->regmap, REG_BBOOST_CTRL, 0x0A);
			break;
		}
		break;
	case 0x05:
		ret |= regmap_write(pchip->regmap, REG_BBOOST_CTRL, 0x0A);
		break;
	}
	/* enable buck boost & wait soft start time */
	ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x01);
	msleep(10);
	/* enable charge pump discharge */
	ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x09);
	msleep(100);
	/* enable boost converter */
	ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x0b);
	msleep(1);
	/* enable charge pump Vneg */
	ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x0f);
	/* enable global led */
	ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x2f);
	/* enable cabc pwm input control */
	if (pchip->pdata->cabc_input != 0)
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x3f);

	/* set LED current to 0 initially */
	ret |= regmap_write(pchip->regmap, REG_RAMP_TAR_LO, 0x00);
	ret |= regmap_update_bits(pchip->regmap, REG_RAMP_TAR_HI,
				  0xcf, (pchip->pdata->ctype << 6) | 0x00);
	/* ramp duration */
	ret |= regmap_write(pchip->regmap,
			    REG_RAMP_DURATION, pchip->pdata->ramp_time);
	/* adgo */
	ret |= regmap_write(pchip->regmap, REG_RAMP_TAR_CTRL, 0x01);

	if (ret == 0)
		pchip->is_enabled = LM3585_ENABLED;
	return ret;
}

static ssize_t lm3585_reset_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct lm3585_chip *pchip = dev_get_drvdata(dev);
	u8 input;
	int ret;

	if (kstrtou8(buf, 0, &input))
		return -EINVAL;

	if (input == 1) {
		ret = lm3585_startup(pchip);
		if (ret < 0)
			return ret;
	}
	return size;
}

static ssize_t lm3585_fault_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct lm3585_chip *pchip = dev_get_drvdata(dev);
	unsigned int fault;
	int ret;

	ret = regmap_read(pchip->regmap, REG_FAULT, &fault);
	if (ret < 0) {
		dev_err(pchip->dev, "fail : i2c access to register.\n");
		return sprintf(buf, "%d\n", ret);
	}

	if (fault & 0x1)
		dev_err(pchip->dev, "LM3585 Temperature Fault.\n");
	if (fault & 0x2)
		dev_err(pchip->dev, "LM3585 Boost Over Voltage Fault.\n");
	if (fault & 0x4)
		dev_err(pchip->dev, "LM3585 LED Short Fault.\n");
	if (fault & 0x8)
		dev_err(pchip->dev, "LM3585 LED Open Fault.\n");

	return sprintf(buf, "%d\n", fault);
}

static ssize_t lm3585_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct lm3585_chip *pchip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pchip->is_enabled);
}

static ssize_t lm3585_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct lm3585_chip *pchip = dev_get_drvdata(dev);
	u8 input;
	int ret;

	if (kstrtou8(buf, 0, &input))
		return -EINVAL;

	if (input == LM3585_DISABLED) {
		/* turn off backlight */
		ret = regmap_write(pchip->regmap, REG_ENABLE, 0x0f);
		/* turn off charge pump */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x0b);
		/* turn off boost */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x09);
		msleep(100);
		/* turn off charge pump discharge */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x01);
		/* turn off buck boost */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x00);
	} else {
		/* turn on Vout */
		ret = regmap_write(pchip->regmap, REG_ENABLE, 0x01);
		/* turn on the Vpos */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x03);
		/* enable charger pump */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x0F);
		/* turn on backlight */
		ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x2F);
		/* enable cabc pwm control */
		if (pchip->pdata->cabc_input != 0)
			ret |= regmap_write(pchip->regmap, REG_ENABLE, 0x3F);
	}
	if (ret < 0)
		return ret;
	pchip->is_enabled = input;

	return size;
}

#define LM3585_DVNODE_MAX 3
#define lm3585_attr(_name, _show, _store)\
{\
	.attr = {\
		.name = _name,\
		.mode = S_IWUSR | S_IRUSR,\
	},\
	.show = _show,\
	.store = _store,\
}

static struct device_attribute lm3585_dev_attr[] = {
	lm3585_attr("fault", lm3585_fault_show, NULL),
	lm3585_attr("enable", lm3585_enable_show, lm3585_enable_store),
	lm3585_attr("reset", NULL, lm3585_reset_store),
};

/* brightness control */
static int lm3585_update_status(struct backlight_device *bl)
{
	int ret = -EINVAL;
	struct lm3585_chip *pchip = bl_get_data(bl);

	if (bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		bl->props.brightness = 0;

	/* adstop */
	ret = regmap_write(pchip->regmap, REG_RAMP_TAR_CTRL, 0x02);
	/* set target iled low */
	ret |= regmap_write(pchip->regmap,
			    REG_RAMP_TAR_LO, bl->props.brightness & 0xff);
	/* set target iled high */
	ret |= regmap_update_bits(pchip->regmap,
				  REG_RAMP_TAR_HI, 0x0f,
				  bl->props.brightness >> 8);
	/* ramp duration */
	ret |= regmap_write(pchip->regmap,
			    REG_RAMP_DURATION, pchip->pdata->ramp_time);
	/* adgo */
	ret |= regmap_write(pchip->regmap, REG_RAMP_TAR_CTRL, 0x01);

	if (ret < 0)
		dev_err(pchip->dev, "fail : i2c access to register.\n");
	else
		ret = bl->props.brightness;

	return ret;
}

static int lm3585_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops lm3585_bled_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3585_update_status,
	.get_brightness = lm3585_get_brightness,
};

static const struct regmap_config lm3585_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

#ifdef CONFIG_OF
static struct lm3585_platform_data *lm3585_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct lm3585_platform_data *pdata = NULL;
	u32 rdata;

	if (!node) {
		dev_err(&client->dev, "can't find data in device tree\n");
		return pdata;
	}

	pdata = devm_kzalloc(&client->dev,
			     sizeof(struct lm3585_platform_data), GFP_KERNEL);
	if (pdata == NULL)
		return pdata;

	of_property_read_u32(node, "led_enable", &rdata);
	pdata->led_enable = (enum lm3585_leds_enable)rdata;
	of_property_read_u32(node, "ctype", &rdata);
	pdata->ctype = (enum lm3585_curve_type)rdata;
	of_property_read_u32(node, "cabc_input", &rdata);
	pdata->cabc_input = (enum lm3585_cabc_input)rdata;
	of_property_read_u32(node, "ramp_time", &rdata);
	pdata->ramp_time = rdata;

	return pdata;
}

static const struct of_device_id of_lm3585_bl_match[] = {
	{.compatible = "ti,lm3585",},
	{},
};

#else
static struct lm3585_platform_data *lm3585_parse_dt(struct i2c_client *client)
{
	return NULL;
}
#endif

static int lm3585_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lm3585_chip *pchip;
	struct lm3585_platform_data *pdata = dev_get_platdata(&client->dev);
	struct backlight_properties props;
	int ret, icnt;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check.\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev,
			     sizeof(struct lm3585_chip), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &lm3585_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(pchip->dev, "fail : allocate i2c register map.\n");
		return ret;
	}

	if (pdata == NULL) {
		pdata = lm3585_parse_dt(client);
		if (pdata == NULL) {
			pdata = devm_kzalloc(pchip->dev,
					     sizeof(struct
						    lm3585_platform_data),
					     GFP_KERNEL);
			if (pdata == NULL) {
				dev_err(pchip->dev, "fail to get memory\n");
				return -ENOMEM;
			}
			pdata->ramp_time = 0;
			pdata->ctype = LM3585_CURVE_LINEAR;
			pdata->led_enable = LM3585_LED_ALL_ON;
			pdata->cabc_input = LM3585_CABC_INPUT_DISABLE;
		}
	}
	pchip->pdata = pdata;

	i2c_set_clientdata(client, pchip);

	ret = lm3585_startup(pchip);
	if (ret < 0) {
		dev_err(pchip->dev, "fail : start up.\n");
		return ret;
	}

	props.brightness = 0;
	props.type = BACKLIGHT_RAW;
	props.max_brightness = LM3585_BR_MAX;
	pchip->bled = devm_backlight_device_register(pchip->dev,
						     LM3585_NAME, pchip->dev,
						     pchip, &lm3585_bled_ops,
						     &props);
	if (IS_ERR(pchip->bled)) {
		dev_err(pchip->dev, "fail : backlight register.\n");
		ret = PTR_ERR(pchip->bled);
		return ret;
	}

	for (icnt = 0; icnt < LM3585_DVNODE_MAX; icnt++) {
		ret =
		    device_create_file(&(pchip->bled->dev),
				       &lm3585_dev_attr[icnt]);
		if (ret < 0) {
			dev_err(pchip->dev, "fail : node create\n");
			goto err_dn_out;
		}
	}

	dev_info(pchip->dev, "LM3585 backlight Initialized.\n");
	return 0;

err_dn_out:
	while (--icnt >= 0)
		device_remove_file(&(pchip->bled->dev), &lm3585_dev_attr[icnt]);

	return ret;
}

static int lm3585_remove(struct i2c_client *client)
{
	struct lm3585_chip *pchip = i2c_get_clientdata(client);
	int icnt;

	for (icnt = 0; icnt < LM3585_DVNODE_MAX; icnt++)
		device_remove_file(&(pchip->bled->dev), &lm3585_dev_attr[icnt]);
	return 0;
}

static const struct i2c_device_id lm3585_id[] = {
	{LM3585_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3585_id);
static struct i2c_driver lm3585_i2c_driver = {
	.driver = {
		   .name = LM3585_NAME,
		   },
	.probe = lm3585_probe,
	.remove = lm3585_remove,
	.id_table = lm3585_id,
};

module_i2c_driver(lm3585_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments LM3585 Backlight Driver");
MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_LICENSE("GPL v2");
