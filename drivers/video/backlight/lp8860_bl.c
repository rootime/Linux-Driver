/*
* Simple driver for Texas Instruments lp8860 Backlight driver chip
*
* Copyright (C) 2014 Texas Instruments
* Author: Daniel Jeong  <gshark.jeong@gmail.com>
* 		  Ldd Mlp <ldd-mlp@list.ti.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_data/lp8860_bl.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define REG_CL0_BRT_H	0x00
#define REG_CL0_BRT_L	0x01
#define REG_CL0_I_H		0x02
#define REG_CL0_I_L		0x03

#define REG_CL1_BRT_H	0x04
#define REG_CL1_BRT_L	0x05
#define REG_CL1_I		0x06

#define REG_CL2_BRT_H	0x07
#define REG_CL2_BRT_L	0x08
#define REG_CL2_I		0x09

#define REG_CL3_BRT_H	0x0a
#define REG_CL3_BRT_L	0x0b
#define REG_CL3_I		0x0c

#define REG_CONF	0x0d
#define REG_STATUS	0x0e
#define REG_ID		0x12

#define REG_ROM_CTRL 	0x19
#define REG_ROM_ULOCK	0x1a
#define REG_ROM_START	0x60
#define REG_ROM_END		0x78

#define REG_EEPROM_START	0x60
#define REG_EEPROM_END		0x78
#define REG_MAX	0xFF

#define ULCODE0	0x08
#define ULCODE1	0xba
#define ULCODE2	0xef
#define LCODE	0x00

struct lp8860_chip {
	struct device *dev;

	struct lp8860_platform_data *pdata;
	struct backlight_device *bled[LP8860_LED_MAX];
	struct regmap *regmap;

	/* mlp test interface */
	unsigned int reg;
	unsigned int reg_data;
};

/* brightness control */
static int lp8860_bled_update_status(struct backlight_device *bl,
				     enum lp8860_leds nsr)
{
	int ret = -EINVAL;
	struct lp8860_chip *pchip = bl_get_data(bl);

	if (pchip->pdata->mode)
		return 0;

	if (bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		bl->props.brightness = 0;

	switch (nsr) {
	case LP8860_LED0:
		ret = regmap_write(pchip->regmap,
				   REG_CL0_BRT_H, bl->props.brightness >> 8);
		ret |= regmap_write(pchip->regmap,
				    REG_CL0_BRT_L, bl->props.brightness & 0xff);
		break;
	case LP8860_LED1:
		ret = regmap_write(pchip->regmap,
				   REG_CL1_BRT_H,
				   (bl->props.brightness >> 8) & 0x1f);
		ret |= regmap_write(pchip->regmap, REG_CL1_BRT_L,
				 bl->props.brightness & 0xff);
		break;
	case LP8860_LED2:
		ret = regmap_write(pchip->regmap,
				   REG_CL2_BRT_H,
				   (bl->props.brightness >> 8) & 0x1f);
		ret |= regmap_write(pchip->regmap, REG_CL2_BRT_L,
				 bl->props.brightness & 0xff);
		break;
	case LP8860_LED3:
		ret = regmap_write(pchip->regmap,
				   REG_CL3_BRT_H,
				   (bl->props.brightness >> 8) & 0x1f);
		ret |= regmap_write(pchip->regmap, REG_CL3_BRT_L,
				 bl->props.brightness & 0xff);
		break;
	default:
		BUG();
	}
	if (ret < 0)
		dev_err(pchip->dev, "fail : i2c access to register.\n");
	else
		ret = bl->props.brightness;

	return ret;
}

static int lp8860_bled_get_brightness(struct backlight_device *bl,
				      enum lp8860_leds nsr)
{
	struct lp8860_chip *pchip = bl_get_data(bl);
	unsigned int rval_h, rval_l;
	int ret = -EINVAL;

	switch (nsr) {
	case LP8860_LED0:
		ret = regmap_read(pchip->regmap, REG_CL0_BRT_H, &rval_h);
		ret |= regmap_read(pchip->regmap, REG_CL0_BRT_L, &rval_l);
		break;
	case LP8860_LED1:
		ret = regmap_read(pchip->regmap, REG_CL1_BRT_H, &rval_h);
		ret |= regmap_read(pchip->regmap, REG_CL1_BRT_L, &rval_l);
		break;
	case LP8860_LED2:
		ret = regmap_read(pchip->regmap, REG_CL2_BRT_H, &rval_h);
		ret |= regmap_read(pchip->regmap, REG_CL2_BRT_L, &rval_l);
		break;
	case LP8860_LED3:
		ret = regmap_read(pchip->regmap, REG_CL3_BRT_H, &rval_h);
		ret |= regmap_read(pchip->regmap, REG_CL3_BRT_L, &rval_l);
		break;
	default:
		BUG();
	}
	if (ret < 0) {
		dev_err(pchip->dev, "fail : i2c access to register.\n");
		return ret;
	}
	bl->props.brightness = (rval_h << 8) | rval_l;
	return bl->props.brightness;
}

static int lp8860_update_status_bled0(struct backlight_device *bl)
{
	return lp8860_bled_update_status(bl, LP8860_LED0);
}

static int lp8860_get_brightness_bled0(struct backlight_device *bl)
{
	return lp8860_bled_get_brightness(bl, LP8860_LED0);
}

static int lp8860_update_status_bled1(struct backlight_device *bl)
{
	return lp8860_bled_update_status(bl, LP8860_LED1);
}

static int lp8860_get_brightness_bled1(struct backlight_device *bl)
{
	return lp8860_bled_get_brightness(bl, LP8860_LED1);
}

static int lp8860_update_status_bled2(struct backlight_device *bl)
{
	return lp8860_bled_update_status(bl, LP8860_LED2);
}

static int lp8860_get_brightness_bled2(struct backlight_device *bl)
{
	return lp8860_bled_get_brightness(bl, LP8860_LED2);
}

static int lp8860_update_status_bled3(struct backlight_device *bl)
{
	return lp8860_bled_update_status(bl, LP8860_LED3);
}

static int lp8860_get_brightness_bled3(struct backlight_device *bl)
{
	return lp8860_bled_get_brightness(bl, LP8860_LED3);
}

#define lp8860_bled_ops(_id)\
{\
	.options = BL_CORE_SUSPENDRESUME,\
	.update_status = lp8860_update_status_bled##_id,\
	.get_brightness = lp8860_get_brightness_bled##_id,\
}

static const struct backlight_ops lp8860_bled_ops[LP8860_LED_MAX] = {
	[LP8860_LED0] = lp8860_bled_ops(0),
	[LP8860_LED1] = lp8860_bled_ops(1),
	[LP8860_LED2] = lp8860_bled_ops(2),
	[LP8860_LED3] = lp8860_bled_ops(3),
};

/* current control */
static int lp8860_set_current(struct device *dev,
			      const char *buf, enum lp8860_leds nsr)
{
	struct lp8860_chip *pchip = dev_get_drvdata(dev);
	unsigned int ival;
	ssize_t ret;

	ret = kstrtouint(buf, 10, &ival);
	if (ret)
		return ret;

	switch (nsr) {
	case LP8860_LED0:
		ival = min_t(unsigned int, ival, LP8860_LED0_BR_MAX);
		ret = regmap_write(pchip->regmap, REG_CL0_I_H, ival >> 8);
		ret |= regmap_write(pchip->regmap, REG_CL0_I_L, ival & 0xff);
		break;
	case LP8860_LED1:
		ival = min_t(unsigned int, ival, LP8860_LED1_BR_MAX);
		ret = regmap_write(pchip->regmap, REG_CL1_I, ival & 0xff);
		break;
	case LP8860_LED2:
		ival = min_t(unsigned int, ival, LP8860_LED2_BR_MAX);
		ret = regmap_write(pchip->regmap, REG_CL2_I, ival & 0xff);
		break;
	case LP8860_LED3:
		ival = min_t(unsigned int, ival, LP8860_LED3_BR_MAX);
		ret = regmap_write(pchip->regmap, REG_CL3_I, ival & 0xff);
		break;
	default:
		BUG();
	}
	if (ret < 0)
		dev_err(pchip->dev, "fail : i2c access error.\n");

	return ret;
}

static ssize_t lp8860_current_store_bled0(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	int ret;

	ret = lp8860_set_current(dev, buf, LP8860_LED0);
	if (ret < 0)
		return ret;
	return size;
}

static ssize_t lp8860_current_store_bled1(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	int ret;

	ret = lp8860_set_current(dev, buf, LP8860_LED1);
	if (ret < 0)
		return ret;
	return size;
}

static ssize_t lp8860_current_store_bled2(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	int ret;

	ret = lp8860_set_current(dev, buf, LP8860_LED2);
	if (ret < 0)
		return ret;
	return size;
}

static ssize_t lp8860_current_store_bled3(struct device *dev,
					  struct device_attribute *devAttr,
					  const char *buf, size_t size)
{
	int ret;

	ret = lp8860_set_current(dev, buf, LP8860_LED3);
	if (ret < 0)
		return ret;
	return size;
}

#define lp8860_attr(_name, _show, _store)\
{\
	.attr = {\
		.name = _name,\
		.mode = S_IWUSR|S_IRUSR,\
	},\
	.show = _show,\
	.store = _store,\
}

static struct device_attribute lp8860_dev_attr[LP8860_LED_MAX] = {
	[LP8860_LED0] = lp8860_attr("current", NULL,
				    lp8860_current_store_bled0),
	[LP8860_LED1] = lp8860_attr("current", NULL,
				    lp8860_current_store_bled1),
	[LP8860_LED2] = lp8860_attr("current", NULL,
				    lp8860_current_store_bled2),
	[LP8860_LED3] = lp8860_attr("current", NULL,
				    lp8860_current_store_bled3),
};

/* get argument from input buffer */
static int token_store_data(char *buf, char *cmd, unsigned int *reg,
			    unsigned int *mask, unsigned int *data)
{
	char *token;
	char *desc = " >\t\n";
	unsigned int tokenval;

	/* command */
	token = strsep(&buf, desc);
	if (token == NULL)
		goto err_out;
	*cmd = *token;
	if (*cmd == 'w' || *cmd == 'r') {
		/* register no. to read/write */
		token = strsep(&buf, desc);
		if (token == NULL)
			goto err_out;
		if (kstrtouint((const char *)token, 16, &tokenval) < 0)
			goto err_out;
		*reg = tokenval;

		/* write command  */
		if (*cmd == 'w') {
			/* mask bit */
			token = strsep(&buf, desc);
			if (token == NULL)
				goto err_out;
			if (kstrtouint((const char *)token, 16, &tokenval) < 0)
				goto err_out;
			*mask = tokenval;

			/* write data */
			token = strsep(&buf, desc);
			if (token == NULL)
				goto err_out;
			if (kstrtouint((const char *)token, 16, &tokenval) < 0)
				goto err_out;
			*data = tokenval;
		} else {
			*mask = 0;
			*data = 0;
		}
		return 0;
	}
err_out:
	return -EINVAL;
}

/* register access */
static ssize_t lp8860_register_show(struct device *dev,
				    struct device_attribute *devAttr, char *buf)
{
	struct lp8860_chip *pchip = dev_get_drvdata(dev);

	if (pchip->reg < 0x0 || pchip->reg > 0xff)
		return sprintf(buf, "Fail : invalid access %x\n", pchip->reg);
	return sprintf(buf, "%x %x", pchip->reg, pchip->reg_data);
}

static ssize_t lp8860_register_store(struct device *dev,
				     struct device_attribute *devAttr,
				     const char *buf, size_t size)
{
	struct lp8860_chip *pchip = dev_get_drvdata(dev);
	int ret;
	char cmd;
	unsigned int reg, mask, data;

	ret = token_store_data((char *)buf, &cmd, &reg, &mask, &data);
	if (ret < 0)
		goto err_input;
	pchip->reg = reg;
	if (cmd == 'w' || cmd == 'W') {
		ret = regmap_update_bits(pchip->regmap, reg, mask, data);
		if (ret < 0) {
			pchip->reg_data = ret;
			goto err_i2c;
		}
	}
	ret = regmap_read(pchip->regmap, reg, &data);
	if (ret < 0) {
		pchip->reg_data = ret;
		goto err_i2c;
	}
	pchip->reg_data = data;
	return size;

err_i2c:
	dev_err(pchip->dev, "fail : i2c access error\n");
	return size;

err_input:
	dev_err(pchip->dev, "failed : input fail\n");
	return size;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUSR,
		   lp8860_register_show, lp8860_register_store);

/*
 * Input data
 * 0 : read from eeprom to eeprom registers.
 * 1 : eeprom unlock before writing
 * 2 : brun and lock
 */
static ssize_t lp8860_eeprom_store(struct device *dev,
				   struct device_attribute *devAttr,
				   const char *buf, size_t size)
{
	struct lp8860_chip *pchip = dev_get_drvdata(dev);
	unsigned int ival;
	int ret;

	ret = kstrtouint(buf, 10, &ival);
	if (ret)
		goto err_input;

	switch (ival) {
		/*
		 * Read sequence 
		 * 1. Unlock EEPROM by writing
		 *    the unlock codes to register 1Ah(08, BA, EF)
		 * 2. Write EE_PROG to 1 in address 19h. (02h to address 19h)
		 * 3. Wait 100ms
		 * 4. Write EE_PROG to 0 in address 19h. (00h to address 19h)
		 * 5. Lock EEPROM by writing 0h to register 1Ah.
		 */
	case 0:
		ret = regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE0);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE1);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE2);
		ret |= regmap_write(pchip->regmap, REG_ROM_CTRL, 0x01);
		msleep(100);
		ret |= regmap_write(pchip->regmap, REG_ROM_CTRL, 0x00);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, LCODE);
		if (ret < 0)
			goto err_i2c;
		dev_info(pchip->dev, "LP8860 EEPROM Read Done\n");
		break;
		/*
		 * Programming sequence 
		 *    (program data permanently from registers to NVM)
		 * 1. Unlock EEPROM by writing
		 *    the unlock codes to register 1Ah(08, BA, EF)
		 * 2. Write data to EEPROM registers (address 60h...78h)
		 *    (use registers device node)
		 * 3. Write EE_PROG to 1 in address 19h. (02h to address 19h)
		 * 4. Wait 100ms
		 * 5. Write EE_PROG to 0 in address 19h. (00h to address 19h)
		 * 6. Lock EEPROM by writing 0h to register 1Ah.
		 */
	case 1:
		ret = regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE0);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE1);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, ULCODE2);
		if (ret < 0)
			goto err_i2c;
		dev_info(pchip->dev, "LP8860 EEPROM is Unlocked\n");
		break;
	case 2:
		ret = regmap_write(pchip->regmap, REG_ROM_CTRL, 0x02);
		msleep(100);
		ret |= regmap_write(pchip->regmap, REG_ROM_CTRL, 0x00);
		ret |= regmap_write(pchip->regmap, REG_ROM_ULOCK, LCODE);
		if (ret < 0)
			goto err_i2c;
		dev_info(pchip->dev, "LP8860 EEPROM is Locked\n");
		break;
	default:
		goto err_input;
	}

	return size;

err_i2c:
	dev_err(pchip->dev, "fail : i2c access error.\n");
	return ret;

err_input:
	dev_err(pchip->dev, "fail : input fail.\n");
	return -EINVAL;
}

static DEVICE_ATTR(eeprom, S_IWUSR, NULL, lp8860_eeprom_store);

/* backlight register and remove */
static char *lp8860_bled_name[LP8860_LED_MAX] = {
	[LP8860_LED0] = "bled0",
	[LP8860_LED1] = "bled1",
	[LP8860_LED2] = "bled2",
	[LP8860_LED3] = "bled3",
};

static int lp8860_backlight_remove(struct lp8860_chip *pchip)
{
	int icnt;

	device_remove_file(&(pchip->bled[0]->dev), &dev_attr_registers);
	device_remove_file(&(pchip->bled[0]->dev), &dev_attr_eeprom);
	for (icnt = LP8860_LED0; icnt < LP8860_LED_MAX; icnt++) {
		if (pchip->bled[icnt]) {
			backlight_device_unregister(pchip->bled[icnt]);
			device_remove_file(&(pchip->bled[icnt]->dev),
					   &lp8860_dev_attr[icnt]);
		}
	}
	return 0;
}

static int lp8860_backlight_registers(struct lp8860_chip *pchip)
{
	struct backlight_properties props;
	struct lp8860_platform_data *pdata = pchip->pdata;
	int icnt, ret;

	props.type = BACKLIGHT_RAW;
	for (icnt = LP8860_LED0; icnt < LP8860_LED_MAX; icnt++) {
		props.max_brightness = pdata->max_brt[icnt];
		pchip->bled[icnt] =
		    backlight_device_register(lp8860_bled_name[icnt],
					      pchip->dev, pchip,
					      &lp8860_bled_ops[icnt], &props);
		if (IS_ERR(pchip->bled[icnt])) {
			dev_err(pchip->dev, "fail : backlight register.\n");
			ret = PTR_ERR(pchip->bled);
			goto err_out;
		}

		ret = device_create_file(&(pchip->bled[icnt]->dev),
					 &lp8860_dev_attr[icnt]);
		if (ret < 0) {
			dev_err(pchip->dev, "fail : to add sysfs entries.\n");
			goto err_out;
		}
	}
	/* to access eeprom */
	ret = device_create_file(&(pchip->bled[LP8860_LED0]->dev),
				 &dev_attr_eeprom);
	if (ret < 0) {
		dev_err(pchip->dev, "fail : to add sysfs entries.\n");
		goto err_out;
	}
	/* to access registers */
	ret = device_create_file(&(pchip->bled[LP8860_LED0]->dev),
				 &dev_attr_registers);
	if (ret < 0) {
		dev_err(pchip->dev, "failed : add sysfs entries\n");
		goto err_out;
	}
	return 0;

err_out:
	lp8860_backlight_remove(pchip);
	return ret;
}

static const struct regmap_config lp8860_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int lp8860_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lp8860_chip *pchip;
	struct lp8860_platform_data *pdata = dev_get_platdata(&client->dev);
	int ret, icnt;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check.\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev,
			     sizeof(struct lp8860_chip), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &lp8860_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(pchip->dev, "fail : allocate i2c register map.\n");
		return ret;
	}

	if (pdata == NULL) {
		pdata = devm_kzalloc(pchip->dev,
				     sizeof(struct lp8860_platform_data),
				     GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;
		pdata->max_brt[LP8860_LED0] = LP8860_LED0_BR_MAX;
		for (icnt = LP8860_LED1; icnt < LP8860_LED_MAX; icnt++)
			pdata->max_brt[icnt] = LP8860_LED1_BR_MAX;
		pchip->pdata = pdata;
	} else {
		pchip->pdata = pdata;
	}
	i2c_set_clientdata(client, pchip);
	ret = lp8860_backlight_registers(pchip);
	return ret;
}

static int lp8860_remove(struct i2c_client *client)
{
	return lp8860_backlight_remove(i2c_get_clientdata(client));
}

static const struct i2c_device_id lp8860_id[] = {
	{LP8860_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lp8860_id);
static struct i2c_driver lp8860_i2c_driver = {
	.driver = {
		   .name = LP8860_NAME,
		   },
	.probe = lp8860_probe,
	.remove = lp8860_remove,
	.id_table = lp8860_id,
};

module_i2c_driver(lp8860_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments LP8860 Backlight Driver");
MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_LICENSE("GPL v2");
