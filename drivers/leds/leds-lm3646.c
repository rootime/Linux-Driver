
/*
* Simple driver for Texas Instruments LM3646 LED Flash driver chip
* Copyright (C) 2013 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/platform_data/leds-lm3646.h>

#define REG_REV			0x00
#define REG_MODE		0x01
#define REG_STR_CTRL	0x04
#define REG_MAX_BR		0x05
#define REG_FLASH_BR	0x06
#define REG_TORCH_BR	0x07

#define STR_OFF "off"
#define NUM_OFF "0"

#define WARM_FLASH_CTRL_SIZE 8

struct warm_flash {
	u8 max_current;
	u8 led1_current;
};

enum lm3646_mode {
	MODE_STDBY = 0x0,
	MODE_TORCH = 0x2,
	MODE_FLASH = 0x3,
	MODE_MAX
};

enum lm3646_devfile {
	DFILE_FLASH_CTRL = 0,
	DFILE_FLASH_LED1,
	DFILE_FLASH_DUR,
	DFILE_TORCH_CTRL,
	DFILE_TORCH_LED1,
	DFILE_WARM_FLASH,
	DFILE_MAX
};

struct lm3646 {
	struct device *dev;

	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;

	struct work_struct work_flash;
	struct work_struct work_torch;

	u8 br_flash;
	u8 br_torch;

	struct lm3646_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;
};

static int lm3646_read_byte(struct lm3646 *pchip, u8 addr)
{
	int rval, ret;
	ret = regmap_read(pchip->regmap, addr, &rval);
	if (ret < 0)
		return ret;
	return rval;
}

static int lm3646_update_byte(struct lm3646 *pchip, u8 addr, u8 mask, u8 data)
{
	return regmap_update_bits(pchip->regmap, addr, mask, data);
}

static int lm3646_chip_init(struct lm3646 *pchip,
			    struct lm3646_platform_data *pdata)
{
	int rval;

	rval = lm3646_read_byte(pchip, REG_REV);
	if (rval < 0)
		goto out;
	dev_info(pchip->dev, "LM3646 CHIP_ID/REV[0x%x]\n", rval);

	if (pdata == NULL) {
		pdata =
		    kzalloc(sizeof(struct lm3646_platform_data), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->flash_imax = 0x0F;
		pdata->torch_imax = 0x07;
		pdata->led1_flash_imax = 0x7F;
		pdata->led1_torch_imax = 0x7F;
	}
	pchip->pdata = pdata;

	rval = lm3646_update_byte(pchip, REG_MODE, 0x08, pdata->tx_pin);
	if (rval < 0)
		goto out;
	rval = lm3646_update_byte(pchip, REG_TORCH_BR, 0xFF,
				  pdata->torch_pin | pdata->led1_torch_imax);
	if (rval < 0)
		goto out;
	rval = lm3646_update_byte(pchip, REG_FLASH_BR, 0xFF,
				  pdata->strobe_pin | pdata->led1_flash_imax);
	if (rval < 0)
		goto out;
	rval = lm3646_update_byte(pchip, REG_MAX_BR, 0x7F,
				  (pdata->torch_imax << 4) | pdata->flash_imax);
	if (rval < 0)
		goto out;
	pchip->br_flash = pdata->flash_imax;
	pchip->br_torch = pdata->torch_imax;

	return rval;
out:
	dev_err(pchip->dev, "i2c acces fail.\n");
	return rval;
}

static void lm3646_mode_ctrl(struct lm3646 *pchip,
			     const char *buf, enum lm3646_mode mode)
{
	int rval;

	if (strncmp(buf, STR_OFF, 3) == 0 || strncmp(buf, NUM_OFF, 1) == 0)
		mode = MODE_STDBY;

	mutex_lock(&pchip->lock);
	rval = lm3646_update_byte(pchip, REG_MODE, 0x03, mode);
	mutex_unlock(&pchip->lock);
	if (rval < 0)
		dev_err(pchip->dev, "i2c access fail.\n");
}

static void lm3646_input_control(struct lm3646 *pchip,
				 const char *buf, u8 reg, u8 mask)
{
	int rval, ival;

	rval = kstrtouint(buf, 10, &ival);
	if (rval) {
		dev_err(pchip->dev, "str to int fail.\n");
		return;
	}
	mutex_lock(&pchip->lock);
	rval = lm3646_update_byte(pchip, reg, mask, ival);
	mutex_unlock(&pchip->lock);
	if (rval < 0)
		dev_err(pchip->dev, "i2c access fail.\n");
}

/* torch brightness(max current) control */
static void lm3646_deferred_torch_brightness_set(struct work_struct *work)
{
	int rval;
	struct lm3646 *pchip = container_of(work, struct lm3646, work_torch);

	rval =
	    lm3646_update_byte(pchip, REG_MAX_BR, 0x70, (pchip->br_torch) << 4);
	if (rval < 0)
		dev_err(pchip->dev, "i2c access fail.\n");
}

static void lm3646_torch_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct lm3646 *pchip = container_of(cdev, struct lm3646, cdev_torch);

	pchip->br_torch = brightness;
	schedule_work(&pchip->work_torch);
}

/* torch on/off(mode) control */
static ssize_t lm3646_torch_ctrl_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_torch);

	lm3646_mode_ctrl(pchip, buf, MODE_TORCH);
	return size;
}

/* torch dual led control */
static ssize_t lm3646_torch_iled1_ctrl_store(struct device *dev,
					     struct device_attribute *devAttr,
					     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_torch);

	lm3646_input_control(pchip, buf, REG_TORCH_BR, 0x7F);
	return size;
}

/* flash brightness(max current) control */
static void lm3646_deferred_flash_brightness_set(struct work_struct *work)
{
	int rval;
	struct lm3646 *pchip = container_of(work, struct lm3646, work_flash);

	rval = lm3646_update_byte(pchip, REG_MAX_BR, 0x0F, pchip->br_flash);
	if (rval < 0)
		dev_err(pchip->dev, "i2c access fail.\n");
}

static void lm3646_flash_brightness_set(struct led_classdev *cdev,
					enum led_brightness brightness)
{
	struct lm3646 *pchip = container_of(cdev, struct lm3646, cdev_flash);

	pchip->br_flash = brightness;
	schedule_work(&pchip->work_flash);
}

/* flash on(mode) control */
static ssize_t lm3646_flash_ctrl_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_flash);

	lm3646_mode_ctrl(pchip, buf, MODE_FLASH);
	return size;
}

/* flash dual led control */
static ssize_t lm3646_flash_iled1_ctrl_store(struct device *dev,
					     struct device_attribute *devAttr,
					     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_flash);

	lm3646_input_control(pchip, buf, REG_FLASH_BR, 0x7F);
	return size;
}

/* flash duration(timeout) control */
static ssize_t lm3646_flash_duration_store(struct device *dev,
					   struct device_attribute *devAttr,
					   const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_flash);

	lm3646_input_control(pchip, buf, REG_STR_CTRL, 0x07);
	return size;
}

/* warm-flash setting data */
static struct warm_flash warm_flash_set[WARM_FLASH_CTRL_SIZE] = {
	/* LED1 = MAX, LED2 = Diabled */
	[0] = {0x0F, 0x7F},
	[1] = {0x0F, 0x3F},
	[2] = {0x0F, 0x1F},
	[3] = {0x0F, 0x0F},
	[4] = {0x0F, 0x07},
	[5] = {0x0F, 0x03},
	[6] = {0x0F, 0x01},
	/* LED1 = Diabled, LED2 = MAX */
	[7] = {0x0F, 0x00},
};

/* flash duration(timeout) control */
static ssize_t lm3646_warm_flash_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3646 *pchip =
	    container_of(led_cdev, struct lm3646, cdev_flash);

	int rval, ival;

	rval = kstrtouint(buf, 10, &ival);
	if (rval) {
		dev_err(pchip->dev, "str to int fail.\n");
		goto out_err;
	}

	if (ival > WARM_FLASH_CTRL_SIZE - 1) {
		dev_err(pchip->dev, "input error.\n");
		goto out_err;
	}

	mutex_lock(&pchip->lock);
	rval =
	    lm3646_update_byte(pchip, REG_MAX_BR, 0x0F,
			       warm_flash_set[ival].max_current);
	if (rval < 0)
		goto out;
	rval =
	    lm3646_update_byte(pchip, REG_FLASH_BR, 0x7F,
			       warm_flash_set[ival].led1_current);
	if (rval < 0)
		goto out;
	if (pchip->pdata->strobe_pin == LM3646_STROBE_PIN_DISABLED)
		lm3646_update_byte(pchip, REG_MODE, 0x03, MODE_FLASH);
out:
	mutex_unlock(&pchip->lock);
	if (rval < 0)
		dev_err(pchip->dev, "i2c access fail.\n");
out_err:
	return size;
}

#define lm3646_attr(_name, _show, _store)\
{\
	.attr = {\
		.name = _name,\
		.mode = 0644,\
	},\
	.show = _show,\
	.store = _store,\
}

static struct device_attribute dev_attr_ctrl[DFILE_MAX] = {
	[DFILE_FLASH_CTRL] = lm3646_attr("ctrl", NULL, lm3646_flash_ctrl_store),
	[DFILE_FLASH_LED1] =
	    lm3646_attr("iled1", NULL, lm3646_flash_iled1_ctrl_store),
	[DFILE_FLASH_DUR] = lm3646_attr("duration",
					NULL, lm3646_flash_duration_store),
	[DFILE_TORCH_CTRL] = lm3646_attr("ctrl", NULL, lm3646_torch_ctrl_store),
	[DFILE_TORCH_LED1] =
	    lm3646_attr("iled1", NULL, lm3646_torch_iled1_ctrl_store),
	[DFILE_WARM_FLASH] =
	    lm3646_attr("warmness", NULL, lm3646_warm_flash_store),
};

static const struct regmap_config lm3646_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

/* module initialize */
static int lm3646_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lm3646_platform_data *pdata = client->dev.platform_data;
	struct lm3646 *pchip;

	int err;
	/* i2c check */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3646), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	pchip->dev = &client->dev;
	pchip->regmap = devm_regmap_init_i2c(client, &lm3646_regmap);
	if (IS_ERR(pchip->regmap)) {
		err = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			err);
		return err;
	}
	mutex_init(&pchip->lock);
	i2c_set_clientdata(client, pchip);

	/* platform data check */
	err = lm3646_chip_init(pchip, pdata);
	if (err < 0)
		goto err_out;
	/* flash brightness control */
	INIT_WORK(&pchip->work_flash, lm3646_deferred_flash_brightness_set);
	pchip->cdev_flash.name = "flash";
	pchip->cdev_flash.max_brightness = 16;
	pchip->cdev_flash.brightness = pchip->br_flash;
	pchip->cdev_flash.brightness_set = lm3646_flash_brightness_set;
	pchip->cdev_flash.default_trigger = "flash";
	err = led_classdev_register((struct device *)
				    &client->dev, &pchip->cdev_flash);
	if (err < 0)
		goto err_out;
	/* flash on control */
	err = device_create_file(pchip->cdev_flash.dev,
				 &dev_attr_ctrl[DFILE_FLASH_CTRL]);
	if (err < 0)
		goto err_create_flash_ctrl_file;
	/* flash duration control */
	err = device_create_file(pchip->cdev_flash.dev,
				 &dev_attr_ctrl[DFILE_FLASH_DUR]);
	if (err < 0)
		goto err_create_flash_duration_file;
	/* flash - dual led control */
	err = device_create_file(pchip->cdev_flash.dev,
				 &dev_attr_ctrl[DFILE_FLASH_LED1]);
	if (err < 0)
		goto err_create_flash_iled1_file;

	/* flash - warmness input */
	err = device_create_file(pchip->cdev_flash.dev,
				 &dev_attr_ctrl[DFILE_WARM_FLASH]);
	if (err < 0)
		goto err_create_flash_warmness_file;

	/* torch brightness control */
	INIT_WORK(&pchip->work_torch, lm3646_deferred_torch_brightness_set);
	pchip->cdev_torch.name = "torch";
	pchip->cdev_torch.max_brightness = 8;
	pchip->cdev_torch.brightness = pchip->br_torch;
	pchip->cdev_torch.brightness_set = lm3646_torch_brightness_set;
	pchip->cdev_torch.default_trigger = "torch";
	err = led_classdev_register((struct device *)
				    &client->dev, &pchip->cdev_torch);
	if (err < 0)
		goto err_create_torch_file;
	/* torch on/off control */
	err = device_create_file(pchip->cdev_torch.dev,
				 &dev_attr_ctrl[DFILE_TORCH_CTRL]);
	if (err < 0)
		goto err_create_torch_ctrl_file;
	/* torch - dual led control */
	err = device_create_file(pchip->cdev_torch.dev,
				 &dev_attr_ctrl[DFILE_TORCH_LED1]);
	if (err < 0)
		goto err_create_torch_iled1_file;
	return 0;

err_create_torch_iled1_file:
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_TORCH_CTRL]);
err_create_torch_ctrl_file:
	led_classdev_unregister(&pchip->cdev_torch);
err_create_torch_file:
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_WARM_FLASH]);
err_create_flash_warmness_file:
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_FLASH_LED1]);
err_create_flash_iled1_file:
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_FLASH_DUR]);
err_create_flash_duration_file:
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_FLASH_CTRL]);
err_create_flash_ctrl_file:
	led_classdev_unregister(&pchip->cdev_flash);
err_out:
	return err;
}

static int lm3646_remove(struct i2c_client *client)
{
	struct lm3646 *pchip = i2c_get_clientdata(client);
	/* set standby mode */
	lm3646_update_byte(pchip, REG_MODE, 0x03, MODE_STDBY);
	/* flash */
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_FLASH_LED1]);
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_FLASH_DUR]);
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_TORCH_CTRL]);
	device_remove_file(pchip->cdev_flash.dev,
			   &dev_attr_ctrl[DFILE_WARM_FLASH]);
	led_classdev_unregister(&pchip->cdev_flash);
	/* torch */
	device_remove_file(pchip->cdev_torch.dev,
			   &dev_attr_ctrl[DFILE_TORCH_LED1]);
	device_remove_file(pchip->cdev_torch.dev,
			   &dev_attr_ctrl[DFILE_TORCH_CTRL]);
	led_classdev_unregister(&pchip->cdev_torch);

	return 0;
}

static const struct i2c_device_id lm3646_id[] = {
	{LM3646_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3646_id);

static struct i2c_driver lm3646_i2c_driver = {
	.driver = {
		   .name = LM3646_NAME,
		   .owner = THIS_MODULE,
		   .pm = NULL,
		   },
	.probe = lm3646_probe,
	.remove = lm3646_remove,
	.id_table = lm3646_id,
};

module_i2c_driver(lm3646_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Flash Lighting driver for LM3646");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL v2");
