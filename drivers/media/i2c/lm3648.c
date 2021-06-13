/*
 * drivers/media/i2c/lm3648.c
 * General device driver for TI lm3648, Dual FLASH LED Driver
 *
 * Copyright (C) 2014 Texas Instruments
 *
 * Contact: Daniel Jeong <gshark.jeong@gmail.com>
 *			Ldd-Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <media/lm3648.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

/* registers definitions */
#define REG_ENABLE		0x01
#define REG_FLASH_BR	0x03
#define REG_TORCH_BR	0x05
#define REG_FLASH_TOUT	0x08
#define REG_FLAG1		0x0a
#define REG_FLAG2		0x0b
#define REG_STROBE_SRC	0x01

#define MASK_ENABLE		0x0c
#define MASK_TORCH_BR	0x7f
#define MASK_FLASH_BR	0x3f
#define MASK_FLASH_TOUT	0x0f
#define MASK_FLAG1		0xff
#define MASK_FLAG2		0x1f
#define MASK_STROBE_SRC	0x20

/* Fault1 Mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_UVLO		(1<<1)
#define FAULT_OVERTEMP	(1<<2)
#define FAULT_OCP		(1<<3)
#define FAULT_SHORT_CIRCUIT	(0x7<<4)

/* Fault2 Mask */
#define FAULT_NTC_TRIP	(0x19<<0)
#define FAULT_OVP		(1<<1)
#define FAULT_IVFM		(1<<2)

enum led_mode {
	MODE_SHDN	= 0x00,
	MODE_IR_DRV	= 0x04,
	MODE_TORCH	= 0x08,
	MODE_FLASH	= 0x0c,
};

/*
 * struct lm3648_flash
 *
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 contols
 * @subdev_led: V4L2 subdev
 * @mode_reg : mode register value
 */
struct lm3648_flash {
	struct device *dev;
	struct lm3648_platform_data *pdata;
	struct regmap *regmap;

	struct v4l2_ctrl_handler ctrls_led;
	struct v4l2_subdev subdev_led;

	u8 mode_reg;
};

#define to_lm3648_flash(_ctrl)	\
	container_of(_ctrl->handler, struct lm3648_flash, ctrls_led)

/* enable mode control */
static int lm3648_mode_ctrl(struct lm3648_flash *flash,
			    enum v4l2_flash_led_mode led_mode)
{
	switch (led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->mode_reg | MODE_SHDN);
	case V4L2_FLASH_LED_MODE_IR:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->mode_reg | MODE_IR_DRV);
	case V4L2_FLASH_LED_MODE_TORCH:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->mode_reg | MODE_TORCH);
	case V4L2_FLASH_LED_MODE_FLASH:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->mode_reg | MODE_FLASH);
	}
	return -EINVAL;
}

/* V4L2 controls  */
static int lm3648_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lm3648_flash *flash = to_lm3648_flash(ctrl);
	unsigned int reg_val;
	int rval;

	if (ctrl->id != V4L2_CID_FLASH_FAULT)
		return -EINVAL;

	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
	if (rval < 0)
		return rval;
	ctrl->val = 0;
	if (reg_val & FAULT_TIMEOUT)
		ctrl->val |= V4L2_FLASH_FAULT_TIMEOUT;
	if (reg_val & FAULT_UVLO)
		ctrl->val |= V4L2_FLASH_FAULT_UNDER_VOLTAGE;
	if (reg_val & FAULT_OVERTEMP)
		ctrl->val |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
	if (reg_val & FAULT_OCP)
		ctrl->val |= V4L2_FLASH_FAULT_OVER_CURRENT;
	if (reg_val & FAULT_SHORT_CIRCUIT)
		ctrl->val |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;

	rval = regmap_read(flash->regmap, REG_FLAG2, &reg_val);
	if (rval < 0)
		return rval;
	if (reg_val & FAULT_NTC_TRIP)
		ctrl->val |= V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE;
	if (reg_val & FAULT_OVP)
		ctrl->val |= V4L2_FLASH_FAULT_OVER_VOLTAGE;
	if (reg_val & FAULT_IVFM)
		ctrl->val |= V4L2_FLASH_FAULT_INPUT_VOLTAGE;

	return 0;
}

static int lm3648_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lm3648_flash *flash = to_lm3648_flash(ctrl);
	unsigned int reg_val;
	int rval = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		if (ctrl->val != V4L2_FLASH_LED_MODE_FLASH)
			return lm3648_mode_ctrl(flash, ctrl->val);
		/* switch to SHDN mode before flash strobe on */
		return lm3648_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);

	case V4L2_CID_FLASH_STROBE_SOURCE:
		return regmap_update_bits(flash->regmap,
					  REG_STROBE_SRC, MASK_STROBE_SRC,
					  (ctrl->val) << 5);

	case V4L2_CID_FLASH_STROBE:
		/* read and check current mode of chip to start flash */
		rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
		if (rval < 0 || ((reg_val & MASK_ENABLE) != MODE_SHDN))
			return rval;
		/* flash on */
		return lm3648_mode_ctrl(flash, V4L2_FLASH_LED_MODE_FLASH);

	case V4L2_CID_FLASH_STROBE_STOP:
		/*
		 * flash mode will be turned automatically
		 * from FLASH mode to SHDN mode after flash duration timeout
		 * read and check current mode of chip to stop flash
		 */
		rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
		if (rval < 0)
			return rval;
		if ((reg_val & MASK_ENABLE) == MODE_FLASH)
			return lm3648_mode_ctrl(flash,
						V4L2_FLASH_LED_MODE_NONE);
		return rval;

	case V4L2_CID_FLASH_TIMEOUT:
		return regmap_update_bits(flash->regmap,
					  REG_FLASH_TOUT, MASK_FLASH_TOUT,
					  LM3648_FLASH_TOUT_ms_TO_REG
					  (ctrl->val));

	case V4L2_CID_FLASH_INTENSITY:
		return regmap_update_bits(flash->regmap,
					  REG_FLASH_BR, MASK_FLASH_BR,
					  LM3648_FLASH_BRT_uA_TO_REG
					  (ctrl->val));

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		return regmap_update_bits(flash->regmap,
					  REG_TORCH_BR, MASK_TORCH_BR,
					  LM3648_TORCH_BRT_uA_TO_REG
					  (ctrl->val));
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops lm3648_led_ctrl_ops = {
	.g_volatile_ctrl = lm3648_get_ctrl,
	.s_ctrl = lm3648_set_ctrl,
};

static int lm3648_init_controls(struct lm3648_flash *flash)
{
	struct v4l2_ctrl *fault;
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led;
	const struct v4l2_ctrl_ops *ops = &lm3648_led_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 8);
	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_IR, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);
	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  LM3648_FLASH_TOUT_MIN,
			  LM3648_FLASH_TOUT_MAX,
			  LM3648_FLASH_TOUT_LOW_STEP,
			  LM3648_FLASH_TOUT_MAX);

	/* max flash current */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  LM3648_FLASH_BRT_MIN,
			  LM3648_FLASH_BRT_MAX,
			  LM3648_FLASH_BRT_STEP,
			  LM3648_FLASH_BRT_MAX);

	/* max torch current */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  LM3648_TORCH_BRT_MIN,
			  LM3648_TORCH_BRT_MAX,
			  LM3648_TORCH_BRT_STEP,
			  LM3648_TORCH_BRT_MAX);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_TIMEOUT
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_UNDER_VOLTAGE
				  | V4L2_FLASH_FAULT_INPUT_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_CURRENT
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_OVER_VOLTAGE, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led.ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops lm3648_ops = {
	.core = NULL,
};

static const struct regmap_config lm3648_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int lm3648_subdev_init(struct lm3648_flash *flash)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	int rval;

	v4l2_i2c_subdev_init(&flash->subdev_led, client, &lm3648_ops);
	flash->subdev_led.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(flash->subdev_led.name, LM3648_NAME);
	rval = lm3648_init_controls(flash);
	if (rval)
		goto err_out;
	rval = media_entity_init(&flash->subdev_led.entity, 0, NULL, 0);
	if (rval < 0)
		goto err_out;
	flash->subdev_led.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;
	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led);
	return rval;
}

static int lm3648_init_device(struct lm3648_flash *flash)
{
	unsigned int reg_val;
	int rval;

	/* LED Enable - on */
	rval = regmap_update_bits(flash->regmap, REG_ENABLE, 0x03, 0x03);
	rval |= regmap_update_bits(flash->regmap, REG_FLASH_BR, 0xc0, 0x80);
	rval |= regmap_update_bits(flash->regmap, REG_TORCH_BR, 0x80, 0x80);
	if (rval < 0)
		return rval;
	/* read the value of mode register to reduce redundant i2c accesses */
	rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
	if (rval < 0)
		return rval;
	flash->mode_reg = reg_val & 0xf3;

	/* output disable */
	rval = lm3648_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);
	if (rval < 0)
		return rval;

	/* Reset flag register */
	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
	if (rval < 0)
		return rval;
	return regmap_read(flash->regmap, REG_FLAG2, &reg_val);
}

static int lm3648_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct lm3648_flash *flash;
	struct lm3648_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &lm3648_regmap);
	if (IS_ERR(flash->regmap))
		return PTR_ERR(flash->regmap);

	/* check device tree if there is no platform data */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct lm3648_platform_data),
				     GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;
		/* use default data in case of no platform data */
		pdata->flash_timeout = LM3648_FLASH_TOUT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;

	rval = lm3648_subdev_init(flash);
	if (rval < 0)
		return rval;

	rval = lm3648_init_device(flash);
	if (rval < 0)
		return rval;

	i2c_set_clientdata(client, flash);

	return 0;
}

static int lm3648_remove(struct i2c_client *client)
{
	struct lm3648_flash *flash = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(&flash->subdev_led);
	v4l2_ctrl_handler_free(&flash->ctrls_led);
	media_entity_cleanup(&flash->subdev_led.entity);

	return 0;
}

static const struct i2c_device_id lm3648_id_table[] = {
	{LM3648_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3648_id_table);

static struct i2c_driver lm3648_i2c_driver = {
	.driver = {
		   .name = LM3648_NAME,
		   },
	.probe = lm3648_probe,
	.remove = lm3648_remove,
	.id_table = lm3648_id_table,
};

module_i2c_driver(lm3648_i2c_driver);

MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_DESCRIPTION("Texas Instruments LM3648 Flash LED driver");
MODULE_LICENSE("GPL");
