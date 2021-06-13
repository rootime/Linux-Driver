/*
 * drivers/media/i2c/lm3643.c
 * General device driver for TI LM3643, FLASH LED Driver
 *
 * Copyright (C) 2014 Texas Instruments
 *
 * Contact: Daniel Jeong <gshark.jeong@gmail.com>
 *			Ldd-Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <media/lm3643.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

/* registers definitions */
#define REG_ENABLE		0x01
#define REG_FLASH_LED0_BR	0x03
#define REG_FLASH_LED1_BR	0x04
#define REG_TORCH_LED0_BR	0x05
#define REG_TORCH_LED1_BR	0x06
#define REG_FLASH_TOUT		0x08
#define REG_FLAG0		0x0a
#define REG_FLAG1		0x0b

/* Fault Mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_SHORT_CIRCUIT	(0x70)
#define FAULT_UVLO		(1<<1)
#define FAULT_IVFM		(1<<2)
#define FAULT_OCP		(1<<3)
#define FAULT_OVERTEMP	(1<<2)
#define FAULT_NTC_TRIP	(1<<0)
#define FAULT_OVP		(1<<1)

#define MASK_MODE	0x0C
enum led_mode {
	MODE_SHDN = 0x00,
	MODE_IR_DRV = 0x04,
	MODE_TORCH = 0x08,
	MODE_FLASH = 0x0C,
};

/*
 * struct lm3643_flash
 *
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 contols
 * @subdev_led: V4L2 subdev
 */
struct lm3643_flash {
	struct device *dev;
	struct lm3643_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	struct v4l2_ctrl_handler ctrls_led[LM3643_LED_MAX];
	struct v4l2_subdev subdev_led[LM3643_LED_MAX];
	/* caching control register to reduce redundant i2c accesses */
	u8 enable_reg;

};

#define to_lm3643_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct lm3643_flash, ctrls_led[_no])

/* enable mode control */
static int lm3643_mode_ctrl(struct lm3643_flash *flash,
			    enum v4l2_flash_led_mode led_mode)
{
	switch (led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->enable_reg | MODE_SHDN);
	case V4L2_FLASH_LED_MODE_IR:
		return regmap_write(flash->regmap,
				    REG_ENABLE,
				    flash->enable_reg | MODE_IR_DRV);
	case V4L2_FLASH_LED_MODE_TORCH:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->enable_reg | MODE_TORCH);
	case V4L2_FLASH_LED_MODE_FLASH:
		return regmap_write(flash->regmap,
				    REG_ENABLE, flash->enable_reg | MODE_FLASH);
	}
	return -EINVAL;
}

/* torch0/1 brightness control */
static int lm3643_torch_brt_ctrl(struct lm3643_flash *flash,
				 enum lm3643_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	br_bits = LM3643_TORCH_BRT_uA_TO_REG(brt);
	if (led_no == LM3643_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_TORCH_LED0_BR, 0x7f, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_TORCH_LED1_BR, 0x7f, br_bits);
	return rval;
}

/* flash0/1 brightness control */
static int lm3643_flash_brt_ctrl(struct lm3643_flash *flash,
				 enum lm3643_led_id led_no, unsigned int brt)
{
	int rval;
	u8 br_bits;

	br_bits = LM3643_FLASH_BRT_uA_TO_REG(brt);
	if (led_no == LM3643_LED0)
		rval = regmap_update_bits(flash->regmap,
					  REG_FLASH_LED0_BR, 0x7f, br_bits);
	else
		rval = regmap_update_bits(flash->regmap,
					  REG_FLASH_LED1_BR, 0x7f, br_bits);
	return rval;
}

static int lm3643_get_ctrl(struct v4l2_ctrl *ctrl, enum lm3643_led_id led_no)
{
	struct lm3643_flash *flash = to_lm3643_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		unsigned int reg_val;

		rval = regmap_read(flash->regmap, REG_FLAG0, &reg_val);
		if (rval < 0)
			goto out;
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

		rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
		if (rval < 0)
			goto out;
		if (reg_val & FAULT_IVFM)
			ctrl->val |= V4L2_FLASH_FAULT_INPUT_VOLTAGE;
		if (reg_val & FAULT_NTC_TRIP)
			ctrl->val |= V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE;
		if (reg_val & FAULT_OVP)
			ctrl->val |= V4L2_FLASH_FAULT_OVER_VOLTAGE;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int lm3643_set_ctrl(struct v4l2_ctrl *ctrl, enum lm3643_led_id led_no)
{
	struct lm3643_flash *flash = to_lm3643_flash(ctrl, led_no);
	u8 tout_bits;
	int rval = -EINVAL;
	unsigned int reg_val;

	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		if (ctrl->val != V4L2_FLASH_LED_MODE_FLASH) {
			rval = lm3643_mode_ctrl(flash, ctrl->val);
			break;
		}
		/* switch to SHDN mode before flash strobe on */
		rval = lm3643_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x20, (ctrl->val) << 6);
		if (rval < 0)
			goto err_out;
		rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
		if (rval < 0)
			goto err_out;
		flash->enable_reg = reg_val & (~MASK_MODE);
		break;

	case V4L2_CID_FLASH_STROBE:

		/* read and check current mode of chip to start flash */
		rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
		if (rval < 0 || ((reg_val & MASK_MODE) != MODE_SHDN))
			goto err_out;
		/* flash on */
		rval = lm3643_mode_ctrl(flash, V4L2_FLASH_LED_MODE_FLASH);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		/*
		 * flash mode will be turned automatically
		 * from FLASH mode to SHDN mode after flash duration timeout
		 * read and check current mode of chip to stop flash
		 */
		rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
		if (rval < 0)
			goto err_out;
		if ((reg_val & MASK_MODE) == MODE_FLASH)
			rval = lm3643_mode_ctrl(flash,
						V4L2_FLASH_LED_MODE_NONE);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		tout_bits = LM3643_FLASH_TOUT_ms_TO_REG(ctrl->val);
		rval = regmap_update_bits(flash->regmap,
					  REG_FLASH_TOUT, 0x0f, tout_bits);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = lm3643_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = lm3643_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int lm3643_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3643_get_ctrl(ctrl, LM3643_LED1);
}

static int lm3643_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3643_set_ctrl(ctrl, LM3643_LED1);
}

static int lm3643_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3643_get_ctrl(ctrl, LM3643_LED0);
}

static int lm3643_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return lm3643_set_ctrl(ctrl, LM3643_LED0);
}

static const struct v4l2_ctrl_ops lm3643_led_ctrl_ops[LM3643_LED_MAX] = {
	[LM3643_LED0] = {
			 .g_volatile_ctrl = lm3643_led0_get_ctrl,
			 .s_ctrl = lm3643_led0_set_ctrl,
			 },
	[LM3643_LED1] = {
			 .g_volatile_ctrl = lm3643_led1_get_ctrl,
			 .s_ctrl = lm3643_led1_set_ctrl,
			 }
};

static int lm3643_init_controls(struct lm3643_flash *flash,
				enum lm3643_led_id led_no)
{
	struct v4l2_ctrl *fault;
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &lm3643_led_ctrl_ops[led_no];

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
			  LM3643_FLASH_TOUT_MIN,
			  LM3643_FLASH_TOUT_MAX,
			  LM3643_FLASH_TOUT_LOW_STEP, LM3643_FLASH_TOUT_MAX);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  LM3643_FLASH_BRT_MIN, LM3643_FLASH_BRT_MAX,
			  LM3643_FLASH_BRT_STEP, LM3643_FLASH_BRT_MAX);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  LM3643_TORCH_BRT_MIN, LM3643_TORCH_BRT_MAX,
			  LM3643_TORCH_BRT_STEP, LM3643_TORCH_BRT_MAX);

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

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops lm3643_ops = {
	.core = NULL,
};

static const struct regmap_config lm3643_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int lm3643_subdev_init(struct lm3643_flash *flash,
			      enum lm3643_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	int rval;

	v4l2_i2c_subdev_init(&flash->subdev_led[led_no], client, &lm3643_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(flash->subdev_led[led_no].name, led_name);
	rval = lm3643_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_init(&flash->subdev_led[led_no].entity, 0, NULL, 0);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

static int lm3643_init_device(struct lm3643_flash *flash)
{
	struct lm3643_platform_data *pdata = flash->pdata;
	int rval;
	unsigned int reg_val;

	rval = regmap_read(flash->regmap, REG_ENABLE, &reg_val);
	if (rval < 0)
		return rval;
	/* set led0/1 enable bits */
	reg_val |= flash->pdata->led0_enable;
	reg_val |= flash->pdata->led1_enable;
	/* save enable register to reduce redundant i2c accesses */
	flash->enable_reg = reg_val & 0xf3;

	rval = regmap_write(flash->regmap, REG_ENABLE, reg_val);
	if (rval < 0)
		return rval;

	/* set output mode */
	rval = lm3643_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);
	if (rval < 0)
		return rval;

	/* set override */
	rval = regmap_update_bits(flash->regmap,
		  REG_FLASH_LED0_BR, 0x80, pdata->flash1_override);
	rval |= regmap_update_bits(flash->regmap,
		  REG_TORCH_LED0_BR, 0x80, pdata->torch1_override);
	if (rval < 0)
		return rval;

	/* Reset flag 0 register */
	rval = regmap_read(flash->regmap, REG_FLAG0, &reg_val);
	if (rval < 0)
		return rval;

	/* Reset flag 1 register */
	return regmap_read(flash->regmap, REG_FLAG1, &reg_val);

}

static int lm3643_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct lm3643_flash *flash;
	struct lm3643_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &lm3643_regmap);
	if (IS_ERR(flash->regmap))
		return PTR_ERR(flash->regmap);

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->led0_enable = 0x01;
		pdata->led1_enable = 0x01 << 1;
		pdata->flash1_override = 0x01 << 7;
		pdata->torch1_override = 0x01 << 7;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);

	rval = lm3643_subdev_init(flash, LM3643_LED0, "lm3643-led0");
	if (rval < 0)
		return rval;

	rval = lm3643_subdev_init(flash, LM3643_LED1, "lm3643-led1");
	if (rval < 0)
		return rval;

	rval = lm3643_init_device(flash);
	if (rval < 0)
		return rval;

	i2c_set_clientdata(client, flash);

	return 0;
}

static int lm3643_remove(struct i2c_client *client)
{
	struct lm3643_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	for (i = LM3643_LED0; i < LM3643_LED_MAX; i++) {
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
	}

	return 0;
}

static const struct i2c_device_id lm3643_id_table[] = {
	{LM3643_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3643_id_table);

static struct i2c_driver lm3643_i2c_driver = {
	.driver = {
		   .name = LM3643_NAME,
		   .pm = NULL,
		   },
	.probe = lm3643_probe,
	.remove = lm3643_remove,
	.id_table = lm3643_id_table,
};

module_i2c_driver(lm3643_i2c_driver);

MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_DESCRIPTION("Texas Instruments LM3643 LED flash driver");
MODULE_LICENSE("GPL");
