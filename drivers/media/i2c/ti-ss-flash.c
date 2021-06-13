/*
 * drivers/media/i2c/ti-ss-flash.c
 * General device driver for Single String FLASH LED Drivers of TI
 * It covers lm3556, lm3561, lm3642, lm3646 and lm3648.
 *
 * Copyright (C) 2015 Texas Instruments
 *
 * Contact: Daniel Jeong <gshark.jeong@gmail.com>
 *			Ldd-Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/ti-ss-flash.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

/* operation mode */
enum led_opmode {
	OPMODE_SHDN = 0x0,
	OPMODE_INDI_IR,
	OPMODE_TORCH,
	OPMODE_FLASH,
};

/*
 * register data
 * @reg : register
 * @mask : mask bits
 * @shift : bit shift of data
 */
struct ctrl_reg {
	unsigned int reg;
	unsigned int mask;
	unsigned int shift;
};

/*
 * unit data
 * @min : min value of brightness or timeout
 *        brightness : uA
 *		  timeout    : ms
 * @step : step value of brightness or timeout
 *        brightness : uA
 *		  timeout    : ms
 * @knee: knee point of step of brightness/timeout
 *        brightness : uA
 *		  timeout    : ms
 * @knee_step : step value of brightness or timeout after knee point
 *        brightness : uA
 *		  timeout    : ms
 * @max : max value of brightness or timeout
 *        brightness : uA
 *		  timeout    : ms
 * @ctrl : register info to control brightness or timeout
 */
struct ssflash_config {
	unsigned int min;
	unsigned int step;
	unsigned int knee;
	unsigned int knee_step;
	unsigned int max;
	struct ctrl_reg ctrl;
};

/*
 * @reg : fault register
 * @mask : fault mask bit
 * @v4l2_fault : bit mapping to V4L2_FLASH_FAULT_
 *               refer to include//uapi/linux/v4l2-controls.h
 */
struct ssflash_fault {
	unsigned int reg;
	unsigned int mask;
	unsigned int v4l2_fault;
};

#define NUM_V4L2_FAULT 9

/*
 * ssflash data
 * @name: device name
 * @mode: operation mode control data
 * @flash_br: flash brightness register and bit data
 * @timeout: timeout control data
 * @strobe: strobe control data
 * @torch_br: torch brightness register and bit data
 * @fault: fault data
 * @func: initialize function
 */
struct ssflash_data {
	char *name;
	struct ctrl_reg mode;
	struct ssflash_config flash_br;
	struct ssflash_config timeout;
	struct ctrl_reg strobe;

	struct ssflash_config torch_br;
	struct ssflash_fault fault[NUM_V4L2_FAULT];

	int (*func)(struct regmap *regmap);
};

/*
 * struct ssflash_flash
 * @dev: device
 * @regmap: reg map for interface
 * @ctrls_led: V4L2 contols
 * @subdev_led: V4L2 subdev
 * @data : chip control data
 * @mode_reg: value of mode register
 */
struct ssflash_flash {
	struct device *dev;
	struct regmap *regmap;

	struct v4l2_ctrl_handler ctrls_led;
	struct v4l2_subdev subdev_led;
	const struct ssflash_data *data;
	u8 mode_reg;
};

#define to_ssflash_flash(_ctrl)	\
	container_of(_ctrl->handler, struct ssflash_flash, ctrls_led)

static const struct ssflash_data flash_lm3556 = {
	.name = LM3556_NAME,
	.mode = {
		.reg = 0x0a, .mask = 0x03, .shift = 0
	},
	.flash_br = {
		.min = 93750, .step = 93750,
		.max = 1500000,
		.ctrl = {
			.reg = 0x09, .mask = 0xf, .shift = 0
		},
	},
	.timeout = {
		.min = 100,	.step = 100,
		.max = 800,
		.ctrl = {
			.reg = 0x08, .mask = 0x07, .shift = 0
		}
	},
	.torch_br = {
		.min = 46880, .step = 46880,
		.max = 375000,
		.ctrl = {
			.reg = 0x09, .mask = 0x70, .shift = 4
		}
	},
	.strobe = {
		.reg = 0x0a, .mask = 0x20, .shift = 5
	},
	.fault = {
		[0] = {
			.reg = 0x0b, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_TIMEOUT,
		},
		[1] = {
			.reg = 0x0b, .mask = 0x02,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_TEMPERATURE,
		},
		[2] = {
			.reg = 0x0b, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_SHORT_CIRCUIT,
		},
		[3] = {
			.reg = 0x0b, .mask = 0x08,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_VOLTAGE,
		},

		[4] = {
			.reg = 0x0b, .mask = 0x10,
			.v4l2_fault = V4L2_FLASH_FAULT_UNDER_VOLTAGE,
		},
		[5] = {
			.reg = 0x0b, .mask = 0x20,
			.v4l2_fault = V4L2_FLASH_FAULT_INPUT_VOLTAGE,
		},
	},
	.func = NULL
};

static const struct ssflash_data flash_lm3561 = {
	.name = LM3561_NAME,
	.mode = {
		.reg = 0x10, .mask = 0x03, .shift = 0
	},
	.flash_br = {
		.min = 36000, .step = 37600,
		.max = 600000,
		.ctrl = {
			.reg = 0xb0, .mask = 0xf, .shift = 0
		},
	},
	.timeout = {
		.min = 32,	.step = 32,
		.max = 1024,
		.ctrl = {
			.reg = 0xc0, .mask = 0x1f, .shift = 0
		}
	},
	.torch_br = {
		.min = 18000, .step = 18800,
		.max = 149600,
		.ctrl = {
			.reg = 0xa0, .mask = 0x07, .shift = 0
		}
	},
	.strobe = {
		.reg = 0xe0, .mask = 0x04, .shift = 2
	},
	.fault = {
		[0] = {
			.reg = 0xd0, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_TIMEOUT,
		},
		[1] = {
			.reg = 0xd0, .mask = 0x02,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_TEMPERATURE,
		},
		[2] = {
			.reg = 0xd0, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_SHORT_CIRCUIT,
		},
		[3] = {
			.reg = 0xd0, .mask = 0x20,
			.v4l2_fault = V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE,
		},
		[4] = {
			.reg = 0xd0, .mask = 0x80,
			.v4l2_fault = V4L2_FLASH_FAULT_INPUT_VOLTAGE,
		},
	},
	.func = NULL
};

static const struct ssflash_data flash_lm3642 = {
	.name = LM3642_NAME,
	.mode = {
		.reg = 0x0a, .mask = 0x03, .shift = 0
	},
	.flash_br = {
		.min = 93750, .step = 93750,
		.max = 1500000,
		.ctrl = {
			.reg = 0x09, .mask = 0xf, .shift = 0
		},
	},
	.timeout = {
		.min = 100,	.step = 100,
		.max = 800,
		.ctrl = {
			.reg = 0x08, .mask = 0x07, .shift = 0
		}
	},
	.torch_br = {
		.min = 46880, .step = 46880,
		.max = 375000,
		.ctrl = {
			.reg = 0x09, .mask = 0x70, .shift = 4
		}
	},
	.strobe = {
		.reg = 0x0a, .mask = 0x20, .shift = 5
	},
	.fault = {
		[0] = {
			.reg = 0x0b, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_TIMEOUT,
		},
		[1] = {
			.reg = 0x0b, .mask = 0x02,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_TEMPERATURE,
		},
		[2] = {
			.reg = 0x0b, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_SHORT_CIRCUIT,
		},
		[3] = {
			.reg = 0x0b, .mask = 0x20,
			.v4l2_fault = V4L2_FLASH_FAULT_INPUT_VOLTAGE,
		},
	},
	.func = NULL
};

/*
 * LM3646 is a dual string flash LED driver
 * but there are no regiters to control LED2 directly.
 * LED1 and LED2 are controlled through LED1 registers.
 */
static int ssflash_lm3646_init(struct regmap *regmap)
{
	int rval;

	/*
	 *max flash & torch current
	 * LED2 = max current value - LED1 Current value
	 * if max current value < LED1 current value
	 *    LED1 = max current value
	 *    LED2 = off
	 */
	rval = regmap_update_bits(regmap, 0x05, 0x0f, 0x0f);
	rval |= regmap_update_bits(regmap, 0x05, 0x70, 0x07 << 4);
	if(rval < 0)
		return rval;
	return 0;
}

static const struct ssflash_data flash_lm3646 = {
	.name = LM3646_NAME,
	.mode = {
		.reg = 0x01, .mask = 0x03, .shift = 0
	},
	.flash_br = {
		.min = 0, .step = 23100,
		.knee = 23100, .knee_step = 11700,
		.max = 1500000,
		.ctrl = {
			.reg = 0x06, .mask = 0x7f, .shift = 0
		},
	},
	.timeout = {
		.min = 50,	.step = 100,
		.max = 400,
		.ctrl = {
			.reg = 0x04, .mask = 0x0f, .shift = 0
		}
	},
	.torch_br = {
		.min = 0, .step = 2600,
		.knee = 2600, .knee_step = 1400,
		.max = 187100,
		.ctrl = {
			.reg = 0x7, .mask = 0x7f, .shift = 0
		}
	},
	.strobe = {
		.reg = 0x6, .mask = 0x80, .shift = 7
	},
	.fault = {
		[0] = {
			.reg = 0x08, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_TIMEOUT,
		},
		[1] = {
			.reg = 0x08, .mask = 0x02,
			.v4l2_fault = V4L2_FLASH_FAULT_SHORT_CIRCUIT,
		},
		[2] = {
			.reg = 0x08, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_UNDER_VOLTAGE,
		},
		[3] = {
			.reg = 0x08, .mask = 0x08,
			.v4l2_fault = V4L2_FLASH_FAULT_INPUT_VOLTAGE,
		},
		[4] = {
			.reg = 0x08, .mask = 0x10,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_CURRENT,
		},
		[5] = {
			.reg = 0x08, .mask = 0x20,
			.v4l2_fault =V4L2_FLASH_FAULT_OVER_TEMPERATURE,
		},
		[6] = {
			.reg = 0x08, .mask = 0x40,
			.v4l2_fault = V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE,
		},
		[7] = {
			.reg = 0x08, .mask = 0x80,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_VOLTAGE,
		},
	},
	.func = ssflash_lm3646_init
};

static int ssflash_lm3648_init(struct regmap *regmap)
{
	int rval;

	/* enable LED */
	rval = regmap_update_bits(regmap, 0x01, 0x03, 0x03);
	/* bit[7:6] must be set to 10b for proper operation */
	rval |= regmap_update_bits(regmap, 0x03, 0xc0, 0x80);
	/* bit[7] must be set to 1b for proper operation */
	rval |= regmap_update_bits(regmap, 0x05, 0x80, 0x80);
	if (rval < 0)
		return rval;
	return 0;
}

static const struct ssflash_data flash_lm3648 = {
	.name = LM3648_NAME,
	.mode = {
		.reg = 0x01, .mask = 0x0c, .shift = 2
	},
	.flash_br = {
		.min = 21800, .step = 23450,
		.max = 1500000,
		.ctrl = {
			.reg = 0x03, .mask = 0x3f, .shift = 0
		},
	},
	.timeout = {
		.min = 10,	.step = 10,
		.knee = 100, .knee_step = 50,
		.max = 1024,
		.ctrl = {
			.reg = 0x08, .mask = 0x0f, .shift = 0
		}
	},
	.torch_br = {
		.min = 1954, .step = 2800,
		.max = 357600,
		.ctrl = {
			.reg = 0x5, .mask = 0x7f, .shift = 0
		}
	},
	.strobe = {
		.reg = 0x1, .mask = 0x20, .shift = 5
	},
	.fault = {
		[0] = {
			.reg = 0x0a, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_TIMEOUT,
		},
		[1] = {
			.reg = 0x0a, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_OVER_TEMPERATURE,
		},
		[2] = {
			.reg = 0x0a, .mask = 0x70,
			.v4l2_fault = V4L2_FLASH_FAULT_SHORT_CIRCUIT,
		},
		[3] = {
			.reg = 0x0b, .mask = 0x04,
			.v4l2_fault = V4L2_FLASH_FAULT_INPUT_VOLTAGE,
		},
		[4] = {
			.reg = 0x0b, .mask = 0x01,
			.v4l2_fault = V4L2_FLASH_FAULT_LED_OVER_TEMPERATURE,
		},
	},
	.func = ssflash_lm3648_init
};

static u8 ssflash_conv_val_to_reg(unsigned int val,
	unsigned int min, unsigned int step,
	unsigned int knee, unsigned int knee_step,
	unsigned int max)
{
	if (val >= max)
		return 0xff;

	if (knee <= min)
		return (u8)(val < min ? 0 : (val - min)/step);
	return (u8)val < min ? 0 :
			(val < knee ? (val-min)/step :
				(val-knee)/knee_step + (knee-min)/step);
}

/* mode control */
static int ssflash_mode_ctrl(struct ssflash_flash *flash,
			    enum v4l2_flash_led_mode v4l_led_mode)
{
	const struct ssflash_data *data = flash->data;
	enum led_opmode opmode;

	switch (v4l_led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		opmode = OPMODE_SHDN;
	break;
	case V4L2_FLASH_LED_MODE_TORCH:
		opmode = OPMODE_TORCH;
	break;
	case V4L2_FLASH_LED_MODE_FLASH:
		opmode = OPMODE_FLASH;
	break;
	default:
		return -EINVAL;
	}
	return regmap_update_bits(flash->regmap, data->mode.reg,
				data->mode.mask, opmode << data->mode.shift);
}

/* torch brightness control */
static int ssflash_torch_brt_ctrl
	(struct ssflash_flash *flash, unsigned int brt)
{
	const struct ssflash_data *data = flash->data;
	int rval;
	u8 br_bits;

	br_bits = ssflash_conv_val_to_reg(brt,
				data->torch_br.min, data->torch_br.step,
				data->torch_br.knee, data->torch_br.knee_step,
				data->torch_br.max);

	rval = regmap_update_bits(flash->regmap,
			data->torch_br.ctrl.reg,
			data->torch_br.ctrl.mask,
			br_bits << data->torch_br.ctrl.shift);

	return rval;
}

/* flash brightness control */
static int ssflash_flash_brt_ctrl(struct ssflash_flash *flash,
				 unsigned int brt)
{
	const struct ssflash_data *data = flash->data;
	int rval;
	u8 br_bits;

	br_bits = ssflash_conv_val_to_reg(brt,
					data->flash_br.min,
					data->flash_br.step,
					data->flash_br.knee,
					data->flash_br.knee_step,
					data->flash_br.max);

	rval = regmap_update_bits(flash->regmap,
				data->flash_br.ctrl.reg,
				data->flash_br.ctrl.mask,
				br_bits << data->flash_br.ctrl.shift);

	return rval;
}

/* fault status */
static int ssflash_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ssflash_flash *flash = to_ssflash_flash(ctrl);
	const struct ssflash_data *data = flash->data;
	int rval = -EINVAL;

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		int icnt;
		s32 fault = 0;
		unsigned int reg_val;
		unsigned int reg = 0xfff;

		for (icnt = 0; icnt < NUM_V4L2_FAULT; icnt++) {
			if (data->fault[icnt].mask == 0x0)
				continue;
			if (data->fault[icnt].reg != reg) {
				reg = data->fault[icnt].reg;
				rval = regmap_read(
						flash->regmap, reg, &reg_val);
				if (rval < 0)
					goto out;
			}
			if (reg_val & data->fault[icnt].mask)
				fault |= data->fault[icnt].v4l2_fault;
		}
		ctrl->cur.val = fault;
	}

out:
	return rval;
}

static int ssflash_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ssflash_flash *flash = to_ssflash_flash(ctrl);
	const struct ssflash_data *data = flash->data;
	u8 tout_bits;
	unsigned int reg_val;
	int rval = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		if (ctrl->val != V4L2_FLASH_LED_MODE_FLASH)
			return ssflash_mode_ctrl(flash, ctrl->val);
		/* switch to SHDN mode before flash strobe on */
		return ssflash_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		return ssflash_torch_brt_ctrl(flash, ctrl->val);

	case V4L2_CID_FLASH_INTENSITY:
		return ssflash_flash_brt_ctrl(flash, ctrl->val);

	case V4L2_CID_FLASH_STROBE_SOURCE:
		return regmap_update_bits(flash->regmap,
					data->strobe.reg, data->strobe.mask,
					(ctrl->val) << data->strobe.shift);

	case V4L2_CID_FLASH_STROBE:
		/* read and check current mode of chip to start flash */
		rval = regmap_read(flash->regmap, data->mode.reg, &reg_val);
		if (rval < 0 ||
				(((reg_val & data->mode.mask)>>data->mode.shift)
				!= OPMODE_SHDN))
			return rval;
		/* flash on */
		return ssflash_mode_ctrl(flash,
					V4L2_FLASH_LED_MODE_FLASH);

	case V4L2_CID_FLASH_STROBE_STOP:
		/*
		 * flash mode will be turned automatically
		 * from FLASH mode to SHDN mode after flash duration timeout
		 * read and check current mode of chip to stop flash
		 */
		rval = regmap_read(flash->regmap, data->mode.reg, &reg_val);
		if (rval < 0)
			return rval;
		if (((reg_val & data->mode.mask)
				>> data->mode.shift) == OPMODE_FLASH)
			return ssflash_mode_ctrl(flash,
						V4L2_FLASH_LED_MODE_NONE);
		return rval;

	case V4L2_CID_FLASH_TIMEOUT:
		tout_bits = ssflash_conv_val_to_reg(ctrl->val,
						data->timeout.min,
						data->timeout.step,
						data->timeout.knee,
						data->timeout.knee_step,
						data->timeout.max);

		return regmap_update_bits(flash->regmap,
					data->timeout.ctrl.reg,
					data->timeout.ctrl.mask,
					tout_bits << data->timeout.ctrl.shift);
	}

	return rval;
}

static int ssflash_led_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return ssflash_get_ctrl(ctrl);
}

static int ssflash_led_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return ssflash_set_ctrl(ctrl);
}

static const struct v4l2_ctrl_ops ssflash_led_ctrl_ops = {
	 .g_volatile_ctrl = ssflash_led_get_ctrl,
	 .s_ctrl = ssflash_led_set_ctrl,
};

static int ssflash_init_controls(struct ssflash_flash *flash)
{
	struct v4l2_ctrl *fault;
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led;
	const struct v4l2_ctrl_ops *ops = &ssflash_led_ctrl_ops;
	const struct ssflash_data *data = flash->data;
	s64 fault_max = 0;
	int icnt;

	v4l2_ctrl_handler_init(hdl, 8);
	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
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
			  data->timeout.min,
			  data->timeout.max,
			  data->timeout.step,
			  data->timeout.max);
	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  data->flash_br.min,
			  data->flash_br.max,
			  data->flash_br.step,
			  data->flash_br.max);
	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  data->torch_br.min,
			  data->torch_br.max,
			  data->torch_br.step,
			  data->torch_br.max);
	/* fault */
	for (icnt = 0; icnt < NUM_V4L2_FAULT; icnt++)
		fault_max |= data->fault[icnt].v4l2_fault;
	fault = v4l2_ctrl_new_std(hdl,
				ops, V4L2_CID_FLASH_FAULT, 0, fault_max, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	flash->subdev_led.ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops ssflash_ops = {
	.core = NULL,
};

static const struct regmap_config ssflash_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

static int ssflash_subdev_init(struct ssflash_flash *flash)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	int rval;

	v4l2_i2c_subdev_init(&flash->subdev_led, client, &ssflash_ops);
	flash->subdev_led.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strcpy(flash->subdev_led.name, flash->data->name);

	rval = ssflash_init_controls(flash);
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

static int ssflash_init_device(struct ssflash_flash *flash)
{
	int rval;

	/* output disable */
	rval = ssflash_mode_ctrl(flash, V4L2_FLASH_LED_MODE_NONE);
	if (rval < 0)
		return rval;

	if (flash->data->func != NULL) {
		rval = flash->data->func(flash->regmap);
		if (rval < 0)
			return rval;
	}

	return rval;
}

static int ssflash_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct ssflash_flash *flash;
	int rval;

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &ssflash_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	flash->dev = &client->dev;
	flash->data = (const struct ssflash_data *)devid->driver_data;

	rval = ssflash_subdev_init(flash);
	if (rval < 0)
		return rval;

	rval = ssflash_init_device(flash);
	if (rval < 0)
		return rval;

	i2c_set_clientdata(client, flash);

	return 0;
}

static int ssflash_remove(struct i2c_client *client)
{
	struct ssflash_flash *flash = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(&flash->subdev_led);
	v4l2_ctrl_handler_free(&flash->ctrls_led);
	media_entity_cleanup(&flash->subdev_led.entity);

	return 0;
}

static const struct i2c_device_id ssflash_id_table[] = {
	{LM3556_NAME, (unsigned long)&flash_lm3556},
	{LM3561_NAME, (unsigned long)&flash_lm3561},
	{LM3642_NAME, (unsigned long)&flash_lm3642},
	{LM3646_NAME, (unsigned long)&flash_lm3646},
	{LM3648_NAME, (unsigned long)&flash_lm3648},
	{}
};

MODULE_DEVICE_TABLE(i2c, ssflash_id_table);

static struct i2c_driver ssflash_i2c_driver = {
	.driver = {
		.name = SINGLE_STRING_FLASH_NAME,
	},
	.probe = ssflash_probe,
	.remove = ssflash_remove,
	.id_table = ssflash_id_table,
};

module_i2c_driver(ssflash_i2c_driver);

MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_DESCRIPTION("Texas Instruments single string LED flash driver");
MODULE_LICENSE("GPL");
