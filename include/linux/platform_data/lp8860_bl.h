/*
 * Simple driver for Texas Instruments LP8860 Backlight driver chip
 * Copyright (C) 2014 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LP8860_H
#define __LP8860_H

#define LP8860_NAME "lp8860"
#define LP8860_ADDR 0x2d

#define LP8860_LED0_BR_MAX 65535
#define LP8860_LED1_BR_MAX 8191
#define LP8860_LED2_BR_MAX 8191
#define LP8860_LED3_BR_MAX 8191

#define LP8860_LED0_I_MAX 4095
#define LP8860_LED1_I_MAX 255
#define LP8860_LED2_I_MAX 255
#define LP8860_LED3_I_MAX 255

enum lp8860_leds {
	LP8860_LED0 = 0,
	LP8860_LED1,
	LP8860_LED2,
	LP8860_LED3,
	LP8860_LED_MAX
};

enum lp8860_ctrl_mode {
	LP8860_CTRL_I2C = 0,
	LP8860_CTRL_I2C_PWM,
};

/* struct lp8860 platform data
 * @mode : control mode
 * @max_brt : maximum brightness.
 *		LED0 0 ~ 65535
 *		LED1 0 ~ 8191
 *		LED2 0 ~ 8191
 *		LED3 0 ~ 8191
 */
struct lp8860_platform_data {

	enum lp8860_ctrl_mode mode;
	int max_brt[LP8860_LED_MAX];
};

#endif /* __LP8860_H */
