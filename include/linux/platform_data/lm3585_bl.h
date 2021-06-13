/*
 * Simple driver for Texas Instruments LM3585 Backlight driver chip
 * Copyright (C) 2014 Texas Instruments
 * Author: Daniel Jeong  <gshark.jeong@gmail.com>
 *		  Ldd Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LM3585_H
#define __LM3585_H

#define LM3585_NAME "lm3585"
#define LM3585_ADDR 0x31

#define LM3585_BR_MAX 4095

enum lm3585_leds_enable {
	LM3585_LED_ALL_OFF = 0x00,
	LM3585_LED_0_ON = 0x01,
	LM3585_LED_1_ON = 0x02,
	LM3585_LED_2_ON = 0x04,
	LM3585_LED_3_ON = 0x08,
	LM3585_LED_4_ON = 0x10,
	LM3585_LED_5_ON = 0x20,
	LM3585_LED_6_ON = 0x40,
	LM3585_LED_7_ON = 0x80,
	LM3585_LED_8_ON = 0x100,
	LM3585_LED_9_ON = 0x200,
	LM3585_LED_ALL_ON = 0x3ff,
};

enum lm3585_cabc_input {
	LM3585_CABC_INPUT_DISABLE = 0,
	LM3585_CABC_INPUT_ENABLE,
};

enum lm3585_curve_type {
	LM3585_CURVE_LINEAR = 0,
	LM3585_CURVE_QUADRATIC,
	LM3585_CURVE_CUBIC,
	LM3585_CURVE_QUARTIC,
};

/* 
 * struct lm3585 platform data
 * @led_enable : each bit disables the chosen LED output or enable all.
 * @ctype : curve type
 * @cabc_input : the CABC PWM input control 
 * @ramp_time : the duration of the ILED transitioin. 0~3fh
 */
struct lm3585_platform_data {

	enum lm3585_leds_enable led_enable;
	enum lm3585_curve_type 	ctype;
	enum lm3585_cabc_input 	cabc_input;	
	int  ramp_time;
};

#endif /* __LM3585_H */
