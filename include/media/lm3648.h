/*
 * include/media/lm3648.h
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

#ifndef __LM3648_H__
#define __LM3648_H__

#include <media/v4l2-subdev.h>

#define LM3648_NAME	"lm3648"
#define LM3648_I2C_ADDR	(0x63)

/*  TOTAL FLASH Brightness Max
 *	min 21800uA, step 23450uA, max 1500000uA
 */
#define LM3648_FLASH_BRT_MIN 21800
#define LM3648_FLASH_BRT_STEP 23450
#define LM3648_FLASH_BRT_MAX 1500000
#define LM3648_FLASH_BRT_uA_TO_REG(a)	\
	((a) < LM3648_FLASH_BRT_MIN ? 0 :	\
	 ((((a) - LM3648_FLASH_BRT_MIN) / LM3648_FLASH_BRT_STEP)))

/*  TOTAL TORCH Brightness Max
 *	min 1954uA, step 2800uA, max 357600uA
 */
#define LM3648_TORCH_BRT_MIN 1954
#define LM3648_TORCH_BRT_STEP 2800
#define LM3648_TORCH_BRT_MAX 357600
#define LM3648_TORCH_BRT_uA_TO_REG(a)	\
	((a) < LM3648_TORCH_BRT_MIN ? 0 :	\
	 ((((a) - LM3648_TORCH_BRT_MIN) / LM3648_TORCH_BRT_STEP)))

/*  FLASH TIMEOUT DURATION
 *	min 10ms, max 400ms
 *  step 10ms in range from  10ms to 100ms
 *  setp 50ms in range from 100ms to 400ms
 */
#define LM3648_FLASH_TOUT_MIN		10
#define LM3648_FLASH_TOUT_LOW_STEP	10
#define LM3648_FLASH_TOUT_LOW_MAX	100
#define LM3648_FLASH_TOUT_HIGH_STEP	50
#define LM3648_FLASH_TOUT_MAX		400
#define LM3648_FLASH_TOUT_ms_TO_REG(a)	\
	((a) < LM3648_FLASH_TOUT_MIN ? 0 :	\
	 ((a) <= LM3648_FLASH_TOUT_LOW_MAX ?	\
	  (((a) - LM3648_FLASH_TOUT_MIN) / LM3648_FLASH_TOUT_LOW_STEP) :	\
	   ((((a) - LM3648_FLASH_TOUT_LOW_MAX) / LM3648_FLASH_TOUT_HIGH_STEP)	\
		+((LM3648_FLASH_TOUT_LOW_MAX - LM3648_FLASH_TOUT_MIN)	\
		/ LM3648_FLASH_TOUT_LOW_STEP))))

/* struct lm3648_platform_data
 *
 * @flash_timeout: flash timeout
 */
struct lm3648_platform_data {

	u32 flash_timeout;
};

#endif /* __LM3648_H__ */
