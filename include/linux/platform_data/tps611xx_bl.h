/*
 * Simple driver for Texas Instruments TPS61163a Backlight driver chip
 * Copyright (C) 2014 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __TPS611XX_H
#define __TPS611XX_H

#define TPS611XX_NAME "tps611xx_bl"
#define TPS61158_NAME "tps61158_bl"
#define TPS61161_NAME "tps61161_bl"
#define TPS61163_NAME "tps61163_bl"
#define TPS61165_NAME "tps61165_bl"

/* struct tps61163a platform data
 * @use_rfa : request for acknowledge
 * @en_gpio_num : en pin gpio number
 */
struct tps611xx_platform_data {

	int rfa_en;
	unsigned int en_gpio_num;
};

#endif /* __TPS61163A_H */
