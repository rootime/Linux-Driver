/*
 * Simple driver for Texas Instruments TPS61163a Backlight driver chip
 * Copyright (C) 2014 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __TPS61163A_H
#define __TPS61163A_H

#define TPS61163A_NAME "tps61163a_bl"

/* struct tps61163a platform data
 * @en_gpio_num : en pin gpio number
 */
struct tps61163a_platform_data {

	unsigned int en_gpio_num;
};

#endif /* __TPS61163A_H */
