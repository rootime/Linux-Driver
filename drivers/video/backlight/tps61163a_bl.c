/*
 * Simple driver for Texas Instruments TPS61163A Backlight driver chip
 *        using EasyScale Interface
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: Daniel Jeong  <gshark.jeong@gmail.com>
 * 	       Ldd Mlp <ldd-mlp@list.ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_data/tps61163a_bl.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define ADDRESS_TPS61163A	0x8F0000
#define T_ES_DELAY_NS 		100000
#define T_ES_DETECTION_NS 	260000
#define T_START_NS			2000
#define T_EOS_NS			2000
#define T_RESET_MS			3
#define T_LOW_LOGIC_1_NS	3000
#define T_LOW_LOGIC_0_NS	7000
#define T_LOGIC_NS			T_LOW_LOGIC_1_NS + T_LOW_LOGIC_0_NS

#define BRIGHTNESS_MAX		0x1ff
#define BRIGHTNESS_BMASK	0x1ff

struct tps61163a_bl_data {
	struct device *dev;
	struct backlight_device *bled;
	struct tps61163a_platform_data *pdata;

	unsigned int en_gpio;
};

static int tps61163a_backlight_update_status(struct backlight_device *bl)
{
	struct tps61163a_bl_data *pchip = bl_get_data(bl);
	unsigned int es_data, bmask;
	
	es_data = ADDRESS_TPS61163A | (bl->props.brightness & BRIGHTNESS_BMASK);

	/* t_start : 2us high before data byte */
	gpio_direction_output(pchip->en_gpio,1);
	ndelay(T_START_NS);

	for(bmask = 0x01; bmask < 0x1000000 ; bmask <<= 1){
		int t_low;
		if(es_data & bmask)
			t_low = T_LOW_LOGIC_1_NS;
		else
			t_low = T_LOW_LOGIC_0_NS;

		gpio_direction_output(pchip->en_gpio,0);
		ndelay(t_low);
		gpio_direction_output(pchip->en_gpio,1);
		ndelay(T_LOGIC_NS - t_low);
	}
	gpio_direction_output(pchip->en_gpio,0);
	/* t_eos : 2us low after address byte */
	ndelay(T_EOS_NS);
	gpio_direction_output(pchip->en_gpio,1);

	
	return bl->props.brightness;
}

static int tps61163a_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tps61163a_bl_ops = {
	.update_status	= tps61163a_backlight_update_status,
	.get_brightness	= tps61163a_backlight_get_brightness,
};

static ssize_t tps61163a_enable_store(struct device *dev,
				   struct device_attribute *devAttr,
				   const char *buf, size_t size)
{
	struct tps61163a_bl_data *pchip = dev_get_drvdata(dev);
	unsigned int input;
	int ret;

	ret = kstrtouint(buf, 10, &input);
	if (ret) 
		return -EINVAL;

	if(input == 0){
		gpio_direction_output(pchip->en_gpio,0);
		/* low more than 2.5 ms to reset */
		mdelay(T_RESET_MS);
	} else {
		gpio_direction_output(pchip->en_gpio,1);
		ndelay(T_ES_DELAY_NS);
		gpio_direction_output(pchip->en_gpio,0);
		ndelay(T_ES_DETECTION_NS);
		gpio_direction_output(pchip->en_gpio,1);
	}

	return size;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, tps61163a_enable_store);

#ifdef CONFIG_OF
static int tps61163a_backlight_parse_dt(struct device *dev)
{
	struct device_node *node = dev->of_node;
	u32 value;
	int ret;

	if (!node)
		return -ENODEV;

	ret = of_property_read_u32(node, "en_gpio_num",&value);
	if (ret < 0)
		return ret;	

	return value;
}

static struct of_device_id tps61163a_backlight_of_match[] = {
	{ .compatible = TPS61163A_NAME },
	{ }
};

MODULE_DEVICE_TABLE(of, tps61163a_backlight_of_match);
#else
static int tps61163a_backlight_parse_dt(struct device *dev,
				  struct platform_tps61163a_backlight_data *data)
{
	return -ENODEV;
}
#endif

static int tps61163a_backlight_probe(struct platform_device *pdev)
{
	struct tps61163a_bl_data *pchip;
	struct backlight_properties props;
	struct tps61163a_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int ret;
	
	pchip = devm_kzalloc(&pdev->dev,
			     sizeof(struct tps61163a_bl_data), GFP_KERNEL);
	if (pchip == NULL)
		return -ENOMEM;
	pchip->dev = &pdev->dev;

	if(pdata == NULL){
		pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct tps61163a_platform_data), GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;
		pdata->en_gpio_num = tps61163a_backlight_parse_dt(&pdev->dev);
		if(pdata->en_gpio_num < 0)
			return -EINVAL;
	}

	pchip->en_gpio = pdata->en_gpio_num;
	pchip->pdata = pdata;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.brightness = BRIGHTNESS_MAX;
	props.max_brightness = BRIGHTNESS_MAX;
	props.type = BACKLIGHT_RAW;
	pchip->bled =
		backlight_device_register(TPS61163A_NAME, pchip->dev, pchip,
					      &tps61163a_bl_ops, &props);
		if (IS_ERR(pchip->bled))
			return PTR_ERR(pchip->bled);

	/* for enable/disable */
	ret = device_create_file(&(pchip->bled->dev),
				 &dev_attr_enable);
	if (ret < 0) {
		dev_err(pchip->dev, "failed : add sysfs entries\n");
		goto err_out;
	}
	platform_set_drvdata(pdev, pchip);

	/* EasyScale init */
	ret = gpio_request( pchip->en_gpio,"tps61163a");
	if(ret ){
		dev_err(pchip->dev, "failed : get gpio %d\n",pchip->en_gpio);
		goto err_out;
	}
	/*
	 * ES Detection Window
	 *   - ES detect delay : 100us
	 *   - ES detect time  : 260us
	 * it should be finished in 1ms
	 */
	gpio_direction_output(pchip->en_gpio,1);
	ndelay(T_ES_DELAY_NS);
	gpio_direction_output(pchip->en_gpio,0);
	ndelay(T_ES_DETECTION_NS);
	gpio_direction_output(pchip->en_gpio,1);

	dev_info(pchip->dev,"tps61163a based on EasyScale is initialized\n");
	return 0;

err_out: 

	backlight_device_unregister(pchip->bled);
	return ret;
}

static int tps61163a_backlight_remove(struct platform_device *pdev)
{
	struct tps61163a_bl_data *pchip = platform_get_drvdata(pdev);

	backlight_device_unregister(pchip->bled);
	gpio_direction_output(pchip->en_gpio,0);
	mdelay(T_RESET_MS);
	return 0;
}

static struct platform_driver tps61163a_backlight_driver = {
	.driver		= {
		.name		= TPS61163A_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tps61163a_backlight_of_match),
	},
	.probe		= tps61163a_backlight_probe,
	.remove		= tps61163a_backlight_remove,
};

module_platform_driver(tps61163a_backlight_driver);

MODULE_DESCRIPTION("EasyScale based tps61163a Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps61163a_bl");
