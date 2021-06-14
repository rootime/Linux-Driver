// SPDX-License-Identifier: GPL-2.0
//
// Author: Daniel Geonsi Jeong <rootime@kakao.com>
//
#include <linux/init.h> 
#include <linux/kernel.h> 
#include <linux/module.h> 
 
static int __init driver_init(void) { 
    printk(KERN_INFO ">>> skeleton driver init\n");
    return 0; 
} 
 
static void __exit driver_exit(void) { 
    printk(KERN_INFO ">>> skeleton driver exit\n");
} 
 
module_init(driver_init); 
module_exit(driver_exit); 
MODULE_AUTHOR("Daniel Geonsi Jeong <rootime@kakao.com>"); 
MODULE_LICENSE("GPL v2");