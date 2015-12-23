/***********************************************************
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** CONFIG_MACH_OPPO
** File: - pn544.h
* Description: Head file for pn544.
*           To define pn544 array and register address.
** Version: 1.0
** Date : 2013/10/15
** Author: yuyi@Independent Group
**
****************************************************************/

#define PN544_MAGIC	0xE9

/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN544_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)

struct pn544_i2c_platform_data {
	unsigned int irq_gpio;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
	unsigned int clk_req_gpio;
};

extern void pn544_power_init(void);
