/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/msm-charger.h>
#include <linux/i2c/bq27520.h> /* use the same platform data as bq27520 */
#include <linux/of_gpio.h> //sjc0623 add
#include <linux/qpnp/power-on.h>//sjc1121
#ifdef CONFIG_VENDOR_EDIT
/*OPPO 2013-09-22 liaofuchun add for bq27541 encryption*/
#include <linux/random.h>

#include <linux/rtc.h>
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
#include <linux/qpnp-charger.h>
#endif /*CONFIG_VENDOR_EDIT*/
extern char *BQ27541_HMACSHA1_authenticate(char *Message,char *Key,char *result);
#endif //CONFIG_VENDOR_EDIT

#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
/* OPPO 2014-06-23 sjc Add begin for 14021 */
static int mcu_en_gpio = 0;
void mcu_en_gpio_set(int value)
{
	if (value) {
		if (gpio_is_valid(mcu_en_gpio))
			gpio_set_value(mcu_en_gpio, 0);///1);
	} else {
		if (gpio_is_valid(mcu_en_gpio)) {
			gpio_set_value(mcu_en_gpio, 1);
			usleep_range(10000, 10000);
			gpio_set_value(mcu_en_gpio, 0);
		}
	}
}

#else
void mcu_en_gpio_set(int value)
{
	return;
}
#endif //CONFIG_OPPO_MSM_14021

extern int load_soc(void);//sjc1121
extern void backup_soc_ex(int soc); /* yangfangbiao@oneplus.cn, 2015/01/19  Add for  sync with android 4.4  */


/* OPPO 2013-12-20 liaofuchun add for fastchg firmware update */
#ifdef CONFIG_PIC1503_FASTCG
extern unsigned char Pic16F_firmware_data[];
extern int pic_fw_ver_count;
extern int pic_need_to_up_fw;
extern int pic_have_updated;
extern int pic16f_fw_update(bool pull96);
#endif
/* OPPO 2013-12-20 liaofuchun add end */

#define DRIVER_VERSION			"1.1.0"
/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)
/* OPPO 2013-08-24 wangjc Add begin for filter soc. */
#ifdef CONFIG_VENDOR_EDIT
#define CAPACITY_SALTATE_COUNTER 4
#define CAPACITY_SALTATE_COUNTER_NOT_CHARGING	7//40sec
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2015/01/06  Add for  sync with KK charge standard  */
#define CAPACITY_SALTATE_COUNTER_60				10//	1min
#define CAPACITY_SALTATE_COUNTER_95				25//	2.5min
#define CAPACITY_SALTATE_COUNTER_FULL			50//	5min
#define CAPACITY_SALTATE_COUNTER_CHARGING_TERM	10//	1min
#endif /*CONFIG_VENDOR_EDIT*/


#define	SOC_SHUTDOWN_VALID_LIMITS	20 /* yangfangbiao@oneplus.cn, 2015/01/06  Add for  sync with KK charge standard  */
#define TEN_MINUTES		600

#define BATT_SOC_INTERVAL					6000//6S

#endif

#define FASTCG_REQUESET_IRQ_INTERVAL		8000	//8s

/* OPPO 2013-08-24 wangjc Add end */
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};

struct bq27541_device_info {
	struct device			*dev;
	int				id;
	struct bq27541_access_methods	*bus;
	struct i2c_client		*client;
	struct work_struct		counter;
	/* 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work		hw_config;
/* OPPO 2013-08-24 wangjc Add begin for filter soc. */
#ifdef CONFIG_VENDOR_EDIT
	int cc_pre;
	int fcc_pre;
	int soc_pre;
	int temp_pre;
	int batt_vol_pre;
	int current_pre;
	int saltate_counter;
	bool is_authenticated;	//wangjc add for authentication
	bool fast_chg_started;
	bool fast_switch_to_normal;
	bool fast_normal_to_warm;	//lfc add for fastchg over temp
	int battery_type;			//lfc add for battery type
	struct power_supply		*batt_psy;
	int irq;
	struct work_struct fastcg_work;

	bool alow_reading;
	struct timer_list watchdog;
	struct wake_lock fastchg_wake_lock;
	bool fast_chg_allow;
	bool fast_low_temp_full;
/* jingchun.wang@Onlinerd.Driver, 2014/02/12  Add for retry when config fail */
	int retry_count;
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	unsigned long rtc_resume_time;
	unsigned long rtc_suspend_time;
	atomic_t suspended;
	/* jingchun.wang@Onlinerd.Driver, 2015/02/10  Add for update soc */
	struct delayed_work		update_soc_work;
/* Fuchun.Liao@Mobile.BSP.CHG 2015-03-12 add for delay 8s to request fastcg irq*/
	struct delayed_work		fastcg_request_irq_work;
#endif
	bool fast_chg_ing;
/* OPPO 2013-08-24 wangjc Add end */
};

static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */

static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di);

static int bq27541_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	static int count = 0;

#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}
#endif /*CONFIG_VENDOR_EDIT*/

	if(di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_TEMP, &temp, 0, di);
#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/01/08  Add for don't report battery not connect when reading error once. */
		if (ret) {
			count++;
			dev_err(di->dev, "error reading temperature\n");
			if(count > 1) {
				count = 0;
				/* jingchun.wang@Onlinerd.Driver, 2014/01/22  Add for it report bad status when plug out battery */
				di->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
				return -400;
			} else {
				return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
			}
		}
		count = 0;
#endif /*CONFIG_VENDOR_EDIT*/
	} else {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;	
	}

	di->temp_pre = temp;

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/* OPPO 2013-08-24 wangjc Add begin for add adc interface. */
#ifdef CONFIG_VENDOR_EDIT
#define BQ27541_REG_CC		0x2a
static int bq27541_battery_cc(struct bq27541_device_info *di)//sjc20150105
{
	int ret;
	int cc = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->cc_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_CC, &cc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading cc.\n");
			return ret;
		}
	} else {
		return di->cc_pre;
	}
	
	di->cc_pre = cc;
	return cc;
}

static int bq27541_battery_fcc(struct bq27541_device_info *di)//sjc20150105
{
	int ret;
	int fcc = 0;

	if (atomic_read(&di->suspended) == 1)
		return di->fcc_pre;

	if (di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_FCC, &fcc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading fcc.\n");
			return ret;
		}
	} else {
		return di->fcc_pre;
	}

	di->fcc_pre = fcc;
	return fcc;
}

static int bq27541_remaining_capacity(struct bq27541_device_info *di)
{
	int ret;
	int cap = 0;

	if(di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_RM, &cap, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading capacity.\n");
			return ret;
		}
	}

	return cap;
}

static int bq27541_battery_voltage(struct bq27541_device_info *di);
extern int get_charging_status(void);
extern int fuelgauge_battery_temp_region_get(void);
static int bq27541_soc_calibrate(struct bq27541_device_info *di, int soc)
{
	union power_supply_propval ret = {0,};
	unsigned int soc_calib;
	int counter_temp = 0;
	static int charging_status = 0;//sjc1121
	static int charging_status_pre = 0;
	int soc_load;//sjc1121
	int soc_temp;
	
	if(!di->batt_psy){
		di->batt_psy = power_supply_get_by_name("battery");

		//get the soc before reboot
		soc_load = load_soc();
		if (soc_load == -1) {
			//get last soc error
			di->soc_pre = soc;
		} else if(abs(soc - soc_load) > SOC_SHUTDOWN_VALID_LIMITS) {
			//the battery maybe changed
				di->soc_pre = soc;
			} else {
			//compare the soc and the last soc
			if(soc_load > soc) {
				di->soc_pre = soc_load -1;
			} else {
				di->soc_pre = soc_load;
			}
		}
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2015/02/3  Modify for V2.4 charge standard */
		if (!di->batt_psy) {
			return di->soc_pre;
		}
#endif /*CONFIG_VENDOR_EDIT*/
		//store the soc when boot first time
		backup_soc_ex(di->soc_pre);
	}

	soc_temp  = di->soc_pre;

	if(di->batt_psy){
		ret.intval = get_charging_status();//sjc20150104
	
		if(ret.intval == POWER_SUPPLY_STATUS_CHARGING || ret.intval == POWER_SUPPLY_STATUS_FULL) { // is charging
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
			charging_status = 1;
		} else {
			charging_status = 0;
		}
		if (charging_status ^ charging_status_pre) {
			charging_status_pre = charging_status;
			di->saltate_counter = 0;
		}
		if (charging_status) { // is charging
			if (ret.intval == POWER_SUPPLY_STATUS_FULL) {
				soc_calib = di->soc_pre;
				if (di->soc_pre < 100
						&& (fuelgauge_battery_temp_region_get() == CV_BATTERY_TEMP_REGION__LITTLE_COOL
						|| fuelgauge_battery_temp_region_get() == CV_BATTERY_TEMP_REGION__NORMAL)) {//sjc20150104
					if (di->saltate_counter < CAPACITY_SALTATE_COUNTER_CHARGING_TERM) {
						di->saltate_counter++;
					} else {
						soc_calib = di->soc_pre + 1;
						di->saltate_counter = 0;
					}
				}
			} else {
#endif /*CONFIG_VENDOR_EDIT*/
				if(soc > di->soc_pre) {
					di->saltate_counter++;
					//pr_info("wjc count:%d\n", di->saltate_counter);
					if(di->saltate_counter < CAPACITY_SALTATE_COUNTER)
						return di->soc_pre;
					else
						di->saltate_counter = 0;

					
					soc_calib = di->soc_pre + 1;
					//pr_info("wjc soc_calib:%d\n", soc_calib);
				} else if(soc < (di->soc_pre - 1)) {
					di->saltate_counter++;
					
					if (di->soc_pre == 100) {
						counter_temp = CAPACITY_SALTATE_COUNTER_FULL;//t>=5min
					} else if (di->soc_pre > 95) {
						counter_temp = CAPACITY_SALTATE_COUNTER_95;///t>=2.5min
					} else if (di->soc_pre > 60) {
						counter_temp = CAPACITY_SALTATE_COUNTER_60;//t>=1min
					}else {
						counter_temp = CAPACITY_SALTATE_COUNTER_NOT_CHARGING;//t>=40sec
					}

					if(di->saltate_counter < counter_temp)
						return di->soc_pre;
					else
						di->saltate_counter = 0;
				
					/* jingchun.wang@Onlinerd.Driver, 2013/04/14  Add for allow soc fail when charging. */
					soc_calib = di->soc_pre - 1;
				} else {
					soc_calib = di->soc_pre;
				}
			}
		} else { // not charging
			if ((abs(soc - di->soc_pre) >  0) 
					|| (di->batt_vol_pre <= 3300 * 1000 && di->batt_vol_pre > 2500 * 1000)) {
				//sjc1118 add for batt_vol is too low but soc is not jumping
				di->saltate_counter++;
				
				if (di->soc_pre == 100) {
					counter_temp = CAPACITY_SALTATE_COUNTER_FULL;//t>=5min
				} else if (di->soc_pre > 95) {
					counter_temp = CAPACITY_SALTATE_COUNTER_95;///t>=2.5min
				} else if (di->soc_pre > 60) {
					counter_temp = CAPACITY_SALTATE_COUNTER_60;//t>=1min
				}else {
					counter_temp = CAPACITY_SALTATE_COUNTER_NOT_CHARGING;//t>=40sec
				}
				
				/* sjc1020, when batt_vol is too low(and soc is jumping), decrease faster to avoid dead battery shutdown */
				if (di->batt_vol_pre <= 3300 * 1000 && di->batt_vol_pre > 2500 * 1000) {
					if (bq27541_battery_voltage(di) <= 3300 * 1000 && bq27541_battery_voltage(di) > 2500 * 1000)//check again
						counter_temp = CAPACITY_SALTATE_COUNTER - 1;//about 9s
				}
				
				if(di->saltate_counter < counter_temp)
					return di->soc_pre;
				else
					di->saltate_counter = 0;
			}
			else
				di->saltate_counter = 0;
			
			if(soc < di->soc_pre)
				soc_calib = di->soc_pre - 1;
			else if (di->batt_vol_pre <= 3300 * 1000 && di->batt_vol_pre > 2500 * 1000 && di->soc_pre > 0)//sjc1118 add for batt_vol is too low but soc is not jumping
				soc_calib = di->soc_pre - 1;
			else
				soc_calib = di->soc_pre;
		}
	} else {
		soc_calib = soc;
	}
	if(soc_calib > 100)
		soc_calib = 100;
	di->soc_pre = soc_calib;

	if(soc_temp  !=  soc_calib) {
		//store when soc changed
		backup_soc_ex(soc_calib);
		pr_info("soc:%d, soc_calib:%d\n", soc, soc_calib);
	}
	return soc_calib;
}

static int bq27541_battery_soc(struct bq27541_device_info *di, int time)
{
	int ret;
	int soc = 0;
	int soc_delt = 0;

#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		return di->soc_pre;
	}
#endif /*CONFIG_VENDOR_EDIT*/

	if(di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_SOC, &soc, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading soc.ret:%d\n",ret);
			goto read_soc_err;
		}
	} else {
		if(di->soc_pre)
			return di->soc_pre;
		else
			return 0;
	}

#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	if(time != 0) {
		if(soc < di->soc_pre) {
			soc_delt  =  di->soc_pre - soc;
			//allow capacity decrease 1% every 10minutes when sleep
			if(time/TEN_MINUTES <  soc_delt) {
				di->soc_pre  -=  time/TEN_MINUTES;
			} else  {
				di->soc_pre = soc;
			}
		}
	}
#endif /*CONFIG_VENDOR_EDIT*/
	soc = bq27541_soc_calibrate(di,soc);
	return soc;
	
read_soc_err:
	if(di->soc_pre)
		return di->soc_pre;
	else
		return 0;
}

static int bq27541_average_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;

#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		return -di->current_pre;
	}
#endif /*CONFIG_VENDOR_EDIT*/

	if(di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_AI, &curr, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading current.\n");
			return ret;
		}
	} else {
		return -di->current_pre;
	}
	// negative current
	if(curr&0x8000)
		curr = -((~(curr-1))&0xFFFF);
	di->current_pre = curr;
	return -curr;
}
#endif
/* OPPO 2013-08-24 wangjc Add end */

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;

#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/27  Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		return di->batt_vol_pre;
	}
#endif /*CONFIG_VENDOR_EDIT*/

	if(di->alow_reading == true) {
		ret = bq27541_read(BQ27541_REG_VOLT, &volt, 0, di);
		if (ret) {
			dev_err(di->dev, "error reading voltage,ret:%d\n",ret);
			return ret;
		}
	} else {
		return di->batt_vol_pre;
	}

	di->batt_vol_pre = volt * 1000;

	return volt * 1000;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}

/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static int bq27541_chip_config(struct bq27541_device_info *di)
{
	int flags = 0, ret = 0;

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading register %02x ret = %d\n",
			 BQ27541_REG_CNTL, ret);
		return ret;
	}
	udelay(66);

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27541_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0;
	struct bq27541_device_info *di;
	unsigned long flags;
	int count = 0;

	di = container_of(work, struct bq27541_device_info, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27541_read(BQ27541_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		/* delay 66uS, waiting time between continuous reading
		 * results
		 */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;
		udelay(66);
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error reading datalog register\n");
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

struct bq27541_device_info *bq27541_di;

static int bq27541_get_battery_mvolts(void)
{
	return bq27541_battery_voltage(bq27541_di);
}

static int bq27541_get_battery_temperature(void)
{
	return bq27541_battery_temperature(bq27541_di);
}
static int bq27541_is_battery_present(void)
{
	return 1;
}
static int bq27541_is_battery_temp_within_range(void)
{
	return 1;
}
static int bq27541_is_battery_id_valid(void)
{
	return 1;
}

/* OPPO 2013-08-24 wangjc Add begin for add adc interface. */
#ifdef CONFIG_VENDOR_EDIT
static int bq27541_get_batt_cc(void)//sjc20150105
{
	return bq27541_battery_cc(bq27541_di);
}

static int bq27541_get_batt_fcc(void)//sjc20150105
{
	return bq27541_battery_fcc(bq27541_di);
}

static int bq27541_get_batt_remaining_capacity(void)
{
	return bq27541_remaining_capacity(bq27541_di);
}

static int bq27541_get_battery_soc(void)
{
	if(bq27541_di) {
		if(!bq27541_di->soc_pre)
			return bq27541_battery_soc(bq27541_di, 0);
		else {
			return bq27541_di->soc_pre;
		}
	}
	else
		return 50;
}


static int bq27541_get_average_current(void)
{
	return bq27541_average_current(bq27541_di);
}

//wangjc add for authentication
static int bq27541_is_battery_authenticated(void)
{
	if(bq27541_di) {
		return bq27541_di->is_authenticated;
	}
	return false;
}

static int bq27541_fast_chg_started(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_chg_started;
	}
	return false;
}

static int bq27541_fast_switch_to_normal(void)
{
	if(bq27541_di) {
		//pr_err("%s fast_switch_to_normal:%d\n",__func__,bq27541_di->fast_switch_to_normal);
		return bq27541_di->fast_switch_to_normal;
	}
	return false;
}

static int bq27541_set_switch_to_noraml_false(void)
{
	if(bq27541_di) {
		bq27541_di->fast_switch_to_normal = false;
	}

	return 0;
}

static int bq27541_get_fast_low_temp_full(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_low_temp_full;
	}
	return false;
}

static int bq27541_set_fast_low_temp_full_false(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_low_temp_full = false;
	}
	return 0;
}
#endif
/* OPPO 2013-08-24 wangjc Add end */
/* OPPO 2013-12-12 liaofuchun add for set/get fastchg allow begin*/

static int bq27541_fast_normal_to_warm(void)
{
	if(bq27541_di) {
		//pr_err("%s fast_switch_to_normal:%d\n",__func__,bq27541_di->fast_switch_to_normal);
		return bq27541_di->fast_normal_to_warm;
	}
	return 0;
}

static int bq27541_set_fast_normal_to_warm_false(void)
{
	if(bq27541_di) {
		bq27541_di->fast_normal_to_warm = false;
	}

	return 0;
}

static int bq27541_set_fast_chg_allow(int enable)
{
	if(bq27541_di) {
		bq27541_di->fast_chg_allow = enable;
	}
	return 0;
}

static int bq27541_get_fast_chg_allow(void)
{
	if(bq27541_di) {
		return bq27541_di->fast_chg_allow;
	}
	return 0;
}

static int bq27541_get_fast_chg_ing(void)
{
	if(bq27541_di) {
			return bq27541_di->fast_chg_ing;
		}
	return 0;
}

/* OPPO 2013-12-12 liaofuchun add for set/get fastchg allow end */
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
static struct qpnp_battery_gauge bq27541_batt_gauge = {
#else
static struct msm_battery_gauge bq27541_batt_gauge = {
#endif /*CONFIG_VENDOR_EDIT*/
	.get_battery_mvolts		= bq27541_get_battery_mvolts,
	.get_battery_temperature	= bq27541_get_battery_temperature,
	.is_battery_present		= bq27541_is_battery_present,
	.is_battery_temp_within_range	= bq27541_is_battery_temp_within_range,
	.is_battery_id_valid		= bq27541_is_battery_id_valid,
/* OPPO 2013-09-30 wangjc Add begin for add new interface */
#ifdef CONFIG_VENDOR_EDIT
	.get_batt_cc				= bq27541_get_batt_cc, /* yangfangbiao@oneplus.cn, 2015/01/06  Add for  sync with KK charge standard  */
	.get_batt_fcc				= bq27541_get_batt_fcc, /* yangfangbiao@oneplus.cn, 2015/01/06  Add for  sync with KK charge standard  */
	.get_batt_remaining_capacity = bq27541_get_batt_remaining_capacity,
	.get_battery_soc			= bq27541_get_battery_soc,
	.get_average_current		= bq27541_get_average_current,
	//wangjc add for authentication
	.is_battery_authenticated	= bq27541_is_battery_authenticated,
	.fast_chg_started			= bq27541_fast_chg_started,
	.fast_switch_to_normal		= bq27541_fast_switch_to_normal,
	.set_switch_to_noraml_false	= bq27541_set_switch_to_noraml_false,
	.set_fast_chg_allow			= bq27541_set_fast_chg_allow,
	.get_fast_chg_allow			= bq27541_get_fast_chg_allow,
	.fast_normal_to_warm		= bq27541_fast_normal_to_warm,
	.set_normal_to_warm_false	= bq27541_set_fast_normal_to_warm_false,
	.get_fast_chg_ing			= bq27541_get_fast_chg_ing,
	.get_fast_low_temp_full		= bq27541_get_fast_low_temp_full,
	.set_low_temp_full_false	= bq27541_set_fast_low_temp_full_false,
#endif
/* OPPO 2013-09-30 wangjc Add end */
};
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
static bool bq27541_authenticate(struct i2c_client *client);
static int bq27541_batt_type_detect(struct i2c_client *client);
#endif /*CONFIG_VENDOR_EDIT*/
static void bq27541_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;
	struct bq27541_device_info *di;

	di  = container_of(work, struct bq27541_device_info, hw_config.work);
	ret = bq27541_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27541\n");
#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/12  Add for retry when config fail */
		di->retry_count--;
		if(di->retry_count > 0) {
			schedule_delayed_work(&di->hw_config, HZ);
		}
#endif /*CONFIG_VENDOR_EDIT*/
		return;
	}
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */	
	qpnp_battery_gauge_register(&bq27541_batt_gauge);
#else
	msm_battery_gauge_register(&bq27541_batt_gauge);
#endif /*CONFIG_VENDOR_EDIT*/
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &type, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &fw_ver, 0, di);

#ifdef CONFIG_VENDOR_EDIT
/*OPPO 2013-09-18 liaofuchun add begin for check authenticate data*/
	di->is_authenticated = bq27541_authenticate(di->client);
	di->battery_type = bq27541_batt_type_detect(di->client);
#endif //CONFIG_VENDOR_EDIT

	dev_info(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			type, fw_ver);
	dev_info(di->dev, "Complete bq27541 configuration 0x%02X\n", flags);
}

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
/* OPPO 2013-12-09 wangjc Modify begin for use standard i2c interface */
#ifndef CONFIG_VENDOR_EDIT
	struct i2c_msg msg[1];
#else
	struct i2c_msg msg[2];
#endif
/* OPPO 2013-12-09 wangjc Modify end */
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;
/* OPPO 2013-09-30 wangjc Add begin for eliminate conflict */
#ifdef CONFIG_VENDOR_EDIT
	mutex_lock(&battery_mutex);
#endif
/* OPPO 2013-09-30 wangjc Add end */

/* OPPO 2013-12-09 wangjc Modify begin for use standard i2c interface */
#ifndef CONFIG_VENDOR_EDIT
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
			mutex_unlock(&battery_mutex);
#endif /*CONFIG_VENDOR_EDIT*/
			return 0;
		}
	}
#else	
	/* Write register */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	data[0] = reg;

	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	if (!b_single)
		msg[1].len = 2;
	else
		msg[1].len = 1;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0) {
		if (!b_single)
			*rt_value = get_unaligned_le16(data);
		else
			*rt_value = data[0];

		mutex_unlock(&battery_mutex);

		return 0;
	}
#endif
/* OPPO 2013-12-09 wangjc Modify end */
/* OPPO 2013-09-30 wangjc Add begin for eliminate conflict */
#ifdef CONFIG_VENDOR_EDIT
	mutex_unlock(&battery_mutex);
#endif
/* OPPO 2013-09-30 wangjc Add end */
	return err;
}

#ifdef CONFIG_BQ27541_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27541_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27541_REG_ICR && reg > 0x00) {
		ret = bq27541_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27541_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27541_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27541_SUBCMD_FW_VER ||
		 subcmd == BQ27541_SUBCMD_HW_VER ||
		 subcmd == BQ27541_SUBCMD_CHEM_ID) {

		bq27541_cntl_cmd(di, subcmd); /* Retrieve Chip status */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_CNTL, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27541_read_stdcmd,
	bq27541_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27541_read_subcmd,
	bq27541_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name			= "bq27541-test",
	.id			= -1,
	.dev.platform_data	= NULL,
};
#endif

#ifdef CONFIG_VENDOR_EDIT
/*OPPO 2013-09-18 liaofuchun add begin for bq27541 authenticate */
#define BLOCKDATACTRL	0X61
#define DATAFLASHBLOCK	0X3F
#define AUTHENDATA		0X40
#define AUTHENCHECKSUM	0X54
#define MESSAGE_LEN		20
#define KEY_LEN			16

/* OPPO 2014-02-25 sjc Modify begin for FIND7OP not use authenticate */
#if defined (CONFIG_OPPO_DEVICE_FIND7OP) || defined (CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024)
static bool bq27541_authenticate(struct i2c_client *client)
{
	return true;
}
#else
static bool bq27541_authenticate(struct i2c_client *client)
{
	char recv_buf[MESSAGE_LEN]={0x0};
	char send_buf[MESSAGE_LEN]={0x0};
	char result[MESSAGE_LEN]={0x0};
	char Key[KEY_LEN]={0x77,0x30,0xa1,0x28,0x0a,0xa1,0x13,0x20,0xef,0xcd,0xab,0x89,0x67,0x45,0x23,0x01};
	char checksum_buf[1] ={0x0};
	char authen_cmd_buf[1] = {0x00};
	int i,rc;
	pr_info("%s Enter\n",__func__);

	// step 0: produce 20 bytes random data and checksum
	get_random_bytes(send_buf,20);	
	for(i = 0;i < 20;i++){
		checksum_buf[0] = checksum_buf[0] + send_buf[i];
	}
	checksum_buf[0] = 0xff - (checksum_buf[0]&0xff);

	/* step 1: unseal mode->write 0x01 to blockdatactrl
	authen_cmd_buf[0] = 0x01;
	rc = i2c_smbus_write_i2c_block_data(client,BLOCKDATACTRL,1,&authen_cmd_buf[0]);
	}	*/
	
	// step 1: seal mode->write 0x00 to dataflashblock
	rc = i2c_smbus_write_i2c_block_data(client,DATAFLASHBLOCK,1,&authen_cmd_buf[0]);
	if( rc < 0 ){
		pr_info("%s i2c write error\n",__func__);
		return false;
	}
	// step 2: write 20 bytes to authendata_reg
	i2c_smbus_write_i2c_block_data(client,AUTHENDATA,MESSAGE_LEN,&send_buf[0]);
	msleep(1);
	// step 3: write checksum to authenchecksum_reg for compute
	i2c_smbus_write_i2c_block_data(client,AUTHENCHECKSUM,1,&checksum_buf[0]);
	msleep(50);
	// step 4: read authendata
	i2c_smbus_read_i2c_block_data(client,AUTHENDATA,MESSAGE_LEN,&recv_buf[0]);
	// step 5: phone do hmac(sha1-generic) algorithm
	BQ27541_HMACSHA1_authenticate(send_buf,Key,result);
	// step 6: compare recv_buf from bq27541 and result by phone
	rc = strncmp(recv_buf,result,MESSAGE_LEN);
	if(rc == 0){
		pr_info("bq27541_authenticate success\n");
		return true;
	}
	pr_info("bq27541_authenticate error,dump buf:\n");
	for(i = 0;i < 20;i++){
		pr_info("send_buf[%d]:0x%x,recv_buf[%d]:0x%x ?= result[%d]:0x%x\n",i,send_buf[i],i,recv_buf[i],i,result[i]);
	}
	return false;
}
#endif //CONFIG_OPPO_DEVICE_FIND7OP
/* OPPO 2014-02-25 sjc Modify end */
#endif //CONFIG_VENDOR_EDIT

#ifdef CONFIG_VENDOR_EDIT
//Fuchun.Liao@EXP.Driver,2014/01/10 add for check battery type
#define BATTERY_2700MA		0
#define BATTERY_3000MA		1
#define BATTERY_N3_ATL		2
#define BATTERY_N3_SONY	3
#define TYPE_INFO_LEN		8

#if defined (CONFIG_OPPO_DEVICE_FIND7OP) || defined (CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024)
#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
static int bq27541_batt_type_detect(struct i2c_client *client)
{
	char blockA_cmd_buf[1] = {0x01};
	char rc = 0;
	char recv_buf[TYPE_INFO_LEN] = {0x0};
	int i = 0;
	
	rc = i2c_smbus_write_i2c_block_data(client,DATAFLASHBLOCK,1,&blockA_cmd_buf[0]);
	if ( rc < 0 ) {
		pr_info("%s i2c write error\n",__func__);
		return 0;
	}
	msleep(30);	//it is needed
	i2c_smbus_read_i2c_block_data(client, AUTHENDATA, TYPE_INFO_LEN, &recv_buf[0]);
	if ((recv_buf[0] == 0x01) && (recv_buf[1] == 0x09) && (recv_buf[2] == 0x08) && (recv_buf[3] == 0x06))
		rc = BATTERY_N3_ATL;
	else if ((recv_buf[0] == 0x19) && (recv_buf[1] == 0x89) && (recv_buf[2] == 0x04) && (recv_buf[3] == 0x02))
		rc = BATTERY_N3_SONY;
	else {
		for(i = 0; i < TYPE_INFO_LEN; i++)
			pr_info("%s error,recv_buf[%d]:0x%x\n",__func__,i,recv_buf[i]);
		rc =  BATTERY_N3_SONY;
	}
	pr_info("%s battery_type:%d\n",__func__,rc);
	return rc;
}
#else
static int bq27541_batt_type_detect(struct i2c_client *client)
{
	return BATTERY_3000MA;
}
#endif //CONFIG_OPPO_MSM_14021
#else //defined (CONFIG_OPPO_DEVICE_FIND7OP) || defined (CONFIG_OPPO_MSM_14021)
/* jingchun.wang@Onlinerd.Driver, 2014/03/10  Modify for 14001 */
static int bq27541_batt_type_detect(struct i2c_client *client)
{
	char blockA_cmd_buf[1] = {0x01};
	char rc = 0;
	char recv_buf[TYPE_INFO_LEN] = {0x0};
	int i = 0;
	
	rc = i2c_smbus_write_i2c_block_data(client,DATAFLASHBLOCK,1,&blockA_cmd_buf[0]);
	if( rc < 0 ){
		pr_info("%s i2c write error\n",__func__);
		return 0;
	}
	msleep(30);	//it is needed
	i2c_smbus_read_i2c_block_data(client,AUTHENDATA,TYPE_INFO_LEN,&recv_buf[0]);
	if((recv_buf[0] == 0x01) && (recv_buf[1] == 0x09) && (recv_buf[2] == 0x08) && (recv_buf[3] == 0x06))
		rc = BATTERY_2700MA;
	else if((recv_buf[0] == 0x02) && (recv_buf[1] == 0x00) && (recv_buf[2] == 0x01) && (recv_buf[3] == 0x03))
		rc = BATTERY_3000MA;
	else {
		for(i = 0;i < TYPE_INFO_LEN;i++)
			pr_info("%s error,recv_buf[%d]:0x%x\n",__func__,i,recv_buf[i]);
		rc =  BATTERY_2700MA;
	}
	pr_info("%s battery_type:%d\n",__func__,rc);
	return rc;
}
#endif //defined (CONFIG_OPPO_DEVICE_FIND7OP) || defined (CONFIG_OPPO_MSM_14021)
#endif //CONFIG_VENDOR_EDIT

/* OPPO 2013-12-12 liaofuchun add for fastchg */
#ifdef CONFIG_PIC1503_FASTCG
#define AP_TX_EN	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define AP_TX_DIS	GPIO_CFG(0, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)

static irqreturn_t irq_rx_handler(int irq, void *dev_id)
{
	struct bq27541_device_info *di = dev_id;
	//pr_info("%s\n", __func__);
	
	schedule_work(&di->fastcg_work);
	return IRQ_HANDLED;
}

#define AP_SWITCH_USB	GPIO_CFG(96, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static void fastcg_work_func(struct work_struct *work)
{
	int data = 0;
	int i;
	int bit = 0;
	int retval = 0;
	int ret_info = 0;
	static int fw_ver_info = 0;
	int volt = 0;
	int temp = 0;
	int soc = 0;
	int current_now = 0;
	int remain_cap = 0;
	static bool isnot_power_on = 0;
		
	free_irq(bq27541_di->irq, bq27541_di);

	for(i = 0; i < 7; i++) {
		gpio_set_value(0, 0);
		gpio_tlmm_config(AP_TX_EN, GPIO_CFG_ENABLE);
		usleep_range(1000,1000);
		gpio_set_value(0, 1);
		gpio_tlmm_config(AP_TX_DIS, GPIO_CFG_ENABLE);
		usleep_range(19000,19000);
		bit = gpio_get_value(1);
		data |= bit<<(6-i);	
		if((i == 2) && (data != 0x50) && (!fw_ver_info)){	//data recvd not start from "101"
			pr_err("%s data err:%d\n",__func__,data);
			if(bq27541_di->fast_chg_started == true) {
				bq27541_di->alow_reading = true;
				bq27541_di->fast_chg_started = false;
				bq27541_di->fast_chg_allow = false;
				bq27541_di->fast_switch_to_normal = false;
				bq27541_di->fast_normal_to_warm = false;
				bq27541_di->fast_chg_ing = false;
				gpio_set_value(96, 0);
				mcu_en_gpio_set(1);//sjc0623 add
				retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
				if (retval) {
					pr_err("%s switch usb error %d\n", __func__, retval);
				}
				power_supply_changed(bq27541_di->batt_psy);
			}
			goto out;
		}
	}

	pr_err("%s recv data:0x%x\n", __func__, data);
	
	//lfc add for power_supply_changed NULL pointer when batt_psy unregistered
	if(bq27541_di->batt_psy == NULL){
		msleep(2);
#ifndef CONFIG_VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2015/03/13  Modify for gpio config error 
		gpio_tlmm_config(GPIO_CFG(1,0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),1);
		gpio_direction_output(1, 0);
#else /*CONFIG_VENDOR_EDIT*/
		gpio_tlmm_config(GPIO_CFG(1,0,GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),1);
		gpio_direction_input(1);
#endif /*CONFIG_VENDOR_EDIT*/
		retval = request_irq(bq27541_di->irq, irq_rx_handler, IRQF_TRIGGER_RISING, "mcu_data", bq27541_di);	//0X01:rising edge,0x02:falling edge
		if(retval < 0)
			pr_err("%s request ap rx irq failed.\n", __func__);
			
		return ;
	}
		
	if(data == 0x52) {
		//request fast charging
		wake_lock(&bq27541_di->fastchg_wake_lock);
		pic_need_to_up_fw = 0;
		fw_ver_info = 0;
		bq27541_di->alow_reading = false;
		bq27541_di->fast_chg_started = true;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_normal_to_warm = false;
		
		mod_timer(&bq27541_di->watchdog,
		  jiffies + msecs_to_jiffies(10000));
		if(!isnot_power_on){
			isnot_power_on = 1;
			ret_info = 0x1;
		} else {
			ret_info = 0x2;
		}
	} else if(data == 0x54) {
		//fast charge stopped
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_switch_to_normal = false;
		bq27541_di->fast_normal_to_warm = false;
		bq27541_di->fast_chg_ing = false;
		//switch off fast chg
		pr_info("%s fastchg stop unexpectly,switch off fastchg\n", __func__);
		
		gpio_set_value(96, 0);
		mcu_en_gpio_set(1);//sjc0623 add
		retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s switch usb error %d\n", __func__, retval);
		}
		del_timer(&bq27541_di->watchdog);
		ret_info = 0x2;
	} else if(data == 0x58) {
		//tell ap can read i2c
		bq27541_di->alow_reading = true;
		//reading
		bq27541_di->fast_chg_ing = true;
		volt = bq27541_get_battery_mvolts();
		temp = bq27541_get_battery_temperature();
		remain_cap = bq27541_get_batt_remaining_capacity();
		soc = bq27541_battery_soc(bq27541_di, 0);
		current_now = bq27541_get_average_current();
		pr_err("%s volt:%d,temp:%d,remain_cap:%d,soc:%d,current:%d\n",__func__,volt,temp,
			remain_cap,soc,current_now);	
		
		//don't read
		bq27541_di->alow_reading = false;
		mod_timer(&bq27541_di->watchdog,
			  jiffies + msecs_to_jiffies(10000));
		ret_info = 0x2;
	} else if(data == 0x5a){
		//fastchg full,vbatt > 4350
#if 0	//lfc modify for it(set fast_switch_to_normal ture) is earlier than usb_plugged_out irq(set it false)
		bq27541_di->fast_switch_to_normal = true;
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
#endif
		//switch off fast chg
		pr_info("%s fastchg full,switch off fastchg,set GPIO96 0\n", __func__);
		gpio_set_value(96, 0);
		mcu_en_gpio_set(1);//sjc0623 add
		retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s switch usb error %d\n", __func__, retval);
		}
		del_timer(&bq27541_di->watchdog);
		ret_info = 0x2;
	} else if(data == 0x53){
		if (bq27541_di->battery_type == BATTERY_3000MA || bq27541_di->battery_type == BATTERY_N3_SONY) {	//13097 ATL battery
			//if temp:10~20 decigec,vddmax = 4250mv
			//switch off fast chg
			pr_info("%s fastchg low temp full,switch off fastchg,set GPIO96 0\n", __func__);
			gpio_set_value(96, 0);
			mcu_en_gpio_set(1);//sjc0623 add
			retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
			if (retval) {
				pr_err("%s switch usb error %d\n", __func__, retval);
			}
		}
		del_timer(&bq27541_di->watchdog);
		ret_info = 0x2;
	} else if(data == 0x59){
		//usb bad connected,stop fastchg
#if 0	//lfc modify for it(set fast_switch_to_normal ture) is earlier than usb_plugged_out irq(set it false)
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_switch_to_normal = true;
#endif
		//switch off fast chg
		pr_info("%s usb bad connect,switch off fastchg\n", __func__);
		gpio_set_value(96, 0);
		mcu_en_gpio_set(1);//sjc0623 add
		retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s switch usb error %d\n", __func__, retval);
		}
		del_timer(&bq27541_di->watchdog);
		ret_info = 0x2;
	} else if(data == 0x5c){
		//fastchg temp over 45 or under 20
		pr_info("%s fastchg temp > 45 or < 20,switch off fastchg,set GPIO96 0\n", __func__);
		gpio_set_value(96, 0);
		mcu_en_gpio_set(1);//sjc0623 add
		retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s switch usb error %d\n", __func__, retval);
		}
		del_timer(&bq27541_di->watchdog);
		ret_info = 0x2;
	} else if(data == 0x56){
		//ready to get fw_ver
		fw_ver_info = 1;
		ret_info = 0x2;
	} else if(fw_ver_info){
		//get fw_ver
		//fw in local is large than mcu1503_fw_ver
		if((!pic_have_updated) && (Pic16F_firmware_data[pic_fw_ver_count - 4] > data)){
			ret_info = 0x2;
			pic_need_to_up_fw = 1;	//need to update fw
		}else{
			ret_info = 0x1;
			pic_need_to_up_fw = 0;	//fw is already new,needn't to up
		}
		pr_info("local_fw:0x%x,need_to_up_fw:%d\n",Pic16F_firmware_data[pic_fw_ver_count - 4],pic_need_to_up_fw);
		fw_ver_info = 0;
	} else {
		gpio_set_value(96, 0);
		mcu_en_gpio_set(1);//sjc0623 add
		retval = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
		if (retval) {
			pr_err("%s data err(101xxxx) switch usb error %d\n", __func__, retval);
			goto out;	//avoid i2c conflict
		}
		msleep(500);	//avoid i2c conflict
		//data err
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_switch_to_normal = false;
		bq27541_di->fast_normal_to_warm = false;
		bq27541_di->fast_chg_ing = false;
		//data err
		pr_info("%s data err(101xxxx),switch off fastchg\n", __func__);
		power_supply_changed(bq27541_di->batt_psy);
		goto out;
	}

	msleep(2);
	gpio_tlmm_config(GPIO_CFG(1,0,GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),1);
	gpio_direction_output(1, 0);

	for(i = 0; i < 3; i++) {
		if(i == 0){	//tell mcu1503 battery_type
			gpio_set_value(1, ret_info >> 1);
		} else if(i == 1){
			gpio_set_value(1, ret_info & 0x1);
		} else {
#if defined (CONFIG_OPPO_DEVICE_FIND7OP) || defined (CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024)
			if (bq27541_di->battery_type == BATTERY_N3_SONY)
				gpio_set_value(1, 0);///in N3 for 4.25V low_temp_full
			else
				gpio_set_value(1, 1);
#else
			gpio_set_value(1,bq27541_di->battery_type);///0 -> 4A or 1 -> 4.5A
#endif
		}
		
		gpio_set_value(0, 0);
		gpio_tlmm_config(AP_TX_EN, GPIO_CFG_ENABLE);
		usleep_range(1000,1000);
		gpio_set_value(0, 1);
		gpio_tlmm_config(AP_TX_DIS, GPIO_CFG_ENABLE);
		usleep_range(19000,19000);
	}

out:
	gpio_tlmm_config(GPIO_CFG(1,0,GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),1);
	gpio_direction_input(1);
	
	//lfc add for it is faster than usb_plugged_out irq to send 0x5a(fast_chg full and usb bad connected) to AP
	if(data == 0x5a || data == 0x59){
		usleep_range(180000,180000);
		bq27541_di->fast_switch_to_normal = true;
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_chg_ing = false;
	}
	//fastchg temp over( > 45 or < 20)

	//lfc add to set fastchg vddmax = 4250mv during 10 ~ 20 decigec for ATL 3000mAH battery
	if(data == 0x53){
		if (bq27541_di->battery_type == BATTERY_3000MA || bq27541_di->battery_type == BATTERY_N3_SONY) {	//13097 ATL battery
			usleep_range(180000,180000);
			bq27541_di->fast_low_temp_full = true;
			bq27541_di->alow_reading = true;
			bq27541_di->fast_chg_started = false;
			bq27541_di->fast_chg_allow = false;
			bq27541_di->fast_chg_ing = false;
		}
	}
	//lfc add to set fastchg vddmax = 4250mv end
	
	if(data == 0x5c){
		usleep_range(180000,180000);
		bq27541_di->fast_normal_to_warm = true;
		bq27541_di->alow_reading = true;
		bq27541_di->fast_chg_started = false;
		bq27541_di->fast_chg_allow = false;
		bq27541_di->fast_chg_ing = false;
	}
	
	if(pic_need_to_up_fw){
		msleep(500);
		del_timer(&bq27541_di->watchdog);
		pic16f_fw_update(false);
		pic_need_to_up_fw = 0;
		mod_timer(&bq27541_di->watchdog,
			  jiffies + msecs_to_jiffies(10000));
	}
	
	retval = request_irq(bq27541_di->irq, irq_rx_handler, IRQF_TRIGGER_RISING, "mcu_data", bq27541_di);	//0X01:rising edge,0x02:falling edge
	if(retval < 0) {
	pr_err("%s request ap rx irq failed.\n", __func__);
	}
	if((data == 0x52) || (data == 0x58)){
		power_supply_changed(bq27541_di->batt_psy);
	}

	if(data == 0x53){
		if (bq27541_di->battery_type == BATTERY_3000MA || bq27541_di->battery_type == BATTERY_N3_SONY) {
			power_supply_changed(bq27541_di->batt_psy);
			wake_unlock(&bq27541_di->fastchg_wake_lock);
		}
	}
		
	if((data == 0x54) || (data == 0x5a) || (data == 0x59) || (data == 0x5c)){
		power_supply_changed(bq27541_di->batt_psy);
		wake_unlock(&bq27541_di->fastchg_wake_lock);
	}
}

void di_watchdog(unsigned long data)
{
	struct bq27541_device_info *di = (struct bq27541_device_info *)data;

	int ret = 0;
	pr_err("di_watchdog can't receive mcu data\n");
	di->alow_reading = true;
	di->fast_chg_started = false;
	di->fast_switch_to_normal = false;
	di->fast_low_temp_full = false;
	di->fast_chg_allow = false;
	di->fast_normal_to_warm = false;
	di->fast_chg_ing = false;
	//switch off fast chg
	pr_info("%s switch off fastchg\n", __func__);

	gpio_set_value(96, 0);
	mcu_en_gpio_set(1);//sjc0623 add
	ret = gpio_tlmm_config(AP_SWITCH_USB, GPIO_CFG_ENABLE);
	if (ret) {
		pr_info("%s switch usb error %d\n", __func__, ret);
	}
	wake_unlock(&bq27541_di->fastchg_wake_lock);
}
#endif
/* OPPO 2013-12-12 liaofuchun add for fastchg */

#ifdef CONFIG_PIC1503_FASTCG
/* Fuchun.Liao@Mobile.BSP.CHG 2015-03-12 add for delay 8s to request fastcg irq */
static void fastcg_request_irq(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq27541_device_info *di = container_of(dwork,
				struct bq27541_device_info, fastcg_request_irq_work);
	int retval;
	
	gpio_direction_input(1);
	di->irq = gpio_to_irq(1);
	retval = request_irq(di->irq, irq_rx_handler, IRQF_TRIGGER_RISING, "mcu_data", di);	//0X01:rising edge,0x02:falling edge
	if(retval < 0) {
		pr_err("%s request ap rx irq failed.\n", __func__);
	}
}
#endif

#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
static void update_soc(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bq27541_device_info *di = container_of(dwork,
				struct bq27541_device_info, update_soc_work);

	bq27541_battery_soc(di, 0);
		
	/*update time 6s*/
	schedule_delayed_work(&di->update_soc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (BATT_SOC_INTERVAL)));
}
#define MAX_RETRY_COUNT	5
#endif /*CONFIG_VENDOR_EDIT*/
static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *di;
	struct bq27541_access_methods *bus;
	int num;
	int retval = 0;
	
#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
/* OPPO 2014-06-23 sjc Add begin for 14021 */
	struct device_node *dev_node = client->dev.of_node;
	int ret;

	if (dev_node) {
		mcu_en_gpio = of_get_named_gpio(dev_node, "microchip,mcu-en-gpio", 0);
	} else {
		mcu_en_gpio = 0;
		printk(KERN_ERR "%s: mcu_en_gpio failed\n", __func__);
	}
	if (gpio_is_valid(mcu_en_gpio)) {
		ret = gpio_request(mcu_en_gpio, "mcu_en_gpio");
		if (ret) {
			printk(KERN_ERR "%s: gpio_request failed for %d ret=%d\n", __func__, mcu_en_gpio, ret);
		} else {
			gpio_set_value(mcu_en_gpio, 0);
		}
	}
#endif //CONFIG_OPPO_MSM_14021

	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;
/* OPPO 2013-08-19 wangjc Add begin for error temp */
#ifdef CONFIG_VENDOR_EDIT
	di->temp_pre = 0;
#endif
/* OPPO 2013-08-19 wangjc Add end */
	di->alow_reading = true;
	di->fast_chg_ing = false;
	di->fast_low_temp_full = false;
#ifdef CONFIG_VENDOR_EDIT
/* jingchun.wang@Onlinerd.Driver, 2014/02/12  Add for retry when config fail */
	di->retry_count = MAX_RETRY_COUNT;
	di->saltate_counter = 0;
	atomic_set(&di->suspended, 0);
	di->cc_pre = bq27541_battery_cc(di);//sjc20150105
	di->fcc_pre = bq27541_battery_fcc(di);
#endif /*CONFIG_VENDOR_EDIT*/

#ifdef CONFIG_BQ27541_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_4;
	} else
		goto batt_failed_4;
#endif

	if (retval) {
		dev_err(&client->dev, "failed to setup bq27541\n");
		goto batt_failed_4;
	}

	if (retval) {
		dev_err(&client->dev, "failed to powerup bq27541\n");
		goto batt_failed_4;
	}

	spin_lock_init(&lock);

	bq27541_di = di;
	INIT_WORK(&di->counter, bq27541_coulomb_counter_work);
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
	schedule_delayed_work(&di->hw_config, 0);
#else
	schedule_delayed_work(&di->hw_config, BQ27541_INIT_DELAY);
#endif /*CONFIG_VENDOR_EDIT*/
	
	INIT_DELAYED_WORK(&di->update_soc_work,
							update_soc);
	schedule_delayed_work(&di->update_soc_work,
			      round_jiffies_relative(msecs_to_jiffies
						(BATT_SOC_INTERVAL)));
	/* OPPO 2013-12-22 wangjc add for fastchg*/
	#ifdef CONFIG_PIC1503_FASTCG
	init_timer(&di->watchdog);
	di->watchdog.data = (unsigned long)di;
	di->watchdog.function = di_watchdog;
	wake_lock_init(&di->fastchg_wake_lock,		
		WAKE_LOCK_SUSPEND, "fastcg_wake_lock");
	INIT_WORK(&di->fastcg_work,fastcg_work_func);
	gpio_request(1, "mcu_clk");
	gpio_tlmm_config(GPIO_CFG(1,0,GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),1);

	INIT_DELAYED_WORK(&di->fastcg_request_irq_work,fastcg_request_irq);
	schedule_delayed_work(&di->fastcg_request_irq_work,
			      round_jiffies_relative(msecs_to_jiffies
						(FASTCG_REQUESET_IRQ_INTERVAL)));
	#endif
	/* OPPO 2013-12-22 wangjc add end*/
	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	
#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
/* OPPO 2014-06-23 sjc Add begin for 14021 */	
	if (gpio_is_valid(mcu_en_gpio))//sjc0623 add
		gpio_free(mcu_en_gpio);
#endif
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
	qpnp_battery_gauge_unregister(&bq27541_batt_gauge);
#else
	msm_battery_gauge_unregister(&bq27541_batt_gauge);
#endif /*CONFIG_VENDOR_EDIT*/
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);

	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */

extern int msmrtc_alarm_read_time(struct rtc_time *tm);
static int bq27541_battery_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret=0;
	struct rtc_time	rtc_suspend_rtc_time;
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	
	atomic_set(&di->suspended, 1);
	ret = msmrtc_alarm_read_time(&rtc_suspend_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_suspend_rtc_time, &di->rtc_suspend_time);
	
	return 0;
}

/*1 minute*/
#define RESUME_TIME  1*60 
static int bq27541_battery_resume(struct i2c_client *client)
{
	int ret=0;
	int suspend_time;
	struct rtc_time	rtc_resume_rtc_time;
	struct bq27541_device_info *di = i2c_get_clientdata(client);
			
	atomic_set(&di->suspended, 0);
	ret = msmrtc_alarm_read_time(&rtc_resume_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_resume_rtc_time, &di->rtc_resume_time);
	suspend_time =  di->rtc_resume_time - di->rtc_suspend_time;

	/*update pre capacity when sleep time more than 1minutes*/
	bq27541_battery_soc(bq27541_di, suspend_time); 

	return 0;
}

#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
/* OPPO 2014-11-18 sjc Add begin for 14021 */
#define CONTROL_CMD				0x00
#define CONTROL_STATUS				0x00
#define SEAL_POLLING_RETRY_LIMIT	100
#define BQ27541_UNSEAL_KEY			11151986
#define RESET_SUBCMD				0x0041

static void control_cmd_write(struct bq27541_device_info *di, u16 cmd)
{
	int value;
	
	//dev_dbg(di->dev, "%s: %04x\n", __FUNCTION__, cmd);
	bq27541_cntl_cmd(di, 0x0041);
	msleep(10);
	bq27541_read(CONTROL_STATUS, &value, 0, di);
	printk(KERN_ERR "bq27541 CONTROL_STATUS: 0x%x\n", value);
}
/*
static int control_cmd_read(struct bq27541_device_info *di, u16 cmd)
{
	int ret = 0;
	int value = 0;
	
	dev_dbg(di->dev, "%s: %04x\n", __FUNCTION__, cmd);

	bq27541_write(CONTROL_CMD, cmd, false, di);

	msleep(10);

	ret = bq27541_read(CONTROL_CMD, &value, false, di);
	if (ret) {
		printk(KERN_ERR "control_cmd_read() fail !\n");
		return ret;
	}

	return value;
	
}
*/
static int sealed(struct bq27541_device_info *di)
{
	//return control_cmd_read(di, CONTROL_STATUS) & (1 << 13);
	int value = 0;
	
	bq27541_cntl_cmd(di,CONTROL_STATUS);
	msleep(10);
	bq27541_read(CONTROL_STATUS, &value, 0, di);
	pr_err("%s REG_CNTL: 0x%x\n", __func__, value);

	return value & BIT(14);
}

static int unseal(struct bq27541_device_info *di, u32 key)
{
	int i = 0;

	if (!sealed(di))
		goto out;

	//bq27541_write(CONTROL_CMD, key & 0xFFFF, false, di);
	bq27541_cntl_cmd(di, 0x1115);
	msleep(10);
	//bq27541_write(CONTROL_CMD, (key & 0xFFFF0000) >> 16, false, di);
	bq27541_cntl_cmd(di, 0x1986);
	msleep(10);
	bq27541_cntl_cmd(di, 0xffff);
	msleep(10);
	bq27541_cntl_cmd(di, 0xffff);
	msleep(10);

	while (i < SEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed(di))
			break;
		msleep(10);
	}

out:
	printk(KERN_ERR "bq27541 %s: i=%d\n", __FUNCTION__, i);

	if ( i == SEAL_POLLING_RETRY_LIMIT) {
		printk(KERN_ERR "bq27541 %s failed\n", __FUNCTION__);
		return 0;
	} else {
		return 1;
	}
}

static void bq27541_reset(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	if (bq27541_get_battery_mvolts() <= 3250 * 1000 
			&& bq27541_get_battery_mvolts() > 2500 * 1000
			&& bq27541_get_battery_soc() == 0 
			&& bq27541_get_battery_temperature() > 150) {
		if (!unseal(di, BQ27541_UNSEAL_KEY)) {
			printk(KERN_ERR "bq27541 unseal fail !\n");
			return;
		}
		printk(KERN_ERR "bq27541 unseal OK !\n");
		
		control_cmd_write(di, RESET_SUBCMD);
	}
	return;
}
#else
static void bq27541_reset(struct i2c_client *client) {}
#endif //CONFIG_OPPO_MSM_14021

static const struct of_device_id bq27541_match[] = {
	{ .compatible = "ti,bq27541-battery" },
	{ },
};

#endif /*CONFIG_VENDOR_EDIT*/
static const struct i2c_device_id bq27541_id[] = {
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
	{ "bq27541-battery", 1 },
#else /*CONFIG_VENDOR_EDIT*/
	{ "bq27541", 1 },
#endif /*CONFIG_VENDOR_EDIT*/
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);

static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
			.name = "bq27541-battery",
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
		.owner	= THIS_MODULE,
		.of_match_table = bq27541_match,
#endif /*CONFIG_VENDOR_EDIT*/
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
#ifdef CONFIG_VENDOR_EDIT
/* yangfangbiao@oneplus.cn, 2014/12/27  Add for  sync with android 4.4  */
	.shutdown	= bq27541_reset,
	.suspend	= bq27541_battery_suspend ,
	.resume		= bq27541_battery_resume,
#endif /*CONFIG_VENDOR_EDIT*/
	.id_table	= bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
