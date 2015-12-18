/*
 * Generic GPIO card-detect helper
 *
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mmc/cd-gpio.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/slab.h>
#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
//Zhilong.Zhang@OnlineRd.Driver, 2014/09/24, Add for wakeup the phone when insert or pull out the micro sd card
#include <linux/pcb_version.h>
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
//rongchun.Zhang@EXP.BasicDrv, 2014/12/12, Add for hot plug Tf card system crash
int TF_CARD_STATUS=1;
#endif /* VENDOR_EDIT */
struct mmc_cd_gpio {
	unsigned int gpio;
	bool status;
	char label[0];
};

static int mmc_cd_get_status(struct mmc_host *host)
{
	int ret = -ENOSYS;
	struct mmc_cd_gpio *cd = host->hotplug.handler_priv;

	if (!cd || !gpio_is_valid(cd->gpio))
		goto out;

	ret = !gpio_get_value_cansleep(cd->gpio) ^
		!!(host->caps2 & MMC_CAP2_CD_ACTIVE_HIGH);
out:
	return ret;
}

static irqreturn_t mmc_cd_gpio_irqt(int irq, void *dev_id)
{
	struct mmc_host *host = dev_id;
	struct mmc_cd_gpio *cd = host->hotplug.handler_priv;
	int status;
	
#ifdef VENDOR_EDIT
//rongchun.Zhang@EXP.BasicDrv, 2014/12/12, Add for hot plug Tf card system crash
	TF_CARD_STATUS = status = mmc_cd_get_status(host);
#endif /* VENDOR_EDIT */

	if (unlikely(status < 0))
		goto out;

	if (status ^ cd->status) {
		pr_info("%s: slot status change detected (%d -> %d), GPIO_ACTIVE_%s\n",
				mmc_hostname(host), cd->status, status,
				(host->caps2 & MMC_CAP2_CD_ACTIVE_HIGH) ?
				"HIGH" : "LOW");
		cd->status = status;

#ifdef VENDOR_EDIT
        //Lycan.Wang@Prd.BasicDrv, 2014-07-10 Add for retry 5 times when new sdcard init error
        host->detect_change_retry = 5;
#endif /* VENDOR_EDIT */

		/* Schedule a card detection after a debounce timeout */
		mmc_detect_change(host, msecs_to_jiffies(100));
	}
out:
	return IRQ_HANDLED;
}

int mmc_cd_gpio_request(struct mmc_host *host, unsigned int gpio)
{
	size_t len = strlen(dev_name(host->parent)) + 4;
	struct mmc_cd_gpio *cd;
	int irq = gpio_to_irq(gpio);
	int ret;

	if (irq < 0)
		return irq;

	cd = kmalloc(sizeof(*cd) + len, GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	snprintf(cd->label, len, "%s cd", dev_name(host->parent));

	ret = gpio_request_one(gpio, GPIOF_DIR_IN, cd->label);
	if (ret < 0)
		goto egpioreq;

	cd->gpio = gpio;
	host->hotplug.irq = irq;
	host->hotplug.handler_priv = cd;

	ret = mmc_cd_get_status(host);
	if (ret < 0)
		goto eirqreq;

	cd->status = ret;

	ret = request_threaded_irq(irq, NULL, mmc_cd_gpio_irqt,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				   cd->label, host);
	if (ret < 0)
		goto eirqreq;

#if (defined(CONFIG_OPPO_MSM_14021) || defined(CONFIG_OPPO_MSM_14024))
//Zhilong.Zhang@OnlineRd.Driver, 2014/09/24, Add for wakeup the phone when insert or pull out the micro sd card
#if 0
	if (get_pcb_version() >= HW_VERSION__32) {
		ret = enable_irq_wake(irq);
		if(ret < 0)
		{
			printk(KERN_ERR "%s, enable_irq_wake %d\n", __func__, ret);
		}
	}
#endif	
#endif /* VENDOR_EDIT */	

	return 0;

eirqreq:
	gpio_free(gpio);
egpioreq:
	kfree(cd);
	return ret;
}
EXPORT_SYMBOL(mmc_cd_gpio_request);

void mmc_cd_gpio_free(struct mmc_host *host)
{
	struct mmc_cd_gpio *cd = host->hotplug.handler_priv;

	if (!cd || !gpio_is_valid(cd->gpio))
		return;

	free_irq(host->hotplug.irq, host);
	gpio_free(cd->gpio);
	cd->gpio = -EINVAL;
	kfree(cd);
}
EXPORT_SYMBOL(mmc_cd_gpio_free);
