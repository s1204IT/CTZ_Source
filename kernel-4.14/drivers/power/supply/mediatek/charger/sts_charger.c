/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Copyright (C) 2020 SANYO Techno Solutions Tottori Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/extcon.h>

#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/mtk_boot.h>
#include <mt-plat/charger_type.h>
#include <pmic.h>
#include <mt-plat/charger_class.h>
//#include "mtk_charger_intf.h"

static struct wakeup_source charger_wakelock;
static wait_queue_head_t  wait_que;
static bool charger_thread_timeout;
static struct hrtimer charger_kthread_timer;
volatile bool fg_init_flag = false;
volatile bool chg_init_flag = false;

#ifndef A05BA
#define MAX_CAPA 5660
#else
#define MAX_CAPA 4160
#endif

bool upmu_is_chr_det(void);

#ifdef CONFIG_EXTCON_USB_CHG
struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;

	unsigned int vbus_state;
	unsigned long debounce_jiffies;
	struct delayed_work wq_detcable;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};
#endif

void __attribute__((weak)) fg_charger_in_handler(void)
{
	pr_notice("%s not defined\n", __func__);
}

static enum charger_type g_chr_type;

static const char * const mtk_chg_type_name[] = {
	"Charger Unknown",
	"Standard USB Host",
	"Charging USB Host",
	"Non-standard Charger",
	"Standard Charger",
	"Apple 2.1A Charger",
	"Apple 1.0A Charger",
	"Apple 0.5A Charger",
	"Wireless Charger",
};

static void dump_charger_name(enum charger_type type)
{
	switch (type) {
	case CHARGER_UNKNOWN:
	case STANDARD_HOST:
	case CHARGING_HOST:
	case NONSTANDARD_CHARGER:
	case STANDARD_CHARGER:
	case APPLE_2_1A_CHARGER:
	case APPLE_1_0A_CHARGER:
	case APPLE_0_5A_CHARGER:
		pr_info("%s: charger type: %d, %s\n", __func__, type,
			mtk_chg_type_name[type]);
		break;
	default:
		pr_info("%s: charger type: %d, Not Defined!!!\n", __func__,
			type);
		break;
	}
}

/* Power Supply */
struct mt_charger {
	struct device *dev;
	struct power_supply_desc chg_desc;
	struct power_supply_config chg_cfg;
	struct power_supply *chg_psy;
	struct power_supply_desc ac_desc;
	struct power_supply_config ac_cfg;
	struct power_supply *ac_psy;
	struct power_supply_desc usb_desc;
	struct power_supply_config usb_cfg;
	struct power_supply *usb_psy;
	struct power_supply_desc battery_desc;
	struct power_supply_config battery_cfg;
	struct power_supply *battery_psy;
	#ifdef CONFIG_EXTCON_USB_CHG
	struct usb_extcon_info *extcon_info;
	struct delayed_work extcon_work;
	#endif
	bool chg_online; /* Has charger in or not */
	enum charger_type chg_type;

	int bat_vol;
	int bat_soc;
	int bat_temp;
	int chg_stat;

	int chgled_gpio;
	int chrdet_irq;

	int acdet_gpio;
	int usbdet_gpio;
	int usbvbus_gpio;
};

struct mt_charger *g_mtk_chg;

#ifdef CONFIG_GAUGE_LC709203F
#include "lc709203f.h"
#endif

static int battery_get_bat_voltage(void)
{
	if(fg_init_flag == false)
		return 4000;
#ifdef CONFIG_GAUGE_LC709203F
	return lc709203f_get_cell_voltage();
#else
	return 4000;
#endif
}

static int battery_get_bat_temperature(void)
{
	if(fg_init_flag == false)
		return 25;
#ifdef CONFIG_GAUGE_LC709203F
	return lc709203f_get_cell_temperature();
#else
	return 25;
#endif
}

static int battery_get_soc(void)
{
	if(fg_init_flag == false)
		return 100;
#ifdef CONFIG_GAUGE_LC709203F
	return lc709203f_get_rsoc();
#else
	return 100;
#endif
}

#ifdef CONFIG_CHARGER_BQ24160
#include "bq24160.h"
#endif

static void chg_init(void)
{
	if(chg_init_flag == false)
		return;
#ifdef CONFIG_CHARGER_BQ24160
	bq24160_charge_init();
#endif
}

static void chg_wdt_reset(void)
{
	if(chg_init_flag == false)
		return;
#ifdef CONFIG_CHARGER_BQ24160
	bq24160_set_tmr_rst(0x01);
#endif
}

static int get_charger_type(void)
{
	if(g_mtk_chg == NULL)
		return 0;

	if(gpio_get_value(g_mtk_chg->acdet_gpio) == 0)
		return STANDARD_CHARGER;
	else if(gpio_get_value(g_mtk_chg->usbdet_gpio) == 0 
			&& gpio_get_value(g_mtk_chg->usbvbus_gpio) == 0)
		return STANDARD_HOST;

	return 0;
}

static int get_charger_stat(void)
{
	if(chg_init_flag == false)
		return 1;
#ifdef CONFIG_CHARGER_BQ24160
	{
		unsigned int stat;

		stat = bq24160_get_stat();
		if((stat == 3) || (stat == 4))//Charging
			return 1;
		else if ((gpio_get_value(g_mtk_chg->acdet_gpio) == 1) && (gpio_get_value(g_mtk_chg->usbdet_gpio) == 0))//USB経由の充電
		{
			if(bq24160_get_ts_fault() == 0)
			{
				if((stat == 2) || (stat == 5))
				{
					return 1;
				}
			}
		}
		return 0;
	}
#else
	return 1;
#endif
}

static int get_charger_fault(void)
{
	if(chg_init_flag == false)
		return 0;
#ifdef CONFIG_CHARGER_BQ24160
	return bq24160_get_fault();
#else
	return 0;
#endif
}

/* Power Supply Functions */
static int mt_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
//	struct mt_charger *mtk_chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (get_charger_type() != 0)
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_charger_type();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_EXTCON_USB_CHG
static void usb_extcon_detect_cable(struct work_struct *work)
{
	struct usb_extcon_info *info = container_of(to_delayed_work(work),
						struct usb_extcon_info,
						wq_detcable);

	/* check and update cable state */
	if (info->vbus_state)
		extcon_set_state_sync(info->edev, EXTCON_USB, true);
	else
		extcon_set_state_sync(info->edev, EXTCON_USB, false);
}
#endif

void _wake_up_charger(void);

static int mt_charger_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct mt_charger *mtk_chg = power_supply_get_drvdata(psy);
	#ifdef CONFIG_EXTCON_USB_CHG
	struct usb_extcon_info *info;
	#endif
	static int usb_connect = -1;

	pr_info("%s\n", __func__);

	if (!mtk_chg) {
		pr_notice("%s: no mtk chg data\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_EXTCON_USB_CHG
	info = mtk_chg->extcon_info;
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mtk_chg->chg_online = val->intval;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		mtk_chg->chg_type = val->intval;
		g_chr_type = val->intval;
		break;
	default:
		return -EINVAL;
	}

	dump_charger_name(mtk_chg->chg_type);

	/* usb */
	if ((mtk_chg->chg_type == STANDARD_HOST) ||
		(mtk_chg->chg_type == CHARGING_HOST) ||
		((mtk_chg->chg_type == NONSTANDARD_CHARGER) && (gpio_get_value(g_mtk_chg->usbdet_gpio) == 0))) {
#ifdef CONFIG_CHARGER_BQ24160
		if(mtk_chg->chg_type != NONSTANDARD_CHARGER) {
			printk("USB 500mA charge\n");
			bq24160_set_iusb_limit(2);//500mA
		}
		else {
			printk("USB 100mA charge\n");
			bq24160_set_iusb_limit(0);//100mA
		}
#endif
		if(usb_connect != 1)
		{
			mt_usb_connect();
			usb_connect = 1;
		}
		#ifdef CONFIG_EXTCON_USB_CHG
		info->vbus_state = 1;
		if (!IS_ERR(info->edev))
			queue_delayed_work(system_power_efficient_wq,
				&info->wq_detcable, info->debounce_jiffies);
		#endif
	} else {
		switch (mtk_chg->chg_type) {
			case STANDARD_CHARGER:
			case APPLE_2_1A_CHARGER:
			case APPLE_1_0A_CHARGER:
#ifdef CONFIG_CHARGER_BQ24160
				printk("USB 800mA charge\n");
				bq24160_set_iusb_limit(3);//800mA
#endif
			break;
			case APPLE_0_5A_CHARGER:
#ifdef CONFIG_CHARGER_BQ24160
				printk("USB 500mA charge\n");
				bq24160_set_iusb_limit(2);//500mA
#endif
			break;
			default:
#ifdef CONFIG_CHARGER_BQ24160
				printk("USB 100mA charge\n");
				bq24160_set_iusb_limit(0);//100mA
#endif
			break;
		}
		if(gpio_get_value(g_mtk_chg->usbdet_gpio) == 1)
			usb_connect = 0;
		if(usb_connect == 0)
		{
			mt_usb_disconnect();
			#ifdef CONFIG_EXTCON_USB_CHG
			info->vbus_state = 0;
			if (!IS_ERR(info->edev))
				queue_delayed_work(system_power_efficient_wq,
					&info->wq_detcable, info->debounce_jiffies);
			#endif
		}
	}

	_wake_up_charger();
	
	power_supply_changed(mtk_chg->ac_psy);
	power_supply_changed(mtk_chg->usb_psy);

	return 0;
}

static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
//	struct mt_charger *mtk_chg = power_supply_get_drvdata(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(get_charger_type() == STANDARD_CHARGER)
				val->intval = 1;
			else
				val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 1000000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 1000000;
		break;
		default:
		return -EINVAL;
	}

	return 0;
}

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
//	struct mt_charger *mtk_chg = power_supply_get_drvdata(psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(get_charger_type() == STANDARD_HOST)
				val->intval = 1;
			else
				val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 500000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mt_charger *mtk_chg = power_supply_get_drvdata(psy);
	int stat;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if(get_charger_type() == 0)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else //ac or usb chg connnect
		{
			if(mtk_chg->bat_soc == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if(get_charger_stat() == 0)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
/*
	POWER_SUPPLY_HEALTH_UNKNOWN = 0,
	POWER_SUPPLY_HEALTH_GOOD,
	POWER_SUPPLY_HEALTH_OVERHEAT,
	POWER_SUPPLY_HEALTH_DEAD,
	POWER_SUPPLY_HEALTH_OVERVOLTAGE,
	POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
	POWER_SUPPLY_HEALTH_COLD,
	POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE,
	POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE,
*/
/*
000-Normal
001- Thermal Shutdown
010- Battery Temperature Fault
011- Watchdog Timer Expired (bq24160/1/1B/3 only)
100- Safety Timer Expired (bq24160/1/1B/3 only)
101- IN Supply Fault
110- USB Supply Fault
111- Battery Fault
*/
		stat = get_charger_fault();
		switch (stat) {
			case 0:
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
			case 1:
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
			case 2:
				if(mtk_chg->bat_temp > 25)
					val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
				else
					val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
			case 3:
				if(get_charger_type() == 0 || get_charger_stat() == 0)
					val->intval = POWER_SUPPLY_HEALTH_GOOD;
				else
					val->intval = POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
			break;
			case 4:
				if(get_charger_type() == 0 || get_charger_stat() == 0)
					val->intval = POWER_SUPPLY_HEALTH_GOOD;
				else
					val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
			break;
			case 7:
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			break;
			default:
				val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = mtk_chg->bat_soc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mtk_chg->bat_vol * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = mtk_chg->bat_temp * 10;
//		val->intval = mtk_chg->bat_temp;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = MAX_CAPA * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = mtk_chg->bat_soc * MAX_CAPA * 1000 / 100;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property mt_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static enum power_supply_property mt_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

#ifdef CONFIG_EXTCON_USB_CHG
static void init_extcon_work(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct mt_charger *mt_chg =
		container_of(dw, struct mt_charger, extcon_work);
	struct device_node *node = mt_chg->dev->of_node;
	struct usb_extcon_info *info;

	info = mt_chg->extcon_info;
	if (!info)
		return;

	if (of_property_read_bool(node, "extcon")) {
		info->edev = extcon_get_edev_by_phandle(mt_chg->dev, 0);
		if (IS_ERR(info->edev)) {
			schedule_delayed_work(&mt_chg->extcon_work,
				msecs_to_jiffies(50));
			return;
		}
	}

	INIT_DELAYED_WORK(&info->wq_detcable, usb_extcon_detect_cable);
}
#endif

/***************/
int mtk_chr_is_charger_exist(unsigned char *exist)
{
	if (get_charger_type() == 0)
		*exist = 0;
	else
		*exist = 1;
	return 0;
}

int charger_dev_enable_otg(struct charger_device *chg_dev, bool en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL && chg_dev->ops->enable_otg)
		return chg_dev->ops->enable_otg(chg_dev, en);

	return -ENOTSUPP;
}

static int charger_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

static struct class *charger_class;

struct charger_device *get_charger_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct charger_device *)NULL;
	dev = class_find_device(charger_class, NULL, name,
				charger_match_device_by_name);

	return dev ? to_charger_device(dev) : NULL;

}
EXPORT_SYMBOL(get_charger_by_name);

//void mtk_charger_int_handler(void)
//{
//}
/**************************/

void _wake_up_charger(void)
{
	if (!charger_wakelock.active)
		__pm_stay_awake(&charger_wakelock);
	charger_thread_timeout = true;
	wake_up(&wait_que);
}

static enum hrtimer_restart charger_kthread_hrtimer_func(struct hrtimer *timer)
{
	_wake_up_charger();
	return HRTIMER_NORESTART;
}

static void mtk_charger_init_timer(void)
{
	hrtimer_init(&charger_kthread_timer,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
}

static void mtk_charger_start_timer(void)
{
	ktime_t ktime = ktime_set(2, 0);
	charger_kthread_timer.function = charger_kthread_hrtimer_func;
	hrtimer_start(&charger_kthread_timer, ktime, HRTIMER_MODE_REL);
}

static void kpoc_power_off_check(struct mt_charger *mt_chg)
{
	unsigned int boot_mode = get_boot_mode();

	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			printk("Unplug Charger/USB in KPOC mode, shutdown\n");
			kernel_power_off();
	}
}

static void charger_check_status(struct mt_charger *mt_chg)
{
	static int bat_soc_cache = -1;
	static int bat_temp_cache = -1;
	static int bat_vol_cache = -1;
	static int chg_stat_cache = -1;
	bool changed = false;

	if(mt_chg->bat_soc != bat_soc_cache)
	{
		bat_soc_cache = mt_chg->bat_soc;
		changed = true;
	}
	if(mt_chg->bat_temp != bat_temp_cache)
	{
		bat_temp_cache = mt_chg->bat_temp;
		changed = true;
	}
	if((mt_chg->bat_vol / 100) != (bat_vol_cache / 100))
	{
		bat_vol_cache = mt_chg->bat_vol;
		changed = true;
	}
	if(mt_chg->chg_stat != chg_stat_cache)
	{
		chg_stat_cache = mt_chg->chg_stat;
		changed = true;
	}

	if (mt_chg->bat_temp >= 60) {
		mt_chg->bat_soc = 0;
		changed = true;
		kernel_power_off();
	}

	if(changed)
		power_supply_changed(mt_chg->battery_psy);
}

#ifdef CONFIG_TOUCHSCREEN_NT36XXX
extern void set_charger_mode(int enable);
#endif

static int charger_routine_thread(void *data)
{
	struct mt_charger *mt_chg = data;
	bool chg_ini = false;
	int bat_vol,bat_soc,bat_temp;

	while (1) {
		wait_event(wait_que,
			(charger_thread_timeout == true));
		charger_thread_timeout = false;

		if((fg_init_flag == true) && (chg_init_flag == true))
		{
			if (!charger_wakelock.active)
				__pm_stay_awake(&charger_wakelock);

			bat_vol = battery_get_bat_voltage();
			if(bat_vol != INT_MIN)
				mt_chg->bat_vol = bat_vol;

			bat_soc = battery_get_soc();
			if(bat_soc != INT_MIN)
				mt_chg->bat_soc = bat_soc;

			bat_temp = battery_get_bat_temperature();
			if(bat_temp != INT_MIN)
				mt_chg->bat_temp = bat_temp;

			mt_chg->chg_stat = get_charger_stat();

			printk("Vbat=%d,T=%d,Soc=%d,CT:%d,det:%d\n",
				mt_chg->bat_vol,
				mt_chg->bat_temp,
				mt_chg->bat_soc,
				get_charger_type(),upmu_is_chr_det());

			charger_check_status(mt_chg);

			if (upmu_is_chr_det() == false) {
//				printk("charger NOT exist!\n");
				kpoc_power_off_check(mt_chg);
			#ifdef CONFIG_TOUCHSCREEN_NT36XXX
				set_charger_mode(0);
			#endif
				chg_ini = false;
				gpio_direction_output(mt_chg->chgled_gpio, 0);
				__pm_relax(&charger_wakelock);
			}
			else {
			#ifdef CONFIG_TOUCHSCREEN_NT36XXX
				set_charger_mode(1);
			#endif
				if(mt_chg->bat_soc < 100 && get_charger_stat())
					gpio_direction_output(mt_chg->chgled_gpio, 1);
				else
					gpio_direction_output(mt_chg->chgled_gpio, 0);

//				printk("charger exist!\n");
				if(chg_ini == false)
				{
					chg_init();
					chg_ini = true;
				}
				else
				{
					chg_wdt_reset();
				}
			}
		}
		else
		{
//			printk("charger and fuel gauge not init\n");
		}
		mtk_charger_start_timer();
	}

	return 0;
}

static irqreturn_t mt_charger_int_handler(int irq, void *dev_id)
{
	printk("!!!!!!!!!!!mt_charger_int_handler\n");

	_wake_up_charger();

	return IRQ_HANDLED;
}

extern void chrdet_int_handler(void); //pmic_chr_type_det_v2.c
int pmic_chrdet_init_flag = 0;

static irqreturn_t mt_charger_int_handler_ac(int irq, void *dev_id)
{
	printk("!!!!!!!!!!!mt_charger_int_handler ac\n");
//	msleep(200);
//	chrdet_int_handler();
	_wake_up_charger();
	power_supply_changed(g_mtk_chg->ac_psy);
	return IRQ_HANDLED;
}

static irqreturn_t mt_charger_int_handler_usb(int irq, void *dev_id)
{
	if(gpio_get_value(g_mtk_chg->usbvbus_gpio) == 0)
	{
		printk("!!!!!!!!!!!mt_charger_int_handler usb\n");
		msleep(200);
		if( pmic_chrdet_init_flag == 1 )
		{
			chrdet_int_handler();
		}
	}
	return IRQ_HANDLED;
}

static int mt_charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mt_charger *mt_chg = NULL;
	#ifdef CONFIG_EXTCON_USB_CHG
	struct usb_extcon_info *info;
	#endif

	mt_chg = devm_kzalloc(&pdev->dev, sizeof(*mt_chg), GFP_KERNEL);
	if (!mt_chg)
		return -ENOMEM;

	mt_chg->dev = &pdev->dev;
	mt_chg->chg_online = false;
	mt_chg->chg_type = CHARGER_UNKNOWN;

	mt_chg->chgled_gpio = of_get_named_gpio( pdev->dev.of_node, "chgled-gpio", 0);
	if(mt_chg->chgled_gpio >= 0) {
		gpio_request(mt_chg->chgled_gpio, NULL);
		gpio_direction_output(mt_chg->chgled_gpio, 0);
	}

	mt_chg->chrdet_irq = platform_get_irq(pdev, 0);
	if (mt_chg->chrdet_irq <= 0)
		pr_err("******** don't support irq from dts ********\n");
	else {
		ret = request_threaded_irq(mt_chg->chrdet_irq, NULL,
					   mt_charger_int_handler,
					   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mt_charger_chrdet", pdev);
		if (ret) {
			pr_err("%s: request_threaded_irq err = %d\n", __func__, ret);
			return ret;
		}
		irq_set_irq_wake(mt_chg->chrdet_irq, 1);
	}

	mt_chg->acdet_gpio = of_get_named_gpio( pdev->dev.of_node, "acdet-gpio", 0);
//	pr_info( "[%s] of_get_named_gpio(acdet): %d\n", __func__, acdet_gpio);

	if(mt_chg->acdet_gpio >= 0) {
		gpio_request( mt_chg->acdet_gpio, NULL);
		gpio_direction_input( mt_chg->acdet_gpio );	// Low: active
		ret = request_threaded_irq(gpio_to_irq(mt_chg->acdet_gpio), NULL,
					   mt_charger_int_handler_ac,
					   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mt_charger_acdet", pdev);
		if (ret) {
			pr_err("%s: request_threaded_irq ac err = %d\n", __func__, ret);
			return ret;
		}
		irq_set_irq_wake(gpio_to_irq(mt_chg->acdet_gpio), 1);
	}

	mt_chg->usbdet_gpio = of_get_named_gpio( pdev->dev.of_node, "usbdet-gpio", 0);
//	pr_info( "[%s] of_get_named_gpio(usbdet): %d\n", __func__, usbdet_gpio);

	if(mt_chg->usbdet_gpio >= 0) {
		gpio_request( mt_chg->usbdet_gpio, NULL);
		gpio_direction_input( mt_chg->usbdet_gpio );	// Low: active
		ret = request_threaded_irq(gpio_to_irq(mt_chg->usbdet_gpio), NULL,
					   mt_charger_int_handler_usb,
					   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mt_charger_usbdet", pdev);
		if (ret) {
			pr_err("%s: request_threaded_irq ac err = %d\n", __func__, ret);
			return ret;
		}
		irq_set_irq_wake(gpio_to_irq(mt_chg->usbdet_gpio), 1);
	}

	mt_chg->usbvbus_gpio = of_get_named_gpio( pdev->dev.of_node, "usbvbus-gpio", 0);
//	pr_info( "[%s] of_get_named_gpio(usbvbus): %d\n", __func__, usbvbus_gpio);

	if(mt_chg->usbvbus_gpio >= 0) {
		gpio_request( mt_chg->usbvbus_gpio, NULL);
	}
	
	mt_chg->chg_desc.name = "charger";
	mt_chg->chg_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	mt_chg->chg_desc.properties = mt_charger_properties;
	mt_chg->chg_desc.num_properties = ARRAY_SIZE(mt_charger_properties);
	mt_chg->chg_desc.set_property = mt_charger_set_property;
	mt_chg->chg_desc.get_property = mt_charger_get_property;
	mt_chg->chg_cfg.drv_data = mt_chg;

	mt_chg->ac_desc.name = "ac";
	mt_chg->ac_desc.type = POWER_SUPPLY_TYPE_MAINS;
	mt_chg->ac_desc.properties = mt_ac_properties;
	mt_chg->ac_desc.num_properties = ARRAY_SIZE(mt_ac_properties);
	mt_chg->ac_desc.get_property = mt_ac_get_property;
	mt_chg->ac_cfg.drv_data = mt_chg;

	mt_chg->usb_desc.name = "usb";
	mt_chg->usb_desc.type = POWER_SUPPLY_TYPE_USB;
	mt_chg->usb_desc.properties = mt_usb_properties;
	mt_chg->usb_desc.num_properties = ARRAY_SIZE(mt_usb_properties);
	mt_chg->usb_desc.get_property = mt_usb_get_property;
	mt_chg->usb_cfg.drv_data = mt_chg;

	mt_chg->bat_soc = 100;
	mt_chg->bat_vol = 4000;
	mt_chg->bat_temp = 25;
	mt_chg->chg_stat = -1;
	mt_chg->battery_desc.name = "battery";
	mt_chg->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	mt_chg->battery_desc.properties = mt_battery_properties;
	mt_chg->battery_desc.num_properties = ARRAY_SIZE(mt_battery_properties);
	mt_chg->battery_desc.get_property = mt_battery_get_property;
	mt_chg->battery_desc.no_thermal = true;
	mt_chg->battery_cfg.drv_data = mt_chg;

	mt_chg->chg_psy = power_supply_register(&pdev->dev,
		&mt_chg->chg_desc, &mt_chg->chg_cfg);
	if (IS_ERR(mt_chg->chg_psy)) {
		dev_notice(&pdev->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(mt_chg->chg_psy));
		ret = PTR_ERR(mt_chg->chg_psy);
		return ret;
	}

	mt_chg->ac_psy = power_supply_register(&pdev->dev, &mt_chg->ac_desc,
		&mt_chg->ac_cfg);
	if (IS_ERR(mt_chg->ac_psy)) {
		dev_notice(&pdev->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(mt_chg->ac_psy));
		ret = PTR_ERR(mt_chg->ac_psy);
		goto err_ac_psy;
	}

	mt_chg->usb_psy = power_supply_register(&pdev->dev, &mt_chg->usb_desc,
		&mt_chg->usb_cfg);
	if (IS_ERR(mt_chg->usb_psy)) {
		dev_notice(&pdev->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(mt_chg->usb_psy));
		ret = PTR_ERR(mt_chg->usb_psy);
		goto err_usb_psy;
	}

	mt_chg->battery_psy = power_supply_register(&pdev->dev, &mt_chg->battery_desc,
		&mt_chg->battery_cfg);
	if (IS_ERR(mt_chg->battery_psy)) {
		dev_notice(&pdev->dev, "Failed to register power supply: %ld\n",
			PTR_ERR(mt_chg->battery_psy));
		ret = PTR_ERR(mt_chg->battery_psy);
		goto err_battery_psy;
	}

	g_mtk_chg = mt_chg;
	platform_set_drvdata(pdev, mt_chg);
	device_init_wakeup(&pdev->dev, 1);

	#ifdef CONFIG_EXTCON_USB_CHG
	info = devm_kzalloc(mt_chg->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = mt_chg->dev;
	mt_chg->extcon_info = info;

	INIT_DELAYED_WORK(&mt_chg->extcon_work, init_extcon_work);
	schedule_delayed_work(&mt_chg->extcon_work, 0);
	#endif

	wakeup_source_init(&charger_wakelock, "charger suspend wakelock");
	init_waitqueue_head(&wait_que);
	mtk_charger_init_timer();
	charger_thread_timeout = false;
	kthread_run(charger_routine_thread, mt_chg, "charger_thread");
	_wake_up_charger();

	pr_info("%s\n", __func__);
	return 0;

err_battery_psy:
	power_supply_unregister(mt_chg->usb_psy);
err_usb_psy:
	power_supply_unregister(mt_chg->ac_psy);
err_ac_psy:
	power_supply_unregister(mt_chg->chg_psy);
	return ret;
}

static int mt_charger_remove(struct platform_device *pdev)
{
	struct mt_charger *mt_charger = platform_get_drvdata(pdev);

	power_supply_unregister(mt_charger->chg_psy);
	power_supply_unregister(mt_charger->ac_psy);
	power_supply_unregister(mt_charger->usb_psy);
	power_supply_unregister(mt_charger->battery_psy);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mt_charger_suspend(struct device *dev)
{
	/* struct mt_charger *mt_charger = dev_get_drvdata(dev); */
	return 0;
}

static int mt_charger_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mt_charger *mt_charger = platform_get_drvdata(pdev);

	power_supply_changed(mt_charger->chg_psy);
	power_supply_changed(mt_charger->ac_psy);
	power_supply_changed(mt_charger->usb_psy);
	power_supply_changed(mt_charger->battery_psy);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mt_charger_pm_ops, mt_charger_suspend,
	mt_charger_resume);

static const struct of_device_id mt_charger_match[] = {
	{ .compatible = "mediatek,mt-charger", },
	{ },
};
static struct platform_driver mt_charger_driver = {
	.probe = mt_charger_probe,
	.remove = mt_charger_remove,
	.driver = {
		.name = "mt-charger-det",
		.owner = THIS_MODULE,
		.pm = &mt_charger_pm_ops,
		.of_match_table = mt_charger_match,
	},
};

/* Legacy api to prevent build error */
bool upmu_is_chr_det(void)
{
	if (get_charger_type())
		return true;

	return false;
}

/* Legacy api to prevent build error */
bool pmic_chrdet_status(void)
{
	if (upmu_is_chr_det())
		return true;

	pr_notice("%s: No charger\n", __func__);
	return false;
}

enum charger_type mt_get_charger_type(void)
{
	return g_chr_type;
}

static s32 __init mt_charger_det_init(void)
{
	return platform_driver_register(&mt_charger_driver);
}

static void __exit mt_charger_det_exit(void)
{
	platform_driver_unregister(&mt_charger_driver);
}

subsys_initcall(mt_charger_det_init);
module_exit(mt_charger_det_exit);

MODULE_DESCRIPTION("mt-charger-detection");
MODULE_AUTHOR("MediaTek");
MODULE_LICENSE("GPL v2");
