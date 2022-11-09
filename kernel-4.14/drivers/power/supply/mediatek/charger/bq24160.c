/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/*
 * Copyright (C) 2018 SANYO Techno Solutions Tottori Co., Ltd.
 *
 * Changelog:
 *
 *
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#endif
#include "bq24160.h"
//#include <mt-plat/charging.h>
#include <mt-plat/mtk_boot.h>

extern volatile bool chg_init_flag;
/**********************************************************
 *
 *	[I2C Slave Setting]
 *
 *********************************************************/
#define bq24160_SLAVE_ADDR_WRITE   0xD6
#define bq24160_SLAVE_ADDR_READ    0xD7

#ifdef CONFIG_OF
static const struct of_device_id bq24160_id[] = {
		{ .compatible = "ti,bq24160" },
		{},
};
MODULE_DEVICE_TABLE(of, bq24160_id);
#endif

static struct i2c_client *new_client;
static const struct i2c_device_id bq24160_i2c_id[] = {
   { "bq24160", 0},
   {}
};
//MODULE_DEVICE_TABLE( i2c, bq24160_i2c_id);

kal_bool chargin_hw_init_done = KAL_FALSE;

static int bq24160_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static void bq24160_shutdown(struct i2c_client *client)
{
	pr_debug("[bq24160_shutdown] driver shutdown\n");
}
static struct i2c_driver bq24160_driver = {
		.driver = {
				.name    = "bq24160",
#ifdef CONFIG_OF
				.of_match_table = of_match_ptr(bq24160_id),
#endif
		},
		.id_table    = bq24160_i2c_id,
		.probe       = bq24160_driver_probe,
		.shutdown    = bq24160_shutdown,
};

//static int acdet_gpio;
//static int usbdet_gpio;
static int platform_boot_mode;

/**********************************************************
 *
 *[Global Variable]
 *
 *********************************************************/
#define bq24160_REG_NUM 8
unsigned char bq24160_reg[bq24160_REG_NUM] = {0};

#ifdef CONFIG_TOUCHSCREEN_MTK_WACOM_DIGI
extern void digi_mutex_lock(void);
extern void digi_mutex_unlock(void);
#endif

static DEFINE_MUTEX(bq24160_i2c_access);
/**********************************************************
 *
 *	[I2C Function For Read/Write bq24160]
 *
 *********************************************************/
int bq24160_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char     readData = 0;
	int      ret = 0;
	struct i2c_msg msg[2];
	struct i2c_adapter *adap = new_client->adapter;

	mutex_lock(&bq24160_i2c_access);

	msg[0].addr	 = new_client->addr;
	msg[0].flags = 0;
	msg[0].len	 = 1;
	msg[0].buf	 = &cmd;

	msg[1].addr	 = new_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len	 = 1;
	msg[1].buf	 = &readData;

#ifdef CONFIG_TOUCHSCREEN_MTK_WACOM_DIGI
	digi_mutex_lock();
#endif
	ret = i2c_transfer(adap, msg, 2);
#ifdef CONFIG_TOUCHSCREEN_MTK_WACOM_DIGI
	digi_mutex_unlock();
#endif
	if (ret < 0) {
		mutex_unlock(&bq24160_i2c_access);
		return 0;
	}
	*returnData = readData;

	mutex_unlock(&bq24160_i2c_access);

//	pr_debug( "[%s ] cmd: 0x%02X, val: 0x%02X.\n", __func__, cmd, *returnData);

	return 1;
}

int bq24160_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;
	struct i2c_msg msg;
	struct i2c_adapter *adap = new_client->adapter;

//	pr_debug( "[%s] cmd: 0x%02X, val: 0x%02X.\n", __func__, cmd, writeData);

	mutex_lock(&bq24160_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;
	msg.addr	  = new_client->addr;
	msg.flags	  = 0;
	msg.len		  = 2;
	msg.buf		  = (char *)write_data;

#ifdef CONFIG_TOUCHSCREEN_MTK_WACOM_DIGI
	digi_mutex_lock();
#endif
	ret = i2c_transfer(adap, &msg, 1);
#ifdef CONFIG_TOUCHSCREEN_MTK_WACOM_DIGI
	digi_mutex_unlock();
#endif
	if (ret < 0) {
		mutex_unlock(&bq24160_i2c_access);
		return 0;
	}

	mutex_unlock(&bq24160_i2c_access);

	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq24160_read_interface(
				  unsigned char	 RegNum,
				  unsigned char	*val,
				  unsigned char	 MASK,
				  unsigned char SHIFT)
{
	unsigned char bq24160_reg = 0;
	int ret = 0;

	pr_debug("--------------------------------------------------\n");

	ret = bq24160_read_byte(RegNum, &bq24160_reg);

	pr_debug("[bq24160_read_interface] Reg[%x]=0x%x\n", RegNum, bq24160_reg);

	bq24160_reg &= (MASK << SHIFT);
	*val = (bq24160_reg >> SHIFT);

	pr_debug("[bq24160_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq24160_config_interface(
				    unsigned char	RegNum,
				    unsigned char	val,
				    unsigned char	MASK,
				    unsigned char SHIFT)
{
	unsigned char bq24160_reg = 0;
	int ret = 0;

	pr_debug("--------------------------------------------------\n");

	ret = bq24160_read_byte(RegNum, &bq24160_reg);
	pr_debug("[bq24160_config_interface] Reg[%x]=0x%x\n", RegNum, bq24160_reg);

	if(RegNum == bq24160_CON2)
		bq24160_reg &= 0x7f; /*clear reset bit*/

	if(RegNum == bq24160_CON0)
		bq24160_reg &= 0x88; /*clear read only bit*/

	bq24160_reg &= ~(MASK << SHIFT);
	bq24160_reg |= (val << SHIFT);

	ret = bq24160_write_byte(RegNum, bq24160_reg);
	pr_debug("[bq24160_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24160_reg);

	/* Check */
//	bq24160_read_byte( RegNum, &bq24160_reg);
//	pr_debug("[bq24160_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24160_reg);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq24160_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT) );
}

unsigned int bq24160_get_tmr_rst(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON0),
				     (&val),
				     (unsigned char) (CON0_TMR_RST_MASK),
				     (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON0),
				     (&val),
				     (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

void bq24160_set_supply_sel(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_SUPPLY_SEL_MASK),
				       (unsigned char) (CON0_SUPPLY_SEL_SHIFT) );
}

unsigned int bq24160_get_supply_sel(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON0),
				     (&val),
				     (unsigned char) (CON0_SUPPLY_SEL_MASK),
				     (unsigned char) (CON0_SUPPLY_SEL_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON0),
				     (&val),
				     (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}


/* CON1---------------------------------------------------- */

unsigned int bq24160_get_instat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON1),
				     (&val),
				     (unsigned char) (CON1_INSTAT_MASK),
				     (unsigned char) (CON1_INSTAT_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_usbstat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON1),
				     (&val),
				     (unsigned char) (CON1_USBSTAT_MASK),
				     (unsigned char) (CON1_USBSTAT_SHIFT)
	    );
	return val;
}

void bq24160_set_otg_lock(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_LOCK_MASK),
				       (unsigned char) (CON1_OTG_LOCK_SHIFT) );
}

unsigned int bq24160_get_otg_lock(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON1),
				     (&val),
				     (unsigned char) (CON1_OTG_LOCK_MASK),
				     (unsigned char) (CON1_OTG_LOCK_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_batstat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON1),
				     (&val),
				     (unsigned char) (CON1_BATSTAT_MASK),
				     (unsigned char) (CON1_BATSTAT_SHIFT)
	    );
	return val;
}

void bq24160_set_en_nobatop(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_EN_NOBATOP_MASK),
				       (unsigned char) (CON1_EN_NOBATOP_SHIFT) );
}

unsigned int bq24160_get_en_nobatop(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON1),
				     (&val),
				     (unsigned char) (CON1_EN_NOBATOP_MASK),
				     (unsigned char) (CON1_EN_NOBATOP_SHIFT)
	    );
	return val;
}

/* CON2---------------------------------------------------- */

void bq24160_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_RESET_MASK),
				       (unsigned char) (CON2_RESET_SHIFT) );
}

void bq24160_set_iusb_limit(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_IUSB_LIMIT_MASK),
				       (unsigned char) (CON2_IUSB_LIMIT_SHIFT) );
}

unsigned int bq24160_get_iusb_limit(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON2),
				     (&val),
				     (unsigned char) (CON2_IUSB_LIMIT_MASK),
				     (unsigned char) (CON2_IUSB_LIMIT_SHIFT)
	    );
	return val;
}

void bq24160_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_EN_STAT_MASK),
				       (unsigned char) (CON2_EN_STAT_SHIFT) );
}

unsigned int bq24160_get_en_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON2),
				     (&val),
				     (unsigned char) (CON2_EN_STAT_MASK),
				     (unsigned char) (CON2_EN_STAT_SHIFT)
	    );
	return val;
}

void bq24160_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_TE_MASK),
				       (unsigned char) (CON2_TE_SHIFT) );
}

unsigned int bq24160_get_te(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON2),
				     (&val),
				     (unsigned char) (CON2_TE_MASK),
				     (unsigned char) (CON2_TE_SHIFT)
	    );
	return val;
}

void bq24160_set_cen(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_CE_MASK),
				       (unsigned char) (CON2_CE_SHIFT) );
}

unsigned int bq24160_get_cen(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON2),
				     (&val),
				     (unsigned char) (CON2_CE_MASK),
				     (unsigned char) (CON2_CE_SHIFT)
	    );
	return val;
}

void bq24160_set_hz_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_HZ_MODE_MASK),
				       (unsigned char) (CON2_HZ_MODE_SHIFT) );
}

unsigned int bq24160_get_hz_mode(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON2),
				     (&val),
				     (unsigned char) (CON2_HZ_MODE_MASK),
				     (unsigned char) (CON2_HZ_MODE_SHIFT)
	    );
	return val;
}

/* CON3---------------------------------------------------- */

void bq24160_set_vbreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_VBREG_MASK),
				       (unsigned char) (CON3_VBREG_SHIFT) );
}

unsigned int bq24160_get_vbreg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON3),
				     (&val),
				     (unsigned char) (CON3_VBREG_MASK),
				     (unsigned char) (CON3_VBREG_SHIFT)
	    );
	return val;
}

void bq24160_set_iinlimit(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IINLIMIT_MASK),
				       (unsigned char) (CON3_IINLIMIT_SHIFT) );
}

unsigned int bq24160_get_iinlimit(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON3),
				     (&val),
				     (unsigned char) (CON3_IINLIMIT_MASK),
				     (unsigned char) (CON3_IINLIMIT_SHIFT)
	    );
	return val;
}

void bq24160_set_ddet_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_DDET_EN_MASK),
				       (unsigned char) (CON3_DDET_EN_SHIFT) );
}

unsigned int bq24160_get_ddet_en(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON3),
				     (&val),
				     (unsigned char) (CON3_DDET_EN_MASK),
				     (unsigned char) (CON3_DDET_EN_SHIFT)
	    );
	return val;
}

/* CON4---------------------------------------------------- */

unsigned int bq24160_get_vendor(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON4),
				     (&val),
				     (unsigned char) (CON4_VENDOR_MASK),
				     (unsigned char) (CON4_VENDOR_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON4),
				     (&val),
				     (unsigned char) (CON4_PN_MASK),
				     (unsigned char) (CON4_PN_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON4),
				     (&val),
				     (unsigned char) (CON4_REVISION_MASK),
				     (unsigned char) (CON4_REVISION_SHIFT)
	    );
	return val;
}

/* CON5---------------------------------------------------- */
void bq24160_set_ichrg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_ICHRG_MASK),
				       (unsigned char) (CON5_ICHRG_SHIFT) );
}

unsigned int bq24160_get_ichrg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON5),
				     (&val),
				     (unsigned char) (CON5_ICHRG_MASK),
				     (unsigned char) (CON5_ICHRG_SHIFT)
	    );
	return val;
}

void bq24160_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_ITERM_MASK),
				       (unsigned char) (CON5_ITERM_SHIFT) );
}

unsigned int bq24160_get_iterm(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON5),
				     (&val),
				     (unsigned char) (CON5_ITERM_MASK),
				     (unsigned char) (CON5_ITERM_SHIFT)
	    );
	return val;
}


/* CON6---------------------------------------------------- */
unsigned int bq24160_get_minsys_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON6),
				     (&val),
				     (unsigned char) (CON6_MINSYS_STATUS_MASK),
				     (unsigned char) (CON6_MINSYS_STATUS_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_dpm(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON6),
				     (&val),
				     (unsigned char) (CON6_DPM_MASK),
				     (unsigned char) (CON6_DPM_SHIFT)
	    );
	return val;
}

void bq24160_set_vindpm_usb(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_USB_MASK),
				       (unsigned char) (CON6_VINDPM_USB_SHIFT) );
}

unsigned int bq24160_get_vindpm_usb(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON6),
				     (&val),
				     (unsigned char) (CON6_VINDPM_USB_MASK),
				     (unsigned char) (CON6_VINDPM_USB_SHIFT)
	    );
	return val;
}

void bq24160_set_vindpm_in(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_IN_MASK),
				       (unsigned char) (CON6_VINDPM_IN_SHIFT) );
}

unsigned int bq24160_get_vindpm_in(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON6),
				     (&val),
				     (unsigned char) (CON6_VINDPM_IN_MASK),
				     (unsigned char) (CON6_VINDPM_IN_SHIFT)
	    );
	return val;
}

/* CON7---------------------------------------------------- */
void bq24160_set_2xtmr_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_2XTMR_EN_MASK),
				       (unsigned char) (CON7_2XTMR_EN_SHIFT) );
}

unsigned int bq24160_get_2xtmr_en(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON7),
				     (&val),
				     (unsigned char) (CON7_2XTMR_EN_MASK),
				     (unsigned char) (CON7_2XTMR_EN_SHIFT)
	    );
	return val;
}

void bq24160_set_tmr(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR_MASK),
				       (unsigned char) (CON7_TMR_SHIFT) );
}

unsigned int bq24160_get_tmr(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON7),
				     (&val),
				     (unsigned char) (CON7_TMR_MASK),
				     (unsigned char) (CON7_TMR_SHIFT)
	    );
	return val;
}

void bq24160_set_ts_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TS_EN_MASK),
				       (unsigned char) (CON7_TS_EN_SHIFT) );
}

unsigned int bq24160_get_ts_en(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON7),
				     (&val),
				     (unsigned char) (CON7_TS_EN_MASK),
				     (unsigned char) (CON7_TS_EN_SHIFT)
	    );
	return val;
}

unsigned int bq24160_get_ts_fault(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON7),
				     (&val),
				     (unsigned char) (CON7_TS_FAULT_MASK),
				     (unsigned char) (CON7_TS_FAULT_SHIFT)
	    );
	return val;
}

void bq24160_set_low_chg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24160_config_interface((unsigned char) (bq24160_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_LOW_CHG_MASK),
				       (unsigned char) (CON7_LOW_CHG_SHIFT) );
}

unsigned int bq24160_get_low_chg(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24160_read_interface((unsigned char) (bq24160_CON7),
				     (&val),
				     (unsigned char) (CON7_LOW_CHG_MASK),
				     (unsigned char) (CON7_LOW_CHG_SHIFT)
	    );
	return val;
}

/* GPIO---------------------------------------------------- */
unsigned int bq24160_get_charger_type(void)
{
#if 0
	unsigned char val = 0;
	if(gpio_get_value(acdet_gpio) == 0)
		val = 1;
	else if(gpio_get_value(usbdet_gpio) == 0)
		val = 2;

	if(chargin_hw_init_done) {
		if(bq24160_get_instat() == 3 && bq24160_get_usbstat() == 3)
			val = 0;
	}
#else
	unsigned char val = 0;
	int acstat,usbstat;

	if(chargin_hw_init_done) {
		acstat = bq24160_get_instat();
		usbstat = bq24160_get_usbstat();

		if(acstat == 3 && usbstat == 3)
			val = 0;
		else if(acstat == 0)
			val = 1;
		else if(usbstat == 0)
			val = 2;
		else
			val = 0;
	}
#endif

	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
 *********************************************************/
void bq24160_dump_register(void)
{
	int i = 0;

//	pr_debug("[bq24160] ");
	printk("[bq24160] ");
	for (i = 0; i < bq24160_REG_NUM; i++) {
		bq24160_read_byte(i, &bq24160_reg[i]);
		printk("[0x%x]=0x%x ", i, bq24160_reg[i]);
	}
	printk("\n");
}

extern int android_device_connected(void);

void bq24160_charge_init(void)
{
//	bq24160_dump_register();
	bq24160_set_hz_mode(0);/*disable*/
	bq24160_set_te(0x1);/*enable*/
//	if(android_device_connected())
		bq24160_set_iusb_limit(0x2);/*500mA*/
	bq24160_set_cen(0);/*charger enable*/
	if (platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			    || platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
//		bq24160_set_iinlimit(1);//2.5A
		bq24160_set_iinlimit(0);//1.5A
	}
	else {
		bq24160_set_iinlimit(0);//1.5A
	}
	bq24160_set_tmr_rst(0x1); /*Kick watchdog*/
#ifdef A05BA
	bq24160_set_tmr(0x1);/*6hour*/
	bq24160_set_iterm(0x2);/*150mA*/
	bq24160_set_ichrg(0x0a);/*1.3A*/
	bq24160_set_vbreg(0x27);/*4.28V*/
#else
	bq24160_set_tmr(0x2);/*9hour*/
	bq24160_set_iterm(0x4);/*250mA*/
	bq24160_set_ichrg(0x0a);/*1.3A*/
	bq24160_set_vbreg(0x29);/*4.32V*/
#endif
	bq24160_set_en_stat(0x1);/*enable*/
	bq24160_set_ddet_en(1);

	bq24160_dump_register();
}

static int bq24160_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ena_gpio;

	pr_notice( "[%s] called.\n", __func__);

#ifdef CONFIG_OF
	// ena_gpio
	ena_gpio = of_get_named_gpio( client->dev.of_node, "ena-gpio", 0);
	pr_info( "[%s] of_get_named_gpio( ena): %d\n", __func__, ena_gpio);

	if(ena_gpio >= 0) {
		gpio_request( ena_gpio, NULL);
		gpio_direction_output( ena_gpio, 0);	// Low: active
	}
#if 0
	acdet_gpio = of_get_named_gpio( client->dev.of_node, "acdet-gpio", 0);
	pr_info( "[%s] of_get_named_gpio(acdet): %d\n", __func__, acdet_gpio);

	if(acdet_gpio >= 0) {
		gpio_request( acdet_gpio, NULL);
		gpio_direction_input( acdet_gpio );	// Low: active
	}

	usbdet_gpio = of_get_named_gpio( client->dev.of_node, "usbdet-gpio", 0);
	pr_info( "[%s] of_get_named_gpio(usbdet): %d\n", __func__, usbdet_gpio);

	if(usbdet_gpio >= 0) {
		gpio_request( usbdet_gpio, NULL);
		gpio_direction_input( usbdet_gpio );	// Low: active
	}
#endif
#endif /* CONFIG_OF */

	new_client = client;

	platform_boot_mode = get_boot_mode();

	/* --------------------- */
//	bq24160_dump_register();

	chargin_hw_init_done = KAL_TRUE;
	chg_init_flag = true;
	
	return 0;
}

static int __init bq24160_init(void)
{
	pr_notice("[bq24160_init] init start\n");

	if (i2c_add_driver(&bq24160_driver) != 0)
		pr_notice("[bq24160_init] failed to register bq24160 i2c driver.\n");
	else
		pr_notice( "[%s] Success to register bq24160 i2c driver.\n", __func__);

	return 0;
}

static void __exit bq24160_exit(void)
{
	i2c_del_driver(&bq24160_driver);
}

module_init(bq24160_init);
module_exit(bq24160_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24160 Driver");
MODULE_AUTHOR("STS");
