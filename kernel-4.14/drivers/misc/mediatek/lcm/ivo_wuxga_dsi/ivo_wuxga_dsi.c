/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* Copyright (C) 2019 SANYO Techno Solutions Tottori Co., Ltd.
*
*/

#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#endif

#include "lcm_drv.h"

static struct regulator *lcm_vgp;

/* get LDO supply */
int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_notice("Kernel/LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_notice("Kernel/LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_notice("Kernel/lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_err("Kernel/LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_err("Kernel/LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("Kernel/LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		pr_err("Kernel/LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_err("Kernel/LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("Kernel/LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_err("Kernel/LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("Kernel/LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("Kernel/LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static unsigned int GPIO_LCD_PWR;
static unsigned int GPIO_LCD_BL_EN;
static unsigned int GPIO_LCD_RST;

static int lcm_get_gpio(struct device *dev)
{
	GPIO_LCD_PWR = of_get_named_gpio(dev->of_node, "gpio_lcd_pwr", 0);
	gpio_request(GPIO_LCD_PWR, "GPIO_LCD_PWR");
	pr_err("[Kernel/LCM] GPIO_LCD_PWR = 0x%x\n", GPIO_LCD_PWR);

	GPIO_LCD_BL_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN, "GPIO_LCD_BL_EN");
	pr_err("[Kernel/LCM] GPIO_LCD_BL_EN = 0x%x\n", GPIO_LCD_BL_EN);

	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	pr_err("[Kernel/LCM] GPIO_LCD_RST = 0x%x\n", GPIO_LCD_RST);

	return 0;
}

static int lcm_gpio_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	pr_err("Kernel/LCM: %s start \n",__func__);

	lcm_get_vgp_supply(dev);
	lcm_get_gpio(dev);

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,ivo_wuxga_dsi",},
	{}
};

static struct platform_driver lcm_driver = {
    .probe = lcm_gpio_probe,
	.driver = {
		   .name = "ivo_wuxga_dsi",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define FRAME_WIDTH  (1200)
#define FRAME_HEIGHT (1920)

#ifndef	GPIO_OUT_ZERO
#define	GPIO_OUT_ZERO	(0)
#endif

#ifndef	GPIO_OUT_ONE
#define	GPIO_OUT_ONE	(1)
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static struct LCM_UTIL_FUNCS lcm_util = {0};

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             0xFE
#define REGFLAG_END_OF_TABLE      0xFF   // END OF REGISTERS MARKER

#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)


static struct LCM_setting_table lcm_initialization_setting[] = {
#if 0
	//Page0
	{0xE0, 1, {0x00}},
	//--- PASSWORD  ----//
	{0xE1, 1, {0x93}},
	{0xE2, 1, {0x65}},
	{0xE3, 1, {0xF8}},

	//Page1
	{0xE0, 1, {0x01}},

	//Set VCOM                                                                                          
	{0x00, 1, {0x00}},
	{0x01, 1, {0x65}}, //5B                                                                             
	//Set VCOM_Reverse                                                                                  
	{0x03, 1, {0x00}},
	{0x04, 1, {0x6B}}, //5B

	//Set Gamma Power, VGMP,VGMN,VGSP,VGSN
	{0x17, 1, {0x00}},
	{0x18, 1, {0xEF}},
	{0x19, 1, {0x00}},
	{0x1A, 1, {0x00}},
	{0x1B, 1, {0xEF}},
	{0x1C, 1, {0x00}},

	//Set Gate Power
	{0x1F, 1, {0x6A}},
	{0x20, 1, {0x28}},
	{0x21, 1, {0x28}},
	{0x22, 1, {0x7E}},

	//SETPANEL
	{0x37, 1, {0x59}},

	//SET RGBCYC
	{0x38, 1, {0x05}},
	{0x39, 1, {0x00}},
	{0x3A, 1, {0x01}},
	{0x3C, 1, {0x78}},
	{0x3D, 1, {0xFF}},
	{0x3E, 1, {0xFF}},
	{0x3F, 1, {0xFF}},

	//Set TCON
	{0x40, 1, {0x06}},
	{0x41, 1, {0xA0}},
	{0x43, 1, {0x14}},
	{0x44, 1, {0x0D}},
	{0x45, 1, {0x28}},
//	{0x4A, 1, {0x30}},//for bist mode
	{0x4B, 1, {0x04}},

	//--- power voltage  ----//
	{0x55, 1, {0x0F}},
	{0x56, 1, {0x01}},
	{0x57, 1, {0x69}},
	{0x58, 1, {0x0A}},
	{0x59, 1, {0x0A}},
	{0x5A, 1, {0x2B}},
	{0x5B, 1, {0x14}},

	//--- Gamma 2.2 ----// 
	{0x5D, 1, {0x79}},
	{0x5E, 1, {0x50}},
	{0x5F, 1, {0x37}},
	{0x60, 1, {0x21}},
	{0x61, 1, {0x19}},
	{0x62, 1, {0x0B}},
	{0x63, 1, {0x08}},
	{0x64, 1, {0x00}},
	{0x65, 1, {0x1A}},
	{0x66, 1, {0x1D}},
	{0x67, 1, {0x1F}},
	{0x68, 1, {0x3D}},
	{0x69, 1, {0x31}},
	{0x6A, 1, {0x40}},
	{0x6B, 1, {0x33}},
	{0x6C, 1, {0x34}},
	{0x6D, 1, {0x26}},
	{0x6E, 1, {0x1B}},
	{0x6F, 1, {0x0F}},

	{0x70, 1, {0x79}},
	{0x71, 1, {0x59}},
	{0x72, 1, {0x43}},
	{0x73, 1, {0x36}},
	{0x74, 1, {0x30}},
	{0x75, 1, {0x1A}},
	{0x76, 1, {0x27}},
	{0x77, 1, {0x08}},
	{0x78, 1, {0x29}},
	{0x79, 1, {0x2C}},
	{0x7A, 1, {0x31}},
	{0x7B, 1, {0x54}},
	{0x7C, 1, {0x41}},
	{0x7D, 1, {0x4A}},
	{0x7E, 1, {0x44}},
	{0x7F, 1, {0x41}},
	{0x80, 1, {0x34}},
	{0x81, 1, {0x26}},
	{0x82, 1, {0x0F}},

	//Page2, for GIP
	{0xE0, 1, {0x02}},

	//GIP_L Pin mapping
	{0x00, 1, {0x49}},
	{0x01, 1, {0x48}},
	{0x02, 1, {0x47}},
	{0x03, 1, {0x46}},
	{0x04, 1, {0x45}},
	{0x05, 1, {0x44}},
	{0x06, 1, {0x4A}},
	{0x07, 1, {0x4B}},
	{0x08, 1, {0x50}},
	{0x09, 1, {0x1F}},
	{0x0A, 1, {0x1F}},
	{0x0B, 1, {0x1F}},
	{0x0C, 1, {0x1F}},
	{0x0D, 1, {0x1F}},
	{0x0E, 1, {0x1F}},
	{0x0F, 1, {0x51}},
	{0x10, 1, {0x52}},
	{0x11, 1, {0x53}},
	{0x12, 1, {0x40}},
	{0x13, 1, {0x41}},
	{0x14, 1, {0x42}},
	{0x15, 1, {0x43}},

	//GIP_R Pin mapping
	{0x16, 1, {0x49}},
	{0x17, 1, {0x48}},
	{0x18, 1, {0x47}},
	{0x19, 1, {0x46}},
	{0x1A, 1, {0x45}},
	{0x1B, 1, {0x44}},
	{0x1C, 1, {0x4A}},
	{0x1D, 1, {0x4B}},
	{0x1E, 1, {0x50}},
	{0x1F, 1, {0x1F}},
	{0x20, 1, {0x1F}},
	{0x21, 1, {0x1F}},
	{0x22, 1, {0x1F}},
	{0x23, 1, {0x1F}},
	{0x24, 1, {0x1F}},
	{0x25, 1, {0x51}},
	{0x26, 1, {0x52}},
	{0x27, 1, {0x53}},
	{0x28, 1, {0x40}},
	{0x29, 1, {0x41}},
	{0x2A, 1, {0x42}},
	{0x2B, 1, {0x43}},

	//GIP_L_GS Pin mapping
	{0x2C, 1, {0x0A}},
	{0x2D, 1, {0x0B}},
	{0x2E, 1, {0x04}},
	{0x2F, 1, {0x05}},
	{0x30, 1, {0x06}},
	{0x31, 1, {0x07}},
	{0x32, 1, {0x09}},
	{0x33, 1, {0x08}},
	{0x34, 1, {0x03}},
	{0x35, 1, {0x1F}},
	{0x36, 1, {0x1F}},
	{0x37, 1, {0x1F}},
	{0x38, 1, {0x1F}},
	{0x39, 1, {0x1F}},
	{0x3A, 1, {0x1F}},
	{0x3B, 1, {0x02}},
	{0x3C, 1, {0x01}},
	{0x3D, 1, {0x00}},
	{0x3E, 1, {0x13}},
	{0x3F, 1, {0x12}},
	{0x40, 1, {0x11}},
	{0x41, 1, {0x10}},

	//GIP_R_GS Pin mapping
	{0x42, 1, {0x0A}},
	{0x43, 1, {0x0B}},
	{0x44, 1, {0x04}},
	{0x45, 1, {0x05}},
	{0x46, 1, {0x06}},
	{0x47, 1, {0x07}},
	{0x48, 1, {0x09}},
	{0x49, 1, {0x08}},
	{0x4A, 1, {0x03}},
	{0x4B, 1, {0x1F}},
	{0x4C, 1, {0x1F}},
	{0x4D, 1, {0x1F}},
	{0x4E, 1, {0x1F}},
	{0x4F, 1, {0x1F}},
	{0x50, 1, {0x1F}},
	{0x51, 1, {0x02}},
	{0x52, 1, {0x01}},
	{0x53, 1, {0x00}},
	{0x54, 1, {0x13}},
	{0x55, 1, {0x12}},
	{0x56, 1, {0x11}},
	{0x57, 1, {0x10}},

	//GIP Timing  
	{0x58, 1, {0x40}},
	{0x59, 1, {0x00}},
	{0x5A, 1, {0x00}},
	{0x5B, 1, {0x30}},
	{0x5C, 1, {0x07}},
	{0x5D, 1, {0x40}},
	{0x5E, 1, {0x01}},
	{0x5F, 1, {0x02}},
	{0x60, 1, {0x40}},
	{0x61, 1, {0x01}},
	{0x62, 1, {0x02}},
	{0x63, 1, {0x70}},
	{0x64, 1, {0x6B}},
	{0x65, 1, {0x75}},
	{0x66, 1, {0x0B}},
	{0x67, 1, {0x74}},
	{0x68, 1, {0x01}},
	{0x69, 1, {0x64}},
	{0x6A, 1, {0x65}},
	{0x6B, 1, {0x00}},
	{0x6C, 1, {0x00}},
	{0x6D, 1, {0x04}},
	{0x6E, 1, {0x04}},
	{0x6F, 1, {0x89}},
	{0x70, 1, {0x00}},
	{0x71, 1, {0x00}},
	{0x72, 1, {0x06}},
	{0x73, 1, {0x7B}},
	{0x74, 1, {0x00}},
	{0x75, 1, {0x3C}},
	{0x76, 1, {0x00}},
	{0x77, 1, {0x0D}},
	{0x78, 1, {0x2C}},
	{0x79, 1, {0x00}},
	{0x7A, 1, {0x00}},
	{0x7B, 1, {0x00}},
	{0x7C, 1, {0x00}},
	{0x7D, 1, {0x03}},
	{0x7E, 1, {0x7B}},

	//Page4
	{0xE0, 1, {0x04}},
	{0x09, 1, {0x14}},
	{0x2B, 1, {0x2B}},
	{0x2E, 1, {0x44}},

	// Add ESD Protect
	//Page0
	{0xE0, 1, {0x00}},
	{0xE6, 1, {0x02}},
	{0xE7, 1, {0x0C}},
#endif

	//SLP OUT
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {0x00}},
	//DISP ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 5, {0x00}},

	//--- TE----//
	{0x35, 1, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {0x00}}
};

static struct LCM_setting_table lcm_dispoff_setting[] = {
	//DISP OFF
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 50, {0x00}},
	//SLP IN
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 100, {0x00}}
};

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_set_value(GPIO, output);
}

static void lcm_init_power(void)
{
	pr_err("[Kernel/LCM] lcm_init_power() enter\n");
	lcm_vgp_supply_enable();
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
}

static void lcm_suspend_power(void)
{
	pr_err("[Kernel/LCM] lcm_suspend_power() enter\n");

	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
	lcm_vgp_supply_disable();
//	MDELAY(500);
}

static void lcm_resume_power(void)
{
	pr_err("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_vgp_supply_enable();
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
//	MDELAY(50);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE :
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				break;
		}
	}
}

static void init_lcm_registers(void)
{
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	pr_err("[Kernel/LCM] lcm_get_params() enter\n");

	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;

	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;
	params->dsi.intermediat_buffer_num = 0;	/* This para should be 0 at video mode */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.horizontal_sync_active 	= 1;
	params->dsi.horizontal_backporch  	= 60;
	params->dsi.horizontal_frontporch 	= 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.vertical_sync_active 	= 1;
	params->dsi.vertical_backporch  	= 25;
	params->dsi.vertical_frontporch 	= 35;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.PLL_CLOCK 		  	= 478;
	params->dsi.ssc_disable = 1;
}

static void lcm_init_lcm(void)
{
	init_lcm_registers();
}

static void lcm_suspend(void)
{
	//lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);

	push_table(lcm_dispoff_setting, sizeof(lcm_dispoff_setting) / sizeof(struct LCM_setting_table), 1);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);

	MDELAY(120);

	pr_err("[Kernel/LCM] lcm_suspend() enter\n");
}

static void lcm_resume(void)
{
	pr_err("[Kernel/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);

	init_lcm_registers();

	MDELAY(33);

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
}

struct LCM_DRIVER ivo_wuxga_dsi_lcm_drv = {
    .name		= "ivo_wuxga_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power		= lcm_init_power,
	.resume_power 	= lcm_resume_power,
	.suspend_power 	= lcm_suspend_power,
};
