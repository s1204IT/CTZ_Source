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

#include <linux/i2c.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#endif

#include "lcm_drv.h"


/***********************************************/
#if 0
static struct i2c_client *lcd_i2c_client;
static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "kd,lcd_i2c" },
		{},
};
static const struct i2c_device_id lcd_i2c_id[] = {
	{ "lcd_i2c", 0 },
	{ }
};

static int lcd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_info("lcd_i2c_probe %x\n",client->addr);
	lcd_i2c_client  = client;
	return 0;
}

static int lcd_i2c_remove(struct i2c_client *client)
{
	lcd_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static struct i2c_driver lcd_i2c_driver = {
	.id_table	= lcd_i2c_id,
	.probe		= lcd_i2c_probe,
	.remove		= lcd_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "lcd_i2c",
		.of_match_table = lcm_of_match,
	},

};

static int lcd_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = lcd_i2c_client;
	char write_data[2] = {0};

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("lcd i2c write data fail !!\n");
	return ret;
}
#endif
/***********************************************/

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

	lcm_get_gpio(dev);

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,kd101n92_45ni_a003_dsi",},
	{}
};

static struct platform_driver lcm_driver = {
    .probe = lcm_gpio_probe,
	.driver = {
		   .name = "kd101n92_45ni_a003_dsi",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
//	i2c_add_driver(&lcd_i2c_driver);
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
//	i2c_del_driver(&lcd_i2c_driver);
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

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)

#define MDELAY(n) (lcm_util.mdelay(n))

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
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
}

static void lcm_suspend_power(void)
{
	pr_err("[Kernel/LCM] lcm_suspend_power() enter\n");

//	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
//	MDELAY(500);
}

static void lcm_resume_power(void)
{
	pr_err("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(50);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

#if 0
static void init_lcm_registers(void)
{
	lcd_write_byte(0xB0,0x5A);
	lcd_write_byte(0xB1,0x00);
	lcd_write_byte(0x89,0x01);
	lcd_write_byte(0x91,0x3F);

	lcd_write_byte(0xB1,0x03);
	lcd_write_byte(0x2C,0x28);
	lcd_write_byte(0x00,0xF1);
	lcd_write_byte(0x01,0x78);
	lcd_write_byte(0x02,0x3C);
	lcd_write_byte(0x03,0x1E);
	lcd_write_byte(0x04,0x8F);
	lcd_write_byte(0x05,0x01);
	lcd_write_byte(0x06,0x00);
	lcd_write_byte(0x07,0x00);
	lcd_write_byte(0x08,0x00);
	lcd_write_byte(0x09,0x00);
	lcd_write_byte(0x0A,0x01);
	lcd_write_byte(0x0B,0x3C);
	lcd_write_byte(0x0C,0x00);
	lcd_write_byte(0x0D,0x00);
	lcd_write_byte(0x0E,0x24);
	lcd_write_byte(0x0F,0x1C);

	lcd_write_byte(0x10,0xC8);
	lcd_write_byte(0x11,0x60);
	lcd_write_byte(0x12,0x70);
	lcd_write_byte(0x13,0x01);
	lcd_write_byte(0x14,0xE3);
	lcd_write_byte(0x15,0xFF);
	lcd_write_byte(0x16,0x3D);
	lcd_write_byte(0x17,0x0E);
	lcd_write_byte(0x18,0x01);
	lcd_write_byte(0x19,0x00);
	lcd_write_byte(0x1A,0x00);
	lcd_write_byte(0x1B,0xFC);

	lcd_write_byte(0x1C,0x0B);
	lcd_write_byte(0x1D,0xA0);
	lcd_write_byte(0x1E,0x03);
	lcd_write_byte(0x1F,0x04);

	lcd_write_byte(0x20,0x0C);
	lcd_write_byte(0x21,0x00);
	lcd_write_byte(0x22,0x04);
	lcd_write_byte(0x23,0x81);
	lcd_write_byte(0x24,0x1F);
	lcd_write_byte(0x25,0x10);
	lcd_write_byte(0x26,0x9B);
	lcd_write_byte(0x2D,0x01);
	lcd_write_byte(0x2E,0x84);
	lcd_write_byte(0x2F,0x00);

	lcd_write_byte(0x30,0x02);
	lcd_write_byte(0x31,0x08);
	lcd_write_byte(0x32,0x01);
	lcd_write_byte(0x33,0x1C);
	lcd_write_byte(0x34,0x70);
	lcd_write_byte(0x35,0xFF);
	lcd_write_byte(0x36,0xFF);
	lcd_write_byte(0x37,0xFF);
	lcd_write_byte(0x38,0xFF);
	lcd_write_byte(0x39,0xFF);
	lcd_write_byte(0x3A,0x05);
	lcd_write_byte(0x3B,0x00);
	lcd_write_byte(0x3C,0x00);
	lcd_write_byte(0x3D,0x00);
	lcd_write_byte(0x3E,0x0F);
	lcd_write_byte(0x3F,0xA4);

	lcd_write_byte(0x40,0x28);
	lcd_write_byte(0x41,0xFC);
	lcd_write_byte(0x42,0x01);
	lcd_write_byte(0x43,0x00);
	lcd_write_byte(0x44,0x05);
	lcd_write_byte(0x45,0xF0);
	lcd_write_byte(0x46,0x01);
	lcd_write_byte(0x47,0x02);
	lcd_write_byte(0x48,0x00);
	lcd_write_byte(0x49,0x58);
	lcd_write_byte(0x4A,0x00);
	lcd_write_byte(0x4B,0x05);
	lcd_write_byte(0x4C,0x03);
	lcd_write_byte(0x4D,0xD0);
	lcd_write_byte(0x4E,0x13);
	lcd_write_byte(0x4F,0xFF);

	lcd_write_byte(0x50,0x0A);
	lcd_write_byte(0x51,0x53);
	lcd_write_byte(0x52,0x26);
	lcd_write_byte(0x53,0x22);
	lcd_write_byte(0x54,0x09);
	lcd_write_byte(0x55,0x22);
	lcd_write_byte(0x56,0x00);
	lcd_write_byte(0x57,0x1C);
	lcd_write_byte(0x58,0x03);
	lcd_write_byte(0x59,0x3F);
	lcd_write_byte(0x5A,0x28);
	lcd_write_byte(0x5B,0x01);
	lcd_write_byte(0x5C,0xCC);
	lcd_write_byte(0x5D,0x21);
	lcd_write_byte(0x5E,0x04);
	lcd_write_byte(0x5F,0x13);

	lcd_write_byte(0x60,0x42);
	lcd_write_byte(0x61,0x08);
	lcd_write_byte(0x62,0x64);
	lcd_write_byte(0x63,0xEB);
	lcd_write_byte(0x64,0x10);
	lcd_write_byte(0x65,0xA8);
	lcd_write_byte(0x66,0x84);
	lcd_write_byte(0x67,0x8E);
	lcd_write_byte(0x68,0x29);
	lcd_write_byte(0x69,0x11);
	lcd_write_byte(0x6A,0x42);
	lcd_write_byte(0x6B,0x38);
	lcd_write_byte(0x6C,0x21);
	lcd_write_byte(0x6D,0x84);
	lcd_write_byte(0x6E,0x50);
	lcd_write_byte(0x6F,0xB6);

	lcd_write_byte(0x70,0x0E);
	lcd_write_byte(0x71,0xA1);
	lcd_write_byte(0x72,0xCE);
	lcd_write_byte(0x73,0xF8);
	lcd_write_byte(0x74,0xDA);
	lcd_write_byte(0x75,0x1A);
	lcd_write_byte(0x76,0x00);
	lcd_write_byte(0x77,0x00);
	lcd_write_byte(0x78,0x5F);
	lcd_write_byte(0x79,0xE0);
	lcd_write_byte(0x7A,0x01);
	lcd_write_byte(0x7C,0xFF);
	lcd_write_byte(0x7D,0xFF);//00
	lcd_write_byte(0x7E,0xFF);//46
	lcd_write_byte(0x7F,0xCE);

	lcd_write_byte(0xB1,0x02);
	lcd_write_byte(0x00,0xFF);
	lcd_write_byte(0x01,0x01);
	lcd_write_byte(0x02,0x00);
	lcd_write_byte(0x03,0x00);
	lcd_write_byte(0x04,0x00);
	lcd_write_byte(0x05,0x00);
	lcd_write_byte(0x06,0x00);
	lcd_write_byte(0x07,0x00);
	lcd_write_byte(0x08,0xC0);
	lcd_write_byte(0x09,0x00);
	lcd_write_byte(0x0A,0x00);
	lcd_write_byte(0x0B,0x04);
	lcd_write_byte(0x0C,0xE6);
	lcd_write_byte(0x0D,0x0D);
	lcd_write_byte(0x0F,0x08);

//========gamma code(20200306)===========
	lcd_write_byte(0x10,0xE5);
	lcd_write_byte(0x11,0xA8);
	lcd_write_byte(0x12,0xF4);
	lcd_write_byte(0x13,0x74);
	lcd_write_byte(0x14,0x6A);
	lcd_write_byte(0x15,0xD9);
	lcd_write_byte(0x16,0x64);
	lcd_write_byte(0x17,0x21);
	lcd_write_byte(0x18,0x31);
	lcd_write_byte(0x19,0x93);
	lcd_write_byte(0x1A,0xA6);
	lcd_write_byte(0x1B,0x0F);
	lcd_write_byte(0x1C,0xFF);
	lcd_write_byte(0x1D,0xFF);
	lcd_write_byte(0x1E,0xFF);
	lcd_write_byte(0x1F,0xFF);

	lcd_write_byte(0x20,0xFF);
	lcd_write_byte(0x21,0xFF);
	lcd_write_byte(0x22,0xFF);
	lcd_write_byte(0x23,0xFF);
	lcd_write_byte(0x24,0xFF);
	lcd_write_byte(0x25,0xFF);
	lcd_write_byte(0x26,0xFF);
	lcd_write_byte(0x27,0x1F);
	lcd_write_byte(0x28,0xC8);//VCOM 
	lcd_write_byte(0x29,0xFF);
	lcd_write_byte(0x2A,0xFF);
	lcd_write_byte(0x2B,0xFF);
	lcd_write_byte(0x2C,0x07);//vcom otp times
	lcd_write_byte(0x2D,0x03);

	lcd_write_byte(0x33,0x09);
	lcd_write_byte(0x35,0x7F);
	lcd_write_byte(0x36,0x0C);
	lcd_write_byte(0x38,0x7F);
	lcd_write_byte(0x3A,0x80);
	lcd_write_byte(0x3B,0x55);
	lcd_write_byte(0x3C,0xE2);
	lcd_write_byte(0x3D,0x32);
	lcd_write_byte(0x3E,0x00);
	lcd_write_byte(0x3F,0x58);

	lcd_write_byte(0x40,0x06);
	lcd_write_byte(0x41,0x80);
	lcd_write_byte(0x42,0xCB);
	lcd_write_byte(0x43,0x2C);
	lcd_write_byte(0x44,0x61);
	lcd_write_byte(0x45,0x39);
	lcd_write_byte(0x46,0x00);
	lcd_write_byte(0x47,0x00);
	lcd_write_byte(0x48,0x8B);
	lcd_write_byte(0x49,0xD2);
	lcd_write_byte(0x4A,0x01);
	lcd_write_byte(0x4B,0x00);
	lcd_write_byte(0x4C,0x10);
	lcd_write_byte(0x4D,0xC0);
	lcd_write_byte(0x4E,0x0F);
	lcd_write_byte(0x4F,0xF1);

	lcd_write_byte(0x50,0x78);
	lcd_write_byte(0x51,0x7A);
	lcd_write_byte(0x52,0x34);
	lcd_write_byte(0x53,0x99);
	lcd_write_byte(0x54,0xA2);
	lcd_write_byte(0x55,0x03);
	lcd_write_byte(0x56,0x6C);
	lcd_write_byte(0x57,0x1A);
	lcd_write_byte(0x58,0x05);
	lcd_write_byte(0x59,0x30);
	lcd_write_byte(0x5A,0x1E);
	lcd_write_byte(0x5B,0x8F);
	lcd_write_byte(0x5C,0xC7);
	lcd_write_byte(0x5D,0xE3);
	lcd_write_byte(0x5E,0xF1);
	lcd_write_byte(0x5F,0x78);

	lcd_write_byte(0x60,0x3C);
	lcd_write_byte(0x61,0x36);
	lcd_write_byte(0x62,0x1E);
	lcd_write_byte(0x63,0x1B);
	lcd_write_byte(0x64,0x8F);
	lcd_write_byte(0x65,0xC7);
	lcd_write_byte(0x66,0xE3);
	lcd_write_byte(0x67,0x31);
	lcd_write_byte(0x68,0x14);
	lcd_write_byte(0x69,0x89);
	lcd_write_byte(0x6A,0x70);
	lcd_write_byte(0x6B,0x8C);
	lcd_write_byte(0x6C,0x8D);
	lcd_write_byte(0x6D,0x8D);
	lcd_write_byte(0x6E,0x8D);
	lcd_write_byte(0x6F,0x8D);

	lcd_write_byte(0x70,0xC7);
	lcd_write_byte(0x71,0xE3);
	lcd_write_byte(0x72,0xF1);
	lcd_write_byte(0x73,0xD8);
	lcd_write_byte(0x74,0xD8);
	lcd_write_byte(0x75,0xD8);
	lcd_write_byte(0x76,0x18);
	lcd_write_byte(0x77,0x00);
	lcd_write_byte(0x78,0x00);
	lcd_write_byte(0x79,0x00);
	lcd_write_byte(0x7A,0xC6);
	lcd_write_byte(0x7B,0xC6);
	lcd_write_byte(0x7C,0xC6);
	lcd_write_byte(0x7D,0xC6);
	lcd_write_byte(0x7E,0xC6);
	lcd_write_byte(0x7F,0xE3);

	lcd_write_byte(0x0B,0x04);

	lcd_write_byte(0xB1,0x03);
	lcd_write_byte(0x2C,0x2C); 
       
	lcd_write_byte(0xB1,0x00);
	lcd_write_byte(0x89,0x03); //display on
}
#endif

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

	params->dsi.PLL_CLOCK 		  	= 499;
	params->dsi.ssc_disable = 1;
}

static void lcm_init_lcm(void)
{
	unsigned int data_array[16];

	pr_err("[Kernel/LCM] lcm_init() enter\n");
//	init_lcm_registers();

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(180);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(180);

	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);

	MDELAY(120);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);

//	MDELAY(120);

	pr_err("[Kernel/LCM] lcm_suspend() enter\n");
}

static void lcm_resume(void)
{
#if 1
	unsigned int data_array[16];

	pr_err("[Kernel/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(180);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(33);
#else
	pr_err("[Kernel/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);

	init_lcm_registers();

	MDELAY(100);

//	lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
#endif
}

void sts_lcd_backlight_enable(bool on)
{
	if(on)
	{
		lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	}
	else
	{
		lcm_set_gpio_output(GPIO_LCD_BL_EN, GPIO_OUT_ZERO);
	}
}

struct LCM_DRIVER kd101n92_45ni_a003_dsi_lcm_drv = {
    .name		= "kd101n92_45ni_a003_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power		= lcm_init_power,
	.resume_power 	= lcm_resume_power,
	.suspend_power 	= lcm_suspend_power,
};
