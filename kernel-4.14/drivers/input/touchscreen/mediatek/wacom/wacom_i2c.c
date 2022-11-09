/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2019 Tatsunosuke Tobita, Wacom.
 *		<tobita.tatsunosuke@wacom.co.jp>
 * Copyright (c) 2011-2019 Martin Chen, Wacom.
 *		<martin.chen@wacom.com>, modify for G12T plus
 *
 * Copyright (C) 2019 SANYO Techno Solutions Tottori Co., Ltd.
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */
#include "wacom.h"
#include <linux/acpi.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

//#define SWAP
//#define SWAP_X

#define RESET 1
#define MAX_SLOT 1028
//#define SHOW_REPORTDESC
#ifdef CONFIG_ACPI
	#define WACOM_HARDWARE_ID "WCOM50C1"
#endif

#define MTK_INPUT
#ifdef MTK_INPUT
#include "tpd.h"
#endif

//#include <linux/wakelock.h>
//struct wake_lock wacom_i2c_suspend_lock;

static bool irq_enabled = true;

static int touch_suspend = 0;

static int touch_action = 0;

static int input_end_send_flag = 0;
static int input_end_key_send_flag = 0;

static int tp_behavior_enable = 1;

static struct wacom_i2c *wac_i2c = NULL;

#include <linux/semaphore.h>
static struct semaphore tp_semaphore;

#define AES_FW_BASE             0x100


// char cmd_data[] = {0x04, 0x00, 0x34, 0x02, 0x05, 0x00};
// 0x04 : wacom command register - LSB
// 0x00 : wacom command register - MSB
// 0x34 : b7-b6 Reserved
//        b5-b4 ReportType  00b Reserved  01b Input  10b Output  11b Feature 
//        b3-b0 ReportID = 4
// 0x02 : b7-b4 Reserved
//        b3-b0 OpCode  0010b GET_REPORT  0011b SET_REPORT
// 0x05 : wacom data register - LSB
// 0x00 : wacom data register - MSB
static int get_touch_query(struct i2c_client *client)
{
	int ret;
	char cmd_data[] = {0x04, 0x00, 0x34, 0x02, 0x05, 0x00};
	char databuf[18] = {0};
	unsigned int fw_ver = 0;
//	int i;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd_data),
			.buf = cmd_data,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(databuf),
			.buf = databuf,
		},
	};
	
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

#if 0
	printk("get_touch_query ");
	for(i = 0; i < sizeof(databuf);i++)
		printk("%02x ",databuf[i]);
	printk("----\n");
#endif

	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining query failed: %d\n", __func__, ret);
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s input/output error occured;\n returned: %dbyte(s)\n", __func__, ret);
	}

	fw_ver = (unsigned int)databuf[14] << 8 | databuf[13];
	fw_ver = fw_ver * AES_FW_BASE + databuf[15];

	return fw_ver;
}

static void parse_report_desc(struct wacom_features *features, u8 *report_desc, int report_desc_size)
{
	bool finger = false, pen = false;
	int i;
	int usage = 0, mouse = 0;

	for (i = 0; i < report_desc_size; i++) {
		switch (report_desc[i]) {
		case USAGE_PAGE:
			switch (report_desc[i + 1]) {
			case USAGE_PAGE_DIGITIZERS:
				usage = USAGE_PAGE_DIGITIZERS;
				if (report_desc[i + 3] == USAGE_TOUCHSCREEN || report_desc[i + 3] == USAGE_PEN) {
					mouse = 0;
					i += 4;
				}
				else
					i++;
				
				break;

			case USAGE_PAGE_DESKTOP:
				usage = USAGE_PAGE_DESKTOP;
				if (report_desc[i + 3] == USAGE_MOUSE) {
					mouse = 1;
					i += 4;
				}
				else
					i++;
				
				break;
			}
			break;

		case USAGE:
			switch (report_desc[i + 1]) {
			case USAGE_X:
				if (usage == USAGE_PAGE_DESKTOP && mouse == 0) {
					if (pen)
						features->x_max = get_unaligned_le16(&report_desc[i + 3]);
					else if (finger)
						features->x_touch = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else if (usage == USAGE_PAGE_DIGITIZERS && mouse == 0) {
					if (pen)
						features->pressure_max = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else
					i++;
				
				break;
				
			case USAGE_Y:
				if (usage == USAGE_PAGE_DESKTOP && mouse == 0) {
					if (pen)
						features->y_max = get_unaligned_le16(&report_desc[i + 3]);
					else if (finger)
						features->y_touch = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else
					i++;
				
				break;
			case USAGE_FINGER:
				finger = true;
				pen = false;
				i++;
				break;

			case USAGE_STYLUS:
				pen = true;
				finger = false;
				i++;
				break;
			}
			break;
		}		
	}
}

static int wacom_get_report_descriptor(struct i2c_client *client, struct wacom_features *features,
			 HID_DESC hid_desc)
{
#ifdef SHOW_REPORTDESC
	int i = 0;
#endif
	int ret = -1;
	int report_desc_size = hid_desc.wReportDescLength;
	u8 cmd_reportDesc[] = {hid_desc.wReportDescRegister, 0x00};
	u8 *report_desc = NULL;
	struct i2c_msg msgs_desc[2];

	dev_dbg(&client->dev, "Retrieving report descriptor\n");
	report_desc = kzalloc(sizeof(u8) * report_desc_size, GFP_KERNEL);
	if (!report_desc) {
		dev_err(&client->dev, "No memory left for this device\n");
		return -ENOMEM;
	}

	msgs_desc[0].addr = client->addr;
	msgs_desc[0].flags = 0;
	msgs_desc[0].len = sizeof(cmd_reportDesc);
	msgs_desc[0].buf = cmd_reportDesc;

	msgs_desc[1].addr = client->addr;
	msgs_desc[1].flags = I2C_M_RD;
	msgs_desc[1].len = report_desc_size;
	msgs_desc[1].buf = report_desc;
		
	ret = i2c_transfer(client->adapter, msgs_desc, ARRAY_SIZE(msgs_desc));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining report descriptor failed\n", __func__);
		goto errReportDesc;
	}
	if (ret != ARRAY_SIZE(msgs_desc)) {
		ret = -EIO;
		goto errReportDesc;
	}

#ifdef SHOW_REPORTDESC
	for (i = 0; i < report_desc_size; i ++) {
		if( i%8 == 0)
			printk( "\n RP%d:0x ", i );
		printk("%02x ",  report_desc[i]);
	}
#endif

	parse_report_desc(features, report_desc, report_desc_size);
	ret = 0;

	dev_dbg(&client->dev, "addr: %x x_max:%d, y_max:%d\n", client->addr, 
	       features->x_max, features->y_max);
	dev_dbg(&client->dev, "addr: %x x_touch:%d, y_touch:%d\n", client->addr, 
	       features->x_touch, features->y_touch);
	dev_dbg(&client->dev, "pressure_max:%d, fw_version:%x\n",
	       features->pressure_max, features->fw_version);
	
errReportDesc:
	kfree(report_desc);
	report_desc = NULL;
	return ret;
}

static int wacom_query_device(struct i2c_client *client, struct wacom_features *features)
{
	int ret = -1;
	u8 cmd_devDesc[] = {HID_DESC_REGISTER, 0x00};
	HID_DESC hid_descriptor = {0};		
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd_devDesc),
			.buf = cmd_devDesc,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(HID_DESC),
			.buf = (u8 *)(&hid_descriptor),
		},
	};
	
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining query failed: %d\n", __func__, ret);
		goto errQueryDevice;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s input/output error occured;\n returned: %dbyte(s)\n", __func__, ret);
		ret = -EIO;
		goto errQueryDevice;
	}

	features->input_size = hid_descriptor.wMaxInputLength;
	features->vendorId = hid_descriptor.wVendorID;
	features->productId = hid_descriptor.wProductID;
	
	hid_descriptor.wVersion = get_touch_query(client);
	
	features->fw_version = hid_descriptor.wVersion;
	memcpy(&features->hid_desc, &hid_descriptor, sizeof(HID_DESC));	

	printk(
		"dev_VID:0x%x, dev_PID:0x%x dev_Version:0x%x\n",
		hid_descriptor.wVendorID, hid_descriptor.wProductID, hid_descriptor.wVersion );

	ret = wacom_get_report_descriptor(client, features, hid_descriptor);
	if (ret < 0)
		goto errQueryDevice;

	ret = 0;
errQueryDevice:
	return ret;
}

void wacom_touch_cancel(void)
{
	struct input_dev *input = wac_i2c->input_touch;

	if( touch_action == 0 )
	{
		if(input_end_send_flag == 1)
		{
			input_end_send_flag = 0;
			input_report_key(input, BTN_TOUCH, 0);
			input_mt_sync(input);
			input_sync(input);
		}
		touch_action = 1;
	}
}

static void set_touch_coord(struct wacom_i2c *wac_i2c, u8 *data) {

	struct input_dev *input = wac_i2c->input_touch;
	int *finger_num = &wac_i2c->finger_num;
	bool *mt_rdy = &wac_i2c->rdy;
	int i, tsw = 0;
	int x, y, id = 0;
#ifdef INPUT_WITH_WIDTH
	int cw, ch;	// contact width and height
#endif
	int touch_num = 0;

	if(wac_i2c->digi_gpio > 0)
	{
		if(gpio_get_value(wac_i2c->digi_gpio) == 0)
		{
		}
		else
		{
			if(touch_action)
				touch_action = 0;
		}
	}

	/*When data[4] has a value, then it is the first finger packet.*/
	if (data[4] != 0x00) {
		*finger_num = data[4]-1;
		*mt_rdy = false;
	}

	/*One packet holds the status for two fingers.*/
	for (i = 0; i < FINGERNUM_IN_PACKET; i++) {
		tsw = data[IDX_MT_TSW + (i * TOUCH_DATA_OFFSET)] & 0x01;
		id = le16_to_cpup((__le16 *)&data[IDX_MT_ID + (i * TOUCH_DATA_OFFSET)]);
		x = le16_to_cpup((__le16 *)&data[IDX_MT_X + (i * TOUCH_DATA_OFFSET)]);
		y = le16_to_cpup((__le16 *)&data[IDX_MT_Y + (i * TOUCH_DATA_OFFSET)]);
#ifdef INPUT_WITH_WIDTH
		cw = le16_to_cpup((__le16 *)&data[IDX_MT_WIDTH + (i * TOUCH_DATA_OFFSET)]);
		ch = le16_to_cpup((__le16 *)&data[IDX_MT_HEIGHT + (i * TOUCH_DATA_OFFSET)]);
#endif

		if (tsw) {
			if(touch_action == 0)
			{
				input_report_abs(input, ABS_MT_PRESSURE, tsw * 0xff);
				input_report_abs(input, ABS_MT_TRACKING_ID, id);
#ifdef SWAP_X
				x = wac_i2c->features->x_touch - x;
#endif
#ifdef SWAP
				input_report_abs(input, ABS_MT_POSITION_X, y);
				input_report_abs(input, ABS_MT_POSITION_Y, x);
#else
				input_report_abs(input, ABS_MT_POSITION_X, x);
				input_report_abs(input, ABS_MT_POSITION_Y, y);
#endif
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, tsw * 0xff);
				input_report_abs(input, ABS_MT_TOUCH_MINOR, tsw * 0xf0);
#ifdef INPUT_WITH_WIDTH
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, cw);// or ABS_MT_WIDTH_MAJOR
				input_report_abs(input, ABS_MT_TOUCH_MINOR, ch);
#endif
				input_report_key(input, BTN_TOUCH, 1);
				input_mt_sync(input);
			}
			touch_num++;
//			printk("set_touch_coord %d:%d,%d\n",id,x,y);
		}

		if (*finger_num != 0) {
			(*finger_num)--;
		}
		else {
			if (!(*mt_rdy))
				*mt_rdy = true;
			else
				*mt_rdy = false;
			break;
		}
	}

	if(touch_num == 0) {
		input_report_key(input, BTN_TOUCH, 0);
		input_mt_sync(input);
		input_sync(input);
		input_end_send_flag = 0;
	}
	else {
		if (*mt_rdy && touch_action == 0) {
			input_sync(input);
			input_end_send_flag = 1;
		}
	}

	return;
}

static void set_touch_key(struct wacom_i2c *wac_i2c, u8 *data)
{
	struct input_dev *input = wac_i2c->input_touch;

	
	if(wac_i2c->key_on == 0)
	{
		if(data[3] & 0x01)
		{
			input_event(input, EV_KEY, KEY_HOMEPAGE, 1);
			input_sync(input);
			wac_i2c->key_on = KEY_HOMEPAGE;
			input_end_key_send_flag = 1;
		}
	}
	else
	{
		if((data[3] & 0x01) == 0)
		{
			input_event(input, EV_KEY, KEY_HOMEPAGE, 0);
			input_sync(input);
			wac_i2c->key_on = 0;
			input_end_key_send_flag = 0;
		}
	}
	return;
}

static void set_aes_coord(struct wacom_i2c *wac_i2c, u8 *data)
{
	struct input_dev *input = wac_i2c->input_aes;
	unsigned int x, y, pressure;
	unsigned char tsw, f1, f2, ers;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);

	if (!wac_i2c->rdy)
		wac_i2c->tool = (data[3] & 0x0c) ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;	

	wac_i2c->rdy = data[3] & 0x20;
	
	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
#ifdef SWAP_X
	x = wac_i2c->features->x_max - x;
#endif
#ifdef SWAP_Y
	y = wac_i2c->features->y_max - y;
#endif
#ifdef SWAP
	input_report_abs(input, ABS_X, y);
	input_report_abs(input, ABS_Y, x);
#else
	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);
#endif
	
//	printk("set_aes_coord %d,%d\n",x,y);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_key(input, wac_i2c->tool, wac_i2c->rdy);
	input_sync(input);

	return;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;
	u8 *data = wac_i2c->data;
	int error;
//	int i;

	mutex_lock(&wac_i2c->lock);
	if( tp_behavior_enable == 0 )
	{
		mutex_unlock(&wac_i2c->lock);
		return IRQ_HANDLED;
	}

	if( touch_suspend == 1 )
	{
		mutex_unlock(&wac_i2c->lock);
		return IRQ_HANDLED;
	}

	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, WACOM_TOUCH_INPUTSIZE);
	if (error < 0)
		goto out;

#if 0
	printk("wacom_i2c_irq:");
	for(i = 0; i < WACOM_TOUCH_INPUTSIZE;i++)
		printk("%02x ",data[i]);
	printk("----\n");
#endif

	if (data[2] == 0x06)
		set_aes_coord(wac_i2c, data);
	else if (data[2] == 0x1c)
		set_touch_coord(wac_i2c, data);
	else if (data[2] == 0x31)
	{
		set_touch_key(wac_i2c, data);
	}

out:
	mutex_unlock(&wac_i2c->lock);
	return IRQ_HANDLED;
}

#if RESET
#define RESET_CMD_SIZE	4
static int wacom_i2c_reset(struct i2c_client *client)
{
	int ret = -1;
	char cmd_Reset[RESET_CMD_SIZE] = {0x04, 0x00, 0x00, 0x01};
	char buf[2] = {0xff, 0xff};

	printk("%s \n", __func__);

	ret = i2c_master_send(client, cmd_Reset, RESET_CMD_SIZE);
	if (ret != RESET_CMD_SIZE) {
		printk("%s: Sending reset command failed \n", __func__);
		goto errReset;
	}

	msleep(100);

	/*Confirm zero'd 2 byte data is recieved for HID over I2C spec*/
	ret = i2c_master_recv(client, buf, 2);
	if (ret != 2 || (buf[0] & 0xff) || (buf[1] & 0xff)) {
		printk("%s: Receving data failed %d %02x %02x\n", __func__,ret,buf[0],buf[1]);
		goto errReset;
	}

	ret = 0;
errReset:
	return ret;
}
#endif


static void wacom_i2c_enable_irq(struct i2c_client *client, bool enable)
{
#if RESET
	int ret = -1;
#endif

	if (enable && !irq_enabled) {
#if RESET
		ret = wacom_i2c_reset(client);
		if (ret < 0)
			dev_err(&client->dev, "%s SET_RESET failed\n", __func__);
#endif

		printk("%s enabled irq \n ", __func__);
		enable_irq(client->irq);

		irq_enabled = true;
	} else if (!enable && irq_enabled) {
		printk("%s disabled irq\n ", __func__);
//		disable_irq(client->irq);
		disable_irq_nosync(client->irq);
		irq_enabled = false;
	}

	return;
}

static void wacom_set_inputevent (struct input_dev *input, struct i2c_client *client,
				  struct wacom_features *features, int device)
{

	input->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	switch (device) {	
	case IEVENT_AES:
		__set_bit(BTN_TOOL_RUBBER, input->keybit);
		__set_bit(BTN_STYLUS, input->keybit);
		__set_bit(BTN_STYLUS2, input->keybit);
		__set_bit(BTN_TOUCH, input->keybit);
		__set_bit(BTN_TOOL_PEN, input->keybit);
#ifdef  SWAP
		input_set_abs_params(input, ABS_X, 0, features->y_max, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, features->x_max, 0, 0);
#else
		input_set_abs_params(input, ABS_X, 0, features->x_max, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, features->y_max, 0, 0);
#endif
		input_set_abs_params(input, ABS_PRESSURE, 0, features->pressure_max, 0, 0);
		break;

	case IEVENT_TOUCH:
		__set_bit(KEY_VOLUMEUP, input->keybit);
		__set_bit(KEY_VOLUMEDOWN, input->keybit);
		__set_bit(KEY_POWER, input->keybit);
		__set_bit(KEY_HOME, input->keybit);
		__set_bit(KEY_HOMEPAGE, input->keybit);
		set_bit(BTN_TOUCH, input->keybit);
		set_bit(ABS_MT_TRACKING_ID, input->absbit);
		set_bit(ABS_MT_TOUCH_MAJOR,input->absbit);
		set_bit(ABS_MT_TOUCH_MINOR, input->absbit);
		set_bit(ABS_MT_POSITION_X, input->absbit);
		set_bit(ABS_MT_POSITION_Y, input->absbit);
#ifdef  SWAP
		input_set_abs_params(input, ABS_X, 0, features->y_touch, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, features->x_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, features->y_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, features->x_touch, 0, 0);
#else
		input_set_abs_params(input, ABS_X, 0, features->x_touch, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, features->y_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, features->x_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, features->y_touch, 0, 0);
#endif
		input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

		input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
		input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

		break;

	default: 
		return;
	}
}

static void wacom_i2c_reset_gpio(int reset_gpio)
{
	gpio_request(reset_gpio, "wacom_reset");
	gpio_direction_output(reset_gpio, 0);
	msleep(2);
	gpio_direction_output(reset_gpio, 1);
	msleep(100);
	gpio_free(reset_gpio);
}

static void tp_behavior_setting(int value)
{
    int res;
	struct input_dev *input = wac_i2c->input_touch;
	
	mutex_lock(&wac_i2c->lock);
	
	if ( tp_behavior_enable != value ) {
		tp_behavior_enable = value;
		if ( value ) {
			res = regulator_enable(tpd->reg);
			if(res != 0) {
				printk(KERN_ERR "regulator_enable error res=%d \n", res);
			}
			
			wacom_i2c_reset_gpio(wac_i2c->reset_gpio);
			
			/*Enable IRQ*/
			wacom_i2c_enable_irq(wac_i2c->client, true);
		} else {
			/* Disable the IRQ*/
			wacom_i2c_enable_irq(wac_i2c->client, false);
			
			gpio_request(wac_i2c->reset_gpio, "wacom_reset");
			gpio_direction_output(wac_i2c->reset_gpio, 0);
			msleep(2);
			gpio_free(wac_i2c->reset_gpio);
			
			res = regulator_disable(tpd->reg);
			if(res != 0) {
				printk(KERN_ERR "regulator_disable error res=%d \n", res);
			}
		}

		if( input_end_send_flag != 0 )
		{
			input_report_key(input, BTN_TOUCH, 0);
			input_mt_sync(input);
			input_sync(input);
			input_end_send_flag = 0;
		}

		if( input_end_key_send_flag != 0 )
		{
			if( wac_i2c->key_on != 0 )
			{
				input_event(input, EV_KEY, wac_i2c->key_on, 0);
				input_sync(input);
				wac_i2c->key_on = 0;
				input_end_key_send_flag = 0;
			}
		}
	}

	mutex_unlock(&wac_i2c->lock);
}

// Touch Paner FW Version
ssize_t tp_fwver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct wacom_features features;
	if (wacom_query_device(wac_i2c->client, &features) >= 0) {
		return sprintf(buf, "%04X\n", features.fw_version);
	}

	return sprintf(buf, "%04X\n", wac_i2c->features->fw_version);
}
DEVICE_ATTR(tp_fwver, 0444, tp_fwver_show, NULL);


static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
//	struct wacom_i2c *wac_i2c;
	struct input_dev *input_aes,*input_touch;
	struct wacom_features features;
	int error = -1;
	int gpio = -1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	gpio = of_get_named_gpio(client->dev.of_node, "rst-gpio", 0);
	if (gpio < 0) {
		dev_err(&client->dev,
			"of_get_named_gpio(\"%s\") failed. (%d)\n", "rst-gpio", gpio);
		return -1;
	}

	wacom_i2c_reset_gpio(gpio);

	/*Check, first, whether or not there's a device corresponding to the address*/
	error = wacom_query_device(client, &features);
	if (error < 0) {
		printk("wacom_query_device failed \n");
		goto err_exit;
	}

	printk("wacom_i2c probe started(addr: %x) \n", client->addr);
	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input_aes = input_allocate_device();
	input_touch = input_allocate_device();
	if (!wac_i2c || !input_aes || !input_touch) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->features = kzalloc(sizeof(struct wacom_features), GFP_KERNEL);
	if (!wac_i2c->features) {
		printk("failed to preserve the memory");
		goto err_free_mem;
	}
	memcpy(wac_i2c->features, &features, sizeof(struct wacom_features));

	wac_i2c->client = client;
	wac_i2c->input_aes = input_aes;
	wac_i2c->input_touch = input_touch;

	mutex_init(&wac_i2c->lock);

	wac_i2c->reset_gpio = gpio;
	wac_i2c->int_gpio = of_get_named_gpio(client->dev.of_node, "int-gpio", 0);
	if (wac_i2c->int_gpio < 0) {
		dev_err(&client->dev,
			"of_get_named_gpio(\"%s\") failed. (%d)\n", "int-gpio", wac_i2c->int_gpio);
		return -1;
	}
	gpio_request(wac_i2c->int_gpio, NULL);
	gpio_direction_input(wac_i2c->int_gpio);

	wac_i2c->digi_gpio = of_get_named_gpio(client->dev.of_node, "digi-gpio", 0);
	if (wac_i2c->digi_gpio < 0) {
		dev_err(&client->dev,
			"of_get_named_gpio(\"%s\") failed. (%d)\n", "digi-gpio", wac_i2c->digi_gpio);
		return -1;
	}
	gpio_request(wac_i2c->digi_gpio, NULL);
	gpio_direction_input(wac_i2c->digi_gpio);

	wac_i2c->key_on = 0;

	input_aes->name = "Wacom I2C AES";
	input_aes->id.bustype = BUS_I2C;
	input_aes->id.vendor = wac_i2c->features->vendorId;
	input_aes->id.product = wac_i2c->features->productId;
	input_aes->id.version = wac_i2c->features->fw_version;
	input_aes->dev.parent = &client->dev;
	wacom_set_inputevent(input_aes, client, wac_i2c->features, IEVENT_AES);
	input_set_drvdata(input_aes, wac_i2c);

	input_touch->name = "Wacom_I2C_TOUCH";
	input_touch->id.bustype = BUS_I2C;
	input_touch->id.vendor = wac_i2c->features->vendorId;
	input_touch->id.product = wac_i2c->features->productId;
	input_touch->id.version = wac_i2c->features->fw_version;
	input_touch->dev.parent = &client->dev;
	wacom_set_inputevent(input_touch, client, wac_i2c->features, IEVENT_TOUCH);
	input_set_drvdata(input_touch, wac_i2c);

	error = request_threaded_irq(client->irq, NULL, wacom_i2c_irq,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     "wacom_aes_irq", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_features;
	}

	/* Disable the IRQ*/
	wacom_i2c_enable_irq(client, false);

	error = input_register_device(wac_i2c->input_aes);
	if (error) {
		dev_err(&client->dev, 
			"Failed to register input device, error: %d\n", error);
		goto err_free_input_aes;
	}

	error = input_register_device(wac_i2c->input_touch);
	if (error) {
		dev_err(&client->dev, 
			"Failed to register input device, error: %d\n", error);
		goto err_free_input_touch;
	}

	i2c_set_clientdata(client, wac_i2c);

	/*Enable IRQ*/
	wacom_i2c_enable_irq(client, true);

	sema_init(&tp_semaphore, 1);

	device_create_file(&client->dev, &dev_attr_tp_fwver);

	tpd_load_status = 1;

	return 0;

 err_free_input_touch:
	input_free_device(input_touch);

 err_free_input_aes:
	input_free_device(input_aes);
	free_irq(client->irq, wac_i2c);

 err_free_features:
	kfree(wac_i2c->features);
	wac_i2c->features = NULL;

 err_free_mem:
	kfree(wac_i2c);
	wac_i2c = NULL;

 err_exit:
	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s \n", __func__);
	free_irq(client->irq, wac_i2c);
	input_unregister_device(wac_i2c->input_touch);
	kfree(wac_i2c->features);
	mutex_destroy(&wac_i2c->lock);
	kfree(wac_i2c);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wacom_i2c_suspend(struct device *dev)
{
	dev_dbg(&wac_i2c->client->dev, "%s \n", __func__);
	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	dev_dbg(&wac_i2c->client->dev, "%s \n", __func__);
	return 0;
}
static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);
#endif

#ifdef CONFIG_OF
static struct of_device_id wacom_i2c_of_idtable[] = {
	{ .compatible = "mediatek,cap_touch", },
	{}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_TOUCH", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id wacom_i2c_acpi_match[] = {
	{ WACOM_HARDWARE_ID, 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, wacom_i2c_acpi_match);
#endif

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
//#ifndef MTK_INPUT
		.pm	= &wacom_i2c_pm,
//#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(wacom_i2c_acpi_match),
#endif
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(wacom_i2c_of_idtable),
#endif /* CONFIG_OF */
	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};

#ifndef MTK_INPUT
static int __init wacom_i2c_init(void)
{
	printk("wacom i2c driver started\n");
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

module_init(wacom_i2c_init);
module_exit(wacom_i2c_exit);
#endif

static int tpd_local_init(void)
{
    int res;
    unsigned int volt;
    const unsigned int SET_VOLTAGE = 3300000;
	
	printk(KERN_ERR "%s called\n", __func__);
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	res = regulator_set_voltage(tpd->reg, SET_VOLTAGE, SET_VOLTAGE);
	if(res != 0) {
		printk(KERN_ERR "regulator_set_voltage error\n");
		return res;
	}
	
	volt = regulator_get_voltage(tpd->reg);
	if(volt != SET_VOLTAGE) {
		printk(KERN_ERR "regulator_get_voltage error[%d]\n", volt);
		return res;
	}
	
	res = regulator_enable(tpd->reg);
	if(res != 0) {
		printk(KERN_ERR "regulator_enable error\n");
		return res;
	}

	if( (res = i2c_add_driver(&wacom_i2c_driver)) < 0 ) {
		pr_err("i2c_add_driver error\n");
		return res;
	}

	return res;
}


static void tpd_suspend(struct device *dev)
{
	if(tpd_load_status == 1)
	{
		tp_behavior_setting(0);
		touch_suspend = 1;
	}
	return;
}

static void tpd_resume(struct device *dev)
{
	if(tpd_load_status == 1)
	{
		tp_behavior_setting(1);
		touch_suspend = 0;
	}
	return;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "generic",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int __init tpd_driver_init(void)
{
	printk(KERN_INFO "%s called\n", __func__);
	tpd_get_dts_info();
	if(tpd_driver_add(&tpd_device_driver) < 0) {
		printk(KERN_ERR "tpd_driver_add error\n");
	}
	return 0;
}

static void __exit tpd_driver_exit(void)
{
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM AES I2C Device Driver");
MODULE_LICENSE("GPL");
