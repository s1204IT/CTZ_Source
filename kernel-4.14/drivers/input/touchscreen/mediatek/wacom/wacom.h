/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2014 Tatsunosuke Tobita, Wacom.
 * <tobita.tatsunosuke@wacom.co.jp>
 * Copyright (c) 2011-2019 Martin Chen, Wacom.
 *		<martin.chen@wacom.com>, modify for G12T plus
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */

#ifndef WACOM_I2C_H
#define WACOM_I2C_H

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <asm/unaligned.h>
#include <linux/input-event-codes.h>
#include <linux/hrtimer.h>

/*For multi-touch operation**/
#define IDX_MT_KEY			3
#define IDX_MT_TSW			5
#define IDX_MT_ID			6
#define INPUT_WITH_WIDTH	//without width format (MS hid)
#ifdef INPUT_WITH_WIDTH		//with width format
//	#define WACOM_TOUCH_INPUTSIZE   (42 + 20)
	#define WACOM_TOUCH_INPUTSIZE   (29)
	#define TOUCH_DATA_OFFSET       (7 + 4) //with width format
	#define IDX_MT_WIDTH		8
	#define IDX_MT_HEIGHT		10
	#define IDX_MT_X			12
	#define IDX_MT_Y			14
#else                      //without width format (MS hid)
	#define WACOM_TOUCH_INPUTSIZE   (42)
	#define TOUCH_DATA_OFFSET       (7) 
	#define IDX_MT_X			8
	#define IDX_MT_Y			10
#endif
//
#ifdef IDX_MT_KEY
	// I assume this is what the key defined, change them here
	#define MY_KEY0			KEY_VOLUMEDOWN
	#define MY_KEY1			KEY_VOLUMEUP
	#define MY_KEY2			KEY_MUTE
	#define MY_KEY3			KEY_PAUSE
#endif
//
#define WACOM_AES_INPUTSIZE     18

enum {
	IEVENT_AES = 0,
	IEVENT_TOUCH
};


#define FINGERNUM_IN_PACKET     2

/*HID over I2C spec*/
#define HID_DESC_REGISTER       0x01
#define USAGE_PAGE              0x05
#define USAGE_PAGE_DIGITIZERS   0x0d
#define USAGE_PAGE_DESKTOP      0x01
#define USAGE                   0x09
#define USAGE_PEN               0x02
#define USAGE_MOUSE             0x02
#define USAGE_FINGER            0x22
#define USAGE_STYLUS            0x20
#define USAGE_TOUCHSCREEN       0x04
#define USAGE_X                 0x30
#define USAGE_TIPPRESSURE       0x30
#define USAGE_Y                 0x31

typedef struct hid_descriptors {
	u16 wHIDDescLength;
	u16 bcdVersion;
	u16 wReportDescLength;
	u16 wReportDescRegister;
	u16 wInputRegister;
	u16 wMaxInputLength;
	u16 wOutputRegister;
	u16 wMaxOutputLength;
	u16 wCommandRegister;
	u16 wDataRegister;
	u16 wVendorID;
	u16 wProductID;
	u16 wVersion;
	u16 RESERVED_HIGH;
	u16 RESERVED_LOW;
} HID_DESC;

struct wacom_features {
	HID_DESC hid_desc;
	unsigned int input_size;
	int x_max;
	int y_max;
	int x_touch;
	int y_touch;
	int pressure_max;
	int fw_version;
	int vendorId;
	int productId;
};

struct wacom_i2c {
	struct i2c_client *client;
	struct input_dev *input_aes;
	struct input_dev *input_touch;
	struct wacom_features *features;
	struct device *dev;
	struct class *class;
	struct hrtimer timer;
	struct hrtimer checktimer;
	struct work_struct	work;
	struct work_struct	work_calib;
	struct mutex lock;

	int finger_num;
	int tool;
	u8 data[WACOM_TOUCH_INPUTSIZE];
	u8 cmd;
	bool rdy;

	int reset_gpio;
	int int_gpio;
	int key_on;
	int digi_gpio;
};
static int wacom_query_device(struct i2c_client *client, struct wacom_features *features);
#endif
