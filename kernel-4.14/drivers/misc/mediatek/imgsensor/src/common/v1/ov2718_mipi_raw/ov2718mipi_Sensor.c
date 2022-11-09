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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     ov2718mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov2718mipi_Sensor.h"

/***********************Modify Following Strings for Debug********************/
#define PFX "OV2718_camera_sensor"
#define LOG_1 cam_pr_debug("OV2718,MIPI 2LANE\n")
#define LOG_2 cam_pr_debug(\
	"prv 1280*960@30fps,420Mbps/lane; cap 5M@15fps,420Mbps/lane\n")
/***************************   Modify end    *********************************/

#define cam_pr_debug(fmt, args...) \
	pr_debug(PFX "[%s] " fmt, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {

	.sensor_id = OV2718_SENSOR_ID,
	.checksum_value = 0x19546ed1,
	.pre = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 94250000,
		.linelength = 2094,
		.framelength = 750,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 600,
	},
	.slim_video = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 74750000,
		.linelength = 2212,
		.framelength = 1133,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 30,
		.max_framerate = 300,
	},
	.margin = 4,        /* sensor framelength & shutter margin */
	.min_shutter = 1,	/*min shutter*/
	.max_frame_length = 0x7fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,    /* isp gain delay frame for AE cycle */
	.ihdr_support = 0,  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0, /* 1,le first ; 0, se first */
	.sensor_mode_num = 6,/*support sensor mode num*/

	.cap_delay_frame = 8,
	.pre_delay_frame = 2,   /* enter preview delay frame num */
	.video_delay_frame = 2, /* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,    /* enter slim video delay frame num */
	.custom1_delay_frame = 8,
	.isp_driving_current = ISP_DRIVING_2MA, /* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 0,/*0,AUTO; 1,MANNUAL*/
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW12_B,
	.mclk = 26,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_2_LANE,    /* mipi lane num */
	.i2c_addr_table = {0x6c, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6c,	/* record current sensor's i2c write id */
	.hdr_mode = 0
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
{1920, 1080, 0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080},
{1920, 1080, 0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080},
{1920, 1080, 0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080},
{1280, 720,  0, 0, 1280, 720,  1280, 720,  0, 0, 1280, 720,  0, 0, 1280,  720},
{1920, 1080, 0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080},
{1920, 1080, 0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080}
};

/* add from OV2718_v1_mirror_on_flip_off_mtk2.0.1.ini */

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1,
		imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8),
				(char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	write_cmos_sensor(0x30b2, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x30b3, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x30b0, imgsensor.line_length >> 8);
	write_cmos_sensor(0x30b1, imgsensor.line_length & 0xFF);
	cam_pr_debug("dummyline = %d\n", imgsensor.dummy_line);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line; */
	kal_uint32 frame_length = imgsensor.frame_length;

	cam_pr_debug("framerate = %d, min framelength should enable = %d\n",
		framerate,
		min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	/* cam_pr_debug("frame_length =%d\n", frame_length); */
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	    (frame_length > imgsensor.min_frame_length) ? frame_length :
	    imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}



/*************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter =
	    (shutter >
	     (imgsensor_info.max_frame_length -
	      imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
					 imgsensor_info.margin) : shutter;

	/* frame_length and shutter should be an even number. */

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x30b2,
					imgsensor.frame_length >> 8);
			write_cmos_sensor(0x30b3,
					imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x30b2, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x30b3, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter */
	write_cmos_sensor(0x30b6, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x30b7,  shutter & 0xFF);
	cam_pr_debug("shutter =%d\n", shutter);
}


static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = ((gain / BASEGAIN) << 4) +
		((gain % BASEGAIN) * 16 / BASEGAIN);
	reg_gain = reg_gain & 0xFFFF;

	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	if (imgsensor.hdr_mode == 2) {
		kal_uint16 reg_gain = 0x0000;
		kal_uint16 gain_a;
		kal_uint16 hcg_d_h;
		kal_uint16 hcg_d_l;
		kal_uint16 lcg_d_h;
		kal_uint16 lcg_d_l;

		if (gain < 2 * BASEGAIN)
			gain = 2 * BASEGAIN;
		else if (gain > 96 * BASEGAIN)
			gain = 96 * BASEGAIN;
#if 1
		if (gain < 3 * 64) {
			gain_a = 0x05;	/*analog 2x*/
			lcg_d_h = (3 * 64 * 2) >> 8;/*((gain/64/2)*256)>>8*/
			lcg_d_l = (3 * 64 * 2) & 0xff;
			hcg_d_h = (gain * 2) >> 8;/*((gain/64/2)*256)>>8*/
			hcg_d_l = (gain * 2) & 0xff;
		} else if (gain >= 3 * 64 && gain < (int)(4.375 * 64)) {
			gain_a = 0x05;	/*analog 2x*/
			lcg_d_h = hcg_d_h = (gain * 2) >> 8;
			lcg_d_l = hcg_d_l = (gain * 2) & 0xff;
		} else if (gain >= (int)(4.375 * 64) && gain <
			(int)(8.75 * 64)) {
			gain_a = 0x0a;	/*analog 4x*/
			lcg_d_h = hcg_d_h = gain >> 8;/*((gain/64/4)*256)>>8*/
			lcg_d_l = hcg_d_l = gain & 0xff;
		} else {
			gain_a = 0x0f;	/*analog 8x*/
			lcg_d_h = hcg_d_h = gain >> 9;/*((gain/64/8)*256)>>8*/
			lcg_d_l = hcg_d_l = (gain >> 1) & 0xff;
		}
#endif
		/*reg_gain = gain2reg(gain); */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain;
		spin_unlock(&imgsensor_drv_lock);
		cam_pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

		write_cmos_sensor(0x30bb, gain_a);
		write_cmos_sensor(0x315A, hcg_d_h);
		write_cmos_sensor(0x315B, hcg_d_l);
		write_cmos_sensor(0x315c, lcg_d_h);
		write_cmos_sensor(0x315d, lcg_d_l);
		cam_pr_debug("ov2718r2a hdr gain register,0x30bb=0x%x,0x315A=0x%x,0x315B=0x%x,0x315c=0x%x,0x315d=0x%x\n",
			 read_cmos_sensor(0x30bb), read_cmos_sensor(0x315A),
			 read_cmos_sensor(0x315B),
			 read_cmos_sensor(0x315C), read_cmos_sensor(0x315D));
	} else {
		kal_uint16 reg_gain;
		kal_uint16 gain_a;
		kal_uint16 gain_d_h;
		kal_uint16 gain_d_l;

		if (gain < 3 * BASEGAIN || gain > 176 * BASEGAIN) {
			cam_pr_debug("Error gain setting");
			if (gain < 3 * BASEGAIN)
				gain = 3 * BASEGAIN;
			else if (gain > 176 * BASEGAIN)
				gain = 176 * BASEGAIN;
		}
#if 1
		if (gain >= 3 * 64 && gain < (int)(4.375 * 64)) {
			gain_a = 0x01;	/*LCG  2*A gain*/
			gain_d_h = (gain * 2) >> 8;	/*((gain/128)*256)>>8*/
			gain_d_l = (gain * 2) & 0xff;
		} else if (gain >= (int)(4.375 * 64) && gain <
				(int)(8.750 * 64)) {
			gain_a = 0x02;	/*LCG  4*A gain*/
			gain_d_h = gain >> 8;
			gain_d_l = gain & 0xff;
		} else if (gain >= (int)(8.750 * 64) && gain < 22 * 64) {
			gain_a = 0x03;	/*LCG  8*A gain*/
			gain_d_h = gain >> 9;	/*((gain/512)*256)>>8*/
			gain_d_l = (gain >> 1) & 0xff;
		} else if (gain >= 22 * 64 && gain < 44 * 64) {
			gain = (int)(gain / 11);
			gain_a = 0x40;	/*HCG  1*A gain*/
			gain_d_h = (gain * 4) >> 8;
			gain_d_l = (gain * 4) & 0xff;
		} else if (gain >= 44 * 64 && gain < 88 * 64) {
			gain = (int)(gain / 11);
			gain_a = 0x41;/*HCG 2*A gain*/
			gain_d_h = (gain * 2) >> 8;
			gain_d_l = (gain * 2) & 0xff;
		} else if (gain >= 88 * 64 && gain <= 176 * 64) {
			gain = (int)(gain / 11);
			gain_a = 0x42;	/*HCG 4*A gain*/
			gain_d_h = gain >> 8;
			gain_d_l = gain & 0xff;
		}
#endif
		reg_gain = gain2reg(gain);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain;
		spin_unlock(&imgsensor_drv_lock);
		cam_pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
		write_cmos_sensor(0x30bb, gain_a);
		write_cmos_sensor(0x315A, gain_d_h);
		write_cmos_sensor(0x315B, gain_d_l);


		/* Extend frame length first */
		write_cmos_sensor(0x315C, gain_d_h);
		write_cmos_sensor(0x315D, gain_d_l);

		cam_pr_debug("ov2718r2a gain register,0x30bb=0x%x,0x315A=0x%x,0x315B=0x%x,0x315C=0x%x,0x315D=0x%x\n",
			 read_cmos_sensor(0x30bb), read_cmos_sensor(0x315A),
			 read_cmos_sensor(0x315B),
			 read_cmos_sensor(0x315C), read_cmos_sensor(0x315D));
	}
	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se,
		kal_uint16 gain)
{
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	cam_pr_debug("image_mirror = %d\n", image_mirror);

	/********************************************************
	 *
	 *   0x3820[2] ISP Vertical flip
	 *   0x3820[1] Sensor Vertical flip
	 *
	 *   0x3821[2] ISP Horizontal mirror
	 *   0x3821[1] Sensor Horizontal mirror
	 *
	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x30c0,
			((read_cmos_sensor(0x30c0) & 0xF3) | 0x00));
		write_cmos_sensor(0x30a9,
			((read_cmos_sensor(0x30a9) & 0xFE) | 0x00));
		write_cmos_sensor(0x3252,
			((read_cmos_sensor(0x3252) & 0xFC) | 0x00));
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x30c0,
			((read_cmos_sensor(0x30c0) & 0xF3) | 0x04));
		write_cmos_sensor(0x30a9,
			((read_cmos_sensor(0x30a9) & 0xFE) | 0x01));
		write_cmos_sensor(0x3252,
			((read_cmos_sensor(0x3252) & 0xFC) | 0x01));
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x30c0,
			((read_cmos_sensor(0x30c0) & 0xF3) | 0x08));
		write_cmos_sensor(0x30a9,
			((read_cmos_sensor(0x30a9) & 0xFE) | 0x00));
		write_cmos_sensor(0x3252,
			((read_cmos_sensor(0x3252) & 0xFC) | 0x02));
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x30c0,
			((read_cmos_sensor(0x30c0) & 0xF3) | 0x0C));
		write_cmos_sensor(0x30a9,
			((read_cmos_sensor(0x30a9) & 0xFE) | 0x01));
		write_cmos_sensor(0x3252,
			((read_cmos_sensor(0x3252) & 0xFC) | 0x03));
		break;
	default:
		cam_pr_debug("Error image_mirror setting\n");
	}
}
#endif

/*************************************************************************
 * FUNCTION
 *    night_mode
 *
 * DESCRIPTION
 *    This function night mode of sensor.
 *
 * PARAMETERS
 *    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}			/*    night_mode    */

static void sensor_hdr(void)
{
	cam_pr_debug("E\n");

	write_cmos_sensor(0x3013, 0x01);
	mDELAY(10);

	write_cmos_sensor(0x3000, 0x02);
	write_cmos_sensor(0x3001, 0x28);
	write_cmos_sensor(0x3002, 0x03);
	write_cmos_sensor(0x3003, 0x01);
	write_cmos_sensor(0x3004, 0x02);
	write_cmos_sensor(0x3005, 0x26);
	write_cmos_sensor(0x3006, 0x00);
	write_cmos_sensor(0x3007, 0x07);
	write_cmos_sensor(0x3008, 0x01);
	write_cmos_sensor(0x3009, 0x00);
	write_cmos_sensor(0x300c, 0x6c);
	write_cmos_sensor(0x300e, 0x80);
	write_cmos_sensor(0x300f, 0x00);
	write_cmos_sensor(0x3012, 0x00);//stream off
	write_cmos_sensor(0x3013, 0x00);
	write_cmos_sensor(0x3014, 0xc4);
	write_cmos_sensor(0x3015, 0x00);
	write_cmos_sensor(0x3017, 0x00);
	write_cmos_sensor(0x3018, 0x00);
	write_cmos_sensor(0x3019, 0x00);
	write_cmos_sensor(0x301a, 0x00);
	write_cmos_sensor(0x301b, 0x01);
	write_cmos_sensor(0x301e, 0x17);
	write_cmos_sensor(0x301f, 0xe1);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x3031, 0x72);
	write_cmos_sensor(0x3032, 0xf0);
	write_cmos_sensor(0x3033, 0x30);
	write_cmos_sensor(0x3034, 0x3f);
	write_cmos_sensor(0x3035, 0x5f);
	write_cmos_sensor(0x3036, 0x02);
	write_cmos_sensor(0x3037, 0x9f);
	write_cmos_sensor(0x3038, 0x04);
	write_cmos_sensor(0x3039, 0xb7);
	write_cmos_sensor(0x303a, 0x04);
	write_cmos_sensor(0x303b, 0x07);
	write_cmos_sensor(0x303c, 0xf0);
	write_cmos_sensor(0x303d, 0x00);
	write_cmos_sensor(0x303e, 0x0b);
	write_cmos_sensor(0x303f, 0xe3);
	write_cmos_sensor(0x3040, 0xf3);
	write_cmos_sensor(0x3041, 0x29);
	write_cmos_sensor(0x3042, 0xf6);
	write_cmos_sensor(0x3043, 0x65);
	write_cmos_sensor(0x3044, 0x06);
	write_cmos_sensor(0x3045, 0x0f);
	write_cmos_sensor(0x3046, 0x59);
	write_cmos_sensor(0x3047, 0x07);
	write_cmos_sensor(0x3048, 0x82);
	write_cmos_sensor(0x3049, 0xcf);
	write_cmos_sensor(0x304a, 0x12);
	write_cmos_sensor(0x304b, 0x40);
	write_cmos_sensor(0x304c, 0x33);
	write_cmos_sensor(0x304d, 0xa4);
	write_cmos_sensor(0x304e, 0x0b);
	write_cmos_sensor(0x304f, 0x3d);
	write_cmos_sensor(0x3050, 0x10);
	write_cmos_sensor(0x3060, 0x00);
	write_cmos_sensor(0x3061, 0x64);
	write_cmos_sensor(0x3062, 0x00);
	write_cmos_sensor(0x3063, 0xe4);
	write_cmos_sensor(0x3064, 0x0b);
	write_cmos_sensor(0x3065, 0x60);
	write_cmos_sensor(0x3066, 0x80);
	write_cmos_sensor(0x3080, 0x00);
	write_cmos_sensor(0x3081, 0x00);
	write_cmos_sensor(0x3082, 0x01);
	write_cmos_sensor(0x3083, 0xe3);
	write_cmos_sensor(0x3084, 0x06);
	write_cmos_sensor(0x3085, 0x00);
	write_cmos_sensor(0x3086, 0x10);
	write_cmos_sensor(0x3087, 0x10);
	write_cmos_sensor(0x3089, 0x00);
	write_cmos_sensor(0x308a, 0x01);
	write_cmos_sensor(0x3093, 0x00);
	write_cmos_sensor(0x30a0, 0x00);
	write_cmos_sensor(0x30a1, 0x00);
	write_cmos_sensor(0x30a2, 0x00);
	write_cmos_sensor(0x30a3, 0x00);
	write_cmos_sensor(0x30a4, 0x07);
	write_cmos_sensor(0x30a5, 0x8f);
	write_cmos_sensor(0x30a6, 0x04);
	write_cmos_sensor(0x30a7, 0x47);
	write_cmos_sensor(0x30a8, 0x00);
	write_cmos_sensor(0x30a9, 0x01);
	write_cmos_sensor(0x30aa, 0x00);
	write_cmos_sensor(0x30ab, 0x00);
	write_cmos_sensor(0x30ac, 0x07);
	write_cmos_sensor(0x30ad, 0x90);
	write_cmos_sensor(0x30ae, 0x04);
	write_cmos_sensor(0x30af, 0x48);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0xae);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x65);
	write_cmos_sensor(0x30b4, 0x00);
	write_cmos_sensor(0x30b5, 0x00);
	write_cmos_sensor(0x30b6, 0x00);
	write_cmos_sensor(0x30b7, 0x10);
	write_cmos_sensor(0x30b8, 0x00);
	write_cmos_sensor(0x30b9, 0x02);
	write_cmos_sensor(0x30ba, 0x10);
	write_cmos_sensor(0x30bb, 0x00);
	write_cmos_sensor(0x30bc, 0x00);
	write_cmos_sensor(0x30bd, 0x03);
	write_cmos_sensor(0x30be, 0x5c);
	write_cmos_sensor(0x30bf, 0x00);
	write_cmos_sensor(0x30c0, 0x04);
	write_cmos_sensor(0x30c1, 0x00);
	write_cmos_sensor(0x30c2, 0x20);
	write_cmos_sensor(0x30c3, 0x00);
	write_cmos_sensor(0x30c4, 0x4a);
	write_cmos_sensor(0x30c5, 0x00);
	write_cmos_sensor(0x30c7, 0x00);
	write_cmos_sensor(0x30c8, 0x00);
	write_cmos_sensor(0x30d1, 0x00);
	write_cmos_sensor(0x30d2, 0x00);
	write_cmos_sensor(0x30d3, 0x80);
	write_cmos_sensor(0x30d4, 0x00);
	write_cmos_sensor(0x30d9, 0x09);
	write_cmos_sensor(0x30da, 0x64);
	write_cmos_sensor(0x30dd, 0x00);
	write_cmos_sensor(0x30de, 0x16);
	write_cmos_sensor(0x30df, 0x00);
	write_cmos_sensor(0x30e0, 0x17);
	write_cmos_sensor(0x30e1, 0x00);
	write_cmos_sensor(0x30e2, 0x18);
	write_cmos_sensor(0x30e3, 0x10);
	write_cmos_sensor(0x30e4, 0x04);
	write_cmos_sensor(0x30e5, 0x00);
	write_cmos_sensor(0x30e6, 0x00);
	write_cmos_sensor(0x30e7, 0x00);
	write_cmos_sensor(0x30e8, 0x00);
	write_cmos_sensor(0x30e9, 0x00);
	write_cmos_sensor(0x30ea, 0x00);
	write_cmos_sensor(0x30eb, 0x00);
	write_cmos_sensor(0x30ec, 0x00);
	write_cmos_sensor(0x30ed, 0x00);
	write_cmos_sensor(0x3101, 0x00);
	write_cmos_sensor(0x3102, 0x00);
	write_cmos_sensor(0x3103, 0x00);
	write_cmos_sensor(0x3104, 0x00);
	write_cmos_sensor(0x3105, 0x8c);
	write_cmos_sensor(0x3106, 0x87);
	write_cmos_sensor(0x3107, 0xc0);
	write_cmos_sensor(0x3108, 0x9d);
	write_cmos_sensor(0x3109, 0x8d);
	write_cmos_sensor(0x310a, 0x8d);
	write_cmos_sensor(0x310b, 0x6a);
	write_cmos_sensor(0x310c, 0x3a);
	write_cmos_sensor(0x310d, 0x5a);
	write_cmos_sensor(0x310e, 0x00);
	write_cmos_sensor(0x3120, 0x00);
	write_cmos_sensor(0x3121, 0x00);
	write_cmos_sensor(0x3122, 0x00);
	write_cmos_sensor(0x3123, 0x00);
	write_cmos_sensor(0x3124, 0x00);
	write_cmos_sensor(0x3125, 0x70);
	write_cmos_sensor(0x3126, 0x1f);
	write_cmos_sensor(0x3127, 0x0f);
	write_cmos_sensor(0x3128, 0x00);
	write_cmos_sensor(0x3129, 0x3a);
	write_cmos_sensor(0x312a, 0x02);
	write_cmos_sensor(0x312b, 0x0f);
	write_cmos_sensor(0x312c, 0x00);
	write_cmos_sensor(0x312d, 0x0f);
	write_cmos_sensor(0x312e, 0x1d);
	write_cmos_sensor(0x312f, 0x00);
	write_cmos_sensor(0x3130, 0x00);
	write_cmos_sensor(0x3131, 0x00);
	write_cmos_sensor(0x3132, 0x00);
	write_cmos_sensor(0x3140, 0x0a);
	write_cmos_sensor(0x3141, 0x03);
	write_cmos_sensor(0x3142, 0x00);
	write_cmos_sensor(0x3143, 0x00);
	write_cmos_sensor(0x3144, 0x00);
	write_cmos_sensor(0x3145, 0x00);
	write_cmos_sensor(0x3146, 0x00);
	write_cmos_sensor(0x3147, 0x00);
	write_cmos_sensor(0x3148, 0x00);
	write_cmos_sensor(0x3149, 0x00);
	write_cmos_sensor(0x314a, 0x00);
	write_cmos_sensor(0x314b, 0x00);
	write_cmos_sensor(0x314c, 0x00);
	write_cmos_sensor(0x314d, 0x00);
	write_cmos_sensor(0x314e, 0x1c);
	write_cmos_sensor(0x314f, 0xff);
	write_cmos_sensor(0x3150, 0xff);
	write_cmos_sensor(0x3151, 0xff);
	write_cmos_sensor(0x3152, 0x10);
	write_cmos_sensor(0x3153, 0x10);
	write_cmos_sensor(0x3154, 0x10);
	write_cmos_sensor(0x3155, 0x00);
	write_cmos_sensor(0x3156, 0x03);
	write_cmos_sensor(0x3157, 0x00);
	write_cmos_sensor(0x3158, 0x0f);
	write_cmos_sensor(0x3159, 0xff);
	write_cmos_sensor(0x315a, 0x01);
	write_cmos_sensor(0x315b, 0x00);
	write_cmos_sensor(0x315c, 0x01);
	write_cmos_sensor(0x315d, 0x00);
	write_cmos_sensor(0x315e, 0x01);
	write_cmos_sensor(0x315f, 0x00);
	write_cmos_sensor(0x3160, 0x00);
	write_cmos_sensor(0x3161, 0x00);
	write_cmos_sensor(0x3162, 0x00);
	write_cmos_sensor(0x3163, 0x00);
	write_cmos_sensor(0x3164, 0x00);
	write_cmos_sensor(0x3165, 0x40);
	write_cmos_sensor(0x3190, 0x02);
	write_cmos_sensor(0x3191, 0x99);
	write_cmos_sensor(0x3193, 0x08);
	write_cmos_sensor(0x3194, 0x13);
	write_cmos_sensor(0x3195, 0x33);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x10);
	write_cmos_sensor(0x3198, 0x00);
	write_cmos_sensor(0x3199, 0x7f);
	write_cmos_sensor(0x319a, 0x00);
	write_cmos_sensor(0x319b, 0x00);
	write_cmos_sensor(0x319c, 0x80);
	write_cmos_sensor(0x319d, 0xbf);
	write_cmos_sensor(0x319e, 0xc0);
	write_cmos_sensor(0x319f, 0xff);
	write_cmos_sensor(0x31a0, 0x24);
	write_cmos_sensor(0x31a1, 0x55);
	write_cmos_sensor(0x31a2, 0x00);
	write_cmos_sensor(0x31a3, 0x08);
	write_cmos_sensor(0x31a6, 0x00);
	write_cmos_sensor(0x31a7, 0x00);
	write_cmos_sensor(0x31b0, 0x00);
	write_cmos_sensor(0x31b1, 0x00);
	write_cmos_sensor(0x31b2, 0x02);
	write_cmos_sensor(0x31b3, 0x00);
	write_cmos_sensor(0x31b4, 0x00);
	write_cmos_sensor(0x31b5, 0x01);
	write_cmos_sensor(0x31b6, 0x00);
	write_cmos_sensor(0x31b7, 0x00);
	write_cmos_sensor(0x31b8, 0x00);
	write_cmos_sensor(0x31b9, 0x00);
	write_cmos_sensor(0x31ba, 0x00);
	write_cmos_sensor(0x31d0, 0x3c);
	write_cmos_sensor(0x31d1, 0x34);
	write_cmos_sensor(0x31d2, 0x3c);
	write_cmos_sensor(0x31d3, 0x00);
	write_cmos_sensor(0x31d4, 0x2d);
	write_cmos_sensor(0x31d5, 0x00);
	write_cmos_sensor(0x31d6, 0x01);
	write_cmos_sensor(0x31d7, 0x06);
	write_cmos_sensor(0x31d8, 0x00);
	write_cmos_sensor(0x31d9, 0x64);
	write_cmos_sensor(0x31da, 0x00);
	write_cmos_sensor(0x31db, 0x30);
	write_cmos_sensor(0x31dc, 0x04);
	write_cmos_sensor(0x31dd, 0x69);
	write_cmos_sensor(0x31de, 0x0a);
	write_cmos_sensor(0x31df, 0x3c);
	write_cmos_sensor(0x31e0, 0x04);
	write_cmos_sensor(0x31e1, 0x32);
	write_cmos_sensor(0x31e2, 0x00);
	write_cmos_sensor(0x31e3, 0x00);
	write_cmos_sensor(0x31e4, 0x08);
	write_cmos_sensor(0x31e5, 0x80);
	write_cmos_sensor(0x31e6, 0x00);
	write_cmos_sensor(0x31e7, 0x2c);
	write_cmos_sensor(0x31e8, 0x6c);
	write_cmos_sensor(0x31e9, 0xac);
	write_cmos_sensor(0x31ea, 0xec);
	write_cmos_sensor(0x31eb, 0x3f);
	write_cmos_sensor(0x31ec, 0x0f);
	write_cmos_sensor(0x31ed, 0x20);
	write_cmos_sensor(0x31ee, 0x04);
	write_cmos_sensor(0x31ef, 0x48);
	write_cmos_sensor(0x31f0, 0x07);
	write_cmos_sensor(0x31f1, 0x90);
	write_cmos_sensor(0x31f2, 0x04);
	write_cmos_sensor(0x31f3, 0x48);
	write_cmos_sensor(0x31f4, 0x07);
	write_cmos_sensor(0x31f5, 0x90);
	write_cmos_sensor(0x31f6, 0x04);
	write_cmos_sensor(0x31f7, 0x48);
	write_cmos_sensor(0x31f8, 0x07);
	write_cmos_sensor(0x31f9, 0x90);
	write_cmos_sensor(0x31fa, 0x04);
	write_cmos_sensor(0x31fb, 0x48);
	write_cmos_sensor(0x31fd, 0xcb);
	write_cmos_sensor(0x31fe, 0x03);
	write_cmos_sensor(0x31ff, 0x03);
	write_cmos_sensor(0x3200, 0x00);
	write_cmos_sensor(0x3201, 0xff);
	write_cmos_sensor(0x3202, 0x00);
	write_cmos_sensor(0x3203, 0xff);
	write_cmos_sensor(0x3204, 0xff);
	write_cmos_sensor(0x3205, 0xff);
	write_cmos_sensor(0x3206, 0xff);
	write_cmos_sensor(0x3207, 0xff);
	write_cmos_sensor(0x3208, 0xff);
	write_cmos_sensor(0x3209, 0xff);
	write_cmos_sensor(0x320a, 0xff);
	write_cmos_sensor(0x320b, 0x1b);
	write_cmos_sensor(0x320c, 0x1f);
	write_cmos_sensor(0x320d, 0x1e);
	write_cmos_sensor(0x320e, 0x30);
	write_cmos_sensor(0x320f, 0x2d);
	write_cmos_sensor(0x3210, 0x2c);
	write_cmos_sensor(0x3211, 0x2b);
	write_cmos_sensor(0x3212, 0x2a);
	write_cmos_sensor(0x3213, 0x24);
	write_cmos_sensor(0x3214, 0x22);
	write_cmos_sensor(0x3215, 0x00);
	write_cmos_sensor(0x3216, 0x04);
	write_cmos_sensor(0x3217, 0x2c);
	write_cmos_sensor(0x3218, 0x6c);
	write_cmos_sensor(0x3219, 0xac);
	write_cmos_sensor(0x321a, 0xec);
	write_cmos_sensor(0x321b, 0x00);
	write_cmos_sensor(0x3230, 0x3a);
	write_cmos_sensor(0x3231, 0x00);
	write_cmos_sensor(0x3232, 0x80);
	write_cmos_sensor(0x3233, 0x00);
	write_cmos_sensor(0x3234, 0x10);
	write_cmos_sensor(0x3235, 0xaa);
	write_cmos_sensor(0x3236, 0x55);
	write_cmos_sensor(0x3237, 0x99);
	write_cmos_sensor(0x3238, 0x66);
	write_cmos_sensor(0x3239, 0x08);
	write_cmos_sensor(0x323a, 0x88);
	write_cmos_sensor(0x323b, 0x00);
	write_cmos_sensor(0x323c, 0x00);
	write_cmos_sensor(0x323d, 0x03);
	write_cmos_sensor(0x3250, 0x33);
	write_cmos_sensor(0x3251, 0x00);
	write_cmos_sensor(0x3252, 0x21);
	write_cmos_sensor(0x3253, 0x00);
	write_cmos_sensor(0x3254, 0x11);
	write_cmos_sensor(0x3255, 0x01);
	write_cmos_sensor(0x3256, 0x00);
	write_cmos_sensor(0x3257, 0x00);
	write_cmos_sensor(0x3258, 0x00);
	write_cmos_sensor(0x3270, 0x01);
	write_cmos_sensor(0x3271, 0x60);
	write_cmos_sensor(0x3272, 0xc0);
	write_cmos_sensor(0x3273, 0x00);
	write_cmos_sensor(0x3274, 0x80);
	write_cmos_sensor(0x3275, 0x40);
	write_cmos_sensor(0x3276, 0x02);
	write_cmos_sensor(0x3277, 0x08);
	write_cmos_sensor(0x3278, 0x10);
	write_cmos_sensor(0x3279, 0x04);
	write_cmos_sensor(0x327a, 0x00);
	write_cmos_sensor(0x327b, 0x03);
	write_cmos_sensor(0x327c, 0x10);
	write_cmos_sensor(0x327d, 0x60);
	write_cmos_sensor(0x327e, 0xc0);
	write_cmos_sensor(0x327f, 0x06);
	write_cmos_sensor(0x3288, 0x10);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x08);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x04);
	write_cmos_sensor(0x328d, 0x00);
	write_cmos_sensor(0x328e, 0x02);
	write_cmos_sensor(0x328f, 0x00);
	write_cmos_sensor(0x3290, 0x20);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x10);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x08);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x04);
	write_cmos_sensor(0x3297, 0x00);
	write_cmos_sensor(0x3298, 0x40);
	write_cmos_sensor(0x3299, 0x00);
	write_cmos_sensor(0x329a, 0x20);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x10);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x08);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x40);
	write_cmos_sensor(0x32a3, 0x00);
	write_cmos_sensor(0x32a4, 0x20);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x10);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32a8, 0x00);
	write_cmos_sensor(0x32a9, 0x00);
	write_cmos_sensor(0x32aa, 0x00);
	write_cmos_sensor(0x32ab, 0x00);
	write_cmos_sensor(0x32ac, 0x00);
	write_cmos_sensor(0x32ad, 0x00);
	write_cmos_sensor(0x32ae, 0x00);
	write_cmos_sensor(0x32af, 0x00);
	write_cmos_sensor(0x32b0, 0x00);
	write_cmos_sensor(0x32b1, 0x00);
	write_cmos_sensor(0x32b2, 0x00);
	write_cmos_sensor(0x32b3, 0x00);
	write_cmos_sensor(0x32b4, 0x00);
	write_cmos_sensor(0x32b5, 0x00);
	write_cmos_sensor(0x32b6, 0x00);
	write_cmos_sensor(0x32b7, 0x00);
	write_cmos_sensor(0x32b8, 0x00);
	write_cmos_sensor(0x32b9, 0x00);
	write_cmos_sensor(0x32ba, 0x00);
	write_cmos_sensor(0x32bb, 0x00);
	write_cmos_sensor(0x32bc, 0x00);
	write_cmos_sensor(0x32bd, 0x00);
	write_cmos_sensor(0x32be, 0x00);
	write_cmos_sensor(0x32bf, 0x00);
	write_cmos_sensor(0x32c0, 0x00);
	write_cmos_sensor(0x32c1, 0x00);
	write_cmos_sensor(0x32c2, 0x00);
	write_cmos_sensor(0x32c3, 0x00);
	write_cmos_sensor(0x32c4, 0x00);
	write_cmos_sensor(0x32c5, 0x00);
	write_cmos_sensor(0x32c6, 0x00);
	write_cmos_sensor(0x32c7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x32c9, 0x00);
	write_cmos_sensor(0x3330, 0x03);
	write_cmos_sensor(0x3331, 0xc8);
	write_cmos_sensor(0x3332, 0x02);
	write_cmos_sensor(0x3333, 0x24);
	write_cmos_sensor(0x3334, 0x00);
	write_cmos_sensor(0x3335, 0x00);
	write_cmos_sensor(0x3336, 0x00);
	write_cmos_sensor(0x3337, 0x00);
	write_cmos_sensor(0x3338, 0x03);
	write_cmos_sensor(0x3339, 0xc8);
	write_cmos_sensor(0x333a, 0x02);
	write_cmos_sensor(0x333b, 0x24);
	write_cmos_sensor(0x333c, 0x00);
	write_cmos_sensor(0x333d, 0x00);
	write_cmos_sensor(0x333e, 0x00);
	write_cmos_sensor(0x333f, 0x00);
	write_cmos_sensor(0x3340, 0x03);
	write_cmos_sensor(0x3341, 0xc8);
	write_cmos_sensor(0x3342, 0x02);
	write_cmos_sensor(0x3343, 0x24);
	write_cmos_sensor(0x3344, 0x00);
	write_cmos_sensor(0x3345, 0x00);
	write_cmos_sensor(0x3346, 0x00);
	write_cmos_sensor(0x3347, 0x00);
	write_cmos_sensor(0x3348, 0x40);
	write_cmos_sensor(0x3349, 0x00);
	write_cmos_sensor(0x334a, 0x00);
	write_cmos_sensor(0x334b, 0x00);
	write_cmos_sensor(0x334c, 0x00);
	write_cmos_sensor(0x334d, 0x00);
	write_cmos_sensor(0x334e, 0x80);
	write_cmos_sensor(0x3360, 0x01);
	write_cmos_sensor(0x3361, 0x00);
	write_cmos_sensor(0x3362, 0x01);
	write_cmos_sensor(0x3363, 0x00);
	write_cmos_sensor(0x3364, 0x01);
	write_cmos_sensor(0x3365, 0x00);
	write_cmos_sensor(0x3366, 0x01);
	write_cmos_sensor(0x3367, 0x00);
	write_cmos_sensor(0x3368, 0x01);
	write_cmos_sensor(0x3369, 0x00);
	write_cmos_sensor(0x336a, 0x01);
	write_cmos_sensor(0x336b, 0x00);
	write_cmos_sensor(0x336c, 0x01);
	write_cmos_sensor(0x336d, 0x00);
	write_cmos_sensor(0x336e, 0x01);
	write_cmos_sensor(0x336f, 0x00);
	write_cmos_sensor(0x3370, 0x01);
	write_cmos_sensor(0x3371, 0x00);
	write_cmos_sensor(0x3372, 0x01);
	write_cmos_sensor(0x3373, 0x00);
	write_cmos_sensor(0x3374, 0x01);
	write_cmos_sensor(0x3375, 0x00);
	write_cmos_sensor(0x3376, 0x01);
	write_cmos_sensor(0x3377, 0x00);
	write_cmos_sensor(0x3378, 0x00);
	write_cmos_sensor(0x3379, 0x00);
	write_cmos_sensor(0x337a, 0x00);
	write_cmos_sensor(0x337b, 0x00);
	write_cmos_sensor(0x337c, 0x00);
	write_cmos_sensor(0x337d, 0x00);
	write_cmos_sensor(0x337e, 0x00);
	write_cmos_sensor(0x337f, 0x00);
	write_cmos_sensor(0x3380, 0x00);
	write_cmos_sensor(0x3381, 0x00);
	write_cmos_sensor(0x3382, 0x00);
	write_cmos_sensor(0x3383, 0x00);
	write_cmos_sensor(0x3384, 0x00);
	write_cmos_sensor(0x3385, 0x00);
	write_cmos_sensor(0x3386, 0x00);
	write_cmos_sensor(0x3387, 0x00);
	write_cmos_sensor(0x3388, 0x00);
	write_cmos_sensor(0x3389, 0x00);
	write_cmos_sensor(0x338a, 0x00);
	write_cmos_sensor(0x338b, 0x00);
	write_cmos_sensor(0x338c, 0x00);
	write_cmos_sensor(0x338d, 0x00);
	write_cmos_sensor(0x338e, 0x00);
	write_cmos_sensor(0x338f, 0x00);
	write_cmos_sensor(0x3390, 0x00);
	write_cmos_sensor(0x3391, 0x00);
	write_cmos_sensor(0x3392, 0x00);
	write_cmos_sensor(0x3393, 0x00);
	write_cmos_sensor(0x3394, 0x00);
	write_cmos_sensor(0x3395, 0x00);
	write_cmos_sensor(0x3396, 0x00);
	write_cmos_sensor(0x3397, 0x00);
	write_cmos_sensor(0x3398, 0x00);
	write_cmos_sensor(0x3399, 0x00);
	write_cmos_sensor(0x339a, 0x00);
	write_cmos_sensor(0x339b, 0x00);
	write_cmos_sensor(0x33b0, 0x00);
	write_cmos_sensor(0x33b1, 0x50);
	write_cmos_sensor(0x33b2, 0x01);
	write_cmos_sensor(0x33b3, 0xff);
	write_cmos_sensor(0x33b4, 0xe0);
	write_cmos_sensor(0x33b5, 0x6b);
	write_cmos_sensor(0x33b6, 0x00);
	write_cmos_sensor(0x33b7, 0x00);
	write_cmos_sensor(0x33b8, 0x00);
	write_cmos_sensor(0x33b9, 0x00);
	write_cmos_sensor(0x33ba, 0x00);
	write_cmos_sensor(0x33bb, 0x1f);
	write_cmos_sensor(0x33bc, 0x01);
	write_cmos_sensor(0x33bd, 0x01);
	write_cmos_sensor(0x33be, 0x01);
	write_cmos_sensor(0x33bf, 0x01);
	write_cmos_sensor(0x33c0, 0x00);
	write_cmos_sensor(0x33c1, 0x00);
	write_cmos_sensor(0x33c2, 0x00);
	write_cmos_sensor(0x33c3, 0x00);
	write_cmos_sensor(0x33e0, 0x14);
	write_cmos_sensor(0x33e1, 0x0f);
	write_cmos_sensor(0x33e2, 0x04);
	write_cmos_sensor(0x33e3, 0x02);
	write_cmos_sensor(0x33e4, 0x01);
	write_cmos_sensor(0x33e5, 0x01);
	write_cmos_sensor(0x33e6, 0x00);
	write_cmos_sensor(0x33e7, 0x04);
	write_cmos_sensor(0x33e8, 0x0c);
	write_cmos_sensor(0x33e9, 0x00);
	write_cmos_sensor(0x33ea, 0x01);
	write_cmos_sensor(0x33eb, 0x02);
	write_cmos_sensor(0x33ec, 0x03);
	write_cmos_sensor(0x33ed, 0x02);
	write_cmos_sensor(0x33ee, 0x05);
	write_cmos_sensor(0x33ef, 0x0a);
	write_cmos_sensor(0x33f0, 0x08);
	write_cmos_sensor(0x33f1, 0x04);
	write_cmos_sensor(0x33f2, 0x04);
	write_cmos_sensor(0x33f3, 0x00);
	write_cmos_sensor(0x33f4, 0x03);
	write_cmos_sensor(0x33f5, 0x14);
	write_cmos_sensor(0x33f6, 0x0f);
	write_cmos_sensor(0x33f7, 0x02);
	write_cmos_sensor(0x33f8, 0x01);
	write_cmos_sensor(0x33f9, 0x01);
	write_cmos_sensor(0x33fa, 0x01);
	write_cmos_sensor(0x33fb, 0x00);
	write_cmos_sensor(0x33fc, 0x04);
	write_cmos_sensor(0x33fd, 0x0c);
	write_cmos_sensor(0x33fe, 0x00);
	write_cmos_sensor(0x33ff, 0x01);
	write_cmos_sensor(0x3400, 0x02);
	write_cmos_sensor(0x3401, 0x03);
	write_cmos_sensor(0x3402, 0x01);
	write_cmos_sensor(0x3403, 0x02);
	write_cmos_sensor(0x3404, 0x08);
	write_cmos_sensor(0x3405, 0x08);
	write_cmos_sensor(0x3406, 0x04);
	write_cmos_sensor(0x3407, 0x04);
	write_cmos_sensor(0x3408, 0x00);
	write_cmos_sensor(0x3409, 0x03);
	write_cmos_sensor(0x340a, 0x14);
	write_cmos_sensor(0x340b, 0x0f);
	write_cmos_sensor(0x340c, 0x04);
	write_cmos_sensor(0x340d, 0x02);
	write_cmos_sensor(0x340e, 0x01);
	write_cmos_sensor(0x340f, 0x01);
	write_cmos_sensor(0x3410, 0x00);
	write_cmos_sensor(0x3411, 0x04);
	write_cmos_sensor(0x3412, 0x0c);
	write_cmos_sensor(0x3413, 0x00);
	write_cmos_sensor(0x3414, 0x01);
	write_cmos_sensor(0x3415, 0x02);
	write_cmos_sensor(0x3416, 0x03);
	write_cmos_sensor(0x3417, 0x02);
	write_cmos_sensor(0x3418, 0x05);
	write_cmos_sensor(0x3419, 0x0a);
	write_cmos_sensor(0x341a, 0x08);
	write_cmos_sensor(0x341b, 0x04);
	write_cmos_sensor(0x341c, 0x04);
	write_cmos_sensor(0x341d, 0x00);
	write_cmos_sensor(0x341e, 0x03);
	write_cmos_sensor(0x3440, 0x00);
	write_cmos_sensor(0x3441, 0x00);
	write_cmos_sensor(0x3442, 0x00);
	write_cmos_sensor(0x3443, 0x00);
	write_cmos_sensor(0x3444, 0x02);
	write_cmos_sensor(0x3445, 0xf0);
	write_cmos_sensor(0x3446, 0x02);
	write_cmos_sensor(0x3447, 0x08);
	write_cmos_sensor(0x3448, 0x00);
	write_cmos_sensor(0x3460, 0x40);
	write_cmos_sensor(0x3461, 0x40);
	write_cmos_sensor(0x3462, 0x40);
	write_cmos_sensor(0x3463, 0x40);
	write_cmos_sensor(0x3464, 0x03);
	write_cmos_sensor(0x3465, 0x01);
	write_cmos_sensor(0x3466, 0x01);
	write_cmos_sensor(0x3467, 0x02);
	write_cmos_sensor(0x3468, 0x30);
	write_cmos_sensor(0x3469, 0x00);
	write_cmos_sensor(0x346a, 0x35);
	write_cmos_sensor(0x346b, 0x00);
	write_cmos_sensor(0x3480, 0x40);
	write_cmos_sensor(0x3481, 0x00);
	write_cmos_sensor(0x3482, 0x00);
	write_cmos_sensor(0x3483, 0x00);
	write_cmos_sensor(0x3484, 0x0d);
	write_cmos_sensor(0x3485, 0x00);
	write_cmos_sensor(0x3486, 0x00);
	write_cmos_sensor(0x3487, 0x00);
	write_cmos_sensor(0x3488, 0x00);
	write_cmos_sensor(0x3489, 0x00);
	write_cmos_sensor(0x348a, 0x00);
	write_cmos_sensor(0x348b, 0x04);
	write_cmos_sensor(0x348c, 0x00);
	write_cmos_sensor(0x348d, 0x01);
	write_cmos_sensor(0x348f, 0x01);
	write_cmos_sensor(0x3030, 0x0a);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x7000, 0x58);
	write_cmos_sensor(0x7001, 0x7a);
	write_cmos_sensor(0x7002, 0x1a);
	write_cmos_sensor(0x7003, 0xc1);
	write_cmos_sensor(0x7004, 0x03);
	write_cmos_sensor(0x7005, 0xda);
	write_cmos_sensor(0x7006, 0xbd);
	write_cmos_sensor(0x7007, 0x03);
	write_cmos_sensor(0x7008, 0xbd);
	write_cmos_sensor(0x7009, 0x06);
	write_cmos_sensor(0x700a, 0xe6);
	write_cmos_sensor(0x700b, 0xec);
	write_cmos_sensor(0x700c, 0xbc);
	write_cmos_sensor(0x700d, 0xff);
	write_cmos_sensor(0x700e, 0xbc);
	write_cmos_sensor(0x700f, 0x73);
	write_cmos_sensor(0x7010, 0xda);
	write_cmos_sensor(0x7011, 0x72);
	write_cmos_sensor(0x7012, 0x76);
	write_cmos_sensor(0x7013, 0xb6);
	write_cmos_sensor(0x7014, 0xee);
	write_cmos_sensor(0x7015, 0xcf);
	write_cmos_sensor(0x7016, 0xac);
	write_cmos_sensor(0x7017, 0xd0);
	write_cmos_sensor(0x7018, 0xac);
	write_cmos_sensor(0x7019, 0xd1);
	write_cmos_sensor(0x701a, 0x50);
	write_cmos_sensor(0x701b, 0xac);
	write_cmos_sensor(0x701c, 0xd2);
	write_cmos_sensor(0x701d, 0xbc);
	write_cmos_sensor(0x701e, 0x2e);
	write_cmos_sensor(0x701f, 0xb4);
	write_cmos_sensor(0x7020, 0x00);
	write_cmos_sensor(0x7021, 0xdc);
	write_cmos_sensor(0x7022, 0xdf);
	write_cmos_sensor(0x7023, 0xb0);
	write_cmos_sensor(0x7024, 0x6e);
	write_cmos_sensor(0x7025, 0xbd);
	write_cmos_sensor(0x7026, 0x01);
	write_cmos_sensor(0x7027, 0xd7);
	write_cmos_sensor(0x7028, 0xed);
	write_cmos_sensor(0x7029, 0xe1);
	write_cmos_sensor(0x702a, 0x36);
	write_cmos_sensor(0x702b, 0x30);
	write_cmos_sensor(0x702c, 0xd3);
	write_cmos_sensor(0x702d, 0x2e);
	write_cmos_sensor(0x702e, 0x54);
	write_cmos_sensor(0x702f, 0x46);
	write_cmos_sensor(0x7030, 0xbc);
	write_cmos_sensor(0x7031, 0x22);
	write_cmos_sensor(0x7032, 0x66);
	write_cmos_sensor(0x7033, 0xbc);
	write_cmos_sensor(0x7034, 0x24);
	write_cmos_sensor(0x7035, 0x2c);
	write_cmos_sensor(0x7036, 0x28);
	write_cmos_sensor(0x7037, 0xbc);
	write_cmos_sensor(0x7038, 0x3c);
	write_cmos_sensor(0x7039, 0xa1);
	write_cmos_sensor(0x703a, 0xac);
	write_cmos_sensor(0x703b, 0xd8);
	write_cmos_sensor(0x703c, 0xd6);
	write_cmos_sensor(0x703d, 0xb4);
	write_cmos_sensor(0x703e, 0x04);
	write_cmos_sensor(0x703f, 0x46);
	write_cmos_sensor(0x7040, 0xb7);
	write_cmos_sensor(0x7041, 0x04);
	write_cmos_sensor(0x7042, 0xbe);
	write_cmos_sensor(0x7043, 0x08);
	write_cmos_sensor(0x7044, 0xc3);
	write_cmos_sensor(0x7045, 0xd9);
	write_cmos_sensor(0x7046, 0xad);
	write_cmos_sensor(0x7047, 0xc3);
	write_cmos_sensor(0x7048, 0xbc);
	write_cmos_sensor(0x7049, 0x19);
	write_cmos_sensor(0x704a, 0xc1);
	write_cmos_sensor(0x704b, 0x27);
	write_cmos_sensor(0x704c, 0xe7);
	write_cmos_sensor(0x704d, 0x00);
	write_cmos_sensor(0x704e, 0xb9);
	write_cmos_sensor(0x704f, 0x64);
	write_cmos_sensor(0x7050, 0x50);
	write_cmos_sensor(0x7051, 0x20);
	write_cmos_sensor(0x7052, 0xb8);
	write_cmos_sensor(0x7053, 0x02);
	write_cmos_sensor(0x7054, 0xbc);
	write_cmos_sensor(0x7055, 0x17);
	write_cmos_sensor(0x7056, 0xdb);
	write_cmos_sensor(0x7057, 0xc7);
	write_cmos_sensor(0x7058, 0xb8);
	write_cmos_sensor(0x7059, 0x00);
	write_cmos_sensor(0x705a, 0x28);
	write_cmos_sensor(0x705b, 0x54);
	write_cmos_sensor(0x705c, 0xb4);
	write_cmos_sensor(0x705d, 0x14);
	write_cmos_sensor(0x705e, 0xab);
	write_cmos_sensor(0x705f, 0xbe);
	write_cmos_sensor(0x7060, 0x06);
	write_cmos_sensor(0x7061, 0xd8);
	write_cmos_sensor(0x7062, 0xd6);
	write_cmos_sensor(0x7063, 0x00);
	write_cmos_sensor(0x7064, 0xb4);
	write_cmos_sensor(0x7065, 0xc7);
	write_cmos_sensor(0x7066, 0x62);
	write_cmos_sensor(0x7067, 0x07);
	write_cmos_sensor(0x7068, 0xb9);
	write_cmos_sensor(0x7069, 0x05);
	write_cmos_sensor(0x706a, 0xee);
	write_cmos_sensor(0x706b, 0xe6);
	write_cmos_sensor(0x706c, 0xad);
	write_cmos_sensor(0x706d, 0xb4);
	write_cmos_sensor(0x706e, 0x26);
	write_cmos_sensor(0x706f, 0x19);
	write_cmos_sensor(0x7070, 0xc1);
	write_cmos_sensor(0x7071, 0x3b);
	write_cmos_sensor(0x7072, 0xc3);
	write_cmos_sensor(0x7073, 0xaf);
	write_cmos_sensor(0x7074, 0xc0);
	write_cmos_sensor(0x7075, 0x3d);
	write_cmos_sensor(0x7076, 0xc3);
	write_cmos_sensor(0x7077, 0xbe);
	write_cmos_sensor(0x7078, 0xe7);
	write_cmos_sensor(0x7079, 0x00);
	write_cmos_sensor(0x707a, 0x15);
	write_cmos_sensor(0x707b, 0xc2);
	write_cmos_sensor(0x707c, 0x41);
	write_cmos_sensor(0x707d, 0xc3);
	write_cmos_sensor(0x707e, 0xa4);
	write_cmos_sensor(0x707f, 0xc0);
	write_cmos_sensor(0x7080, 0x3d);
	write_cmos_sensor(0x7081, 0x00);
	write_cmos_sensor(0x7082, 0xb9);
	write_cmos_sensor(0x7083, 0x64);
	write_cmos_sensor(0x7084, 0x29);
	write_cmos_sensor(0x7085, 0x00);
	write_cmos_sensor(0x7086, 0xb8);
	write_cmos_sensor(0x7087, 0x12);
	write_cmos_sensor(0x7088, 0xbe);
	write_cmos_sensor(0x7089, 0x01);
	write_cmos_sensor(0x708a, 0xd0);
	write_cmos_sensor(0x708b, 0xbc);
	write_cmos_sensor(0x708c, 0x01);
	write_cmos_sensor(0x708d, 0xac);
	write_cmos_sensor(0x708e, 0x37);
	write_cmos_sensor(0x708f, 0xd2);
	write_cmos_sensor(0x7090, 0xac);
	write_cmos_sensor(0x7091, 0x45);
	write_cmos_sensor(0x7092, 0xad);
	write_cmos_sensor(0x7093, 0x28);
	write_cmos_sensor(0x7094, 0x00);
	write_cmos_sensor(0x7095, 0xb8);
	write_cmos_sensor(0x7096, 0x00);
	write_cmos_sensor(0x7097, 0xbc);
	write_cmos_sensor(0x7098, 0x01);
	write_cmos_sensor(0x7099, 0x36);
	write_cmos_sensor(0x709a, 0xd3);
	write_cmos_sensor(0x709b, 0x30);
	write_cmos_sensor(0x709c, 0x04);
	write_cmos_sensor(0x709d, 0xe0);
	write_cmos_sensor(0x709e, 0xd8);
	write_cmos_sensor(0x709f, 0xb4);
	write_cmos_sensor(0x70a0, 0xe9);
	write_cmos_sensor(0x70a1, 0x00);
	write_cmos_sensor(0x70a2, 0xbe);
	write_cmos_sensor(0x70a3, 0x05);
	write_cmos_sensor(0x70a4, 0x62);
	write_cmos_sensor(0x70a5, 0x07);
	write_cmos_sensor(0x70a6, 0xb9);
	write_cmos_sensor(0x70a7, 0x05);
	write_cmos_sensor(0x70a8, 0xad);
	write_cmos_sensor(0x70a9, 0xc3);
	write_cmos_sensor(0x70aa, 0xcf);
	write_cmos_sensor(0x70ab, 0x00);
	write_cmos_sensor(0x70ac, 0x15);
	write_cmos_sensor(0x70ad, 0xc2);
	write_cmos_sensor(0x70ae, 0x5a);
	write_cmos_sensor(0x70af, 0xc3);
	write_cmos_sensor(0x70b0, 0xc9);
	write_cmos_sensor(0x70b1, 0xc0);
	write_cmos_sensor(0x70b2, 0x56);
	write_cmos_sensor(0x70b3, 0x00);
	write_cmos_sensor(0x70b4, 0x46);
	write_cmos_sensor(0x70b5, 0xa1);
	write_cmos_sensor(0x70b6, 0xb9);
	write_cmos_sensor(0x70b7, 0x64);
	write_cmos_sensor(0x70b8, 0x29);
	write_cmos_sensor(0x70b9, 0x00);
	write_cmos_sensor(0x70ba, 0xb8);
	write_cmos_sensor(0x70bb, 0x02);
	write_cmos_sensor(0x70bc, 0xbe);
	write_cmos_sensor(0x70bd, 0x02);
	write_cmos_sensor(0x70be, 0xd0);
	write_cmos_sensor(0x70bf, 0xdc);
	write_cmos_sensor(0x70c0, 0xac);
	write_cmos_sensor(0x70c1, 0xbc);
	write_cmos_sensor(0x70c2, 0x01);
	write_cmos_sensor(0x70c3, 0x37);
	write_cmos_sensor(0x70c4, 0xac);
	write_cmos_sensor(0x70c5, 0xd2);
	write_cmos_sensor(0x70c6, 0x45);
	write_cmos_sensor(0x70c7, 0xad);
	write_cmos_sensor(0x70c8, 0x28);
	write_cmos_sensor(0x70c9, 0x00);
	write_cmos_sensor(0x70ca, 0xb8);
	write_cmos_sensor(0x70cb, 0x00);
	write_cmos_sensor(0x70cc, 0xbc);
	write_cmos_sensor(0x70cd, 0x01);
	write_cmos_sensor(0x70ce, 0x36);
	write_cmos_sensor(0x70cf, 0x30);
	write_cmos_sensor(0x70d0, 0xe0);
	write_cmos_sensor(0x70d1, 0xd8);
	write_cmos_sensor(0x70d2, 0xb5);
	write_cmos_sensor(0x70d3, 0x0b);
	write_cmos_sensor(0x70d4, 0xd6);
	write_cmos_sensor(0x70d5, 0xbe);
	write_cmos_sensor(0x70d6, 0x07);
	write_cmos_sensor(0x70d7, 0x00);
	write_cmos_sensor(0x70d8, 0x62);
	write_cmos_sensor(0x70d9, 0x07);
	write_cmos_sensor(0x70da, 0xb9);
	write_cmos_sensor(0x70db, 0x05);
	write_cmos_sensor(0x70dc, 0xad);
	write_cmos_sensor(0x70dd, 0xc3);
	write_cmos_sensor(0x70de, 0xcf);
	write_cmos_sensor(0x70df, 0x46);
	write_cmos_sensor(0x70e0, 0xcd);
	write_cmos_sensor(0x70e1, 0x07);
	write_cmos_sensor(0x70e2, 0xcd);
	write_cmos_sensor(0x70e3, 0x00);
	write_cmos_sensor(0x70e4, 0xe3);
	write_cmos_sensor(0x70e5, 0x18);
	write_cmos_sensor(0x70e6, 0xc2);
	write_cmos_sensor(0x70e7, 0xa2);
	write_cmos_sensor(0x70e8, 0xb9);
	write_cmos_sensor(0x70e9, 0x64);
	write_cmos_sensor(0x70ea, 0xd1);
	write_cmos_sensor(0x70eb, 0xdd);
	write_cmos_sensor(0x70ec, 0xac);
	write_cmos_sensor(0x70ed, 0xcf);
	write_cmos_sensor(0x70ee, 0xdf);
	write_cmos_sensor(0x70ef, 0xb5);
	write_cmos_sensor(0x70f0, 0x19);
	write_cmos_sensor(0x70f1, 0x46);
	write_cmos_sensor(0x70f2, 0x50);
	write_cmos_sensor(0x70f3, 0xb6);
	write_cmos_sensor(0x70f4, 0xee);
	write_cmos_sensor(0x70f5, 0xe8);
	write_cmos_sensor(0x70f6, 0xe6);
	write_cmos_sensor(0x70f7, 0xbc);
	write_cmos_sensor(0x70f8, 0x31);
	write_cmos_sensor(0x70f9, 0xe1);
	write_cmos_sensor(0x70fa, 0x36);
	write_cmos_sensor(0x70fb, 0x30);
	write_cmos_sensor(0x70fc, 0xd3);
	write_cmos_sensor(0x70fd, 0x2e);
	write_cmos_sensor(0x70fe, 0x54);
	write_cmos_sensor(0x70ff, 0xbd);
	write_cmos_sensor(0x7100, 0x03);
	write_cmos_sensor(0x7101, 0xec);
	write_cmos_sensor(0x7102, 0x2c);
	write_cmos_sensor(0x7103, 0x50);
	write_cmos_sensor(0x7104, 0x20);
	write_cmos_sensor(0x7105, 0x04);
	write_cmos_sensor(0x7106, 0xb8);
	write_cmos_sensor(0x7107, 0x02);
	write_cmos_sensor(0x7108, 0xbc);
	write_cmos_sensor(0x7109, 0x18);
	write_cmos_sensor(0x710a, 0xc7);
	write_cmos_sensor(0x710b, 0xb8);
	write_cmos_sensor(0x710c, 0x00);
	write_cmos_sensor(0x710d, 0x28);
	write_cmos_sensor(0x710e, 0x54);
	write_cmos_sensor(0x710f, 0x02);
	write_cmos_sensor(0x7110, 0xb4);
	write_cmos_sensor(0x7111, 0xda);
	write_cmos_sensor(0x7112, 0xbe);
	write_cmos_sensor(0x7113, 0x04);
	write_cmos_sensor(0x7114, 0xd6);
	write_cmos_sensor(0x7115, 0xd8);
	write_cmos_sensor(0x7116, 0xab);
	write_cmos_sensor(0x7117, 0x00);
	write_cmos_sensor(0x7118, 0x62);
	write_cmos_sensor(0x7119, 0x07);
	write_cmos_sensor(0x711a, 0xb9);
	write_cmos_sensor(0x711b, 0x05);
	write_cmos_sensor(0x711c, 0xad);
	write_cmos_sensor(0x711d, 0xc3);
	write_cmos_sensor(0x711e, 0xbc);
	write_cmos_sensor(0x711f, 0xe7);
	write_cmos_sensor(0x7120, 0xb9);
	write_cmos_sensor(0x7121, 0x64);
	write_cmos_sensor(0x7122, 0x29);
	write_cmos_sensor(0x7123, 0x00);
	write_cmos_sensor(0x7124, 0xb8);
	write_cmos_sensor(0x7125, 0x02);
	write_cmos_sensor(0x7126, 0xbe);
	write_cmos_sensor(0x7127, 0x00);
	write_cmos_sensor(0x7128, 0x45);
	write_cmos_sensor(0x7129, 0xad);
	write_cmos_sensor(0x712a, 0xe2);
	write_cmos_sensor(0x712b, 0x28);
	write_cmos_sensor(0x712c, 0x00);
	write_cmos_sensor(0x712d, 0xb8);
	write_cmos_sensor(0x712e, 0x00);
	write_cmos_sensor(0x712f, 0xe0);
	write_cmos_sensor(0x7130, 0xd8);
	write_cmos_sensor(0x7131, 0xb4);
	write_cmos_sensor(0x7132, 0xe9);
	write_cmos_sensor(0x7133, 0xbe);
	write_cmos_sensor(0x7134, 0x03);
	write_cmos_sensor(0x7135, 0x00);
	write_cmos_sensor(0x7136, 0x30);
	write_cmos_sensor(0x7137, 0x62);
	write_cmos_sensor(0x7138, 0x07);
	write_cmos_sensor(0x7139, 0xb9);
	write_cmos_sensor(0x713a, 0x05);
	write_cmos_sensor(0x713b, 0xad);
	write_cmos_sensor(0x713c, 0xc3);
	write_cmos_sensor(0x713d, 0xcf);
	write_cmos_sensor(0x713e, 0x42);
	write_cmos_sensor(0x713f, 0xe4);
	write_cmos_sensor(0x7140, 0xcd);
	write_cmos_sensor(0x7141, 0x07);
	write_cmos_sensor(0x7142, 0xcd);
	write_cmos_sensor(0x7143, 0x00);
	write_cmos_sensor(0x7144, 0x17);
	write_cmos_sensor(0x7145, 0xc2);
	write_cmos_sensor(0x7146, 0xbb);
	write_cmos_sensor(0x7147, 0xde);
	write_cmos_sensor(0x7148, 0xcf);
	write_cmos_sensor(0x7149, 0xdf);
	write_cmos_sensor(0x714a, 0xac);
	write_cmos_sensor(0x714b, 0xd1);
	write_cmos_sensor(0x714c, 0x44);
	write_cmos_sensor(0x714d, 0xac);
	write_cmos_sensor(0x714e, 0xb9);
	write_cmos_sensor(0x714f, 0x76);
	write_cmos_sensor(0x7150, 0xb8);
	write_cmos_sensor(0x7151, 0x08);
	write_cmos_sensor(0x7152, 0xb6);
	write_cmos_sensor(0x7153, 0xfe);
	write_cmos_sensor(0x7154, 0xb4);
	write_cmos_sensor(0x7155, 0xca);
	write_cmos_sensor(0x7156, 0xd6);
	write_cmos_sensor(0x7157, 0xd8);
	write_cmos_sensor(0x7158, 0xab);
	write_cmos_sensor(0x7159, 0x00);
	write_cmos_sensor(0x715a, 0xe1);
	write_cmos_sensor(0x715b, 0x36);
	write_cmos_sensor(0x715c, 0x30);
	write_cmos_sensor(0x715d, 0xd3);
	write_cmos_sensor(0x715e, 0xbc);
	write_cmos_sensor(0x715f, 0x29);
	write_cmos_sensor(0x7160, 0xb4);
	write_cmos_sensor(0x7161, 0x1f);
	write_cmos_sensor(0x7162, 0xaa);
	write_cmos_sensor(0x7163, 0xbd);
	write_cmos_sensor(0x7164, 0x01);
	write_cmos_sensor(0x7165, 0xb8);
	write_cmos_sensor(0x7166, 0x0c);
	write_cmos_sensor(0x7167, 0x45);
	write_cmos_sensor(0x7168, 0xa4);
	write_cmos_sensor(0x7169, 0xbd);
	write_cmos_sensor(0x716a, 0x03);
	write_cmos_sensor(0x716b, 0xec);
	write_cmos_sensor(0x716c, 0xbc);
	write_cmos_sensor(0x716d, 0x3d);
	write_cmos_sensor(0x716e, 0xc3);
	write_cmos_sensor(0x716f, 0xcf);
	write_cmos_sensor(0x7170, 0x42);
	write_cmos_sensor(0x7171, 0xb8);
	write_cmos_sensor(0x7172, 0x00);
	write_cmos_sensor(0x7173, 0xe4);
	write_cmos_sensor(0x7174, 0xd5);
	write_cmos_sensor(0x7175, 0x00);
	write_cmos_sensor(0x7176, 0xb6);
	write_cmos_sensor(0x7177, 0x00);
	write_cmos_sensor(0x7178, 0x74);
	write_cmos_sensor(0x7179, 0xbd);
	write_cmos_sensor(0x717a, 0x03);
	write_cmos_sensor(0x717b, 0xb5);
	write_cmos_sensor(0x717c, 0x39);
	write_cmos_sensor(0x717d, 0x40);
	write_cmos_sensor(0x717e, 0x58);
	write_cmos_sensor(0x717f, 0x6a);
	write_cmos_sensor(0x7180, 0xdd);
	write_cmos_sensor(0x7181, 0x19);
	write_cmos_sensor(0x7182, 0xc1);
	write_cmos_sensor(0x7183, 0xc8);
	write_cmos_sensor(0x7184, 0xbd);
	write_cmos_sensor(0x7185, 0x06);
	write_cmos_sensor(0x7186, 0x17);
	write_cmos_sensor(0x7187, 0xc1);
	write_cmos_sensor(0x7188, 0xc6);
	write_cmos_sensor(0x7189, 0xe8);
	write_cmos_sensor(0x718a, 0xc0);
	write_cmos_sensor(0x718b, 0xc8);
	write_cmos_sensor(0x718c, 0xe6);
	write_cmos_sensor(0x718d, 0x95);
	write_cmos_sensor(0x718e, 0x15);
	write_cmos_sensor(0x718f, 0x00);
	write_cmos_sensor(0x7190, 0xbc);
	write_cmos_sensor(0x7191, 0x19);
	write_cmos_sensor(0x7192, 0xb9);
	write_cmos_sensor(0x7193, 0xf6);
	write_cmos_sensor(0x7194, 0x14);
	write_cmos_sensor(0x7195, 0xc1);
	write_cmos_sensor(0x7196, 0xd0);
	write_cmos_sensor(0x7197, 0xd1);
	write_cmos_sensor(0x7198, 0xac);
	write_cmos_sensor(0x7199, 0x37);
	write_cmos_sensor(0x719a, 0xbc);
	write_cmos_sensor(0x719b, 0x35);
	write_cmos_sensor(0x719c, 0x36);
	write_cmos_sensor(0x719d, 0x30);
	write_cmos_sensor(0x719e, 0xe1);
	write_cmos_sensor(0x719f, 0xd3);
	write_cmos_sensor(0x71a0, 0x7a);
	write_cmos_sensor(0x71a1, 0xb6);
	write_cmos_sensor(0x71a2, 0x0c);
	write_cmos_sensor(0x71a3, 0xff);
	write_cmos_sensor(0x71a4, 0xb4);
	write_cmos_sensor(0x71a5, 0xc7);
	write_cmos_sensor(0x71a6, 0xd9);
	write_cmos_sensor(0x71a7, 0x00);
	write_cmos_sensor(0x71a8, 0xbd);
	write_cmos_sensor(0x71a9, 0x01);
	write_cmos_sensor(0x71aa, 0x56);
	write_cmos_sensor(0x71ab, 0xc0);
	write_cmos_sensor(0x71ac, 0xda);
	write_cmos_sensor(0x71ad, 0xb4);
	write_cmos_sensor(0x71ae, 0x1f);
	write_cmos_sensor(0x71af, 0x56);
	write_cmos_sensor(0x71b0, 0xaa);
	write_cmos_sensor(0x71b1, 0xbc);
	write_cmos_sensor(0x71b2, 0x08);
	write_cmos_sensor(0x71b3, 0x00);
	write_cmos_sensor(0x71b4, 0x57);
	write_cmos_sensor(0x71b5, 0xe8);
	write_cmos_sensor(0x71b6, 0xb5);
	write_cmos_sensor(0x71b7, 0x36);
	write_cmos_sensor(0x71b8, 0x00);
	write_cmos_sensor(0x71b9, 0x54);
	write_cmos_sensor(0x71ba, 0xe7);
	write_cmos_sensor(0x71bb, 0xc8);
	write_cmos_sensor(0x71bc, 0xb4);
	write_cmos_sensor(0x71bd, 0x1f);
	write_cmos_sensor(0x71be, 0x56);
	write_cmos_sensor(0x71bf, 0xaa);
	write_cmos_sensor(0x71c0, 0xbc);
	write_cmos_sensor(0x71c1, 0x08);
	write_cmos_sensor(0x71c2, 0x57);
	write_cmos_sensor(0x71c3, 0x00);
	write_cmos_sensor(0x71c4, 0xb5);
	write_cmos_sensor(0x71c5, 0x36);
	write_cmos_sensor(0x71c6, 0x00);
	write_cmos_sensor(0x71c7, 0x54);
	write_cmos_sensor(0x71c8, 0xc8);
	write_cmos_sensor(0x71c9, 0xb5);
	write_cmos_sensor(0x71ca, 0x18);
	write_cmos_sensor(0x71cb, 0xd9);
	write_cmos_sensor(0x71cc, 0x00);
	write_cmos_sensor(0x71cd, 0xbd);
	write_cmos_sensor(0x71ce, 0x01);
	write_cmos_sensor(0x71cf, 0x56);
	write_cmos_sensor(0x71d0, 0x08);
	write_cmos_sensor(0x71d1, 0x57);
	write_cmos_sensor(0x71d2, 0xe8);
	write_cmos_sensor(0x71d3, 0xb4);
	write_cmos_sensor(0x71d4, 0x42);
	write_cmos_sensor(0x71d5, 0x00);
	write_cmos_sensor(0x71d6, 0x54);
	write_cmos_sensor(0x71d7, 0xe7);
	write_cmos_sensor(0x71d8, 0xc8);
	write_cmos_sensor(0x71d9, 0xab);
	write_cmos_sensor(0x71da, 0x00);
	write_cmos_sensor(0x71db, 0x66);
	write_cmos_sensor(0x71dc, 0x62);
	write_cmos_sensor(0x71dd, 0x06);
	write_cmos_sensor(0x71de, 0x74);
	write_cmos_sensor(0x71df, 0xb9);
	write_cmos_sensor(0x71e0, 0x05);
	write_cmos_sensor(0x71e1, 0xb7);
	write_cmos_sensor(0x71e2, 0x14);
	write_cmos_sensor(0x71e3, 0x0e);
	write_cmos_sensor(0x71e4, 0xb7);
	write_cmos_sensor(0x71e5, 0x04);
	write_cmos_sensor(0x71e6, 0xc8);
	write_cmos_sensor(0x7600, 0x04);
	write_cmos_sensor(0x7601, 0x80);
	write_cmos_sensor(0x7602, 0x07);
	write_cmos_sensor(0x7603, 0x44);
	write_cmos_sensor(0x7604, 0x05);
	write_cmos_sensor(0x7605, 0x33);
	write_cmos_sensor(0x7606, 0x0f);
	write_cmos_sensor(0x7607, 0x00);
	write_cmos_sensor(0x7608, 0x07);
	write_cmos_sensor(0x7609, 0x40);
	write_cmos_sensor(0x760a, 0x04);
	write_cmos_sensor(0x760b, 0xe5);
	write_cmos_sensor(0x760c, 0x06);
	write_cmos_sensor(0x760d, 0x50);
	write_cmos_sensor(0x760e, 0x04);
	write_cmos_sensor(0x760f, 0xe4);
	write_cmos_sensor(0x7610, 0x00);
	write_cmos_sensor(0x7611, 0x00);
	write_cmos_sensor(0x7612, 0x06);
	write_cmos_sensor(0x7613, 0x5c);
	write_cmos_sensor(0x7614, 0x00);
	write_cmos_sensor(0x7615, 0x0f);
	write_cmos_sensor(0x7616, 0x06);
	write_cmos_sensor(0x7617, 0x1c);
	write_cmos_sensor(0x7618, 0x00);
	write_cmos_sensor(0x7619, 0x02);
	write_cmos_sensor(0x761a, 0x06);
	write_cmos_sensor(0x761b, 0xa2);
	write_cmos_sensor(0x761c, 0x00);
	write_cmos_sensor(0x761d, 0x01);
	write_cmos_sensor(0x761e, 0x06);
	write_cmos_sensor(0x761f, 0xae);
	write_cmos_sensor(0x7620, 0x00);
	write_cmos_sensor(0x7621, 0x0e);
	write_cmos_sensor(0x7622, 0x05);
	write_cmos_sensor(0x7623, 0x30);
	write_cmos_sensor(0x7624, 0x07);
	write_cmos_sensor(0x7625, 0x00);
	write_cmos_sensor(0x7626, 0x0f);
	write_cmos_sensor(0x7627, 0x00);
	write_cmos_sensor(0x7628, 0x04);
	write_cmos_sensor(0x7629, 0xe5);
	write_cmos_sensor(0x762a, 0x05);
	write_cmos_sensor(0x762b, 0x33);
	write_cmos_sensor(0x762c, 0x06);
	write_cmos_sensor(0x762d, 0x12);
	write_cmos_sensor(0x762e, 0x00);
	write_cmos_sensor(0x762f, 0x01);
	write_cmos_sensor(0x7630, 0x06);
	write_cmos_sensor(0x7631, 0x52);
	write_cmos_sensor(0x7632, 0x00);
	write_cmos_sensor(0x7633, 0x01);
	write_cmos_sensor(0x7634, 0x06);
	write_cmos_sensor(0x7635, 0x5e);
	write_cmos_sensor(0x7636, 0x04);
	write_cmos_sensor(0x7637, 0xe4);
	write_cmos_sensor(0x7638, 0x00);
	write_cmos_sensor(0x7639, 0x01);
	write_cmos_sensor(0x763a, 0x05);
	write_cmos_sensor(0x763b, 0x30);
	write_cmos_sensor(0x763c, 0x0f);
	write_cmos_sensor(0x763d, 0x00);
	write_cmos_sensor(0x763e, 0x06);
	write_cmos_sensor(0x763f, 0xa6);
	write_cmos_sensor(0x7640, 0x00);
	write_cmos_sensor(0x7641, 0x02);
	write_cmos_sensor(0x7642, 0x06);
	write_cmos_sensor(0x7643, 0x26);
	write_cmos_sensor(0x7644, 0x00);
	write_cmos_sensor(0x7645, 0x02);
	write_cmos_sensor(0x7646, 0x05);
	write_cmos_sensor(0x7647, 0x33);
	write_cmos_sensor(0x7648, 0x06);
	write_cmos_sensor(0x7649, 0x20);
	write_cmos_sensor(0x764a, 0x0f);
	write_cmos_sensor(0x764b, 0x00);
	write_cmos_sensor(0x764c, 0x06);
	write_cmos_sensor(0x764d, 0x56);
	write_cmos_sensor(0x764e, 0x00);
	write_cmos_sensor(0x764f, 0x02);
	write_cmos_sensor(0x7650, 0x06);
	write_cmos_sensor(0x7651, 0x16);
	write_cmos_sensor(0x7652, 0x05);
	write_cmos_sensor(0x7653, 0x33);
	write_cmos_sensor(0x7654, 0x06);
	write_cmos_sensor(0x7655, 0x10);
	write_cmos_sensor(0x7656, 0x0f);
	write_cmos_sensor(0x7657, 0x00);
	write_cmos_sensor(0x7658, 0x06);
	write_cmos_sensor(0x7659, 0x10);
	write_cmos_sensor(0x765a, 0x0f);
	write_cmos_sensor(0x765b, 0x00);
	write_cmos_sensor(0x765c, 0x06);
	write_cmos_sensor(0x765d, 0x20);
	write_cmos_sensor(0x765e, 0x0f);
	write_cmos_sensor(0x765f, 0x00);
	write_cmos_sensor(0x7660, 0x00);
	write_cmos_sensor(0x7661, 0x00);
	write_cmos_sensor(0x7662, 0x00);
	write_cmos_sensor(0x7663, 0x02);
	write_cmos_sensor(0x7664, 0x04);
	write_cmos_sensor(0x7665, 0xe5);
	write_cmos_sensor(0x7666, 0x04);
	write_cmos_sensor(0x7667, 0xe4);
	write_cmos_sensor(0x7668, 0x0f);
	write_cmos_sensor(0x7669, 0x00);
	write_cmos_sensor(0x766a, 0x00);
	write_cmos_sensor(0x766b, 0x00);
	write_cmos_sensor(0x766c, 0x00);
	write_cmos_sensor(0x766d, 0x01);
	write_cmos_sensor(0x766e, 0x04);
	write_cmos_sensor(0x766f, 0xe5);
	write_cmos_sensor(0x7670, 0x04);
	write_cmos_sensor(0x7671, 0xe4);
	write_cmos_sensor(0x7672, 0x0f);
	write_cmos_sensor(0x7673, 0x00);
	write_cmos_sensor(0x7674, 0x00);
	write_cmos_sensor(0x7675, 0x02);
	write_cmos_sensor(0x7676, 0x04);
	write_cmos_sensor(0x7677, 0xe4);
	write_cmos_sensor(0x7678, 0x00);
	write_cmos_sensor(0x7679, 0x02);
	write_cmos_sensor(0x767a, 0x04);
	write_cmos_sensor(0x767b, 0xc4);
	write_cmos_sensor(0x767c, 0x00);
	write_cmos_sensor(0x767d, 0x02);
	write_cmos_sensor(0x767e, 0x04);
	write_cmos_sensor(0x767f, 0xc4);
	write_cmos_sensor(0x7680, 0x05);
	write_cmos_sensor(0x7681, 0x83);
	write_cmos_sensor(0x7682, 0x0f);
	write_cmos_sensor(0x7683, 0x00);
	write_cmos_sensor(0x7684, 0x00);
	write_cmos_sensor(0x7685, 0x02);
	write_cmos_sensor(0x7686, 0x04);
	write_cmos_sensor(0x7687, 0xe4);
	write_cmos_sensor(0x7688, 0x00);
	write_cmos_sensor(0x7689, 0x02);
	write_cmos_sensor(0x768a, 0x04);
	write_cmos_sensor(0x768b, 0xc4);
	write_cmos_sensor(0x768c, 0x00);
	write_cmos_sensor(0x768d, 0x02);
	write_cmos_sensor(0x768e, 0x04);
	write_cmos_sensor(0x768f, 0xc4);
	write_cmos_sensor(0x7690, 0x05);
	write_cmos_sensor(0x7691, 0x83);
	write_cmos_sensor(0x7692, 0x03);
	write_cmos_sensor(0x7693, 0x0b);
	write_cmos_sensor(0x7694, 0x05);
	write_cmos_sensor(0x7695, 0x83);
	write_cmos_sensor(0x7696, 0x00);
	write_cmos_sensor(0x7697, 0x07);
	write_cmos_sensor(0x7698, 0x05);
	write_cmos_sensor(0x7699, 0x03);
	write_cmos_sensor(0x769a, 0x00);
	write_cmos_sensor(0x769b, 0x05);
	write_cmos_sensor(0x769c, 0x05);
	write_cmos_sensor(0x769d, 0x32);
	write_cmos_sensor(0x769e, 0x05);
	write_cmos_sensor(0x769f, 0x30);
	write_cmos_sensor(0x76a0, 0x00);
	write_cmos_sensor(0x76a1, 0x02);
	write_cmos_sensor(0x76a2, 0x05);
	write_cmos_sensor(0x76a3, 0x78);
	write_cmos_sensor(0x76a4, 0x00);
	write_cmos_sensor(0x76a5, 0x01);
	write_cmos_sensor(0x76a6, 0x05);
	write_cmos_sensor(0x76a7, 0x7c);
	write_cmos_sensor(0x76a8, 0x03);
	write_cmos_sensor(0x76a9, 0x9a);
	write_cmos_sensor(0x76aa, 0x05);
	write_cmos_sensor(0x76ab, 0x83);
	write_cmos_sensor(0x76ac, 0x00);
	write_cmos_sensor(0x76ad, 0x04);
	write_cmos_sensor(0x76ae, 0x05);
	write_cmos_sensor(0x76af, 0x03);
	write_cmos_sensor(0x76b0, 0x00);
	write_cmos_sensor(0x76b1, 0x03);
	write_cmos_sensor(0x76b2, 0x05);
	write_cmos_sensor(0x76b3, 0x32);
	write_cmos_sensor(0x76b4, 0x05);
	write_cmos_sensor(0x76b5, 0x30);
	write_cmos_sensor(0x76b6, 0x00);
	write_cmos_sensor(0x76b7, 0x02);
	write_cmos_sensor(0x76b8, 0x05);
	write_cmos_sensor(0x76b9, 0x78);
	write_cmos_sensor(0x76ba, 0x00);
	write_cmos_sensor(0x76bb, 0x01);
	write_cmos_sensor(0x76bc, 0x05);
	write_cmos_sensor(0x76bd, 0x7c);
	write_cmos_sensor(0x76be, 0x03);
	write_cmos_sensor(0x76bf, 0x99);
	write_cmos_sensor(0x76c0, 0x05);
	write_cmos_sensor(0x76c1, 0x83);
	write_cmos_sensor(0x76c2, 0x00);
	write_cmos_sensor(0x76c3, 0x03);
	write_cmos_sensor(0x76c4, 0x05);
	write_cmos_sensor(0x76c5, 0x03);
	write_cmos_sensor(0x76c6, 0x00);
	write_cmos_sensor(0x76c7, 0x01);
	write_cmos_sensor(0x76c8, 0x05);
	write_cmos_sensor(0x76c9, 0x32);
	write_cmos_sensor(0x76ca, 0x05);
	write_cmos_sensor(0x76cb, 0x30);
	write_cmos_sensor(0x76cc, 0x00);
	write_cmos_sensor(0x76cd, 0x02);
	write_cmos_sensor(0x76ce, 0x05);
	write_cmos_sensor(0x76cf, 0x78);
	write_cmos_sensor(0x76d0, 0x00);
	write_cmos_sensor(0x76d1, 0x01);
	write_cmos_sensor(0x76d2, 0x05);
	write_cmos_sensor(0x76d3, 0x7c);
	write_cmos_sensor(0x76d4, 0x03);
	write_cmos_sensor(0x76d5, 0x98);
	write_cmos_sensor(0x76d6, 0x05);
	write_cmos_sensor(0x76d7, 0x83);
	write_cmos_sensor(0x76d8, 0x00);
	write_cmos_sensor(0x76d9, 0x00);
	write_cmos_sensor(0x76da, 0x05);
	write_cmos_sensor(0x76db, 0x03);
	write_cmos_sensor(0x76dc, 0x00);
	write_cmos_sensor(0x76dd, 0x01);
	write_cmos_sensor(0x76de, 0x05);
	write_cmos_sensor(0x76df, 0x32);
	write_cmos_sensor(0x76e0, 0x05);
	write_cmos_sensor(0x76e1, 0x30);
	write_cmos_sensor(0x76e2, 0x00);
	write_cmos_sensor(0x76e3, 0x02);
	write_cmos_sensor(0x76e4, 0x05);
	write_cmos_sensor(0x76e5, 0x78);
	write_cmos_sensor(0x76e6, 0x00);
	write_cmos_sensor(0x76e7, 0x01);
	write_cmos_sensor(0x76e8, 0x05);
	write_cmos_sensor(0x76e9, 0x7c);
	write_cmos_sensor(0x76ea, 0x03);
	write_cmos_sensor(0x76eb, 0x97);
	write_cmos_sensor(0x76ec, 0x05);
	write_cmos_sensor(0x76ed, 0x83);
	write_cmos_sensor(0x76ee, 0x00);
	write_cmos_sensor(0x76ef, 0x00);
	write_cmos_sensor(0x76f0, 0x05);
	write_cmos_sensor(0x76f1, 0x03);
	write_cmos_sensor(0x76f2, 0x05);
	write_cmos_sensor(0x76f3, 0x32);
	write_cmos_sensor(0x76f4, 0x05);
	write_cmos_sensor(0x76f5, 0x30);
	write_cmos_sensor(0x76f6, 0x00);
	write_cmos_sensor(0x76f7, 0x02);
	write_cmos_sensor(0x76f8, 0x05);
	write_cmos_sensor(0x76f9, 0x78);
	write_cmos_sensor(0x76fa, 0x00);
	write_cmos_sensor(0x76fb, 0x01);
	write_cmos_sensor(0x76fc, 0x05);
	write_cmos_sensor(0x76fd, 0x7c);
	write_cmos_sensor(0x76fe, 0x03);
	write_cmos_sensor(0x76ff, 0x96);
	write_cmos_sensor(0x7700, 0x05);
	write_cmos_sensor(0x7701, 0x83);
	write_cmos_sensor(0x7702, 0x05);
	write_cmos_sensor(0x7703, 0x03);
	write_cmos_sensor(0x7704, 0x05);
	write_cmos_sensor(0x7705, 0x32);
	write_cmos_sensor(0x7706, 0x05);
	write_cmos_sensor(0x7707, 0x30);
	write_cmos_sensor(0x7708, 0x00);
	write_cmos_sensor(0x7709, 0x02);
	write_cmos_sensor(0x770a, 0x05);
	write_cmos_sensor(0x770b, 0x78);
	write_cmos_sensor(0x770c, 0x00);
	write_cmos_sensor(0x770d, 0x01);
	write_cmos_sensor(0x770e, 0x05);
	write_cmos_sensor(0x770f, 0x7c);
	write_cmos_sensor(0x7710, 0x03);
	write_cmos_sensor(0x7711, 0x95);
	write_cmos_sensor(0x7712, 0x05);
	write_cmos_sensor(0x7713, 0x83);
	write_cmos_sensor(0x7714, 0x05);
	write_cmos_sensor(0x7715, 0x03);
	write_cmos_sensor(0x7716, 0x05);
	write_cmos_sensor(0x7717, 0x32);
	write_cmos_sensor(0x7718, 0x05);
	write_cmos_sensor(0x7719, 0x30);
	write_cmos_sensor(0x771a, 0x00);
	write_cmos_sensor(0x771b, 0x02);
	write_cmos_sensor(0x771c, 0x05);
	write_cmos_sensor(0x771d, 0x78);
	write_cmos_sensor(0x771e, 0x00);
	write_cmos_sensor(0x771f, 0x01);
	write_cmos_sensor(0x7720, 0x05);
	write_cmos_sensor(0x7721, 0x7c);
	write_cmos_sensor(0x7722, 0x03);
	write_cmos_sensor(0x7723, 0x94);
	write_cmos_sensor(0x7724, 0x05);
	write_cmos_sensor(0x7725, 0x83);
	write_cmos_sensor(0x7726, 0x00);
	write_cmos_sensor(0x7727, 0x01);
	write_cmos_sensor(0x7728, 0x05);
	write_cmos_sensor(0x7729, 0x03);
	write_cmos_sensor(0x772a, 0x00);
	write_cmos_sensor(0x772b, 0x01);
	write_cmos_sensor(0x772c, 0x05);
	write_cmos_sensor(0x772d, 0x32);
	write_cmos_sensor(0x772e, 0x05);
	write_cmos_sensor(0x772f, 0x30);
	write_cmos_sensor(0x7730, 0x00);
	write_cmos_sensor(0x7731, 0x02);
	write_cmos_sensor(0x7732, 0x05);
	write_cmos_sensor(0x7733, 0x78);
	write_cmos_sensor(0x7734, 0x00);
	write_cmos_sensor(0x7735, 0x01);
	write_cmos_sensor(0x7736, 0x05);
	write_cmos_sensor(0x7737, 0x7c);
	write_cmos_sensor(0x7738, 0x03);
	write_cmos_sensor(0x7739, 0x93);
	write_cmos_sensor(0x773a, 0x05);
	write_cmos_sensor(0x773b, 0x83);
	write_cmos_sensor(0x773c, 0x00);
	write_cmos_sensor(0x773d, 0x00);
	write_cmos_sensor(0x773e, 0x05);
	write_cmos_sensor(0x773f, 0x03);
	write_cmos_sensor(0x7740, 0x00);
	write_cmos_sensor(0x7741, 0x00);
	write_cmos_sensor(0x7742, 0x05);
	write_cmos_sensor(0x7743, 0x32);
	write_cmos_sensor(0x7744, 0x05);
	write_cmos_sensor(0x7745, 0x30);
	write_cmos_sensor(0x7746, 0x00);
	write_cmos_sensor(0x7747, 0x02);
	write_cmos_sensor(0x7748, 0x05);
	write_cmos_sensor(0x7749, 0x78);
	write_cmos_sensor(0x774a, 0x00);
	write_cmos_sensor(0x774b, 0x01);
	write_cmos_sensor(0x774c, 0x05);
	write_cmos_sensor(0x774d, 0x7c);
	write_cmos_sensor(0x774e, 0x03);
	write_cmos_sensor(0x774f, 0x92);
	write_cmos_sensor(0x7750, 0x05);
	write_cmos_sensor(0x7751, 0x83);
	write_cmos_sensor(0x7752, 0x05);
	write_cmos_sensor(0x7753, 0x03);
	write_cmos_sensor(0x7754, 0x00);
	write_cmos_sensor(0x7755, 0x00);
	write_cmos_sensor(0x7756, 0x05);
	write_cmos_sensor(0x7757, 0x32);
	write_cmos_sensor(0x7758, 0x05);
	write_cmos_sensor(0x7759, 0x30);
	write_cmos_sensor(0x775a, 0x00);
	write_cmos_sensor(0x775b, 0x02);
	write_cmos_sensor(0x775c, 0x05);
	write_cmos_sensor(0x775d, 0x78);
	write_cmos_sensor(0x775e, 0x00);
	write_cmos_sensor(0x775f, 0x01);
	write_cmos_sensor(0x7760, 0x05);
	write_cmos_sensor(0x7761, 0x7c);
	write_cmos_sensor(0x7762, 0x03);
	write_cmos_sensor(0x7763, 0x91);
	write_cmos_sensor(0x7764, 0x05);
	write_cmos_sensor(0x7765, 0x83);
	write_cmos_sensor(0x7766, 0x05);
	write_cmos_sensor(0x7767, 0x03);
	write_cmos_sensor(0x7768, 0x05);
	write_cmos_sensor(0x7769, 0x32);
	write_cmos_sensor(0x776a, 0x05);
	write_cmos_sensor(0x776b, 0x30);
	write_cmos_sensor(0x776c, 0x00);
	write_cmos_sensor(0x776d, 0x02);
	write_cmos_sensor(0x776e, 0x05);
	write_cmos_sensor(0x776f, 0x78);
	write_cmos_sensor(0x7770, 0x00);
	write_cmos_sensor(0x7771, 0x01);
	write_cmos_sensor(0x7772, 0x05);
	write_cmos_sensor(0x7773, 0x7c);
	write_cmos_sensor(0x7774, 0x03);
	write_cmos_sensor(0x7775, 0x90);
	write_cmos_sensor(0x7776, 0x05);
	write_cmos_sensor(0x7777, 0x83);
	write_cmos_sensor(0x7778, 0x05);
	write_cmos_sensor(0x7779, 0x03);
	write_cmos_sensor(0x777a, 0x05);
	write_cmos_sensor(0x777b, 0x32);
	write_cmos_sensor(0x777c, 0x05);
	write_cmos_sensor(0x777d, 0x30);
	write_cmos_sensor(0x777e, 0x00);
	write_cmos_sensor(0x777f, 0x02);
	write_cmos_sensor(0x7780, 0x05);
	write_cmos_sensor(0x7781, 0x78);
	write_cmos_sensor(0x7782, 0x00);
	write_cmos_sensor(0x7783, 0x01);
	write_cmos_sensor(0x7784, 0x05);
	write_cmos_sensor(0x7785, 0x7c);
	write_cmos_sensor(0x7786, 0x02);
	write_cmos_sensor(0x7787, 0x90);
	write_cmos_sensor(0x7788, 0x05);
	write_cmos_sensor(0x7789, 0x03);
	write_cmos_sensor(0x778a, 0x07);
	write_cmos_sensor(0x778b, 0x00);
	write_cmos_sensor(0x778c, 0x0f);
	write_cmos_sensor(0x778d, 0x00);
	write_cmos_sensor(0x778e, 0x08);
	write_cmos_sensor(0x778f, 0x30);
	write_cmos_sensor(0x7790, 0x08);
	write_cmos_sensor(0x7791, 0xee);
	write_cmos_sensor(0x7792, 0x0f);
	write_cmos_sensor(0x7793, 0x00);
	write_cmos_sensor(0x7794, 0x05);
	write_cmos_sensor(0x7795, 0x33);
	write_cmos_sensor(0x7796, 0x04);
	write_cmos_sensor(0x7797, 0xe5);
	write_cmos_sensor(0x7798, 0x06);
	write_cmos_sensor(0x7799, 0x52);
	write_cmos_sensor(0x779a, 0x04);
	write_cmos_sensor(0x779b, 0xe4);
	write_cmos_sensor(0x779c, 0x00);
	write_cmos_sensor(0x779d, 0x00);
	write_cmos_sensor(0x779e, 0x06);
	write_cmos_sensor(0x779f, 0x5e);
	write_cmos_sensor(0x77a0, 0x00);
	write_cmos_sensor(0x77a1, 0x0f);
	write_cmos_sensor(0x77a2, 0x06);
	write_cmos_sensor(0x77a3, 0x1e);
	write_cmos_sensor(0x77a4, 0x00);
	write_cmos_sensor(0x77a5, 0x02);
	write_cmos_sensor(0x77a6, 0x06);
	write_cmos_sensor(0x77a7, 0xa2);
	write_cmos_sensor(0x77a8, 0x00);
	write_cmos_sensor(0x77a9, 0x01);
	write_cmos_sensor(0x77aa, 0x06);
	write_cmos_sensor(0x77ab, 0xae);
	write_cmos_sensor(0x77ac, 0x00);
	write_cmos_sensor(0x77ad, 0x03);
	write_cmos_sensor(0x77ae, 0x05);
	write_cmos_sensor(0x77af, 0x30);
	write_cmos_sensor(0x77b0, 0x09);
	write_cmos_sensor(0x77b1, 0x19);
	write_cmos_sensor(0x77b2, 0x0f);
	write_cmos_sensor(0x77b3, 0x00);
	write_cmos_sensor(0x77b4, 0x05);
	write_cmos_sensor(0x77b5, 0x33);
	write_cmos_sensor(0x77b6, 0x04);
	write_cmos_sensor(0x77b7, 0xe5);
	write_cmos_sensor(0x77b8, 0x06);
	write_cmos_sensor(0x77b9, 0x52);
	write_cmos_sensor(0x77ba, 0x04);
	write_cmos_sensor(0x77bb, 0xe4);
	write_cmos_sensor(0x77bc, 0x00);
	write_cmos_sensor(0x77bd, 0x00);
	write_cmos_sensor(0x77be, 0x06);
	write_cmos_sensor(0x77bf, 0x5e);
	write_cmos_sensor(0x77c0, 0x00);
	write_cmos_sensor(0x77c1, 0x0f);
	write_cmos_sensor(0x77c2, 0x06);
	write_cmos_sensor(0x77c3, 0x1e);
	write_cmos_sensor(0x77c4, 0x00);
	write_cmos_sensor(0x77c5, 0x02);
	write_cmos_sensor(0x77c6, 0x06);
	write_cmos_sensor(0x77c7, 0xa2);
	write_cmos_sensor(0x77c8, 0x00);
	write_cmos_sensor(0x77c9, 0x01);
	write_cmos_sensor(0x77ca, 0x06);
	write_cmos_sensor(0x77cb, 0xae);
	write_cmos_sensor(0x77cc, 0x00);
	write_cmos_sensor(0x77cd, 0x03);
	write_cmos_sensor(0x77ce, 0x05);
	write_cmos_sensor(0x77cf, 0x30);
	write_cmos_sensor(0x77d0, 0x0f);
	write_cmos_sensor(0x77d1, 0x00);
	write_cmos_sensor(0x77d2, 0x00);
	write_cmos_sensor(0x77d3, 0x00);
	write_cmos_sensor(0x77d4, 0x00);
	write_cmos_sensor(0x77d5, 0x02);
	write_cmos_sensor(0x77d6, 0x04);
	write_cmos_sensor(0x77d7, 0xe5);
	write_cmos_sensor(0x77d8, 0x04);
	write_cmos_sensor(0x77d9, 0xe4);
	write_cmos_sensor(0x77da, 0x05);
	write_cmos_sensor(0x77db, 0x33);
	write_cmos_sensor(0x77dc, 0x07);
	write_cmos_sensor(0x77dd, 0x10);
	write_cmos_sensor(0x77de, 0x00);
	write_cmos_sensor(0x77df, 0x00);
	write_cmos_sensor(0x77e0, 0x01);
	write_cmos_sensor(0x77e1, 0xbb);
	write_cmos_sensor(0x77e2, 0x00);
	write_cmos_sensor(0x77e3, 0x00);
	write_cmos_sensor(0x77e4, 0x01);
	write_cmos_sensor(0x77e5, 0xaa);
	write_cmos_sensor(0x77e6, 0x00);
	write_cmos_sensor(0x77e7, 0x00);
	write_cmos_sensor(0x77e8, 0x01);
	write_cmos_sensor(0x77e9, 0x99);
	write_cmos_sensor(0x77ea, 0x00);
	write_cmos_sensor(0x77eb, 0x00);
	write_cmos_sensor(0x77ec, 0x01);
	write_cmos_sensor(0x77ed, 0x88);
	write_cmos_sensor(0x77ee, 0x00);
	write_cmos_sensor(0x77ef, 0x00);
	write_cmos_sensor(0x77f0, 0x01);
	write_cmos_sensor(0x77f1, 0x77);
	write_cmos_sensor(0x77f2, 0x00);
	write_cmos_sensor(0x77f3, 0x00);
	write_cmos_sensor(0x77f4, 0x01);
	write_cmos_sensor(0x77f5, 0x66);
	write_cmos_sensor(0x77f6, 0x00);
	write_cmos_sensor(0x77f7, 0x00);
	write_cmos_sensor(0x77f8, 0x01);
	write_cmos_sensor(0x77f9, 0x55);
	write_cmos_sensor(0x77fa, 0x00);
	write_cmos_sensor(0x77fb, 0x00);
	write_cmos_sensor(0x77fc, 0x01);
	write_cmos_sensor(0x77fd, 0x44);
	write_cmos_sensor(0x77fe, 0x00);
	write_cmos_sensor(0x77ff, 0x00);
	write_cmos_sensor(0x7800, 0x01);
	write_cmos_sensor(0x7801, 0x33);
	write_cmos_sensor(0x7802, 0x00);
	write_cmos_sensor(0x7803, 0x00);
	write_cmos_sensor(0x7804, 0x01);
	write_cmos_sensor(0x7805, 0x22);
	write_cmos_sensor(0x7806, 0x00);
	write_cmos_sensor(0x7807, 0x00);
	write_cmos_sensor(0x7808, 0x01);
	write_cmos_sensor(0x7809, 0x11);
	write_cmos_sensor(0x780a, 0x00);
	write_cmos_sensor(0x780b, 0x00);
	write_cmos_sensor(0x780c, 0x01);
	write_cmos_sensor(0x780d, 0x00);
	write_cmos_sensor(0x780e, 0x01);
	write_cmos_sensor(0x780f, 0xff);
	write_cmos_sensor(0x7810, 0x07);
	write_cmos_sensor(0x7811, 0x00);
	write_cmos_sensor(0x7812, 0x02);
	write_cmos_sensor(0x7813, 0xa0);
	write_cmos_sensor(0x7814, 0x0f);
	write_cmos_sensor(0x7815, 0x00);
	write_cmos_sensor(0x7816, 0x08);
	write_cmos_sensor(0x7817, 0x35);
	write_cmos_sensor(0x7818, 0x06);
	write_cmos_sensor(0x7819, 0x52);
	write_cmos_sensor(0x781a, 0x04);
	write_cmos_sensor(0x781b, 0xe4);
	write_cmos_sensor(0x781c, 0x00);
	write_cmos_sensor(0x781d, 0x00);
	write_cmos_sensor(0x781e, 0x06);
	write_cmos_sensor(0x781f, 0x5e);
	write_cmos_sensor(0x7820, 0x05);
	write_cmos_sensor(0x7821, 0x33);
	write_cmos_sensor(0x7822, 0x09);
	write_cmos_sensor(0x7823, 0x19);
	write_cmos_sensor(0x7824, 0x06);
	write_cmos_sensor(0x7825, 0x1e);
	write_cmos_sensor(0x7826, 0x05);
	write_cmos_sensor(0x7827, 0x33);
	write_cmos_sensor(0x7828, 0x00);
	write_cmos_sensor(0x7829, 0x01);
	write_cmos_sensor(0x782a, 0x06);
	write_cmos_sensor(0x782b, 0x24);
	write_cmos_sensor(0x782c, 0x06);
	write_cmos_sensor(0x782d, 0x20);
	write_cmos_sensor(0x782e, 0x0f);
	write_cmos_sensor(0x782f, 0x00);
	write_cmos_sensor(0x7830, 0x08);
	write_cmos_sensor(0x7831, 0x35);
	write_cmos_sensor(0x7832, 0x07);
	write_cmos_sensor(0x7833, 0x10);
	write_cmos_sensor(0x7834, 0x00);
	write_cmos_sensor(0x7835, 0x00);
	write_cmos_sensor(0x7836, 0x01);
	write_cmos_sensor(0x7837, 0xbb);
	write_cmos_sensor(0x7838, 0x00);
	write_cmos_sensor(0x7839, 0x00);
	write_cmos_sensor(0x783a, 0x01);
	write_cmos_sensor(0x783b, 0xaa);
	write_cmos_sensor(0x783c, 0x00);
	write_cmos_sensor(0x783d, 0x00);
	write_cmos_sensor(0x783e, 0x01);
	write_cmos_sensor(0x783f, 0x99);
	write_cmos_sensor(0x7840, 0x00);
	write_cmos_sensor(0x7841, 0x00);
	write_cmos_sensor(0x7842, 0x01);
	write_cmos_sensor(0x7843, 0x88);
	write_cmos_sensor(0x7844, 0x00);
	write_cmos_sensor(0x7845, 0x00);
	write_cmos_sensor(0x7846, 0x01);
	write_cmos_sensor(0x7847, 0x77);
	write_cmos_sensor(0x7848, 0x00);
	write_cmos_sensor(0x7849, 0x00);
	write_cmos_sensor(0x784a, 0x01);
	write_cmos_sensor(0x784b, 0x66);
	write_cmos_sensor(0x784c, 0x00);
	write_cmos_sensor(0x784d, 0x00);
	write_cmos_sensor(0x784e, 0x01);
	write_cmos_sensor(0x784f, 0x55);
	write_cmos_sensor(0x7850, 0x00);
	write_cmos_sensor(0x7851, 0x00);
	write_cmos_sensor(0x7852, 0x01);
	write_cmos_sensor(0x7853, 0x44);
	write_cmos_sensor(0x7854, 0x00);
	write_cmos_sensor(0x7855, 0x00);
	write_cmos_sensor(0x7856, 0x01);
	write_cmos_sensor(0x7857, 0x33);
	write_cmos_sensor(0x7858, 0x00);
	write_cmos_sensor(0x7859, 0x00);
	write_cmos_sensor(0x785a, 0x01);
	write_cmos_sensor(0x785b, 0x22);
	write_cmos_sensor(0x785c, 0x00);
	write_cmos_sensor(0x785d, 0x00);
	write_cmos_sensor(0x785e, 0x01);
	write_cmos_sensor(0x785f, 0x11);
	write_cmos_sensor(0x7860, 0x00);
	write_cmos_sensor(0x7861, 0x00);
	write_cmos_sensor(0x7862, 0x01);
	write_cmos_sensor(0x7863, 0x00);
	write_cmos_sensor(0x7864, 0x07);
	write_cmos_sensor(0x7865, 0x00);
	write_cmos_sensor(0x7866, 0x01);
	write_cmos_sensor(0x7867, 0xff);
	write_cmos_sensor(0x7868, 0x02);
	write_cmos_sensor(0x7869, 0xa0);
	write_cmos_sensor(0x786a, 0x0f);
	write_cmos_sensor(0x786b, 0x00);
	write_cmos_sensor(0x786c, 0x08);
	write_cmos_sensor(0x786d, 0x3a);
	write_cmos_sensor(0x786e, 0x08);
	write_cmos_sensor(0x786f, 0x6a);
	write_cmos_sensor(0x7870, 0x0f);
	write_cmos_sensor(0x7871, 0x00);
	write_cmos_sensor(0x7872, 0x04);
	write_cmos_sensor(0x7873, 0xc0);
	write_cmos_sensor(0x7874, 0x09);
	write_cmos_sensor(0x7875, 0x19);
	write_cmos_sensor(0x7876, 0x04);
	write_cmos_sensor(0x7877, 0x99);
	write_cmos_sensor(0x7878, 0x07);
	write_cmos_sensor(0x7879, 0x14);
	write_cmos_sensor(0x787a, 0x00);
	write_cmos_sensor(0x787b, 0x01);
	write_cmos_sensor(0x787c, 0x04);
	write_cmos_sensor(0x787d, 0xa4);
	write_cmos_sensor(0x787e, 0x00);
	write_cmos_sensor(0x787f, 0x07);
	write_cmos_sensor(0x7880, 0x04);
	write_cmos_sensor(0x7881, 0xa6);
	write_cmos_sensor(0x7882, 0x00);
	write_cmos_sensor(0x7883, 0x00);
	write_cmos_sensor(0x7884, 0x04);
	write_cmos_sensor(0x7885, 0xa0);
	write_cmos_sensor(0x7886, 0x04);
	write_cmos_sensor(0x7887, 0x80);
	write_cmos_sensor(0x7888, 0x04);
	write_cmos_sensor(0x7889, 0x00);
	write_cmos_sensor(0x788a, 0x05);
	write_cmos_sensor(0x788b, 0x03);
	write_cmos_sensor(0x788c, 0x06);
	write_cmos_sensor(0x788d, 0x00);
	write_cmos_sensor(0x788e, 0x0f);
	write_cmos_sensor(0x788f, 0x00);
	write_cmos_sensor(0x7890, 0x0f);
	write_cmos_sensor(0x7891, 0x00);
	write_cmos_sensor(0x7892, 0x0f);
	write_cmos_sensor(0x7893, 0x00);
	write_cmos_sensor(0x3000, 0x05);
	write_cmos_sensor(0x3001, 0x5c);
	write_cmos_sensor(0x3002, 0x07);
	write_cmos_sensor(0x3003, 0x01);
	write_cmos_sensor(0x3004, 0x06);
	write_cmos_sensor(0x3005, 0x43);
	write_cmos_sensor(0x3006, 0x00);
	write_cmos_sensor(0x3007, 0x07);
	write_cmos_sensor(0x3008, 0x01);
	write_cmos_sensor(0x30a0, 0x00);
	write_cmos_sensor(0x30a1, 0x04);
	write_cmos_sensor(0x30a2, 0x00);
	write_cmos_sensor(0x30a3, 0x04);
	write_cmos_sensor(0x30a4, 0x07);
	write_cmos_sensor(0x30a5, 0x8b);
	write_cmos_sensor(0x30a6, 0x04);
	write_cmos_sensor(0x30a7, 0x43);
	write_cmos_sensor(0x30a8, 0x00);
	write_cmos_sensor(0x30a9, 0x05);
	write_cmos_sensor(0x30aa, 0x00);
	write_cmos_sensor(0x30ab, 0x04);
	write_cmos_sensor(0x30ac, 0x07);
	write_cmos_sensor(0x30ad, 0x80);
	write_cmos_sensor(0x30ae, 0x04);
	write_cmos_sensor(0x30af, 0x38);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0xa4);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x6d);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x0a);
	write_cmos_sensor(0x3195, 0x12);
	write_cmos_sensor(0x31e3, 0x01);
	write_cmos_sensor(0x31e4, 0x08);
	write_cmos_sensor(0x33e2, 0x04);
	write_cmos_sensor(0x33e3, 0x02);
	write_cmos_sensor(0x33e4, 0x01);
	write_cmos_sensor(0x33e5, 0x01);
	write_cmos_sensor(0x33e8, 0x0c);
	write_cmos_sensor(0x33e9, 0x02);
	write_cmos_sensor(0x33ea, 0x02);
	write_cmos_sensor(0x33eb, 0x02);
	write_cmos_sensor(0x33ec, 0x03);
	write_cmos_sensor(0x33ed, 0x02);
	write_cmos_sensor(0x33ee, 0x05);
	write_cmos_sensor(0x33ef, 0x0a);
	write_cmos_sensor(0x33f7, 0x02);
	write_cmos_sensor(0x33f8, 0x01);
	write_cmos_sensor(0x33f9, 0x01);
	write_cmos_sensor(0x33fa, 0x01);
	write_cmos_sensor(0x33fd, 0x0c);
	write_cmos_sensor(0x33fe, 0x02);
	write_cmos_sensor(0x33ff, 0x02);
	write_cmos_sensor(0x3400, 0x02);
	write_cmos_sensor(0x3401, 0x03);
	write_cmos_sensor(0x3402, 0x01);
	write_cmos_sensor(0x3403, 0x02);
	write_cmos_sensor(0x3404, 0x08);
	write_cmos_sensor(0x340c, 0x04);
	write_cmos_sensor(0x340d, 0x02);
	write_cmos_sensor(0x340e, 0x01);
	write_cmos_sensor(0x340f, 0x01);
	write_cmos_sensor(0x3412, 0x0c);
	write_cmos_sensor(0x3413, 0x02);
	write_cmos_sensor(0x3414, 0x02);
	write_cmos_sensor(0x3415, 0x02);
	write_cmos_sensor(0x3416, 0x03);
	write_cmos_sensor(0x3417, 0x02);
	write_cmos_sensor(0x3418, 0x05);
	write_cmos_sensor(0x3419, 0x0a);
	write_cmos_sensor(0x3250, 0xf7);
	write_cmos_sensor(0x3288, 0x2a);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x15);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x0a);
	write_cmos_sensor(0x328d, 0x80);
	write_cmos_sensor(0x328e, 0x05);
	write_cmos_sensor(0x328f, 0x40);
	write_cmos_sensor(0x3290, 0x54);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x2a);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x15);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x0a);
	write_cmos_sensor(0x3297, 0x80);
	write_cmos_sensor(0x3298, 0x7f);
	write_cmos_sensor(0x3299, 0xff);
	write_cmos_sensor(0x329a, 0x54);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x2a);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x15);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x7f);
	write_cmos_sensor(0x32a3, 0xff);
	write_cmos_sensor(0x32a4, 0x54);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x2a);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x30bb, 0x14);
	write_cmos_sensor(0x315a, 0x02);
	write_cmos_sensor(0x315b, 0x00);
	write_cmos_sensor(0x315c, 0x01);
	write_cmos_sensor(0x315d, 0x80);
	write_cmos_sensor(0x315e, 0x01);
	write_cmos_sensor(0x315f, 0x80);

	mDELAY(10);

	/*set_mirror_flip(imgsensor.mirror);*/
	cam_pr_debug("mediaserver: ov2718r2a HDR setting\n");
}

static void sensor_nhdr(void)
{
	cam_pr_debug("E\n");
#if 1

	write_cmos_sensor(0x3013, 0x01);
	mDELAY(10);

	write_cmos_sensor(0x3000, 0x05);
	write_cmos_sensor(0x3001, 0x5c);
	write_cmos_sensor(0x3002, 0x07);
	write_cmos_sensor(0x3003, 0x01);
	write_cmos_sensor(0x3004, 0x06);
	write_cmos_sensor(0x3005, 0x43);
	write_cmos_sensor(0x3006, 0x00);
	write_cmos_sensor(0x3007, 0x07);
	write_cmos_sensor(0x3008, 0x01);
	write_cmos_sensor(0x3009, 0x00);
	write_cmos_sensor(0x300c, 0x6c);
	write_cmos_sensor(0x300d, 0xe1);
	write_cmos_sensor(0x300e, 0x80);
	write_cmos_sensor(0x300f, 0x00);
	write_cmos_sensor(0x3012, 0x00);//stream off
	write_cmos_sensor(0x3013, 0x00);
	write_cmos_sensor(0x3014, 0x84);
	write_cmos_sensor(0x3015, 0x00);
	write_cmos_sensor(0x3017, 0x00);
	write_cmos_sensor(0x3018, 0x00);
	write_cmos_sensor(0x3019, 0x00);
	write_cmos_sensor(0x301a, 0x00);
	write_cmos_sensor(0x301b, 0x01);
	write_cmos_sensor(0x301e, 0x17);
	write_cmos_sensor(0x301f, 0xe1);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x3031, 0x72);
	write_cmos_sensor(0x3032, 0xf0);
	write_cmos_sensor(0x3033, 0x30);
	write_cmos_sensor(0x3034, 0x3f);
	write_cmos_sensor(0x3035, 0x5f);
	write_cmos_sensor(0x3036, 0x02);
	write_cmos_sensor(0x3037, 0x9f);
	write_cmos_sensor(0x3038, 0x04);
	write_cmos_sensor(0x3039, 0xb7);
	write_cmos_sensor(0x303a, 0x04);
	write_cmos_sensor(0x303b, 0x07);
	write_cmos_sensor(0x303c, 0xf0);
	write_cmos_sensor(0x303d, 0x00);
	write_cmos_sensor(0x303e, 0x0b);
	write_cmos_sensor(0x303f, 0xe3);
	write_cmos_sensor(0x3040, 0xf3);
	write_cmos_sensor(0x3041, 0x29);
	write_cmos_sensor(0x3042, 0xf6);
	write_cmos_sensor(0x3043, 0x65);
	write_cmos_sensor(0x3044, 0x06);
	write_cmos_sensor(0x3045, 0x0f);
	write_cmos_sensor(0x3046, 0x59);
	write_cmos_sensor(0x3047, 0x07);
	write_cmos_sensor(0x3048, 0x82);
	write_cmos_sensor(0x3049, 0xcf);
	write_cmos_sensor(0x304a, 0x12);
	write_cmos_sensor(0x304b, 0x40);
	write_cmos_sensor(0x304c, 0x33);
	write_cmos_sensor(0x304d, 0xa4);
	write_cmos_sensor(0x304e, 0x0b);
	write_cmos_sensor(0x304f, 0x3d);
	write_cmos_sensor(0x3050, 0x10);
	write_cmos_sensor(0x3060, 0x00);
	write_cmos_sensor(0x3061, 0x64);
	write_cmos_sensor(0x3062, 0x00);
	write_cmos_sensor(0x3063, 0xe4);
	write_cmos_sensor(0x3064, 0x0b);
	write_cmos_sensor(0x3065, 0x60);
	write_cmos_sensor(0x3066, 0x80);
	write_cmos_sensor(0x3080, 0x00);
	write_cmos_sensor(0x3081, 0x00);
	write_cmos_sensor(0x3082, 0x01);
	write_cmos_sensor(0x3083, 0xe3);
	write_cmos_sensor(0x3084, 0x06);
	write_cmos_sensor(0x3085, 0x00);
	write_cmos_sensor(0x3086, 0x10);
	write_cmos_sensor(0x3087, 0x10);
	write_cmos_sensor(0x3089, 0x00);
	write_cmos_sensor(0x308a, 0x01);
	write_cmos_sensor(0x3093, 0x00);
	write_cmos_sensor(0x30a0, 0x00);
	write_cmos_sensor(0x30a1, 0x04);
	write_cmos_sensor(0x30a2, 0x00);
	write_cmos_sensor(0x30a3, 0x04);
	write_cmos_sensor(0x30a4, 0x07);
	write_cmos_sensor(0x30a5, 0x8b);
	write_cmos_sensor(0x30a6, 0x04);
	write_cmos_sensor(0x30a7, 0x43);
	write_cmos_sensor(0x30a8, 0x00);
	write_cmos_sensor(0x30a9, 0x05);
	write_cmos_sensor(0x30aa, 0x00);
	write_cmos_sensor(0x30ab, 0x04);
	write_cmos_sensor(0x30ac, 0x07);
	write_cmos_sensor(0x30ad, 0x80);
	write_cmos_sensor(0x30ae, 0x04);
	write_cmos_sensor(0x30af, 0x38);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0x98);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x65);
	write_cmos_sensor(0x30b4, 0x00);
	write_cmos_sensor(0x30b5, 0x00);
	write_cmos_sensor(0x30b6, 0x00);
	write_cmos_sensor(0x30b7, 0x10);
	write_cmos_sensor(0x30b8, 0x00);
	write_cmos_sensor(0x30b9, 0x02);
	write_cmos_sensor(0x30ba, 0x10);
	write_cmos_sensor(0x30bb, 0x00);
	write_cmos_sensor(0x30bc, 0x00);
	write_cmos_sensor(0x30bd, 0x03);
	write_cmos_sensor(0x30be, 0x5c);
	write_cmos_sensor(0x30bf, 0x00);
	write_cmos_sensor(0x30c0, 0x04);
	write_cmos_sensor(0x30c1, 0x00);
	write_cmos_sensor(0x30c2, 0x20);
	write_cmos_sensor(0x30c3, 0x00);
	write_cmos_sensor(0x30c4, 0x4a);
	write_cmos_sensor(0x30c5, 0x00);
	write_cmos_sensor(0x30c7, 0x00);
	write_cmos_sensor(0x30c8, 0x00);
	write_cmos_sensor(0x30d1, 0x00);
	write_cmos_sensor(0x30d2, 0x00);
	write_cmos_sensor(0x30d3, 0x80);
	write_cmos_sensor(0x30d4, 0x00);
	write_cmos_sensor(0x30d9, 0x09);
	write_cmos_sensor(0x30da, 0x64);
	write_cmos_sensor(0x30dd, 0x00);
	write_cmos_sensor(0x30de, 0x16);
	write_cmos_sensor(0x30df, 0x00);
	write_cmos_sensor(0x30e0, 0x17);
	write_cmos_sensor(0x30e1, 0x00);
	write_cmos_sensor(0x30e2, 0x18);
	write_cmos_sensor(0x30e3, 0x10);
	write_cmos_sensor(0x30e4, 0x04);
	write_cmos_sensor(0x30e5, 0x00);
	write_cmos_sensor(0x30e6, 0x00);
	write_cmos_sensor(0x30e7, 0x00);
	write_cmos_sensor(0x30e8, 0x00);
	write_cmos_sensor(0x30e9, 0x00);
	write_cmos_sensor(0x30ea, 0x00);
	write_cmos_sensor(0x30eb, 0x00);
	write_cmos_sensor(0x30ec, 0x00);
	write_cmos_sensor(0x30ed, 0x00);
	write_cmos_sensor(0x3101, 0x00);
	write_cmos_sensor(0x3102, 0x00);
	write_cmos_sensor(0x3103, 0x00);
	write_cmos_sensor(0x3104, 0x00);
	write_cmos_sensor(0x3105, 0x8c);
	write_cmos_sensor(0x3106, 0x87);
	write_cmos_sensor(0x3107, 0xc0);
	write_cmos_sensor(0x3108, 0x9d);
	write_cmos_sensor(0x3109, 0x8d);
	write_cmos_sensor(0x310a, 0x8d);
	write_cmos_sensor(0x310b, 0x6a);
	write_cmos_sensor(0x310c, 0x3a);
	write_cmos_sensor(0x310d, 0x5a);
	write_cmos_sensor(0x310e, 0x00);
	write_cmos_sensor(0x3120, 0x00);
	write_cmos_sensor(0x3121, 0x00);
	write_cmos_sensor(0x3122, 0x00);
	write_cmos_sensor(0x3123, 0xf0);
	write_cmos_sensor(0x3124, 0x00);
	write_cmos_sensor(0x3125, 0x70);
	write_cmos_sensor(0x3126, 0x1f);
	write_cmos_sensor(0x3127, 0x0f);
	write_cmos_sensor(0x3128, 0x00);
	write_cmos_sensor(0x3129, 0x3a);
	write_cmos_sensor(0x312a, 0x02);
	write_cmos_sensor(0x312b, 0x0f);
	write_cmos_sensor(0x312c, 0x00);
	write_cmos_sensor(0x312d, 0x0f);
	write_cmos_sensor(0x312e, 0x1d);
	write_cmos_sensor(0x312f, 0x00);
	write_cmos_sensor(0x3130, 0x00);
	write_cmos_sensor(0x3131, 0x00);
	write_cmos_sensor(0x3132, 0x00);
	write_cmos_sensor(0x3140, 0x0a);
	write_cmos_sensor(0x3141, 0x03);
	write_cmos_sensor(0x3142, 0x00);
	write_cmos_sensor(0x3143, 0x00);
	write_cmos_sensor(0x3144, 0x00);
	write_cmos_sensor(0x3145, 0x00);
	write_cmos_sensor(0x3146, 0x00);
	write_cmos_sensor(0x3147, 0x00);
	write_cmos_sensor(0x3148, 0x00);
	write_cmos_sensor(0x3149, 0x00);
	write_cmos_sensor(0x314a, 0x00);
	write_cmos_sensor(0x314b, 0x00);
	write_cmos_sensor(0x314c, 0x00);
	write_cmos_sensor(0x314d, 0x00);
	write_cmos_sensor(0x314e, 0x1c);
	write_cmos_sensor(0x314f, 0xff);
	write_cmos_sensor(0x3150, 0xff);
	write_cmos_sensor(0x3151, 0xff);
	write_cmos_sensor(0x3152, 0x10);
	write_cmos_sensor(0x3153, 0x10);
	write_cmos_sensor(0x3154, 0x10);
	write_cmos_sensor(0x3155, 0x00);
	write_cmos_sensor(0x3156, 0x03);
	write_cmos_sensor(0x3157, 0x00);
	write_cmos_sensor(0x3158, 0x0f);
	write_cmos_sensor(0x3159, 0xff);
	write_cmos_sensor(0x315a, 0x01);
	write_cmos_sensor(0x315b, 0x00);
	write_cmos_sensor(0x315c, 0x01);
	write_cmos_sensor(0x315d, 0x00);
	write_cmos_sensor(0x315e, 0x01);
	write_cmos_sensor(0x315f, 0x00);
	write_cmos_sensor(0x3160, 0x00);
	write_cmos_sensor(0x3161, 0x40);
	write_cmos_sensor(0x3162, 0x00);
	write_cmos_sensor(0x3163, 0x40);
	write_cmos_sensor(0x3164, 0x00);
	write_cmos_sensor(0x3165, 0x40);
	write_cmos_sensor(0x3190, 0x38);
	write_cmos_sensor(0x3191, 0x99);
	write_cmos_sensor(0x3193, 0x08);
	write_cmos_sensor(0x3194, 0x13);
	write_cmos_sensor(0x3195, 0x33);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x10);
	write_cmos_sensor(0x3198, 0x00);
	write_cmos_sensor(0x3199, 0x7f);
	write_cmos_sensor(0x319a, 0x80);
	write_cmos_sensor(0x319b, 0xff);
	write_cmos_sensor(0x319c, 0x80);
	write_cmos_sensor(0x319d, 0xbf);
	write_cmos_sensor(0x319e, 0xc0);
	write_cmos_sensor(0x319f, 0xff);
	write_cmos_sensor(0x31a0, 0x24);
	write_cmos_sensor(0x31a1, 0x55);
	write_cmos_sensor(0x31a2, 0x00);
	write_cmos_sensor(0x31a3, 0x00);
	write_cmos_sensor(0x31a6, 0x00);
	write_cmos_sensor(0x31a7, 0x00);
	write_cmos_sensor(0x31b0, 0x00);
	write_cmos_sensor(0x31b1, 0x00);
	write_cmos_sensor(0x31b2, 0x02);
	write_cmos_sensor(0x31b3, 0x00);
	write_cmos_sensor(0x31b4, 0x00);
	write_cmos_sensor(0x31b5, 0x01);
	write_cmos_sensor(0x31b6, 0x00);
	write_cmos_sensor(0x31b7, 0x00);
	write_cmos_sensor(0x31b8, 0x00);
	write_cmos_sensor(0x31b9, 0x00);
	write_cmos_sensor(0x31ba, 0x00);
	write_cmos_sensor(0x31d0, 0x3c);
	write_cmos_sensor(0x31d1, 0x34);
	write_cmos_sensor(0x31d2, 0x3c);
	write_cmos_sensor(0x31d3, 0x00);
	write_cmos_sensor(0x31d4, 0x2d);
	write_cmos_sensor(0x31d5, 0x00);
	write_cmos_sensor(0x31d6, 0x01);
	write_cmos_sensor(0x31d7, 0x06);
	write_cmos_sensor(0x31d8, 0x00);
	write_cmos_sensor(0x31d9, 0x64);
	write_cmos_sensor(0x31da, 0x00);
	write_cmos_sensor(0x31db, 0x30);
	write_cmos_sensor(0x31dc, 0x04);
	write_cmos_sensor(0x31dd, 0x69);
	write_cmos_sensor(0x31de, 0x0a);
	write_cmos_sensor(0x31df, 0x3c);
	write_cmos_sensor(0x31e0, 0x04);
	write_cmos_sensor(0x31e1, 0x32);
	write_cmos_sensor(0x31e2, 0x00);
	write_cmos_sensor(0x31e3, 0x00);
	write_cmos_sensor(0x31e4, 0x08);
	write_cmos_sensor(0x31e5, 0x80);
	write_cmos_sensor(0x31e6, 0x00);
	write_cmos_sensor(0x31e7, 0x2b);
	write_cmos_sensor(0x31e8, 0x6b);
	write_cmos_sensor(0x31e9, 0xab);
	write_cmos_sensor(0x31ea, 0xeb);
	write_cmos_sensor(0x31eb, 0x3f);
	write_cmos_sensor(0x31ec, 0x0f);
	write_cmos_sensor(0x31ed, 0x20);
	write_cmos_sensor(0x31ee, 0x04);
	write_cmos_sensor(0x31ef, 0x48);
	write_cmos_sensor(0x31f0, 0x07);
	write_cmos_sensor(0x31f1, 0x90);
	write_cmos_sensor(0x31f2, 0x04);
	write_cmos_sensor(0x31f3, 0x48);
	write_cmos_sensor(0x31f4, 0x07);
	write_cmos_sensor(0x31f5, 0x90);
	write_cmos_sensor(0x31f6, 0x04);
	write_cmos_sensor(0x31f7, 0x48);
	write_cmos_sensor(0x31f8, 0x07);
	write_cmos_sensor(0x31f9, 0x90);
	write_cmos_sensor(0x31fa, 0x04);
	write_cmos_sensor(0x31fb, 0x48);
	write_cmos_sensor(0x31fd, 0xcb);
	write_cmos_sensor(0x31fe, 0x0f);
	write_cmos_sensor(0x31ff, 0x03);
	write_cmos_sensor(0x3200, 0x00);
	write_cmos_sensor(0x3201, 0xff);
	write_cmos_sensor(0x3202, 0x00);
	write_cmos_sensor(0x3203, 0xff);
	write_cmos_sensor(0x3204, 0xff);
	write_cmos_sensor(0x3205, 0xff);
	write_cmos_sensor(0x3206, 0xff);
	write_cmos_sensor(0x3207, 0xff);
	write_cmos_sensor(0x3208, 0xff);
	write_cmos_sensor(0x3209, 0xff);
	write_cmos_sensor(0x320a, 0xff);
	write_cmos_sensor(0x320b, 0x1b);
	write_cmos_sensor(0x320c, 0x1f);
	write_cmos_sensor(0x320d, 0x1e);
	write_cmos_sensor(0x320e, 0x30);
	write_cmos_sensor(0x320f, 0x2d);
	write_cmos_sensor(0x3210, 0x2c);
	write_cmos_sensor(0x3211, 0x2b);
	write_cmos_sensor(0x3212, 0x2a);
	write_cmos_sensor(0x3213, 0x24);
	write_cmos_sensor(0x3214, 0x22);
	write_cmos_sensor(0x3215, 0x00);
	write_cmos_sensor(0x3216, 0x04);
	write_cmos_sensor(0x3217, 0x2b);
	write_cmos_sensor(0x3218, 0x6b);
	write_cmos_sensor(0x3219, 0xab);
	write_cmos_sensor(0x321a, 0xeb);
	write_cmos_sensor(0x321b, 0x00);
	write_cmos_sensor(0x3230, 0x3a);
	write_cmos_sensor(0x3231, 0x00);
	write_cmos_sensor(0x3232, 0x80);
	write_cmos_sensor(0x3233, 0x00);
	write_cmos_sensor(0x3234, 0x10);
	write_cmos_sensor(0x3235, 0xaa);
	write_cmos_sensor(0x3236, 0x55);
	write_cmos_sensor(0x3237, 0x99);
	write_cmos_sensor(0x3238, 0x66);
	write_cmos_sensor(0x3239, 0x08);
	write_cmos_sensor(0x323a, 0x88);
	write_cmos_sensor(0x323b, 0x00);
	write_cmos_sensor(0x323c, 0x00);
	write_cmos_sensor(0x323d, 0x03);
	write_cmos_sensor(0x3250, 0x33);
	write_cmos_sensor(0x3251, 0x00);
	write_cmos_sensor(0x3252, 0x21);
	write_cmos_sensor(0x3253, 0x00);
	write_cmos_sensor(0x3254, 0x00);
	write_cmos_sensor(0x3255, 0x01);
	write_cmos_sensor(0x3256, 0x00);
	write_cmos_sensor(0x3257, 0x00);
	write_cmos_sensor(0x3258, 0x00);
	write_cmos_sensor(0x3270, 0x01);
	write_cmos_sensor(0x3271, 0x60);
	write_cmos_sensor(0x3272, 0xc0);
	write_cmos_sensor(0x3273, 0x00);
	write_cmos_sensor(0x3274, 0x80);
	write_cmos_sensor(0x3275, 0x40);
	write_cmos_sensor(0x3276, 0x02);
	write_cmos_sensor(0x3277, 0x08);
	write_cmos_sensor(0x3278, 0x10);
	write_cmos_sensor(0x3279, 0x04);
	write_cmos_sensor(0x327a, 0x00);
	write_cmos_sensor(0x327b, 0x03);
	write_cmos_sensor(0x327c, 0x10);
	write_cmos_sensor(0x327d, 0x60);
	write_cmos_sensor(0x327e, 0xc0);
	write_cmos_sensor(0x327f, 0x06);
	write_cmos_sensor(0x3288, 0x10);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x08);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x04);
	write_cmos_sensor(0x328d, 0x00);
	write_cmos_sensor(0x328e, 0x02);
	write_cmos_sensor(0x328f, 0x00);
	write_cmos_sensor(0x3290, 0x20);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x10);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x08);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x04);
	write_cmos_sensor(0x3297, 0x00);
	write_cmos_sensor(0x3298, 0x40);
	write_cmos_sensor(0x3299, 0x00);
	write_cmos_sensor(0x329a, 0x20);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x10);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x08);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x40);
	write_cmos_sensor(0x32a3, 0x00);
	write_cmos_sensor(0x32a4, 0x20);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x10);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32a8, 0x00);
	write_cmos_sensor(0x32a9, 0x00);
	write_cmos_sensor(0x32aa, 0x00);
	write_cmos_sensor(0x32ab, 0x00);
	write_cmos_sensor(0x32ac, 0x00);
	write_cmos_sensor(0x32ad, 0x00);
	write_cmos_sensor(0x32ae, 0x00);
	write_cmos_sensor(0x32af, 0x00);
	write_cmos_sensor(0x32b0, 0x00);
	write_cmos_sensor(0x32b1, 0x00);
	write_cmos_sensor(0x32b2, 0x00);
	write_cmos_sensor(0x32b3, 0x00);
	write_cmos_sensor(0x32b4, 0x00);
	write_cmos_sensor(0x32b5, 0x00);
	write_cmos_sensor(0x32b6, 0x00);
	write_cmos_sensor(0x32b7, 0x00);
	write_cmos_sensor(0x32b8, 0x00);
	write_cmos_sensor(0x32b9, 0x00);
	write_cmos_sensor(0x32ba, 0x00);
	write_cmos_sensor(0x32bb, 0x00);
	write_cmos_sensor(0x32bc, 0x00);
	write_cmos_sensor(0x32bd, 0x00);
	write_cmos_sensor(0x32be, 0x00);
	write_cmos_sensor(0x32bf, 0x00);
	write_cmos_sensor(0x32c0, 0x00);
	write_cmos_sensor(0x32c1, 0x00);
	write_cmos_sensor(0x32c2, 0x00);
	write_cmos_sensor(0x32c3, 0x00);
	write_cmos_sensor(0x32c4, 0x00);
	write_cmos_sensor(0x32c5, 0x00);
	write_cmos_sensor(0x32c6, 0x00);
	write_cmos_sensor(0x32c7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x32c9, 0x00);
	write_cmos_sensor(0x3330, 0x03);
	write_cmos_sensor(0x3331, 0xc8);
	write_cmos_sensor(0x3332, 0x02);
	write_cmos_sensor(0x3333, 0x24);
	write_cmos_sensor(0x3334, 0x00);
	write_cmos_sensor(0x3335, 0x00);
	write_cmos_sensor(0x3336, 0x00);
	write_cmos_sensor(0x3337, 0x00);
	write_cmos_sensor(0x3338, 0x03);
	write_cmos_sensor(0x3339, 0xc8);
	write_cmos_sensor(0x333a, 0x02);
	write_cmos_sensor(0x333b, 0x24);
	write_cmos_sensor(0x333c, 0x00);
	write_cmos_sensor(0x333d, 0x00);
	write_cmos_sensor(0x333e, 0x00);
	write_cmos_sensor(0x333f, 0x00);
	write_cmos_sensor(0x3340, 0x03);
	write_cmos_sensor(0x3341, 0xc8);
	write_cmos_sensor(0x3342, 0x02);
	write_cmos_sensor(0x3343, 0x24);
	write_cmos_sensor(0x3344, 0x00);
	write_cmos_sensor(0x3345, 0x00);
	write_cmos_sensor(0x3346, 0x00);
	write_cmos_sensor(0x3347, 0x00);
	write_cmos_sensor(0x3348, 0x40);
	write_cmos_sensor(0x3349, 0x00);
	write_cmos_sensor(0x334a, 0x00);
	write_cmos_sensor(0x334b, 0x00);
	write_cmos_sensor(0x334c, 0x00);
	write_cmos_sensor(0x334d, 0x00);
	write_cmos_sensor(0x334e, 0x80);
	write_cmos_sensor(0x3360, 0x01);
	write_cmos_sensor(0x3361, 0x00);
	write_cmos_sensor(0x3362, 0x01);
	write_cmos_sensor(0x3363, 0x00);
	write_cmos_sensor(0x3364, 0x01);
	write_cmos_sensor(0x3365, 0x00);
	write_cmos_sensor(0x3366, 0x01);
	write_cmos_sensor(0x3367, 0x00);
	write_cmos_sensor(0x3368, 0x01);
	write_cmos_sensor(0x3369, 0x00);
	write_cmos_sensor(0x336a, 0x01);
	write_cmos_sensor(0x336b, 0x00);
	write_cmos_sensor(0x336c, 0x01);
	write_cmos_sensor(0x336d, 0x00);
	write_cmos_sensor(0x336e, 0x01);
	write_cmos_sensor(0x336f, 0x00);
	write_cmos_sensor(0x3370, 0x01);
	write_cmos_sensor(0x3371, 0x00);
	write_cmos_sensor(0x3372, 0x01);
	write_cmos_sensor(0x3373, 0x00);
	write_cmos_sensor(0x3374, 0x01);
	write_cmos_sensor(0x3375, 0x00);
	write_cmos_sensor(0x3376, 0x01);
	write_cmos_sensor(0x3377, 0x00);
	write_cmos_sensor(0x3378, 0x00);
	write_cmos_sensor(0x3379, 0x00);
	write_cmos_sensor(0x337a, 0x00);
	write_cmos_sensor(0x337b, 0x00);
	write_cmos_sensor(0x337c, 0x00);
	write_cmos_sensor(0x337d, 0x00);
	write_cmos_sensor(0x337e, 0x00);
	write_cmos_sensor(0x337f, 0x00);
	write_cmos_sensor(0x3380, 0x00);
	write_cmos_sensor(0x3381, 0x00);
	write_cmos_sensor(0x3382, 0x00);
	write_cmos_sensor(0x3383, 0x00);
	write_cmos_sensor(0x3384, 0x00);
	write_cmos_sensor(0x3385, 0x00);
	write_cmos_sensor(0x3386, 0x00);
	write_cmos_sensor(0x3387, 0x00);
	write_cmos_sensor(0x3388, 0x00);
	write_cmos_sensor(0x3389, 0x00);
	write_cmos_sensor(0x338a, 0x00);
	write_cmos_sensor(0x338b, 0x00);
	write_cmos_sensor(0x338c, 0x00);
	write_cmos_sensor(0x338d, 0x00);
	write_cmos_sensor(0x338e, 0x00);
	write_cmos_sensor(0x338f, 0x00);
	write_cmos_sensor(0x3390, 0x00);
	write_cmos_sensor(0x3391, 0x00);
	write_cmos_sensor(0x3392, 0x00);
	write_cmos_sensor(0x3393, 0x00);
	write_cmos_sensor(0x3394, 0x00);
	write_cmos_sensor(0x3395, 0x00);
	write_cmos_sensor(0x3396, 0x00);
	write_cmos_sensor(0x3397, 0x00);
	write_cmos_sensor(0x3398, 0x00);
	write_cmos_sensor(0x3399, 0x00);
	write_cmos_sensor(0x339a, 0x00);
	write_cmos_sensor(0x339b, 0x00);
	write_cmos_sensor(0x33b0, 0x00);
	write_cmos_sensor(0x33b1, 0x50);
	write_cmos_sensor(0x33b2, 0x01);
	write_cmos_sensor(0x33b3, 0xff);
	write_cmos_sensor(0x33b4, 0xe0);
	write_cmos_sensor(0x33b5, 0x6b);
	write_cmos_sensor(0x33b6, 0x00);
	write_cmos_sensor(0x33b7, 0x00);
	write_cmos_sensor(0x33b8, 0x00);
	write_cmos_sensor(0x33b9, 0x00);
	write_cmos_sensor(0x33ba, 0x02);
	write_cmos_sensor(0x33bb, 0x08);
	write_cmos_sensor(0x33bc, 0x01);
	write_cmos_sensor(0x33bd, 0x01);
	write_cmos_sensor(0x33be, 0x01);
	write_cmos_sensor(0x33bf, 0x01);
	write_cmos_sensor(0x33c0, 0x00);
	write_cmos_sensor(0x33c1, 0x00);
	write_cmos_sensor(0x33c2, 0x00);
	write_cmos_sensor(0x33c3, 0x00);
	write_cmos_sensor(0x33e0, 0x14);
	write_cmos_sensor(0x33e1, 0x0f);
	write_cmos_sensor(0x33e2, 0x04);
	write_cmos_sensor(0x33e3, 0x02);
	write_cmos_sensor(0x33e4, 0x01);
	write_cmos_sensor(0x33e5, 0x01);
	write_cmos_sensor(0x33e6, 0x00);
	write_cmos_sensor(0x33e7, 0x04);
	write_cmos_sensor(0x33e8, 0x0c);
	write_cmos_sensor(0x33e9, 0x00);
	write_cmos_sensor(0x33ea, 0x01);
	write_cmos_sensor(0x33eb, 0x02);
	write_cmos_sensor(0x33ec, 0x03);
	write_cmos_sensor(0x33ed, 0x02);
	write_cmos_sensor(0x33ee, 0x05);
	write_cmos_sensor(0x33ef, 0x0a);
	write_cmos_sensor(0x33f0, 0x08);
	write_cmos_sensor(0x33f1, 0x04);
	write_cmos_sensor(0x33f2, 0x04);
	write_cmos_sensor(0x33f3, 0x00);
	write_cmos_sensor(0x33f4, 0x03);
	write_cmos_sensor(0x33f5, 0x14);
	write_cmos_sensor(0x33f6, 0x0f);
	write_cmos_sensor(0x33f7, 0x02);
	write_cmos_sensor(0x33f8, 0x01);
	write_cmos_sensor(0x33f9, 0x01);
	write_cmos_sensor(0x33fa, 0x01);
	write_cmos_sensor(0x33fb, 0x00);
	write_cmos_sensor(0x33fc, 0x04);
	write_cmos_sensor(0x33fd, 0x0c);
	write_cmos_sensor(0x33fe, 0x00);
	write_cmos_sensor(0x33ff, 0x01);
	write_cmos_sensor(0x3400, 0x02);
	write_cmos_sensor(0x3401, 0x03);
	write_cmos_sensor(0x3402, 0x01);
	write_cmos_sensor(0x3403, 0x02);
	write_cmos_sensor(0x3404, 0x08);
	write_cmos_sensor(0x3405, 0x08);
	write_cmos_sensor(0x3406, 0x04);
	write_cmos_sensor(0x3407, 0x04);
	write_cmos_sensor(0x3408, 0x00);
	write_cmos_sensor(0x3409, 0x03);
	write_cmos_sensor(0x340a, 0x14);
	write_cmos_sensor(0x340b, 0x0f);
	write_cmos_sensor(0x340c, 0x04);
	write_cmos_sensor(0x340d, 0x02);
	write_cmos_sensor(0x340e, 0x01);
	write_cmos_sensor(0x340f, 0x01);
	write_cmos_sensor(0x3410, 0x00);
	write_cmos_sensor(0x3411, 0x04);
	write_cmos_sensor(0x3412, 0x0c);
	write_cmos_sensor(0x3413, 0x00);
	write_cmos_sensor(0x3414, 0x01);
	write_cmos_sensor(0x3415, 0x02);
	write_cmos_sensor(0x3416, 0x03);
	write_cmos_sensor(0x3417, 0x02);
	write_cmos_sensor(0x3418, 0x05);
	write_cmos_sensor(0x3419, 0x0a);
	write_cmos_sensor(0x341a, 0x08);
	write_cmos_sensor(0x341b, 0x04);
	write_cmos_sensor(0x341c, 0x04);
	write_cmos_sensor(0x341d, 0x00);
	write_cmos_sensor(0x341e, 0x03);
	write_cmos_sensor(0x3440, 0x00);
	write_cmos_sensor(0x3441, 0x00);
	write_cmos_sensor(0x3442, 0x00);
	write_cmos_sensor(0x3443, 0x00);
	write_cmos_sensor(0x3444, 0x02);
	write_cmos_sensor(0x3445, 0xf0);
	write_cmos_sensor(0x3446, 0x02);
	write_cmos_sensor(0x3447, 0x08);
	write_cmos_sensor(0x3448, 0x00);
	write_cmos_sensor(0x3460, 0x40);
	write_cmos_sensor(0x3461, 0x40);
	write_cmos_sensor(0x3462, 0x40);
	write_cmos_sensor(0x3463, 0x40);
	write_cmos_sensor(0x3464, 0x03);
	write_cmos_sensor(0x3465, 0x01);
	write_cmos_sensor(0x3466, 0x01);
	write_cmos_sensor(0x3467, 0x02);
	write_cmos_sensor(0x3468, 0x30);
	write_cmos_sensor(0x3469, 0x00);
	write_cmos_sensor(0x346a, 0x33);
	write_cmos_sensor(0x346b, 0xbf);
	write_cmos_sensor(0x3480, 0x40);
	write_cmos_sensor(0x3481, 0x00);
	write_cmos_sensor(0x3482, 0x00);
	write_cmos_sensor(0x3483, 0x00);
	write_cmos_sensor(0x3484, 0x0d);
	write_cmos_sensor(0x3485, 0x00);
	write_cmos_sensor(0x3486, 0x00);
	write_cmos_sensor(0x3487, 0x00);
	write_cmos_sensor(0x3488, 0x00);
	write_cmos_sensor(0x3489, 0x00);
	write_cmos_sensor(0x348a, 0x00);
	write_cmos_sensor(0x348b, 0x04);
	write_cmos_sensor(0x348c, 0x00);
	write_cmos_sensor(0x348d, 0x01);
	write_cmos_sensor(0x348f, 0x01);
	write_cmos_sensor(0x3030, 0x0a);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x7000, 0x58);
	write_cmos_sensor(0x7001, 0x7a);
	write_cmos_sensor(0x7002, 0x1a);
	write_cmos_sensor(0x7003, 0xc1);
	write_cmos_sensor(0x7004, 0x03);
	write_cmos_sensor(0x7005, 0xda);
	write_cmos_sensor(0x7006, 0xbd);
	write_cmos_sensor(0x7007, 0x03);
	write_cmos_sensor(0x7008, 0xbd);
	write_cmos_sensor(0x7009, 0x06);
	write_cmos_sensor(0x700a, 0xe6);
	write_cmos_sensor(0x700b, 0xec);
	write_cmos_sensor(0x700c, 0xbc);
	write_cmos_sensor(0x700d, 0xff);
	write_cmos_sensor(0x700e, 0xbc);
	write_cmos_sensor(0x700f, 0x73);
	write_cmos_sensor(0x7010, 0xda);
	write_cmos_sensor(0x7011, 0x72);
	write_cmos_sensor(0x7012, 0x76);
	write_cmos_sensor(0x7013, 0xb6);
	write_cmos_sensor(0x7014, 0xee);
	write_cmos_sensor(0x7015, 0xcf);
	write_cmos_sensor(0x7016, 0xac);
	write_cmos_sensor(0x7017, 0xd0);
	write_cmos_sensor(0x7018, 0xac);
	write_cmos_sensor(0x7019, 0xd1);
	write_cmos_sensor(0x701a, 0x50);
	write_cmos_sensor(0x701b, 0xac);
	write_cmos_sensor(0x701c, 0xd2);
	write_cmos_sensor(0x701d, 0xbc);
	write_cmos_sensor(0x701e, 0x2e);
	write_cmos_sensor(0x701f, 0xb4);
	write_cmos_sensor(0x7020, 0x00);
	write_cmos_sensor(0x7021, 0xdc);
	write_cmos_sensor(0x7022, 0xdf);
	write_cmos_sensor(0x7023, 0xb0);
	write_cmos_sensor(0x7024, 0x6e);
	write_cmos_sensor(0x7025, 0xbd);
	write_cmos_sensor(0x7026, 0x01);
	write_cmos_sensor(0x7027, 0xd7);
	write_cmos_sensor(0x7028, 0xed);
	write_cmos_sensor(0x7029, 0xe1);
	write_cmos_sensor(0x702a, 0x36);
	write_cmos_sensor(0x702b, 0x30);
	write_cmos_sensor(0x702c, 0xd3);
	write_cmos_sensor(0x702d, 0x2e);
	write_cmos_sensor(0x702e, 0x54);
	write_cmos_sensor(0x702f, 0x46);
	write_cmos_sensor(0x7030, 0xbc);
	write_cmos_sensor(0x7031, 0x22);
	write_cmos_sensor(0x7032, 0x66);
	write_cmos_sensor(0x7033, 0xbc);
	write_cmos_sensor(0x7034, 0x24);
	write_cmos_sensor(0x7035, 0x2c);
	write_cmos_sensor(0x7036, 0x28);
	write_cmos_sensor(0x7037, 0xbc);
	write_cmos_sensor(0x7038, 0x3c);
	write_cmos_sensor(0x7039, 0xa1);
	write_cmos_sensor(0x703a, 0xac);
	write_cmos_sensor(0x703b, 0xd8);
	write_cmos_sensor(0x703c, 0xd6);
	write_cmos_sensor(0x703d, 0xb4);
	write_cmos_sensor(0x703e, 0x04);
	write_cmos_sensor(0x703f, 0x46);
	write_cmos_sensor(0x7040, 0xb7);
	write_cmos_sensor(0x7041, 0x04);
	write_cmos_sensor(0x7042, 0xbe);
	write_cmos_sensor(0x7043, 0x08);
	write_cmos_sensor(0x7044, 0xc3);
	write_cmos_sensor(0x7045, 0xd9);
	write_cmos_sensor(0x7046, 0xad);
	write_cmos_sensor(0x7047, 0xc3);
	write_cmos_sensor(0x7048, 0xbc);
	write_cmos_sensor(0x7049, 0x19);
	write_cmos_sensor(0x704a, 0xc1);
	write_cmos_sensor(0x704b, 0x27);
	write_cmos_sensor(0x704c, 0xe7);
	write_cmos_sensor(0x704d, 0x00);
	write_cmos_sensor(0x704e, 0xb9);
	write_cmos_sensor(0x704f, 0x64);
	write_cmos_sensor(0x7050, 0x50);
	write_cmos_sensor(0x7051, 0x20);
	write_cmos_sensor(0x7052, 0xb8);
	write_cmos_sensor(0x7053, 0x02);
	write_cmos_sensor(0x7054, 0xbc);
	write_cmos_sensor(0x7055, 0x17);
	write_cmos_sensor(0x7056, 0xdb);
	write_cmos_sensor(0x7057, 0xc7);
	write_cmos_sensor(0x7058, 0xb8);
	write_cmos_sensor(0x7059, 0x00);
	write_cmos_sensor(0x705a, 0x28);
	write_cmos_sensor(0x705b, 0x54);
	write_cmos_sensor(0x705c, 0xb4);
	write_cmos_sensor(0x705d, 0x14);
	write_cmos_sensor(0x705e, 0xab);
	write_cmos_sensor(0x705f, 0xbe);
	write_cmos_sensor(0x7060, 0x06);
	write_cmos_sensor(0x7061, 0xd8);
	write_cmos_sensor(0x7062, 0xd6);
	write_cmos_sensor(0x7063, 0x00);
	write_cmos_sensor(0x7064, 0xb4);
	write_cmos_sensor(0x7065, 0xc7);
	write_cmos_sensor(0x7066, 0x62);
	write_cmos_sensor(0x7067, 0x07);
	write_cmos_sensor(0x7068, 0xb9);
	write_cmos_sensor(0x7069, 0x05);
	write_cmos_sensor(0x706a, 0xee);
	write_cmos_sensor(0x706b, 0xe6);
	write_cmos_sensor(0x706c, 0xad);
	write_cmos_sensor(0x706d, 0xb4);
	write_cmos_sensor(0x706e, 0x26);
	write_cmos_sensor(0x706f, 0x19);
	write_cmos_sensor(0x7070, 0xc1);
	write_cmos_sensor(0x7071, 0x3b);
	write_cmos_sensor(0x7072, 0xc3);
	write_cmos_sensor(0x7073, 0xaf);
	write_cmos_sensor(0x7074, 0xc0);
	write_cmos_sensor(0x7075, 0x3d);
	write_cmos_sensor(0x7076, 0xc3);
	write_cmos_sensor(0x7077, 0xbe);
	write_cmos_sensor(0x7078, 0xe7);
	write_cmos_sensor(0x7079, 0x00);
	write_cmos_sensor(0x707a, 0x15);
	write_cmos_sensor(0x707b, 0xc2);
	write_cmos_sensor(0x707c, 0x41);
	write_cmos_sensor(0x707d, 0xc3);
	write_cmos_sensor(0x707e, 0xa4);
	write_cmos_sensor(0x707f, 0xc0);
	write_cmos_sensor(0x7080, 0x3d);
	write_cmos_sensor(0x7081, 0x00);
	write_cmos_sensor(0x7082, 0xb9);
	write_cmos_sensor(0x7083, 0x64);
	write_cmos_sensor(0x7084, 0x29);
	write_cmos_sensor(0x7085, 0x00);
	write_cmos_sensor(0x7086, 0xb8);
	write_cmos_sensor(0x7087, 0x12);
	write_cmos_sensor(0x7088, 0xbe);
	write_cmos_sensor(0x7089, 0x01);
	write_cmos_sensor(0x708a, 0xd0);
	write_cmos_sensor(0x708b, 0xbc);
	write_cmos_sensor(0x708c, 0x01);
	write_cmos_sensor(0x708d, 0xac);
	write_cmos_sensor(0x708e, 0x37);
	write_cmos_sensor(0x708f, 0xd2);
	write_cmos_sensor(0x7090, 0xac);
	write_cmos_sensor(0x7091, 0x45);
	write_cmos_sensor(0x7092, 0xad);
	write_cmos_sensor(0x7093, 0x28);
	write_cmos_sensor(0x7094, 0x00);
	write_cmos_sensor(0x7095, 0xb8);
	write_cmos_sensor(0x7096, 0x00);
	write_cmos_sensor(0x7097, 0xbc);
	write_cmos_sensor(0x7098, 0x01);
	write_cmos_sensor(0x7099, 0x36);
	write_cmos_sensor(0x709a, 0xd3);
	write_cmos_sensor(0x709b, 0x30);
	write_cmos_sensor(0x709c, 0x04);
	write_cmos_sensor(0x709d, 0xe0);
	write_cmos_sensor(0x709e, 0xd8);
	write_cmos_sensor(0x709f, 0xb4);
	write_cmos_sensor(0x70a0, 0xe9);
	write_cmos_sensor(0x70a1, 0x00);
	write_cmos_sensor(0x70a2, 0xbe);
	write_cmos_sensor(0x70a3, 0x05);
	write_cmos_sensor(0x70a4, 0x62);
	write_cmos_sensor(0x70a5, 0x07);
	write_cmos_sensor(0x70a6, 0xb9);
	write_cmos_sensor(0x70a7, 0x05);
	write_cmos_sensor(0x70a8, 0xad);
	write_cmos_sensor(0x70a9, 0xc3);
	write_cmos_sensor(0x70aa, 0xcf);
	write_cmos_sensor(0x70ab, 0x00);
	write_cmos_sensor(0x70ac, 0x15);
	write_cmos_sensor(0x70ad, 0xc2);
	write_cmos_sensor(0x70ae, 0x5a);
	write_cmos_sensor(0x70af, 0xc3);
	write_cmos_sensor(0x70b0, 0xc9);
	write_cmos_sensor(0x70b1, 0xc0);
	write_cmos_sensor(0x70b2, 0x56);
	write_cmos_sensor(0x70b3, 0x00);
	write_cmos_sensor(0x70b4, 0x46);
	write_cmos_sensor(0x70b5, 0xa1);
	write_cmos_sensor(0x70b6, 0xb9);
	write_cmos_sensor(0x70b7, 0x64);
	write_cmos_sensor(0x70b8, 0x29);
	write_cmos_sensor(0x70b9, 0x00);
	write_cmos_sensor(0x70ba, 0xb8);
	write_cmos_sensor(0x70bb, 0x02);
	write_cmos_sensor(0x70bc, 0xbe);
	write_cmos_sensor(0x70bd, 0x02);
	write_cmos_sensor(0x70be, 0xd0);
	write_cmos_sensor(0x70bf, 0xdc);
	write_cmos_sensor(0x70c0, 0xac);
	write_cmos_sensor(0x70c1, 0xbc);
	write_cmos_sensor(0x70c2, 0x01);
	write_cmos_sensor(0x70c3, 0x37);
	write_cmos_sensor(0x70c4, 0xac);
	write_cmos_sensor(0x70c5, 0xd2);
	write_cmos_sensor(0x70c6, 0x45);
	write_cmos_sensor(0x70c7, 0xad);
	write_cmos_sensor(0x70c8, 0x28);
	write_cmos_sensor(0x70c9, 0x00);
	write_cmos_sensor(0x70ca, 0xb8);
	write_cmos_sensor(0x70cb, 0x00);
	write_cmos_sensor(0x70cc, 0xbc);
	write_cmos_sensor(0x70cd, 0x01);
	write_cmos_sensor(0x70ce, 0x36);
	write_cmos_sensor(0x70cf, 0x30);
	write_cmos_sensor(0x70d0, 0xe0);
	write_cmos_sensor(0x70d1, 0xd8);
	write_cmos_sensor(0x70d2, 0xb5);
	write_cmos_sensor(0x70d3, 0x0b);
	write_cmos_sensor(0x70d4, 0xd6);
	write_cmos_sensor(0x70d5, 0xbe);
	write_cmos_sensor(0x70d6, 0x07);
	write_cmos_sensor(0x70d7, 0x00);
	write_cmos_sensor(0x70d8, 0x62);
	write_cmos_sensor(0x70d9, 0x07);
	write_cmos_sensor(0x70da, 0xb9);
	write_cmos_sensor(0x70db, 0x05);
	write_cmos_sensor(0x70dc, 0xad);
	write_cmos_sensor(0x70dd, 0xc3);
	write_cmos_sensor(0x70de, 0xcf);
	write_cmos_sensor(0x70df, 0x46);
	write_cmos_sensor(0x70e0, 0xcd);
	write_cmos_sensor(0x70e1, 0x07);
	write_cmos_sensor(0x70e2, 0xcd);
	write_cmos_sensor(0x70e3, 0x00);
	write_cmos_sensor(0x70e4, 0xe3);
	write_cmos_sensor(0x70e5, 0x18);
	write_cmos_sensor(0x70e6, 0xc2);
	write_cmos_sensor(0x70e7, 0xa2);
	write_cmos_sensor(0x70e8, 0xb9);
	write_cmos_sensor(0x70e9, 0x64);
	write_cmos_sensor(0x70ea, 0xd1);
	write_cmos_sensor(0x70eb, 0xdd);
	write_cmos_sensor(0x70ec, 0xac);
	write_cmos_sensor(0x70ed, 0xcf);
	write_cmos_sensor(0x70ee, 0xdf);
	write_cmos_sensor(0x70ef, 0xb5);
	write_cmos_sensor(0x70f0, 0x19);
	write_cmos_sensor(0x70f1, 0x46);
	write_cmos_sensor(0x70f2, 0x50);
	write_cmos_sensor(0x70f3, 0xb6);
	write_cmos_sensor(0x70f4, 0xee);
	write_cmos_sensor(0x70f5, 0xe8);
	write_cmos_sensor(0x70f6, 0xe6);
	write_cmos_sensor(0x70f7, 0xbc);
	write_cmos_sensor(0x70f8, 0x31);
	write_cmos_sensor(0x70f9, 0xe1);
	write_cmos_sensor(0x70fa, 0x36);
	write_cmos_sensor(0x70fb, 0x30);
	write_cmos_sensor(0x70fc, 0xd3);
	write_cmos_sensor(0x70fd, 0x2e);
	write_cmos_sensor(0x70fe, 0x54);
	write_cmos_sensor(0x70ff, 0xbd);
	write_cmos_sensor(0x7100, 0x03);
	write_cmos_sensor(0x7101, 0xec);
	write_cmos_sensor(0x7102, 0x2c);
	write_cmos_sensor(0x7103, 0x50);
	write_cmos_sensor(0x7104, 0x20);
	write_cmos_sensor(0x7105, 0x04);
	write_cmos_sensor(0x7106, 0xb8);
	write_cmos_sensor(0x7107, 0x02);
	write_cmos_sensor(0x7108, 0xbc);
	write_cmos_sensor(0x7109, 0x18);
	write_cmos_sensor(0x710a, 0xc7);
	write_cmos_sensor(0x710b, 0xb8);
	write_cmos_sensor(0x710c, 0x00);
	write_cmos_sensor(0x710d, 0x28);
	write_cmos_sensor(0x710e, 0x54);
	write_cmos_sensor(0x710f, 0x02);
	write_cmos_sensor(0x7110, 0xb4);
	write_cmos_sensor(0x7111, 0xda);
	write_cmos_sensor(0x7112, 0xbe);
	write_cmos_sensor(0x7113, 0x04);
	write_cmos_sensor(0x7114, 0xd6);
	write_cmos_sensor(0x7115, 0xd8);
	write_cmos_sensor(0x7116, 0xab);
	write_cmos_sensor(0x7117, 0x00);
	write_cmos_sensor(0x7118, 0x62);
	write_cmos_sensor(0x7119, 0x07);
	write_cmos_sensor(0x711a, 0xb9);
	write_cmos_sensor(0x711b, 0x05);
	write_cmos_sensor(0x711c, 0xad);
	write_cmos_sensor(0x711d, 0xc3);
	write_cmos_sensor(0x711e, 0xbc);
	write_cmos_sensor(0x711f, 0xe7);
	write_cmos_sensor(0x7120, 0xb9);
	write_cmos_sensor(0x7121, 0x64);
	write_cmos_sensor(0x7122, 0x29);
	write_cmos_sensor(0x7123, 0x00);
	write_cmos_sensor(0x7124, 0xb8);
	write_cmos_sensor(0x7125, 0x02);
	write_cmos_sensor(0x7126, 0xbe);
	write_cmos_sensor(0x7127, 0x00);
	write_cmos_sensor(0x7128, 0x45);
	write_cmos_sensor(0x7129, 0xad);
	write_cmos_sensor(0x712a, 0xe2);
	write_cmos_sensor(0x712b, 0x28);
	write_cmos_sensor(0x712c, 0x00);
	write_cmos_sensor(0x712d, 0xb8);
	write_cmos_sensor(0x712e, 0x00);
	write_cmos_sensor(0x712f, 0xe0);
	write_cmos_sensor(0x7130, 0xd8);
	write_cmos_sensor(0x7131, 0xb4);
	write_cmos_sensor(0x7132, 0xe9);
	write_cmos_sensor(0x7133, 0xbe);
	write_cmos_sensor(0x7134, 0x03);
	write_cmos_sensor(0x7135, 0x00);
	write_cmos_sensor(0x7136, 0x30);
	write_cmos_sensor(0x7137, 0x62);
	write_cmos_sensor(0x7138, 0x07);
	write_cmos_sensor(0x7139, 0xb9);
	write_cmos_sensor(0x713a, 0x05);
	write_cmos_sensor(0x713b, 0xad);
	write_cmos_sensor(0x713c, 0xc3);
	write_cmos_sensor(0x713d, 0xcf);
	write_cmos_sensor(0x713e, 0x42);
	write_cmos_sensor(0x713f, 0xe4);
	write_cmos_sensor(0x7140, 0xcd);
	write_cmos_sensor(0x7141, 0x07);
	write_cmos_sensor(0x7142, 0xcd);
	write_cmos_sensor(0x7143, 0x00);
	write_cmos_sensor(0x7144, 0x17);
	write_cmos_sensor(0x7145, 0xc2);
	write_cmos_sensor(0x7146, 0xbb);
	write_cmos_sensor(0x7147, 0xde);
	write_cmos_sensor(0x7148, 0xcf);
	write_cmos_sensor(0x7149, 0xdf);
	write_cmos_sensor(0x714a, 0xac);
	write_cmos_sensor(0x714b, 0xd1);
	write_cmos_sensor(0x714c, 0x44);
	write_cmos_sensor(0x714d, 0xac);
	write_cmos_sensor(0x714e, 0xb9);
	write_cmos_sensor(0x714f, 0x76);
	write_cmos_sensor(0x7150, 0xb8);
	write_cmos_sensor(0x7151, 0x08);
	write_cmos_sensor(0x7152, 0xb6);
	write_cmos_sensor(0x7153, 0xfe);
	write_cmos_sensor(0x7154, 0xb4);
	write_cmos_sensor(0x7155, 0xca);
	write_cmos_sensor(0x7156, 0xd6);
	write_cmos_sensor(0x7157, 0xd8);
	write_cmos_sensor(0x7158, 0xab);
	write_cmos_sensor(0x7159, 0x00);
	write_cmos_sensor(0x715a, 0xe1);
	write_cmos_sensor(0x715b, 0x36);
	write_cmos_sensor(0x715c, 0x30);
	write_cmos_sensor(0x715d, 0xd3);
	write_cmos_sensor(0x715e, 0xbc);
	write_cmos_sensor(0x715f, 0x29);
	write_cmos_sensor(0x7160, 0xb4);
	write_cmos_sensor(0x7161, 0x1f);
	write_cmos_sensor(0x7162, 0xaa);
	write_cmos_sensor(0x7163, 0xbd);
	write_cmos_sensor(0x7164, 0x01);
	write_cmos_sensor(0x7165, 0xb8);
	write_cmos_sensor(0x7166, 0x0c);
	write_cmos_sensor(0x7167, 0x45);
	write_cmos_sensor(0x7168, 0xa4);
	write_cmos_sensor(0x7169, 0xbd);
	write_cmos_sensor(0x716a, 0x03);
	write_cmos_sensor(0x716b, 0xec);
	write_cmos_sensor(0x716c, 0xbc);
	write_cmos_sensor(0x716d, 0x3d);
	write_cmos_sensor(0x716e, 0xc3);
	write_cmos_sensor(0x716f, 0xcf);
	write_cmos_sensor(0x7170, 0x42);
	write_cmos_sensor(0x7171, 0xb8);
	write_cmos_sensor(0x7172, 0x00);
	write_cmos_sensor(0x7173, 0xe4);
	write_cmos_sensor(0x7174, 0xd5);
	write_cmos_sensor(0x7175, 0x00);
	write_cmos_sensor(0x7176, 0xb6);
	write_cmos_sensor(0x7177, 0x00);
	write_cmos_sensor(0x7178, 0x74);
	write_cmos_sensor(0x7179, 0xbd);
	write_cmos_sensor(0x717a, 0x03);
	write_cmos_sensor(0x717b, 0xb5);
	write_cmos_sensor(0x717c, 0x39);
	write_cmos_sensor(0x717d, 0x40);
	write_cmos_sensor(0x717e, 0x58);
	write_cmos_sensor(0x717f, 0x6a);
	write_cmos_sensor(0x7180, 0xdd);
	write_cmos_sensor(0x7181, 0x19);
	write_cmos_sensor(0x7182, 0xc1);
	write_cmos_sensor(0x7183, 0xc8);
	write_cmos_sensor(0x7184, 0xbd);
	write_cmos_sensor(0x7185, 0x06);
	write_cmos_sensor(0x7186, 0x17);
	write_cmos_sensor(0x7187, 0xc1);
	write_cmos_sensor(0x7188, 0xc6);
	write_cmos_sensor(0x7189, 0xe8);
	write_cmos_sensor(0x718a, 0xc0);
	write_cmos_sensor(0x718b, 0xc8);
	write_cmos_sensor(0x718c, 0xe6);
	write_cmos_sensor(0x718d, 0x95);
	write_cmos_sensor(0x718e, 0x15);
	write_cmos_sensor(0x718f, 0x00);
	write_cmos_sensor(0x7190, 0xbc);
	write_cmos_sensor(0x7191, 0x19);
	write_cmos_sensor(0x7192, 0xb9);
	write_cmos_sensor(0x7193, 0xf6);
	write_cmos_sensor(0x7194, 0x14);
	write_cmos_sensor(0x7195, 0xc1);
	write_cmos_sensor(0x7196, 0xd0);
	write_cmos_sensor(0x7197, 0xd1);
	write_cmos_sensor(0x7198, 0xac);
	write_cmos_sensor(0x7199, 0x37);
	write_cmos_sensor(0x719a, 0xbc);
	write_cmos_sensor(0x719b, 0x35);
	write_cmos_sensor(0x719c, 0x36);
	write_cmos_sensor(0x719d, 0x30);
	write_cmos_sensor(0x719e, 0xe1);
	write_cmos_sensor(0x719f, 0xd3);
	write_cmos_sensor(0x71a0, 0x7a);
	write_cmos_sensor(0x71a1, 0xb6);
	write_cmos_sensor(0x71a2, 0x0c);
	write_cmos_sensor(0x71a3, 0xff);
	write_cmos_sensor(0x71a4, 0xb4);
	write_cmos_sensor(0x71a5, 0xc7);
	write_cmos_sensor(0x71a6, 0xd9);
	write_cmos_sensor(0x71a7, 0x00);
	write_cmos_sensor(0x71a8, 0xbd);
	write_cmos_sensor(0x71a9, 0x01);
	write_cmos_sensor(0x71aa, 0x56);
	write_cmos_sensor(0x71ab, 0xc0);
	write_cmos_sensor(0x71ac, 0xda);
	write_cmos_sensor(0x71ad, 0xb4);
	write_cmos_sensor(0x71ae, 0x1f);
	write_cmos_sensor(0x71af, 0x56);
	write_cmos_sensor(0x71b0, 0xaa);
	write_cmos_sensor(0x71b1, 0xbc);
	write_cmos_sensor(0x71b2, 0x08);
	write_cmos_sensor(0x71b3, 0x00);
	write_cmos_sensor(0x71b4, 0x57);
	write_cmos_sensor(0x71b5, 0xe8);
	write_cmos_sensor(0x71b6, 0xb5);
	write_cmos_sensor(0x71b7, 0x36);
	write_cmos_sensor(0x71b8, 0x00);
	write_cmos_sensor(0x71b9, 0x54);
	write_cmos_sensor(0x71ba, 0xe7);
	write_cmos_sensor(0x71bb, 0xc8);
	write_cmos_sensor(0x71bc, 0xb4);
	write_cmos_sensor(0x71bd, 0x1f);
	write_cmos_sensor(0x71be, 0x56);
	write_cmos_sensor(0x71bf, 0xaa);
	write_cmos_sensor(0x71c0, 0xbc);
	write_cmos_sensor(0x71c1, 0x08);
	write_cmos_sensor(0x71c2, 0x57);
	write_cmos_sensor(0x71c3, 0x00);
	write_cmos_sensor(0x71c4, 0xb5);
	write_cmos_sensor(0x71c5, 0x36);
	write_cmos_sensor(0x71c6, 0x00);
	write_cmos_sensor(0x71c7, 0x54);
	write_cmos_sensor(0x71c8, 0xc8);
	write_cmos_sensor(0x71c9, 0xb5);
	write_cmos_sensor(0x71ca, 0x18);
	write_cmos_sensor(0x71cb, 0xd9);
	write_cmos_sensor(0x71cc, 0x00);
	write_cmos_sensor(0x71cd, 0xbd);
	write_cmos_sensor(0x71ce, 0x01);
	write_cmos_sensor(0x71cf, 0x56);
	write_cmos_sensor(0x71d0, 0x08);
	write_cmos_sensor(0x71d1, 0x57);
	write_cmos_sensor(0x71d2, 0xe8);
	write_cmos_sensor(0x71d3, 0xb4);
	write_cmos_sensor(0x71d4, 0x42);
	write_cmos_sensor(0x71d5, 0x00);
	write_cmos_sensor(0x71d6, 0x54);
	write_cmos_sensor(0x71d7, 0xe7);
	write_cmos_sensor(0x71d8, 0xc8);
	write_cmos_sensor(0x71d9, 0xab);
	write_cmos_sensor(0x71da, 0x00);
	write_cmos_sensor(0x71db, 0x66);
	write_cmos_sensor(0x71dc, 0x62);
	write_cmos_sensor(0x71dd, 0x06);
	write_cmos_sensor(0x71de, 0x74);
	write_cmos_sensor(0x71df, 0xb9);
	write_cmos_sensor(0x71e0, 0x05);
	write_cmos_sensor(0x71e1, 0xb7);
	write_cmos_sensor(0x71e2, 0x14);
	write_cmos_sensor(0x71e3, 0x0e);
	write_cmos_sensor(0x71e4, 0xb7);
	write_cmos_sensor(0x71e5, 0x04);
	write_cmos_sensor(0x71e6, 0xc8);
	write_cmos_sensor(0x7600, 0x04);
	write_cmos_sensor(0x7601, 0x80);
	write_cmos_sensor(0x7602, 0x07);
	write_cmos_sensor(0x7603, 0x44);
	write_cmos_sensor(0x7604, 0x05);
	write_cmos_sensor(0x7605, 0x33);
	write_cmos_sensor(0x7606, 0x0f);
	write_cmos_sensor(0x7607, 0x00);
	write_cmos_sensor(0x7608, 0x07);
	write_cmos_sensor(0x7609, 0x40);
	write_cmos_sensor(0x760a, 0x04);
	write_cmos_sensor(0x760b, 0xe5);
	write_cmos_sensor(0x760c, 0x06);
	write_cmos_sensor(0x760d, 0x50);
	write_cmos_sensor(0x760e, 0x04);
	write_cmos_sensor(0x760f, 0xe4);
	write_cmos_sensor(0x7610, 0x00);
	write_cmos_sensor(0x7611, 0x00);
	write_cmos_sensor(0x7612, 0x06);
	write_cmos_sensor(0x7613, 0x5c);
	write_cmos_sensor(0x7614, 0x00);
	write_cmos_sensor(0x7615, 0x0f);
	write_cmos_sensor(0x7616, 0x06);
	write_cmos_sensor(0x7617, 0x1c);
	write_cmos_sensor(0x7618, 0x00);
	write_cmos_sensor(0x7619, 0x02);
	write_cmos_sensor(0x761a, 0x06);
	write_cmos_sensor(0x761b, 0xa2);
	write_cmos_sensor(0x761c, 0x00);
	write_cmos_sensor(0x761d, 0x01);
	write_cmos_sensor(0x761e, 0x06);
	write_cmos_sensor(0x761f, 0xae);
	write_cmos_sensor(0x7620, 0x00);
	write_cmos_sensor(0x7621, 0x0e);
	write_cmos_sensor(0x7622, 0x05);
	write_cmos_sensor(0x7623, 0x30);
	write_cmos_sensor(0x7624, 0x07);
	write_cmos_sensor(0x7625, 0x00);
	write_cmos_sensor(0x7626, 0x0f);
	write_cmos_sensor(0x7627, 0x00);
	write_cmos_sensor(0x7628, 0x04);
	write_cmos_sensor(0x7629, 0xe5);
	write_cmos_sensor(0x762a, 0x05);
	write_cmos_sensor(0x762b, 0x33);
	write_cmos_sensor(0x762c, 0x06);
	write_cmos_sensor(0x762d, 0x12);
	write_cmos_sensor(0x762e, 0x00);
	write_cmos_sensor(0x762f, 0x01);
	write_cmos_sensor(0x7630, 0x06);
	write_cmos_sensor(0x7631, 0x52);
	write_cmos_sensor(0x7632, 0x00);
	write_cmos_sensor(0x7633, 0x01);
	write_cmos_sensor(0x7634, 0x06);
	write_cmos_sensor(0x7635, 0x5e);
	write_cmos_sensor(0x7636, 0x04);
	write_cmos_sensor(0x7637, 0xe4);
	write_cmos_sensor(0x7638, 0x00);
	write_cmos_sensor(0x7639, 0x01);
	write_cmos_sensor(0x763a, 0x05);
	write_cmos_sensor(0x763b, 0x30);
	write_cmos_sensor(0x763c, 0x0f);
	write_cmos_sensor(0x763d, 0x00);
	write_cmos_sensor(0x763e, 0x06);
	write_cmos_sensor(0x763f, 0xa6);
	write_cmos_sensor(0x7640, 0x00);
	write_cmos_sensor(0x7641, 0x02);
	write_cmos_sensor(0x7642, 0x06);
	write_cmos_sensor(0x7643, 0x26);
	write_cmos_sensor(0x7644, 0x00);
	write_cmos_sensor(0x7645, 0x02);
	write_cmos_sensor(0x7646, 0x05);
	write_cmos_sensor(0x7647, 0x33);
	write_cmos_sensor(0x7648, 0x06);
	write_cmos_sensor(0x7649, 0x20);
	write_cmos_sensor(0x764a, 0x0f);
	write_cmos_sensor(0x764b, 0x00);
	write_cmos_sensor(0x764c, 0x06);
	write_cmos_sensor(0x764d, 0x56);
	write_cmos_sensor(0x764e, 0x00);
	write_cmos_sensor(0x764f, 0x02);
	write_cmos_sensor(0x7650, 0x06);
	write_cmos_sensor(0x7651, 0x16);
	write_cmos_sensor(0x7652, 0x05);
	write_cmos_sensor(0x7653, 0x33);
	write_cmos_sensor(0x7654, 0x06);
	write_cmos_sensor(0x7655, 0x10);
	write_cmos_sensor(0x7656, 0x0f);
	write_cmos_sensor(0x7657, 0x00);
	write_cmos_sensor(0x7658, 0x06);
	write_cmos_sensor(0x7659, 0x10);
	write_cmos_sensor(0x765a, 0x0f);
	write_cmos_sensor(0x765b, 0x00);
	write_cmos_sensor(0x765c, 0x06);
	write_cmos_sensor(0x765d, 0x20);
	write_cmos_sensor(0x765e, 0x0f);
	write_cmos_sensor(0x765f, 0x00);
	write_cmos_sensor(0x7660, 0x00);
	write_cmos_sensor(0x7661, 0x00);
	write_cmos_sensor(0x7662, 0x00);
	write_cmos_sensor(0x7663, 0x02);
	write_cmos_sensor(0x7664, 0x04);
	write_cmos_sensor(0x7665, 0xe5);
	write_cmos_sensor(0x7666, 0x04);
	write_cmos_sensor(0x7667, 0xe4);
	write_cmos_sensor(0x7668, 0x0f);
	write_cmos_sensor(0x7669, 0x00);
	write_cmos_sensor(0x766a, 0x00);
	write_cmos_sensor(0x766b, 0x00);
	write_cmos_sensor(0x766c, 0x00);
	write_cmos_sensor(0x766d, 0x01);
	write_cmos_sensor(0x766e, 0x04);
	write_cmos_sensor(0x766f, 0xe5);
	write_cmos_sensor(0x7670, 0x04);
	write_cmos_sensor(0x7671, 0xe4);
	write_cmos_sensor(0x7672, 0x0f);
	write_cmos_sensor(0x7673, 0x00);
	write_cmos_sensor(0x7674, 0x00);
	write_cmos_sensor(0x7675, 0x02);
	write_cmos_sensor(0x7676, 0x04);
	write_cmos_sensor(0x7677, 0xe4);
	write_cmos_sensor(0x7678, 0x00);
	write_cmos_sensor(0x7679, 0x02);
	write_cmos_sensor(0x767a, 0x04);
	write_cmos_sensor(0x767b, 0xc4);
	write_cmos_sensor(0x767c, 0x00);
	write_cmos_sensor(0x767d, 0x02);
	write_cmos_sensor(0x767e, 0x04);
	write_cmos_sensor(0x767f, 0xc4);
	write_cmos_sensor(0x7680, 0x05);
	write_cmos_sensor(0x7681, 0x83);
	write_cmos_sensor(0x7682, 0x0f);
	write_cmos_sensor(0x7683, 0x00);
	write_cmos_sensor(0x7684, 0x00);
	write_cmos_sensor(0x7685, 0x02);
	write_cmos_sensor(0x7686, 0x04);
	write_cmos_sensor(0x7687, 0xe4);
	write_cmos_sensor(0x7688, 0x00);
	write_cmos_sensor(0x7689, 0x02);
	write_cmos_sensor(0x768a, 0x04);
	write_cmos_sensor(0x768b, 0xc4);
	write_cmos_sensor(0x768c, 0x00);
	write_cmos_sensor(0x768d, 0x02);
	write_cmos_sensor(0x768e, 0x04);
	write_cmos_sensor(0x768f, 0xc4);
	write_cmos_sensor(0x7690, 0x05);
	write_cmos_sensor(0x7691, 0x83);
	write_cmos_sensor(0x7692, 0x03);
	write_cmos_sensor(0x7693, 0x0b);
	write_cmos_sensor(0x7694, 0x05);
	write_cmos_sensor(0x7695, 0x83);
	write_cmos_sensor(0x7696, 0x00);
	write_cmos_sensor(0x7697, 0x07);
	write_cmos_sensor(0x7698, 0x05);
	write_cmos_sensor(0x7699, 0x03);
	write_cmos_sensor(0x769a, 0x00);
	write_cmos_sensor(0x769b, 0x05);
	write_cmos_sensor(0x769c, 0x05);
	write_cmos_sensor(0x769d, 0x32);
	write_cmos_sensor(0x769e, 0x05);
	write_cmos_sensor(0x769f, 0x30);
	write_cmos_sensor(0x76a0, 0x00);
	write_cmos_sensor(0x76a1, 0x02);
	write_cmos_sensor(0x76a2, 0x05);
	write_cmos_sensor(0x76a3, 0x78);
	write_cmos_sensor(0x76a4, 0x00);
	write_cmos_sensor(0x76a5, 0x01);
	write_cmos_sensor(0x76a6, 0x05);
	write_cmos_sensor(0x76a7, 0x7c);
	write_cmos_sensor(0x76a8, 0x03);
	write_cmos_sensor(0x76a9, 0x9a);
	write_cmos_sensor(0x76aa, 0x05);
	write_cmos_sensor(0x76ab, 0x83);
	write_cmos_sensor(0x76ac, 0x00);
	write_cmos_sensor(0x76ad, 0x04);
	write_cmos_sensor(0x76ae, 0x05);
	write_cmos_sensor(0x76af, 0x03);
	write_cmos_sensor(0x76b0, 0x00);
	write_cmos_sensor(0x76b1, 0x03);
	write_cmos_sensor(0x76b2, 0x05);
	write_cmos_sensor(0x76b3, 0x32);
	write_cmos_sensor(0x76b4, 0x05);
	write_cmos_sensor(0x76b5, 0x30);
	write_cmos_sensor(0x76b6, 0x00);
	write_cmos_sensor(0x76b7, 0x02);
	write_cmos_sensor(0x76b8, 0x05);
	write_cmos_sensor(0x76b9, 0x78);
	write_cmos_sensor(0x76ba, 0x00);
	write_cmos_sensor(0x76bb, 0x01);
	write_cmos_sensor(0x76bc, 0x05);
	write_cmos_sensor(0x76bd, 0x7c);
	write_cmos_sensor(0x76be, 0x03);
	write_cmos_sensor(0x76bf, 0x99);
	write_cmos_sensor(0x76c0, 0x05);
	write_cmos_sensor(0x76c1, 0x83);
	write_cmos_sensor(0x76c2, 0x00);
	write_cmos_sensor(0x76c3, 0x03);
	write_cmos_sensor(0x76c4, 0x05);
	write_cmos_sensor(0x76c5, 0x03);
	write_cmos_sensor(0x76c6, 0x00);
	write_cmos_sensor(0x76c7, 0x01);
	write_cmos_sensor(0x76c8, 0x05);
	write_cmos_sensor(0x76c9, 0x32);
	write_cmos_sensor(0x76ca, 0x05);
	write_cmos_sensor(0x76cb, 0x30);
	write_cmos_sensor(0x76cc, 0x00);
	write_cmos_sensor(0x76cd, 0x02);
	write_cmos_sensor(0x76ce, 0x05);
	write_cmos_sensor(0x76cf, 0x78);
	write_cmos_sensor(0x76d0, 0x00);
	write_cmos_sensor(0x76d1, 0x01);
	write_cmos_sensor(0x76d2, 0x05);
	write_cmos_sensor(0x76d3, 0x7c);
	write_cmos_sensor(0x76d4, 0x03);
	write_cmos_sensor(0x76d5, 0x98);
	write_cmos_sensor(0x76d6, 0x05);
	write_cmos_sensor(0x76d7, 0x83);
	write_cmos_sensor(0x76d8, 0x00);
	write_cmos_sensor(0x76d9, 0x00);
	write_cmos_sensor(0x76da, 0x05);
	write_cmos_sensor(0x76db, 0x03);
	write_cmos_sensor(0x76dc, 0x00);
	write_cmos_sensor(0x76dd, 0x01);
	write_cmos_sensor(0x76de, 0x05);
	write_cmos_sensor(0x76df, 0x32);
	write_cmos_sensor(0x76e0, 0x05);
	write_cmos_sensor(0x76e1, 0x30);
	write_cmos_sensor(0x76e2, 0x00);
	write_cmos_sensor(0x76e3, 0x02);
	write_cmos_sensor(0x76e4, 0x05);
	write_cmos_sensor(0x76e5, 0x78);
	write_cmos_sensor(0x76e6, 0x00);
	write_cmos_sensor(0x76e7, 0x01);
	write_cmos_sensor(0x76e8, 0x05);
	write_cmos_sensor(0x76e9, 0x7c);
	write_cmos_sensor(0x76ea, 0x03);
	write_cmos_sensor(0x76eb, 0x97);
	write_cmos_sensor(0x76ec, 0x05);
	write_cmos_sensor(0x76ed, 0x83);
	write_cmos_sensor(0x76ee, 0x00);
	write_cmos_sensor(0x76ef, 0x00);
	write_cmos_sensor(0x76f0, 0x05);
	write_cmos_sensor(0x76f1, 0x03);
	write_cmos_sensor(0x76f2, 0x05);
	write_cmos_sensor(0x76f3, 0x32);
	write_cmos_sensor(0x76f4, 0x05);
	write_cmos_sensor(0x76f5, 0x30);
	write_cmos_sensor(0x76f6, 0x00);
	write_cmos_sensor(0x76f7, 0x02);
	write_cmos_sensor(0x76f8, 0x05);
	write_cmos_sensor(0x76f9, 0x78);
	write_cmos_sensor(0x76fa, 0x00);
	write_cmos_sensor(0x76fb, 0x01);
	write_cmos_sensor(0x76fc, 0x05);
	write_cmos_sensor(0x76fd, 0x7c);
	write_cmos_sensor(0x76fe, 0x03);
	write_cmos_sensor(0x76ff, 0x96);
	write_cmos_sensor(0x7700, 0x05);
	write_cmos_sensor(0x7701, 0x83);
	write_cmos_sensor(0x7702, 0x05);
	write_cmos_sensor(0x7703, 0x03);
	write_cmos_sensor(0x7704, 0x05);
	write_cmos_sensor(0x7705, 0x32);
	write_cmos_sensor(0x7706, 0x05);
	write_cmos_sensor(0x7707, 0x30);
	write_cmos_sensor(0x7708, 0x00);
	write_cmos_sensor(0x7709, 0x02);
	write_cmos_sensor(0x770a, 0x05);
	write_cmos_sensor(0x770b, 0x78);
	write_cmos_sensor(0x770c, 0x00);
	write_cmos_sensor(0x770d, 0x01);
	write_cmos_sensor(0x770e, 0x05);
	write_cmos_sensor(0x770f, 0x7c);
	write_cmos_sensor(0x7710, 0x03);
	write_cmos_sensor(0x7711, 0x95);
	write_cmos_sensor(0x7712, 0x05);
	write_cmos_sensor(0x7713, 0x83);
	write_cmos_sensor(0x7714, 0x05);
	write_cmos_sensor(0x7715, 0x03);
	write_cmos_sensor(0x7716, 0x05);
	write_cmos_sensor(0x7717, 0x32);
	write_cmos_sensor(0x7718, 0x05);
	write_cmos_sensor(0x7719, 0x30);
	write_cmos_sensor(0x771a, 0x00);
	write_cmos_sensor(0x771b, 0x02);
	write_cmos_sensor(0x771c, 0x05);
	write_cmos_sensor(0x771d, 0x78);
	write_cmos_sensor(0x771e, 0x00);
	write_cmos_sensor(0x771f, 0x01);
	write_cmos_sensor(0x7720, 0x05);
	write_cmos_sensor(0x7721, 0x7c);
	write_cmos_sensor(0x7722, 0x03);
	write_cmos_sensor(0x7723, 0x94);
	write_cmos_sensor(0x7724, 0x05);
	write_cmos_sensor(0x7725, 0x83);
	write_cmos_sensor(0x7726, 0x00);
	write_cmos_sensor(0x7727, 0x01);
	write_cmos_sensor(0x7728, 0x05);
	write_cmos_sensor(0x7729, 0x03);
	write_cmos_sensor(0x772a, 0x00);
	write_cmos_sensor(0x772b, 0x01);
	write_cmos_sensor(0x772c, 0x05);
	write_cmos_sensor(0x772d, 0x32);
	write_cmos_sensor(0x772e, 0x05);
	write_cmos_sensor(0x772f, 0x30);
	write_cmos_sensor(0x7730, 0x00);
	write_cmos_sensor(0x7731, 0x02);
	write_cmos_sensor(0x7732, 0x05);
	write_cmos_sensor(0x7733, 0x78);
	write_cmos_sensor(0x7734, 0x00);
	write_cmos_sensor(0x7735, 0x01);
	write_cmos_sensor(0x7736, 0x05);
	write_cmos_sensor(0x7737, 0x7c);
	write_cmos_sensor(0x7738, 0x03);
	write_cmos_sensor(0x7739, 0x93);
	write_cmos_sensor(0x773a, 0x05);
	write_cmos_sensor(0x773b, 0x83);
	write_cmos_sensor(0x773c, 0x00);
	write_cmos_sensor(0x773d, 0x00);
	write_cmos_sensor(0x773e, 0x05);
	write_cmos_sensor(0x773f, 0x03);
	write_cmos_sensor(0x7740, 0x00);
	write_cmos_sensor(0x7741, 0x00);
	write_cmos_sensor(0x7742, 0x05);
	write_cmos_sensor(0x7743, 0x32);
	write_cmos_sensor(0x7744, 0x05);
	write_cmos_sensor(0x7745, 0x30);
	write_cmos_sensor(0x7746, 0x00);
	write_cmos_sensor(0x7747, 0x02);
	write_cmos_sensor(0x7748, 0x05);
	write_cmos_sensor(0x7749, 0x78);
	write_cmos_sensor(0x774a, 0x00);
	write_cmos_sensor(0x774b, 0x01);
	write_cmos_sensor(0x774c, 0x05);
	write_cmos_sensor(0x774d, 0x7c);
	write_cmos_sensor(0x774e, 0x03);
	write_cmos_sensor(0x774f, 0x92);
	write_cmos_sensor(0x7750, 0x05);
	write_cmos_sensor(0x7751, 0x83);
	write_cmos_sensor(0x7752, 0x05);
	write_cmos_sensor(0x7753, 0x03);
	write_cmos_sensor(0x7754, 0x00);
	write_cmos_sensor(0x7755, 0x00);
	write_cmos_sensor(0x7756, 0x05);
	write_cmos_sensor(0x7757, 0x32);
	write_cmos_sensor(0x7758, 0x05);
	write_cmos_sensor(0x7759, 0x30);
	write_cmos_sensor(0x775a, 0x00);
	write_cmos_sensor(0x775b, 0x02);
	write_cmos_sensor(0x775c, 0x05);
	write_cmos_sensor(0x775d, 0x78);
	write_cmos_sensor(0x775e, 0x00);
	write_cmos_sensor(0x775f, 0x01);
	write_cmos_sensor(0x7760, 0x05);
	write_cmos_sensor(0x7761, 0x7c);
	write_cmos_sensor(0x7762, 0x03);
	write_cmos_sensor(0x7763, 0x91);
	write_cmos_sensor(0x7764, 0x05);
	write_cmos_sensor(0x7765, 0x83);
	write_cmos_sensor(0x7766, 0x05);
	write_cmos_sensor(0x7767, 0x03);
	write_cmos_sensor(0x7768, 0x05);
	write_cmos_sensor(0x7769, 0x32);
	write_cmos_sensor(0x776a, 0x05);
	write_cmos_sensor(0x776b, 0x30);
	write_cmos_sensor(0x776c, 0x00);
	write_cmos_sensor(0x776d, 0x02);
	write_cmos_sensor(0x776e, 0x05);
	write_cmos_sensor(0x776f, 0x78);
	write_cmos_sensor(0x7770, 0x00);
	write_cmos_sensor(0x7771, 0x01);
	write_cmos_sensor(0x7772, 0x05);
	write_cmos_sensor(0x7773, 0x7c);
	write_cmos_sensor(0x7774, 0x03);
	write_cmos_sensor(0x7775, 0x90);
	write_cmos_sensor(0x7776, 0x05);
	write_cmos_sensor(0x7777, 0x83);
	write_cmos_sensor(0x7778, 0x05);
	write_cmos_sensor(0x7779, 0x03);
	write_cmos_sensor(0x777a, 0x05);
	write_cmos_sensor(0x777b, 0x32);
	write_cmos_sensor(0x777c, 0x05);
	write_cmos_sensor(0x777d, 0x30);
	write_cmos_sensor(0x777e, 0x00);
	write_cmos_sensor(0x777f, 0x02);
	write_cmos_sensor(0x7780, 0x05);
	write_cmos_sensor(0x7781, 0x78);
	write_cmos_sensor(0x7782, 0x00);
	write_cmos_sensor(0x7783, 0x01);
	write_cmos_sensor(0x7784, 0x05);
	write_cmos_sensor(0x7785, 0x7c);
	write_cmos_sensor(0x7786, 0x02);
	write_cmos_sensor(0x7787, 0x90);
	write_cmos_sensor(0x7788, 0x05);
	write_cmos_sensor(0x7789, 0x03);
	write_cmos_sensor(0x778a, 0x07);
	write_cmos_sensor(0x778b, 0x00);
	write_cmos_sensor(0x778c, 0x0f);
	write_cmos_sensor(0x778d, 0x00);
	write_cmos_sensor(0x778e, 0x08);
	write_cmos_sensor(0x778f, 0x30);
	write_cmos_sensor(0x7790, 0x08);
	write_cmos_sensor(0x7791, 0xee);
	write_cmos_sensor(0x7792, 0x0f);
	write_cmos_sensor(0x7793, 0x00);
	write_cmos_sensor(0x7794, 0x05);
	write_cmos_sensor(0x7795, 0x33);
	write_cmos_sensor(0x7796, 0x04);
	write_cmos_sensor(0x7797, 0xe5);
	write_cmos_sensor(0x7798, 0x06);
	write_cmos_sensor(0x7799, 0x52);
	write_cmos_sensor(0x779a, 0x04);
	write_cmos_sensor(0x779b, 0xe4);
	write_cmos_sensor(0x779c, 0x00);
	write_cmos_sensor(0x779d, 0x00);
	write_cmos_sensor(0x779e, 0x06);
	write_cmos_sensor(0x779f, 0x5e);
	write_cmos_sensor(0x77a0, 0x00);
	write_cmos_sensor(0x77a1, 0x0f);
	write_cmos_sensor(0x77a2, 0x06);
	write_cmos_sensor(0x77a3, 0x1e);
	write_cmos_sensor(0x77a4, 0x00);
	write_cmos_sensor(0x77a5, 0x02);
	write_cmos_sensor(0x77a6, 0x06);
	write_cmos_sensor(0x77a7, 0xa2);
	write_cmos_sensor(0x77a8, 0x00);
	write_cmos_sensor(0x77a9, 0x01);
	write_cmos_sensor(0x77aa, 0x06);
	write_cmos_sensor(0x77ab, 0xae);
	write_cmos_sensor(0x77ac, 0x00);
	write_cmos_sensor(0x77ad, 0x03);
	write_cmos_sensor(0x77ae, 0x05);
	write_cmos_sensor(0x77af, 0x30);
	write_cmos_sensor(0x77b0, 0x09);
	write_cmos_sensor(0x77b1, 0x19);
	write_cmos_sensor(0x77b2, 0x0f);
	write_cmos_sensor(0x77b3, 0x00);
	write_cmos_sensor(0x77b4, 0x05);
	write_cmos_sensor(0x77b5, 0x33);
	write_cmos_sensor(0x77b6, 0x04);
	write_cmos_sensor(0x77b7, 0xe5);
	write_cmos_sensor(0x77b8, 0x06);
	write_cmos_sensor(0x77b9, 0x52);
	write_cmos_sensor(0x77ba, 0x04);
	write_cmos_sensor(0x77bb, 0xe4);
	write_cmos_sensor(0x77bc, 0x00);
	write_cmos_sensor(0x77bd, 0x00);
	write_cmos_sensor(0x77be, 0x06);
	write_cmos_sensor(0x77bf, 0x5e);
	write_cmos_sensor(0x77c0, 0x00);
	write_cmos_sensor(0x77c1, 0x0f);
	write_cmos_sensor(0x77c2, 0x06);
	write_cmos_sensor(0x77c3, 0x1e);
	write_cmos_sensor(0x77c4, 0x00);
	write_cmos_sensor(0x77c5, 0x02);
	write_cmos_sensor(0x77c6, 0x06);
	write_cmos_sensor(0x77c7, 0xa2);
	write_cmos_sensor(0x77c8, 0x00);
	write_cmos_sensor(0x77c9, 0x01);
	write_cmos_sensor(0x77ca, 0x06);
	write_cmos_sensor(0x77cb, 0xae);
	write_cmos_sensor(0x77cc, 0x00);
	write_cmos_sensor(0x77cd, 0x03);
	write_cmos_sensor(0x77ce, 0x05);
	write_cmos_sensor(0x77cf, 0x30);
	write_cmos_sensor(0x77d0, 0x0f);
	write_cmos_sensor(0x77d1, 0x00);
	write_cmos_sensor(0x77d2, 0x00);
	write_cmos_sensor(0x77d3, 0x00);
	write_cmos_sensor(0x77d4, 0x00);
	write_cmos_sensor(0x77d5, 0x02);
	write_cmos_sensor(0x77d6, 0x04);
	write_cmos_sensor(0x77d7, 0xe5);
	write_cmos_sensor(0x77d8, 0x04);
	write_cmos_sensor(0x77d9, 0xe4);
	write_cmos_sensor(0x77da, 0x05);
	write_cmos_sensor(0x77db, 0x33);
	write_cmos_sensor(0x77dc, 0x07);
	write_cmos_sensor(0x77dd, 0x10);
	write_cmos_sensor(0x77de, 0x00);
	write_cmos_sensor(0x77df, 0x00);
	write_cmos_sensor(0x77e0, 0x01);
	write_cmos_sensor(0x77e1, 0xbb);
	write_cmos_sensor(0x77e2, 0x00);
	write_cmos_sensor(0x77e3, 0x00);
	write_cmos_sensor(0x77e4, 0x01);
	write_cmos_sensor(0x77e5, 0xaa);
	write_cmos_sensor(0x77e6, 0x00);
	write_cmos_sensor(0x77e7, 0x00);
	write_cmos_sensor(0x77e8, 0x01);
	write_cmos_sensor(0x77e9, 0x99);
	write_cmos_sensor(0x77ea, 0x00);
	write_cmos_sensor(0x77eb, 0x00);
	write_cmos_sensor(0x77ec, 0x01);
	write_cmos_sensor(0x77ed, 0x88);
	write_cmos_sensor(0x77ee, 0x00);
	write_cmos_sensor(0x77ef, 0x00);
	write_cmos_sensor(0x77f0, 0x01);
	write_cmos_sensor(0x77f1, 0x77);
	write_cmos_sensor(0x77f2, 0x00);
	write_cmos_sensor(0x77f3, 0x00);
	write_cmos_sensor(0x77f4, 0x01);
	write_cmos_sensor(0x77f5, 0x66);
	write_cmos_sensor(0x77f6, 0x00);
	write_cmos_sensor(0x77f7, 0x00);
	write_cmos_sensor(0x77f8, 0x01);
	write_cmos_sensor(0x77f9, 0x55);
	write_cmos_sensor(0x77fa, 0x00);
	write_cmos_sensor(0x77fb, 0x00);
	write_cmos_sensor(0x77fc, 0x01);
	write_cmos_sensor(0x77fd, 0x44);
	write_cmos_sensor(0x77fe, 0x00);
	write_cmos_sensor(0x77ff, 0x00);
	write_cmos_sensor(0x7800, 0x01);
	write_cmos_sensor(0x7801, 0x33);
	write_cmos_sensor(0x7802, 0x00);
	write_cmos_sensor(0x7803, 0x00);
	write_cmos_sensor(0x7804, 0x01);
	write_cmos_sensor(0x7805, 0x22);
	write_cmos_sensor(0x7806, 0x00);
	write_cmos_sensor(0x7807, 0x00);
	write_cmos_sensor(0x7808, 0x01);
	write_cmos_sensor(0x7809, 0x11);
	write_cmos_sensor(0x780a, 0x00);
	write_cmos_sensor(0x780b, 0x00);
	write_cmos_sensor(0x780c, 0x01);
	write_cmos_sensor(0x780d, 0x00);
	write_cmos_sensor(0x780e, 0x01);
	write_cmos_sensor(0x780f, 0xff);
	write_cmos_sensor(0x7810, 0x07);
	write_cmos_sensor(0x7811, 0x00);
	write_cmos_sensor(0x7812, 0x02);
	write_cmos_sensor(0x7813, 0xa0);
	write_cmos_sensor(0x7814, 0x0f);
	write_cmos_sensor(0x7815, 0x00);
	write_cmos_sensor(0x7816, 0x08);
	write_cmos_sensor(0x7817, 0x35);
	write_cmos_sensor(0x7818, 0x06);
	write_cmos_sensor(0x7819, 0x52);
	write_cmos_sensor(0x781a, 0x04);
	write_cmos_sensor(0x781b, 0xe4);
	write_cmos_sensor(0x781c, 0x00);
	write_cmos_sensor(0x781d, 0x00);
	write_cmos_sensor(0x781e, 0x06);
	write_cmos_sensor(0x781f, 0x5e);
	write_cmos_sensor(0x7820, 0x05);
	write_cmos_sensor(0x7821, 0x33);
	write_cmos_sensor(0x7822, 0x09);
	write_cmos_sensor(0x7823, 0x19);
	write_cmos_sensor(0x7824, 0x06);
	write_cmos_sensor(0x7825, 0x1e);
	write_cmos_sensor(0x7826, 0x05);
	write_cmos_sensor(0x7827, 0x33);
	write_cmos_sensor(0x7828, 0x00);
	write_cmos_sensor(0x7829, 0x01);
	write_cmos_sensor(0x782a, 0x06);
	write_cmos_sensor(0x782b, 0x24);
	write_cmos_sensor(0x782c, 0x06);
	write_cmos_sensor(0x782d, 0x20);
	write_cmos_sensor(0x782e, 0x0f);
	write_cmos_sensor(0x782f, 0x00);
	write_cmos_sensor(0x7830, 0x08);
	write_cmos_sensor(0x7831, 0x35);
	write_cmos_sensor(0x7832, 0x07);
	write_cmos_sensor(0x7833, 0x10);
	write_cmos_sensor(0x7834, 0x00);
	write_cmos_sensor(0x7835, 0x00);
	write_cmos_sensor(0x7836, 0x01);
	write_cmos_sensor(0x7837, 0xbb);
	write_cmos_sensor(0x7838, 0x00);
	write_cmos_sensor(0x7839, 0x00);
	write_cmos_sensor(0x783a, 0x01);
	write_cmos_sensor(0x783b, 0xaa);
	write_cmos_sensor(0x783c, 0x00);
	write_cmos_sensor(0x783d, 0x00);
	write_cmos_sensor(0x783e, 0x01);
	write_cmos_sensor(0x783f, 0x99);
	write_cmos_sensor(0x7840, 0x00);
	write_cmos_sensor(0x7841, 0x00);
	write_cmos_sensor(0x7842, 0x01);
	write_cmos_sensor(0x7843, 0x88);
	write_cmos_sensor(0x7844, 0x00);
	write_cmos_sensor(0x7845, 0x00);
	write_cmos_sensor(0x7846, 0x01);
	write_cmos_sensor(0x7847, 0x77);
	write_cmos_sensor(0x7848, 0x00);
	write_cmos_sensor(0x7849, 0x00);
	write_cmos_sensor(0x784a, 0x01);
	write_cmos_sensor(0x784b, 0x66);
	write_cmos_sensor(0x784c, 0x00);
	write_cmos_sensor(0x784d, 0x00);
	write_cmos_sensor(0x784e, 0x01);
	write_cmos_sensor(0x784f, 0x55);
	write_cmos_sensor(0x7850, 0x00);
	write_cmos_sensor(0x7851, 0x00);
	write_cmos_sensor(0x7852, 0x01);
	write_cmos_sensor(0x7853, 0x44);
	write_cmos_sensor(0x7854, 0x00);
	write_cmos_sensor(0x7855, 0x00);
	write_cmos_sensor(0x7856, 0x01);
	write_cmos_sensor(0x7857, 0x33);
	write_cmos_sensor(0x7858, 0x00);
	write_cmos_sensor(0x7859, 0x00);
	write_cmos_sensor(0x785a, 0x01);
	write_cmos_sensor(0x785b, 0x22);
	write_cmos_sensor(0x785c, 0x00);
	write_cmos_sensor(0x785d, 0x00);
	write_cmos_sensor(0x785e, 0x01);
	write_cmos_sensor(0x785f, 0x11);
	write_cmos_sensor(0x7860, 0x00);
	write_cmos_sensor(0x7861, 0x00);
	write_cmos_sensor(0x7862, 0x01);
	write_cmos_sensor(0x7863, 0x00);
	write_cmos_sensor(0x7864, 0x07);
	write_cmos_sensor(0x7865, 0x00);
	write_cmos_sensor(0x7866, 0x01);
	write_cmos_sensor(0x7867, 0xff);
	write_cmos_sensor(0x7868, 0x02);
	write_cmos_sensor(0x7869, 0xa0);
	write_cmos_sensor(0x786a, 0x0f);
	write_cmos_sensor(0x786b, 0x00);
	write_cmos_sensor(0x786c, 0x08);
	write_cmos_sensor(0x786d, 0x3a);
	write_cmos_sensor(0x786e, 0x08);
	write_cmos_sensor(0x786f, 0x6a);
	write_cmos_sensor(0x7870, 0x0f);
	write_cmos_sensor(0x7871, 0x00);
	write_cmos_sensor(0x7872, 0x04);
	write_cmos_sensor(0x7873, 0xc0);
	write_cmos_sensor(0x7874, 0x09);
	write_cmos_sensor(0x7875, 0x19);
	write_cmos_sensor(0x7876, 0x04);
	write_cmos_sensor(0x7877, 0x99);
	write_cmos_sensor(0x7878, 0x07);
	write_cmos_sensor(0x7879, 0x14);
	write_cmos_sensor(0x787a, 0x00);
	write_cmos_sensor(0x787b, 0x01);
	write_cmos_sensor(0x787c, 0x04);
	write_cmos_sensor(0x787d, 0xa4);
	write_cmos_sensor(0x787e, 0x00);
	write_cmos_sensor(0x787f, 0x07);
	write_cmos_sensor(0x7880, 0x04);
	write_cmos_sensor(0x7881, 0xa6);
	write_cmos_sensor(0x7882, 0x00);
	write_cmos_sensor(0x7883, 0x00);
	write_cmos_sensor(0x7884, 0x04);
	write_cmos_sensor(0x7885, 0xa0);
	write_cmos_sensor(0x7886, 0x04);
	write_cmos_sensor(0x7887, 0x80);
	write_cmos_sensor(0x7888, 0x04);
	write_cmos_sensor(0x7889, 0x00);
	write_cmos_sensor(0x788a, 0x05);
	write_cmos_sensor(0x788b, 0x03);
	write_cmos_sensor(0x788c, 0x06);
	write_cmos_sensor(0x788d, 0x00);
	write_cmos_sensor(0x788e, 0x0f);
	write_cmos_sensor(0x788f, 0x00);
	write_cmos_sensor(0x7890, 0x0f);
	write_cmos_sensor(0x7891, 0x00);
	write_cmos_sensor(0x7892, 0x0f);
	write_cmos_sensor(0x7893, 0x00);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x0a);
	write_cmos_sensor(0x3195, 0x29);
	write_cmos_sensor(0x315a, 0x02);
	write_cmos_sensor(0x315b, 0x00);
	write_cmos_sensor(0x30bb, 0x40);
	write_cmos_sensor(0x33e2, 0x02);
	write_cmos_sensor(0x33e3, 0x01);
	write_cmos_sensor(0x33e4, 0x01);
	write_cmos_sensor(0x33e5, 0x01);
	write_cmos_sensor(0x33e8, 0x0c);
	write_cmos_sensor(0x33e9, 0x02);
	write_cmos_sensor(0x33ea, 0x02);
	write_cmos_sensor(0x33eb, 0x02);
	write_cmos_sensor(0x33ec, 0x03);
	write_cmos_sensor(0x33ed, 0x01);
	write_cmos_sensor(0x33ee, 0x02);
	write_cmos_sensor(0x33ef, 0x08);
	write_cmos_sensor(0x33f7, 0x02);
	write_cmos_sensor(0x33f8, 0x01);
	write_cmos_sensor(0x33f9, 0x01);
	write_cmos_sensor(0x33fa, 0x01);
	write_cmos_sensor(0x33fd, 0x0c);
	write_cmos_sensor(0x33fe, 0x02);
	write_cmos_sensor(0x33ff, 0x02);
	write_cmos_sensor(0x3400, 0x02);
	write_cmos_sensor(0x3401, 0x03);
	write_cmos_sensor(0x3402, 0x01);
	write_cmos_sensor(0x3403, 0x02);
	write_cmos_sensor(0x3404, 0x08);
	write_cmos_sensor(0x3250, 0xf7);
	write_cmos_sensor(0x3288, 0x2a);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x15);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x0a);
	write_cmos_sensor(0x328d, 0x80);
	write_cmos_sensor(0x328e, 0x05);
	write_cmos_sensor(0x328f, 0x40);
	write_cmos_sensor(0x3290, 0x54);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x2a);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x15);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x0a);
	write_cmos_sensor(0x3297, 0x80);
	write_cmos_sensor(0x3298, 0x7f);
	write_cmos_sensor(0x3299, 0xff);
	write_cmos_sensor(0x329a, 0x54);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x2a);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x15);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x7f);
	write_cmos_sensor(0x32a3, 0xff);
	write_cmos_sensor(0x32a4, 0x54);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x2a);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0x98);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x6d);
	write_cmos_sensor(0x3197, 0x00);
	write_cmos_sensor(0x3195, 0x29);
	write_cmos_sensor(0x315a, 0x02);
	write_cmos_sensor(0x315b, 0x86);
	write_cmos_sensor(0x315c, 0x02);
	write_cmos_sensor(0x315d, 0x86);
	write_cmos_sensor(0x304b, 0x00);
	write_cmos_sensor(0x304d, 0xa4);
	write_cmos_sensor(0x3033, 0x30);
	write_cmos_sensor(0x31A3, 0x08);
	write_cmos_sensor(0x31FE, 0x03);
	write_cmos_sensor(0x3195, 0x11);
	write_cmos_sensor(0x3288, 0x2a);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x15);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x0a);
	write_cmos_sensor(0x328d, 0x80);
	write_cmos_sensor(0x328e, 0x05);
	write_cmos_sensor(0x328f, 0x40);
	write_cmos_sensor(0x3290, 0x54);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x2a);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x15);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x0a);
	write_cmos_sensor(0x3297, 0x80);
	write_cmos_sensor(0x3298, 0x7f);
	write_cmos_sensor(0x3299, 0xff);
	write_cmos_sensor(0x329a, 0x54);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x2a);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x15);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x7f);
	write_cmos_sensor(0x32a3, 0xff);
	write_cmos_sensor(0x32a4, 0x54);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x2a);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0x98);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x6d);
	write_cmos_sensor(0x3197, 0x00);
	write_cmos_sensor(0x3195, 0x29);
	write_cmos_sensor(0x315a, 0x02);
	write_cmos_sensor(0x315b, 0x86);
	write_cmos_sensor(0x315c, 0x02);
	write_cmos_sensor(0x315d, 0x86);
	write_cmos_sensor(0x304b, 0x00);
	write_cmos_sensor(0x304d, 0xa4);
	write_cmos_sensor(0x3033, 0x30);
	write_cmos_sensor(0x31A3, 0x08);
	write_cmos_sensor(0x31FE, 0x03);
#endif
	cam_pr_debug("exit ov2718 r2a nhdr setting\n");
	mDELAY(10);

	cam_pr_debug("mediaserver: ov2718r2a nhdr setting\n");
}

#if 0
static void sensor_init(void)
{
	sensor_nhdr();
	cam_pr_debug("OV2718r2a Sensor init E\n");
}
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	cam_pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n",
			enable);
	if (enable)

		write_cmos_sensor(0x3012, 0x01);/*Stream on*/
	else
		write_cmos_sensor(0x3012, 0x00);/*Stream Off*/

	mdelay(10);
	return ERROR_NONE;
}    /*    MIPI_sensor_Init  */

static void preview_setting(void)
{
	cam_pr_debug("mediaserver:ov2718r2a %s\n", __func__);
	if (imgsensor.hdr_mode == 2)
		sensor_hdr();
	else
		sensor_nhdr();
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	cam_pr_debug("E! currefps:%d\n", currefps);

	if (imgsensor.hdr_mode == 2)
		sensor_hdr();
	else
		sensor_nhdr();
}

static void normal_video_setting(kal_uint16 currefps)
{
	cam_pr_debug("E! currefps:%d\n", currefps);
}

static void video_1080p_setting(void)
{
}

static void video_720p_setting(void)
{
	cam_pr_debug("E\n");
	write_cmos_sensor(0x3013, 0x01);
	mDELAY(10);
	write_cmos_sensor(0x3000, 0x02);
	write_cmos_sensor(0x3001, 0x1d);
	write_cmos_sensor(0x3002, 0x03);
	write_cmos_sensor(0x3003, 0x01);
	write_cmos_sensor(0x3004, 0x02);
	write_cmos_sensor(0x3005, 0x1b);
	write_cmos_sensor(0x3006, 0x00);
	write_cmos_sensor(0x3007, 0x07);
	write_cmos_sensor(0x3008, 0x01);
	write_cmos_sensor(0x3009, 0x00);
	write_cmos_sensor(0x300c, 0x6c);
	write_cmos_sensor(0x300e, 0x80);
	write_cmos_sensor(0x300f, 0x00);
	write_cmos_sensor(0x3012, 0x00); //Stream Off
	write_cmos_sensor(0x3013, 0x00);
	write_cmos_sensor(0x3014, 0xc4);
	write_cmos_sensor(0x3015, 0x00);
	write_cmos_sensor(0x3017, 0x00);
	write_cmos_sensor(0x3018, 0x00);
	write_cmos_sensor(0x3019, 0x00);
	write_cmos_sensor(0x301a, 0x00);
	write_cmos_sensor(0x301b, 0x0e);
	write_cmos_sensor(0x301e, 0x17);
	write_cmos_sensor(0x301f, 0xe1);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x3031, 0x62);
	write_cmos_sensor(0x3032, 0xf0);
	write_cmos_sensor(0x3033, 0x30);
	write_cmos_sensor(0x3034, 0x3f);
	write_cmos_sensor(0x3035, 0x5f);
	write_cmos_sensor(0x3036, 0x02);
	write_cmos_sensor(0x3037, 0x9f);
	write_cmos_sensor(0x3038, 0x04);
	write_cmos_sensor(0x3039, 0xb7);
	write_cmos_sensor(0x303a, 0x04);
	write_cmos_sensor(0x303b, 0x07);
	write_cmos_sensor(0x303c, 0xf0);
	write_cmos_sensor(0x303d, 0x00);
	write_cmos_sensor(0x303e, 0x0b);
	write_cmos_sensor(0x303f, 0xe3);
	write_cmos_sensor(0x3040, 0xf3);
	write_cmos_sensor(0x3041, 0x29);
	write_cmos_sensor(0x3042, 0xf6);
	write_cmos_sensor(0x3043, 0x65);
	write_cmos_sensor(0x3044, 0x06);
	write_cmos_sensor(0x3045, 0x0f);
	write_cmos_sensor(0x3046, 0x59);
	write_cmos_sensor(0x3047, 0x07);
	write_cmos_sensor(0x3048, 0x82);
	write_cmos_sensor(0x3049, 0xcf);
	write_cmos_sensor(0x304a, 0x12);
	write_cmos_sensor(0x304b, 0x40);
	write_cmos_sensor(0x304c, 0x33);
	write_cmos_sensor(0x304d, 0xa4);
	write_cmos_sensor(0x304e, 0x0b);
	write_cmos_sensor(0x304f, 0x3d);
	write_cmos_sensor(0x3050, 0x10);
	write_cmos_sensor(0x3060, 0x00);
	write_cmos_sensor(0x3061, 0x64);
	write_cmos_sensor(0x3062, 0x00);
	write_cmos_sensor(0x3063, 0xe4);
	write_cmos_sensor(0x3066, 0x80);
	write_cmos_sensor(0x3080, 0x00);
	write_cmos_sensor(0x3081, 0x00);
	write_cmos_sensor(0x3082, 0x01);
	write_cmos_sensor(0x3083, 0xe3);
	write_cmos_sensor(0x3084, 0x06);
	write_cmos_sensor(0x3085, 0x00);
	write_cmos_sensor(0x3086, 0x10);
	write_cmos_sensor(0x3087, 0x10);
	write_cmos_sensor(0x3089, 0x00);
	write_cmos_sensor(0x308a, 0x01);
	write_cmos_sensor(0x3093, 0x00);
	write_cmos_sensor(0x30a0, 0x00);
	write_cmos_sensor(0x30a1, 0x04);
	write_cmos_sensor(0x30a2, 0x00);
	write_cmos_sensor(0x30a3, 0x08);
	write_cmos_sensor(0x30a4, 0x07);
	write_cmos_sensor(0x30a5, 0x8b);
	write_cmos_sensor(0x30a6, 0x04);
	write_cmos_sensor(0x30a7, 0x3f);
	write_cmos_sensor(0x30a8, 0x00);
	write_cmos_sensor(0x30a9, 0x04);
	write_cmos_sensor(0x30aa, 0x00);
	write_cmos_sensor(0x30ab, 0x00);
	write_cmos_sensor(0x30ac, 0x07);
	write_cmos_sensor(0x30ad, 0x80);
	write_cmos_sensor(0x30ae, 0x04);
	write_cmos_sensor(0x30af, 0x38);

	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0x98);
	write_cmos_sensor(0x30b2, 0x04);
	write_cmos_sensor(0x30b3, 0x65);
	write_cmos_sensor(0x30b4, 0x00);
	write_cmos_sensor(0x30b5, 0x00);
	write_cmos_sensor(0x30b6, 0x00);
	write_cmos_sensor(0x30b7, 0x10);
	write_cmos_sensor(0x30b8, 0x00);
	write_cmos_sensor(0x30b9, 0x02);
	write_cmos_sensor(0x30ba, 0x10);
	write_cmos_sensor(0x30bb, 0x00);
	write_cmos_sensor(0x30bc, 0x00);
	write_cmos_sensor(0x30bd, 0x03);
	write_cmos_sensor(0x30be, 0x5c);
	write_cmos_sensor(0x30bf, 0x00);
	write_cmos_sensor(0x30c0, 0x04);
	write_cmos_sensor(0x30c1, 0x00);
	write_cmos_sensor(0x30c2, 0x20);
	write_cmos_sensor(0x30c3, 0x00);
	write_cmos_sensor(0x30c4, 0x4a);
	write_cmos_sensor(0x30c5, 0x00);
	write_cmos_sensor(0x30c7, 0x00);
	write_cmos_sensor(0x30c8, 0x00);
	write_cmos_sensor(0x30d1, 0x00);
	write_cmos_sensor(0x30d2, 0x00);
	write_cmos_sensor(0x30d3, 0x80);
	write_cmos_sensor(0x30d4, 0x00);
	write_cmos_sensor(0x30d9, 0x09);
	write_cmos_sensor(0x30da, 0x64);
	write_cmos_sensor(0x30dd, 0x00);
	write_cmos_sensor(0x30de, 0x16);
	write_cmos_sensor(0x30df, 0x00);
	write_cmos_sensor(0x30e0, 0x17);
	write_cmos_sensor(0x30e1, 0x00);
	write_cmos_sensor(0x30e2, 0x18);
	write_cmos_sensor(0x30e3, 0x10);
	write_cmos_sensor(0x30e4, 0x04);
	write_cmos_sensor(0x30e5, 0x00);
	write_cmos_sensor(0x30e6, 0x00);
	write_cmos_sensor(0x30e7, 0x00);
	write_cmos_sensor(0x30e8, 0x00);
	write_cmos_sensor(0x30e9, 0x00);
	write_cmos_sensor(0x30ea, 0x00);
	write_cmos_sensor(0x30eb, 0x00);
	write_cmos_sensor(0x30ec, 0x00);
	write_cmos_sensor(0x30ed, 0x00);
	write_cmos_sensor(0x3101, 0x00);
	write_cmos_sensor(0x3102, 0x00);
	write_cmos_sensor(0x3103, 0x00);
	write_cmos_sensor(0x3104, 0x00);
	write_cmos_sensor(0x3105, 0x8c);
	write_cmos_sensor(0x3106, 0x87);
	write_cmos_sensor(0x3107, 0xa4);
	write_cmos_sensor(0x3108, 0x9d);
	write_cmos_sensor(0x3109, 0x89);
	write_cmos_sensor(0x310a, 0x8a);
	write_cmos_sensor(0x310b, 0x6a);
	write_cmos_sensor(0x310c, 0x3a);
	write_cmos_sensor(0x310d, 0x5a);
	write_cmos_sensor(0x310e, 0x00);
	write_cmos_sensor(0x3120, 0x00);
	write_cmos_sensor(0x3121, 0x00);
	write_cmos_sensor(0x3122, 0x00);
	write_cmos_sensor(0x3123, 0x00);
	write_cmos_sensor(0x3124, 0x00);
	write_cmos_sensor(0x3125, 0x70);
	write_cmos_sensor(0x3126, 0x1f);
	write_cmos_sensor(0x3127, 0x0f);
	write_cmos_sensor(0x3128, 0x00);
	write_cmos_sensor(0x3129, 0x3a);
	write_cmos_sensor(0x312a, 0x02);
	write_cmos_sensor(0x312b, 0x0f);
	write_cmos_sensor(0x312c, 0x00);
	write_cmos_sensor(0x312d, 0x0f);
	write_cmos_sensor(0x312e, 0x1d);
	write_cmos_sensor(0x312f, 0x00);
	write_cmos_sensor(0x3130, 0x00);
	write_cmos_sensor(0x3131, 0x00);
	write_cmos_sensor(0x3132, 0x00);
	write_cmos_sensor(0x3140, 0x0a);
	write_cmos_sensor(0x3141, 0x03);
	write_cmos_sensor(0x3142, 0x00);
	write_cmos_sensor(0x3143, 0x00);
	write_cmos_sensor(0x3144, 0x00);
	write_cmos_sensor(0x3145, 0x00);
	write_cmos_sensor(0x3146, 0x00);
	write_cmos_sensor(0x3147, 0x00);
	write_cmos_sensor(0x3148, 0x00);
	write_cmos_sensor(0x3149, 0x00);
	write_cmos_sensor(0x314a, 0x00);
	write_cmos_sensor(0x314b, 0x00);
	write_cmos_sensor(0x314c, 0x00);
	write_cmos_sensor(0x314d, 0x00);
	write_cmos_sensor(0x314e, 0x1c);
	write_cmos_sensor(0x314f, 0xff);
	write_cmos_sensor(0x3150, 0xff);
	write_cmos_sensor(0x3151, 0xff);
	write_cmos_sensor(0x3152, 0x10);
	write_cmos_sensor(0x3153, 0x10);
	write_cmos_sensor(0x3154, 0x10);
	write_cmos_sensor(0x3155, 0x00);
	write_cmos_sensor(0x3156, 0x03);
	write_cmos_sensor(0x3157, 0x00);
	write_cmos_sensor(0x3158, 0x0f);
	write_cmos_sensor(0x3159, 0xff);
	write_cmos_sensor(0x315a, 0x01);
	write_cmos_sensor(0x315b, 0x00);
	write_cmos_sensor(0x315c, 0x01);
	write_cmos_sensor(0x315d, 0x00);
	write_cmos_sensor(0x315e, 0x01);
	write_cmos_sensor(0x315f, 0x00);
	write_cmos_sensor(0x3160, 0x00);
	write_cmos_sensor(0x3161, 0x40);
	write_cmos_sensor(0x3162, 0x00);
	write_cmos_sensor(0x3163, 0x40);
	write_cmos_sensor(0x3164, 0x00);
	write_cmos_sensor(0x3165, 0x40);
	write_cmos_sensor(0x3190, 0x08);
	write_cmos_sensor(0x3191, 0x99);
	write_cmos_sensor(0x3193, 0x08);
	write_cmos_sensor(0x3194, 0x13);
	write_cmos_sensor(0x3195, 0x33);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x10);
	write_cmos_sensor(0x3198, 0x00);
	write_cmos_sensor(0x3199, 0x7f);
	write_cmos_sensor(0x319a, 0x80);
	write_cmos_sensor(0x319b, 0xff);
	write_cmos_sensor(0x319c, 0x80);
	write_cmos_sensor(0x319d, 0xbf);
	write_cmos_sensor(0x319e, 0xc0);
	write_cmos_sensor(0x319f, 0xff);
	write_cmos_sensor(0x31a0, 0x24);
	write_cmos_sensor(0x31a1, 0x55);
	write_cmos_sensor(0x31a2, 0x00);
	write_cmos_sensor(0x31a3, 0x08);
	write_cmos_sensor(0x31a6, 0x00);
	write_cmos_sensor(0x31a7, 0x00);
	write_cmos_sensor(0x31b0, 0x00);
	write_cmos_sensor(0x31b1, 0x00);
	write_cmos_sensor(0x31b2, 0x02);
	write_cmos_sensor(0x31b3, 0x00);
	write_cmos_sensor(0x31b4, 0x00);
	write_cmos_sensor(0x31b5, 0x01);
	write_cmos_sensor(0x31b6, 0x00);
	write_cmos_sensor(0x31b7, 0x00);
	write_cmos_sensor(0x31b8, 0x00);
	write_cmos_sensor(0x31b9, 0x00);
	write_cmos_sensor(0x31ba, 0x00);
	write_cmos_sensor(0x31d0, 0x3c);
	write_cmos_sensor(0x31d1, 0x34);
	write_cmos_sensor(0x31d2, 0x3c);
	write_cmos_sensor(0x31d3, 0x00);
	write_cmos_sensor(0x31d4, 0x2d);
	write_cmos_sensor(0x31d5, 0x00);
	write_cmos_sensor(0x31d6, 0x01);
	write_cmos_sensor(0x31d7, 0x06);
	write_cmos_sensor(0x31d8, 0x00);
	write_cmos_sensor(0x31d9, 0x64);
	write_cmos_sensor(0x31da, 0x00);
	write_cmos_sensor(0x31db, 0x30);
	write_cmos_sensor(0x31dc, 0x04);
	write_cmos_sensor(0x31dd, 0x69);
	write_cmos_sensor(0x31de, 0x0a);
	write_cmos_sensor(0x31df, 0x3c);
	write_cmos_sensor(0x31e0, 0x04);
	write_cmos_sensor(0x31e1, 0x32);
	write_cmos_sensor(0x31e2, 0x00);
	write_cmos_sensor(0x31e3, 0x00);
	write_cmos_sensor(0x31e4, 0x08);
	write_cmos_sensor(0x31e5, 0x80);
	write_cmos_sensor(0x31e6, 0x00);
	write_cmos_sensor(0x31e7, 0x2c);
	write_cmos_sensor(0x31e8, 0x6c);
	write_cmos_sensor(0x31e9, 0xac);
	write_cmos_sensor(0x31ea, 0xec);
	write_cmos_sensor(0x31eb, 0x3f);
	write_cmos_sensor(0x31ec, 0x0f);
	write_cmos_sensor(0x31ed, 0x20);
	write_cmos_sensor(0x31ee, 0x04);
	write_cmos_sensor(0x31ef, 0x48);
	write_cmos_sensor(0x31f0, 0x07);
	write_cmos_sensor(0x31f1, 0x90);
	write_cmos_sensor(0x31f2, 0x04);
	write_cmos_sensor(0x31f3, 0x48);
	write_cmos_sensor(0x31f4, 0x07);
	write_cmos_sensor(0x31f5, 0x90);
	write_cmos_sensor(0x31f6, 0x04);
	write_cmos_sensor(0x31f7, 0x48);
	write_cmos_sensor(0x31f8, 0x07);
	write_cmos_sensor(0x31f9, 0x90);
	write_cmos_sensor(0x31fa, 0x04);
	write_cmos_sensor(0x31fb, 0x48);
	write_cmos_sensor(0x31fd, 0xcb);
	write_cmos_sensor(0x31fe, 0x03);
	write_cmos_sensor(0x31ff, 0x03);
	write_cmos_sensor(0x3200, 0x00);
	write_cmos_sensor(0x3201, 0xff);
	write_cmos_sensor(0x3202, 0x00);
	write_cmos_sensor(0x3203, 0xff);
	write_cmos_sensor(0x3204, 0xff);
	write_cmos_sensor(0x3205, 0xff);
	write_cmos_sensor(0x3206, 0xff);
	write_cmos_sensor(0x3207, 0xff);
	write_cmos_sensor(0x3208, 0xff);
	write_cmos_sensor(0x3209, 0xff);
	write_cmos_sensor(0x320a, 0xff);
	write_cmos_sensor(0x320b, 0x1b);
	write_cmos_sensor(0x320c, 0x1f);
	write_cmos_sensor(0x320d, 0x1e);
	write_cmos_sensor(0x320e, 0x30);
	write_cmos_sensor(0x320f, 0x2d);
	write_cmos_sensor(0x3210, 0x2c);
	write_cmos_sensor(0x3211, 0x2b);
	write_cmos_sensor(0x3212, 0x2a);
	write_cmos_sensor(0x3213, 0x24);
	write_cmos_sensor(0x3214, 0x22);
	write_cmos_sensor(0x3215, 0x00);
	write_cmos_sensor(0x3216, 0x04);
	write_cmos_sensor(0x3217, 0x2c);
	write_cmos_sensor(0x3218, 0x6c);
	write_cmos_sensor(0x3219, 0xac);
	write_cmos_sensor(0x321a, 0xec);
	write_cmos_sensor(0x321b, 0x00);
	write_cmos_sensor(0x3230, 0x3a);
	write_cmos_sensor(0x3231, 0x00);
	write_cmos_sensor(0x3232, 0x80);
	write_cmos_sensor(0x3233, 0x00);
	write_cmos_sensor(0x3234, 0x10);
	write_cmos_sensor(0x3235, 0xaa);
	write_cmos_sensor(0x3236, 0x55);
	write_cmos_sensor(0x3237, 0x99);
	write_cmos_sensor(0x3238, 0x66);
	write_cmos_sensor(0x3239, 0x08);
	write_cmos_sensor(0x323a, 0x88);
	write_cmos_sensor(0x323b, 0x00);
	write_cmos_sensor(0x323c, 0x00);
	write_cmos_sensor(0x323d, 0x03);
	write_cmos_sensor(0x3250, 0x33);
	write_cmos_sensor(0x3251, 0x00);
	write_cmos_sensor(0x3252, 0x21);
	write_cmos_sensor(0x3253, 0x00);
	write_cmos_sensor(0x3254, 0x00);
	write_cmos_sensor(0x3255, 0x01);
	write_cmos_sensor(0x3256, 0x00);
	write_cmos_sensor(0x3257, 0x00);
	write_cmos_sensor(0x3258, 0x00);
	write_cmos_sensor(0x3270, 0x01);
	write_cmos_sensor(0x3271, 0xc0);
	write_cmos_sensor(0x3272, 0xf0);
	write_cmos_sensor(0x3273, 0x01);
	write_cmos_sensor(0x3274, 0x00);
	write_cmos_sensor(0x3275, 0x40);
	write_cmos_sensor(0x3276, 0x02);
	write_cmos_sensor(0x3277, 0x08);
	write_cmos_sensor(0x3278, 0x10);
	write_cmos_sensor(0x3279, 0x04);
	write_cmos_sensor(0x327a, 0x00);
	write_cmos_sensor(0x327b, 0x03);
	write_cmos_sensor(0x327c, 0x10);
	write_cmos_sensor(0x327d, 0x60);
	write_cmos_sensor(0x327e, 0xc0);
	write_cmos_sensor(0x327f, 0x06);
	write_cmos_sensor(0x3288, 0x10);
	write_cmos_sensor(0x3289, 0x00);
	write_cmos_sensor(0x328a, 0x08);
	write_cmos_sensor(0x328b, 0x00);
	write_cmos_sensor(0x328c, 0x04);
	write_cmos_sensor(0x328d, 0x00);
	write_cmos_sensor(0x328e, 0x02);
	write_cmos_sensor(0x328f, 0x00);
	write_cmos_sensor(0x3290, 0x20);
	write_cmos_sensor(0x3291, 0x00);
	write_cmos_sensor(0x3292, 0x10);
	write_cmos_sensor(0x3293, 0x00);
	write_cmos_sensor(0x3294, 0x08);
	write_cmos_sensor(0x3295, 0x00);
	write_cmos_sensor(0x3296, 0x04);
	write_cmos_sensor(0x3297, 0x00);
	write_cmos_sensor(0x3298, 0x40);
	write_cmos_sensor(0x3299, 0x00);
	write_cmos_sensor(0x329a, 0x20);
	write_cmos_sensor(0x329b, 0x00);
	write_cmos_sensor(0x329c, 0x10);
	write_cmos_sensor(0x329d, 0x00);
	write_cmos_sensor(0x329e, 0x08);
	write_cmos_sensor(0x329f, 0x00);
	write_cmos_sensor(0x32a0, 0x7f);
	write_cmos_sensor(0x32a1, 0xff);
	write_cmos_sensor(0x32a2, 0x40);
	write_cmos_sensor(0x32a3, 0x00);
	write_cmos_sensor(0x32a4, 0x20);
	write_cmos_sensor(0x32a5, 0x00);
	write_cmos_sensor(0x32a6, 0x10);
	write_cmos_sensor(0x32a7, 0x00);
	write_cmos_sensor(0x32a8, 0x00);
	write_cmos_sensor(0x32a9, 0x00);
	write_cmos_sensor(0x32aa, 0x00);
	write_cmos_sensor(0x32ab, 0x00);
	write_cmos_sensor(0x32ac, 0x00);
	write_cmos_sensor(0x32ad, 0x00);
	write_cmos_sensor(0x32ae, 0x00);
	write_cmos_sensor(0x32af, 0x00);
	write_cmos_sensor(0x32b0, 0x00);
	write_cmos_sensor(0x32b1, 0x00);
	write_cmos_sensor(0x32b2, 0x00);
	write_cmos_sensor(0x32b3, 0x00);
	write_cmos_sensor(0x32b4, 0x00);
	write_cmos_sensor(0x32b5, 0x00);
	write_cmos_sensor(0x32b6, 0x00);
	write_cmos_sensor(0x32b7, 0x00);
	write_cmos_sensor(0x32b8, 0x00);
	write_cmos_sensor(0x32b9, 0x00);
	write_cmos_sensor(0x32ba, 0x00);
	write_cmos_sensor(0x32bb, 0x00);
	write_cmos_sensor(0x32bc, 0x00);
	write_cmos_sensor(0x32bd, 0x00);
	write_cmos_sensor(0x32be, 0x00);
	write_cmos_sensor(0x32bf, 0x00);
	write_cmos_sensor(0x32c0, 0x00);
	write_cmos_sensor(0x32c1, 0x00);
	write_cmos_sensor(0x32c2, 0x00);
	write_cmos_sensor(0x32c3, 0x00);
	write_cmos_sensor(0x32c4, 0x00);
	write_cmos_sensor(0x32c5, 0x00);
	write_cmos_sensor(0x32c6, 0x00);
	write_cmos_sensor(0x32c7, 0x00);
	write_cmos_sensor(0x32c8, 0x87);
	write_cmos_sensor(0x32c9, 0x00);
	write_cmos_sensor(0x3330, 0x03);
	write_cmos_sensor(0x3331, 0xc8);
	write_cmos_sensor(0x3332, 0x02);
	write_cmos_sensor(0x3333, 0x24);
	write_cmos_sensor(0x3334, 0x00);
	write_cmos_sensor(0x3335, 0x00);
	write_cmos_sensor(0x3336, 0x00);
	write_cmos_sensor(0x3337, 0x00);
	write_cmos_sensor(0x3338, 0x03);
	write_cmos_sensor(0x3339, 0xc8);
	write_cmos_sensor(0x333a, 0x02);
	write_cmos_sensor(0x333b, 0x24);
	write_cmos_sensor(0x333c, 0x00);
	write_cmos_sensor(0x333d, 0x00);
	write_cmos_sensor(0x333e, 0x00);
	write_cmos_sensor(0x333f, 0x00);
	write_cmos_sensor(0x3340, 0x03);
	write_cmos_sensor(0x3341, 0xc8);
	write_cmos_sensor(0x3342, 0x02);
	write_cmos_sensor(0x3343, 0x24);
	write_cmos_sensor(0x3344, 0x00);
	write_cmos_sensor(0x3345, 0x00);
	write_cmos_sensor(0x3346, 0x00);
	write_cmos_sensor(0x3347, 0x00);
	write_cmos_sensor(0x3348, 0x40);
	write_cmos_sensor(0x3349, 0x00);
	write_cmos_sensor(0x334a, 0x00);
	write_cmos_sensor(0x334b, 0x00);
	write_cmos_sensor(0x334c, 0x00);
	write_cmos_sensor(0x334d, 0x00);
	write_cmos_sensor(0x334e, 0x80);
	write_cmos_sensor(0x3360, 0x01);
	write_cmos_sensor(0x3361, 0x00);
	write_cmos_sensor(0x3362, 0x01);
	write_cmos_sensor(0x3363, 0x00);
	write_cmos_sensor(0x3364, 0x01);
	write_cmos_sensor(0x3365, 0x00);
	write_cmos_sensor(0x3366, 0x01);
	write_cmos_sensor(0x3367, 0x00);
	write_cmos_sensor(0x3368, 0x01);
	write_cmos_sensor(0x3369, 0x00);
	write_cmos_sensor(0x336a, 0x01);
	write_cmos_sensor(0x336b, 0x00);
	write_cmos_sensor(0x336c, 0x01);
	write_cmos_sensor(0x336d, 0x00);
	write_cmos_sensor(0x336e, 0x01);
	write_cmos_sensor(0x336f, 0x00);
	write_cmos_sensor(0x3370, 0x01);
	write_cmos_sensor(0x3371, 0x00);
	write_cmos_sensor(0x3372, 0x01);
	write_cmos_sensor(0x3373, 0x00);
	write_cmos_sensor(0x3374, 0x01);
	write_cmos_sensor(0x3375, 0x00);
	write_cmos_sensor(0x3376, 0x01);
	write_cmos_sensor(0x3377, 0x00);
	write_cmos_sensor(0x3378, 0x00);
	write_cmos_sensor(0x3379, 0x00);
	write_cmos_sensor(0x337a, 0x00);
	write_cmos_sensor(0x337b, 0x00);
	write_cmos_sensor(0x337c, 0x00);
	write_cmos_sensor(0x337d, 0x00);
	write_cmos_sensor(0x337e, 0x00);
	write_cmos_sensor(0x337f, 0x00);
	write_cmos_sensor(0x3380, 0x00);
	write_cmos_sensor(0x3381, 0x00);
	write_cmos_sensor(0x3382, 0x00);
	write_cmos_sensor(0x3383, 0x00);
	write_cmos_sensor(0x3384, 0x00);
	write_cmos_sensor(0x3385, 0x00);
	write_cmos_sensor(0x3386, 0x00);
	write_cmos_sensor(0x3387, 0x00);
	write_cmos_sensor(0x3388, 0x00);
	write_cmos_sensor(0x3389, 0x00);
	write_cmos_sensor(0x338a, 0x00);
	write_cmos_sensor(0x338b, 0x00);
	write_cmos_sensor(0x338c, 0x00);
	write_cmos_sensor(0x338d, 0x00);
	write_cmos_sensor(0x338e, 0x00);
	write_cmos_sensor(0x338f, 0x00);
	write_cmos_sensor(0x3390, 0x00);
	write_cmos_sensor(0x3391, 0x00);
	write_cmos_sensor(0x3392, 0x00);
	write_cmos_sensor(0x3393, 0x00);
	write_cmos_sensor(0x3394, 0x00);
	write_cmos_sensor(0x3395, 0x00);
	write_cmos_sensor(0x3396, 0x00);
	write_cmos_sensor(0x3397, 0x00);
	write_cmos_sensor(0x3398, 0x00);
	write_cmos_sensor(0x3399, 0x00);
	write_cmos_sensor(0x339a, 0x00);
	write_cmos_sensor(0x339b, 0x00);
	write_cmos_sensor(0x33b0, 0x00);
	write_cmos_sensor(0x33b1, 0x50);
	write_cmos_sensor(0x33b2, 0x01);
	write_cmos_sensor(0x33b3, 0xff);
	write_cmos_sensor(0x33b4, 0xe0);
	write_cmos_sensor(0x33b5, 0x6b);
	write_cmos_sensor(0x33b6, 0x00);
	write_cmos_sensor(0x33b7, 0x00);
	write_cmos_sensor(0x33b8, 0x00);
	write_cmos_sensor(0x33b9, 0x00);
	write_cmos_sensor(0x33ba, 0x00);
	write_cmos_sensor(0x33bb, 0x1f);
	write_cmos_sensor(0x33bc, 0x01);
	write_cmos_sensor(0x33bd, 0x01);
	write_cmos_sensor(0x33be, 0x01);
	write_cmos_sensor(0x33bf, 0x01);
	write_cmos_sensor(0x33c0, 0x00);
	write_cmos_sensor(0x33c1, 0x00);
	write_cmos_sensor(0x33c2, 0x00);
	write_cmos_sensor(0x33c3, 0x00);
	write_cmos_sensor(0x33e0, 0x14);
	write_cmos_sensor(0x33e1, 0x0f);
	write_cmos_sensor(0x33e2, 0x02);
	write_cmos_sensor(0x33e3, 0x01);
	write_cmos_sensor(0x33e4, 0x01);
	write_cmos_sensor(0x33e5, 0x01);
	write_cmos_sensor(0x33e6, 0x00);
	write_cmos_sensor(0x33e7, 0x04);
	write_cmos_sensor(0x33e8, 0x0c);
	write_cmos_sensor(0x33e9, 0x02);
	write_cmos_sensor(0x33ea, 0x02);
	write_cmos_sensor(0x33eb, 0x02);
	write_cmos_sensor(0x33ec, 0x03);
	write_cmos_sensor(0x33ed, 0x01);
	write_cmos_sensor(0x33ee, 0x02);
	write_cmos_sensor(0x33ef, 0x08);
	write_cmos_sensor(0x33f0, 0x08);
	write_cmos_sensor(0x33f1, 0x04);
	write_cmos_sensor(0x33f2, 0x04);
	write_cmos_sensor(0x33f3, 0x00);
	write_cmos_sensor(0x33f4, 0x03);
	write_cmos_sensor(0x33f5, 0x14);
	write_cmos_sensor(0x33f6, 0x0f);
	write_cmos_sensor(0x33f7, 0x02);
	write_cmos_sensor(0x33f8, 0x01);
	write_cmos_sensor(0x33f9, 0x01);
	write_cmos_sensor(0x33fa, 0x01);
	write_cmos_sensor(0x33fb, 0x00);
	write_cmos_sensor(0x33fc, 0x04);
	write_cmos_sensor(0x33fd, 0x0c);
	write_cmos_sensor(0x33fe, 0x02);
	write_cmos_sensor(0x33ff, 0x02);
	write_cmos_sensor(0x3400, 0x02);
	write_cmos_sensor(0x3401, 0x03);
	write_cmos_sensor(0x3402, 0x01);
	write_cmos_sensor(0x3403, 0x02);
	write_cmos_sensor(0x3404, 0x08);
	write_cmos_sensor(0x3405, 0x08);
	write_cmos_sensor(0x3406, 0x04);
	write_cmos_sensor(0x3407, 0x04);
	write_cmos_sensor(0x3408, 0x00);
	write_cmos_sensor(0x3409, 0x03);
	write_cmos_sensor(0x340a, 0x14);
	write_cmos_sensor(0x340b, 0x0f);
	write_cmos_sensor(0x340c, 0x04);
	write_cmos_sensor(0x340d, 0x02);
	write_cmos_sensor(0x340e, 0x01);
	write_cmos_sensor(0x340f, 0x01);
	write_cmos_sensor(0x3410, 0x00);
	write_cmos_sensor(0x3411, 0x04);
	write_cmos_sensor(0x3412, 0x0c);
	write_cmos_sensor(0x3413, 0x02);
	write_cmos_sensor(0x3414, 0x02);
	write_cmos_sensor(0x3415, 0x02);
	write_cmos_sensor(0x3416, 0x03);
	write_cmos_sensor(0x3417, 0x02);
	write_cmos_sensor(0x3418, 0x05);
	write_cmos_sensor(0x3419, 0x0a);
	write_cmos_sensor(0x341a, 0x08);
	write_cmos_sensor(0x341b, 0x04);
	write_cmos_sensor(0x341c, 0x04);
	write_cmos_sensor(0x341d, 0x00);
	write_cmos_sensor(0x341e, 0x03);
	write_cmos_sensor(0x3440, 0x00);
	write_cmos_sensor(0x3441, 0x00);
	write_cmos_sensor(0x3442, 0x00);
	write_cmos_sensor(0x3443, 0x00);
	write_cmos_sensor(0x3444, 0x02);
	write_cmos_sensor(0x3445, 0xf0);
	write_cmos_sensor(0x3446, 0x02);
	write_cmos_sensor(0x3447, 0x08);
	write_cmos_sensor(0x3448, 0x00);
	write_cmos_sensor(0x3460, 0x40);
	write_cmos_sensor(0x3461, 0x40);
	write_cmos_sensor(0x3462, 0x40);
	write_cmos_sensor(0x3463, 0x40);
	write_cmos_sensor(0x3464, 0x03);
	write_cmos_sensor(0x3465, 0x01);
	write_cmos_sensor(0x3466, 0x01);
	write_cmos_sensor(0x3467, 0x00);
	write_cmos_sensor(0x3468, 0x30);
	write_cmos_sensor(0x3469, 0x00);
	write_cmos_sensor(0x346a, 0x33);
	write_cmos_sensor(0x346b, 0xbf);
	write_cmos_sensor(0x3480, 0x40);
	write_cmos_sensor(0x3481, 0x00);
	write_cmos_sensor(0x3482, 0x00);
	write_cmos_sensor(0x3483, 0x00);
	write_cmos_sensor(0x3484, 0x0d);
	write_cmos_sensor(0x3485, 0x00);
	write_cmos_sensor(0x3486, 0x00);
	write_cmos_sensor(0x3487, 0x00);
	write_cmos_sensor(0x3488, 0x00);
	write_cmos_sensor(0x3489, 0x00);
	write_cmos_sensor(0x348a, 0x00);
	write_cmos_sensor(0x348b, 0x04);
	write_cmos_sensor(0x348c, 0x00);
	write_cmos_sensor(0x348d, 0x01);
	write_cmos_sensor(0x348f, 0x01);
	write_cmos_sensor(0x3030, 0x0a);
	write_cmos_sensor(0x3030, 0x02);
	write_cmos_sensor(0x7000, 0x58);
	write_cmos_sensor(0x7001, 0x7a);
	write_cmos_sensor(0x7002, 0x1a);
	write_cmos_sensor(0x7003, 0xc1);
	write_cmos_sensor(0x7004, 0x03);
	write_cmos_sensor(0x7005, 0xda);
	write_cmos_sensor(0x7006, 0x18);
	write_cmos_sensor(0x7007, 0xc1);
	write_cmos_sensor(0x7008, 0x08);
	write_cmos_sensor(0x7009, 0xbd);
	write_cmos_sensor(0x700a, 0x03);
	write_cmos_sensor(0x700b, 0xbd);
	write_cmos_sensor(0x700c, 0x06);
	write_cmos_sensor(0x700d, 0xe6);
	write_cmos_sensor(0x700e, 0xec);
	write_cmos_sensor(0x700f, 0x00);
	write_cmos_sensor(0x7010, 0x19);
	write_cmos_sensor(0x7011, 0xc2);
	write_cmos_sensor(0x7012, 0x0c);
	write_cmos_sensor(0x7013, 0xbc);
	write_cmos_sensor(0x7014, 0xf0);
	write_cmos_sensor(0x7015, 0xbc);
	write_cmos_sensor(0x7016, 0xe6);
	write_cmos_sensor(0x7017, 0x00);
	write_cmos_sensor(0x7018, 0x1a);
	write_cmos_sensor(0x7019, 0xc2);
	write_cmos_sensor(0x701a, 0x10);
	write_cmos_sensor(0x701b, 0xbc);
	write_cmos_sensor(0x701c, 0x8c);
	write_cmos_sensor(0x701d, 0xbc);
	write_cmos_sensor(0x701e, 0x34);
	write_cmos_sensor(0x701f, 0x00);
	write_cmos_sensor(0x7020, 0xda);
	write_cmos_sensor(0x7021, 0x72);
	write_cmos_sensor(0x7022, 0x76);
	write_cmos_sensor(0x7023, 0xb6);
	write_cmos_sensor(0x7024, 0xee);
	write_cmos_sensor(0x7025, 0xcf);
	write_cmos_sensor(0x7026, 0xac);
	write_cmos_sensor(0x7027, 0xd0);
	write_cmos_sensor(0x7028, 0xac);
	write_cmos_sensor(0x7029, 0xd1);
	write_cmos_sensor(0x702a, 0x50);
	write_cmos_sensor(0x702b, 0xac);
	write_cmos_sensor(0x702c, 0xd2);
	write_cmos_sensor(0x702d, 0xbc);
	write_cmos_sensor(0x702e, 0x2e);
	write_cmos_sensor(0x702f, 0x18);
	write_cmos_sensor(0x7030, 0xc2);
	write_cmos_sensor(0x7031, 0x1c);
	write_cmos_sensor(0x7032, 0xbd);
	write_cmos_sensor(0x7033, 0x03);
	write_cmos_sensor(0x7034, 0xbd);
	write_cmos_sensor(0x7035, 0x06);
	write_cmos_sensor(0x7036, 0xe8);
	write_cmos_sensor(0x7037, 0x00);
	write_cmos_sensor(0x7038, 0xb4);
	write_cmos_sensor(0x7039, 0x00);
	write_cmos_sensor(0x703a, 0xdc);
	write_cmos_sensor(0x703b, 0xdf);
	write_cmos_sensor(0x703c, 0xb0);
	write_cmos_sensor(0x703d, 0x6e);
	write_cmos_sensor(0x703e, 0xbd);
	write_cmos_sensor(0x703f, 0x01);
	write_cmos_sensor(0x7040, 0xd7);
	write_cmos_sensor(0x7041, 0xed);
	write_cmos_sensor(0x7042, 0xe1);
	write_cmos_sensor(0x7043, 0x36);
	write_cmos_sensor(0x7044, 0x30);
	write_cmos_sensor(0x7045, 0xd3);
	write_cmos_sensor(0x7046, 0x2e);
	write_cmos_sensor(0x7047, 0x54);
	write_cmos_sensor(0x7048, 0x46);
	write_cmos_sensor(0x7049, 0xbc);
	write_cmos_sensor(0x704a, 0x22);
	write_cmos_sensor(0x704b, 0x66);
	write_cmos_sensor(0x704c, 0xbc);
	write_cmos_sensor(0x704d, 0x24);
	write_cmos_sensor(0x704e, 0x2c);
	write_cmos_sensor(0x704f, 0x28);
	write_cmos_sensor(0x7050, 0xbc);
	write_cmos_sensor(0x7051, 0x3c);
	write_cmos_sensor(0x7052, 0xa1);
	write_cmos_sensor(0x7053, 0xac);
	write_cmos_sensor(0x7054, 0xd8);
	write_cmos_sensor(0x7055, 0xd6);
	write_cmos_sensor(0x7056, 0xb4);
	write_cmos_sensor(0x7057, 0x04);
	write_cmos_sensor(0x7058, 0x46);
	write_cmos_sensor(0x7059, 0xb7);
	write_cmos_sensor(0x705a, 0x04);
	write_cmos_sensor(0x705b, 0xbe);
	write_cmos_sensor(0x705c, 0x08);
	write_cmos_sensor(0x705d, 0xc3);
	write_cmos_sensor(0x705e, 0xe5);
	write_cmos_sensor(0x705f, 0xad);
	write_cmos_sensor(0x7060, 0xc3);
	write_cmos_sensor(0x7061, 0xc8);
	write_cmos_sensor(0x7062, 0x19);
	write_cmos_sensor(0x7063, 0xc1);
	write_cmos_sensor(0x7064, 0x33);
	write_cmos_sensor(0x7065, 0xe7);
	write_cmos_sensor(0x7066, 0x50);
	write_cmos_sensor(0x7067, 0x20);
	write_cmos_sensor(0x7068, 0xb8);
	write_cmos_sensor(0x7069, 0x02);
	write_cmos_sensor(0x706a, 0xbc);
	write_cmos_sensor(0x706b, 0x17);
	write_cmos_sensor(0x706c, 0xdb);
	write_cmos_sensor(0x706d, 0xc7);
	write_cmos_sensor(0x706e, 0xb8);
	write_cmos_sensor(0x706f, 0x00);
	write_cmos_sensor(0x7070, 0x28);
	write_cmos_sensor(0x7071, 0x54);
	write_cmos_sensor(0x7072, 0xb4);
	write_cmos_sensor(0x7073, 0x14);
	write_cmos_sensor(0x7074, 0xab);
	write_cmos_sensor(0x7075, 0xbe);
	write_cmos_sensor(0x7076, 0x06);
	write_cmos_sensor(0x7077, 0xd8);
	write_cmos_sensor(0x7078, 0xd6);
	write_cmos_sensor(0x7079, 0x00);
	write_cmos_sensor(0x707a, 0xee);
	write_cmos_sensor(0x707b, 0xe6);
	write_cmos_sensor(0x707c, 0x18);
	write_cmos_sensor(0x707d, 0xc2);
	write_cmos_sensor(0x707e, 0x40);
	write_cmos_sensor(0x707f, 0xec);
	write_cmos_sensor(0x7080, 0xb4);
	write_cmos_sensor(0x7081, 0xc7);
	write_cmos_sensor(0x7082, 0x62);
	write_cmos_sensor(0x7083, 0x07);
	write_cmos_sensor(0x7084, 0xb9);
	write_cmos_sensor(0x7085, 0x05);
	write_cmos_sensor(0x7086, 0xad);
	write_cmos_sensor(0x7087, 0xb4);
	write_cmos_sensor(0x7088, 0x26);
	write_cmos_sensor(0x7089, 0x19);
	write_cmos_sensor(0x708a, 0xc1);
	write_cmos_sensor(0x708b, 0x48);
	write_cmos_sensor(0x708c, 0xc3);
	write_cmos_sensor(0x708d, 0xbc);
	write_cmos_sensor(0x708e, 0xc0);
	write_cmos_sensor(0x708f, 0x4a);
	write_cmos_sensor(0x7090, 0xc3);
	write_cmos_sensor(0x7091, 0xca);
	write_cmos_sensor(0x7092, 0xe7);
	write_cmos_sensor(0x7093, 0x00);
	write_cmos_sensor(0x7094, 0x15);
	write_cmos_sensor(0x7095, 0xc2);
	write_cmos_sensor(0x7096, 0x4e);
	write_cmos_sensor(0x7097, 0xc3);
	write_cmos_sensor(0x7098, 0xb1);
	write_cmos_sensor(0x7099, 0xc0);
	write_cmos_sensor(0x709a, 0x4a);
	write_cmos_sensor(0x709b, 0x00);
	write_cmos_sensor(0x709c, 0xb9);
	write_cmos_sensor(0x709d, 0x64);
	write_cmos_sensor(0x709e, 0x29);
	write_cmos_sensor(0x709f, 0x00);
	write_cmos_sensor(0x70a0, 0xb8);
	write_cmos_sensor(0x70a1, 0x12);
	write_cmos_sensor(0x70a2, 0xbe);
	write_cmos_sensor(0x70a3, 0x01);
	write_cmos_sensor(0x70a4, 0xd0);
	write_cmos_sensor(0x70a5, 0xbc);
	write_cmos_sensor(0x70a6, 0x01);
	write_cmos_sensor(0x70a7, 0xac);
	write_cmos_sensor(0x70a8, 0x37);
	write_cmos_sensor(0x70a9, 0xd2);
	write_cmos_sensor(0x70aa, 0xac);
	write_cmos_sensor(0x70ab, 0x45);
	write_cmos_sensor(0x70ac, 0xad);
	write_cmos_sensor(0x70ad, 0x28);
	write_cmos_sensor(0x70ae, 0x00);
	write_cmos_sensor(0x70af, 0xb8);
	write_cmos_sensor(0x70b0, 0x00);
	write_cmos_sensor(0x70b1, 0xbc);
	write_cmos_sensor(0x70b2, 0x01);
	write_cmos_sensor(0x70b3, 0x36);
	write_cmos_sensor(0x70b4, 0xd3);
	write_cmos_sensor(0x70b5, 0x30);
	write_cmos_sensor(0x70b6, 0x04);
	write_cmos_sensor(0x70b7, 0xe0);
	write_cmos_sensor(0x70b8, 0xd8);
	write_cmos_sensor(0x70b9, 0xb4);
	write_cmos_sensor(0x70ba, 0xe9);
	write_cmos_sensor(0x70bb, 0x00);
	write_cmos_sensor(0x70bc, 0xbe);
	write_cmos_sensor(0x70bd, 0x05);
	write_cmos_sensor(0x70be, 0x62);
	write_cmos_sensor(0x70bf, 0x07);
	write_cmos_sensor(0x70c0, 0xb9);
	write_cmos_sensor(0x70c1, 0x05);
	write_cmos_sensor(0x70c2, 0xad);
	write_cmos_sensor(0x70c3, 0xc3);
	write_cmos_sensor(0x70c4, 0xdb);
	write_cmos_sensor(0x70c5, 0x00);
	write_cmos_sensor(0x70c6, 0x15);
	write_cmos_sensor(0x70c7, 0xc2);
	write_cmos_sensor(0x70c8, 0x67);
	write_cmos_sensor(0x70c9, 0xc3);
	write_cmos_sensor(0x70ca, 0xd5);
	write_cmos_sensor(0x70cb, 0xc0);
	write_cmos_sensor(0x70cc, 0x63);
	write_cmos_sensor(0x70cd, 0x00);
	write_cmos_sensor(0x70ce, 0x46);
	write_cmos_sensor(0x70cf, 0xa1);
	write_cmos_sensor(0x70d0, 0xb9);
	write_cmos_sensor(0x70d1, 0x64);
	write_cmos_sensor(0x70d2, 0x29);
	write_cmos_sensor(0x70d3, 0x00);
	write_cmos_sensor(0x70d4, 0xb8);
	write_cmos_sensor(0x70d5, 0x02);
	write_cmos_sensor(0x70d6, 0xbe);
	write_cmos_sensor(0x70d7, 0x02);
	write_cmos_sensor(0x70d8, 0xd0);
	write_cmos_sensor(0x70d9, 0xdc);
	write_cmos_sensor(0x70da, 0xac);
	write_cmos_sensor(0x70db, 0xbc);
	write_cmos_sensor(0x70dc, 0x01);
	write_cmos_sensor(0x70dd, 0x37);
	write_cmos_sensor(0x70de, 0xac);
	write_cmos_sensor(0x70df, 0xd2);
	write_cmos_sensor(0x70e0, 0x45);
	write_cmos_sensor(0x70e1, 0xad);
	write_cmos_sensor(0x70e2, 0x28);
	write_cmos_sensor(0x70e3, 0x00);
	write_cmos_sensor(0x70e4, 0xb8);
	write_cmos_sensor(0x70e5, 0x00);
	write_cmos_sensor(0x70e6, 0xbc);
	write_cmos_sensor(0x70e7, 0x01);
	write_cmos_sensor(0x70e8, 0x36);
	write_cmos_sensor(0x70e9, 0x30);
	write_cmos_sensor(0x70ea, 0xe0);
	write_cmos_sensor(0x70eb, 0xd8);
	write_cmos_sensor(0x70ec, 0xb5);
	write_cmos_sensor(0x70ed, 0x0b);
	write_cmos_sensor(0x70ee, 0xd6);
	write_cmos_sensor(0x70ef, 0xbe);
	write_cmos_sensor(0x70f0, 0x07);
	write_cmos_sensor(0x70f1, 0x00);
	write_cmos_sensor(0x70f2, 0x62);
	write_cmos_sensor(0x70f3, 0x07);
	write_cmos_sensor(0x70f4, 0xb9);
	write_cmos_sensor(0x70f5, 0x05);
	write_cmos_sensor(0x70f6, 0xad);
	write_cmos_sensor(0x70f7, 0xc3);
	write_cmos_sensor(0x70f8, 0xdb);
	write_cmos_sensor(0x70f9, 0x46);
	write_cmos_sensor(0x70fa, 0xcd);
	write_cmos_sensor(0x70fb, 0x07);
	write_cmos_sensor(0x70fc, 0xcd);
	write_cmos_sensor(0x70fd, 0x00);
	write_cmos_sensor(0x70fe, 0xe3);
	write_cmos_sensor(0x70ff, 0x18);
	write_cmos_sensor(0x7100, 0xc2);
	write_cmos_sensor(0x7101, 0xae);
	write_cmos_sensor(0x7102, 0xb9);
	write_cmos_sensor(0x7103, 0x64);
	write_cmos_sensor(0x7104, 0xd1);
	write_cmos_sensor(0x7105, 0x50);
	write_cmos_sensor(0x7106, 0xdd);
	write_cmos_sensor(0x7107, 0xac);
	write_cmos_sensor(0x7108, 0xcf);
	write_cmos_sensor(0x7109, 0xdf);
	write_cmos_sensor(0x710a, 0xb6);
	write_cmos_sensor(0x710b, 0xee);
	write_cmos_sensor(0x710c, 0xbc);
	write_cmos_sensor(0x710d, 0x13);
	write_cmos_sensor(0x710e, 0xe1);
	write_cmos_sensor(0x710f, 0x36);
	write_cmos_sensor(0x7110, 0x30);
	write_cmos_sensor(0x7111, 0xd3);
	write_cmos_sensor(0x7112, 0x2e);
	write_cmos_sensor(0x7113, 0x54);
	write_cmos_sensor(0x7114, 0xbc);
	write_cmos_sensor(0x7115, 0x1e);
	write_cmos_sensor(0x7116, 0x2c);
	write_cmos_sensor(0x7117, 0x50);
	write_cmos_sensor(0x7118, 0x20);
	write_cmos_sensor(0x7119, 0x04);
	write_cmos_sensor(0x711a, 0xb8);
	write_cmos_sensor(0x711b, 0x02);
	write_cmos_sensor(0x711c, 0xbc);
	write_cmos_sensor(0x711d, 0x18);
	write_cmos_sensor(0x711e, 0xc7);
	write_cmos_sensor(0x711f, 0xb8);
	write_cmos_sensor(0x7120, 0x00);
	write_cmos_sensor(0x7121, 0x28);
	write_cmos_sensor(0x7122, 0x54);
	write_cmos_sensor(0x7123, 0xb4);
	write_cmos_sensor(0x7124, 0xca);
	write_cmos_sensor(0x7125, 0x46);
	write_cmos_sensor(0x7126, 0xbe);
	write_cmos_sensor(0x7127, 0x04);
	write_cmos_sensor(0x7128, 0xd6);
	write_cmos_sensor(0x7129, 0xd8);
	write_cmos_sensor(0x712a, 0xab);
	write_cmos_sensor(0x712b, 0x00);
	write_cmos_sensor(0x712c, 0x62);
	write_cmos_sensor(0x712d, 0x07);
	write_cmos_sensor(0x712e, 0xb9);
	write_cmos_sensor(0x712f, 0x05);
	write_cmos_sensor(0x7130, 0xad);
	write_cmos_sensor(0x7131, 0xc3);
	write_cmos_sensor(0x7132, 0xba);
	write_cmos_sensor(0x7133, 0xb9);
	write_cmos_sensor(0x7134, 0x64);
	write_cmos_sensor(0x7135, 0x29);
	write_cmos_sensor(0x7136, 0x00);
	write_cmos_sensor(0x7137, 0xb8);
	write_cmos_sensor(0x7138, 0x02);
	write_cmos_sensor(0x7139, 0xbe);
	write_cmos_sensor(0x713a, 0x00);
	write_cmos_sensor(0x713b, 0x45);
	write_cmos_sensor(0x713c, 0xad);
	write_cmos_sensor(0x713d, 0xe2);
	write_cmos_sensor(0x713e, 0x28);
	write_cmos_sensor(0x713f, 0x00);
	write_cmos_sensor(0x7140, 0xb8);
	write_cmos_sensor(0x7141, 0x00);
	write_cmos_sensor(0x7142, 0xe6);
	write_cmos_sensor(0x7143, 0xbd);
	write_cmos_sensor(0x7144, 0x03);
	write_cmos_sensor(0x7145, 0xec);
	write_cmos_sensor(0x7146, 0xe0);
	write_cmos_sensor(0x7147, 0xd8);
	write_cmos_sensor(0x7148, 0xb4);
	write_cmos_sensor(0x7149, 0xe9);
	write_cmos_sensor(0x714a, 0xbe);
	write_cmos_sensor(0x714b, 0x03);
	write_cmos_sensor(0x714c, 0x00);
	write_cmos_sensor(0x714d, 0x30);
	write_cmos_sensor(0x714e, 0x62);
	write_cmos_sensor(0x714f, 0x07);
	write_cmos_sensor(0x7150, 0xb9);
	write_cmos_sensor(0x7151, 0x05);
	write_cmos_sensor(0x7152, 0xad);
	write_cmos_sensor(0x7153, 0xc3);
	write_cmos_sensor(0x7154, 0xdb);
	write_cmos_sensor(0x7155, 0x42);
	write_cmos_sensor(0x7156, 0xe4);
	write_cmos_sensor(0x7157, 0xcd);
	write_cmos_sensor(0x7158, 0x07);
	write_cmos_sensor(0x7159, 0xcd);
	write_cmos_sensor(0x715a, 0x00);
	write_cmos_sensor(0x715b, 0x00);
	write_cmos_sensor(0x715c, 0x17);
	write_cmos_sensor(0x715d, 0xc2);
	write_cmos_sensor(0x715e, 0xc7);
	write_cmos_sensor(0x715f, 0xde);
	write_cmos_sensor(0x7160, 0xcf);
	write_cmos_sensor(0x7161, 0xdf);
	write_cmos_sensor(0x7162, 0xac);
	write_cmos_sensor(0x7163, 0xd1);
	write_cmos_sensor(0x7164, 0x44);
	write_cmos_sensor(0x7165, 0xac);
	write_cmos_sensor(0x7166, 0xb9);
	write_cmos_sensor(0x7167, 0x76);
	write_cmos_sensor(0x7168, 0xb8);
	write_cmos_sensor(0x7169, 0x08);
	write_cmos_sensor(0x716a, 0xb6);
	write_cmos_sensor(0x716b, 0xfe);
	write_cmos_sensor(0x716c, 0xb4);
	write_cmos_sensor(0x716d, 0xca);
	write_cmos_sensor(0x716e, 0xd6);
	write_cmos_sensor(0x716f, 0xd8);
	write_cmos_sensor(0x7170, 0xab);
	write_cmos_sensor(0x7171, 0x00);
	write_cmos_sensor(0x7172, 0xe1);
	write_cmos_sensor(0x7173, 0x36);
	write_cmos_sensor(0x7174, 0x30);
	write_cmos_sensor(0x7175, 0xd3);
	write_cmos_sensor(0x7176, 0xbc);
	write_cmos_sensor(0x7177, 0x29);
	write_cmos_sensor(0x7178, 0xb4);
	write_cmos_sensor(0x7179, 0x1f);
	write_cmos_sensor(0x717a, 0xaa);
	write_cmos_sensor(0x717b, 0xbd);
	write_cmos_sensor(0x717c, 0x01);
	write_cmos_sensor(0x717d, 0xb8);
	write_cmos_sensor(0x717e, 0x0c);
	write_cmos_sensor(0x717f, 0x45);
	write_cmos_sensor(0x7180, 0xe6);
	write_cmos_sensor(0x7181, 0xbd);
	write_cmos_sensor(0x7182, 0x03);
	write_cmos_sensor(0x7183, 0xec);
	write_cmos_sensor(0x7184, 0xbc);
	write_cmos_sensor(0x7185, 0x3d);
	write_cmos_sensor(0x7186, 0xc3);
	write_cmos_sensor(0x7187, 0xdb);
	write_cmos_sensor(0x7188, 0x42);
	write_cmos_sensor(0x7189, 0xb8);
	write_cmos_sensor(0x718a, 0x00);
	write_cmos_sensor(0x718b, 0xe4);
	write_cmos_sensor(0x718c, 0xd5);
	write_cmos_sensor(0x718d, 0x00);
	write_cmos_sensor(0x718e, 0xb6);
	write_cmos_sensor(0x718f, 0x00);
	write_cmos_sensor(0x7190, 0x7a);
	write_cmos_sensor(0x7191, 0xbd);
	write_cmos_sensor(0x7192, 0x03);
	write_cmos_sensor(0x7193, 0xb5);
	write_cmos_sensor(0x7194, 0x39);
	write_cmos_sensor(0x7195, 0x40);
	write_cmos_sensor(0x7196, 0x58);
	write_cmos_sensor(0x7197, 0xdd);
	write_cmos_sensor(0x7198, 0x1a);
	write_cmos_sensor(0x7199, 0xc2);
	write_cmos_sensor(0x719a, 0xce);
	write_cmos_sensor(0x719b, 0xe8);
	write_cmos_sensor(0x719c, 0xbc);
	write_cmos_sensor(0x719d, 0x19);
	write_cmos_sensor(0x719e, 0xb9);
	write_cmos_sensor(0x719f, 0xfa);
	write_cmos_sensor(0x71a0, 0x14);
	write_cmos_sensor(0x71a1, 0xc1);
	write_cmos_sensor(0x71a2, 0xd7);
	write_cmos_sensor(0x71a3, 0x76);
	write_cmos_sensor(0x71a4, 0xd1);
	write_cmos_sensor(0x71a5, 0xac);
	write_cmos_sensor(0x71a6, 0x37);
	write_cmos_sensor(0x71a7, 0xbc);
	write_cmos_sensor(0x71a8, 0x35);
	write_cmos_sensor(0x71a9, 0x36);
	write_cmos_sensor(0x71aa, 0x30);
	write_cmos_sensor(0x71ab, 0xe1);
	write_cmos_sensor(0x71ac, 0xd3);
	write_cmos_sensor(0x71ad, 0x7a);
	write_cmos_sensor(0x71ae, 0xb6);
	write_cmos_sensor(0x71af, 0x0c);
	write_cmos_sensor(0x71b0, 0xff);
	write_cmos_sensor(0x71b1, 0xb4);
	write_cmos_sensor(0x71b2, 0xc7);
	write_cmos_sensor(0x71b3, 0xd9);
	write_cmos_sensor(0x71b4, 0x00);
	write_cmos_sensor(0x71b5, 0xbd);
	write_cmos_sensor(0x71b6, 0x01);
	write_cmos_sensor(0x71b7, 0x56);
	write_cmos_sensor(0x71b8, 0xc0);
	write_cmos_sensor(0x71b9, 0xe0);
	write_cmos_sensor(0x71ba, 0xb4);
	write_cmos_sensor(0x71bb, 0x1f);
	write_cmos_sensor(0x71bc, 0x56);
	write_cmos_sensor(0x71bd, 0xaa);
	write_cmos_sensor(0x71be, 0xbc);
	write_cmos_sensor(0x71bf, 0x08);
	write_cmos_sensor(0x71c0, 0x57);
	write_cmos_sensor(0x71c1, 0xe8);
	write_cmos_sensor(0x71c2, 0xb5);
	write_cmos_sensor(0x71c3, 0x36);
	write_cmos_sensor(0x71c4, 0x00);
	write_cmos_sensor(0x71c5, 0x54);
	write_cmos_sensor(0x71c6, 0xe7);
	write_cmos_sensor(0x71c7, 0xc8);
	write_cmos_sensor(0x71c8, 0xb4);
	write_cmos_sensor(0x71c9, 0x1f);
	write_cmos_sensor(0x71ca, 0x56);
	write_cmos_sensor(0x71cb, 0xaa);
	write_cmos_sensor(0x71cc, 0xbc);
	write_cmos_sensor(0x71cd, 0x08);
	write_cmos_sensor(0x71ce, 0x57);
	write_cmos_sensor(0x71cf, 0x00);
	write_cmos_sensor(0x71d0, 0xb5);
	write_cmos_sensor(0x71d1, 0x36);
	write_cmos_sensor(0x71d2, 0x00);
	write_cmos_sensor(0x71d3, 0x54);
	write_cmos_sensor(0x71d4, 0xc8);
	write_cmos_sensor(0x71d5, 0xb5);
	write_cmos_sensor(0x71d6, 0x18);
	write_cmos_sensor(0x71d7, 0xd9);
	write_cmos_sensor(0x71d8, 0x00);
	write_cmos_sensor(0x71d9, 0xbd);
	write_cmos_sensor(0x71da, 0x01);
	write_cmos_sensor(0x71db, 0x56);
	write_cmos_sensor(0x71dc, 0x08);
	write_cmos_sensor(0x71dd, 0x57);
	write_cmos_sensor(0x71de, 0xe8);
	write_cmos_sensor(0x71df, 0xb4);
	write_cmos_sensor(0x71e0, 0x42);
	write_cmos_sensor(0x71e1, 0x00);
	write_cmos_sensor(0x71e2, 0x54);
	write_cmos_sensor(0x71e3, 0xe7);
	write_cmos_sensor(0x71e4, 0xc8);
	write_cmos_sensor(0x71e5, 0xab);
	write_cmos_sensor(0x71e6, 0x00);
	write_cmos_sensor(0x71e7, 0x66);
	write_cmos_sensor(0x71e8, 0x62);
	write_cmos_sensor(0x71e9, 0x06);
	write_cmos_sensor(0x71ea, 0x74);
	write_cmos_sensor(0x71eb, 0xb9);
	write_cmos_sensor(0x71ec, 0x05);
	write_cmos_sensor(0x71ed, 0xb7);
	write_cmos_sensor(0x71ee, 0x14);
	write_cmos_sensor(0x71ef, 0x0e);
	write_cmos_sensor(0x71f0, 0xb7);
	write_cmos_sensor(0x71f1, 0x04);
	write_cmos_sensor(0x71f2, 0xc8);
	write_cmos_sensor(0x7600, 0x04);
	write_cmos_sensor(0x7601, 0x80);
	write_cmos_sensor(0x7602, 0x07);
	write_cmos_sensor(0x7603, 0x44);
	write_cmos_sensor(0x7604, 0x05);
	write_cmos_sensor(0x7605, 0x33);
	write_cmos_sensor(0x7606, 0x0f);
	write_cmos_sensor(0x7607, 0x00);
	write_cmos_sensor(0x7608, 0x07);
	write_cmos_sensor(0x7609, 0x40);
	write_cmos_sensor(0x760a, 0x04);
	write_cmos_sensor(0x760b, 0xe5);
	write_cmos_sensor(0x760c, 0x06);
	write_cmos_sensor(0x760d, 0x50);
	write_cmos_sensor(0x760e, 0x04);
	write_cmos_sensor(0x760f, 0xe4);
	write_cmos_sensor(0x7610, 0x00);
	write_cmos_sensor(0x7611, 0x00);
	write_cmos_sensor(0x7612, 0x06);
	write_cmos_sensor(0x7613, 0x5c);
	write_cmos_sensor(0x7614, 0x00);
	write_cmos_sensor(0x7615, 0x0f);
	write_cmos_sensor(0x7616, 0x06);
	write_cmos_sensor(0x7617, 0x1c);
	write_cmos_sensor(0x7618, 0x00);
	write_cmos_sensor(0x7619, 0x02);
	write_cmos_sensor(0x761a, 0x06);
	write_cmos_sensor(0x761b, 0xa2);
	write_cmos_sensor(0x761c, 0x00);
	write_cmos_sensor(0x761d, 0x01);
	write_cmos_sensor(0x761e, 0x06);
	write_cmos_sensor(0x761f, 0xae);
	write_cmos_sensor(0x7620, 0x00);
	write_cmos_sensor(0x7621, 0x0e);
	write_cmos_sensor(0x7622, 0x05);
	write_cmos_sensor(0x7623, 0x30);
	write_cmos_sensor(0x7624, 0x07);
	write_cmos_sensor(0x7625, 0x00);
	write_cmos_sensor(0x7626, 0x0f);
	write_cmos_sensor(0x7627, 0x00);
	write_cmos_sensor(0x7628, 0x04);
	write_cmos_sensor(0x7629, 0xe5);
	write_cmos_sensor(0x762a, 0x05);
	write_cmos_sensor(0x762b, 0x33);
	write_cmos_sensor(0x762c, 0x06);
	write_cmos_sensor(0x762d, 0x12);
	write_cmos_sensor(0x762e, 0x00);
	write_cmos_sensor(0x762f, 0x01);
	write_cmos_sensor(0x7630, 0x06);
	write_cmos_sensor(0x7631, 0x52);
	write_cmos_sensor(0x7632, 0x00);
	write_cmos_sensor(0x7633, 0x01);
	write_cmos_sensor(0x7634, 0x06);
	write_cmos_sensor(0x7635, 0x5e);
	write_cmos_sensor(0x7636, 0x04);
	write_cmos_sensor(0x7637, 0xe4);
	write_cmos_sensor(0x7638, 0x00);
	write_cmos_sensor(0x7639, 0x01);
	write_cmos_sensor(0x763a, 0x05);
	write_cmos_sensor(0x763b, 0x30);
	write_cmos_sensor(0x763c, 0x0f);
	write_cmos_sensor(0x763d, 0x00);
	write_cmos_sensor(0x763e, 0x06);
	write_cmos_sensor(0x763f, 0xa6);
	write_cmos_sensor(0x7640, 0x00);
	write_cmos_sensor(0x7641, 0x02);
	write_cmos_sensor(0x7642, 0x06);
	write_cmos_sensor(0x7643, 0x26);
	write_cmos_sensor(0x7644, 0x00);
	write_cmos_sensor(0x7645, 0x02);
	write_cmos_sensor(0x7646, 0x05);
	write_cmos_sensor(0x7647, 0x33);
	write_cmos_sensor(0x7648, 0x06);
	write_cmos_sensor(0x7649, 0x20);
	write_cmos_sensor(0x764a, 0x0f);
	write_cmos_sensor(0x764b, 0x00);
	write_cmos_sensor(0x764c, 0x06);
	write_cmos_sensor(0x764d, 0x56);
	write_cmos_sensor(0x764e, 0x00);
	write_cmos_sensor(0x764f, 0x02);
	write_cmos_sensor(0x7650, 0x06);
	write_cmos_sensor(0x7651, 0x16);
	write_cmos_sensor(0x7652, 0x05);
	write_cmos_sensor(0x7653, 0x33);
	write_cmos_sensor(0x7654, 0x06);
	write_cmos_sensor(0x7655, 0x10);
	write_cmos_sensor(0x7656, 0x0f);
	write_cmos_sensor(0x7657, 0x00);
	write_cmos_sensor(0x7658, 0x06);
	write_cmos_sensor(0x7659, 0x10);
	write_cmos_sensor(0x765a, 0x0f);
	write_cmos_sensor(0x765b, 0x00);
	write_cmos_sensor(0x765c, 0x06);
	write_cmos_sensor(0x765d, 0x20);
	write_cmos_sensor(0x765e, 0x0f);
	write_cmos_sensor(0x765f, 0x00);
	write_cmos_sensor(0x7660, 0x00);
	write_cmos_sensor(0x7661, 0x00);
	write_cmos_sensor(0x7662, 0x00);
	write_cmos_sensor(0x7663, 0x02);
	write_cmos_sensor(0x7664, 0x04);
	write_cmos_sensor(0x7665, 0xe5);
	write_cmos_sensor(0x7666, 0x04);
	write_cmos_sensor(0x7667, 0xe4);
	write_cmos_sensor(0x7668, 0x0f);
	write_cmos_sensor(0x7669, 0x00);
	write_cmos_sensor(0x766a, 0x00);
	write_cmos_sensor(0x766b, 0x00);
	write_cmos_sensor(0x766c, 0x00);
	write_cmos_sensor(0x766d, 0x01);
	write_cmos_sensor(0x766e, 0x04);
	write_cmos_sensor(0x766f, 0xe5);
	write_cmos_sensor(0x7670, 0x04);
	write_cmos_sensor(0x7671, 0xe4);
	write_cmos_sensor(0x7672, 0x0f);
	write_cmos_sensor(0x7673, 0x00);
	write_cmos_sensor(0x7674, 0x00);
	write_cmos_sensor(0x7675, 0x02);
	write_cmos_sensor(0x7676, 0x04);
	write_cmos_sensor(0x7677, 0xe4);
	write_cmos_sensor(0x7678, 0x00);
	write_cmos_sensor(0x7679, 0x02);
	write_cmos_sensor(0x767a, 0x04);
	write_cmos_sensor(0x767b, 0xc4);
	write_cmos_sensor(0x767c, 0x00);
	write_cmos_sensor(0x767d, 0x02);
	write_cmos_sensor(0x767e, 0x04);
	write_cmos_sensor(0x767f, 0xc4);
	write_cmos_sensor(0x7680, 0x05);
	write_cmos_sensor(0x7681, 0x83);
	write_cmos_sensor(0x7682, 0x0f);
	write_cmos_sensor(0x7683, 0x00);
	write_cmos_sensor(0x7684, 0x00);
	write_cmos_sensor(0x7685, 0x02);
	write_cmos_sensor(0x7686, 0x04);
	write_cmos_sensor(0x7687, 0xe4);
	write_cmos_sensor(0x7688, 0x00);
	write_cmos_sensor(0x7689, 0x02);
	write_cmos_sensor(0x768a, 0x04);
	write_cmos_sensor(0x768b, 0xc4);
	write_cmos_sensor(0x768c, 0x00);
	write_cmos_sensor(0x768d, 0x02);
	write_cmos_sensor(0x768e, 0x04);
	write_cmos_sensor(0x768f, 0xc4);
	write_cmos_sensor(0x7690, 0x05);
	write_cmos_sensor(0x7691, 0x83);
	write_cmos_sensor(0x7692, 0x03);
	write_cmos_sensor(0x7693, 0x0b);
	write_cmos_sensor(0x7694, 0x05);
	write_cmos_sensor(0x7695, 0x83);
	write_cmos_sensor(0x7696, 0x00);
	write_cmos_sensor(0x7697, 0x07);
	write_cmos_sensor(0x7698, 0x05);
	write_cmos_sensor(0x7699, 0x03);
	write_cmos_sensor(0x769a, 0x00);
	write_cmos_sensor(0x769b, 0x05);
	write_cmos_sensor(0x769c, 0x05);
	write_cmos_sensor(0x769d, 0x32);
	write_cmos_sensor(0x769e, 0x05);
	write_cmos_sensor(0x769f, 0x30);
	write_cmos_sensor(0x76a0, 0x00);
	write_cmos_sensor(0x76a1, 0x02);
	write_cmos_sensor(0x76a2, 0x05);
	write_cmos_sensor(0x76a3, 0x78);
	write_cmos_sensor(0x76a4, 0x00);
	write_cmos_sensor(0x76a5, 0x01);
	write_cmos_sensor(0x76a6, 0x05);
	write_cmos_sensor(0x76a7, 0x7c);
	write_cmos_sensor(0x76a8, 0x03);
	write_cmos_sensor(0x76a9, 0x9a);
	write_cmos_sensor(0x76aa, 0x05);
	write_cmos_sensor(0x76ab, 0x83);
	write_cmos_sensor(0x76ac, 0x00);
	write_cmos_sensor(0x76ad, 0x04);
	write_cmos_sensor(0x76ae, 0x05);
	write_cmos_sensor(0x76af, 0x03);
	write_cmos_sensor(0x76b0, 0x00);
	write_cmos_sensor(0x76b1, 0x03);
	write_cmos_sensor(0x76b2, 0x05);
	write_cmos_sensor(0x76b3, 0x32);
	write_cmos_sensor(0x76b4, 0x05);
	write_cmos_sensor(0x76b5, 0x30);
	write_cmos_sensor(0x76b6, 0x00);
	write_cmos_sensor(0x76b7, 0x02);
	write_cmos_sensor(0x76b8, 0x05);
	write_cmos_sensor(0x76b9, 0x78);
	write_cmos_sensor(0x76ba, 0x00);
	write_cmos_sensor(0x76bb, 0x01);
	write_cmos_sensor(0x76bc, 0x05);
	write_cmos_sensor(0x76bd, 0x7c);
	write_cmos_sensor(0x76be, 0x03);
	write_cmos_sensor(0x76bf, 0x99);
	write_cmos_sensor(0x76c0, 0x05);
	write_cmos_sensor(0x76c1, 0x83);
	write_cmos_sensor(0x76c2, 0x00);
	write_cmos_sensor(0x76c3, 0x03);
	write_cmos_sensor(0x76c4, 0x05);
	write_cmos_sensor(0x76c5, 0x03);
	write_cmos_sensor(0x76c6, 0x00);
	write_cmos_sensor(0x76c7, 0x01);
	write_cmos_sensor(0x76c8, 0x05);
	write_cmos_sensor(0x76c9, 0x32);
	write_cmos_sensor(0x76ca, 0x05);
	write_cmos_sensor(0x76cb, 0x30);
	write_cmos_sensor(0x76cc, 0x00);
	write_cmos_sensor(0x76cd, 0x02);
	write_cmos_sensor(0x76ce, 0x05);
	write_cmos_sensor(0x76cf, 0x78);
	write_cmos_sensor(0x76d0, 0x00);
	write_cmos_sensor(0x76d1, 0x01);
	write_cmos_sensor(0x76d2, 0x05);
	write_cmos_sensor(0x76d3, 0x7c);
	write_cmos_sensor(0x76d4, 0x03);
	write_cmos_sensor(0x76d5, 0x98);
	write_cmos_sensor(0x76d6, 0x05);
	write_cmos_sensor(0x76d7, 0x83);
	write_cmos_sensor(0x76d8, 0x00);
	write_cmos_sensor(0x76d9, 0x00);
	write_cmos_sensor(0x76da, 0x05);
	write_cmos_sensor(0x76db, 0x03);
	write_cmos_sensor(0x76dc, 0x00);
	write_cmos_sensor(0x76dd, 0x01);
	write_cmos_sensor(0x76de, 0x05);
	write_cmos_sensor(0x76df, 0x32);
	write_cmos_sensor(0x76e0, 0x05);
	write_cmos_sensor(0x76e1, 0x30);
	write_cmos_sensor(0x76e2, 0x00);
	write_cmos_sensor(0x76e3, 0x02);
	write_cmos_sensor(0x76e4, 0x05);
	write_cmos_sensor(0x76e5, 0x78);
	write_cmos_sensor(0x76e6, 0x00);
	write_cmos_sensor(0x76e7, 0x01);
	write_cmos_sensor(0x76e8, 0x05);
	write_cmos_sensor(0x76e9, 0x7c);
	write_cmos_sensor(0x76ea, 0x03);
	write_cmos_sensor(0x76eb, 0x97);
	write_cmos_sensor(0x76ec, 0x05);
	write_cmos_sensor(0x76ed, 0x83);
	write_cmos_sensor(0x76ee, 0x00);
	write_cmos_sensor(0x76ef, 0x00);
	write_cmos_sensor(0x76f0, 0x05);
	write_cmos_sensor(0x76f1, 0x03);
	write_cmos_sensor(0x76f2, 0x05);
	write_cmos_sensor(0x76f3, 0x32);
	write_cmos_sensor(0x76f4, 0x05);
	write_cmos_sensor(0x76f5, 0x30);
	write_cmos_sensor(0x76f6, 0x00);
	write_cmos_sensor(0x76f7, 0x02);
	write_cmos_sensor(0x76f8, 0x05);
	write_cmos_sensor(0x76f9, 0x78);
	write_cmos_sensor(0x76fa, 0x00);
	write_cmos_sensor(0x76fb, 0x01);
	write_cmos_sensor(0x76fc, 0x05);
	write_cmos_sensor(0x76fd, 0x7c);
	write_cmos_sensor(0x76fe, 0x03);
	write_cmos_sensor(0x76ff, 0x96);
	write_cmos_sensor(0x7700, 0x05);
	write_cmos_sensor(0x7701, 0x83);
	write_cmos_sensor(0x7702, 0x05);
	write_cmos_sensor(0x7703, 0x03);
	write_cmos_sensor(0x7704, 0x05);
	write_cmos_sensor(0x7705, 0x32);
	write_cmos_sensor(0x7706, 0x05);
	write_cmos_sensor(0x7707, 0x30);
	write_cmos_sensor(0x7708, 0x00);
	write_cmos_sensor(0x7709, 0x02);
	write_cmos_sensor(0x770a, 0x05);
	write_cmos_sensor(0x770b, 0x78);
	write_cmos_sensor(0x770c, 0x00);
	write_cmos_sensor(0x770d, 0x01);
	write_cmos_sensor(0x770e, 0x05);
	write_cmos_sensor(0x770f, 0x7c);
	write_cmos_sensor(0x7710, 0x03);
	write_cmos_sensor(0x7711, 0x95);
	write_cmos_sensor(0x7712, 0x05);
	write_cmos_sensor(0x7713, 0x83);
	write_cmos_sensor(0x7714, 0x05);
	write_cmos_sensor(0x7715, 0x03);
	write_cmos_sensor(0x7716, 0x05);
	write_cmos_sensor(0x7717, 0x32);
	write_cmos_sensor(0x7718, 0x05);
	write_cmos_sensor(0x7719, 0x30);
	write_cmos_sensor(0x771a, 0x00);
	write_cmos_sensor(0x771b, 0x02);
	write_cmos_sensor(0x771c, 0x05);
	write_cmos_sensor(0x771d, 0x78);
	write_cmos_sensor(0x771e, 0x00);
	write_cmos_sensor(0x771f, 0x01);
	write_cmos_sensor(0x7720, 0x05);
	write_cmos_sensor(0x7721, 0x7c);
	write_cmos_sensor(0x7722, 0x03);
	write_cmos_sensor(0x7723, 0x94);
	write_cmos_sensor(0x7724, 0x05);
	write_cmos_sensor(0x7725, 0x83);
	write_cmos_sensor(0x7726, 0x00);
	write_cmos_sensor(0x7727, 0x01);
	write_cmos_sensor(0x7728, 0x05);
	write_cmos_sensor(0x7729, 0x03);
	write_cmos_sensor(0x772a, 0x00);
	write_cmos_sensor(0x772b, 0x01);
	write_cmos_sensor(0x772c, 0x05);
	write_cmos_sensor(0x772d, 0x32);
	write_cmos_sensor(0x772e, 0x05);
	write_cmos_sensor(0x772f, 0x30);
	write_cmos_sensor(0x7730, 0x00);
	write_cmos_sensor(0x7731, 0x02);
	write_cmos_sensor(0x7732, 0x05);
	write_cmos_sensor(0x7733, 0x78);
	write_cmos_sensor(0x7734, 0x00);
	write_cmos_sensor(0x7735, 0x01);
	write_cmos_sensor(0x7736, 0x05);
	write_cmos_sensor(0x7737, 0x7c);
	write_cmos_sensor(0x7738, 0x03);
	write_cmos_sensor(0x7739, 0x93);
	write_cmos_sensor(0x773a, 0x05);
	write_cmos_sensor(0x773b, 0x83);
	write_cmos_sensor(0x773c, 0x00);
	write_cmos_sensor(0x773d, 0x00);
	write_cmos_sensor(0x773e, 0x05);
	write_cmos_sensor(0x773f, 0x03);
	write_cmos_sensor(0x7740, 0x00);
	write_cmos_sensor(0x7741, 0x00);
	write_cmos_sensor(0x7742, 0x05);
	write_cmos_sensor(0x7743, 0x32);
	write_cmos_sensor(0x7744, 0x05);
	write_cmos_sensor(0x7745, 0x30);
	write_cmos_sensor(0x7746, 0x00);
	write_cmos_sensor(0x7747, 0x02);
	write_cmos_sensor(0x7748, 0x05);
	write_cmos_sensor(0x7749, 0x78);
	write_cmos_sensor(0x774a, 0x00);
	write_cmos_sensor(0x774b, 0x01);
	write_cmos_sensor(0x774c, 0x05);
	write_cmos_sensor(0x774d, 0x7c);
	write_cmos_sensor(0x774e, 0x03);
	write_cmos_sensor(0x774f, 0x92);
	write_cmos_sensor(0x7750, 0x05);
	write_cmos_sensor(0x7751, 0x83);
	write_cmos_sensor(0x7752, 0x05);
	write_cmos_sensor(0x7753, 0x03);
	write_cmos_sensor(0x7754, 0x00);
	write_cmos_sensor(0x7755, 0x00);
	write_cmos_sensor(0x7756, 0x05);
	write_cmos_sensor(0x7757, 0x32);
	write_cmos_sensor(0x7758, 0x05);
	write_cmos_sensor(0x7759, 0x30);
	write_cmos_sensor(0x775a, 0x00);
	write_cmos_sensor(0x775b, 0x02);
	write_cmos_sensor(0x775c, 0x05);
	write_cmos_sensor(0x775d, 0x78);
	write_cmos_sensor(0x775e, 0x00);
	write_cmos_sensor(0x775f, 0x01);
	write_cmos_sensor(0x7760, 0x05);
	write_cmos_sensor(0x7761, 0x7c);
	write_cmos_sensor(0x7762, 0x03);
	write_cmos_sensor(0x7763, 0x91);
	write_cmos_sensor(0x7764, 0x05);
	write_cmos_sensor(0x7765, 0x83);
	write_cmos_sensor(0x7766, 0x05);
	write_cmos_sensor(0x7767, 0x03);
	write_cmos_sensor(0x7768, 0x05);
	write_cmos_sensor(0x7769, 0x32);
	write_cmos_sensor(0x776a, 0x05);
	write_cmos_sensor(0x776b, 0x30);
	write_cmos_sensor(0x776c, 0x00);
	write_cmos_sensor(0x776d, 0x02);
	write_cmos_sensor(0x776e, 0x05);
	write_cmos_sensor(0x776f, 0x78);
	write_cmos_sensor(0x7770, 0x00);
	write_cmos_sensor(0x7771, 0x01);
	write_cmos_sensor(0x7772, 0x05);
	write_cmos_sensor(0x7773, 0x7c);
	write_cmos_sensor(0x7774, 0x03);
	write_cmos_sensor(0x7775, 0x90);
	write_cmos_sensor(0x7776, 0x05);
	write_cmos_sensor(0x7777, 0x83);
	write_cmos_sensor(0x7778, 0x05);
	write_cmos_sensor(0x7779, 0x03);
	write_cmos_sensor(0x777a, 0x05);
	write_cmos_sensor(0x777b, 0x32);
	write_cmos_sensor(0x777c, 0x05);
	write_cmos_sensor(0x777d, 0x30);
	write_cmos_sensor(0x777e, 0x00);
	write_cmos_sensor(0x777f, 0x02);
	write_cmos_sensor(0x7780, 0x05);
	write_cmos_sensor(0x7781, 0x78);
	write_cmos_sensor(0x7782, 0x00);
	write_cmos_sensor(0x7783, 0x01);
	write_cmos_sensor(0x7784, 0x05);
	write_cmos_sensor(0x7785, 0x7c);
	write_cmos_sensor(0x7786, 0x02);
	write_cmos_sensor(0x7787, 0x90);
	write_cmos_sensor(0x7788, 0x05);
	write_cmos_sensor(0x7789, 0x03);
	write_cmos_sensor(0x778a, 0x07);
	write_cmos_sensor(0x778b, 0x00);
	write_cmos_sensor(0x778c, 0x0f);
	write_cmos_sensor(0x778d, 0x00);
	write_cmos_sensor(0x778e, 0x08);
	write_cmos_sensor(0x778f, 0x30);
	write_cmos_sensor(0x7790, 0x08);
	write_cmos_sensor(0x7791, 0xee);
	write_cmos_sensor(0x7792, 0x0f);
	write_cmos_sensor(0x7793, 0x00);
	write_cmos_sensor(0x7794, 0x05);
	write_cmos_sensor(0x7795, 0x33);
	write_cmos_sensor(0x7796, 0x04);
	write_cmos_sensor(0x7797, 0xe5);
	write_cmos_sensor(0x7798, 0x06);
	write_cmos_sensor(0x7799, 0x52);
	write_cmos_sensor(0x779a, 0x04);
	write_cmos_sensor(0x779b, 0xe4);
	write_cmos_sensor(0x779c, 0x00);
	write_cmos_sensor(0x779d, 0x00);
	write_cmos_sensor(0x779e, 0x06);
	write_cmos_sensor(0x779f, 0x5e);
	write_cmos_sensor(0x77a0, 0x00);
	write_cmos_sensor(0x77a1, 0x0f);
	write_cmos_sensor(0x77a2, 0x06);
	write_cmos_sensor(0x77a3, 0x1e);
	write_cmos_sensor(0x77a4, 0x00);
	write_cmos_sensor(0x77a5, 0x02);
	write_cmos_sensor(0x77a6, 0x06);
	write_cmos_sensor(0x77a7, 0xa2);
	write_cmos_sensor(0x77a8, 0x00);
	write_cmos_sensor(0x77a9, 0x01);
	write_cmos_sensor(0x77aa, 0x06);
	write_cmos_sensor(0x77ab, 0xae);
	write_cmos_sensor(0x77ac, 0x00);
	write_cmos_sensor(0x77ad, 0x03);
	write_cmos_sensor(0x77ae, 0x05);
	write_cmos_sensor(0x77af, 0x30);
	write_cmos_sensor(0x77b0, 0x09);
	write_cmos_sensor(0x77b1, 0x19);
	write_cmos_sensor(0x77b2, 0x0f);
	write_cmos_sensor(0x77b3, 0x00);
	write_cmos_sensor(0x77b4, 0x05);
	write_cmos_sensor(0x77b5, 0x33);
	write_cmos_sensor(0x77b6, 0x04);
	write_cmos_sensor(0x77b7, 0xe5);
	write_cmos_sensor(0x77b8, 0x06);
	write_cmos_sensor(0x77b9, 0x52);
	write_cmos_sensor(0x77ba, 0x04);
	write_cmos_sensor(0x77bb, 0xe4);
	write_cmos_sensor(0x77bc, 0x00);
	write_cmos_sensor(0x77bd, 0x00);
	write_cmos_sensor(0x77be, 0x06);
	write_cmos_sensor(0x77bf, 0x5e);
	write_cmos_sensor(0x77c0, 0x00);
	write_cmos_sensor(0x77c1, 0x0f);
	write_cmos_sensor(0x77c2, 0x06);
	write_cmos_sensor(0x77c3, 0x1e);
	write_cmos_sensor(0x77c4, 0x00);
	write_cmos_sensor(0x77c5, 0x02);
	write_cmos_sensor(0x77c6, 0x06);
	write_cmos_sensor(0x77c7, 0xa2);
	write_cmos_sensor(0x77c8, 0x00);
	write_cmos_sensor(0x77c9, 0x01);
	write_cmos_sensor(0x77ca, 0x06);
	write_cmos_sensor(0x77cb, 0xae);
	write_cmos_sensor(0x77cc, 0x00);
	write_cmos_sensor(0x77cd, 0x03);
	write_cmos_sensor(0x77ce, 0x05);
	write_cmos_sensor(0x77cf, 0x30);
	write_cmos_sensor(0x77d0, 0x0f);
	write_cmos_sensor(0x77d1, 0x00);
	write_cmos_sensor(0x77d2, 0x00);
	write_cmos_sensor(0x77d3, 0x00);
	write_cmos_sensor(0x77d4, 0x00);
	write_cmos_sensor(0x77d5, 0x02);
	write_cmos_sensor(0x77d6, 0x04);
	write_cmos_sensor(0x77d7, 0xe5);
	write_cmos_sensor(0x77d8, 0x04);
	write_cmos_sensor(0x77d9, 0xe4);
	write_cmos_sensor(0x77da, 0x05);
	write_cmos_sensor(0x77db, 0x33);
	write_cmos_sensor(0x77dc, 0x07);
	write_cmos_sensor(0x77dd, 0x10);
	write_cmos_sensor(0x77de, 0x00);
	write_cmos_sensor(0x77df, 0x00);
	write_cmos_sensor(0x77e0, 0x01);
	write_cmos_sensor(0x77e1, 0xbb);
	write_cmos_sensor(0x77e2, 0x00);
	write_cmos_sensor(0x77e3, 0x00);
	write_cmos_sensor(0x77e4, 0x01);
	write_cmos_sensor(0x77e5, 0xaa);
	write_cmos_sensor(0x77e6, 0x00);
	write_cmos_sensor(0x77e7, 0x00);
	write_cmos_sensor(0x77e8, 0x01);
	write_cmos_sensor(0x77e9, 0x99);
	write_cmos_sensor(0x77ea, 0x00);
	write_cmos_sensor(0x77eb, 0x00);
	write_cmos_sensor(0x77ec, 0x01);
	write_cmos_sensor(0x77ed, 0x88);
	write_cmos_sensor(0x77ee, 0x00);
	write_cmos_sensor(0x77ef, 0x00);
	write_cmos_sensor(0x77f0, 0x01);
	write_cmos_sensor(0x77f1, 0x77);
	write_cmos_sensor(0x77f2, 0x00);
	write_cmos_sensor(0x77f3, 0x00);
	write_cmos_sensor(0x77f4, 0x01);
	write_cmos_sensor(0x77f5, 0x66);
	write_cmos_sensor(0x77f6, 0x00);
	write_cmos_sensor(0x77f7, 0x00);
	write_cmos_sensor(0x77f8, 0x01);
	write_cmos_sensor(0x77f9, 0x55);
	write_cmos_sensor(0x77fa, 0x00);
	write_cmos_sensor(0x77fb, 0x00);
	write_cmos_sensor(0x77fc, 0x01);
	write_cmos_sensor(0x77fd, 0x44);
	write_cmos_sensor(0x77fe, 0x00);
	write_cmos_sensor(0x77ff, 0x00);
	write_cmos_sensor(0x7800, 0x01);
	write_cmos_sensor(0x7801, 0x33);
	write_cmos_sensor(0x7802, 0x00);
	write_cmos_sensor(0x7803, 0x00);
	write_cmos_sensor(0x7804, 0x01);
	write_cmos_sensor(0x7805, 0x22);
	write_cmos_sensor(0x7806, 0x00);
	write_cmos_sensor(0x7807, 0x00);
	write_cmos_sensor(0x7808, 0x01);
	write_cmos_sensor(0x7809, 0x11);
	write_cmos_sensor(0x780a, 0x00);
	write_cmos_sensor(0x780b, 0x00);
	write_cmos_sensor(0x780c, 0x01);
	write_cmos_sensor(0x780d, 0x00);
	write_cmos_sensor(0x780e, 0x01);
	write_cmos_sensor(0x780f, 0xff);
	write_cmos_sensor(0x7810, 0x07);
	write_cmos_sensor(0x7811, 0x00);
	write_cmos_sensor(0x7812, 0x02);
	write_cmos_sensor(0x7813, 0xa0);
	write_cmos_sensor(0x7814, 0x0f);
	write_cmos_sensor(0x7815, 0x00);
	write_cmos_sensor(0x7816, 0x08);
	write_cmos_sensor(0x7817, 0x35);
	write_cmos_sensor(0x7818, 0x06);
	write_cmos_sensor(0x7819, 0x52);
	write_cmos_sensor(0x781a, 0x04);
	write_cmos_sensor(0x781b, 0xe4);
	write_cmos_sensor(0x781c, 0x00);
	write_cmos_sensor(0x781d, 0x00);
	write_cmos_sensor(0x781e, 0x06);
	write_cmos_sensor(0x781f, 0x5e);
	write_cmos_sensor(0x7820, 0x05);
	write_cmos_sensor(0x7821, 0x33);
	write_cmos_sensor(0x7822, 0x09);
	write_cmos_sensor(0x7823, 0x19);
	write_cmos_sensor(0x7824, 0x06);
	write_cmos_sensor(0x7825, 0x1e);
	write_cmos_sensor(0x7826, 0x05);
	write_cmos_sensor(0x7827, 0x33);
	write_cmos_sensor(0x7828, 0x00);
	write_cmos_sensor(0x7829, 0x01);
	write_cmos_sensor(0x782a, 0x06);
	write_cmos_sensor(0x782b, 0x24);
	write_cmos_sensor(0x782c, 0x06);
	write_cmos_sensor(0x782d, 0x20);
	write_cmos_sensor(0x782e, 0x0f);
	write_cmos_sensor(0x782f, 0x00);
	write_cmos_sensor(0x7830, 0x08);
	write_cmos_sensor(0x7831, 0x35);
	write_cmos_sensor(0x7832, 0x07);
	write_cmos_sensor(0x7833, 0x10);
	write_cmos_sensor(0x7834, 0x00);
	write_cmos_sensor(0x7835, 0x00);
	write_cmos_sensor(0x7836, 0x01);
	write_cmos_sensor(0x7837, 0xbb);
	write_cmos_sensor(0x7838, 0x00);
	write_cmos_sensor(0x7839, 0x00);
	write_cmos_sensor(0x783a, 0x01);
	write_cmos_sensor(0x783b, 0xaa);
	write_cmos_sensor(0x783c, 0x00);
	write_cmos_sensor(0x783d, 0x00);
	write_cmos_sensor(0x783e, 0x01);
	write_cmos_sensor(0x783f, 0x99);
	write_cmos_sensor(0x7840, 0x00);
	write_cmos_sensor(0x7841, 0x00);
	write_cmos_sensor(0x7842, 0x01);
	write_cmos_sensor(0x7843, 0x88);
	write_cmos_sensor(0x7844, 0x00);
	write_cmos_sensor(0x7845, 0x00);
	write_cmos_sensor(0x7846, 0x01);
	write_cmos_sensor(0x7847, 0x77);
	write_cmos_sensor(0x7848, 0x00);
	write_cmos_sensor(0x7849, 0x00);
	write_cmos_sensor(0x784a, 0x01);
	write_cmos_sensor(0x784b, 0x66);
	write_cmos_sensor(0x784c, 0x00);
	write_cmos_sensor(0x784d, 0x00);
	write_cmos_sensor(0x784e, 0x01);
	write_cmos_sensor(0x784f, 0x55);
	write_cmos_sensor(0x7850, 0x00);
	write_cmos_sensor(0x7851, 0x00);
	write_cmos_sensor(0x7852, 0x01);
	write_cmos_sensor(0x7853, 0x44);
	write_cmos_sensor(0x7854, 0x00);
	write_cmos_sensor(0x7855, 0x00);
	write_cmos_sensor(0x7856, 0x01);
	write_cmos_sensor(0x7857, 0x33);
	write_cmos_sensor(0x7858, 0x00);
	write_cmos_sensor(0x7859, 0x00);
	write_cmos_sensor(0x785a, 0x01);
	write_cmos_sensor(0x785b, 0x22);
	write_cmos_sensor(0x785c, 0x00);
	write_cmos_sensor(0x785d, 0x00);
	write_cmos_sensor(0x785e, 0x01);
	write_cmos_sensor(0x785f, 0x11);
	write_cmos_sensor(0x7860, 0x00);
	write_cmos_sensor(0x7861, 0x00);
	write_cmos_sensor(0x7862, 0x01);
	write_cmos_sensor(0x7863, 0x00);
	write_cmos_sensor(0x7864, 0x07);
	write_cmos_sensor(0x7865, 0x00);
	write_cmos_sensor(0x7866, 0x01);
	write_cmos_sensor(0x7867, 0xff);
	write_cmos_sensor(0x7868, 0x02);
	write_cmos_sensor(0x7869, 0xa0);
	write_cmos_sensor(0x786a, 0x0f);
	write_cmos_sensor(0x786b, 0x00);
	write_cmos_sensor(0x786c, 0x08);
	write_cmos_sensor(0x786d, 0x3a);
	write_cmos_sensor(0x786e, 0x08);
	write_cmos_sensor(0x786f, 0x6a);
	write_cmos_sensor(0x7870, 0x0f);
	write_cmos_sensor(0x7871, 0x00);
	write_cmos_sensor(0x7872, 0x04);
	write_cmos_sensor(0x7873, 0xc0);
	write_cmos_sensor(0x7874, 0x09);
	write_cmos_sensor(0x7875, 0x19);
	write_cmos_sensor(0x7876, 0x04);
	write_cmos_sensor(0x7877, 0x99);
	write_cmos_sensor(0x7878, 0x07);
	write_cmos_sensor(0x7879, 0x14);
	write_cmos_sensor(0x787a, 0x00);
	write_cmos_sensor(0x787b, 0x01);
	write_cmos_sensor(0x787c, 0x04);
	write_cmos_sensor(0x787d, 0xa4);
	write_cmos_sensor(0x787e, 0x00);
	write_cmos_sensor(0x787f, 0x07);
	write_cmos_sensor(0x7880, 0x04);
	write_cmos_sensor(0x7881, 0xa6);
	write_cmos_sensor(0x7882, 0x00);
	write_cmos_sensor(0x7883, 0x00);
	write_cmos_sensor(0x7884, 0x04);
	write_cmos_sensor(0x7885, 0xa0);
	write_cmos_sensor(0x7886, 0x04);
	write_cmos_sensor(0x7887, 0x80);
	write_cmos_sensor(0x7888, 0x04);
	write_cmos_sensor(0x7889, 0x00);
	write_cmos_sensor(0x788a, 0x05);
	write_cmos_sensor(0x788b, 0x03);
	write_cmos_sensor(0x788c, 0x06);
	write_cmos_sensor(0x788d, 0x00);
	write_cmos_sensor(0x788e, 0x0f);
	write_cmos_sensor(0x788f, 0x00);
	write_cmos_sensor(0x7890, 0x0f);
	write_cmos_sensor(0x7891, 0x00);
	write_cmos_sensor(0x7892, 0x0f);
	write_cmos_sensor(0x7893, 0x00);
	write_cmos_sensor(0x30a0, 0x01);
	write_cmos_sensor(0x30a1, 0x44);
	write_cmos_sensor(0x30a2, 0x00);
	write_cmos_sensor(0x30a3, 0xbc);
	write_cmos_sensor(0x30a4, 0x06);
	write_cmos_sensor(0x30a5, 0x4b);
	write_cmos_sensor(0x30a6, 0x03);
	write_cmos_sensor(0x30a7, 0x8b);
	write_cmos_sensor(0x30a8, 0x00);
	write_cmos_sensor(0x30a9, 0x05);
	write_cmos_sensor(0x30aa, 0x00);
	write_cmos_sensor(0x30ab, 0x00);
	write_cmos_sensor(0x30ac, 0x05);
	write_cmos_sensor(0x30ad, 0x00);
	write_cmos_sensor(0x30ae, 0x02);
	write_cmos_sensor(0x30af, 0xd0);
	write_cmos_sensor(0x30b0, 0x08);
	write_cmos_sensor(0x30b1, 0x2e);
	write_cmos_sensor(0x30b2, 0x02);
	write_cmos_sensor(0x30b3, 0xee);
	write_cmos_sensor(0x30b6, 0x02);
	write_cmos_sensor(0x30b7, 0xea);
	write_cmos_sensor(0x3196, 0x00);
	write_cmos_sensor(0x3197, 0x0a);
	write_cmos_sensor(0x3195, 0x19);
	write_cmos_sensor(0x31e3, 0x01);
	write_cmos_sensor(0x31e4, 0x08);
	write_cmos_sensor(0x3250, 0xf7);
	mDELAY(10);

	cam_pr_debug("Exit!");
}

static void hs_video_setting(void)
{
	cam_pr_debug("E\n");
	video_720p_setting();
}

static void slim_video_setting(void)
{
	cam_pr_debug("E\n");

	video_1080p_setting();
}
static void custom1_setting(void)
{
	cam_pr_debug("E %s\n", __func__);
	sensor_hdr();
}
/*************************************************************************
 * FUNCTION
 *    get_imgsensor_id
 *
 * DESCRIPTION
 *    This function get the sensor ID
 *
 * PARAMETERS
 *    *sensorID : return the sensor ID
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	cam_pr_debug("E\n");

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				cam_pr_debug("----i2c write id: 0x%x, sensor id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
				goto get_imgsensor_id_exit;
			}
			cam_pr_debug("----Read sensor id fail, write id:0x%x id: 0x%x\n",
			    imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

get_imgsensor_id_exit:
	cam_pr_debug("X\n");

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	LOG_2;

	cam_pr_debug("E\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				cam_pr_debug("----i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			cam_pr_debug("----Read sensor id fail, write id:0x%x id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	/* for zsd multi capture setting */
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*    open  */

/*************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	cam_pr_debug("E\n");

	return ERROR_NONE;
}				/*    close  */

/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *    *sensor_config_data : address pointer of line numbers in
 *    *one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	/* set_mirror_flip(imgsensor.mirror); */
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    preview   */

/*************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			cam_pr_debug("Warning: current_fps %d!\n",
					imgsensor.current_fps);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);

	/* set_mirror_flip(imgsensor.mirror); */
	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	/* set_mirror_flip(imgsensor.mirror); */
	return ERROR_NONE;
}				/*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	/* set_mirror_flip(imgsensor.mirror); */
	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	/* set_mirror_flip(imgsensor.mirror); */
	return ERROR_NONE;
}				/*    slim_video     */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("E %s\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();

	return ERROR_NONE;
}

static kal_uint32 get_resolution(
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{
	cam_pr_debug("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
	return ERROR_NONE;
}				/*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MSDK_SENSOR_INFO_STRUCT *sensor_info,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->HDR_Support = 5;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	cam_pr_debug("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	default:
		cam_pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{				/* This Function not used after ROME */
	cam_pr_debug("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MUINT32 framerate)
{
	/* kal_int16 dummyLine; */
	kal_uint32 frame_length;

	cam_pr_debug("scenario_id = %d, framerate = %d\n",
			scenario_id, framerate);

	if (framerate == 0)
		return ERROR_NONE;
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 /
		    imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
				imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10 /
		    imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.normal_video.framelength) ?
		    (frame_length -
		     imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength +
		    imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			frame_length =
			imgsensor_info.cap.pclk / framerate * 10 /
			imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			    (frame_length >
			     imgsensor_info.cap.framelength) ?
			    (frame_length -
			     imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength +
			imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
		    imgsensor_info.hs_video.pclk / framerate * 10 /
		    imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length >
			 imgsensor_info.hs_video.framelength) ?
			(frame_length -
			 imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
		    imgsensor_info.slim_video.pclk / framerate * 10 /
		    imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.slim_video.framelength) ?
		    (frame_length -
		     imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength +
		    imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		break;
	default:		/* coding with  preview scenario by default */
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 /
		    imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ?
		    (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/*set_dummy();*/
		cam_pr_debug("error scenario_id = %d, we use preview scenario\n",
				scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,
	MUINT32 *framerate)
{
	cam_pr_debug("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	/* check,power+volum+ */
	cam_pr_debug("enable: %d\n", enable);
	write_cmos_sensor(0x3253, 0x80);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);


	return ERROR_NONE;
}

static kal_uint32 ov2718_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	kal_uint32 rgain_32, grgain_32, gbgain_32, bgain_32;
	kal_uint32 roffset_32, boffset_32;

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;
	roffset_32 = ((rgain_32 - 256) / 2);
	boffset_32 = ((bgain_32 - 256) / 2);
	cam_pr_debug("ov2718r2a_awb_gain\n");
	cam_pr_debug("[ov2718a_awb_gain] ABS_GAIN_GR:%d, grgain_32:%d\n",
			pSetSensorAWB->ABS_GAIN_GR,
		grgain_32);
	cam_pr_debug("[ov2718a_awb_gain] ABS_GAIN_R:%d, rgain_32:%d\n",
			pSetSensorAWB->ABS_GAIN_R,
		rgain_32);
	cam_pr_debug("[ov2718a_awb_gain] ABS_GAIN_B:%d, bgain_32:%d\n",
			pSetSensorAWB->ABS_GAIN_B,
		bgain_32);
	cam_pr_debug("[ov2718a_awb_gain] ABS_GAIN_GB:%d, gbgain_32:%d\n",
			pSetSensorAWB->ABS_GAIN_GB,
		gbgain_32);
	cam_pr_debug("[ov2718a_awb_gain] r_offset:%d, b_offset:%d\n",
			roffset_32, boffset_32);

	/*HCG gain*/
	write_cmos_sensor(0x3360, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3361, rgain_32 & 0xFF);
	write_cmos_sensor(0x3362, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3363, grgain_32 & 0xFF);
	write_cmos_sensor(0x3364, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3365, gbgain_32 & 0xFF);
	write_cmos_sensor(0x3366, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3367, bgain_32 & 0xFF);
	/*HCG offset*/
	write_cmos_sensor(0x3378, (roffset_32 >> 16) & 0xFF);
	write_cmos_sensor(0x3379, (roffset_32 >> 8) & 0xFF);
	write_cmos_sensor(0x337a, roffset_32 & 0xFF);

	write_cmos_sensor(0x3381, (boffset_32 >> 16) & 0xFF);
	write_cmos_sensor(0x3382, (boffset_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3383, boffset_32 & 0xFF);

	/*LCG*/
	write_cmos_sensor(0x3368, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3369, rgain_32 & 0xFF);
	write_cmos_sensor(0x336a, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x336b, grgain_32 & 0xFF);
	write_cmos_sensor(0x336c, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x336d, gbgain_32 & 0xFF);
	write_cmos_sensor(0x336e, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor(0x336f, bgain_32 & 0xFF);
	/*LCG offset*/
	write_cmos_sensor(0x3384, (roffset_32 >> 16) & 0xFF);
	write_cmos_sensor(0x3385, (roffset_32 >> 8) & 0xFF);
	write_cmos_sensor(0x3386, roffset_32 & 0xFF);

	write_cmos_sensor(0x338d, (boffset_32 >> 16) & 0xFF);
	write_cmos_sensor(0x338e, (boffset_32 >> 8) & 0xFF);
	write_cmos_sensor(0x338f, boffset_32 & 0xFF);
	cam_pr_debug("[ov2718a_awb_gain] r offset 0x3378:0x%x, 0x3379:0x%x, 0x337a:0x%x\n",
		read_cmos_sensor(0x3378),
		read_cmos_sensor(0x3379),
		read_cmos_sensor(0x337a));

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
		UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
		(struct SET_SENSOR_AWB_GAIN *)feature_para;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	cam_pr_debug("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		cam_pr_debug("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16,
				*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM) *feature_data,
				*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
				(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		cam_pr_debug("current fps :%d\n", (UINT32) *feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		cam_pr_debug("ihdr enable :%d\n", (UINT8)*feature_data);
		spin_lock(&imgsensor_drv_lock);
			imgsensor.hdr_mode = (UINT8)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		cam_pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				(UINT32) *feature_data);

		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
			(uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
					(void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
					(void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
					(void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
					(void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
					(void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		cam_pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
				(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));
		ihdr_write_shutter_gain((UINT16) *feature_data,
				(UINT16) *(feature_data + 1),
					(UINT16) *(feature_data + 2));

		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		ov2718_awb_gain(pSetSensorAWB);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		cam_pr_debug("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu\n",
				*feature_data);
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		cam_pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		cam_pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME\n");
		streaming_control(KAL_TRUE);
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV2718MIPISensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}    /*    OV2718MIPISensorInit    */
