/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _CAMERA_IF_H_
#define _CAMERA_IF_H_

enum camera_get_param_type {
	GET_PARAM = 0
};

enum camera_set_param_type {
	SET_PARAM_FRAME_SIZE,
	SET_PARAM_BRIGHTNESS,
	SET_PARAM_CONTRAST,
	SET_PARAM_SATURATION,
	SET_PARAM_SHARPNESS,
	SET_PARAM_HUE,
	SET_PARAM_GAMMA,
	SET_PARAM_AUTO_WHITE_BALANCE,
	SET_PARAM_DO_WHITE_BALANCE,
	SET_PARAM_WHITE_BALANCE_TEMP,
	SET_PARAM_EXPOSURE,
	SET_PARAM_AUTOGAIN,
	SET_PARAM_GAIN,
	SET_PARAM_POWER_LINE_FREQ,
	SET_PARAM_BACKLIGHT_COMPENSATION,
	SET_PARAM_MIN_FPS,
	SET_PARAM_MAX_FPS,
	SET_PARAM_CAMERA_CLASS,
	SET_PARAM_EXPOSURE_AUTO,
	SET_PARAM_EXPOSURE_ABSOLUTE,
	SET_PARAM_EXPOSURE_AUTO_PRI,
	SET_PARAM_FOCUS_AUTO,
	SET_PARAM_FOCUS_RELATIVE,
	SET_PARAM_FOCUS_ABSOLUTE,
	SET_PARAM_PRIVACY,
	SET_PARAM_BAND_STOP_FILT,
	SET_PARAM_PAN_RELEATIVE,
	SET_PARAM_PAN_RESET,
	SET_PARAM_PAN_ABSOLUTE,
	SET_PARAM_TILT_RELATIVE,
	SET_PARAM_TILT_RESET,
	SET_PARAM_TILT_ABSOLUTE,
	SET_PARAM_ZOOM_ABSOLUTE,
	SET_PARAM_ZOOM_RELATIVE,
	SET_PARAM_ZOOM_CONTINUOUS
};

int  camera_if_init(void *ctx);

void camera_if_deinit(void *ctx);

int  camera_if_start_stream(void *ctx);

int  camera_if_stop_stream(void *ctx);

int  camera_if_init_buffer(void *ctx, void *fb);

int  camera_if_deinit_buffer(void *ctx, void *fb);
int  camera_if_capture(void *ctx, void *fb);

int  camera_if_get_param(void *ctx,
			enum camera_get_param_type type,
			void *out);

int  camera_if_set_param(void *ctx,
			enum camera_set_param_type type,
			void *in);

#endif /*_CAMERA_IF_H_*/
