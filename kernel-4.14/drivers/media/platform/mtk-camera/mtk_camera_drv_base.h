/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _CAMERA_DRV_BASE_
#define _CAMERA_DRV_BASE_

#include "mtk_camera_if.h"

struct mtk_camera_if {
	int (*init)(void *ctx, unsigned long *handle);

	int (*capture)(unsigned long handle, void *fb);

	int (*start_stream)(unsigned long handle);

	int (*init_buffer)(unsigned long handle, void *fb);

	int (*deinit_buffer)(unsigned long handle, void *fb);

	int (*get_param)(unsigned long handle, enum camera_get_param_type type,
			 void *out);

	int (*set_param)(unsigned long handle, enum camera_set_param_type type,
			 void *in);

	int (*stop_stream)(unsigned long handle);

	void (*deinit)(unsigned long handle);
};

#endif /*_CAMERA_DRV_BASE_*/
