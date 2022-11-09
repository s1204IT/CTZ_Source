/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _MTK_CAMERA_UTIL_H_
#define _MTK_CAMERA_UTIL_H_

extern int mtk_camera_dbg_level;

enum camera_buffer_type {
	BUFFER_STEREO = 0,
	BUFFER_MAIN1  = 1,
	BUFFER_MAIN2  = 2,
};

enum camera_buffer_status {
	BUFFER_FILLED = 0,
	BUFFER_EMPTY  = 1,
	BUFFER_ERROR  = 2,
};

struct plane_buffer {
	unsigned long size;
	unsigned long payload;
	dma_addr_t dma_addr;
	unsigned int bytesperline;
};

struct mtk_camera_mem {
	unsigned int status;
	unsigned int index;
	unsigned int format;
	unsigned int num_planes;
	unsigned long size;
	struct list_head list;
	enum camera_buffer_type type;
	struct plane_buffer planes[VIDEO_MAX_PLANES];
};

#define DEBUG	1

#define ALIGN_CEIL(value, align)	\
	(((value) + (align) - 1L) & ~((align) - 1L))

#if defined(DEBUG)

#define mtk_camera_debug(level, fmt, args...)				 \
	do {								 \
		if (mtk_camera_dbg_level >= 0)			 \
			pr_info("[MTKCAM] level=%d %s(),%d: " fmt "\n",\
				level, __func__, __LINE__, ##args);	 \
	} while (0)

#define mtk_camera_err(fmt, args...)                \
	pr_info("[MTKCAM][ERROR] %s:%d: " fmt "\n", __func__, __LINE__, \
	       ##args)

#define mtk_camera_debug_enter()  mtk_camera_debug(3, "+")
#define mtk_camera_debug_leave()  mtk_camera_debug(3, "-")

#else

#define mtk_camera_debug(level, fmt, args...)
#define mtk_camera_err(fmt, args...)
#define mtk_camera_debug_enter()
#define mtk_camera_debug_leave()

#endif

#endif /*_MTK_CAMERA_UTIL_H_*/
