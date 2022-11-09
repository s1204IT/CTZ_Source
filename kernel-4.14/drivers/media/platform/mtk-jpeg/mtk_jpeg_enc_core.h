/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Xia Jiang <xia.jiang@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _MTK_JPEG_CORE_H
#define _MTK_JPEG_CORE_H

#include <linux/interrupt.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>

#define MTK_JPEG_ENC_NAME		"mtk-jpeg-enc"

#define MTK_JPEG_FMT_FLAG_ENC_OUTPUT	(1 << 0)
#define MTK_JPEG_FMT_FLAG_ENC_CAPTURE	(1 << 1)

#define MTK_JPEG_ENC_FMT_TYPE_OUTPUT	1
#define MTK_JPEG_ENC_FMT_TYPE_CAPTURE	2

#define MTK_JPEG_MIN_WIDTH	32
#define MTK_JPEG_MIN_HEIGHT	32
#define MTK_JPEG_MAX_WIDTH	8192
#define MTK_JPEG_MAX_HEIGHT	8192
#define MTK_JPEG_MAX_N_PALNE	4

#define MTK_JPEG_BENCHMARK	1

enum mtk_jpeg_ctx_state {
	MTK_JPEG_INIT = 0,
	MTK_JPEG_RUNNING,
	MTK_JPEG_SOURCE_CHANGE,
};

enum jpeg_enc_yuv_fmt {
	JPEG_YUV_FORMAT_YUYV = 0,
	JPEG_YUV_FORMAT_YVYU = 1,
	JPEG_YUV_FORMAT_NV12 = 2,
	JEPG_YUV_FORMAT_NV21 = 3,
};

enum JPEG_ENCODE_QUALITY_ENUM {
	JPEG_ENCODE_QUALITY_Q60 = 0x0,
	JPEG_ENCODE_QUALITY_Q80 = 0x1,
	JPEG_ENCODE_QUALITY_Q90 = 0x2,
	JPEG_ENCODE_QUALITY_Q95 = 0x3,

	JPEG_ENCODE_QUALITY_Q39 = 0x4,
	JPEG_ENCODE_QUALITY_Q68 = 0x5,
	JPEG_ENCODE_QUALITY_Q84 = 0x6,
	JPEG_ENCODE_QUALITY_Q92 = 0x7,

	JPEG_ENCODE_QUALITY_Q48 = 0x8,
	JPEG_ENCODE_QUALITY_Q74 = 0xA,
	JPEG_ENCODE_QUALITY_Q87 = 0xB,

	JPEG_ENCODE_QUALITY_Q34 = 0xC,
	JPEG_ENCODE_QUALITY_Q64 = 0xE,
	JPEG_ENCODE_QUALITY_Q82 = 0xF,

	JPEG_ENCODE_QUALITY_Q97 = 0x10,

	JPEG_ENCODE_QUALITY_ALL = 0xFFFFFFFF
};

/**
 * struct mt_jpeg - JPEG IP abstraction
 * @lock:		the mutex protecting this structure
 * @dev_lock:		the mutex protecting JPEG IP
 * @irq_lock:		spinlock protecting the device contexts
 * @workqueue:		decode work queue
 * @dev:		JPEG device
 * @v4l2_dev:		v4l2 device for mem2mem mode
 * @m2m_dev:		v4l2 mem2mem device data
 * @alloc_ctx:		videobuf2 memory allocator's context
 * @dec_vdev:		video device node for decoder mem2mem mode
 * @dec_reg_base:	JPEG registers mapping
 * @clk_venc_jdec:	JPEG clock
 * @larb:		SMI device
 */
struct mtk_jpeg_enc_dev {
	struct mutex		lock;
	struct mutex		dev_lock;
	spinlock_t		irq_lock;
	struct workqueue_struct	*workqueue;
	struct device		*dev;
	struct platform_device *plat_dev;
	struct v4l2_device	v4l2_dev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct device   *alloc_ctx;
	struct video_device	*enc_vdev;
	void __iomem		*enc_reg_base;
	struct clk		*clk_venc_jenc;
	struct device		*larb;
};

/**
 * struct jpeg_fmt - driver's internal color format data
 * @name:	format descritpion
 * @fourcc:	the fourcc code, 0 if not applicable
 * @depth:	number of bits per pixel
 * @colplanes:	number of color planes (1 for packed formats)
 * @h_align:	horizontal alignment order (align to 2^h_align)
 * @v_align:	vertical alignment order (align to 2^v_align)
 * @flags:	flags describing format applicability
 */
struct mtk_jpeg_enc_fmt {
	char	*name;
	u32	fourcc;
	int	depth;
	int	colplanes;
	int	h_align;
	int	v_align;
	u32	flags;
};

/**
 * mtk_jpeg_enc_q_data - parameters of one queue
 * @fmt:	driver-specific format of this queue
 * @w:		image width
 * @h:		image height
 * @size:		image buffer size in bytes
 * @bytesperline:	distance in bytes between
 *			the leftmost pixels in two adjacent lines
 * @sizeimage:	image buffer size in bytes
 */
struct mtk_jpeg_enc_q_data {
	struct mtk_jpeg_enc_fmt	*fmt;
	u32			w;
	u32			h;
	u32			size;
	u32			bytesperline[MTK_JPEG_MAX_N_PALNE];
	u32			sizeimage[MTK_JPEG_MAX_N_PALNE];
};

struct jpeg_enc_param {
	u32 enable_exif;
	u32 enc_quality;
	u32 restart_interval;
};

struct mtk_jpeg_enc_param {
	u32 enc_w;
	u32 enc_h;
	u32 enable_exif;
	u32 enc_quality;
	u32 enc_format;
	u32 restart_interval;
	u32 img_stride;
	u32 mem_stride;
	u32 total_encdu;
};

/**
 * mtk_jpeg_enc_ctx - the device context data
 * @jpeg:			JPEG IP device for this context
 * @out_q:			source (output) queue information
 * @cap_q:			destination (capture) queue queue information
 * @fh:			V4L2 file handle
 * @enc_irq_ret:		jpeg encoder irq value
 * @jpeg_param:		parameters for HW encoding
 * @state:			state of the context
 * @ctrl_hdl:			handler for v4l2 framework
 */
struct mtk_jpeg_enc_ctx {
	struct mtk_jpeg_enc_dev		*jpeg;
	struct mtk_jpeg_enc_q_data		out_q;
	struct mtk_jpeg_enc_q_data		cap_q;
	struct v4l2_fh			fh;
	u32				enc_irq_ret;
	enum mtk_jpeg_ctx_state		state;
	struct jpeg_enc_param		jpeg_param;
	struct v4l2_ctrl_handler	ctrl_hdl;
#if MTK_JPEG_BENCHMARK
	struct timeval			jpeg_enc_dec_start;
	struct timeval			jpeg_hw_sta;
	struct timeval			jpeg_hw_end;
	uint32_t			total_enc_dec_cnt;
	uint32_t			total_enc_dec_time;
	uint32_t			total_parse_cnt;
	uint32_t			total_parse_time;
#endif
};

static inline struct mtk_jpeg_enc_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mtk_jpeg_enc_ctx, ctrl_hdl);
}
#endif /* _MTK_JPEG_CORE_H */
