/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _MTK_CAMERA_DRV_H_
#define _MTK_CAMERA_DRV_H_

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <linux/delay.h>

#include "mtk_camera_util.h"
#include "mtk_camera_drv_base.h"

#define MTK_CAMERA_NAME		"mtk-v4l2-camera"
#define MTK_CAMERA_DEVICE	"mtk-camera"
#define MTK_PLATFORM_STR	"platform:mt8183"

#define MTK_CAMERA_MAX_PLANES   3
#define MTK_CAMERA_MIN_BUFFERS  6

#define MTK_CAMERA_MAX_CTRLS_HINT	40

/**
 * struct mtk_camera_fmt
 * Structure used to store information about pixelformats
 */
struct mtk_camera_fmt {
	char *name;
	u32  fourcc;
	u32  bpp[3];
	bool mplane;
	u32  num_planes;
};

/* struct mtk_camera_ctrls - the image processor control set
 * @brightness:
 * @saturation:
 * @auto_white_balance
 */
struct mtk_camera_ctrls {
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *gamma;
	struct v4l2_ctrl *auto_white_balance;
	struct v4l2_ctrl *do_white_balance;
	struct v4l2_ctrl *white_balance_temperature;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *autogain;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *power_line_frequency;
	struct v4l2_ctrl *backlight_compensation;
	struct v4l2_ctrl *min_fps;
	struct v4l2_ctrl *max_fps;
	struct v4l2_ctrl *camera_class;
	struct v4l2_ctrl *exposure_auto;
	struct v4l2_ctrl *exposure_absolute;
	struct v4l2_ctrl *exposure_auto_priority;
	struct v4l2_ctrl *focus_auto;
	struct v4l2_ctrl *focus_relative;
	struct v4l2_ctrl *focus_absolute;
	struct v4l2_ctrl *privacy;
	struct v4l2_ctrl *band_stop_filter;
	struct v4l2_ctrl *pan_relative;
	struct v4l2_ctrl *pan_reset;
	struct v4l2_ctrl *pan_absolute;
	struct v4l2_ctrl *tilt_relative;
	struct v4l2_ctrl *tilt_reset;
	struct v4l2_ctrl *tilt_absolute;
	struct v4l2_ctrl *zoom_absolute;
	struct v4l2_ctrl *zoon_relative;
	struct v4l2_ctrl *zoom_continuous;
};

/**
 * struct mtk_q_data - Structure used to store information about queue
 */
struct mtk_q_data {
	unsigned int    width;
	unsigned int    height;
	enum v4l2_field field;
	struct mtk_camera_fmt *fmt;
	unsigned int    sizeimage[MTK_CAMERA_MAX_PLANES];
	unsigned int    bytesperline[MTK_CAMERA_MAX_PLANES];
};

enum mtk_instance_state {
	MTK_STATE_FREE = 0,
	MTK_STATE_INIT = 1,
	MTK_STATE_START = 2,
	MTK_STATE_FLUSH = 3,
	MTK_STATE_ABORT = 4,
};

struct mtk_camera_ctx {
	struct mtk_camera_dev *dev;
	struct list_head list;

	struct v4l2_fh fh;
	struct vb2_queue queue;
	enum mtk_instance_state state;
	struct mtk_q_data q_data;
	int id;
	int camera_id;

	const struct mtk_camera_if *cam_if;
	bool cam_if_rdy;
	unsigned long drv_handle;

	void (*callback)(struct mtk_camera_mem *fb);

	struct v4l2_ctrl_handler ctrl_hdl;
	struct mtk_camera_ctrls		ctrls;
	bool ctrls_rdy;

	enum v4l2_colorspace colorspace;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	enum v4l2_xfer_func xfer_func;
};

struct camera_buffer {
	struct vb2_v4l2_buffer	vb;
	struct mtk_camera_mem	mem;
};

struct mtk_camera_dev {
	struct v4l2_device v4l2_dev;
	struct video_device *video;

	struct platform_device *plat_dev;
	struct platform_device *vcu_plat_dev;
	struct list_head ctx_list;
	struct mtk_camera_ctx *curr_ctx;

	unsigned long id_counter;
	int cameraid;

	struct vb2_queue *queue;

	struct mutex dev_mutex;
	struct mutex capture_mutex;
};

static inline struct mtk_camera_ctx *fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct mtk_camera_ctx, fh);
}

static inline struct mtk_camera_ctx *ctrl_to_ctx(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mtk_camera_ctx, ctrl_hdl);
}

void mtk_camera_unlock(struct mtk_camera_ctx *ctx);
void mtk_camera_lock(struct mtk_camera_ctx *ctx);

#endif /* _MTK_CAMERA_DRV_H_ */
