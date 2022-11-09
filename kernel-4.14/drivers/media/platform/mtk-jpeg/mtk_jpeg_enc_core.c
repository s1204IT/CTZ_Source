/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Xia Jiang <xia.jiang@mediatek.com>
 *         Maoguang Meng <maoguang.Meng@mediatek.com>
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


#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <soc/mediatek/smi.h>
//#include <asm/dma-iommu.h>
//#include "smi_public.h"

#include "mtk_jpeg_enc_core.h"
#include "mtk_jpeg_enc_hw.h"
//#include "mtkbuf-dma-cache-sg.h"
//#include "mtk-dma-contig.h"

#define MTK_MAX_CTRLS_HINT	20
#define DFT_CFG_WIDTH	MTK_JPEG_MIN_WIDTH
#define DFT_CFG_HEIGHT	MTK_JPEG_MIN_HEIGHT
#define OUT_FMT_IDX	3
#define CAP_FMT_IDX	0


static struct mtk_jpeg_enc_fmt mtk_jpeg_enc_formats[] = {
	{
		.name		= "JPEG JFIF",
		.fourcc		= V4L2_PIX_FMT_JPEG,
		.colplanes	= 1,
		.flags		= MTK_JPEG_FMT_FLAG_ENC_CAPTURE,
	},
	{
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.depth		= 12,
		.colplanes	= 2,
		.h_align	= 4,
		.v_align	= 4,
		.flags		= MTK_JPEG_FMT_FLAG_ENC_OUTPUT,
	},
	{
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr",
		.fourcc		= V4L2_PIX_FMT_NV21M,
		.depth		= 16,
		.colplanes	= 2,
		.h_align	= 4,
		.v_align	= 4,
		.flags		= MTK_JPEG_FMT_FLAG_ENC_OUTPUT,
	},
	{
		.name		= "YUV 4:2:2 contiguous 1-planar, YCbCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 12,
		.colplanes	= 1,
		.h_align	= 4,
		.v_align	= 3,
		.flags		= MTK_JPEG_FMT_FLAG_ENC_OUTPUT,
	},
	{
		.name		= "YUV 4:2:2 contiguous 1-planar, YCbCr",
		.fourcc		= V4L2_PIX_FMT_YVYU,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 4,
		.v_align	= 3,
		.flags		= MTK_JPEG_FMT_FLAG_ENC_OUTPUT,
	},
};
#define MTK_JPEG_ENC_NUM_FORMATS ARRAY_SIZE(mtk_jpeg_enc_formats)

enum {
	MTK_JPEG_BUF_FLAGS_INIT			= 0,
	MTK_JPEG_BUF_FLAGS_LAST_FRAME		= 1,
};

struct mtk_jpeg_enc_src_buf {
	struct mtk_jpeg_enc_param enc_param;
	struct vb2_buffer b;
	struct list_head list;
	int flags;
};

static int debug;
module_param(debug, int, 0644);

#if 0//def CONFIG_MTK_IOMMU
static int mtk_jpeg_enc_iommu_init(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;
	int err;

	np = of_parse_phandle(dev->of_node, "iommus", 0);
	if (!np) {
		dev_dbg(dev, "can't find iommus node\n");
		return 0;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		of_node_put(np);
		dev_dbg(dev, "can't find iommu device by node\n");
		return -ENODEV;
	}

	dev_dbg(dev, "%s() %s\n", __func__, dev_name(&pdev->dev));

	err = arm_iommu_attach_device(dev, pdev->dev.archdata.iommu);

	if (err) {
		of_node_put(np);
		dev_dbg(dev, "iommu_dma_attach_device fail %d\n", err);
		return -ENODEV;
	}

	return 0;
}

static void mtk_jpeg_enc_iommu_deinit(struct device *dev)
{
	arm_iommu_detach_device(dev);
}
#endif

static inline struct mtk_jpeg_enc_ctx *mtk_jpeg_enc_fh_to_ctx(
						struct v4l2_fh *fh)
{
	return container_of(fh, struct mtk_jpeg_enc_ctx, fh);
}

static int mtk_jpeg_enc_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	strlcpy(cap->driver, MTK_JPEG_ENC_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MTK_JPEG_ENC_NAME, sizeof(cap->card));
	cap->bus_info[0] = 0;
	cap->capabilities = V4L2_CAP_STREAMING |
			    V4L2_CAP_VIDEO_M2M_MPLANE;

	return 0;
}

static int vidioc_jpeg_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_jpeg_enc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct jpeg_enc_param *p = &ctx->jpeg_param;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_JPEG_RESTART_INTERVAL:
		v4l2_dbg(2, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_RESTART_INTERVAL val = %d",
			       ctrl->val);
		p->restart_interval = ctrl->val;
		break;
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		v4l2_dbg(2, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_COMPRESSION_QUALITY val = %d",
			       ctrl->val);
		p->enc_quality = ctrl->val;
		break;
	case V4L2_CID_JPEG_ACTIVE_MARKER:
		v4l2_dbg(2, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_ACTIVE_MARKER val = %d",
			       ctrl->val);
		p->enable_exif = ctrl->val;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops mtk_jpeg_enc_ctrl_ops = {
	.s_ctrl = vidioc_jpeg_enc_s_ctrl,
};


static int mtk_jpeg_enc_enum_fmt(struct mtk_jpeg_enc_fmt *mtk_jpeg_formats,
				int n,
			     struct v4l2_fmtdesc *f, u32 type)
{
	int i, num = 0;

	for (i = 0; i < n; ++i) {
		if (mtk_jpeg_formats[i].flags & type) {
			if (num == f->index)
				break;
			++num;
		}
	}

	if (i >= n)
		return -EINVAL;

	strlcpy(f->description, mtk_jpeg_formats[i].name,
				 sizeof(f->description));
	f->pixelformat = mtk_jpeg_formats[i].fourcc;

	return 0;
}

static int mtk_jpeg_enc_enum_fmt_vid_cap(struct file *file,
				void *priv,
				struct v4l2_fmtdesc *f)
{
	return mtk_jpeg_enc_enum_fmt(mtk_jpeg_enc_formats,
				MTK_JPEG_ENC_NUM_FORMATS, f,
				MTK_JPEG_FMT_FLAG_ENC_CAPTURE);
}

static int mtk_jpeg_enc_enum_fmt_vid_out(struct file *file,
				void *priv,
				struct v4l2_fmtdesc *f)
{
	return mtk_jpeg_enc_enum_fmt(mtk_jpeg_enc_formats,
				MTK_JPEG_ENC_NUM_FORMATS, f,
				MTK_JPEG_FMT_FLAG_ENC_OUTPUT);
}

static struct mtk_jpeg_enc_q_data *mtk_jpeg_enc_get_q_data(
				struct mtk_jpeg_enc_ctx *ctx,
				enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->out_q;
	else
		return &ctx->cap_q;
}

static int mtk_jpeg_enc_g_fmt_vid_mplane(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct mtk_jpeg_enc_q_data *q_data = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct mtk_jpeg_enc_ctx *ctx = mtk_jpeg_enc_fh_to_ctx(priv);
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	int i;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (!V4L2_TYPE_IS_OUTPUT(f->type) && ctx->state == MTK_JPEG_INIT)
		return -EINVAL;

	q_data = mtk_jpeg_enc_get_q_data(ctx, f->type);

	pix_mp->width = q_data->w;
	pix_mp->height = q_data->h;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->pixelformat = q_data->fmt->fourcc;
	pix_mp->num_planes = q_data->fmt->colplanes;

	v4l2_dbg(1, debug, &jpeg->v4l2_dev, "(%d) g_fmt:%s wxh:%ux%u\n",
		f->type, q_data->fmt->name,
		pix_mp->width, pix_mp->height);

	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];

		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			 "plane[%d] bpl=%u, size=%u\n",
			 i,
			 pix_mp->plane_fmt[i].bytesperline,
			 pix_mp->plane_fmt[i].sizeimage);
	}

	return 0;
}

static struct mtk_jpeg_enc_fmt *mtk_jpeg_enc_find_format(
						struct mtk_jpeg_enc_ctx *ctx,
						 u32 pixelformat,
						 unsigned int fmt_type)
{
	unsigned int k, fmt_flag;

	fmt_flag = (fmt_type == MTK_JPEG_ENC_FMT_TYPE_OUTPUT) ?
		   MTK_JPEG_FMT_FLAG_ENC_OUTPUT :
		   MTK_JPEG_FMT_FLAG_ENC_CAPTURE;

	for (k = 0; k < MTK_JPEG_ENC_NUM_FORMATS; k++) {
		struct mtk_jpeg_enc_fmt *fmt = &mtk_jpeg_enc_formats[k];

		if (fmt->fourcc == pixelformat && fmt->flags & fmt_flag)
			return fmt;
	}

	return NULL;
}

static void mtk_jpeg_bound_align_image(u32 *w, unsigned int wmin,
				       unsigned int wmax, unsigned int walign,
				       u32 *h, unsigned int hmin,
				       unsigned int hmax, unsigned int halign)
{
	int width, height, w_step, h_step;

	width = *w;
	height = *h;
	w_step = 1 << walign;
	h_step = 1 << halign;

	v4l_bound_align_image(w, wmin, wmax, walign, h, hmin, hmax, halign, 0);
	if (*w < width && (*w + w_step) <= wmax)
		*w += w_step;
	if (*h < height && (*h + h_step) <= hmax)
		*h += h_step;
}

static void mtk_jpeg_enc_adjust_fmt_mplane(struct mtk_jpeg_enc_ctx *ctx,
			struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct mtk_jpeg_enc_q_data *q_data;
	int i;

	q_data = mtk_jpeg_enc_get_q_data(ctx, f->type);

	pix_mp->width = q_data->w;
	pix_mp->height = q_data->h;
	pix_mp->pixelformat = q_data->fmt->fourcc;

	for (i = 0; i < pix_mp->num_planes; i++) {
		pix_mp->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
	}
}

static int mtk_jpeg_enc_try_fmt_mplane(struct v4l2_format *f,
	struct mtk_jpeg_enc_fmt *fmt, struct mtk_jpeg_enc_ctx *ctx, int q_type)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	int i;
	int align_w, align_h;

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->num_planes = fmt->colplanes;

	if (ctx->state != MTK_JPEG_INIT) {
		mtk_jpeg_enc_adjust_fmt_mplane(ctx, f);
		goto end;
	}

	if (q_type == MTK_JPEG_ENC_FMT_TYPE_CAPTURE) {
		mtk_jpeg_bound_align_image(&pix_mp->width, MTK_JPEG_MIN_WIDTH,
					   MTK_JPEG_MAX_WIDTH, 0,
					   &pix_mp->height, MTK_JPEG_MIN_HEIGHT,
					   MTK_JPEG_MAX_HEIGHT, 0);

		if (fmt->fourcc == V4L2_PIX_FMT_JPEG) {
			pix_mp->plane_fmt[0].sizeimage =
				mtk_jpeg_align(pix_mp->plane_fmt[0].sizeimage,
				128) + 128;
			pix_mp->plane_fmt[0].bytesperline = 0;
		}
		goto end;
	}

	/* type is MTK_JPEG_FMT_TYPE_CAPTURE */
	align_w = pix_mp->width;
	align_h = pix_mp->height;
	align_w = ((align_w + 1) >> 1) << 1;
	if (pix_mp->num_planes == 1U) {
		align_w = align_w << 1;
		mtk_jpeg_bound_align_image(&align_w, MTK_JPEG_MIN_WIDTH,
				   MTK_JPEG_MAX_WIDTH, 5,
				   &align_h, MTK_JPEG_MIN_HEIGHT,
				   MTK_JPEG_MAX_HEIGHT, 3);
		pix_mp->plane_fmt[0].bytesperline = align_w;
		pix_mp->plane_fmt[0].sizeimage =
			align_w * align_h;
	} else if (pix_mp->num_planes == 2U) {
		mtk_jpeg_bound_align_image(&align_w, MTK_JPEG_MIN_WIDTH,
				   MTK_JPEG_MAX_WIDTH, 4,
				   &align_h, MTK_JPEG_MIN_HEIGHT,
				   MTK_JPEG_MAX_HEIGHT, 4);
		pix_mp->plane_fmt[0].bytesperline = align_w;
		pix_mp->plane_fmt[0].sizeimage =
			align_w * align_h;
		pix_mp->plane_fmt[1].bytesperline = align_w;
		pix_mp->plane_fmt[1].sizeimage =
			(align_w * align_h) / 2;
	} else {
		v4l2_err(&ctx->jpeg->v4l2_dev,
				"Unsupport num planes = %d\n",
				pix_mp->num_planes);
	}
end:
	v4l2_dbg(2, debug, &jpeg->v4l2_dev,
		"(%d) try_fmt:%s wxh:%ux%u\n",
		f->type, fmt->name,
		pix_mp->width, pix_mp->height);

	for (i = 0; i < pix_mp->num_planes; i++) {
		v4l2_dbg(2, debug, &jpeg->v4l2_dev,
				"plane[%d] bpl=%u, size=%u\n",
				i,
				pix_mp->plane_fmt[i].bytesperline,
				pix_mp->plane_fmt[i].sizeimage);
	}
	return 0;
}

static int mtk_jpeg_enc_try_fmt_vid_cap_mplane(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mtk_jpeg_enc_ctx *ctx = mtk_jpeg_enc_fh_to_ctx(priv);
	struct mtk_jpeg_enc_fmt *fmt;

	fmt = mtk_jpeg_enc_find_format(ctx, f->fmt.pix_mp.pixelformat,
				   MTK_JPEG_ENC_FMT_TYPE_CAPTURE);
	if (!fmt) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
				"Fourcc format (0x%08x) invalid\n",
				f->fmt.pix_mp.pixelformat);
		return -EINVAL;
	}

	return mtk_jpeg_enc_try_fmt_mplane(f, fmt, ctx,
				MTK_JPEG_ENC_FMT_TYPE_CAPTURE);
}

static int mtk_jpeg_enc_try_fmt_vid_out_mplane(struct file *file,
				void *priv,
				struct v4l2_format *f)
{
	struct mtk_jpeg_enc_ctx *ctx = mtk_jpeg_enc_fh_to_ctx(priv);
	struct mtk_jpeg_enc_fmt *fmt;

	fmt = mtk_jpeg_enc_find_format(ctx, f->fmt.pix_mp.pixelformat,
				   MTK_JPEG_ENC_FMT_TYPE_OUTPUT);
	if (!fmt) {
		v4l2_err(&ctx->jpeg->v4l2_dev,
			 "Fourcc format (0x%08x) invalid\n",
			 f->fmt.pix_mp.pixelformat);
		return -EINVAL;
	}

	return mtk_jpeg_enc_try_fmt_mplane(f, fmt, ctx,
				MTK_JPEG_ENC_FMT_TYPE_OUTPUT);
}

static int mtk_jpeg_enc_s_fmt_mplane(struct mtk_jpeg_enc_ctx *ctx,
				struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct mtk_jpeg_enc_q_data *q_data = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	unsigned int f_type;
	int i;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = mtk_jpeg_enc_get_q_data(ctx, f->type);

	if (vb2_is_busy(vq)) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"queue busy\n");
		return -EBUSY;
	}

	f_type = V4L2_TYPE_IS_OUTPUT(f->type) ?
		MTK_JPEG_ENC_FMT_TYPE_OUTPUT :
		MTK_JPEG_ENC_FMT_TYPE_CAPTURE;

	q_data->fmt = mtk_jpeg_enc_find_format(ctx,
		pix_mp->pixelformat, f_type);
	q_data->w = pix_mp->width;
	q_data->h = pix_mp->height;

	v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		"(%d) s_fmt:%s wxh:%ux%u\n",
		f->type, q_data->fmt->name, q_data->w, q_data->h);

	for (i = 0; i < q_data->fmt->colplanes; i++) {
		q_data->bytesperline[i] = pix_mp->plane_fmt[i].bytesperline;
		q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;

		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"plane[%d] bpl=%u, size=%u\n",
			i, q_data->bytesperline[i], q_data->sizeimage[i]);
	}

	return 0;
}


static int mtk_jpeg_enc_s_fmt_vid_out_mplane(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	int ret;

	ret = mtk_jpeg_enc_try_fmt_vid_out_mplane(file, priv, f);
	if (ret)
		return ret;

	return mtk_jpeg_enc_s_fmt_mplane(mtk_jpeg_enc_fh_to_ctx(priv), f);
}

static int mtk_jpeg_enc_s_fmt_vid_cap_mplane(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	int ret;

	ret = mtk_jpeg_enc_try_fmt_vid_cap_mplane(file, priv, f);
	if (ret)
		return ret;

	return mtk_jpeg_enc_s_fmt_mplane(mtk_jpeg_enc_fh_to_ctx(priv), f);
}

static int mtk_jpeg_enc_subscribe_event(struct v4l2_fh *fh,
				    const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	default:
		return -EINVAL;
	}
}

static void mtk_jpeg_set_param(struct mtk_jpeg_enc_ctx *ctx,
				struct mtk_jpeg_enc_param *param)
{
	struct mtk_jpeg_enc_q_data *q_data_src = &ctx->out_q;
	struct jpeg_enc_param *jpeg_params = &ctx->jpeg_param;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	u32 width_even;
	u32 Is420;
	u32 padding_width;
	u32 padding_height;

	switch (q_data_src->fmt->fourcc) {
	case V4L2_PIX_FMT_YUYV:
		param->enc_format = JPEG_YUV_FORMAT_YUYV;
		break;
	case V4L2_PIX_FMT_YVYU:
		param->enc_format = JPEG_YUV_FORMAT_YVYU;
		break;
	case V4L2_PIX_FMT_NV12M:
		param->enc_format = JPEG_YUV_FORMAT_NV12;
		break;
	case V4L2_PIX_FMT_NV21M:
		param->enc_format = JPEG_YUV_FORMAT_NV12;
		break;
	default:
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Unsupport fourcc =%d\n", q_data_src->fmt->fourcc);
		break;
	}
	param->enc_w = q_data_src->w;
	param->enc_h = q_data_src->h;

	if (jpeg_params->enc_quality >= 97)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q97;
	else if (jpeg_params->enc_quality >= 95)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q95;
	else if (jpeg_params->enc_quality >= 92)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q92;
	else if (jpeg_params->enc_quality >= 90)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q90;
	else if (jpeg_params->enc_quality >= 87)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q87;
	else if (jpeg_params->enc_quality >= 84)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q84;
	else if (jpeg_params->enc_quality >= 80)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q80;
	else if (jpeg_params->enc_quality >= 74)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q74;
	else if (jpeg_params->enc_quality >= 64)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q64;
	else if (jpeg_params->enc_quality >= 60)
		param->enc_quality = JPEG_ENCODE_QUALITY_Q60;
	else
		param->enc_quality = JPEG_ENCODE_QUALITY_Q48;

	param->enable_exif = jpeg_params->enable_exif;
	param->restart_interval = jpeg_params->restart_interval;

	width_even = ((param->enc_w + 1) >> 1) << 1;
	Is420 = (param->enc_format == JPEG_YUV_FORMAT_NV12
		|| param->enc_format == JPEG_YUV_FORMAT_NV12) ? 1:0;
	padding_width = mtk_jpeg_align(param->enc_w, 16);
	padding_height = mtk_jpeg_align(param->enc_h, Is420 ? 16 : 8);
	if (!Is420)
		width_even = width_even << 1;

	param->img_stride = mtk_jpeg_align(width_even, (Is420 ? 16 : 32));
	param->mem_stride = mtk_jpeg_align(width_even, (Is420 ? 16 : 32));
	param->total_encdu = ((padding_width >> 4) *
		(padding_height >> (Is420 ? 4 : 3)) *
		(Is420 ? 6 : 4)) - 1;

	v4l2_err(&jpeg->v4l2_dev,
		"fmt %d, w,h %d,%d, exif %d, quality %d\n",
		param->enc_format, param->enc_w,
		param->enc_h, param->enable_exif,
		param->enc_quality);
	v4l2_err(&jpeg->v4l2_dev,
		"inter %d,img_s %d, mem_s %d, totalEncDu %d\n",
		param->restart_interval, param->img_stride,
		param->mem_stride, param->total_encdu);
}

static int mtk_jpeg_enc_qbuf(struct file *file,
				void *priv,
				struct v4l2_buffer *buf)
{
	struct v4l2_fh *fh = file->private_data;
	struct vb2_queue *vq;
	struct vb2_buffer *vb;
	struct mtk_jpeg_enc_src_buf *jpeg_src_buf;

	if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		goto end;

	vq = v4l2_m2m_get_vq(fh->m2m_ctx, buf->type);
	vb = vq->bufs[buf->index];

	jpeg_src_buf = container_of(vb, struct mtk_jpeg_enc_src_buf, b);
	jpeg_src_buf->flags = (buf->m.planes[0].bytesused == 0) ?
		MTK_JPEG_BUF_FLAGS_LAST_FRAME : MTK_JPEG_BUF_FLAGS_INIT;
end:
	return v4l2_m2m_qbuf(file, fh->m2m_ctx, buf);
}

static const struct v4l2_ioctl_ops mtk_jpeg_enc_ioctl_ops = {
	.vidioc_querycap                = mtk_jpeg_enc_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = mtk_jpeg_enc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out_mplane = mtk_jpeg_enc_enum_fmt_vid_out,
	.vidioc_try_fmt_vid_cap_mplane  = mtk_jpeg_enc_try_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_out_mplane  = mtk_jpeg_enc_try_fmt_vid_out_mplane,
	.vidioc_g_fmt_vid_cap_mplane    = mtk_jpeg_enc_g_fmt_vid_mplane,
	.vidioc_g_fmt_vid_out_mplane    = mtk_jpeg_enc_g_fmt_vid_mplane,
	.vidioc_s_fmt_vid_cap_mplane    = mtk_jpeg_enc_s_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_out_mplane    = mtk_jpeg_enc_s_fmt_vid_out_mplane,
	.vidioc_qbuf                    = mtk_jpeg_enc_qbuf,

	.vidioc_reqbufs                 = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_m2m_ioctl_querybuf,
	.vidioc_dqbuf                   = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf                  = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon                = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff               = v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event         = mtk_jpeg_enc_subscribe_event,
	.vidioc_unsubscribe_event       = v4l2_event_unsubscribe,
};

static int mtk_jpeg_enc_queue_setup(struct vb2_queue *q,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(q);
	struct mtk_jpeg_enc_q_data *q_data = NULL;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	int i;

	q_data = mtk_jpeg_enc_get_q_data(ctx, q->type);
	if (q_data == NULL)
		return -EINVAL;

	*num_planes = q_data->fmt->colplanes;

	for (i = 0; i < q_data->fmt->colplanes; i++) {
		sizes[i] = q_data->sizeimage[i];
		if (jpeg->alloc_ctx)
			alloc_ctxs[i] = jpeg->alloc_ctx;
	}

	return 0;
}

static int mtk_jpeg_enc_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_jpeg_enc_q_data *q_data = NULL;
	int i;

	q_data = mtk_jpeg_enc_get_q_data(ctx, vb->vb2_queue->type);
	if (q_data == NULL)
		return -EINVAL;

	for (i = 0; i < q_data->fmt->colplanes; i++)
		vb2_set_plane_payload(vb, i, q_data->sizeimage[i]);

	return 0;
}

static void mtk_jpeg_enc_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	struct mtk_jpeg_enc_src_buf *jpeg_src_buf;
	#if MTK_JPEG_BENCHMARK
	struct timeval begin, end;
	#endif

	jpeg_src_buf = container_of(vb, struct mtk_jpeg_enc_src_buf, b);

	v4l2_dbg(2, debug, &jpeg->v4l2_dev,
		"(%d) buf_q vb=%p",
		vb->vb2_queue->type, vb);

	if (vb->vb2_queue->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		goto end;

	if (jpeg_src_buf->flags & MTK_JPEG_BUF_FLAGS_LAST_FRAME) {
		v4l2_err(&jpeg->v4l2_dev, "Got eos");
		goto end;
	}
#if MTK_JPEG_BENCHMARK
	do_gettimeofday(&begin);
#endif
#if MTK_JPEG_BENCHMARK
	do_gettimeofday(&end);
	ctx->total_parse_cnt++;
	ctx->total_parse_time +=
			((end.tv_sec - begin.tv_sec) * 1000000 +
			end.tv_usec - begin.tv_usec);
#endif

	if (ctx->state == MTK_JPEG_INIT)
		ctx->state = MTK_JPEG_RUNNING;
end:
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void *mtk_jpeg_enc_buf_remove(struct mtk_jpeg_enc_ctx *ctx,
				 enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	else
		return v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
}

static void mtk_jpeg_enc_clk_on(struct mtk_jpeg_enc_dev *jpeg)
{
	int ret;
#if 0
	ret = mtk_smi_larb_get(jpeg->larb);
	ret = smi_bus_prepare_enable(SMI_LARB3_REG_INDX, "jpegenc", true);
#endif
	ret = clk_prepare_enable(jpeg->clk_venc_jenc);
	if (ret)
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		"mtk_smi_larb_get larbvdec fail %d\n", ret);
}

static void mtk_jpeg_enc_clk_off(struct mtk_jpeg_enc_dev *jpeg)
{
	clk_disable_unprepare(jpeg->clk_venc_jenc);
	//mtk_smi_larb_put(jpeg->larb);
	//smi_bus_disable_unprepare(SMI_LARB3_REG_INDX, "jpegenc", true);
}

static int mtk_jpeg_enc_start_streaming(struct vb2_queue *q,
				unsigned int count)
{
	//struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(q);
	int ret = 0;
    //mtk_jpeg_enc_clk_on(ctx->jpeg);

	//ret = pm_runtime_get_sync(ctx->jpeg->dev);
	//v4l2_dbg(2,debug,&ctx->jpeg->v4l2_dev,"%s(%d)",__func__,__LINE__);
	return ret > 0 ? 0 : ret;
}

static void mtk_jpeg_enc_stop_streaming(struct vb2_queue *q)
{
	struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_buffer *vb;

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->state = MTK_JPEG_INIT;

	vb = mtk_jpeg_enc_buf_remove(ctx, q->type);
	while (vb != NULL) {
		v4l2_m2m_buf_done(to_vb2_v4l2_buffer(vb), VB2_BUF_STATE_ERROR);
		vb = mtk_jpeg_enc_buf_remove(ctx, q->type);
	}
	//pm_runtime_put_sync(ctx->jpeg->dev);
}

static void mtk_jpeg_enc_buf_finish(struct vb2_buffer *vb)
{
//	struct mtk_jpeg_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
//	struct mtk_jpeg_enc_src_buf *mtkbuf;
//	struct vb2_v4l2_buffer *vb2_v4l2;
}

static struct vb2_ops mtk_jpeg_enc_qops = {
	.queue_setup        = mtk_jpeg_enc_queue_setup,
	.buf_prepare        = mtk_jpeg_enc_buf_prepare,
	.buf_queue          = mtk_jpeg_enc_buf_queue,
	.wait_prepare       = vb2_ops_wait_prepare,
	.wait_finish        = vb2_ops_wait_finish,
	.buf_finish         = mtk_jpeg_enc_buf_finish,
	.start_streaming    = mtk_jpeg_enc_start_streaming,
	.stop_streaming     = mtk_jpeg_enc_stop_streaming,
};

static void mtk_jpeg_set_enc_dst(struct mtk_jpeg_enc_ctx *ctx,
				 struct vb2_buffer *dst_buf,
				 struct mtk_jpeg_bs *bs)
{
	bs->dma_addr =  vb2_dma_contig_plane_dma_addr(dst_buf, 0)
					& (~JPEG_ENC_DST_ADDR_OFFSET_MASK);
	bs->dma_addr_offset = 0;
	bs->dma_addr_offsetmask = bs->dma_addr & JPEG_ENC_DST_ADDR_OFFSET_MASK;
	bs->size = mtk_jpeg_align(vb2_plane_size(dst_buf, 0), 128);
}

static int mtk_jpeg_set_enc_src(struct mtk_jpeg_enc_ctx *ctx,
				struct vb2_buffer *src_buf,
				struct mtk_jpeg_fb *fb)
{
	int i;

	v4l2_dbg(1, debug, &ctx->jpeg->v4l2_dev,
		"idx:%d,type:%d,fd:%d %d,%d\n",
		src_buf->index, src_buf->type, src_buf->planes[0].m.fd,
		src_buf->planes[0].bytesused, src_buf->planes[0].length);

	for (i = 0; i < src_buf->num_planes; i++)
		fb->fb_addr[i].dma_addr =
			vb2_dma_contig_plane_dma_addr(src_buf, i);

	return 0;
}

static void mtk_jpeg_enc_device_run(void *priv)
{
	struct mtk_jpeg_enc_ctx *ctx = priv;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;
	struct vb2_buffer *src_buf, *dst_buf;
	enum vb2_buffer_state buf_state = VB2_BUF_STATE_ERROR;
	unsigned long flags;
	struct mtk_jpeg_enc_param *param;
	struct mtk_jpeg_enc_src_buf *jpeg_src_buf;
	struct mtk_jpeg_bs bs;
	struct mtk_jpeg_fb fb;
	int i;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	jpeg_src_buf = container_of(src_buf, struct mtk_jpeg_enc_src_buf, b);

	param = &jpeg_src_buf->enc_param;
	memset(param, 0, sizeof(*param));
	mtk_jpeg_set_param(ctx, param);

	if (jpeg_src_buf->flags & MTK_JPEG_BUF_FLAGS_LAST_FRAME) {
		v4l2_err(&jpeg->v4l2_dev, "MTK_JPEG_BUF_FLAGS_LAST_FRAME");
		for (i = 0; i < dst_buf->num_planes; i++)
			vb2_set_plane_payload(dst_buf, i, 0);
		buf_state = VB2_BUF_STATE_DONE;
		goto enc_end;
	}
	mtk_jpeg_enc_clk_on(jpeg);
	v4l2_err(&jpeg->v4l2_dev,
		"idx:%d,type:%d,fd:%d %d,%d\n",
		src_buf->index, src_buf->type,
		src_buf->planes[0].m.fd,
		src_buf->planes[0].bytesused, src_buf->planes[0].length);
	v4l2_err(&jpeg->v4l2_dev,
		"device_run fmt %d, w,h %d,%d, enable_exif %d,\n",
		jpeg_src_buf->enc_param.enc_format,
		jpeg_src_buf->enc_param.enc_w,
		jpeg_src_buf->enc_param.enc_h,
		jpeg_src_buf->enc_param.enable_exif);
	v4l2_err(&jpeg->v4l2_dev,
		"quali %d , inter %d,img_s %d, mem_s %d, totalEncDu %d\n",
		jpeg_src_buf->enc_param.enc_quality,
		jpeg_src_buf->enc_param.restart_interval,
		jpeg_src_buf->enc_param.img_stride,
		jpeg_src_buf->enc_param.mem_stride,
		jpeg_src_buf->enc_param.total_encdu);

	mtk_jpeg_set_enc_dst(ctx, dst_buf, &bs);
	mtk_jpeg_set_enc_src(ctx, src_buf, &fb);

#if MTK_JPEG_BENCHMARK
	do_gettimeofday(&ctx->jpeg_hw_sta);
	ctx->total_enc_dec_cnt++;
#endif
	spin_lock_irqsave(&jpeg->irq_lock, flags);
	mtk_jpeg_enc_reset(jpeg->enc_reg_base);
	mtk_jpeg_enc_set_config(jpeg->enc_reg_base,
				&jpeg_src_buf->enc_param, &bs, &fb);

	mtk_jpeg_enc_start(jpeg->enc_reg_base);
	spin_unlock_irqrestore(&jpeg->irq_lock, flags);
	return;

enc_end:
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(src_buf), buf_state);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(dst_buf), buf_state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, ctx->fh.m2m_ctx);
}

static int mtk_jpeg_enc_job_ready(void *priv)
{
	struct mtk_jpeg_enc_ctx *ctx = priv;

	return (ctx->state == MTK_JPEG_RUNNING) ? 1 : 0;
}

static void mtk_jpeg_enc_job_abort(void *priv)
{
}

static struct v4l2_m2m_ops mtk_jpeg_enc_m2m_ops = {
	.device_run = mtk_jpeg_enc_device_run,
	.job_ready  = mtk_jpeg_enc_job_ready,
	.job_abort  = mtk_jpeg_enc_job_abort,
};

void mtk_jpeg_enc_set_default_params(struct mtk_jpeg_enc_ctx *ctx)
{
	struct mtk_jpeg_enc_q_data *q_data;
	u32 align_w, align_h;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;

	q_data = &ctx->out_q;
	memset(q_data, 0, sizeof(struct mtk_jpeg_enc_q_data));
	q_data->w = DFT_CFG_WIDTH;
	q_data->h = DFT_CFG_HEIGHT;

	q_data->fmt = &mtk_jpeg_enc_formats[OUT_FMT_IDX];

	align_w = q_data->w;
	align_h = q_data->h;
	align_w = ((align_w+1)>>1) << 1;
	align_w = align_w << 1;
	v4l_bound_align_image(&align_w,
				MTK_JPEG_MIN_WIDTH,
				MTK_JPEG_MAX_WIDTH, 5,
				&align_h,
				MTK_JPEG_MIN_HEIGHT,
				MTK_JPEG_MAX_HEIGHT, 3, 0);

	if (align_w < DFT_CFG_WIDTH &&
		(align_w + 32) <= MTK_JPEG_MAX_WIDTH)
		align_w += 32;
	if (align_h < DFT_CFG_HEIGHT &&
		(align_h + 8) <= MTK_JPEG_MAX_HEIGHT)
		align_h += 8;

	q_data->sizeimage[0] = align_w * align_h;
	q_data->bytesperline[0] = align_w;

	q_data = &ctx->cap_q;
	memset(q_data, 0, sizeof(struct mtk_jpeg_enc_q_data));
	q_data->w = DFT_CFG_WIDTH;
	q_data->h = DFT_CFG_HEIGHT;
	q_data->fmt = &mtk_jpeg_enc_formats[CAP_FMT_IDX];
	ctx->cap_q.sizeimage[0] =
		DFT_CFG_WIDTH * DFT_CFG_HEIGHT * 4;
	ctx->cap_q.bytesperline[0] = 0;
}

int mtk_jpeg_enc_ctrls_setup(struct mtk_jpeg_enc_ctx *ctx)
{
	const struct v4l2_ctrl_ops *ops = &mtk_jpeg_enc_ctrl_ops;
	struct v4l2_ctrl_handler *handler = &ctx->ctrl_hdl;
	struct mtk_jpeg_enc_dev *jpeg = ctx->jpeg;

	v4l2_ctrl_handler_init(handler, MTK_MAX_CTRLS_HINT);

	v4l2_ctrl_new_std(handler, ops, V4L2_CID_JPEG_RESTART_INTERVAL,
			0, 100, 1, 0);
	if (handler->error) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_RESTART_INTERVAL Init control handler fail %d\n",
		handler->error);
		return handler->error;
	}
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_JPEG_COMPRESSION_QUALITY,
			48, 100, 1, 90);
	if (handler->error) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_COMPRESSION_QUALITY Init control handler fail %d\n",
		handler->error);
		return handler->error;
	}
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_JPEG_ENABLE_EXIF,
			0, 1, 1, 0);
	if (handler->error) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"V4L2_CID_JPEG_ACTIVE_MARKER Init control handler fail %d\n",
			handler->error);
		return handler->error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);

	return 0;
}

static int mtk_jpeg_enc_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct mtk_jpeg_enc_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_DMABUF | VB2_MMAP;// | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct mtk_jpeg_enc_src_buf);
	src_vq->ops = &mtk_jpeg_enc_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->jpeg->lock;
	src_vq->dev = &ctx->jpeg->plat_dev->dev;

	src_vq->allow_zero_bytesused = 1;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_DMABUF | VB2_MMAP;// | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &mtk_jpeg_enc_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->jpeg->lock;
	dst_vq->dev = &ctx->jpeg->plat_dev->dev;
	dst_vq->allow_zero_bytesused = 1;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		vb2_queue_release(src_vq);

	return ret;
}

static irqreturn_t mtk_jpeg_enc_irq(int irq, void *priv)
{
	struct mtk_jpeg_enc_dev *jpeg = priv;
	struct mtk_jpeg_enc_ctx *ctx;
	struct vb2_buffer *src_buf, *dst_buf;
	struct mtk_jpeg_enc_src_buf *jpeg_src_buf;
	enum vb2_buffer_state buf_state = VB2_BUF_STATE_ERROR;
	u32 enc_ret, enc_size;

	ctx = v4l2_m2m_get_curr_priv(jpeg->m2m_dev);
	if (ctx == NULL) {
		v4l2_err(&jpeg->v4l2_dev, "Context is NULL\n");
		return IRQ_HANDLED;
	}
#if MTK_JPEG_BENCHMARK
	do_gettimeofday(&ctx->jpeg_hw_end);
	ctx->total_enc_dec_time +=
		((ctx->jpeg_hw_end.tv_sec - ctx->jpeg_hw_sta.tv_sec) * 1000000
		+ ctx->jpeg_hw_end.tv_usec - ctx->jpeg_hw_sta.tv_usec);
#endif

	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	jpeg_src_buf = container_of(src_buf, struct mtk_jpeg_enc_src_buf, b);

	enc_ret = mtk_jpeg_enc_get_int_status(jpeg->enc_reg_base);
	ctx->enc_irq_ret = mtk_jpeg_enc_enum_result(jpeg->enc_reg_base,
				enc_ret,
				&enc_size); //for dst size

	if (ctx->enc_irq_ret >= MTK_JPEG_ENC_RESULT_STALL)
		mtk_jpeg_enc_reset(jpeg->enc_reg_base);

	if (ctx->enc_irq_ret != MTK_JPEG_ENC_RESULT_DONE) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev, "encode failed\n");
		goto enc_end;
	}

	v4l2_err(&jpeg->v4l2_dev, "enc_size:%d", enc_size);
	vb2_set_plane_payload(dst_buf, 0, enc_size); // for dst sie

	buf_state = VB2_BUF_STATE_DONE;
	mtk_jpeg_enc_clk_off(jpeg);
enc_end:
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(src_buf), buf_state);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(dst_buf), buf_state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, ctx->fh.m2m_ctx);
	return IRQ_HANDLED;
}

static int mtk_jpeg_enc_open(struct file *file)
{
	struct mtk_jpeg_enc_dev *jpeg = video_drvdata(file);
	struct video_device *vfd = video_devdata(file);
	struct mtk_jpeg_enc_ctx *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&jpeg->lock)) {
		ret = -ERESTARTSYS;
		goto free;
	}

	v4l2_fh_init(&ctx->fh, vfd);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ret = mtk_jpeg_enc_ctrls_setup(ctx);
		if (ret) {
			v4l2_dbg(1, debug, &jpeg->v4l2_dev,
				"Failed to setup controls() (%d)\n", ret);
			goto error;
		}
	ctx->jpeg = jpeg;
	ctx->out_q.fmt = mtk_jpeg_enc_find_format(ctx, V4L2_PIX_FMT_YUYV,
					      MTK_JPEG_ENC_FMT_TYPE_OUTPUT);
	ctx->cap_q.fmt = mtk_jpeg_enc_find_format(ctx, V4L2_PIX_FMT_JPEG,
					      MTK_JPEG_ENC_FMT_TYPE_CAPTURE);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jpeg->m2m_dev, ctx,
					    mtk_jpeg_enc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto error;
	}

	mtk_jpeg_enc_set_default_params(ctx);

	ret = pm_runtime_get_sync(jpeg->dev);
	if (ret)
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		"%s power domain fail %d", __func__, ret);
	ret = pm_runtime_get_sync(jpeg->larb);
	if (ret)
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		"%s larbvenc fail %d", __func__, ret);

	mutex_unlock(&jpeg->lock);
#if MTK_JPEG_BENCHMARK
	do_gettimeofday(&ctx->jpeg_enc_dec_start);
#endif
	//mtk_jpeg_enc_clk_on(ctx->jpeg);

	return 0;

error:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&jpeg->lock);
free:
	kfree(ctx);
	return ret;
}

static int mtk_jpeg_enc_release(struct file *file)
{
	struct mtk_jpeg_enc_dev *jpeg = video_drvdata(file);
	struct mtk_jpeg_enc_ctx *ctx =
		mtk_jpeg_enc_fh_to_ctx(file->private_data);

#if MTK_JPEG_BENCHMARK
	struct timeval end;
	uint32_t total_time;

	do_gettimeofday(&end);
	total_time = (end.tv_sec - ctx->jpeg_enc_dec_start.tv_sec) * 1000000 +
		 end.tv_usec - ctx->jpeg_enc_dec_start.tv_usec;
	v4l2_err(&jpeg->v4l2_dev, "\n\nMTK_JPEG_BENCHMARK");
	v4l2_err(&jpeg->v4l2_dev, "  total_enc_dec_cnt: %u ",
				ctx->total_enc_dec_cnt);
	v4l2_err(&jpeg->v4l2_dev, "  total_enc_dec_time: %u us",
				ctx->total_enc_dec_time);
	v4l2_err(&jpeg->v4l2_dev, "  total_parse_cnt: %u ",
				ctx->total_parse_cnt);
	v4l2_err(&jpeg->v4l2_dev, "  total_parse_time: %u us",
				ctx->total_parse_time);
	v4l2_err(&jpeg->v4l2_dev, "  total_time: %u us",
		total_time);
	if (ctx->total_enc_dec_cnt) {
		v4l2_err(&jpeg->v4l2_dev, "  dec fps: %u",
			1000000 /
			(ctx->total_enc_dec_time / ctx->total_enc_dec_cnt));
		v4l2_err(&jpeg->v4l2_dev, "  avg fps: %u",
			1000000 /
			(total_time / ctx->total_enc_dec_cnt));
	}
#endif
	mutex_lock(&jpeg->lock);

	pm_runtime_put_sync(jpeg->larb);
	pm_runtime_put_sync(jpeg->dev);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	mutex_unlock(&jpeg->lock);
	return 0;
}

static const struct v4l2_file_operations mtk_jpeg_enc_fops = {
	.owner          = THIS_MODULE,
	.open           = mtk_jpeg_enc_open,
	.release        = mtk_jpeg_enc_release,
	.poll           = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap           = v4l2_m2m_fop_mmap,
};

static int mtk_jpeg_enc_clk_init(struct mtk_jpeg_enc_dev *jpeg)
{
	struct device_node *node;
	//struct platform_device *pdev;
	struct platform_device *pJpeg = NULL;
#if 0
	node = of_parse_phandle(jpeg->dev->of_node, "mediatek,larb", 0);
	if (!node)
		return -EINVAL;
	pdev = of_find_device_by_node(node);
	if (WARN_ON(!pdev)) {
		of_node_put(node);
		return -EINVAL;
	}
	jpeg->larb = &pdev->dev;

	jpeg->clk_venc_jenc = devm_clk_get(jpeg->dev, "MT_CG_VENC_JPGENC");
	if (jpeg->clk_venc_jenc == NULL)
		return -EINVAL;
#else
	node = of_find_compatible_node(NULL, NULL, "mediatek,venc_gcon");

	pJpeg = of_find_device_by_node(node);
	if (WARN_ON(!pJpeg)) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev, "pJpeg clk error!");
		of_node_put(node);
		return -EINVAL;
	}
	jpeg->clk_venc_jenc =
		devm_clk_get(&pJpeg->dev, "MT_CG_VENC_JPGENC");
	if (IS_ERR(jpeg->clk_venc_jenc))
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		"get MT_CG_VENC_JPGENC clk error!");

	node = of_parse_phandle(pJpeg->dev.of_node, "mediatek,larb", 0);
	if (!node)
		return -EINVAL;
	pJpeg = of_find_device_by_node(node);
	if (WARN_ON(!pJpeg)) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"mediatek,larb clk error!");
		of_node_put(node);
		return -EINVAL;
	}
	jpeg->larb = &pJpeg->dev;

	//pm_runtime_enable(&pdev->dev);
#endif
	pm_runtime_enable(&jpeg->plat_dev->dev);//yinbo

	return 0;
}

static int mtk_jpeg_enc_probe(struct platform_device *pdev)
{
	struct mtk_jpeg_enc_dev *jpeg;
	struct resource *res;
	u32 enc_irq;
	int ret;

	jpeg = devm_kzalloc(&pdev->dev, sizeof(*jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;

	mutex_init(&jpeg->lock);
	mutex_init(&jpeg->dev_lock);
	spin_lock_init(&jpeg->irq_lock);
	jpeg->dev = &pdev->dev;
	jpeg->plat_dev = pdev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jpeg->enc_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jpeg->enc_reg_base)) {
		ret = PTR_ERR(jpeg->enc_reg_base);
		v4l2_dbg(1, debug, &jpeg->v4l2_dev, "devm_ioremap_resource failed.\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	enc_irq = platform_get_irq(pdev, 0);
	if (res == NULL || enc_irq < 0) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to get enc_irq %d.\n", enc_irq);
		ret = -EINVAL;
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, enc_irq, mtk_jpeg_enc_irq, 0,
			       pdev->name, jpeg);
	if (ret) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to request enc_irq %d (%d)\n",
			enc_irq, ret);
		ret = -EINVAL;
		goto err_req_irq;
	}

	ret = mtk_jpeg_enc_clk_init(jpeg);
	if (ret) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to init clk, err %d\n", ret);
		ret = -EINVAL;
		goto err_clk_init;
	}

	ret = v4l2_device_register(&pdev->dev, &jpeg->v4l2_dev);
	if (ret) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to register v4l2 device\n");
		ret = -EINVAL;
		goto err_dev_register;
	}

#if 0
	jpeg->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(jpeg->alloc_ctx)) {
		jpeg->alloc_ctx = NULL;
		ret = PTR_ERR(jpeg->alloc_ctx);
		goto err_vb2_ctx_init;
	}
#endif
	jpeg->m2m_dev = v4l2_m2m_init(&mtk_jpeg_enc_m2m_ops);
	if (IS_ERR(jpeg->m2m_dev)) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to init mem2mem device\n");
		ret = PTR_ERR(jpeg->m2m_dev);
		goto err_m2m_init;
	}

	#if 1
	//jpeg->alloc_ctx =
	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));
	if (IS_ERR(jpeg->alloc_ctx)) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to init memory allocator\n");
		ret = PTR_ERR(jpeg->alloc_ctx);
		goto err_alloc_ctx;
	}
	#endif
	jpeg->enc_vdev = video_device_alloc();
	if (!jpeg->enc_vdev) {
		ret = -ENOMEM;
		goto err_alloc_ctx;
	}
	snprintf(jpeg->enc_vdev->name, sizeof(jpeg->enc_vdev->name),
		 "%s", MTK_JPEG_ENC_NAME);
	jpeg->enc_vdev->fops = &mtk_jpeg_enc_fops;
	jpeg->enc_vdev->ioctl_ops = &mtk_jpeg_enc_ioctl_ops;
	jpeg->enc_vdev->minor = -1;
	jpeg->enc_vdev->release = video_device_release;
	jpeg->enc_vdev->lock = &jpeg->lock;
	jpeg->enc_vdev->v4l2_dev = &jpeg->v4l2_dev;
	jpeg->enc_vdev->vfl_dir = VFL_DIR_M2M;

	ret = video_register_device(jpeg->enc_vdev, VFL_TYPE_GRABBER, 4);
	if (ret) {
		v4l2_dbg(1, debug, &jpeg->v4l2_dev,
			"Failed to register video device\n");
		goto err_enc_vdev_register;
	}

#if 0//def CONFIG_MTK_IOMMU
	ret = mtk_jpeg_enc_iommu_init(&pdev->dev);
	if (ret) {
		v4l2_info(&jpeg->v4l2_dev,
			 "Failed to attach iommu device err = %d\n", ret);
		goto err_enc_vdev_register;
	}
#endif

	video_set_drvdata(jpeg->enc_vdev, jpeg);
	platform_set_drvdata(pdev, jpeg);
	pm_runtime_enable(&pdev->dev);
	v4l2_dbg(1, debug, &jpeg->v4l2_dev,
		  "jpeg encoder device registered as /dev/video%d\n",
		  jpeg->enc_vdev->num);

	return 0;

err_enc_vdev_register:
	video_device_release(jpeg->enc_vdev);
#if 0

err_enc_vdev_alloc:
	vb2_dma_contig_clear_max_seg_size(jpeg->alloc_ctx);
//#endif
err_vb2_ctx_init:
	video_unregister_device(jpeg->enc_vdev);
#endif
err_alloc_ctx:
	v4l2_m2m_release(jpeg->m2m_dev);

err_m2m_init:
	v4l2_device_unregister(&jpeg->v4l2_dev);

err_dev_register:

err_clk_init:

err_req_irq:

	return ret;
}

static int mtk_jpeg_enc_remove(struct platform_device *pdev)
{
	struct mtk_jpeg_enc_dev *jpeg = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
#if 0//def CONFIG_MTK_IOMMU
	mtk_jpeg_enc_iommu_deinit(&pdev->dev);
#endif
	video_unregister_device(jpeg->enc_vdev);
	video_device_release(jpeg->enc_vdev);
	//vb2_dma_contig_clear_max_seg_size(jpeg->alloc_ctx);
	v4l2_m2m_release(jpeg->m2m_dev);
	v4l2_device_unregister(&jpeg->v4l2_dev);

	return 0;
}

#if 0
static int mtk_jpeg_enc_pm_suspend(struct device *dev)
{
	struct mtk_jpeg_enc_dev *jpeg = dev_get_drvdata(dev);
	//mtk_jpeg_enc_reset(jpeg->enc_reg_base);
	//mtk_jpeg_enc_clk_off(jpeg);
	return 0;
}

static int mtk_jpeg_enc_pm_resume(struct device *dev)
{
	struct mtk_jpeg_enc_dev *jpeg = dev_get_drvdata(dev);
	//mtk_jpeg_enc_clk_on(jpeg);
	//mtk_jpeg_enc_reset(jpeg->enc_reg_base);
	return 0;
}

static int mtk_jpeg_enc_suspend(struct device *dev)
{
	int ret;

	if (pm_runtime_suspended(dev))
		return 0;

	ret = mtk_jpeg_enc_pm_suspend(dev);
	return ret;
}

static int mtk_jpeg_enc_resume(struct device *dev)
{
	int ret;

	if (pm_runtime_suspended(dev))
		return 0;

	ret = mtk_jpeg_enc_pm_resume(dev);

	return ret;
}

static const struct dev_pm_ops mtk_jpeg_enc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mtk_jpeg_enc_suspend, mtk_jpeg_enc_resume)
	SET_RUNTIME_PM_OPS(mtk_jpeg_enc_pm_suspend,
	mtk_jpeg_enc_pm_resume, NULL)
};
#endif
static const struct of_device_id mtk_jpeg_enc_match[] = {
	{
		.compatible = "mediatek,mt2701-jpgenc",
		.data       = NULL,
	},
	{
		.compatible = "mediatek,mt8168-jpgenc",
		.data       = NULL,
	},
	{
		.compatible = "mediatek,jpgenc",
		.data       = NULL,
	},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_jpeg_match);

static struct platform_driver mtk_jpeg_enc_driver = {
	.probe = mtk_jpeg_enc_probe,
	.remove = mtk_jpeg_enc_remove,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = MTK_JPEG_ENC_NAME,
		.of_match_table = mtk_jpeg_enc_match,
//		.pm             = &mtk_jpeg_enc_pm_ops,
	},
};

module_platform_driver(mtk_jpeg_enc_driver);

MODULE_DESCRIPTION("MediaTek JPEG enc codec driver");
MODULE_LICENSE("GPL v2");
