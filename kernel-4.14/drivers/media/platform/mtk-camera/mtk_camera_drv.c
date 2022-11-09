// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_camera_drv.h"
#include "mtk_camera_util.h"
#include "mtk_camera_if.h"
#include "mtk_vcu.h"

module_param_named(debug, mtk_camera_dbg_level, int, 0644);

#define MTK_VIDEO_CAPTURE_DEF_FORMAT        V4L2_PIX_FMT_YVU420
#define MTK_VIDEO_CAPTURE_DEF_WIDTH         1920U
#define MTK_VIDEO_CAPTURE_DEF_HEIGHT        1080U

#define MTK_VIDEO_CAPTURE_MIN_WIDTH         2U
#define MTK_VIDEO_CAPTURE_MAX_WIDTH         8190U
#define MTK_VIDEO_CAPTURE_MIN_HEIGHT        2U
#define MTK_VIDEO_CAPTURE_MAX_HEIGHT        8190U

static struct mtk_camera_fmt mtk_camera_formats[] = {
	{
		.name   = "YUYV",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.bpp	= {16, 0, 0},
		.mplane = false,
		.num_planes = 1,
	},

	{
		.name   = "YV12",
		.fourcc = V4L2_PIX_FMT_YVU420,
		.bpp	= {8, 2, 2},
		.mplane = true,
		.num_planes = 3,
	},

	{
		.name	= "NV12",
		.fourcc = V4L2_PIX_FMT_NV12,
		.bpp	= {8, 4, 0},
		.mplane = true,
		.num_planes = 2,
	},

};

/* Sizes must be in increasing order */
static struct v4l2_frmsize_discrete mtk_camera_sizes[] = {
	{ 1280, 720  },
	{ 1920, 1080 },
	{ 2560, 1920 },
	{ 2688, 1944 },
	{ 3840, 2160 },
};
/* -----------------------------------------------------------------------------
 * Video queue operations
 */
const char *getCameraNameFromId(int camera_id)
{
	switch (camera_id) {
	case 0:
		return "mtkcam-main";

	case 1:
		return "mtkcam-sub";

	default:
		break;
	}
	mtk_camera_err("unknown camera_id:%d", camera_id);
	return "unknown";
}
static int vb2ops_camera_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vq);
	struct mtk_q_data *q_data = &ctx->q_data;
	int i;

	if (q_data == NULL) {
		mtk_camera_err("q_data is NULL");
		return -EINVAL;
	}

	if (*nplanes) {
		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] < q_data->bytesperline[i] * q_data->height)
				return -EINVAL;
		}
	} else {
		if (q_data->fmt->mplane) {
			*nplanes = q_data->fmt->num_planes;
			for (i = 0; i < *nplanes; i++) {
				sizes[i] =
				q_data->bytesperline[i] * q_data->height;
			}
		} else {
			*nplanes = 1;
			sizes[0] = q_data->sizeimage[0];
		}
	}

	*nbuffers = max_t(unsigned int, *nbuffers, MTK_CAMERA_MIN_BUFFERS);
	vq->min_buffers_needed = *nbuffers;

	mtk_camera_debug(1,
			"%s:[%d]\t type = %d, get %d plane(s), %d buffer(s) of size %d %d",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vq->type, *nplanes, *nbuffers,
			sizes[0], sizes[1]);
	return 0;
}

static int vb2ops_camera_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	mtk_camera_debug(1, "%s:[%d] (%d) id=%d, state=%d, vb=%p",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vb->vb2_queue->type,
			vb->index, vb->state, vb);

	return 0;
}

static void vb2ops_camera_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vb2_v4l2 = container_of(vb,
				struct vb2_v4l2_buffer, vb2_buf);
	struct camera_buffer *buffer = container_of(vb2_v4l2,
				struct camera_buffer, vb);
	struct mtk_camera_mem *fb = &buffer->mem;
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int i, ret = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) id=%d, state=%d, vb=%p",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vb->vb2_queue->type,
			vb->index, vb->state, vb);

	fb->size = 0;
	fb->index = vb->index;
	fb->num_planes = vb->num_planes;
	for (i = 0; i < vb->num_planes; i++) {
		fb->planes[i].dma_addr = vb2_dma_contig_plane_dma_addr(vb, i);
		fb->planes[i].size = vb2_plane_size(vb, i);
		fb->size += fb->planes[i].size;
		mtk_camera_debug(1, "plane %d, dma_addr 0x%llx, size %lu",
				i, (uint64_t)fb->planes[i].dma_addr,
				fb->planes[i].size);
	}
	mtk_camera_debug(1, "total size %lu", fb->size);

	ret = camera_if_capture(ctx, fb);
	if (ret) {
		mtk_camera_err("%s:[%d]: camera_if_capture() fail ret=%d",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ret);
	}
}

static int vb2ops_camera_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vb2_v4l2 = container_of(vb,
				struct vb2_v4l2_buffer, vb2_buf);
	struct camera_buffer *buffer = container_of(vb2_v4l2,
				struct camera_buffer, vb);
	struct mtk_camera_mem *fb = &buffer->mem;
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_q_data *q_data = &ctx->q_data;
	int i, ret = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) id=%d, state=%d, vb=%p vb->num_planes:%d",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vb->vb2_queue->type,
			vb->index, vb->state, vb, vb->num_planes);

	fb->size = 0;
	fb->index = vb->index;
	fb->format = q_data->fmt->fourcc;
	fb->num_planes = vb->num_planes;
	for (i = 0; i < vb->num_planes; i++) {
		fb->planes[i].dma_addr = vb2_dma_contig_plane_dma_addr(vb, i);
		fb->planes[i].size = vb2_plane_size(vb, i);
		fb->size += fb->planes[i].size;
		mtk_camera_debug(1, "plane %d, dma_addr 0x%llx, size %lu",
				i, (uint64_t)fb->planes[i].dma_addr,
				fb->planes[i].size);
	}
	mtk_camera_debug(1, "total size %lu", fb->size);

	if (ctx->state == MTK_STATE_INIT) {
		ret = camera_if_init_buffer(ctx, fb);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_use_buffer() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return -EINVAL;
		}
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);

	return 0;
}
static void vb2ops_camera_buf_deinit(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vb2_v4l2 = container_of(vb,
				struct vb2_v4l2_buffer, vb2_buf);
	struct camera_buffer *buffer = container_of(vb2_v4l2,
				struct camera_buffer, vb);
	struct mtk_camera_mem *fb = &buffer->mem;
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int ret = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) id=%d, state=%d, vb=%p",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vb->vb2_queue->type,
			vb->index, vb->state, vb);

	if (ctx->state == MTK_STATE_INIT) {
		ret = camera_if_deinit_buffer(ctx, fb);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_use_buffer() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return;
		}
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);
}

static int vb2ops_camera_start_streaming(
	struct vb2_queue *q, unsigned int count)
{
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(q);
	int ret = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) state=(%x)",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, q->type, ctx->state);

	if (ctx->state == MTK_STATE_INIT) {
		ret = camera_if_start_stream(ctx);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_start_stream() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return -EINVAL;
		}
		ctx->state = MTK_STATE_START;
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);

	return 0;
}

static void vb2ops_camera_stop_streaming(struct vb2_queue *q)
{
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(q);
	int ret = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) state=(%x)",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, q->type, ctx->state);

	if (ctx->state == MTK_STATE_START) {
		ret = camera_if_stop_stream(ctx);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_stop_stream() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return;
		}
		ctx->state = MTK_STATE_FLUSH;

		vb2_wait_for_all_buffers(q);
		ctx->state = MTK_STATE_INIT;

		mtk_camera_debug(1, "%s:[%d] wait buffer done",
			getCameraNameFromId(ctx->camera_id), ctx->id);
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);
}

static const struct vb2_ops mtk_camera_vb2_ops = {
	.queue_setup		= vb2ops_camera_queue_setup,
	.buf_prepare		= vb2ops_camera_buf_prepare,
	.buf_queue			= vb2ops_camera_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.buf_init			= vb2ops_camera_buf_init,
	.buf_cleanup		= vb2ops_camera_buf_deinit,
	.start_streaming	= vb2ops_camera_start_streaming,
	.stop_streaming		= vb2ops_camera_stop_streaming,
};

static struct mtk_camera_fmt *mtk_camera_find_format(struct v4l2_format *f)
{
	struct mtk_camera_fmt *fmt;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_camera_formats); i++) {
		fmt = &mtk_camera_formats[i];
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			if (fmt->fourcc == f->fmt.pix_mp.pixelformat) {
				mtk_camera_debug(1, "matched mplane format %s",
					fmt->name);
				return fmt;
			}
		} else {
			if (fmt->fourcc == f->fmt.pix.pixelformat) {
				mtk_camera_debug(1, "matched format %s",
					fmt->name);
				return fmt;
			}
		}
	}

	return NULL;
}

static int mtk_camera_try_fmt(struct v4l2_format *f, struct mtk_camera_fmt *fmt)
{
	struct v4l2_pix_format *pix_fmt = &f->fmt.pix;
	unsigned int bytesperline = 0;
	int i;

	pix_fmt->field = V4L2_FIELD_NONE;

	/* Guaranteed to be a match */
	for (i = 0; i < ARRAY_SIZE(mtk_camera_sizes); i++)
		if ((mtk_camera_sizes[i].width == pix_fmt->width)
			&& (mtk_camera_sizes[i].height == pix_fmt->height)) {
			mtk_camera_debug(1, "matched size %ux%u",
				pix_fmt->width, pix_fmt->height);
			break;
		}

	/* Clamp the width and height. */
	pix_fmt->width = clamp(pix_fmt->width,
				MTK_VIDEO_CAPTURE_MIN_WIDTH,
				MTK_VIDEO_CAPTURE_MAX_WIDTH);
	pix_fmt->height = clamp(pix_fmt->height,
				MTK_VIDEO_CAPTURE_MIN_HEIGHT,
				MTK_VIDEO_CAPTURE_MAX_HEIGHT);

	pix_fmt->bytesperline = 0;
	pix_fmt->sizeimage = 0;
	for (i = 0; i < fmt->num_planes; i++) {
		bytesperline = pix_fmt->width * fmt->bpp[i] / 8;
		pix_fmt->bytesperline += bytesperline;
		pix_fmt->sizeimage += pix_fmt->height * bytesperline;
		mtk_camera_debug(1, "plane %d, bytesperline: %u, sizeimage: %u",
			i, bytesperline, pix_fmt->height * bytesperline);
	}

	mtk_camera_debug(1, "format %s, width:%u, height:%u, sizeimage: %u",
		fmt->name, pix_fmt->width, pix_fmt->height, pix_fmt->sizeimage);

	pix_fmt->flags = 0;

	return 0;
}

static int mtk_camera_try_fmt_mplane(struct v4l2_format *f,
	struct mtk_camera_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_fmt_mp = &f->fmt.pix_mp;
	unsigned int bytesperline = 0;
	int i;
	u32 memory_type = 0;
	u32 org_w, org_h;

	pix_fmt_mp->field = V4L2_FIELD_NONE;
	/* Clamp the width and height. */
	pix_fmt_mp->width = clamp(pix_fmt_mp->width,
				MTK_VIDEO_CAPTURE_MIN_WIDTH,
				MTK_VIDEO_CAPTURE_MAX_WIDTH);
	pix_fmt_mp->height = clamp(pix_fmt_mp->height,
				MTK_VIDEO_CAPTURE_MIN_HEIGHT,
				MTK_VIDEO_CAPTURE_MAX_HEIGHT);

	org_w = pix_fmt_mp->width;
	org_h = pix_fmt_mp->height;

	if ((pix_fmt_mp->pixelformat == V4L2_PIX_FMT_YUV420M) ||
	    (pix_fmt_mp->pixelformat == V4L2_PIX_FMT_YUV420M) ||
	    (pix_fmt_mp->pixelformat == V4L2_PIX_FMT_NV12) ||
	    (pix_fmt_mp->pixelformat == V4L2_PIX_FMT_NV12M) ||
	    (pix_fmt_mp->pixelformat == V4L2_PIX_FMT_NV21M))
		memory_type = pix_fmt_mp->field;
	else
		memory_type = 0;

	mtk_camera_debug(1, "mplane format %s, width:%u, height:%u, memory_type: %d",
		fmt->name, pix_fmt_mp->width, pix_fmt_mp->height, memory_type);

	if (memory_type) {
		/* for dma buffer handle */
		pix_fmt_mp->width = ALIGN_CEIL(pix_fmt_mp->width, 16);
		pix_fmt_mp->height = ALIGN_CEIL(pix_fmt_mp->height, 32);
	}

	for (i = 0; i < pix_fmt_mp->num_planes; i++) {
		memset(&(pix_fmt_mp->plane_fmt[i].reserved[0]), 0x0,
			   sizeof(pix_fmt_mp->plane_fmt[0].reserved));
		pix_fmt_mp->plane_fmt[i].sizeimage = 0;
		pix_fmt_mp->plane_fmt[i].bytesperline = 0;
	}

	for (i = 0; i < pix_fmt_mp->num_planes; i++) {
		bytesperline = pix_fmt_mp->width * fmt->bpp[i] / 8;
		pix_fmt_mp->plane_fmt[i].bytesperline += bytesperline;
		pix_fmt_mp->plane_fmt[i].sizeimage =
			pix_fmt_mp->height * bytesperline;
		mtk_camera_debug(1, "mplane format %s, width:%u, height:%u, sizeimage: %u",
			fmt->name, pix_fmt_mp->width,
			pix_fmt_mp->height, pix_fmt_mp->plane_fmt[i].sizeimage);
	}

	pix_fmt_mp->flags = 0;
	memset(&pix_fmt_mp->reserved, 0x0, sizeof(pix_fmt_mp->reserved));

	if (memory_type) {
		pix_fmt_mp->width = org_w;
		pix_fmt_mp->height = org_h;
	}

	return 0;
}
/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
camera_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, MTK_CAMERA_DEVICE, sizeof(cap->driver));
	strlcpy(cap->bus_info, MTK_PLATFORM_STR, sizeof(cap->bus_info));
	strlcpy(cap->card, MTK_PLATFORM_STR, sizeof(cap->card));

	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_STREAMING | V4L2_CAP_DEVICE_CAPS |
		V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	return 0;
}

static int
camera_enum_format(struct file *file, void *fh,
					struct v4l2_fmtdesc *f)
{
	const struct mtk_camera_fmt *fmt;

	if (f->index >= ARRAY_SIZE(mtk_camera_formats))
		return -EINVAL;

	fmt = &mtk_camera_formats[f->index];

	f->pixelformat = fmt->fourcc;
	memset(f->reserved, 0, sizeof(f->reserved));

	return 0;
}

static int
camera_get_format(struct file *file, void *fh,
					struct v4l2_format *format)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format *pix = &format->fmt.pix;
	struct mtk_q_data *q_data = &ctx->q_data;
	struct mtk_camera_fmt *fmt = q_data->fmt;

	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = ctx->colorspace;
	pix->ycbcr_enc = ctx->ycbcr_enc;
	pix->quantization = ctx->quantization;
	pix->xfer_func = ctx->xfer_func;

	/*
	 * Width and height are set to the dimensions
	 * of the movie, the buffer is bigger and
	 * further processing stages should crop to this
	 * rectangle.
	 */
	pix->width  = q_data->width;
	pix->height = q_data->height;

	/*
	 * Set pixelformat to the format in which mt vcodec
	 * outputs the decoded frame
	 */
	pix->pixelformat = q_data->fmt->fourcc;
	if (!fmt->mplane) {
		pix->bytesperline = q_data->bytesperline[0];
		pix->sizeimage = q_data->sizeimage[0];
	}

	return 0;
}

static int
camera_get_format_mplane(struct file *file, void *fh,
					struct v4l2_format *format)
{
	int i;
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format_mplane *pix_fmt_mp = &format->fmt.pix_mp;
	struct mtk_q_data *q_data = &ctx->q_data;

	mtk_camera_debug(1, "%s:camera get format mplane [%d]",
		getCameraNameFromId(ctx->camera_id), ctx->id);

	pix_fmt_mp->field = V4L2_FIELD_NONE;
	pix_fmt_mp->colorspace = ctx->colorspace;
	pix_fmt_mp->ycbcr_enc = ctx->ycbcr_enc;
	pix_fmt_mp->quantization = ctx->quantization;
	pix_fmt_mp->xfer_func = ctx->xfer_func;
	pix_fmt_mp->width  = q_data->width;
	pix_fmt_mp->height = q_data->height;

	for (i = 0; i < pix_fmt_mp->num_planes; i++) {
		pix_fmt_mp->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix_fmt_mp->plane_fmt[i].sizeimage = q_data->sizeimage[i];
	}

	return 0;
}

static int
camera_set_format(struct file *file, void *fh,
					struct v4l2_format *format)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format *pix = &format->fmt.pix;
	struct mtk_q_data *q_data = &ctx->q_data;
	struct mtk_camera_fmt *fmt;
	unsigned int bytesperline = 0;
	uint32_t size[3];
	int i, ret = 0;

	mtk_camera_debug(1, "%s:camera set format [%d]",
		getCameraNameFromId(ctx->camera_id), ctx->id);

	if (vb2_is_busy(&ctx->queue)) {
		mtk_camera_err("capture buffers already requested");
		ret = -EBUSY;
	}

	if (!q_data) {
		mtk_camera_err("%s:[%d]: q_data is NULL",
			getCameraNameFromId(ctx->camera_id), ctx->id);
		return -EINVAL;
	}

	fmt = mtk_camera_find_format(format);
	if (fmt == NULL) {
		format->fmt.pix.pixelformat = MTK_VIDEO_CAPTURE_DEF_FORMAT;
		fmt = mtk_camera_find_format(format);
	}
	q_data->fmt = fmt;

	mtk_camera_try_fmt(format, q_data->fmt);
	if (fmt->mplane) {
		for (i = 0; i < fmt->num_planes; i++) {
			bytesperline = pix->width * fmt->bpp[i] / 8;
			q_data->bytesperline[i] = bytesperline;
			q_data->sizeimage[i] = pix->height * bytesperline;
			mtk_camera_debug(1, "format %s, bytesperline:%u, sizeimage: %u",
				fmt->name, q_data->bytesperline[i],
				q_data->sizeimage[i]);
		}
	} else {
		for (i = 0; i < fmt->num_planes; i++)
			bytesperline += pix->width * fmt->bpp[i] / 8;
		q_data->bytesperline[0] = bytesperline;
		q_data->sizeimage[0] = pix->height * bytesperline;
		mtk_camera_debug(1, "format %s, bytesperline:%u, sizeimage: %u",
			fmt->name, q_data->bytesperline[0],
			q_data->sizeimage[0]);
	}

	q_data->width = pix->width;
	q_data->height = pix->height;
	size[0] = pix->width;
	size[1] = pix->height;
	size[2] = q_data->fmt->fourcc;

	ctx->colorspace = format->fmt.pix.colorspace;
	ctx->ycbcr_enc = format->fmt.pix.ycbcr_enc;
	ctx->quantization = format->fmt.pix.quantization;
	ctx->xfer_func = format->fmt.pix.xfer_func;

	if (ctx->state == MTK_STATE_FREE) {
		ret = camera_if_init(ctx);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_init() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return -EINVAL;
		}
		camera_if_set_param(ctx, SET_PARAM_FRAME_SIZE, (void *)size);
		ctx->state = MTK_STATE_INIT;
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);

	mtk_camera_debug(1, "ret %d", ret);

	return 0;
}


static int
camera_set_format_mplane(struct file *file, void *fh,
					struct v4l2_format *format)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	struct v4l2_pix_format_mplane *pix_mp;
	struct mtk_q_data *q_data = &ctx->q_data;
	struct mtk_camera_fmt *fmt;
	uint32_t size[3];
	int i, ret = 0;

	mtk_camera_debug(1, "%s:camera set format mplane [%d]",
		getCameraNameFromId(ctx->camera_id), ctx->id);

	if (vb2_is_busy(&ctx->queue)) {
		mtk_camera_err("capture buffers already requested");
		ret = -EBUSY;
	}

	if (!q_data) {
		mtk_camera_err("%s:[%d]: q_data is NULL",
			getCameraNameFromId(ctx->camera_id), ctx->id);
		return -EINVAL;
	}
	pix_mp = &format->fmt.pix_mp;

	fmt = mtk_camera_find_format(format);
	if (fmt == NULL) {
		format->fmt.pix_mp.pixelformat = MTK_VIDEO_CAPTURE_DEF_FORMAT;
		fmt = mtk_camera_find_format(format);
	}
	q_data->fmt = fmt;

	mtk_camera_try_fmt_mplane(format, q_data->fmt);

	for (i = 0; i < fmt->num_planes; i++) {
		q_data->bytesperline[i] = pix_mp->plane_fmt[i].bytesperline;
		q_data->sizeimage[i] = pix_mp->plane_fmt[i].sizeimage;
		mtk_camera_debug(1, "format %s, bytesperline:%u, sizeimage: %u",
			fmt->name, q_data->bytesperline[i],
			q_data->sizeimage[i]);
	}

	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	size[0] = pix_mp->width;
	size[1] = pix_mp->height;
	size[2] = fmt->fourcc;

	ctx->colorspace = format->fmt.pix_mp.colorspace;
	ctx->ycbcr_enc = format->fmt.pix_mp.ycbcr_enc;
	ctx->quantization = format->fmt.pix_mp.quantization;
	ctx->xfer_func = format->fmt.pix_mp.xfer_func;

	ctx->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	ctx->queue.is_multiplanar = 1;


	if (ctx->state == MTK_STATE_FREE) {
		ret = camera_if_init(ctx);
		if (ret) {
			mtk_camera_err("%s:[%d]: camera_if_init() fail ret=%d",
				getCameraNameFromId(ctx->camera_id),
				ctx->id, ret);
			return -EINVAL;
		}

		mtk_camera_err("set format mplane, set format:%d", size[2]);
		camera_if_set_param(ctx, SET_PARAM_FRAME_SIZE, (void *)size);
		ctx->state = MTK_STATE_INIT;
	} else
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);

	mtk_camera_debug(1, "ret %d", ret);

	return 0;
}


static int
camera_try_format(struct file *file, void *fh,
					struct v4l2_format *format)
{
	struct mtk_camera_fmt *fmt;

	fmt = mtk_camera_find_format(format);
	if (fmt == NULL) {
		format->fmt.pix.pixelformat =
			MTK_VIDEO_CAPTURE_DEF_FORMAT;
		fmt = mtk_camera_find_format(format);
	}

	return mtk_camera_try_fmt(format, fmt);
}

static int
camera_try_format_mplane(struct file *file, void *fh,
					struct v4l2_format *format)
{
	struct mtk_camera_fmt *fmt;

	fmt = mtk_camera_find_format(format);
	if (fmt == NULL) {
		format->fmt.pix_mp.pixelformat =
			MTK_VIDEO_CAPTURE_DEF_FORMAT;
		fmt = mtk_camera_find_format(format);
	}

	return mtk_camera_try_fmt_mplane(format, fmt);
}

static int
camera_get_param(struct file *file, void *fh,
					struct v4l2_streamparm *parm)
{

	return 0;
}

static int
camera_set_param(struct file *file, void *fh,
					struct v4l2_streamparm *parm)
{
	return 0;
}

static int
camera_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= ARRAY_SIZE(mtk_camera_sizes))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete = mtk_camera_sizes[fsize->index];
	return 0;
}

/* timeperframe is arbitrary and continuous */
static int
camera_enum_frameintervals(struct file *file, void *fh,
					struct v4l2_frmivalenum *fival)
{
	if (fival->index != 0)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 30;
	return 0;
}

static int
camera_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *rb)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], cnt[%d] mem[%d] type[%d, %d] point[0x%p]",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, rb->count, rb->memory,
		rb->type, ctx->queue.type, &ctx->queue);

	ret = vb2_reqbufs(&ctx->queue, rb);

	mtk_camera_debug(1, "ret %d cnt[%d]", ret, rb->count);

	return ret;
}

static int
camera_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], queue %p",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, &ctx->queue);

	ret = vb2_querybuf(&ctx->queue, b);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

static int
camera_expbuf(struct file *file, void *fh, struct v4l2_exportbuffer *p)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], fd[%d], flag[%d], idx[%d], plane[%d]",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, p->fd, p->flags, p->index, p->plane);

	ret = vb2_expbuf(&ctx->queue, p);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

static int
camera_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], length[%d], bytes[%d]",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, b->length, b->bytesused);

	if (ctx->state == MTK_STATE_FREE || ctx->state == MTK_STATE_FLUSH) {
		mtk_camera_err("%s:[%d] state=(%x) invalid call",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, ctx->state);
		return -EIO;
	}

	ret = vb2_qbuf(&ctx->queue, b);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

static int
camera_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], length[%d], bytes[%d]",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, b->length, b->bytesused);

	ret = vb2_dqbuf(&ctx->queue, b, file->f_flags & O_NONBLOCK);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

int camera_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], queue %p",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, &ctx->queue);

	ret = vb2_streamon(&ctx->queue, i);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

int camera_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(fh);
	int ret;

	mtk_camera_debug(1, "%s: [%d], queue %p",
		getCameraNameFromId(ctx->camera_id),
		ctx->id, &ctx->queue);

	ret = vb2_streamoff(&ctx->queue, i);

	mtk_camera_debug(1, "ret %d", ret);

	return ret;
}

static int
camera_enum_input(struct file *file, void *fh, struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int
camera_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int
camera_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops mtk_camera_ioctl_ops = {
	.vidioc_querycap		= camera_querycap,

	.vidioc_enum_fmt_vid_cap = camera_enum_format,
	.vidioc_g_fmt_vid_cap	= camera_get_format,

	.vidioc_s_fmt_vid_cap	= camera_set_format,
	.vidioc_try_fmt_vid_cap	= camera_try_format,

	.vidioc_try_fmt_vid_cap_mplane	= camera_try_format_mplane,

	.vidioc_s_fmt_vid_cap_mplane	= camera_set_format_mplane,

	.vidioc_g_fmt_vid_cap_mplane	= camera_get_format_mplane,
	.vidioc_enum_fmt_vid_cap_mplane = camera_enum_format,


	.vidioc_g_parm			= camera_get_param,
	.vidioc_s_parm			= camera_set_param,

	.vidioc_enum_framesizes = camera_enum_framesizes,
	.vidioc_enum_frameintervals = camera_enum_frameintervals,

	.vidioc_reqbufs			= camera_reqbufs,
	.vidioc_querybuf		= camera_querybuf,
	.vidioc_expbuf			= camera_expbuf,
	.vidioc_qbuf			= camera_qbuf,
	.vidioc_dqbuf			= camera_dqbuf,
	.vidioc_streamon		= camera_streamon,
	.vidioc_streamoff		= camera_streamoff,

	.vidioc_enum_input		= camera_enum_input,
	.vidioc_g_input			= camera_g_input,
	.vidioc_s_input			= camera_s_input,
};

static void mtk_handle_buffer(struct mtk_camera_mem *fb)
{
	struct camera_buffer *buffer = container_of(fb,
		struct camera_buffer, mem);
	struct vb2_v4l2_buffer *vb2_v4l2 = &buffer->vb;
	struct vb2_buffer *vb = &vb2_v4l2->vb2_buf;
	struct mtk_camera_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	int i = 0;

	mtk_camera_debug(1, "%s:[%d] (%d) id=%d, state=%d, vb=%p",
			getCameraNameFromId(ctx->camera_id),
			ctx->id, vb->vb2_queue->type,
			vb->index, vb->state, vb);

	/*should be replaced by payload */
	for (i = 0; i < fb->num_planes; i++)
		vb2_set_plane_payload(vb, i, fb->planes[i].size);

	if (fb->status == BUFFER_FILLED)
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	else
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

void mtk_camera_unlock(struct mtk_camera_ctx *ctx)
{
	mutex_unlock(&ctx->dev->capture_mutex);
}

void mtk_camera_lock(struct mtk_camera_ctx *ctx)
{
	mutex_lock(&ctx->dev->capture_mutex);
}

static void mtk_camera_release(struct mtk_camera_ctx *ctx)
{
	camera_if_deinit(ctx);
	ctx->state = MTK_STATE_FREE;
}

#define V4L2_CID_CAMERA_MIN_FPS          (V4L2_CID_USER_MTK_CAMERA_BASE + 0)
#define V4L2_CID_CAMERA_MAX_FPS          (V4L2_CID_USER_MTK_CAMERA_BASE + 1)


static int
camera_set_ctrl(struct mtk_camera_ctx *ctx,
	enum camera_set_param_type type, uint32_t value)
{
	int ret = 0;

	if (!ctx->cam_if_rdy) {
		mtk_camera_err("[%d]: camera_if not ready!", ctx->id);
		return -EINVAL;
	}
	camera_if_set_param(ctx, type, (void *)&value);
	mtk_camera_err("type:%d,value:%d cam_if_rdy:%d",
		type, value, ctx->cam_if_rdy);
	return ret;
}


static int mtk_camera_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_camera_ctx *ctx = ctrl_to_ctx(ctrl);
	int ret = 0;

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	switch (ctrl->id) {

	case V4L2_CID_BRIGHTNESS:
		ret = camera_set_ctrl(ctx, SET_PARAM_BRIGHTNESS,
			ctrl->val);
		break;

	case V4L2_CID_CONTRAST:
		ret = camera_set_ctrl(ctx, SET_PARAM_CONTRAST,
			ctrl->val);
		break;

	case V4L2_CID_SATURATION:
		ret = camera_set_ctrl(ctx, SET_PARAM_SATURATION,
			ctrl->val);
		break;

	case V4L2_CID_SHARPNESS:
		ret = camera_set_ctrl(ctx, SET_PARAM_SHARPNESS,
			ctrl->val);
		break;

	case V4L2_CID_HUE:
		ret = camera_set_ctrl(ctx, SET_PARAM_HUE,
			ctrl->val);
		break;

	case V4L2_CID_GAMMA:
		ret = camera_set_ctrl(ctx, SET_PARAM_GAMMA,
			ctrl->val);
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = camera_set_ctrl(ctx, SET_PARAM_AUTO_WHITE_BALANCE,
			ctrl->val);
		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		ret = camera_set_ctrl(ctx, SET_PARAM_DO_WHITE_BALANCE,
			ctrl->val);
		break;

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		ret = camera_set_ctrl(ctx, SET_PARAM_WHITE_BALANCE_TEMP,
			ctrl->val);
		break;

	case V4L2_CID_EXPOSURE:
		ret = camera_set_ctrl(ctx, SET_PARAM_EXPOSURE,
			ctrl->val);
		break;

	case V4L2_CID_AUTOGAIN:
		ret = camera_set_ctrl(ctx, SET_PARAM_AUTOGAIN,
			ctrl->val);
		break;

	case V4L2_CID_GAIN:
		ret = camera_set_ctrl(ctx, SET_PARAM_GAIN,
			ctrl->val);
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = camera_set_ctrl(ctx, SET_PARAM_POWER_LINE_FREQ,
			ctrl->val);
		break;

	case V4L2_CID_BACKLIGHT_COMPENSATION:
		ret = camera_set_ctrl(ctx, SET_PARAM_BACKLIGHT_COMPENSATION,
			ctrl->val);
		break;

	case V4L2_CID_CAMERA_MIN_FPS:
		ret = camera_set_ctrl(ctx, SET_PARAM_MIN_FPS,
			ctrl->val);
		break;

	case V4L2_CID_CAMERA_MAX_FPS:
		ret = camera_set_ctrl(ctx, SET_PARAM_MAX_FPS,
			ctrl->val);
		break;

	case V4L2_CID_CAMERA_CLASS:
		ret = camera_set_ctrl(ctx, SET_PARAM_CAMERA_CLASS,
			ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		ret = camera_set_ctrl(ctx, SET_PARAM_EXPOSURE_AUTO,
			ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		ret = camera_set_ctrl(ctx, SET_PARAM_EXPOSURE_ABSOLUTE,
			ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
		ret = camera_set_ctrl(ctx, SET_PARAM_EXPOSURE_AUTO_PRI,
			ctrl->val);
		break;

	case V4L2_CID_FOCUS_AUTO:
		ret = camera_set_ctrl(ctx, SET_PARAM_FOCUS_AUTO,
			ctrl->val);
		break;

	case V4L2_CID_FOCUS_RELATIVE:
		ret = camera_set_ctrl(ctx, SET_PARAM_FOCUS_RELATIVE,
			ctrl->val);
		break;

	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = camera_set_ctrl(ctx, SET_PARAM_FOCUS_ABSOLUTE,
			ctrl->val);
		break;

	case V4L2_CID_PRIVACY:
		ret = camera_set_ctrl(ctx, SET_PARAM_PRIVACY,
			ctrl->val);
		break;

	case V4L2_CID_BAND_STOP_FILTER:
		ret = camera_set_ctrl(ctx, SET_PARAM_BAND_STOP_FILT,
			ctrl->val);
		break;

	case V4L2_CID_PAN_RELATIVE:
		ret = camera_set_ctrl(ctx, SET_PARAM_PAN_RELEATIVE,
			ctrl->val);
		break;
	case V4L2_CID_PAN_RESET:
		ret = camera_set_ctrl(ctx, SET_PARAM_PAN_RESET,
			ctrl->val);
		break;

	case V4L2_CID_PAN_ABSOLUTE:
		ret = camera_set_ctrl(ctx, SET_PARAM_PAN_ABSOLUTE,
			ctrl->val);
		break;

	case V4L2_CID_TILT_RELATIVE:
		ret = camera_set_ctrl(ctx, SET_PARAM_TILT_RELATIVE,
			ctrl->val);
		break;
	case V4L2_CID_TILT_RESET:
		ret = camera_set_ctrl(ctx, SET_PARAM_TILT_RESET,
			ctrl->val);
		break;

	case V4L2_CID_TILT_ABSOLUTE:
		ret = camera_set_ctrl(ctx, SET_PARAM_TILT_ABSOLUTE,
			ctrl->val);
		break;

	case V4L2_CID_ZOOM_ABSOLUTE:
		ret = camera_set_ctrl(ctx, SET_PARAM_ZOOM_ABSOLUTE,
			ctrl->val);
		break;
	case V4L2_CID_ZOOM_RELATIVE:
		ret = camera_set_ctrl(ctx, SET_PARAM_ZOOM_RELATIVE,
			ctrl->val);
		break;

	case V4L2_CID_ZOOM_CONTINUOUS:
		ret = camera_set_ctrl(ctx, SET_PARAM_ZOOM_CONTINUOUS,
			ctrl->val);
		break;

	default:
		break;

	}

	return ret;
}


static const struct v4l2_ctrl_ops mtk_camera_ctrl_ops = {
	.s_ctrl = mtk_camera_s_ctrl,
};


static const struct v4l2_ctrl_config mtk_camera_ctrl_min_fps = {
	.ops = &mtk_camera_ctrl_ops,
	.id = V4L2_CID_CAMERA_MIN_FPS,
	.name = "Min Fps",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 60,
	.step = 1,
	.def = 20,
};

static const struct v4l2_ctrl_config mtk_camera_ctrl_max_fps = {
	.ops = &mtk_camera_ctrl_ops,
	.id = V4L2_CID_CAMERA_MAX_FPS,
	.name = "Max Fps",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 60,
	.step = 1,
	.def = 60,
};


static int mtk_camera_ctrls_create(struct mtk_camera_ctx *ctx)
{
	int ret = 0;

	ret = v4l2_ctrl_handler_init(&ctx->ctrl_hdl, MTK_CAMERA_MAX_CTRLS_HINT);

	ctx->ctrls.brightness = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_BRIGHTNESS,
			0x0, 0xff, 0x1, 0x37);

	ctx->ctrls.contrast = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_CONTRAST,
			0x10, 0x40, 1, 0x20);

	ctx->ctrls.saturation = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_SATURATION,
			0x0, 0xff, 0x1, 0x80);

	ctx->ctrls.sharpness = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_SHARPNESS,
			0x0, 0x7, 1, 0);

	ctx->ctrls.hue = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_HUE,
			-0x16, 0x16, 1, 0);

	ctx->ctrls.gamma = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_GAMMA,
			0x0, 0xff, 1, 0);

	ctx->ctrls.auto_white_balance = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_AUTO_WHITE_BALANCE,
			0x0, 0xff, 1, 1);

	ctx->ctrls.do_white_balance = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_DO_WHITE_BALANCE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.white_balance_temperature = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_WHITE_BALANCE_TEMPERATURE,
			0xa8c, 0x1964, 1, 0x1964);

	ctx->ctrls.exposure = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_EXPOSURE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.autogain = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_AUTOGAIN,
			0x0, 0xff, 1, 0);

	ctx->ctrls.gain = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_GAIN,
			0x0, 0xff, 1, 0x20);

	ctx->ctrls.power_line_frequency = v4l2_ctrl_new_std_menu(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_POWER_LINE_FREQUENCY,
			0x3, 0, 1);

	ctx->ctrls.backlight_compensation = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_BACKLIGHT_COMPENSATION,
			0x0, 0x4, 1, 1);

	ctx->ctrls.camera_class = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_CAMERA_CLASS,
			0x0, 0xff, 1, 0);

	ctx->ctrls.exposure_auto = v4l2_ctrl_new_std_menu(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_EXPOSURE_AUTO,
			0x3, 0, 0);

	ctx->ctrls.exposure_absolute = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_EXPOSURE_ABSOLUTE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.exposure_auto_priority = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_EXPOSURE_AUTO_PRIORITY,
			0x0, 0xff, 1, 0);

	ctx->ctrls.focus_absolute = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_FOCUS_ABSOLUTE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.focus_relative = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_FOCUS_RELATIVE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.focus_auto = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_FOCUS_AUTO,
			0x0, 0xff, 1, 0);

	ctx->ctrls.privacy = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_PRIVACY,
			0x0, 0xff, 1, 0);

	ctx->ctrls.band_stop_filter = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_BAND_STOP_FILTER,
			0x0, 0xff, 1, 0);

	ctx->ctrls.pan_relative = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_PAN_RELATIVE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.pan_absolute = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_PAN_ABSOLUTE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.pan_reset = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_PAN_RESET,
			0x0, 0xff, 1, 0);

	ctx->ctrls.tilt_absolute = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_TILT_ABSOLUTE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.tilt_relative = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_TILT_RELATIVE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.tilt_reset = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_TILT_RESET,
			0x0, 0xff, 1, 0);

	ctx->ctrls.zoom_absolute = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_ZOOM_ABSOLUTE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.zoom_continuous = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_ZOOM_CONTINUOUS,
			0x0, 0xff, 1, 0);

	ctx->ctrls.zoon_relative = v4l2_ctrl_new_std(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_ops,
			V4L2_CID_ZOOM_RELATIVE,
			0x0, 0xff, 1, 0);

	ctx->ctrls.min_fps = v4l2_ctrl_new_custom(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_min_fps,
			NULL);

	ctx->ctrls.max_fps = v4l2_ctrl_new_custom(&ctx->ctrl_hdl,
			&mtk_camera_ctrl_max_fps,
			NULL);

	ctx->ctrls_rdy = ctx->ctrl_hdl.error == 0;

	return 0;
}

int mtk_camera_ctrls_setup(struct mtk_camera_ctx *ctx)
{
	mtk_camera_ctrls_create(ctx);

	if (ctx->ctrl_hdl.error) {
		mtk_camera_err("adding control failed %d",
			ctx->ctrl_hdl.error);
		v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
		dev_info(&ctx->dev->plat_dev->dev,
			"Failed to create control handlers\n");
		return ctx->ctrl_hdl.error;
	}

	v4l2_ctrl_handler_setup(&ctx->ctrl_hdl);

	return 0;
}

void mtk_camera_set_default_params(struct mtk_camera_ctx *ctx)
{
	struct mtk_q_data *q_data = &ctx->q_data;

	ctx->fh.ctrl_handler = &ctx->ctrl_hdl;
	ctx->colorspace = V4L2_COLORSPACE_REC709;
	ctx->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	ctx->quantization = V4L2_QUANTIZATION_DEFAULT;
	ctx->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	memset(q_data, 0, sizeof(struct mtk_q_data));
	q_data->width  = MTK_VIDEO_CAPTURE_DEF_WIDTH;
	q_data->height = MTK_VIDEO_CAPTURE_DEF_HEIGHT;
	q_data->fmt = mtk_camera_formats;
	q_data->field = V4L2_FIELD_NONE;

	q_data->sizeimage[0] = q_data->width * q_data->height * 1.5;
	q_data->bytesperline[0] = q_data->width * 1.5;
}

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int fops_camera_open(struct file *file)
{
	struct mtk_camera_dev *dev = video_drvdata(file);
	struct mtk_camera_ctx *ctx = NULL;
	struct vb2_queue *queue;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_lock(&dev->dev_mutex);
	ctx->id = dev->id_counter++;
	ctx->camera_id = dev->cameraid;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	INIT_LIST_HEAD(&ctx->list);
	ctx->dev = dev;
	ctx->cam_if_rdy = false;

	ret = mtk_camera_ctrls_setup(ctx);
	if (ret) {
		mtk_camera_err("Failed to setup video capture controls");
		goto err_ctrls_setup;
	}

	queue = &ctx->queue;
	queue->type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes	= VB2_DMABUF | VB2_MMAP;
	queue->drv_priv	= ctx;
	queue->buf_struct_size = sizeof(struct camera_buffer);
	queue->ops		= &mtk_camera_vb2_ops;
	queue->mem_ops	= &vb2_dma_contig_memops;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock		= &ctx->dev->dev_mutex;
	queue->allow_zero_bytesused = 1;
	queue->dev = &ctx->dev->plat_dev->dev;

	ret = vb2_queue_init(&ctx->queue);
	if (ret < 0) {
		mtk_camera_err("Failed to initialize videobuf2 queue");
		goto err_vb2_init;
	}

	dev->queue = &ctx->queue;

	mtk_camera_set_default_params(ctx);

	ctx->callback = mtk_handle_buffer;

	if (v4l2_fh_is_singular(&ctx->fh)) {
		/*
		 * vcu_load_firmware checks if it was loaded already and
		 * does nothing in that case
		 */
		ret = vcu_load_firmware(dev->vcu_plat_dev);
		if (ret < 0) {
			/*
			 * Return 0 if downloading firmware successfully,
			 * otherwise it is failed
			 */
			mtk_camera_err("vcu_load_firmware failed!");
			goto err_load_fw;
		}
	}

	list_add(&ctx->list, &dev->ctx_list);

	mutex_unlock(&dev->dev_mutex);
	mtk_camera_debug(0, "%s:%s capture [%d]",
		getCameraNameFromId(ctx->camera_id),
		dev_name(&dev->plat_dev->dev), ctx->id);
	return ret;

	/* Deinit when failure occurred */
err_load_fw:
	vb2_queue_release(&ctx->queue);
err_vb2_init:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_ctrls_setup:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int fops_camera_release(struct file *file)
{
	struct mtk_camera_dev *dev = video_drvdata(file);
	struct mtk_camera_ctx *ctx = fh_to_ctx(file->private_data);

	mtk_camera_debug(0, "%s:[%d] video capture",
		getCameraNameFromId(ctx->camera_id), ctx->id);
	mutex_lock(&dev->dev_mutex);

	camera_streamoff(file, &ctx->fh, ctx->queue.type);

	vb2_queue_release(&ctx->queue);
	mtk_camera_release(ctx);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);

	list_del_init(&ctx->list);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static unsigned int fops_camera_poll(struct file *file, poll_table *wait)
{
	struct mtk_camera_dev *dev = video_drvdata(file);
	struct mtk_camera_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;

	mtk_camera_debug(0, "%s,[%d] video capture",
		getCameraNameFromId(ctx->camera_id), ctx->id);

	mutex_lock(&dev->dev_mutex);
	ret = vb2_poll(&ctx->queue, file, wait);
	mutex_unlock(&dev->dev_mutex);

	mtk_camera_debug(1, "ret %d", ret);
	return ret;
}

static int fops_camera_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct mtk_camera_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;

	mtk_camera_debug(0, "%s:[%d] video capture",
		getCameraNameFromId(ctx->camera_id), ctx->id);

	ret = vb2_mmap(&ctx->queue, vma);

	mtk_camera_debug(1, "ret %d", ret);
	return ret;
}

static const struct v4l2_file_operations mtk_camera_fops = {
	.owner		= THIS_MODULE,
	.open		= fops_camera_open,
	.release	= fops_camera_release,
	.poll		= fops_camera_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= fops_camera_mmap,
};

static int mtk_camera_probe(struct platform_device *pdev)
{
	struct mtk_camera_dev *dev;
	struct video_device *video;
	int ret, cameraid;
	struct device *device;

	device = &pdev->dev;
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->ctx_list);
	dev->plat_dev = pdev;
	dev->vcu_plat_dev = vcu_get_plat_device(dev->plat_dev);
	if (dev->vcu_plat_dev == NULL) {
		mtk_camera_err("[VCU] vcu device in not ready");
		return -EPROBE_DEFER;
	}

	mutex_init(&dev->capture_mutex);
	mutex_init(&dev->dev_mutex);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s",
		"[/MTK_V4L2_camera]");

	ret = of_property_read_u32(device->of_node,
		"mediatek,cameraid", &cameraid);
	if (ret != 0) {
		mtk_camera_err("failed to find mediatek,cameraid\n");
		return ret;
	}
	dev->cameraid = cameraid;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		mtk_camera_err("v4l2_device_register err=%d", ret);
		return -ENOMEM;
	}

	video = video_device_alloc();
	if (!video) {
		mtk_camera_err("Failed to allocate video device");
		goto err_alloc;
	}

	video->fops = &mtk_camera_fops;
	video->ioctl_ops = &mtk_camera_ioctl_ops;
	video->release = video_device_release;
	video->lock = &dev->dev_mutex;
	video->v4l2_dev  = &dev->v4l2_dev;
	video->vfl_type = VFL_TYPE_GRABBER;

	snprintf(video->name, sizeof(video->name), "%s", MTK_CAMERA_NAME);

	video_set_drvdata(video, dev);
	dev->video = video;
	platform_set_drvdata(pdev, dev);

	ret = video_register_device(video, VFL_TYPE_GRABBER, 0);
	if (ret) {
		mtk_camera_err("Failed to register video device");
		goto err__mem_init;
	}

	mtk_camera_debug(0, "video capture registered as /dev/video%d",
		video->num);

	return 0;

err__mem_init:
	video_unregister_device(video);
err_alloc:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static const struct of_device_id mtk_camera_match[] = {
	{.compatible = "mediatek,mt8168-camera",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_camera_match);

static int mtk_camera_remove(struct platform_device *pdev)
{
	struct mtk_camera_dev *dev = platform_get_drvdata(pdev);

	mtk_camera_debug(0, "dev %p", dev);

	if (dev->video)
		video_unregister_device(dev->video);

	v4l2_device_unregister(&dev->v4l2_dev);

	mutex_destroy(&dev->capture_mutex);
	mutex_destroy(&dev->dev_mutex);

	return 0;
}

static struct platform_driver mtk_camera_driver = {
	.probe	= mtk_camera_probe,
	.remove	= mtk_camera_remove,
	.driver	= {
		.name	= MTK_CAMERA_NAME,
		.of_match_table = mtk_camera_match,
	},
};

module_platform_driver(mtk_camera_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek V4L2 video capture driver");
