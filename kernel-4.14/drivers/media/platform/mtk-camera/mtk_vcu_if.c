// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "mtk_camera_drv_base.h"
#include "mtk_camera_drv.h"
#include "mtk_camera_util.h"
#include "mtk_vcu_if.h"

static void handle_init_ack_msg(struct cam_vcu_ipi_ack *m)
{
	struct video_inst *v =
		(struct video_inst *)(unsigned long)m->ap_inst_addr;

	mtk_camera_debug(1, "+ ap_inst_addr = 0x%llx",
		(uint64_t)m->ap_inst_addr);

	/* mapping VCU address to kernel virtual address */
	//inst->vsi = vcu_mapping_dm_addr(inst->dev, msg->vcu_inst_addr);
	v->inst_addr = m->vcu_inst_addr;
	mtk_camera_debug(1, "- vcu_inst_addr = 0x%llx", (uint64_t)v->inst_addr);
}

static void handle_capture_ack_msg(struct cam_vcu_ipi_ack *m)
{
	struct video_inst *v =
		(struct video_inst *)(unsigned long)m->ap_inst_addr;
	struct mtk_camera_ctx *ctx = v->ctx;
	struct fb_info_out *info = &m->info;
	struct mtk_camera_mem *mem = NULL;

	mtk_camera_debug(3, "+ ap_inst_addr = 0x%llx, handle 0x%x",
		(uint64_t)m->ap_inst_addr, info->dma_addr);

	list_for_each_entry(mem, &v->queue, list) {
		if (mem->planes[0].dma_addr == info->dma_addr) {
			if (info->status == 0)
				mem->status = BUFFER_FILLED;
			else
				mem->status = BUFFER_ERROR;
			ctx->callback(mem);
			mtk_camera_debug(3, "- vcu_inst_addr: 0x%llx, buffer handle 0x%x",
				v->inst_addr, info->dma_addr);
			return;
		}
	}
	mtk_camera_err("invalid buffer handle %x\n", info->dma_addr);
}
/*
 * This function runs in interrupt context and it means there's a IPI MSG
 * from VCU.
 */
int vcu_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct cam_vcu_ipi_ack *m = data;
	struct video_inst *v =
		(struct video_inst *)((unsigned long)m->ap_inst_addr);
	int ret = 0;

	mtk_camera_debug(3, "+ id=%x status = %d\n", m->msg_id, m->status);

	v->failure = m->status;

	if (m->status == 0) {
		switch (m->msg_id) {
		case VCU_IPIMSG_CAM_INIT_ACK:
			handle_init_ack_msg(data);
			break;
		case VCU_IPIMSG_CAM_START_STREAM_ACK:
		case VCU_IPIMSG_CAM_STOP_STREAM_ACK:
		case VCU_IPIMSG_CAM_INIT_BUFFER_ACK:
		case VCU_IPIMSG_CAM_DEINIT_BUFFER_ACK:
		case VCU_IPIMSG_CAM_START_ACK:
		case VCU_IPIMSG_CAM_DEINIT_ACK:
		case VCU_IPIMSG_CAM_SET_PARAM_ACK:
		case AP_IPIMSG_CAM_GET_PARAM:
			break;
		case VCU_IPIMSG_CAM_END_ACK:
			handle_capture_ack_msg(data);
			ret = 1;
			break;
		default:
			mtk_camera_err("invalid msg=%x", m->msg_id);
			ret = 1;
			break;
		}
	}

	mtk_camera_debug(3, "- id=%x", m->msg_id);
	v->signaled = 1;

	return ret;
}

static int camera_vcu_send_msg(struct video_inst *v, void *m, int l)
{
	uint32_t msg_id = *(uint32_t *)m;
	int err = 0;

	mtk_camera_debug(3, "id=%x", msg_id);

	v->failure  = 0;
	v->signaled = 0;

	err = vcu_ipi_send(v->dev, v->id, m, l);
	if (err) {
		mtk_camera_err("send fail vcu_id=%d msg_id=%x status=%d",
			       v->id, msg_id, err);
		return err;
	}

	return v->failure;
}

static int camera_send_ap_ipi(struct video_inst *v,
		unsigned int id, struct fb_info_in *n)
{
	struct cam_ap_ipi_cmd m;

	memset(&m, 0, sizeof(m));
	m.msg_id = id;
	m.ipi_id = v->id;
	m.vcu_inst_addr = v->inst_addr;
	m.ap_inst_addr = (uint64_t)(unsigned long)v;
	if (n != NULL)
		m.info = *n;

	return camera_vcu_send_msg(v, &m, sizeof(m));
}

static int camera_vcu_set_param(struct video_inst *inst,
		unsigned int id, void *param, unsigned int size)
{
	struct cam_ap_ipi_set_param msg;
	uint32_t *param_ptr = (uint32_t *)param;
	int i = 0;

	mtk_camera_debug(3, "+ id=%x", AP_IPIMSG_CAM_SET_PARAM);

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_CAM_SET_PARAM;
	msg.ipi_id = inst->id;
	msg.id = id;
	msg.vcu_inst_addr = inst->inst_addr;
	msg.ap_inst_addr = (uint64_t)(unsigned long)inst;

	for (i = 0; i < size; ++i)
		msg.data[i] = *(param_ptr + i);

	return camera_vcu_send_msg(inst, &msg, sizeof(msg));
}


static int camera_init(void *ctx, unsigned long *handle)
{
	struct video_inst *inst = NULL;
	struct mtk_camera_ctx *contex = (struct mtk_camera_ctx *)ctx;
	struct cam_ap_ipi_init msg;
	int err = 0;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst || !ctx)
		return -ENOMEM;

	inst->ctx = contex;
	inst->failure  = 0;
	inst->signaled = 0;
	if (contex->camera_id == 0)
		inst->id = IPI_CAMERA_MAIN;
	else
		inst->id = IPI_CAMERA_SUB;

	mtk_camera_debug(0, "camera_id:%d ipi:%d", contex->camera_id, inst->id);

	INIT_LIST_HEAD(&inst->queue);
	inst->dev = vcu_get_plat_device(contex->dev->plat_dev);

	err = vcu_ipi_register(inst->dev, inst->id,
		vcu_ipi_handler, NULL, NULL);
	if (err != 0) {
		mtk_camera_err("vcu_ipi_register fail status=%d", err);
		return err;
	}

	*handle = (unsigned long)inst;

	memset(&msg, 0, sizeof(msg));
	msg.msg_id = AP_IPIMSG_CAM_INIT;
	msg.ipi_id = inst->id;
	msg.ap_inst_addr = (uint64_t)(unsigned long)inst;

	return camera_vcu_send_msg(inst, (void *)&msg, sizeof(msg));
}

int camera_init_buffer(unsigned long handle, void *fb)
{
	struct video_inst *inst = (struct video_inst *)handle;
	struct mtk_camera_mem *mem = (struct mtk_camera_mem *)fb;
	struct fb_info_in info;
	int i;

	info.index = (uint32_t)(mem->index);
	info.format = (uint32_t)(mem->format);
	info.num_planes = (uint32_t)(mem->num_planes);

	for (i = 0; i < info.num_planes; i++) {
		info.dma_addr[i] = (uint32_t)(mem->planes[i].dma_addr);
		info.dma_size[i] = (uint32_t)(mem->planes[i].size);
		mtk_camera_debug(3, "inst %p, vsi %p, dma_addr[%d] 0x%x",
			inst, inst->vsi, i, mem->planes[i].dma_addr);
	}

	list_add_tail(&mem->list, &inst->queue);

	return camera_send_ap_ipi(inst, AP_IPIMSG_CAM_INIT_BUFFER, &info);
}

int camera_deinit_buffer(unsigned long handle, void *fb)
{
	struct video_inst *inst = (struct video_inst *)handle;
	struct fb_info_in info;
	int i;

	struct mtk_camera_mem *mem = (struct mtk_camera_mem *)fb;
	struct mtk_camera_mem *child, *t;

	mtk_camera_debug(3, "inst %p, vsi %p, fb 0x%x",
		inst, inst->vsi, mem->planes[0].dma_addr);

	info.num_planes = (uint32_t)(mem->num_planes);
	for (i = 0; i < info.num_planes; i++) {
		info.dma_addr[i] = (uint32_t)(mem->planes[i].dma_addr);
		info.dma_size[i] = (uint32_t)(mem->planes[i].size);
	}
	list_for_each_entry_safe(child, t,
							&inst->queue, list) {
		if (child == mem) {
			list_del(&child->list);
			mtk_camera_debug(3, "remove fb 0x%x",
				child->planes[0].dma_addr);
		}
	}

	return camera_send_ap_ipi(inst, AP_IPIMSG_CAM_DEINIT_BUFFER, &info);
}

int camera_start_stream(unsigned long handle)
{
	struct video_inst *inst = (struct video_inst *)handle;

	return camera_send_ap_ipi(inst, AP_IPIMSG_CAM_START_STREAM, NULL);
}

int camera_capture(unsigned long handle, void *fb)
{
	struct video_inst *inst = (struct video_inst *)handle;
	struct mtk_camera_mem *mem = (struct mtk_camera_mem *)fb;
	struct fb_info_in info;
	int i;

	mtk_camera_debug(3, "inst %p, vsi %p, fb 0x%x",
		inst, inst->vsi, mem->planes[0].dma_addr);

	info.index = (uint32_t)(mem->index);
	info.num_planes = (uint32_t)(mem->num_planes);

	for (i = 0; i < info.num_planes; i++) {
		info.dma_addr[i] = (uint32_t)(mem->planes[i].dma_addr);
		info.dma_size[i] = (uint32_t)mem->planes[i].size;
	}

	return camera_send_ap_ipi(inst, AP_IPIMSG_CAM_START, &info);
}

int camera_get_param(unsigned long handle,
		 enum camera_get_param_type type, void *out)
{
	return 0;
}

int camera_set_param(unsigned long handle,
		 enum camera_set_param_type type, void *in)
{
	struct video_inst *inst = (struct video_inst *)handle;
	int ret = 0;

	switch (type) {
	case SET_PARAM_FRAME_SIZE:
		camera_vcu_set_param(inst, (unsigned int)type, in, 3U);
		break;
	case SET_PARAM_BRIGHTNESS:
	case SET_PARAM_CONTRAST:
	case SET_PARAM_SATURATION:
	case SET_PARAM_SHARPNESS:
	case SET_PARAM_HUE:
	case SET_PARAM_GAMMA:
	case SET_PARAM_AUTO_WHITE_BALANCE:
	case SET_PARAM_DO_WHITE_BALANCE:
	case SET_PARAM_WHITE_BALANCE_TEMP:
	case SET_PARAM_EXPOSURE:
	case SET_PARAM_AUTOGAIN:
	case SET_PARAM_GAIN:
	case SET_PARAM_POWER_LINE_FREQ:
	case SET_PARAM_BACKLIGHT_COMPENSATION:
	case SET_PARAM_MIN_FPS:
	case SET_PARAM_MAX_FPS:
	case SET_PARAM_CAMERA_CLASS:
	case SET_PARAM_EXPOSURE_AUTO:
	case SET_PARAM_EXPOSURE_ABSOLUTE:
	case SET_PARAM_EXPOSURE_AUTO_PRI:
	case SET_PARAM_FOCUS_AUTO:
	case SET_PARAM_FOCUS_RELATIVE:
	case SET_PARAM_FOCUS_ABSOLUTE:
	case SET_PARAM_PRIVACY:
	case SET_PARAM_BAND_STOP_FILT:
	case SET_PARAM_PAN_RELEATIVE:
	case SET_PARAM_PAN_RESET:
	case SET_PARAM_PAN_ABSOLUTE:
	case SET_PARAM_TILT_RELATIVE:
	case SET_PARAM_TILT_RESET:
	case SET_PARAM_TILT_ABSOLUTE:
	case SET_PARAM_ZOOM_ABSOLUTE:
	case SET_PARAM_ZOOM_RELATIVE:
	case SET_PARAM_ZOOM_CONTINUOUS:
		camera_vcu_set_param(inst, (unsigned int)type, in, 1U);
		break;

	default:
		mtk_camera_err("invalid set parameter type=%d\n", (int)type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int camera_stop_stream(unsigned long handle)
{
	struct video_inst *inst = (struct video_inst *)handle;

	return camera_send_ap_ipi(inst, AP_IPIMSG_CAM_STOP_STREAM, NULL);
}

void camera_deinit(unsigned long handle)
{
	struct video_inst *inst = (struct video_inst *)handle;

	if (inst != NULL) {
		camera_send_ap_ipi(inst, AP_IPIMSG_CAM_DEINIT, NULL);
		kfree(inst);
	}
}

static struct mtk_camera_if sdk = {
	camera_init,
	camera_capture,
	camera_start_stream,
	camera_init_buffer,
	camera_deinit_buffer,
	camera_get_param,
	camera_set_param,
	camera_stop_stream,
	camera_deinit,
};

struct mtk_camera_if *get_camera_if(void)
{
	return &sdk;
}
