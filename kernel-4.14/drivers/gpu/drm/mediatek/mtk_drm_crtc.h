/*
 * Copyright (c) 2015 MediaTek Inc.
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

#ifndef MTK_DRM_CRTC_H
#define MTK_DRM_CRTC_H

#include <drm/drm_crtc.h>
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_plane.h"
//#include <drm/drm_writeback.h>

#define MTK_LUT_SIZE	512
#define MTK_MAX_BPC	10
#define MTK_MIN_BPC	3

/**
 * struct mtk_drm_crtc - MediaTek specific crtc structure.
 * @base: crtc object.
 * @enabled: records whether crtc_enable succeeded
 * @planes: array of 4 drm_plane structures, one for each overlay plane
 * @pending_planes: whether any plane has pending changes to be applied
 * @config_regs: memory mapped mmsys configuration register space
 * @mutex: handle to one of the ten disp_mutex streams
 * @ddp_comp_nr: number of components in ddp_comp
 * @ddp_comp: array of pointers the mtk_ddp_comp structures used by this crtc
 */
struct mtk_drm_crtc {
	struct drm_crtc			base;
	bool				enabled;

	bool				pending_needs_vblank;
	struct drm_pending_vblank_event	*event;

#ifdef CONFIG_MTK_DISPLAY_CMDQ
	struct cmdq_client		*cmdq_client;
	struct cmdq_base		*mutex_cmdq_base;
	int				cmdq_event;
#endif

	struct drm_plane		*planes;
	unsigned int			layer_nr;
	bool				pending_planes;

	void __iomem			*config_regs;
	const struct mtk_mmsys_reg_data *mmsys_reg_data;
	struct mtk_disp_mutex		*mutex;
	unsigned int			ddp_comp_nr;
	struct mtk_ddp_comp		**ddp_comp;
};

int mtk_drm_crtc_enable_vblank(struct drm_device *drm, unsigned int pipe);
void mtk_drm_crtc_disable_vblank(struct drm_device *drm, unsigned int pipe);
void mtk_drm_crtc_commit(struct drm_crtc *crtc);
void mtk_crtc_ddp_irq(struct drm_crtc *crtc, struct mtk_ddp_comp *ovl);
int mtk_drm_crtc_create(struct drm_device *drm_dev,
			const enum mtk_ddp_comp_id *path,
			unsigned int path_len);
#ifdef CONFIG_MTK_DISPLAY_CMDQ
void mtk_drm_crtc_plane_update(struct drm_crtc *crtc, struct drm_plane *plane,
			       struct mtk_plane_pending_state *pending);
#endif
#endif /* MTK_DRM_CRTC_H */
