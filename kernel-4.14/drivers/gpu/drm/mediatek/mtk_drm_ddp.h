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

#ifndef MTK_DRM_DDP_H
#define MTK_DRM_DDP_H

#include "mtk_drm_ddp_comp.h"

struct regmap;
struct device;
struct mtk_disp_mutex;
struct mtk_mmsys_reg_data;
enum mtk_mmsys_id {
	MMSYS_MT2701,
	MMSYS_MT8173,
	MMSYS_MT8168,
	MMSYS_MAX,
};

const struct mtk_mmsys_reg_data *mtk_ddp_get_mmsys_data(enum mtk_mmsys_id id);
void mtk_ddp_add_comp_to_path(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *reg_data,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next);
void mtk_ddp_remove_comp_from_path(void __iomem *config_regs,
				   const struct mtk_mmsys_reg_data *reg_data,
				   enum mtk_ddp_comp_id cur,
				   enum mtk_ddp_comp_id next);

struct mtk_disp_mutex *mtk_disp_mutex_get(struct device *dev, unsigned int id);
int mtk_disp_mutex_prepare(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_add_comp(struct mtk_disp_mutex *mutex,
			     enum mtk_ddp_comp_id id);
void mtk_disp_mutex_enable(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_disable(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_remove_comp(struct mtk_disp_mutex *mutex,
				enum mtk_ddp_comp_id id);
void mtk_disp_mutex_unprepare(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_put(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_acquire(struct mtk_disp_mutex *mutex);
void mtk_disp_mutex_release(struct mtk_disp_mutex *mutex);

#endif /* MTK_DRM_DDP_H */
