/*
 * Copyright (c) 2015 MediaTek Inc.
 * Authors:
 *	YT Shen <yt.shen@mediatek.com>
 *	CK Hu <ck.hu@mediatek.com>
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include "mtk_drm_drv.h"
#include "mtk_drm_plane.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_debugfs.h"
#ifdef CONFIG_MTK_DISPLAY_CMDQ
#include <linux/soc/mediatek/mtk-cmdq.h>
#endif
#include "mtk_drm_crtc.h"

#define DISP_OD_EN				0x0000
#define DISP_OD_INTEN				0x0008
#define DISP_OD_INTSTA				0x000c
#define DISP_OD_CFG				0x0020
#define DISP_OD_SIZE				0x0030
#define DISP_DITHER_5				0x0114
#define DISP_DITHER_7				0x011c
#define DISP_DITHER_15				0x013c
#define DISP_DITHER_16				0x0140

#define DISP_REG_UFO_START			0x0000

#define DISP_CCORR_EN				0x0000
#define CCORR_EN				BIT(0)
#define DISP_CCORR_CFG				0x0020
#define CCORR_RELAY_MODE			BIT(0)
#define DISP_CCORR_SIZE				0x0030

#define DISP_DITHER_EN				0x0000
#define DITHER_EN				BIT(0)
#define DISP_DITHER_CFG				0x0020
#define DITHER_RELAY_MODE			BIT(0)
#define DISP_DITHER_SIZE			0x0030

#define DISP_AAL_EN				0x0000
#define DISP_AAL_CFG				0x0020
#define AAL_RELAY_MODE				BIT(0)

#define DISP_AAL_SIZE				0x0030

#define DISP_GAMMA_EN				0x0000
#define DISP_GAMMA_CFG				0x0020
#define GAMMA_RELAY_MODE			BIT(0)
#define DISP_GAMMA_SIZE				0x0030
#define DISP_GAMMA_LUT				0x0700

#define LUT_10BIT_MASK				0x03ff

#define DISP_REG_WDMA_INTSTA			0x0004
#define DISP_REG_WDMA_EN			0x0008
#define WDMA_EN					BIT(0)
#define DISP_REG_WDMA_RST			0x000c
#define DISP_REG_WDMA_CFG			0x0014
#define DISP_REG_WDMA_SRC_SIZE			0x0018
#define DISP_REG_WDMA_CLIP_SIZE			0x001c
#define DISP_REG_WDMA_DST_WIN_BYTE		0x0028
#define DISP_REG_WDMA_BUF_CON1			0x0038
#define WDMA_ULTRA_ENABLE			BIT(31)
#define WDMA_PRE_ULTRA_ENABLE			BIT(30)

#define OD_RELAYMODE				BIT(0)

#define UFO_BYPASS				BIT(2)

#define AAL_EN					BIT(0)

#define GAMMA_EN				BIT(0)
#define GAMMA_LUT_EN				BIT(1)

#define DISP_DITHERING				BIT(2)
#define DISP_DITHER_ENGINE_EN			BIT(1)
#define DITHER_LSB_ERR_SHIFT_R(x)		(((x) & 0x7) << 28)
#define DITHER_OVFLW_BIT_R(x)			(((x) & 0x7) << 24)
#define DITHER_ADD_LSHIFT_R(x)			(((x) & 0x7) << 20)
#define DITHER_ADD_RSHIFT_R(x)			(((x) & 0x7) << 16)
#define DITHER_NEW_BIT_MODE			BIT(0)
#define DITHER_LSB_ERR_SHIFT_B(x)		(((x) & 0x7) << 28)
#define DITHER_OVFLW_BIT_B(x)			(((x) & 0x7) << 24)
#define DITHER_ADD_LSHIFT_B(x)			(((x) & 0x7) << 20)
#define DITHER_ADD_RSHIFT_B(x)			(((x) & 0x7) << 16)
#define DITHER_LSB_ERR_SHIFT_G(x)		(((x) & 0x7) << 12)
#define DITHER_OVFLW_BIT_G(x)			(((x) & 0x7) << 8)
#define DITHER_ADD_LSHIFT_G(x)			(((x) & 0x7) << 4)
#define DITHER_ADD_RSHIFT_G(x)			(((x) & 0x7) << 0)

void mtk_ddp_write(struct mtk_ddp_comp *comp, unsigned int value,
		   unsigned int offset, void *handle)
{
#ifdef CONFIG_MTK_DISPLAY_CMDQ
	if (handle)
		cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
			       (dma_addr_t)(comp->regs_pa + offset),
			       value, 0xffffffff);
	else
		writel(value, comp->regs + offset);
#else
	writel(value, comp->regs + offset);
#endif
}

void mtk_ddp_write_relaxed(struct mtk_ddp_comp *comp, unsigned int value,
			   unsigned int offset, void *handle)
{
#ifdef CONFIG_MTK_DISPLAY_CMDQ
	if (handle)
		cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
			       (dma_addr_t)(comp->regs_pa + offset),
			       value, 0xffffffff);
	else
		writel_relaxed(value, comp->regs + offset);
#else
	writel_relaxed(value, comp->regs + offset);
#endif
}

void mtk_ddp_write_mask(struct mtk_ddp_comp *comp, unsigned int value,
			unsigned int offset, unsigned int mask, void *handle)
{
#ifdef CONFIG_MTK_DISPLAY_CMDQ
	if (handle)
		cmdq_pkt_write((struct cmdq_pkt *)handle, comp->cmdq_base,
			       (dma_addr_t)(comp->regs_pa + offset),
			       value, mask);
	else {
		unsigned int tmp = readl(comp->regs + offset);

		tmp = (tmp & ~mask) | (value & mask);
		writel(tmp, comp->regs + offset);
	}
#else
	unsigned int tmp = readl(comp->regs + offset);

	tmp = (tmp & ~mask) | (value & mask);
	writel(tmp, comp->regs + offset);
#endif
}

void mtk_dither_set(struct mtk_ddp_comp *comp, unsigned int bpc,
		    unsigned int CFG, void *handle)
{
	/* If bpc equal to 0, the dithering function didn't be enabled */
	if (bpc == 0)
		return;

	if (bpc >= MTK_MIN_BPC) {
		mtk_ddp_write(comp, 0, DISP_DITHER_5, handle);
		mtk_ddp_write(comp, 0, DISP_DITHER_7, handle);
		mtk_ddp_write(comp,
			DITHER_LSB_ERR_SHIFT_R(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_R(MTK_MAX_BPC - bpc) |
			DITHER_NEW_BIT_MODE,
			DISP_DITHER_15, handle);
		mtk_ddp_write(comp,
			DITHER_LSB_ERR_SHIFT_B(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_B(MTK_MAX_BPC - bpc) |
			DITHER_LSB_ERR_SHIFT_G(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_G(MTK_MAX_BPC - bpc),
			DISP_DITHER_16, handle);
		mtk_ddp_write_mask(comp, DISP_DITHER_ENGINE_EN,
				   CFG, DISP_DITHER_ENGINE_EN, handle);
	}
}

void mtk_dither_set_in_gamma(struct mtk_ddp_comp *comp, unsigned int bpc,
			     unsigned int CFG, void *handle)
{
	/* If bpc equal to 0, the dithering function didn't be enabled */
	if (bpc == 0)
		return;

	if (bpc >= MTK_MIN_BPC) {
		mtk_ddp_write(comp, 0, DISP_DITHER_5, handle);
		mtk_ddp_write(comp, 0, DISP_DITHER_7, handle);
		mtk_ddp_write(comp,
			DITHER_LSB_ERR_SHIFT_R(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_R(MTK_MAX_BPC - bpc) |
			DITHER_NEW_BIT_MODE,
			DISP_DITHER_15, handle);
		mtk_ddp_write(comp,
			DITHER_LSB_ERR_SHIFT_B(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_B(MTK_MAX_BPC - bpc) |
			DITHER_LSB_ERR_SHIFT_G(MTK_MAX_BPC - bpc) |
			DITHER_ADD_LSHIFT_G(MTK_MAX_BPC - bpc),
			DISP_DITHER_16, handle);
		mtk_ddp_write_mask(comp, DISP_DITHERING,
				   CFG, DISP_DITHERING, handle);
	}
}


static void mtk_od_config(struct mtk_ddp_comp *comp, unsigned int w,
			  unsigned int h, unsigned int vrefresh,
			  unsigned int bpc, void *handle)
{
	mtk_ddp_write(comp, w << 16 | h, DISP_OD_SIZE, handle);
	mtk_ddp_write(comp, OD_RELAYMODE, DISP_OD_CFG, handle);
	mtk_dither_set(comp, bpc, DISP_OD_CFG, handle);
}

static void mtk_od_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, 1, DISP_OD_EN, handle);
}

static void mtk_ufoe_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, UFO_BYPASS, DISP_REG_UFO_START, handle);
}

static void mtk_aal_config(struct mtk_ddp_comp *comp, unsigned int w,
			   unsigned int h, unsigned int vrefresh,
			   unsigned int bpc, void *handle)
{
	mtk_ddp_write(comp, h << 16 | w, DISP_AAL_SIZE, handle);
}

static void mtk_aal_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, AAL_EN, DISP_AAL_EN, handle);
}

static void mtk_aal_stop(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write_relaxed(comp, 0x0, DISP_AAL_EN, handle);
}

static void mtk_ccorr_config(struct mtk_ddp_comp *comp, unsigned int w,
			     unsigned int h, unsigned int vrefresh,
			     unsigned int bpc, void *handle)
{
	mtk_ddp_write(comp, h << 16 | w, DISP_CCORR_SIZE, handle);
	mtk_ddp_write(comp, CCORR_RELAY_MODE, DISP_CCORR_CFG, handle);
}

static void mtk_ccorr_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, CCORR_EN, DISP_CCORR_EN, handle);
}

static void mtk_ccorr_stop(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write_relaxed(comp, 0x0, DISP_CCORR_EN, handle);
}

static void mtk_dither_config(struct mtk_ddp_comp *comp, unsigned int w,
			      unsigned int h, unsigned int vrefresh,
			      unsigned int bpc, void *handle)
{
	mtk_ddp_write(comp, h << 16 | w, DISP_DITHER_SIZE, handle);
	mtk_ddp_write_mask(comp, DITHER_RELAY_MODE, DISP_DITHER_CFG,
			   DITHER_RELAY_MODE, handle);
	mtk_dither_set(comp, bpc, DISP_DITHER_CFG, handle);
}

static void mtk_dither_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, DITHER_EN, DISP_DITHER_EN, handle);
}

static void mtk_dither_stop(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write_relaxed(comp, 0x0, DISP_DITHER_EN, handle);
}

static void mtk_gamma_config(struct mtk_ddp_comp *comp, unsigned int w,
			     unsigned int h, unsigned int vrefresh,
			     unsigned int bpc, void *handle)
{
	mtk_ddp_write(comp, h << 16 | w, DISP_GAMMA_SIZE, handle);
	mtk_dither_set_in_gamma(comp, bpc, DISP_GAMMA_CFG, handle);
}

static void mtk_gamma_start(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write(comp, GAMMA_EN, DISP_GAMMA_EN, handle);
}

static void mtk_gamma_stop(struct mtk_ddp_comp *comp, void *handle)
{
	mtk_ddp_write_relaxed(comp, 0x0, DISP_GAMMA_EN, handle);
}

static void mtk_gamma_set(struct mtk_ddp_comp *comp,
			  struct drm_crtc_state *state, void *handle)
{
	unsigned int i, reg;
	struct drm_color_lut *lut;
	void __iomem *lut_base;
	u32 word;

	if (state->gamma_lut) {
		reg = readl(comp->regs + DISP_GAMMA_CFG);
		reg = reg | GAMMA_LUT_EN;
		writel(reg, comp->regs + DISP_GAMMA_CFG);
		lut_base = comp->regs + DISP_GAMMA_LUT;
		lut = (struct drm_color_lut *)state->gamma_lut->data;
		for (i = 0; i < MTK_LUT_SIZE; i++) {
			word = (((lut[i].red >> 6) & LUT_10BIT_MASK) << 20) +
				(((lut[i].green >> 6) & LUT_10BIT_MASK) << 10) +
				((lut[i].blue >> 6) & LUT_10BIT_MASK);
			writel(word, (lut_base + i * 4));
		}
	}
}

static const struct mtk_ddp_comp_funcs ddp_aal = {
	.config = mtk_aal_config,
	.start = mtk_aal_start,
	.stop = mtk_aal_stop,
};

static const struct mtk_ddp_comp_funcs ddp_gamma = {
	.gamma_set = mtk_gamma_set,
	.config = mtk_gamma_config,
	.start = mtk_gamma_start,
	.stop = mtk_gamma_stop,
};

static const struct mtk_ddp_comp_funcs ddp_od = {
	.config = mtk_od_config,
	.start = mtk_od_start,
};

static const struct mtk_ddp_comp_funcs ddp_ufoe = {
	.start = mtk_ufoe_start,
};

static const struct mtk_ddp_comp_funcs ddp_ccorr = {
	.config = mtk_ccorr_config,
	.start = mtk_ccorr_start,
	.stop = mtk_ccorr_stop,

};

static const struct mtk_ddp_comp_funcs ddp_dither = {
	.config = mtk_dither_config,
	.start = mtk_dither_start,
	.stop = mtk_dither_stop,
};

static const char * const mtk_ddp_comp_stem[MTK_DDP_COMP_TYPE_MAX] = {
	[MTK_DISP_OVL] = "ovl",
	[MTK_DISP_RDMA] = "rdma",
	[MTK_DISP_WDMA] = "wdma",
	[MTK_DISP_COLOR] = "color",
	[MTK_DISP_AAL] = "aal",
	[MTK_DISP_GAMMA] = "gamma",
	[MTK_DISP_UFOE] = "ufoe",
	[MTK_DSI] = "dsi",
	[MTK_DPI] = "dpi",
	[MTK_DISP_PWM] = "pwm",
	[MTK_DISP_MUTEX] = "mutex",
	[MTK_DISP_OD] = "od",
	[MTK_DISP_BLS] = "bls",
	[MTK_DISP_CCORR] = "ccorr",
	[MTK_DISP_DITHER] = "dither",
};

struct mtk_ddp_comp_match {
	enum mtk_ddp_comp_type type;
	int alias_id;
	const struct mtk_ddp_comp_funcs *funcs;
};

static const struct mtk_ddp_comp_match mtk_ddp_matches[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL]	= { MTK_DISP_AAL,	0, &ddp_aal },
	[DDP_COMPONENT_BLS]	= { MTK_DISP_BLS,	0, NULL },
	[DDP_COMPONENT_CCORR]  = { MTK_DISP_CCORR,	0, &ddp_ccorr },
	[DDP_COMPONENT_COLOR0]	= { MTK_DISP_COLOR,	0, NULL },
	[DDP_COMPONENT_COLOR1]	= { MTK_DISP_COLOR,	1, NULL },
	[DDP_COMPONENT_DITHER]	= { MTK_DISP_DITHER,	0, &ddp_dither },
	[DDP_COMPONENT_DPI0]	= { MTK_DPI,		0, NULL },
	[DDP_COMPONENT_DSI0]	= { MTK_DSI,		0, NULL },
	[DDP_COMPONENT_DSI1]	= { MTK_DSI,		1, NULL },
	[DDP_COMPONENT_GAMMA]	= { MTK_DISP_GAMMA,	0, &ddp_gamma },
	[DDP_COMPONENT_OD]	= { MTK_DISP_OD,	0, &ddp_od },
	[DDP_COMPONENT_OVL0]	= { MTK_DISP_OVL,	0, NULL },
	[DDP_COMPONENT_OVL1]	= { MTK_DISP_OVL,	1, NULL },
	[DDP_COMPONENT_PWM0]	= { MTK_DISP_PWM,	0, NULL },
	[DDP_COMPONENT_RDMA0]	= { MTK_DISP_RDMA,	0, NULL },
	[DDP_COMPONENT_RDMA1]	= { MTK_DISP_RDMA,	1, NULL },
	[DDP_COMPONENT_RDMA2]	= { MTK_DISP_RDMA,	2, NULL },
	[DDP_COMPONENT_UFOE]	= { MTK_DISP_UFOE,	0, &ddp_ufoe },
	[DDP_COMPONENT_WDMA0]	= { MTK_DISP_WDMA,	0, NULL },
	[DDP_COMPONENT_WDMA1]	= { MTK_DISP_WDMA,	1, NULL },
};

int mtk_ddp_comp_get_id(struct device_node *node,
			enum mtk_ddp_comp_type comp_type)
{
	int id = of_alias_get_id(node, mtk_ddp_comp_stem[comp_type]);
	int i;

	for (i = 0; i < ARRAY_SIZE(mtk_ddp_matches); i++) {
		if (comp_type == mtk_ddp_matches[i].type &&
		    (id < 0 || id == mtk_ddp_matches[i].alias_id))
			return i;
	}

	return -EINVAL;
}

enum mtk_ddp_comp_type mtk_ddp_comp_get_type(enum mtk_ddp_comp_id comp_id)
{
	if (comp_id < DDP_COMPONENT_ID_MAX)
		return mtk_ddp_matches[comp_id].type;

	return MTK_DDP_COMP_TYPE_MAX;
}

int mtk_ddp_comp_init(struct device *dev, struct device_node *node,
		      struct mtk_ddp_comp *comp, enum mtk_ddp_comp_id comp_id,
		      const struct mtk_ddp_comp_funcs *funcs)
{
	enum mtk_ddp_comp_type type;
	struct device_node *larb_node;
	struct platform_device *larb_pdev;
	struct platform_device *comp_pdev;
	struct resource res;

	if (comp_id < 0 || comp_id >= DDP_COMPONENT_ID_MAX)
		return -EINVAL;

	type = mtk_ddp_matches[comp_id].type;

	comp->id = comp_id;
	comp->funcs = funcs ?: mtk_ddp_matches[comp_id].funcs;

	if (comp_id == DDP_COMPONENT_BLS ||
	    comp_id == DDP_COMPONENT_DPI0 ||
	    comp_id == DDP_COMPONENT_DSI0 ||
	    comp_id == DDP_COMPONENT_PWM0) {
	    comp->regs_pa = 0;
		comp->regs = NULL;
		comp->clk = NULL;
		comp->irq = 0;
		return 0;
	}

	if (of_address_to_resource(node, 0, &res) != 0) {
		dev_info(dev, "Missing reg in %s node\n", node->full_name);
		return -EINVAL;
	}

	comp->regs_pa = res.start;
	comp->regs = of_iomap(node, 0);
	comp->irq = of_irq_get(node, 0);
	comp->clk = of_clk_get(node, 0);
	if (IS_ERR(comp->clk))
		comp->clk = NULL;

	comp_pdev = of_find_device_by_node(node);
	if (!comp_pdev) {
		dev_info(dev, "Waiting for device %s\n",
			 node->full_name);
		return -EPROBE_DEFER;
	}
	comp->dev = &comp_pdev->dev;

	/* Only DMA capable components need the LARB property */
	comp->larb_dev = NULL;
	if (type != MTK_DISP_OVL &&
	    type != MTK_DISP_RDMA &&
	    type != MTK_DISP_WDMA)
		return 0;

	larb_node = of_parse_phandle(node, "mediatek,larb", 0);
	if (!larb_node) {
		dev_err(dev,
			"Missing mediadek,larb phandle in %pOF node\n", node);
		return -EINVAL;
	}

	larb_pdev = of_find_device_by_node(larb_node);
	if (!larb_pdev) {
		dev_warn(dev, "Waiting for larb device %pOF\n", larb_node);
		of_node_put(larb_node);
		return -EPROBE_DEFER;
	}
	of_node_put(larb_node);

	comp->larb_dev = &larb_pdev->dev;

	return 0;
}

int mtk_ddp_comp_register(struct drm_device *drm, struct mtk_ddp_comp *comp)
{
	struct mtk_drm_private *private = drm->dev_private;

	if (private->ddp_comp[comp->id])
		return -EBUSY;

	private->ddp_comp[comp->id] = comp;
	return 0;
}

void mtk_ddp_comp_unregister(struct drm_device *drm, struct mtk_ddp_comp *comp)
{
	struct mtk_drm_private *private = drm->dev_private;

	private->ddp_comp[comp->id] = NULL;
}
