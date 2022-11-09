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

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_debugfs.h"

#define DISP_REG_CONFIG_DISP_OVL0_MOUT_EN	0x040
#define DISP_REG_CONFIG_DISP_OVL1_MOUT_EN	0x044
#define DISP_REG_CONFIG_DISP_OD_MOUT_EN		0x048
#define DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN	0x04c
#define DISP_REG_CONFIG_DISP_UFOE_MOUT_EN	0x050
#define DISP_REG_CONFIG_DISP_COLOR0_SEL_IN	0x084
#define DISP_REG_CONFIG_DISP_COLOR1_SEL_IN	0x088
#define DISP_REG_CONFIG_DPI_SEL_IN		0x0ac
#define DISP_REG_CONFIG_DISP_RDMA1_MOUT_EN	0x0c8
#define DISP_REG_CONFIG_MMSYS_CG_CON0		0x100

#define DISP_REG_CONFIG_DISP_OVL_MOUT_EN	0x030
#define DISP_REG_CONFIG_OUT_SEL			0x04c
#define DISP_REG_CONFIG_DSI_SEL			0x050

#define MT8168_DISP_OVL0_MOUT_EN		0xf3c
#define MT8168_DISP_RSZ0_MOUT_EN		0xf44
#define MT8168_DISP_DITHER0_MOUT_EN		0xf50

#define MT8168_DISP_PATH0_SEL_IN		0xf54
#define MT8168_DISP_RSZ0_SEL_IN			0xf58
#define MT8168_DISP_RDMA0_RSZ0_SEL_IN		0xf60
#define MT8168_DISP_COLOR0_OUT_SEL_IN		0xf64
#define MT8168_DSI0_SEL_IN			0xf68
#define MT8168_DISP_WDMA0_SEL_IN		0xf6c
#define MT8168_DPI0_SEL_IN			0xfd8

#define MT8168_DISP_RDMA0_RSZ0_IN_SOUT_SEL	0xf48
#define MT8168_DISP_RDMA0_SOUT_SEL		0xf4c
#define MT8168_DISP_RDMA1_SOUT_SEL		0xfd0

#define MT8173_DISP_MUTEX0_MOD0			0x2C
#define MT8173_DISP_MUTEX0_SOF0			0x30
#define MT8168_DISP_MUTEX0_MOD0			0x30
#define MT8168_DISP_MUTEX0_SOF0			0x2C

#define DISP_REG_MUTEX_EN(n)			(0x20 + 0x20 * (n))
#define DISP_REG_MUTEX(n)			(0x24 + 0x20 * (n))
#define DISP_REG_MUTEX_RST(n)			(0x28 + 0x20 * (n))
#define DISP_REG_MUTEX_MOD(data, n)	((data)->mutex_mod_reg + 0x20 * (n))
#define DISP_REG_MUTEX_SOF(data, n)	((data)->mutex_sof_reg + 0x20 * (n))

#define INT_MUTEX				BIT(1)

#define MT8173_MUTEX_MOD_DISP_OVL0		BIT(11)
#define MT8173_MUTEX_MOD_DISP_OVL1		BIT(12)
#define MT8173_MUTEX_MOD_DISP_RDMA0		BIT(13)
#define MT8173_MUTEX_MOD_DISP_RDMA1		BIT(14)
#define MT8173_MUTEX_MOD_DISP_RDMA2		BIT(15)
#define MT8173_MUTEX_MOD_DISP_WDMA0		BIT(16)
#define MT8173_MUTEX_MOD_DISP_WDMA1		BIT(17)
#define MT8173_MUTEX_MOD_DISP_COLOR0		BIT(18)
#define MT8173_MUTEX_MOD_DISP_COLOR1		BIT(19)
#define MT8173_MUTEX_MOD_DISP_AAL		BIT(20)
#define MT8173_MUTEX_MOD_DISP_GAMMA		BIT(21)
#define MT8173_MUTEX_MOD_DISP_UFOE		BIT(22)
#define MT8173_MUTEX_MOD_DISP_PWM0		BIT(23)
#define MT8173_MUTEX_MOD_DISP_PWM1		BIT(24)
#define MT8173_MUTEX_MOD_DISP_OD		BIT(25)

#define MT2701_MUTEX_MOD_DISP_OVL		BIT(3)
#define MT2701_MUTEX_MOD_DISP_WDMA		BIT(6)
#define MT2701_MUTEX_MOD_DISP_COLOR		BIT(7)
#define MT2701_MUTEX_MOD_DISP_BLS		BIT(9)
#define MT2701_MUTEX_MOD_DISP_RDMA0		BIT(10)
#define MT2701_MUTEX_MOD_DISP_RDMA1		BIT(12)

#define MT8168_MUTEX_MOD_DISP_OVL0		BIT(7)
#define MT8168_MUTEX_MOD_DISP_RDMA0		BIT(9)
#define MT8168_MUTEX_MOD_DISP_RDMA1		BIT(10)
#define MT8168_MUTEX_MOD_DISP_WDMA0		BIT(11)
#define MT8168_MUTEX_MOD_DISP_COLOR0		BIT(12)
#define MT8168_MUTEX_MOD_DISP_CCORR		BIT(13)
#define MT8168_MUTEX_MOD_DISP_AAL		BIT(14)
#define MT8168_MUTEX_MOD_DISP_GAMMA		BIT(15)
#define MT8168_MUTEX_MOD_DISP_DITHER		BIT(16)
#define MT8168_MUTEX_MOD_DISP_DSI0		BIT(17)
#define MT8168_MUTEX_MOD_DISP_RSZ0		BIT(19)
#define MT8168_MUTEX_MOD_DISP_PWM0		BIT(20)
#define MT8168_MUTEX_MOD_DISP_DPI0		BIT(22)

#define MUTEX_SOF_SINGLE_MODE			0
#define MUTEX_SOF_DSI0				1
#define MUTEX_SOF_DSI1				2
#define MUTEX_SOF_DPI0				2/*8168 3*/
#define MUTEX_EOF_DSI0				BIT(6) /*1<<6*/
#define MUTEX_EOF_DPI0				BIT(7) /*2<<6*/

#define MT8168_DISP_MUTEX_CFG			0x8
#define MUTEX_DISABLE_CLK_GATING		0x0
#define MUTEX_ENABLE_CLK_GATING			0x1

#define OVL0_MOUT_EN_COLOR0			0x1
#define OD_MOUT_EN_RDMA0			0x1
#define UFOE_MOUT_EN_DSI0			0x1
#define COLOR0_SEL_IN_OVL0			0x1
#define OVL1_MOUT_EN_COLOR1			0x1
#define GAMMA_MOUT_EN_RDMA1			0x1
#define RDMA1_MOUT_DPI0				0x2
#define DPI0_SEL_IN_RDMA1			0x1
#define COLOR1_SEL_IN_OVL1			0x1

#define OVL_MOUT_EN_RDMA			0x1
#define BLS_TO_DSI_RDMA1_TO_DPI1		0x8
#define DSI_SEL_IN_BLS				0x0

#define MT8168_OVL0_MOUT_EN_PATH0_SEL		0x1
#define MT8168_DITHER0_MOUT_EN_DSI0		0x1

#define MT8168_PATH0_SEL_IN_OVL0		0x0
#define MT8168_RDMA0_RSZ0_SEL_IN_RDMA0		0x0
#define MT8168_COLOR0_OUT_SEL_IN_COLOR0		0x0
#define MT8168_COLOR0_OUT_SEL_IN_RDMA0_SOUT	0x1
#define MT8168_DSI0_SEL_IN_RDMA0_SOUT		0x0
#define MT8168_DSI0_SEL_IN_DITHER0		0x1
#define MT8168_DPI0_SEL_IN_RDMA1		0x0

#define MT8168_RDMA0_SOUT_DSI0			0x0
#define MT8168_RDMA0_SOUT_COLOR0		0x1
#define MT8168_RDMA0_SOUT_COLOR0_OUT_SEL	0x2
#define MT8168_RDMA0_SOUT_DPI0			0x3
#define MT8168_RDMA0_RSZ0_IN_SOUT_SEL		0x0

#define DISP_REG_OVL0_MOUT_EN(data)		((data)->ovl0_mout_en)
#define DISP_REG_OVL0_MOUT_EN_COLOR0(data)	((data)->ovl0_mout_en_color0)
#define DISP_REG_OVL0_MOUT_EN_RDMA0(data)	((data)->ovl0_mout_en_rdma0)
#define DISP_REG_DITHER0_MOUT_EN(data)		((data)->dither0_mout_en)
#define DISP_REG_DITHER0_MOUT_EN_DSI0(data)	((data)->dither0_mout_en_dsi0)

#define DISP_REG_DPI0_SEL_IN(data)		((data)->dpi0_sel_in)
#define DISP_REG_DPI0_SEL_IN_RDMA1(data)	((data)->dpi0_sel_in_rdma1)
#define DISP_REG_RDMA0_SEL_IN(data)		((data)->rdma0_sel_in)
#define DISP_REG_RDMA0_SEL_IN_OVL0(data)	((data)->rdma0_sel_in_ovl0)
#define DISP_REG_RDMA0_SOUT_SEL_IN(data)	((data)->rdma0_sout_sel_in)
#define DISP_REG_RDMA0_SOUT_SEL_IN_RDMA0(data)	\
					((data)->rdma0_sout_sel_in_rdma0)
#define DISP_REG_DSI0_SEL_IN(data)		((data)->dsi0_sel_in)
#define DISP_REG_DSI0_SEL_IN_DITHER0(data)	((data)->dsi0_sel_in_dither0)
#define DISP_REG_DSI0_SEL_IN_RDMA0_SOUT(data)	((data)->dsi0_sel_in_rdma0_sout)
#define DISP_REG_CCORR0_SEL_IN(data)		((data)->ccorr0_sel_in)
#define DISP_REG_CCORR0_SEL_IN_COLOR0(data)	((data)->ccorr0_sel_in_color0)
#define DISP_REG_CCORR0_SEL_IN_RDMA0_SOUT(data)	\
					((data)->ccorr0_sel_in_rdma0_sout)
#define DISP_REG_RDMA0_SOUT(data)		((data)->rdma0_sout)
#define DISP_REG_RDMA0_SOUT_DSI0(data)		((data)->rdma0_sout_dsi0)
#define DISP_REG_RDMA0_SOUT_COLOR0(data)	((data)->rdma0_sout_color0)
#define DISP_REG_RDMA0_SOUT_CCORR(data)		((data)->rdma0_sout_ccorr)
#define DISP_REG_RDMA0_SOUT_DPI0(data)		((data)->rdma0_sout_dpi0)

struct mtk_disp_mutex {
	int id;
	bool claimed;
};

enum mtk_ddp_mutex_sof_id {
	DDP_MUTEX_SOF_SINGLE_MODE,
	DDP_MUTEX_SOF_DSI0,
	DDP_MUTEX_SOF_DSI1,
	DDP_MUTEX_SOF_DPI0,
	DDP_MUTEX_SOF_MAX,
};

struct mtk_ddp_data {
	const unsigned int *mutex_mod;
	const unsigned int *mutex_sof;
	unsigned int mutex_mod_reg;
	unsigned int mutex_sof_reg;
	bool clk_gating_config;
};

struct mtk_ddp {
	struct device			*dev;
	struct clk			*clk;
	void __iomem			*regs;
	struct mtk_disp_mutex		mutex[10];
	const struct mtk_ddp_data	*data;
};

struct mtk_mmsys_reg_data {
	u32 ovl0_mout_en;
	u32 ovl0_mout_en_color0;
	u32 ovl0_mout_en_rdma0;
	u32 dither0_mout_en;
	u32 dither0_mout_en_dsi0;

	u32 dpi0_sel_in;
	u32 dpi0_sel_in_rdma1;
	u32 rdma0_sel_in;
	u32 rdma0_sel_in_ovl0;
	u32 rdma0_sout_sel_in;
	u32 rdma0_sout_sel_in_rdma0;
	u32 ccorr0_sel_in;
	u32 ccorr0_sel_in_color0;
	u32 ccorr0_sel_in_rdma0_sout;
	u32 dsi0_sel_in;
	u32 dsi0_sel_in_dither0;
	u32 dsi0_sel_in_rdma0_sout;

	u32 rdma0_sout;
	u32 rdma0_sout_dsi0;
	u32 rdma0_sout_color0;
	u32 rdma0_sout_ccorr;
	u32 rdma0_sout_dpi0;
};

static const unsigned int mt2701_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_BLS] = MT2701_MUTEX_MOD_DISP_BLS,
	[DDP_COMPONENT_COLOR0] = MT2701_MUTEX_MOD_DISP_COLOR,
	[DDP_COMPONENT_OVL0] = MT2701_MUTEX_MOD_DISP_OVL,
	[DDP_COMPONENT_RDMA0] = MT2701_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT2701_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_WDMA0] = MT2701_MUTEX_MOD_DISP_WDMA,
};

static const unsigned int mt8173_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL] = MT8173_MUTEX_MOD_DISP_AAL,
	[DDP_COMPONENT_COLOR0] = MT8173_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_COLOR1] = MT8173_MUTEX_MOD_DISP_COLOR1,
	[DDP_COMPONENT_GAMMA] = MT8173_MUTEX_MOD_DISP_GAMMA,
	[DDP_COMPONENT_OD] = MT8173_MUTEX_MOD_DISP_OD,
	[DDP_COMPONENT_OVL0] = MT8173_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_OVL1] = MT8173_MUTEX_MOD_DISP_OVL1,
	[DDP_COMPONENT_PWM0] = MT8173_MUTEX_MOD_DISP_PWM0,
	[DDP_COMPONENT_PWM1] = MT8173_MUTEX_MOD_DISP_PWM1,
	[DDP_COMPONENT_RDMA0] = MT8173_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8173_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_RDMA2] = MT8173_MUTEX_MOD_DISP_RDMA2,
	[DDP_COMPONENT_UFOE] = MT8173_MUTEX_MOD_DISP_UFOE,
	[DDP_COMPONENT_WDMA0] = MT8173_MUTEX_MOD_DISP_WDMA0,
	[DDP_COMPONENT_WDMA1] = MT8173_MUTEX_MOD_DISP_WDMA1,
};

static const unsigned int mt8168_mutex_mod[DDP_COMPONENT_ID_MAX] = {
	[DDP_COMPONENT_AAL] = MT8168_MUTEX_MOD_DISP_AAL,
	[DDP_COMPONENT_COLOR0] = MT8168_MUTEX_MOD_DISP_COLOR0,
	[DDP_COMPONENT_CCORR] = MT8168_MUTEX_MOD_DISP_CCORR,
	[DDP_COMPONENT_GAMMA] = MT8168_MUTEX_MOD_DISP_GAMMA,
	[DDP_COMPONENT_DITHER] = MT8168_MUTEX_MOD_DISP_DITHER,
	[DDP_COMPONENT_OVL0] = MT8168_MUTEX_MOD_DISP_OVL0,
	[DDP_COMPONENT_PWM0] = MT8168_MUTEX_MOD_DISP_PWM0,
	[DDP_COMPONENT_RDMA0] = MT8168_MUTEX_MOD_DISP_RDMA0,
	[DDP_COMPONENT_RDMA1] = MT8168_MUTEX_MOD_DISP_RDMA1,
	[DDP_COMPONENT_WDMA0] = MT8168_MUTEX_MOD_DISP_WDMA0,
};

static const unsigned int mt8173_mutex_sof[DDP_MUTEX_SOF_MAX] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0,
	[DDP_MUTEX_SOF_DSI1] = MUTEX_SOF_DSI1,
	[DDP_MUTEX_SOF_DPI0] = MUTEX_SOF_DPI0,
};

static const unsigned int mt8168_mutex_sof[DDP_MUTEX_SOF_MAX] = {
	[DDP_MUTEX_SOF_SINGLE_MODE] = MUTEX_SOF_SINGLE_MODE,
	[DDP_MUTEX_SOF_DSI0] = MUTEX_SOF_DSI0 | MUTEX_EOF_DSI0,
	[DDP_MUTEX_SOF_DSI1] = MUTEX_SOF_DSI1,
	[DDP_MUTEX_SOF_DPI0] = MUTEX_SOF_DPI0 | MUTEX_EOF_DPI0,
};


static const struct mtk_ddp_data mt2701_ddp_driver_data = {
	.mutex_mod = mt2701_mutex_mod,
	.mutex_sof = mt8173_mutex_sof,
	.mutex_mod_reg = MT8173_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT8173_DISP_MUTEX0_SOF0,
	.clk_gating_config = false,
};

static const struct mtk_ddp_data mt8173_ddp_driver_data = {
	.mutex_mod = mt8173_mutex_mod,
	.mutex_sof = mt8173_mutex_sof,
	.mutex_mod_reg = MT8173_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT8173_DISP_MUTEX0_SOF0,
	.clk_gating_config = false,
};

static const struct mtk_ddp_data mt8168_ddp_driver_data = {
	.mutex_mod = mt8168_mutex_mod,
	.mutex_sof = mt8168_mutex_sof,
	.mutex_mod_reg = MT8168_DISP_MUTEX0_MOD0,
	.mutex_sof_reg = MT8168_DISP_MUTEX0_SOF0,
	.clk_gating_config = true,
};

const struct mtk_mmsys_reg_data mt2701_mmsys_reg_data = {
	.ovl0_mout_en = DISP_REG_CONFIG_DISP_OVL_MOUT_EN,
	.ovl0_mout_en_color0 = OVL0_MOUT_EN_COLOR0,
	.ovl0_mout_en_rdma0 = OVL_MOUT_EN_RDMA,
	.dither0_mout_en = 0,
	.dither0_mout_en_dsi0 = 0,

	.dpi0_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.dpi0_sel_in_rdma1 = DPI0_SEL_IN_RDMA1,
	.rdma0_sel_in = 0,
	.rdma0_sel_in_ovl0 = 0,
	.rdma0_sout_sel_in = 0,
	.rdma0_sout_sel_in_rdma0 = 0,
	.ccorr0_sel_in = 0,
	.ccorr0_sel_in_color0 = 0,
	.ccorr0_sel_in_rdma0_sout = 0,
	.dsi0_sel_in = 0,
	.dsi0_sel_in_dither0 = 0,
	.dsi0_sel_in_rdma0_sout = 0,

	.rdma0_sout = 0,
	.rdma0_sout_dsi0 = 0,
	.rdma0_sout_color0 = 0,
	.rdma0_sout_ccorr = 0,
	.rdma0_sout_dpi0 = 0,
};

const struct mtk_mmsys_reg_data mt8173_mmsys_reg_data = {
	.ovl0_mout_en = DISP_REG_CONFIG_DISP_OVL0_MOUT_EN,
	.ovl0_mout_en_color0 = OVL0_MOUT_EN_COLOR0,
	.ovl0_mout_en_rdma0 = OVL_MOUT_EN_RDMA,
	.dither0_mout_en = 0,
	.dither0_mout_en_dsi0 = 0,

	.dpi0_sel_in = DISP_REG_CONFIG_DPI_SEL_IN,
	.dpi0_sel_in_rdma1 = DPI0_SEL_IN_RDMA1,
	.rdma0_sel_in = 0,
	.rdma0_sel_in_ovl0 = 0,
	.rdma0_sout_sel_in = 0,
	.rdma0_sout_sel_in_rdma0 = 0,
	.ccorr0_sel_in = 0,
	.ccorr0_sel_in_color0 = 0,
	.ccorr0_sel_in_rdma0_sout = 0,
	.dsi0_sel_in = 0,
	.dsi0_sel_in_dither0 = 0,
	.dsi0_sel_in_rdma0_sout = 0,

	.rdma0_sout = 0,
	.rdma0_sout_dsi0 = 0,
	.rdma0_sout_color0 = 0,
	.rdma0_sout_ccorr = 0,
	.rdma0_sout_dpi0 = 0,
};

const struct mtk_mmsys_reg_data mt8168_mmsys_reg_data = {
	.ovl0_mout_en = MT8168_DISP_OVL0_MOUT_EN,
	.ovl0_mout_en_color0 = 0,
	.ovl0_mout_en_rdma0 = MT8168_OVL0_MOUT_EN_PATH0_SEL,
	.dither0_mout_en = MT8168_DISP_DITHER0_MOUT_EN,
	.dither0_mout_en_dsi0 = MT8168_DITHER0_MOUT_EN_DSI0,

	.dpi0_sel_in = MT8168_DPI0_SEL_IN,
	.dpi0_sel_in_rdma1 = DPI0_SEL_IN_RDMA1,
	.rdma0_sel_in = MT8168_DISP_PATH0_SEL_IN,
	.rdma0_sel_in_ovl0 = MT8168_PATH0_SEL_IN_OVL0,
	.rdma0_sout_sel_in = MT8168_DISP_RDMA0_RSZ0_SEL_IN,
	.rdma0_sout_sel_in_rdma0 = MT8168_RDMA0_RSZ0_SEL_IN_RDMA0,
	.ccorr0_sel_in = MT8168_DISP_COLOR0_OUT_SEL_IN,
	.ccorr0_sel_in_color0 = MT8168_COLOR0_OUT_SEL_IN_COLOR0,
	.ccorr0_sel_in_rdma0_sout = MT8168_COLOR0_OUT_SEL_IN_RDMA0_SOUT,
	.dsi0_sel_in = MT8168_DSI0_SEL_IN,
	.dsi0_sel_in_dither0 = MT8168_DSI0_SEL_IN_DITHER0,
	.dsi0_sel_in_rdma0_sout = MT8168_DSI0_SEL_IN_RDMA0_SOUT,

	.rdma0_sout = MT8168_DISP_RDMA0_SOUT_SEL,
	.rdma0_sout_dsi0 = MT8168_RDMA0_SOUT_DSI0,
	.rdma0_sout_color0 = MT8168_RDMA0_SOUT_COLOR0,
	.rdma0_sout_ccorr = MT8168_RDMA0_SOUT_COLOR0_OUT_SEL,
	.rdma0_sout_dpi0 = MT8168_RDMA0_SOUT_DPI0,
};

static void mtk_ddp_mout_en(void __iomem *config_regs,
			    const struct mtk_mmsys_reg_data *data,
			    enum mtk_ddp_comp_id cur,
			    enum mtk_ddp_comp_id next,
			    bool connect)
{
	unsigned int value, addr, reg;

	if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_COLOR0) {
		addr = DISP_REG_OVL0_MOUT_EN(data);
		value = DISP_REG_OVL0_MOUT_EN_COLOR0(data);
	} else if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_RDMA0) {
		addr = DISP_REG_OVL0_MOUT_EN(data);
		value = DISP_REG_OVL0_MOUT_EN_RDMA0(data);
	} else if (cur == DDP_COMPONENT_OD && next == DDP_COMPONENT_RDMA0) {
		addr = DISP_REG_CONFIG_DISP_OD_MOUT_EN;
		value = OD_MOUT_EN_RDMA0;
	} else if (cur == DDP_COMPONENT_UFOE && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_CONFIG_DISP_UFOE_MOUT_EN;
		value = UFOE_MOUT_EN_DSI0;
	} else if (cur == DDP_COMPONENT_OVL1 && next == DDP_COMPONENT_COLOR1) {
		addr = DISP_REG_CONFIG_DISP_OVL1_MOUT_EN;
		value = OVL1_MOUT_EN_COLOR1;
	} else if (cur == DDP_COMPONENT_GAMMA && next == DDP_COMPONENT_RDMA1) {
		addr = DISP_REG_CONFIG_DISP_GAMMA_MOUT_EN;
		value = GAMMA_MOUT_EN_RDMA1;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI0) {
		addr = DISP_REG_CONFIG_DISP_RDMA1_MOUT_EN;
		value = RDMA1_MOUT_DPI0;
	} else if (cur == DDP_COMPONENT_DITHER && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_DITHER0_MOUT_EN(data);
		value = DISP_REG_DITHER0_MOUT_EN_DSI0(data);
	} else {
		addr = 0;
		value = 0;
	}
	if (addr) {
		if (connect)
			reg = readl_relaxed(config_regs + addr) | value;
		else
			reg = readl_relaxed(config_regs + addr) & ~value;
		writel_relaxed(reg, config_regs + addr);
	}

}

static void mtk_ddp_sel_in(void __iomem *config_regs,
			   const struct mtk_mmsys_reg_data *data,
			   enum mtk_ddp_comp_id cur,
			   enum mtk_ddp_comp_id next,
			   bool connect)
{
	unsigned int value, addr, reg;

	if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_COLOR0) {
		addr = DISP_REG_CONFIG_DISP_COLOR0_SEL_IN;
		value = COLOR0_SEL_IN_OVL0;
	} else if (cur == DDP_COMPONENT_RDMA1 && next == DDP_COMPONENT_DPI0) {
		addr = DISP_REG_DPI0_SEL_IN(data);
		value = DISP_REG_DPI0_SEL_IN_RDMA1(data);
	} else if (cur == DDP_COMPONENT_OVL1 && next == DDP_COMPONENT_COLOR1) {
		addr = DISP_REG_CONFIG_DISP_COLOR1_SEL_IN;
		value = COLOR1_SEL_IN_OVL1;
	} else if (cur == DDP_COMPONENT_BLS && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_CONFIG_DSI_SEL;
		value = DSI_SEL_IN_BLS;
	} else if (cur == DDP_COMPONENT_OVL0 && next == DDP_COMPONENT_RDMA0) {
		addr = DISP_REG_RDMA0_SEL_IN(data);
		value = DISP_REG_RDMA0_SEL_IN_OVL0(data);
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_COLOR0) {
		addr = DISP_REG_RDMA0_SOUT_SEL_IN(data);
		value = DISP_REG_RDMA0_SOUT_SEL_IN_RDMA0(data);
	} else if (cur == DDP_COMPONENT_COLOR0 && next == DDP_COMPONENT_CCORR) {
		addr = DISP_REG_CCORR0_SEL_IN(data);
		value = DISP_REG_CCORR0_SEL_IN_COLOR0(data);
	} else if (cur == DDP_COMPONENT_DITHER && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_DSI0_SEL_IN(data);
		value = DISP_REG_DSI0_SEL_IN_DITHER0(data);
	} else {
		addr = 0;
		value = 0;
	}
	if (addr) {
		if (connect)
			reg = readl_relaxed(config_regs + addr) | value;
		else
			reg = readl_relaxed(config_regs + addr) & ~value;
		writel_relaxed(reg, config_regs + addr);
	}

}

static void  mtk_ddp_sout_sel(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *data,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next,
			      bool connect)
{
	unsigned int value, addr;

	if (cur == DDP_COMPONENT_BLS && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_CONFIG_OUT_SEL;
		value = BLS_TO_DSI_RDMA1_TO_DPI1;
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_COLOR0) {
		addr = DISP_REG_RDMA0_SOUT(data);
		value = DISP_REG_RDMA0_SOUT_COLOR0(data);
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_CCORR) {
		addr = DISP_REG_RDMA0_SOUT(data);
		value = DISP_REG_RDMA0_SOUT_CCORR(data);
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DSI0) {
		addr = DISP_REG_RDMA0_SOUT(data);
		value =  DISP_REG_RDMA0_SOUT_DSI0(data);
	} else if (cur == DDP_COMPONENT_RDMA0 && next == DDP_COMPONENT_DPI0) {
		addr = DISP_REG_RDMA0_SOUT(data);
		value =  DISP_REG_RDMA0_SOUT_DPI0(data);
	} else {
		addr = 0;
		value = 0;
	}
	if (connect && addr)
		writel_relaxed(value, config_regs + addr);

}

void mtk_ddp_add_comp_to_path(void __iomem *config_regs,
			      const struct mtk_mmsys_reg_data *reg_data,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next)
{
	mtk_ddp_mout_en(config_regs, reg_data, cur, next, true);

	mtk_ddp_sel_in(config_regs, reg_data, cur, next, true);

	mtk_ddp_sout_sel(config_regs, reg_data, cur, next, true);

}

void mtk_ddp_remove_comp_from_path(void __iomem *config_regs,
				   const struct mtk_mmsys_reg_data *reg_data,
				   enum mtk_ddp_comp_id cur,
				   enum mtk_ddp_comp_id next)
{
	mtk_ddp_mout_en(config_regs, reg_data, cur, next, false);

	mtk_ddp_sel_in(config_regs, reg_data, cur, next, false);
}

const struct mtk_mmsys_reg_data *mtk_ddp_get_mmsys_data(enum mtk_mmsys_id id)
{
	const struct mtk_mmsys_reg_data *data = NULL;

	switch (id) {
	case MMSYS_MT2701:
		data = &mt2701_mmsys_reg_data;
		break;
	case MMSYS_MT8173:
		data = &mt8173_mmsys_reg_data;
		break;
	case MMSYS_MT8168:
		data = &mt8168_mmsys_reg_data;
		break;

	default:
		pr_info("mtk drm not support mmsys id %d\n", id);
		break;
	}

	return data;
}

struct mtk_disp_mutex *mtk_disp_mutex_get(struct device *dev, unsigned int id)
{
	struct mtk_ddp *ddp = dev_get_drvdata(dev);

	if (id >= 10)
		return ERR_PTR(-EINVAL);
	if (ddp->mutex[id].claimed)
		return ERR_PTR(-EBUSY);

	ddp->mutex[id].claimed = true;

	return &ddp->mutex[id];
}

void mtk_disp_mutex_put(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	mutex->claimed = false;
}

int mtk_disp_mutex_prepare(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	if (ddp->data->clk_gating_config)
		writel_relaxed(MUTEX_DISABLE_CLK_GATING,
			       ddp->regs + MT8168_DISP_MUTEX_CFG);

	return clk_prepare_enable(ddp->clk);
}

void mtk_disp_mutex_unprepare(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	if (ddp->data->clk_gating_config)
		writel_relaxed(MUTEX_ENABLE_CLK_GATING,
			       ddp->regs + MT8168_DISP_MUTEX_CFG);
	clk_disable_unprepare(ddp->clk);
}

void mtk_disp_mutex_add_comp(struct mtk_disp_mutex *mutex,
			     enum mtk_ddp_comp_id id)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	unsigned int reg;
	unsigned int sof_id;
	unsigned int offset;

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	switch (id) {
	case DDP_COMPONENT_DSI0:
		reg = MUTEX_SOF_DSI0;
		sof_id = DDP_MUTEX_SOF_DSI0;
		break;
	case DDP_COMPONENT_DSI1:
		reg = MUTEX_SOF_DSI0;
		sof_id = DDP_MUTEX_SOF_DSI0;
		break;
	case DDP_COMPONENT_DPI0:
		reg = MUTEX_SOF_DPI0;
		sof_id = DDP_MUTEX_SOF_DPI0;
		break;
	default:
		offset = DISP_REG_MUTEX_MOD(ddp->data, mutex->id);
		reg = readl_relaxed(ddp->regs + offset);
		reg |= ddp->data->mutex_mod[id];
		writel_relaxed(reg, ddp->regs + offset);
		return;
	}

	writel_relaxed(ddp->data->mutex_sof[sof_id],
		       ddp->regs + DISP_REG_MUTEX_SOF(ddp->data, mutex->id));

}

void mtk_disp_mutex_remove_comp(struct mtk_disp_mutex *mutex,
				enum mtk_ddp_comp_id id)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	unsigned int reg, offset;

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	switch (id) {
	case DDP_COMPONENT_DSI0:
	case DDP_COMPONENT_DSI1:
	case DDP_COMPONENT_DPI0:
		writel_relaxed(MUTEX_SOF_SINGLE_MODE,
			       ddp->regs +
			       DISP_REG_MUTEX_SOF(ddp->data, mutex->id));
		break;
	default:
		offset = DISP_REG_MUTEX_MOD(ddp->data, mutex->id);
		reg = readl_relaxed(ddp->regs + offset);
		reg &= ~(ddp->data->mutex_mod[id]);
		writel_relaxed(reg, ddp->regs + offset);
		break;
	}
}

void mtk_disp_mutex_enable(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	writel(1, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
}

void mtk_disp_mutex_disable(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	WARN_ON(&ddp->mutex[mutex->id] != mutex);

	writel(0, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
}

void mtk_disp_mutex_acquire(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);
	u32 tmp;

	writel(1, ddp->regs + DISP_REG_MUTEX_EN(mutex->id));
	writel(1, ddp->regs + DISP_REG_MUTEX(mutex->id));
	if (readl_poll_timeout_atomic(ddp->regs + DISP_REG_MUTEX(mutex->id),
				      tmp, tmp & INT_MUTEX, 1, 10000))
		pr_err("could not acquire mutex %d\n", mutex->id);
}

void mtk_disp_mutex_release(struct mtk_disp_mutex *mutex)
{
	struct mtk_ddp *ddp = container_of(mutex, struct mtk_ddp,
					   mutex[mutex->id]);

	writel(0, ddp->regs + DISP_REG_MUTEX(mutex->id));
}

static int mtk_ddp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ddp *ddp;
	struct resource *regs;
	int i;

	MTK_DRM_DEBUG(dev, "\n");

	ddp = devm_kzalloc(dev, sizeof(*ddp), GFP_KERNEL);
	if (!ddp)
		return -ENOMEM;

	for (i = 0; i < 10; i++)
		ddp->mutex[i].id = i;

	if (!of_find_property(dev->of_node, "clocks", &i)) {
		pr_info("mediatek-drm %s: has no clocks, set freerun\n",
			dev_name(dev));
	} else {
		ddp->clk = devm_clk_get(dev, NULL);
		if (IS_ERR(ddp->clk)) {
			pr_info("mediatek-drm %s: Failed to get clock\n",
				dev_name(dev));
			return PTR_ERR(ddp->clk);
		}
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ddp->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(ddp->regs)) {
		dev_err(dev, "Failed to map mutex registers\n");
		return PTR_ERR(ddp->regs);
	}

	ddp->data = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, ddp);

	MTK_DRM_DEBUG(dev, "ddp mutex reg base 0x%p\n", ddp->regs);

	return 0;
}

static int mtk_ddp_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ddp_driver_dt_match[] = {
	{ .compatible = "mediatek,mt2701-disp-mutex",
	  .data = &mt2701_ddp_driver_data},
	{ .compatible = "mediatek,mt8173-disp-mutex",
	  .data = &mt8173_ddp_driver_data},
	{ .compatible = "mediatek,mt8168-disp-mutex",
	  .data = &mt8168_ddp_driver_data},
	{},
};

MODULE_DEVICE_TABLE(of, ddp_driver_dt_match);

struct platform_driver mtk_ddp_driver = {
	.probe		= mtk_ddp_probe,
	.remove		= mtk_ddp_remove,
	.driver		= {
		.name	= "mediatek-ddp",
		.owner	= THIS_MODULE,
		.of_match_table = ddp_driver_dt_match,
	},
};
