#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

ccflags-y += -I$(srctree)/drivers/clk/mediatek
ccflags-y += -I$(srctree)/drivers/devfreq
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/freqhopping/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/$(MTK_PLATFORM)/



SKIP_SMI_CONFIG := true
PLL_HOPPING_READY := false

ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6771"))
ccflags-y += -DDYNAMIC_DISP_HRT
SUPPORT_USE_DDR_TYPE := true
SUPPORT_MMP := true
else ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt8168"))
ccflags-y += -DSMI_AQU
ENABLE_SCENARIO := true
PLL_HOPPING_READY := true
endif

MMDVFS_PLATFORM_DIR=$(srctree)/drivers/misc/mediatek/mmdvfs/$(MTK_PLATFORM)

obj-y += mmdvfs_pmqos.o
obj-y += mmdvfs_ioctl.o

ifeq (true, $(PLL_HOPPING_READY))
ccflags-y += -DPLL_HOPPING_READY
endif
ifeq (true, $(SKIP_SMI_CONFIG))
ccflags-y += -DMMDVFS_SKIP_SMI_CONFIG
endif
ifneq ($(wildcard $(MMDVFS_PLATFORM_DIR)),)
obj-y += $(subst ",,$(MTK_PLATFORM))/
else
obj-y += mmdvfs_plat_default.o
endif
ifeq (true, $(APPLY_CLK_LOG))
ccflags-y += -DAPPLY_CLK_LOG
endif

ifeq (true, $(ENABLE_SCENARIO))
ccflags-y += -DENABLE_SCENARIO
obj-y += mmdvfs_scenario.o
obj-y += mmdvfs_mgr_v3.o
obj-y += mmdvfs_mgr_util.o
obj-y += mmdvfs_profile_adaptor.o
endif

ifeq (true, $(QOS_BOUND_DETECT))
ccflags-y += -DQOS_BOUND_DETECT
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/qos/$(MTK_PLATFORM)/
endif

ifeq ($(CONFIG_MMPROFILE),y)
ifeq (true, $(SUPPORT_MMP))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/
ccflags-y += -DMMDVFS_MMP
endif
endif

ifeq ($(CONFIG_MTK_DRAMC),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/
ccflags-y += -DUSE_MTK_DRAMC
endif

ifeq (true, $(SUPPORT_USE_DDR_TYPE))
ccflags-y += -DUSE_DDR_TYPE
ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/submodule/
endif

