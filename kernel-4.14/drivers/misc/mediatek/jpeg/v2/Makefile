# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2019 MediaTek Inc.

# drivers/image/jpeg/Makefile

ccflags-y = -DSDEBUG_T

ifeq ($(CONFIG_MACH_MT6757),y)
  ccflags-y += -DJPEG_DEC_DRIVER
endif

ifeq ($(CONFIG_MACH_MT6799),y)
  ccflags-y += -DJPEG_DEC_DRIVER
endif

ifeq ($(CONFIG_MACH_MT6765),y)
  ccflags-y += -DSMI_CG_SUPPORT
  ccflags-y += -DQOS_MT6765_SUPPORT
endif

ifeq ($(CONFIG_MACH_MT6761),y)
  ccflags-y += -DSMI_CG_SUPPORT
  ccflags-y += -DQOS_MT6761_SUPPORT
endif

ifeq ($(CONFIG_MACH_MT8163),y)
  ccflags-y += -DJPEG_PM_DOMAIN_ENABLE
endif

ifeq ($(CONFIG_MACH_MT8173),y)
  ccflags-y += -DJPEG_DEC_DRIVER
  ccflags-y += -DJPEG_PM_DOMAIN_ENABLE
endif

ifeq ($(CONFIG_MACH_MT8168),y)
  ccflags-y += -DJPEG_PM_DOMAIN_ENABLE

ifeq ($(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT),y)
  ccflags-y += -DCONFIG_MTK_SEC_JPEG_SUPPORT
endif
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/

#obj-$(CONFIG_MTK_JPEG) += jpeg_driver.o
obj-y += jpeg_driver.o

#jpeg_driver-objs := jpeg_cmdq.o jpeg_drv_enc.o jpeg_drv_dec.o jpeg_drv.o
jpeg_driver-objs := jpeg_drv_enc.o jpeg_drv_dec.o jpeg_drv.o

# EOF
