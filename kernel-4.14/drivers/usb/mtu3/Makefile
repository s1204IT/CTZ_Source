# SPDX-License-Identifier: GPL-2.0

ccflags-$(CONFIG_USB_MTU3_DEBUG)	+= -DDEBUG

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat

ifneq ($(filter y,$(CONFIG_MACH_MT8168)),)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/mt8168/
endif

obj-$(CONFIG_USB_MTU3)	+= mtu3.o

mtu3-y	:= mtu3_plat.o

ifneq ($(filter y,$(CONFIG_USB_MTU3_HOST) $(CONFIG_USB_MTU3_DUAL_ROLE)),)
	mtu3-y	+= mtu3_host.o
endif

ifneq ($(filter y,$(CONFIG_USB_MTU3_GADGET) $(CONFIG_USB_MTU3_DUAL_ROLE)),)
	mtu3-y	+= mtu3_core.o mtu3_gadget_ep0.o mtu3_gadget.o mtu3_qmu.o
endif

ifneq ($(CONFIG_USB_MTU3_DUAL_ROLE),)
	mtu3-y	+= mtu3_dr.o
endif
