#
# Copyright (C) 2018 MediaTek Inc.
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

# MTK TEE efuse read makefile.

#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/trustzone/tz_cross
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/trustzone/tz_cross
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/trustzone
#ccflags-y += -I$(srctree)/include/trustzone

obj-$(CONFIG_MTK_SECURE_EFUSE) += tee_read.o
