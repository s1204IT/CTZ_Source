obj-y += mtk_camera_drv.o \
		 mtk_camera_util.o \
		 mtk_camera_if.o \
		 mtk_vcu_if.o

subdir-ccflags-y += -Werror
ifdef CONFIG_VIDEO_MEDIATEK_VCU
	ccflags-y += -I$(srctree)/drivers/media/platform/mtk-vcu
endif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/variant
