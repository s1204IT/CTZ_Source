/*
 * Copyright (c) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef MTK_MDP_TYPE_H__
#define MTK_MDP_TYPE_H__

#define DP_COLORFMT_PACK(					\
			PACKED, LOOSE, VIDEO, PLANE,	\
			HFACTOR, VFACTOR, BITS, GROUP,	\
			SWAP_ENABLE, UNIQUEID)			\
	(((PACKED)      << 27) |				\
	 ((LOOSE)       << 26) |				\
	 ((VIDEO)       << 23) |				\
	 ((PLANE)       << 21) |				\
	 ((HFACTOR)     << 19) |				\
	 ((VFACTOR)     << 18) |				\
	 ((BITS)        << 8)  |				\
	 ((GROUP)       << 6)  |				\
	 ((SWAP_ENABLE) << 5)  |				\
	 ((UNIQUEID)    << 0))

enum DP_COLOR_ENUM {
DP_COLOR_UNKNOWN        = 0,

DP_COLOR_FULLG8         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0,  8, 2,  0, 21),
DP_COLOR_FULLG10        = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 10, 2,  0, 21),
DP_COLOR_FULLG12        = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 12, 2,  0, 21),
DP_COLOR_FULLG14        = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 14, 2,  0, 21),
DP_COLOR_UFO10          = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 10, 2,  0, 24),

DP_COLOR_BAYER8         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0,  8, 2,  0, 20),
DP_COLOR_BAYER10        = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 10, 2,  0, 20),
DP_COLOR_BAYER12        = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 12, 2,  0, 20),
DP_COLOR_RGB48          = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 48, 0,  0, 23),
//for Bayer+Mono raw-16
DP_COLOR_RGB565_RAW     = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 16, 2,  0, 0),

// fix 16 bits for pixel
DP_COLOR_BAYER8_UNPAK   = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0,  8, 2,  0, 22),
DP_COLOR_BAYER10_UNPAK  = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 10, 2,  0, 22),
DP_COLOR_BAYER12_UNPAK  = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 12, 2,  0, 22),
DP_COLOR_BAYER14_UNPAK  = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 14, 2,  0, 22),

// Unified format
DP_COLOR_GREY           = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0,  8, 1,  0, 7),

DP_COLOR_RGB565         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 16, 0,  0, 0),
DP_COLOR_BGR565         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 16, 0,  1, 0),
DP_COLOR_RGB888         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 24, 0,  1, 1),
DP_COLOR_BGR888         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 24, 0,  0, 1),
DP_COLOR_RGBA8888       = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 32, 0,  1, 2),
DP_COLOR_BGRA8888       = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 32, 0,  0, 2),
DP_COLOR_ARGB8888       = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 32, 0,  1, 3),
DP_COLOR_ABGR8888       = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 32, 0,  0, 3),

DP_COLOR_UYVY           = DP_COLORFMT_PACK(0, 0, 0, 1, 1, 0, 16, 1,  0, 4),
DP_COLOR_VYUY           = DP_COLORFMT_PACK(0, 0, 0, 1, 1, 0, 16, 1,  1, 4),
DP_COLOR_YUYV           = DP_COLORFMT_PACK(0, 0, 0, 1, 1, 0, 16, 1,  0, 5),
DP_COLOR_YVYU           = DP_COLORFMT_PACK(0, 0, 0, 1, 1, 0, 16, 1,  1, 5),

DP_COLOR_I420           = DP_COLORFMT_PACK(0, 0, 0, 3, 1, 1,  8, 1,  0, 8),
DP_COLOR_YV12           = DP_COLORFMT_PACK(0, 0, 0, 3, 1, 1,  8, 1,  1, 8),
DP_COLOR_I422           = DP_COLORFMT_PACK(0, 0, 0, 3, 1, 0,  8, 1,  0, 9),
DP_COLOR_YV16           = DP_COLORFMT_PACK(0, 0, 0, 3, 1, 0,  8, 1,  1, 9),
DP_COLOR_I444           = DP_COLORFMT_PACK(0, 0, 0, 3, 0, 0,  8, 1,  0, 10),
DP_COLOR_YV24           = DP_COLORFMT_PACK(0, 0, 0, 3, 0, 0,  8, 1,  1, 10),

DP_COLOR_NV12           = DP_COLORFMT_PACK(0, 0, 0, 2, 1, 1,  8, 1,  0, 12),
DP_COLOR_NV21           = DP_COLORFMT_PACK(0, 0, 0, 2, 1, 1,  8, 1,  1, 12),
DP_COLOR_NV16           = DP_COLORFMT_PACK(0, 0, 0, 2, 1, 0,  8, 1,  0, 13),
DP_COLOR_NV61           = DP_COLORFMT_PACK(0, 0, 0, 2, 1, 0,  8, 1,  1, 13),
DP_COLOR_NV24           = DP_COLORFMT_PACK(0, 0, 0, 2, 0, 0,  8, 1,  0, 14),
DP_COLOR_NV42           = DP_COLORFMT_PACK(0, 0, 0, 2, 0, 0,  8, 1,  1, 14),

// Mediatek proprietary format
//Frame mode + Block mode
DP_COLOR_420_BLKP_UFO   = DP_COLORFMT_PACK(0, 0, 5, 2, 1, 1, 256, 1, 0, 12),
//Frame mode + Block mode
DP_COLOR_420_BLKP       = DP_COLORFMT_PACK(0, 0, 1, 2, 1, 1, 256, 1, 0, 12),
//Field mode + Block mode
DP_COLOR_420_BLKI       = DP_COLORFMT_PACK(0, 0, 3, 2, 1, 1, 256, 1, 0, 12),
//Frame mode
DP_COLOR_422_BLKP       = DP_COLORFMT_PACK(0, 0, 1, 1, 1, 0, 512, 1, 0, 4),

DP_COLOR_IYU2           = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 24,  1, 0, 25),
DP_COLOR_YUV444         = DP_COLORFMT_PACK(0, 0, 0, 1, 0, 0, 24,  1, 0, 30),

// Mediatek proprietary 10bit format
DP_COLOR_RGBA1010102    = DP_COLORFMT_PACK(1, 0, 0, 1, 0, 0, 32,  0, 1, 2),
DP_COLOR_BGRA1010102    = DP_COLORFMT_PACK(1, 0, 0, 1, 0, 0, 32,  0, 0, 2),
//Packed 10bit UYVY
DP_COLOR_UYVY_10P       = DP_COLORFMT_PACK(1, 0, 0, 1, 1, 0, 20,  1, 0, 4),
//Packed 10bit NV21
DP_COLOR_NV21_10P       = DP_COLORFMT_PACK(1, 0, 0, 2, 1, 1, 10,  1, 1, 12),
//Frame mode + Block mode
DP_COLOR_420_BLKP_10_H  = DP_COLORFMT_PACK(1, 0, 1, 2, 1, 1, 320, 1, 0, 12),
//Frame mode + HEVC tile mode
DP_COLOR_420_BLKP_10_V  = DP_COLORFMT_PACK(1, 1, 1, 2, 1, 1, 320, 1, 0, 12),
//Frame mode + Block mode
DP_COLOR_420_BLKP_UFO_10_H  = DP_COLORFMT_PACK(1, 0, 5, 2, 1, 1, 320, 1, 0, 12),
//Frame mode + HEVC tile mode
DP_COLOR_420_BLKP_UFO_10_V  = DP_COLORFMT_PACK(1, 1, 5, 2, 1, 1, 320, 1, 0, 12),

// Loose 10bit format
DP_COLOR_UYVY_10L       = DP_COLORFMT_PACK(0, 1, 0, 1, 1, 0, 20,  1, 0, 4),
DP_COLOR_VYUY_10L       = DP_COLORFMT_PACK(0, 1, 0, 1, 1, 0, 20,  1, 1, 4),
DP_COLOR_YUYV_10L       = DP_COLORFMT_PACK(0, 1, 0, 1, 1, 0, 20,  1, 0, 5),
DP_COLOR_YVYU_10L       = DP_COLORFMT_PACK(0, 1, 0, 1, 1, 0, 20,  1, 1, 5),
DP_COLOR_NV12_10L       = DP_COLORFMT_PACK(0, 1, 0, 2, 1, 1, 10,  1, 0, 12),
DP_COLOR_NV21_10L       = DP_COLORFMT_PACK(0, 1, 0, 2, 1, 1, 10,  1, 1, 12),
DP_COLOR_NV16_10L       = DP_COLORFMT_PACK(0, 1, 0, 2, 1, 0, 10,  1, 0, 13),
DP_COLOR_NV61_10L       = DP_COLORFMT_PACK(0, 1, 0, 2, 1, 0, 10,  1, 1, 13),
DP_COLOR_YV12_10L       = DP_COLORFMT_PACK(0, 1, 0, 3, 1, 1, 10,  1, 1, 8),
DP_COLOR_I420_10L       = DP_COLORFMT_PACK(0, 1, 0, 3, 1, 1, 10,  1, 0, 8),
};
#endif  /* MTK_MDP_TYPE_H__ */
