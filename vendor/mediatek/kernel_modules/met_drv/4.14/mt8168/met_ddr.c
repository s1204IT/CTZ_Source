/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
//#include <linux/module.h>
#include <linux/string.h>

#include "mtk_dramc.h"
#include "met_ddr.h"

int met_check_ddr_type(unsigned int DRAM_TYPE)
{
	switch (DRAM_TYPE) {
		case TYPE_PCDDR3:
		case TYPE_LPDDR3:
			return MET_DDRTYPE_DDR3;
		case TYPE_PCDDR4:
		case TYPE_LPDDR4:
		case TYPE_LPDDR4X:
		case TYPE_LPDDR4P:
			return MET_DDRTYPE_DDR4;
	}
	return MET_DDRTYPE_UNKNOWN;
}
