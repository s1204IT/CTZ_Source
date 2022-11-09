/*
 * Copyright (C) 2018 MediaTek Inc.
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

#ifndef __SENINF_TEE_COMMON_H__
#define __SENINF_TEE_COMMON_H__

#include "seninf_reg.h"

#define SENINF_MUX_MAX_NUM 6

typedef struct {
	SENINF1_REG_MUX_CTRL			SENINF_MUX_CTRL;
	SENINF1_REG_MUX_CTRL_EXT		SENINF_MUX_CTRL_EXT;
} SENINF_TEE_REG_MUX;

typedef struct {
	SENINF_REG_TOP_MUX_CTRL 		SENINF_TOP_MUX_CTRL;
	SENINF_REG_TOP_CAM_MUX_CTRL 	SENINF_TOP_CAM_MUX_CTRL;
    SENINF_TEE_REG_MUX              seninf_mux[SENINF_MUX_MAX_NUM];
} SENINF_TEE_REG;

typedef enum {
    SENINF_TEE_CMD_SYNC_TO_PA,
    SENINF_TEE_CMD_SYNC_TO_VA,
} SENINF_TEE_CMD;

typedef void* seninf_ca_open_session_t(void);
typedef int seninf_ca_close_session_t(void*);
typedef int seninf_ca_sync_to_pa_t(void*, void *);
typedef int seninf_ca_sync_to_va_t(void*, void *);

#define SENINF_CA_LIB "uree_seninf.so"
#define SENINF_CA_OPEN_SESSION "seninf_ca_open_session"
#define SENINF_CA_CLOSE_SESSION "seninf_ca_close_session"
#define SENINF_CA_SYNC_TO_PA "seninf_ca_sync_to_pa"
#define SENINF_CA_SYNC_TO_VA "seninf_ca_sync_to_va"


#endif

