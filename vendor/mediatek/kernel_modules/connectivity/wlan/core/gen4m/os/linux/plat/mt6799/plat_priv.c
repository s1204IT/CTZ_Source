/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2016 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#include <linux/threads.h>
#include "precomp.h"
#include "mach/mtk_ppm_api.h"

/*parameter for mt_ppm_sysboost_freq in kHZ*/
#define FREQUENCY_M(x)				(x*1000)

/*E1 is 1638000 KHz, E2 is TBD */
#define CLUSTER_B_CORE_MAX_FREQ			1638

/*E1 is 1378000 KHz  E2 is TBD */
#define CLUSTER_L_CORE_MAX_FREQ			1378

int32_t kalBoostCpu(IN struct ADAPTER *prAdapter,
		    IN uint32_t u4TarPerfLevel, IN uint32_t u4BoostCpuTh)
{
	u_int8_t fgBoostCpu = FALSE;

	if (u4TarPerfLevel >= u4BoostCpuTh) {
		fgBoostCpu = TRUE;
		mt_ppm_sysboost_set_core_limit(BOOST_BY_WIFI, CLUSTER_B_LKG,
					2, 2);
		mt_ppm_sysboost_set_freq_limit(BOOST_BY_WIFI, CLUSTER_B_LKG,
					FREQUENCY_M(CLUSTER_B_CORE_MAX_FREQ),
					FREQUENCY_M(CLUSTER_B_CORE_MAX_FREQ));
		mt_ppm_sysboost_set_core_limit(BOOST_BY_WIFI, CLUSTER_L_LKG,
					4, 4);
		mt_ppm_sysboost_set_freq_limit(BOOST_BY_WIFI, CLUSTER_L_LKG,
					FREQUENCY_M(CLUSTER_L_CORE_MAX_FREQ),
					FREQUENCY_M(CLUSTER_L_CORE_MAX_FREQ));
	} else {
		fgBoostCpu = FALSE;
		mt_ppm_sysboost_core(BOOST_BY_WIFI, 0);
		mt_ppm_sysboost_freq(BOOST_BY_WIFI, FREQUENCY_M(0));
	}
	DBGLOG(SW4, WARN,
	       "BoostCpu:%d,TarPerfLevel:%d,u4BoostCpuTh:%d\n",
	       fgBoostCpu, u4TarPerfLevel, u4BoostCpuTh);
	return 0;
}
