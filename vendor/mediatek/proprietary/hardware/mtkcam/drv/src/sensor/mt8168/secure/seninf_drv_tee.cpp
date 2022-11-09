/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#define LOG_TAG "SeninfDrvTee"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <linux/mman-proprietary.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <utils/threads.h>
#include <utils/Errors.h>
#include <cutils/log.h>
#include <cutils/properties.h>
#include <cutils/atomic.h>
#include <mtkcam/def/common.h>
#include "kd_seninf.h"
#include "seninf_type.h"
#include "seninf_drv_tee.h"

#ifndef USING_MTK_LDVT
#define LOG_MSG(fmt, arg...)    ALOGD("[%s]" fmt, __FUNCTION__, ##arg)
#define LOG_WRN(fmt, arg...)    ALOGD("[%s]Warning(%5d):" fmt, __FUNCTION__, __LINE__, ##arg)
#define LOG_ERR(fmt, arg...)    ALOGE("[%s]Err(%5d):" fmt, __FUNCTION__, __LINE__, ##arg)
#else
#include "uvvf.h"
#if 1
#define LOG_MSG(fmt, arg...)    VV_MSG("[%s]" fmt, __FUNCTION__, ##arg)
#define LOG_WRN(fmt, arg...)    VV_MSG("[%s]Warning(%5d):" fmt, __FUNCTION__, __LINE__, ##arg)
#define LOG_ERR(fmt, arg...)    VV_MSG("[%s]Err(%5d):" fmt, __FUNCTION__, __LINE__, ##arg)
#else
#define LOG_MSG(fmt, arg...)
#define LOG_WRN(fmt, arg...)
#define LOG_ERR(fmt, arg...)
#endif
#endif

using namespace NSCam;

/*******************************************************************************
*
********************************************************************************/
SeninfDrvTeeImp::SeninfDrvTeeImp() : SeninfDrvImp()
{
    LOG_MSG("[SeninfDrvTeeImp]\n");
    memset(&mTeeReg, 0, sizeof(SENINF_TEE_REG));
    mUser = 0;
    mpCa = nullptr;
    seninf_ca_open_session = nullptr;
    seninf_ca_close_session = nullptr;
    seninf_ca_sync_to_pa = nullptr;
    seninf_ca_sync_to_va = nullptr;
}

/*******************************************************************************
*
********************************************************************************/
SeninfDrvTeeImp::~SeninfDrvTeeImp()
{

}

/*******************************************************************************
*
********************************************************************************/
SeninfDrv*
SeninfDrvTeeImp::
getInstance()
{
    static SeninfDrvTeeImp singleton;
    return &singleton;
}

/*******************************************************************************
*
********************************************************************************/
static void* pDynamicLink;
int SeninfDrvTeeImp::init()
{
    int ret = 0;
    int errRet = 0;

    Mutex::Autolock lock(mLock);

    LOG_MSG("SeninfDrvTee init %d \n", mUser);
    if (mUser) {
        android_atomic_inc(&mUser);
        return 0;
    }

    pDynamicLink = dlopen(SENINF_CA_LIB, RTLD_NOW);

    if (pDynamicLink == nullptr) {
        LOG_ERR("seninf_ca lib open failed (%s)", dlerror());
        return (-20);
    }

    if (seninf_ca_open_session == nullptr) {
        seninf_ca_open_session = (seninf_ca_open_session_t*) dlsym(pDynamicLink, SENINF_CA_OPEN_SESSION);
        if (seninf_ca_open_session == nullptr) {
            LOG_ERR("dlsym seninf_ca_open_session failed!");
            errRet = (-25);
            goto Exit;
        }
    }

    if (seninf_ca_close_session == nullptr) {
        seninf_ca_close_session = (seninf_ca_close_session_t*) dlsym(pDynamicLink, SENINF_CA_CLOSE_SESSION);
        if (seninf_ca_close_session == nullptr) {
            LOG_ERR("dlsym seninf_ca_close_session failed!");
            errRet = (-26);
            goto Exit;
        }
    }

    if (seninf_ca_sync_to_pa == nullptr) {
        seninf_ca_sync_to_pa = (seninf_ca_sync_to_pa_t*) dlsym(pDynamicLink, SENINF_CA_SYNC_TO_PA);
        if (seninf_ca_sync_to_pa == nullptr) {
            LOG_ERR("dlsym seninf_ca_sync_to_pa failed!");
            errRet = (-27);
            goto Exit;
        }
    }

    if (seninf_ca_sync_to_va == nullptr) {
        seninf_ca_sync_to_va = (seninf_ca_sync_to_va_t*) dlsym(pDynamicLink, SENINF_CA_SYNC_TO_VA);
        if (seninf_ca_sync_to_va == nullptr) {
            LOG_ERR("dlsym seninf_ca_write_reg failed!");
            errRet = (-28);
            goto Exit;
        }
    }

    mpCa = seninf_ca_open_session();
    if (mpCa == nullptr) {
        LOG_ERR("seninf_ca_open_session failed!");
        errRet = (-21);
        goto Exit;
    }

    ret = SeninfDrvImp::init();

    memset(&mTeeReg, 0, sizeof(SENINF_TEE_REG));
    seninf_ca_sync_to_va(mpCa, (void *)&mTeeReg);

    android_atomic_inc(&mUser);

    return ret;

Exit:
    if(errRet) {
        dlclose(pDynamicLink);
        pDynamicLink = nullptr;
        mpCa = nullptr;
        seninf_ca_open_session = nullptr;
        seninf_ca_close_session = nullptr;
        seninf_ca_sync_to_pa = nullptr;
        seninf_ca_sync_to_va = nullptr;
    }

    return errRet;
}

/*******************************************************************************
*
********************************************************************************/
int SeninfDrvTeeImp::uninit()
{
    int ret = 0;

    Mutex::Autolock lock(mLock);

    LOG_MSG("SeninfDrvTee uninit %d \n", mUser);
    android_atomic_dec(&mUser);

    if (mUser) {
        return 0;
    }

    seninf_ca_close_session(mpCa);

    memset(&mTeeReg, 0, sizeof(SENINF_TEE_REG));

    mpCa = nullptr;
    seninf_ca_open_session = nullptr;
    seninf_ca_close_session = nullptr;
    seninf_ca_sync_to_pa = nullptr;
    seninf_ca_sync_to_va = nullptr;

    if(pDynamicLink) {
        dlclose(pDynamicLink);
        pDynamicLink = nullptr;
    }

    ret = SeninfDrvImp::uninit();

    return ret;
}

/*******************************************************************************
*
********************************************************************************/
int SeninfDrvTeeImp::sendCommand(int cmd, unsigned long arg1, unsigned long arg2, unsigned long arg3)
{
    int ret = 0;
    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX*)mTeeReg.seninf_mux;
    int i = 0;
    

    Mutex::Autolock lock(mLock);//for uninit, some pointer will be set to NULL

    switch (cmd) {
    case CMD_SENINF_SYNC_REG_TO_PA:
        LOG_MSG("Sync registers to PA");
        for(i = 0; i < SENINF_MUX_MAX_NUM; i++)
        {
            pSeninf_mux += i;
            LOG_MSG("SENINF%d:(0x%x/0x%x)\n", i, SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL), SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL_EXT));
        }
        seninf_ca_sync_to_pa(mpCa, (void *)&mTeeReg);
        break;

    case CMD_SENINF_SECURE_DEBUG_ENABLE:
        {
            SENINF_TEE_REG seninf_reg;
            memset(&seninf_reg, 0, sizeof(SENINF_TEE_REG));

            seninf_ca_sync_to_va(mpCa, (void *)&seninf_reg);
            seninf_reg.seninf_mux[SENINF_MUX1].SENINF_MUX_CTRL_EXT.Raw |= (1<<31);
            seninf_ca_sync_to_pa(mpCa, (void*)&seninf_reg);
        }
        break;

    default:
        ret = SeninfDrvImp::sendCommand(cmd, arg1, arg2, arg3);
        break;
    }

    return ret;
}

/*******************************************************************************
*
********************************************************************************/
int SeninfDrvTeeImp::setSeninfTopMuxCtrl(unsigned int seninfMuxIdx, SENINF_ENUM seninfSrc)
{
    int ret = 0;
    SENINF_TEE_REG *pSeninf_top_mux = &mTeeReg;

    switch (seninfMuxIdx) {
        case SENINF_MUX2:
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF2_MUX_SRC_SEL)= seninfSrc;
            break;
        case SENINF_MUX3:
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF3_MUX_SRC_SEL)= seninfSrc;
            break;
        case SENINF_MUX4:
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF4_MUX_SRC_SEL)= seninfSrc;
            break;
        case SENINF_MUX5:
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF5_MUX_SRC_SEL)= seninfSrc;
            break;
        case SENINF_MUX6:
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF6_MUX_SRC_SEL)= seninfSrc;
            break;
        default://SENINF_MUX1
            SENINF_BITS(pSeninf_top_mux, SENINF_TOP_MUX_CTRL, SENINF1_MUX_SRC_SEL)= seninfSrc;
            break;
    }

    LOG_MSG("SENINF_TOP_MUX_CTRL(0x%x)\n", SENINF_READ_REG(pSeninf_top_mux, SENINF_TOP_MUX_CTRL));
    return ret;
}

/*******************************************************************************
*
********************************************************************************/
int SeninfDrvTeeImp::setSeninfCamTGMuxCtrl(SENINF_TOP_P1_ENUM camTarget, SENINF_MUX_ENUM seninfMuxSrc)
{
    int ret = 0;
    SENINF_TEE_REG *pSeninf_cam_mux = &mTeeReg;

    switch (camTarget){
        case SENINF_TOP_SV1:
            SENINF_BITS(pSeninf_cam_mux, SENINF_TOP_CAM_MUX_CTRL, SENINF_CAM2_MUX_SRC_SEL)= seninfMuxSrc;
            break;        
        case SENINF_TOP_SV2:
            SENINF_BITS(pSeninf_cam_mux, SENINF_TOP_CAM_MUX_CTRL, SENINF_CAM3_MUX_SRC_SEL)= seninfMuxSrc;
            break;
        default://SENINF_MUX1
            SENINF_BITS(pSeninf_cam_mux, SENINF_TOP_CAM_MUX_CTRL, SENINF_CAM0_MUX_SRC_SEL)= seninfMuxSrc;
            break;
    }
    
    LOG_MSG("SENINF_CAM_MUX_CTRL(0x%x)\n", SENINF_READ_REG(pSeninf_cam_mux, SENINF_TOP_CAM_MUX_CTRL));
    return ret;
}

/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::getSeninfTopMuxCtrl(SENINF_MUX_ENUM seninfMuXIdx)
{

    SENINF_TEE_REG *pSeninf = &mTeeReg;
    unsigned int seninfSrc = 0;
    unsigned int temp0 = 0;
    unsigned int temp1 = 0;

    temp0 = SENINF_READ_REG(pSeninf, SENINF_TOP_MUX_CTRL_0);
    temp1 = SENINF_READ_REG(pSeninf, SENINF_TOP_MUX_CTRL_1);
    switch (seninfMuXIdx){
    case SENINF_MUX1:
        seninfSrc = (temp0&0xF);
        break;
    case SENINF_MUX2:
        seninfSrc = (temp0&0xF00)>>8;
        break;
    case SENINF_MUX3:
        seninfSrc = (temp0&0xF0000)>>16;
        break;
    case SENINF_MUX4:
        seninfSrc = (temp0&0xF000000)>>24;
        break;
    case SENINF_MUX5:
        seninfSrc = (temp1&0xF);
        break;
    case SENINF_MUX6:
        seninfSrc = (temp1&0xF00)>>8;
        break;
    default:
        LOG_MSG("No support SENINF_MUX%d", seninfMuXIdx);
        break;
    }

    return seninfSrc;  
}
*/
/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::getSeninfCamTGMuxCtrl(unsigned int targetCamTG)
{
    SENINF_TEE_REG *pSeninf_cam_mux = &mTeeReg; //0X1A004400
    unsigned int seninfMuxSrc = 0;
    unsigned int temp0 = SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_CTRL_0);
    unsigned int temp1 = SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_CTRL_1);
    unsigned int temp2 = SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_CTRL_2);

    switch (targetCamTG){
    case SENINF_CAM_MUX0:
        seninfMuxSrc = (temp0&0xF);
        break;
    case SENINF_CAM_MUX1:
        seninfMuxSrc = (temp0&0xF00)>>8;
        break;
    case SENINF_CAM_MUX2:
        seninfMuxSrc = (temp0&0xF0000)>>16;
        break;
    case SENINF_CAM_MUX3:
        seninfMuxSrc = (temp0&0xF000000)>>24;
        break;
    case SENINF_CAM_MUX4:
        seninfMuxSrc = (temp1&0xF);
        break;
    case SENINF_CAM_MUX5:
        seninfMuxSrc = (temp1&0xF00)>>8;
        break;
    case SENINF_CAM_MUX6:
        seninfMuxSrc = (temp1&0xF0000)>>16;
        break;
    case SENINF_CAM_MUX7:
        seninfMuxSrc = (temp1&0xF000000)>>24;
        break;
    case SENINF_CAM_MUX8:
        seninfMuxSrc = (temp2&0xF);
        break;
    case SENINF_CAM_MUX9:
        seninfMuxSrc = (temp2&0xF00)>>8;
        break;
    case SENINF_CAM_MUX10:
        seninfMuxSrc = (temp2&0xF0000)>>16;
        break;
    default:
        LOG_MSG("No support SENINF_CAM_MUX%d", targetCamTG);
        break;
    }
    return seninfMuxSrc;
}
*/
/*******************************************************************************
*
********************************************************************************/
int SeninfDrvTeeImp::setSeninfMuxCtrl(unsigned long Muxsel, unsigned long hsPol, unsigned long vsPol,
		SENINF_SOURCE_ENUM inSrcTypeSel, TG_FORMAT_ENUM inDataType,	unsigned int PixelMode)
{

    int ret = 0;
    unsigned int temp = 0;
    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX*)mTeeReg.seninf_mux;
    pSeninf_mux += Muxsel;

    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_MUX_EN) = 1;
    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_SRC_SEL) = inSrcTypeSel;
    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT) = (inSrcTypeSel == TEST_MODEL) ? 0 : 1;
    if(1 == PixelMode) { /*2 Pixel*/
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT) = 0;
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_PIX_SEL) = 1;
    } else if(2 == PixelMode) {
        /* 4 Pixel*/
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT) = 1;
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_PIX_SEL) = 0;
    } else {
        /* 1 pixel*/
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT) = 0;
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_PIX_SEL) = 0;
    }

    if(JPEG_FMT != inDataType) {
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_FULL_WR_EN) = 2;
    } else {
        SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_FULL_WR_EN) = 0;
    } if ((CSI2 == inSrcTypeSel)||(MIPI_SENSOR <= inSrcTypeSel)) {
        if(JPEG_FMT != inDataType) {
            /*Need to use Default for New design*/
            SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_FLUSH_EN) = 0x1B;
            SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_PUSH_EN) = 0x1F;
        } else {
            SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_FLUSH_EN) = 0x18;
            SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, FIFO_PUSH_EN) = 0x1E;
        }
    }
    /*Disable send fifo to cam*/
    // SENINF_BITS(pSeninf, SENINF1_MUX_SPARE, SENINF_FIFO_FULL_SEL) = 0; keep default =1
    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_HSYNC_POL) = hsPol;
    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL, SENINF_VSYNC_POL) = vsPol;

    temp = SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL);
    SENINF_WRITE_REG(pSeninf_mux,SENINF_MUX_CTRL, temp|0x3);//reset
    SENINF_WRITE_REG(pSeninf_mux,SENINF_MUX_CTRL, temp&0xFFFFFFFC);//clear reset
    temp = SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL_EXT);
    SENINF_WRITE_REG(pSeninf_mux,SENINF_MUX_CTRL_EXT, temp|0x100);//use overrun detect setting as before ECO

    LOG_MSG("SENINF%d_MUX_CTRL(0x%x)\n", Muxsel,SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL));
    LOG_MSG("SENINF%d_MUX_CTRL_EXT(0x%x)\n", Muxsel,SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CTRL_EXT));

    return ret;
}

/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::setSeninfMuxCrop(
    SENINF_MUX_ENUM mux, unsigned int start_x, unsigned int end_x, bool enable)
{
    seninf1_mux_REGS * pSeninf_mux = (seninf1_mux_REGS *)mpSeninfMuxBaseAddr[mux]; //1A00 4D00
    SENINF_TEE_REG_MUX *pSeninf_mux_tee = (SENINF_TEE_REG_MUX *)mTeeReg.seninf_mux;
    pSeninf_mux_tee += mux;

    SENINF_BITS(pSeninf_mux, SENINF_MUX_CROP_PIX_CTRL, rg_seninf_mux_crop_start_8pix_cnt) = start_x/8;
    SENINF_BITS(pSeninf_mux, SENINF_MUX_CROP_PIX_CTRL, rg_seninf_mux_crop_end_8pix_cnt) = start_x/8 +
                                                        (end_x - start_x + 1)/8 - 1 + (((end_x - start_x + 1) % 8) > 0 ?1 :0);
    SENINF_BITS(pSeninf_mux_tee, SENINF_MUX_CTRL_1, rg_seninf_mux_crop_en) = enable;

    LOG_MSG("SENINF_MUX_CROP_PIX_CTRL 0x%x SENINF_MUX_CTRL_1 0x%x, mux %d, start %d, end %d, enable %d \n",
        SENINF_READ_REG(pSeninf_mux, SENINF_MUX_CROP_PIX_CTRL),
        SENINF_READ_REG(pSeninf_mux_tee, SENINF_MUX_CTRL_1),
        mux,
        start_x,
        end_x,
        enable);
    return 0;
}
*/
/*******************************************************************************
*
********************************************************************************/
//MBOOL SeninfDrvTeeImp::isMUXUsed(SENINF_MUX_ENUM mux)
//{
/*
    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX *)mTeeReg.seninf_mux;
    pSeninf_mux += mux;

    return SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_0, seninf_mux_en);
*/
//    return MFALSE;
//}

/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::enableMUX(SENINF_MUX_ENUM mux)
{
    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX *)mTeeReg.seninf_mux;
    pSeninf_mux += mux;

    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_0, seninf_mux_en) = 1;

    return 0;
}
*/    

/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::disableMUX(SENINF_MUX_ENUM mux)
{

    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX *)mTeeReg.seninf_mux;
    pSeninf_mux += mux;

    SENINF_BITS(pSeninf_mux, SENINF_MUX_CTRL_0, seninf_mux_en) = 0;

    return 0;    
}
*/
/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::debug()
{
    SENINF_TEE_REG  reg;
    volatile UINT32 *pRegPa = (UINT32 *)&reg;
    volatile UINT32 *pRegVa = (UINT32 *)&mTeeReg;

    seninf_ca_sync_to_va(mpCa, (void *)&reg);

    LOG_MSG("Seninf TEE registers (PA VA):");

    for (int i = 0; i < sizeof(SENINF_TEE_REG) / sizeof(UINT32); i++, pRegPa++, pRegVa++) {
        LOG_MSG("%x %x", *pRegPa, *pRegVa);
    }

    return SeninfDrvImp::debug();
}
*/
/*******************************************************************************
*
********************************************************************************/
//int SeninfDrvTeeImp::reset(CUSTOM_CFG_CSI_PORT mipiPort)
//{
/*
    int i;
    SENINF_TEE_REG_MUX *pSeninf_mux = (SENINF_TEE_REG_MUX *)mTeeReg.seninf_mux;
    SENINF_ENUM         seninf      = getCSIInfo(mipiPort)->seninf;

    SeninfDrvImp::reset(mipiPort);

    for(i = SENINF_MUX1; i < SENINF_MUX_NUM; i++)
        if(getSeninfTopMuxCtrl((SENINF_MUX_ENUM)i) == seninf && isMUXUsed((SENINF_MUX_ENUM)i)) {
            SENINF_BITS((pSeninf_mux + i), SENINF_MUX_CTRL_0, seninf_mux_sw_rst) = 1;
            usleep(1);
            SENINF_BITS((pSeninf_mux + i), SENINF_MUX_CTRL_0, seninf_mux_sw_rst) = 0;
        }

    return 0;
*/
//}

/*******************************************************************************
*
********************************************************************************/
/*
MBOOL SeninfDrvTeeImp::isCamMUXUsed(SENINF_CAM_MUX_ENUM cam_mux)
{
    SENINF_TEE_REG *pSeninf_cam_mux = (SENINF_TEE_REG *)&mTeeReg;
    LOG_MSG("SENINF_CAM_MUX_EN 0x%x isCamMUXUsed %d", SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN), SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN)&(1>>cam_mux));
    return SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN)&(1<<cam_mux);
}
*/
/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::enableCamMUX(SENINF_CAM_MUX_ENUM cam_mux)
{
    SENINF_TEE_REG *pSeninf_cam_mux = (SENINF_TEE_REG *)&mTeeReg;
    unsigned int temp = SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN);
    SENINF_WRITE_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN, temp|(1<<cam_mux));
    LOG_MSG("SENINF_CAM_MUX_EN 0x%x cam_mux %d", SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN), cam_mux);
    return 0;
}
*/
/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::disableCamMUX(SENINF_CAM_MUX_ENUM cam_mux)
{
    SENINF_TEE_REG *pSeninf_cam_mux = (SENINF_TEE_REG *)&mTeeReg;
    unsigned int temp = SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN);
    SENINF_WRITE_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN, temp&(~(1<<cam_mux)));
    LOG_MSG("SENINF_CAM_MUX_EN 0x%x cam_mux %d", SENINF_READ_REG(pSeninf_cam_mux, SENINF_CAM_MUX_EN), cam_mux);
    return 0;
}
*/
/*******************************************************************************
*
********************************************************************************/
/*
int SeninfDrvTeeImp::setSeninfCamMuxSrc(SENINF_MUX_ENUM src, SENINF_CAM_MUX_ENUM target)
{
    LOG_MSG("cam mux target %d src %d\n", target, src);

    SENINF_TEE_REG *mpSeninfCamMux_base = (SENINF_TEE_REG *)&mTeeReg;
    switch(target) {
        case SENINF_CAM_MUX0:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_0, rg_seninf_cam_mux0_src_sel) = src;
            break;
        case SENINF_CAM_MUX1:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_0, rg_seninf_cam_mux1_src_sel) = src;
            break;
        case SENINF_CAM_MUX2:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_0, rg_seninf_cam_mux2_src_sel) = src;
            break;
        case SENINF_CAM_MUX3:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_0, rg_seninf_cam_mux3_src_sel) = src;
            break;
        case SENINF_CAM_MUX4:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_1, rg_seninf_cam_mux4_src_sel) = src;
            break;
        case SENINF_CAM_MUX5:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_1, rg_seninf_cam_mux5_src_sel) = src;
            break;
        case SENINF_CAM_MUX6:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_1, rg_seninf_cam_mux6_src_sel) = src;
            break;
        case SENINF_CAM_MUX7:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_1, rg_seninf_cam_mux7_src_sel) = src;
            break;
        case SENINF_CAM_MUX8:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_2, rg_seninf_cam_mux8_src_sel) = src;
            break;
        case SENINF_CAM_MUX9:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_2, rg_seninf_cam_mux9_src_sel) = src;
            break;
        case SENINF_CAM_MUX10:
            SENINF_BITS(mpSeninfCamMux_base, SENINF_CAM_MUX_CTRL_2, rg_seninf_cam_mux10_src_sel) = src;
            break;

        default:
            LOG_MSG("No support SENINF_CAM_MUX s: %d t: %d", src, target);
            break;
    }
    return 0;
}
*/
