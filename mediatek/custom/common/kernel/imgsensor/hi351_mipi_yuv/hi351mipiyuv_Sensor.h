/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */ 
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   Qihao Geng (mtk70548)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 07 08 2013 yan.xu
 * [ALPS00732488] HI351 mipi yuv sensor driver check in
 * .
 *
 * 07 01 2013 yan.xu
 * [ALPS00732488] HI351 mipi yuv sensor driver check in
 * .
 *
 * 06 14 2013 yan.xu
 * [ALPS00732488] HI351 mipi yuv sensor driver check in
 * .
 *
 * 05 27 2013 yan.xu
 * [ALPS00732488] HI351 mipi yuv sensor driver check in
 * .
 *
 * 07 30 2012 qihao.geng
 * NULL
 * 1. support burst i2c write to save entry time.
 * 2. solve the view angle difference issue.
 * 3. video fix frame rate is OK and ready to release.
 *
 * 07 25 2012 qihao.geng
 * NULL
 * Increase the ZSD and read/write shutter/gain function.
 *
 * 07 20 2012 qihao.geng
 * NULL
 * HI351 MIPI Sensor Driver Check In 1st Versoin, can preview/capture.
 *
 * 07 19 2012 qihao.geng
 * NULL
 * HI351 MIPI YUV Sensor Driver Add, 1st version.
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef __HI351_MIPI_YUV_SENSOR_H
    #define __HI351_MIPI_YUV_SENSOR_H

    #define __HI351_DEBUG_TRACE__
#define HI351YUV_DEBUG
#ifdef HI351YUV_DEBUG
    #define HI351_TRACE printk
#else
    #define HI351_TRACE(x,...)
#endif


/* SENSOR READ/WRITE ID */
#define HI351_WRITE_ID                      0x40
#define HI351_READ_ID                       0x41

#define HI351_PV_PERIOD_PIXEL_NUMS      (1140+110)    /* Default preview line length */
#define HI351_PV_PERIOD_LINE_NUMS       (802+164)     /* Default preview frame length */
#define HI351_FULL_PERIOD_PIXEL_NUMS    (2180+110) //0x6E    /* Default full size line length */
#define HI351_FULL_PERIOD_LINE_NUMS     (1586+164) //0xA4   /* Default full size frame length */

/* SENSOR PREVIEW SIZE & START GRAB PIXEL OFFSET */
#define IMAGE_SENSOR_PV_GRAB_START_X        (0)//(0)
#define IMAGE_SENSOR_PV_GRAB_START_Y        (2)//(1)
#define HI351_IMAGE_SENSOR_PV_WIDTH         (1024-16)//-4
#define HI351_IMAGE_SENSOR_PV_HEIGHT        (768-2)//-1

#define HI351_IMAGE_SENSOR_VIDEO_WIDTH      (1024)
#define HI351_IMAGE_SENSOR_VIDEO_HEIGHT     (768)


/* SENSOR FULL SIZE & START GRAB PIXEL OFFSET */
#define IMAGE_SENSOR_FULL_GRAB_START_X      (0)
#define IMAGE_SENSOR_FULL_GRAB_START_Y      (0)//1
#define HI351_IMAGE_SENSOR_FULL_WIDTH       (2048-32)//-8
#define HI351_IMAGE_SENSOR_FULL_HEIGHT      (1536-30)//(1536 - 24)

#define BYTE_LEN                            (0x11)
#define BURST_TYPE_ED                       (0xEE)

#define BURST_TYPE                          (0x33)
//#define DELAY_TYPE                          (0xEE)

#define HI351_TEST_PATTERN_CHECKSUM			0x77404507

typedef struct
{
    UINT8 addr;
    UINT8 val;
    UINT8 type;     /* BYTE_LEN or DELAY_TYPE or BURST_TYPE */
} HI351_I2C_REG_STRUCT;

/* Export functions */
UINT32 HI351Open(void);
UINT32 HI351GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 HI351GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI351Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI351FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 HI351Close(void);

#endif /* #ifndef __HI351_MIPI_YUV_SENSOR_H */

