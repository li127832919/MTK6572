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
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 11 16 2012 jianrong.zhang
 * [ALPS00361874] [Must Resolve][MT6517TD_AST3001][camcorder]preview play the video when set effects as choose your video
 * Copy from ALPS.ICS2.TDD.FPB.
 *
 * 03 31 2010 jianhua.tang
 * [DUMA00158728]S5K5CAGX YUV sensor driver 
 * .
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 26 2009 mtk02204
 * [DUMA00003515] [PC_Lint] Remove PC_Lint check warnings of camera related drivers.
 * 
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

#define MIPI_INTERFACE
/* SENSOR READ/WRITE ID */
#define S5K5CAGX_IIC_ID_HIGH
#define S5K5CAGX_MTK_INTERNAL_USE

#ifdef S5K5CAGX_IIC_ID_HIGH
	#define S5K5CAGX_WRITE_ID		0x5A   //MIPI		
	#define S5K5CAGX_READ_ID		0x5B
#else
	#define S5K5CAGX_WRITE_ID		0x78   //PARA
	#define S5K5CAGX_READ_ID		0x79
#endif

// Div 10 is the actual frame rate, 300 means 30fps, 75 means 7.5fps
	// Plz keep these setting un-changed
	#define S5K5CAGX_CAM_NOM_MAX_FPS				300		// 30fps
	#define S5K5CAGX_CAM_NOM_MIN_FPS				100		// 10fps
	#define S5K5CAGX_CAM_NIT_MAX_FPS				300		// 30fps
	#define S5K5CAGX_CAM_NIT_MIN_FPS				50		// 5.0fps

	/* It need double the sensor output frame if enable advance resizer because one for display & another for encode. */
	#define S5K5CAGX_VID_NOM_FIX_FPS			300		// 30fps
	#define S5K5CAGX_VID_NIT_FIX_FPS			150		// 15fps

// Msec / 10 is the actual frame time, 1000 means 100ms.
    #define S5K5CAGX_CAM_NOM_MIN_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_CAM_NOM_MAX_FPS)
    #define S5K5CAGX_CAM_NOM_MAX_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_CAM_NOM_MIN_FPS)
    #define S5K5CAGX_CAM_NIT_MIN_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_CAM_NIT_MAX_FPS)
    #define S5K5CAGX_CAM_NIT_MAX_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_CAM_NIT_MIN_FPS)

    #define S5K5CAGX_VID_NOM_FIX_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_VID_NOM_FIX_FPS)
    #define S5K5CAGX_VID_NIT_FIX_FR_TIME			((1000 * 10 * 10) / S5K5CAGX_VID_NIT_FIX_FPS)

    #define S5K5CAGX_PREVIEW_PCLK                   156000000
    #define S5K5CAGX_VIDEO_PCLK                     156000000
    #define S5K5CAGX_CAPTURE_PCLK                   130000000

	// Grab Window Setting for preview mode.
    #define S5K5CAGX_X_START                             (4)    
    #define S5K5CAGX_Y_START                             (4)  
    #define S5K5CAGX_CAP_X_START                         (8)
    #define S5K5CAGX_CAP_Y_START                         (8)
	#define S5K5CAGX_IMAGE_SENSOR_PV_WIDTH					(1024-16)//(640-4)
	#define S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT					(768-12)//(480-3)
    #define S5K5CAGX_IMAGE_SENSOR_VIDEO_WIDTH               (S5K5CAGX_IMAGE_SENSOR_PV_WIDTH)
    #define S5K5CAGX_IMAGE_SENSOR_VIDEO_HEIGHT              (S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT)
	#define S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH			(2048-32)//(2048-8)
	#define S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT			(1536-24)//(1536-6)




    #define S5K5CA_SET_SHUTTER_RATIO        (1)
    #define S5K5CA_SET_GAIN_RATIO           (1)
    #define S5K5CA_READ_SHUTTER_RATIO (4)    


typedef enum S5K5CAGX_AWBAE
{
    S5K5CAGX_BIT0 = 0x01,
    S5K5CAGX_BIT1 = 0x02,
    S5K5CAGX_BIT2 = 0x04,
    S5K5CAGX_BIT3 = 0x08,
    S5K5CAGX_BIT4 = 0x10,
    S5K5CAGX_BIT5 = 0x20,
    S5K5CAGX_BIT6 = 0x40,
    S5K5CAGX_BIT7 = 0x80,
    S5K5CAGX_BIT8 = 0x0100,
    S5K5CAGX_BIT9 = 0x0200,
    S5K5CAGX_BIT10 = 0x0400,
    S5K5CAGX_BIT11 = 0x0800,
    S5K5CAGX_BIT12 = 0x1000,
    S5K5CAGX_BIT13 = 0x2000,
    S5K5CAGX_BIT14 = 0x4000,
    S5K5CAGX_BIT15 = 0x8000
}S5K5CAGX_AWBAE_Enum;


typedef enum S5K5CAGX_CAMCO_MODE
{
    S5K5CAGX_MODE_PREVIEW=0,
	S5K5CAGX_MODE_VIDEO,
	S5K5CAGX_MODE_CAPTURE,
	S5K5CAGX_MODE_ZSD,
    S5K5CAGX_VIDEO_MAX
} S5K5CAGX_Camco_MODE;

typedef enum S5K5CAGX_YUV_CHOOSE
{		
  S5K5CAGX_YUV_INIT=0, //no write yuv setting
  S5K5CAGX_YUV_NVRAM,
  S5K5CAGX_YUV_MANUALREF,
  S5K5CAGX_YUV_MAX
} S5K5CAGX_YUV_CHOOSE;

typedef enum S5K5CAGX_AEAWB_CHOOSE
{		
  S5K5CAGX_AEAWB_PRE=0, //no write yuv setting
  S5K5CAGX_AEAWB_PARA,
  S5K5CAGX_AEAWB_MAX
} S5K5CAGX_AEAWB_CHOOSE;


struct S5K5CAGX_sensor_struct
{
        kal_uint16 sensor_id;

        kal_uint16 Dummy_Pixels;
        kal_uint16 Dummy_Lines;
        kal_uint32 Preview_PClk;
        kal_uint32 video_PClk;
        kal_uint32 capture_PClk;

        kal_uint32 Preview_Lines_In_Frame;
        kal_uint32 Capture_Lines_In_Frame;

        kal_uint32 Preview_Pixels_In_Line;
        kal_uint32 Capture_Pixels_In_Line;
        kal_uint16 Preview_Shutter;
        kal_uint16 Capture_Shutter;

        kal_uint16 StartX;
        kal_uint16 StartY;
        kal_uint16 iGrabWidth;
        kal_uint16 iGrabheight;

        kal_uint16 Capture_Size_Width;
        kal_uint16 Capture_Size_Height;
        kal_uint32 Digital_Zoom_Factor;

        kal_uint16 Max_Zoom_Factor;

        kal_uint32 Min_Frame_Rate;
        kal_uint32 Max_Frame_Rate;
        kal_uint32 Fixed_Frame_Rate;
        kal_uint32 sceneMode;      
        S5K5CAGX_Camco_MODE Camco_mode;
        AE_FLICKER_MODE_T Banding;
        
        kal_bool NightMode;
        kal_uint32  iShutter;
        kal_uint32  iGain;
        kal_bool manualAEStart;
};

#endif /* __SENSOR_H */

