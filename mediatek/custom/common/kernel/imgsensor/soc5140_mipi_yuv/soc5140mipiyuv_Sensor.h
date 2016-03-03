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
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

/* SENSOR VGA SIZE */
#define SOC5140_WRITE_ID                         (0x78)
#define SOC5140_READ_ID                          (0x79)
#define SOC5140_IMAGE_SENSOR_FULL_HACTIVE		 (2560-32)//2592-32
#define SOC5140_IMAGE_SENSOR_FULL_VACTIVE		 (1920-24)//1944
#define SOC5140_IMAGE_SENSOR_PV_HACTIVE			 (640 - 16)
#define SOC5140_IMAGE_SENSOR_PV_VACTIVE			 (480 - 12)
#define SOC5140_AF_PRV_W                         (255)
#define SOC5140_AF_PRV_H                         (255)
#define SCO5140_PRE_START_X                      (4)
#define SCO5140_PRE_START_Y                      (4)
#define SCO5140_CAP_START_X                      (12)
#define SCO5140_CAP_START_Y                      (12)


/* Sensor Exposure Line Limitation */
#define AGAIN_LIMITATION                            (0x7f - 1)
#define DGAIN_LIMITAITON                            (0x280 - 1)

typedef enum SOC5140_YUV_CHOOSE
{		
  SOC5140_YUV_INIT=0, //no write yuv setting
  SOC5140_YUV_NOREFRESH,  //Camera Capture
  SOC5140_YUV_REFRESH,
  SOC5140_YUV_MANUALREF,
  SOC5140_YUV_MAX
} SOC5140_YUV_CHOOSE;

//0x8404 state
typedef enum SOC5140_8404_STATE
{
  SOC5140_8404_W_OK = 0,//Camera Preview
  SOC5140_8404_BEFOR_W_ERR,//Camera Preview
  SOC5140_8404_AFTER_W_ERR,//Camera Capture
  SOC5140_8404_MAX
} SOC5140_8404_STATE;

//Mode state
typedef enum SOC5140_CAMCO_MODE
{		
  SOC5140_CAM_PREVIEW=0,//Camera Preview
  SOC5140_CAM_CAPTURE,//Camera Capture
  SOC5140_CAM_VIDEO,
  SOC5140_CAM_PREVIEW_ZSD,
  SOC5140_VIDEO_MAX
} SOC5140_Camco_MODE;

struct SOC5140_sensor_struct
{
	kal_uint16 sensor_id;
	kal_uint32 Preview_PClk;
	kal_uint32 CapturePclk;
	kal_uint32 ZsdturePclk;	

	kal_bool   userAskAeLock;
    kal_bool   userAskAwbLock;

	kal_uint16 Max_Zoom_Factor;
	kal_uint32 Min_Frame_Rate;
	kal_uint32 Max_Frame_Rate;
	kal_uint32 Fixed_Frame_Rate;
	//kal_bool Night_Mode;
	SOC5140_Camco_MODE Camco_mode;
	AE_FLICKER_MODE_T Banding;
	kal_bool NightMode;
    kal_bool  aeStateOnOriginalSet;
    kal_uint32  	sceneMode;
	kal_bool    	manualAEStart;
    kal_uint32      currentAGain;
    kal_uint32      currentDGain;    
	kal_uint32      currentExposureTime;
};


//export functions
UINT32 SOC5140Open(void);
UINT32 SOC5140GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 SOC5140GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SOC5140Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 SOC5140FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 SOC5140Close(void);
#endif /* __SENSOR_H */

