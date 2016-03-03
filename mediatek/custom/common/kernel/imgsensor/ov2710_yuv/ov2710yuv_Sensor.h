/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2005
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
 *   ov2710yuv_Sensor.h
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver declare and macro define in the header file.
 *
 * Author:
 * -------
 *   Mormo
 *
 *=============================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 * 2011/10/25 Firsty Released By Mormo;
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
 
#ifndef __ov2710_SENSOR_H
#define __ov2710_SENSOR_H

#define OV2710_IMAGE_SENSOR_VGA_GRAB_PIXELS			0
#define OV2710_IMAGE_SENSOR_VGA_GRAB_LINES			1

#define OV2710_SVGA_PERIOD_PIXEL_NUMS					808
#define OV2710_SVGA_PERIOD_LINE_NUMS					608

#define OV2710_IMAGE_SENSOR_SVGA_WIDTH					(800-4)
#define OV2710_IMAGE_SENSOR_SVGA_HEIGHT				(600-4)

#define OV2710_IMAGE_SENSOR_SVGA_GRAB_PIXELS			0
#define OV2710_IMAGE_SENSOR_SVGA_GRAB_LINES			1

#define OV2710_IMAGE_SENSOR_HD_WIDTH					  (1280)
#define OV2710_IMAGE_SENSOR_HD_HEIGHT					(720)

#define OV2710_IMAGE_SENSOR_PV_GRAB_PIXELS			0
#define OV2710_IMAGE_SENSOR_PV_GRAB_LINES			1

#define OV2710_IMAGE_SENSOR_PV_WIDTH					(1280 - 8) //(IMAGE_SENSOR_SVGA_WIDTH - 8)
#define OV2710_IMAGE_SENSOR_PV_HEIGHT					(720 - 6) //(IMAGE_SENSOR_SVGA_HEIGHT - 6)

#define OV2710_IMAGE_SENSOR_FULL_WIDTH					(1280 - 8) //(IMAGE_SENSOR_HD_WIDTH - 8)
#define OV2710_IMAGE_SENSOR_FULL_HEIGHT				(720 - 6) //(IMAGE_SENSOR_HD_HEIGHT - 6)

#define OV2710_WRITE_ID						0x54 
#define OV2710_READ_ID							0x55

struct OV2710_sensor_STRUCT
{    
      kal_bool first_init;
	  kal_bool pv_mode;                 //True: Preview Mode; False: Capture Mode
	  kal_bool night_mode;              //True: Night Mode; False: Auto Mode
	  kal_bool MPEG4_Video_mode;      //Video Mode: MJPEG or MPEG4
	  kal_uint8 mirror;
	  kal_uint32 pv_pclk;               //Preview Pclk
	  kal_uint32 cp_pclk;               //Capture Pclk
	  kal_uint16 pv_dummy_pixels;          //Dummy Pixels
	  kal_uint16 pv_dummy_lines;           //Dummy Lines
	  kal_uint16 cp_dummy_pixels;          //Dummy Pixels
	  kal_uint16 cp_dummy_lines;           //Dummy Lines         
	  kal_uint16 fix_framerate;         //Fixed Framerate
	  kal_uint32 wb;
	  kal_uint32 exposure;
	  kal_uint32 effect;
	  kal_uint32 banding;
	  kal_uint16 pv_line_length;
	  kal_uint16 pv_frame_height;
	  kal_uint16 cp_line_length;
	  kal_uint16 cp_frame_height;
	  kal_uint16 video_current_frame_rate;
};

UINT32 ov2710Open(void);
UINT32 ov2710Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ov2710FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 ov2710GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 ov2710GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 ov2710Close(void);

#endif /* __SENSOR_H */
