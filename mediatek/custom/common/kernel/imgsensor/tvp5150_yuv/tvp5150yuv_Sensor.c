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
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
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
 *   sensor.c
 *
 * Project:
 * --------
 *   YUSU
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Jackie Su (MTK02380)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 03 15 2011 koli.lin
 * [ALPS00034474] [Need Patch] [Volunteer Patch]
 * Move sensor driver current setting to isp of middleware.
 *
 * 10 12 2010 koli.lin
 * [ALPS00127101] [Camera] AE will flash
 * [Camera]Create Vsync interrupt to handle the exposure time, sensor gain and raw gain control.
 *
 * 08 27 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Check in AD5820 Constant AF function.
 *
 * 08 26 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Add AD5820 Lens driver function.
 * must disable SWIC and bus log, otherwise the lens initial time take about 30 second.(without log about 3 sec)
 *
 * 08 19 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Merge dual camera relative settings. Main TVP5150, SUB O7675 ready.
 *
 * 08 18 2010 ronnie.lai
 * [DUMA00032601] [Camera][ISP]
 * Mmodify ISP setting and add TVP5150 sensor driver.
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "tvp5150yuv_Sensor.h"
#include "tvp5150yuv_Camera_Sensor_para.h"
#include "tvp5150yuv_CameraCustomized.h"

#include "kd_camera_feature.h"
#include "tvp5150_reg.h"
//#include "tvp5150_pcode.h"

#define TVP5150_ONLINE_DEBUG
#ifdef TVP5150_ONLINE_DEBUG
#include <linux/proc_fs.h>
#endif

kal_bool  TVP5150YUV_MPEG4_encode_mode = KAL_FALSE;
kal_uint16  TVP5150YUV_sensor_gain_base=0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 TVP5150YUV_MAX_EXPOSURE_LINES = TVP5150_PV_PERIOD_LINE_NUMS-4;
kal_uint8  TVP5150YUV_MIN_EXPOSURE_LINES = 2;
kal_uint32 TVP5150YUV_isp_master_clock;
kal_uint16 TVP5150YUV_CURRENT_FRAME_LINES = TVP5150_PV_PERIOD_LINE_NUMS;

static kal_uint16 TVP5150YUV_dummy_pixels=0, TVP5150YUV_dummy_lines=0;
kal_uint16 TVP5150YUV_PV_dummy_pixels=0,TVP5150YUV_PV_dummy_lines=0;

//kal_uint8 TVP5150YUV_sensor_write_I2C_address = TVP5150_WRITE_ID;
//kal_uint8 TVP5150YUV_sensor_read_I2C_address = TVP5150_READ_ID;

static kal_uint32 TVP5150YUV_zoom_factor = 0; 


static UINT8 ZONE[4] = {24, 18, 40, 30};////version0.21, aug.2009,center 4:3 window


#define LOG_TAG "[TVP5150Yuv]"
#define SENSORDB(fmt, arg...) printk( LOG_TAG  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT

kal_uint16 TVP5150YUV_g_iDummyLines = 28; 

#if WINMO_USE
HANDLE TVP5150YUVhDrvI2C;
I2C_TRANSACTION TVP5150YUVI2CConfig;
#endif 

UINT8 TVP5150YUVPixelClockDivider=0;
kal_uint32 TVP51500YUV_sensor_pclk=27000000;;
kal_uint32 TVP5150YUV_PV_pclk = 5525;//27; 

kal_uint32 TVP5150YUV_CAP_pclk = 6175;//27;

kal_uint16 TVP5150YUV_pv_exposure_lines=0x360,TVP5150YUV_g_iBackupExtraExp,TVP5150YUV_extra_exposure_lines = 0;

kal_uint16 TVP5150YUV_sensor_id=0;

MSDK_SENSOR_CONFIG_STRUCT TVP5150YUVSensorConfigData;

kal_uint32 TVP5150YUV_FAC_SENSOR_REG;
kal_uint16 TVP5150YUV_sensor_flip_value;


/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT TVP5150YUVSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT TVP5150YUVSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;

typedef enum
{
  TVP5150_720P,      
  TVP5150_5M,     //5M 2592x1944
} TVP5150_RES_TYPE;
TVP5150_RES_TYPE TVP5150YUV_g_RES=TVP5150_720P;

typedef enum
{
  TVP5150_MODE_PREVIEW,
  TVP5150_MODE_CAPTURE   //5M    2592x1944
} TVP5150_MODE;
TVP5150_MODE g_iTVP5150YUV_Mode=TVP5150_MODE_PREVIEW;

struct tvp5150_reg{
	u8 reg_index;	
	u8 reg_value;
	};

static struct tvp5150_reg init_data[] = {		
	{TI5150_REG_VIN_SOURCE,        	0x00},  		// video input source selection,current sel ALP1A ; R/W	
	{TI5150_REG_ANALOG_CHAN_CTRL, 	0x15},			// Analog channel controls; R/W	
	{TI5150_REG_OPERATION_MOD_CTRL, 0x00},			// Operation mode controls, current for normal mode R/W	
	{TI5150_REG_MISC_CTRL,			0x4d},			// Miscellaneous controls; R/W#if 1	
	{TI5150_REG_AUTOSWITCH_MASK,	0xc0},			// Autoswitch mask; R/W	
	{TI5150_REG_VIDEO_STANDARD,		0x02},			// Video standard; R/W , autoswitch mode#else				
	{TI5150_REG_CONFIG_SHARED_PINS,	0x02},			// Configuration shared pins; R/W	
	{TI5150_REG_COLOR_KILLER_THRESHOLD_CTRL, 0x10},	//ColorKiller; R/W		
	{TI5150_REG_LUM_PROCESS_CTRL1,	0x60},			 // Luminance processing control #1; R/W	
	{TI5150_REG_LUM_PROCESS_CTRL2,	0x00},			 // Luminance processing control #2; R/W	
	{TI5150_REG_LUM_PROCESS_CTRL3,	0x00},			 // Luminance processing control #3; R/W	
	{TI5150_REG_CHROMINANCE_CTRL1,	0x0c},			 // Chrominance Control #1 Register	
	{TI5150_REG_CHROMINANCE_CTRL2,	0x14},			 // Chrominance Control #2 Register	
	{TI5150_REG_HSYNC_START_REG,	0x5a},			 // Horizontal Sync (HSYNC) Start Register	
	{TI5150_REG_OUT_DATA_RATE_SEL,	0x47},			 // Output and data rates select; R/W	
	{TI5150_REG_BRIGHTNESS,			0x6E},			 // Luminance control; R/W	
	{TI5150_REG_SATURATION,			0x78}, 			 // Chroma saturation; R/W		
	{TI5150_REG_HUE_CTRL,			0x00},			 // Chroma hue control; R/W	
	{TI5150_REG_CONTRAST,			0x72},			 // Luminance contrast; R/W
};

#define TVP5150_INIT_REG_NUM ARRAY_SIZE(init_data) 
//#define TVP5150_INIT_REG_NUM2 ARRAY_SIZE(tvp5150_init_enable)

//extern int iReadRegI2C(u8 a_u2Addr , u8 * a_puBuff , u16 i2cId);
//extern int iWriteRegI2C(u8 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//extern static int write_sensor_reg_8bit(int slave_addr,u8 addr, u8 value);
//extern static int read_sensor_reg_8bit(int slave_addr,u8 addr, u8 *pvalue);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId); 
extern int Tvp5150_MultiWriteReg(u8 cmd,u8 *pData, u16 lens);
//extern int Tvp5150_WriteReg(u8 reg, u8 val);
//extern int Tvp5150_ReadReg(u8 reg);


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

int Tvp5150_WriteReg(u8 addr, u8 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) ,(char)(para & 0xFF)};

  iWriteRegI2C(puSendCmd, 2, TVP5150_WRITE_ID);
  return TRUE;
}

int Tvp5150_ReadReg(u8 addr)
{
  char puGetByte=0;
  char puSendCmd = (char)(addr & 0xFF);
  iReadRegI2C(&puSendCmd, 1, &puGetByte, 1, TVP5150_WRITE_ID);
  return puGetByte;
}


//#define TVP5150YUV_write_cmos_sensor(addr, para) iWriteReg((u8) addr , (u32) para , 1, TVP5150_WRITE_ID)
//#define TVP5150YUV_burst_write_cmos_sensor(pData, bytes)  iBurstWriteReg(pData, bytes, TVP5150_WRITE_ID)

static UINT32 g_sensorAfStatus = 0;
static UINT32 g_sensorAfCancel = 0;
static UINT32 g_sensorAfCount  = 0;
static UINT32 g_sensorAfTimer  = 200;

#define PROFILE 1

#if PROFILE 
static struct timeval TVP5150YUV_ktv1, TVP5150YUV_ktv2; 
inline void TVP5150YUV_imgSensorProfileStart(void)
{
    do_gettimeofday(&TVP5150YUV_ktv1);    
}

inline void TVP5150YUV_imgSensorProfileEnd(char *tag)
{
}
#else 
inline static void TVP5150YUV_imgSensorProfileStart() {}
inline static void TVP5150YUV_imgSensorProfileEnd(char *tag) {}
#endif 

kal_uint16 TVP5150_read_cmos_sensor(kal_uint32 addr)
{
	return Tvp5150_ReadReg(addr);
}

#define Sleep(ms) mdelay(ms)


void TVP5150YUV_camera_para_to_sensor(void)
{

}


/*************************************************************************
* FUNCTION
*    TVP5150YUV_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void TVP5150YUV_sensor_to_camera_para(void)
{

}


/*************************************************************************
* FUNCTION
*    TVP5150YUV_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  TVP5150YUV_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void TVP5150YUV_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void TVP5150YUV_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;
    
    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Global");
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"GLOBAL_GAIN");
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }
            temp_para=TVP5150YUVSensorCCT[temp_addr].Para;
  //          temp_gain = TVP5150YUVReg2Gain(temp_para);

  //          temp_gain=(temp_gain*1000)/BASEGAIN;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min=1000;
            info_ptr->Max=15875;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }
                
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=TVP5150YUV_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}

//void TVP5150YUV_set_isp_driving_current(kal_uint8 current)
//{
//}

kal_bool TVP5150YUV_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }

 //           temp_para = TVP5150YUVGain2Reg(ItemValue);


            TVP5150YUVSensorCCT[temp_addr].Para = temp_para;
            Tvp5150_WriteReg(TVP5150YUVSensorCCT[temp_addr].Addr,temp_para);

 //           TVP5150YUV_sensor_gain_base=read_TVP5150YUV_gain();

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {
                        TVP5150YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_2MA);
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        TVP5150YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_4MA);
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        TVP5150YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_6MA);
                    }
                    else
                    {
                        TVP5150YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                        //OV5640YUV_set_isp_driving_current(ISP_DRIVING_8MA);
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    TVP5150YUV_FAC_SENSOR_REG=ItemValue;
                    break;
                case 1:
                    Tvp5150_WriteReg(TVP5150YUV_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}




void TVP5150YUV_Sensor_Init_set_720P(void);
//void TVP5150YUV_set_5M_init(void);
void TVP5150YUV_IQ(void);

/*******************************************************************************
*
********************************************************************************/
static void TVP5150YUV_Sensor_Init(void)
{
    SENSORDB("lln:: TVP5150YUV_Sensor_Init, use TVP5150YUV_Sensor_Init_set_720P");

    SENSORDB("Init Success \n");
}   /*  TVP5150YUV_Sensor_Init  */


void TVP5150YUV_Sensor_Init_set_720P(void)
{

}

void TVP5150YUV_set_preview_init(void)
{

}

UINT32 TVP5150YUVGetSensorID(UINT32 *sensorID) 
{
	
	SENSORDB("TVP5150GetSensorID\n");	

	*sensorID = Tvp5150_ReadReg(TVP5150_MSB_DEV_ID)<<8| Tvp5150_ReadReg(TVP5150_LSB_DEV_ID);

	SENSORDB("TVP5150 Sensor Read ID %x\n",*sensorID);
	if (*sensorID != TVP5150_SENSOR_ID) 
	{
      *sensorID=0xFFFFFFFF;
	  return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	return ERROR_NONE;
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*   TVP5150YUVOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/


static int tvp5150_Detect(void)
{	
	u8 msb_id, lsb_id,x_id;		

	printk("1.############tvp5150 Detect############\n");
	msb_id = Tvp5150_ReadReg(0x80);
	lsb_id = Tvp5150_ReadReg(0x81);	
	printk("2.############# Read Reg:0x80 = %x,0x81= %x ############\n",msb_id,lsb_id);	

	x_id = Tvp5150_ReadReg(0x88);
	printk("2.#############0x88=%x\n",x_id);

	return 0;	
}

UINT32 TVP5150YUVOpen(void)
{
    int i;

    printk("======TVP5150YUVOpen()========\n");
 
	tvp5150_Detect();
	mdelay(10);
	for (i = 0; i < TVP5150_INIT_REG_NUM; i++)	
		{		
			Tvp5150_WriteReg(init_data[i].reg_index,init_data[i].reg_value);
	} 
    return ERROR_NONE;
}
/*************************************************************************
* FUNCTION
*   TVP5150YUVClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 TVP5150YUVClose(void)
{
    return ERROR_NONE;
}	/* TVP5150YUVClose() */

/*************************************************************************
* FUNCTION
*   TVP5150YUVPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 TVP5150YUVPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;

    g_iTVP5150YUV_Mode = TVP5150_MODE_PREVIEW;


    printk("TVP5150YUVPreview(====================)\n");

    TVP5150YUV_set_preview_init();

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        TVP5150YUV_MPEG4_encode_mode = KAL_TRUE;
    }
    else
    {
        TVP5150YUV_MPEG4_encode_mode = KAL_FALSE;
    }

    iStartX += TVP5150_IMAGE_SENSOR_PV_STARTX;
    iStartY += TVP5150_IMAGE_SENSOR_PV_STARTY;
  

    TVP5150YUV_dummy_pixels = 0;
    TVP5150YUV_dummy_lines = 0;
    TVP5150YUV_PV_dummy_pixels = TVP5150YUV_dummy_pixels;
    TVP5150YUV_PV_dummy_lines = TVP5150YUV_dummy_lines;

//    TVP5150YUV_SetDummy(TVP5150YUV_dummy_pixels, TVP5150YUV_dummy_lines);
//    TVP5150YUV_SetShutter(TVP5150YUV_pv_exposure_lines);

    memcpy(&TVP5150YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    image_window->GrabStartX= 0;//iStartX;
    image_window->GrabStartY= 0;//iStartY;
    image_window->ExposureWindowWidth= TVP5150_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
    image_window->ExposureWindowHeight= TVP5150_IMAGE_SENSOR_PV_HEIGHT - 2*iStartY;

    printk("=============cameraPreview========,width=%d,height=%d\n", image_window->ExposureWindowWidth, image_window->ExposureWindowHeight);
    return ERROR_NONE;
}	/* TVP5150YUVPreview() */


UINT32 TVP5150YUVCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint32 shutter=TVP5150YUV_pv_exposure_lines;
    //kal_uint32 shutter = 0;
    kal_uint32 temp_shutter = 0;	
    kal_uint16 iStartX = 0, iStartY = 0;
    kal_uint32 pv_gain = 0;	
    kal_uint8 temp = 0;//for night mode	

    g_iTVP5150YUV_Mode = TVP5150_MODE_CAPTURE;

    if(sensor_config_data->EnableShutterTansfer==KAL_TRUE)
        shutter=sensor_config_data->CaptureShutter;

    if ((image_window->ImageTargetWidth<= TVP5150_IMAGE_SENSOR_PV_WIDTH) &&
        (image_window->ImageTargetHeight<= TVP5150_IMAGE_SENSOR_PV_HEIGHT)) {
        TVP5150YUV_dummy_pixels= 0;
        TVP5150YUV_dummy_lines = 0;

        //shutter = ((UINT32)(shutter*(TVP5150_PV_PERIOD_PIXEL_NUMS_HTS  + TVP5150YUV_PV_dummy_pixels)))/
        //                                                ((TVP5150_FULL_PERIOD_PIXEL_NUMS_HTS + TVP5150YUV_dummy_pixels)) ;
        //shutter = shutter * TVP5150YUV_CAP_pclk / TVP5150YUV_PV_pclk; 
        
        iStartX = TVP5150_IMAGE_SENSOR_PV_STARTX;
        iStartY = TVP5150_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX=iStartX;
        image_window->GrabStartY=iStartY;
        image_window->ExposureWindowWidth=TVP5150_IMAGE_SENSOR_PV_WIDTH - 2*iStartX;
        image_window->ExposureWindowHeight=TVP5150_IMAGE_SENSOR_PV_HEIGHT- 2*iStartY;
    }
    else { // 5M  Mode
        TVP5150YUV_dummy_pixels= 0;
        TVP5150YUV_dummy_lines = 0;        
 //       TVP5150YUV_set_5M();
        //sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR;      
        //TVP5150SetFlipMirror(sensor_config_data->SensorImageMirror); 

 	//TVP5150YUV_CAP_pclk = 5600;
 	//TVP5150YUV_PV_pclk = 5600;		
	//To avoid overflow...
        TVP5150YUV_CAP_pclk = 480;//27;
        TVP5150YUV_PV_pclk = 480;//27;		
        temp_shutter = (shutter*(TVP5150_PV_PERIOD_PIXEL_NUMS_HTS+TVP5150YUV_PV_dummy_pixels)*TVP5150YUV_CAP_pclk)
        			/(TVP5150_FULL_PERIOD_PIXEL_NUMS_HTS+TVP5150YUV_dummy_pixels)/TVP5150YUV_PV_pclk;	
        
        //shutter = (kal_uint32)(temp_shutter);
        SENSORDB("cap shutter calutaed = %d, 0x%x\n", shutter,shutter);
	//shutter = shutter*2; 
        //SVGA Internal CLK = 1/4 UXGA Internal CLK
        //shutter = 4* shutter;
       // shutter = ((UINT32)(shutter*(TVP5150_IMAGE_SENSOR_720P_PIXELS_LINE + TVP5150_PV_PERIOD_EXTRA_PIXEL_NUMS + TVP5150YUV_PV_dummy_pixels)))/
        //                                                ((TVP5150_IMAGE_SENSOR_5M_PIXELS_LINE+ TVP5150_FULL_PERIOD_EXTRA_PIXEL_NUMS + TVP5150YUV_dummy_pixels)) ;
        //shutter = shutter * TVP5150YUV_CAP_pclk / TVP5150YUV_PV_pclk; 
        iStartX = 2* TVP5150_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2* TVP5150_IMAGE_SENSOR_PV_STARTY;

        image_window->GrabStartX=iStartX;
        image_window->GrabStartY=iStartY;
        image_window->ExposureWindowWidth=TVP5150_IMAGE_SENSOR_FULL_WIDTH -2*iStartX;
        image_window->ExposureWindowHeight=TVP5150_IMAGE_SENSOR_FULL_HEIGHT-2*iStartY;
    }//5M Capture
    // config flashlight preview setting
    if(TVP5150_5M == TVP5150YUV_g_RES) //add start
    {
        sensor_config_data->DefaultPclk = 27000000;
        sensor_config_data->Pixels = TVP5150_IMAGE_SENSOR_5M_PIXELS_LINE + TVP5150YUV_PV_dummy_pixels;
        sensor_config_data->FrameLines =TVP5150_PV_PERIOD_LINE_NUMS+TVP5150YUV_PV_dummy_lines;
    }
    else
    {
        sensor_config_data->DefaultPclk = 27000000;
        sensor_config_data->Pixels = TVP5150_IMAGE_SENSOR_5M_PIXELS_LINE+TVP5150YUV_dummy_pixels;
        sensor_config_data->FrameLines =TVP5150_FULL_PERIOD_LINE_NUMS+TVP5150YUV_dummy_lines;
    }

    sensor_config_data->Lines = image_window->ExposureWindowHeight;
    sensor_config_data->Shutter =shutter;
  
 //   TVP5150YUV_SetDummy(TVP5150YUV_dummy_pixels, TVP5150YUV_dummy_lines);
    memcpy(&TVP5150YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}	/* TVP5150YUVCapture() */

UINT32 TVP5150YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    /*
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH - 4*TVP5150_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT - 4*TVP5150_IMAGE_SENSOR_PV_STARTY;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH - 2*TVP5150_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT - 2*TVP5150_IMAGE_SENSOR_PV_STARTY;
    */
    pSensorResolution->SensorFullWidth=TVP5150_CVBS_OUTPUT_WIDTH;
    pSensorResolution->SensorFullHeight=TVP5150_CVBS_OUTPUT_HEIGHT;
    pSensorResolution->SensorPreviewWidth=TVP5150_CVBS_OUTPUT_WIDTH;
    pSensorResolution->SensorPreviewHeight=TVP5150_CVBS_OUTPUT_HEIGHT;


    return ERROR_NONE;
}   /* TVP5150YUVGetResolution() */
UINT32 TVP5150YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    /*
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH - 2*TVP5150_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT - 2*TVP5150_IMAGE_SENSOR_PV_STARTY;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH - 4*TVP5150_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_HEIGHT - 4*TVP5150_IMAGE_SENSOR_PV_STARTY;
    */
    pSensorInfo->SensorPreviewResolutionX=TVP5150_CVBS_OUTPUT_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=TVP5150_CVBS_OUTPUT_HEIGHT;
    pSensorInfo->SensorFullResolutionX=TVP5150_CVBS_OUTPUT_WIDTH;
    pSensorInfo->SensorFullResolutionY=TVP5150_CVBS_OUTPUT_HEIGHT;


    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;//SENSOR_OUTPUT_FORMAT_YUYV;
	 
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; /*??? */
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

    /*
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxWidth=CAM_SIZE_VGA_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].MaxHeight=CAM_SIZE_VGA_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_100_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxWidth=CAM_SIZE_VGA_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].MaxHeight=CAM_SIZE_VGA_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].ISOSupported=TRUE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_200_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxWidth=CAM_SIZE_VGA_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].MaxHeight=CAM_SIZE_VGA_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_400_MODE].BinningEnable=FALSE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxWidth=CAM_SIZE_VGA_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].MaxHeight=CAM_SIZE_VGA_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_800_MODE].BinningEnable=TRUE;

    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxWidth=CAM_SIZE_VGA_WIDTH;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].MaxHeight=CAM_SIZE_VGA_HEIGHT;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].ISOSupported=FALSE;
    pSensorInfo->SensorISOBinningInfo.ISOBinningInfo[ISO_1600_MODE].BinningEnable=TRUE;
    */

    //pSensorInfo->CaptureDelayFrame = 1; 
    pSensorInfo->CaptureDelayFrame = 2; 
    pSensorInfo->PreviewDelayFrame = 2; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      

//
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 1;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            pSensorInfo->SensorClockFreq=27;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = TVP5150_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = TVP5150_IMAGE_SENSOR_PV_STARTY;             
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            pSensorInfo->SensorClockFreq=27;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = TVP5150_IMAGE_SENSOR_PV_STARTX; 
            pSensorInfo->SensorGrabStartY = TVP5150_IMAGE_SENSOR_PV_STARTY;             
            break;
        default:
            pSensorInfo->SensorClockFreq=27;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 10;             
            break;
    }

    TVP5150YUVPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &TVP5150YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* TVP5150YUVGetInfo() */


UINT32 TVP5150YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    printk("TVP5150YUVControl(===========%d========)\n", ScenarioId);

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
        //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
            TVP5150YUVPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
            TVP5150YUVCapture(pImageWindow, pSensorConfigData);
            break;
            //s_porting add
            //s_porting add
            //s_porting add
        default:
            return ERROR_INVALID_SCENARIO_ID;
            //e_porting add
            //e_porting add
            //e_porting add
    }
    return TRUE;
} /* TVP5150YUVControl() */


UINT32 TVP5150YUVSetVideoMode(UINT16 u2FrameRate)
{
    return TRUE;
}
kal_uint32 TVP5150_set_param_wb(kal_uint32 para)
{
    return KAL_TRUE;
} 

kal_uint32 TVP5150_set_param_exposure(kal_uint32 para)
{
    return KAL_TRUE;
} 

kal_uint32 TVP5150_set_param_effect(kal_uint32 para)
{
    return KAL_TRUE;
} 

UINT32 TVP5150YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	printk("\n TVP5150YUVSensorSetting() is called; \n");
	printk("cmd=%d, para = 0x%x\n", iCmd, iPara);
	
	switch (iCmd) {
		case FID_ZOOM_FACTOR:
	        TVP5150YUV_zoom_factor = iPara; 		
		break;
			default:
		break;
    }

    return TRUE;

}

UINT32 TVP5150YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{    
    UINT8   *pFeatureData8 =pFeaturePara;
    
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    //printk("%s:FeatureId = %d\n",__FUNCTION__,FeatureId);
    switch (FeatureId)
    {

#if WINMO_USE
        case SENSOR_FEATURE_GET_INIT_OPERATION_PARA:
        {
            PCAMERA_DRIVER_OPERATION_PARA_STRUCT pSensorOperData;
            pSensorOperData = (PCAMERA_DRIVER_OPERATION_PARA_STRUCT)pFeaturePara;

            pSensorOperData->CaptureDelayFrame = 2;         /* wait stable frame when sensor change mode (pre to cap) */
            pSensorOperData->PreviewDelayFrame = 3;         /* wait stable frame when sensor change mode (cap to pre) */
            pSensorOperData->PreviewDisplayWaitFrame = 2;   /* wait stable frame when sensor change mode (cap to pre) */
            pSensorOperData->AECalDelayFrame = 0;               /* The frame of calculation default 0 */
            pSensorOperData->AEShutDelayFrame = 0;              /* The frame of setting shutter default 0 for TG int */
            pSensorOperData->AESensorGainDelayFrame = 1;    /* The frame of setting sensor gain */
            pSensorOperData->AEISPGainDelayFrame = 2;          /* The frame of setting gain */
            pSensorOperData->AECalPeriod = 3;                           /* AE AWB calculation period */
            pSensorOperData->FlashlightMode=FLASHLIGHT_LED_CONSTANT;

            break;
        }
#endif 

        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++=IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16=IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
            *pFeatureReturnPara16++=TVP5150_PV_PERIOD_EXTRA_PIXEL_NUMS + TVP5150_PV_PERIOD_PIXEL_NUMS + TVP5150YUV_dummy_pixels;//TVP5150_PV_PERIOD_PIXEL_NUMS+TVP5150YUV_dummy_pixels;
            *pFeatureReturnPara16=TVP5150_PV_PERIOD_LINE_NUMS+TVP5150YUV_dummy_lines;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *pFeatureReturnPara32 = 27000000; //19500000;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
 //           TVP5150YUV_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
//            TVP5150YUV_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
 //           TVP5150YUV_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            TVP5150YUV_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            Tvp5150_WriteReg(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = Tvp5150_ReadReg(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
                TVP5150YUVSensorCCT[i].Addr=*pFeatureData32++;
                TVP5150YUVSensorCCT[i].Para=*pFeatureData32++;
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=TVP5150YUVSensorCCT[i].Addr;
                *pFeatureData32++=TVP5150YUVSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
                TVP5150YUVSensorReg[i].Addr=*pFeatureData32++;
                TVP5150YUVSensorReg[i].Para=*pFeatureData32++;
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=TVP5150YUVSensorReg[i].Addr;
                *pFeatureData32++=TVP5150YUVSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=TVP5150_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, TVP5150YUVSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, TVP5150YUVSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &TVP5150YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            TVP5150YUV_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            TVP5150YUV_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=TVP5150YUV_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            TVP5150YUV_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            TVP5150YUV_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            TVP5150YUV_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
			//test by lingnan
            //pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            TVP5150YUVSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_YUV_CMD:
	    TVP5150YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));//may cause crash
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            TVP5150YUVGetSensorID(pFeatureReturnPara32); 
            break;			            
        default:
            break;
    }
    return ERROR_NONE;
}	/* TVP5150YUVFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncTVP5150YUV=
{
    TVP5150YUVOpen,
    TVP5150YUVGetInfo,
    TVP5150YUVGetResolution,
    TVP5150YUVFeatureControl,
    TVP5150YUVControl,
    TVP5150YUVClose
};

UINT32 TVP5150_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncTVP5150YUV;

    return ERROR_NONE;
}   /* TVP5150_YUV_SensorInit() */
