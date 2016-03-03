
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
*****************************************************************************//*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
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
 * 07 11 2011 jun.pei
 * [ALPS00059464] OV2710 sensor check in
 * .
 *
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
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov2710yuv_Sensor.h"
#include "ov2710yuv_Camera_Sensor_para.h"
#include "ov2710yuv_CameraCustomized.h"

//#define OV2710YUV_DEBUG
#ifdef OV2710YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

static DEFINE_SPINLOCK(OV2710_yuv_drv_lock);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 OV2710_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 3,OV2710_WRITE_ID);
	return TRUE;

}
kal_uint16 OV2710_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,OV2710_WRITE_ID);
    return get_byte;
}

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0
#define Support720p 1
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* follow is define by jun
********************************************************************************/
MSDK_SENSOR_CONFIG_STRUCT OV2710SensorConfigData;

static struct OV2710_sensor_STRUCT OV2710_sensor;
static kal_uint32 OV2710_zoom_factor = 0; 
static int sensor_id_fail = 0;	

static void OV2710_Init_Parameter(void)
{
    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.first_init = KAL_TRUE;
    OV2710_sensor.pv_mode = KAL_TRUE;
    OV2710_sensor.night_mode = KAL_FALSE;
    OV2710_sensor.MPEG4_Video_mode = KAL_FALSE;

    OV2710_sensor.cp_pclk = OV2710_sensor.pv_pclk;

    OV2710_sensor.pv_dummy_pixels = 0;
    OV2710_sensor.pv_dummy_lines = 0;
    OV2710_sensor.cp_dummy_pixels = 0;
    OV2710_sensor.cp_dummy_lines = 0;

    OV2710_sensor.wb = 0;
    OV2710_sensor.exposure = 0;
    OV2710_sensor.effect = 0;
    OV2710_sensor.banding = AE_FLICKER_MODE_50HZ;

    OV2710_sensor.pv_line_length = 640;
    OV2710_sensor.pv_frame_height = 480;
    OV2710_sensor.cp_line_length = 640;
    OV2710_sensor.cp_frame_height = 480;
    spin_unlock(&OV2710_yuv_drv_lock);    
}


static void OV2710_AWB_enable(kal_bool AWB_enable)
{	 
	kal_uint16 temp_AWB_reg = 0;

	temp_AWB_reg = OV2710_read_cmos_sensor(0x3201);
	
	if (AWB_enable == KAL_TRUE)
	{
		OV2710_write_cmos_sensor(0x3201, (temp_AWB_reg | 0x10));
	}
	else
	{
		OV2710_write_cmos_sensor(0x3201, (temp_AWB_reg & (~0x10)));
	}

}


static kal_uint16 OV2710_power_on(void)
{
    kal_uint16 OV2710_sensor_id = 0;
    spin_lock(&OV2710_yuv_drv_lock);
    //OV2710_sensor.pv_pclk = 48000000;
    spin_unlock(&OV2710_yuv_drv_lock);
    //Software Reset
    OV2710_write_cmos_sensor(0x3021,0x01);

    /* Read Sensor ID  */
    OV2710_sensor_id = ( OV2710_read_cmos_sensor(0x3000)<< 8 )|OV2710_read_cmos_sensor(0x3001);
    //OV2710_sensor_id = 0x1410; 


    SENSORDB("[OV2710YUV]:read Sensor ID:%x\n",OV2710_sensor_id);	
    return OV2710_sensor_id;
}

void OV2710_Initial_Setting(void)
{
   OV2710_write_cmos_sensor(0x3069,0x07); //0x02
   OV2710_write_cmos_sensor(0x306a,0x07); //0x03
  
   OV2710_write_cmos_sensor(0x32F0,0x02);
   OV2710_write_cmos_sensor(0x3109,0x04);
   OV2710_write_cmos_sensor(0x3040,0x04);
   OV2710_write_cmos_sensor(0x3041,0x02);
   OV2710_write_cmos_sensor(0x3042,0xFF);
   OV2710_write_cmos_sensor(0x3043,0x08);
   OV2710_write_cmos_sensor(0x3052,0xE0);
   OV2710_write_cmos_sensor(0x305F,0x33);
   OV2710_write_cmos_sensor(0x3100,0x07);
   OV2710_write_cmos_sensor(0x3106,0x03);
                                     
   OV2710_write_cmos_sensor(0x3105,0x01);
   OV2710_write_cmos_sensor(0x3108,0x05);
   OV2710_write_cmos_sensor(0x3110,0x22);
   OV2710_write_cmos_sensor(0x3111,0x57);
   OV2710_write_cmos_sensor(0x3112,0x22);
   OV2710_write_cmos_sensor(0x3113,0x55);
   OV2710_write_cmos_sensor(0x3114,0x05);
   OV2710_write_cmos_sensor(0x3135,0x00);

                                      
   OV2710_write_cmos_sensor(0x3290,0x01);
   OV2710_write_cmos_sensor(0x3291,0x80);
   OV2710_write_cmos_sensor(0x3296,0x01);
   OV2710_write_cmos_sensor(0x3297,0x73);
                                     
   OV2710_write_cmos_sensor(0x3250,0x80);
   OV2710_write_cmos_sensor(0x3251,0x03);
   OV2710_write_cmos_sensor(0x3252,0xFF);
   OV2710_write_cmos_sensor(0x3253,0x00);
   OV2710_write_cmos_sensor(0x3254,0x03);
   OV2710_write_cmos_sensor(0x3255,0xFF);
   OV2710_write_cmos_sensor(0x3256,0x00);
   OV2710_write_cmos_sensor(0x3257,0x50);
                                    
   OV2710_write_cmos_sensor(0x3270,0x00);
   OV2710_write_cmos_sensor(0x3271,0x0C);
   OV2710_write_cmos_sensor(0x3272,0x18);
   OV2710_write_cmos_sensor(0x3273,0x32);
   OV2710_write_cmos_sensor(0x3274,0x44);
   OV2710_write_cmos_sensor(0x3275,0x54);
   OV2710_write_cmos_sensor(0x3276,0x70);
   OV2710_write_cmos_sensor(0x3277,0x88);
   OV2710_write_cmos_sensor(0x3278,0x9D);
   OV2710_write_cmos_sensor(0x3279,0xB0);
   OV2710_write_cmos_sensor(0x327A,0xCF);
   OV2710_write_cmos_sensor(0x327B,0xE2);
   OV2710_write_cmos_sensor(0x327C,0xEF);
   OV2710_write_cmos_sensor(0x327D,0xF7);
   OV2710_write_cmos_sensor(0x327E,0xFF);
                                     
   OV2710_write_cmos_sensor(0x3302,0x00);
   OV2710_write_cmos_sensor(0x3303,0x40);
   OV2710_write_cmos_sensor(0x3304,0x00);
   OV2710_write_cmos_sensor(0x3305,0x96);
   OV2710_write_cmos_sensor(0x3306,0x00);
   OV2710_write_cmos_sensor(0x3307,0x29);
   OV2710_write_cmos_sensor(0x3308,0x07);
   OV2710_write_cmos_sensor(0x3309,0xBA);
   OV2710_write_cmos_sensor(0x330A,0x06);
   OV2710_write_cmos_sensor(0x330B,0xF5);
   OV2710_write_cmos_sensor(0x330C,0x01);
   OV2710_write_cmos_sensor(0x330D,0x51);
   OV2710_write_cmos_sensor(0x330E,0x01);
   OV2710_write_cmos_sensor(0x330F,0x30);
   OV2710_write_cmos_sensor(0x3310,0x07);
   OV2710_write_cmos_sensor(0x3311,0x16);
   OV2710_write_cmos_sensor(0x3312,0x07);
   OV2710_write_cmos_sensor(0x3313,0xBA);
                                       
   OV2710_write_cmos_sensor(0x3326,0x02);
   OV2710_write_cmos_sensor(0x32F6,0x0F);
   OV2710_write_cmos_sensor(0x32F9,0x42);
   OV2710_write_cmos_sensor(0x32FA,0x24);
   OV2710_write_cmos_sensor(0x3325,0x4A);
   OV2710_write_cmos_sensor(0x3330,0x00);
   OV2710_write_cmos_sensor(0x3331,0x0A);
   OV2710_write_cmos_sensor(0x3332,0xFF);
   OV2710_write_cmos_sensor(0x3338,0x30);
   OV2710_write_cmos_sensor(0x3339,0x84);
   OV2710_write_cmos_sensor(0x333A,0x48);
   OV2710_write_cmos_sensor(0x333F,0x07);
                                      
   OV2710_write_cmos_sensor(0x3360,0x10);
   OV2710_write_cmos_sensor(0x3361,0x18);
   OV2710_write_cmos_sensor(0x3362,0x1f);
   OV2710_write_cmos_sensor(0x3363,0x37);
   OV2710_write_cmos_sensor(0x3364,0x80);
   OV2710_write_cmos_sensor(0x3365,0x80);
   OV2710_write_cmos_sensor(0x3366,0x68);
   OV2710_write_cmos_sensor(0x3367,0x60);
   OV2710_write_cmos_sensor(0x3368,0x30);
   OV2710_write_cmos_sensor(0x3369,0x28);
   OV2710_write_cmos_sensor(0x336A,0x20);
   OV2710_write_cmos_sensor(0x336B,0x10);
   OV2710_write_cmos_sensor(0x336C,0x00);
   OV2710_write_cmos_sensor(0x336D,0x20);
   OV2710_write_cmos_sensor(0x336E,0x1C);
   OV2710_write_cmos_sensor(0x336F,0x18);
   OV2710_write_cmos_sensor(0x3370,0x10);
   OV2710_write_cmos_sensor(0x3371,0x38);
   OV2710_write_cmos_sensor(0x3372,0x3C);
   OV2710_write_cmos_sensor(0x3373,0x3F);
   OV2710_write_cmos_sensor(0x3374,0x3F);
   OV2710_write_cmos_sensor(0x338A,0x34);
   OV2710_write_cmos_sensor(0x338B,0x7F);
   OV2710_write_cmos_sensor(0x338C,0x10);
   OV2710_write_cmos_sensor(0x338D,0x23);
   OV2710_write_cmos_sensor(0x338E,0x7F);
   OV2710_write_cmos_sensor(0x338F,0x14);
                                      
   OV2710_write_cmos_sensor(0x3375,0x0A); 
   OV2710_write_cmos_sensor(0x3376,0x0C); 
   OV2710_write_cmos_sensor(0x3377,0x10); 
   OV2710_write_cmos_sensor(0x3378,0x14); 
                                     
   OV2710_write_cmos_sensor(0x3012,0x02);
   OV2710_write_cmos_sensor(0x3013,0xD0);
}


/*************************************************************************
* FUNCTION
*	OV2710Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2710Open(void)
{
    spin_lock(&OV2710_yuv_drv_lock);
    sensor_id_fail = 0; 
    spin_unlock(&OV2710_yuv_drv_lock);
    SENSORDB("[Enter]:OV2710 Open func:");

    if (OV2710_power_on() != OV2710_SENSOR_ID) 
    {
        SENSORDB("[OV2710]Error:read sensor ID fail\n");
        spin_lock(&OV2710_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&OV2710_yuv_drv_lock);
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    /* Apply sensor initail setting*/
    OV2710_Initial_Setting();
    SENSORDB("wwm [Exit]:OV2710 Open func %x\n",OV2710_read_cmos_sensor(0x3271));     
    OV2710_Init_Parameter(); 

    SENSORDB("[Exit]:OV2710 Open func\n");     
    return ERROR_NONE;
}	/* OV2710Open() */

/*************************************************************************
* FUNCTION
*	OV2710_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 OV2710_GetSensorID(kal_uint32 *sensorID)
{
    SENSORDB("[Enter]:OV2710 Open func ");
    *sensorID = OV2710_power_on() ;

    if (*sensorID != OV2710_SENSOR_ID) 
    {
        SENSORDB("[OV2710]Error:read sensor ID fail\n");
        spin_lock(&OV2710_yuv_drv_lock);
        sensor_id_fail = 1;
        spin_unlock(&OV2710_yuv_drv_lock);
        *sensorID = 0xFFFFFFFF; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }	   

    return ERROR_NONE;    
}   /* OV2710Open  */


/*************************************************************************
* FUNCTION
*	OV2710Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV2710Close(void)
{

	return ERROR_NONE;
}	/* OV2710Close() */


static void OV2710_Set_Mirror_Flip(kal_uint16 image_mirror)
{
    /********************************************************
    * Page Mode 0: Reg 0x0011 bit[1:0] = [Y Flip : X Flip]
    * 0: Off; 1: On.
    *********************************************************/ 
    kal_uint16 temp_data;   
    SENSORDB("[Enter]:OV2710 set Mirror_flip func:image_mirror=%d\n",image_mirror);	
    OV2710_write_cmos_sensor(0x3022,0x26);     //Page 0	
    temp_data = (OV2710_read_cmos_sensor(0x3022) & 0xfc);
    spin_lock(&OV2710_yuv_drv_lock);
    //OV2710_sensor.mirror = (OV2710_read_cmos_sensor(0x11) & 0xfc); 
    switch (image_mirror) 
    {
    case IMAGE_NORMAL:
        //OV2710_sensor.mirror |= 0x00;
        temp_data |= 0x02;
        break;
    case IMAGE_H_MIRROR:
        //OV2710_sensor.mirror |= 0x01;
        temp_data |= 0x00;
        break;
    case IMAGE_V_MIRROR:
        //OV2710_sensor.mirror |= 0x02;
        temp_data |= 0x03;
        break;
    case IMAGE_HV_MIRROR:
        //OV2710_sensor.mirror |= 0x03;
        temp_data |= 0x01;
        break;
    default:
        //OV2710_sensor.mirror |= 0x00;
        temp_data |= 0x00;
    }
    OV2710_sensor.mirror = temp_data;
    spin_unlock(&OV2710_yuv_drv_lock);
    OV2710_write_cmos_sensor(0x3022, OV2710_sensor.mirror);
    SENSORDB("[Exit]:OV2710 set Mirror_flip func\n");
}

#if 0
static void OV2710_set_dummy(kal_uint16 dummy_pixels,kal_uint16 dummy_lines)
{	
   /* OV2710_write_cmos_sensor(0x03, 0x00);                        //Page 0
    OV2710_write_cmos_sensor(0x40,((dummy_pixels & 0x0F00))>>8);       //HBLANK
    OV2710_write_cmos_sensor(0x41,(dummy_pixels & 0xFF));
    OV2710_write_cmos_sensor(0x42,((dummy_lines & 0xFF00)>>8));       //VBLANK ( Vsync Type 1)
    OV2710_write_cmos_sensor(0x43,(dummy_lines & 0xFF));*/
}  
#endif

static void OV2710_preview(void)
{
#if 0
   OV2710_write_cmos_sensor(0x320A, 0x00);
   OV2710_write_cmos_sensor(0x32BF, 0x60); 
   OV2710_write_cmos_sensor(0x32C0, 0x7A); 
   OV2710_write_cmos_sensor(0x32C1, 0x6C); 
   OV2710_write_cmos_sensor(0x32C2, 0x7A); 
   OV2710_write_cmos_sensor(0x32C3, 0x00); 
   OV2710_write_cmos_sensor(0x32C4, 0x20); 
   OV2710_write_cmos_sensor(0x32C5, 0x00); 
   OV2710_write_cmos_sensor(0x32C6, 0x18); 
   OV2710_write_cmos_sensor(0x32C7, 0x00); 
   OV2710_write_cmos_sensor(0x32C8, 0x91); 
   OV2710_write_cmos_sensor(0x32C9, 0x6C); 
   OV2710_write_cmos_sensor(0x32CA, 0x6C); 
   OV2710_write_cmos_sensor(0x32CB, 0x7A); 
   OV2710_write_cmos_sensor(0x32CC, 0x92); 
   OV2710_write_cmos_sensor(0x32CD, 0x92); 
   OV2710_write_cmos_sensor(0x32DB, 0x72); 
   OV2710_write_cmos_sensor(0x3200, 0x3E); 
   OV2710_write_cmos_sensor(0x3201, 0x0F); 
   OV2710_write_cmos_sensor(0x3028, 0x07); 
   OV2710_write_cmos_sensor(0x3029, 0x00); 
   OV2710_write_cmos_sensor(0x302A, 0x04); 
   OV2710_write_cmos_sensor(0x3022, 0x24); 
   OV2710_write_cmos_sensor(0x3023, 0x24); 
   OV2710_write_cmos_sensor(0x3002, 0x00); 
   OV2710_write_cmos_sensor(0x3003, 0x04); 
   OV2710_write_cmos_sensor(0x3004, 0x00); 
   OV2710_write_cmos_sensor(0x3005, 0x04); 
   OV2710_write_cmos_sensor(0x3006, 0x05); 
   OV2710_write_cmos_sensor(0x3007, 0x03); 
   OV2710_write_cmos_sensor(0x3008, 0x02); 
   OV2710_write_cmos_sensor(0x3009, 0xD3); 
   OV2710_write_cmos_sensor(0x300A, 0x06); 
   OV2710_write_cmos_sensor(0x300B, 0x7C); 
   OV2710_write_cmos_sensor(0x300C, 0x02); 
   OV2710_write_cmos_sensor(0x300D, 0xE0); 
   OV2710_write_cmos_sensor(0x300E, 0x05); 
   OV2710_write_cmos_sensor(0x300F, 0x00); 
   OV2710_write_cmos_sensor(0x3010, 0x02); 
   OV2710_write_cmos_sensor(0x3011, 0xD0); 
   OV2710_write_cmos_sensor(0x32B8, 0x3F); 
   OV2710_write_cmos_sensor(0x32B9, 0x31); 
   OV2710_write_cmos_sensor(0x32BB, 0x87); 
   OV2710_write_cmos_sensor(0x32BC, 0x38); 
   OV2710_write_cmos_sensor(0x32BD, 0x3C); 
   OV2710_write_cmos_sensor(0x32BE, 0x34); 
   OV2710_write_cmos_sensor(0x3201, 0x3F); 
   OV2710_write_cmos_sensor(0x3021, 0x06); 
   OV2710_write_cmos_sensor(0x3060, 0x01);
#else
OV2710_write_cmos_sensor(0x320A, 0x00);//linebuffer off.
    OV2710_write_cmos_sensor(0x32BF, 0x60); 
    OV2710_write_cmos_sensor(0x32C0, 0x60); 
    OV2710_write_cmos_sensor(0x32C1, 0x60); 
    OV2710_write_cmos_sensor(0x32C2, 0x60); 
    OV2710_write_cmos_sensor(0x32C3, 0x00); 
    OV2710_write_cmos_sensor(0x32C4, 0x20); 
    OV2710_write_cmos_sensor(0x32C5, 0x20); 
    OV2710_write_cmos_sensor(0x32C6, 0x20); 
    OV2710_write_cmos_sensor(0x32C7, 0x00); 
    OV2710_write_cmos_sensor(0x32C8, 0xDE); 
    OV2710_write_cmos_sensor(0x32C9, 0x60); 
    OV2710_write_cmos_sensor(0x32CA, 0x80); 
    OV2710_write_cmos_sensor(0x32CB, 0x80); 
    OV2710_write_cmos_sensor(0x32CC, 0x80); 
    OV2710_write_cmos_sensor(0x32CD, 0x80); 
    OV2710_write_cmos_sensor(0x32DB, 0x7B); 
    OV2710_write_cmos_sensor(0x32E0, 0x05); 
    OV2710_write_cmos_sensor(0x32E1, 0x00); 
    OV2710_write_cmos_sensor(0x32E2, 0x02); 
    OV2710_write_cmos_sensor(0x32E3, 0xD0); 
    OV2710_write_cmos_sensor(0x32E4, 0x00); 
    OV2710_write_cmos_sensor(0x32E5, 0x00); 
    OV2710_write_cmos_sensor(0x32E6, 0x00); 
    OV2710_write_cmos_sensor(0x32E7, 0x00); 
    OV2710_write_cmos_sensor(0x3200, 0x3E); 
    OV2710_write_cmos_sensor(0x3201, 0x0F); 
    OV2710_write_cmos_sensor(0x3028, 0x24); 
    OV2710_write_cmos_sensor(0x3029, 0x20); 
    OV2710_write_cmos_sensor(0x302A, 0x04); 
    OV2710_write_cmos_sensor(0x3022, 0x24);
    OV2710_write_cmos_sensor(0x3023, 0x24); 
    OV2710_write_cmos_sensor(0x3002, 0x00); 
    OV2710_write_cmos_sensor(0x3003, 0x04); 
    OV2710_write_cmos_sensor(0x3004, 0x00); 
    OV2710_write_cmos_sensor(0x3005, 0x04); 
    OV2710_write_cmos_sensor(0x3006, 0x05); 
    OV2710_write_cmos_sensor(0x3007, 0x03); 
    OV2710_write_cmos_sensor(0x3008, 0x02); 
    OV2710_write_cmos_sensor(0x3009, 0xD3); 
    OV2710_write_cmos_sensor(0x300A, 0x06); 
    OV2710_write_cmos_sensor(0x300B, 0x82); 
    OV2710_write_cmos_sensor(0x300C, 0x02); 
    OV2710_write_cmos_sensor(0x300D, 0xE4); 
    OV2710_write_cmos_sensor(0x300E, 0x05); 
    OV2710_write_cmos_sensor(0x300F, 0x00); 
    OV2710_write_cmos_sensor(0x3010, 0x02); 
    OV2710_write_cmos_sensor(0x3011, 0xD0); 
    OV2710_write_cmos_sensor(0x32B8, 0x3F); 
    OV2710_write_cmos_sensor(0x32B9, 0x31); 
    OV2710_write_cmos_sensor(0x32BB, 0x87); 
    OV2710_write_cmos_sensor(0x32BC, 0x38); 
    OV2710_write_cmos_sensor(0x32BD, 0x3C); 
    OV2710_write_cmos_sensor(0x32BE, 0x34); 
    OV2710_write_cmos_sensor(0x3201, 0x7F); 
    OV2710_write_cmos_sensor(0x3021, 0x06); 
    OV2710_write_cmos_sensor(0x3060, 0x01);
#endif

}

static void OV2710_Capture(void)
{
//[YUYV_1600x1200_8.33_10.00_Fps_50Hz]
#if 0
   OV2710_write_cmos_sensor(0x320A, 0x00);
   OV2710_write_cmos_sensor(0x32BF, 0x60); 
   OV2710_write_cmos_sensor(0x32C0, 0x7A); 
   OV2710_write_cmos_sensor(0x32C1, 0x6C); 
   OV2710_write_cmos_sensor(0x32C2, 0x7A); 
   OV2710_write_cmos_sensor(0x32C3, 0x00); 
   OV2710_write_cmos_sensor(0x32C4, 0x20); 
   OV2710_write_cmos_sensor(0x32C5, 0x00); 
   OV2710_write_cmos_sensor(0x32C6, 0x18); 
   OV2710_write_cmos_sensor(0x32C7, 0x00); 
   OV2710_write_cmos_sensor(0x32C8, 0x91); 
   OV2710_write_cmos_sensor(0x32C9, 0x6C); 
   OV2710_write_cmos_sensor(0x32CA, 0x6C); 
   OV2710_write_cmos_sensor(0x32CB, 0x7A); 
   OV2710_write_cmos_sensor(0x32CC, 0x92); 
   OV2710_write_cmos_sensor(0x32CD, 0x92); 
   OV2710_write_cmos_sensor(0x32DB, 0x72); 
   OV2710_write_cmos_sensor(0x3200, 0x3E); 
   OV2710_write_cmos_sensor(0x3201, 0x0F); 
   OV2710_write_cmos_sensor(0x3028, 0x07); 
   OV2710_write_cmos_sensor(0x3029, 0x00); 
   OV2710_write_cmos_sensor(0x302A, 0x04); 
   OV2710_write_cmos_sensor(0x3022, 0x24); 
   OV2710_write_cmos_sensor(0x3023, 0x24); 
   OV2710_write_cmos_sensor(0x3002, 0x00); 
   OV2710_write_cmos_sensor(0x3003, 0x04); 
   OV2710_write_cmos_sensor(0x3004, 0x00); 
   OV2710_write_cmos_sensor(0x3005, 0x04); 
   OV2710_write_cmos_sensor(0x3006, 0x05); 
   OV2710_write_cmos_sensor(0x3007, 0x03); 
   OV2710_write_cmos_sensor(0x3008, 0x02); 
   OV2710_write_cmos_sensor(0x3009, 0xD3); 
   OV2710_write_cmos_sensor(0x300A, 0x06); 
   OV2710_write_cmos_sensor(0x300B, 0x7C); 
   OV2710_write_cmos_sensor(0x300C, 0x02); 
   OV2710_write_cmos_sensor(0x300D, 0xE0); 
   OV2710_write_cmos_sensor(0x300E, 0x05); 
   OV2710_write_cmos_sensor(0x300F, 0x00); 
   OV2710_write_cmos_sensor(0x3010, 0x02); 
   OV2710_write_cmos_sensor(0x3011, 0xD0); 
   OV2710_write_cmos_sensor(0x32B8, 0x3F); 
   OV2710_write_cmos_sensor(0x32B9, 0x31); 
   OV2710_write_cmos_sensor(0x32BB, 0x87); 
   OV2710_write_cmos_sensor(0x32BC, 0x38); 
   OV2710_write_cmos_sensor(0x32BD, 0x3C); 
   OV2710_write_cmos_sensor(0x32BE, 0x34); 
   OV2710_write_cmos_sensor(0x3201, 0x3F); 
   OV2710_write_cmos_sensor(0x3021, 0x06); 
   OV2710_write_cmos_sensor(0x3060, 0x01);
#else
    OV2710_write_cmos_sensor(0x320A, 0x00);//linebuffer off.
    OV2710_write_cmos_sensor(0x32BF, 0x60); 
    OV2710_write_cmos_sensor(0x32C0, 0x78); 
    OV2710_write_cmos_sensor(0x32C1, 0x78); 
    OV2710_write_cmos_sensor(0x32C2, 0x78); 
    OV2710_write_cmos_sensor(0x32C3, 0x00); 
    OV2710_write_cmos_sensor(0x32C4, 0x20); 
    OV2710_write_cmos_sensor(0x32C5, 0x20); 
    OV2710_write_cmos_sensor(0x32C6, 0x20); 
    OV2710_write_cmos_sensor(0x32C7, 0x00); 
    OV2710_write_cmos_sensor(0x32C8, 0xDE); 
    OV2710_write_cmos_sensor(0x32C9, 0x78); 
    OV2710_write_cmos_sensor(0x32CA, 0x98); 
    OV2710_write_cmos_sensor(0x32CB, 0x98); 
    OV2710_write_cmos_sensor(0x32CC, 0x98); 
    OV2710_write_cmos_sensor(0x32CD, 0x98); 
    OV2710_write_cmos_sensor(0x32DB, 0x7B); 
    OV2710_write_cmos_sensor(0x32E0, 0x05); 
    OV2710_write_cmos_sensor(0x32E1, 0x00); 
    OV2710_write_cmos_sensor(0x32E2, 0x02); 
    OV2710_write_cmos_sensor(0x32E3, 0xD0); 
    OV2710_write_cmos_sensor(0x32E4, 0x00); 
    OV2710_write_cmos_sensor(0x32E5, 0x00); 
    OV2710_write_cmos_sensor(0x32E6, 0x00); 
    OV2710_write_cmos_sensor(0x32E7, 0x00); 
    OV2710_write_cmos_sensor(0x3200, 0x3E); 
    OV2710_write_cmos_sensor(0x3201, 0x0F); 
    OV2710_write_cmos_sensor(0x3028, 0x24); 
    OV2710_write_cmos_sensor(0x3029, 0x20); 
    OV2710_write_cmos_sensor(0x302A, 0x04); 
    OV2710_write_cmos_sensor(0x3022, 0x24);
    OV2710_write_cmos_sensor(0x3023, 0x24); 
    OV2710_write_cmos_sensor(0x3002, 0x00); 
    OV2710_write_cmos_sensor(0x3003, 0x04); 
    OV2710_write_cmos_sensor(0x3004, 0x00); 
    OV2710_write_cmos_sensor(0x3005, 0x04); 
    OV2710_write_cmos_sensor(0x3006, 0x05); 
    OV2710_write_cmos_sensor(0x3007, 0x03); 
    OV2710_write_cmos_sensor(0x3008, 0x02); 
    OV2710_write_cmos_sensor(0x3009, 0xD3); 
    OV2710_write_cmos_sensor(0x300A, 0x06); 
    OV2710_write_cmos_sensor(0x300B, 0x82); 
    OV2710_write_cmos_sensor(0x300C, 0x08); 
    OV2710_write_cmos_sensor(0x300D, 0xAC); 
    OV2710_write_cmos_sensor(0x300E, 0x05); 
    OV2710_write_cmos_sensor(0x300F, 0x00); 
    OV2710_write_cmos_sensor(0x3010, 0x02); 
    OV2710_write_cmos_sensor(0x3011, 0xD0); 
    OV2710_write_cmos_sensor(0x32B8, 0x3F); 
    OV2710_write_cmos_sensor(0x32B9, 0x31); 
    OV2710_write_cmos_sensor(0x32BB, 0x87); 
    OV2710_write_cmos_sensor(0x32BC, 0x38); 
    OV2710_write_cmos_sensor(0x32BD, 0x3C); 
    OV2710_write_cmos_sensor(0x32BE, 0x34); 
    OV2710_write_cmos_sensor(0x3201, 0x7F); 
    OV2710_write_cmos_sensor(0x3021, 0x06); 
    OV2710_write_cmos_sensor(0x3060, 0x01); 

#endif


}

static void OV2710_Cal_Min_Frame_Rate(kal_uint16 min_framerate)
{
  
}


static void OV2710_Fix_Video_Frame_Rate(kal_uint16 fix_framerate)
{
 
}


void OV2710_night_mode(kal_bool enable)
{
    SENSORDB("[Enter]OV2710 night mode func:enable = %d\n",enable);
    SENSORDB("OV2710_sensor.video_mode = %d\n",OV2710_sensor.MPEG4_Video_mode); 
    SENSORDB("OV2710_sensor.night_mode = %d\n",OV2710_sensor.night_mode);
    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.night_mode = enable;
    spin_unlock(&OV2710_yuv_drv_lock);

    if(OV2710_sensor.MPEG4_Video_mode == KAL_TRUE)
        return;

    if(enable)
    {
        OV2710_write_cmos_sensor(0x32C4, 0x28);
				OV2710_write_cmos_sensor(0x302A, 0x08);                         
    }
    else
    {
        OV2710_write_cmos_sensor(0x32C4, 0x20);
			  OV2710_write_cmos_sensor(0x302A, 0x04);
    }
}

/*************************************************************************
* FUNCTION
*	OV2710Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 OV2710Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    spin_lock(&OV2710_yuv_drv_lock);
    sensor_config_data->SensorImageMirror = IMAGE_HV_MIRROR; 
    if(OV2710_sensor.first_init == KAL_TRUE)
    {
        OV2710_sensor.MPEG4_Video_mode = OV2710_sensor.MPEG4_Video_mode;
    }
    else
    {
        OV2710_sensor.MPEG4_Video_mode = !OV2710_sensor.MPEG4_Video_mode;
    }
    spin_unlock(&OV2710_yuv_drv_lock);

    SENSORDB("[Enter]:OV2710 preview func:");		
    SENSORDB("OV2710_sensor.video_mode = %d\n",OV2710_sensor.MPEG4_Video_mode); 

    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.first_init = KAL_FALSE;	
    OV2710_sensor.pv_mode = KAL_TRUE;		
    spin_unlock(&OV2710_yuv_drv_lock);

    {   
        SENSORDB("[OV2710]preview mode\n");
        OV2710_preview();
    SENSORDB("wwm [exit] preview  %x\n",OV2710_read_cmos_sensor(0x32bc));     
    }

    //OV2710_Set_Mirror_Flip(sensor_config_data->SensorImageMirror);

    SENSORDB("[Exit]:OV2710 preview func\n");
    return TRUE; 
}	/* OV2710_Preview */


UINT32 OV2710Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("[OV2710][Enter]OV2710_capture_func\n");
    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.pv_mode = KAL_FALSE;	
    spin_unlock(&OV2710_yuv_drv_lock);
 		{   
        SENSORDB("[OV2710]preview mode\n");
        OV2710_Capture();
    SENSORDB("wwm [exit] Capture  %x\n",OV2710_read_cmos_sensor(0x32c4));     
    }
    return ERROR_NONE;
}	/* OV2710Capture() */


UINT32 OV2710GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[Enter]:OV2710 get Resolution func\n");

    pSensorResolution->SensorFullWidth=OV2710_IMAGE_SENSOR_FULL_WIDTH - 10;  
    pSensorResolution->SensorFullHeight=OV2710_IMAGE_SENSOR_FULL_HEIGHT - 10-10;
    pSensorResolution->SensorPreviewWidth=OV2710_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->SensorPreviewHeight=OV2710_IMAGE_SENSOR_PV_HEIGHT - 12-10;
    pSensorResolution->SensorVideoWidth=OV2710_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->SensorVideoHeight=OV2710_IMAGE_SENSOR_PV_HEIGHT - 12-10;
    pSensorResolution->Sensor3DFullWidth=OV2710_IMAGE_SENSOR_FULL_WIDTH - 10;  
    pSensorResolution->Sensor3DFullHeight=OV2710_IMAGE_SENSOR_FULL_HEIGHT - 10-10;
    pSensorResolution->Sensor3DPreviewWidth=OV2710_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->Sensor3DPreviewHeight=OV2710_IMAGE_SENSOR_PV_HEIGHT - 12-10;
    pSensorResolution->Sensor3DVideoWidth=OV2710_IMAGE_SENSOR_PV_WIDTH - 16;
    pSensorResolution->Sensor3DVideoHeight=OV2710_IMAGE_SENSOR_PV_HEIGHT - 12-10;

    SENSORDB("[Exit]:OV2710 get Resolution func\n");	
    return ERROR_NONE;
}	/* OV2710GetResolution() */

UINT32 OV2710GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[Enter]:OV2710 getInfo func:ScenarioId = %d\n",ScenarioId);

    pSensorInfo->SensorPreviewResolutionX=OV2710_IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=OV2710_IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=OV2710_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=OV2710_IMAGE_SENSOR_FULL_HEIGHT;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=30;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;//low is to reset 
    pSensorInfo->SensorResetDelayCount=4;  //4ms 
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV; //SENSOR_OUTPUT_FORMAT_YVYU;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;// SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1; 
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;


    pSensorInfo->CaptureDelayFrame = 4; 
    pSensorInfo->PreviewDelayFrame = 4;//10; 
    pSensorInfo->VideoDelayFrame = 0; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;   	

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:		
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;  	
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=	3;
        pSensorInfo->SensorClockRisingCount= 0;
        pSensorInfo->SensorClockFallingCount= 2;
        pSensorInfo->SensorPixelClockCount= 3;
        pSensorInfo->SensorDataLatchCount= 2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;//1;     			
        break;
    default:
        pSensorInfo->SensorClockFreq=24;
        pSensorInfo->SensorClockDividCount=3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=3;
        pSensorInfo->SensorDataLatchCount=2;
        pSensorInfo->SensorGrabStartX = 1; 
        pSensorInfo->SensorGrabStartY = 10;//1;     			
        break;
    }
    //	OV2710_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &OV2710SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    SENSORDB("[Exit]:OV2710 getInfo func\n");	
    return ERROR_NONE;
}	/* OV2710GetInfo() */


UINT32 OV2710Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    SENSORDB("[Enter]:OV2710 Control func:ScenarioId = %d\n",ScenarioId);

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
    //case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:		
        OV2710Preview(pImageWindow, pSensorConfigData); 
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:
        OV2710Capture(pImageWindow, pSensorConfigData); 
        break;
    default:
        break; 
    }

    SENSORDB("[Exit]:OV2710 Control func\n");	
    return TRUE;
}	/* OV2710Control() */


/*************************************************************************
* FUNCTION
*	OV2710_set_param_wb
*
* DESCRIPTION
*	wb setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV2710_set_param_wb(UINT16 para)

{
    //This sensor need more time to balance AWB, 
    //we suggest higher fps or drop some frame to avoid garbage color when preview initial
    SENSORDB("[Enter]OV2710 set_param_wb func:para = %d\n",para);

    if(OV2710_sensor.wb == para) return KAL_TRUE;	

    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.wb = para;
    spin_unlock(&OV2710_yuv_drv_lock);
    
    switch (para)
    {          
    case AWB_MODE_AUTO:			
			      OV2710_AWB_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			      OV2710_AWB_enable(KAL_FALSE);
			      OV2710_write_cmos_sensor(0x3290, 0x01); //manual R G B
            OV2710_write_cmos_sensor(0x3291, 0x51);
            OV2710_write_cmos_sensor(0x3296, 0x01);
            OV2710_write_cmos_sensor(0x3297, 0x00);
						OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny
			      OV2710_AWB_enable(KAL_FALSE);
			      OV2710_write_cmos_sensor(0x3290, 0x01); //manual R G B
            OV2710_write_cmos_sensor(0x3291, 0x38);
            OV2710_write_cmos_sensor(0x3296, 0x01);
            OV2710_write_cmos_sensor(0x3297, 0x68);
			      OV2710_write_cmos_sensor(0x3060, 0x01);		
		break;
		
		case AWB_MODE_INCANDESCENT: //office
			      OV2710_AWB_enable(KAL_FALSE);
						OV2710_write_cmos_sensor(0x3290, 0x01); //manual R G B
            OV2710_write_cmos_sensor(0x3291, 0x30);
            OV2710_write_cmos_sensor(0x3296, 0x01);
            OV2710_write_cmos_sensor(0x3297, 0xCB);
						OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case AWB_MODE_TUNGSTEN: //home
			      OV2710_AWB_enable(KAL_FALSE);
						OV2710_write_cmos_sensor(0x3290, 0x01); //manual R G B
            OV2710_write_cmos_sensor(0x3291, 0x00);
            OV2710_write_cmos_sensor(0x3296, 0x02);
            OV2710_write_cmos_sensor(0x3297, 0x30);
			      OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case AWB_MODE_FLUORESCENT:
			      OV2710_AWB_enable(KAL_FALSE);
						OV2710_write_cmos_sensor(0x3290, 0x01); //manual R G B
            OV2710_write_cmos_sensor(0x3291, 0x70);
            OV2710_write_cmos_sensor(0x3296, 0x01);
            OV2710_write_cmos_sensor(0x3297, 0xFF);
						OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		default:
		return FALSE;
		
	
	}

	return TRUE;
} /* OV2710_set_param_wb */

/*************************************************************************
* FUNCTION
*	OV2710_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV2710_set_param_effect(UINT16 para)
{
   SENSORDB("[Enter]OV2710 set_param_effect func:para = %d\n",para);
   
    if(OV2710_sensor.effect == para) return KAL_TRUE;

    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.effect = para;
    spin_unlock(&OV2710_yuv_drv_lock);
    
	 	kal_uint32  ret = KAL_TRUE;
	 	UINT8  temp_reg;
	 	
	  temp_reg = OV2710_read_cmos_sensor(0x32f1);
	  temp_reg = temp_reg &0xF8;
   
#if 1
	switch (para)
	{
		 				case MEFFECT_OFF:
            OV2710_write_cmos_sensor(0x32f1,0x00);
            OV2710_write_cmos_sensor(0x32f4,0x80);
            OV2710_write_cmos_sensor(0x32f5,0x80);
            break;

            case MEFFECT_SEPIA:
            OV2710_write_cmos_sensor(0x32f1,temp_reg|0x02);
            break;

            case MEFFECT_NEGATIVE:
            OV2710_write_cmos_sensor(0x32f1,temp_reg|0x03);
            break;

            case MEFFECT_SEPIAGREEN:
            OV2710_write_cmos_sensor(0x32f1,temp_reg|0x05);
            OV2710_write_cmos_sensor(0x32f4,0x60);
            OV2710_write_cmos_sensor(0x32f5,0x20);
            break;

            case MEFFECT_SEPIABLUE:
            OV2710_write_cmos_sensor(0x32f1,temp_reg|0x05);
            OV2710_write_cmos_sensor(0x32f4,0xf0);
            OV2710_write_cmos_sensor(0x32f5,0x80);
            break;
            case MEFFECT_MONO: //B&W
            OV2710_write_cmos_sensor(0x32f1,temp_reg|0x01);
	    			break;
		default:
			ret = FALSE;
	}
#else
	switch (para)
	{
		case MEFFECT_OFF:
	OV2710_write_cmos_sensor(0x3270, 0x00); // Gamma_9712
	OV2710_write_cmos_sensor(0x3271, 0x0C);
	OV2710_write_cmos_sensor(0x3272, 0x18);
	OV2710_write_cmos_sensor(0x3273, 0x32);
	OV2710_write_cmos_sensor(0x3274, 0x44);
	OV2710_write_cmos_sensor(0x3275, 0x54);
	OV2710_write_cmos_sensor(0x3276, 0x70);
	OV2710_write_cmos_sensor(0x3277, 0x88);
	OV2710_write_cmos_sensor(0x3278, 0x9D);
	OV2710_write_cmos_sensor(0x3279, 0xB0);
	OV2710_write_cmos_sensor(0x327A, 0xCF);
	OV2710_write_cmos_sensor(0x327B, 0xE2);
	OV2710_write_cmos_sensor(0x327C, 0xEF);
	OV2710_write_cmos_sensor(0x327D, 0xF7);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case MEFFECT_SEPIA:

	OV2710_write_cmos_sensor(0x3270, 0x00); // Gamma3
	OV2710_write_cmos_sensor(0x3271, 0x0B);
	OV2710_write_cmos_sensor(0x3272, 0x16);
	OV2710_write_cmos_sensor(0x3273, 0x2B);
	OV2710_write_cmos_sensor(0x3274, 0x3F);
	OV2710_write_cmos_sensor(0x3275, 0x51);
	OV2710_write_cmos_sensor(0x3276, 0x72);
	OV2710_write_cmos_sensor(0x3277, 0x8F);
	OV2710_write_cmos_sensor(0x3278, 0xA7);
	OV2710_write_cmos_sensor(0x3279, 0xBC);
	OV2710_write_cmos_sensor(0x327A, 0xDC);
	OV2710_write_cmos_sensor(0x327B, 0xF0);
	OV2710_write_cmos_sensor(0x327C, 0xFA);
	OV2710_write_cmos_sensor(0x327D, 0xFE);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case MEFFECT_NEGATIVE:
	
	OV2710_write_cmos_sensor(0x3270, 0x08); // Gamma_5
	OV2710_write_cmos_sensor(0x3271, 0x14);
	OV2710_write_cmos_sensor(0x3272, 0x20);
	OV2710_write_cmos_sensor(0x3273, 0x36);
	OV2710_write_cmos_sensor(0x3274, 0x4b);
	OV2710_write_cmos_sensor(0x3275, 0x5D);
	OV2710_write_cmos_sensor(0x3276, 0x7E);
	OV2710_write_cmos_sensor(0x3277, 0x98);
	OV2710_write_cmos_sensor(0x3278, 0xAC);
	OV2710_write_cmos_sensor(0x3279, 0xBD);
	OV2710_write_cmos_sensor(0x327A, 0xD4);
	OV2710_write_cmos_sensor(0x327B, 0xE5);
	OV2710_write_cmos_sensor(0x327C, 0xF0);
	OV2710_write_cmos_sensor(0x327D, 0xF9);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);		
		break;
		
		case MEFFECT_SEPIAGREEN:
	OV2710_write_cmos_sensor(0x3270, 0x00); // Gamma_Winson
	OV2710_write_cmos_sensor(0x3271, 0x08);
	OV2710_write_cmos_sensor(0x3272, 0x10);
	OV2710_write_cmos_sensor(0x3273, 0x20);
	OV2710_write_cmos_sensor(0x3274, 0x32);
	OV2710_write_cmos_sensor(0x3275, 0x44);
	OV2710_write_cmos_sensor(0x3276, 0x62);
	OV2710_write_cmos_sensor(0x3277, 0x7b);
	OV2710_write_cmos_sensor(0x3278, 0x92);
	OV2710_write_cmos_sensor(0x3279, 0xA8);
	OV2710_write_cmos_sensor(0x327A, 0xCD);
	OV2710_write_cmos_sensor(0x327B, 0xE7);
	OV2710_write_cmos_sensor(0x327C, 0xF5);
	OV2710_write_cmos_sensor(0x327D, 0xFA);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		
		case MEFFECT_SEPIABLUE:
	OV2710_write_cmos_sensor(0x3270, 0x00); // Gamma_9712_2
	OV2710_write_cmos_sensor(0x3271, 0x08);
	OV2710_write_cmos_sensor(0x3272, 0x13);
	OV2710_write_cmos_sensor(0x3273, 0x2b);
	OV2710_write_cmos_sensor(0x3274, 0x41);
	OV2710_write_cmos_sensor(0x3275, 0x53);
	OV2710_write_cmos_sensor(0x3276, 0x72);
	OV2710_write_cmos_sensor(0x3277, 0x8B);
	OV2710_write_cmos_sensor(0x3278, 0x9E);
	OV2710_write_cmos_sensor(0x3279, 0xB3);
	OV2710_write_cmos_sensor(0x327A, 0xCF);
	OV2710_write_cmos_sensor(0x327B, 0xE2);
	OV2710_write_cmos_sensor(0x327C, 0xEF);
	OV2710_write_cmos_sensor(0x327D, 0xF7);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);
	

		break;

		case MEFFECT_MONO:
	OV2710_write_cmos_sensor(0x3270, 0x00); // Gamma
	OV2710_write_cmos_sensor(0x3271, 0x0D);
	OV2710_write_cmos_sensor(0x3272, 0x22);
	OV2710_write_cmos_sensor(0x3273, 0x41);
	OV2710_write_cmos_sensor(0x3274, 0x5E);
	OV2710_write_cmos_sensor(0x3275, 0x72);
	OV2710_write_cmos_sensor(0x3276, 0x8F);
	OV2710_write_cmos_sensor(0x3277, 0xA4);
	OV2710_write_cmos_sensor(0x3278, 0xb6);
	OV2710_write_cmos_sensor(0x3279, 0xC6);
	OV2710_write_cmos_sensor(0x327A, 0xDE);
	OV2710_write_cmos_sensor(0x327B, 0xF2);
	OV2710_write_cmos_sensor(0x327C, 0xFc);
	OV2710_write_cmos_sensor(0x327D, 0xFF);
	OV2710_write_cmos_sensor(0x327E, 0xFF);
	OV2710_write_cmos_sensor(0x3060, 0x01);
		break;
		default:
			ret = FALSE;
	}

#endif
	return ret;
} /* OV2710_set_param_effect */

/*************************************************************************
* FUNCTION
*	OV2710_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV2710_set_param_banding(UINT16 para)
{
    SENSORDB("[Enter]OV2710 set_param_banding func:para = %d\n",para);

    if(OV2710_sensor.banding == para) return KAL_TRUE;

    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.banding = para;
    spin_unlock(&OV2710_yuv_drv_lock);

    switch (para)
    {
    case AE_FLICKER_MODE_50HZ:
        {
				   OV2710_write_cmos_sensor(0x32BF, 0x60);  		
				   OV2710_write_cmos_sensor(0x32C0, 0x7A);  
				   OV2710_write_cmos_sensor(0x32C1, 0x7A);  
				   OV2710_write_cmos_sensor(0x32C2, 0x7A);  
				   OV2710_write_cmos_sensor(0x32C3, 0x00);    
				   OV2710_write_cmos_sensor(0x32C7, 0x00);  
				   OV2710_write_cmos_sensor(0x32C8, 0x91);  
				   OV2710_write_cmos_sensor(0x32C9, 0x7A);  
				   OV2710_write_cmos_sensor(0x32CA, 0x9A);  
				   OV2710_write_cmos_sensor(0x32CB, 0x9A);  
				   OV2710_write_cmos_sensor(0x32CC, 0x9A);  
				   OV2710_write_cmos_sensor(0x32CD, 0x9A);  
				   OV2710_write_cmos_sensor(0x32DB, 0x72);       
        }
        break;
    case AE_FLICKER_MODE_60HZ:
        {
			   OV2710_write_cmos_sensor(0x32BF, 0x60); 
			   OV2710_write_cmos_sensor(0x32C0, 0x80); 
			   OV2710_write_cmos_sensor(0x32C1, 0x7F); 
			   OV2710_write_cmos_sensor(0x32C2, 0x7F); 
			   OV2710_write_cmos_sensor(0x32C3, 0x00);  
			   OV2710_write_cmos_sensor(0x32C7, 0x00); 
			   OV2710_write_cmos_sensor(0x32C8, 0x78); 
			   OV2710_write_cmos_sensor(0x32C9, 0x7F); 
			   OV2710_write_cmos_sensor(0x32CA, 0x9F); 
			   OV2710_write_cmos_sensor(0x32CB, 0x9F); 
			   OV2710_write_cmos_sensor(0x32CC, 0x9F); 
			   OV2710_write_cmos_sensor(0x32CD, 0xA0); 
			   OV2710_write_cmos_sensor(0x32DB, 0x6E); 
        }
        break;
    default:
        return KAL_FALSE;
    }
    
    return KAL_TRUE;
} /* OV2710_set_param_banding */




/*************************************************************************
* FUNCTION
*	OV2710_set_param_exposure
*
* DESCRIPTION
*	exposure setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV2710_set_param_exposure(UINT16 para)
{
    SENSORDB("[Enter]OV2710 set_param_exposure func:para = %d\n",para);

    if(OV2710_sensor.exposure == para) return KAL_TRUE;

    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.exposure = para;
    spin_unlock(&OV2710_yuv_drv_lock);

    switch (para)
    {
    case AE_EV_COMP_13:  //+4 EV
				OV2710_write_cmos_sensor(0x32f2, 0xc0);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;  
    case AE_EV_COMP_10:  //+3 EV
				OV2710_write_cmos_sensor(0x32f2, 0xb0);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_07:  //+2 EV
				OV2710_write_cmos_sensor(0x32f2, 0xa0);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_03:	 //	+1 EV	
				OV2710_write_cmos_sensor(0x32f2, 0x90);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_00:  // +0 EV
				OV2710_write_cmos_sensor(0x32f2, 0x80);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_n03:  // -1 EV
				OV2710_write_cmos_sensor(0x32f2, 0x70);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_n07:	// -2 EV		
				OV2710_write_cmos_sensor(0x32f2, 0x60);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;    
    case AE_EV_COMP_n10:   //-3 EV
				OV2710_write_cmos_sensor(0x32f2, 0x50);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;
    case AE_EV_COMP_n13:  // -4 EV
				OV2710_write_cmos_sensor(0x32f2, 0x40);
				OV2710_write_cmos_sensor(0x32F6, 0x0F);
        break;
    default:
        return FALSE;
    }

    return TRUE;	
} /* OV2710_set_param_exposure */

void OV2710_set_AE_mode(UINT32 iPara)
{
/*    SENSORDB("[Enter]OV2710_set_AE_mode func:iPara = %d\n",iPara);
    switch (iPara)
    { 
    case AE_Average:  //AE_mode:Average
				OV2710_write_cmos_sensor(0x32B0,0x55);
				OV2710_write_cmos_sensor(0x32B1,0xAA);
				OV2710_write_cmos_sensor(0x32B2,0x10);
        break;    
    case AE_Center:  //AE_mode:Center
				OV2710_write_cmos_sensor(0x32B0,0x55);
				OV2710_write_cmos_sensor(0x32B1,0xAA);
				OV2710_write_cmos_sensor(0x32B2,0x14);
        break;    
    case AE_Spot:	 //AE_mode:Spot
				OV2710_write_cmos_sensor(0x32B0,0x77);
				OV2710_write_cmos_sensor(0x32B1,0x99);
				OV2710_write_cmos_sensor(0x32B2,0x00);
        break;    
    default:
        return FALSE;    
    }
     return TRUE;
*/
}
UINT32 OV2710YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    SENSORDB("[Enter]OV2710YUVSensorSetting func:cmd = %d\n",iCmd);

    switch (iCmd) 
    {
    case FID_SCENE_MODE:	    //auto mode or night mode
        if (iPara == SCENE_MODE_OFF)//auto mode
        {
            OV2710_night_mode(FALSE); 
        }
        else if (iPara == SCENE_MODE_NIGHTSCENE)//night mode
        {
            OV2710_night_mode(TRUE); 
        }	
        break; 	    
    case FID_AWB_MODE:
        OV2710_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        OV2710_set_param_effect(iPara);
        break;
    case FID_AE_EV:	    	    
        OV2710_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:	    	    	    
        OV2710_set_param_banding(iPara);
        break;
    case FID_ZOOM_FACTOR:
        spin_lock(&OV2710_yuv_drv_lock);
        OV2710_zoom_factor = iPara; 
        spin_unlock(&OV2710_yuv_drv_lock);
        break; 
    case FID_AE_SCENE_MODE: 
        OV2710_set_AE_mode(iPara);
        break; 
    default:
        break;
    }
    return TRUE;
}   /* OV2710YUVSensorSetting */

UINT32 OV2710YUVSetVideoMode(UINT16 u2FrameRate)
{
    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.MPEG4_Video_mode = KAL_TRUE;
    spin_unlock(&OV2710_yuv_drv_lock);
    SENSORDB("[Enter]OV2710 Set Video Mode:FrameRate= %d\n",u2FrameRate);
    SENSORDB("OV2710_sensor.video_mode = %d\n",OV2710_sensor.MPEG4_Video_mode);

    if(u2FrameRate == 30) u2FrameRate = 20;
   
    spin_lock(&OV2710_yuv_drv_lock);
    OV2710_sensor.fix_framerate = u2FrameRate * 10;
    spin_unlock(&OV2710_yuv_drv_lock);
    
    if(OV2710_sensor.fix_framerate <= 300 )
    {
        OV2710_Fix_Video_Frame_Rate(OV2710_sensor.fix_framerate); 
    }
    else 
    {
        SENSORDB("Wrong Frame Rate"); 
    }
        
    return TRUE;
}

void OV2710GetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("OV2710GetAFMaxNumFocusAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

void OV2710GetAEMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{     
    *pFeatureReturnPara32 = 0;    
    SENSORDB("OV2710GetAEMaxNumMeteringAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

void OV2710GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = OV2710_sensor.wb;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}

UINT32 OV2710FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    //UINT16 u2Temp = 0; 
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
    case SENSOR_FEATURE_GET_RESOLUTION:
        *pFeatureReturnPara16++=OV2710_IMAGE_SENSOR_FULL_WIDTH;
        *pFeatureReturnPara16=OV2710_IMAGE_SENSOR_FULL_HEIGHT;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PERIOD:
        *pFeatureReturnPara16++=OV2710_IMAGE_SENSOR_PV_WIDTH;//+OV2710_sensor.pv_dummy_pixels;
        *pFeatureReturnPara16=OV2710_IMAGE_SENSOR_PV_HEIGHT;//+OV2710_sensor.pv_dummy_lines;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
        //*pFeatureReturnPara32 = OV2710_sensor_pclk/10;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_ESHUTTER:

        break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
        OV2710_night_mode((BOOL) *pFeatureData16);
        break;
    case SENSOR_FEATURE_SET_GAIN:
        break; 
    case SENSOR_FEATURE_SET_FLASHLIGHT:
        break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
        break;
    case SENSOR_FEATURE_SET_REGISTER:
        OV2710_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
        break;
    case SENSOR_FEATURE_GET_REGISTER:
        pSensorRegData->RegData = OV2710_read_cmos_sensor(pSensorRegData->RegAddr);
        break;
    case SENSOR_FEATURE_GET_CONFIG_PARA:
        memcpy(pSensorConfigData, &OV2710SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
        *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
        break;
    case SENSOR_FEATURE_SET_CCT_REGISTER:
    case SENSOR_FEATURE_GET_CCT_REGISTER:
    case SENSOR_FEATURE_SET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_ENG_REGISTER:
    case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
    case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
    case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
        break;
    case SENSOR_FEATURE_GET_GROUP_COUNT:
        // *pFeatureReturnPara32++=0;
        //*pFeatureParaLen=4;
        break; 

    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
        // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
        // if EEPROM does not exist in camera module.
        *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
        *pFeatureParaLen=4;
        break;
    case SENSOR_FEATURE_SET_YUV_CMD:
        OV2710YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
        break;	
    case SENSOR_FEATURE_SET_VIDEO_MODE:
        OV2710YUVSetVideoMode(*pFeatureData16);
        break; 
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
        OV2710_GetSensorID(pFeatureData32); 
        break; 
    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
        OV2710GetAFMaxNumFocusAreas(pFeatureReturnPara32);            
        *pFeatureParaLen=4;
        break;        
    case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
        OV2710GetAEMaxNumMeteringAreas(pFeatureReturnPara32);            
        *pFeatureParaLen=4;
        break;   
    case SENSOR_FEATURE_GET_EXIF_INFO:
        SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
        SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);          
        OV2710GetExifInfo(*pFeatureData32);
        break;        
    default:
        break;			
    }
    return ERROR_NONE;
}	/* OV2710FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV2710=
{
    OV2710Open,
    OV2710GetInfo,
    OV2710GetResolution,
    OV2710FeatureControl,
    OV2710Control,
    OV2710Close
};

UINT32 OV2710_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV2710;

    return ERROR_NONE;
}	/* SensorInit() */


