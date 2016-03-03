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

#include "ov2675mipiyuv_Sensor.h"
#include "ov2675mipiyuv_Camera_Sensor_para.h"
#include "ov2675mipiyuv_CameraCustomized.h"

#define OV2675MIPIYUV_DEBUG
#ifdef OV2675MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define __SLT_DRV_OV3660_BURST_SHOTS__	//Debug: Brightless became lower and lower in burst shot mode
#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
static kal_uint8 preview_init_flag = 0;
#endif


static DEFINE_SPINLOCK(ov2675_drv_lock);

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define OV2675_MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV2675_MIPI_WRITE_ID)
#define OV2675_MIPI_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,OV2675_MIPI_WRITE_ID)
kal_uint16 OV2675_MIPI_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV2675_MIPI_WRITE_ID);
    return get_byte;
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/


#define	OV2675_MIPI_LIMIT_EXPOSURE_LINES				(1253)
#define	OV2675_MIPI_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	OV2675_MIPI_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	OV2675_MIPI_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 
static kal_uint8 OV2675_MIPI_exposure_line_h = 0, OV2675_MIPI_exposure_line_l = 0,OV2675_MIPI_extra_exposure_line_h = 0, OV2675_MIPI_extra_exposure_line_l = 0;

static kal_bool OV2675_MIPI_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool OV2675_MIPI_sensor_cap_state = KAL_FALSE; //Preview or Capture
static kal_bool OV2675_MIPI_sensor_zsd_mode = KAL_FALSE; //Preview or Capture

static kal_uint16 OV2675_MIPI_dummy_pixels=0, OV2675_MIPI_dummy_lines=0;

static kal_uint16 OV2675_MIPI_exposure_lines=0, OV2675_MIPI_extra_exposure_lines = 0;


static kal_int8 OV2675_MIPI_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 OV2675_MIPI_Capture_Max_Gain16= 6*16;
static kal_uint16 OV2675_MIPI_Capture_Gain16=0 ;    
static kal_uint16 OV2675_MIPI_Capture_Shutter=0;
static kal_uint16 OV2675_MIPI_Capture_Extra_Lines=0;

static kal_uint16  OV2675_MIPI_PV_Dummy_Pixels =0,OV2675_MIPI_Capture_Dummy_Pixels =0, OV2675_MIPI_Capture_Dummy_Lines =0;
static kal_uint16  OV2675_MIPI_PV_Gain16 = 0;
static kal_uint16  OV2675_MIPI_PV_Shutter = 0;
static kal_uint16  OV2675_MIPI_PV_Extra_Lines = 0;
static kal_uint8 ModeChange=0;
static kal_uint8 firstmode=1;


kal_uint16 OV2675_MIPI_iOV2675_MIPI_Mode=0;
kal_uint32 OV2675_MIPI_capture_pclk_in_M=520,OV2675_MIPI_preview_pclk_in_M=390,OV2675_MIPI_PV_dummy_pixels=0,OV2675_MIPI_PV_dummy_lines=0,OV2675_MIPI_isp_master_clock=0;

static kal_uint32  OV2675_MIPI_sensor_pclk=390;
static kal_bool OV2675_MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV2675_MIPI_AE_ENABLE = KAL_TRUE; 

static kal_uint32 Capture_Shutter = 0; 
static kal_uint32 Capture_Gain = 0; 

UINT8 OV2675_MIPI_PixelClockDivider=0;

//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT OV2675SensorConfigData;

void OV2675_MIPI_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
    kal_uint8 temp_reg1, temp_reg2;
    kal_uint16 temp_reg;

    /*Very Important: The line_length must < 0x1000, it is to say 0x3028 must < 0x10, or else the sensor will crash*/
    /*The dummy_pixel must < 2156*/
    if (pixels >= 2156) 
        pixels = 2155;
    if (pixels < 0x100)
    {
        OV2675_MIPI_write_cmos_sensor(0x302c,(pixels&0xFF)); //EXHTS[7:0]
        temp_reg = OV2675_MIPI_FULL_PERIOD_PIXEL_NUMS;
        OV2675_MIPI_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        OV2675_MIPI_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }
    else
    {
        OV2675_MIPI_write_cmos_sensor(0x302c,0);
        temp_reg = pixels + OV2675_MIPI_FULL_PERIOD_PIXEL_NUMS;
        OV2675_MIPI_write_cmos_sensor(0x3029,(temp_reg&0xFF));         //H_length[7:0]
        OV2675_MIPI_write_cmos_sensor(0x3028,((temp_reg&0xFF00)>>8));  //H_length[15:8]
    }

    // read out and + line
    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x302B);    // VTS[b7~b0]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x302A);    // VTS[b15~b8]
    temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

    temp_reg += lines;

    OV2675_MIPI_write_cmos_sensor(0x302B,(temp_reg&0xFF));         //VTS[7:0]
    OV2675_MIPI_write_cmos_sensor(0x302A,((temp_reg&0xFF00)>>8));  //VTS[15:8]
}    /* OV2675_MIPI_set_dummy */

kal_uint16 OV2675_MIPI_read_OV2675_MIPI_gain(void)
{
    kal_uint8  temp_reg;
    kal_uint16 sensor_gain;

    temp_reg=OV2675_MIPI_read_cmos_sensor(0x3000);  


    sensor_gain=(16+(temp_reg&0x0F));
    if(temp_reg&0x10)
        sensor_gain<<=1;
    if(temp_reg&0x20)
        sensor_gain<<=1;
      
    if(temp_reg&0x40)
        sensor_gain<<=1;
      
    if(temp_reg&0x80)
        sensor_gain<<=1;
      
    return sensor_gain;
}  /* OV2675_MIPI_read_OV2675_MIPI_gain */
kal_uint16 OV2675_MIPI_read_shutter(void)
{
    kal_uint8 temp_reg1, temp_reg2;
    kal_uint16 temp_reg, extra_exp_lines;

    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x3003);    // AEC[b7~b0]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x3002);    // AEC[b15~b8]
    temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x302E);    // EXVTS[b7~b0]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x302D);    // EXVTS[b15~b8]
    extra_exp_lines = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_PV_Shutter = temp_reg ;
    OV2675_MIPI_PV_Extra_Lines = extra_exp_lines;
spin_unlock(&ov2675_drv_lock);
    return temp_reg + extra_exp_lines;
}    /* OV2675_MIPI_read_shutter */

void OV2675_MIPI_write_OV2675_MIPI_gain(kal_uint16 gain)
{    
    kal_uint16 temp_reg;
   
	RETAILMSG(1, (TEXT("OV2675 write gain: %d\r\n"), gain));
   
   if(gain > 248)  return ;//ASSERT(0);
   
    temp_reg = 0;
    if (gain > 31)
    {
        temp_reg |= 0x10;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x20;
        gain = gain >> 1;
    }

    if (gain > 31)
    {
        temp_reg |= 0x40;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x80;
        gain = gain >> 1;
    }
    
    if (gain > 16)
    {
        temp_reg |= ((gain -16) & 0x0f);
    }   
  
   OV2675_MIPI_write_cmos_sensor(0x3000,temp_reg);
}  /* OV2675_MIPI_write_OV2675_MIPI_gain */

static void OV2675_MIPI_write_shutter(kal_uint16 shutter)
{
    if (OV2675_MIPI_gPVmode) 
    {
        if (shutter <= OV2675_MIPI_PV_EXPOSURE_LIMITATION) 
        {
	spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_extra_exposure_lines = 0;
	spin_unlock(&ov2675_drv_lock);
        }
        else 
        {
	spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_extra_exposure_lines=shutter - OV2675_MIPI_PV_EXPOSURE_LIMITATION;
	spin_unlock(&ov2675_drv_lock);
        }

        if (shutter > OV2675_MIPI_PV_EXPOSURE_LIMITATION) 
        {
            shutter = OV2675_MIPI_PV_EXPOSURE_LIMITATION;
        }
    }
    else 
    {
        if (shutter <= OV2675_MIPI_FULL_EXPOSURE_LIMITATION) 
        {
	spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_extra_exposure_lines = 0;
	spin_unlock(&ov2675_drv_lock);
    }
        else 
        {
	spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_extra_exposure_lines = shutter - OV2675_MIPI_FULL_EXPOSURE_LIMITATION;
	spin_unlock(&ov2675_drv_lock);
        }

        if (shutter > OV2675_MIPI_FULL_EXPOSURE_LIMITATION) {
            shutter = OV2675_MIPI_FULL_EXPOSURE_LIMITATION;
        }
    }

    // set extra exposure line
    OV2675_MIPI_write_cmos_sensor(0x302E, OV2675_MIPI_extra_exposure_lines & 0xFF);          // EXVTS[b7~b0]
    OV2675_MIPI_write_cmos_sensor(0x302D, (OV2675_MIPI_extra_exposure_lines & 0xFF00) >> 8); // EXVTS[b15~b8]

    /* Max exporsure time is 1 frmae period event if Tex is set longer than 1 frame period */
    OV2675_MIPI_write_cmos_sensor(0x3003, shutter & 0xFF);           //AEC[7:0]
    OV2675_MIPI_write_cmos_sensor(0x3002, (shutter & 0xFF00) >> 8);  //AEC[8:15]

}    /* OV2675_MIPI_write_shutter */


void OV2675_MIPI_Computer_AECAGC(kal_uint16 preview_clk_in_M, kal_uint16 capture_clk_in_M)
{
    kal_uint16 PV_Line_Width;
    kal_uint16 Capture_Line_Width;
    kal_uint16 Capture_Maximum_Shutter;
    kal_uint16 Capture_Exposure;
    kal_uint16 Capture_Gain16;
    kal_uint32 Capture_Banding_Filter;
    kal_uint32 Gain_Exposure=0;

    PV_Line_Width = OV2675_MIPI_PV_PERIOD_PIXEL_NUMS + OV2675_MIPI_PV_Dummy_Pixels;   

    Capture_Line_Width = OV2675_MIPI_FULL_PERIOD_PIXEL_NUMS + OV2675_MIPI_Capture_Dummy_Pixels;
    Capture_Maximum_Shutter = OV2675_MIPI_FULL_EXPOSURE_LIMITATION + OV2675_MIPI_Capture_Dummy_Lines;

    if (OV2675_MIPI_Banding_setting == AE_FLICKER_MODE_50HZ)

        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/100/(2*Capture_Line_Width));
    else
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/120/(2*Capture_Line_Width) );

	spin_lock(&ov2675_drv_lock);
    //OV2675_MIPI_PV_Gain16 = OV2675_MIPI_read_OV2675_MIPI_gain();
	spin_unlock(&ov2675_drv_lock);
    Gain_Exposure = 1 * OV2675_MIPI_PV_Gain16;  //For OV2675
    ///////////////////////
    Gain_Exposure *=(OV2675_MIPI_PV_Shutter+OV2675_MIPI_PV_Extra_Lines);
    Gain_Exposure *=PV_Line_Width;  //970
    //   Gain_Exposure /=g_Preview_PCLK_Frequency;
    Gain_Exposure /=Capture_Line_Width;//1940
    Gain_Exposure = Gain_Exposure*capture_clk_in_M/preview_clk_in_M;// for clock   

    //redistribute gain and exposure
    if (Gain_Exposure < (kal_uint32)(Capture_Banding_Filter * 16))     // Exposure < 1/100/120
    {
       if(Gain_Exposure<16){//exposure line smaller than 2 lines and gain smaller than 0x08 
            Gain_Exposure = Gain_Exposure*4;     
            Capture_Exposure = 1;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2/4;
        }
        else
        {
            Capture_Exposure = Gain_Exposure /16;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
        }
    }
    else 
    {
        if (Gain_Exposure >(kal_uint32)( Capture_Maximum_Shutter * 16)) // Exposure > Capture_Maximum_Shutter
        {
           
            Capture_Exposure = Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
            if (Capture_Gain16 > OV2675_MIPI_Capture_Max_Gain16) 
            {
                // gain reach maximum, insert extra line
                Capture_Exposure = (kal_uint16)(Gain_Exposure*11 /10 /OV2675_MIPI_Capture_Max_Gain16);
                
                // Exposure = n/100/120
                Capture_Exposure = Capture_Exposure/Capture_Banding_Filter * Capture_Banding_Filter;
                Capture_Gain16 = ((Gain_Exposure *4)/ Capture_Exposure+3)/4;
            }
        }
        else  // 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100/120
        {
            Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;
            Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;
        }
    }

    	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_Capture_Gain16 = Capture_Gain16;
    OV2675_MIPI_Capture_Extra_Lines = (Capture_Exposure > Capture_Maximum_Shutter)?
            (Capture_Exposure - Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter):0;     
    

    OV2675_MIPI_Capture_Shutter = Capture_Exposure - OV2675_MIPI_Capture_Extra_Lines;
	spin_unlock(&ov2675_drv_lock);
}

void OV2675_MIPI_set_isp_driving_current(kal_uint8 current)
{
}


static void OV2675_MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
		SENSORDB("[OV2675]OV2675_MIPI_set_AE_mode mode  KAL_TRUE\n");
        temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
        OV2675_MIPI_write_cmos_sensor(0x3013, temp_AE_reg| 0x05);
    }
    else
    {
        // turn off AEC/AGC
		SENSORDB("[OV2675]OV2675_MIPI_set_AE_mode mode  KAL_FALSE\n");
        temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
        OV2675_MIPI_write_cmos_sensor(0x3013, temp_AE_reg&~0x05);
    }
}


static void OV2675_MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 temp_AWB_reg = 0;

    //return ;

    if (AWB_enable == KAL_TRUE)
    {
        //enable Auto WB
        temp_AWB_reg = OV2675_MIPI_read_cmos_sensor(0x3324);
        OV2675_MIPI_write_cmos_sensor(0x3324, temp_AWB_reg & ~0x40);        
    }
    else
    {
        //turn off AWB
        temp_AWB_reg = OV2675_MIPI_read_cmos_sensor(0x3324);
        OV2675_MIPI_write_cmos_sensor(0x3324, temp_AWB_reg | 0x40);        
    }
}


BOOL OV2675_MIPI_set_param_banding(UINT16 para)
{
    kal_uint8 banding;
    banding = OV2675_MIPI_read_cmos_sensor(0x3014);
    
    switch (para)
    {
    case AE_FLICKER_MODE_50HZ:
		                spin_lock(&ov2675_drv_lock);		
		                OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;		
		                spin_unlock(&ov2675_drv_lock);
		
		                OV2675_MIPI_write_cmos_sensor(0x3014,banding|0x80);    /* enable banding and 50 Hz */				
                    break;    
    case AE_FLICKER_MODE_60HZ:
		                spin_lock(&ov2675_drv_lock);
		                OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;	
		                spin_unlock(&ov2675_drv_lock);	
	 
                    OV2675_MIPI_write_cmos_sensor(0x3014,(banding&~0x80));    /* enable banding and 60 Hz */            
                    break;
            default:
                    return FALSE;
    }
    return TRUE;
} /* OV2675_MIPI_set_param_banding */

/*************************************************************************
* FUNCTION
*	OV2675_MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of OV2675.
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
void OV2675_MIPI_night_mode(kal_bool enable)
{
	
	kal_uint8 night = OV2675_MIPI_read_cmos_sensor(0x3014); //bit[3], 0: disable, 1:enable
	//	kal_uint8 temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
		 		 
		// OV2675_MIPI_write_cmos_sensor(0x3013, (temp_AE_reg&(~0x05)));		   
		/* ==Video Preview, Auto Mode, use 39MHz PCLK, 30fps; Night Mode use 39M, 15fps */
		if (OV2675_MIPI_sensor_cap_state == KAL_FALSE) 
		{
			if (enable) 
			{
			  //  OV2675_MIPI_write_cmos_sensor(0x302D, OV2675_MIPI_extra_exposure_line_h);
			  //  OV2675_MIPI_write_cmos_sensor(0x302E, OV2675_MIPI_extra_exposure_line_l);	
		if (OV2675_MIPI_VEDIO_encode_mode == KAL_TRUE) 
		{
					// set Max gain to 16X
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);//15fps
            OV2675_MIPI_write_cmos_sensor(0x3011, 0x01);
            OV2675_MIPI_write_cmos_sensor(0x300e, 0x34);
            OV2675_MIPI_write_cmos_sensor(0x302a, 0x02);  
            OV2675_MIPI_write_cmos_sensor(0x302b, 0x9e);
                    
            OV2675_MIPI_write_cmos_sensor(0x3070, 0x64);
            OV2675_MIPI_write_cmos_sensor(0x3072, 0x54);
            OV2675_MIPI_write_cmos_sensor(0x301c, 0x05);
            OV2675_MIPI_write_cmos_sensor(0x301d, 0x07);
            OV2675_MIPI_write_cmos_sensor(0x3015, 0x02);
            OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
            OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);   
		}
		else 
		{
					/* Camera mode only */	
                   
            OV2675_MIPI_write_cmos_sensor(0x3015, 0x02); 
            OV2675_MIPI_write_cmos_sensor(0x3014, night&0xf7); //Disable night mode
            OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
			OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
			OV2675_MIPI_write_cmos_sensor(0x3015, 0x42);  			
			OV2675_MIPI_write_cmos_sensor(0x3014, night|0x08);					  
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//25fps
            OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
		}	   
			}
			else 
		{
				/* when enter normal mode (disable night mode) without light, the AE vibrate */
		if (OV2675_MIPI_VEDIO_encode_mode == KAL_TRUE) 
		{
					/* MJPEG or MPEG4 Apps */
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//25fps
            OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
            OV2675_MIPI_write_cmos_sensor(0x300e, 0x34);
            OV2675_MIPI_write_cmos_sensor(0x302a, 0x02);  
            OV2675_MIPI_write_cmos_sensor(0x302b, 0x9e);
            OV2675_MIPI_write_cmos_sensor(0x3070, 0xc8);
            OV2675_MIPI_write_cmos_sensor(0x3072, 0xa9);
            OV2675_MIPI_write_cmos_sensor(0x301c, 0x02);
            OV2675_MIPI_write_cmos_sensor(0x301d, 0x03);
            OV2675_MIPI_write_cmos_sensor(0x3015, 0x02);
            OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
            OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
		}
		else 
		{
					/* Camera mode only */
					// set Max gain to 4X
			SENSORDB("[OV2675]HI257night mode\n");
            OV2675_MIPI_write_cmos_sensor(0x3015, 0x02); 
			OV2675_MIPI_write_cmos_sensor(0x3014, night&0xf7); //Disable night mode                                       
            OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
			OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
			OV2675_MIPI_write_cmos_sensor(0x3015, 0x22);  			
			OV2675_MIPI_write_cmos_sensor(0x3014, night|0x08);					  
			 OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//25fps
            OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
				}					
		}
	}
        OV2675_MIPI_set_param_banding(OV2675_MIPI_Banding_setting); 
}	/* OV2675_MIPI_night_mode */
void OV2675_MIPI_night_ZSD_mode(kal_bool enable)

{
	
	kal_uint8 night = OV2675_MIPI_read_cmos_sensor(0x3014); //bit[3], 0: disable, 1:enable
	//	kal_uint8 temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
	SENSORDB("[]OV2675_MIPI_night_ZSD_mode\n");
		 		 
		// OV2675_MIPI_write_cmos_sensor(0x3013, (temp_AE_reg&(~0x05)));		   
		/* ==Video Preview, Auto Mode, use 39MHz PCLK, 30fps; Night Mode use 39M, 15fps */
		if (OV2675_MIPI_sensor_cap_state == KAL_FALSE) 
		{
			if (enable) 
			{
			  SENSORDB("[]OV2675_MIPI_night_ZSD_mode disable enable\n");
					/* Camera mode only */	
           		OV2675_MIPI_write_cmos_sensor(0x3015, 0x02); 
            	OV2675_MIPI_write_cmos_sensor(0x3014, night&0xf7); //Disable night mode
            	OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x3015, 0x42);  			
				OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//25fps
				OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x3014, night|0x08);					  
			}
			else 
			{
				/* when enter normal mode (disable night mode) without light, the AE vibrate */
				SENSORDB("[]OV2675_MIPI_night_ZSD_mode disable\n");
				OV2675_MIPI_write_cmos_sensor(0x3015, 0x02); 
				OV2675_MIPI_write_cmos_sensor(0x3014, night&0xf7); //Disable night mode									   
				OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x3015, 0x22);				
				OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//25fps
				OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
				OV2675_MIPI_write_cmos_sensor(0x3014, night|0x08);					  
						//  OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);//15fps
				//OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
						
		}}
        OV2675_MIPI_set_param_banding(OV2675_MIPI_Banding_setting); 
}


/*************************************************************************
* FUNCTION
*	OV2675_MIPI_GetSensorID
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
static kal_uint32 OV2675_MIPI_GetSensorID(kal_uint32 *sensorID)

{
	volatile signed char i;
		kal_uint32 sensor_id=0;
		kal_uint8 temp_sccb_addr = 0;
		//s_move to here from CISModulePowerOn()

		OV2675_MIPI_write_cmos_sensor(0x3012,0x80);// Reset sensor
			mDELAY(10);
		
		
			//	Read sensor ID to adjust I2C is OK?
			for(i=0;i<3;i++)
			{
				sensor_id = (OV2675_MIPI_read_cmos_sensor(0x300A) << 8) | OV2675_MIPI_read_cmos_sensor(0x300B);
				printk("++++OV2675_MIPI_GetSensorID,read id = 0x%x\n", sensor_id);
				if(sensor_id != OV2675_SENSOR_ID)
				{
					*sensorID =0xFFFFFFFF;
					return ERROR_SENSOR_CONNECT_FAIL;
				}
			}
			    *sensorID = sensor_id;

			RETAILMSG(1, (TEXT("OV2675 Sensor Read ID OK \r\n")));
		
    return ERROR_NONE;    
}   
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	OV2675Open
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
UINT32 OV2675Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	zoom_factor = 0; 
	OV2675_MIPI_write_cmos_sensor(0x3012,0x80);// Reset sensor
    Sleep(10);


	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (OV2675_MIPI_read_cmos_sensor(0x300A) << 8) | OV2675_MIPI_read_cmos_sensor(0x300B);
		printk("++++OV2675Open,read id = 0x%x\n", sensor_id);
		
		if(sensor_id != OV2675_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	RETAILMSG(1, (TEXT("OV2675 Sensor Read ID OK \r\n")));
	
//init MIPI
		OV2675_MIPI_write_cmos_sensor(0x3640,0x06);

		OV2675_MIPI_write_cmos_sensor(0x308c,0x80);
		OV2675_MIPI_write_cmos_sensor(0x308d,0x0e);
		OV2675_MIPI_write_cmos_sensor(0x360b,0x00);
		OV2675_MIPI_write_cmos_sensor(0x30b0,0xff);
		OV2675_MIPI_write_cmos_sensor(0x30b1,0xff);
		OV2675_MIPI_write_cmos_sensor(0x30b2,0x24);

		OV2675_MIPI_write_cmos_sensor(0x3085,0x20);

		OV2675_MIPI_write_cmos_sensor(0x300e,0x34);
		OV2675_MIPI_write_cmos_sensor(0x300f,0xa6);
		OV2675_MIPI_write_cmos_sensor(0x3010,0x80);
		OV2675_MIPI_write_cmos_sensor(0x3082,0x01);
		OV2675_MIPI_write_cmos_sensor(0x30f4,0x01);
		OV2675_MIPI_write_cmos_sensor(0x3090,0x33);
		OV2675_MIPI_write_cmos_sensor(0x3091,0xc0);
		OV2675_MIPI_write_cmos_sensor(0x30ac,0x42);

		OV2675_MIPI_write_cmos_sensor(0x30d1,0x08);
		OV2675_MIPI_write_cmos_sensor(0x30a8,0x54);
		OV2675_MIPI_write_cmos_sensor(0x3015,0x02);
		OV2675_MIPI_write_cmos_sensor(0x3093,0x00);
		OV2675_MIPI_write_cmos_sensor(0x307e,0xe5);
		OV2675_MIPI_write_cmos_sensor(0x3079,0x00);
		OV2675_MIPI_write_cmos_sensor(0x30aa,0x82);
		OV2675_MIPI_write_cmos_sensor(0x3017,0x40);
		OV2675_MIPI_write_cmos_sensor(0x30f3,0x83);
		OV2675_MIPI_write_cmos_sensor(0x306a,0x0c);
		OV2675_MIPI_write_cmos_sensor(0x306d,0x00);
		OV2675_MIPI_write_cmos_sensor(0x336a,0x3c);
		OV2675_MIPI_write_cmos_sensor(0x3076,0x6a);
		OV2675_MIPI_write_cmos_sensor(0x30d9,0x95);
		OV2675_MIPI_write_cmos_sensor(0x3016,0x52);
		OV2675_MIPI_write_cmos_sensor(0x3601,0x30);
		OV2675_MIPI_write_cmos_sensor(0x304e,0x88);
		OV2675_MIPI_write_cmos_sensor(0x30f1,0x82);
		OV2675_MIPI_write_cmos_sensor(0x306f,0x14);

		OV2675_MIPI_write_cmos_sensor(0x3012,0x10);
		OV2675_MIPI_write_cmos_sensor(0x3011,0x00);//15fps
		OV2675_MIPI_write_cmos_sensor(0x302a,0x03);//0x02
		OV2675_MIPI_write_cmos_sensor(0x302b,0x24);//0x84

		OV2675_MIPI_write_cmos_sensor(0x3391,0x06);
		OV2675_MIPI_write_cmos_sensor(0x3394,0x50);
		OV2675_MIPI_write_cmos_sensor(0x3395,0x50);

		OV2675_MIPI_write_cmos_sensor(0x3015,0x02);
		OV2675_MIPI_write_cmos_sensor(0x302d,0x00);
		OV2675_MIPI_write_cmos_sensor(0x302e,0x00);

		  //;AEC/AGC
		OV2675_MIPI_write_cmos_sensor(0x3013,0xf7);
		OV2675_MIPI_write_cmos_sensor(0x3018,0x80);
		OV2675_MIPI_write_cmos_sensor(0x3019,0x70);
		OV2675_MIPI_write_cmos_sensor(0x301a,0xc4);
		  //;D5060
		OV2675_MIPI_write_cmos_sensor(0x30af,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3048,0x1f);
		OV2675_MIPI_write_cmos_sensor(0x3049,0x4e);
		OV2675_MIPI_write_cmos_sensor(0x304a,0x40);
		OV2675_MIPI_write_cmos_sensor(0x304f,0x40);
		OV2675_MIPI_write_cmos_sensor(0x304b,0x02);
		OV2675_MIPI_write_cmos_sensor(0x304c,0x00); 		
		OV2675_MIPI_write_cmos_sensor(0x304d,0x42);
		OV2675_MIPI_write_cmos_sensor(0x304f,0x40);
		OV2675_MIPI_write_cmos_sensor(0x30a3,0x91);
		OV2675_MIPI_write_cmos_sensor(0x3013,0xf7);
		OV2675_MIPI_write_cmos_sensor(0x3014,0x84);
		OV2675_MIPI_write_cmos_sensor(0x3071,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3070,0xb9);
		OV2675_MIPI_write_cmos_sensor(0x3073,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3072,0x9a);
		OV2675_MIPI_write_cmos_sensor(0x301c,0x02);
		OV2675_MIPI_write_cmos_sensor(0x301d,0x03);
		OV2675_MIPI_write_cmos_sensor(0x304d,0x42);
		OV2675_MIPI_write_cmos_sensor(0x304a,0x40);
		OV2675_MIPI_write_cmos_sensor(0x304f,0x40);
		OV2675_MIPI_write_cmos_sensor(0x3095,0x07);
		OV2675_MIPI_write_cmos_sensor(0x3096,0x16);
		OV2675_MIPI_write_cmos_sensor(0x3097,0x1d);
		  //;Window Setup
		OV2675_MIPI_write_cmos_sensor(0x3020,0x01);
		OV2675_MIPI_write_cmos_sensor(0x3021,0x1a);
		OV2675_MIPI_write_cmos_sensor(0x3022,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3023,0x06);
		OV2675_MIPI_write_cmos_sensor(0x3024,0x06);
		OV2675_MIPI_write_cmos_sensor(0x3025,0x58);
		OV2675_MIPI_write_cmos_sensor(0x3026,0x02);
		OV2675_MIPI_write_cmos_sensor(0x3027,0x61);
		OV2675_MIPI_write_cmos_sensor(0x3088,0x02);
		OV2675_MIPI_write_cmos_sensor(0x3089,0x80);
		OV2675_MIPI_write_cmos_sensor(0x308a,0x01);
		OV2675_MIPI_write_cmos_sensor(0x308b,0xe0);
		OV2675_MIPI_write_cmos_sensor(0x3316,0x64);
		OV2675_MIPI_write_cmos_sensor(0x3317,0x25);
		OV2675_MIPI_write_cmos_sensor(0x3318,0x80);
		OV2675_MIPI_write_cmos_sensor(0x3319,0x08);
		OV2675_MIPI_write_cmos_sensor(0x331a,0x28);
		OV2675_MIPI_write_cmos_sensor(0x331b,0x1e);
		OV2675_MIPI_write_cmos_sensor(0x331c,0x00);
		OV2675_MIPI_write_cmos_sensor(0x331d,0x38);
		OV2675_MIPI_write_cmos_sensor(0x3100,0x00);  
		  //awb
		OV2675_MIPI_write_cmos_sensor(0x3320,0xfa);
		OV2675_MIPI_write_cmos_sensor(0x3321,0x11);
		OV2675_MIPI_write_cmos_sensor(0x3322,0x92);
		OV2675_MIPI_write_cmos_sensor(0x3323,0x01);
		OV2675_MIPI_write_cmos_sensor(0x3324,0x97);
		OV2675_MIPI_write_cmos_sensor(0x3325,0x02);
		OV2675_MIPI_write_cmos_sensor(0x3326,0xff);
		OV2675_MIPI_write_cmos_sensor(0x3327,0x10);
		OV2675_MIPI_write_cmos_sensor(0x3328,0x10);
		OV2675_MIPI_write_cmos_sensor(0x3329,0x1f);
		OV2675_MIPI_write_cmos_sensor(0x332a,0x56);
		OV2675_MIPI_write_cmos_sensor(0x332b,0x54);
		OV2675_MIPI_write_cmos_sensor(0x332c,0xbe);
		OV2675_MIPI_write_cmos_sensor(0x332d,0xce);
		OV2675_MIPI_write_cmos_sensor(0x332e,0x2e);
		OV2675_MIPI_write_cmos_sensor(0x332f,0x30);
		OV2675_MIPI_write_cmos_sensor(0x3330,0x4d);
		OV2675_MIPI_write_cmos_sensor(0x3331,0x44);
		OV2675_MIPI_write_cmos_sensor(0x3332,0xf0);
		OV2675_MIPI_write_cmos_sensor(0x3333,0x0a);
		OV2675_MIPI_write_cmos_sensor(0x3334,0xf0);
		OV2675_MIPI_write_cmos_sensor(0x3335,0xf0);
		OV2675_MIPI_write_cmos_sensor(0x3336,0xf0);
		OV2675_MIPI_write_cmos_sensor(0x3337,0x40);
		OV2675_MIPI_write_cmos_sensor(0x3338,0x40);
		OV2675_MIPI_write_cmos_sensor(0x3339,0x40);
		OV2675_MIPI_write_cmos_sensor(0x333a,0x00);
		OV2675_MIPI_write_cmos_sensor(0x333b,0x00);   
		  //cmx
		OV2675_MIPI_write_cmos_sensor(0x3380,0x28);
		OV2675_MIPI_write_cmos_sensor(0x3381,0x48);
		OV2675_MIPI_write_cmos_sensor(0x3382,0x12);
		OV2675_MIPI_write_cmos_sensor(0x3383,0x15);
		OV2675_MIPI_write_cmos_sensor(0x3384,0x9e);
		OV2675_MIPI_write_cmos_sensor(0x3385,0xb3);
		OV2675_MIPI_write_cmos_sensor(0x3386,0xb3);
		OV2675_MIPI_write_cmos_sensor(0x3387,0xa7);
		OV2675_MIPI_write_cmos_sensor(0x3388,0x0c);
		OV2675_MIPI_write_cmos_sensor(0x3389,0x98);
		OV2675_MIPI_write_cmos_sensor(0x338a,0x01);  
		  //gamma 
		OV2675_MIPI_write_cmos_sensor(0x3340,0x06);
		OV2675_MIPI_write_cmos_sensor(0x3341,0x0c);
		OV2675_MIPI_write_cmos_sensor(0x3342,0x1c);
		OV2675_MIPI_write_cmos_sensor(0x3343,0x36);
		OV2675_MIPI_write_cmos_sensor(0x3344,0x4e);
		OV2675_MIPI_write_cmos_sensor(0x3345,0x5f);
		OV2675_MIPI_write_cmos_sensor(0x3346,0x6d);
		OV2675_MIPI_write_cmos_sensor(0x3347,0x78);
		OV2675_MIPI_write_cmos_sensor(0x3348,0x84);
		OV2675_MIPI_write_cmos_sensor(0x3349,0x95); 
		OV2675_MIPI_write_cmos_sensor(0x334a,0xa5);  
		OV2675_MIPI_write_cmos_sensor(0x334b,0xb4);  
		OV2675_MIPI_write_cmos_sensor(0x334c,0xc8);  
		OV2675_MIPI_write_cmos_sensor(0x334d,0xde); 
		OV2675_MIPI_write_cmos_sensor(0x334e,0xf0);  
		OV2675_MIPI_write_cmos_sensor(0x334f,0x15);

		OV2675_MIPI_write_cmos_sensor(0x3090,0x03);  
		OV2675_MIPI_write_cmos_sensor(0x307c,0x10);

		OV2675_MIPI_write_cmos_sensor(0x3350,0x32);
		OV2675_MIPI_write_cmos_sensor(0x3351,0x26);
		OV2675_MIPI_write_cmos_sensor(0x3352,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3353,0x2f);
		OV2675_MIPI_write_cmos_sensor(0x3354,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3355,0x85); 
		OV2675_MIPI_write_cmos_sensor(0x3356,0x33);
		OV2675_MIPI_write_cmos_sensor(0x3357,0x26);
		OV2675_MIPI_write_cmos_sensor(0x3358,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3359,0x2c);
		OV2675_MIPI_write_cmos_sensor(0x335a,0x00);
		OV2675_MIPI_write_cmos_sensor(0x335b,0x85); 
		OV2675_MIPI_write_cmos_sensor(0x335c,0x32);
		OV2675_MIPI_write_cmos_sensor(0x335d,0x26);
		OV2675_MIPI_write_cmos_sensor(0x335e,0x00);
		OV2675_MIPI_write_cmos_sensor(0x335f,0x2a);
		OV2675_MIPI_write_cmos_sensor(0x3360,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3361,0x85); 
		OV2675_MIPI_write_cmos_sensor(0x3363,0x70);
		OV2675_MIPI_write_cmos_sensor(0x3364,0x7f);
		OV2675_MIPI_write_cmos_sensor(0x3365,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3366,0x00);

		OV2675_MIPI_write_cmos_sensor(0x3362,0x90);
		  //uv adjust
		OV2675_MIPI_write_cmos_sensor(0x3301,0xff);
		OV2675_MIPI_write_cmos_sensor(0x338b,0x13);
		OV2675_MIPI_write_cmos_sensor(0x338c,0x10);
		OV2675_MIPI_write_cmos_sensor(0x338d,0x40);  
		  //Sharpness/De-noise
		OV2675_MIPI_write_cmos_sensor(0x3370,0xd0);
		OV2675_MIPI_write_cmos_sensor(0x3371,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3372,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3373,0x50);
		OV2675_MIPI_write_cmos_sensor(0x3374,0x10);
		OV2675_MIPI_write_cmos_sensor(0x3375,0x10);
		OV2675_MIPI_write_cmos_sensor(0x3376,0x05);
		OV2675_MIPI_write_cmos_sensor(0x3377,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3378,0x04);
		OV2675_MIPI_write_cmos_sensor(0x3379,0x80); 
		 
		OV2675_MIPI_write_cmos_sensor(0x3069,0x86);
		OV2675_MIPI_write_cmos_sensor(0x3087,0x02); 

		  //;Other functions
		OV2675_MIPI_write_cmos_sensor(0x3300,0xfc);
		OV2675_MIPI_write_cmos_sensor(0x3302,0x11);
		OV2675_MIPI_write_cmos_sensor(0x3400,0x02);	 
		OV2675_MIPI_write_cmos_sensor(0x3606,0x20);
		OV2675_MIPI_write_cmos_sensor(0x3601,0x30);
		OV2675_MIPI_write_cmos_sensor(0x30f3,0x83);
		OV2675_MIPI_write_cmos_sensor(0x304e,0x88);	

		OV2675_MIPI_write_cmos_sensor(0x30a8,0x54);
		OV2675_MIPI_write_cmos_sensor(0x30aa,0x82);
		OV2675_MIPI_write_cmos_sensor(0x30a3,0x91);
		OV2675_MIPI_write_cmos_sensor(0x30a1,0x41);
		  //mipi
		OV2675_MIPI_write_cmos_sensor(0x363b,0x01);
		OV2675_MIPI_write_cmos_sensor(0x309e,0x08);
		OV2675_MIPI_write_cmos_sensor(0x3606,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3630,0x35);
		OV2675_MIPI_write_cmos_sensor(0x3086,0x0f);
		OV2675_MIPI_write_cmos_sensor(0x3086,0x00);

		OV2675_MIPI_write_cmos_sensor(0x3010,0x80);
		OV2675_MIPI_write_cmos_sensor(0x300e,0x34);
		OV2675_MIPI_write_cmos_sensor(0x3011,0x00);

		OV2675_MIPI_write_cmos_sensor(0x304e,0x04);
		OV2675_MIPI_write_cmos_sensor(0x363b,0x01);
		OV2675_MIPI_write_cmos_sensor(0x309e,0x08);
		OV2675_MIPI_write_cmos_sensor(0x3606,0x00);
		OV2675_MIPI_write_cmos_sensor(0x3084,0x01);
		OV2675_MIPI_write_cmos_sensor(0x3634,0x26);
		OV2675_MIPI_write_cmos_sensor(0x3086,0x0f);
		OV2675_MIPI_write_cmos_sensor(0x3086,0x00);

		return ERROR_NONE;
}	/* OV2675Open() */

/*************************************************************************
* FUNCTION
*	OV2675Close
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
UINT32 OV2675Close(void)
{
//	CISModulePowerOn(FALSE);

	return ERROR_NONE;
}	/* OV2675Close() */

/*************************************************************************
* FUNCTION
*	OV2675Preview
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
UINT32 OV2675Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
  kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;
   
	spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_sensor_cap_state = KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);  


#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
	preview_init_flag = 1;
#endif
	OV2675_MIPI_sensor_zsd_mode=KAL_FALSE;

//	Sleep(50); //caosq add

    printk("++++%s\n", __FUNCTION__);	//
    
    OV2675_MIPI_set_AE_mode(KAL_FALSE);

    //4  <1> preview config sequence
    OV2675_MIPI_write_cmos_sensor(0x3002, OV2675_MIPI_exposure_line_h * 1); //090623
    OV2675_MIPI_write_cmos_sensor(0x3003, OV2675_MIPI_exposure_line_l * 1);  //090623
    OV2675_MIPI_write_cmos_sensor(0x302D, OV2675_MIPI_extra_exposure_line_h * 1); //090623
    OV2675_MIPI_write_cmos_sensor(0x302E, OV2675_MIPI_extra_exposure_line_l * 1);  //090623

    OV2675_MIPI_write_OV2675_MIPI_gain(OV2675_MIPI_PV_Gain16);
    Sleep(100);
    
	  spin_lock(&ov2675_drv_lock);
	       OV2675_MIPI_sensor_pclk=390;
	  spin_unlock(&ov2675_drv_lock);
   
    //YUV SVGA (800x600)
  
    if(OV2675_MIPI_VEDIO_encode_mode == KAL_FALSE)
    {
    OV2675_MIPI_write_cmos_sensor(0x3010, 0x80); 
    OV2675_MIPI_write_cmos_sensor(0x300e, 0x34); 
    OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
    OV2675_MIPI_write_cmos_sensor(0x302a, 0x03);
    OV2675_MIPI_write_cmos_sensor(0x302b, 0x23); 
    
    OV2675_MIPI_write_cmos_sensor(0x3070, 0xc8);
    OV2675_MIPI_write_cmos_sensor(0x3072, 0xa9);
    OV2675_MIPI_write_cmos_sensor(0x301c, 0x03);
    OV2675_MIPI_write_cmos_sensor(0x301d, 0x03);
    }
    OV2675_MIPI_write_cmos_sensor(0x3012, 0x10); 
    OV2675_MIPI_write_cmos_sensor(0x306f, 0x14);
    OV2675_MIPI_write_cmos_sensor(0x3362, 0x90);
   
    OV2675_MIPI_write_cmos_sensor(0x3020, 0x01);  
    OV2675_MIPI_write_cmos_sensor(0x3021, 0x1a); 
    OV2675_MIPI_write_cmos_sensor(0x3022, 0x00); 
    OV2675_MIPI_write_cmos_sensor(0x3023, 0x06);
    OV2675_MIPI_write_cmos_sensor(0x3024, 0x06); 
    OV2675_MIPI_write_cmos_sensor(0x3025, 0x58); 
    OV2675_MIPI_write_cmos_sensor(0x3026, 0x02);
    OV2675_MIPI_write_cmos_sensor(0x3027, 0x5e); 
    OV2675_MIPI_write_cmos_sensor(0x3088, 0x03);
    OV2675_MIPI_write_cmos_sensor(0x3089, 0x20); 
    OV2675_MIPI_write_cmos_sensor(0x308a, 0x02);
    OV2675_MIPI_write_cmos_sensor(0x308b, 0x58); 
    OV2675_MIPI_write_cmos_sensor(0x3316, 0x64); 
    OV2675_MIPI_write_cmos_sensor(0x3317, 0x25); 
    OV2675_MIPI_write_cmos_sensor(0x3318, 0x80); 
    OV2675_MIPI_write_cmos_sensor(0x3319, 0x08);
    OV2675_MIPI_write_cmos_sensor(0x331a, 0x64);
    OV2675_MIPI_write_cmos_sensor(0x331b, 0x4b);       
    OV2675_MIPI_write_cmos_sensor(0x330c, 0x00);    
    OV2675_MIPI_write_cmos_sensor(0x331d, 0x38);
    OV2675_MIPI_write_cmos_sensor(0x3302, 0x11);    

    OV2675_MIPI_write_cmos_sensor(0x3373, 0x40);
    OV2675_MIPI_write_cmos_sensor(0x3376, 0x05);   
    
    Sleep(50);   

    //===preview setting end===
    /* ==Camera Preview, MT6235 use 36MHz PCLK, 30fps 60Hz, 25fps in 50Hz== */
    /* after set exposure line, there should be delay for 2~4 frame time, then enable AEC */
    //Use preview_ae_stable_frame to drop frame   

    // turn on AEC/AGC
    // OV2675_MIPI_set_AE_mode(KAL_TRUE); 
    temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
    OV2675_MIPI_write_cmos_sensor(0x3013, temp_AE_reg|0x05);

    //enable Auto WB
    //OV2675_MIPI_set_AWB_mode(KAL_TRUE); 
    //temp_AWB_reg = OV2675_MIPI_read_cmos_sensor(0x3324);
    //OV2675_MIPI_write_cmos_sensor(0x3324, temp_AWB_reg&~0x40);
   
	  spin_lock(&ov2675_drv_lock);
       OV2675_MIPI_gPVmode = KAL_TRUE;
	  spin_unlock(&ov2675_drv_lock);

    if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
        RETAILMSG(1,(TEXT("Camera Video preview\r\n")));
	spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_VEDIO_encode_mode = KAL_TRUE;
	spin_unlock(&ov2675_drv_lock);

        iDummyPixels = 0;
        iDummyLines = 0;

        /* to fix VSYNC, to fix frame rate */
        iTemp = OV2675_MIPI_read_cmos_sensor(0x3014);
        OV2675_MIPI_write_cmos_sensor(0x3014, iTemp & 0xf7); //Disable night mode
        OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
        OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
        if (image_window->ImageTargetWidth <= OV2675_MIPI_VIDEO_QCIF_WIDTH)
        {
		spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_iOV2675_MIPI_Mode = OV2675_MIPI_MODE_QCIF_VIDEO;
		spin_unlock(&ov2675_drv_lock);
        }
        else
        {
		spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_iOV2675_MIPI_Mode = OV2675_MIPI_MODE_QVGA_VIDEO;
		spin_unlock(&ov2675_drv_lock);
        }
        //image_window->wait_stable_frame = 3;	
    }
    else
    {
        RETAILMSG(1,(TEXT("Camera preview\r\n")));
        //sensor_config_data->frame_rate == 30
        //ISP_PREVIEW_MODE
        //4  <2> if preview of capture PICTURE

        /* preview: 30 fps with 36M PCLK */
	spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);

        iDummyPixels = 0; 
        iDummyLines = 0; 

	spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_iOV2675_MIPI_Mode = OV2675_MIPI_MODE_PREVIEW;
	spin_unlock(&ov2675_drv_lock);
        // Set for dynamic sensor delay. 2009-09-09
        //image_window->wait_stable_frame = 3;	
    }
        if(1)
        {
        	iStartX = 13;
          iStartY = 1;
   
          OV2675_MIPI_write_cmos_sensor(0x3090,0x03);
          OV2675_MIPI_write_cmos_sensor(0x307c,0x10); 
             
          OV2675_MIPI_write_cmos_sensor(0x3350,0x32);
          OV2675_MIPI_write_cmos_sensor(0x3351,0x26);
          OV2675_MIPI_write_cmos_sensor(0x3352,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3353,0x2f);
          OV2675_MIPI_write_cmos_sensor(0x3354,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3355,0x85);
                                                   
          OV2675_MIPI_write_cmos_sensor(0x3356,0x33);
          OV2675_MIPI_write_cmos_sensor(0x3357,0x26);
          OV2675_MIPI_write_cmos_sensor(0x3358,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3359,0x2c);
          OV2675_MIPI_write_cmos_sensor(0x335a,0x00);
          OV2675_MIPI_write_cmos_sensor(0x335b,0x85);
                                                   
          OV2675_MIPI_write_cmos_sensor(0x335c,0x32);
          OV2675_MIPI_write_cmos_sensor(0x335d,0x26);
          OV2675_MIPI_write_cmos_sensor(0x335e,0x00);
          OV2675_MIPI_write_cmos_sensor(0x335f,0x2a);
          OV2675_MIPI_write_cmos_sensor(0x3360,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3361,0x85);
             
          OV2675_MIPI_write_cmos_sensor(0x3363,0x70);
          OV2675_MIPI_write_cmos_sensor(0x3364,0x7f);
          OV2675_MIPI_write_cmos_sensor(0x3365,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3366,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3362,0x90);
        }
        else
        {
          iStartX = 1;
          iStartY = 1;
          
          OV2675_MIPI_write_cmos_sensor(0x3090,0x0b); 
          OV2675_MIPI_write_cmos_sensor(0x307c,0x13); 
             
          OV2675_MIPI_write_cmos_sensor(0x3350,0x34);
          OV2675_MIPI_write_cmos_sensor(0x3351,0x29);
          OV2675_MIPI_write_cmos_sensor(0x3352,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3353,0x2e);
          OV2675_MIPI_write_cmos_sensor(0x3354,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3355,0x85);
             
          OV2675_MIPI_write_cmos_sensor(0x3356,0x34);
          OV2675_MIPI_write_cmos_sensor(0x3357,0x28);
          OV2675_MIPI_write_cmos_sensor(0x3358,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3359,0x2d);
          OV2675_MIPI_write_cmos_sensor(0x335a,0x00);
          OV2675_MIPI_write_cmos_sensor(0x335b,0x85);
                                                   
          OV2675_MIPI_write_cmos_sensor(0x335c,0x35);
          OV2675_MIPI_write_cmos_sensor(0x335d,0x29);
          OV2675_MIPI_write_cmos_sensor(0x335e,0x00);
          OV2675_MIPI_write_cmos_sensor(0x335f,0x2a);
          OV2675_MIPI_write_cmos_sensor(0x3360,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3361,0x85);
             
          OV2675_MIPI_write_cmos_sensor(0x3363,0x70);
          OV2675_MIPI_write_cmos_sensor(0x3364,0x7f);
          OV2675_MIPI_write_cmos_sensor(0x3365,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3366,0x00);
          OV2675_MIPI_write_cmos_sensor(0x3362,0x90);           
    }

    //4 <6> set dummy
	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_PV_Dummy_Pixels = iDummyPixels;
	spin_unlock(&ov2675_drv_lock);
   // OV2675_MIPI_set_dummy(iDummyPixels, iDummyLines);
    temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
    OV2675_MIPI_write_cmos_sensor(0x3013, temp_AE_reg|0x05);
    
    Sleep(100);  


    //4 <7> set shutter
    image_window->GrabStartX = iStartX;
    image_window->GrabStartY = iStartY;
    image_window->ExposureWindowWidth = OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH - iStartX -2;
    image_window->ExposureWindowHeight = OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT- iStartY -2;
            temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_DELAY_AFTER_PREVIEW = 1;
	spin_unlock(&ov2675_drv_lock);

	// copy sensor_config_data
	memcpy(&OV2675SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	
  	return ERROR_NONE;
}	/* OV2675Preview() */

UINT32 OV2675ZSD(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	// 1600x1200	  
	 spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_sensor_zsd_mode=KAL_TRUE;
	OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE;
	OV2675_MIPI_sensor_cap_state = KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);	
	SENSORDB("[OV2675ZSD]enterOV2675ZSD\n");
	OV2675_MIPI_write_cmos_sensor(0x3012,0x00);
	OV2675_MIPI_write_cmos_sensor(0x302a,0x04);
	OV2675_MIPI_write_cmos_sensor(0x302b,0xd4); 
	OV2675_MIPI_write_cmos_sensor(0x306f,0x54);
	OV2675_MIPI_write_cmos_sensor(0x3362,0x80); 		
	
	OV2675_MIPI_write_cmos_sensor(0x3020,0x01);
	OV2675_MIPI_write_cmos_sensor(0x3021,0x18);
	OV2675_MIPI_write_cmos_sensor(0x3022,0x00);
	OV2675_MIPI_write_cmos_sensor(0x3023,0x0a);
	OV2675_MIPI_write_cmos_sensor(0x3024,0x06);
	OV2675_MIPI_write_cmos_sensor(0x3025,0x58);
	OV2675_MIPI_write_cmos_sensor(0x3026,0x04);
	OV2675_MIPI_write_cmos_sensor(0x3027,0xbc); 
	OV2675_MIPI_write_cmos_sensor(0x3088,0x06);
	OV2675_MIPI_write_cmos_sensor(0x3089,0x40); 
	OV2675_MIPI_write_cmos_sensor(0x308a,0x04);
	OV2675_MIPI_write_cmos_sensor(0x308b,0xb0);
	OV2675_MIPI_write_cmos_sensor(0x3316,0x64);
	OV2675_MIPI_write_cmos_sensor(0x3317,0x4b); 
	OV2675_MIPI_write_cmos_sensor(0x3318,0x00);
	OV2675_MIPI_write_cmos_sensor(0x3319,0x2c); 
	OV2675_MIPI_write_cmos_sensor(0x331a,0x64); 
	OV2675_MIPI_write_cmos_sensor(0x331b,0x4b);
	OV2675_MIPI_write_cmos_sensor(0x331c,0x00); 
	OV2675_MIPI_write_cmos_sensor(0x331d,0x4c); 
	OV2675_MIPI_write_cmos_sensor(0x3302,0x01);
	
	OV2675_MIPI_write_cmos_sensor(0x3373,0x40);  
	OV2675_MIPI_write_cmos_sensor(0x3376,0x04); 
	
	   OV2675_MIPI_write_cmos_sensor(0x3010, 0x80); //15fps
	   OV2675_MIPI_write_cmos_sensor(0x300e, 0x34); 
	   OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
	   OV2675_MIPI_write_cmos_sensor(0x302a, 0x05);
	   OV2675_MIPI_write_cmos_sensor(0x302b, 0x3b); 
	
	   OV2675_MIPI_write_cmos_sensor(0x3070, 0xc8);
	   OV2675_MIPI_write_cmos_sensor(0x3072, 0xa9);
	   OV2675_MIPI_write_cmos_sensor(0x301c, 0x06);
	   OV2675_MIPI_write_cmos_sensor(0x301d, 0x08);
	
		   if(1)
		   {
	   
			 OV2675_MIPI_write_cmos_sensor(0x3090,0x03);
			 OV2675_MIPI_write_cmos_sensor(0x307c,0x10); 
				
			 OV2675_MIPI_write_cmos_sensor(0x3350,0x32);
			 OV2675_MIPI_write_cmos_sensor(0x3351,0x26);
			 OV2675_MIPI_write_cmos_sensor(0x3352,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3353,0x2f);
			 OV2675_MIPI_write_cmos_sensor(0x3354,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3355,0x85);
													  
			 OV2675_MIPI_write_cmos_sensor(0x3356,0x33);
			 OV2675_MIPI_write_cmos_sensor(0x3357,0x26);
			 OV2675_MIPI_write_cmos_sensor(0x3358,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3359,0x2c);
			 OV2675_MIPI_write_cmos_sensor(0x335a,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x335b,0x85);
													  
			 OV2675_MIPI_write_cmos_sensor(0x335c,0x32);
			 OV2675_MIPI_write_cmos_sensor(0x335d,0x26);
			 OV2675_MIPI_write_cmos_sensor(0x335e,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x335f,0x2a);
			 OV2675_MIPI_write_cmos_sensor(0x3360,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3361,0x85);
				
			 OV2675_MIPI_write_cmos_sensor(0x3363,0x70);
			 OV2675_MIPI_write_cmos_sensor(0x3364,0x7f);
			 OV2675_MIPI_write_cmos_sensor(0x3365,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3366,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3362,0x80);
		   }
		   else
		   {
			 
			 OV2675_MIPI_write_cmos_sensor(0x3090,0x0b); 
			 OV2675_MIPI_write_cmos_sensor(0x307c,0x13); 
				
			 OV2675_MIPI_write_cmos_sensor(0x3350,0x34);
			 OV2675_MIPI_write_cmos_sensor(0x3351,0x29);
			 OV2675_MIPI_write_cmos_sensor(0x3352,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3353,0x2e);
			 OV2675_MIPI_write_cmos_sensor(0x3354,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3355,0x85);
				
			 OV2675_MIPI_write_cmos_sensor(0x3356,0x34);
			 OV2675_MIPI_write_cmos_sensor(0x3357,0x28);
			 OV2675_MIPI_write_cmos_sensor(0x3358,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3359,0x2d);
			 OV2675_MIPI_write_cmos_sensor(0x335a,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x335b,0x85);
													  
			 OV2675_MIPI_write_cmos_sensor(0x335c,0x35);
			 OV2675_MIPI_write_cmos_sensor(0x335d,0x29);
			 OV2675_MIPI_write_cmos_sensor(0x335e,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x335f,0x2a);
			 OV2675_MIPI_write_cmos_sensor(0x3360,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3361,0x85);
				
			 OV2675_MIPI_write_cmos_sensor(0x3363,0x70);
			 OV2675_MIPI_write_cmos_sensor(0x3364,0x7f);
			 OV2675_MIPI_write_cmos_sensor(0x3365,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3366,0x00);
			 OV2675_MIPI_write_cmos_sensor(0x3362,0x80);		   
	   }
		   OV2675_MIPI_write_cmos_sensor(0x3013,0xf7); //turn on AEC/AGC	

 mdelay(200);
 printk("++++%s\n", __FUNCTION__);	//

    

    //printk("Capture Shutter = %d\, Capture Gain = %d\n", Capture_Shutter, Capture_Gain); 
    // AEC/AGC/AWB will be enable in preview and param_wb function
    /* total delay 4 frame for AE stable */
	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_DELAY_AFTER_PREVIEW = 2;
	spin_unlock(&ov2675_drv_lock);

	// copy sensor_config_data
    memcpy(&OV2675SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2675Capture() */

UINT32 OV2675Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    volatile kal_uint32 shutter = OV2675_MIPI_exposure_lines, temp_reg;
    kal_uint8 temp_AE_reg, temp;
    kal_uint16 AE_setting_delay = 0;

#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
	static kal_uint32 shutter_save, gain_save;
#endif

	spin_lock(&ov2675_drv_lock);
   OV2675_MIPI_sensor_cap_state = KAL_TRUE;
   
   OV2675_MIPI_sensor_zsd_mode=KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);  
 
 printk("++++%s\n", __FUNCTION__);	//

    temp_reg = OV2675_MIPI_read_cmos_sensor(0x3014);
    OV2675_MIPI_write_cmos_sensor(0x3014, temp_reg & 0xf7); //Disable night mode
        
    // turn off AEC/AGC
    OV2675_MIPI_set_AE_mode(KAL_FALSE);
    //temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
    //OV2675_MIPI_write_cmos_sensor(0x3013, (temp_AE_reg&(~0x05)) );        

    OV2675_MIPI_set_AWB_mode(KAL_FALSE); 

	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_exposure_line_h = OV2675_MIPI_read_cmos_sensor(0x3002);
    OV2675_MIPI_exposure_line_l = OV2675_MIPI_read_cmos_sensor(0x3003);
    OV2675_MIPI_extra_exposure_line_h = OV2675_MIPI_read_cmos_sensor(0x302D);
    OV2675_MIPI_extra_exposure_line_l = OV2675_MIPI_read_cmos_sensor(0x302E);
	spin_unlock(&ov2675_drv_lock);
    OV2675_MIPI_PV_Gain16 = OV2675_MIPI_read_OV2675_MIPI_gain();
    shutter = OV2675_MIPI_read_shutter();
/*
    if ((image_window->ImageTargetWidth<=OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT))
    {    /* Less than PV Mode */
/*
	spin_lock(&ov2675_drv_lock);
        image_window->GrabStartY = 1;
        image_window->ExposureWindowWidth= OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH - 3;
        image_window->ExposureWindowHeight = OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT - 3;

    }*/
  //  else 
    {    
    	 /* 2M FULL Mode */
        image_window->GrabStartX=  1;
        image_window->GrabStartY=6;
        image_window->ExposureWindowWidth=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH - image_window->GrabStartX - 2;
        image_window->ExposureWindowHeight=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT -image_window->GrabStartY - 2;    	 
        #if 0	 
        //calculator auto uv
        OV2675_MIPI_write_cmos_sensor(0x330c,0x3e);
        temp = OV2675_MIPI_read_cmos_sensor(0x330f);
        OV2675_MIPI_write_cmos_sensor(0x3301,0xbf);//turn off auto uv
        // temp = (temp&0x1f + 0x01)<<1;
        temp = temp&0x1f;
        temp = temp + 0x01;
        temp = 2*temp;
        OV2675_MIPI_write_cmos_sensor(0x3391, (OV2675_MIPI_read_cmos_sensor(0x3391) | 0x02));  //  enable Saturation
        OV2675_MIPI_write_cmos_sensor(0x3394,temp);
        OV2675_MIPI_write_cmos_sensor(0x3395,temp);
        #endif
        
        // 1600x1200      
        OV2675_MIPI_write_cmos_sensor(0x3012,0x00);
        OV2675_MIPI_write_cmos_sensor(0x302a,0x04);
        OV2675_MIPI_write_cmos_sensor(0x302b,0xd4); 
        OV2675_MIPI_write_cmos_sensor(0x306f,0x54);
        OV2675_MIPI_write_cmos_sensor(0x3362,0x80);         
      
        OV2675_MIPI_write_cmos_sensor(0x3020,0x01);
        OV2675_MIPI_write_cmos_sensor(0x3021,0x18);
        OV2675_MIPI_write_cmos_sensor(0x3022,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3023,0x0a);
        OV2675_MIPI_write_cmos_sensor(0x3024,0x06);
        OV2675_MIPI_write_cmos_sensor(0x3025,0x58);
        OV2675_MIPI_write_cmos_sensor(0x3026,0x04);
        OV2675_MIPI_write_cmos_sensor(0x3027,0xbc); 
        OV2675_MIPI_write_cmos_sensor(0x3088,0x06);
        OV2675_MIPI_write_cmos_sensor(0x3089,0x40); 
        OV2675_MIPI_write_cmos_sensor(0x308a,0x04);
        OV2675_MIPI_write_cmos_sensor(0x308b,0xb0);
        OV2675_MIPI_write_cmos_sensor(0x3316,0x64);
        OV2675_MIPI_write_cmos_sensor(0x3317,0x4b); 
        OV2675_MIPI_write_cmos_sensor(0x3318,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3319,0x2c); 
        OV2675_MIPI_write_cmos_sensor(0x331a,0x64); 
        OV2675_MIPI_write_cmos_sensor(0x331b,0x4b);
        OV2675_MIPI_write_cmos_sensor(0x331c,0x00); 
        OV2675_MIPI_write_cmos_sensor(0x331d,0x4c); 
        OV2675_MIPI_write_cmos_sensor(0x3302,0x01);
        
        OV2675_MIPI_write_cmos_sensor(0x3373,0x40);  
        OV2675_MIPI_write_cmos_sensor(0x3376,0x04); 
        
        //OV2675_MIPI_write_cmos_sensor(0x3016,0xc2); 
        //OV2675_MIPI_write_cmos_sensor(0x3069,0x80);        

	spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_gPVmode = KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);
           
        if ((image_window->ImageTargetWidth<=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH)&&
        (image_window->ImageTargetHeight<=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT))
        {     
	        if (zoom_factor  <  3) 
	        {  
			//48Mhz Full size Capture CLK
		     OV2675_MIPI_write_cmos_sensor(0x3011, 0x00); //01  //longxuewei
		     OV2675_MIPI_write_cmos_sensor(0x300e, 0x34);  //38	
		     OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);
				// OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);

			spin_lock(&ov2675_drv_lock);
	            OV2675_MIPI_sensor_pclk = 780;//520  //longxuewei
			OV2675_MIPI_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
			OV2675_MIPI_dummy_lines=0;
			OV2675_MIPI_capture_pclk_in_M = 780;   //Don't change the clk
			spin_unlock(&ov2675_drv_lock);
			//10 fps 1 frame = 100ms = 30
			AE_setting_delay = 26; 	        
	        }
	        else 
	        {
			//48Mhz Full size Capture CLK/2
			OV2675_MIPI_write_cmos_sensor(0x3011, 0x01);
			OV2675_MIPI_write_cmos_sensor(0x300e, 0x38); 
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);

			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_sensor_pclk = 260;	            
			OV2675_MIPI_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
			OV2675_MIPI_dummy_lines=0;
			OV2675_MIPI_capture_pclk_in_M = 260;   //Don't change the clk
			spin_unlock(&ov2675_drv_lock);

			//9.3 fps, 1 frame = 200ms
			AE_setting_delay = 30;

	        }                  
        }
        else//Interpolate to 3M
        {
  	        if (image_window->ZoomFactor >= 340)
	        {  	        
			//48Mhz Full size Capture CLK/4
			OV2675_MIPI_write_cmos_sensor(0x3011, 0x03);
			OV2675_MIPI_write_cmos_sensor(0x300e, 0x38);  
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x84);

			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_sensor_pclk = 130;	
			OV2675_MIPI_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
			OV2675_MIPI_dummy_lines=0;
			OV2675_MIPI_capture_pclk_in_M = 130;   //Don't change the clk
			spin_unlock(&ov2675_drv_lock);

			//9.3 fps, 1 frame = 200ms
			AE_setting_delay = 34;
	        }
	        else if (image_window->ZoomFactor >= 240) 
	        {          
			//48Mhz Full size Capture CLK/2
			OV2675_MIPI_write_cmos_sensor(0x3011, 0x01); 
			OV2675_MIPI_write_cmos_sensor(0x300e, 0x38);  
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);

			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_sensor_pclk = 260;
			OV2675_MIPI_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
			OV2675_MIPI_dummy_lines=0;
			OV2675_MIPI_capture_pclk_in_M = 260;   //Don't change the clk
			spin_unlock(&ov2675_drv_lock);

			//9.3 fps, 1 frame = 200ms
			AE_setting_delay = 30;
	        }
	        else 
	        {  
			//48Mhz Full size Capture CLK
			OV2675_MIPI_write_cmos_sensor(0x3011, 0x00); 
			OV2675_MIPI_write_cmos_sensor(0x300e, 0x38);
			OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);

			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_sensor_pclk = 520;
			OV2675_MIPI_dummy_pixels=0;  /*If Capture fail, you can add this dummy*/
			OV2675_MIPI_dummy_lines=0;
			OV2675_MIPI_capture_pclk_in_M = 520;   //Don't change the clk
			spin_unlock(&ov2675_drv_lock);

			//10 fps 1 frame = 100ms = 30
			AE_setting_delay = 26; 
	        }                  
        }
       
	spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_Capture_Dummy_Pixels = OV2675_MIPI_dummy_pixels ;
        OV2675_MIPI_Capture_Dummy_Lines = OV2675_MIPI_dummy_lines;
	spin_unlock(&ov2675_drv_lock);

        //Jerry It need to change gain to shutter
        OV2675_MIPI_Computer_AECAGC(OV2675_MIPI_preview_pclk_in_M, OV2675_MIPI_capture_pclk_in_M);
        shutter = OV2675_MIPI_Capture_Shutter + OV2675_MIPI_Capture_Extra_Lines;       

        // set dummy
        OV2675_MIPI_set_dummy(OV2675_MIPI_dummy_pixels, OV2675_MIPI_dummy_lines);
       
        if (shutter < 1) 
        {
            shutter = 1;
        }
        if (OV2675_MIPI_AE_ENABLE == KAL_TRUE)
        //  if (OV2675_MIPI_AE_ENABLE == KAL_FALSE)
        {
		spin_lock(&ov2675_drv_lock);
		Capture_Shutter = shutter; 
		Capture_Gain = OV2675_MIPI_Capture_Gain16;
		spin_unlock(&ov2675_drv_lock);
        }

	#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
		if(preview_init_flag == 1)
		{
			shutter_save = Capture_Shutter;
			gain_save = Capture_Gain;
			preview_init_flag = 0;
		}
		else
		{
			spin_lock(&ov2675_drv_lock);
			Capture_Shutter = shutter_save;
			Capture_Gain = gain_save;
			spin_unlock(&ov2675_drv_lock);
		}
		//printk("OV3660_Capture,shutter=%d,gain=%d",shutter,gain);
	#endif

        
        // set shutter OVT
      //  OV2675_MIPI_write_shutter(Capture_Shutter);
          OV2675_MIPI_write_shutter(shutter);
		/*
        if(OV2675_MIPI_Capture_Gain16>62)
            OV2675_MIPI_write_OV2675_MIPI_gain(16); 
        else
            OV2675_MIPI_write_OV2675_MIPI_gain((OV2675_MIPI_Capture_Gain16+5)); 
        */
        //kal_sleep_task(23);  //delay 1frame
       // Sleep(95);
         Capture_Gain = OV2675_MIPI_Capture_Gain16;

        OV2675_MIPI_write_OV2675_MIPI_gain(Capture_Gain); 
    }

    //printk("Capture Shutter = %d\, Capture Gain = %d\n", Capture_Shutter, Capture_Gain); 
    // AEC/AGC/AWB will be enable in preview and param_wb function
    /* total delay 4 frame for AE stable */
	spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_DELAY_AFTER_PREVIEW = 2;
	spin_unlock(&ov2675_drv_lock);

	// copy sensor_config_data
    memcpy(&OV2675SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2675Capture() */

UINT32 OV2675GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{ 
	pSensorResolution->SensorFullWidth=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH - 16;  //modify by yanxu
	pSensorResolution->SensorFullHeight=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT  -7;
	pSensorResolution->SensorPreviewWidth=OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH -4;
	pSensorResolution->SensorPreviewHeight=OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT -3;
	pSensorResolution->SensorVideoWidth=OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH - 4 ;
	pSensorResolution->SensorVideoHeight=OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT - 3 ;
	

	return ERROR_NONE;
}	/* OV2675GetResolution() */
void OV2675GetDelayInfo(UINT32 delayAddr)
{
	SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay = 3;
	pDelayInfo->EffectDelay = 2;
	pDelayInfo->AwbDelay = 2;
}

void OV2675_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
		  SENSORDB("SENSOR_3A_AE_LOCK \n,");
          OV2675_MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
		  SENSORDB("SENSOR_3A_AE_UNLOCK \n,");
          OV2675_MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
		  SENSORDB("SENSOR_3A_AWB_LOCK \n,");
		  OV2675_MIPI_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
		  SENSORDB("SENSOR_3A_AE_LOCK \n,");
		  OV2675_MIPI_set_AWB_mode(KAL_TRUE);
      break;
      default:
      	break;
   }
   return;
}

void OV2675GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
	*pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
	SENSORDB("S5K8AAYX_MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}
void OV2675AutoTestCmd(UINT32 *cmd, UINT32 *para)
{
	switch(*cmd){
		case YUV_AUTOTEST_SET_SHADDING:
			SENSORDB("YUV_AUTOTEST_SET_SHADDING:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_GAMMA:
			SENSORDB("YUV_AUTOTEST_SET_GAMMA:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_AE:
			SENSORDB("YUV_AUTOTEST_SET_AE:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_SHUTTER:
			SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_GAIN:
			SENSORDB("YUV_AUTOTEST_SET_GAIN:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			*para = 8228;
		break;
		default:	
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
		break;
	}
}

UINT32 OV2675GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;    //SENSOR_INTERFACE_TYPE_PARALLEL

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 		
	pSensorInfo->YUVAwbDelayFrame=2;
	pSensorInfo->YUVEffectDelayFrame=2;
	pSensorInfo->SensorMasterClockSwitch = 0; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;  	

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			   pSensorInfo->SensorClockFreq=26;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = 4; 
         pSensorInfo->SensorGrabStartY = 2;        
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
		     break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			   pSensorInfo->SensorClockFreq=26;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = 4; 
         pSensorInfo->SensorGrabStartY = 2;   
		 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
				 break;
		default:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
        	pSensorInfo->SensorGrabStartX = 4; 
         	pSensorInfo->SensorGrabStartY = 2; 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
	       break;
	}
	spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	spin_unlock(&ov2675_drv_lock);
	memcpy(pSensorConfigData, &OV2675SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2675GetInfo() */


UINT32 OV2675Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			   OV2675Preview(pImageWindow, pSensorConfigData);
		     break;
			 case MSDK_SCENARIO_ID_CAMERA_ZSD:
				 OV2675ZSD(pImageWindow, pSensorConfigData);
				 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			   OV2675Capture(pImageWindow, pSensorConfigData);
         mdelay(300); //delay 3 frame
		 break;
		default:
		     break; 
	}
	return TRUE;
}	/* OV2675Control() */

/* [TC] YUV sensor */	

BOOL OV2675_MIPI_set_param_wb(UINT16 para)
{
    kal_uint8  temp_reg,temp_AE_reg;

    temp_reg=OV2675_MIPI_read_cmos_sensor(0x3306);

    switch (para)
    {
        case AWB_MODE_OFF:
            //OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            //OV2675_MIPI_set_AWB_mode(OV2675_MIPI_AWB_ENABLE);
            //break;                     
        case AWB_MODE_AUTO:
		spin_lock(&ov2675_drv_lock);
	            OV2675_MIPI_AWB_ENABLE = KAL_TRUE;  
		spin_unlock(&ov2675_drv_lock);
		OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_set_AWB_mode(OV2675_MIPI_AWB_ENABLE);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg&~0x2);   // select Auto WB
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg|0x2);  // select manual WB
            OV2675_MIPI_write_cmos_sensor(0x3337, 0x68); //manual R G B
            OV2675_MIPI_write_cmos_sensor(0x3338, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3339, 0x4e);              
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;

        case AWB_MODE_DAYLIGHT: //sunny
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg|0x2);  // Disable AWB
            OV2675_MIPI_write_cmos_sensor(0x3337, 0x5e);
            OV2675_MIPI_write_cmos_sensor(0x3338, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3339, 0x46);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;

        case AWB_MODE_INCANDESCENT: //office
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg|0x2);  // Disable AWB
            OV2675_MIPI_write_cmos_sensor(0x3337, 0x5e);
            OV2675_MIPI_write_cmos_sensor(0x3338, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3339, 0x58);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;

        case AWB_MODE_TUNGSTEN: //home
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg|0x2);  // Disable AWB
            OV2675_MIPI_write_cmos_sensor(0x3337, 0x54);
            OV2675_MIPI_write_cmos_sensor(0x3338, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3339, 0x70);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;

        case AWB_MODE_FLUORESCENT:
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
            OV2675_MIPI_write_cmos_sensor(0x3306, temp_reg|0x2); // Disable AWB
            OV2675_MIPI_write_cmos_sensor(0x3337, 0x65);
            OV2675_MIPI_write_cmos_sensor(0x3338, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3339, 0x41);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
            break;
#if WINMO_USE
        case AWB_MODE_MANUAL:
            // TODO
            break;
#endif 

        default:
            return FALSE;
    }

    return TRUE;
} /* OV2675_MIPI_set_param_wb */

BOOL OV2675_MIPI_set_param_effect(UINT16 para)
{
   BOOL  ret = TRUE;
   //UINT8  temp_reg;
   //temp_reg=OV2675_MIPI_read_cmos_sensor(0x3391);
    switch (para)
    {
        case MEFFECT_OFF: 
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x06);
			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			 mdelay(200);
			 OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
 
             break;

        case MEFFECT_SEPIA:
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x1e);
            OV2675_MIPI_write_cmos_sensor(0x3396, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3397, 0xa6);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
              break;

        case MEFFECT_NEGATIVE:
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x46);

			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			 mdelay(200);
			 OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
             break;

        case MEFFECT_SEPIAGREEN:
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x1e);
            OV2675_MIPI_write_cmos_sensor(0x3396, 0x60);
            OV2675_MIPI_write_cmos_sensor(0x3397, 0x60);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
             break;

        case MEFFECT_SEPIABLUE:
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x1e);
            OV2675_MIPI_write_cmos_sensor(0x3396, 0xf0);
            OV2675_MIPI_write_cmos_sensor(0x3397, 0x60);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			mdelay(200);
			OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
             break;
		    case MEFFECT_MONO: //B&W
				OV2675_MIPI_write_cmos_sensor(0x308c, 0x88);
             OV2675_MIPI_write_cmos_sensor(0x3391, 0x26);
			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0xff);
			 mdelay(200);
			 OV2675_MIPI_write_cmos_sensor(0x308c, 0x80);
			 OV2675_MIPI_write_cmos_sensor(0x30ff, 0x00);
 			      break;
#if WINMO_USE
        case CAM_EFFECT_ENC_GRAYINV:
        case CAM_EFFECT_ENC_COPPERCARVING:
        case CAM_EFFECT_ENC_BLUECARVING:
        case CAM_EFFECT_ENC_EMBOSSMENT:
        case CAM_EFFECT_ENC_SKETCH:
        case CAM_EFFECT_ENC_BLACKBOARD:
        case CAM_EFFECT_ENC_WHITEBOARD:
        case CAM_EFFECT_ENC_JEAN:
        case CAM_EFFECT_ENC_OIL:
#endif 
        default:
            ret = FALSE;
    }

    return ret;

} /* OV2675_MIPI_set_param_effect */

BOOL OV2675_MIPI_set_param_exposure(UINT16 para)
{
 #if 1
 kal_uint8  temp_reg;

    temp_reg=OV2675_MIPI_read_cmos_sensor(0x3391);

    switch (para)
    {
        case AE_EV_COMP_n20:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x40);
            break;

       /* case AE_EVs_COMP_n10:    
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x28);/* for difference */
          //  break;

        case AE_EV_COMP_n10:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x20);
            break;

       /* case AE_EV_COMP_n03:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x10);
            break;*/

        case AE_EV_COMP_00:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x00);
            break;
/*
        case AE_EV_COMP_03:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x10);
            break;
*/
        case AE_EV_COMP_10:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x20);
            break;
/*
        case AE_EV_COMP_10:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x28);/* for difference */
         /*   break;*/

        case AE_EV_COMP_20:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x40);
            break;

        default:
            return FALSE;
    }
#endif

    return TRUE;
} /* OV2675_MIPI_set_param_exposure */
#if 1
void OV2675_MIPI_SetBrightness(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
    //S5K4ECGX_write_cmos_sensor(0xFCFC ,0xD000);  
    //S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    //S5K4ECGX_write_cmos_sensor(0x002A ,0x0230);  
 
    switch (para)
    {
        case ISP_BRIGHT_LOW:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_LOW\n");
			OV2675_MIPI_write_cmos_sensor(0x3018, 0x40);
			OV2675_MIPI_write_cmos_sensor(0x3019 ,0x30);  
			OV2675_MIPI_write_cmos_sensor(0x301a ,0x71);

             break; 
        case ISP_BRIGHT_HIGH:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_HIGH\n");
			OV2675_MIPI_write_cmos_sensor(0x3018, 0xa8);
			OV2675_MIPI_write_cmos_sensor(0x3019 ,0x98);  
			OV2675_MIPI_write_cmos_sensor(0x301a ,0xe6);

             break; 
        case ISP_BRIGHT_MIDDLE:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_MIDDLE\n");
			OV2675_MIPI_write_cmos_sensor(0x3018, 0x78);
			OV2675_MIPI_write_cmos_sensor(0x3019 ,0x68);  
			OV2675_MIPI_write_cmos_sensor(0x301a ,0xc4);
        default:
             break; 
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);
    return;
}
#endif


void OV2675_MIPI_SetContrast(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
 
    switch (para)
    {
        case ISP_CONTRAST_LOW:
			OV2675_MIPI_write_cmos_sensor(0x3398, 0x10);
			OV2675_MIPI_write_cmos_sensor(0x3399 ,0x10);  
             break; 
        case ISP_CONTRAST_HIGH:
			OV2675_MIPI_write_cmos_sensor(0x3398, 0x30);
			OV2675_MIPI_write_cmos_sensor(0x3399 ,0x30);  
             break; 
        case ISP_CONTRAST_MIDDLE:
			OV2675_MIPI_write_cmos_sensor(0x3398, 0x20);
			OV2675_MIPI_write_cmos_sensor(0x3399 ,0x20);  
        default:
             break; 
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);	
    return;
}



void OV2675_MIPI_SetSetIso(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
    switch (para)
    {
        case AE_ISO_100:
             //ISO 100
            
			 OV2675_MIPI_write_cmos_sensor(0x3015, (OV2675_MIPI_read_cmos_sensor(0x3015)&0xf0)|0x01);  
             break; 
        case AE_ISO_200:
             //ISO 200
           
			 OV2675_MIPI_write_cmos_sensor(0x3015, (OV2675_MIPI_read_cmos_sensor(0x3015)&0xf0)|0x02);  
        default:
        case AE_ISO_AUTO:
             //ISO Auto
            
			// OV2675_MIPI_write_cmos_sensor(0x3015, 0x01);  
             break; 
    }	
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);	
}


void OV2675_MIPI_SetSaturation(UINT16 para)
{
    switch (para)
    {
        case ISP_SAT_HIGH:
			OV2675_MIPI_write_cmos_sensor(0x3391, OV2675_MIPI_read_cmos_sensor(0x3391)|0x06);
			OV2675_MIPI_write_cmos_sensor(0x3394 ,0x50);  
			OV2675_MIPI_write_cmos_sensor(0x3395 ,0x50);
			
             break; 
        case ISP_SAT_LOW:
			
			OV2675_MIPI_write_cmos_sensor(0x3391, OV2675_MIPI_read_cmos_sensor(0x3391)|0x06);
			OV2675_MIPI_write_cmos_sensor(0x3394 ,0x30);  
			OV2675_MIPI_write_cmos_sensor(0x3395 ,0x30);
             break; 
        case ISP_SAT_MIDDLE:
			OV2675_MIPI_write_cmos_sensor(0x3391, OV2675_MIPI_read_cmos_sensor(0x3391)|0x06);
			OV2675_MIPI_write_cmos_sensor(0x3394 ,0x40);  
			OV2675_MIPI_write_cmos_sensor(0x3395 ,0x40);
        default:
             break; 
    }	
}




UINT32 OV2675MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
//   if( OV2675_MIPI_sensor_cap_state == KAL_TRUE)
//	   return TRUE;

	switch (iCmd) {
	case FID_SCENE_MODE:	
//	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF||iPara == SCENE_MODE_NORMAL)
	    {
	        if(OV2675_MIPI_sensor_zsd_mode==KAL_TRUE)
	        OV2675_MIPI_night_ZSD_mode(0); 
			else
				
	        OV2675_MIPI_night_mode(0); 
			

	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
	    
		if(OV2675_MIPI_sensor_zsd_mode==KAL_TRUE)
	        OV2675_MIPI_night_ZSD_mode(1); 
		else
               OV2675_MIPI_night_mode(1); 		
			
	    }	    
	    break; 	    
	case FID_AWB_MODE:
//	    printk("Set AWB Mode:%d\n", iPara); 	    
           OV2675_MIPI_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
//	    printk("Set Color Effect:%d\n", iPara); 	    	    
           OV2675_MIPI_set_param_effect(iPara);
	break;
	case FID_AE_EV:
#if WINMO_USE	    
	case ISP_FEATURE_EXPOSURE:
#endif 	    
//           printk("Set EV:%d\n", iPara); 	    	    
           OV2675_MIPI_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
//           printk("Set Flicker:%d\n", iPara); 	    	    	    
           OV2675_MIPI_set_param_banding(iPara);
	break;
    case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
		spin_lock(&ov2675_drv_lock);
               // OV2675_MIPI_AE_ENABLE = KAL_FALSE; 
                OV2675_MIPI_AE_ENABLE = KAL_TRUE;
		spin_unlock(&ov2675_drv_lock);
            }
            else {
		spin_lock(&ov2675_drv_lock);
                OV2675_MIPI_AE_ENABLE = KAL_TRUE; 
		spin_unlock(&ov2675_drv_lock);
	    }
            OV2675_MIPI_set_AE_mode(OV2675_MIPI_AE_ENABLE);
            break; 
	case FID_ZOOM_FACTOR:
		spin_lock(&ov2675_drv_lock);
		zoom_factor = iPara; 
		spin_unlock(&ov2675_drv_lock);
        break; 
		//case FID_AE_SCENE_MODE: 
		//	OV2675_MIPI__set_AE_mode(iPara);
		//	break; 
			case FID_ISP_CONTRAST:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_CONTRAST:%d\n",iPara);
				OV2675_MIPI_SetContrast(iPara);
				break;
			case FID_ISP_BRIGHT:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_BRIGHT:%d\n",iPara);
				OV2675_MIPI_SetBrightness(iPara);
				break;
			case FID_ISP_SAT:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_SAT:%d\n",iPara);
				OV2675_MIPI_SetSaturation(iPara);
				break;
			/*case FID_AE_ISO:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_ISO:%d\n",iPara);
				OV2675_MIPI_SetSetIso(iPara);
				break;*/
	default:
	break;
	}
	return TRUE;
}   /* OV2675MIPIYUVSensorSetting */

UINT32 OV2675MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  
    iTemp = OV2675_MIPI_read_cmos_sensor(0x3014);
    OV2675_MIPI_write_cmos_sensor(0x3014, iTemp&0xf7); //Disable night mode

    if (u2FrameRate == 30)
    {
    	  OV2675_MIPI_write_cmos_sensor(0x3010, 0x80);//30fps
        OV2675_MIPI_write_cmos_sensor(0x3011, 0x00);
        OV2675_MIPI_write_cmos_sensor(0x300e, 0x34);
        OV2675_MIPI_write_cmos_sensor(0x302a, 0x02);  
        OV2675_MIPI_write_cmos_sensor(0x302b, 0x9e);
                
        OV2675_MIPI_write_cmos_sensor(0x3070, 0xc8);
        OV2675_MIPI_write_cmos_sensor(0x3072, 0xa9);
        OV2675_MIPI_write_cmos_sensor(0x301c, 0x02);
        OV2675_MIPI_write_cmos_sensor(0x301d, 0x03);
        OV2675_MIPI_write_cmos_sensor(0x3015, 0x02);
        OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
        OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);
    }
    else if (u2FrameRate == 15)       
    {
    	  OV2675_MIPI_write_cmos_sensor(0x3010, 0x82);//15fps
        OV2675_MIPI_write_cmos_sensor(0x3011, 0x01);
        OV2675_MIPI_write_cmos_sensor(0x300e, 0x34);
        OV2675_MIPI_write_cmos_sensor(0x302a, 0x02);  
        OV2675_MIPI_write_cmos_sensor(0x302b, 0x9e);
                
        OV2675_MIPI_write_cmos_sensor(0x3070, 0x64);
        OV2675_MIPI_write_cmos_sensor(0x3072, 0x54);
        OV2675_MIPI_write_cmos_sensor(0x301c, 0x05);
        OV2675_MIPI_write_cmos_sensor(0x301d, 0x07);
        OV2675_MIPI_write_cmos_sensor(0x3015, 0x02);
        OV2675_MIPI_write_cmos_sensor(0x302d, 0x00);
        OV2675_MIPI_write_cmos_sensor(0x302e, 0x00);   
    }
    else 
    {
        printk("Wrong frame rate setting \n");
    }

	spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_VEDIO_encode_mode = KAL_TRUE;
	spin_unlock(&ov2675_drv_lock);
        
    return TRUE;
}

UINT32 OV2675MIPIYUVSetSoftwarePWDNMode(kal_bool bEnable)
{
    SENSORDB("[OV2675MIPIYUVSetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);
    /*if(bEnable) {   // enable software power down mode   
	     OV2675_MIPI_write_cmos_sensor(0x3086, 0x01);
    } else {
       OV2675_MIPI_write_cmos_sensor(0x3086, 0x00);  
    }*/
    return TRUE;
}
/*************************************************************************
* FUNCTION
*OV2675MIPIClose
*
* DESCRIPTION
* This OV2675SetMaxFramerateByScenario is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
  UINT32 OV2675MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("OV2675SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	/*switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = 134200000;
			lineLength = IMX111MIPI_PV_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX111MIPI_PV_FRAME_LENGTH_LINES;
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	*/
	return ERROR_NONE;
}
  /*************************************************************************
  * FUNCTION
  * OV2675GetDefaultFramerateByScenario
  *
  * DESCRIPTION
  * This function is to turn off sensor module power.
  * RETURNS
  * None
  *
  * GLOBALS AFFECTED
  *
  *************************************************************************/
UINT32 OV2675MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 220;
			break;		// 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

  	}

UINT32 OV2675FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=OV2675_MIPI_PV_PERIOD_PIXEL_NUMS+OV2675_MIPI_PV_dummy_pixels;
			*pFeatureReturnPara16=OV2675_MIPI_PV_PERIOD_LINE_NUMS+OV2675_MIPI_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = OV2675_MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV2675_MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&ov2675_drv_lock);
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV2675_MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV2675_MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV2675SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
                        *pFeatureReturnPara32++=0;
                        *pFeatureParaLen=4;	    
		    break; 
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 OV2675_MIPI_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
//		       printk("OV2675 MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			OV2675MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       OV2675MIPIYUVSetVideoMode(*pFeatureData16);
		       break; 
	        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
	            OV2675MIPIYUVSetSoftwarePWDNMode((BOOL)*pFeatureData16);        	        	
	            break;
			//	case SENSOR_CMD_SET_VIDEO_FRAME_RATE:
			//		break;
				case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
					OV2675GetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
					break;
					case SENSOR_FEATURE_SET_YUV_3A_CMD:
					   OV2675_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
					   break;
				case SENSOR_FEATURE_GET_DELAY_INFO:
					SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
					OV2675GetDelayInfo(*pFeatureData32);
					break;
				case SENSOR_FEATURE_AUTOTEST_CMD:
					SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
					OV2675AutoTestCmd((*pFeatureData32),*(pFeatureData32+1));
					break;
		default:
			break;			
	}
	return ERROR_NONE;
}	/* OV2675FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV2675=
{
	OV2675Open,
	OV2675GetInfo,
	OV2675GetResolution,
	OV2675FeatureControl,
	OV2675Control,
	OV2675Close
};

UINT32 OV2675_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2675;

	return ERROR_NONE;
}	/* SensorInit() */



