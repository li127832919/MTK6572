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
 *   adv7280_yuv_Sensor.c
 *
 * Project:
 * --------
 *   MAUI
 *
 * Description:
 * ------------
 *   Image sensor driver function
 *   V1.0.0
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
 * 2011/10/25 Firsty Released By Mormo(using "ADV7280.set Revision1721" )
 *   
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *=============================================================
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "adv7280_yuv_Sensor.h"
#include "adv7280_yuv_Camera_Sensor_para.h"
#include "adv7280_yuv_CameraCustomized.h"

#define ADV7280YUV_DEBUG
#ifdef ADV7280YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif
struct ADV7280_Sensor_Struct
{
	//kal_uint8   Banding;
//	kal_bool	  NightMode;
//	kal_bool	  VideoMode;
//	kal_uint16  Fps;
//	kal_uint16  ShutterStep;
//	kal_uint8   IsPVmode;
//	kal_uint32  PreviewDummyPixels;
//	kal_uint32  PreviewDummyLines;
//	kal_uint32  CaptureDummyPixels;
//	kal_uint32  CaptureDummyLines;
//	kal_uint32  PreviewPclk;
//	kal_uint32  CapturePclk;
//	kal_uint32  ZsdturePclk;
//	kal_uint32  PreviewShutter;
//	kal_uint32  PreviewExtraShutter;
//	kal_uint32  SensorGain;
//	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
//	kal_uint32      currentExposureTime;
//    kal_uint32      currentShutter;
//	kal_uint32      currentextshutter;
//    kal_uint32      currentAxDGain;
	kal_uint32  	sceneMode;
    unsigned char isoSpeed;
	kal_bool    	AE_ENABLE;
	
//	unsigned char zsd_flag;
//	ADV7280MIPI_SENSOR_MODE SensorMode;
	kal_uint16 wb;

} ;

static struct ADV7280_Sensor_Struct ADV7280_SensorDriver;
static DEFINE_SPINLOCK(ADV7280_drv_lock);
static MSDK_SCENARIO_ID_ENUM ADV7280_CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
#define ADV7280_TEST_PATTERN_CHECKSUM 0x12345678

static kal_uint32 ADV7280_zoom_factor = 0; 


///////////////////////////////////
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int kdCheckGreySensor;

/*************************************************************************
* FUNCTION
*    ADV7280_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void ADV7280_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    //iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), ADV7280_WRITE_ID); 

#if (defined(__ADV7280_DEBUG_TRACE__))
  if (sizeof(out_buff) != rt) printk("I2C write %x, %x error\n", addr, para);
#endif
}

/*************************************************************************
* FUNCTION
*    ADV7280_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 ADV7280_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;
   
    //if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), ADV7280_WRITE_ID)) {
    //    SENSORDB("ERROR: ADV7280_read_cmos_sensor \n");
    //}

#if (defined(__ADV7280_DEBUG_TRACE__))
  if (size != rt) printk("I2C read %x error\n", addr);
#endif

  return in_buff[0];
}

/*******************************************************************************
 * // Adapter for Winmo typedef
 ********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

kal_bool   ADV7280_MPEG4_encode_mode = KAL_FALSE;
kal_uint16 ADV7280_dummy_pixels = 0, ADV7280_dummy_lines = 0;
kal_bool   ADV7280_MODE_CAPTURE = KAL_FALSE;
kal_bool   ADV7280_CAM_BANDING_50HZ = KAL_FALSE;

kal_uint32 ADV7280_isp_master_clock;
static kal_uint32 ADV7280_g_fPV_PCLK = 24;

kal_uint8 ADV7280_sensor_write_I2C_address = ADV7280_WRITE_ID;
kal_uint8 ADV7280_sensor_read_I2C_address = ADV7280_READ_ID;

UINT8 ADV7280PixelClockDivider=0;

MSDK_SENSOR_CONFIG_STRUCT ADV7280SensorConfigData;

#define ADV7280_SET_PAGE0 	ADV7280_write_cmos_sensor(0xfe, 0x00)
#define ADV7280_SET_PAGE1 	ADV7280_write_cmos_sensor(0xfe, 0x01)

kal_bool ADV7280_night_mode_enable = KAL_FALSE;

/*************************************************************************
 * FUNCTION
 *	ADV7280_SetShutter
 *
 * DESCRIPTION
 *	This function set e-shutter of ADV7280 to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_Set_Shutter(kal_uint16 iShutter)
{
} /* Set_ADV7280_Shutter */


/*************************************************************************
 * FUNCTION
 *	ADV7280_read_Shutter
 *
 * DESCRIPTION
 *	This function read e-shutter of ADV7280 .
 *
 * PARAMETERS
 *  None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 ADV7280_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 shutter;

	temp_reg1 = ADV7280_read_cmos_sensor(0x04);
	temp_reg2 = ADV7280_read_cmos_sensor(0x03);

	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	return shutter;
} /* ADV7280_read_shutter */


/*************************************************************************
 * FUNCTION
 *	ADV7280_write_reg
 *
 * DESCRIPTION
 *	This function set the register of ADV7280.
 *
 * PARAMETERS
 *	addr : the register index of ADV7280
 *  para : setting parameter of the specified register of ADV7280
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_write_reg(kal_uint32 addr, kal_uint32 para)
{
	ADV7280_write_cmos_sensor(addr, para);
} /* ADV7280_write_reg() */


/*************************************************************************
 * FUNCTION
 *	ADV7280_read_cmos_sensor
 *
 * DESCRIPTION
 *	This function read parameter of specified register from ADV7280.
 *
 * PARAMETERS
 *	addr : the register index of ADV7280
 *
 * RETURNS
 *	the data that read from ADV7280
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint32 ADV7280_read_reg(kal_uint32 addr)
{
	return ADV7280_read_cmos_sensor(addr);
} /* OV7670_read_reg() */


/*************************************************************************
* FUNCTION
*	ADV7280_awb_enable
*
* DESCRIPTION
*	This function enable or disable the awb (Auto White Balance).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable awb, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static void ADV7280_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	//temp_AWB_reg = ADV7280_read_cmos_sensor(0x42);
	spin_lock(&ADV7280_drv_lock);
	temp_AWB_reg = ADV7280_SensorDriver.wb;
	spin_unlock(&ADV7280_drv_lock);
	
	SENSORDB("[ADV7280]ADV7280_awb_enable reg 0x42=%x:\n ", temp_AWB_reg);
	if (enalbe)
	{
		temp_AWB_reg = (temp_AWB_reg |0x02);
		ADV7280_write_cmos_sensor(0x42, temp_AWB_reg);
	}
	else
	{
		temp_AWB_reg = (temp_AWB_reg & (~0x02));
		ADV7280_write_cmos_sensor(0x42, temp_AWB_reg);
	}

	spin_lock(&ADV7280_drv_lock);
	ADV7280_SensorDriver.wb = temp_AWB_reg;
	spin_unlock(&ADV7280_drv_lock);

}
static void ADV7280_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	SENSORDB("[ADV7280]enter ADV7280_set_AE_mode function:\n ");

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC

    }
    else
    {
        // turn off AEC/AGC
    }
	SENSORDB("[ADV7280]exit ADV7280_set_AE_mode function:\n ");
}

/*************************************************************************
 * FUNCTION
 *	ADV7280_config_window
 *
 * DESCRIPTION
 *	This function config the hardware window of ADV7280 for getting specified
 *  data of that window.
 *
 * PARAMETERS
 *	start_x : start column of the interested window
 *  start_y : start row of the interested window
 *  width  : column widht of the itnerested window
 *  height : row depth of the itnerested window
 *
 * RETURNS
 *	the data that read from ADV7280
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_config_window(kal_uint16 startx, kal_uint16 starty, kal_uint16 width, kal_uint16 height)
{
} /* ADV7280_config_window */


/*************************************************************************
 * FUNCTION
 *	ADV7280_SetGain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
kal_uint16 ADV7280_SetGain(kal_uint16 iGain)
{
	return iGain;
}

/*************************************************************************
 * FUNCTION
 *	ADV7280_GAMMA_Select
 *
 * DESCRIPTION
 *	This function select gamma of ADV7280.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_GAMMA_Select(kal_uint32 GammaLvl)
{
	switch(GammaLvl)
	{
		case ADV7280_RGB_Gamma_m1:											  //smallest gamma curve
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x06);
			ADV7280_write_cmos_sensor(0xc0, 0x12);
			ADV7280_write_cmos_sensor(0xc1, 0x22);
			ADV7280_write_cmos_sensor(0xc2, 0x35);
			ADV7280_write_cmos_sensor(0xc3, 0x4b);
			ADV7280_write_cmos_sensor(0xc4, 0x5f);
			ADV7280_write_cmos_sensor(0xc5, 0x72);
			ADV7280_write_cmos_sensor(0xc6, 0x8d);
			ADV7280_write_cmos_sensor(0xc7, 0xa4);
			ADV7280_write_cmos_sensor(0xc8, 0xb8);
			ADV7280_write_cmos_sensor(0xc9, 0xc8);
			ADV7280_write_cmos_sensor(0xca, 0xd4);
			ADV7280_write_cmos_sensor(0xcb, 0xde);
			ADV7280_write_cmos_sensor(0xcc, 0xe6);
			ADV7280_write_cmos_sensor(0xcd, 0xf1);
			ADV7280_write_cmos_sensor(0xce, 0xf8);
			ADV7280_write_cmos_sensor(0xcf, 0xfd);
			break;
		case ADV7280_RGB_Gamma_m2:
			ADV7280_write_cmos_sensor(0xBF, 0x08);
			ADV7280_write_cmos_sensor(0xc0, 0x0F);
			ADV7280_write_cmos_sensor(0xc1, 0x21);
			ADV7280_write_cmos_sensor(0xc2, 0x32);
			ADV7280_write_cmos_sensor(0xc3, 0x43);
			ADV7280_write_cmos_sensor(0xc4, 0x50);
			ADV7280_write_cmos_sensor(0xc5, 0x5E);
			ADV7280_write_cmos_sensor(0xc6, 0x78);
			ADV7280_write_cmos_sensor(0xc7, 0x90);
			ADV7280_write_cmos_sensor(0xc8, 0xA6);
			ADV7280_write_cmos_sensor(0xc9, 0xB9);
			ADV7280_write_cmos_sensor(0xcA, 0xC9);
			ADV7280_write_cmos_sensor(0xcB, 0xD6);
			ADV7280_write_cmos_sensor(0xcC, 0xE0);
			ADV7280_write_cmos_sensor(0xcD, 0xEE);
			ADV7280_write_cmos_sensor(0xcE, 0xF8);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case ADV7280_RGB_Gamma_m3:			
			ADV7280_write_cmos_sensor(0xBF, 0x0B);
			ADV7280_write_cmos_sensor(0xc0, 0x16);
			ADV7280_write_cmos_sensor(0xc1, 0x29);
			ADV7280_write_cmos_sensor(0xc2, 0x3C);
			ADV7280_write_cmos_sensor(0xc3, 0x4F);
			ADV7280_write_cmos_sensor(0xc4, 0x5F);
			ADV7280_write_cmos_sensor(0xc5, 0x6F);
			ADV7280_write_cmos_sensor(0xc6, 0x8A);
			ADV7280_write_cmos_sensor(0xc7, 0x9F);
			ADV7280_write_cmos_sensor(0xc8, 0xB4);
			ADV7280_write_cmos_sensor(0xc9, 0xC6);
			ADV7280_write_cmos_sensor(0xcA, 0xD3);
			ADV7280_write_cmos_sensor(0xcB, 0xDD);
			ADV7280_write_cmos_sensor(0xcC, 0xE5);
			ADV7280_write_cmos_sensor(0xcD, 0xF1);
			ADV7280_write_cmos_sensor(0xcE, 0xFA);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case ADV7280_RGB_Gamma_m4:
			ADV7280_write_cmos_sensor(0xBF, 0x0E);
			ADV7280_write_cmos_sensor(0xc0, 0x1C);
			ADV7280_write_cmos_sensor(0xc1, 0x34);
			ADV7280_write_cmos_sensor(0xc2, 0x48);
			ADV7280_write_cmos_sensor(0xc3, 0x5A);
			ADV7280_write_cmos_sensor(0xc4, 0x6B);
			ADV7280_write_cmos_sensor(0xc5, 0x7B);
			ADV7280_write_cmos_sensor(0xc6, 0x95);
			ADV7280_write_cmos_sensor(0xc7, 0xAB);
			ADV7280_write_cmos_sensor(0xc8, 0xBF);
			ADV7280_write_cmos_sensor(0xc9, 0xCE);
			ADV7280_write_cmos_sensor(0xcA, 0xD9);
			ADV7280_write_cmos_sensor(0xcB, 0xE4);
			ADV7280_write_cmos_sensor(0xcC, 0xEC);
			ADV7280_write_cmos_sensor(0xcD, 0xF7);
			ADV7280_write_cmos_sensor(0xcE, 0xFD);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case ADV7280_RGB_Gamma_m5:
			ADV7280_write_cmos_sensor(0xBF, 0x10);
			ADV7280_write_cmos_sensor(0xc0, 0x20);
			ADV7280_write_cmos_sensor(0xc1, 0x38);
			ADV7280_write_cmos_sensor(0xc2, 0x4E);
			ADV7280_write_cmos_sensor(0xc3, 0x63);
			ADV7280_write_cmos_sensor(0xc4, 0x76);
			ADV7280_write_cmos_sensor(0xc5, 0x87);
			ADV7280_write_cmos_sensor(0xc6, 0xA2);
			ADV7280_write_cmos_sensor(0xc7, 0xB8);
			ADV7280_write_cmos_sensor(0xc8, 0xCA);
			ADV7280_write_cmos_sensor(0xc9, 0xD8);
			ADV7280_write_cmos_sensor(0xcA, 0xE3);
			ADV7280_write_cmos_sensor(0xcB, 0xEB);
			ADV7280_write_cmos_sensor(0xcC, 0xF0);
			ADV7280_write_cmos_sensor(0xcD, 0xF8);
			ADV7280_write_cmos_sensor(0xcE, 0xFD);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);
			break;
			
		case ADV7280_RGB_Gamma_m6:
			ADV7280_write_cmos_sensor(0xBF, 0x14);
			ADV7280_write_cmos_sensor(0xc0, 0x28);
			ADV7280_write_cmos_sensor(0xc1, 0x44);
			ADV7280_write_cmos_sensor(0xc2, 0x5D);
			ADV7280_write_cmos_sensor(0xc3, 0x72);
			ADV7280_write_cmos_sensor(0xc4, 0x86);
			ADV7280_write_cmos_sensor(0xc5, 0x95);
			ADV7280_write_cmos_sensor(0xc6, 0xB1);
			ADV7280_write_cmos_sensor(0xc7, 0xC6);
			ADV7280_write_cmos_sensor(0xc8, 0xD5);
			ADV7280_write_cmos_sensor(0xc9, 0xE1);
			ADV7280_write_cmos_sensor(0xcA, 0xEA);
			ADV7280_write_cmos_sensor(0xcB, 0xF1);
			ADV7280_write_cmos_sensor(0xcC, 0xF5);
			ADV7280_write_cmos_sensor(0xcD, 0xFB);
			ADV7280_write_cmos_sensor(0xcE, 0xFE);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);							// largest gamma curve
			break;
		case ADV7280_RGB_Gamma_night:									//Gamma for night mode
			ADV7280_write_cmos_sensor(0xBF, 0x0B);
			ADV7280_write_cmos_sensor(0xc0, 0x16);
			ADV7280_write_cmos_sensor(0xc1, 0x29);
			ADV7280_write_cmos_sensor(0xc2, 0x3C);
			ADV7280_write_cmos_sensor(0xc3, 0x4F);
			ADV7280_write_cmos_sensor(0xc4, 0x5F);
			ADV7280_write_cmos_sensor(0xc5, 0x6F);
			ADV7280_write_cmos_sensor(0xc6, 0x8A);
			ADV7280_write_cmos_sensor(0xc7, 0x9F);
			ADV7280_write_cmos_sensor(0xc8, 0xB4);
			ADV7280_write_cmos_sensor(0xc9, 0xC6);
			ADV7280_write_cmos_sensor(0xcA, 0xD3);
			ADV7280_write_cmos_sensor(0xcB, 0xDD);
			ADV7280_write_cmos_sensor(0xcC, 0xE5);
			ADV7280_write_cmos_sensor(0xcD, 0xF1);
			ADV7280_write_cmos_sensor(0xcE, 0xFA);
			ADV7280_write_cmos_sensor(0xcF, 0xFF);
			break;
		default:
			break;
	}
}

/*************************************************************************
 * FUNCTION
 *	ADV7280_NightMode
 *
 * DESCRIPTION
 *	This function night mode of ADV7280.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void ADV7280_night_mode(kal_bool bEnable)
{
	if (bEnable)
	{
		ADV7280_SET_PAGE1;
		ADV7280_write_cmos_sensor(0xfe, 0x01);
		ADV7280_write_cmos_sensor(0x11, 0xa1);
		//ADV7280_write_cmos_sensor(0x13, 0x68);//ae target
		if(ADV7280_MPEG4_encode_mode == KAL_TRUE) 
			{
			ADV7280_SET_PAGE0;
			ADV7280_write_cmos_sensor(0xfa, 0x10);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x33, 0x00);
		SENSORDB("ADV7280_night_mode mode VIDEO = %d \n", bEnable);
			}
		else
			{
			ADV7280_SET_PAGE0;
			ADV7280_write_cmos_sensor(0xfa, 0x00);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x33, 0x30);
		SENSORDB("ADV7280_night_mode mode preview = %d \n", bEnable);
			
			}
		ADV7280_write_cmos_sensor(0x21, 0xb0);
		ADV7280_write_cmos_sensor(0x22, 0x60);
		ADV7280_SET_PAGE0;
		ADV7280_write_cmos_sensor(0x40, 0xef);
		ADV7280_write_cmos_sensor(0x41, 0x74);
		//ADV7280_write_cmos_sensor(0x42, 0x7e);
		//ADV7280_write_cmos_sensor(0xd1, 0x40);//saturation Cb
		//ADV7280_write_cmos_sensor(0xd2, 0x40);//saturation Cr
		//ADV7280_write_cmos_sensor(0xd3, 0x4b);//contrast
		//ADV7280_write_cmos_sensor(0xd5, 0x2b);//luma offset ,modify in bright and EV
		ADV7280_write_cmos_sensor(0xde, 0x30);
	
		ADV7280_GAMMA_Select(ADV7280_RGB_Gamma_night);
	}
	else 
	{
		ADV7280_SET_PAGE1;
		ADV7280_write_cmos_sensor(0x11, 0xa1);
		//ADV7280_write_cmos_sensor(0x13, 0x40);//ae target
		if(ADV7280_MPEG4_encode_mode == KAL_TRUE)
			{
			ADV7280_SET_PAGE0;
			ADV7280_write_cmos_sensor(0xfa, 0x00);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x33, 0x00);
		SENSORDB("ADV7280_night_mode mode VIDEO = %d\n", bEnable);
			}
		else
			{
			ADV7280_SET_PAGE0;
			ADV7280_write_cmos_sensor(0xfa, 0x00);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x33, 0x20);
		SENSORDB("ADV7280_night_mode mode preview = %d\n", bEnable);
			
			}
		
		ADV7280_write_cmos_sensor(0x21, 0xb0);
		ADV7280_write_cmos_sensor(0x22, 0x48);
		ADV7280_SET_PAGE0;
		ADV7280_write_cmos_sensor(0x40, 0xff);
		ADV7280_write_cmos_sensor(0x41, 0x04);
		//ADV7280_write_cmos_sensor(0x42, 0x7e);
		//ADV7280_write_cmos_sensor(0xd1, 0x30);//saturation Cb
		//ADV7280_write_cmos_sensor(0xd2, 0x30);//saturation Cr
		//ADV7280_write_cmos_sensor(0xd3, 0x40);//contrast
		//ADV7280_write_cmos_sensor(0xd5, 0x00);//luma offset ,modify in bright and EV
		//ADV7280_write_cmos_sensor(0xde, 0x34);//2013.01.29
		ADV7280_GAMMA_Select(ADV7280_RGB_Gamma_m1);		
	}
	
	spin_lock(&ADV7280_drv_lock);
	//ADV7280_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
	ADV7280_night_mode_enable = bEnable;
	spin_unlock(&ADV7280_drv_lock);
	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_night_mode mode = %d", bEnable);
} /* ADV7280_NightMode */


/*************************************************************************
* FUNCTION
*	ADV7280_Sensor_Init
*
* DESCRIPTION
*	This function apply all of the initial setting to sensor.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
*************************************************************************/
void ADV7280_Sensor_Init(void)
{
	ADV7280_write_cmos_sensor(0xfe, 0x80);
	ADV7280_write_cmos_sensor(0xfc, 0x16); 
	ADV7280_write_cmos_sensor(0xfc, 0x16); 
	ADV7280_write_cmos_sensor(0xfe, 0x00);
        ADV7280_write_cmos_sensor(0x4f, 0x00); //AEC off 
	ADV7280_write_cmos_sensor(0x73, 0x90); 
	ADV7280_write_cmos_sensor(0x74, 0x80); 
	ADV7280_write_cmos_sensor(0x75, 0x80); 
	ADV7280_write_cmos_sensor(0x76, 0x94);

	ADV7280_write_cmos_sensor(0x09, 0x00); 
	ADV7280_write_cmos_sensor(0x0a, 0x02); 
	ADV7280_write_cmos_sensor(0x0b, 0x00); 
	ADV7280_write_cmos_sensor(0x0c, 0x02);
	ADV7280_write_cmos_sensor(0x03, 0x02); 
	ADV7280_write_cmos_sensor(0x04, 0x98);//3
	
			
	ADV7280_write_cmos_sensor(0x17, 0x14); 
	ADV7280_write_cmos_sensor(0x19, 0x05); 
	ADV7280_write_cmos_sensor(0x1b, 0x24); 
	ADV7280_write_cmos_sensor(0x1c, 0x04); 
	ADV7280_write_cmos_sensor(0x1e, 0x08); 
	ADV7280_write_cmos_sensor(0x1f, 0xc0);
	ADV7280_write_cmos_sensor(0x20, 0x00); 
	ADV7280_write_cmos_sensor(0x21, 0x48);
	ADV7280_write_cmos_sensor(0x22, 0xba);
	ADV7280_write_cmos_sensor(0x23, 0x22); 
	//ADV7280_write_cmos_sensor(0x24, 0x16); 
	//ADV7280_write_cmos_sensor(0x24, 0x17); 
	//ADV7280_write_cmos_sensor(0x24, 0x16); 
	ADV7280_write_cmos_sensor(0x24, 0x17); 

	////////////////////blk////////////////////
	ADV7280_write_cmos_sensor(0x26, 0xf7);//blk speed    97
	ADV7280_write_cmos_sensor(0x32, 0x04);
	ADV7280_write_cmos_sensor(0x33, 0x20);
	ADV7280_write_cmos_sensor(0x34, 0x20);
	ADV7280_write_cmos_sensor(0x35, 0x20);
	ADV7280_write_cmos_sensor(0x36, 0x20);
	////////////////////ISP BLOCK ENABLE////////////////////
	ADV7280_write_cmos_sensor(0x40, 0xff); 
	ADV7280_write_cmos_sensor(0x41, 0x04);
	ADV7280_write_cmos_sensor(0x42, 0x7e); 
	ADV7280_write_cmos_sensor(0x46, 0x02); 
	//ADV7280_write_cmos_sensor(0x46, 0x06); 
	ADV7280_write_cmos_sensor(0x4b, 0xcb);
	ADV7280_write_cmos_sensor(0x4d, 0x01); 
	ADV7280_write_cmos_sensor(0x4f, 0x00);
	ADV7280_write_cmos_sensor(0x70, 0x48);
	
	////////////////////DNDD////////////////////
	ADV7280_write_cmos_sensor(0x80, 0xe7); 
	ADV7280_write_cmos_sensor(0x82, 0x55); 
	ADV7280_write_cmos_sensor(0x87, 0x4a); 
	////////////////////ASDE////////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x01);
	ADV7280_write_cmos_sensor(0x18, 0x22); 
	ADV7280_write_cmos_sensor(0xfe, 0x00);
	ADV7280_write_cmos_sensor(0x9c, 0x0a); 
	ADV7280_write_cmos_sensor(0xa4, 0x60); 
	ADV7280_write_cmos_sensor(0xa5, 0x21); 
	ADV7280_write_cmos_sensor(0xa7, 0x35); 
	ADV7280_write_cmos_sensor(0xdd, 0x54); 
	ADV7280_write_cmos_sensor(0x95, 0x35); 
	////////////////////RGB gamma////////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x00);
	ADV7280_write_cmos_sensor(0xbf, 0x06);
	ADV7280_write_cmos_sensor(0xc0, 0x12);
	ADV7280_write_cmos_sensor(0xc1, 0x22);
	ADV7280_write_cmos_sensor(0xc2, 0x35);
	ADV7280_write_cmos_sensor(0xc3, 0x4b);
	ADV7280_write_cmos_sensor(0xc4, 0x5f);
	ADV7280_write_cmos_sensor(0xc5, 0x72);
	ADV7280_write_cmos_sensor(0xc6, 0x8d);
	ADV7280_write_cmos_sensor(0xc7, 0xa4);
	ADV7280_write_cmos_sensor(0xc8, 0xb8);
	ADV7280_write_cmos_sensor(0xc9, 0xc8);
	ADV7280_write_cmos_sensor(0xca, 0xd4);
	ADV7280_write_cmos_sensor(0xcb, 0xde);
	ADV7280_write_cmos_sensor(0xcc, 0xe6);
	ADV7280_write_cmos_sensor(0xcd, 0xf1);
	ADV7280_write_cmos_sensor(0xce, 0xf8);
	ADV7280_write_cmos_sensor(0xcf, 0xfd);
	//////////////////CC///////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x00);
	ADV7280_write_cmos_sensor(0xb3, 0x44);
	ADV7280_write_cmos_sensor(0xb4, 0xfd);
	ADV7280_write_cmos_sensor(0xb5, 0x02);
	ADV7280_write_cmos_sensor(0xb6, 0xfa);
	ADV7280_write_cmos_sensor(0xb7, 0x48);
	ADV7280_write_cmos_sensor(0xb8, 0xf0);
	//skin
	//ADV7280_write_cmos_sensor(0xb3, 0x3c);
	//ADV7280_write_cmos_sensor(0xb4, 0xFF);
	//ADV7280_write_cmos_sensor(0xb5, 0x03);
	//ADV7280_write_cmos_sensor(0xb6, 0x01);
	//ADV7280_write_cmos_sensor(0xb7, 0x3f);
	//ADV7280_write_cmos_sensor(0xb8, 0xF3);
	// crop 
	ADV7280_write_cmos_sensor(0x50, 0x01);
	ADV7280_write_cmos_sensor(0x19, 0x05);
	ADV7280_write_cmos_sensor(0x20, 0x01);
	ADV7280_write_cmos_sensor(0x22, 0xba);
	ADV7280_write_cmos_sensor(0x21, 0x48);
	////////////////////YCP////////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x00);
	ADV7280_write_cmos_sensor(0xd1, 0x30); 
	ADV7280_write_cmos_sensor(0xd2, 0x30);	
	ADV7280_write_cmos_sensor(0xde, 0x34);//2013.01.29
	////////////////////AEC////////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x01);
	ADV7280_write_cmos_sensor(0x10, 0x40);
	ADV7280_write_cmos_sensor(0x11, 0xa1);
	ADV7280_write_cmos_sensor(0x12, 0x03); //06
	ADV7280_write_cmos_sensor(0x13, 0x60); //50 40
	ADV7280_write_cmos_sensor(0x17, 0x88);
	ADV7280_write_cmos_sensor(0x1a, 0x21);
	ADV7280_write_cmos_sensor(0x21, 0xb0);
	ADV7280_write_cmos_sensor(0x22, 0x48);
	ADV7280_write_cmos_sensor(0x3c, 0x30);//95
	ADV7280_write_cmos_sensor(0x3d, 0x70);//50
	ADV7280_write_cmos_sensor(0x3e, 0x30);//90
	////////////////////AWB////////////////////
	ADV7280_write_cmos_sensor(0xfe, 0x01);
	ADV7280_write_cmos_sensor(0x06, 0x16);
	ADV7280_write_cmos_sensor(0x07, 0x06);
	ADV7280_write_cmos_sensor(0x08, 0x98);
	ADV7280_write_cmos_sensor(0x09, 0xee);
	ADV7280_write_cmos_sensor(0x50, 0xfc);
	ADV7280_write_cmos_sensor(0x51, 0x28);
	ADV7280_write_cmos_sensor(0x52, 0x10); //0b
	ADV7280_write_cmos_sensor(0x53, 0x10);
	ADV7280_write_cmos_sensor(0x54, 0x10);
	ADV7280_write_cmos_sensor(0x55, 0x10); //10
	ADV7280_write_cmos_sensor(0x56, 0x20); //20	//ADV7280_write_cmos_sensor(0x57, 0x40); 
	ADV7280_write_cmos_sensor(0x58, 0x60);
	ADV7280_write_cmos_sensor(0x59, 0x08);  // 28
	ADV7280_write_cmos_sensor(0x5a, 0x02);
	ADV7280_write_cmos_sensor(0x5b, 0x63);
	ADV7280_write_cmos_sensor(0x5c, 0x35);//3
	ADV7280_write_cmos_sensor(0x5d, 0x73);
	ADV7280_write_cmos_sensor(0x5e, 0x11);
	ADV7280_write_cmos_sensor(0x5f, 0x40);
	ADV7280_write_cmos_sensor(0x60, 0x40);
	ADV7280_write_cmos_sensor(0x61, 0xc8);
	ADV7280_write_cmos_sensor(0x62, 0xa0);
	ADV7280_write_cmos_sensor(0x63, 0x40);
	ADV7280_write_cmos_sensor(0x64, 0x50);
	ADV7280_write_cmos_sensor(0x65, 0x98);
	ADV7280_write_cmos_sensor(0x66, 0xfa);
	ADV7280_write_cmos_sensor(0x67, 0x70);
	ADV7280_write_cmos_sensor(0x68, 0x58);
	ADV7280_write_cmos_sensor(0x69, 0x85);
	ADV7280_write_cmos_sensor(0x6a, 0x40);
	ADV7280_write_cmos_sensor(0x6b, 0x39);
	ADV7280_write_cmos_sensor(0x6c, 0x20);//20
	ADV7280_write_cmos_sensor(0x6d, 0x40);//30
	ADV7280_write_cmos_sensor(0x6e, 0x00);
	ADV7280_write_cmos_sensor(0x70, 0x02);
	ADV7280_write_cmos_sensor(0x71, 0x00);
	ADV7280_write_cmos_sensor(0x72, 0x10);
	ADV7280_write_cmos_sensor(0x73, 0x40);
	
	ADV7280_write_cmos_sensor(0x80, 0x58);
	ADV7280_write_cmos_sensor(0x81, 0x50);
	ADV7280_write_cmos_sensor(0x82, 0x44);
	ADV7280_write_cmos_sensor(0x83, 0x40);
	ADV7280_write_cmos_sensor(0x84, 0x40);
	ADV7280_write_cmos_sensor(0x85, 0x40);
	
	ADV7280_write_cmos_sensor(0x74, 0x40);
	ADV7280_write_cmos_sensor(0x75, 0x58);
	ADV7280_write_cmos_sensor(0x76, 0x24);
	ADV7280_write_cmos_sensor(0x77, 0x40);
	ADV7280_write_cmos_sensor(0x78, 0x20);
	ADV7280_write_cmos_sensor(0x79, 0x60);
	ADV7280_write_cmos_sensor(0x7a, 0x58);
	ADV7280_write_cmos_sensor(0x7b, 0x20);
	ADV7280_write_cmos_sensor(0x7c, 0x30);
	ADV7280_write_cmos_sensor(0x7d, 0x35);
	ADV7280_write_cmos_sensor(0x7e, 0x10);
	ADV7280_write_cmos_sensor(0x7f, 0x08);
	///////////////////ABS///////////////////////
	ADV7280_write_cmos_sensor(0x9c, 0x02);
	ADV7280_write_cmos_sensor(0x9d, 0x30);
	////////////////////CC-AWB////////////////////
	ADV7280_write_cmos_sensor(0xd0, 0x00);
	ADV7280_write_cmos_sensor(0xd2, 0x2c); 
	ADV7280_write_cmos_sensor(0xd3, 0x80);
	///////////////////LSC //////////////////////
	//// for xuye062d lens setting
	ADV7280_write_cmos_sensor(0xfe, 0x01);
	ADV7280_write_cmos_sensor(0xa0, 0x00);
	ADV7280_write_cmos_sensor(0xa1, 0x3c);
	ADV7280_write_cmos_sensor(0xa2, 0x50);
	ADV7280_write_cmos_sensor(0xa3, 0x00);
	ADV7280_write_cmos_sensor(0xa8, 0x0f);
	ADV7280_write_cmos_sensor(0xa9, 0x08);
	ADV7280_write_cmos_sensor(0xaa, 0x00);
	ADV7280_write_cmos_sensor(0xab, 0x04);
	ADV7280_write_cmos_sensor(0xac, 0x00);
	ADV7280_write_cmos_sensor(0xad, 0x07);
	ADV7280_write_cmos_sensor(0xae, 0x0e);
	ADV7280_write_cmos_sensor(0xaf, 0x00);
	ADV7280_write_cmos_sensor(0xb0, 0x00);
	ADV7280_write_cmos_sensor(0xb1, 0x09);
	ADV7280_write_cmos_sensor(0xb2, 0x00);
	ADV7280_write_cmos_sensor(0xb3, 0x00);
	ADV7280_write_cmos_sensor(0xb4, 0x35);
	ADV7280_write_cmos_sensor(0xb5, 0x28);
	ADV7280_write_cmos_sensor(0xb6, 0x24);
	ADV7280_write_cmos_sensor(0xba, 0x3c);
	ADV7280_write_cmos_sensor(0xbb, 0x2f);
	ADV7280_write_cmos_sensor(0xbc, 0x2c);
	ADV7280_write_cmos_sensor(0xc0, 0x1b);
	ADV7280_write_cmos_sensor(0xc1, 0x16);
	ADV7280_write_cmos_sensor(0xc2, 0x15);
	ADV7280_write_cmos_sensor(0xc6, 0x21);
	ADV7280_write_cmos_sensor(0xc7, 0x1c);
	ADV7280_write_cmos_sensor(0xc8, 0x1b);
	ADV7280_write_cmos_sensor(0xb7, 0x00);
	ADV7280_write_cmos_sensor(0xb8, 0x00);
	ADV7280_write_cmos_sensor(0xb9, 0x00);
	ADV7280_write_cmos_sensor(0xbd, 0x00);
	ADV7280_write_cmos_sensor(0xbe, 0x00);
	ADV7280_write_cmos_sensor(0xbf, 0x00);
	ADV7280_write_cmos_sensor(0xc3, 0x00);
	ADV7280_write_cmos_sensor(0xc4, 0x00);
	ADV7280_write_cmos_sensor(0xc5, 0x00);
	ADV7280_write_cmos_sensor(0xc9, 0x00);
	ADV7280_write_cmos_sensor(0xca, 0x00);
	ADV7280_write_cmos_sensor(0xcb, 0x00);
	ADV7280_write_cmos_sensor(0xa4, 0x00);
	ADV7280_write_cmos_sensor(0xa5, 0x00);
	ADV7280_write_cmos_sensor(0xa6, 0x00);
	ADV7280_write_cmos_sensor(0xa7, 0x00);
	ADV7280_write_cmos_sensor(0xfe, 0x00);
	////////////////////asde ///////////////////
	ADV7280_write_cmos_sensor(0xa0, 0xaf);
	ADV7280_write_cmos_sensor(0xa2, 0xff);
	ADV7280_write_cmos_sensor(0xa4, 0x60);//2013.1.29
	ADV7280_write_cmos_sensor(0x44, 0xa0);
	//ADV7280_write_cmos_sensor(0x44, 0xa2);
	ADV7280_write_cmos_sensor(0x4f, 0x01);//add
	
	Sleep(300);
	//ADV7280_write_cmos_sensor(0x4f, 0x01);//add
	ADV7280_write_cmos_sensor(0xf0, 0x07); 
	ADV7280_write_cmos_sensor(0xf1, 0x01); 

}


/*************************************************************************
* FUNCTION
*	GC329_Lens_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate lens parameter.
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
void ADV7280_Lens_Select(kal_uint8 Lens_Tag)
{
	switch(Lens_Tag)
	{
		case CHT_806C_2:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x04);
			
			ADV7280_write_cmos_sensor(0xa8, 0x0f);
			ADV7280_write_cmos_sensor(0xa9, 0x08);
			ADV7280_write_cmos_sensor(0xaa, 0x00);
			ADV7280_write_cmos_sensor(0xab, 0x04);
			ADV7280_write_cmos_sensor(0xac, 0x00);
			ADV7280_write_cmos_sensor(0xad, 0x07);
			ADV7280_write_cmos_sensor(0xae, 0x0e);
			ADV7280_write_cmos_sensor(0xaf, 0x00);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x09);
			ADV7280_write_cmos_sensor(0xb2, 0x00);
			ADV7280_write_cmos_sensor(0xb3, 0x00);

			ADV7280_write_cmos_sensor(0xb4, 0x30);
			ADV7280_write_cmos_sensor(0xb5, 0x19);
			ADV7280_write_cmos_sensor(0xb6, 0x21);
			ADV7280_write_cmos_sensor(0xba, 0x3e);
			ADV7280_write_cmos_sensor(0xbb, 0x26);
			ADV7280_write_cmos_sensor(0xbc, 0x2f);
			ADV7280_write_cmos_sensor(0xc0, 0x15);
			ADV7280_write_cmos_sensor(0xc1, 0x11);
			ADV7280_write_cmos_sensor(0xc2, 0x15);
			ADV7280_write_cmos_sensor(0xc6, 0x1f);
			ADV7280_write_cmos_sensor(0xc7, 0x16);
			ADV7280_write_cmos_sensor(0xc8, 0x16);

			ADV7280_write_cmos_sensor(0xb7, 0x00);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x00);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x00);
			ADV7280_write_cmos_sensor(0xc9, 0x0d);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);
			
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case CHT_808C_2:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x02);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x0c);
			ADV7280_write_cmos_sensor(0xa9, 0x03);
			ADV7280_write_cmos_sensor(0xaa, 0x00);
			ADV7280_write_cmos_sensor(0xab, 0x05);
			ADV7280_write_cmos_sensor(0xac, 0x01);
			ADV7280_write_cmos_sensor(0xad, 0x07);
			ADV7280_write_cmos_sensor(0xae, 0x0e);
			ADV7280_write_cmos_sensor(0xaf, 0x00);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x08);
			ADV7280_write_cmos_sensor(0xb2, 0x02);
			ADV7280_write_cmos_sensor(0xb3, 0x00);

			ADV7280_write_cmos_sensor(0xb4, 0x30);
			ADV7280_write_cmos_sensor(0xb5, 0x0f);
			ADV7280_write_cmos_sensor(0xb6, 0x16);
			ADV7280_write_cmos_sensor(0xba, 0x44);
			ADV7280_write_cmos_sensor(0xbb, 0x24);
			ADV7280_write_cmos_sensor(0xbc, 0x2a);
			ADV7280_write_cmos_sensor(0xc0, 0x13);
			ADV7280_write_cmos_sensor(0xc1, 0x0e);
			ADV7280_write_cmos_sensor(0xc2, 0x11);
			ADV7280_write_cmos_sensor(0xc6, 0x28);
			ADV7280_write_cmos_sensor(0xc7, 0x21);
			ADV7280_write_cmos_sensor(0xc8, 0x20);

			ADV7280_write_cmos_sensor(0xb7, 0x00);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x01);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x00);
			ADV7280_write_cmos_sensor(0xc9, 0x00);


			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
			
		case LY_982A_H114:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x0c);
			ADV7280_write_cmos_sensor(0xa9, 0x06);
			ADV7280_write_cmos_sensor(0xaa, 0x02);
			ADV7280_write_cmos_sensor(0xab, 0x13);
			ADV7280_write_cmos_sensor(0xac, 0x06);
			ADV7280_write_cmos_sensor(0xad, 0x05);
			ADV7280_write_cmos_sensor(0xae, 0x0b);
			ADV7280_write_cmos_sensor(0xaf, 0x03);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x08);
			ADV7280_write_cmos_sensor(0xb2, 0x01);
			ADV7280_write_cmos_sensor(0xb3, 0x00);

			ADV7280_write_cmos_sensor(0xb4, 0x34);
			ADV7280_write_cmos_sensor(0xb5, 0x29);
			ADV7280_write_cmos_sensor(0xb6, 0x2e);
			ADV7280_write_cmos_sensor(0xba, 0x30);
			ADV7280_write_cmos_sensor(0xbb, 0x24);
			ADV7280_write_cmos_sensor(0xbc, 0x28);
			ADV7280_write_cmos_sensor(0xc0, 0x1c);
			ADV7280_write_cmos_sensor(0xc1, 0x19);
			ADV7280_write_cmos_sensor(0xc2, 0x19);
			ADV7280_write_cmos_sensor(0xc6, 0x1a);
			ADV7280_write_cmos_sensor(0xc7, 0x19);
			ADV7280_write_cmos_sensor(0xc8, 0x1b);

			ADV7280_write_cmos_sensor(0xb7, 0x01);
			ADV7280_write_cmos_sensor(0xb8, 0x01);
			ADV7280_write_cmos_sensor(0xb9, 0x00);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x03);
			ADV7280_write_cmos_sensor(0xc9, 0x00);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_046A:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x10);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x11);
			ADV7280_write_cmos_sensor(0xa9, 0x0a);
			ADV7280_write_cmos_sensor(0xaa, 0x05);
			ADV7280_write_cmos_sensor(0xab, 0x04);
			ADV7280_write_cmos_sensor(0xac, 0x03);
			ADV7280_write_cmos_sensor(0xad, 0x00);
			ADV7280_write_cmos_sensor(0xae, 0x08);
			ADV7280_write_cmos_sensor(0xaf, 0x01);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x09);
			ADV7280_write_cmos_sensor(0xb2, 0x02);
			ADV7280_write_cmos_sensor(0xb3, 0x03);

			ADV7280_write_cmos_sensor(0xb4, 0x2e);
			ADV7280_write_cmos_sensor(0xb5, 0x16);
			ADV7280_write_cmos_sensor(0xb6, 0x24);
			ADV7280_write_cmos_sensor(0xba, 0x3a);
			ADV7280_write_cmos_sensor(0xbb, 0x1e);
			ADV7280_write_cmos_sensor(0xbc, 0x24);
			ADV7280_write_cmos_sensor(0xc0, 0x09);
			ADV7280_write_cmos_sensor(0xc1, 0x02);
			ADV7280_write_cmos_sensor(0xc2, 0x06);
			ADV7280_write_cmos_sensor(0xc6, 0x25);
			ADV7280_write_cmos_sensor(0xc7, 0x21);
			ADV7280_write_cmos_sensor(0xc8, 0x23);

			ADV7280_write_cmos_sensor(0xb7, 0x00);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x0f);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x00);
			ADV7280_write_cmos_sensor(0xc9, 0x00);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_0620:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x0f);
			ADV7280_write_cmos_sensor(0xa9, 0x06);
			ADV7280_write_cmos_sensor(0xaa, 0x00);
			ADV7280_write_cmos_sensor(0xab, 0x07);
			ADV7280_write_cmos_sensor(0xac, 0x05);
			ADV7280_write_cmos_sensor(0xad, 0x08);
			ADV7280_write_cmos_sensor(0xae, 0x13);
			ADV7280_write_cmos_sensor(0xaf, 0x06);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x06);
			ADV7280_write_cmos_sensor(0xb2, 0x01);
			ADV7280_write_cmos_sensor(0xb3, 0x04);

			ADV7280_write_cmos_sensor(0xb4, 0x2d);
			ADV7280_write_cmos_sensor(0xb5, 0x18);
			ADV7280_write_cmos_sensor(0xb6, 0x22);
			ADV7280_write_cmos_sensor(0xba, 0x45);
			ADV7280_write_cmos_sensor(0xbb, 0x2d);
			ADV7280_write_cmos_sensor(0xbc, 0x34);
			ADV7280_write_cmos_sensor(0xc0, 0x16);
			ADV7280_write_cmos_sensor(0xc1, 0x13);
			ADV7280_write_cmos_sensor(0xc2, 0x19);

			ADV7280_write_cmos_sensor(0xc6, 0x21);
			ADV7280_write_cmos_sensor(0xc7, 0x1c);
			ADV7280_write_cmos_sensor(0xc8, 0x18);

			ADV7280_write_cmos_sensor(0xb7, 0x00);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x00);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x08);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x01);
			ADV7280_write_cmos_sensor(0xc9, 0x00);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x10);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case XY_078V: 
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x14);
			ADV7280_write_cmos_sensor(0xa9, 0x08);
			ADV7280_write_cmos_sensor(0xaa, 0x0a);
			ADV7280_write_cmos_sensor(0xab, 0x11);
			ADV7280_write_cmos_sensor(0xac, 0x05);
			ADV7280_write_cmos_sensor(0xad, 0x07);
			ADV7280_write_cmos_sensor(0xae, 0x0b);
			ADV7280_write_cmos_sensor(0xaf, 0x03);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x09);
			ADV7280_write_cmos_sensor(0xb2, 0x04);
			ADV7280_write_cmos_sensor(0xb3, 0x01);

			ADV7280_write_cmos_sensor(0xb4, 0x2f);
			ADV7280_write_cmos_sensor(0xb5, 0x2a);
			ADV7280_write_cmos_sensor(0xb6, 0x2c);
			ADV7280_write_cmos_sensor(0xba, 0x3a);
			ADV7280_write_cmos_sensor(0xbb, 0x2b);
			ADV7280_write_cmos_sensor(0xbc, 0x32);
			ADV7280_write_cmos_sensor(0xc0, 0x1b);
			ADV7280_write_cmos_sensor(0xc1, 0x18);
			ADV7280_write_cmos_sensor(0xc2, 0x1a);
			ADV7280_write_cmos_sensor(0xc6, 0x12);
			ADV7280_write_cmos_sensor(0xc7, 0x10);
			ADV7280_write_cmos_sensor(0xc8, 0x12);

			ADV7280_write_cmos_sensor(0xb7, 0x0a);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x00);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x00);
			ADV7280_write_cmos_sensor(0xc9, 0x0d);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;

		case YG1001A_F:
			ADV7280_write_cmos_sensor(0xfe, 0x01);
			ADV7280_write_cmos_sensor(0xa0, 0x00);
			ADV7280_write_cmos_sensor(0xa1, 0x3c);
			ADV7280_write_cmos_sensor(0xa2, 0x50);
			ADV7280_write_cmos_sensor(0xa3, 0x00);
			ADV7280_write_cmos_sensor(0xa4, 0x00);
			ADV7280_write_cmos_sensor(0xa5, 0x00);
			ADV7280_write_cmos_sensor(0xa6, 0x00);
			ADV7280_write_cmos_sensor(0xa7, 0x00);

			ADV7280_write_cmos_sensor(0xa8, 0x0e);
			ADV7280_write_cmos_sensor(0xa9, 0x05);
			ADV7280_write_cmos_sensor(0xaa, 0x01);
			ADV7280_write_cmos_sensor(0xab, 0x07);
			ADV7280_write_cmos_sensor(0xac, 0x00);
			ADV7280_write_cmos_sensor(0xad, 0x07);
			ADV7280_write_cmos_sensor(0xae, 0x0e);
			ADV7280_write_cmos_sensor(0xaf, 0x02);
			ADV7280_write_cmos_sensor(0xb0, 0x00);
			ADV7280_write_cmos_sensor(0xb1, 0x0d);
			ADV7280_write_cmos_sensor(0xb2, 0x00);
			ADV7280_write_cmos_sensor(0xb3, 0x00);

			ADV7280_write_cmos_sensor(0xb4, 0x2a);
			ADV7280_write_cmos_sensor(0xb5, 0x0f);
			ADV7280_write_cmos_sensor(0xb6, 0x14);
			ADV7280_write_cmos_sensor(0xba, 0x40);
			ADV7280_write_cmos_sensor(0xbb, 0x26);
			ADV7280_write_cmos_sensor(0xbc, 0x2a);
			ADV7280_write_cmos_sensor(0xc0, 0x0e);
			ADV7280_write_cmos_sensor(0xc1, 0x0a);
			ADV7280_write_cmos_sensor(0xc2, 0x0d);
			ADV7280_write_cmos_sensor(0xc6, 0x27);
			ADV7280_write_cmos_sensor(0xc7, 0x20);
			ADV7280_write_cmos_sensor(0xc8, 0x1f);

			ADV7280_write_cmos_sensor(0xb7, 0x00);
			ADV7280_write_cmos_sensor(0xb8, 0x00);
			ADV7280_write_cmos_sensor(0xb9, 0x00);
			ADV7280_write_cmos_sensor(0xbd, 0x00);
			ADV7280_write_cmos_sensor(0xbe, 0x00);
			ADV7280_write_cmos_sensor(0xbf, 0x00);
			ADV7280_write_cmos_sensor(0xc3, 0x00);
			ADV7280_write_cmos_sensor(0xc4, 0x00);
			ADV7280_write_cmos_sensor(0xc5, 0x00);
			ADV7280_write_cmos_sensor(0xc9, 0x00);
			ADV7280_write_cmos_sensor(0xca, 0x00);
			ADV7280_write_cmos_sensor(0xcb, 0x00);

			ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_write_cmos_sensor(0x4f, 0x01);//aec enable
			break;

		default:
			break;
	}
}


extern int se4500_detect(void);

static int get_se4500_id(void)
{
    int ret;
    int sensor_id;

    ret = se4500_detect();
    if (ret < 0) {
        SENSORDB("se4500 not exist.\n");
        sensor_id = 0x0bad;
    } else {
        // ret == 0  ==> se4500 moudle is not ready
        // ret >  0  ==> se4500 detected
        sensor_id = 0x4500;
    }

    return sensor_id;
}

/*************************************************************************
* FUNCTION
*	ADV7280_GAMMA_Select
*
* DESCRIPTION
*	This function is served for FAE to select the appropriate GAMMA curve.
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
UINT32 ADV7280GetSensorID(UINT32 *sensorID)
{
    kal_uint16 sensor_id=0;
    int i;
	
	SENSORDB("[ADV7280]CONTROLFLOW ADV7280GetSensorID\n");

    /*
    ADV7280_write_cmos_sensor(0xfc, 0x16);
    Sleep(20);

    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = ADV7280_read_cmos_sensor(0x00);
	            	printk("ADV7280 Sensor id = %x\n", sensor_id);
                    sensor_id = 0x4500;
	            	if (sensor_id == ADV7280_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);
    */

    sensor_id = get_se4500_id();

    printk("[==CAMERA YCD==] sensor_id = 0x%x", sensor_id);
    if(sensor_id != ADV7280_SENSOR_ID)
    {
        SENSORDB("ADV7280 Sensor id read failed, ID = %x\n", sensor_id);
		//sensor_id = ADV7280_SENSOR_ID;
		*sensorID = 0xffffffff;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    *sensorID = sensor_id;

    RETAILMSG(1, (TEXT("Sensor Read ID OK \n")));
	
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	ADV7280_Write_More_Registers
*
* DESCRIPTION
*	This function is served for FAE to modify the necessary Init Regs. Do not modify the regs
*     in init_ADV7280() directly.
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
void ADV7280_Write_More_Registers(void)
{
    //ADV7280_GAMMA_Select(0);//0:use default
    //ADV7280_Lens_Select(0);//0:use default
}


/*************************************************************************
 * FUNCTION
 *	ADV7280Open
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
UINT32 ADV7280Open(void)
{
    kal_uint16 sensor_id=0;
    int i;

    kdCheckGreySensor = 1;

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280Open\n");

    ADV7280_write_cmos_sensor(0xfc, 0x16);
    Sleep(20);

    /*
    do
    {
        	// check if sensor ID correct
        	for(i = 0; i < 3; i++)
		{
	            	sensor_id = ADV7280_read_cmos_sensor(0x00);
	            	printk("ADV7280 Sensor id = %x\n", sensor_id);
                    sensor_id = 0x4500; 
	            	if (sensor_id == ADV7280_SENSOR_ID)
			{
	               	break;
	            	}
        	}
        	mdelay(50);
    }while(0);
    */

    sensor_id = get_se4500_id();

    if(sensor_id != ADV7280_SENSOR_ID)
    {
        SENSORDB("ADV7280 Sensor id read failed, ID = %x\n", sensor_id);
		//sensor_id = ADV7280_SENSOR_ID;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    RETAILMSG(1, (TEXT("Sensor Read ID OK \n")));
    // initail sequence write in
    ADV7280_Sensor_Init();
    ADV7280_Write_More_Registers();
	
	spin_lock(&ADV7280_drv_lock);
	//ADV7280_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
	ADV7280_night_mode_enable = KAL_FALSE;
	ADV7280_MPEG4_encode_mode = KAL_FALSE;
	ADV7280_SensorDriver.wb = 0x7e;
	spin_unlock(&ADV7280_drv_lock);
    return ERROR_NONE;
} /* ADV7280Open */

/*************************************************************************
 * FUNCTION
 *	ADV7280Close
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
UINT32 ADV7280Close(void)
{
    kdCheckGreySensor = 0;

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280Close\n");

    return ERROR_NONE;
} /* ADV7280Close */

extern int se4500_s_stream(int streaming);
/*************************************************************************
 * FUNCTION
 * ADV7280Preview
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
UINT32 ADV7280Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{
    kal_uint32 iTemp;
    kal_uint16 iStartX = 0, iStartY = 1;
    
    se4500_s_stream(1);

    if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)		// MPEG4 Encode Mode
    {
		SENSORDB("[ADV7280]CONTROLFLOW ADV7280Preview Video\n");
		    spin_lock(&ADV7280_drv_lock);
        ADV7280_MPEG4_encode_mode = KAL_TRUE;
    spin_unlock(&ADV7280_drv_lock);
       
    }
    else
    {
		SENSORDB("[ADV7280]CONTROLFLOW ADV7280Preview camera\n");
		    spin_lock(&ADV7280_drv_lock);
        ADV7280_MPEG4_encode_mode = KAL_FALSE;
    spin_unlock(&ADV7280_drv_lock);
    }
    

    image_window->GrabStartX= IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY= IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth = IMAGE_SENSOR_PV_WIDTH;
    image_window->ExposureWindowHeight =IMAGE_SENSOR_PV_HEIGHT;

    // copy sensor_config_data
    memcpy(&ADV7280SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	ADV7280_night_mode(ADV7280_night_mode_enable);
    return ERROR_NONE;
} /* ADV7280Preview */


/*************************************************************************
 * FUNCTION
 *	ADV7280Capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
UINT32 ADV7280Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
        MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{

		    spin_lock(&ADV7280_drv_lock);
    ADV7280_MODE_CAPTURE=KAL_TRUE;
        ADV7280_MPEG4_encode_mode = KAL_FALSE;
    spin_unlock(&ADV7280_drv_lock);
	
	SENSORDB("[ADV7280]CONTROLFLOW ADV7280Capture\n");

    image_window->GrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
    image_window->GrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
    image_window->ExposureWindowWidth= IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = IMAGE_SENSOR_FULL_HEIGHT;


    // copy sensor_config_data
    memcpy(&ADV7280SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	//ADV7280_night_mode(ADV7280_night_mode_enable);
    return ERROR_NONE;
} /* ADV7280_Capture() */



UINT32 ADV7280GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth=IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight=IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight=IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorVideoWidth=IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorVideoHeight=IMAGE_SENSOR_PV_HEIGHT;
    return ERROR_NONE;
} /* ADV7280GetResolution() */


UINT32 ADV7280GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
        MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    pSensorInfo->SensorPreviewResolutionX=IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY=IMAGE_SENSOR_PV_HEIGHT;
    pSensorInfo->SensorFullResolutionX=IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY=IMAGE_SENSOR_FULL_WIDTH;

    pSensorInfo->SensorCameraPreviewFrameRate=30;
    pSensorInfo->SensorVideoFrameRate=30;
    pSensorInfo->SensorStillCaptureFrameRate=10;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=1;
    pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 0;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 1; // 12
    pSensorInfo->VideoDelayFrame = 1;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    default:
        pSensorInfo->SensorClockFreq=26;
        pSensorInfo->SensorClockDividCount= 3;
        pSensorInfo->SensorClockRisingCount=0;
        pSensorInfo->SensorClockFallingCount=2;
        pSensorInfo->SensorPixelClockCount=0;
        pSensorInfo->SensorDataLatchCount=0;
        pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_VGA_GRAB_PIXELS;
        pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_VGA_GRAB_LINES;
        break;
    }
    ADV7280PixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &ADV7280SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
} /* ADV7280GetInfo() */


UINT32 ADV7280Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
        MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280Control ScenarioId = %d\n", ScenarioId); 

	spin_lock(&ADV7280_drv_lock);
	ADV7280_CurrentScenarioId = ScenarioId;
	spin_unlock(&ADV7280_drv_lock);

    switch (ScenarioId)
    {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
        ADV7280Preview(pImageWindow, pSensorConfigData);
        break;
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
	case MSDK_SCENARIO_ID_CAMERA_ZSD:
   // case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
        ADV7280Capture(pImageWindow, pSensorConfigData);
        break;
    }


    return ERROR_NONE;
}	/* ADV7280Control() */

BOOL ADV7280_set_param_wb(UINT16 para)
{

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_param_wb para = %d\n", para); 

	switch (para)
	{
		case AWB_MODE_OFF:
		
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x57);
			ADV7280_write_cmos_sensor(0x78, 0x4d);
			ADV7280_write_cmos_sensor(0x79, 0x45);
			
		break;
		
		case AWB_MODE_AUTO:
		default:
			ADV7280_write_cmos_sensor(0x77, 0x57);
			ADV7280_write_cmos_sensor(0x78, 0x4d);
			ADV7280_write_cmos_sensor(0x79, 0x45);
			ADV7280_awb_enable(KAL_TRUE);
		break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy,D75
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x7a); //8c  //WB_manual_gain 
			ADV7280_write_cmos_sensor(0x78, 0x5a); //50
			ADV7280_write_cmos_sensor(0x79, 0x40);
		break;
		
		case AWB_MODE_DAYLIGHT: //sunny,D65
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x70); //55//74 6f
			ADV7280_write_cmos_sensor(0x78, 0x58); //44//52
			ADV7280_write_cmos_sensor(0x79, 0x40); //48			
		break;
		
		case AWB_MODE_INCANDESCENT: //office,TL84
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x40);//48
			ADV7280_write_cmos_sensor(0x78, 0x43);//40
			ADV7280_write_cmos_sensor(0x79, 0x45);//5c
		break;
		
		case AWB_MODE_TUNGSTEN: //home,A
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x40);//40
			ADV7280_write_cmos_sensor(0x78, 0x51);//54
			ADV7280_write_cmos_sensor(0x79, 0x65);//70
		break;
		
		case AWB_MODE_FLUORESCENT://CWF
			ADV7280_awb_enable(KAL_FALSE);
			ADV7280_write_cmos_sensor(0x77, 0x46);//40
			ADV7280_write_cmos_sensor(0x78, 0x40);//42
			ADV7280_write_cmos_sensor(0x79, 0x45);//50
		break;
		
		//default:
		//return FALSE;
	}

	return TRUE;
} /* ADV7280_set_param_wb */


BOOL ADV7280_set_param_effect(UINT16 para)
{
	kal_uint32  ret = KAL_TRUE;

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_param_effect para = %d\n", para); 

	switch (para)
	{
		case MEFFECT_OFF:
		default:
			ADV7280_write_cmos_sensor(0x43 , 0x00);
		break;

		case MEFFECT_SEPIA:
			ADV7280_write_cmos_sensor(0x43 , 0x02);
			ADV7280_write_cmos_sensor(0xda , 0xd0);
			ADV7280_write_cmos_sensor(0xdb , 0x28);
		break;
		
		case MEFFECT_NEGATIVE:
			ADV7280_write_cmos_sensor(0x43 , 0x01);
		break;
		
		case MEFFECT_SEPIAGREEN:
			ADV7280_write_cmos_sensor(0x43 , 0x02);
			ADV7280_write_cmos_sensor(0xda , 0xc0);
			ADV7280_write_cmos_sensor(0xdb , 0xc0);
		break;
		
		case MEFFECT_SEPIABLUE:
			ADV7280_write_cmos_sensor(0x43 , 0x02);
			ADV7280_write_cmos_sensor(0xda , 0x50);
			ADV7280_write_cmos_sensor(0xdb , 0xe0);
		break;

		case MEFFECT_MONO:
			ADV7280_write_cmos_sensor(0x43 , 0x02);
			ADV7280_write_cmos_sensor(0xda , 0x00);
			ADV7280_write_cmos_sensor(0xdb , 0x00);
		break;
		//default:
		//	ret = FALSE;
	}

	return ret;

} /* ADV7280_set_param_effect */


BOOL ADV7280_set_param_banding(UINT16 para)
{

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_param_banding para = %d\n", para); 

	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			ADV7280_write_cmos_sensor(0x05, 0x00); 	
			ADV7280_write_cmos_sensor(0x06, 0x59);
			ADV7280_write_cmos_sensor(0x07, 0x00);
			ADV7280_write_cmos_sensor(0x08, 0x15);
			
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			ADV7280_write_cmos_sensor(0x2a, 0xa6);   //anti-flicker step [7:0]
				
			ADV7280_write_cmos_sensor(0x2b, 0x01);   //exp level 0  30.00fps
			ADV7280_write_cmos_sensor(0x2c, 0xf2); 
			ADV7280_write_cmos_sensor(0x2d, 0x04);   //exp level 1  14.29fps
			ADV7280_write_cmos_sensor(0x2e, 0x8a); 
			ADV7280_write_cmos_sensor(0x2f, 0x06);   //exp level 2  10.00fps
			ADV7280_write_cmos_sensor(0x30, 0x7c); 
			ADV7280_write_cmos_sensor(0x31, 0x0a);   //exp level 3  5.26fps
			ADV7280_write_cmos_sensor(0x32, 0x60); 
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_SET_PAGE0;

			ADV7280_CAM_BANDING_50HZ = KAL_TRUE;
			break;

		case AE_FLICKER_MODE_60HZ:
			ADV7280_write_cmos_sensor(0x05, 0x00);
			ADV7280_write_cmos_sensor(0x06, 0x5b);//30
			ADV7280_write_cmos_sensor(0x07, 0x00);
			ADV7280_write_cmos_sensor(0x08, 0x21);

			
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			ADV7280_write_cmos_sensor(0x2a, 0x8a);   //anti-flicker step [7:0]
				
			ADV7280_write_cmos_sensor(0x2b, 0x01);   //exp level 0  30.00fps
			ADV7280_write_cmos_sensor(0x2c, 0x9e); 
			ADV7280_write_cmos_sensor(0x2d, 0x04);   //exp level 0  15.00fps
			ADV7280_write_cmos_sensor(0x2e, 0x50); 
			ADV7280_write_cmos_sensor(0x2f, 0x06);   //exp level 0  10.00fps
			ADV7280_write_cmos_sensor(0x30, 0x78); 
			ADV7280_write_cmos_sensor(0x31, 0x09);   //exp level 0  5.00fps
			ADV7280_write_cmos_sensor(0x32, 0xb4); 
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_SET_PAGE0;

			ADV7280_CAM_BANDING_50HZ = KAL_FALSE;
		break;
		case AE_FLICKER_MODE_AUTO:
		case AE_FLICKER_MODE_OFF:
		default:
			ADV7280_write_cmos_sensor(0x05, 0x00); 	
			ADV7280_write_cmos_sensor(0x06, 0x59);
			ADV7280_write_cmos_sensor(0x07, 0x00);
			ADV7280_write_cmos_sensor(0x08, 0x15);
			
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x29, 0x00);   //anti-flicker step [11:8]
			ADV7280_write_cmos_sensor(0x2a, 0xa6);   //anti-flicker step [7:0]
				
			ADV7280_write_cmos_sensor(0x2b, 0x01);   //exp level 0  30.00fps
			ADV7280_write_cmos_sensor(0x2c, 0xf2); 
			ADV7280_write_cmos_sensor(0x2d, 0x04);   //exp level 1  14.29fps
			ADV7280_write_cmos_sensor(0x2e, 0x8a); 
			ADV7280_write_cmos_sensor(0x2f, 0x06);   //exp level 2  10.00fps
			ADV7280_write_cmos_sensor(0x30, 0x7c); 
			ADV7280_write_cmos_sensor(0x31, 0x0a);   //exp level 3  5.26fps
			ADV7280_write_cmos_sensor(0x32, 0x60); 
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			ADV7280_SET_PAGE0;
			ADV7280_CAM_BANDING_50HZ = KAL_TRUE;
			break;
		//return FALSE;
	}
	
    //	Sleep(500);
	return TRUE;
} /* ADV7280_set_param_banding */

BOOL ADV7280_set_param_exposure_for_HDR(UINT16 para)
{
	
	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_param_exposure_for_HDR\n ");

	return TRUE;
}

BOOL ADV7280_set_param_exposure(UINT16 para)
{

	 SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_param_exposure function:\n ");
	 SENSORDB("[ADV7280]para=%d:\n",para);
	 //spin_lock(&ADV7280_drv_lock);
/*	if (SCENE_MODE_HDR == ADV7280Sensor.sceneMode && 
	 SENSOR_MODE_CAPTURE == ADV7280Sensor.SensorMode)
	{
		//spin_unlock(&ADV7280_drv_lock);
		ADV7280_set_param_exposure_for_HDR(para);
		return TRUE;
	}*/


	switch (para)
	{
		case AE_EV_COMP_n20:
			ADV7280_write_cmos_sensor(0xd5, 0x90);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x30);
			ADV7280_SET_PAGE0;
		break;
		case AE_EV_COMP_n13:
			ADV7280_write_cmos_sensor(0xd5, 0xc0);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x30);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_n10:
			ADV7280_write_cmos_sensor(0xd5, 0xd0);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x38);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_n07:
			ADV7280_write_cmos_sensor(0xd5, 0xe0);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x40);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_n03:
			ADV7280_write_cmos_sensor(0xd5, 0xf0);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x48);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_00:
		default:
			ADV7280_write_cmos_sensor(0xd5, 0x00);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x60);
			ADV7280_SET_PAGE0;
		break;

		case AE_EV_COMP_03:
			ADV7280_write_cmos_sensor(0xd5, 0x10);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x60);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_07:
			ADV7280_write_cmos_sensor(0xd5, 0x20);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x70);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_10:
			ADV7280_write_cmos_sensor(0xd5, 0x30);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x80);
			ADV7280_SET_PAGE0;
		break;
		
		case AE_EV_COMP_13:
			ADV7280_write_cmos_sensor(0xd5, 0x40);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x90);
			ADV7280_SET_PAGE0;
		break;
		case AE_EV_COMP_20:
			ADV7280_write_cmos_sensor(0xd5, 0x70);
		//	ADV7280_SET_PAGE1;
		//	ADV7280_write_cmos_sensor(0x13, 0x90);
			ADV7280_SET_PAGE0;
		break;
		//default:
		//return FALSE;
	}

	return TRUE;
} /* ADV7280_set_param_exposure */
///add
void ADV7280_set_contrast(UINT16 para)
{   
    SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_contrast function:\n ");
    switch (para)
    {
        case ISP_CONTRAST_LOW:			 
			ADV7280_write_cmos_sensor(0xfe, 0x00);	 
			ADV7280_write_cmos_sensor(0xd3, 0x30);
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
			break;
        case ISP_CONTRAST_HIGH:			 
			ADV7280_write_cmos_sensor(0xfe, 0x00); 	
			ADV7280_write_cmos_sensor(0xd3, 0x60);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case ISP_CONTRAST_MIDDLE: 
        default:
			ADV7280_write_cmos_sensor(0xfe, 0x00);	 
			ADV7280_write_cmos_sensor(0xd3, 0x40);
			ADV7280_write_cmos_sensor(0xfe, 0x00);	 
			break;
        //default:
		//	break;
    }
    SENSORDB("[ADV7280]exit ADV7280_set_contrast function:\n ");
    return;
}

void ADV7280_set_brightness(UINT16 para)
{
    SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_brightness function:\n ");
	//return;
    switch (para)
    {
        case ISP_BRIGHT_LOW:
		//case AE_EV_COMP_n13:
		//	ADV7280_write_cmos_sensor(0xd5, 0xc0);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x13, 0x30);
			ADV7280_SET_PAGE0;
		break;
        case ISP_BRIGHT_HIGH:
		//case AE_EV_COMP_13:
		//	ADV7280_write_cmos_sensor(0xd5, 0x40);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x13, 0x90);
			ADV7280_SET_PAGE0;
			break;
        case ISP_BRIGHT_MIDDLE:
        default:
		//case AE_EV_COMP_00:
		//	ADV7280_write_cmos_sensor(0xd5, 0x00);
			ADV7280_SET_PAGE1;
			ADV7280_write_cmos_sensor(0x13, 0x60);
			ADV7280_SET_PAGE0;
		break;
		//	return KAL_FALSE;
		//	break;
    }
    SENSORDB("[ADV7280]exit ADV7280_set_brightness function:\n ");
    return;
}
void ADV7280_set_saturation(UINT16 para)
{
	SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_saturation function:\n ");
    switch (para)
    {
        case ISP_SAT_HIGH:
			ADV7280_write_cmos_sensor(0xfe, 0x00); 	
			ADV7280_write_cmos_sensor(0xd0, 0x60);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case ISP_SAT_LOW:
			ADV7280_write_cmos_sensor(0xfe, 0x00); 	
			ADV7280_write_cmos_sensor(0xd0, 0x30);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case ISP_SAT_MIDDLE:
        default:
			ADV7280_write_cmos_sensor(0xfe, 0x00); 	
			ADV7280_write_cmos_sensor(0xd0, 0x40);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
		//	return KAL_FALSE;
		//	break;
    }
	SENSORDB("[ADV7280]exit ADV7280_set_saturation function:\n ");
     return;
}
void ADV7280_set_scene_mode(UINT16 para)
{
	SENSORDB("[ADV7280]CONTROLFLOW enter ADV7280_set_scene_mode function:\n ");
	SENSORDB("[ADV7280] ADV7280_set_scene_mode=%d",para);	
	spin_lock(&ADV7280_drv_lock);
	ADV7280_SensorDriver.sceneMode=para;
	spin_unlock(&ADV7280_drv_lock);
    switch (para)
    { 

		case SCENE_MODE_NIGHTSCENE:
          	ADV7280_night_mode(KAL_TRUE); 
			break;
        case SCENE_MODE_PORTRAIT:
			/*Night mode disable*/
          	ADV7280_night_mode(KAL_FALSE); 
			ADV7280_write_cmos_sensor(0xfe, 0x01); 	
			ADV7280_write_cmos_sensor(0x67, 0x70);
			ADV7280_write_cmos_sensor(0x68, 0x58);
			ADV7280_write_cmos_sensor(0x69, 0x70);
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
            break;
        case SCENE_MODE_LANDSCAPE:
			/*Night mode disable*/
          	ADV7280_night_mode(KAL_FALSE); 
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
			ADV7280_write_cmos_sensor(0xd0, 0x48);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
	 
             break;
        case SCENE_MODE_SUNSET:
			/*Night mode disable*/
          	ADV7280_night_mode(KAL_FALSE); 
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
			ADV7280_write_cmos_sensor(0xd1, 0x50);
			ADV7280_write_cmos_sensor(0xd2, 0x40);	
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
	 
            break;
        case SCENE_MODE_SPORTS:
             /*Night mode disable*/
          	ADV7280_night_mode(KAL_FALSE); 
			ADV7280_write_cmos_sensor(0xfe, 0x01); 	
			ADV7280_write_cmos_sensor(0x33, 0x10);
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
	 
            break;
        case SCENE_MODE_BEACH:
        case SCENE_MODE_SNOW:
          	ADV7280_night_mode(KAL_FALSE); 			
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
			ADV7280_write_cmos_sensor(0xd1, 0x46);
			ADV7280_write_cmos_sensor(0xd2, 0x40);	
			ADV7280_write_cmos_sensor(0xfe, 0x00); 
            break;
        case SCENE_MODE_HDR:
        //    if (1 == ADV7280_SensorDriver.manualAEStart)
            {
                ADV7280_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&ADV7280_drv_lock);
            	//ADV7280_SensorDriver.manualAEStart = 0;
                //ADV7280_SensorDriver.currentExposureTime = 0;
                //ADV7280_SensorDriver.currentAxDGain = 0;
				spin_unlock(&ADV7280_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
        default:
			 /*Night mode disable*/
	   /*	 ADV7280_write_cmos_sensor(0xfe, 0x01);	 
			 ADV7280_write_cmos_sensor(0x67, 0x70);
			 ADV7280_write_cmos_sensor(0x68, 0x58);
			 ADV7280_write_cmos_sensor(0x69, 0x85);
			 ADV7280_write_cmos_sensor(0xfe, 0x00); 
			 */
/*ADV7280_write_cmos_sensor(0xfe, 0x01); 	
ADV7280_write_cmos_sensor(0x67, 0x70);
ADV7280_write_cmos_sensor(0x68, 0x58);
ADV7280_write_cmos_sensor(0x69, 0x85);
ADV7280_write_cmos_sensor(0xfe, 0x00); */
          	ADV7280_night_mode(KAL_FALSE); 
			break;
		//	return KAL_FALSE;
        //    break;
    }
	SENSORDB("[ADV7280]exit ADV7280_set_scene_mode function:\n ");
	//if( ADV7280_SensorDriver.sceneMode != SCENE_MODE_NIGHTSCENE) && ( ADV7280_SensorDriver.sceneMode != SCENE_MODE_NIGHTSCENE)
	return;
}
void ADV7280_set_iso(UINT16 para)
{
    spin_lock(&ADV7280_drv_lock);
    ADV7280_SensorDriver.isoSpeed = para;
    spin_unlock(&ADV7280_drv_lock);   

	SENSORDB("[ADV7280]CONTROLFLOW ADV7280_set_iso:\n ");

    switch (para)
	{
        case AE_ISO_100:
             //ISO 100
			ADV7280_write_cmos_sensor(0xfe, 0x01); 	
			ADV7280_write_cmos_sensor(0x21, 0x40);
			ADV7280_write_cmos_sensor(0x22, 0x40);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case AE_ISO_200:
             //ISO 200
			ADV7280_write_cmos_sensor(0xfe, 0x01); 	
			ADV7280_write_cmos_sensor(0x21, 0x60);
			ADV7280_write_cmos_sensor(0x22, 0x40);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
        case AE_ISO_400:
             //ISO 400
			ADV7280_write_cmos_sensor(0xfe, 0x01); 	
			ADV7280_write_cmos_sensor(0x21, 0xC0);
			ADV7280_write_cmos_sensor(0x22, 0x40);
			ADV7280_write_cmos_sensor(0xfe, 0x00);
			break;
		case AE_ISO_AUTO:
			default:
/*ADV7280_write_cmos_sensor(0xfe, 0x01); 
		ADV7280_write_cmos_sensor(0x21, 0xb0);
		ADV7280_write_cmos_sensor(0x22, 0x60);
ADV7280_write_cmos_sensor(0xfe, 0x00);*/
			if (ADV7280_night_mode_enable)
			{
				ADV7280_write_cmos_sensor(0xfe, 0x01);
				ADV7280_write_cmos_sensor(0x21, 0xb0);
				ADV7280_write_cmos_sensor(0x22, 0x60);
				ADV7280_write_cmos_sensor(0xfe, 0x00);
			}
			else 
			{
				ADV7280_write_cmos_sensor(0xfe, 0x01);		
				ADV7280_write_cmos_sensor(0x21, 0xb0);
				ADV7280_write_cmos_sensor(0x22, 0x48);
				ADV7280_write_cmos_sensor(0xfe, 0x00);
		    }
			break;
	}
    return;
}

UINT32 ADV7280YUVSetVideoMode(UINT16 u2FrameRate)    // lanking add
{
  
		    spin_lock(&ADV7280_drv_lock);
        ADV7280_MPEG4_encode_mode = KAL_TRUE;
    spin_unlock(&ADV7280_drv_lock);
	SENSORDB("[ADV7280]CONTROLFLOW ADV7280 Frame Rate= %d\n", u2FrameRate); 
     if (u2FrameRate == 30)
   	{
   	    /*********video frame ************/
		
   	}
    else if (u2FrameRate == 15)       
    	{
    	
   	    /*********video frame ************/
		
    	}
    else
   	{
   	
            SENSORDB("ADV7280 wrong Frame Rate= %d\n", u2FrameRate); 
			
   	}

	  ADV7280_night_mode(ADV7280_night_mode_enable);

      return TRUE;

}


UINT32 ADV7280YUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
 /*   switch (iCmd) {
    case FID_AWB_MODE:
        ADV7280_set_param_wb(iPara);
        break;
    case FID_COLOR_EFFECT:
        ADV7280_set_param_effect(iPara);
        break;
    case FID_AE_EV:
        ADV7280_set_param_exposure(iPara);
        break;
    case FID_AE_FLICKER:
        ADV7280_set_param_banding(iPara);
        break;
	case FID_SCENE_MODE:
		ADV7280_night_mode(iPara);
		break;
		
    default:
        break;
    }*/
	
		switch (iCmd) {
			case FID_SCENE_MODE:
				ADV7280_set_scene_mode(iPara);
				break;		
			case FID_AWB_MODE:
					ADV7280_set_param_wb(iPara);
				  break;
			case FID_COLOR_EFFECT:				
					ADV7280_set_param_effect(iPara);
				  break;
			case FID_AE_EV:   
					ADV7280_set_param_exposure(iPara);
				break;
			case FID_AE_FLICKER:					
					ADV7280_set_param_banding(iPara);
				  break;
			case FID_AE_SCENE_MODE: 
				if (iPara == AE_MODE_OFF) 
				{
					spin_lock(&ADV7280_drv_lock);
					ADV7280_SensorDriver.AE_ENABLE = KAL_FALSE; 
					spin_unlock(&ADV7280_drv_lock);
				}
				else 
				{
					spin_lock(&ADV7280_drv_lock);
					ADV7280_SensorDriver.AE_ENABLE = KAL_TRUE; 
					spin_unlock(&ADV7280_drv_lock);
				}
				ADV7280_set_AE_mode(ADV7280_SensorDriver.AE_ENABLE);
			break; 
			case FID_ISP_CONTRAST:
				ADV7280_set_contrast(iPara);
				break;
			case FID_ISP_BRIGHT:
				ADV7280_set_brightness(iPara);
				break;
			case FID_ISP_SAT:
				ADV7280_set_saturation(iPara);
			break; 
		case FID_ZOOM_FACTOR:
				SENSORDB("FID_ZOOM_FACTOR:%d\n", iPara);		
				spin_lock(&ADV7280_drv_lock);
				ADV7280_zoom_factor = iPara; 
				spin_unlock(&ADV7280_drv_lock);
				break; 
			case FID_AE_ISO:
				ADV7280_set_iso(iPara);
				break;
#if 0 //afc
			case FID_AF_MODE:
				 ADV7280_set_param_afmode(iPara);
						break;	   
#endif            
		  default:
					  break;
		}

    return TRUE;
} /* ADV7280YUVSensorSetting */
void ADV7280GetExifInfo(UINT32 exifAddr)
{
	SENSORDB("[ADV7280]enter ADV7280GetExifInfo function\n");
	SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
	pExifInfo->FNumber = 20;
	pExifInfo->AEISOSpeed = ADV7280_SensorDriver.isoSpeed;
	pExifInfo->FlashLightTimeus = 0;
	pExifInfo->RealISOValue = ADV7280_SensorDriver.isoSpeed;
	SENSORDB("[ADV7280]exit ADV7280GetExifInfo function\n");
}

static void ADV7280SetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
/*	SENSORDB("[ADV7280]enter ADV7280SetDummy function:\n ");
	if (ADV7280_SensorDriver.IsPVmode)	
	{
		dummy_pixels = dummy_pixels+ADV7280_PV_PERIOD_PIXEL_NUMS; 
		ADV7280_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));		   
		ADV7280_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
  
		dummy_lines= dummy_lines+ADV7280_PV_PERIOD_LINE_NUMS; 
		ADV7280_write_cmos_sensor(0x380F,(dummy_lines&0xFF));	   
		ADV7280_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
	} 
	else
	{
		dummy_pixels = dummy_pixels+ADV7280_FULL_PERIOD_PIXEL_NUMS; 
		ADV7280_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));		   
		ADV7280_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
  
		dummy_lines= dummy_lines+ADV7280_FULL_PERIOD_LINE_NUMS; 
		ADV7280_write_cmos_sensor(0x380F,(dummy_lines&0xFF));	   
		ADV7280_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
	} 
	SENSORDB("[ADV7280]exit ADV7280SetDummy function:\n ");*/
}	 /* ADV7280_set_dummy */


/**************************/
static void ADV7280GetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;	
}

static void ADV7280GetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{

	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=100;//ADV7280ReadShutter();
	Info->SensorAECur.AeCurGain=10;//ADV7280ReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=10;//((ADV7280YUV_read_cmos_sensor(0x3401)&&0xff)+((ADV7280YUV_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=10;//((ADV7280YUV_read_cmos_sensor(0x3405)&&0xff)+((ADV7280YUV_read_cmos_sensor(0x3404)&&0xff)*256));

}
UINT32 ADV7280MaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		SENSORDB("ADV7280MaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		SENSORDB("[ADV7280]enter ADV7280MaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 26000000;
				lineLength = VGA_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - VGA_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&ADV7280_drv_lock);
				//ADV7280_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				ADV7280_dummy_lines = dummyLine;
				spin_unlock(&ADV7280_drv_lock);
				ADV7280SetDummy(ADV7280_dummy_lines, ADV7280_dummy_pixels);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 26000000;
				lineLength = VGA_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - VGA_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&ADV7280_drv_lock);
				//ADV7280_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				ADV7280_dummy_lines = dummyLine;
				spin_unlock(&ADV7280_drv_lock);
				ADV7280SetDummy(ADV7280_dummy_lines, ADV7280_dummy_pixels);		
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:	
				pclk = 26000000;
				lineLength = VGA_PERIOD_PIXEL_NUMS;
				frameHeight = ( pclk)/frameRate/lineLength;
				dummyLine = frameHeight - VGA_PERIOD_LINE_NUMS;
				if(dummyLine<0)
					dummyLine = 0;
				
				spin_lock(&ADV7280_drv_lock);
				//ADV7280_SensorDriver.SensorMode= SENSOR_MODE_PREVIEW;
				ADV7280_dummy_lines = dummyLine;
				spin_unlock(&ADV7280_drv_lock);
				ADV7280SetDummy(ADV7280_dummy_lines, ADV7280_dummy_pixels);				
				break;		
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
				break;		
			default:
				break;
		}	
		SENSORDB("[ADV7280]exit ADV7280MaxFramerateByScenario function:\n ");
		return ERROR_NONE;
	}
UINT32 ADV7280GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	SENSORDB("[ADV7280]enter ADV7280GetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 300;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}
	SENSORDB("[ADV7280]exit ADV7280GetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void ADV7280_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	SENSORDB("[ADV7280]enter ADV7280_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	SENSORDB("[ADV7280]ADV7280_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	SENSORDB("[ADV7280]exit ADV7280_get_AEAWB_lock function:\n ");
}
void ADV7280_GetDelayInfo(UINT32 delayAddr)
{
	SENSORDB("[ADV7280]enter ADV7280_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	SENSORDB("[ADV7280]exit ADV7280_GetDelayInfo function:\n ");
}
void ADV7280_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
	SENSORDB("[ADV7280]enter ADV7280_AutoTestCmd function:\n ");
	switch(*cmd)
	{
		case YUV_AUTOTEST_SET_SHADDING:
			SENSORDB("YUV_AUTOTEST_SET_SHADDING:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAMMA:
			SENSORDB("YUV_AUTOTEST_SET_GAMMA:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_AE:
			SENSORDB("YUV_AUTOTEST_SET_AE:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_SHUTTER:
			SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAIN:
			SENSORDB("YUV_AUTOTEST_SET_GAIN:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			//*para=8228;
			break;
		default:
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	SENSORDB("[ADV7280]exit ADV7280_AutoTestCmd function:\n ");
}
UINT32 ADV7280SetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("[ADV7280_ADV7280SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	
	ADV7280_write_cmos_sensor(0xfe, 0x00);	
	ADV7280_write_cmos_sensor(0x4c, 0x01);
	ADV7280_write_cmos_sensor(0xfe, 0x00);

	if(bEnable)
	{
		//ADV7280_write_cmos_sensor(0x503d,0x80);
	}
	else
	{
		//ADV7280_write_cmos_sensor(0x503d,0x00);
	}
	return ERROR_NONE;
}

void ADV7280_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	SENSORDB("[ADV7280]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&ADV7280_drv_lock);
          ADV7280_SensorDriver.userAskAeLock = TRUE;
          spin_unlock(&ADV7280_drv_lock);
      //    ADV7280_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&ADV7280_drv_lock);
          ADV7280_SensorDriver.userAskAeLock = FALSE;
          spin_unlock(&ADV7280_drv_lock);
      //    ADV7280_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&ADV7280_drv_lock);
          ADV7280_SensorDriver.userAskAwbLock = TRUE;
          spin_unlock(&ADV7280_drv_lock);
      //    ADV7280_awb_enable(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&ADV7280_drv_lock);
          ADV7280_SensorDriver.userAskAwbLock = FALSE;
          spin_unlock(&ADV7280_drv_lock);
      //    ADV7280_awb_enable(KAL_TRUE);
      break;
      default:
      	break;
   }
   SENSORDB("[ADV7280]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}

UINT32 ADV7280FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	//SENSORDB("[ADV7280]CONTROLFLOW [ADV7280FeatureControl]feature id=%d \n",FeatureId);
	SENSORDB("[ADV7280] [ADV7280FeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=VGA_PERIOD_PIXEL_NUMS;
			*pFeatureReturnPara16=VGA_PERIOD_LINE_NUMS;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(ADV7280_CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=VGA_PERIOD_PIXEL_NUMS + ADV7280_dummy_pixels;
					*pFeatureReturnPara16=VGA_PERIOD_LINE_NUMS + ADV7280_dummy_lines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=VGA_PERIOD_PIXEL_NUMS + ADV7280_dummy_pixels;
					*pFeatureReturnPara16=VGA_PERIOD_LINE_NUMS + ADV7280_dummy_lines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(ADV7280_CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = ADV7280_g_fPV_PCLK * 1000 *1000;  //unit: Hz 			
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = ADV7280_g_fPV_PCLK * 1000 *1000;  //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			ADV7280GetExifInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			ADV7280_night_mode((BOOL) *pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_GAIN:
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			ADV7280_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = ADV7280_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &ADV7280SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:			 
			ADV7280SetTestPatternMode((BOOL)*pFeatureData16);			
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			ADV7280GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=ADV7280_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			ADV7280YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
			ADV7280_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			ADV7280YUVSetVideoMode(*pFeatureData16);
			break; 
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			ADV7280GetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			ADV7280GetCurAeAwbInfo(*pFeatureData32); 		
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			//ADV7280MaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			ADV7280GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			ADV7280_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			ADV7280_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_AUTOTEST_CMD:
			SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			ADV7280_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;

		//AF
		case SENSOR_FEATURE_INITIALIZE_AF:			 
			 break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
		//	  ADV7280_FOCUS_Move_to(*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_AF_STATUS:
		//	  ADV7280_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_AF_INF:
		//	  ADV7280_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
			*pFeatureParaLen=4; 		   
			break;
		case SENSOR_FEATURE_GET_AF_MACRO:
		//	  ADV7280_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
			*pFeatureParaLen=4; 		   
			break;
		case SENSOR_FEATURE_CONSTANT_AF:
		//	ADV7280_FOCUS_OVT_AFC_Constant_Focus();
			 break;
		case SENSOR_FEATURE_SET_AF_WINDOW:		 
			//ADV7280_FOCUS_Set_AF_Window(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			//ADV7280_FOCUS_OVT_AFC_Single_Focus();
			break;	
		case SENSOR_FEATURE_CANCEL_AF:
			//ADV7280_FOCUS_OVT_AFC_Cancel_Focus();
			break;					
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			//ADV7280_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);			
			*pFeatureParaLen=4;
			break;		  
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			//ADV7280_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32); 		   
			*pFeatureParaLen=4;
			break;		  
		case SENSOR_FEATURE_SET_AE_WINDOW:
			SENSORDB("AE zone addr = 0x%x\n",*pFeatureData32);			
			//ADV7280_FOCUS_Set_AE_Window(*pFeatureData32);
			break; 
		default:
			SENSORDB("ADV7280FeatureControl:default \n");
			break;			
	}
	SENSORDB("[ADV7280]exit ADV7280FeatureControl function:\n ");
	return ERROR_NONE;
}	/* ADV7280MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncADV7280YUV=
{
	ADV7280Open,
	ADV7280GetInfo,
	ADV7280GetResolution,
	ADV7280FeatureControl,
	ADV7280Control,
	ADV7280Close
};


UINT32 ADV7280_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncADV7280YUV;
	return ERROR_NONE;
} /* SensorInit() */



