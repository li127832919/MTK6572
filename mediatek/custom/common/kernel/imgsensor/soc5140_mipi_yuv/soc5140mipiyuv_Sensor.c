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
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 01 04 2012 hao.wang
 * [ALPS00109603] getsensorid func check in
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
#include <linux/xlog.h>

//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "soc5140mipiyuv_Sensor.h"
#include "soc5140mipiyuv_Camera_Sensor_para.h"
#include "soc5140mipiyuv_CameraCustomized.h"

#define SOC5140YUV_DEBUG
#ifdef SOC5140YUV_DEBUG
#define SENSORDB(fmt, arg...) xlog_printk(ANDROID_LOG_INFO ,"[SOC5140YUV]", fmt, ##arg)
#else
#define SENSORDB(fmt, arg...)
#endif

#define SOC5640_TEST_PATTERN_CHECKSUM (0x7ba87eae)
#define WINMO_USE 0
#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)

static int awbMode = AWB_MODE_AUTO;
kal_bool soc5140_pattern= KAL_FALSE;
static MSDK_SCENARIO_ID_ENUM soc5140_currentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
struct SOC5140_sensor_struct SOC5140_Sensor_Driver;
MSDK_SENSOR_CONFIG_STRUCT SOC5140SensorConfigData;

static int write_statue = SOC5140_YUV_MANUALREF;
static int zsd_setting = 0;
static DEFINE_SPINLOCK(soc5140_drv_lock);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 SOC5140_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 4,SOC5140_WRITE_ID);
    return ERROR_NONE;
}

kal_uint16 SOC5140_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,SOC5140_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

kal_uint16 SOC5140_read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,SOC5140_WRITE_ID);
    return get_byte;
}

void SOC5140_write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	iWriteRegI2C(puSendCmd , 3,SOC5140_WRITE_ID);
}

void SOC5140_8405_Polling(kal_uint32 para)
{
   kal_uint16  temp_data = 0xff;
   kal_uint16  count = 0;

	SENSORDB("****** kevinSOC5140_8405_Polling \n");
	
    SOC5140_write_cmos_sensor(0x098E, 0x8405);
   while(count < 40)
   {   
      temp_data = SOC5140_read_cmos_sensor_8(0x8405);

     	SENSORDB("  kevin temp_data = %d\n", temp_data);
          
      if(temp_data == para)
      {
       	SENSORDB("  kevin 8405 ok  \n");

           
           break;
      }
      count ++;
      if(count > 39)
      {
       	SENSORDB("  kevin8405  error \n");

      }
      mdelay(10);
      
  }

}

int SOC5140_8404_Polling_Sub(kal_uint32 wate_t, kal_uint32 para)
{
   kal_uint16  temp_data = 0xff;
   kal_uint16  count = 0;
   kal_uint16  r8404_status = SOC5140_8404_W_OK;

   SOC5140_write_cmos_sensor(0x098E, 0x8404);
   SENSORDB("read register before write \n");
   while(count < wate_t)
   {  
      temp_data = SOC5140_read_cmos_sensor_8(0x8404);
	  SENSORDB("  temp_data1 = %d\n", temp_data);
      if(temp_data == 0)
      {
        SOC5140_write_cmos_sensor_8(0x8404, para);
        break;
      }
      count ++;
      if(count > wate_t - 1)
      {
       	r8404_status = SOC5140_8404_BEFOR_W_ERR;
      }
      mdelay(10);
   }
   
   if(SOC5140_8404_AFTER_W_ERR == r8404_status)
   {
   	  SENSORDB("1 SOC5140_8404_AFTER_W_ERR\n");
   }
   else if(SOC5140_8404_BEFOR_W_ERR == r8404_status)
   {
   	  SENSORDB("1 SOC5140_8404_BEFOR_W_ERR\n");
   }

    SENSORDB("register write \n");
    count = 0;
    while(count < wate_t)
    {   
       temp_data = SOC5140_read_cmos_sensor_8(0x8404);
   	    SENSORDB("  temp_data2 = %d\n", temp_data);
       if(temp_data == 0)
       {
            break;
       }
       count ++;
       if(count > wate_t - 1)
       {
        	r8404_status = SOC5140_8404_AFTER_W_ERR;
       }
       mdelay(10);
    }

    if(SOC5140_8404_AFTER_W_ERR == r8404_status)
    {
    	  SENSORDB("2 SOC5140_8404_AFTER_W_ERR\n");
    }
    else if(SOC5140_8404_BEFOR_W_ERR == r8404_status)
    {
    	  SENSORDB("2 SOC5140_8404_BEFOR_W_ERR\n");
    }
}

void SOC5140_8404_Polling(kal_uint32 addr, kal_uint32 para)
{
   kal_uint16  temp_data = 0xff;
   kal_uint16  count = 0;

   SENSORDB("****** SOC5140_8404_Polling \n");
   if(SOC5140_YUV_NOREFRESH == write_statue)
   {
        SENSORDB("SOC5140_YUV_NOREFRESH \n");
        return;
   }
   else if(SOC5140_YUV_REFRESH == write_statue)
   {
        SENSORDB("SOC5140_YUV_NOREFRESH \n");
        SOC5140_8404_Polling_Sub(30, 0x06);
        SOC5140_8404_Polling_Sub(30, 0x05);
   }
   else
   {
        SENSORDB("normal \n");
        SOC5140_8404_Polling_Sub(100, para);
   }
}

void SOC5140_Stream_Off(void)
{
   	SENSORDB("******* SOC5140_Stream_Off\n");
    SOC5140_write_cmos_sensor(0x3c00, 0x0000);
}

void SOC5140_Cap_Stream_On(void)
{
    kal_uint16  temp_data = 0x0;

   	SENSORDB("******* SOC5140_Cap_Stream_On\n");
    temp_data = SOC5140_read_cmos_sensor_8(0x8505);
    while(0x07 != temp_data)
    {
      	SENSORDB("SOC5140_Cap_Stream_On err\n");
    }

    SOC5140_write_cmos_sensor(0x3400, 0x7A24);
}

void SOC5140_Pre_Stream_On(void)
{
    kal_uint16  temp_data = 0x0;

   	SENSORDB("******* SOC5140_Pre_Stream_On\n");
    temp_data = SOC5140_read_cmos_sensor_8(0x8505);
    while(0x03 != temp_data)
    {
   	    SENSORDB("SOC5140_Pre_Stream_On err\n");
    }

    SOC5140_write_cmos_sensor(0x3400, 0x7A24);
}
void SOC5140_Initial_Setting(void)
{
	SENSORDB( "\n ****** SOC5140_Initial_Setting \n");
    //[**********Step1*************]
    // VCO= 768 MHz, CPU= 48 MHz, Tx= 96 MHz
    SOC5140_write_cmos_sensor(0x0010, 0x0340);		   //pll_dividers = 819
    SOC5140_write_cmos_sensor(0x0012, 0x0070);		   //pll_p_dividers = 255
    SOC5140_write_cmos_sensor(0x0014, 0x20F5);		   //pll_control = 8949
    SOC5140_write_cmos_sensor(0x001A, 0x001C);		   //reset_and_misc_control = 28
    SOC5140_write_cmos_sensor(0x001E, 0x0444);		   //pad_slew_pad_config = 0
    SOC5140_write_cmos_sensor(0x0022, 0x0048);		   //vdd_dis_counter = 48
    SOC5140_write_cmos_sensor(0x002A, 0x7F7C);		   //pll_p4_p5_p6_dividers = 32511
    SOC5140_write_cmos_sensor(0x002C, 0x0000);		   //pll_p7_divider = 15
    SOC5140_write_cmos_sensor(0x0018, 0x400C);		   //standby_control_and_status = 16396
    mDELAY(10);                
    SOC5140_write_cmos_sensor(0x098E, 0x0000);
    SOC5140_write_cmos_sensor(0x301A, 0x0030);
    SOC5140_write_cmos_sensor(0x316C, 0xB430);			 // DAC_TXLO
    SOC5140_write_cmos_sensor(0x31E0, 0x0003);			 // PIX_DEF_ID
    SOC5140_write_cmos_sensor(0x3E2E, 0xF319);			 // SAMP_SPARE
    SOC5140_write_cmos_sensor(0x3EE6, 0xA7C1);
    // data pedestal and fuse defect correction fixes
    SOC5140_write_cmos_sensor(0x301E, 0x00A8);			 // (168) DATA_PEDESTAL_
    SOC5140_write_cmos_sensor_8(0xDC33, 0x2A); 			   // (42) SYS_FIRST_BLACK_LEVEL
    // OTPM comparator value
    SOC5140_write_cmos_sensor(0x3812, 0x212C);
    //[Step2-Fixup MIPI]
    // MIPI interface into standby
    SOC5140_write_cmos_sensor(0x3400, 0x7A26);			 // MIPI_CONTROL
    mDELAY(10);
    //******************************************************************************
    // these two operations are required by the Aptina Dual MIPI FPGA adapter board,
    // all other applications should leave these lines disabled
    //SERIAL_REG= 0xCA, 0x00, 0x8005, 8:16
    //SERIAL_REG= 0xCA, 0x00, 0x0005, 8:16
    //******************************************************************************
    SOC5140_write_cmos_sensor(0x340A, 0x001F);		//TXSS_MIPI_CONTROL_ADDL
    SOC5140_write_cmos_sensor(0x3410, 0x0f00);		//MIPI_TIMING_T_HS_ZERO
    SOC5140_write_cmos_sensor(0x3412, 0x0b07);		//MIPI_TIMING_T_HS_EXIT_HS_TRAIL
    SOC5140_write_cmos_sensor(0x3414, 0x0d01);		//MIPI_TIMING_T_CLK_POST_CLK_PRE
    SOC5140_write_cmos_sensor(0x3416, 0x071d);		//MIPI_T_CLK_TRAIL_CLK_ZERO
    SOC5140_write_cmos_sensor(0x3418, 0x0006);		//MIPI_TIMING_T_LPX
    SOC5140_write_cmos_sensor(0x341A, 0x0a0c);		//MIPI_INIT_TIMING
    SOC5140_write_cmos_sensor(0x3CA0, 0x0001);		//TXSS_PARAMETERS
    SOC5140_write_cmos_sensor(0x3CA2, 0x0007);		//TXC_PARAMETERS
    SOC5140_write_cmos_sensor(0x3CAA, 0x0d0d);		//TXC_TIMING
    // MIPI interface out of standby
    SOC5140_write_cmos_sensor(0x3400, 0x7A24);     // MIPI_CONTROL
    mDELAY(10);
    // MIPI variable settings
    SOC5140_write_cmos_sensor(0xC8D4, 0x0000); 		// CAM_OUTPUT_1_MIPI_CHANNEL
    SOC5140_write_cmos_sensor(0xD822, 0x4710); 		// JPEG_JPSS_CTRL_VAR

    //[**********step3*************]
    //  k28a_rev03_patch16_CR32853_KYE_VPTECH_FAST_FOCUS_REV8
    SOC5140_write_cmos_sensor(0x0982, 0x0000);
    SOC5140_write_cmos_sensor(0x098A, 0x0000);
    SOC5140_write_cmos_sensor(0x886C, 0xC0F1);
    SOC5140_write_cmos_sensor(0x886E, 0xC5E1);
    SOC5140_write_cmos_sensor(0x8870, 0x246A);
    SOC5140_write_cmos_sensor(0x8872, 0x1280);
    SOC5140_write_cmos_sensor(0x8874, 0xC4E1);
    SOC5140_write_cmos_sensor(0x8876, 0xD20F);
    SOC5140_write_cmos_sensor(0x8878, 0x2069);
    SOC5140_write_cmos_sensor(0x887A, 0x0000);
    SOC5140_write_cmos_sensor(0x887C, 0x6A62);
    SOC5140_write_cmos_sensor(0x887E, 0x1303);
    SOC5140_write_cmos_sensor(0x8880, 0x0084);
    SOC5140_write_cmos_sensor(0x8882, 0x1734);
    SOC5140_write_cmos_sensor(0x8884, 0x7005);
    SOC5140_write_cmos_sensor(0x8886, 0xD801);
    SOC5140_write_cmos_sensor(0x8888, 0x8A41);
    SOC5140_write_cmos_sensor(0x888A, 0xD900);
    SOC5140_write_cmos_sensor(0x888C, 0x0D5A);
    SOC5140_write_cmos_sensor(0x888E, 0x0664);
    SOC5140_write_cmos_sensor(0x8890, 0x8B61);
    SOC5140_write_cmos_sensor(0x8892, 0xE80B);
    SOC5140_write_cmos_sensor(0x8894, 0x000D);
    SOC5140_write_cmos_sensor(0x8896, 0x0020);
    SOC5140_write_cmos_sensor(0x8898, 0xD508);
    SOC5140_write_cmos_sensor(0x889A, 0x1504);
    SOC5140_write_cmos_sensor(0x889C, 0x1400);
    SOC5140_write_cmos_sensor(0x889E, 0x7840);
    SOC5140_write_cmos_sensor(0x88A0, 0xD007);
    SOC5140_write_cmos_sensor(0x88A2, 0x0DFB);
    SOC5140_write_cmos_sensor(0x88A4, 0x9004);
    SOC5140_write_cmos_sensor(0x88A6, 0xC4C1);
    SOC5140_write_cmos_sensor(0x88A8, 0x2029);
    SOC5140_write_cmos_sensor(0x88AA, 0x0300);
    SOC5140_write_cmos_sensor(0x88AC, 0x0219);
    SOC5140_write_cmos_sensor(0x88AE, 0x06C4);
    SOC5140_write_cmos_sensor(0x88B0, 0xFF80);
    SOC5140_write_cmos_sensor(0x88B2, 0x08D4);
    SOC5140_write_cmos_sensor(0x88B4, 0xFF80);
    SOC5140_write_cmos_sensor(0x88B6, 0x086C);
    SOC5140_write_cmos_sensor(0x88B8, 0xFF80);
    SOC5140_write_cmos_sensor(0x88BA, 0x08C0);
    SOC5140_write_cmos_sensor(0x88BC, 0xFF80);
    SOC5140_write_cmos_sensor(0x88BE, 0x08D4);
    SOC5140_write_cmos_sensor(0x88C0, 0xFF80);
    SOC5140_write_cmos_sensor(0x88C2, 0x08DC);
    SOC5140_write_cmos_sensor(0x88C4, 0xFF80);
    SOC5140_write_cmos_sensor(0x88C6, 0x0F58);
    SOC5140_write_cmos_sensor(0x88C8, 0xFF80);
    SOC5140_write_cmos_sensor(0x88CA, 0x0920);
    SOC5140_write_cmos_sensor(0x88CC, 0xFF80);
    SOC5140_write_cmos_sensor(0x88CE, 0x1010);
    SOC5140_write_cmos_sensor(0x88D0, 0xFF80);
    SOC5140_write_cmos_sensor(0x88D2, 0x1030);
    SOC5140_write_cmos_sensor(0x88D4, 0x0010);
    SOC5140_write_cmos_sensor(0x88D6, 0x0008);
    SOC5140_write_cmos_sensor(0x88D8, 0x0000);
    SOC5140_write_cmos_sensor(0x88DA, 0x0000);
    SOC5140_write_cmos_sensor(0x88DC, 0xD102);
    SOC5140_write_cmos_sensor(0x88DE, 0xD003);
    SOC5140_write_cmos_sensor(0x88E0, 0x7FE0);
    SOC5140_write_cmos_sensor(0x88E2, 0xB035);
    SOC5140_write_cmos_sensor(0x88E4, 0xFF80);
    SOC5140_write_cmos_sensor(0x88E6, 0x10C8);
    SOC5140_write_cmos_sensor(0x88E8, 0xFF80);
    SOC5140_write_cmos_sensor(0x88EA, 0x0118);
    SOC5140_write_cmos_sensor(0x88EC, 0xC0F1);
    SOC5140_write_cmos_sensor(0x88EE, 0xC5E1);
    SOC5140_write_cmos_sensor(0x88F0, 0xD5EC);
    SOC5140_write_cmos_sensor(0x88F2, 0x8D04);
    SOC5140_write_cmos_sensor(0x88F4, 0x8D25);
    SOC5140_write_cmos_sensor(0x88F6, 0xB808);
    SOC5140_write_cmos_sensor(0x88F8, 0x7825);
    SOC5140_write_cmos_sensor(0x88FA, 0x0821);
    SOC5140_write_cmos_sensor(0x88FC, 0x01DE);
    SOC5140_write_cmos_sensor(0x88FE, 0xD0EA);
    SOC5140_write_cmos_sensor(0x8900, 0x8000);
    SOC5140_write_cmos_sensor(0x8902, 0x8008);
    SOC5140_write_cmos_sensor(0x8904, 0x7840);
    SOC5140_write_cmos_sensor(0x8906, 0x8D04);
    SOC5140_write_cmos_sensor(0x8908, 0x8D25);
    SOC5140_write_cmos_sensor(0x890A, 0xB808);
    SOC5140_write_cmos_sensor(0x890C, 0x7825);
    SOC5140_write_cmos_sensor(0x890E, 0xB8A7);
    SOC5140_write_cmos_sensor(0x8910, 0x2841);
    SOC5140_write_cmos_sensor(0x8912, 0x0201);
    SOC5140_write_cmos_sensor(0x8914, 0xAD24);
    SOC5140_write_cmos_sensor(0x8916, 0xAD05);
    SOC5140_write_cmos_sensor(0x8918, 0x09A6);
    SOC5140_write_cmos_sensor(0x891A, 0x0104);
    SOC5140_write_cmos_sensor(0x891C, 0x01A9);
    SOC5140_write_cmos_sensor(0x891E, 0x06C4);
    SOC5140_write_cmos_sensor(0x8920, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8922, 0x0932);
    SOC5140_write_cmos_sensor(0x8924, 0x06E4);
    SOC5140_write_cmos_sensor(0x8926, 0xDA38);
    SOC5140_write_cmos_sensor(0x8928, 0xD1E0);
    SOC5140_write_cmos_sensor(0x892A, 0xD5E1);
    SOC5140_write_cmos_sensor(0x892C, 0x76A9);
    SOC5140_write_cmos_sensor(0x892E, 0x0EC6);
    SOC5140_write_cmos_sensor(0x8930, 0x06A4);
    SOC5140_write_cmos_sensor(0x8932, 0x70C9);
    SOC5140_write_cmos_sensor(0x8934, 0xD0DF);
    SOC5140_write_cmos_sensor(0x8936, 0xA501);
    SOC5140_write_cmos_sensor(0x8938, 0xD0DF);
    SOC5140_write_cmos_sensor(0x893A, 0xA503);
    SOC5140_write_cmos_sensor(0x893C, 0xD0DF);
    SOC5140_write_cmos_sensor(0x893E, 0xA506);
    SOC5140_write_cmos_sensor(0x8940, 0xD0DF);
    SOC5140_write_cmos_sensor(0x8942, 0xA509);
    SOC5140_write_cmos_sensor(0x8944, 0xD0D8);
    SOC5140_write_cmos_sensor(0x8946, 0xA0C0);
    SOC5140_write_cmos_sensor(0x8948, 0xD0DE);
    SOC5140_write_cmos_sensor(0x894A, 0x802E);
    SOC5140_write_cmos_sensor(0x894C, 0x9117);
    SOC5140_write_cmos_sensor(0x894E, 0x0171);
    SOC5140_write_cmos_sensor(0x8950, 0x06E4);
    SOC5140_write_cmos_sensor(0x8952, 0xB10E);
    SOC5140_write_cmos_sensor(0x8954, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8956, 0xD0D3);
    SOC5140_write_cmos_sensor(0x8958, 0x8806);
    SOC5140_write_cmos_sensor(0x895A, 0x080F);
    SOC5140_write_cmos_sensor(0x895C, 0x0051);
    SOC5140_write_cmos_sensor(0x895E, 0xD0D2);
    SOC5140_write_cmos_sensor(0x8960, 0x8000);
    SOC5140_write_cmos_sensor(0x8962, 0x8008);
    SOC5140_write_cmos_sensor(0x8964, 0x7840);
    SOC5140_write_cmos_sensor(0x8966, 0x0A1E);
    SOC5140_write_cmos_sensor(0x8968, 0x0104);
    SOC5140_write_cmos_sensor(0x896A, 0xC0D1);
    SOC5140_write_cmos_sensor(0x896C, 0x7EE0);
    SOC5140_write_cmos_sensor(0x896E, 0x78E0);
    SOC5140_write_cmos_sensor(0x8970, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8972, 0x08D6);
    SOC5140_write_cmos_sensor(0x8974, 0x06C4);
    SOC5140_write_cmos_sensor(0x8976, 0xD7CC);
    SOC5140_write_cmos_sensor(0x8978, 0x8700);
    SOC5140_write_cmos_sensor(0x897A, 0x8009);
    SOC5140_write_cmos_sensor(0x897C, 0x7840);
    SOC5140_write_cmos_sensor(0x897E, 0xE080);
    SOC5140_write_cmos_sensor(0x8980, 0x0276);
    SOC5140_write_cmos_sensor(0x8982, 0x0002);
    SOC5140_write_cmos_sensor(0x8984, 0xD5C7);
    SOC5140_write_cmos_sensor(0x8986, 0xD6D0);
    SOC5140_write_cmos_sensor(0x8988, 0x1530);
    SOC5140_write_cmos_sensor(0x898A, 0x1081);
    SOC5140_write_cmos_sensor(0x898C, 0x1531);
    SOC5140_write_cmos_sensor(0x898E, 0x1080);
    SOC5140_write_cmos_sensor(0x8990, 0xB908);
    SOC5140_write_cmos_sensor(0x8992, 0x7905);
    SOC5140_write_cmos_sensor(0x8994, 0x2941);
    SOC5140_write_cmos_sensor(0x8996, 0x0200);
    SOC5140_write_cmos_sensor(0x8998, 0x1D32);
    SOC5140_write_cmos_sensor(0x899A, 0x1002);
    SOC5140_write_cmos_sensor(0x899C, 0x1D33);
    SOC5140_write_cmos_sensor(0x899E, 0x1042);
    SOC5140_write_cmos_sensor(0x89A0, 0x962D);
    SOC5140_write_cmos_sensor(0x89A2, 0x2540);
    SOC5140_write_cmos_sensor(0x89A4, 0x15D1);
    SOC5140_write_cmos_sensor(0x89A6, 0x2941);
    SOC5140_write_cmos_sensor(0x89A8, 0x0200);
    SOC5140_write_cmos_sensor(0x89AA, 0x1D30);
    SOC5140_write_cmos_sensor(0x89AC, 0x1002);
    SOC5140_write_cmos_sensor(0x89AE, 0x8D06);
    SOC5140_write_cmos_sensor(0x89B0, 0x2540);
    SOC5140_write_cmos_sensor(0x89B2, 0x1610);
    SOC5140_write_cmos_sensor(0x89B4, 0x1D31);
    SOC5140_write_cmos_sensor(0x89B6, 0x1042);
    SOC5140_write_cmos_sensor(0x89B8, 0x081B);
    SOC5140_write_cmos_sensor(0x89BA, 0x0051);
    SOC5140_write_cmos_sensor(0x89BC, 0x8700);
    SOC5140_write_cmos_sensor(0x89BE, 0x8008);
    SOC5140_write_cmos_sensor(0x89C0, 0x7840);
    SOC5140_write_cmos_sensor(0x89C2, 0xD900);
    SOC5140_write_cmos_sensor(0x89C4, 0x2941);
    SOC5140_write_cmos_sensor(0x89C6, 0x0200);
    SOC5140_write_cmos_sensor(0x89C8, 0xAD00);
    SOC5140_write_cmos_sensor(0x89CA, 0xAD21);
    SOC5140_write_cmos_sensor(0x89CC, 0xD801);
    SOC5140_write_cmos_sensor(0x89CE, 0x1D4D);
    SOC5140_write_cmos_sensor(0x89D0, 0x1002);
    SOC5140_write_cmos_sensor(0x89D2, 0x154D);
    SOC5140_write_cmos_sensor(0x89D4, 0x1080);
    SOC5140_write_cmos_sensor(0x89D6, 0xB861);
    SOC5140_write_cmos_sensor(0x89D8, 0xE085);
    SOC5140_write_cmos_sensor(0x89DA, 0x0218);
    SOC5140_write_cmos_sensor(0x89DC, 0x000D);
    SOC5140_write_cmos_sensor(0x89DE, 0x2740);
    SOC5140_write_cmos_sensor(0x89E0, 0x7381);
    SOC5140_write_cmos_sensor(0x89E2, 0x2132);
    SOC5140_write_cmos_sensor(0x89E4, 0x0000);
    SOC5140_write_cmos_sensor(0x89E6, 0x7914);
    SOC5140_write_cmos_sensor(0x89E8, 0x7900);
    SOC5140_write_cmos_sensor(0x89EA, 0x0323);
    SOC5140_write_cmos_sensor(0x89EC, 0x67BB);
    SOC5140_write_cmos_sensor(0x89EE, 0xD62E);
    SOC5140_write_cmos_sensor(0x89F0, 0x8D11);
    SOC5140_write_cmos_sensor(0x89F2, 0xD1B6);
    SOC5140_write_cmos_sensor(0x89F4, 0x8924);
    SOC5140_write_cmos_sensor(0x89F6, 0x2032);
    SOC5140_write_cmos_sensor(0x89F8, 0x2000);
    SOC5140_write_cmos_sensor(0x89FA, 0xDE02);
    SOC5140_write_cmos_sensor(0x89FC, 0x082B);
    SOC5140_write_cmos_sensor(0x89FE, 0x0040);
    SOC5140_write_cmos_sensor(0x8A00, 0x8D20);
    SOC5140_write_cmos_sensor(0x8A02, 0x8D41);
    SOC5140_write_cmos_sensor(0x8A04, 0xB908);
    SOC5140_write_cmos_sensor(0x8A06, 0x7945);
    SOC5140_write_cmos_sensor(0x8A08, 0xB983);
    SOC5140_write_cmos_sensor(0x8A0A, 0x2941);
    SOC5140_write_cmos_sensor(0x8A0C, 0x0202);
    SOC5140_write_cmos_sensor(0x8A0E, 0xAD40);
    SOC5140_write_cmos_sensor(0x8A10, 0xAD21);
    SOC5140_write_cmos_sensor(0x8A12, 0xD1AF);
    SOC5140_write_cmos_sensor(0x8A14, 0x8120);
    SOC5140_write_cmos_sensor(0x8A16, 0x8121);
    SOC5140_write_cmos_sensor(0x8A18, 0x7940);
    SOC5140_write_cmos_sensor(0x8A1A, 0x8D06);
    SOC5140_write_cmos_sensor(0x8A1C, 0xE001);
    SOC5140_write_cmos_sensor(0x8A1E, 0xAD06);
    SOC5140_write_cmos_sensor(0x8A20, 0x1D4D);
    SOC5140_write_cmos_sensor(0x8A22, 0x1382);
    SOC5140_write_cmos_sensor(0x8A24, 0xF0E9);
    SOC5140_write_cmos_sensor(0x8A26, 0x8D06);
    SOC5140_write_cmos_sensor(0x8A28, 0x1D4D);
    SOC5140_write_cmos_sensor(0x8A2A, 0x1382);
    SOC5140_write_cmos_sensor(0x8A2C, 0xE001);
    SOC5140_write_cmos_sensor(0x8A2E, 0xAD06);
    SOC5140_write_cmos_sensor(0x8A30, 0x8D00);
    SOC5140_write_cmos_sensor(0x8A32, 0x8D21);
    SOC5140_write_cmos_sensor(0x8A34, 0xB808);
    SOC5140_write_cmos_sensor(0x8A36, 0x7825);
    SOC5140_write_cmos_sensor(0x8A38, 0xB885);
    SOC5140_write_cmos_sensor(0x8A3A, 0x2841);
    SOC5140_write_cmos_sensor(0x8A3C, 0x0201);
    SOC5140_write_cmos_sensor(0x8A3E, 0xAD20);
    SOC5140_write_cmos_sensor(0x8A40, 0xAD01);
    SOC5140_write_cmos_sensor(0x8A42, 0x8D31);
    SOC5140_write_cmos_sensor(0x8A44, 0xF01A);
    SOC5140_write_cmos_sensor(0x8A46, 0x8D31);
    SOC5140_write_cmos_sensor(0x8A48, 0x8D12);
    SOC5140_write_cmos_sensor(0x8A4A, 0x8D46);
    SOC5140_write_cmos_sensor(0x8A4C, 0x7822);
    SOC5140_write_cmos_sensor(0x8A4E, 0x0863);
    SOC5140_write_cmos_sensor(0x8A50, 0x0082);
    SOC5140_write_cmos_sensor(0x8A52, 0x1532);
    SOC5140_write_cmos_sensor(0x8A54, 0x1080);
    SOC5140_write_cmos_sensor(0x8A56, 0x1533);
    SOC5140_write_cmos_sensor(0x8A58, 0x1081);
    SOC5140_write_cmos_sensor(0x8A5A, 0x1531);
    SOC5140_write_cmos_sensor(0x8A5C, 0x1082);
    SOC5140_write_cmos_sensor(0x8A5E, 0xB808);
    SOC5140_write_cmos_sensor(0x8A60, 0x7825);
    SOC5140_write_cmos_sensor(0x8A62, 0x1530);
    SOC5140_write_cmos_sensor(0x8A64, 0x1081);
    SOC5140_write_cmos_sensor(0x8A66, 0xB908);
    SOC5140_write_cmos_sensor(0x8A68, 0x7945);
    SOC5140_write_cmos_sensor(0x8A6A, 0xD29A);
    SOC5140_write_cmos_sensor(0x8A6C, 0x0992);
    SOC5140_write_cmos_sensor(0x8A6E, 0x0020);
    SOC5140_write_cmos_sensor(0x8A70, 0x8A40);
    SOC5140_write_cmos_sensor(0x8A72, 0x8D31);
    SOC5140_write_cmos_sensor(0x8A74, 0x081F);
    SOC5140_write_cmos_sensor(0x8A76, 0x0051);
    SOC5140_write_cmos_sensor(0x8A78, 0x8D06);
    SOC5140_write_cmos_sensor(0x8A7A, 0x6038);
    SOC5140_write_cmos_sensor(0x8A7C, 0xD194);
    SOC5140_write_cmos_sensor(0x8A7E, 0x8120);
    SOC5140_write_cmos_sensor(0x8A80, 0x8121);
    SOC5140_write_cmos_sensor(0x8A82, 0x7960);
    SOC5140_write_cmos_sensor(0x8A84, 0x2132);
    SOC5140_write_cmos_sensor(0x8A86, 0x2000);
    SOC5140_write_cmos_sensor(0x8A88, 0x8D06);
    SOC5140_write_cmos_sensor(0x8A8A, 0xE001);
    SOC5140_write_cmos_sensor(0x8A8C, 0xAD06);
    SOC5140_write_cmos_sensor(0x8A8E, 0xD806);
    SOC5140_write_cmos_sensor(0x8A90, 0xF0B1);
    SOC5140_write_cmos_sensor(0x8A92, 0xE88F);
    SOC5140_write_cmos_sensor(0x8A94, 0x8D66);
    SOC5140_write_cmos_sensor(0x8A96, 0x633B);
    SOC5140_write_cmos_sensor(0x8A98, 0x63BB);
    SOC5140_write_cmos_sensor(0x8A9A, 0xD08D);
    SOC5140_write_cmos_sensor(0x8A9C, 0x8000);
    SOC5140_write_cmos_sensor(0x8A9E, 0x8021);
    SOC5140_write_cmos_sensor(0x8AA0, 0x7960);
    SOC5140_write_cmos_sensor(0x8AA2, 0x8B17);
    SOC5140_write_cmos_sensor(0x8AA4, 0x8D06);
    SOC5140_write_cmos_sensor(0x8AA6, 0xE001);
    SOC5140_write_cmos_sensor(0x8AA8, 0xAD06);
    SOC5140_write_cmos_sensor(0x8AAA, 0xD803);
    SOC5140_write_cmos_sensor(0x8AAC, 0xF0A3);
    SOC5140_write_cmos_sensor(0x8AAE, 0x2032);
    SOC5140_write_cmos_sensor(0x8AB0, 0x2040);
    SOC5140_write_cmos_sensor(0x8AB2, 0xAD07);
    SOC5140_write_cmos_sensor(0x8AB4, 0xD804);
    SOC5140_write_cmos_sensor(0x8AB6, 0xF09F);
    SOC5140_write_cmos_sensor(0x8AB8, 0x1532);
    SOC5140_write_cmos_sensor(0x8ABA, 0x1080);
    SOC5140_write_cmos_sensor(0x8ABC, 0x1533);
    SOC5140_write_cmos_sensor(0x8ABE, 0x1081);
    SOC5140_write_cmos_sensor(0x8AC0, 0x1531);
    SOC5140_write_cmos_sensor(0x8AC2, 0x1082);
    SOC5140_write_cmos_sensor(0x8AC4, 0xB808);
    SOC5140_write_cmos_sensor(0x8AC6, 0x7825);
    SOC5140_write_cmos_sensor(0x8AC8, 0x1530);
    SOC5140_write_cmos_sensor(0x8ACA, 0x1081);
    SOC5140_write_cmos_sensor(0x8ACC, 0xB908);
    SOC5140_write_cmos_sensor(0x8ACE, 0x7945);
    SOC5140_write_cmos_sensor(0x8AD0, 0xD280);
    SOC5140_write_cmos_sensor(0x8AD2, 0x092E);
    SOC5140_write_cmos_sensor(0x8AD4, 0x0020);
    SOC5140_write_cmos_sensor(0x8AD6, 0x8A41);
    SOC5140_write_cmos_sensor(0x8AD8, 0x8D51);
    SOC5140_write_cmos_sensor(0x8ADA, 0x8D32);
    SOC5140_write_cmos_sensor(0x8ADC, 0x8DC6);
    SOC5140_write_cmos_sensor(0x8ADE, 0x7942);
    SOC5140_write_cmos_sensor(0x8AE0, 0x62DB);
    SOC5140_write_cmos_sensor(0x8AE2, 0x091F);
    SOC5140_write_cmos_sensor(0x8AE4, 0x03A2);
    SOC5140_write_cmos_sensor(0x8AE6, 0x63BB);
    SOC5140_write_cmos_sensor(0x8AE8, 0xE88B);
    SOC5140_write_cmos_sensor(0x8AEA, 0x8D00);
    SOC5140_write_cmos_sensor(0x8AEC, 0x8D21);
    SOC5140_write_cmos_sensor(0x8AEE, 0xB808);
    SOC5140_write_cmos_sensor(0x8AF0, 0x7825);
    SOC5140_write_cmos_sensor(0x8AF2, 0xB885);
    SOC5140_write_cmos_sensor(0x8AF4, 0x2841);
    SOC5140_write_cmos_sensor(0x8AF6, 0x0201);
    SOC5140_write_cmos_sensor(0x8AF8, 0xAD20);
    SOC5140_write_cmos_sensor(0x8AFA, 0xAD01);
    SOC5140_write_cmos_sensor(0x8AFC, 0xF1CF);
    SOC5140_write_cmos_sensor(0x8AFE, 0xDF04);
    SOC5140_write_cmos_sensor(0x8B00, 0x092B);
    SOC5140_write_cmos_sensor(0x8B02, 0x03A3);
    SOC5140_write_cmos_sensor(0x8B04, 0x1D4D);
    SOC5140_write_cmos_sensor(0x8B06, 0x13C2);
    SOC5140_write_cmos_sensor(0x8B08, 0x1530);
    SOC5140_write_cmos_sensor(0x8B0A, 0x108E);
    SOC5140_write_cmos_sensor(0x8B0C, 0x1531);
    SOC5140_write_cmos_sensor(0x8B0E, 0x1081);
    SOC5140_write_cmos_sensor(0x8B10, 0x1533);
    SOC5140_write_cmos_sensor(0x8B12, 0x108F);
    SOC5140_write_cmos_sensor(0x8B14, 0xBE08);
    SOC5140_write_cmos_sensor(0x8B16, 0x7E25);
    SOC5140_write_cmos_sensor(0x8B18, 0x1532);
    SOC5140_write_cmos_sensor(0x8B1A, 0x1081);
    SOC5140_write_cmos_sensor(0x8B1C, 0xB908);
    SOC5140_write_cmos_sensor(0x8B1E, 0x79E5);
    SOC5140_write_cmos_sensor(0x8B20, 0x0907);
    SOC5140_write_cmos_sensor(0x8B22, 0x0382);
    SOC5140_write_cmos_sensor(0x8B24, 0xE883);
    SOC5140_write_cmos_sensor(0x8B26, 0x8B16);
    SOC5140_write_cmos_sensor(0x8B28, 0xF002);
    SOC5140_write_cmos_sensor(0x8B2A, 0x8B15);
    SOC5140_write_cmos_sensor(0x8B2C, 0x8D22);
    SOC5140_write_cmos_sensor(0x8B2E, 0xAD07);
    SOC5140_write_cmos_sensor(0x8B30, 0x8D03);
    SOC5140_write_cmos_sensor(0x8B32, 0xD367);
    SOC5140_write_cmos_sensor(0x8B34, 0xB908);
    SOC5140_write_cmos_sensor(0x8B36, 0x8DC1);
    SOC5140_write_cmos_sensor(0x8B38, 0x7905);
    SOC5140_write_cmos_sensor(0x8B3A, 0x8D00);
    SOC5140_write_cmos_sensor(0x8B3C, 0xB808);
    SOC5140_write_cmos_sensor(0x8B3E, 0x78C5);
    SOC5140_write_cmos_sensor(0x8B40, 0x0921);
    SOC5140_write_cmos_sensor(0x8B42, 0x011E);
    SOC5140_write_cmos_sensor(0x8B44, 0xB883);
    SOC5140_write_cmos_sensor(0x8B46, 0x2841);
    SOC5140_write_cmos_sensor(0x8B48, 0x0201);
    SOC5140_write_cmos_sensor(0x8B4A, 0xAD20);
    SOC5140_write_cmos_sensor(0x8B4C, 0x8320);
    SOC5140_write_cmos_sensor(0x8B4E, 0xAD01);
    SOC5140_write_cmos_sensor(0x8B50, 0x8121);
    SOC5140_write_cmos_sensor(0x8B52, 0x7960);
    SOC5140_write_cmos_sensor(0x8B54, 0x2032);
    SOC5140_write_cmos_sensor(0x8B56, 0x2080);
    SOC5140_write_cmos_sensor(0x8B58, 0x8D06);
    SOC5140_write_cmos_sensor(0x8B5A, 0xE001);
    SOC5140_write_cmos_sensor(0x8B5C, 0xAD06);
    SOC5140_write_cmos_sensor(0x8B5E, 0xF04D);
    SOC5140_write_cmos_sensor(0x8B60, 0x8D00);
    SOC5140_write_cmos_sensor(0x8B62, 0x8D21);
    SOC5140_write_cmos_sensor(0x8B64, 0xB808);
    SOC5140_write_cmos_sensor(0x8B66, 0x7825);
    SOC5140_write_cmos_sensor(0x8B68, 0xB886);
    SOC5140_write_cmos_sensor(0x8B6A, 0x2841);
    SOC5140_write_cmos_sensor(0x8B6C, 0x0201);
    SOC5140_write_cmos_sensor(0x8B6E, 0xAD20);
    SOC5140_write_cmos_sensor(0x8B70, 0xAD01);
    SOC5140_write_cmos_sensor(0x8B72, 0xD057);
    SOC5140_write_cmos_sensor(0x8B74, 0x8000);
    SOC5140_write_cmos_sensor(0x8B76, 0x8021);
    SOC5140_write_cmos_sensor(0x8B78, 0x7960);
    SOC5140_write_cmos_sensor(0x8B7A, 0x8D07);
    SOC5140_write_cmos_sensor(0x8B7C, 0x1545);
    SOC5140_write_cmos_sensor(0x8B7E, 0x1080);
    SOC5140_write_cmos_sensor(0x8B80, 0x1546);
    SOC5140_write_cmos_sensor(0x8B82, 0x1081);
    SOC5140_write_cmos_sensor(0x8B84, 0xB808);
    SOC5140_write_cmos_sensor(0x8B86, 0x7825);
    SOC5140_write_cmos_sensor(0x8B88, 0x085D);
    SOC5140_write_cmos_sensor(0x8B8A, 0x005E);
    SOC5140_write_cmos_sensor(0x8B8C, 0x8D06);
    SOC5140_write_cmos_sensor(0x8B8E, 0xE001);
    SOC5140_write_cmos_sensor(0x8B90, 0xAD06);
    SOC5140_write_cmos_sensor(0x8B92, 0xD805);
    SOC5140_write_cmos_sensor(0x8B94, 0xF02F);
    SOC5140_write_cmos_sensor(0x8B96, 0x1530);
    SOC5140_write_cmos_sensor(0x8B98, 0x1082);
    SOC5140_write_cmos_sensor(0x8B9A, 0x1531);
    SOC5140_write_cmos_sensor(0x8B9C, 0x1080);
    SOC5140_write_cmos_sensor(0x8B9E, 0xD14D);
    SOC5140_write_cmos_sensor(0x8BA0, 0xBA08);
    SOC5140_write_cmos_sensor(0x8BA2, 0x7A05);
    SOC5140_write_cmos_sensor(0x8BA4, 0x8903);
    SOC5140_write_cmos_sensor(0x8BA6, 0x080F);
    SOC5140_write_cmos_sensor(0x8BA8, 0x0083);
    SOC5140_write_cmos_sensor(0x8BAA, 0x8902);
    SOC5140_write_cmos_sensor(0x8BAC, 0x8E44);
    SOC5140_write_cmos_sensor(0x8BAE, 0x0839);
    SOC5140_write_cmos_sensor(0x8BB0, 0x0082);
    SOC5140_write_cmos_sensor(0x8BB2, 0x1545);
    SOC5140_write_cmos_sensor(0x8BB4, 0x1082);
    SOC5140_write_cmos_sensor(0x8BB6, 0x1546);
    SOC5140_write_cmos_sensor(0x8BB8, 0x1080);
    SOC5140_write_cmos_sensor(0x8BBA, 0xBA08);
    SOC5140_write_cmos_sensor(0x8BBC, 0x7A05);
    SOC5140_write_cmos_sensor(0x8BBE, 0x0A29);
    SOC5140_write_cmos_sensor(0x8BC0, 0x005E);
    SOC5140_write_cmos_sensor(0x8BC2, 0x8D00);
    SOC5140_write_cmos_sensor(0x8BC4, 0x8D21);
    SOC5140_write_cmos_sensor(0x8BC6, 0xB808);
    SOC5140_write_cmos_sensor(0x8BC8, 0x7825);
    SOC5140_write_cmos_sensor(0x8BCA, 0xB88D);
    SOC5140_write_cmos_sensor(0x8BCC, 0x2841);
    SOC5140_write_cmos_sensor(0x8BCE, 0x0201);
    SOC5140_write_cmos_sensor(0x8BD0, 0xAD20);
    SOC5140_write_cmos_sensor(0x8BD2, 0xAD01);
    SOC5140_write_cmos_sensor(0x8BD4, 0x0A11);
    SOC5140_write_cmos_sensor(0x8BD6, 0x009E);
    SOC5140_write_cmos_sensor(0x8BD8, 0xD03D);
    SOC5140_write_cmos_sensor(0x8BDA, 0x8000);
    SOC5140_write_cmos_sensor(0x8BDC, 0x8021);
    SOC5140_write_cmos_sensor(0x8BDE, 0x7960);
    SOC5140_write_cmos_sensor(0x8BE0, 0x1550);
    SOC5140_write_cmos_sensor(0x8BE2, 0x1080);
    SOC5140_write_cmos_sensor(0x8BE4, 0xD800);
    SOC5140_write_cmos_sensor(0x8BE6, 0x09AA);
    SOC5140_write_cmos_sensor(0x8BE8, 0x0164);
    SOC5140_write_cmos_sensor(0x8BEA, 0x1D4D);
    SOC5140_write_cmos_sensor(0x8BEC, 0x1002);
    SOC5140_write_cmos_sensor(0x8BEE, 0xF005);
    SOC5140_write_cmos_sensor(0x8BF0, 0xD800);
    SOC5140_write_cmos_sensor(0x8BF2, 0x1D4D);
    SOC5140_write_cmos_sensor(0x8BF4, 0x1002);
    SOC5140_write_cmos_sensor(0x8BF6, 0x06B1);
    SOC5140_write_cmos_sensor(0x8BF8, 0x0684);
    SOC5140_write_cmos_sensor(0x8BFA, 0x78E0);
    SOC5140_write_cmos_sensor(0x8BFC, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8BFE, 0x0E4E);
    SOC5140_write_cmos_sensor(0x8C00, 0x06A4);
    SOC5140_write_cmos_sensor(0x8C02, 0x7308);
    SOC5140_write_cmos_sensor(0x8C04, 0x0919);
    SOC5140_write_cmos_sensor(0x8C06, 0x0023);
    SOC5140_write_cmos_sensor(0x8C08, 0x721A);
    SOC5140_write_cmos_sensor(0x8C0A, 0xD026);
    SOC5140_write_cmos_sensor(0x8C0C, 0x1030);
    SOC5140_write_cmos_sensor(0x8C0E, 0x0082);
    SOC5140_write_cmos_sensor(0x8C10, 0x1031);
    SOC5140_write_cmos_sensor(0x8C12, 0x0081);
    SOC5140_write_cmos_sensor(0x8C14, 0xBA08);
    SOC5140_write_cmos_sensor(0x8C16, 0x7A25);
    SOC5140_write_cmos_sensor(0x8C18, 0xDD00);
    SOC5140_write_cmos_sensor(0x8C1A, 0xF005);
    SOC5140_write_cmos_sensor(0x8C1C, 0xDD01);
    SOC5140_write_cmos_sensor(0x8C1E, 0x7268);
    SOC5140_write_cmos_sensor(0x8C20, 0x7328);
    SOC5140_write_cmos_sensor(0x8C22, 0x2302);
    SOC5140_write_cmos_sensor(0x8C24, 0x0080);
    SOC5140_write_cmos_sensor(0x8C26, 0x2885);
    SOC5140_write_cmos_sensor(0x8C28, 0x0901);
    SOC5140_write_cmos_sensor(0x8C2A, 0x702F);
    SOC5140_write_cmos_sensor(0x8C2C, 0x0EFA);
    SOC5140_write_cmos_sensor(0x8C2E, 0x06A4);
    SOC5140_write_cmos_sensor(0x8C30, 0x7168);
    SOC5140_write_cmos_sensor(0x8C32, 0xD31C);
    SOC5140_write_cmos_sensor(0x8C34, 0x8BC6);
    SOC5140_write_cmos_sensor(0x8C36, 0x8B31);
    SOC5140_write_cmos_sensor(0x8C38, 0x780F);
    SOC5140_write_cmos_sensor(0x8C3A, 0xD226);
    SOC5140_write_cmos_sensor(0x8C3C, 0x663E);
    SOC5140_write_cmos_sensor(0x8C3E, 0xD123);
    SOC5140_write_cmos_sensor(0x8C40, 0xBE62);
    SOC5140_write_cmos_sensor(0x8C42, 0x7ECF);
    SOC5140_write_cmos_sensor(0x8C44, 0x89E4);
    SOC5140_write_cmos_sensor(0x8C46, 0x2214);
    SOC5140_write_cmos_sensor(0x8C48, 0x0381);
    SOC5140_write_cmos_sensor(0x8C4A, 0xED07);
    SOC5140_write_cmos_sensor(0x8C4C, 0x2840);
    SOC5140_write_cmos_sensor(0x8C4E, 0x020E);
    SOC5140_write_cmos_sensor(0x8C50, 0x7EE5);
    SOC5140_write_cmos_sensor(0x8C52, 0xB1CC);
    SOC5140_write_cmos_sensor(0x8C54, 0xF007);
    SOC5140_write_cmos_sensor(0x8C56, 0x7E12);
    SOC5140_write_cmos_sensor(0x8C58, 0xE601);
    SOC5140_write_cmos_sensor(0x8C5A, 0x7ECF);
    SOC5140_write_cmos_sensor(0x8C5C, 0xBE08);
    SOC5140_write_cmos_sensor(0x8C5E, 0x7FC5);
    SOC5140_write_cmos_sensor(0x8C60, 0xB1EC);
    SOC5140_write_cmos_sensor(0x8C62, 0x080D);
    SOC5140_write_cmos_sensor(0x8C64, 0x2003);
    SOC5140_write_cmos_sensor(0x8C66, 0xED0D);
    SOC5140_write_cmos_sensor(0x8C68, 0xD800);
    SOC5140_write_cmos_sensor(0x8C6A, 0xF01A);
    SOC5140_write_cmos_sensor(0x8C6C, 0x134D);
    SOC5140_write_cmos_sensor(0x8C6E, 0x0080);
    SOC5140_write_cmos_sensor(0x8C70, 0x080B);
    SOC5140_write_cmos_sensor(0x8C72, 0x0190);
    SOC5140_write_cmos_sensor(0x8C74, 0x8A0E);
    SOC5140_write_cmos_sensor(0x8C76, 0x080B);
    SOC5140_write_cmos_sensor(0x8C78, 0x0291);
    SOC5140_write_cmos_sensor(0x8C7A, 0xD801);
    SOC5140_write_cmos_sensor(0x8C7C, 0xF010);
    SOC5140_write_cmos_sensor(0x8C7E, 0x1330);
    SOC5140_write_cmos_sensor(0x8C80, 0x0081);
    SOC5140_write_cmos_sensor(0x8C82, 0x1331);
    SOC5140_write_cmos_sensor(0x8C84, 0x008D);
    SOC5140_write_cmos_sensor(0x8C86, 0xD802);
    SOC5140_write_cmos_sensor(0x8C88, 0xB908);
    SOC5140_write_cmos_sensor(0x8C8A, 0x79A5);
    SOC5140_write_cmos_sensor(0x8C8C, 0xB22A);
    SOC5140_write_cmos_sensor(0x8C8E, 0x1332);
    SOC5140_write_cmos_sensor(0x8C90, 0x0081);
    SOC5140_write_cmos_sensor(0x8C92, 0x1333);
    SOC5140_write_cmos_sensor(0x8C94, 0x008D);
    SOC5140_write_cmos_sensor(0x8C96, 0xB908);
    SOC5140_write_cmos_sensor(0x8C98, 0x79A5);
    SOC5140_write_cmos_sensor(0x8C9A, 0xB22B);
    SOC5140_write_cmos_sensor(0x8C9C, 0x0611);
    SOC5140_write_cmos_sensor(0x8C9E, 0x0684);
    SOC5140_write_cmos_sensor(0x8CA0, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CA2, 0x0290);
    SOC5140_write_cmos_sensor(0x8CA4, 0x8000);
    SOC5140_write_cmos_sensor(0x8CA6, 0x008C);
    SOC5140_write_cmos_sensor(0x8CA8, 0x0000);
    SOC5140_write_cmos_sensor(0x8CAA, 0xF3BC);
    SOC5140_write_cmos_sensor(0x8CAC, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CAE, 0x1120);
    SOC5140_write_cmos_sensor(0x8CB0, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CB2, 0x08EC);
    SOC5140_write_cmos_sensor(0x8CB4, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CB6, 0x0954);
    SOC5140_write_cmos_sensor(0x8CB8, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CBA, 0x0970);
    SOC5140_write_cmos_sensor(0x8CBC, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CBE, 0x0CD4);
    SOC5140_write_cmos_sensor(0x8CC0, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CC2, 0x06C8);
    SOC5140_write_cmos_sensor(0x8CC4, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CC6, 0x050C);
    SOC5140_write_cmos_sensor(0x8CC8, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CCA, 0x0158);
    SOC5140_write_cmos_sensor(0x8CCC, 0x8000);
    SOC5140_write_cmos_sensor(0x8CCE, 0x0008);
    SOC5140_write_cmos_sensor(0x8CD0, 0xFF80);
    SOC5140_write_cmos_sensor(0x8CD2, 0x10C8);
    SOC5140_write_cmos_sensor(0x8CD4, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8CD6, 0x0D7E);
    SOC5140_write_cmos_sensor(0x8CD8, 0x0684);
    SOC5140_write_cmos_sensor(0x8CDA, 0x17C8);
    SOC5140_write_cmos_sensor(0x8CDC, 0xF00D);
    SOC5140_write_cmos_sensor(0x8CDE, 0x1545);
    SOC5140_write_cmos_sensor(0x8CE0, 0x1080);
    SOC5140_write_cmos_sensor(0x8CE2, 0x1546);
    SOC5140_write_cmos_sensor(0x8CE4, 0x1081);
    SOC5140_write_cmos_sensor(0x8CE6, 0xB808);
    SOC5140_write_cmos_sensor(0x8CE8, 0x7825);
    SOC5140_write_cmos_sensor(0x8CEA, 0xB8E0);
    SOC5140_write_cmos_sensor(0x8CEC, 0xDE00);
    SOC5140_write_cmos_sensor(0x8CEE, 0xF208);
    SOC5140_write_cmos_sensor(0x8CF0, 0x8D00);
    SOC5140_write_cmos_sensor(0x8CF2, 0x8D21);
    SOC5140_write_cmos_sensor(0x8CF4, 0xB808);
    SOC5140_write_cmos_sensor(0x8CF6, 0x7825);
    SOC5140_write_cmos_sensor(0x8CF8, 0x2044);
    SOC5140_write_cmos_sensor(0x8CFA, 0x020E);
    SOC5140_write_cmos_sensor(0x8CFC, 0x8D00);
    SOC5140_write_cmos_sensor(0x8CFE, 0x8D21);
    SOC5140_write_cmos_sensor(0x8D00, 0xB808);
    SOC5140_write_cmos_sensor(0x8D02, 0x7825);
    SOC5140_write_cmos_sensor(0x8D04, 0x082F);
    SOC5140_write_cmos_sensor(0x8D06, 0x00DE);
    SOC5140_write_cmos_sensor(0x8D08, 0x7108);
    SOC5140_write_cmos_sensor(0x8D0A, 0x2186);
    SOC5140_write_cmos_sensor(0x8D0C, 0x0FFE);
    SOC5140_write_cmos_sensor(0x8D0E, 0x262F);
    SOC5140_write_cmos_sensor(0x8D10, 0xF04A);
    SOC5140_write_cmos_sensor(0x8D12, 0xF211);
    SOC5140_write_cmos_sensor(0x8D14, 0x17BC);
    SOC5140_write_cmos_sensor(0x8D16, 0xF002);
    SOC5140_write_cmos_sensor(0x8D18, 0x8A25);
    SOC5140_write_cmos_sensor(0x8D1A, 0xE906);
    SOC5140_write_cmos_sensor(0x8D1C, 0xB961);
    SOC5140_write_cmos_sensor(0x8D1E, 0xAA25);
    SOC5140_write_cmos_sensor(0x8D20, 0xD806);
    SOC5140_write_cmos_sensor(0x8D22, 0xF01E);
    SOC5140_write_cmos_sensor(0x8D24, 0x8A24);
    SOC5140_write_cmos_sensor(0x8D26, 0xB8A3);
    SOC5140_write_cmos_sensor(0x8D28, 0xAA25);
    SOC5140_write_cmos_sensor(0x8D2A, 0x2841);
    SOC5140_write_cmos_sensor(0x8D2C, 0x0201);
    SOC5140_write_cmos_sensor(0x8D2E, 0xAD20);
    SOC5140_write_cmos_sensor(0x8D30, 0xAD01);
    SOC5140_write_cmos_sensor(0x8D32, 0x0D56);
    SOC5140_write_cmos_sensor(0x8D34, 0x0144);
    SOC5140_write_cmos_sensor(0x8D36, 0x1545);
    SOC5140_write_cmos_sensor(0x8D38, 0x1081);
    SOC5140_write_cmos_sensor(0x8D3A, 0x1546);
    SOC5140_write_cmos_sensor(0x8D3C, 0x1082);
    SOC5140_write_cmos_sensor(0x8D3E, 0xB908);
    SOC5140_write_cmos_sensor(0x8D40, 0x7945);
    SOC5140_write_cmos_sensor(0x8D42, 0xB9E0);
    SOC5140_write_cmos_sensor(0x8D44, 0x26CC);
    SOC5140_write_cmos_sensor(0x8D46, 0x9022);
    SOC5140_write_cmos_sensor(0x8D48, 0xF20A);
    SOC5140_write_cmos_sensor(0x8D4A, 0x8D20);
    SOC5140_write_cmos_sensor(0x8D4C, 0x8D41);
    SOC5140_write_cmos_sensor(0x8D4E, 0xB908);
    SOC5140_write_cmos_sensor(0x8D50, 0x7945);
    SOC5140_write_cmos_sensor(0x8D52, 0xB983);
    SOC5140_write_cmos_sensor(0x8D54, 0x2941);
    SOC5140_write_cmos_sensor(0x8D56, 0x0202);
    SOC5140_write_cmos_sensor(0x8D58, 0xAD40);
    SOC5140_write_cmos_sensor(0x8D5A, 0xAD21);
    SOC5140_write_cmos_sensor(0x8D5C, 0x0561);
    SOC5140_write_cmos_sensor(0x8D5E, 0x0684);
    SOC5140_write_cmos_sensor(0x8D60, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8D62, 0x0CEE);
    SOC5140_write_cmos_sensor(0x8D64, 0x06A4);
    SOC5140_write_cmos_sensor(0x8D66, 0x7098);
    SOC5140_write_cmos_sensor(0x8D68, 0xD284);
    SOC5140_write_cmos_sensor(0x8D6A, 0x1206);
    SOC5140_write_cmos_sensor(0x8D6C, 0x0086);
    SOC5140_write_cmos_sensor(0x8D6E, 0x2240);
    SOC5140_write_cmos_sensor(0x8D70, 0x0205);
    SOC5140_write_cmos_sensor(0x8D72, 0x264C);
    SOC5140_write_cmos_sensor(0x8D74, 0x8000);
    SOC5140_write_cmos_sensor(0x8D76, 0x20CA);
    SOC5140_write_cmos_sensor(0x8D78, 0x0101);
    SOC5140_write_cmos_sensor(0x8D7A, 0xF237);
    SOC5140_write_cmos_sensor(0x8D7C, 0x8AA7);
    SOC5140_write_cmos_sensor(0x8D7E, 0x6D69);
    SOC5140_write_cmos_sensor(0x8D80, 0x7B6D);
    SOC5140_write_cmos_sensor(0x8D82, 0x0B3F);
    SOC5140_write_cmos_sensor(0x8D84, 0x0012);
    SOC5140_write_cmos_sensor(0x8D86, 0x7068);
    SOC5140_write_cmos_sensor(0x8D88, 0x780D);
    SOC5140_write_cmos_sensor(0x8D8A, 0x2040);
    SOC5140_write_cmos_sensor(0x8D8C, 0x007C);
    SOC5140_write_cmos_sensor(0x8D8E, 0x20A8);
    SOC5140_write_cmos_sensor(0x8D90, 0x0640);
    SOC5140_write_cmos_sensor(0x8D92, 0x71CF);
    SOC5140_write_cmos_sensor(0x8D94, 0xFF80);
    SOC5140_write_cmos_sensor(0x8D96, 0x0158);
    SOC5140_write_cmos_sensor(0x8D98, 0x8924);
    SOC5140_write_cmos_sensor(0x8D9A, 0x2532);
    SOC5140_write_cmos_sensor(0x8D9C, 0x00C0);
    SOC5140_write_cmos_sensor(0x8D9E, 0xBD61);
    SOC5140_write_cmos_sensor(0x8DA0, 0x0819);
    SOC5140_write_cmos_sensor(0x8DA2, 0x0063);
    SOC5140_write_cmos_sensor(0x8DA4, 0x7DAF);
    SOC5140_write_cmos_sensor(0x8DA6, 0x76CF);
    SOC5140_write_cmos_sensor(0x8DA8, 0xFF80);
    SOC5140_write_cmos_sensor(0x8DAA, 0x0290);
    SOC5140_write_cmos_sensor(0x8DAC, 0x8EF1);
    SOC5140_write_cmos_sensor(0x8DAE, 0x2640);
    SOC5140_write_cmos_sensor(0x8DB0, 0x1601);
    SOC5140_write_cmos_sensor(0x8DB2, 0x61E9);
    SOC5140_write_cmos_sensor(0x8DB4, 0x090F);
    SOC5140_write_cmos_sensor(0x8DB6, 0x0002);
    SOC5140_write_cmos_sensor(0x8DB8, 0xAAA7);
    SOC5140_write_cmos_sensor(0x8DBA, 0xBB61);
    SOC5140_write_cmos_sensor(0x8DBC, 0x7B6D);
    SOC5140_write_cmos_sensor(0x8DBE, 0x7088);
    SOC5140_write_cmos_sensor(0x8DC0, 0xF005);
    SOC5140_write_cmos_sensor(0x8DC2, 0x8E26);
    SOC5140_write_cmos_sensor(0x8DC4, 0xAAA7);
    SOC5140_write_cmos_sensor(0x8DC6, 0xB961);
    SOC5140_write_cmos_sensor(0x8DC8, 0xAE26);
    SOC5140_write_cmos_sensor(0x8DCA, 0x0B1F);
    SOC5140_write_cmos_sensor(0x8DCC, 0x0013);
    SOC5140_write_cmos_sensor(0x8DCE, 0x1A07);
    SOC5140_write_cmos_sensor(0x8DD0, 0x0182);
    SOC5140_write_cmos_sensor(0x8DD2, 0xD26B);
    SOC5140_write_cmos_sensor(0x8DD4, 0x8A20);
    SOC5140_write_cmos_sensor(0x8DD6, 0x8A61);
    SOC5140_write_cmos_sensor(0x8DD8, 0xB908);
    SOC5140_write_cmos_sensor(0x8DDA, 0x7965);
    SOC5140_write_cmos_sensor(0x8DDC, 0xB9A3);
    SOC5140_write_cmos_sensor(0x8DDE, 0x2941);
    SOC5140_write_cmos_sensor(0x8DE0, 0x020C);
    SOC5140_write_cmos_sensor(0x8DE2, 0xAA80);
    SOC5140_write_cmos_sensor(0x8DE4, 0xAA21);
    SOC5140_write_cmos_sensor(0x8DE6, 0x04D1);
    SOC5140_write_cmos_sensor(0x8DE8, 0x0684);
    SOC5140_write_cmos_sensor(0x8DEA, 0x78E0);
    SOC5140_write_cmos_sensor(0x8DEC, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8DEE, 0xC5E1);
    SOC5140_write_cmos_sensor(0x8DF0, 0xD363);
    SOC5140_write_cmos_sensor(0x8DF2, 0x8B24);
    SOC5140_write_cmos_sensor(0x8DF4, 0x8B45);
    SOC5140_write_cmos_sensor(0x8DF6, 0xB908);
    SOC5140_write_cmos_sensor(0x8DF8, 0x7945);
    SOC5140_write_cmos_sensor(0x8DFA, 0xE188);
    SOC5140_write_cmos_sensor(0x8DFC, 0x21CC);
    SOC5140_write_cmos_sensor(0x8DFE, 0x8422);
    SOC5140_write_cmos_sensor(0x8E00, 0xF41F);
    SOC5140_write_cmos_sensor(0x8E02, 0x8B26);
    SOC5140_write_cmos_sensor(0x8E04, 0x093B);
    SOC5140_write_cmos_sensor(0x8E06, 0x0051);
    SOC5140_write_cmos_sensor(0x8E08, 0xD15C);
    SOC5140_write_cmos_sensor(0x8E0A, 0xD80A);
    SOC5140_write_cmos_sensor(0x8E0C, 0xA90E);
    SOC5140_write_cmos_sensor(0x8E0E, 0xD05D);
    SOC5140_write_cmos_sensor(0x8E10, 0x8804);
    SOC5140_write_cmos_sensor(0x8E12, 0x1330);
    SOC5140_write_cmos_sensor(0x8E14, 0x0082);
    SOC5140_write_cmos_sensor(0x8E16, 0x1331);
    SOC5140_write_cmos_sensor(0x8E18, 0x008D);
    SOC5140_write_cmos_sensor(0x8E1A, 0xBA08);
    SOC5140_write_cmos_sensor(0x8E1C, 0x7AA5);
    SOC5140_write_cmos_sensor(0x8E1E, 0xB148);
    SOC5140_write_cmos_sensor(0x8E20, 0x8952);
    SOC5140_write_cmos_sensor(0x8E22, 0xA90F);
    SOC5140_write_cmos_sensor(0x8E24, 0x0813);
    SOC5140_write_cmos_sensor(0x8E26, 0x00A2);
    SOC5140_write_cmos_sensor(0x8E28, 0x132C);
    SOC5140_write_cmos_sensor(0x8E2A, 0x0083);
    SOC5140_write_cmos_sensor(0x8E2C, 0xDA00);
    SOC5140_write_cmos_sensor(0x8E2E, 0xA953);
    SOC5140_write_cmos_sensor(0x8E30, 0x7862);
    SOC5140_write_cmos_sensor(0x8E32, 0x780F);
    SOC5140_write_cmos_sensor(0x8E34, 0xF005);
    SOC5140_write_cmos_sensor(0x8E36, 0xDA01);
    SOC5140_write_cmos_sensor(0x8E38, 0xA953);
    SOC5140_write_cmos_sensor(0x8E3A, 0x6078);
    SOC5140_write_cmos_sensor(0x8E3C, 0x780F);
    SOC5140_write_cmos_sensor(0x8E3E, 0x080E);
    SOC5140_write_cmos_sensor(0x8E40, 0x0000);
    SOC5140_write_cmos_sensor(0x8E42, 0x0485);
    SOC5140_write_cmos_sensor(0x8E44, 0x0684);
    SOC5140_write_cmos_sensor(0x8E46, 0x78E0);
    SOC5140_write_cmos_sensor(0x8E48, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8E4A, 0x0BFE);
    SOC5140_write_cmos_sensor(0x8E4C, 0x0684);
    SOC5140_write_cmos_sensor(0x8E4E, 0xD64D);
    SOC5140_write_cmos_sensor(0x8E50, 0x7508);
    SOC5140_write_cmos_sensor(0x8E52, 0x8E01);
    SOC5140_write_cmos_sensor(0x8E54, 0xD14A);
    SOC5140_write_cmos_sensor(0x8E56, 0x2046);
    SOC5140_write_cmos_sensor(0x8E58, 0x00C0);
    SOC5140_write_cmos_sensor(0x8E5A, 0xAE01);
    SOC5140_write_cmos_sensor(0x8E5C, 0x1145);
    SOC5140_write_cmos_sensor(0x8E5E, 0x0080);
    SOC5140_write_cmos_sensor(0x8E60, 0x1146);
    SOC5140_write_cmos_sensor(0x8E62, 0x0082);
    SOC5140_write_cmos_sensor(0x8E64, 0xB808);
    SOC5140_write_cmos_sensor(0x8E66, 0x7845);
    SOC5140_write_cmos_sensor(0x8E68, 0x0817);
    SOC5140_write_cmos_sensor(0x8E6A, 0x001E);
    SOC5140_write_cmos_sensor(0x8E6C, 0x8900);
    SOC5140_write_cmos_sensor(0x8E6E, 0x8941);
    SOC5140_write_cmos_sensor(0x8E70, 0xB808);
    SOC5140_write_cmos_sensor(0x8E72, 0x7845);
    SOC5140_write_cmos_sensor(0x8E74, 0x080B);
    SOC5140_write_cmos_sensor(0x8E76, 0x00DE);
    SOC5140_write_cmos_sensor(0x8E78, 0x70A9);
    SOC5140_write_cmos_sensor(0x8E7A, 0xFFBA);
    SOC5140_write_cmos_sensor(0x8E7C, 0x7508);
    SOC5140_write_cmos_sensor(0x8E7E, 0x1604);
    SOC5140_write_cmos_sensor(0x8E80, 0x1090);
    SOC5140_write_cmos_sensor(0x8E82, 0x0D93);
    SOC5140_write_cmos_sensor(0x8E84, 0x1400);
    SOC5140_write_cmos_sensor(0x8E86, 0x8EEA);
    SOC5140_write_cmos_sensor(0x8E88, 0x8E0B);
    SOC5140_write_cmos_sensor(0x8E8A, 0x214A);
    SOC5140_write_cmos_sensor(0x8E8C, 0x2040);
    SOC5140_write_cmos_sensor(0x8E8E, 0x8E2D);
    SOC5140_write_cmos_sensor(0x8E90, 0xBF08);
    SOC5140_write_cmos_sensor(0x8E92, 0x7F05);
    SOC5140_write_cmos_sensor(0x8E94, 0x8E0C);
    SOC5140_write_cmos_sensor(0x8E96, 0xB808);
    SOC5140_write_cmos_sensor(0x8E98, 0x7825);
    SOC5140_write_cmos_sensor(0x8E9A, 0x7710);
    SOC5140_write_cmos_sensor(0x8E9C, 0x21C2);
    SOC5140_write_cmos_sensor(0x8E9E, 0x244C);
    SOC5140_write_cmos_sensor(0x8EA0, 0x081D);
    SOC5140_write_cmos_sensor(0x8EA2, 0x03E3);
    SOC5140_write_cmos_sensor(0x8EA4, 0xD9FF);
    SOC5140_write_cmos_sensor(0x8EA6, 0x2702);
    SOC5140_write_cmos_sensor(0x8EA8, 0x1002);
    SOC5140_write_cmos_sensor(0x8EAA, 0x2A05);
    SOC5140_write_cmos_sensor(0x8EAC, 0x037E);
    SOC5140_write_cmos_sensor(0x8EAE, 0x0C7A);
    SOC5140_write_cmos_sensor(0x8EB0, 0x06A4);
    SOC5140_write_cmos_sensor(0x8EB2, 0x702F);
    SOC5140_write_cmos_sensor(0x8EB4, 0x7810);
    SOC5140_write_cmos_sensor(0x8EB6, 0x7F02);
    SOC5140_write_cmos_sensor(0x8EB8, 0x7FF0);
    SOC5140_write_cmos_sensor(0x8EBA, 0xF00B);
    SOC5140_write_cmos_sensor(0x8EBC, 0x78E2);
    SOC5140_write_cmos_sensor(0x8EBE, 0x2805);
    SOC5140_write_cmos_sensor(0x8EC0, 0x037E);
    SOC5140_write_cmos_sensor(0x8EC2, 0x0C66);
    SOC5140_write_cmos_sensor(0x8EC4, 0x06A4);
    SOC5140_write_cmos_sensor(0x8EC6, 0x702F);
    SOC5140_write_cmos_sensor(0x8EC8, 0x7810);
    SOC5140_write_cmos_sensor(0x8ECA, 0x671F);
    SOC5140_write_cmos_sensor(0x8ECC, 0x7FF0);
    SOC5140_write_cmos_sensor(0x8ECE, 0x7FEF);
    SOC5140_write_cmos_sensor(0x8ED0, 0x8E08);
    SOC5140_write_cmos_sensor(0x8ED2, 0xBF06);
    SOC5140_write_cmos_sensor(0x8ED4, 0xD12C);
    SOC5140_write_cmos_sensor(0x8ED6, 0xB8C3);
    SOC5140_write_cmos_sensor(0x8ED8, 0x78E5);
    SOC5140_write_cmos_sensor(0x8EDA, 0xB88F);
    SOC5140_write_cmos_sensor(0x8EDC, 0x1908);
    SOC5140_write_cmos_sensor(0x8EDE, 0x0024);
    SOC5140_write_cmos_sensor(0x8EE0, 0x2841);
    SOC5140_write_cmos_sensor(0x8EE2, 0x0201);
    SOC5140_write_cmos_sensor(0x8EE4, 0x1E26);
    SOC5140_write_cmos_sensor(0x8EE6, 0x1042);
    SOC5140_write_cmos_sensor(0x8EE8, 0x0D15);
    SOC5140_write_cmos_sensor(0x8EEA, 0x1423);
    SOC5140_write_cmos_sensor(0x8EEC, 0x1E27);
    SOC5140_write_cmos_sensor(0x8EEE, 0x1002);
    SOC5140_write_cmos_sensor(0x8EF0, 0x214C);
    SOC5140_write_cmos_sensor(0x8EF2, 0xA000);
    SOC5140_write_cmos_sensor(0x8EF4, 0x214A);
    SOC5140_write_cmos_sensor(0x8EF6, 0x2040);
    SOC5140_write_cmos_sensor(0x8EF8, 0x21C2);
    SOC5140_write_cmos_sensor(0x8EFA, 0x2442);
    SOC5140_write_cmos_sensor(0x8EFC, 0x8E21);
    SOC5140_write_cmos_sensor(0x8EFE, 0x214F);
    SOC5140_write_cmos_sensor(0x8F00, 0x0040);
    SOC5140_write_cmos_sensor(0x8F02, 0x090F);
    SOC5140_write_cmos_sensor(0x8F04, 0x2010);
    SOC5140_write_cmos_sensor(0x8F06, 0x2145);
    SOC5140_write_cmos_sensor(0x8F08, 0x0181);
    SOC5140_write_cmos_sensor(0x8F0A, 0xAE21);
    SOC5140_write_cmos_sensor(0x8F0C, 0xF003);
    SOC5140_write_cmos_sensor(0x8F0E, 0xB8A2);
    SOC5140_write_cmos_sensor(0x8F10, 0xAE01);
    SOC5140_write_cmos_sensor(0x8F12, 0x0FFE);
    SOC5140_write_cmos_sensor(0x8F14, 0xFFA3);
    SOC5140_write_cmos_sensor(0x8F16, 0x70A9);
    SOC5140_write_cmos_sensor(0x8F18, 0x038D);
    SOC5140_write_cmos_sensor(0x8F1A, 0x0684);
    SOC5140_write_cmos_sensor(0x8F1C, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8F1E, 0xC5E1);
    SOC5140_write_cmos_sensor(0x8F20, 0xD518);
    SOC5140_write_cmos_sensor(0x8F22, 0x8D00);
    SOC5140_write_cmos_sensor(0x8F24, 0xB8E7);
    SOC5140_write_cmos_sensor(0x8F26, 0x20D1);
    SOC5140_write_cmos_sensor(0x8F28, 0x80E2);
    SOC5140_write_cmos_sensor(0x8F2A, 0xF20D);
    SOC5140_write_cmos_sensor(0x8F2C, 0xD117);
    SOC5140_write_cmos_sensor(0x8F2E, 0xB8A7);
    SOC5140_write_cmos_sensor(0x8F30, 0xAD00);
    SOC5140_write_cmos_sensor(0x8F32, 0xD017);
    SOC5140_write_cmos_sensor(0x8F34, 0x7228);
    SOC5140_write_cmos_sensor(0x8F36, 0x8123);
    SOC5140_write_cmos_sensor(0x8F38, 0xA040);
    SOC5140_write_cmos_sensor(0x8F3A, 0x7960);
    SOC5140_write_cmos_sensor(0x8F3C, 0xD801);
    SOC5140_write_cmos_sensor(0x8F3E, 0xD800);
    SOC5140_write_cmos_sensor(0x8F40, 0xAD05);
    SOC5140_write_cmos_sensor(0x8F42, 0x0F56);
    SOC5140_write_cmos_sensor(0x8F44, 0xFF83);
    SOC5140_write_cmos_sensor(0x8F46, 0x0381);
    SOC5140_write_cmos_sensor(0x8F48, 0x0684);
    SOC5140_write_cmos_sensor(0x8F4A, 0x78E0);
    SOC5140_write_cmos_sensor(0x8F4C, 0xD20D);
    SOC5140_write_cmos_sensor(0x8F4E, 0x8A21);
    SOC5140_write_cmos_sensor(0x8F50, 0xB9A1);
    SOC5140_write_cmos_sensor(0x8F52, 0x782F);
    SOC5140_write_cmos_sensor(0x8F54, 0x7FE0);
    SOC5140_write_cmos_sensor(0x8F56, 0xAA21);
    SOC5140_write_cmos_sensor(0x8F58, 0xD00E);
    SOC5140_write_cmos_sensor(0x8F5A, 0xD10C);
    SOC5140_write_cmos_sensor(0x8F5C, 0xA100);
    SOC5140_write_cmos_sensor(0x8F5E, 0xD00E);
    SOC5140_write_cmos_sensor(0x8F60, 0xA101);
    SOC5140_write_cmos_sensor(0x8F62, 0xD00E);
    SOC5140_write_cmos_sensor(0x8F64, 0xA102);
    SOC5140_write_cmos_sensor(0x8F66, 0xD00E);
    SOC5140_write_cmos_sensor(0x8F68, 0xA103);
    SOC5140_write_cmos_sensor(0x8F6A, 0xD009);
    SOC5140_write_cmos_sensor(0x8F6C, 0xA020);
    SOC5140_write_cmos_sensor(0x8F6E, 0xD005);
    SOC5140_write_cmos_sensor(0x8F70, 0xD988);
    SOC5140_write_cmos_sensor(0x8F72, 0xA820);
    SOC5140_write_cmos_sensor(0x8F74, 0xF1D4);
    SOC5140_write_cmos_sensor(0x8F76, 0x78E0);
    SOC5140_write_cmos_sensor(0x8F78, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F7A, 0x10C8);
    SOC5140_write_cmos_sensor(0x8F7C, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F7E, 0x0290);
    SOC5140_write_cmos_sensor(0x8F80, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F82, 0x0158);
    SOC5140_write_cmos_sensor(0x8F84, 0xFF00);
    SOC5140_write_cmos_sensor(0x8F86, 0x0618);
    SOC5140_write_cmos_sensor(0x8F88, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F8A, 0x1158);
    SOC5140_write_cmos_sensor(0x8F8C, 0x8000);
    SOC5140_write_cmos_sensor(0x8F8E, 0x0008);
    SOC5140_write_cmos_sensor(0x8F90, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F92, 0x0F1C);
    SOC5140_write_cmos_sensor(0x8F94, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F96, 0x0DEC);
    SOC5140_write_cmos_sensor(0x8F98, 0xFF80);
    SOC5140_write_cmos_sensor(0x8F9A, 0x0F4C);
    SOC5140_write_cmos_sensor(0x8F9C, 0x0000);
    SOC5140_write_cmos_sensor(0x8F9E, 0x0998);
    SOC5140_write_cmos_sensor(0x8FA0, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8FA2, 0xC5E1);
    SOC5140_write_cmos_sensor(0x8FA4, 0xD02C);
    SOC5140_write_cmos_sensor(0x8FA6, 0x0982);
    SOC5140_write_cmos_sensor(0x8FA8, 0x0664);
    SOC5140_write_cmos_sensor(0x8FAA, 0x88AE);
    SOC5140_write_cmos_sensor(0x8FAC, 0x0D23);
    SOC5140_write_cmos_sensor(0x8FAE, 0x1051);
    SOC5140_write_cmos_sensor(0x8FB0, 0xD12A);
    SOC5140_write_cmos_sensor(0x8FB2, 0x1145);
    SOC5140_write_cmos_sensor(0x8FB4, 0x0080);
    SOC5140_write_cmos_sensor(0x8FB6, 0x1146);
    SOC5140_write_cmos_sensor(0x8FB8, 0x0082);
    SOC5140_write_cmos_sensor(0x8FBA, 0xB808);
    SOC5140_write_cmos_sensor(0x8FBC, 0x7845);
    SOC5140_write_cmos_sensor(0x8FBE, 0x0813);
    SOC5140_write_cmos_sensor(0x8FC0, 0x00DE);
    SOC5140_write_cmos_sensor(0x8FC2, 0xD027);
    SOC5140_write_cmos_sensor(0x8FC4, 0x8000);
    SOC5140_write_cmos_sensor(0x8FC6, 0x8041);
    SOC5140_write_cmos_sensor(0x8FC8, 0x7A60);
    SOC5140_write_cmos_sensor(0x8FCA, 0x1150);
    SOC5140_write_cmos_sensor(0x8FCC, 0x0080);
    SOC5140_write_cmos_sensor(0x8FCE, 0x02F9);
    SOC5140_write_cmos_sensor(0x8FD0, 0x0684);
    SOC5140_write_cmos_sensor(0x8FD2, 0x78E0);
    SOC5140_write_cmos_sensor(0x8FD4, 0xC0F1);
    SOC5140_write_cmos_sensor(0x8FD6, 0x0A7E);
    SOC5140_write_cmos_sensor(0x8FD8, 0x0684);
    SOC5140_write_cmos_sensor(0x8FDA, 0xD622);
    SOC5140_write_cmos_sensor(0x8FDC, 0x8EA9);
    SOC5140_write_cmos_sensor(0x8FDE, 0x8E2A);
    SOC5140_write_cmos_sensor(0x8FE0, 0xBD08);
    SOC5140_write_cmos_sensor(0x8FE2, 0x7D25);
    SOC5140_write_cmos_sensor(0x8FE4, 0x2550);
    SOC5140_write_cmos_sensor(0x8FE6, 0x10C2);
    SOC5140_write_cmos_sensor(0x8FE8, 0x2A41);
    SOC5140_write_cmos_sensor(0x8FEA, 0x0201);
    SOC5140_write_cmos_sensor(0x8FEC, 0xAE29);
    SOC5140_write_cmos_sensor(0x8FEE, 0x0F9A);
    SOC5140_write_cmos_sensor(0x8FF0, 0x05A4);
    SOC5140_write_cmos_sensor(0x8FF2, 0xAE4A);
    SOC5140_write_cmos_sensor(0x8FF4, 0x0D17);
    SOC5140_write_cmos_sensor(0x8FF6, 0x10DE);
    SOC5140_write_cmos_sensor(0x8FF8, 0x8E09);
    SOC5140_write_cmos_sensor(0x8FFA, 0x8E2A);
    SOC5140_write_cmos_sensor(0x8FFC, 0xB808);
    SOC5140_write_cmos_sensor(0x8FFE, 0x7825);
    SOC5140_write_cmos_sensor(0x9000, 0xB883);
    SOC5140_write_cmos_sensor(0x9002, 0x2841);
    SOC5140_write_cmos_sensor(0x9004, 0x0201);
    SOC5140_write_cmos_sensor(0x9006, 0xAE29);
    SOC5140_write_cmos_sensor(0x9008, 0xAE0A);
    SOC5140_write_cmos_sensor(0x900A, 0x02B5);
    SOC5140_write_cmos_sensor(0x900C, 0x0684);
    SOC5140_write_cmos_sensor(0x900E, 0x78E0);
    SOC5140_write_cmos_sensor(0x9010, 0xC0F1);
    SOC5140_write_cmos_sensor(0x9012, 0x0A42);
    SOC5140_write_cmos_sensor(0x9014, 0x06A4);
    SOC5140_write_cmos_sensor(0x9016, 0xDA34);
    SOC5140_write_cmos_sensor(0x9018, 0xD113);
    SOC5140_write_cmos_sensor(0x901A, 0xD514);
    SOC5140_write_cmos_sensor(0x901C, 0x76A9);
    SOC5140_write_cmos_sensor(0x901E, 0x0FD6);
    SOC5140_write_cmos_sensor(0x9020, 0x0664);
    SOC5140_write_cmos_sensor(0x9022, 0x70C9);
    SOC5140_write_cmos_sensor(0x9024, 0xD012);
    SOC5140_write_cmos_sensor(0x9026, 0xA504);
    SOC5140_write_cmos_sensor(0x9028, 0xD012);
    SOC5140_write_cmos_sensor(0x902A, 0x0295);
    SOC5140_write_cmos_sensor(0x902C, 0x06A4);
    SOC5140_write_cmos_sensor(0x902E, 0xA0C0);
    SOC5140_write_cmos_sensor(0x9030, 0xC0F1);
    SOC5140_write_cmos_sensor(0x9032, 0xC5E1);
    SOC5140_write_cmos_sensor(0x9034, 0xD50D);
    SOC5140_write_cmos_sensor(0x9036, 0xD110);
    SOC5140_write_cmos_sensor(0x9038, 0x2540);
    SOC5140_write_cmos_sensor(0x903A, 0x1D00);
    SOC5140_write_cmos_sensor(0x903C, 0x0FB6);
    SOC5140_write_cmos_sensor(0x903E, 0x0664);
    SOC5140_write_cmos_sensor(0x9040, 0xDA50);
    SOC5140_write_cmos_sensor(0x9042, 0xD00E);
    SOC5140_write_cmos_sensor(0x9044, 0x2540);
    SOC5140_write_cmos_sensor(0x9046, 0x1D01);
    SOC5140_write_cmos_sensor(0x9048, 0xA517);
    SOC5140_write_cmos_sensor(0x904A, 0xD00D);
    SOC5140_write_cmos_sensor(0x904C, 0x0279);
    SOC5140_write_cmos_sensor(0x904E, 0x06A4);
    SOC5140_write_cmos_sensor(0x9050, 0xA020);
    SOC5140_write_cmos_sensor(0x9052, 0x78E0);
    SOC5140_write_cmos_sensor(0x9054, 0xFF80);
    SOC5140_write_cmos_sensor(0x9056, 0x07A8);
    SOC5140_write_cmos_sensor(0x9058, 0xFF80);
    SOC5140_write_cmos_sensor(0x905A, 0x0290);
    SOC5140_write_cmos_sensor(0x905C, 0x8000);
    SOC5140_write_cmos_sensor(0x905E, 0x0008);
    SOC5140_write_cmos_sensor(0x9060, 0xFF80);
    SOC5140_write_cmos_sensor(0x9062, 0x02CC);
    SOC5140_write_cmos_sensor(0x9064, 0x0000);
    SOC5140_write_cmos_sensor(0x9066, 0xFA88);
    SOC5140_write_cmos_sensor(0x9068, 0xFF80);
    SOC5140_write_cmos_sensor(0x906A, 0x1168);
    SOC5140_write_cmos_sensor(0x906C, 0xFF80);
    SOC5140_write_cmos_sensor(0x906E, 0x0FD4);
    SOC5140_write_cmos_sensor(0x9070, 0x8000);
    SOC5140_write_cmos_sensor(0x9072, 0x0194);
    SOC5140_write_cmos_sensor(0x9074, 0x0000);
    SOC5140_write_cmos_sensor(0x9076, 0xFB08);
    SOC5140_write_cmos_sensor(0x9078, 0xFF80);
    SOC5140_write_cmos_sensor(0x907A, 0x0FA0);
    SOC5140_write_cmos_sensor(0x907C, 0x8000);
    SOC5140_write_cmos_sensor(0x907E, 0x01A0);
    SOC5140_write_cmos_sensor(0x9080, 0xE280);
    SOC5140_write_cmos_sensor(0x9082, 0x24CA);
    SOC5140_write_cmos_sensor(0x9084, 0x7082);
    SOC5140_write_cmos_sensor(0x9086, 0x78E0);
    SOC5140_write_cmos_sensor(0x9088, 0x20E8);
    SOC5140_write_cmos_sensor(0x908A, 0x01A2);
    SOC5140_write_cmos_sensor(0x908C, 0x1002);
    SOC5140_write_cmos_sensor(0x908E, 0x0D02);
    SOC5140_write_cmos_sensor(0x9090, 0x1902);
    SOC5140_write_cmos_sensor(0x9092, 0x0094);
    SOC5140_write_cmos_sensor(0x9094, 0x7FE0);
    SOC5140_write_cmos_sensor(0x9096, 0x7028);
    SOC5140_write_cmos_sensor(0x9098, 0x7308);
    SOC5140_write_cmos_sensor(0x909A, 0x1000);
    SOC5140_write_cmos_sensor(0x909C, 0x0900);
    SOC5140_write_cmos_sensor(0x909E, 0x7904);
    SOC5140_write_cmos_sensor(0x90A0, 0x7947);
    SOC5140_write_cmos_sensor(0x90A2, 0x1B00);
    SOC5140_write_cmos_sensor(0x90A4, 0x0064);
    SOC5140_write_cmos_sensor(0x90A6, 0x7EE0);
    SOC5140_write_cmos_sensor(0x90A8, 0xE280);
    SOC5140_write_cmos_sensor(0x90AA, 0x24CA);
    SOC5140_write_cmos_sensor(0x90AC, 0x7082);
    SOC5140_write_cmos_sensor(0x90AE, 0x78E0);
    SOC5140_write_cmos_sensor(0x90B0, 0x20E8);
    SOC5140_write_cmos_sensor(0x90B2, 0x01A2);
    SOC5140_write_cmos_sensor(0x90B4, 0x1102);
    SOC5140_write_cmos_sensor(0x90B6, 0x0502);
    SOC5140_write_cmos_sensor(0x90B8, 0x1802);
    SOC5140_write_cmos_sensor(0x90BA, 0x00B4);
    SOC5140_write_cmos_sensor(0x90BC, 0x7FE0);
    SOC5140_write_cmos_sensor(0x90BE, 0x7028);
    SOC5140_write_cmos_sensor(0x90C0, 0x0000);
    SOC5140_write_cmos_sensor(0x90C2, 0x0000);
    SOC5140_write_cmos_sensor(0x90C4, 0x0000);
    SOC5140_write_cmos_sensor(0x90C6, 0x0000);
    SOC5140_write_cmos_sensor(0x098E, 0x0000); 	        // return variable access to logical mode
    SOC5140_write_cmos_sensor(0x8016, 0x086C); 	        // MON_ADDRESS_LO
    SOC5140_write_cmos_sensor(0x8018, 0xFF80); 	        // MON_ADDRESS_HI
    SOC5140_write_cmos_sensor(0x8002, 0x0001); 	        // MON_CMD
    mDELAY(10);

    //[**********Step4*************]
    //[Step4-Timing]
    SOC5140_write_cmos_sensor(0xC86C, 0x0518);
    SOC5140_write_cmos_sensor(0xC86E, 0x03D4);
    SOC5140_write_cmos_sensor(0xC83A, 0x000C);
    SOC5140_write_cmos_sensor(0xC83C, 0x0018);
    SOC5140_write_cmos_sensor(0xC83E, 0x07B1);
    SOC5140_write_cmos_sensor(0xC840, 0x0A45);
    SOC5140_write_cmos_sensor(0xC842, 0x0001);
    SOC5140_write_cmos_sensor(0xC844, 0x0103);
    SOC5140_write_cmos_sensor(0xC846, 0x0103);
    SOC5140_write_cmos_sensor(0xC848, 0x0103);
    SOC5140_write_cmos_sensor(0xC84A, 0x0103);
    SOC5140_write_cmos_sensor(0xC84C, 0x00F6);
    SOC5140_write_cmos_sensor(0xC84E, 0x0001);
    SOC5140_write_cmos_sensor_8(0xC850, 0x00);
    SOC5140_write_cmos_sensor_8(0xC851, 0x00);
    SOC5140_write_cmos_sensor(0xC852, 0x019C);
    SOC5140_write_cmos_sensor(0xC854, 0x0732);
    SOC5140_write_cmos_sensor(0xC856, 0x048E);
    SOC5140_write_cmos_sensor(0xC858, 0x0002);
    SOC5140_write_cmos_sensor(0xC85A, 0x0001);
    SOC5140_write_cmos_sensor(0xC85C, 0x0423);
    SOC5140_write_cmos_sensor(0xC85E, 0xFFFF);
    SOC5140_write_cmos_sensor(0xC860, 0x0423);
    SOC5140_write_cmos_sensor(0xC862, 0x0E87);
    SOC5140_write_cmos_sensor(0xC864, 0xFFFE);
    SOC5140_write_cmos_sensor(0xC866, 0x7F7C);
    SOC5140_write_cmos_sensor(0xC868, 0x0423);
    SOC5140_write_cmos_sensor(0xC86A, 0x0E87);
    SOC5140_write_cmos_sensor(0xC870, 0x0014);
    SOC5140_write_cmos_sensor(0xC8AA, 0x0280);
    SOC5140_write_cmos_sensor(0xC8AC, 0x01E0);
    SOC5140_write_cmos_sensor(0xC8AE, 0x0001);
    SOC5140_write_cmos_sensor(0xC8B0, 0x0002);
    SOC5140_write_cmos_sensor(0xC8B8, 0x0004);
    SOC5140_write_cmos_sensor(0xC8A4, 0x0A28);
    SOC5140_write_cmos_sensor(0xC8A6, 0x07A0);
    SOC5140_write_cmos_sensor(0xC872, 0x0010);
    SOC5140_write_cmos_sensor(0xC874, 0x001C);
    SOC5140_write_cmos_sensor(0xC876, 0x07AF);
    SOC5140_write_cmos_sensor(0xC878, 0x0A43);
    SOC5140_write_cmos_sensor(0xC87A, 0x0001);
    SOC5140_write_cmos_sensor(0xC87C, 0x0101);
    SOC5140_write_cmos_sensor(0xC87E, 0x0101);
    SOC5140_write_cmos_sensor(0xC880, 0x0101);
    SOC5140_write_cmos_sensor(0xC882, 0x0101);
    SOC5140_write_cmos_sensor(0xC884, 0x00F2);
    SOC5140_write_cmos_sensor(0xC886, 0x0000);
    SOC5140_write_cmos_sensor_8(0xC888, 0x00);
    SOC5140_write_cmos_sensor_8(0xC889, 0x00);
    SOC5140_write_cmos_sensor(0xC88A, 0x009C);
    SOC5140_write_cmos_sensor(0xC88C, 0x034A);
    SOC5140_write_cmos_sensor(0xC88E, 0x02A6);
    SOC5140_write_cmos_sensor(0xC890, 0x0002);
    SOC5140_write_cmos_sensor(0xC892, 0x0001);
    SOC5140_write_cmos_sensor(0xC894, 0x07EF);
    SOC5140_write_cmos_sensor(0xC896, 0xFFFF);
    SOC5140_write_cmos_sensor(0xC898, 0x07EF);
    SOC5140_write_cmos_sensor(0xC89A, 0x1E48);
    SOC5140_write_cmos_sensor(0xC89C, 0xFFFE);
    SOC5140_write_cmos_sensor(0xC89E, 0x7F7C);
    SOC5140_write_cmos_sensor(0xC8A0, 0x07EF);
    SOC5140_write_cmos_sensor(0xC8A2, 0x1E48);
    SOC5140_write_cmos_sensor(0xC8A8, 0x0014);
    SOC5140_write_cmos_sensor(0xC8C0, 0x0A20);
    SOC5140_write_cmos_sensor(0xC8C2, 0x0798);
    SOC5140_write_cmos_sensor(0xC8C4, 0x0001);
    SOC5140_write_cmos_sensor(0xC8C6, 0x0002);
    SOC5140_write_cmos_sensor(0xC8CE, 0x0004);
    SOC5140_write_cmos_sensor(0xA010, 0x0134);
    SOC5140_write_cmos_sensor(0xA012, 0x0148);
    SOC5140_write_cmos_sensor(0xA014, 0x00FF);
    SOC5140_write_cmos_sensor(0xA016, 0x0113);
    SOC5140_write_cmos_sensor(0xA018, 0x013E);
    SOC5140_write_cmos_sensor(0xA01A, 0x0098);
    SOC5140_write_cmos_sensor(0xA01C, 0x0109);
    SOC5140_write_cmos_sensor(0xA01E, 0x007F);
    /// kevin add @20130613 start
	  SOC5140_write_cmos_sensor(0xC862, 0x1146);	//Min Line Length (A) = 3719
	  SOC5140_write_cmos_sensor(0xC86A, 0x1146);	//Line Length (A) = 3719
	  SOC5140_write_cmos_sensor(0xA010, 0x0101);	//fd_min_expected50hz_flicker_period = 257
	  SOC5140_write_cmos_sensor(0xA012, 0x0115);	//fd_max_expected50hz_flicker_period = 277
	  SOC5140_write_cmos_sensor(0xA014, 0x00D5);	//fd_min_expected60hz_flicker_period = 213
	  SOC5140_write_cmos_sensor(0xA016, 0x00E9);	//fd_max_expected60hz_flicker_period = 233
	  SOC5140_write_cmos_sensor(0xA018, 0x010b);	//fd_expected50hz_flicker_period (A) = 267
	  SOC5140_write_cmos_sensor(0xA01A, 0x0098);	//fd_expected50hz_flicker_period (B) = 152
	  SOC5140_write_cmos_sensor(0xA01C, 0x00DF);	//fd_expected60hz_flicker_period (A) = 223
	  SOC5140_write_cmos_sensor(0xA01E, 0x007F);	//fd_expected60hz_flicker_period (B) = 127
     /// kevin add @20130613 end
    SOC5140_write_cmos_sensor_8(0xDC0A, 0x06);
    SOC5140_write_cmos_sensor(0xDC1C, 0x2710);


    //[**********step5*************]
    //The PGA solution is included here.

    //[**********step6*************]
    SOC5140_write_cmos_sensor(0xAC00, 0x00BB);  //AWB_STATUS - default, AWB_MODE

    SOC5140_write_cmos_sensor(0xAC46, 0x0221); 	         // AWB_LEFT_CCM_0
    SOC5140_write_cmos_sensor(0xAC48, 0xFEAE); 	         // AWB_LEFT_CCM_1
    SOC5140_write_cmos_sensor(0xAC4A, 0x0032); 	         // AWB_LEFT_CCM_2
    SOC5140_write_cmos_sensor(0xAC4C, 0xFFC5); 	         // AWB_LEFT_CCM_3
    SOC5140_write_cmos_sensor(0xAC4E, 0x0154); 	         // AWB_LEFT_CCM_4
    SOC5140_write_cmos_sensor(0xAC50, 0xFFE7); 	         // AWB_LEFT_CCM_5
    SOC5140_write_cmos_sensor(0xAC52, 0xFFB1); 	         // AWB_LEFT_CCM_6
    SOC5140_write_cmos_sensor(0xAC54, 0xFEC5); 	         // AWB_LEFT_CCM_7
    SOC5140_write_cmos_sensor(0xAC56, 0x028A); 	         // AWB_LEFT_CCM_8
    SOC5140_write_cmos_sensor(0xAC58, 0x00C6); 	         // AWB_LEFT_CCM_R2BRATIO - AWB tuning 
    SOC5140_write_cmos_sensor(0xAC5C, 0x01CD); 	         // AWB_RIGHT_CCM_0
    SOC5140_write_cmos_sensor(0xAC5E, 0xFF63); 	         // AWB_RIGHT_CCM_1
    SOC5140_write_cmos_sensor(0xAC60, 0xFFD0); 	         // AWB_RIGHT_CCM_2
    SOC5140_write_cmos_sensor(0xAC62, 0xFFCD); 	         // AWB_RIGHT_CCM_3
    SOC5140_write_cmos_sensor(0xAC64, 0x013B); 	         // AWB_RIGHT_CCM_4
    SOC5140_write_cmos_sensor(0xAC66, 0xFFF8); 	         // AWB_RIGHT_CCM_5
    SOC5140_write_cmos_sensor(0xAC68, 0xFFFB); 	         // AWB_RIGHT_CCM_6
    SOC5140_write_cmos_sensor(0xAC6A, 0xFF78); 	         // AWB_RIGHT_CCM_7
    SOC5140_write_cmos_sensor(0xAC6C, 0x018D); 	         // AWB_RIGHT_CCM_8
    SOC5140_write_cmos_sensor(0xAC6E, 0x0055); 	         // AWB_RIGHT_CCM_R2BRATIO - AWB tuning
    SOC5140_write_cmos_sensor(0xB842, 0x0037); 	         // STAT_AWB_GRAY_CHECKER_OFFSET_X
    SOC5140_write_cmos_sensor(0xB844, 0x0044); 	         // STAT_AWB_GRAY_CHECKER_OFFSET_Y
    SOC5140_write_cmos_sensor(0x3240, 0x0024); 	         // AWB_XY_SCALE
    SOC5140_write_cmos_sensor(0x3240, 0x0024); 	         // AWB_XY_SCALE
    SOC5140_write_cmos_sensor(0x3242, 0x0000); 	         // AWB_WEIGHT_R0
    SOC5140_write_cmos_sensor(0x3244, 0x0000); 	         // AWB_WEIGHT_R1
    SOC5140_write_cmos_sensor(0x3246, 0x01AC); 	         // AWB_WEIGHT_R2
    SOC5140_write_cmos_sensor(0x3248, 0x7F00); 	         // AWB_WEIGHT_R3
    SOC5140_write_cmos_sensor(0x324A, 0xA500); 	         // AWB_WEIGHT_R4
    SOC5140_write_cmos_sensor(0x324C, 0x1540); 	         // AWB_WEIGHT_R5
    SOC5140_write_cmos_sensor(0x324E, 0x01AC); 	         // AWB_WEIGHT_R6
    SOC5140_write_cmos_sensor(0x3250, 0x003E); 	         // AWB_WEIGHT_R7
    //REG_BURST= 0xAC3C, 0x3E79    															 // AWB_MIN_ACCEPTED_PRE_AWB_R2G_RATIO, AWB_MAX_ACCEPTED_PRE_AWB_R2G_RATIO - unnecessary
    //REG_BURST= 0xAC3E, 0x225E                                  // AWB_MIN_ACCEPTED_PRE_AWB_B2G_RATIO, AWB_MAX_ACCEPTED_PRE_AWB_B2G_RATIO - unnecessary
    SOC5140_write_cmos_sensor_8(0xACB0, 0x32);              // AWB_RG_MIN, AWB_RG_MAX - AWB tuning
    SOC5140_write_cmos_sensor_8(0xACB1, 0x5A);              // AWB_RG_MIN_BRIGHT, AWB_RG_MAX_BRIGHT - AWB tuning
    SOC5140_write_cmos_sensor_8(0xACB2, 0x32);              // AWB_BG_MIN, AWB_BG_MAX - AWB tuning
    SOC5140_write_cmos_sensor_8(0xACB3, 0x5A);              // AWB_BG_MIN_BRIGHT, AWB_BG_MAX_BRIGHT - AWB tuning
    SOC5140_write_cmos_sensor_8(0xACB4, 0x23); 	           // AWB_START_NUM_INT_LINES
    SOC5140_write_cmos_sensor_8(0xACB5, 0x55);   	         // AWB_END_NUM_INT_LINES
    SOC5140_write_cmos_sensor_8(0xACB6, 0x44);
    SOC5140_write_cmos_sensor_8(0xACB7, 0x55);
    SOC5140_write_cmos_sensor(0xACB8, 0x004B);
    SOC5140_write_cmos_sensor(0xACBA, 0x0019);

    //[**********Step7*************]
    //[Step7-CPIPE_Calibration]
    SOC5140_write_cmos_sensor_8(0xD80F, 0x04);	        // JPEG_QSCALE_0
    SOC5140_write_cmos_sensor_8(0xD810, 0x0A);        // JPEG_QSCALE_1
    SOC5140_write_cmos_sensor_8(0xC8D2, 0x04);        // CAM_OUTPUT_1_JPEG_QSCALE_0
    SOC5140_write_cmos_sensor_8(0xC8D3, 0x0A);        // CAM_OUTPUT_1_JPEG_QSCALE_1
    SOC5140_write_cmos_sensor_8(0xC8BC, 0x04);        // CAM_OUTPUT_0_JPEG_QSCALE_0
    SOC5140_write_cmos_sensor_8(0xC8BD, 0x0A);          // CAM_OUTPUT_0_JPEG_QSCALE_1
    SOC5140_write_cmos_sensor(0x326E, 0x0006);	        // LOW_PASS_YUV_FILTER
    SOC5140_write_cmos_sensor_8(0xDC36, 0x23);        // SYS_DARK_COLOR_KILL
    SOC5140_write_cmos_sensor_8(0xDC37, 0x62);        // SYS_BRIGHT_COLORKILL
    SOC5140_write_cmos_sensor(0x35A4, 0x0596);	        // BRIGHT_COLOR_KILL_CONTROLS
    SOC5140_write_cmos_sensor(0x35A2, 0x0094);	        // DARK_COLOR_KILL_CONTROLS

    //[Gamma_Curves_REV3]
    SOC5140_write_cmos_sensor_8(0xBC18, 0x00);         // LL_GAMMA_CONTRAST_CURVE_0
    SOC5140_write_cmos_sensor_8(0xBC19, 0x11);         // LL_GAMMA_CONTRAST_CURVE_1
    SOC5140_write_cmos_sensor_8(0xBC1A, 0x23);         // LL_GAMMA_CONTRAST_CURVE_2
    SOC5140_write_cmos_sensor_8(0xBC1B, 0x3F);         // LL_GAMMA_CONTRAST_CURVE_3
    SOC5140_write_cmos_sensor_8(0xBC1C, 0x67);         // LL_GAMMA_CONTRAST_CURVE_4
    SOC5140_write_cmos_sensor_8(0xBC1D, 0x85);         // LL_GAMMA_CONTRAST_CURVE_5
    SOC5140_write_cmos_sensor_8(0xBC1E, 0x9B);         // LL_GAMMA_CONTRAST_CURVE_6
    SOC5140_write_cmos_sensor_8(0xBC1F, 0xAD);         // LL_GAMMA_CONTRAST_CURVE_7
    SOC5140_write_cmos_sensor_8(0xBC20, 0xBB);         // LL_GAMMA_CONTRAST_CURVE_8
    SOC5140_write_cmos_sensor_8(0xBC21, 0xC7);         // LL_GAMMA_CONTRAST_CURVE_9
    SOC5140_write_cmos_sensor_8(0xBC22, 0xD1);         // LL_GAMMA_CONTRAST_CURVE_10
    SOC5140_write_cmos_sensor_8(0xBC23, 0xDA);         // LL_GAMMA_CONTRAST_CURVE_11
    SOC5140_write_cmos_sensor_8(0xBC24, 0xE1);         // LL_GAMMA_CONTRAST_CURVE_12
    SOC5140_write_cmos_sensor_8(0xBC25, 0xE8);	         // LL_GAMMA_CONTRAST_CURVE_13
    SOC5140_write_cmos_sensor_8(0xBC26, 0xEE);         // LL_GAMMA_CONTRAST_CURVE_14
    SOC5140_write_cmos_sensor_8(0xBC27, 0xF3);         // LL_GAMMA_CONTRAST_CURVE_15
    SOC5140_write_cmos_sensor_8(0xBC28, 0xF7);         // LL_GAMMA_CONTRAST_CURVE_16
    SOC5140_write_cmos_sensor_8(0xBC29, 0xFB);         // LL_GAMMA_CONTRAST_CURVE_17
    SOC5140_write_cmos_sensor_8(0xBC2A, 0xFF);         // LL_GAMMA_CONTRAST_CURVE_18
    SOC5140_write_cmos_sensor_8(0xBC2B, 0x00);         // LL_GAMMA_NEUTRAL_CURVE_0
    SOC5140_write_cmos_sensor_8(0xBC2C, 0x11);         // LL_GAMMA_NEUTRAL_CURVE_1
    SOC5140_write_cmos_sensor_8(0xBC2D, 0x23);         // LL_GAMMA_NEUTRAL_CURVE_2
    SOC5140_write_cmos_sensor_8(0xBC2E, 0x3F);         // LL_GAMMA_NEUTRAL_CURVE_3
    SOC5140_write_cmos_sensor_8(0xBC2F, 0x67);         // LL_GAMMA_NEUTRAL_CURVE_4
    SOC5140_write_cmos_sensor_8(0xBC30, 0x85);         // LL_GAMMA_NEUTRAL_CURVE_5
    SOC5140_write_cmos_sensor_8(0xBC31, 0x9B);         // LL_GAMMA_NEUTRAL_CURVE_6
    SOC5140_write_cmos_sensor_8(0xBC32, 0xAD);         // LL_GAMMA_NEUTRAL_CURVE_7
    SOC5140_write_cmos_sensor_8(0xBC33, 0xBB);         // LL_GAMMA_NEUTRAL_CURVE_8
    SOC5140_write_cmos_sensor_8(0xBC34, 0xC7);         // LL_GAMMA_NEUTRAL_CURVE_9
    SOC5140_write_cmos_sensor_8(0xBC35, 0xD1);         // LL_GAMMA_NEUTRAL_CURVE_10
    SOC5140_write_cmos_sensor_8(0xBC36, 0xDA);         // LL_GAMMA_NEUTRAL_CURVE_11
    SOC5140_write_cmos_sensor_8(0xBC37, 0xE1);         // LL_GAMMA_NEUTRAL_CURVE_12
    SOC5140_write_cmos_sensor_8(0xBC38, 0xE8);         // LL_GAMMA_NEUTRAL_CURVE_13
    SOC5140_write_cmos_sensor_8(0xBC39, 0xEE);         // LL_GAMMA_NEUTRAL_CURVE_14
    SOC5140_write_cmos_sensor_8(0xBC3A, 0xF3);         // LL_GAMMA_NEUTRAL_CURVE_15
    SOC5140_write_cmos_sensor_8(0xBC3B, 0xF7);         // LL_GAMMA_NEUTRAL_CURVE_16
    SOC5140_write_cmos_sensor_8(0xBC3C, 0xFB);         // LL_GAMMA_NEUTRAL_CURVE_17
    SOC5140_write_cmos_sensor_8(0xBC3D, 0xFF);         // LL_GAMMA_NEUTRAL_CURVE_18
    SOC5140_write_cmos_sensor_8(0xBC3E, 0x00);         // LL_GAMMA_NR_CURVE_0
    SOC5140_write_cmos_sensor_8(0xBC3F, 0x05);         // LL_GAMMA_NR_CURVE_1
    SOC5140_write_cmos_sensor_8(0xBC40, 0x0F);         // LL_GAMMA_NR_CURVE_2
    SOC5140_write_cmos_sensor_8(0xBC41, 0x21);         // LL_GAMMA_NR_CURVE_3
    SOC5140_write_cmos_sensor_8(0xBC42, 0x3C);         // LL_GAMMA_NR_CURVE_4
    SOC5140_write_cmos_sensor_8(0xBC43, 0x52);         // LL_GAMMA_NR_CURVE_5
    SOC5140_write_cmos_sensor_8(0xBC44, 0x67);         // LL_GAMMA_NR_CURVE_6
    SOC5140_write_cmos_sensor_8(0xBC45, 0x7B);         // LL_GAMMA_NR_CURVE_7
    SOC5140_write_cmos_sensor_8(0xBC46, 0x8D);         // LL_GAMMA_NR_CURVE_8
    SOC5140_write_cmos_sensor_8(0xBC47, 0x9E);         // LL_GAMMA_NR_CURVE_9
    SOC5140_write_cmos_sensor_8(0xBC48, 0xAD);         // LL_GAMMA_NR_CURVE_10
    SOC5140_write_cmos_sensor_8(0xBC49, 0xBA);         // LL_GAMMA_NR_CURVE_11
    SOC5140_write_cmos_sensor_8(0xBC4A, 0xC6);         // LL_GAMMA_NR_CURVE_12
    SOC5140_write_cmos_sensor_8(0xBC4B, 0xD1);         //  LL_GAMMA_NR_CURVE_13
    SOC5140_write_cmos_sensor_8(0xBC4C, 0xDC);         // LL_GAMMA_NR_CURVE_14
    SOC5140_write_cmos_sensor_8(0xBC4D, 0xE5);         //  LL_GAMMA_NR_CURVE_15
    SOC5140_write_cmos_sensor_8(0xBC4E, 0xEE);         // LL_GAMMA_NR_CURVE_16
    SOC5140_write_cmos_sensor_8(0xBC4F, 0xF7);         //  LL_GAMMA_NR_CURVE_17
    SOC5140_write_cmos_sensor_8(0xBC50, 0xFF);         // LL_GAMMA_NR_CURVE_18
    SOC5140_write_cmos_sensor_8(0xB801, 0xE0);         // STAT_MODE
    SOC5140_write_cmos_sensor_8(0xB829, 0x02);	         // STAT_LL_BRIGHTNESS_METRIC_DIVISOR
    SOC5140_write_cmos_sensor_8(0xB862, 0x04);         // STAT_BMTRACKING_SPEED
    SOC5140_write_cmos_sensor_8(0xB863, 0x02);         // STAT_BM_MUL
    SOC5140_write_cmos_sensor_8(0xA409, 0x37);         // AE_RULE_BASE_TARGET
                                                     
    //[BM_GM_Start_Stop]
    SOC5140_write_cmos_sensor(0xBC52, 0x00C8); 	        // LL_START_BRIGHTNESS_METRIC
    SOC5140_write_cmos_sensor(0xBC54, 0x0A28); 	        // LL_END_BRIGHTNESS_METRIC
    SOC5140_write_cmos_sensor(0xBC58, 0x00C8); 	        // LL_START_GAIN_METRIC
    SOC5140_write_cmos_sensor(0xBC5A, 0x12C0); 	        // LL_END_GAIN_METRIC
    SOC5140_write_cmos_sensor(0xBC5E, 0x00FA); 	        // LL_START_APERTURE_GAIN_BM
    SOC5140_write_cmos_sensor(0xBC60, 0x0258); 	        // LL_END_APERTURE_GAIN_BM
    SOC5140_write_cmos_sensor(0xBC66, 0x00FA); 	        // LL_START_APERTURE_GM
    SOC5140_write_cmos_sensor(0xBC68, 0x0258); 	        // LL_END_APERTURE_GM
    SOC5140_write_cmos_sensor(0xBC86, 0x00C8); 	        // LL_START_FFNR_GM
    SOC5140_write_cmos_sensor(0xBC88, 0x0640); 	        // LL_END_FFNR_GM
    SOC5140_write_cmos_sensor(0xBCBC, 0x0040); 	        // LL_SFFB_START_GAIN
    SOC5140_write_cmos_sensor(0xBCBE, 0x01FC); 	        // LL_SFFB_END_GAIN
    SOC5140_write_cmos_sensor(0xBCCC, 0x00C8); 	        // LL_SFFB_START_MAX_GM
    SOC5140_write_cmos_sensor(0xBCCE, 0x0640); 	        // LL_SFFB_END_MAX_GM
    SOC5140_write_cmos_sensor(0xBC90, 0x00C8); 	        // LL_START_GRB_GM
    SOC5140_write_cmos_sensor(0xBC92, 0x0640); 	        // LL_END_GRB_GM
    SOC5140_write_cmos_sensor(0xBC0E, 0x0001); 	        // LL_GAMMA_CURVE_ADJ_START_POS
    SOC5140_write_cmos_sensor(0xBC10, 0x0002); 	        // LL_GAMMA_CURVE_ADJ_MID_POS
    SOC5140_write_cmos_sensor(0xBC12, 0x02BC); 	        // LL_GAMMA_CURVE_ADJ_END_POS
    SOC5140_write_cmos_sensor(0xBCAA, 0x044C); 	        // LL_CDC_THR_ADJ_START_POS
    SOC5140_write_cmos_sensor(0xBCAC, 0x00AF); 	        // LL_CDC_THR_ADJ_MID_POS
    SOC5140_write_cmos_sensor(0xBCAE, 0x0009); 	        // LL_CDC_THR_ADJ_END_POS
    SOC5140_write_cmos_sensor(0xBCD8, 0x00C8); 	        // LL_PCR_START_BM
    SOC5140_write_cmos_sensor(0xBCDA, 0x0A28); 	        // LL_PCR_END_BM
    SOC5140_write_cmos_sensor(0x3380, 0x0504); 	        // KERNEL_CONFIG
    SOC5140_write_cmos_sensor_8(0xBC94, 0x0C);           // LL_GB_START_THRESHOLD_0
    SOC5140_write_cmos_sensor_8(0xBC95, 0x08);           // LL_GB_START_THRESHOLD_1
    SOC5140_write_cmos_sensor_8(0xBC9C, 0x3C);	         // LL_GB_END_THRESHOLD_0
    SOC5140_write_cmos_sensor_8(0xBC9D, 0x28);          // LL_GB_END_THRESHOLD_1

    //[Demosaic_REV3]
    SOC5140_write_cmos_sensor(0x33B0, 0x2A16);              // FFNR_ALPHA_BETA
    SOC5140_write_cmos_sensor_8(0xBC8A, 0x02);	           // LL_START_FF_MIX_THRESH_Y
    SOC5140_write_cmos_sensor_8(0xBC8B, 0x0F);	           // LL_END_FF_MIX_THRESH_Y
    SOC5140_write_cmos_sensor_8(0xBC8C, 0xFF);	           // LL_START_FF_MIX_THRESH_YGAIN
    SOC5140_write_cmos_sensor_8(0xBC8D, 0xFF);	           // LL_END_FF_MIX_THRESH_YGAIN
    SOC5140_write_cmos_sensor_8(0xBC8E, 0xFF);	           // LL_START_FF_MIX_THRESH_GAIN
    SOC5140_write_cmos_sensor_8(0xBC8F, 0x00);	           // LL_END_FF_MIX_THRESH_GAIN
                                     
    //[CDC]
    SOC5140_write_cmos_sensor_8(0xBCB2, 0x20);	          // LL_CDC_DARK_CLUS_SLOPE
    SOC5140_write_cmos_sensor_8(0xBCB3, 0x3A);	          // LL_CDC_DARK_CLUS_SATUR
    SOC5140_write_cmos_sensor_8(0xBCB4, 0x39);	          // LL_CDC_BRIGHT_CLUS_LO_LIGHT_SLOPE
    SOC5140_write_cmos_sensor_8(0xBCB5, 0x20);	          // LL_CDC_BRIGHT_CLUS_MID_LIGHT_SLOPE
    SOC5140_write_cmos_sensor_8(0xBCB6, 0x80);	          // LL_CDC_BRIGHT_CLUS_HI_LIGHT_SLOPE
    SOC5140_write_cmos_sensor_8(0xBCB7, 0x39);	          // LL_CDC_BRIGHT_CLUS_LO_LIGHT_SATUR
    SOC5140_write_cmos_sensor_8(0xBCB8, 0x3A);	          // LL_CDC_BRIGHT_CLUS_MID_LIGHT_SATUR
    SOC5140_write_cmos_sensor_8(0xBCB9, 0x24);	          // LL_CDC_BRIGHT_CLUS_HI_LIGHT_SATUR
    SOC5140_write_cmos_sensor(0xBCAA, 0x03E8); 	          // LL_CDC_THR_ADJ_START_POS
    SOC5140_write_cmos_sensor(0xBCAC, 0x012C); 	          // LL_CDC_THR_ADJ_MID_POS
    SOC5140_write_cmos_sensor(0xBCAE, 0x0009); 	          // LL_CDC_THR_ADJ_END_POS

    //[Aperture_calib]
    SOC5140_write_cmos_sensor(0x33BA, 0x0084); 	          // APEDGE_CONTROL
    SOC5140_write_cmos_sensor(0x33BE, 0x0000); 	          // UA_KNEE_L
    SOC5140_write_cmos_sensor(0x33C2, 0x8800); 	          // UA_WEIGHTS
    SOC5140_write_cmos_sensor(0xBC5E, 0x0154); 	          // LL_START_APERTURE_GAIN_BM
    SOC5140_write_cmos_sensor(0xBC60, 0x0640); 	          // LL_END_APERTURE_GAIN_BM
    SOC5140_write_cmos_sensor_8(0xBC62, 0x0E);  	        // LL_START_APERTURE_KPGAIN
    SOC5140_write_cmos_sensor_8(0xBC63, 0x14);  	        // LL_END_APERTURE_KPGAIN
    SOC5140_write_cmos_sensor_8(0xBC64, 0x0E);  	        // LL_START_APERTURE_KNGAIN
    SOC5140_write_cmos_sensor_8(0xBC65, 0x14);  	        // LL_END_APERTURE_KNGAIN
    SOC5140_write_cmos_sensor_8(0xBCE2, 0x0A);   	        // LL_START_POS_KNEE
    SOC5140_write_cmos_sensor_8(0xBCE3, 0x2B);  	        // LL_END_POS_KNEE
    SOC5140_write_cmos_sensor(0x3210, 0x01B0); 	          // COLOR_PIPELINE_CONTROL
   
    	///// APGA
   	//APGA patch RAM LSC, Zone 2, D65 or DNP, start address is 0x1580 (5504)				

	//[lens_shading_85%]
	SOC5140_write_cmos_sensor(0x0982,	0x0000);	// ACCESS_CTL_STAT	                                    
	SOC5140_write_cmos_sensor(0x098A,	0x1580);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x0150);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x64AD);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x3E31);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x946E);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xAB10);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x02B0);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x9D0C);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x6650);		                                                      
					                                                                    
	SOC5140_write_cmos_sensor(0x098A,	0x1590);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x100E);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x8F10);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x02B0);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x16CE);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x6BD0);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xB5EE);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xEF8F);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0F90);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x15A0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x870E);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x43D1);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x006E);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xCAF0);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x402C);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x85AE);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x22EF);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x180D);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x15B0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xC8B0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x362E);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x520D);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x048E);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xC9AE);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xB890);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x3149);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x148E);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x15C0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x31CF);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xDDAF);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xDD10);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x2B4C);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x9A6E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x1D0F);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x152E);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xB150);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x15D0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x4E31);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x684C);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x0AD1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x2911);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xEAD3);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x32B1);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x896F);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x9D0A);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x15E0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x0451);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xC632);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x2551);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x008F);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x0410);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x5690);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x91B3);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x64B1);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x15F0);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x81F0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x4731);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x03B2);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x9074);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x17AE);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x598A);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xA8D2);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x5AD0);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1600);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x2C93);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x34CE);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x194D);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xF931);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x1430);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x7FF2);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xEB8E);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xD86F);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x1610);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xB3F0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x1612);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x3D32);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xA7CE);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x7DCD);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xAD11);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x1011);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x7652);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1620);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x81CF);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x2611);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xE614);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xF393);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x5156);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x8911);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x3AB1);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x9A34);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x1630);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x8374);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x3436);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x9190);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x5890);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xB454);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xD6B3);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x34B6);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x164A);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x1640);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x2D52);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x9AD5);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xC314);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x0397);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x0364);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x051C);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x0000);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0000);		                                                      
					                                                                     
					                                                                      
	//APGA patch RAMSC, Zone 0, A-light, sta);rt address is 0x164C (5708)				    
	//SOC5140_write_cmos_sensor(0x0982,		00);00	// ACCESS_CTL_STAT	                                  
	SOC5140_write_cmos_sensor(0x098A,	0x164C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x00F0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x7F2C);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x33D1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x904E);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x9F70);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x01B0);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xA6ED);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x3411);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x165C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x47AE);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xDB10);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x02B0);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x4A2C);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x372F);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x804F);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x00CE);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x1030);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x166C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xB28E);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x6871);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x1CCD);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xDBF0);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x9EAA);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xC18D);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x4A0E);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xA827);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x167C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x872F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x188D);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x6C6D);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x26EF);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xCC2E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x8E90);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xAACD);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x072C);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x168C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x024F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xDC4E);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xE22F);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xFE0B);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x868E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x6FCE);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x9A0B);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xC5AD);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x169C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x62D1);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x588C);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x0010);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x0731);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xABF3);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x6351);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xE0CE);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0C10);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x16AC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x4270);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xB533);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x6A90);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x92AB);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x47B0);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x3010);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xD012);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x63D1);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x16BC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xC06F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x3311);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x5EB1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x8F54);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x136E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x8A8E);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xB4F1);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x06B1);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x16CC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x67B1);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x56AF);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x33C8);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xD651);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x4C70);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x5970);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x99AF);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xF78E);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x16DC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x6B2F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x3371);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xB0D1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x9AAE);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x3309);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xDCCF);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x1E31);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x2C10);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x16EC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xF58C);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x1991);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xC454);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xD833);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x30D6);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x9B70);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x22B1);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xD9F4);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x16FC);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xED33);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x57D6);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xAC6A);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x76B0);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x8B14);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xA373);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x1736);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x252E);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x170C);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x75B1);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x89D5);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x9AF4);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x6D56);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x033C);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x0508);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x0000);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0000);		                                                      
					                                                                      
					                                                                      
	//APGA patch RAMSC, Zone 1, CWF or TL84,); start address is 0x1718 (5912)				
	//SOC5140_write_cmos_sensor(0x0982,		00);00	// ACCESS_CTL_STAT	                                  
	SOC5140_write_cmos_sensor(0x098A,	0x1718);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x00D0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x516C);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x3FB1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x842E);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xD2B0);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x0250);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xA30D);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x67F0);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1728);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x2D8E);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x9070);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x02D0);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x5E0C);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x792F);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xE54E);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xA34D);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0FD0);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1738);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xC5EE);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x5F11);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x7DAD);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xF110);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xF52B);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xE54D);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x3B4E);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x7E6D);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1748);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x988F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x784D);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x69ED);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xF1CB);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x9C2E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xF50E);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xBE2D);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x53CC);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1758);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x6D0E);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xB9EE);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xD42F);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x9C8C);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x84AE);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x016E);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x07AE);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xF26C);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1768);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x5A91);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x0C6D);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x1810);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x3AF0);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xCCB3);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x2851);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xCA6E);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x3E0F);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x1778);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x490F);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0xF2D2);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x79B0);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x552D);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x1451);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x684F);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x9DF3);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x68F1);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1788);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xC3EF);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x1ED1);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x33D1);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x8E94);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x1C2E);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xF2AD);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xF870);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x78AD);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x1798);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x7E10);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x6D8E);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x8FEE);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xAF90);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x03D0);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x2970);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0xF8AE);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xBE0F);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x17A8);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x6FF0);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x1731);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x89B2);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x86AE);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xFA4D);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x40AF);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x588F);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0xDACF);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x17B8);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x8F4C);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x3BB0);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xDC94);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0xFDD2);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x4E36);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xA310);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x7F10);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x9954);		                                                      
					                                                                      
	SOC5140_write_cmos_sensor(0x098A,	0x17C8);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0xB633);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x2FD6);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0xB4AD);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x2AD0);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0xB5B4);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0xFB12);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x3816);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x700C);		                                                      
					                                                                     
	SOC5140_write_cmos_sensor(0x098A,	0x17D8);	// Patch RAM Address	                                  
	SOC5140_write_cmos_sensor(0x0990,	0x6A31);		                                                      
	SOC5140_write_cmos_sensor(0x0992,	0x8CB5);		                                                      
	SOC5140_write_cmos_sensor(0x0994,	0x80B4);		                                                      
	SOC5140_write_cmos_sensor(0x0996,	0x78D6);		                                                      
	SOC5140_write_cmos_sensor(0x0998,	0x033C);		                                                      
	SOC5140_write_cmos_sensor(0x099A,	0x0508);		                                                      
	SOC5140_write_cmos_sensor(0x099C,	0x0000);		                                                      
	SOC5140_write_cmos_sensor(0x099E,	0x0000);

	SOC5140_write_cmos_sensor(0x098E, 0x602A); 	// LOGICAL_ADDRESS_ACCESS [IO_NV_MEM_COMMAND]
    SOC5140_write_cmos_sensor(0xE02A, 0x0001); 	// IO_NV_MEM_COMMAND
    mdelay(50);  ///DELAY=50
    SOC5140_write_cmos_sensor(0x098E, 0x1000); 	// LOGICAL_ADDRESS_ACCESS
    SOC5140_write_cmos_sensor_8(0xD004, 0x02); 	// PGA_SOLUTION
    SOC5140_write_cmos_sensor(0xD006, 0x164C); 	// PGA_ZONE_ADDR_0
    SOC5140_write_cmos_sensor(0xD008, 0x1718); 	// PGA_ZONE_ADDR_1
    SOC5140_write_cmos_sensor(0xD00A, 0x1580); 	// PGA_ZONE_ADDR_2
    SOC5140_write_cmos_sensor_8(0xD005, 0x00); 	// PGA_CURRENT_ZONE
    SOC5140_write_cmos_sensor(0xD002, 0x8007); 	// PGA_ALGO
    SOC5140_write_cmos_sensor_8(0xD00C, 0x03); 	// PGA_NO_OF_ZONES
    SOC5140_write_cmos_sensor_8(0xD00D, 0x00); 	// PGA_ZONE_LOW_0
    SOC5140_write_cmos_sensor_8(0xD00E, 0x18); 	// PGA_ZONE_LOW_1
    SOC5140_write_cmos_sensor_8(0xD00F, 0x38); 	// PGA_ZONE_LOW_2
    SOC5140_write_cmos_sensor_8(0xD011, 0x17); 	// PGA_ZONE_HIGH_0
    SOC5140_write_cmos_sensor_8(0xD012, 0x37); 	// PGA_ZONE_HIGH_1
    SOC5140_write_cmos_sensor_8(0xD013, 0x7F); 	// PGA_ZONE_HIGH_2
    SOC5140_write_cmos_sensor(0x3210, 0x01B8); 	          // COLOR_PIPELINE_CONTROL

    //[SFFB_REV3_noisemodel]
    SOC5140_write_cmos_sensor_8(0xBCC0, 0x1F);	          // LL_SFFB_RAMP_START
    SOC5140_write_cmos_sensor_8(0xBCC1, 0x03);	          // LL_SFFB_RAMP_STOP
    SOC5140_write_cmos_sensor_8(0xBCC2, 0x2C);	          // LL_SFFB_SLOPE_START
    SOC5140_write_cmos_sensor_8(0xBCC3, 0x10);	          // LL_SFFB_SLOPE_STOP
    SOC5140_write_cmos_sensor_8(0xBCC4, 0x07);	          // LL_SFFB_THSTART
    SOC5140_write_cmos_sensor_8(0xBCC5, 0x0B);	          // LL_SFFB_THSTOP
    SOC5140_write_cmos_sensor(0xBCBA, 0x0009); 	          // LL_SFFB_CONFIG

    //[**********Step8*************]
    //[FTB_Off]
    SOC5140_write_cmos_sensor(0xBC14, 0xFFFF); 	        // LL_GAMMA_FADE_TO_BLACK_START_POS
    SOC5140_write_cmos_sensor(0xBC16, 0xFFFF); 	        // LL_GAMMA_FADE_TO_BLACK_END_POS

    //[Aperture_preference]
    SOC5140_write_cmos_sensor(0xBC66, 0x0154); 	        // LL_START_APERTURE_GM
    SOC5140_write_cmos_sensor(0xBC68, 0x07D0); 	        // LL_END_APERTURE_GM
    SOC5140_write_cmos_sensor_8(0xBC6A, 0x04);          // LL_START_APERTURE_INTEGER_GAIN
    SOC5140_write_cmos_sensor_8(0xBC6B, 0x00);         // LL_END_APERTURE_INTEGER_GAIN
    SOC5140_write_cmos_sensor_8(0xBC6C, 0x00);	        // LL_START_APERTURE_EXP_GAIN
    SOC5140_write_cmos_sensor_8(0xBC6D, 0x00);	        // LL_END_APERTURE_EXP_GAIN

    //[Gain_max]
    SOC5140_write_cmos_sensor(0xA81C, 0x0040); 	        // AE_TRACK_MIN_AGAIN
    SOC5140_write_cmos_sensor(0xA820, 0x01FC); 	        // AE_TRACK_MAX_AGAIN
    SOC5140_write_cmos_sensor(0xA822, 0x0080); 	        // AE_TRACK_MIN_DGAIN
    SOC5140_write_cmos_sensor(0xA824, 0x0100); 	        // AE_TRACK_MAX_DGAIN

    //[Saturation_REV3]
    SOC5140_write_cmos_sensor_8(0xBC56, 0x64);          	// LL_START_CCM_SATURATION
    SOC5140_write_cmos_sensor_8(0xBC57, 0x1E);          	// LL_END_CCM_SATURATION
    SOC5140_write_cmos_sensor_8(0xBCDE, 0x03);          	// LL_START_SYS_THRESHOLD
    SOC5140_write_cmos_sensor_8(0xBCDF, 0x50);          	// LL_STOP_SYS_THRESHOLD
    SOC5140_write_cmos_sensor_8(0xBCE0, 0x08);          	// LL_START_SYS_GAIN
    SOC5140_write_cmos_sensor_8(0xBCE1, 0x03);          	// LL_STOP_SYS_GAIN

    //[Sobel_REV3]
    SOC5140_write_cmos_sensor(0xBCD0, 0x000A); 	        // LL_SFFB_SOBEL_FLAT_START
    SOC5140_write_cmos_sensor(0xBCD2, 0x00FE); 	        // LL_SFFB_SOBEL_FLAT_STOP
    SOC5140_write_cmos_sensor(0xBCD4, 0x001E); 	        // LL_SFFB_SOBEL_SHARP_START
    SOC5140_write_cmos_sensor(0xBCD6, 0x00FF); 	        // LL_SFFB_SOBEL_SHARP_STOP
    SOC5140_write_cmos_sensor_8(0xBCC6, 0x00);          // LL_SFFB_SHARPENING_START
    SOC5140_write_cmos_sensor_8(0xBCC7, 0x00);          // LL_SFFB_SHARPENING_STOP
    SOC5140_write_cmos_sensor_8(0xBCC8, 0x20);          // LL_SFFB_FLATNESS_START
    SOC5140_write_cmos_sensor_8(0xBCC9, 0x40);          // LLL_SFFB_FLATNESS_STOP
    SOC5140_write_cmos_sensor_8(0xBCCA, 0x04);          // LL_SFFB_TRANSITION_START
    SOC5140_write_cmos_sensor_8(0xBCCB, 0x00);          // LL_SFFB_TRANSITION_STOP

    //[SFFB_slope_zero_enable]
    SOC5140_write_cmos_sensor_8(0xBCE6, 0x03);  	          // LL_SFFB_ZERO_ENABLE

    //[AE_preference]
    SOC5140_write_cmos_sensor_8(0xA410, 0x04);           	// AE_RULE_TARGET_AE_6
    SOC5140_write_cmos_sensor_8(0xA411, 0x06);           	// AE_RULE_TARGET_AE_7
                                 
    //[**********Step9*************]
    //[Sepia effect]
    SOC5140_write_cmos_sensor_8(0xDC3A, 0x23);             // SYS_SEPIA_CR
    SOC5140_write_cmos_sensor_8(0xDC3B, 0xB2);             // SYS_SEPIA_CB

    //[Touch Focus + Fast Focus AF_AFM_INIT]       
    SOC5140_write_cmos_sensor_8(0x8411, 0x00);            // SEQ_STATE_CFG_0_AF
    SOC5140_write_cmos_sensor_8(0x8412, 0x00);            // SEQ_STATE_CFG_0_AS
    SOC5140_write_cmos_sensor_8(0x8419, 0x40);            // SEQ_STATE_CFG_1_AF - use Fast Focus algorithm
    SOC5140_write_cmos_sensor_8(0x841A, 0x00);            // SEQ_STATE_CFG_1_AS

    SOC5140_write_cmos_sensor(0xB002, 0x0182); 	        // AF_MODE
    //SOC5140_write_cmos_sensor_8(0xB015, 0x18);  /// 0x10        	// AF_FS_INIT_POS
    //SOC5140_write_cmos_sensor(0xB016, 0x0030);  //x020          	// AF_FS_NUM_STEPS
    SOC5140_write_cmos_sensor(0xB045, 0x0000);	        // AF_MODE_EX
    SOC5140_write_cmos_sensor(0xC40A, 0x0030); 	        // AFM_POS_MIN
    SOC5140_write_cmos_sensor(0xC40C, 0x00A0); 	        // AFM_POS_MAX

    //AF Window size
    SOC5140_write_cmos_sensor_8(0xB854, 0x60);          	// STAT_SM_WINDOW_POS_X
    SOC5140_write_cmos_sensor_8(0xB855, 0x60);          	// STAT_SM_WINDOW_POS_Y
    SOC5140_write_cmos_sensor_8(0xB856, 0x40);          	// STAT_SM_WINDOW_SIZE_X
    SOC5140_write_cmos_sensor_8(0xB857, 0x40);	         	// STAT_SM_WINDOW_SIZE_Y
                                
    SOC5140_write_cmos_sensor_8(0xB011, 0x00);          	// AF_FS_INIT_POS
    SOC5140_write_cmos_sensor_8(0xB012, 0x0A);          	// AF_FS_NUM_STEPS
    SOC5140_write_cmos_sensor_8(0xB018, 0x00);          	// AF_FS_POS_0
    SOC5140_write_cmos_sensor_8(0xB019, 0x30);          	// AF_FS_POS_1
    SOC5140_write_cmos_sensor_8(0xB01A, 0x48);          	// AF_FS_POS_2
    SOC5140_write_cmos_sensor_8(0xB01B, 0x60);          	// AF_FS_POS_3
    SOC5140_write_cmos_sensor_8(0xB01C, 0x78);          	// AF_FS_POS_4
    SOC5140_write_cmos_sensor_8(0xB01D, 0x90);          	// AF_FS_POS_5
    SOC5140_write_cmos_sensor_8(0xB01E, 0xA8);          	// AF_FS_POS_6
    SOC5140_write_cmos_sensor_8(0xB01F, 0xC0);          	// AF_FS_POS_7
    SOC5140_write_cmos_sensor_8(0xB020, 0xE0);          	// AF_FS_POS_8
    SOC5140_write_cmos_sensor_8(0xB021, 0xFF);          	// AF_FS_POS_9

    //INIT PATCH PAGE used in FF
    SOC5140_write_cmos_sensor_8(0xD400, 0x01);           	// PATCH_FF_SEARCH_FOCUS_LENS_TH
    SOC5140_write_cmos_sensor_8(0xD401, 0x00);           	// PATCH_FF_FOCUS_LENS_TH
    SOC5140_write_cmos_sensor_8(0xD402, 0x28);           	// PATCH_FF_BLUR_DETECT_LUMA_TH
    SOC5140_write_cmos_sensor_8(0xD403, 0x80);           	// PATCH_FF_BLUR_DETECT_SHA_TH
    SOC5140_write_cmos_sensor_8(0xD404, 0x00);           	// PATCH_FB_SKIP_FRAME_TH
    SOC5140_write_cmos_sensor_8(0xD405, 0x00);           	// PATCH_FB_SKIP_FRAME_CNT
    SOC5140_write_cmos_sensor_8(0xD406, 0x00);           	// PATCH_NR_STOP_TH
    SOC5140_write_cmos_sensor_8(0xD407, 0x00);           	// PATCH_NR_STOP_CNT
    SOC5140_write_cmos_sensor_8(0xD408, 0x30);           	// PATCH_NR_STOPS_0
    SOC5140_write_cmos_sensor_8(0xD409, 0x40);           	// PATCH_NR_STOPS_1
    SOC5140_write_cmos_sensor_8(0xD40A, 0x50);           	// PATCH_NR_STOPS_2
    SOC5140_write_cmos_sensor_8(0xD40B, 0x70);           	// PATCH_NR_STOPS_3
    SOC5140_write_cmos_sensor_8(0xD40C, 0x80);           	// PATCH_NR_STOPS_4
    SOC5140_write_cmos_sensor_8(0xD40D, 0x90);           	// PATCH_NR_STOPS_5 
    SOC5140_write_cmos_sensor(0x0018, 0x2008); 	        // STANDBY_CONTROL_AND_STATUS
    mdelay(10);
    write_statue = SOC5140_YUV_INIT;
}
void SOC5140_Normal_Pre(void)
{
	SENSORDB( "\n ****** SOC5140_Normal_Pre \n");
    //[**********Step1*************]
    // VCO= 768 MHz, CPU= 48 MHz, Tx= 96 MHz
    SOC5140_write_cmos_sensor(0x098E, 0xC86C);
    SOC5140_write_cmos_sensor(0xC86C, 0x0518);
    SOC5140_write_cmos_sensor(0xC86E, 0x03D4);
    SOC5140_write_cmos_sensor(0xC83A, 0x000C);
    SOC5140_write_cmos_sensor(0xC83C, 0x0018);
    SOC5140_write_cmos_sensor(0xC83E, 0x07B1);
    SOC5140_write_cmos_sensor(0xC840, 0x0A45);
    SOC5140_write_cmos_sensor(0xC842, 0x0001);
    SOC5140_write_cmos_sensor(0xC844, 0x0103);
    SOC5140_write_cmos_sensor(0xC846, 0x0103);
    SOC5140_write_cmos_sensor(0xC848, 0x0103);
    SOC5140_write_cmos_sensor(0xC84A, 0x0103);
    SOC5140_write_cmos_sensor(0xC84C, 0x00F6);
    SOC5140_write_cmos_sensor(0xC84E, 0x0001);
    SOC5140_write_cmos_sensor_8(0xC850, 0x00);
    SOC5140_write_cmos_sensor_8(0xC851, 0x00);
    SOC5140_write_cmos_sensor(0xC852, 0x019C);
    SOC5140_write_cmos_sensor(0xC854, 0x0732);
    SOC5140_write_cmos_sensor(0xC856, 0x048E);
    SOC5140_write_cmos_sensor(0xC858, 0x0002);
    SOC5140_write_cmos_sensor(0xC85A, 0x0001);
    SOC5140_write_cmos_sensor(0xC85C, 0x0423);
    SOC5140_write_cmos_sensor(0xC85E, 0xFFFF);
    SOC5140_write_cmos_sensor(0xC860, 0x0423);
    SOC5140_write_cmos_sensor(0xC862, 0x0E87);
    SOC5140_write_cmos_sensor(0xC864, 0xFFFE);
    SOC5140_write_cmos_sensor(0xC866, 0x7F7C);
    SOC5140_write_cmos_sensor(0xC868, 0x0423);
    SOC5140_write_cmos_sensor(0xC86A, 0x0E87);
    SOC5140_write_cmos_sensor(0xC870, 0x0014);
    SOC5140_write_cmos_sensor(0xC8AA, 0x0280);
    SOC5140_write_cmos_sensor(0xC8AC, 0x01E0);
    SOC5140_write_cmos_sensor(0xC8AE, 0x0001);
    SOC5140_write_cmos_sensor(0xC8B0, 0x0002);
    SOC5140_write_cmos_sensor(0xC8B8, 0x0004);
    SOC5140_write_cmos_sensor(0xA010, 0x0134);
    SOC5140_write_cmos_sensor(0xA012, 0x0148);
    SOC5140_write_cmos_sensor(0xA014, 0x00FF);
    SOC5140_write_cmos_sensor(0xA016, 0x0113);
    SOC5140_write_cmos_sensor(0xA018, 0x013E);
    SOC5140_write_cmos_sensor(0xA01A, 0x0098);
    SOC5140_write_cmos_sensor(0xA01C, 0x0109);
    SOC5140_write_cmos_sensor(0xA01E, 0x007F);
     /// kevin add @20130613 start
	  SOC5140_write_cmos_sensor(0xC862, 0x1146);	//Min Line Length (A) = 3719
	  SOC5140_write_cmos_sensor(0xC86A, 0x1146);	//Line Length (A) = 3719
	  SOC5140_write_cmos_sensor(0xA010, 0x0101);	//fd_min_expected50hz_flicker_period = 257
	  SOC5140_write_cmos_sensor(0xA012, 0x0115);	//fd_max_expected50hz_flicker_period = 277
	  SOC5140_write_cmos_sensor(0xA014, 0x00D5);	//fd_min_expected60hz_flicker_period = 213
	  SOC5140_write_cmos_sensor(0xA016, 0x00E9);	//fd_max_expected60hz_flicker_period = 233
	  SOC5140_write_cmos_sensor(0xA018, 0x010b);	//fd_expected50hz_flicker_period (A) = 267
	  SOC5140_write_cmos_sensor(0xA01A, 0x0098);	//fd_expected50hz_flicker_period (B) = 152
	  SOC5140_write_cmos_sensor(0xA01C, 0x00DF);	//fd_expected60hz_flicker_period (A) = 223
	  SOC5140_write_cmos_sensor(0xA01E, 0x007F);	//fd_expected60hz_flicker_period (B) = 127
     /// kevin add @20130613 end
    
    //SOC5140_8404_Polling(0x8404,0x06);
    mDELAY(5);
}

void SOC5140_ZSD_Pre(void)
{
	SENSORDB( "\n ****** SOC5140_ZSD_Pre \n");
    //[**********Step1*************]
    // VCO= 768 MHz, CPU= 48 MHz, Tx= 96 MHz
    SOC5140_write_cmos_sensor(0x098E, 0xC86C);
    SOC5140_write_cmos_sensor(0xC86C, 0x0A28);
    SOC5140_write_cmos_sensor(0xC86E, 0x07A0);
    SOC5140_write_cmos_sensor(0xC83A, 0x0010); /// x_star
    SOC5140_write_cmos_sensor(0xC83C, 0x001c); /// y_star
    SOC5140_write_cmos_sensor(0xC83E, 0x07AF);
    SOC5140_write_cmos_sensor(0xC840, 0x0A43);
    SOC5140_write_cmos_sensor(0xC842, 0x0001);
    SOC5140_write_cmos_sensor(0xC844, 0x0101);
    SOC5140_write_cmos_sensor(0xC846, 0x0101);
    SOC5140_write_cmos_sensor(0xC848, 0x0101);
    SOC5140_write_cmos_sensor(0xC84A, 0x0101);
    SOC5140_write_cmos_sensor(0xC84C, 0x00F6);
    SOC5140_write_cmos_sensor(0xC84E, 0x0000);
    SOC5140_write_cmos_sensor_8(0xC850, 0x00);
    SOC5140_write_cmos_sensor_8(0xC851, 0x00);    
    SOC5140_write_cmos_sensor(0xC852, 0x009C);
    SOC5140_write_cmos_sensor(0xC854, 0x034A);    
    SOC5140_write_cmos_sensor(0xC856, 0x02A6);  
    SOC5140_write_cmos_sensor(0xC858, 0x0002);
    SOC5140_write_cmos_sensor(0xC85A, 0x0001);
    SOC5140_write_cmos_sensor(0xC85C, 0x07EF);
    SOC5140_write_cmos_sensor(0xC85E, 0xFFFF);
    SOC5140_write_cmos_sensor(0xC860, 0x07EF);
    SOC5140_write_cmos_sensor(0xC862, 0x1E48);
    SOC5140_write_cmos_sensor(0xC864, 0xFFFE);
    SOC5140_write_cmos_sensor(0xC866, 0x7F7C);
    SOC5140_write_cmos_sensor(0xC868, 0x07EF);
    SOC5140_write_cmos_sensor(0xC86A, 0x1E48);
    SOC5140_write_cmos_sensor(0xC870, 0x0014);
    SOC5140_write_cmos_sensor(0xC8AA, 0x0A20);
    SOC5140_write_cmos_sensor(0xC8AC, 0x0798);
    SOC5140_write_cmos_sensor(0xC8AE, 0x0001);
    SOC5140_write_cmos_sensor(0xC8B0, 0x0002);
    SOC5140_write_cmos_sensor(0xC8B8, 0x0004);
    SOC5140_write_cmos_sensor(0xA010, 0x008E);
    SOC5140_write_cmos_sensor(0xA012, 0x00A2);
    SOC5140_write_cmos_sensor(0xA014, 0x0075);
    SOC5140_write_cmos_sensor(0xA016, 0x0089);
    SOC5140_write_cmos_sensor(0xA018, 0x0098);
    SOC5140_write_cmos_sensor(0xA01A, 0x0098);
    SOC5140_write_cmos_sensor(0xA01C, 0x007F);
    SOC5140_write_cmos_sensor(0xA01E, 0x007F);

    //SOC5140_8404_Polling(0x8404,0x06);
    mDELAY(5);
   /// off
}

static void SOC5140_MIPI_AWB_Lock(void)
{
    SENSORDB("\n ****** SOC5140_MIPI_AWB_Lock\n");
    //SOC5140_write_cmos_sensor(0x098E, 0x8418);
    SOC5140_write_cmos_sensor_8(0x8418, 0x00);
    //SOC5140_write_cmos_sensor_8(0x8438, 0x00);    
    SOC5140_8404_Polling(0x8404, 0x06);
}

static void SOC5140_MIPI_AWB_UnLock(void)
{
    SENSORDB("\n ****** SOC5140_MIPI_AWB_UnLock\n");
   // SOC5140_write_cmos_sensor(0x098E, 0x8418);
    SOC5140_write_cmos_sensor_8(0x8418, 0x02);
    //SOC5140_write_cmos_sensor_8(0x8438, 0x02);    
    SOC5140_8404_Polling(0x8404, 0x06);    
}

//manuel exposure enable
static void SOC5140_MIPI_AE_Lock(void)
{
    SENSORDB("\n ****** SOC5140_MIPI_AE_Lock\n");
   // SOC5140_write_cmos_sensor(0x098E, 0x8416);
    SOC5140_write_cmos_sensor_8(0x8416, 0x00);
    SOC5140_write_cmos_sensor_8(0x843A, 0x00);
    SOC5140_8404_Polling(0x8404, 0x06);
}

//ae enable
static void SOC5140_MIPI_AE_UnLock(void)
{
  SENSORDB("\n ****** SOC5140_MIPI_AE_UnLock\n");
  //SOC5140_write_cmos_sensor(0x098E, 0x8416);
  SOC5140_write_cmos_sensor_8(0x8416, 0x02);
  SOC5140_write_cmos_sensor_8(0x843A, 0x00);
  SOC5140_8404_Polling(0x8404, 0x06);
}

static void SOC5140_SCENE_PORTRAIT(void)
{
	SENSORDB("SCENE_MODE_PORTRAIT");  /// 10~30 fps	
    SOC5140_write_cmos_sensor_8(0xac01, 0xab); 
    SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
    #if 0
    SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
    SOC5140_write_cmos_sensor(0xA81C, 0x0040);
    SOC5140_write_cmos_sensor(0xA81E, 0x012c);
    SOC5140_write_cmos_sensor(0xA820, 0x01fc);
    SOC5140_write_cmos_sensor(0xA822, 0x0080);
    SOC5140_write_cmos_sensor(0xA824, 0x0100);
    #endif
    SOC5140_8404_Polling(0x8404, 0x06);
}

void SOC5140_night_mode(kal_bool enable)
{
	SENSORDB("\n ****** SOC5140_night_mode \n ");
    SENSORDB("[Enter]SOC5140 night mode func:enable = %d\n",enable);

	spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.NightMode = enable;
	spin_unlock(&soc5140_drv_lock);

    if(SOC5140_CAM_PREVIEW == SOC5140_Sensor_Driver.Camco_mode)
    {
    	if (enable)
    	{
            //SOC5140_write_cmos_sensor(0x098E, 0x281A);              
            SOC5140_write_cmos_sensor(0xA81A, 0x14df);  /// 5~
            SOC5140_write_cmos_sensor(0xA818, 0x07D0);   
           // SOC5140_write_cmos_sensor(0xC85C, 0x0424);
    	}
    	else
    	{
            SOC5140_write_cmos_sensor(0xA81A, 0x0a6f);  /// 10FPS
            SOC5140_write_cmos_sensor(0xA818, 0x07D0);
            //SOC5140_write_cmos_sensor(0xC85C, 0x0424);
    	}
    }
    else if(SOC5140_CAM_PREVIEW_ZSD == SOC5140_Sensor_Driver.Camco_mode)
    {
    	if (enable)
    	{                                       ///expouse
            SOC5140_write_cmos_sensor(0xA81A, 0x0BE0);
            SOC5140_write_cmos_sensor(0xA818, 0x07b8);
            //SOC5140_write_cmos_sensor(0xC85C, 0x07EF);
    	}
    	else
    	{
            SOC5140_write_cmos_sensor(0xA81A, 0x05f0); // make sure exps 100ms
            SOC5140_write_cmos_sensor(0xA818, 0x05c0);
            //SOC5140_write_cmos_sensor(0xC85C, 0x07EF);
    	}
    }
    else if(SOC5140_CAM_CAPTURE == SOC5140_Sensor_Driver.Camco_mode)
    {
    	if (enable)
    	{
          
            
    	}
    	else
    	{
       
    	}
    }
    else if(SOC5140_CAM_VIDEO == SOC5140_Sensor_Driver.Camco_mode)
    {
    	
    }
    SOC5140_8404_Polling_Sub(30,0x06);
}






static void SOC5140_SCENE_LANDSCAPE(void)
{
	SENSORDB("SCENE_MODE_LANDSCAPE");	///10~30fps

    SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    SOC5140_write_cmos_sensor_8(0xac97, 0xf0); 
    SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9a, 0xf0); 
    SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
    #if 0	
    if(0 == zsd_setting)
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
        }
        else
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x05f0);
        }
    SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
    SOC5140_write_cmos_sensor(0xA81C, 0x0040);
    SOC5140_write_cmos_sensor(0xA81E, 0x012c);
    SOC5140_write_cmos_sensor(0xA820, 0x01fc);
    SOC5140_write_cmos_sensor(0xA822, 0x0080);
    SOC5140_write_cmos_sensor(0xA824, 0x0100);
    #endif
    SOC5140_8404_Polling(0x8404, 0x06);
}

static void SOC5140_SCENE_SUNSET(void)
{
	SENSORDB("SCENE_MODE_SUNSET");	 /// daylight   10~30fps
    SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 

    SOC5140_write_cmos_sensor_8(0xac97, 0xf0); 
    SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9a, 0xf0); 
    SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
 
    #if 0  
    if(0 == zsd_setting)
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
        }
        else
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x05f0);
        }
    SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
    SOC5140_write_cmos_sensor(0xA81C, 0x0040);
    SOC5140_write_cmos_sensor(0xA81E, 0x012c);
    SOC5140_write_cmos_sensor(0xA820, 0x01fc);
    SOC5140_write_cmos_sensor(0xA822, 0x0080);
    SOC5140_write_cmos_sensor(0xA824, 0x0100);
    #endif
    SOC5140_8404_Polling(0x8404, 0x06);
    
}

static void SOC5140_SCENE_SPORTS(void)
{
 		SENSORDB("SCENE_MODE_SPORTS");	 /// 15~30 fps

 		
        SOC5140_write_cmos_sensor(0x098E, 0x281A);
        if(0 == zsd_setting)
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x0848);
        }
        else
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x03f5);
        }
        
        SOC5140_write_cmos_sensor(0xA81C, 0x0040);
        SOC5140_write_cmos_sensor(0xA81E, 0x0080);          
        SOC5140_write_cmos_sensor(0xA820, 0x01FC);
        SOC5140_write_cmos_sensor(0xA822, 0x0080);
       // SOC5140_write_cmos_sensor(0xA824, 0x0100);
        SOC5140_write_cmos_sensor_8(0xac01, 0xab); 
        SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
        SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
        SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
        SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
        SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
        SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
        SOC5140_8404_Polling(0x8404, 0x06);
        
}

static void SOC5140_SCENE_NIGHTSCENE(void)
{
	SENSORDB("SOC5140_SCENE_NIGHTSCENE");  /// 5~30fps	
           
  
    #if 0 
    SOC5140_write_cmos_sensor(0xA81A, 0x1770);
    SOC5140_write_cmos_sensor(0xA818, 0x07D0);
    SOC5140_write_cmos_sensor_8(0xac01, 0xab); 
    SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
    SOC5140_8404_Polling(0x8404, 0x05);
    #endif

}

static void SOC5140_SCENE_NORMAL(void)
{
	SENSORDB("SCENE_MODE_OFF");	 /// 10~30fps
	
   
    #if 0 
    if(0 == zsd_setting)
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x0C6C);
            SOC5140_write_cmos_sensor(0xA818, 0x07D0);
        }
        else
        {
            SOC5140_write_cmos_sensor(0xA81A, 0x05f2);
            SOC5140_write_cmos_sensor(0xA818, 0x0500);
        }
    SOC5140_write_cmos_sensor_8(0xac01, 0xab); 
    SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
    #endif
    SOC5140_write_cmos_sensor(0xA81C, 0x0040);
    SOC5140_write_cmos_sensor(0xA81E, 0x012c);
    SOC5140_write_cmos_sensor(0xA820, 0x01fc);
    //SOC5140_write_cmos_sensor(0xA822, 0x0080);
    //SOC5140_write_cmos_sensor(0xA824, 0x0100);
    SOC5140_8404_Polling(0x8404, 0x06);  
}


BOOL SOC5140_set_param_banding(UINT16 para)
{
    SENSORDB("\n ****** SOC5140_set_param_banding\n");
	SENSORDB("[Enter]SOC5140 set_param_banding func:para = %d\n",para);

   if(SOC5140_Sensor_Driver.Banding == para)
   {
	    SOC5140_8404_Polling(0x8404, 0x06);
	    return KAL_TRUE;
   }       

    spin_lock(&soc5140_drv_lock);    
    SOC5140_Sensor_Driver.Banding = para;
    spin_unlock(&soc5140_drv_lock);	
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
	    {
       	    //SOC5140_write_cmos_sensor(0x098E, 0x8417);	  
		    SOC5140_write_cmos_sensor_8(0x8417, 0x01);
		    SOC5140_write_cmos_sensor_8(0xA004, 0x32);
		    SOC5140_8404_Polling(0x8404, 0x05);
	    }
		break;

		case AE_FLICKER_MODE_60HZ:
	    {
       	    //SOC5140_write_cmos_sensor(0x098E, 0x8417);	  
		    SOC5140_write_cmos_sensor_8(0x8417, 0x01);
		    SOC5140_write_cmos_sensor_8(0xA004, 0x3C);
		    SOC5140_8404_Polling(0x8404, 0x05);
	    }
		break;

		case AE_FLICKER_MODE_OFF:
		{
		    SOC5140_write_cmos_sensor_8(0x8417, 0x00);
		    SOC5140_write_cmos_sensor_8(0xA004, 0x32);
		    SOC5140_8404_Polling(0x8404, 0x05);
				}
		break;

		case AE_FLICKER_MODE_AUTO:
        default:
        {
		    SOC5140_write_cmos_sensor_8(0x8417, 0x02);
		    SOC5140_write_cmos_sensor_8(0xA004, 0x32);
		    SOC5140_8404_Polling(0x8404, 0x05);
        }
        break;
	}
	
       return KAL_TRUE;
}

void SOC5140_MIPI_set_saturation(UINT16 para)
{
    SENSORDB("\n ****** SOC5140_MIPI_set_saturation\n");

    switch (para)
    {
        case ISP_SAT_HIGH:
            SOC5140_write_cmos_sensor(0x098E, 0x3C02); 
            SOC5140_write_cmos_sensor(0xBC02, 0x01EE);   
            SOC5140_write_cmos_sensor_8(0xAC0E, 0xC8);   
            break;
        case ISP_SAT_LOW:
            SOC5140_write_cmos_sensor(0x098E, 0x3C02); 
            SOC5140_write_cmos_sensor(0xBC02, 0x01EE);   
            SOC5140_write_cmos_sensor_8(0xAC0E, 0x40); 
            break;
        case ISP_SAT_MIDDLE:
                  SOC5140_write_cmos_sensor(0x098E, 0x3C02); 
            SOC5140_write_cmos_sensor(0xBC02, 0x01EE); 
            SOC5140_write_cmos_sensor_8(0xAC0E, 0x86);        
        default:
             break;
    }
                return ERROR_NONE;
}

void SOC5140_MIPI_set_contrast(UINT16 para)
{
    SENSORDB("\n ****** SOC5140_MIPI_set_contrast\n");

    switch (para)
    {
        case ISP_CONTRAST_HIGH:
            //[contrast+h--]
            SOC5140_write_cmos_sensor(0x098E, 0xBC51);
            SOC5140_write_cmos_sensor_8(0xBC51, 0x04);   
            SOC5140_write_cmos_sensor_8(0xBC2B, 0x00);  // LL_GAMMA_NEUTRAL_CURVE_0
            SOC5140_write_cmos_sensor_8(0xBC2C, 0x06);  // LL_GAMMA_NEUTRAL_CURVE_1
            SOC5140_write_cmos_sensor_8(0xBC2D, 0x12);  // LL_GAMMA_NEUTRAL_CURVE_2
            SOC5140_write_cmos_sensor_8(0xBC2E, 0x27);  // LL_GAMMA_NEUTRAL_CURVE_3
            SOC5140_write_cmos_sensor_8(0xBC2F, 0x48);  // LL_GAMMA_NEUTRAL_CURVE_4
            SOC5140_write_cmos_sensor_8(0xBC30, 0x69);  // LL_GAMMA_NEUTRAL_CURVE_5
            SOC5140_write_cmos_sensor_8(0xBC31, 0x8A);  // LL_GAMMA_NEUTRAL_CURVE_6
            SOC5140_write_cmos_sensor_8(0xBC32, 0xA4);  // LL_GAMMA_NEUTRAL_CURVE_7
            SOC5140_write_cmos_sensor_8(0xBC33, 0xB7);  // LL_GAMMA_NEUTRAL_CURVE_8
            SOC5140_write_cmos_sensor_8(0xBC34, 0xC6);  // LL_GAMMA_NEUTRAL_CURVE_9
            SOC5140_write_cmos_sensor_8(0xBC35, 0xD1);  // LL_GAMMA_NEUTRAL_CURVE_10
            SOC5140_write_cmos_sensor_8(0xBC36, 0xDB);  // LL_GAMMA_NEUTRAL_CURVE_11
            SOC5140_write_cmos_sensor_8(0xBC37, 0xE2);  // LL_GAMMA_NEUTRAL_CURVE_12
            SOC5140_write_cmos_sensor_8(0xBC38, 0xE9);  // LL_GAMMA_NEUTRAL_CURVE_13
            SOC5140_write_cmos_sensor_8(0xBC39, 0xEE);  // LL_GAMMA_NEUTRAL_CURVE_14
            SOC5140_write_cmos_sensor_8(0xBC3A, 0xF3);  // LL_GAMMA_NEUTRAL_CURVE_15
            SOC5140_write_cmos_sensor_8(0xBC3B, 0xF7);  // LL_GAMMA_NEUTRAL_CURVE_16
            SOC5140_write_cmos_sensor_8(0xBC3C, 0xFB);  // LL_GAMMA_NEUTRAL_CURVE_17
            SOC5140_write_cmos_sensor_8(0xBC3D, 0xFF);  // LL_GAMMA_NEUTRAL_CURVE_18
            SOC5140_8404_Polling(0x8404, 0x05);  // SEQ_CMD            break;
            break;
 
        case ISP_CONTRAST_LOW:
                //[contrast_l--]
                SOC5140_write_cmos_sensor(0x098E, 0xBC51);
                SOC5140_write_cmos_sensor_8(0xBC51, 0x04);  
                SOC5140_write_cmos_sensor_8(0xBC2B, 0x00);  // LL_GAMMA_NEUTRAL_CURVE_0
                SOC5140_write_cmos_sensor_8(0xBC2C, 0x0C);  // LL_GAMMA_NEUTRAL_CURVE_1
                SOC5140_write_cmos_sensor_8(0xBC2D, 0x22);  // LL_GAMMA_NEUTRAL_CURVE_2
                SOC5140_write_cmos_sensor_8(0xBC2E, 0x3E);  // LL_GAMMA_NEUTRAL_CURVE_3
                SOC5140_write_cmos_sensor_8(0xBC2F, 0x5D);  // LL_GAMMA_NEUTRAL_CURVE_4
                SOC5140_write_cmos_sensor_8(0xBC30, 0x73);  // LL_GAMMA_NEUTRAL_CURVE_5
                SOC5140_write_cmos_sensor_8(0xBC31, 0x85);  // LL_GAMMA_NEUTRAL_CURVE_6
                SOC5140_write_cmos_sensor_8(0xBC32, 0x94);  // LL_GAMMA_NEUTRAL_CURVE_7
                SOC5140_write_cmos_sensor_8(0xBC33, 0xA2);  // LL_GAMMA_NEUTRAL_CURVE_8
                SOC5140_write_cmos_sensor_8(0xBC34, 0xAE);  // LL_GAMMA_NEUTRAL_CURVE_9
                SOC5140_write_cmos_sensor_8(0xBC35, 0xB9);  // LL_GAMMA_NEUTRAL_CURVE_10
                SOC5140_write_cmos_sensor_8(0xBC36, 0xC3);  // LL_GAMMA_NEUTRAL_CURVE_11
                SOC5140_write_cmos_sensor_8(0xBC37, 0xCD);  // LL_GAMMA_NEUTRAL_CURVE_12
                SOC5140_write_cmos_sensor_8(0xBC38, 0xD7);  // LL_GAMMA_NEUTRAL_CURVE_13
                SOC5140_write_cmos_sensor_8(0xBC39, 0xDF);  // LL_GAMMA_NEUTRAL_CURVE_14
                SOC5140_write_cmos_sensor_8(0xBC3A, 0xE8);  // LL_GAMMA_NEUTRAL_CURVE_15
                SOC5140_write_cmos_sensor_8(0xBC3B, 0xF0);  // LL_GAMMA_NEUTRAL_CURVE_16
                SOC5140_write_cmos_sensor_8(0xBC3C, 0xF8);  // LL_GAMMA_NEUTRAL_CURVE_17
                SOC5140_write_cmos_sensor_8(0xBC3D, 0xFF);  // LL_GAMMA_NEUTRAL_CURVE_18
                SOC5140_8404_Polling(0x8404, 0x05);  // SEQ_CMD
            break;
            
        case ISP_CONTRAST_MIDDLE:        
        default:
                //[contrast_m--]
                SOC5140_write_cmos_sensor(0x098E, 0xBC51);
                SOC5140_write_cmos_sensor_8(0xBC51, 0x04);   
                SOC5140_write_cmos_sensor_8(0xBC2B, 0x00);  // LL_GAMMA_NEUTRAL_CURVE_0
                SOC5140_write_cmos_sensor_8(0xBC2C, 0x09);  // LL_GAMMA_NEUTRAL_CURVE_1
                SOC5140_write_cmos_sensor_8(0xBC2D, 0x1A);  // LL_GAMMA_NEUTRAL_CURVE_2
                SOC5140_write_cmos_sensor_8(0xBC2E, 0x34);  // LL_GAMMA_NEUTRAL_CURVE_3
                SOC5140_write_cmos_sensor_8(0xBC2F, 0x54);  // LL_GAMMA_NEUTRAL_CURVE_4
                SOC5140_write_cmos_sensor_8(0xBC30, 0x6F);  // LL_GAMMA_NEUTRAL_CURVE_5
                SOC5140_write_cmos_sensor_8(0xBC31, 0x87);  // LL_GAMMA_NEUTRAL_CURVE_6
                SOC5140_write_cmos_sensor_8(0xBC32, 0x9B);  // LL_GAMMA_NEUTRAL_CURVE_7
                SOC5140_write_cmos_sensor_8(0xBC33, 0xAB);  // LL_GAMMA_NEUTRAL_CURVE_8
                SOC5140_write_cmos_sensor_8(0xBC34, 0xB8);  // LL_GAMMA_NEUTRAL_CURVE_9
                SOC5140_write_cmos_sensor_8(0xBC35, 0xC4);  // LL_GAMMA_NEUTRAL_CURVE_10
                SOC5140_write_cmos_sensor_8(0xBC36, 0xCE);  // LL_GAMMA_NEUTRAL_CURVE_11
                SOC5140_write_cmos_sensor_8(0xBC37, 0xD7);  // LL_GAMMA_NEUTRAL_CURVE_12
                SOC5140_write_cmos_sensor_8(0xBC38, 0xDF);  // LL_GAMMA_NEUTRAL_CURVE_13
                SOC5140_write_cmos_sensor_8(0xBC39, 0xE7);  // LL_GAMMA_NEUTRAL_CURVE_14
                SOC5140_write_cmos_sensor_8(0xBC3A, 0xEE);  // LL_GAMMA_NEUTRAL_CURVE_15
                SOC5140_write_cmos_sensor_8(0xBC3B, 0xF4);  // LL_GAMMA_NEUTRAL_CURVE_16
                SOC5140_write_cmos_sensor_8(0xBC3C, 0xFA);  // LL_GAMMA_NEUTRAL_CURVE_17
                SOC5140_write_cmos_sensor_8(0xBC3D, 0xFF);  // LL_GAMMA_NEUTRAL_CURVE_18
                SOC5140_8404_Polling(0x8404, 0x05);  // SEQ_CMD
            break;
    }

	return;
	
}

void SOC5140_MIPI_set_brightness(UINT16 para)
{
    SENSORDB("\n ****** SOC5140_MIPI_set_brightness\n");
    switch (para)
    {
        case ISP_BRIGHT_HIGH:
            //SOC5140_write_cmos_sensor(0x098E, 0xA401);                            // [AE_BASETARGET]
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00);
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);                   // [SEQ_CMD]
            //SOC5140_write_cmos_sensor(0x3400, 0x7824); 
            //SOC5140_write_cmos_sensor(0x301A, 0x0634);
            SOC5140_write_cmos_sensor(0x337E, 0x1C00);
            //mdelay(300);
            //SOC5140_write_cmos_sensor(0x3400, 0x7a24);
            //mdelay(1000);
            //SOC5140_8404_Polling_Sub(20, 0x06);
            break;
 
        case ISP_BRIGHT_LOW:
 
            //SOC5140_write_cmos_sensor(0x098E, 0xA401);                
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00); 
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);      
            //SOC5140_write_cmos_sensor(0x3400, 0x7824);
            //SOC5140_write_cmos_sensor(0x301A, 0x0634);
            SOC5140_write_cmos_sensor(0x337E, 0xE400);
            //mdelay(300);
            //SOC5140_write_cmos_sensor(0x3400, 0x7a24);
           // mdelay(1000);
            //SOC5140_8404_Polling_Sub(20, 0x05);
            break;
        case ISP_BRIGHT_MIDDLE:
                                //SOC5140_write_cmos_sensor(0x098E, 0xA401);                            // [AE_BASETARGET] 
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00);                       
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);                               // [SEQ_CMD]       
            //SOC5140_write_cmos_sensor(0x3400, 0x7824);
            //SOC5140_write_cmos_sensor(0x301A, 0x0634);
            SOC5140_write_cmos_sensor(0x337E, 0x0000);
            //mdelay(300);
            //SOC5140_write_cmos_sensor(0x3400, 0x7a24);
            //mdelay(1000);
            //SOC5140_8404_Polling_Sub(20, 0x05);
      
        default:
             break;
    }    
	return ERROR_NONE;
}

void SOC5140_MIPI_set_iso(UINT16 para)
{
    SENSORDB("\n ****** SOC5140_MIPI_set_iso\n");
    switch (para)
    {
        case AE_ISO_100:
             //ISO 100
                SOC5140_write_cmos_sensor(0x098E, 0x281C);
                SOC5140_write_cmos_sensor(0xA81C, 0x0040);
                SOC5140_write_cmos_sensor(0xA81E, 0x0080);
                SOC5140_write_cmos_sensor(0xA820, 0x0080);
                //SOC5140_write_cmos_sensor(0xA822, 0x0080);
                //SOC5140_write_cmos_sensor(0xA824, 0x0080);
                SOC5140_8404_Polling(0x8404, 0x06);             
             break;
        case AE_ISO_200:
             //ISO 200
                SOC5140_write_cmos_sensor(0x098E, 0x281C);
                SOC5140_write_cmos_sensor(0xA81C, 0x0040);  
                SOC5140_write_cmos_sensor(0xA81E, 0x0080);  
                SOC5140_write_cmos_sensor(0xA820, 0x00B0);  
                //SOC5140_write_cmos_sensor(0xA822, 0x0080);  
               // SOC5140_write_cmos_sensor(0xA824, 0x0080);  
                SOC5140_8404_Polling(0x8404, 0x06);   
             break;
             
        case AE_ISO_400:
             //ISO 400
                SOC5140_write_cmos_sensor(0x098E, 0x281C);
                SOC5140_write_cmos_sensor(0xA81C, 0x0040);
                SOC5140_write_cmos_sensor(0xA81E, 0x0080);               
                SOC5140_write_cmos_sensor(0xA820, 0x0100);
                //SOC5140_write_cmos_sensor(0xA822, 0x0080);
                //SOC5140_write_cmos_sensor(0xA824, 0x0080);
                SOC5140_8404_Polling(0x8404, 0x06);             
             break;
             
        default:
        //case AE_ISO_AUTO:
             //ISO Auto
            SOC5140_write_cmos_sensor(0x098E, 0x281C);
            SOC5140_write_cmos_sensor(0xA81C, 0x0040);
            SOC5140_write_cmos_sensor(0xA81E, 0x0080);
            SOC5140_write_cmos_sensor(0xA820, 0x01fc);
            //SOC5140_write_cmos_sensor(0xA822, 0x0080);
           // SOC5140_write_cmos_sensor(0xA824, 0x0100);
            SOC5140_8404_Polling(0x8404, 0x06);             
             break;
    }
    return;
}

BOOL SOC5140_set_param_wb(UINT16 para)
{
	SENSORDB("\n ****** SOC5140_set_param_wb \n ");
	SENSORDB("SOC5140_set_param_wb = %d\n",para);
     spin_lock(&soc5140_drv_lock);
     awbMode = para;
   	 spin_unlock(&soc5140_drv_lock);
   
	  switch (para)
	  { 		   
		  case AWB_MODE_AUTO:
			  {
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xab); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
 			  } 			   
			  break;
			  
		  case AWB_MODE_CLOUDY_DAYLIGHT:
			  { 
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0xf0); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0xf0); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
	          }	      			   
			  break;
			  
		  case AWB_MODE_DAYLIGHT:
			  {
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0xc0); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0xc0); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);                              
    		  } 	 
			  break;
			  
		  case AWB_MODE_INCANDESCENT: 
			  {
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0xC0); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0xC0);  			  
			  } 	  
			  break;  
		  case AWB_MODE_FLUORESCENT:
			  {
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0xa0); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x90); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0xa0); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x90);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0x80);
			  }   
			  break;
			  
		  case AWB_MODE_TUNGSTEN:
			 {
    			  SOC5140_write_cmos_sensor_8(0xac01, 0xeb); 
    			  SOC5140_write_cmos_sensor_8(0xac97, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac98, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xac99, 0xb0); 
    			  SOC5140_write_cmos_sensor_8(0xAC9a, 0x80); 
    			  SOC5140_write_cmos_sensor_8(0xAC9b, 0x80);
    			  SOC5140_write_cmos_sensor_8(0xAC9c, 0xb0);  			  
			 }
			 break;
			 
		  default:
			  return ERROR_NONE;
	  }
  	  
    return ERROR_NONE; 
}

BOOL SOC5140_set_param_effect(UINT16 para)
{
	SENSORDB("\n ****** SOC5140_set_param_effect \n ");
	SENSORDB("SOC5140_set_param_effect = %d\n",para);
    switch (para)
    {
        case MEFFECT_OFF:
            {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38); 
                SOC5140_write_cmos_sensor_8(0xDC38, 0x00); 
                SOC5140_8404_Polling(0x8404, 0x06); 
            }
            break;
                                
         case MEFFECT_NEGATIVE:
            {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38);
                SOC5140_write_cmos_sensor_8(0xDC38, 0x03);
                SOC5140_8404_Polling(0x8404, 0x06);
            }
            break;
                                
         case MEFFECT_SEPIA:
            {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38);
                SOC5140_write_cmos_sensor_8(0xDC38, 0x02);
                SOC5140_write_cmos_sensor_8(0xDC3A, 0x20);
                SOC5140_write_cmos_sensor_8(0xDC3B, 0x80);
                SOC5140_8404_Polling(0x8404, 0x06);
            }             
            break;  
                                 
         case MEFFECT_SEPIAGREEN:                     
             {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38);
                SOC5140_write_cmos_sensor_8(0xDC38, 0x02);
                SOC5140_write_cmos_sensor_8(0xDC3A, 0x00);
                SOC5140_write_cmos_sensor_8(0xDC3B, 0x20);
                SOC5140_8404_Polling(0x8404, 0x06);
            }
            break;
                                
         case MEFFECT_SEPIABLUE:
            {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38);
                SOC5140_write_cmos_sensor_8(0xDC38, 0x02);
                SOC5140_write_cmos_sensor_8(0xDC3A, 0xEC);
                SOC5140_write_cmos_sensor_8(0xDC3B, 0x7F);
                SOC5140_8404_Polling(0x8404, 0x06);
            }
            break; 
         
             case MEFFECT_MONO:                                 
             {
                SOC5140_write_cmos_sensor(0x098E, 0xDC38);
                SOC5140_write_cmos_sensor_8(0xDC38, 0x01);
                SOC5140_8404_Polling(0x8404, 0x06);
             }
             break;
                                
             default:
                return KAL_FALSE;
    }
    return KAL_TRUE;
}

static void SOC5140_HVMirror(kal_uint8 image_mirror)
{
	SENSORDB("\n ****** SOC5140_HVMirror \n ");
    switch (image_mirror)
    {
    case IMAGE_NORMAL:
            SOC5140_write_cmos_sensor(0x098E, 0xC850);
            SOC5140_write_cmos_sensor_8(0xC850, 0x00);
            SOC5140_write_cmos_sensor_8(0xC888, 0x00);
            SOC5140_8404_Polling(0x8404, 0x06);    
            break;
                    
    case IMAGE_H_MIRROR:
            SOC5140_write_cmos_sensor(0x098E, 0xC850);
            SOC5140_write_cmos_sensor_8(0xC850, 0x01);
            SOC5140_write_cmos_sensor_8(0xC888, 0x01);
            SOC5140_8404_Polling(0x8404, 0x06);    
            break;
                    
    case IMAGE_V_MIRROR:
            SOC5140_write_cmos_sensor(0x098E, 0xC850);
            SOC5140_write_cmos_sensor_8(0xC850, 0x02);
            SOC5140_write_cmos_sensor_8(0xC888, 0x02);
            SOC5140_8404_Polling(0x8404, 0x06);    
            break;
    
    case IMAGE_HV_MIRROR:
            SOC5140_write_cmos_sensor(0x098E, 0xC850);
            SOC5140_write_cmos_sensor_8(0xC850, 0x03);
            SOC5140_write_cmos_sensor_8(0xC888, 0x03);
            SOC5140_8404_Polling(0x8404, 0x06);
            break;
                    
    default:
            SOC5140_write_cmos_sensor(0x098E, 0xC850);
            SOC5140_write_cmos_sensor_8(0xC850, 0x00);
            SOC5140_write_cmos_sensor_8(0xC888, 0x00);
            SOC5140_8404_Polling(0x8404, 0x06);    
            break;
    }
}

void SOC5140_set_scene_mode(UINT16 para)
{
	SENSORDB("\n ****** SOC5140_set_scene_mode \n ");	
	SENSORDB("[SOC5140_]  ******SOC5140__set_scene_mode=%d",para);	
    spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.sceneMode= para;
	spin_unlock(&soc5140_drv_lock);

    switch (para)
    { 
		case SCENE_MODE_NIGHTSCENE:
		    SOC5140_SCENE_NIGHTSCENE();
		    SOC5140_set_param_wb(awbMode);
		    SOC5140_Sensor_Driver.NightMode = KAL_TRUE;
		    SOC5140_night_mode(KAL_TRUE);
			break;
			
        case SCENE_MODE_PORTRAIT:
            SOC5140_SCENE_PORTRAIT();
		    //SOC5140_set_param_wb(AWB_MODE_AUTO);
		    SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
		    SOC5140_night_mode(KAL_FALSE);
            break;
            
        case SCENE_MODE_LANDSCAPE:
            SOC5140_SCENE_LANDSCAPE();
		    SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
            SOC5140_night_mode(KAL_FALSE);
            break;
             
        case SCENE_MODE_SUNSET:
            SOC5140_SCENE_SUNSET();
		    SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
            SOC5140_night_mode(KAL_FALSE);
            break;
            
        case SCENE_MODE_SPORTS:
            SOC5140_SCENE_SPORTS();
		    SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
            break;
            
        case SCENE_MODE_HDR:
 			SENSORDB("SCENE_MODE_HDR");	
            if (1 == SOC5140_Sensor_Driver.manualAEStart)
            {
                SOC5140_MIPI_AE_UnLock();//Manual AE disable
                spin_lock(&soc5140_drv_lock);
            	SOC5140_Sensor_Driver.manualAEStart = 0;
                SOC5140_Sensor_Driver.currentExposureTime = 0;
                SOC5140_Sensor_Driver.currentAGain = 0;
                SOC5140_Sensor_Driver.currentDGain = 0;
				spin_unlock(&soc5140_drv_lock);
            }  
		    SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
            SOC5140_night_mode(KAL_FALSE);
            break;
            
        case SCENE_MODE_OFF:
        default:
           SOC5140_SCENE_NORMAL();
		   SOC5140_set_param_wb(awbMode);
    	   SOC5140_Sensor_Driver.NightMode = KAL_FALSE;
           SOC5140_night_mode(KAL_FALSE);
           break;
    }

	SENSORDB("[SOC5140]exit SOC5140_set_scene_mode function:\n ");
	return;
}


void SOC5140_MIPI_get_exposure_gain()
{
    kal_uint32 again = 0, dgain = 0, evTime = 0;
    
    SENSORDB("\n ****** SOC5140_MIPI_get_exposure_gain\n");
    again = SOC5140_read_cmos_sensor(0x3028);
    dgain = SOC5140_read_cmos_sensor(0xc8e4);	
    evTime = SOC5140_read_cmos_sensor(0x3012);
    SENSORDB("again=%d, dgain=%d, evTime=%d", again,dgain, evTime);
    spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.currentAGain = again;
    SOC5140_Sensor_Driver.currentDGain = dgain;
    SOC5140_Sensor_Driver.currentExposureTime = evTime;
    spin_unlock(&soc5140_drv_lock);
}

void SOC5140_MIPIGetDelayInfo(UINT32 delayAddr)
{
    SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	SENSORDB("\n ****** SOC5140_MIPIGetDelayInfo \n ");
	
    pDelayInfo->InitDelay = 3;
    pDelayInfo->EffectDelay = 3;
    pDelayInfo->AwbDelay = 3;
    pDelayInfo->AFSwitchDelayFrame=50;
}

BOOL SOC5140MIPI_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 AGain = 0, DGain = 0, exposureTime = 0;
    kal_uint32 temp = 0;
	SENSORDB("\n ****** SOC5140MIPI_set_param_exposure_for_HDR \n ");

    if(0 == SOC5140_Sensor_Driver.manualAEStart)
    {       
        SOC5140_MIPI_AE_Lock();
        spin_lock(&soc5140_drv_lock);	
        SOC5140_Sensor_Driver.manualAEStart = 1;
		spin_unlock(&soc5140_drv_lock);
    }
    //Again 的一倍gain为1/32
	//Dgain 的一倍gain为 1/128
	AGain = SOC5140_Sensor_Driver.currentAGain;
	DGain = SOC5140_Sensor_Driver.currentDGain;
    exposureTime = SOC5140_Sensor_Driver.currentExposureTime;
    SENSORDB("Kevin== 0x3028_gain = 0x%x,  0x3012_exposureTime=0x%x", SOC5140_read_cmos_sensor(0x3028), SOC5140_read_cmos_sensor(0x3012));
    SENSORDB("AGain = 0x%x, DGain =0x%x, exposureTime=0x%x", AGain, DGain, exposureTime);
      
	switch (para)
	{
	   case AE_EV_COMP_20:	//+2 EV
       case AE_EV_COMP_10:	// +1 EV
            do
            {
                SENSORDB("AE_EV_COMP_20 *2\n");
                temp = SOC5140_Sensor_Driver.currentAGain*2;
                if(temp <= AGAIN_LIMITATION)
                {
                    //AGain = AGain + 8;
                    AGain = 2*SOC5140_Sensor_Driver.currentAGain;
                    SOC5140_write_cmos_sensor(0x3028, AGain);
                }
                else
                {
                   AGain = SOC5140_Sensor_Driver.currentAGain;
                   SOC5140_write_cmos_sensor(0x3028, AGain);
                }
               exposureTime = SOC5140_Sensor_Driver.currentExposureTime<<1;
               SOC5140_write_cmos_sensor(0x3012, exposureTime);	
            }while(0);
 		    break;
		 
	   case AE_EV_COMP_00:	// +0 EV  
    	   {
            SENSORDB("AE_EV_COMP_00 * 0\n");
            SOC5140_write_cmos_sensor(0x3028, SOC5140_Sensor_Driver.currentAGain);
            SOC5140_write_cmos_sensor(0x3012, SOC5140_Sensor_Driver.currentExposureTime);	
           }
		   break;
		 
	   case AE_EV_COMP_n10:  // -1 EV
	   case AE_EV_COMP_n20:  // -2 EV
            do
            {
               SENSORDB("AE_EV_COMP_n10 * 0\n");
                temp = SOC5140_Sensor_Driver.currentAGain/2; // 2X
                if(temp > 0x08 )
                {
                    //AGain = AGain - 8;
                    AGain = SOC5140_Sensor_Driver.currentAGain/2;
                    SOC5140_write_cmos_sensor(0x3028, AGain);
                }
                else
                {
                    AGain = SOC5140_Sensor_Driver.currentAGain;
                    SOC5140_write_cmos_sensor(0x3028, AGain);
                }
                exposureTime = SOC5140_Sensor_Driver.currentExposureTime>>1;
                SOC5140_write_cmos_sensor(0x3012, exposureTime);
            }while(0);
		    break;
		    
	   default:
		 break;//return FALSE;
	}
	SENSORDB("[SOC5140]exit SOC5140_set_param_exposure_for_HDR function:\n ");
	return TRUE;
}

BOOL SOC5140_set_param_exposure(UINT16 para)
{
	SENSORDB("\n ****** SOC5140_set_param_exposure \n ");
	SENSORDB("[Enter]SOC5140 set_param_exposure func:para = %d\n",para);
    if (SCENE_MODE_HDR == SOC5140_Sensor_Driver.sceneMode && 
    SOC5140_CAM_CAPTURE == SOC5140_Sensor_Driver.Camco_mode)
    {
      SENSORDB("david88 para = %d\n",para);
      SOC5140MIPI_set_param_exposure_for_HDR(para);
      return ERROR_NONE;
    }

	switch (para)
	{	
		case AE_EV_COMP_20:  //+4 EV
			//SOC5140_write_cmos_sensor(0x098E, 0xA401);		// [AE_BASETARGET]
	        //SOC5140_write_cmos_sensor_8(0xA401, 0x00);
	        //SOC5140_write_cmos_sensor_8(0xA805, 0x04);		// [SEQ_CMD]
	        SOC5140_write_cmos_sensor_8(0xA409, 0x60);
	        // SOC5140_write_cmos_sensor(0xA824, 0x0200);
	        //SOC5140_8404_Polling(0x8404, 0x06);
			break;  

		case AE_EV_COMP_10:  //+2 EV
			//SOC5140_write_cmos_sensor(0x098E, 0xA401);		// [AE_BASETARGET]
	        //SOC5140_write_cmos_sensor_8(0xA401, 0x00);
	        //SOC5140_write_cmos_sensor_8(0xA805, 0x04);		// [SEQ_CMD]
	        SOC5140_write_cmos_sensor_8(0xA409, 0x4B);
	         //SOC5140_write_cmos_sensor(0xA824, 0x0180);
	        //SOC5140_8404_Polling(0x8404, 0x06);	
	        break;    
  
		case AE_EV_COMP_00:  // +0 EV
    		//SOC5140_write_cmos_sensor(0x098E, 0xA401);		// [AE_BASETARGET] 
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00);                       
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);		// [SEQ_CMD]       
            SOC5140_write_cmos_sensor_8(0xA409, 0x38);
             //SOC5140_write_cmos_sensor(0xA824, 0x0100);
            //SOC5140_8404_Polling(0x8404, 0x06);
			break;  
			
		
		case AE_EV_COMP_n10:	// -2 EV	
            //SOC5140_write_cmos_sensor(0x098E, 0xA401);	
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00); 
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);	
            SOC5140_write_cmos_sensor_8(0xA409, 0x28);
             //SOC5140_write_cmos_sensor(0xA824, 0x0100);
            //SOC5140_8404_Polling(0x8404, 0x06); 
			break;    

		case AE_EV_COMP_n20:  // -4 EV
            //SOC5140_write_cmos_sensor(0x098E, 0xA401);	
            //SOC5140_write_cmos_sensor_8(0xA401, 0x00); 
            //SOC5140_write_cmos_sensor_8(0xA805, 0x04);	
            SOC5140_write_cmos_sensor_8(0xA409, 0x1E); 
             //SOC5140_write_cmos_sensor(0xA824, 0x0080);
            //SOC5140_8404_Polling(0x8404, 0x06); 
			break;    
		default:
			break;
	}

	return ERROR_NONE;
}

UINT32 SOC5140SetTestPatternMode(kal_bool bEnable)
{
	SENSORDB("\n ****** SOC5140SetTestPatternMode \n ");
	SENSORDB("[SOC5140SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
		//SOC5140_write_cmos_sensor(0x3070,0x0002);
		soc5140_pattern = KAL_TRUE;
	}
	else
	{
		//SOC5140_write_cmos_sensor(0x3070,0x0000);
		soc5140_pattern = KAL_FALSE;
	}
	return ERROR_NONE;
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////AF  Function///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void MiddlewaresizePoint2PreviewsizePoint(
    int mx,
    int my,
    int mw,
    int mh,
    int * pvx,
    int * pvy,
    int pvw,
    int pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    SENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}

void SOC5140_MIPI_AF_Init(void)
{
//AF initialize
//0x8419  0x00 off
//0x8419  0x03 continue on
//0x8419  0x05 single on
    SENSORDB("\n ****** SOC5140_MIPI_AF_Init\n");
    // SOC5140_write_cmos_sensor_8(0x8439, 0x03);           	// PATCH_NR_STOPS_4
    //SOC5140_write_cmos_sensor_8(0x8419, 0x03);           	// PATCH_NR_STOPS_4
    //SOC5140_8404_Polling(0x8404, 0x06);           	// PATCH_NR_STOPS_5
}

static UINT32 SOC5140_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
    SENSORDB("\n ****** SOC5140_FOCUS_Move_to\n");
	return ERROR_NONE;
}


static void SOC5140_FOCUS_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
    UINT32 af_status = 0;
    UINT32 af_progress = 0;
    *pFeatureReturnPara32 = SENSOR_AF_IDLE;

    SENSORDB("\n ****** SOC5140_FOCUS_Get_AF_Status \n ");
    af_progress =  (UINT32) SOC5140_read_cmos_sensor_8(0xb006);
    af_status =  (UINT32) SOC5140_read_cmos_sensor(0xb000);

    SENSORDB("af_progress = 0x%x \n", af_progress);
#if 0
 if(0 != af_progress)
   {     
      *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
   }
   else
   {
      af_status =  (UINT32) SOC5140_read_cmos_sensor(0xb000);
      SENSORDB("af_status = 0x%x \n", af_status);

      if (0 != (af_status & 0xF000))
      {    
         *pFeatureReturnPara32 = SENSOR_AF_ERROR;   
      }
      else
      {
         *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
      }
  }
#else
   if(0 != af_progress)
   {     
      *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
   }
   else
   {
      af_status = af_status & 0x8000;
      SENSORDB("af_status = 0x%x \n", af_status);

      if (0x0000!= af_status)
      {    
        SENSORDB("SENSOR_AF_ERROR \n");

         *pFeatureReturnPara32 = SENSOR_AF_ERROR;   
      }
      else
      {
         SENSORDB("SENSOR_AF_FOCUSED \n");
         *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
      }
  }
 #endif       
}




static void SOC5140_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void SOC5140_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void SOC5140_FOCUS_Single_Focus()
{
    SENSORDB("\n ****** SOC5140_FOCUS_Single_Focus, Touch AF \n ");
    SOC5140_write_cmos_sensor_8(0x8419,0x05);
    SOC5140_write_cmos_sensor(0xb004,0x0002);
    SOC5140_write_cmos_sensor_8(0xB006,0x01);    //trigger
}

static void SOC5140_FOCUS_Touch_AF(int x_start,int y_start)
{
	SENSORDB("\n ****** SOC5140_FOCUS_Touch_AF \n ");

//default: x: 0x60  y: 0x40 w: 0x40(0xff)    h: 0x40(0xff)   
    //SOC5140_write_cmos_sensor(0x098E, 0xB854);
//    SOC5140_write_cmos_sensor_8(0xB854,x_start);
//    SOC5140_write_cmos_sensor_8(0xB855,y_start);
//    SOC5140_write_cmos_sensor_8(0xB856,x_width);
//    SOC5140_write_cmos_sensor_8(0xB857,y_height);
//    SOC5140_write_cmos_sensor_8(0xB006,0x01);

    //SOC5140_write_cmos_sensor(0x098E, 0xB854);
    int x = 0;
    int y = 0;

    x = x_start + 0x20;
    if(x > 255)
    {
      x_start = x_start -(x - 255) - 1;
    }
   
    y = y_start + 0x20;
    if(y > 255)
    {
        y_start = y_start -(y - 255) - 1;
    }

	SENSORDB("\n x_start = %d, y_start = %d\n ",x_start,y_start );
    SOC5140_write_cmos_sensor_8(0xB854,x_start);
    SOC5140_write_cmos_sensor_8(0xB855,y_start);
    SOC5140_write_cmos_sensor_8(0xB856,0x1c);
    SOC5140_write_cmos_sensor_8(0xB857,0x1c);
	// SENSORDB("[SOC5140]AF x_start=%d,y_start=%d,x_width = %d, y_height = %d \n",x_start, y_start, x_width, y_height);
     SENSORDB("[SOC5140]exit SOC5140_FOCUS_AFC_Touch_AF function:\n ");
}

static void SOC5140_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone
	SENSORDB("\n ****** SOC5140_FOCUS_Set_AF_Window \n ");
	  int FD_XS;
	  int FD_YS;   
	  int x0, y0, x1, y1;
	  int pvx0, pvy0, pvx1, pvy1;
	  int linenum, rownum;
	  int AF_pvx, AF_pvy;
	  int* zone = (UINT32*)zone_addr;
	  x0 = *zone;
	  y0 = *(zone + 1);
	  x1 = *(zone + 2);
	  y1 = *(zone + 3);   
	  FD_XS = *(zone + 4);
	  FD_YS = *(zone + 5);
       SENSORDB("[SOC5140]AF Orig vx0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",x0, y0,x1,y1,FD_XS,FD_YS);
	  MiddlewaresizePoint2PreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, SOC5140_AF_PRV_W, SOC5140_AF_PRV_H);
	  MiddlewaresizePoint2PreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, SOC5140_AF_PRV_W, SOC5140_AF_PRV_H);  
	  SENSORDB("[SOC5140]AF pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	  SENSORDB("[SOC5140]AF pvx0=%d,pvy0=%d\n",pvx1, pvy1);
	  AF_pvx =(pvx0+pvx1)/2;
	  AF_pvy =(pvy0+pvy1)/2;
      //UI_Map_Start_XY_Fun(&AF_pvx, &AF_pvy);
	  SENSORDB("[SOC5140]AF AF_pvx=%d,AF_pvy=%d\n",AF_pvx, AF_pvy);
	  SOC5140_FOCUS_Touch_AF(AF_pvx ,AF_pvy);
	  SENSORDB("[SOC5140]exit SOC5140_FOCUS_Set_AF_Window function:\n ");
}

static void SOC5140_FOCUS_Constant_Focus(void)
{
    SENSORDB("\n ****** SOC5140_FOCUS_Constant_Focus, CAF\n");

   SOC5140_write_cmos_sensor_8(0x8419,0x03);
   SOC5140_write_cmos_sensor(0xb004, 0x0010);
   SOC5140_write_cmos_sensor_8(0xB006,0x01);    //trigger
}

static void SOC5140_FOCUS_Cancel_Focus()
{
    //SOC5140_write_cmos_sensor_8(0x8439, 0x00);           	// PATCH_NR_STOPS_4
	SENSORDB("\n ****** SOC5140_FOCUS_Cancel_Focus \n ");
    
    SOC5140_write_cmos_sensor(0xb004, 0x0001);
    SOC5140_write_cmos_sensor_8(0x8419, 0x01);
    //SOC5140_write_cmos_sensor(0x8439, 0x001);
}

static void SOC5140_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
    *pFeatureReturnPara32 = 1;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void SOC5140_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    SENSORDB("[SOC5140]enter SOC5140_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
    *pFeatureReturnPara32 = 1;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	SENSORDB("[SOC5140]exit SOC5140_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}

static void SOC5140_MIPI_AE_Set_Window2HW(
                        unsigned int s_x, 
                        unsigned int s_y, 
                        unsigned int w, 
                        unsigned int h)
{
    unsigned int x0, y0, x1, y1;

	SENSORDB("\n ****** SOC5140_MIPI_AE_Set_Window2HW \n ");


/*
REG= 0x098E, 0xB820            // LOGICAL_ADDRESS_ACCESS [STAT_AE_WINDOW_POS_X]
REG= 0xB820, 0x00                // STAT_AE_WINDOW_POS_X   起始点X
REG= 0xB821, 0x00                // STAT_AE_WINDOW_POS_Y   起始点Y
REG= 0xB822, 0xFF                // STAT_AE_WINDOW_SIZE_X   AEwindowX宽
REG= 0xB823, 0xEF                // STAT_AE_WINDOW_SIZE_Y  AE window Y宽
REG= 0xB824, 0x05                // STAT_AE_NUM_ZONES_X     window里横向分几个区域
REG= 0xB825, 0x05                // STAT_AE_NUM_ZONES_Y     window里垂直方向分多少个区域
REG= 0xB826, 0xFF                // STAT_AE_ZONE_WEIGHTS   AE 的权重，具体信息见如下

REG= 0x8404, 0x06                // trigger
      // unsigned int i;
       /*
       //SOC5140_write_cmos_sensor(0x098E, 0xB820);
       SOC5140_write_cmos_sensor_8(0xB820, s_x);
       SOC5140_write_cmos_sensor_8(0xB821, s_y);
       SOC5140_write_cmos_sensor_8(0xB822, w);
       SOC5140_write_cmos_sensor_8(0xB823, h);
       SOC5140_write_cmos_sensor_8(0xB824, 4);
       SOC5140_write_cmos_sensor_8(0xB825, 8);
       SOC5140_8404_Polling(0x8404, 0x06);
      */
       //SOC5140_write_cmos_sensor(0x098E, 0xB820);

       x0 = s_x + 0x40;
       if(x0 > 255)
       {
         s_x = s_x -(x0 - 255) - 1;
       }

       y0 = s_y + 0x40;
       if(y0 > 255)
       {
           s_y = s_y -(y0 - 255) - 1;
       }

	   SENSORDB("\n s_x = %d, s_y = %d\n ",s_x,s_y);
       SOC5140_write_cmos_sensor_8(0xB820, s_x);
       SOC5140_write_cmos_sensor_8(0xB821, s_y);
       SOC5140_write_cmos_sensor_8(0xB822, 0x40);
       SOC5140_write_cmos_sensor_8(0xB823, 0x40);
       SOC5140_write_cmos_sensor_8(0xB824, 5);
       SOC5140_write_cmos_sensor_8(0xB825, 5);
       SOC5140_8404_Polling(0x8404, 0x06);

/*
{
        int i = 0;
        int j = 0;

        while(i < 20)
        {
            j = SOC5140_read_cmos_sensor_8(0x8405);
            mdelay(5);

            i++;
        	SENSORDB("j = %d\n ", j);
        }

}
*/
    //SOC5140_MIPI_AE_UnLock();
    return;
}


static void SOC5140_FOCUS_Set_AE_Window(
    unsigned int zone_addr,
    unsigned int prevW,
    unsigned int prevH)
{
    int x0, y0, x1, y1, width, height, i, j;
    int* ptr = (unsigned int*)zone_addr;
    int srcW_maxW; //source window's max width
    int srcW_maxH; //source window's max height
    //unsigned char ae_table[8][8] = {0};
    //unsigned int  stepX, stepY;
    int start_x = 0;
    int start_y = 0;

	SENSORDB("\n ****** SOC5140_FOCUS_Set_AE_Window \n ");



    x0 = *ptr       ;
    y0 = *(ptr + 1) ;
    x1 = *(ptr + 2) ;
    y1 = *(ptr + 3) ;
    width = *(ptr + 4);
    height = *(ptr + 5);
    srcW_maxW = width;
    srcW_maxH = height;
//setting by aptina
    prevW = SOC5140_AF_PRV_W;
    prevH = SOC5140_AF_PRV_H;
    
    SENSORDB("*******SOC5140_FOCUS_Set_AE_Window*********");
    SENSORDB("x0=%d,y0=%d,x1=%d, y1=%d,width=%d,height=%d",x0,y0,x1,y1,width,height);

    spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.aeStateOnOriginalSet = 0;
    if ((x0 == x1) && (y0 == y1))
    {
        SOC5140_Sensor_Driver.aeStateOnOriginalSet = 1;
    }
    spin_unlock(&soc5140_drv_lock);


    if (x0 >= srcW_maxW)
    {
        x0 = srcW_maxW - 1;
    }

    if (x1 >= srcW_maxW)
    {
        x1 = srcW_maxW - 1;
    }

    if (y0 >= srcW_maxH)
    {
        y0 = srcW_maxH - 1;
    }

    if (y1 >= srcW_maxH)
    {
        y1 = srcW_maxH - 1;
    }
    SENSORDB("12x0=%d,y0=%d,x1=%d, y1=%d,width=%d,height=%d",x0,y0,x1,y1,width,height);

    x0 = (x0 * prevW) / srcW_maxW;
    y0 = (y0 * prevH) / srcW_maxH;
    x1 = (x1 * prevW) / srcW_maxW;
    y1 = (y1 * prevH) / srcW_maxH;
    SENSORDB("13x0=%d,y0=%d,x1=%d, y1=%d,width=%d,height=%d",x0,y0,x1,y1,width,height);

    //spin_lock(&soc5140_drv_lock);
    //SOC5140_Sensor_Driver.aeWindows[0] = x0;
    //SOC5140_Sensor_Driver.aeWindows[1] = y0;
    //SOC5140_Sensor_Driver.aeWindows[2] = x1;
    //SOC5140_Sensor_Driver.aeWindows[3] = y1;
    //spin_unlock(&soc5140_drv_lock);

    x0 = (x0 + x1) / 2;
    y0 = (y0 + y1) / 2;
    //stepX = prevW / 8;
    //stepY = prevH / 8;
    //x1 = x0 / stepX;
    //y1 = y0 / stepY;
    SENSORDB("ax0=%d,ay0=%d",x0,y0);

    SOC5140_MIPI_AE_Set_Window2HW(x0, y0, SOC5140_AF_PRV_W, SOC5140_AF_PRV_H);

    //SENSORDB("S5K4ECGX ~~~~S5K4ECGX_MIPI_AE_Set_Window: (%d,%d)~(%d,%d)\n",x0, y0, x1, y1);
  return;
}



////////////////////////////////////////////////////////////////////////////////////////
//////////////////////init & main function/////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void SOC5140_Init_Para(void)
{
	SENSORDB("******  SOC5140_Init_Para");		
	spin_lock(&soc5140_drv_lock);
	SOC5140_Sensor_Driver.Preview_PClk = 48;// 12Mhz
	SOC5140_Sensor_Driver.CapturePclk= 48;
	SOC5140_Sensor_Driver.ZsdturePclk= 48;
	SOC5140_Sensor_Driver.Banding = AE_FLICKER_MODE_50HZ;
	SOC5140_Sensor_Driver.userAskAeLock = KAL_FALSE;
	SOC5140_Sensor_Driver.userAskAwbLock = KAL_FALSE;
	SOC5140_Sensor_Driver.manualAEStart=0;	
    SOC5140_Sensor_Driver.currentExposureTime = 0;
    SOC5140_Sensor_Driver.currentAGain = 0;	
    SOC5140_Sensor_Driver.currentDGain = 0;	
	SOC5140_Sensor_Driver.Min_Frame_Rate = 75;
	SOC5140_Sensor_Driver.Max_Frame_Rate = 300;
    SOC5140_Sensor_Driver.NightMode = 0;
	SOC5140_Sensor_Driver.Camco_mode = SOC5140_CAM_PREVIEW;
	spin_unlock(&soc5140_drv_lock);
}

static kal_uint16 SOC5140_power_on(void)
{
	SOC5140_Sensor_Driver.sensor_id = 0;
	SOC5140_Sensor_Driver.sensor_id = SOC5140_read_cmos_sensor(0x0000);

	SENSORDB("****** SOC5140_power_on\n");
   	
	SENSORDB("[SOC5140]SOC5140_Sensor_Driver.sensor_id =%x\n",SOC5140_Sensor_Driver.sensor_id);
	return SOC5140_Sensor_Driver.sensor_id;
}


UINT32 SOC5140Open(void)
{
  	SENSORDB("****** SOC5140Open\n");

	 if (SOC5140_power_on() != SOC5140_SENSOR_ID) 
 	 {
 	   SENSORDB("[SOC5140]Error:read sensor ID fail\n");
	   return ERROR_SENSOR_CONNECT_FAIL;
 	 }
      
    /* Apply sensor initail setting*/
     SOC5140_Init_Para();
     SOC5140_Initial_Setting();	
     SENSORDB("[Exit]:SOC5140 Open func\n");
	return ERROR_NONE;
}	/* SOC5140Open() */


UINT32 SOC5140GetSensorID(UINT32 *sensorID)
{
  	SENSORDB("****** SOC5140GetSensorID\n");

	 *sensorID = SOC5140_power_on();

    *sensorID = SOC5140_SENSOR_ID;

	 if (*sensorID != SOC5140_SENSOR_ID) 
	 	{
	 	   SENSORDB("[SOC5140]Error:read sensor ID fail\n");
		   *sensorID = 0xFFFFFFFF;
		   return ERROR_SENSOR_CONNECT_FAIL;
	 	}
    
      
     SENSORDB("[Exit]:SOC5140 SOC5140GetSensorID func\n");
     
	return ERROR_NONE;
}	/* SOC5140Open() */

UINT32 SOC5140Close(void)
{
  	SENSORDB("****** SOC5140Close\n");

	return ERROR_NONE;
}	/* SOC5140Close() */


void SOC5140_MIPIGetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
	*pAWBlockRet32 = 1;

  	SENSORDB("****** SOC5140_MIPIGetAEAWBLock\n");
    SENSORDB("SOC5140_MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}

UINT32 SOC5140Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
     SENSORDB("******  SOC5140Capture");
     SENSORDB("%d, %d", __LINE__, SOC5140_Sensor_Driver.sceneMode);

    spin_lock(&soc5140_drv_lock);
	SOC5140_Sensor_Driver.Camco_mode = SOC5140_CAM_CAPTURE;
	spin_unlock(&soc5140_drv_lock);
	
    if((zsd_setting == 0)
        ||((1==zsd_setting)&&(SCENE_MODE_HDR == SOC5140_Sensor_Driver.sceneMode)))
    {
        SOC5140_Stream_Off();
        SOC5140_write_cmos_sensor_8(0x843C, 0xFF); 			   // (42) SYS_FIRST_BLACK_LEVEL
        SOC5140_8404_Polling(0x8404, 0x02);			   // (42) SYS_FIRST_BLACK_LEVEL
        SOC5140_8405_Polling(7);
    }
    //SOC5140_Cap_Stream_On();
	image_window->GrabStartX = SCO5140_CAP_START_X;
	image_window->GrabStartY = SCO5140_CAP_START_Y;
	image_window->ExposureWindowWidth = SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
	image_window->ExposureWindowHeight = SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
   if(SCENE_MODE_HDR == SOC5140_Sensor_Driver.sceneMode)
    {  
        SENSORDB("get exposure gain\n");
        SOC5140_MIPI_get_exposure_gain();
    }

    if(soc5140_pattern)
    {
        SOC5140SetTestPatternMode(KAL_TRUE);
    }
	
	return ERROR_NONE;
}

static UINT32 SOC5140Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("********  SOC5140MIPIPreview\n");

	spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.Camco_mode = SOC5140_CAM_PREVIEW;
	SOC5140_Sensor_Driver.Preview_PClk = 48;// 12Mhz
	SOC5140_Sensor_Driver.Min_Frame_Rate = 75;
	SOC5140_Sensor_Driver.Max_Frame_Rate = 300;
    zsd_setting = 0;
    write_statue = SOC5140_YUV_NOREFRESH;
	spin_unlock(&soc5140_drv_lock);
	
    SOC5140_8404_Polling_Sub(30, 0x01); 	   // (42) SYS_FIRST_BLACK_LEVEL
    SOC5140_write_cmos_sensor_8(0x843C, 0x01); 			   // (42) SYS_FIRST_BLACK_LEVEL
    SOC5140_8405_Polling(3);
    SOC5140_Normal_Pre();
    SOC5140_night_mode(SOC5140_Sensor_Driver.NightMode);
	//SOC5140_8404_Polling_Sub(30,0x06);
	SOC5140_8405_Polling(3);
    	
	image_window->GrabStartX = SCO5140_PRE_START_X;
	image_window->GrabStartY = SCO5140_PRE_START_Y;
	image_window->ExposureWindowWidth = SOC5140_IMAGE_SENSOR_PV_HACTIVE;
	image_window->ExposureWindowHeight = SOC5140_IMAGE_SENSOR_PV_VACTIVE;

	SENSORDB("%d, soc5140_pattern = %d \n",__LINE__, soc5140_pattern);

   // if(soc5140_pattern)
   // {
   //     SOC5140SetTestPatternMode(KAL_TRUE);
   // }

    return ERROR_NONE; 
}	/* SOC5140_Preview */

static UINT32 SOC5140Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("********  SOC5140Video\n");

	spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.Camco_mode = SOC5140_CAM_VIDEO;
	SOC5140_Sensor_Driver.Preview_PClk = 48;// 12Mhz
	SOC5140_Sensor_Driver.Min_Frame_Rate = 75;
	SOC5140_Sensor_Driver.Max_Frame_Rate = 300;
	zsd_setting = 0;
	spin_unlock(&soc5140_drv_lock);

    #if 0
    SOC5140_8404_Polling(0x8404, 0x01); 	   // (42) SYS_FIRST_BLACK_LEVEL
    SOC5140_write_cmos_sensor_8(0x843C, 0x01); 			   // (42) SYS_FIRST_BLACK_LEVEL
    SOC5140_8405_Polling(3);
    #else
    SOC5140_Normal_Pre();
    SOC5140_night_mode(SOC5140_Sensor_Driver.NightMode);
    //SOC5140_8404_Polling_Sub(30,0x06);
    #endif

    SOC5140_8405_Polling(3); 
	image_window->GrabStartX = SCO5140_PRE_START_X;
	image_window->GrabStartY = SCO5140_PRE_START_Y;
	image_window->ExposureWindowWidth = SOC5140_IMAGE_SENSOR_PV_HACTIVE;
	image_window->ExposureWindowHeight = SOC5140_IMAGE_SENSOR_PV_VACTIVE;

	SENSORDB("%d, soc5140_pattern = %d \n",__LINE__, soc5140_pattern);

   // if(soc5140_pattern)
   // {
   //     SOC5140SetTestPatternMode(KAL_TRUE);
   // }

    return ERROR_NONE; 
}

void SOC5140_Preview_ZSD_Init(void)
{
	spin_lock(&soc5140_drv_lock);
    SOC5140_Sensor_Driver.Camco_mode = SOC5140_CAM_PREVIEW_ZSD;
	SOC5140_Sensor_Driver.Preview_PClk = 48;// 12Mhz
	SOC5140_Sensor_Driver.Min_Frame_Rate = 75;
	SOC5140_Sensor_Driver.Max_Frame_Rate = 300;
    zsd_setting = 1;
    write_statue = SOC5140_YUV_NOREFRESH;
	spin_unlock(&soc5140_drv_lock);
}


static UINT32 SOC5140_MIPI_Preview_ZSD(
    MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
    MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("******  SOC5140_MIPI_Preview_ZSD\n");
/*
    //SOC5140_Initial_Setting_ZSD();
    if((SCENE_MODE_HDR != SOC5140_Sensor_Driver.sceneMode) &&
          (SOC5140_CAM_CAPTURE == SOC5140_Sensor_Driver.Camco_mode))
    {
	    SOC5140_Preview_ZSD_Init();
	}
	else
*/	
	
	{
	    SOC5140_Preview_ZSD_Init();
        SOC5140_8404_Polling_Sub(30, 0x01); 	   // (42) SYS_FIRST_BLACK_LEVEL
        SOC5140_write_cmos_sensor_8(0x843C, 0x01); 			   // (42) SYS_FIRST_BLACK_LEVEL
        SOC5140_8405_Polling(3);
    	SOC5140_Stream_Off();
    	SOC5140_ZSD_Pre();
	    SOC5140_night_mode(SOC5140_Sensor_Driver.NightMode);
        //SOC5140_8404_Polling_Sub(30,0x06);
        SOC5140_8405_Polling(3); 
	}
	
    //SOC5140_write_cmos_sensor_8(0x843C, 0xFF); 			   // (42) SYS_FIRST_BLACK_LEVEL
    //SOC5140_8404_Polling(0x8404, 0x02); 			   // (42) SYS_FIRST_BLACK_LEVEL
    //SOC5140_8405_Polling(7);
    //SOC5140_Cap_Stream_On();

	image_window->GrabStartX = SCO5140_CAP_START_X;
	image_window->GrabStartY = SCO5140_CAP_START_Y;
	image_window->ExposureWindowWidth = SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
	image_window->ExposureWindowHeight = SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
	SENSORDB("%d, soc5140_pattern = %d\n", __LINE__, soc5140_pattern);

    if(soc5140_pattern)
    {
        SOC5140SetTestPatternMode(KAL_TRUE);
    }

    return ERROR_NONE; 
}


UINT32 SOC5140GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	SENSORDB("******  SOC5140GetResolution\n");
    SENSORDB("[Enter]:SOC5140 soc5140_currentScenarioId = %d\n", soc5140_currentScenarioId);
	pSensorResolution->SensorFullWidth=SOC5140_IMAGE_SENSOR_FULL_HACTIVE;  
	pSensorResolution->SensorFullHeight=SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
    switch(soc5140_currentScenarioId)
    {
       case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorResolution->SensorPreviewWidth   = SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
            pSensorResolution->SensorPreviewHeight  = SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
            break;
            
       default:
        	pSensorResolution->SensorPreviewWidth=SOC5140_IMAGE_SENSOR_PV_HACTIVE;
        	pSensorResolution->SensorPreviewHeight=SOC5140_IMAGE_SENSOR_PV_VACTIVE;
        	pSensorResolution->SensorVideoWidth=SOC5140_IMAGE_SENSOR_PV_HACTIVE;
        	pSensorResolution->SensorVideoHeight=SOC5140_IMAGE_SENSOR_PV_VACTIVE;
            break;
    }

	
	return ERROR_NONE;
}	/* SOC5140GetResolution() */

UINT32 SOC5140GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("******  SOC5140GetInfo\n");
    SENSORDB("[Enter]:SOC5140 getInfo func:ScenarioId = %d\n",ScenarioId);

    switch(ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
          pSensorInfo->SensorPreviewResolutionX = SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
          pSensorInfo->SensorPreviewResolutionY = SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
          pSensorInfo->SensorCameraPreviewFrameRate=7;
          break;
        default:
          pSensorInfo->SensorPreviewResolutionX = SOC5140_IMAGE_SENSOR_PV_HACTIVE;
          pSensorInfo->SensorPreviewResolutionY = SOC5140_IMAGE_SENSOR_PV_VACTIVE;
          pSensorInfo->SensorCameraPreviewFrameRate=30;
          break;
    }

	pSensorInfo->SensorFullResolutionX=SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
	pSensorInfo->SensorFullResolutionY=SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=30;
	pSensorInfo->SensorWebCamCaptureFrameRate=7;
	pSensorInfo->SensorResetActiveHigh=FALSE; 
	pSensorInfo->SensorResetDelayCount=4;  
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW; 
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	pSensorInfo->SensorInterruptDelayLines = 1; 
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 2; 
	pSensorInfo->VideoDelayFrame = 2; 
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA; 		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;			
			pSensorInfo->SensorDataLatchCount= 2;				
			pSensorInfo->SensorGrabStartX = SCO5140_PRE_START_X; 
			pSensorInfo->SensorGrabStartY = SCO5140_PRE_START_Y;
			//add mipi interface setting
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:		
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;			
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = SCO5140_CAP_START_X; 
			pSensorInfo->SensorGrabStartY = SCO5140_CAP_START_Y;	
			//add mipi interface setting
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=7;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=4;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = SCO5140_PRE_START_X; 
			pSensorInfo->SensorGrabStartY = SCO5140_PRE_START_Y;
            pSensorInfo->SensorPreviewResolutionX = SOC5140_IMAGE_SENSOR_PV_HACTIVE;
            pSensorInfo->SensorPreviewResolutionY = SOC5140_IMAGE_SENSOR_PV_VACTIVE;
        	pSensorInfo->SensorFullResolutionX=SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
        	pSensorInfo->SensorFullResolutionY=SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
			//add mipi interface setting
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
		break;
	}

	memcpy(pSensorConfigData, &SOC5140SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	return ERROR_NONE;
}	/* SOC5140GetInfo() */


UINT32 SOC5140Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	SENSORDB("******  SOC5140Control\n");

   SENSORDB("[Enter]:SOC5140 Control func:ScenarioId = %d\n",ScenarioId);

    spin_lock(&soc5140_drv_lock);
	soc5140_currentScenarioId = ScenarioId;
    spin_unlock(&soc5140_drv_lock);

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			 SOC5140Preview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 SOC5140Video(pImageWindow, pSensorConfigData);
			 break;			 
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:		
			 SOC5140Capture(pImageWindow, pSensorConfigData);
			 break;
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            SOC5140_MIPI_Preview_ZSD(pImageWindow, pSensorConfigData);
            break;
			 
		default:
		     break; 
	}

   SENSORDB("[Exit]:SOC5140 Control func\n");
	
	return ERROR_NONE;
}	/* SOC5140Control() */

UINT32 SOC5140YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	SENSORDB("******  SOC5140YUVSensorSetting\n");
    SENSORDB("[Enter]SOC5140YUVSensorSetting func:cmd = %d\n",iCmd);   

    if(SOC5140_YUV_INIT == write_statue)
    {
       SENSORDB("SOC5140_YUV_INIT\n");   
       return ERROR_NONE;
    }
    else if((SOC5140_YUV_NOREFRESH == write_statue)
             && (FID_AE_FLICKER == iCmd))
    {
       SENSORDB("SOC5140_YUV_NOREFRESH\n");   
         write_statue = SOC5140_YUV_REFRESH;
    }

	switch (iCmd)
	{
    case FID_SCENE_MODE:	    //auto mode or night mode
    	   SOC5140_set_scene_mode(iPara);    	
	    break; 	  
	    
	case FID_AWB_MODE:
           SOC5140_set_param_wb(iPara);
	     break;
	     
	case FID_COLOR_EFFECT:
           SOC5140_set_param_effect(iPara);
	     break;
	     
	case FID_AE_EV:	
           SOC5140_set_param_exposure(iPara);
	     break;
	     
	case FID_AE_FLICKER:
           SOC5140_set_param_banding(iPara);
           write_statue = SOC5140_YUV_MANUALREF;
	     break;
	     
	case FID_AE_SCENE_MODE: 
		if (iPara == AE_MODE_OFF) 
		{            
		    SOC5140_MIPI_AE_Lock();
		}
    	else 
		{
            SOC5140_MIPI_AE_UnLock();
 		}
        break; 
        
	case FID_ISP_CONTRAST:
        SOC5140_MIPI_set_contrast(iPara);
        break;
        
    case FID_ISP_BRIGHT:
        SOC5140_MIPI_set_brightness(iPara);
        break;
        
    case FID_ISP_SAT:
        SOC5140_MIPI_set_saturation(iPara);
        break;
        
	case FID_ZOOM_FACTOR:
		        //SOC5140SENSORDB("FID_ZOOM_FACTOR:%d\n", iPara); 	    
				//spin_lock(&soc5140_drv_lock);
                //zoom_factor = iPara; 
				//spin_unlock(&soc5140_drv_lock);
        break;
        
	case FID_AE_ISO:
        SOC5140_MIPI_set_iso(iPara);
        break;
	     
	default:
	     break;
	}

	return ERROR_NONE;
}   /* SOC5140YUVSensorSetting */

UINT32 SOC5140YUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("*******  SOC5140YUVSetVideoMode\n");

	if (u2FrameRate == 30)
    {

        SOC5140_write_cmos_sensor(0x098E, 0x281A); 
        SOC5140_write_cmos_sensor(0xA81A, 0x03F0); 
        SOC5140_write_cmos_sensor(0xA818, 0x03d0); 
        SOC5140_write_cmos_sensor(0xC85C, 0x0423);
        SOC5140_write_cmos_sensor(0xC860, 0x0423);
        SOC5140_write_cmos_sensor(0xC868, 0x0423); 
        SOC5140_8404_Polling(0x8404, 0x06);

        if(soc5140_pattern)
        {
            SOC5140SetTestPatternMode(KAL_TRUE);
        }
    }
    else if (u2FrameRate == 15)       
    {
        SOC5140_write_cmos_sensor(0x098E, 0x281A); 
        SOC5140_write_cmos_sensor(0xA81A, 0x06f4); 
        SOC5140_write_cmos_sensor(0xA818, 0x0600); 
        SOC5140_write_cmos_sensor(0xC85C, 0x06f5);
        SOC5140_8404_Polling(0x8404, 0x06);
        if(soc5140_pattern)
        {
            SOC5140SetTestPatternMode(KAL_TRUE);
        }
    }
    else 
    {
        SENSORDB("Wrong frame rate setting \n");
    }   
	
    return ERROR_NONE;
}

void SOC5140_MIPIGetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    

	SENSORDB("******  SOC5140_MIPIGetAFMaxNumFocusAreas\n");
    
    SENSORDB("MT9V113_MIPIGetAFMaxNumFocusAreas, *pFeatureReturnPara32 = %d\n",*pFeatureReturnPara32);

}

void SOC5140_MIPIGetAFMaxNumMeteringAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;   

	SENSORDB("******  SOC5140_MIPIGetAFMaxNumMeteringAreas\n");
}

#if 0
UINT32 SOC5140MaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		SOC5140SENSORDB("SOC5140MaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		SOC5140SENSORDB("[SOC5140]enter SOC5140MaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = SOC5140_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - SOC5140_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&soc5140_drv_lock);
				SOC5140Sensor.SensorMode= SENSOR_MODE_PREVIEW;
				SOC5140Sensor.PreviewDummyLines = dummyLine;
				spin_unlock(&soc5140_drv_lock);
				SOC5140SetDummy(SOC5140Sensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = SOC5140_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - SOC5140_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&soc5140_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&soc5140_drv_lock);
				SOC5140SetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 90000000;
				lineLength = SOC5140_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - SOC5140_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&soc5140_drv_lock);
				SOC5140Sensor.CaptureDummyLines = dummyLine;
				SOC5140Sensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&soc5140_drv_lock);
				SOC5140SetDummy(SOC5140Sensor.CaptureDummyPixels, dummyLine);			
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
		SOC5140SENSORDB("[SOC5140]exit SOC5140MaxFramerateByScenario function:\n ");
		return ERROR_NONE;
}
#endif


UINT32 SOC5140MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{


	SENSORDB("******  SOC5140MIPIGetDefaultFramerateByScenario\n");

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 75;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

void SOC5140_MIPIGetExifInfo(UINT32 exifAddr)
{
	SENSORDB("****** SOC5140MIPIGetExifInfo\n");
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = AWB_MODE_AUTO;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}

void SOC5140MIPI_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{

	SENSORDB("****** SOC5140MIPI_3ACtrl\n");
	SENSORDB("[SOC5140]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&soc5140_drv_lock);
          SOC5140_Sensor_Driver.userAskAeLock = KAL_TRUE;
          spin_unlock(&soc5140_drv_lock);
          SOC5140_MIPI_AE_Lock();

      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&soc5140_drv_lock);
          SOC5140_Sensor_Driver.userAskAeLock = KAL_FALSE;
          spin_unlock(&soc5140_drv_lock);
          SOC5140_MIPI_AE_UnLock();

      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&soc5140_drv_lock);
          SOC5140_Sensor_Driver.userAskAwbLock = KAL_TRUE;
          spin_unlock(&soc5140_drv_lock);
          SOC5140_MIPI_AWB_Lock();
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&soc5140_drv_lock);
          SOC5140_Sensor_Driver.userAskAwbLock = KAL_FALSE;
          spin_unlock(&soc5140_drv_lock);
          SOC5140_MIPI_AWB_UnLock();
      break;
      default:
      	break;
   }
   SENSORDB("[SOC5140]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}

void SOC5140MIPI_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{

	SENSORDB("****** SOC5140MIPI_AutoTestCmd\n");

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
			*para=8228;
			break;
		default:
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	SENSORDB("[SOC5140]exit SOC5140_AutoTestCmd function:\n ");
}

UINT32 SOC5140FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
			*pFeatureReturnPara16++=SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
			*pFeatureReturnPara16=SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
			*pFeatureParaLen=4;
		     break;
		     
		case SENSOR_FEATURE_GET_PERIOD:
			switch(soc5140_currentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=SOC5140_IMAGE_SENSOR_FULL_HACTIVE;
					*pFeatureReturnPara16=SOC5140_IMAGE_SENSOR_FULL_VACTIVE;
					*pFeatureParaLen=4;
					break;
					
				default:
        			*pFeatureReturnPara16++=SOC5140_IMAGE_SENSOR_PV_HACTIVE;
        			*pFeatureReturnPara16=SOC5140_IMAGE_SENSOR_PV_VACTIVE;
        			*pFeatureParaLen=4;
		        break;
		     }
		     break;
		     
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:    
			switch(soc5140_currentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = SOC5140_Sensor_Driver.ZsdturePclk * 1000 *1000;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
        			*pFeatureReturnPara32 = SOC5140_Sensor_Driver.Preview_PClk* 1000*1000;
        			*pFeatureParaLen=4;
					break;
			}			
		    break;
		    
		case SENSOR_FEATURE_SET_ESHUTTER:	
		     break;
		     
		case SENSOR_FEATURE_GET_EXIF_INFO:
            SOC5140_MIPIGetExifInfo(*pFeatureData32);
            break;
		     
		case SENSOR_FEATURE_SET_NIGHTMODE:
			 SOC5140_night_mode((BOOL) *pFeatureData16);
		     break;
		     
		case SENSOR_FEATURE_SET_GAIN:
			 break; 
			 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		     break;
		     
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		     break;
		     
		case SENSOR_FEATURE_SET_REGISTER:
			 SOC5140_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		     break;
		case SENSOR_FEATURE_GET_REGISTER:
			 pSensorRegData->RegData = SOC5140_read_cmos_sensor(pSensorRegData->RegAddr);
		     break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			 memcpy(pSensorConfigData, &SOC5140SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
		case SENSOR_FEATURE_SET_TEST_PATTERN:            
			SOC5140SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
			
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=SOC5640_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
			
		case SENSOR_FEATURE_SET_YUV_CMD:
			 SOC5140YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		     break;	
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            SOC5140MIPI_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;
		     
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		     SOC5140YUVSetVideoMode(*pFeatureData16);
		     break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
             SOC5140GetSensorID(pFeatureReturnPara32); 
            break;  

		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			SOC5140_MIPIGetAEAWBLock(*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			SOC5140_MIPIGetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_AUTOTEST_CMD:
			SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			SOC5140MIPI_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;

        //below is AF control

       #if 1
        case SENSOR_FEATURE_INITIALIZE_AF:
              SOC5140_MIPI_AF_Init();
             break;

        case SENSOR_FEATURE_GET_AF_STATUS:
            SOC5140_FOCUS_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;

		case SENSOR_FEATURE_CONSTANT_AF:
			SOC5140_FOCUS_Constant_Focus();
			 break;
 
        case SENSOR_FEATURE_SET_AF_WINDOW:       
			SOC5140_FOCUS_Set_AF_Window(*pFeatureData32);
            break;

        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			SOC5140_FOCUS_Single_Focus();
            break;

        case SENSOR_FEATURE_CANCEL_AF:
            SOC5140_FOCUS_Cancel_Focus();
            break;
            
        case SENSOR_FEATURE_SET_AE_WINDOW:
         #if 0
              if (SCENE_MODE_HDR == SOC5140_Sensor_Driver.sceneMode && 
                SOC5140_CAM_CAPTURE == SOC5140_Sensor_Driver.Camco_mode)
                {
                     break;
                }
                else
                {
                   //david.liu debug
                   SOC5140_FOCUS_Set_AE_Window(*pFeatureData32, SOC5140_Sensor_Driver.Preview_Pixels_In_Line, SOC5140_Sensor_Driver.Preview_Lines_In_Frame);
                   break; 
                }
          #endif
          break;

         case SENSOR_FEATURE_GET_AF_MACRO:
            SOC5140_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;           

        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            SOC5140_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;  

        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            SOC5140_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 
            
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
             SOC5140_FOCUS_Move_to(*pFeatureData16);
            break; 
            
        case SENSOR_FEATURE_GET_AF_INF:
            SOC5140_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break; 
            #endif
		default:
			 break;			
	}
	return ERROR_NONE;
}	/* SOC5140FeatureControl() */

 SENSOR_FUNCTION_STRUCT	SensorFuncSOC5140=
{
	SOC5140Open,
	SOC5140GetInfo,
	SOC5140GetResolution,
	SOC5140FeatureControl,
	SOC5140Control,
	SOC5140Close
};

UINT32 SOC5140_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncSOC5140;

    return ERROR_NONE;
}   /* SensorInit() */

