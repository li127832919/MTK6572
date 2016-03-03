
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
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k5cagx_Sensor.h"
#include "s5k5cagx_Camera_Sensor_para.h"
#include "s5k5cagx_CameraCustomized.h"

#define S5K5CAGX_DEBUG
#define S5K5CA_TEST_PATTERN_CHECKSUM                (0x7ba87eae)
#define	S5K5CAGX_LIMIT_EXPOSURE_LINES				(1253)
#define	S5K5CAGX_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	S5K5CAGX_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	S5K5CAGX_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ

#ifdef S5K5CAGX_DEBUG
#define LOG_TAG "[SENSOR_DRV]"
#define SENSORDB(fmt,arg...) printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(fmt,arg...)  
#endif

#define Sleep(ms) mdelay(ms)

typedef enum
{
    S5K5CA_YUYV = 0,
    S5K5CA_UYVY,
    S5K5CA_YVYU,
    S5K5CA_VYUY
}S5K5CA_TYPE_ENUM;

int count_l = 0;

static int	ae_awb_state = S5K5CAGX_AEAWB_PRE;
static int write_statue = S5K5CAGX_YUV_INIT;
static int awbMode_5ca = AWB_MODE_AUTO;
static kal_uint32 zoom_factor = 0; 
static struct S5K5CAGX_sensor_struct s5k5cagx_stru;
static MSDK_SCENARIO_ID_ENUM s5k5cagx_currentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT S5K5CAGXSensorConfigData;


static void S5K5CAGX_set_AWB_Bit(int bit_op, int set_value);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
static DEFINE_SPINLOCK(s5k5cagx_drv_lock);

static void S5K5CAGX_InitialPara(void);
void S5K5CAGX_SCENE_NIGHT(void);
void S5K5CAGX_night_mode(kal_bool NightModeEnable);



//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//-------------------------------------------SENSOR INIT----------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
inline kal_uint16 S5K5CAGX_read_cmos_sensor(kal_uint16 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K5CAGX_WRITE_ID);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

inline kal_uint16 S5K5CAGX_write_cmos_sensor(kal_uint16 addr, kal_uint32 para) 
{
   char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
   iWriteRegI2C(puSendCmd , 4,S5K5CAGX_WRITE_ID);
}

void  S5K5CAGX_write_reg(kal_uint32 addr, kal_uint32 para)
{
	S5K5CAGX_write_cmos_sensor(0xFCFC,addr>>16); 
	S5K5CAGX_write_cmos_sensor(addr&0xFFFF,para);
}	/*  S5K5CAGX_write_reg() */

kal_uint32  S5K5CAGX_read_reg(kal_uint32 addr)
{
	S5K5CAGX_write_cmos_sensor(0xFCFC, addr>>16); 
	return  S5K5CAGX_read_cmos_sensor(addr&0xFFFF);
}	/*  S5K5CAGX_read_reg() */

void S5K5CAGX_select_page(kal_uint16 page)
{
	S5K5CAGX_write_cmos_sensor(0xFC, page);
}   /* S5K5CAGX_select_page */

void S5K5CAGX_set_isp_driving_current(kal_uint8 current)
{
}

void S5K5CAGX_set_dummy(kal_uint16 dummy_pixels, kal_uint16 dummy_lines)
{		
	//Adjust the extra H-Blanking & V-Blanking.
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x044C); 
	S5K5CAGX_write_cmos_sensor(0x0F12, dummy_pixels); 
	//S5K5CAGX_write_cmos_sensor(0x0F1C, dummy_pixels); 	// Extra H-Blanking
	S5K5CAGX_write_cmos_sensor(0x0F12, dummy_lines); 	// Extra V-Blanking
}

void S5K5CAGX_Initialize_Setting(void)
{
    int format = 0;

    format = S5K5CA_YUYV;

	//================================================================
	// ARM GO
	// Direct mode 
	//================================================================
	S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0010, 0x0001); // Reset
	S5K5CAGX_write_cmos_sensor(0x1030, 0x0000); // Clear host interrupt so main will wait
	S5K5CAGX_write_cmos_sensor(0x0014, 0x0001); // ARM go
	mdelay(100);	//10	// Actually delay 13.845ms , 10ms delay is enough for ARM go

	//================================================================
	// Start T&P part 2010-01-11
	//
	// DO NOT DELETE T&P SECTION COMMENTS! They are required to debug T&P related issues.
	// svn://transrdsrv/svn/svnroot/System/Software/tcevb/SDK+FW/ISP_5CA/Firmware
	// Rev: 32375-32375
	// Signature:
	// md5 78c1a0d32ef22ba270994f08d64a05a0 .btp
	// md5 6765ffc40fde4420aab81f0039a60c38 .htp
	// md5 956e8c724c34dd8b76dd297b92f59677 .RegsMap.h
	// md5 7db8e8f88de22128b8b909128f087a53 .RegsMap.bin
	// md5 506b4144bd48cdb79cbecdda4f7176ba .base.RegsMap.h
	// md5 fd8f92f13566c1a788746b23691c5f5f .base.RegsMap.bin
//////////////////20130423
	S5K5CAGX_write_cmos_sensor(0x0028, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1082);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0155);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0155);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1555);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x05D5);

	//
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x2CF8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB510);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4827);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x21C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8041);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4825);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4A26);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8382);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1D12);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x83C2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4822);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3040);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8041);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4821);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4922);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3060);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8381);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1D09);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x83C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4821);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x491D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8802);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3980);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x804A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8842);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x808A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8882);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x80CA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x88C2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x810A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8902);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x491C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x80CA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8942);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x814A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8982);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x830A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x89C2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x834A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8188);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4819);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFA0C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4819);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6341);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4919);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4819);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFA05);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4816);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3840);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x62C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x63C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4917);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6301);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4917);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3040);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6181);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4917);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4817);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9F5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4917);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4817);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9F1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4917);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4817);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9ED);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC10);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4718);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1100);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x267C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2CE8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3274);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF400);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF520);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2DF1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x89A9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2E43);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0140);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2E75);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB4F7);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2EFF);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2F23);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2FCD);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2FE1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2FB5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3067);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5823);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x30B5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD789);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB570);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6804);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6845);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6881);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6840);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2900);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD007);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x49C2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8949);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x084A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9B8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x80A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x80A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x88A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD010);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x68A9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6828);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x084A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9AC);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1D2D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xCD03);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x084A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9A5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8060);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC70);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4718);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8060);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE7F8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB510);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF9A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x48B1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x49B2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4AB2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2805);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD003);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4BB1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x795B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2B00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD005);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8008);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8010);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC10);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4718);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD1FA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8008);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8010);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE7F6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB5F8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2407);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C06);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD035);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C07);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD033);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x48A3);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8BC1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2900);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD02A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1815);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4AA4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6DEE);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A92);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4296);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD923);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3080);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x69C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF96D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C71);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0280);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF969);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4898);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0061);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1808);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8D80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A01);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0E00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF96C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6DE9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6FE8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4351);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0300);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C49);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF955);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0401);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0430);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4301);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x61F9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4990);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1810);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3080);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x61C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E64);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD2C5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2006);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF95B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2007);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF958);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBCF8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4718);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB510);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF95A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD00A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4881);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8B81);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0089);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1808);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6DC1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4883);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4281);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD901);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE7A1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE79F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB5F8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4F80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x227D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8938);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0152);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4342);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x487E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A01);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0848);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1810);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF91F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x210F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF942);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x497A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8C49);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x090E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0136);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4306);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4979);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD003);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0240);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4330);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8108);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4876);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8D00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2501);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2500);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4972);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4328);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8008);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x207D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF930);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x496E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0328);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4330);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8108);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x88F8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4310);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8088);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD00B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A01);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4869);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8F1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4969);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8809);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4348);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0C00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF918);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF91D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4865);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE7A1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB510);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF91E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4962);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8B49);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0789);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE74A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB510);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF91B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x485E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8880);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0601);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4853);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1609);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8141);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE740);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB5F8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4C54);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3420);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2500);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5765);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0039);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF913);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2600);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x57A6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4C4B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x42AE);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD01B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4D53);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8AE8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD013);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x484C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A01);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8B80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4378);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8B5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x89A9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A41);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x484D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3820);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8AC0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4348);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x17C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0D89);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1808);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1280);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8961);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8160);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE003);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x88A8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1600);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8160);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x200A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5E20);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x42B0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD011);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8AB);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1D40);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C3);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A18);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x214B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF897);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x211F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8BA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x210A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5E61);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0FC9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0149);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4301);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x483C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x81C1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE748);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB5F1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB082);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2500);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4839);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2400);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2028);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4368);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4A39);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4937);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1887);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1840);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0066);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9A01);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1980);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x218C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5A09);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8A80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8812);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8CA);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x53B8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C64);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C14);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xDBF1);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C6D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2D03);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xDBE6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9802);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0E00);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8C5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBCFE);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBC08);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4718);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB570);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6805);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2404);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8C5);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD103);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF8C9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2400);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3540);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x88E8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0500);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD403);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4822);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x89C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2800);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD002);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2008);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4304);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2010);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4304);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x481F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8B80);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0700);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F81);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2900);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4304);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x491C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8B0A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x42A2);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0762);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD502);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4A19);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3220);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8110);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x830C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE693);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0C3C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x26E8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6100);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6500);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A7C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1120);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFFF);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3374);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1D6C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x167C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF400);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C2C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x40A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DD);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF520);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C29);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A54);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1564);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF2A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2440);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3274);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x05A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2894);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1224);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1A3F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF004);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE51F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1F48);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x24BD);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x36DD);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB4CF);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xB5D7);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x36ED);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF53F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF5D9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013D);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF5C9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFAA9);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3723);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5823);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD771);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4778);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x46C0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xC000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE59F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xE12F);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xD75B);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7E77);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);

	//==========================================================================
	// CIS/APS/Analog setting        - 400LSB  SYSCLK 58MHz
	//==========================================================================
	// This registers are for FACTORY ONLY. If you change it without prior notification,
	// YOU are RESPONSIBLE for the FAILURE that will happen in the future.
	//=========================================================================
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x157A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1578);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1576);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1574);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x156E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // Slope calibration tolerance in units of 1/256
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1568);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC);

	//ADC control
	S5K5CAGX_write_cmos_sensor(0x002A, 0x155A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //ADC SAT of 450mV for 10bit default in EVT1
	S5K5CAGX_write_cmos_sensor(0x002A, 0x157E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0C80); // 3200 Max. Reset ramp DCLK counts (default 2048 0x800)
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0578); // 1400 Max. Reset ramp DCLK counts for x3.5
	S5K5CAGX_write_cmos_sensor(0x002A, 0x157C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0190); // 400 Reset ramp for x1 in DCLK counts
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1570);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A0); // 160 LSB
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); // reset threshold
	S5K5CAGX_write_cmos_sensor(0x002A, 0x12C4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); // 106 additional timing columns.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x12C8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08AC); // 2220 ADC columns in normal mode including Hold & Latch
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050); // 80 addition of ADC columns in Y-ave mode (default 244 0x74)
	//WRITE #senHal_ForceModeType			0001	// Long exposure mode

	S5K5CAGX_write_cmos_sensor(0x002A, 0x1696);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // based on APS guidelines
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // based on APS guidelines
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C6); // default. 1492 used for ADC dark characteristics
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C6); // default. 1492 used for ADC dark characteristics

	S5K5CAGX_write_cmos_sensor(0x002A, 0x12B8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0B00); //disable CINTR 0

	S5K5CAGX_write_cmos_sensor(0x002A, 0x1690);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // when set double sampling is activated - requires different set of pointers

	S5K5CAGX_write_cmos_sensor(0x002A, 0x12B0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0055); // comp and pixel bias control 0xF40E - default for EVT1
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); // comp and pixel bias control 0xF40E for binning mode

	S5K5CAGX_write_cmos_sensor(0x002A, 0x337A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); // [7] - is used for rest-only mode (EVT0 value is 0xD and HW 0x6)
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0068); 	// 104M

	S5K5CAGX_write_cmos_sensor(0x002A, 0x327C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1000); //Enable DBLR Regulation
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6998); //VPIX 2.8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0078); //[0] Static RC Filter
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04FE); //[7:4] Full RC Filter
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x8800); //Add Load to CDS block

	// IO Driving current condig.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x3274);
	/*	0x70003274
	cregs_d0_d4_cd10	R/W [9:8]	d0_4_con_cd10 ? 
								00 ? 2mA
								01 ? 4mA
								10 ? 6mA
								11 ? 8mA
							[7:6]	d0_3_con_cd10 ?
							[5:4]	d0_2_con_cd10 ?
							[3:2]	d0_1_con_cd10 ?
							[1:0]	d0_0_con_cd10 ?
	*/
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0155); // 0000 //	   Set IO driving current 4mA
	/*	0x70003276
	cregs_d5_d9_cd10	R/W [9:8]	d0_9_con_cd10 ? 
								00 ? 2mA
								01 ? 4mA
								10 ? 6mA
								11 ? 8mA
							[7:6]	d0_8_con_cd10 ?
							[5:4]	d0_7_con_cd10 ?
							[3:2]	d0_6_con_cd10 ?
							[1:0]	d0_5_con_cd10 ?
	*/
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0155); // 0000 //	   Set IO driving current
	/*	0x70003278	 */
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1555); // 0000 //	   Set IO driving current
	/*	0x7000327A
	cregs_clks_output_cd10	R/W [11:10] SDA_cd10 - Sda pad cd1/cd0 control { Sda_cd1, Sda_cd0}.
								[9:8]	SCL_cd10 - Scl pad cd1/cd0 control { Scl_cd1, Scl_cd0}.
								[7:6]	Pclk_cd10 ? PCLK pad cd1/cd0 control { Pclk _cd1, Pclk _cd0}.
								[5:4]	reserved
								[3:2]	Vsync_cd10 ? Sdat pad cd1/cd0 control { Sdat _cd1, Sdat _cd0}.
								[1:0]	Hsync_cd10 ? Hsync pad cd1/cd0 control { Hsync _cd1, Hsync _cd0}.
	*/
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x05d5); // 0000 //	   Set IO driving current

	S5K5CAGX_write_cmos_sensor(0x002A, 0x169E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); // [3:0]- specifies the target (default 7)- DCLK = 64MHz instead of 116MHz.

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0BF6);

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // 1 - Enable Bayer Downscaler for much power saving in VGA and below - #setot_bUseBayer32Bin		 


	//================================================================
	// Analog Setting Start, 2010-01-13 											   
	//================================================================
	//Asserting CDS pointers - Long exposure MS Normal
	// Conditions: 10bit, ADC_SAT = 450mV ; ramp_del = 40 ; ramp_start = 60
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x12D2); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// senHal_pContSenModesRegsArray[0][0] 	2 700012D2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// senHal_pContSenModesRegsArray[0][1] 	2 700012D4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// senHal_pContSenModesRegsArray[0][2] 	2 700012D6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// senHal_pContSenModesRegsArray[0][3] 	2 700012D8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0884); 	// senHal_pContSenModesRegsArray[1][0] 	2 700012DA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08CF); 	// senHal_pContSenModesRegsArray[1][1] 	2 700012DC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0500); 	// senHal_pContSenModesRegsArray[1][2] 	2 700012DE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x054B); 	// senHal_pContSenModesRegsArray[1][3] 	2 700012E0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[2][0] 	2 700012E2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[2][1] 	2 700012E4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[2][2] 	2 700012E6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[2][3] 	2 700012E8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0885); 	// senHal_pContSenModesRegsArray[3][0] 	2 700012EA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0467); 	// senHal_pContSenModesRegsArray[3][1] 	2 700012EC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0501); 	// senHal_pContSenModesRegsArray[3][2] 	2 700012EE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A5); 	// senHal_pContSenModesRegsArray[3][3] 	2 700012F0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[4][0] 	2 700012F2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x046A); 	// senHal_pContSenModesRegsArray[4][1] 	2 700012F4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[4][2] 	2 700012F6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A8); 	// senHal_pContSenModesRegsArray[4][3] 	2 700012F8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0885); 	// senHal_pContSenModesRegsArray[5][0] 	2 700012FA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08D0); 	// senHal_pContSenModesRegsArray[5][1] 	2 700012FC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0501); 	// senHal_pContSenModesRegsArray[5][2] 	2 700012FE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x054C); 	// senHal_pContSenModesRegsArray[5][3] 	2 70001300
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); 	// senHal_pContSenModesRegsArray[6][0] 	2 70001302
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); 	// senHal_pContSenModesRegsArray[6][1] 	2 70001304
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); 	// senHal_pContSenModesRegsArray[6][2] 	2 70001306
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); 	// senHal_pContSenModesRegsArray[6][3] 	2 70001308
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0881); 	// senHal_pContSenModesRegsArray[7][0] 	2 7000130A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0463); 	// senHal_pContSenModesRegsArray[7][1] 	2 7000130C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04FD); 	// senHal_pContSenModesRegsArray[7][2] 	2 7000130E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A1); 	// senHal_pContSenModesRegsArray[7][3] 	2 70001310
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); 	// senHal_pContSenModesRegsArray[8][0] 	2 70001312
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0489); 	// senHal_pContSenModesRegsArray[8][1] 	2 70001314
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); 	// senHal_pContSenModesRegsArray[8][2] 	2 70001316
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02C7); 	// senHal_pContSenModesRegsArray[8][3] 	2 70001318
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0881); 	// senHal_pContSenModesRegsArray[9][0] 	2 7000131A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08CC); 	// senHal_pContSenModesRegsArray[9][1] 	2 7000131C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04FD); 	// senHal_pContSenModesRegsArray[9][2] 	2 7000131E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0548); 	// senHal_pContSenModesRegsArray[9][3] 	2 70001320
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A2); 	// senHal_pContSenModesRegsArray[10][0]	2 70001322
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D3); 	// senHal_pContSenModesRegsArray[10][1]	2 70001324
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); 	// senHal_pContSenModesRegsArray[10][2]	2 70001326
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); 	// senHal_pContSenModesRegsArray[10][3]	2 70001328
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03F2); 	// senHal_pContSenModesRegsArray[11][0]	2 7000132A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0223); 	// senHal_pContSenModesRegsArray[11][1]	2 7000132C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0230); 	// senHal_pContSenModesRegsArray[11][2]	2 7000132E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0142); 	// senHal_pContSenModesRegsArray[11][3]	2 70001330
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A2); 	// senHal_pContSenModesRegsArray[12][0]	2 70001332
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x063C); 	// senHal_pContSenModesRegsArray[12][1]	2 70001334
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); 	// senHal_pContSenModesRegsArray[12][2]	2 70001336
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0399); 	// senHal_pContSenModesRegsArray[12][3]	2 70001338
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03F2); 	// senHal_pContSenModesRegsArray[13][0]	2 7000133A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x068C); 	// senHal_pContSenModesRegsArray[13][1]	2 7000133C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0230); 	// senHal_pContSenModesRegsArray[13][2]	2 7000133E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03E9); 	// senHal_pContSenModesRegsArray[13][3]	2 70001340
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[14][0]	2 70001342
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[14][1]	2 70001344
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[14][2]	2 70001346
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[14][3]	2 70001348
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); 	// senHal_pContSenModesRegsArray[15][0]	2 7000134A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); 	// senHal_pContSenModesRegsArray[15][1]	2 7000134C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); 	// senHal_pContSenModesRegsArray[15][2]	2 7000134E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); 	// senHal_pContSenModesRegsArray[15][3]	2 70001350
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D3); 	// senHal_pContSenModesRegsArray[16][0]	2 70001352
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D3); 	// senHal_pContSenModesRegsArray[16][1]	2 70001354
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); 	// senHal_pContSenModesRegsArray[16][2]	2 70001356
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); 	// senHal_pContSenModesRegsArray[16][3]	2 70001358
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020B); 	// senHal_pContSenModesRegsArray[17][0]	2 7000135A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024A); 	// senHal_pContSenModesRegsArray[17][1]	2 7000135C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012A); 	// senHal_pContSenModesRegsArray[17][2]	2 7000135E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0169); 	// senHal_pContSenModesRegsArray[17][3]	2 70001360
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[18][0]	2 70001362
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x046B); 	// senHal_pContSenModesRegsArray[18][1]	2 70001364
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// senHal_pContSenModesRegsArray[18][2]	2 70001366
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A9); 	// senHal_pContSenModesRegsArray[18][3]	2 70001368
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0419); 	// senHal_pContSenModesRegsArray[19][0]	2 7000136A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04A5); 	// senHal_pContSenModesRegsArray[19][1]	2 7000136C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0257); 	// senHal_pContSenModesRegsArray[19][2]	2 7000136E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02E3); 	// senHal_pContSenModesRegsArray[19][3]	2 70001370
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0630); 	// senHal_pContSenModesRegsArray[20][0]	2 70001372
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x063C); 	// senHal_pContSenModesRegsArray[20][1]	2 70001374
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038D); 	// senHal_pContSenModesRegsArray[20][2]	2 70001376
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0399); 	// senHal_pContSenModesRegsArray[20][3]	2 70001378
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0668); 	// senHal_pContSenModesRegsArray[21][0]	2 7000137A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x06B3); 	// senHal_pContSenModesRegsArray[21][1]	2 7000137C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C5); 	// senHal_pContSenModesRegsArray[21][2]	2 7000137E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0410); 	// senHal_pContSenModesRegsArray[21][3]	2 70001380
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[22][0]	2 70001382
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[22][1]	2 70001384
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[22][2]	2 70001386
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[22][3]	2 70001388
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A2); 	// senHal_pContSenModesRegsArray[23][0]	2 7000138A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D3); 	// senHal_pContSenModesRegsArray[23][1]	2 7000138C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); 	// senHal_pContSenModesRegsArray[23][2]	2 7000138E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); 	// senHal_pContSenModesRegsArray[23][3]	2 70001390
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[24][0]	2 70001392
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0461); 	// senHal_pContSenModesRegsArray[24][1]	2 70001394
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[24][2]	2 70001396
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029F); 	// senHal_pContSenModesRegsArray[24][3]	2 70001398
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[25][0]	2 7000139A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x063C); 	// senHal_pContSenModesRegsArray[25][1]	2 7000139C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[25][2]	2 7000139E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0399); 	// senHal_pContSenModesRegsArray[25][3]	2 700013A0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[26][0]	2 700013A2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[26][1]	2 700013A4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[26][2]	2 700013A6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[26][3]	2 700013A8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[27][0]	2 700013AA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[27][1]	2 700013AC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[27][2]	2 700013AE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[27][3]	2 700013B0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020C); 	// senHal_pContSenModesRegsArray[28][0]	2 700013B2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024B); 	// senHal_pContSenModesRegsArray[28][1]	2 700013B4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012B); 	// senHal_pContSenModesRegsArray[28][2]	2 700013B6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016A); 	// senHal_pContSenModesRegsArray[28][3]	2 700013B8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x039F); 	// senHal_pContSenModesRegsArray[29][0]	2 700013BA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x045E); 	// senHal_pContSenModesRegsArray[29][1]	2 700013BC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DD); 	// senHal_pContSenModesRegsArray[29][2]	2 700013BE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029C); 	// senHal_pContSenModesRegsArray[29][3]	2 700013C0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x041A); 	// senHal_pContSenModesRegsArray[30][0]	2 700013C2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04A6); 	// senHal_pContSenModesRegsArray[30][1]	2 700013C4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0258); 	// senHal_pContSenModesRegsArray[30][2]	2 700013C6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02E4); 	// senHal_pContSenModesRegsArray[30][3]	2 700013C8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x062D); 	// senHal_pContSenModesRegsArray[31][0]	2 700013CA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0639); 	// senHal_pContSenModesRegsArray[31][1]	2 700013CC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038A); 	// senHal_pContSenModesRegsArray[31][2]	2 700013CE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0396); 	// senHal_pContSenModesRegsArray[31][3]	2 700013D0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0669); 	// senHal_pContSenModesRegsArray[32][0]	2 700013D2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x06B4); 	// senHal_pContSenModesRegsArray[32][1]	2 700013D4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C6); 	// senHal_pContSenModesRegsArray[32][2]	2 700013D6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0411); 	// senHal_pContSenModesRegsArray[32][3]	2 700013D8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x087C); 	// senHal_pContSenModesRegsArray[33][0]	2 700013DA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08C7); 	// senHal_pContSenModesRegsArray[33][1]	2 700013DC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04F8); 	// senHal_pContSenModesRegsArray[33][2]	2 700013DE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0543); 	// senHal_pContSenModesRegsArray[33][3]	2 700013E0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[34][0]	2 700013E2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[34][1]	2 700013E4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[34][2]	2 700013E6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[34][3]	2 700013E8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[35][0]	2 700013EA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[35][1]	2 700013EC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[35][2]	2 700013EE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[35][3]	2 700013F0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); 	// senHal_pContSenModesRegsArray[36][0]	2 700013F2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024E); 	// senHal_pContSenModesRegsArray[36][1]	2 700013F4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012E); 	// senHal_pContSenModesRegsArray[36][2]	2 700013F6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016D); 	// senHal_pContSenModesRegsArray[36][3]	2 700013F8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x039F); 	// senHal_pContSenModesRegsArray[37][0]	2 700013FA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x045E); 	// senHal_pContSenModesRegsArray[37][1]	2 700013FC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DD); 	// senHal_pContSenModesRegsArray[37][2]	2 700013FE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029C); 	// senHal_pContSenModesRegsArray[37][3]	2 70001400
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x041D); 	// senHal_pContSenModesRegsArray[38][0]	2 70001402
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04A9); 	// senHal_pContSenModesRegsArray[38][1]	2 70001404
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x025B); 	// senHal_pContSenModesRegsArray[38][2]	2 70001406
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02E7); 	// senHal_pContSenModesRegsArray[38][3]	2 70001408
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x062D); 	// senHal_pContSenModesRegsArray[39][0]	2 7000140A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0639); 	// senHal_pContSenModesRegsArray[39][1]	2 7000140C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038A); 	// senHal_pContSenModesRegsArray[39][2]	2 7000140E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0396); 	// senHal_pContSenModesRegsArray[39][3]	2 70001410
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x066C); 	// senHal_pContSenModesRegsArray[40][0]	2 70001412
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x06B7); 	// senHal_pContSenModesRegsArray[40][1]	2 70001414
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C9); 	// senHal_pContSenModesRegsArray[40][2]	2 70001416
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0414); 	// senHal_pContSenModesRegsArray[40][3]	2 70001418
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x087C); 	// senHal_pContSenModesRegsArray[41][0]	2 7000141A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08C7); 	// senHal_pContSenModesRegsArray[41][1]	2 7000141C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04F8); 	// senHal_pContSenModesRegsArray[41][2]	2 7000141E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0543); 	// senHal_pContSenModesRegsArray[41][3]	2 70001420
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[42][0]	2 70001422
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[42][1]	2 70001424
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[42][2]	2 70001426
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); 	// senHal_pContSenModesRegsArray[42][3]	2 70001428
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[43][0]	2 7000142A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D0); 	// senHal_pContSenModesRegsArray[43][1]	2 7000142C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[43][2]	2 7000142E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EF); 	// senHal_pContSenModesRegsArray[43][3]	2 70001430
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); 	// senHal_pContSenModesRegsArray[44][0]	2 70001432
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024E); 	// senHal_pContSenModesRegsArray[44][1]	2 70001434
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012E); 	// senHal_pContSenModesRegsArray[44][2]	2 70001436
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016D); 	// senHal_pContSenModesRegsArray[44][3]	2 70001438
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x039F); 	// senHal_pContSenModesRegsArray[45][0]	2 7000143A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x045E); 	// senHal_pContSenModesRegsArray[45][1]	2 7000143C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DD); 	// senHal_pContSenModesRegsArray[45][2]	2 7000143E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029C); 	// senHal_pContSenModesRegsArray[45][3]	2 70001440
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x041D); 	// senHal_pContSenModesRegsArray[46][0]	2 70001442
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04A9); 	// senHal_pContSenModesRegsArray[46][1]	2 70001444
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x025B); 	// senHal_pContSenModesRegsArray[46][2]	2 70001446
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02E7); 	// senHal_pContSenModesRegsArray[46][3]	2 70001448
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x062D); 	// senHal_pContSenModesRegsArray[47][0]	2 7000144A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0639); 	// senHal_pContSenModesRegsArray[47][1]	2 7000144C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038A); 	// senHal_pContSenModesRegsArray[47][2]	2 7000144E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0396); 	// senHal_pContSenModesRegsArray[47][3]	2 70001450
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x066C); 	// senHal_pContSenModesRegsArray[48][0]	2 70001452
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x06B7); 	// senHal_pContSenModesRegsArray[48][1]	2 70001454
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C9); 	// senHal_pContSenModesRegsArray[48][2]	2 70001456
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0414); 	// senHal_pContSenModesRegsArray[48][3]	2 70001458
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x087C); 	// senHal_pContSenModesRegsArray[49][0]	2 7000145A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08C7); 	// senHal_pContSenModesRegsArray[49][1]	2 7000145C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04F8); 	// senHal_pContSenModesRegsArray[49][2]	2 7000145E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0543); 	// senHal_pContSenModesRegsArray[49][3]	2 70001460
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[50][0]	2 70001462
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[50][1]	2 70001464
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[50][2]	2 70001466
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); 	// senHal_pContSenModesRegsArray[50][3]	2 70001468
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D2); 	// senHal_pContSenModesRegsArray[51][0]	2 7000146A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D2); 	// senHal_pContSenModesRegsArray[51][1]	2 7000146C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F1); 	// senHal_pContSenModesRegsArray[51][2]	2 7000146E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F1); 	// senHal_pContSenModesRegsArray[51][3]	2 70001470
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020C); 	// senHal_pContSenModesRegsArray[52][0]	2 70001472
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024B); 	// senHal_pContSenModesRegsArray[52][1]	2 70001474
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012B); 	// senHal_pContSenModesRegsArray[52][2]	2 70001476
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016A); 	// senHal_pContSenModesRegsArray[52][3]	2 70001478
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A1); 	// senHal_pContSenModesRegsArray[53][0]	2 7000147A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0460); 	// senHal_pContSenModesRegsArray[53][1]	2 7000147C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DF); 	// senHal_pContSenModesRegsArray[53][2]	2 7000147E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029E); 	// senHal_pContSenModesRegsArray[53][3]	2 70001480
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x041A); 	// senHal_pContSenModesRegsArray[54][0]	2 70001482
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04A6); 	// senHal_pContSenModesRegsArray[54][1]	2 70001484
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0258); 	// senHal_pContSenModesRegsArray[54][2]	2 70001486
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02E4); 	// senHal_pContSenModesRegsArray[54][3]	2 70001488
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x062F); 	// senHal_pContSenModesRegsArray[55][0]	2 7000148A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x063B); 	// senHal_pContSenModesRegsArray[55][1]	2 7000148C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038C); 	// senHal_pContSenModesRegsArray[55][2]	2 7000148E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0398); 	// senHal_pContSenModesRegsArray[55][3]	2 70001490
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0669); 	// senHal_pContSenModesRegsArray[56][0]	2 70001492
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x06B4); 	// senHal_pContSenModesRegsArray[56][1]	2 70001494
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C6); 	// senHal_pContSenModesRegsArray[56][2]	2 70001496
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0411); 	// senHal_pContSenModesRegsArray[56][3]	2 70001498
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x087E); 	// senHal_pContSenModesRegsArray[57][0]	2 7000149A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x08C9); 	// senHal_pContSenModesRegsArray[57][1]	2 7000149C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04FA); 	// senHal_pContSenModesRegsArray[57][2]	2 7000149E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0545); 	// senHal_pContSenModesRegsArray[57][3]	2 700014A0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A2); 	// senHal_pContSenModesRegsArray[58][0]	2 700014A2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01D3); 	// senHal_pContSenModesRegsArray[58][1]	2 700014A4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); 	// senHal_pContSenModesRegsArray[58][2]	2 700014A6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); 	// senHal_pContSenModesRegsArray[58][3]	2 700014A8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03AF); 	// senHal_pContSenModesRegsArray[59][0]	2 700014AA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); 	// senHal_pContSenModesRegsArray[59][1]	2 700014AC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01ED); 	// senHal_pContSenModesRegsArray[59][2]	2 700014AE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); 	// senHal_pContSenModesRegsArray[59][3]	2 700014B0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[60][0]	2 700014B2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0461); 	// senHal_pContSenModesRegsArray[60][1]	2 700014B4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[60][2]	2 700014B6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029F); 	// senHal_pContSenModesRegsArray[60][3]	2 700014B8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[61][0]	2 700014BA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x046E); 	// senHal_pContSenModesRegsArray[61][1]	2 700014BC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[61][2]	2 700014BE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AC); 	// senHal_pContSenModesRegsArray[61][3]	2 700014C0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[62][0]	2 700014C2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x063C); 	// senHal_pContSenModesRegsArray[62][1]	2 700014C4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[62][2]	2 700014C6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0399); 	// senHal_pContSenModesRegsArray[62][3]	2 700014C8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[63][0]	2 700014CA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0649); 	// senHal_pContSenModesRegsArray[63][1]	2 700014CC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[63][2]	2 700014CE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A6); 	// senHal_pContSenModesRegsArray[63][3]	2 700014D0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[64][0]	2 700014D2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[64][1]	2 700014D4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[64][2]	2 700014D6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[64][3]	2 700014D8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[65][0]	2 700014DA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[65][1]	2 700014DC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[65][2]	2 700014DE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[65][3]	2 700014E0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03AA); 	// senHal_pContSenModesRegsArray[66][0]	2 700014E2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DB); 	// senHal_pContSenModesRegsArray[66][1]	2 700014E4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); 	// senHal_pContSenModesRegsArray[66][2]	2 700014E6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FA); 	// senHal_pContSenModesRegsArray[66][3]	2 700014E8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03B7); 	// senHal_pContSenModesRegsArray[67][0]	2 700014EA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); 	// senHal_pContSenModesRegsArray[67][1]	2 700014EC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01F5); 	// senHal_pContSenModesRegsArray[67][2]	2 700014EE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0107); 	// senHal_pContSenModesRegsArray[67][3]	2 700014F0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[68][0]	2 700014F2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0469); 	// senHal_pContSenModesRegsArray[68][1]	2 700014F4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[68][2]	2 700014F6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A7); 	// senHal_pContSenModesRegsArray[68][3]	2 700014F8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[69][0]	2 700014FA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0476); 	// senHal_pContSenModesRegsArray[69][1]	2 700014FC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[69][2]	2 700014FE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02B4); 	// senHal_pContSenModesRegsArray[69][3]	2 70001500
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[70][0]	2 70001502
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0644); 	// senHal_pContSenModesRegsArray[70][1]	2 70001504
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[70][2]	2 70001506
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A1); 	// senHal_pContSenModesRegsArray[70][3]	2 70001508
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[71][0]	2 7000150A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0651); 	// senHal_pContSenModesRegsArray[71][1]	2 7000150C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[71][2]	2 7000150E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03AE); 	// senHal_pContSenModesRegsArray[71][3]	2 70001510
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[72][0]	2 70001512
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[72][1]	2 70001514
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[72][2]	2 70001516
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[72][3]	2 70001518
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[73][0]	2 7000151A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[73][1]	2 7000151C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[73][2]	2 7000151E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[73][3]	2 70001520
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[74][0]	2 70001522
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[74][1]	2 70001524
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[74][2]	2 70001526
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// senHal_pContSenModesRegsArray[74][3]	2 70001528
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); 	// senHal_pContSenModesRegsArray[75][0]	2 7000152A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); 	// senHal_pContSenModesRegsArray[75][1]	2 7000152C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); 	// senHal_pContSenModesRegsArray[75][2]	2 7000152E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); 	// senHal_pContSenModesRegsArray[75][3]	2 70001530
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x05AD); 	// senHal_pContSenModesRegsArray[76][0]	2 70001532
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03DE); 	// senHal_pContSenModesRegsArray[76][1]	2 70001534
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); 	// senHal_pContSenModesRegsArray[76][2]	2 70001536
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x021C); 	// senHal_pContSenModesRegsArray[76][3]	2 70001538
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x062F); 	// senHal_pContSenModesRegsArray[77][0]	2 7000153A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0460); 	// senHal_pContSenModesRegsArray[77][1]	2 7000153C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x038C); 	// senHal_pContSenModesRegsArray[77][2]	2 7000153E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029E); 	// senHal_pContSenModesRegsArray[77][3]	2 70001540
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x07FC); 	// senHal_pContSenModesRegsArray[78][0]	2 70001542
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0847); 	// senHal_pContSenModesRegsArray[78][1]	2 70001544
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0478); 	// senHal_pContSenModesRegsArray[78][2]	2 70001546
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04C3); 	// senHal_pContSenModesRegsArray[78][3]	2 70001548
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[79][0]	2 7000154A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[79][1]	2 7000154C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[79][2]	2 7000154E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// senHal_pContSenModesRegsArray[79][3]	2 70001550
	//================================================================
	// Analog Setting End
	//================================================================

	//================================================================
	// ISP-FE Setting
	//================================================================
	S5K5CAGX_write_cmos_sensor(0x002A, 0x158A); // ISP-FE Setting
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xEAF0);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x15C6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x15BC);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0200); // added by Shy.

	S5K5CAGX_write_cmos_sensor(0x002A, 0x1608); //Analog Offset for MSM
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #gisp_msm_sAnalogOffset[0] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #gisp_msm_sAnalogOffset[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #gisp_msm_sAnalogOffset[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #gisp_msm_sAnalogOffset[3]

	//================================================================
	// Tuning part	-Calibrations go here, 2010-01-11
	//================================================================

	//==========================================================================
	// Frame rate setting 
	//
	// How to set
	// 1. Exposure value
	// dec2hex((1 / (frame rate you want(ms))) * 100d * 4d), ex, for 30fps, dec2hex(30d*100d*40d)
	// 2. Analog Digital gain
	// dec2hex((Analog gain you want) * 256d)
	//==========================================================================

	// Set preview exposure time
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0530); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2ee0); // #lt_uMaxExp1 		33.3ms							   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4E20); // #lt_uMaxExp2 		50ms							
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 						   
	S5K5CAGX_write_cmos_sensor(0x002A, 0x167C); 					
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9c40); // #evt1_lt_uMaxExp3	100ms							  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 							
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3880); // #evt1_lt_uMaxExp4	200ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	 
	// Set capture exposure time
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0538); 					
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2ee0); // #lt_uCapMaxExp1	33.3ms							   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4E20); // #lt_uCapMaxExp2		50ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1684); 												  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9c40); // #evt1_lt_uCapMaxExp3 100ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3880); // #evt1_lt_uCapMaxExp4 200ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 		   
	// Preview & Capture Gain.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0540); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0150); // #lt_uMaxAnGain1							  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0280); // #lt_uMaxAnGain2							  
	S5K5CAGX_write_cmos_sensor(0x002A, 0x168C); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A0); // #evt1_lt_uMaxAnGain3 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); // #evt1_lt_uMaxAnGain4					
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0544);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #lt_uMaxDigGain
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); // #lt_uMaxTotGain 10x

	S5K5CAGX_write_cmos_sensor(0x002A, 0x1694);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #evt1_senHal_bExpandForbid
	S5K5CAGX_write_cmos_sensor(0x002A, 0x051A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0111); // #lt_uLimitHigh 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F0); // #lt_uLimitLow


	// This code used to solve the PV/Cap birghtness non-consistency issue.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0562);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #lt_bPrevLeiForCaptureMax

	//================================================================
	// AE Target & Mode
	//================================================================
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0F70);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003A); //003B//#TVAR_ae_BrAve 091222
	// AE mode
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0F76);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F);//000D //Disable illumination & contrast// #ae_StatMode
	// AE weight
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0F7E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_0_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_1_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_2_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_3_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_4_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_5_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_6_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_7_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_8_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_9_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_10
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_11
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_12
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_13
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_14
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_15
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_16
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_17
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_18
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_19
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_20
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_21
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0303); //#ae_WeightTbl_16_22
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_23
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_24
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_25
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_26
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_27
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_28
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_29
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_30
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#ae_WeightTbl_16_31

	//================================================================
	// SET FLICKER, Default start form 50Hz
	//================================================================
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0C18);
	//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // 0001: 60Hz start auto / 0000: 50Hz start auto
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // 0001: 60Hz start auto / 0000: 50Hz start auto

	S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2);		// Auto Algorithms disable
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x067F);		// Enable Anti Flicker Auto Algorithm 

	//================================================================
	// SET GAS, Anti Shading, 2010-01-11
	//================================================================
	// GAS alpha
	// R, Gr, Gb, B per light source
	S5K5CAGX_write_cmos_sensor(0x002A, 0x06CE);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[0]//Horizon
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[4]//IncandA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[8]//WW
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[12]//CWF
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[16]//D50
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[20]//D65
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[24]//D75
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[25]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[26]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASAlpha[27]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASOutdoorAlpha[0]//Outdoor
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASOutdoorAlpha[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASOutdoorAlpha[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_GASOutdoorAlpha[3]
	// GAS beta
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[0]// Horizon
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[4]// IncandA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[8]// WW 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[10] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[11] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[12] // CWF
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[13] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[14] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[15] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[16] // D50
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[17] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[18] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[19] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[20] // D65
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[21] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[22] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[23] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[24] // D75
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[25] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[26] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASBeta[27] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASOutdoorBeta[0] // Outdoor
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASOutdoorBeta[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASOutdoorBeta[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_GASOutdoorBeta[3]
	S5K5CAGX_write_cmos_sensor(0x002A, 0x06B4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#wbt_bUseOutdoorASH ON:1 OFF:0

	// Parabloic function
	S5K5CAGX_write_cmos_sensor(0x002A, 0x075A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_bParabolicEstimation
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#ash_uParabolicCenterX
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0300); //#ash_uParabolicCenterY
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#ash_uParabolicScalingA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0011); //#ash_uParabolicScalingB

	S5K5CAGX_write_cmos_sensor(0x002A, 0x06C6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //ash_CGrasAlphas_0_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //ash_CGrasAlphas_1_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //ash_CGrasAlphas_2_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //ash_CGrasAlphas_3_

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E3C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C0); //#awbb_Alpha_Comp_Mode
	S5K5CAGX_write_cmos_sensor(0x002A, 0x074E); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#ash_bLumaMode //use Beta : 0001 not use Beta : 0000

	// GAS LUT start address // 7000_347C 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0754);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x347C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	// GAS LUT
	S5K5CAGX_write_cmos_sensor(0x002A, 0x347C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_ash_pGAS[0]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x018F); //#TVAR_ash_pGAS[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014E); //#TVAR_ash_pGAS[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_ash_pGAS[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F4); //#TVAR_ash_pGAS[4]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DE); //#TVAR_ash_pGAS[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D4); //#TVAR_ash_pGAS[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DA); //#TVAR_ash_pGAS[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F3); //#TVAR_ash_pGAS[8]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011D); //#TVAR_ash_pGAS[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0156); //#TVAR_ash_pGAS[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_ash_pGAS[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DF); //#TVAR_ash_pGAS[12]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019E); //#TVAR_ash_pGAS[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x015C); //#TVAR_ash_pGAS[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0114); //#TVAR_ash_pGAS[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D9); //#TVAR_ash_pGAS[16]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AF); //#TVAR_ash_pGAS[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0096); //#TVAR_ash_pGAS[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008D); //#TVAR_ash_pGAS[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0094); //#TVAR_ash_pGAS[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AE); //#TVAR_ash_pGAS[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DC); //#TVAR_ash_pGAS[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_ash_pGAS[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0168); //#TVAR_ash_pGAS[24]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B5); //#TVAR_ash_pGAS[25]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0169); //#TVAR_ash_pGAS[26]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0124); //#TVAR_ash_pGAS[27]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#TVAR_ash_pGAS[28]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0099); //#TVAR_ash_pGAS[29]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[30]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0055); //#TVAR_ash_pGAS[31]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004B); //#TVAR_ash_pGAS[32]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0054); //#TVAR_ash_pGAS[33]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_ash_pGAS[34]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009F); //#TVAR_ash_pGAS[35]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DF); //#TVAR_ash_pGAS[36]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0134); //#TVAR_ash_pGAS[37]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0183); //#TVAR_ash_pGAS[38]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0145); //#TVAR_ash_pGAS[39]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F8); //#TVAR_ash_pGAS[40]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A9); //#TVAR_ash_pGAS[41]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006C); //#TVAR_ash_pGAS[42]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0042); //#TVAR_ash_pGAS[43]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0029); //#TVAR_ash_pGAS[44]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); //#TVAR_ash_pGAS[45]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0029); //#TVAR_ash_pGAS[46]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0045); //#TVAR_ash_pGAS[47]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0074); //#TVAR_ash_pGAS[48]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B5); //#TVAR_ash_pGAS[49]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010D); //#TVAR_ash_pGAS[50]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x015F); //#TVAR_ash_pGAS[51]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012B); //#TVAR_ash_pGAS[52]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DD); //#TVAR_ash_pGAS[53]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008E); //#TVAR_ash_pGAS[54]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0051); //#TVAR_ash_pGAS[55]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_ash_pGAS[56]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_ash_pGAS[57]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //#TVAR_ash_pGAS[58]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0011); //#TVAR_ash_pGAS[59]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002C); //#TVAR_ash_pGAS[60]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005C); //#TVAR_ash_pGAS[61]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009F); //#TVAR_ash_pGAS[62]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F6); //#TVAR_ash_pGAS[63]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014D); //#TVAR_ash_pGAS[64]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0122); //#TVAR_ash_pGAS[65]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D3); //#TVAR_ash_pGAS[66]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0083); //#TVAR_ash_pGAS[67]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0047); //#TVAR_ash_pGAS[68]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_ash_pGAS[69]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //#TVAR_ash_pGAS[70]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_ash_pGAS[71]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0009); //#TVAR_ash_pGAS[72]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0025); //#TVAR_ash_pGAS[73]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0056); //#TVAR_ash_pGAS[74]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0099); //#TVAR_ash_pGAS[75]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F0); //#TVAR_ash_pGAS[76]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0149); //#TVAR_ash_pGAS[77]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0129); //#TVAR_ash_pGAS[78]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DC); //#TVAR_ash_pGAS[79]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008C); //#TVAR_ash_pGAS[80]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050); //#TVAR_ash_pGAS[81]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0027); //#TVAR_ash_pGAS[82]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); //#TVAR_ash_pGAS[83]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0009); //#TVAR_ash_pGAS[84]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0012); //#TVAR_ash_pGAS[85]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002E); //#TVAR_ash_pGAS[86]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); //#TVAR_ash_pGAS[87]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A3); //#TVAR_ash_pGAS[88]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[89]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0154); //#TVAR_ash_pGAS[90]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013F); //#TVAR_ash_pGAS[91]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F4); //#TVAR_ash_pGAS[92]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A6); //#TVAR_ash_pGAS[93]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); //#TVAR_ash_pGAS[94]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); //#TVAR_ash_pGAS[95]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0029); //#TVAR_ash_pGAS[96]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0021); //#TVAR_ash_pGAS[97]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002B); //#TVAR_ash_pGAS[98]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004A); //#TVAR_ash_pGAS[99]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007B); //#TVAR_ash_pGAS[100]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BF); //#TVAR_ash_pGAS[101]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0118); //#TVAR_ash_pGAS[102]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016C); //#TVAR_ash_pGAS[103]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0163); //#TVAR_ash_pGAS[104]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_ash_pGAS[105]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D0); //#TVAR_ash_pGAS[106]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0095); //#TVAR_ash_pGAS[107]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006C); //#TVAR_ash_pGAS[108]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0054); //#TVAR_ash_pGAS[109]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004C); //#TVAR_ash_pGAS[110]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0057); //#TVAR_ash_pGAS[111]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[112]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A8); //#TVAR_ash_pGAS[113]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EB); //#TVAR_ash_pGAS[114]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0141); //#TVAR_ash_pGAS[115]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0190); //#TVAR_ash_pGAS[116]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0191); //#TVAR_ash_pGAS[117]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0154); //#TVAR_ash_pGAS[118]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010E); //#TVAR_ash_pGAS[119]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#TVAR_ash_pGAS[120]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AD); //#TVAR_ash_pGAS[121]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0096); //#TVAR_ash_pGAS[122]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0090); //#TVAR_ash_pGAS[123]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009A); //#TVAR_ash_pGAS[124]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B6); //#TVAR_ash_pGAS[125]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E9); //#TVAR_ash_pGAS[126]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012C); //#TVAR_ash_pGAS[127]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017D); //#TVAR_ash_pGAS[128]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C7); //#TVAR_ash_pGAS[129]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C1); //#TVAR_ash_pGAS[130]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0183); //#TVAR_ash_pGAS[131]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0146); //#TVAR_ash_pGAS[132]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0114); //#TVAR_ash_pGAS[133]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EC); //#TVAR_ash_pGAS[134]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D8); //#TVAR_ash_pGAS[135]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D4); //#TVAR_ash_pGAS[136]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DC); //#TVAR_ash_pGAS[137]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F7); //#TVAR_ash_pGAS[138]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0126); //#TVAR_ash_pGAS[139]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0164); //#TVAR_ash_pGAS[140]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AB); //#TVAR_ash_pGAS[141]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01F6); //#TVAR_ash_pGAS[142]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016D); //#TVAR_ash_pGAS[143]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013B); //#TVAR_ash_pGAS[144]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0106); //#TVAR_ash_pGAS[145]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DA); //#TVAR_ash_pGAS[146]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B8); //#TVAR_ash_pGAS[147]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A4); //#TVAR_ash_pGAS[148]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009B); //#TVAR_ash_pGAS[149]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A1); //#TVAR_ash_pGAS[150]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B4); //#TVAR_ash_pGAS[151]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D4); //#TVAR_ash_pGAS[152]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#TVAR_ash_pGAS[153]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0135); //#TVAR_ash_pGAS[154]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_ash_pGAS[155]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014E); //#TVAR_ash_pGAS[156]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0111); //#TVAR_ash_pGAS[157]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#TVAR_ash_pGAS[158]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A6); //#TVAR_ash_pGAS[159]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0083); //#TVAR_ash_pGAS[160]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[161]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0066); //#TVAR_ash_pGAS[162]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006D); //#TVAR_ash_pGAS[163]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_ash_pGAS[164]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A0); //#TVAR_ash_pGAS[165]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CF); //#TVAR_ash_pGAS[166]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010C); //#TVAR_ash_pGAS[167]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014F); //#TVAR_ash_pGAS[168]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0127); //#TVAR_ash_pGAS[169]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E7); //#TVAR_ash_pGAS[170]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A6); //#TVAR_ash_pGAS[171]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[172]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0052); //#TVAR_ash_pGAS[173]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003E); //#TVAR_ash_pGAS[174]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0038); //#TVAR_ash_pGAS[175]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003E); //#TVAR_ash_pGAS[176]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0052); //#TVAR_ash_pGAS[177]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0074); //#TVAR_ash_pGAS[178]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A3); //#TVAR_ash_pGAS[179]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E4); //#TVAR_ash_pGAS[180]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0129); //#TVAR_ash_pGAS[181]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0107); //#TVAR_ash_pGAS[182]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_ash_pGAS[183]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0083); //#TVAR_ash_pGAS[184]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0053); //#TVAR_ash_pGAS[185]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_ash_pGAS[186]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_ash_pGAS[187]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0018); //#TVAR_ash_pGAS[188]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); //#TVAR_ash_pGAS[189]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0034); //#TVAR_ash_pGAS[190]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0056); //#TVAR_ash_pGAS[191]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0086); //#TVAR_ash_pGAS[192]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C7); //#TVAR_ash_pGAS[193]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010D); //#TVAR_ash_pGAS[194]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); //#TVAR_ash_pGAS[195]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AE); //#TVAR_ash_pGAS[196]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[197]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003E); //#TVAR_ash_pGAS[198]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_ash_pGAS[199]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000B); //#TVAR_ash_pGAS[200]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); //#TVAR_ash_pGAS[201]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000E); //#TVAR_ash_pGAS[202]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0023); //#TVAR_ash_pGAS[203]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0044); //#TVAR_ash_pGAS[204]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[205]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B6); //#TVAR_ash_pGAS[206]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_ash_pGAS[207]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E8); //#TVAR_ash_pGAS[208]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A5); //#TVAR_ash_pGAS[209]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0065); //#TVAR_ash_pGAS[210]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0037); //#TVAR_ash_pGAS[211]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0017); //#TVAR_ash_pGAS[212]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_ash_pGAS[213]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_ash_pGAS[214]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0009); //#TVAR_ash_pGAS[215]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_ash_pGAS[216]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); //#TVAR_ash_pGAS[217]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0071); //#TVAR_ash_pGAS[218]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B3); //#TVAR_ash_pGAS[219]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[220]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EC); //#TVAR_ash_pGAS[221]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A9); //#TVAR_ash_pGAS[222]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); //#TVAR_ash_pGAS[223]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); //#TVAR_ash_pGAS[224]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001D); //#TVAR_ash_pGAS[225]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000B); //#TVAR_ash_pGAS[226]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //#TVAR_ash_pGAS[227]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_ash_pGAS[228]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0026); //#TVAR_ash_pGAS[229]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0048); //#TVAR_ash_pGAS[230]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0079); //#TVAR_ash_pGAS[231]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BB); //#TVAR_ash_pGAS[232]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0103); //#TVAR_ash_pGAS[233]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[234]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BB); //#TVAR_ash_pGAS[235]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007D); //#TVAR_ash_pGAS[236]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004F); //#TVAR_ash_pGAS[237]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0030); //#TVAR_ash_pGAS[238]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_ash_pGAS[239]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001A); //#TVAR_ash_pGAS[240]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_ash_pGAS[241]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003B); //#TVAR_ash_pGAS[242]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005E); //#TVAR_ash_pGAS[243]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008F); //#TVAR_ash_pGAS[244]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D1); //#TVAR_ash_pGAS[245]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0117); //#TVAR_ash_pGAS[246]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0116); //#TVAR_ash_pGAS[247]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D9); //#TVAR_ash_pGAS[248]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_ash_pGAS[249]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[250]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004F); //#TVAR_ash_pGAS[251]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003E); //#TVAR_ash_pGAS[252]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003A); //#TVAR_ash_pGAS[253]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0045); //#TVAR_ash_pGAS[254]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005C); //#TVAR_ash_pGAS[255]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007F); //#TVAR_ash_pGAS[256]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B0); //#TVAR_ash_pGAS[257]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F2); //#TVAR_ash_pGAS[258]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0135); //#TVAR_ash_pGAS[259]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0139); //#TVAR_ash_pGAS[260]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_ash_pGAS[261]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C9); //#TVAR_ash_pGAS[262]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009D); //#TVAR_ash_pGAS[263]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007F); //#TVAR_ash_pGAS[264]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006D); //#TVAR_ash_pGAS[265]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); //#TVAR_ash_pGAS[266]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[267]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008C); //#TVAR_ash_pGAS[268]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B1); //#TVAR_ash_pGAS[269]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E2); //#TVAR_ash_pGAS[270]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0120); //#TVAR_ash_pGAS[271]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0160); //#TVAR_ash_pGAS[272]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0160); //#TVAR_ash_pGAS[273]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012A); //#TVAR_ash_pGAS[274]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F9); //#TVAR_ash_pGAS[275]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CF); //#TVAR_ash_pGAS[276]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B1); //#TVAR_ash_pGAS[277]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A0); //#TVAR_ash_pGAS[278]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009D); //#TVAR_ash_pGAS[279]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A9); //#TVAR_ash_pGAS[280]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C1); //#TVAR_ash_pGAS[281]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E6); //#TVAR_ash_pGAS[282]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0113); //#TVAR_ash_pGAS[283]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014C); //#TVAR_ash_pGAS[284]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x018E); //#TVAR_ash_pGAS[285]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016A); //#TVAR_ash_pGAS[286]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0137); //#TVAR_ash_pGAS[287]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0102); //#TVAR_ash_pGAS[288]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D6); //#TVAR_ash_pGAS[289]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B5); //#TVAR_ash_pGAS[290]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A3); //#TVAR_ash_pGAS[291]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009E); //#TVAR_ash_pGAS[292]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A9); //#TVAR_ash_pGAS[293]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C3); //#TVAR_ash_pGAS[294]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EA); //#TVAR_ash_pGAS[295]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011A); //#TVAR_ash_pGAS[296]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0155); //#TVAR_ash_pGAS[297]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0191); //#TVAR_ash_pGAS[298]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014D); //#TVAR_ash_pGAS[299]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010F); //#TVAR_ash_pGAS[300]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D3); //#TVAR_ash_pGAS[301]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A3); //#TVAR_ash_pGAS[302]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0082); //#TVAR_ash_pGAS[303]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[304]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); //#TVAR_ash_pGAS[305]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[306]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008E); //#TVAR_ash_pGAS[307]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B4); //#TVAR_ash_pGAS[308]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E8); //#TVAR_ash_pGAS[309]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0127); //#TVAR_ash_pGAS[310]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016F); //#TVAR_ash_pGAS[311]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0126); //#TVAR_ash_pGAS[312]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E5); //#TVAR_ash_pGAS[313]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A5); //#TVAR_ash_pGAS[314]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[315]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0053); //#TVAR_ash_pGAS[316]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); //#TVAR_ash_pGAS[317]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003B); //#TVAR_ash_pGAS[318]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0045); //#TVAR_ash_pGAS[319]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005E); //#TVAR_ash_pGAS[320]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0084); //#TVAR_ash_pGAS[321]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B8); //#TVAR_ash_pGAS[322]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FB); //#TVAR_ash_pGAS[323]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0142); //#TVAR_ash_pGAS[324]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0108); //#TVAR_ash_pGAS[325]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C6); //#TVAR_ash_pGAS[326]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0085); //#TVAR_ash_pGAS[327]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0055); //#TVAR_ash_pGAS[328]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0033); //#TVAR_ash_pGAS[329]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); //#TVAR_ash_pGAS[330]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001B); //#TVAR_ash_pGAS[331]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0025); //#TVAR_ash_pGAS[332]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); //#TVAR_ash_pGAS[333]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#TVAR_ash_pGAS[334]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0095); //#TVAR_ash_pGAS[335]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D9); //#TVAR_ash_pGAS[336]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011F); //#TVAR_ash_pGAS[337]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F5); //#TVAR_ash_pGAS[338]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B0); //#TVAR_ash_pGAS[339]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0071); //#TVAR_ash_pGAS[340]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0042); //#TVAR_ash_pGAS[341]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0021); //#TVAR_ash_pGAS[342]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000D); //#TVAR_ash_pGAS[343]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //#TVAR_ash_pGAS[344]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0011); //#TVAR_ash_pGAS[345]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_ash_pGAS[346]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004C); //#TVAR_ash_pGAS[347]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007E); //#TVAR_ash_pGAS[348]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C0); //#TVAR_ash_pGAS[349]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0108); //#TVAR_ash_pGAS[350]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00EE); //#TVAR_ash_pGAS[351]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AB); //#TVAR_ash_pGAS[352]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006A); //#TVAR_ash_pGAS[353]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003B); //#TVAR_ash_pGAS[354]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001A); //#TVAR_ash_pGAS[355]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); //#TVAR_ash_pGAS[356]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_ash_pGAS[357]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0009); //#TVAR_ash_pGAS[358]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); //#TVAR_ash_pGAS[359]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0043); //#TVAR_ash_pGAS[360]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0074); //#TVAR_ash_pGAS[361]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B5); //#TVAR_ash_pGAS[362]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[363]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F4); //#TVAR_ash_pGAS[364]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B2); //#TVAR_ash_pGAS[365]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0072); //#TVAR_ash_pGAS[366]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0043); //#TVAR_ash_pGAS[367]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0021); //#TVAR_ash_pGAS[368]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_ash_pGAS[369]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); //#TVAR_ash_pGAS[370]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000E); //#TVAR_ash_pGAS[371]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0023); //#TVAR_ash_pGAS[372]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_ash_pGAS[373]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0075); //#TVAR_ash_pGAS[374]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B6); //#TVAR_ash_pGAS[375]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[376]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0108); //#TVAR_ash_pGAS[377]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C7); //#TVAR_ash_pGAS[378]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0088); //#TVAR_ash_pGAS[379]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0057); //#TVAR_ash_pGAS[380]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0034); //#TVAR_ash_pGAS[381]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001F); //#TVAR_ash_pGAS[382]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0018); //#TVAR_ash_pGAS[383]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001F); //#TVAR_ash_pGAS[384]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0034); //#TVAR_ash_pGAS[385]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0057); //#TVAR_ash_pGAS[386]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0085); //#TVAR_ash_pGAS[387]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C5); //#TVAR_ash_pGAS[388]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010A); //#TVAR_ash_pGAS[389]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0125); //#TVAR_ash_pGAS[390]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E5); //#TVAR_ash_pGAS[391]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A8); //#TVAR_ash_pGAS[392]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0078); //#TVAR_ash_pGAS[393]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0055); //#TVAR_ash_pGAS[394]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003F); //#TVAR_ash_pGAS[395]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0037); //#TVAR_ash_pGAS[396]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003E); //#TVAR_ash_pGAS[397]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0052); //#TVAR_ash_pGAS[398]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0074); //#TVAR_ash_pGAS[399]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A2); //#TVAR_ash_pGAS[400]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E0); //#TVAR_ash_pGAS[401]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0122); //#TVAR_ash_pGAS[402]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0149); //#TVAR_ash_pGAS[403]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0110); //#TVAR_ash_pGAS[404]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D7); //#TVAR_ash_pGAS[405]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A8); //#TVAR_ash_pGAS[406]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0085); //#TVAR_ash_pGAS[407]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006F); //#TVAR_ash_pGAS[408]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0067); //#TVAR_ash_pGAS[409]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_ash_pGAS[410]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0081); //#TVAR_ash_pGAS[411]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A2); //#TVAR_ash_pGAS[412]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CF); //#TVAR_ash_pGAS[413]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0108); //#TVAR_ash_pGAS[414]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014A); //#TVAR_ash_pGAS[415]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0178); //#TVAR_ash_pGAS[416]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013B); //#TVAR_ash_pGAS[417]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0109); //#TVAR_ash_pGAS[418]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DE); //#TVAR_ash_pGAS[419]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BB); //#TVAR_ash_pGAS[420]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A5); //#TVAR_ash_pGAS[421]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_ash_pGAS[422]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A2); //#TVAR_ash_pGAS[423]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B4); //#TVAR_ash_pGAS[424]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#TVAR_ash_pGAS[425]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FD); //#TVAR_ash_pGAS[426]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0133); //#TVAR_ash_pGAS[427]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0177); //#TVAR_ash_pGAS[428]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012B); //#TVAR_ash_pGAS[429]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0103); //#TVAR_ash_pGAS[430]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#TVAR_ash_pGAS[431]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B4); //#TVAR_ash_pGAS[432]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009D); //#TVAR_ash_pGAS[433]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0090); //#TVAR_ash_pGAS[434]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008E); //#TVAR_ash_pGAS[435]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0098); //#TVAR_ash_pGAS[436]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AC); //#TVAR_ash_pGAS[437]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CE); //#TVAR_ash_pGAS[438]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FA); //#TVAR_ash_pGAS[439]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0131); //#TVAR_ash_pGAS[440]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0166); //#TVAR_ash_pGAS[441]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0110); //#TVAR_ash_pGAS[442]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DA); //#TVAR_ash_pGAS[443]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A8); //#TVAR_ash_pGAS[444]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0084); //#TVAR_ash_pGAS[445]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006C); //#TVAR_ash_pGAS[446]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); //#TVAR_ash_pGAS[447]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005E); //#TVAR_ash_pGAS[448]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0068); //#TVAR_ash_pGAS[449]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007B); //#TVAR_ash_pGAS[450]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_ash_pGAS[451]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CB); //#TVAR_ash_pGAS[452]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0105); //#TVAR_ash_pGAS[453]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0146); //#TVAR_ash_pGAS[454]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00E6); //#TVAR_ash_pGAS[455]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B2); //#TVAR_ash_pGAS[456]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007F); //#TVAR_ash_pGAS[457]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); //#TVAR_ash_pGAS[458]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0043); //#TVAR_ash_pGAS[459]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0038); //#TVAR_ash_pGAS[460]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0035); //#TVAR_ash_pGAS[461]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); //#TVAR_ash_pGAS[462]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0051); //#TVAR_ash_pGAS[463]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_ash_pGAS[464]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_ash_pGAS[465]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D9); //#TVAR_ash_pGAS[466]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011A); //#TVAR_ash_pGAS[467]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C9); //#TVAR_ash_pGAS[468]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0093); //#TVAR_ash_pGAS[469]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); //#TVAR_ash_pGAS[470]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003D); //#TVAR_ash_pGAS[471]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0027); //#TVAR_ash_pGAS[472]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001C); //#TVAR_ash_pGAS[473]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_ash_pGAS[474]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0021); //#TVAR_ash_pGAS[475]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0033); //#TVAR_ash_pGAS[476]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0051); //#TVAR_ash_pGAS[477]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007B); //#TVAR_ash_pGAS[478]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B6); //#TVAR_ash_pGAS[479]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F6); //#TVAR_ash_pGAS[480]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B4); //#TVAR_ash_pGAS[481]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007E); //#TVAR_ash_pGAS[482]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004C); //#TVAR_ash_pGAS[483]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_ash_pGAS[484]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0015); //#TVAR_ash_pGAS[485]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_ash_pGAS[486]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //#TVAR_ash_pGAS[487]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000E); //#TVAR_ash_pGAS[488]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001F); //#TVAR_ash_pGAS[489]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003A); //#TVAR_ash_pGAS[490]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0064); //#TVAR_ash_pGAS[491]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009D); //#TVAR_ash_pGAS[492]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DC); //#TVAR_ash_pGAS[493]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AE); //#TVAR_ash_pGAS[494]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0076); //#TVAR_ash_pGAS[495]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0045); //#TVAR_ash_pGAS[496]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_ash_pGAS[497]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000E); //#TVAR_ash_pGAS[498]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_ash_pGAS[499]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_ash_pGAS[500]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); //#TVAR_ash_pGAS[501]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0016); //#TVAR_ash_pGAS[502]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0031); //#TVAR_ash_pGAS[503]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0059); //#TVAR_ash_pGAS[504]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0091); //#TVAR_ash_pGAS[505]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CF); //#TVAR_ash_pGAS[506]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B2); //#TVAR_ash_pGAS[507]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007C); //#TVAR_ash_pGAS[508]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x004B); //#TVAR_ash_pGAS[509]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0029); //#TVAR_ash_pGAS[510]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0014); //#TVAR_ash_pGAS[511]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0008); //#TVAR_ash_pGAS[512]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#TVAR_ash_pGAS[513]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_ash_pGAS[514]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_ash_pGAS[515]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0033); //#TVAR_ash_pGAS[516]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); //#TVAR_ash_pGAS[517]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0090); //#TVAR_ash_pGAS[518]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CE); //#TVAR_ash_pGAS[519]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C3); //#TVAR_ash_pGAS[520]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008E); //#TVAR_ash_pGAS[521]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005D); //#TVAR_ash_pGAS[522]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003B); //#TVAR_ash_pGAS[523]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_ash_pGAS[524]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0018); //#TVAR_ash_pGAS[525]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0014); //#TVAR_ash_pGAS[526]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_ash_pGAS[527]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_ash_pGAS[528]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0041); //#TVAR_ash_pGAS[529]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0067); //#TVAR_ash_pGAS[530]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009D); //#TVAR_ash_pGAS[531]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DA); //#TVAR_ash_pGAS[532]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DF); //#TVAR_ash_pGAS[533]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AA); //#TVAR_ash_pGAS[534]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x007A); //#TVAR_ash_pGAS[535]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0058); //#TVAR_ash_pGAS[536]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0041); //#TVAR_ash_pGAS[537]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0034); //#TVAR_ash_pGAS[538]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002F); //#TVAR_ash_pGAS[539]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0034); //#TVAR_ash_pGAS[540]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0043); //#TVAR_ash_pGAS[541]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005C); //#TVAR_ash_pGAS[542]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0081); //#TVAR_ash_pGAS[543]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B7); //#TVAR_ash_pGAS[544]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F0); //#TVAR_ash_pGAS[545]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0106); //#TVAR_ash_pGAS[546]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D4); //#TVAR_ash_pGAS[547]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00A6); //#TVAR_ash_pGAS[548]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0083); //#TVAR_ash_pGAS[549]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006C); //#TVAR_ash_pGAS[550]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005E); //#TVAR_ash_pGAS[551]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); //#TVAR_ash_pGAS[552]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005F); //#TVAR_ash_pGAS[553]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006C); //#TVAR_ash_pGAS[554]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0085); //#TVAR_ash_pGAS[555]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AB); //#TVAR_ash_pGAS[556]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DE); //#TVAR_ash_pGAS[557]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0119); //#TVAR_ash_pGAS[558]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0130); //#TVAR_ash_pGAS[559]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#TVAR_ash_pGAS[560]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D3); //#TVAR_ash_pGAS[561]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B2); //#TVAR_ash_pGAS[562]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009B); //#TVAR_ash_pGAS[563]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008F); //#TVAR_ash_pGAS[564]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0089); //#TVAR_ash_pGAS[565]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x008C); //#TVAR_ash_pGAS[566]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0099); //#TVAR_ash_pGAS[567]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B2); //#TVAR_ash_pGAS[568]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D4); //#TVAR_ash_pGAS[569]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0105); //#TVAR_ash_pGAS[570]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0142); //#TVAR_ash_pGAS[571]
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D30);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A7); //#awbb_GLocusR
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0343); //#awbb_GLocusB
	S5K5CAGX_write_cmos_sensor(0x002A, 0x06B8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D0); //#TVAR_ash_AwbAshCord_0_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0102); //#TVAR_ash_AwbAshCord_1_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010E); //#TVAR_ash_AwbAshCord_2_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0137); //#TVAR_ash_AwbAshCord_3_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0171); //#TVAR_ash_AwbAshCord_4_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0198); //#TVAR_ash_AwbAshCord_5_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01A8); //#TVAR_ash_AwbAshCord_6_

	//================================================================================================
	// SET CCM
	//================================================================================================ 
	// CCM start address // 7000_33A4
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0698);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x33A4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	// Horizon
	S5K5CAGX_write_cmos_sensor(0x002A, 0x33A4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CB); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF8E); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFD2); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms
	// Inca
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_wbt_pBaseCcms[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF7F); //#TVAR_wbt_pBaseCcms[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE4); //#TVAR_wbt_pBaseCcms[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms[24]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms[25]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms[26]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms[27]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms[28]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms[29]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms[30]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms[31]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms[32]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms[33]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms[34]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms[35]
	// WW
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_wbt_pBaseCcms[36]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF7F); //#TVAR_wbt_pBaseCcms[37]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE4); //#TVAR_wbt_pBaseCcms[38]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms[39]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms[40]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms[41]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms[42]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms[43]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms[44]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms[45]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms[46]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms[47]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms[48]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms[49]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms[50]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms[51]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms[52]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms[53]
	// CWF
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_wbt_pBaseCcms[54]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF7F); //#TVAR_wbt_pBaseCcms[55]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE4); //#TVAR_wbt_pBaseCcms[56]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms[57]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms[58]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms[59]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms[60]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms[61]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms[62]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms[63]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms[64]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms[65]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms[66]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms[67]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms[68]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms[69]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms[70]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms[71]
	// D50
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_wbt_pBaseCcms[72]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF7F); //#TVAR_wbt_pBaseCcms[73]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE4); //#TVAR_wbt_pBaseCcms[74]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms[75]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms[76]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms[77]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms[78]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms[79]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms[80]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms[81]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms[82]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms[83]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms[84]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms[85]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms[86]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms[87]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms[88]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms[89]
	// D65
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01C8); //#TVAR_wbt_pBaseCcms[90]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF7F); //#TVAR_wbt_pBaseCcms[91]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE4); //#TVAR_wbt_pBaseCcms[92]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF64); //#TVAR_wbt_pBaseCcms[93]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01B2); //#TVAR_wbt_pBaseCcms[94]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF35); //#TVAR_wbt_pBaseCcms[95]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFDF); //#TVAR_wbt_pBaseCcms[96]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFE9); //#TVAR_wbt_pBaseCcms[97]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pBaseCcms[98]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011C); //#TVAR_wbt_pBaseCcms[99]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011B); //#TVAR_wbt_pBaseCcms[100]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF43); //#TVAR_wbt_pBaseCcms[101]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pBaseCcms[102]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF4C); //#TVAR_wbt_pBaseCcms[103]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CC); //#TVAR_wbt_pBaseCcms[104]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF33); //#TVAR_wbt_pBaseCcms[105]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0173); //#TVAR_wbt_pBaseCcms[106]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012F); //#TVAR_wbt_pBaseCcms[107]
	// Outdoor CCM address // 7000_3380
	S5K5CAGX_write_cmos_sensor(0x002A, 0x06A0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3380);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	// Outdoor CCM

	S5K5CAGX_write_cmos_sensor(0x002A, 0x3380);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); //#TVAR_wbt_pOutdoorCcm[0]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF80); //#TVAR_wbt_pOutdoorCcm[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFD0); //#TVAR_wbt_pOutdoorCcm[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF61); //#TVAR_wbt_pOutdoorCcm[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BD); //#TVAR_wbt_pOutdoorCcm[4]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF34); //#TVAR_wbt_pOutdoorCcm[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFFE); //#TVAR_wbt_pOutdoorCcm[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF6); //#TVAR_wbt_pOutdoorCcm[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x019D); //#TVAR_wbt_pOutdoorCcm[8]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0107); //#TVAR_wbt_pOutdoorCcm[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010F); //#TVAR_wbt_pOutdoorCcm[10] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF67); //#TVAR_wbt_pOutdoorCcm[11] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x016C); //#TVAR_wbt_pOutdoorCcm[12] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF54); //#TVAR_wbt_pOutdoorCcm[13] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01FC); //#TVAR_wbt_pOutdoorCcm[14] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF82); //#TVAR_wbt_pOutdoorCcm[15] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x015D); //#TVAR_wbt_pOutdoorCcm[16] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FD); //#TVAR_wbt_pOutdoorCcm[17] 
	//================================================================================================
	// SET AWB
	//================================================================================================
	// Indoor boundary
	// 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0C48);
//2013/06/15
S5K5CAGX_write_cmos_sensor(0x0f12, 0x03C8);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x03DE);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0372);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x03EA);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0336);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x03DE);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0302);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x03A2);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x02C8);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0368);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0292);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x033A);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0276);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0317);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x025A);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x02EE);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0246);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x02D1);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0232);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x02B8);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x021E);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0298);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0208);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0280);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x01EE);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0267);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x01F0);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0248);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
S5K5CAGX_write_cmos_sensor(0x0f12, 0x0000);
                                       





#if 0	
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C8); //awbb_IndoorGrZones_m_BGrid[0] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03DE); //awbb_IndoorGrZones_m_BGrid[1] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0372); //awbb_IndoorGrZones_m_BGrid[2] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03EA); //awbb_IndoorGrZones_m_BGrid[3] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0336); //awbb_IndoorGrZones_m_BGrid[4] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03DE); //awbb_IndoorGrZones_m_BGrid[5] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //awbb_IndoorGrZones_m_BGrid[6] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03A2); //awbb_IndoorGrZones_m_BGrid[7] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02C8); //awbb_IndoorGrZones_m_BGrid[8] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0368); //awbb_IndoorGrZones_m_BGrid[9] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0292); //awbb_IndoorGrZones_m_BGrid[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x033A); //awbb_IndoorGrZones_m_BGrid[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0276); //awbb_IndoorGrZones_m_BGrid[12]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0306); //awbb_IndoorGrZones_m_BGrid[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x025A); //awbb_IndoorGrZones_m_BGrid[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02C2); //awbb_IndoorGrZones_m_BGrid[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0246); //awbb_IndoorGrZones_m_BGrid[16]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A2); //awbb_IndoorGrZones_m_BGrid[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //awbb_IndoorGrZones_m_BGrid[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0298); //awbb_IndoorGrZones_m_BGrid[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x021E); //awbb_IndoorGrZones_m_BGrid[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x027E); //awbb_IndoorGrZones_m_BGrid[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0208); //awbb_IndoorGrZones_m_BGrid[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x026C); //awbb_IndoorGrZones_m_BGrid[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01EE); //awbb_IndoorGrZones_m_BGrid[24]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x025C); //awbb_IndoorGrZones_m_BGrid[25]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01F0); //awbb_IndoorGrZones_m_BGrid[26]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0248); //awbb_IndoorGrZones_m_BGrid[27]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[28]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[29]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[30]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[31]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[32]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[33]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[34]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[35]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[36]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[37]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[38]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //awbb_IndoorGrZones_m_BGrid[39]
#endif


	
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);// #awbb_IndoorGrZones_m_GridStep
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0C9C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000E);	
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0CA0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0108); // #awbb_IndoorGrZones_m_Boffs




	// Outdoor boundary
	//
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0CA4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x026E); // #awbb_OutdoorGrZones_m_BGrid[0] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A4); // #awbb_OutdoorGrZones_m_BGrid[1] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x024D); // #awbb_OutdoorGrZones_m_BGrid[2] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A8); // #awbb_OutdoorGrZones_m_BGrid[3] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x023C); // #awbb_OutdoorGrZones_m_BGrid[4] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029D); // #awbb_OutdoorGrZones_m_BGrid[5] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0222); // #awbb_OutdoorGrZones_m_BGrid[6] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x028F); // #awbb_OutdoorGrZones_m_BGrid[7] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0206); // #awbb_OutdoorGrZones_m_BGrid[8] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0279); // #awbb_OutdoorGrZones_m_BGrid[9] 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01EE); // #awbb_OutdoorGrZones_m_BGrid[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0261); // #awbb_OutdoorGrZones_m_BGrid[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DE); // #awbb_OutdoorGrZones_m_BGrid[12]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0242); // #awbb_OutdoorGrZones_m_BGrid[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01DB); // #awbb_OutdoorGrZones_m_BGrid[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0216); // #awbb_OutdoorGrZones_m_BGrid[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[16]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #awbb_OutdoorGrZones_m_BGrid[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); // #awbb_OutdoorGrZones_m_GridStep
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0CD8);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0008); // #awbb_OutdoorGrZones_m_Boffs
	
		S5K5CAGX_write_cmos_sensor(0x002A, 0x0CDC);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204);
	 
/*	// Outdoor detection zone??
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D88);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0038); //#awbb_OutdoorDetectionZone_m_BGrid[0]_m_left
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0074); //#awbb_OutdoorDetectionZone_m_BGrid[0]_m_right
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF1); //#awbb_OutdoorDetectionZone_m_BGrid[1]_m_left
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BF); //#awbb_OutdoorDetectionZone_m_BGrid[1]_m_right
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF9B); //#awbb_OutdoorDetectionZone_m_BGrid[2]_m_left
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00DB); //#awbb_OutdoorDetectionZone_m_BGrid[2]_m_right
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF56); //#awbb_OutdoorDetectionZone_m_BGrid[3]_m_left
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F0); //#awbb_OutdoorDetectionZone_m_BGrid[3]_m_right
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFEFF); //#awbb_OutdoorDetectionZone_m_BGrid[4]_m_left
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010F); //#awbb_OutdoorDetectionZone_m_BGrid[4]_m_right
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0E74); //#awbb_OutdoorDetectionZone_ZInfo_m_AbsGridStep
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#awbb_OutdoorDetectionZone_ZInfo_m_GridSz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1387); //#awbb_OutdoorDetectionZone_ZInfo_m_NBoffs
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1388); //#awbb_OutdoorDetectionZone_ZInfo_m_MaxNB 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
*/
	// LowBr boundary
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0CE0);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03D4); //#awbb_LowBrGrZones_m_BGrid[0]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043E); //#awbb_LowBrGrZones_m_BGrid[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x035C); //#awbb_LowBrGrZones_m_BGrid[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0438); //#awbb_LowBrGrZones_m_BGrid[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02F0); //#awbb_LowBrGrZones_m_BGrid[4]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x042D); //#awbb_LowBrGrZones_m_BGrid[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029A); //#awbb_LowBrGrZones_m_BGrid[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03EF); //#awbb_LowBrGrZones_m_BGrid[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x025E); //#awbb_LowBrGrZones_m_BGrid[8]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0395); //#awbb_LowBrGrZones_m_BGrid[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x022E); //#awbb_LowBrGrZones_m_BGrid[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0346); //#awbb_LowBrGrZones_m_BGrid[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0200); //#awbb_LowBrGrZones_m_BGrid[12]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02F6); //#awbb_LowBrGrZones_m_BGrid[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01CE); //#awbb_LowBrGrZones_m_BGrid[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02C8); //#awbb_LowBrGrZones_m_BGrid[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BB); //#awbb_LowBrGrZones_m_BGrid[16]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0287); //#awbb_LowBrGrZones_m_BGrid[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E2); //#awbb_LowBrGrZones_m_BGrid[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0239); //#awbb_LowBrGrZones_m_BGrid[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_LowBrGrZones_m_BGrid[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_LowBrGrZones_m_BGrid[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_LowBrGrZones_m_BGrid[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_LowBrGrZones_m_BGrid[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); //#awbb_LowBrGrZones_m_GridStep
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D18);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AE); //#awbb_LowBrGrZones_m_Boffs

	// AWB ETC
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D1C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x037C); //#awbb_CrclLowT_R_c
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D20);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0157); //#awbb_CrclLowT_B_c
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D24);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3EB8); //#awbb_CrclLowT_Rad_c

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D2C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013D); //#awbb_IntcR
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011E); //#awbb_IntcB

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D28);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0270); //#awbb_OutdoorWP_r
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0240); //#awbb_OutdoorWP_b

	//S5K5CAGX_write_cmos_sensor(0x002A, 0x0E4C);
	//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbboost_useBoosting4Outdoor

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D4C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0187); //#awbb_GamutWidthThr1
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00CF); //#awbb_GamutHeightThr1
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000D); //#awbb_GamutWidthThr2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#awbb_GamutHeightThr2

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D5C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7FFF); //#awbb_LowTempRB
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050); //#awbb_LowTemp_RBzone

	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D46);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x04C0);//#awbb_MvEq_RBthresh  
	
	S5K5CAGX_write_cmos_sensor(0x002A, 0x2316);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0006); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E44); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x09BB); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E36); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFD8); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0DD4); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFC0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFB0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFC0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0020); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFC0); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02D9); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0357); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03D1); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0E6A); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0ED8); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F51); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F5C); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F8F); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1006); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00AC); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00BD); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0049); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F5); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E4A); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 
 

//	S5K5CAGX_write_cmos_sensor(0x002A, 0x0D4A);
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#awbb_MovingScale10

	//S5K5CAGX_write_cmos_sensor(0x002A, 0x0E3E);
	//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_rpl_InvalidOutdoor off
//	S5K5CAGX_write_cmos_sensor(0x002A, 0x22DE);
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#Mon_AWB_ByPassMode // [0]Outdoor [1]LowBr [2]LowTemp

//	S5K5CAGX_write_cmos_sensor(0x002A, 0x337C);
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00B3); //#Tune_TP_ChMoveToNearR
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040); //#Tune_TP_AvMoveToGamutDist

//	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E44);

	// AWB initial point, clear the Initial awb gain to solve the enter preview yellow issue.
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#define awbb_GainsInit_0_
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#define awbb_GainsInit_1_
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#define awbb_GainsInit_2_


	// Set AWB global offset
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0E36);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_RGainOff
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_BGainOff
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#awbb_GGainOff



	//==========================================================================
	//Gamma, 2010-01-11
	//==========================================================================
	//Our //old//STW
	S5K5CAGX_write_cmos_sensor(0x002A, 0x3288); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBIndoor_0__0_ 0x70003288 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBIndoor_0__1_ 0x7000328A 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBIndoor_0__2_ 0x7000328C 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBIndoor_0__3_ 0x7000328E 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBIndoor_0__4_ 0x70003290 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBIndoor_0__5_ 0x70003292 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBIndoor_0__6_ 0x70003294 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBIndoor_0__7_ 0x70003296 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBIndoor_0__8_ 0x70003298 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBIndoor_0__9_ 0x7000329A 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBIndoor_0__10_ 0x7000329C 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBIndoor_0__11_ 0x7000329E 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBIndoor_0__12_ 0x700032A0 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBIndoor_0__13_ 0x700032A2 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBIndoor_0__14_ 0x700032A4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBIndoor_0__15_ 0x700032A6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBIndoor_0__16_ 0x700032A8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBIndoor_0__17_ 0x700032AA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBIndoor_0__18_ 0x700032AC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBIndoor_0__19_ 0x700032AE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBIndoor_1__0_ 0x700032B0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBIndoor_1__1_ 0x700032B2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBIndoor_1__2_ 0x700032B4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBIndoor_1__3_ 0x700032B6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBIndoor_1__4_ 0x700032B8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBIndoor_1__5_ 0x700032BA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBIndoor_1__6_ 0x700032BC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBIndoor_1__7_ 0x700032BE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBIndoor_1__8_ 0x700032C0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBIndoor_1__9_ 0x700032C2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBIndoor_1__10_ 0x700032C4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBIndoor_1__11_ 0x700032C6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBIndoor_1__12_ 0x700032C8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBIndoor_1__13_ 0x700032CA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBIndoor_1__14_ 0x700032CC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBIndoor_1__15_ 0x700032CE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBIndoor_1__16_ 0x700032D0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBIndoor_1__17_ 0x700032D2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBIndoor_1__18_ 0x700032D4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBIndoor_1__19_ 0x700032D6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBIndoor_2__0_ 0x700032D8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBIndoor_2__1_ 0x700032DA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBIndoor_2__2_ 0x700032DC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBIndoor_2__3_ 0x700032DE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBIndoor_2__4_ 0x700032E0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBIndoor_2__5_ 0x700032E2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBIndoor_2__6_ 0x700032E4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBIndoor_2__7_ 0x700032E6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBIndoor_2__8_ 0x700032E8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBIndoor_2__9_ 0x700032EA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBIndoor_2__10_ 0x700032EC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBIndoor_2__11_ 0x700032EE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBIndoor_2__12_ 0x700032F0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBIndoor_2__13_ 0x700032F2
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBIndoor_2__14_ 0x700032F4
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBIndoor_2__15_ 0x700032F6
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBIndoor_2__16_ 0x700032F8
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBIndoor_2__17_ 0x700032FA
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBIndoor_2__18_ 0x700032FC
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBIndoor_2__19_ 0x700032FE

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBOutdoor_0__0_ 0x70003300
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBOutdoor_0__1_ 0x70003302
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBOutdoor_0__2_ 0x70003304
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBOutdoor_0__3_ 0x70003306
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBOutdoor_0__4_ 0x70003308
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBOutdoor_0__5_ 0x7000330A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBOutdoor_0__6_ 0x7000330C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBOutdoor_0__7_ 0x7000330E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBOutdoor_0__8_ 0x70003310
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBOutdoor_0__9_ 0x70003312
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBOutdoor_0__10_0x70003314
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBOutdoor_0__11_0x70003316
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBOutdoor_0__12_0x70003318
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBOutdoor_0__13_0x7000331A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBOutdoor_0__14_0x7000331C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBOutdoor_0__15_0x7000331E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBOutdoor_0__16_0x70003320
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBOutdoor_0__17_0x70003322
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBOutdoor_0__18_0x70003324
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBOutdoor_0__19_0x70003326
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBOutdoor_1__0_ 0x70003328
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBOutdoor_1__1_ 0x7000332A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBOutdoor_1__2_ 0x7000332C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBOutdoor_1__3_ 0x7000332E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBOutdoor_1__4_ 0x70003330
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBOutdoor_1__5_ 0x70003332
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBOutdoor_1__6_ 0x70003334
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBOutdoor_1__7_ 0x70003336
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBOutdoor_1__8_ 0x70003338
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBOutdoor_1__9_ 0x7000333A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBOutdoor_1__10_0x7000333C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBOutdoor_1__11_0x7000333E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBOutdoor_1__12_0x70003340
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBOutdoor_1__13_0x70003342
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBOutdoor_1__14_0x70003344
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBOutdoor_1__15_0x70003346
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBOutdoor_1__16_0x70003348
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBOutdoor_1__17_0x7000334A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBOutdoor_1__18_0x7000334C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBOutdoor_1__19_0x7000334E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#SARR_usDualGammaLutRGBOutdoor_2__0_ 0x70003350
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#SARR_usDualGammaLutRGBOutdoor_2__1_ 0x70003352
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#SARR_usDualGammaLutRGBOutdoor_2__2_ 0x70003354
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#SARR_usDualGammaLutRGBOutdoor_2__3_ 0x70003356
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0062); //#SARR_usDualGammaLutRGBOutdoor_2__4_ 0x70003358
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D5); //#SARR_usDualGammaLutRGBOutdoor_2__5_ 0x7000335A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0138); //#SARR_usDualGammaLutRGBOutdoor_2__6_ 0x7000335C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0161); //#SARR_usDualGammaLutRGBOutdoor_2__7_ 0x7000335E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0186); //#SARR_usDualGammaLutRGBOutdoor_2__8_ 0x70003360
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01BC); //#SARR_usDualGammaLutRGBOutdoor_2__9_ 0x70003362
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E8); //#SARR_usDualGammaLutRGBOutdoor_2__10_0x70003364
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x020F); //#SARR_usDualGammaLutRGBOutdoor_2__11_0x70003366
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0232); //#SARR_usDualGammaLutRGBOutdoor_2__12_0x70003368
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0273); //#SARR_usDualGammaLutRGBOutdoor_2__13_0x7000336A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02AF); //#SARR_usDualGammaLutRGBOutdoor_2__14_0x7000336C
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0309); //#SARR_usDualGammaLutRGBOutdoor_2__15_0x7000336E
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0355); //#SARR_usDualGammaLutRGBOutdoor_2__16_0x70003370
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0394); //#SARR_usDualGammaLutRGBOutdoor_2__17_0x70003372
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03CE); //#SARR_usDualGammaLutRGBOutdoor_2__18_0x70003374
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#SARR_usDualGammaLutRGBOutdoor_2__19_0x70003376

	S5K5CAGX_write_cmos_sensor(0x002A, 0x06A6);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00D8); //#SARR_AwbCcmCord_0_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FC); //#SARR_AwbCcmCord_1_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0120); //#SARR_AwbCcmCord_2_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x014C); //#SARR_AwbCcmCord_3_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0184); //#SARR_AwbCcmCord_4_
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AD); //#SARR_AwbCcmCord_5_

	//================================================================================================
	// SET AFIT
	//================================================================================================
	// Noise index
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0764);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0041); //#afit_uNoiseIndInDoor[0] // 65
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0063); //#afit_uNoiseIndInDoor[1] // 99
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C8); //#afit_uNoiseIndInDoor[2] // 187
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x015E); //#afit_uNoiseIndInDoor[3] // 403->380
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x028A); //#afit_uNoiseIndInDoor[4] // 700
	// AFIT table start address // 7000_07C4
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0770);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x07C4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7000);
	// AFIT table (Variables)
	S5K5CAGX_write_cmos_sensor(0x002A, 0x07C4);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[0]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#TVAR_afit_pBaseVals[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF6); //#TVAR_afit_pBaseVals[4]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_afit_pBaseVals[5]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[6]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_afit_pBaseVals[7]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017C); //#TVAR_afit_pBaseVals[8]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[9]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_afit_pBaseVals[10]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_afit_pBaseVals[11]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012C); //#TVAR_afit_pBaseVals[12]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03E8); //#TVAR_afit_pBaseVals[13]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[14]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); //#TVAR_afit_pBaseVals[15]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_afit_pBaseVals[16]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_afit_pBaseVals[17]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_afit_pBaseVals[18]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA); //#TVAR_afit_pBaseVals[19]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0064); //#TVAR_afit_pBaseVals[20]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0064); //#TVAR_afit_pBaseVals[21]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[22]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[23]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_afit_pBaseVals[24]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0012); //#TVAR_afit_pBaseVals[25]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[26]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[27]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[28]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[29]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A24); //#TVAR_afit_pBaseVals[30]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1701); //#TVAR_afit_pBaseVals[31]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0229); //#TVAR_afit_pBaseVals[32]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[33]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[34]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[35]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[36]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[37]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043B); //#TVAR_afit_pBaseVals[38]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[39]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0301); //#TVAR_afit_pBaseVals[40]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[41]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x051E); //#TVAR_afit_pBaseVals[42]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[43]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[44]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A05); //#TVAR_afit_pBaseVals[45]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A3C); //#TVAR_afit_pBaseVals[46]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A28); //#TVAR_afit_pBaseVals[47]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); //#TVAR_afit_pBaseVals[48]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[49]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1102); //#TVAR_afit_pBaseVals[50]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001B); //#TVAR_afit_pBaseVals[51]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0900); //#TVAR_afit_pBaseVals[52]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600); //#TVAR_afit_pBaseVals[53]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[54]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0305); //#TVAR_afit_pBaseVals[55]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C03); //#TVAR_afit_pBaseVals[56]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x006E); //#TVAR_afit_pBaseVals[57]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0178); //#TVAR_afit_pBaseVals[58]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[59]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[60]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[61]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5002); //#TVAR_afit_pBaseVals[62]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7850); //#TVAR_afit_pBaseVals[63]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2878); //#TVAR_afit_pBaseVals[64]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[65]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[66]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E0C); //#TVAR_afit_pBaseVals[67]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[68]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[69]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4104); //#TVAR_afit_pBaseVals[70]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x123C); //#TVAR_afit_pBaseVals[71]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4012); //#TVAR_afit_pBaseVals[72]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[73]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E03); //#TVAR_afit_pBaseVals[74]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011E); //#TVAR_afit_pBaseVals[75]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0201); //#TVAR_afit_pBaseVals[76]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5050); //#TVAR_afit_pBaseVals[77]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C3C); //#TVAR_afit_pBaseVals[78]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[79]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); //#TVAR_afit_pBaseVals[80]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0714); //#TVAR_afit_pBaseVals[81]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[82]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[83]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0432); //#TVAR_afit_pBaseVals[84]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4050); //#TVAR_afit_pBaseVals[85]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[86]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0440); //#TVAR_afit_pBaseVals[87]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //#TVAR_afit_pBaseVals[88]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E1E); //#TVAR_afit_pBaseVals[89]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[90]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5002); //#TVAR_afit_pBaseVals[91]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C50); //#TVAR_afit_pBaseVals[92]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x283C); //#TVAR_afit_pBaseVals[93]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[94]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[95]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E07); //#TVAR_afit_pBaseVals[96]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[97]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[98]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5004); //#TVAR_afit_pBaseVals[99]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F40); //#TVAR_afit_pBaseVals[100]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[101]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[102]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_afit_pBaseVals[103]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[104]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[105]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#TVAR_afit_pBaseVals[106]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[107]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF6); //#TVAR_afit_pBaseVals[108]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_afit_pBaseVals[109]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[110]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_afit_pBaseVals[111]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017C); //#TVAR_afit_pBaseVals[112]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[113]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_afit_pBaseVals[114]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_afit_pBaseVals[115]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012C); //#TVAR_afit_pBaseVals[116]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03E8); //#TVAR_afit_pBaseVals[117]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[118]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x005A); //#TVAR_afit_pBaseVals[119]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_afit_pBaseVals[120]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); //#TVAR_afit_pBaseVals[121]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000F); //#TVAR_afit_pBaseVals[122]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA); //#TVAR_afit_pBaseVals[123]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); //#TVAR_afit_pBaseVals[124]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); //#TVAR_afit_pBaseVals[125]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[126]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[127]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[128]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0019); //#TVAR_afit_pBaseVals[129]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[130]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[131]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[132]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[133]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A24); //#TVAR_afit_pBaseVals[134]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1701); //#TVAR_afit_pBaseVals[135]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0229); //#TVAR_afit_pBaseVals[136]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[137]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[138]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[139]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[140]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[141]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043B); //#TVAR_afit_pBaseVals[142]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[143]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0301); //#TVAR_afit_pBaseVals[144]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[145]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x051E); //#TVAR_afit_pBaseVals[146]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[147]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[148]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A03); //#TVAR_afit_pBaseVals[149]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A3C); //#TVAR_afit_pBaseVals[150]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A28); //#TVAR_afit_pBaseVals[151]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); //#TVAR_afit_pBaseVals[152]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[153]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1102); //#TVAR_afit_pBaseVals[154]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001B); //#TVAR_afit_pBaseVals[155]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0900); //#TVAR_afit_pBaseVals[156]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600); //#TVAR_afit_pBaseVals[157]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[158]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0305); //#TVAR_afit_pBaseVals[159]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4603); //#TVAR_afit_pBaseVals[160]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[161]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0180); //#TVAR_afit_pBaseVals[162]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[163]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1919); //#TVAR_afit_pBaseVals[164]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[165]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C02); //#TVAR_afit_pBaseVals[166]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x643C); //#TVAR_afit_pBaseVals[167]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2864); //#TVAR_afit_pBaseVals[168]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[169]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[170]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E0C); //#TVAR_afit_pBaseVals[171]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[172]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[173]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4104); //#TVAR_afit_pBaseVals[174]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x123C); //#TVAR_afit_pBaseVals[175]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4012); //#TVAR_afit_pBaseVals[176]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[177]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E03); //#TVAR_afit_pBaseVals[178]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x011E); //#TVAR_afit_pBaseVals[179]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0201); //#TVAR_afit_pBaseVals[180]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3232); //#TVAR_afit_pBaseVals[181]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C3C); //#TVAR_afit_pBaseVals[182]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[183]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); //#TVAR_afit_pBaseVals[184]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0714); //#TVAR_afit_pBaseVals[185]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[186]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[187]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0432); //#TVAR_afit_pBaseVals[188]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4050); //#TVAR_afit_pBaseVals[189]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[190]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0440); //#TVAR_afit_pBaseVals[191]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //#TVAR_afit_pBaseVals[192]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E1E); //#TVAR_afit_pBaseVals[193]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[194]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3202); //#TVAR_afit_pBaseVals[195]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C32); //#TVAR_afit_pBaseVals[196]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x283C); //#TVAR_afit_pBaseVals[197]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[198]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[199]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E07); //#TVAR_afit_pBaseVals[200]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[201]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[202]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5004); //#TVAR_afit_pBaseVals[203]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F40); //#TVAR_afit_pBaseVals[204]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[205]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[206]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_afit_pBaseVals[207]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[208]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[209]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#TVAR_afit_pBaseVals[210]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[211]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFFF6); //#TVAR_afit_pBaseVals[212]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_afit_pBaseVals[213]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[214]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_afit_pBaseVals[215]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017C); //#TVAR_afit_pBaseVals[216]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[217]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_afit_pBaseVals[218]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_afit_pBaseVals[219]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x012C); //#TVAR_afit_pBaseVals[220]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03E8); //#TVAR_afit_pBaseVals[221]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[222]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0078); //#TVAR_afit_pBaseVals[223]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_afit_pBaseVals[224]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#TVAR_afit_pBaseVals[225]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); //#TVAR_afit_pBaseVals[226]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA); //#TVAR_afit_pBaseVals[227]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[228]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[229]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[230]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#TVAR_afit_pBaseVals[231]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0064); //#TVAR_afit_pBaseVals[232]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001B); //#TVAR_afit_pBaseVals[233]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[234]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[235]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x002A); //#TVAR_afit_pBaseVals[236]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0024); //#TVAR_afit_pBaseVals[237]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A24); //#TVAR_afit_pBaseVals[238]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1701); //#TVAR_afit_pBaseVals[239]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0229); //#TVAR_afit_pBaseVals[240]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[241]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[242]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[243]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[244]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[245]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043B); //#TVAR_afit_pBaseVals[246]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[247]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0301); //#TVAR_afit_pBaseVals[248]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[249]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x051E); //#TVAR_afit_pBaseVals[250]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[251]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[252]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A03); //#TVAR_afit_pBaseVals[253]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A3C); //#TVAR_afit_pBaseVals[254]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0528); //#TVAR_afit_pBaseVals[255]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); //#TVAR_afit_pBaseVals[256]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[257]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1102); //#TVAR_afit_pBaseVals[258]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001B); //#TVAR_afit_pBaseVals[259]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0900); //#TVAR_afit_pBaseVals[260]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600); //#TVAR_afit_pBaseVals[261]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[262]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0305); //#TVAR_afit_pBaseVals[263]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4603); //#TVAR_afit_pBaseVals[264]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[265]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0180); //#TVAR_afit_pBaseVals[266]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[267]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2323); //#TVAR_afit_pBaseVals[268]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[269]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2A02); //#TVAR_afit_pBaseVals[270]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C2A); //#TVAR_afit_pBaseVals[271]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x283C); //#TVAR_afit_pBaseVals[272]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[273]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[274]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E0C); //#TVAR_afit_pBaseVals[275]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[276]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[277]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4B04); //#TVAR_afit_pBaseVals[278]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F40); //#TVAR_afit_pBaseVals[279]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[280]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[281]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2303); //#TVAR_afit_pBaseVals[282]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0123); //#TVAR_afit_pBaseVals[283]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0201); //#TVAR_afit_pBaseVals[284]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x262A); //#TVAR_afit_pBaseVals[285]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C2C); //#TVAR_afit_pBaseVals[286]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[287]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); //#TVAR_afit_pBaseVals[288]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0714); //#TVAR_afit_pBaseVals[289]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[290]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[291]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0432); //#TVAR_afit_pBaseVals[292]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4050); //#TVAR_afit_pBaseVals[293]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[294]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0440); //#TVAR_afit_pBaseVals[295]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //#TVAR_afit_pBaseVals[296]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2323); //#TVAR_afit_pBaseVals[297]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[298]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2A02); //#TVAR_afit_pBaseVals[299]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2C26); //#TVAR_afit_pBaseVals[300]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x282C); //#TVAR_afit_pBaseVals[301]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[302]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[303]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E07); //#TVAR_afit_pBaseVals[304]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[305]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[306]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5004); //#TVAR_afit_pBaseVals[307]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F40); //#TVAR_afit_pBaseVals[308]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[309]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[310]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_afit_pBaseVals[311]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[312]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[313]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#TVAR_afit_pBaseVals[314]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[315]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[316]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_afit_pBaseVals[317]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[318]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_afit_pBaseVals[319]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017C); //#TVAR_afit_pBaseVals[320]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[321]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_afit_pBaseVals[322]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_afit_pBaseVals[323]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C8); //#TVAR_afit_pBaseVals[324]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0384); //#TVAR_afit_pBaseVals[325]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[326]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0082); //#TVAR_afit_pBaseVals[327]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_afit_pBaseVals[328]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[329]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[330]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA); //#TVAR_afit_pBaseVals[331]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[332]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[333]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[334]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[335]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x010E); //#TVAR_afit_pBaseVals[336]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[337]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_afit_pBaseVals[338]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[339]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_afit_pBaseVals[340]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[341]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A24); //#TVAR_afit_pBaseVals[342]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1701); //#TVAR_afit_pBaseVals[343]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0229); //#TVAR_afit_pBaseVals[344]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[345]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[346]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[347]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[348]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[349]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043B); //#TVAR_afit_pBaseVals[350]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[351]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0301); //#TVAR_afit_pBaseVals[352]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[353]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x051E); //#TVAR_afit_pBaseVals[354]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[355]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[356]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[357]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A3C); //#TVAR_afit_pBaseVals[358]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0532); //#TVAR_afit_pBaseVals[359]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); //#TVAR_afit_pBaseVals[360]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[361]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1002); //#TVAR_afit_pBaseVals[362]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[363]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0900); //#TVAR_afit_pBaseVals[364]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600); //#TVAR_afit_pBaseVals[365]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[366]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0305); //#TVAR_afit_pBaseVals[367]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4602); //#TVAR_afit_pBaseVals[368]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[369]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0180); //#TVAR_afit_pBaseVals[370]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[371]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2328); //#TVAR_afit_pBaseVals[372]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[373]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2A02); //#TVAR_afit_pBaseVals[374]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2228); //#TVAR_afit_pBaseVals[375]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2822); //#TVAR_afit_pBaseVals[376]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[377]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1903); //#TVAR_afit_pBaseVals[378]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E0F); //#TVAR_afit_pBaseVals[379]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[380]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[381]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9604); //#TVAR_afit_pBaseVals[382]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F42); //#TVAR_afit_pBaseVals[383]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[384]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[385]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2805); //#TVAR_afit_pBaseVals[386]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0123); //#TVAR_afit_pBaseVals[387]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0201); //#TVAR_afit_pBaseVals[388]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2024); //#TVAR_afit_pBaseVals[389]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C1C); //#TVAR_afit_pBaseVals[390]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[391]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); //#TVAR_afit_pBaseVals[392]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A0A); //#TVAR_afit_pBaseVals[393]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A2D); //#TVAR_afit_pBaseVals[394]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[395]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0432); //#TVAR_afit_pBaseVals[396]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4050); //#TVAR_afit_pBaseVals[397]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F0F); //#TVAR_afit_pBaseVals[398]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0440); //#TVAR_afit_pBaseVals[399]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //#TVAR_afit_pBaseVals[400]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2328); //#TVAR_afit_pBaseVals[401]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[402]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3C02); //#TVAR_afit_pBaseVals[403]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C3C); //#TVAR_afit_pBaseVals[404]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x281C); //#TVAR_afit_pBaseVals[405]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[406]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A03); //#TVAR_afit_pBaseVals[407]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2D0A); //#TVAR_afit_pBaseVals[408]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x070A); //#TVAR_afit_pBaseVals[409]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[410]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5004); //#TVAR_afit_pBaseVals[411]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0F40); //#TVAR_afit_pBaseVals[412]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x400F); //#TVAR_afit_pBaseVals[413]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[414]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_afit_pBaseVals[415]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[416]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[417]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#TVAR_afit_pBaseVals[418]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[419]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[420]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C4); //#TVAR_afit_pBaseVals[421]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[422]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x009C); //#TVAR_afit_pBaseVals[423]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x017C); //#TVAR_afit_pBaseVals[424]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03FF); //#TVAR_afit_pBaseVals[425]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000C); //#TVAR_afit_pBaseVals[426]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0010); //#TVAR_afit_pBaseVals[427]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00C8); //#TVAR_afit_pBaseVals[428]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0320); //#TVAR_afit_pBaseVals[429]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0046); //#TVAR_afit_pBaseVals[430]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x015E); //#TVAR_afit_pBaseVals[431]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#TVAR_afit_pBaseVals[432]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[433]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[434]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01AA); //#TVAR_afit_pBaseVals[435]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0014); //#TVAR_afit_pBaseVals[436]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0014); //#TVAR_afit_pBaseVals[437]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[438]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x000A); //#TVAR_afit_pBaseVals[439]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0140); //#TVAR_afit_pBaseVals[440]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x003C); //#TVAR_afit_pBaseVals[441]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_afit_pBaseVals[442]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0023); //#TVAR_afit_pBaseVals[443]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0023); //#TVAR_afit_pBaseVals[444]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0032); //#TVAR_afit_pBaseVals[445]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A24); //#TVAR_afit_pBaseVals[446]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1701); //#TVAR_afit_pBaseVals[447]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0229); //#TVAR_afit_pBaseVals[448]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1403); //#TVAR_afit_pBaseVals[449]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[450]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[451]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0505); //#TVAR_afit_pBaseVals[452]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00FF); //#TVAR_afit_pBaseVals[453]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x043B); //#TVAR_afit_pBaseVals[454]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1414); //#TVAR_afit_pBaseVals[455]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0301); //#TVAR_afit_pBaseVals[456]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF07); //#TVAR_afit_pBaseVals[457]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x051E); //#TVAR_afit_pBaseVals[458]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A1E); //#TVAR_afit_pBaseVals[459]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#TVAR_afit_pBaseVals[460]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[461]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x143C); //#TVAR_afit_pBaseVals[462]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0532); //#TVAR_afit_pBaseVals[463]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); //#TVAR_afit_pBaseVals[464]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0096); //#TVAR_afit_pBaseVals[465]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1002); //#TVAR_afit_pBaseVals[466]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x001E); //#TVAR_afit_pBaseVals[467]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0900); //#TVAR_afit_pBaseVals[468]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600); //#TVAR_afit_pBaseVals[469]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0504); //#TVAR_afit_pBaseVals[470]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0305); //#TVAR_afit_pBaseVals[471]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6402); //#TVAR_afit_pBaseVals[472]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[473]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0180); //#TVAR_afit_pBaseVals[474]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080); //#TVAR_afit_pBaseVals[475]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5050); //#TVAR_afit_pBaseVals[476]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[477]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1C02); //#TVAR_afit_pBaseVals[478]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x191C); //#TVAR_afit_pBaseVals[479]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2819); //#TVAR_afit_pBaseVals[480]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[481]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E03); //#TVAR_afit_pBaseVals[482]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E0F); //#TVAR_afit_pBaseVals[483]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0508); //#TVAR_afit_pBaseVals[484]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[485]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xAA04); //#TVAR_afit_pBaseVals[486]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1452); //#TVAR_afit_pBaseVals[487]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4015); //#TVAR_afit_pBaseVals[488]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0604); //#TVAR_afit_pBaseVals[489]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5006); //#TVAR_afit_pBaseVals[490]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0150); //#TVAR_afit_pBaseVals[491]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0201); //#TVAR_afit_pBaseVals[492]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E1E); //#TVAR_afit_pBaseVals[493]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1212); //#TVAR_afit_pBaseVals[494]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0028); //#TVAR_afit_pBaseVals[495]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x030A); //#TVAR_afit_pBaseVals[496]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A10); //#TVAR_afit_pBaseVals[497]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0819); //#TVAR_afit_pBaseVals[498]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xFF05); //#TVAR_afit_pBaseVals[499]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0432); //#TVAR_afit_pBaseVals[500]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4052); //#TVAR_afit_pBaseVals[501]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1514); //#TVAR_afit_pBaseVals[502]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0440); //#TVAR_afit_pBaseVals[503]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0302); //#TVAR_afit_pBaseVals[504]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5050); //#TVAR_afit_pBaseVals[505]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0101); //#TVAR_afit_pBaseVals[506]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1E02); //#TVAR_afit_pBaseVals[507]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x121E); //#TVAR_afit_pBaseVals[508]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2812); //#TVAR_afit_pBaseVals[509]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); //#TVAR_afit_pBaseVals[510]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1003); //#TVAR_afit_pBaseVals[511]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x190A); //#TVAR_afit_pBaseVals[512]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0508); //#TVAR_afit_pBaseVals[513]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x32FF); //#TVAR_afit_pBaseVals[514]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5204); //#TVAR_afit_pBaseVals[515]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1440); //#TVAR_afit_pBaseVals[516]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4015); //#TVAR_afit_pBaseVals[517]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0204); //#TVAR_afit_pBaseVals[518]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); //#TVAR_afit_pBaseVals[519]

	// AFIT table (Constants)
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7F7A); //#afit_pConstBaseVals[0]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7FBD); //#afit_pConstBaseVals[1]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xBEFC); //#afit_pConstBaseVals[2]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0xF7BC); //#afit_pConstBaseVals[3]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x7E06); //#afit_pConstBaseVals[4]
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0053); //#afit_pConstBaseVals[5]

	// Update Changed Registers
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0664);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x013E); //seti_uContrastCenter

	//================================================================
	// END Tuning part					 
	//================================================================

	//================================================================
	//	Config the PLL
	//
	// Input clock (24Mhz), [0x700001CC] 0x5DC0 --> 24000KHz
	// System clock (40MHz),  [0x700001F6] 0x2710 * 4 --> 24000KHz
	// Output clock (48MHz),  [0x700001F8 ~ 1FA] (0x2ED3 ~ 0x2EEC) * 4 --> 47948KHz ~ 48048KHz
	//================================================================
	// Ex. 24Mhz master clock, S5K5CAGX_master_clk_freq == 240.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x01CC); // Set input CLK // 24MHz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x5DC0); // #REG_TC_IPRM_InClockLSBs // 26Mhz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #REG_TC_IPRM_InClockMSBs
	S5K5CAGX_write_cmos_sensor(0x002A, 0x01EE);
#ifdef MIPI_INTERFACE
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);   //mipi modify
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003);   //mipi modify
#else
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); // #REG_TC_IPRM_UseNPviClocks // Number of PLL setting 2PLL configurations
#endif
	S5K5CAGX_write_cmos_sensor(0x002A, 0x01F6); // Set system CLK 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x38A4); // #REG_TC_IPRM_OpClk4KHz_0  1st 58MHz for 30fps preview	
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x37A4); // #REG_TC_IPRM_MinOutRate4KHz_0  PVI clock 48MHz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x39A4); // #REG_TC_IPRM_MaxOutRate4KHz_0
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x38a4); // #REG_TC_IPRM_OpClk4KHz_2  3thd 58MHz for 30fps preview	

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x4228); // #REG_TC_IPRM_MinOutRate4KHz_2	PVI clock 82MHz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x42a8); // #
 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x38a4); // #REG_TC_IPRM_OpClk4KHz_1  2nd 58MHz for 30fps preview	
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1f00); // #REG_TC_IPRM_MinOutRate4KHz_1  PVI clock 40MHz
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1f40); // #REG_TC_IPRM_MaxOutRate4KHz_1
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0208);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_IPRM_InitParamsUpdated

	//mdelay(5); 

	//================================================================
	//	SET PREVIEW CONFIGURATION_0, Camera Normal 10~30fps
	//================================================================
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); // SET PREVIEW CONFIGURATION_0
	S5K5CAGX_write_cmos_sensor(0x002A, 0x026C); // SET PREVIEW CONFIGURATION_0

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_0TC_PCFG_usWidth  //640	0x0280
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0300); //#REG_0TC_PCFG_usHeight //480	 0x01e0

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //#REG_0TC_PCFG_Format			  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978); //#REG_0TC_PCFG_usMaxOut4KHzRate  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978); //#REG_0TC_PCFG_usMinOut4KHzRate  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); //#REG_0TC_PCFG_OutClkPerPix88	  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800); //#REG_0TC_PCFG_uMaxBpp88		  
       if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#REG_0TC_PCFG_PVIMask //s0050 = FALSE in MSM6290 : s0052 = TRUE in MSM6800 //reg 027A
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4000); //4000-0000 change by yan #REG_0TC_PCFG_OIFMask
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0); //#REG_0TC_PCFG_usJpegPacketSize
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_usJpegTotalPackets
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#REG_0TC_PCFG_uClockInd
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_usFrTimeType

	// Always achieve the best frame rate.
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); //#REG_0TC_PCFG_FrRateQualityType

	S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NOM_MAX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10 //10fps
	S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NOM_MIN_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10 //30fps
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_bSmearOutput
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_sSaturation
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_sSharpBlur
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_sColorTemp
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_uDeviceGammaIndex
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_uPrevMirror
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_uCaptureMirror
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); //#REG_0TC_PCFG_uRotation

	//New Configuration FW Sync Preview
	S5K5CAGX_write_cmos_sensor(0x002A, 0x023C);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0220);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_EnablePreview // Start preview
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_EnablePreviewChanged
	
	
	//================================================================
	//	SET PREVIEW CONFIGURATION_1, Camera Normal 10~30fps
	//================================================================
	
	 S5K5CAGX_write_cmos_sensor(0x002A, 0x029C);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
     S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //0400 //#REG_1TC_PCFG_usWidth//1024                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
     S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);   
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);  //#REG_1TC_PCFG_Format         0270                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
     S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x4228); // #REG_TC_IPRM_MinOutRate4KHz_2	PVI clock 82MHz 0x1308
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x42a8); // # 0x1388
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100);  //#REG_1TC_PCFG_OutClkPerPix88    0276                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
     S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_1TC_PCFG_uMaxBpp88         027                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
     if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); 

        S5K5CAGX_write_cmos_sensor(0x0F12, 0x4000);  //#REG_1TC_PCFG_OIFMask                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C0);  //01E0 //#REG_1TC_PCFG_usJpegPacketSize                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0320);  //0000 //#REG_1TC_PCFG_usJpegTotalPackets                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

		    //S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uClockInd                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);//0x0002
		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_usFrTimeType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);  //1 //#REG_1TC_PCFG_FrRateQualityType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x03e8);  //#REG_0TC_CCFG_usMaxFrTimeMsecMult10 //10fps
	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x029a);  //#REG_0TC_CCFG_usMinFrTimeMsecMult10 //15fps 

		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_bSmearOutput                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSaturation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSharpBlur                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sColorTemp                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uDeviceGammaIndex                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uPrevMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uCaptureMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uRotation      
		//================================================================
	//	SET PREVIEW CONFIGURATION_2, Camera Normal 10~30fps
	//================================================================
	
	 S5K5CAGX_write_cmos_sensor(0x002A, 0x02CC);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //0400 //#REG_1TC_PCFG_usWidth//1024                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);   
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);  //#REG_1TC_PCFG_Format         0270                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x4228); // #REG_TC_IPRM_MinOutRate4KHz_2	PVI clock 82MHz 0x1308
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x42a8); // # 0x1388
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100);  //#REG_1TC_PCFG_OutClkPerPix88    0276                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_1TC_PCFG_uMaxBpp88         027                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
       if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); //#REG_1TC_PCFG_PVIMask //s0050 = FALSE in MSM6290 : s0052 = TRUE in MSM6800 //reg 027A

        S5K5CAGX_write_cmos_sensor(0x0F12, 0x4000);  //#REG_1TC_PCFG_OIFMask                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C0);  //01E0 //#REG_1TC_PCFG_usJpegPacketSize                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0320);  //0000 //#REG_1TC_PCFG_usJpegTotalPackets                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

		    //S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uClockInd                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);//0x0002
		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_usFrTimeType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);  //1 //#REG_1TC_PCFG_FrRateQualityType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x07D0);  //#REG_0TC_CCFG_usMaxFrTimeMsecMult10 //7.5fps
	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x029a);  //#REG_0TC_CCFG_usMinFrTimeMsecMult10 //10fps 

		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_bSmearOutput                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSaturation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSharpBlur                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sColorTemp                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uDeviceGammaIndex                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uPrevMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uCaptureMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uRotation   
	
	 S5K5CAGX_write_cmos_sensor(0x002A, 0x02FC);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //0400 //#REG_1TC_PCFG_usWidth//1024                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);   
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);  //#REG_1TC_PCFG_Format         0270                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x4228); // #REG_TC_IPRM_MinOutRate4KHz_2	PVI clock 82MHz 0x1308
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);//0x42a8); // # 0x1388
	 S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100);  //#REG_1TC_PCFG_OutClkPerPix88    0276                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_1TC_PCFG_uMaxBpp88         027                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
 
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x4000);  //#REG_1TC_PCFG_OIFMask                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x03C0);  //01E0 //#REG_1TC_PCFG_usJpegPacketSize                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0320);  //0000 //#REG_1TC_PCFG_usJpegTotalPackets                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

		    //S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uClockInd                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);//0x0002
		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_usFrTimeType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);  //1 //#REG_1TC_PCFG_FrRateQualityType                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x0310);  //#REG_0TC_CCFG_usMaxFrTimeMsecMult10 //7.5fps
	      S5K5CAGX_write_cmos_sensor(0x0F12, 0x0310);  //#REG_0TC_CCFG_usMinFrTimeMsecMult10 //10fps 

		    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_bSmearOutput                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSaturation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sSharpBlur                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_sColorTemp                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uDeviceGammaIndex                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uPrevMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uCaptureMirror                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_1TC_PCFG_uRotation 	
	
		//==, 0x====);=======================================================================================
	// S, 0xCAPT);URE CONFIGURATION_0
	// #, 0xramt); :YUV
	// #, 0xze: );QXGA
	// #, 0xS : );5 ~ 7.5fps
	//==, 0x====);=======================================================================================
	S5K5CAGX_write_cmos_sensor(0x002A, 0x035C);  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_uCaptureModeJpEG
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_0TC_CCFG_usWidth 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);  //#REG_0TC_CCFG_usHeight
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);  //#REG_0TC_CCFG_Format//5:YUV9:JPEG 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);  //#REG_0TC_CCFG_usMaxOut4KHzRate
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);  //#REG_0TC_CCFG_usMinOut4KHzRate
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100);  //#REG_0TC_CCFG_OutClkPerPix88
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_0TC_CCFG_uMaxBpp88 
       if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070);
     
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_OIFMask   edison
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0);  //#REG_0TC_CCFG_usJpegPacketSize
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_usJpegTotalPackets
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);  //#REG_0TC_CCFG_uClockInd 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_usFrTimeType
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);  //#REG_0TC_CCFG_FrRateQualityType 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x03e8);  //#REG_0TC_CCFG_usMaxFrTimeMsecMult10 //10fps
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029a);  //#REG_0TC_CCFG_usMinFrTimeMsecMult10 //15fps 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_bSmearOutput
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sSaturation 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sSharpBlur
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sColorTemp
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_uDeviceGammaIndex 
/*
       if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070);
         */
	//==, 0x====);=======================================================================================
	// S, 0xCAPT);URE CONFIGURATION_1
	// #, 0xramt); :YUV
	// #, 0xze: );QXGA
	// #, 0xS : );1-2fps   24M	1740 1780, 
	//==, 0x====);=======================================================================================
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0388); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_uCaptureModeJpEG
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_0TC_CCFG_usWidth 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0600);  //#REG_0TC_CCFG_usHeight
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005);  //#REG_0TC_CCFG_Format//5:YUV9:JPEG 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);  //#REG_0TC_CCFG_usMaxOut4KHzRate
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x6978);  //#REG_0TC_CCFG_usMinOut4KHzRate
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100);  //#REG_0TC_CCFG_OutClkPerPix88
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0800);  //#REG_0TC_CCFG_uMaxBpp88
       if(format==S5K5CA_YUYV)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0040);    
        else if(format ==S5K5CA_UYVY)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0050);  
        else if(format ==S5K5CA_YVYU)
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0060); 
        else if(format == S5K5CA_VYUY)   
         S5K5CAGX_write_cmos_sensor(0x0F12, 0x0070); 

	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_OIFMask   edison
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x01E0);  //#REG_0TC_CCFG_usJpegPacketSize
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_usJpegTotalPackets
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);  //#REG_0TC_CCFG_uClockInd 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_usFrTimeType
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);  //#REG_0TC_CCFG_FrRateQualityType 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x07D0);  //#REG_0TC_CCFG_usMaxFrTimeMsecMult10 //5fps
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x029a);  //#REG_0TC_CCFG_usMinFrTimeMsecMult10 //15fps 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_bSmearOutput
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sSaturation 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sSharpBlur
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_sColorTemp
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //#REG_0TC_CCFG_uDeviceGammaIndex	  
	 
	// Fill RAM with alternative op-codes
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); //start add MSW
	S5K5CAGX_write_cmos_sensor(0x002A, 0x2CE8); //start add LSW
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0007); //Modify LSB to control AWBB_YThreshLow
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00e2); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); //Modify LSB to control AWBB_YThreshLowBrLow
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00e2);

	//================================================================
	// ARM Go ... ...
	//================================================================
	S5K5CAGX_write_cmos_sensor(0x0028, 0xD000); 	//Host Interrupt
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1000); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 
	mdelay(10);		// Actually delay 9.2ms , 10ms delay is enough for ARM go
 	
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//-------------------------------------------SENSOR PARAMETER-----------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
void S5K5CAGX_set_saturation(UINT16 iPara)
{
    SENSORDB("\n****** S5K5CAGX_set_saturation \n");

    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }

	//SENSORDB("iPara=%d",iPara);
    switch(iPara)
    {
    	case ISP_SAT_LOW:
    		    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
            S5K5CAGX_write_cmos_sensor(0x002a,0x0210);
        	S5K5CAGX_write_cmos_sensor(0x0f12,0xFFa0);
        break;
	    case ISP_SAT_HIGH:
	    	    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
            S5K5CAGX_write_cmos_sensor(0x002a,0x0210);
	        S5K5CAGX_write_cmos_sensor(0x0f12,0x0080);
		break;
	    case ISP_SAT_MIDDLE:
	     default:
	     	    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
            S5K5CAGX_write_cmos_sensor(0x002a,0x0210);
	        S5K5CAGX_write_cmos_sensor(0x0f12,0x0000);
        break;
    }
         return;
}

void S5K5CAGX_set_contrast(UINT16 iPara)
{

    SENSORDB("\n****** S5K5CAGX_set_contrast \n");
	//SENSORDB("iPara=%d",iPara);
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }
 
    switch(iPara)
    {
	    case ISP_CONTRAST_LOW:
	    	    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
            S5K5CAGX_write_cmos_sensor(0x002a,0x020E);
	        S5K5CAGX_write_cmos_sensor(0x0f12,0xFFA0);
		break;
	    case ISP_CONTRAST_HIGH:
	    	    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
           S5K5CAGX_write_cmos_sensor(0x002a,0x020E);
	        S5K5CAGX_write_cmos_sensor(0x0f12,0x0070);
		break;
	    case ISP_CONTRAST_MIDDLE:
	  	    default:
	    	    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
            S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
             S5K5CAGX_write_cmos_sensor(0x002a,0x020E);
	        S5K5CAGX_write_cmos_sensor(0x0f12,0x0000);
		break;
    }
   return;
} 

void S5K5CAGX_set_ISO_Auto()
{
    SENSORDB("\n****** S5K5CAGX_set_ISO_Auto \n");

    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    S5K5CAGX_write_cmos_sensor(0x002a, 0x04B4);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
}

void S5K5CAGX_set_ISO_100()
{
    SENSORDB("\n****** S5K5CAGX_set_ISO_100 \n");

    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    S5K5CAGX_write_cmos_sensor(0x002a, 0x04B4);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0064);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
}

void S5K5CAGX_set_ISO_200()
{
    SENSORDB("\n****** S5K5CAGX_set_ISO_200 \n");

    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    S5K5CAGX_write_cmos_sensor(0x002a, 0x04B4);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x00c8);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
}

void S5K5CAGX_set_ISO_400()
{
    SENSORDB("\n****** S5K5CAGX_set_ISO_400 \n");

    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    S5K5CAGX_write_cmos_sensor(0x002a, 0x04B4);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0190);
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
}

void S5K5CAGXSetSceneAE(void)
{
    S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0530); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2ee0); // #lt_uMaxExp1 		33.3ms							   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4E20); // #lt_uMaxExp2 		50ms							
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 						   
	S5K5CAGX_write_cmos_sensor(0x002A, 0x167C); 					
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9c40); // #evt1_lt_uMaxExp3	100ms							  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 							
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3880); // #evt1_lt_uMaxExp4	200ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	 
	// Set capture exposure time
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0538); 					
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x2ee0); // #lt_uCapMaxExp1	33.3ms							   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x4E20); // #lt_uCapMaxExp2		50ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x1684); 												  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x9c40); // #evt1_lt_uCapMaxExp3 100ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 														 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x3880); // #evt1_lt_uCapMaxExp4 200ms							 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 		   
	// Preview & Capture Gain.
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0540); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0150); // #lt_uMaxAnGain1							  
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0280); // #lt_uMaxAnGain2							  
	S5K5CAGX_write_cmos_sensor(0x002A, 0x168C); 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x02A0); // #evt1_lt_uMaxAnGain3 						   
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); // #evt1_lt_uMaxAnGain4					
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0544);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // #lt_uMaxDigGain
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00); // #lt_uMaxTotGain 10x

	S5K5CAGX_write_cmos_sensor(0x002A, 0x1694);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #evt1_senHal_bExpandForbid
	S5K5CAGX_write_cmos_sensor(0x002A, 0x051A);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0111); // #lt_uLimitHigh 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x00F0); // #lt_uLimitLow    
}

void S5K5CAGX_set_AWB_Bit(int bit_op, int set_value)
{
        kal_uint16 u16temp_AWB_Reg=0;
        kal_uint16 u16temp_cal = 0;

        mdelay(100);
        SENSORDB("\n****** S5K5CAGX_set_AWB_SetBit \n");
    	S5K5CAGX_write_cmos_sensor(0x002c, 0x7000); 
	    S5K5CAGX_write_cmos_sensor(0x002e, 0x04D2); 	// REG_TC_DBG_AutoAlgEnBits
        u16temp_AWB_Reg=S5K5CAGX_read_cmos_sensor(0x0f12);
        SENSORDB("u16temp_AWB_Reg1=%d \n", u16temp_AWB_Reg);
        
        if(0 == set_value)
        {
             u16temp_cal = ~bit_op;
             u16temp_AWB_Reg = u16temp_AWB_Reg & u16temp_cal;
             SENSORDB("u16temp_AWB_Reg2=%d \n", u16temp_AWB_Reg);
        }
        else
        {
            u16temp_AWB_Reg = u16temp_AWB_Reg | bit_op;
            SENSORDB("u16temp_AWB_Reg3=%d \n", u16temp_AWB_Reg);
        }
        S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
        S5K5CAGX_write_cmos_sensor(0x002a, 0x04d2);
     	S5K5CAGX_write_cmos_sensor(0x0f12, u16temp_AWB_Reg);
}

static void S5K5CAGX_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint16 u16temp_AWB_Reg=0;

    SENSORDB("\n****** S5K5CAGX_set_AWB_mode \n");
	SENSORDB("AWB_enable=%d\n",AWB_enable);
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// REG_TC_DBG_AutoAlgEnBits


	if (AWB_enable == KAL_TRUE)
    {   //enable Auto WB
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2);
        S5K5CAGX_write_cmos_sensor(0x0F12, (u16temp_AWB_Reg|0x08));// turn on AEC/AGC
    }
    else
    {
		//disable Auto WB
		S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2);
		S5K5CAGX_write_cmos_sensor(0x0F12, (u16temp_AWB_Reg&(~0x08)));//3
    }
}

void S5K5CAGX_set_mirror(kal_uint8 image_mirror)
{
	SENSORDB("\n*****S5K5CAGX_set_mirror\n");

	SENSORDB("image_mirror=%d\n",image_mirror);
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0296); 
//DAVID.LIU DEBUG	
    image_mirror = IMAGE_H_MIRROR;
	
	switch (image_mirror)
	{
		case IMAGE_NORMAL:
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// REG_0TC_PCFG_uPrevMirror
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 	// REG_0TC_PCFG_uCaptureMirror
		break;
		case IMAGE_H_MIRROR:
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// REG_0TC_PCFG_uPrevMirror
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 	// REG_0TC_PCFG_uCaptureMirror
		break;
		case IMAGE_V_MIRROR:
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// REG_0TC_PCFG_uPrevMirror
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); 	// REG_0TC_PCFG_uCaptureMirror
		break;
		case IMAGE_HV_MIRROR:
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// REG_0TC_PCFG_uPrevMirror
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0003); 	// REG_0TC_PCFG_uCaptureMirror
		break; 
		default:
			//ASSERT(0);
		break;
	}
	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
}

kal_bool S5K5CAGX_set_param_wb(UINT16 para)
{
    SENSORDB("\n****** S5K5CAGX_set_param_wb \n");

    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }
    else if(S5K5CAGX_YUV_NVRAM == write_statue)
    {
        write_statue = S5K5CAGX_YUV_MANUALREF;
    }
    else
    {
        //if(awbMode_5ca ==  para)
        ///{
        //    return;
        //}
    }

    spin_lock(&s5k5cagx_drv_lock);
    awbMode_5ca = para;
   	spin_unlock(&s5k5cagx_drv_lock);

	switch (para)
	{
		case AWB_MODE_OFF:
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOffs

			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
		    break;
		
		case AWB_MODE_AUTO:
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x077F); 	// #REG_TC_DBG_AutoAlgEnBits, AWB On
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 1);
		    break;
		
		case AWB_MODE_CLOUDY_DAYLIGHT:	//	MWB : Cloudy_D65
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOff
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
			S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0730); //#REG_SF_USER_Rgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0540); //#REG_SF_USER_Bgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 											
		    break;
		
		case AWB_MODE_DAYLIGHT: //	MWB : sun&daylight_D50						
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address	
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOff
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
			S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0630); //#REG_SF_USER_Rgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0540); //#REG_SF_USER_Bgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
		break;
		case AWB_MODE_INCANDESCENT:	//	MWB : Incand_Tungsten						
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOff
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
			S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x04BE); //#REG_SF_USER_Rgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x08a9); //#REG_SF_USER_Bgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
		    break;
		
		case AWB_MODE_FLUORESCENT: //	MWB : Florescent_TL84							  
			//S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //S5K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address		
			//S5K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOff
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
			S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x04B0); //#REG_SF_USER_Rgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0740); //#REG_SF_USER_Bgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
		    break;
		    
		case AWB_MODE_TUNGSTEN:	
			//K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	        //K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
			//K5CAGX_write_cmos_sensor(0x0F12, 0x0777); //#REG_TC_DBG_AutoAlgEnBits,AWBOff
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
 			S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0437); //#REG_SF_USER_Rgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);

			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);

			S5K5CAGX_write_cmos_sensor(0x0F12, 0x09eA); //#REG_SF_USER_Bgain
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
		    break;
		    
		default:
			return KAL_FALSE;	
	}
	return  KAL_TRUE;
} /* S5K5CAGX_set_param_wb */

void S5K5CAGX_set_brightness(UINT16 para)
{
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }

	SENSORDB("\n ****** SOC5140_MIPI_set_brightness \n ");
    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000); 
    S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
    S5K5CAGX_write_cmos_sensor(0x002a,0x020c);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
            S5K5CAGX_write_cmos_sensor(0x0f12,0xff80);
            break;
        case ISP_BRIGHT_HIGH:
        	  S5K5CAGX_write_cmos_sensor(0x0f12,0x00f0);
            break;
        case ISP_BRIGHT_MIDDLE:
        default:
        	   S5K5CAGX_write_cmos_sensor(0x0f12,0x0000);
             break;
    }    
	return ERROR_NONE;
}

kal_bool S5K5CAGX_set_param_effect(UINT16 para)
{
	kal_uint32 ret = KAL_TRUE;


    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return KAL_TRUE;
    }

    SENSORDB("\n****** S5K5CAGX_set_param_effect \n");
	//SENSORDB("para=%d",para);
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002a, 0x021e);
	
	switch (para)
	{
		case MEFFECT_OFF:
    		{
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); // Normal,
    		}
		    break;
		
		case MEFFECT_MONO:
		    {
    		   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // Monochrome (Black & White)
            }
    		break;
    		
		case MEFFECT_SEPIA:
    		{
    		   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0004); // Sepia
            }
    		break;
		
		case MEFFECT_SEPIABLUE:
    		{
		   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0005); // Aqua (Blue)
            }
    		break;
    		
	//	case MEFFECT_SEPIAGREEN:  5CA нч
		//    {
        		
   // 		}
   // 		break;
    		
		case MEFFECT_NEGATIVE:
    		{
    		   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002); // Negative Mono
            }
		    break;
		    
		default:
		   return KAL_FALSE;
	}
	return KAL_TRUE;
} /* S5K5CAGX_set_param_effect */

kal_bool S5K5CAGX_set_param_banding(UINT16 para)
{
    SENSORDB("\n****** S5K5CAGX_set_param_banding \n");

	SENSORDB("Manual anti-flicker,para=%d\n",para);
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return KAL_TRUE;
    }
	
	//K5CAGX_write_cmos_sensor(0x0028, 0x7000); 	// Set page 						
	//K5CAGX_write_cmos_sensor(0x002A, 0x04D2); 	// Set address						
	//K5CAGX_write_cmos_sensor(0x0F12, 0x065F); 	// #REG_TC_DBG_AutoAlgEnBits, AA_FLICKER OFF
	//S5K5CAGX_write_cmos_sensor(0x002a, 0x04BA);	//REG_SF_USER_FlickerQuant, 0: no AFC, 1: 50Hz, 2: 60Hz
    switch (para)
	{
		case AE_FLICKER_MODE_60HZ:
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT5, 0);
	        S5K5CAGX_write_cmos_sensor(0x002a, 0x04BA);	//REG_SF_USER_FlickerQuant, 0: no AFC, 1: 50Hz, 2: 60Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0002);		// 60Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);		//Sync F/W
		break;

		case AE_FLICKER_MODE_50HZ:
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT5, 0);
	        S5K5CAGX_write_cmos_sensor(0x002a, 0x04BA);	//REG_SF_USER_FlickerQuant, 0: no AFC, 1: 50Hz, 2: 60Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);		//50Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);		//Sync F/W
		break;
		
       case AE_FLICKER_MODE_OFF:
           {
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT5, 0);
	        S5K5CAGX_write_cmos_sensor(0x002a, 0x04BA);	//REG_SF_USER_FlickerQuant, 0: no AFC, 1: 50Hz, 2: 60Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);		//50Hz
			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);		//Sync F/W
           }
       break;
       
       case AE_FLICKER_MODE_AUTO:
       default:
           {
			S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT5, 1);
          break;
           }
   }

	return KAL_TRUE;
} /* S5K5CAGX_set_param_banding */

kal_bool S5K5CAGXHdr_CheckHDRstatus(void)
{
    if((s5k5cagx_stru.sceneMode == SCENE_MODE_HDR)
		&&(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE))
    {
		return KAL_TRUE;
	}
	else 
	{
		return KAL_FALSE;
	}
}
void S5K5CAGX_PORTRAIT_mode()
{
   SENSORDB("\n****** S5K5CAGX_PORTRAIT_mode \n");
   S5K5CAGXSetSceneAE();
   S5K5CAGX_write_cmos_sensor(0xFCFC,0xD000);
   S5K5CAGX_write_cmos_sensor(0x0028 ,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002A ,0x020C);
   S5K5CAGX_write_cmos_sensor(0x0F12 ,0x0000);
   S5K5CAGX_write_cmos_sensor(0x002A ,0x0210);
   S5K5CAGX_write_cmos_sensor(0x0F12 ,0x0000);
   S5K5CAGX_write_cmos_sensor(0x0F12,0xFFF6);
           //   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
           //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
           //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits  
			     S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
			     awbMode_5ca = AWB_MODE_AUTO;
}

void S5K5CAGX_LANDSCAPE_mode()
{
   SENSORDB("\n****** S5K5CAGX_LANDSCAPE_mode \n");
   S5K5CAGXSetSceneAE();
   S5K5CAGX_write_cmos_sensor(0xFCFC,0xD000);
   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002A , 0x020C);
   S5K5CAGX_write_cmos_sensor(0x0F12 , 0x0000);
   S5K5CAGX_write_cmos_sensor(0x002A , 0x0210);
   S5K5CAGX_write_cmos_sensor(0x0F12 , 0x001E);
   S5K5CAGX_write_cmos_sensor(0x0F12 , 0x000A);
           //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
           //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
           //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits  
	 S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
	 awbMode_5ca = AWB_MODE_AUTO;
   return ERROR_NONE;
}

void S5K5CAGX_SPORTS_mode()
{
   SENSORDB("\n****** S5K5CAGX_SPORTS_mode \n");

	//<CAMTUNING_SCENE_SPORTS>
    S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000);
    S5K5CAGX_write_cmos_sensor(0x0028 ,0x7000);

/*
    S5K5CAGX_write_cmos_sensor(0x002A ,0x0530);                                                         
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x36B0);   //lt_uMaxExp1	32 30ms  9~10ea// 15fps  // 
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x0534 );                                                              
    S5K5CAGX_write_cmos_sensor(0x0F12  ,0x36B0);   //lt_uMaxExp2	67 65ms	18~20ea // 7.5fps //
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x167C);                                                              
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x36B0);  //9C40//MaxExp3  83 80ms  24~25ea //                 
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x1680);                                                              
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x36B0 );  //MaxExp4   125ms  38ea //                           

    S5K5CAGX_write_cmos_sensor(0x002A ,0x0538);                                                               
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x36B0);  // 15fps //                                                 
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x053C);                                                               
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x36B0 );  // 7.5fps //                                                
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x1684);                                                               
    S5K5CAGX_write_cmos_sensor(0x0F12  ,0x36B0);   //CapMaxExp3 //                                             
    S5K5CAGX_write_cmos_sensor(0x002A   ,0x1688);                                                               
    S5K5CAGX_write_cmos_sensor(0x0F12  ,0x36B0);   //CapMaxExp4 //                                             

    S5K5CAGX_write_cmos_sensor(0x002A ,0x0540);                                                               
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x0200);   //0170//0150//lt_uMaxAnGain1_700lux//                                              
    S5K5CAGX_write_cmos_sensor(0x0F12 , 0x0200);  //0200//0400//lt_uMaxAnGain2_400lux//                              
    S5K5CAGX_write_cmos_sensor(0x002A  ,0x168C );                                                              
    S5K5CAGX_write_cmos_sensor(0x0F12,0x0200);   //0300//MaxAnGain3_200lux//                                       
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x0200);   //MaxAnGain4 //    

    S5K5CAGX_write_cmos_sensor(0x002A ,0x0544);  
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x0100);
    S5K5CAGX_write_cmos_sensor(0x0F12 ,0x8000);
*/	
   // S5K5CAGX_write_cmos_sensor(0x002A ,0x04B4);  
   // S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
   // S5K5CAGX_write_cmos_sensor(0x0F12,0x00C8);
   // S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);
    
    //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
    //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
    //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits  
 		S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
 		awbMode_5ca = AWB_MODE_AUTO;

}


static void S5K5CAGX_AWB_Lock(void)
{
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }

   if(S5K5CAGX_AEAWB_PRE == ae_awb_state)
   {
        return;
   }

	if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
    {
        return;
    }

   SENSORDB("\n ****** S5K5CAGX_AWB_Lock\n");
  
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,0);

    //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
    //S5K5CAGX_write_cmos_sensor(0x002A,0x246E);
    //S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);                                                                         
    return;
}

static void S5K5CAGX_AWB_UnLock(void)
{
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }


   if(S5K5CAGX_AEAWB_PRE == ae_awb_state)
   {
        ae_awb_state = S5K5CAGX_AEAWB_PARA;
        return;
   }

   	if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
    {
        return;
    }

   SENSORDB("\n ****** S5K5CAGX_AWB_UnLock\n");

    S5K5CAGX_set_param_wb(awbMode_5ca);
	//S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
    //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
    //S5K5CAGX_write_cmos_sensor(0x002A,0x246E);  
    //S5K5CAGX_write_cmos_sensor(0x0F12,0x0001); 
    return;
}

//manuel exposure enable
static void S5K5CAGX_AE_Lock(void)
{
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }

    if(S5K5CAGX_AEAWB_PRE == ae_awb_state)
    {
        return;
    }

	if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
    {
        return;
    }

    SENSORDB("\n ****** S5K5CAGX_AE_Lock\n");

     S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,0);
     S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,0);

    //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
    //S5K5CAGX_write_cmos_sensor(0x002A,0x2466);                                                                          
    //S5K5CAGX_write_cmos_sensor(0x0F12,0x0000); 
    return;
}

//ae enable
static void S5K5CAGX_AE_UnLock(void)
{
   if(S5K5CAGX_YUV_INIT == write_statue)
   {
       return;
   }

   if(S5K5CAGX_AEAWB_PRE == ae_awb_state)
   {
        return;
   }

   	if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
    {
        return;
    }

   SENSORDB("\n ****** S5K5CAGX_AE_UnLock\n");

   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,1);
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,1);
   
   //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   //S5K5CAGX_write_cmos_sensor(0x002A,0x2466);                                                                          
   //S5K5CAGX_write_cmos_sensor(0x0F12,0x0001); 
   return;
}

kal_uint16 S5K5CAGXReadShutter(void)
{
   kal_uint32 Shutter_lowIndex=0,Shutter_highIndex=0,Shutter=0;

   SENSORDB("\n ******* S5K5CAGXReadShutter \n");
   S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000);
   S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002E,0x23e8);
   Shutter_lowIndex=S5K5CAGX_read_cmos_sensor(0x0F12);
   Shutter_highIndex=S5K5CAGX_read_cmos_sensor(0x0F12);

   SENSORDB("1Shutter_lowIndex =%d,Shutter_highIndex=%d  \n",Shutter_lowIndex, Shutter_highIndex);

   Shutter = (Shutter_highIndex<<16)|Shutter_lowIndex;
   Shutter = Shutter / 4;
   if(Shutter <1)
   {
        Shutter=1;
   }
            
   SENSORDB("Shutter =0x%x\n",Shutter);

   return Shutter;
}

kal_uint16 S5K5CAGXReadGain()
{
   kal_uint32 Reg=0 ;

   SENSORDB("\n ******* S5K5CAGXReadGain\n");

   S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002E,0x23EC);
   Reg=S5K5CAGX_read_cmos_sensor(0x0F12);         

   SENSORDB("gain = 0x%x\n", Reg);

   if(Reg<1)
   {
     Reg=1;
   }
   return Reg; 
}

kal_uint16 S5K5CAGXReadAwbRGain()
{
   kal_uint32 RGain=0 ;

   SENSORDB("\n ******* S5K5CAGXReadAwbRGain\n");
   S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002E,0x23F8);
   RGain=S5K5CAGX_read_cmos_sensor(0x0F12);     
   //SENSORDB("[5CA]: S5K5CAGXReadAwb_RGain RGain-org =%d \n",RGain);
    SENSORDB("1RGain =%d \n",RGain);

    //sensor gain base is 1024;
    RGain=RGain/8;

   if(RGain<1)
            RGain=1;
   
    SENSORDB("2RGain =%d \n",RGain);
   return RGain;

}

kal_uint16 S5K5CAGXReadAwbBGain()
{
   kal_uint32 BGain=0 ;

   SENSORDB("\n ******* S5K5CAGXReadAwbBGain\n");
   S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002E,0x23FC);
   BGain=S5K5CAGX_read_cmos_sensor(0x0F12); 
   SENSORDB("1BGain =%d \n",BGain);

   
   BGain=BGain/8;

   if(BGain<1)
            BGain=1;
   
   SENSORDB("2BGain =%d \n",BGain);
   return BGain;

}

void S5K5CAGXHdr_Part1(void)
{
   //read org shutter/gain
   kal_uint32 shutter=0,gain=0;

   SENSORDB("\n ******* S5K5CAGXHdr_Part1\n");

   shutter= S5K5CAGXReadShutter();
   gain= S5K5CAGXReadGain();
   
   spin_lock(&s5k5cagx_drv_lock);
   s5k5cagx_stru.iShutter= shutter;
   s5k5cagx_stru.iGain= gain;
   spin_unlock(&s5k5cagx_drv_lock);

   SENSORDB("\n s5k5cagx_stru.iShutter=%d, s5k5cagx_stru.iGain=%d\n",s5k5cagx_stru.iShutter, s5k5cagx_stru.iGain);
 }

void S5K5CAGXHdr_Part2(UINT16 para)
{
   kal_uint32 HDRShutter =0,HDRGain=0;

   SENSORDB("\n********* S5K5CAGXHdr_Part2  \n");
#if 0   
    {
       kal_uint32   gain =0;
       kal_uint32   Shutter_l =0;
       kal_uint32   Shutter_h =0;
       S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002E,0x23EC);
       gain=S5K5CAGX_read_cmos_sensor(0x0F12);         


       S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000);
       S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002E,0x23e8);
       Shutter_l=S5K5CAGX_read_cmos_sensor(0x0F12);
       Shutter_l = Shutter_l / 4;
       Shutter_h=S5K5CAGX_read_cmos_sensor(0x0F12);
       SENSORDB("s_gain =0x%x,Shutter_l=0x%x Shutter_h = 0x%x  \n",gain, Shutter_l,Shutter_h);
   }
#endif   
    
   HDRShutter = s5k5cagx_stru.iShutter;
   HDRGain = s5k5cagx_stru.iGain;
   SENSORDB("1HDRShutter = %x,HDRGain = %x \n", HDRShutter, HDRGain);

   switch (para)
   {
      case AE_EV_COMP_20:
      case AE_EV_COMP_10:
           {
               SENSORDB("EV * 2 \n");
                S5K5CAGX_SCENE_NIGHT();
                S5K5CAGX_night_mode(KAL_TRUE);
                HDRGain = HDRGain<<2;
               HDRShutter = HDRShutter<<2;
           }
           break;
      case AE_EV_COMP_00:
            {
              SENSORDB("EV 0 \n");
            }
            break;
      case AE_EV_COMP_n10:
      case AE_EV_COMP_n20:
            {
              SENSORDB("EV chu 2 \n");
              HDRGain = HDRGain >> 2;
              HDRShutter = HDRShutter >> 2;
            }
             break;
      default:
             break; 
   }

   SENSORDB("2HDRShutter = %x,HDRGain = %x \n", HDRShutter, HDRGain);
   //SANDY CHECK
   HDRShutter = (HDRShutter<0x4E20)? HDRShutter:0x4E20;
   HDRGain = (HDRGain<0x0A00)? HDRGain:0x0A00;
   SENSORDB("3HDRShutter = %x,HDRGain = %x \n", HDRShutter, HDRGain);

   //Write shutter
   //5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   //K5CAGX_write_cmos_sensor(0x002A,0x04D2);
   //K5CAGX_write_cmos_sensor(0x0F12,0x0779);
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,0);
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,0);

   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002A,0x048E);
   S5K5CAGX_write_cmos_sensor(0x0F12,HDRShutter & 0x0000ffff);
   S5K5CAGX_write_cmos_sensor(0x002A,0x0490);
   S5K5CAGX_write_cmos_sensor(0x0F12,HDRShutter & 0xffff0000);
   S5K5CAGX_write_cmos_sensor(0x002A,0x0492);
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);

    //Write Gain
   //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);
   //S5K5CAGX_write_cmos_sensor(0x0F12,0x0779);
   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   S5K5CAGX_write_cmos_sensor(0x002A,0x04AC);
   S5K5CAGX_write_cmos_sensor(0x0F12,HDRGain);
   S5K5CAGX_write_cmos_sensor(0x002A,0x04AE);
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);
   mdelay(300);
   
#if 0
{
       kal_uint32   gain =0;
       kal_uint32   Shutter_l =0;
       kal_uint32   Shutter_h =0;
       S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002E,0x23EC);
       gain=S5K5CAGX_read_cmos_sensor(0x0F12);         


       S5K5CAGX_write_cmos_sensor(0xfcfc,0xd000);
       S5K5CAGX_write_cmos_sensor(0x002C,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002E,0x23e8);
       Shutter_l=S5K5CAGX_read_cmos_sensor(0x0F12);
       Shutter_l = Shutter_l / 4;
       Shutter_h=S5K5CAGX_read_cmos_sensor(0x0F12);
       SENSORDB("s_gain =0x%x,Shutter_l=0x%x Shutter_h = 0x%x  \n",gain, Shutter_l,Shutter_h);
   }
#endif

   spin_lock(&s5k5cagx_drv_lock);
   s5k5cagx_stru.manualAEStart=KAL_TRUE;
   spin_unlock(&s5k5cagx_drv_lock);
}

void S5K5CAGXHdr_Part3()
{
    SENSORDB("\n****** S5K5CAGXHdr_Part3 \n");

   //open auto AE
   //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
   //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);
   //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,1);
   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,1);
   S5K5CAGX_night_mode(KAL_FALSE);
         
   spin_lock(&s5k5cagx_drv_lock);
   s5k5cagx_stru.manualAEStart=KAL_FALSE;
   s5k5cagx_stru.iShutter= 0;
   s5k5cagx_stru.iGain= 0;
   spin_unlock(&s5k5cagx_drv_lock);
}

kal_bool S5K5CAGX_set_param_exposure(UINT16 para)
{
    SENSORDB("\n****** S5K5CAGX_set_param_exposure \n");

    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return KAL_TRUE;
    }

   if(S5K5CAGXHdr_CheckHDRstatus())
   {
        S5K5CAGXHdr_Part2(para);
        return true;
   }
   
	SENSORDB("para=%d\n",para);
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0218);		
    switch (para)
	{	
    	case AE_EV_COMP_n20: 
    	    {
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0080);  // EV-2
            }
    		break;
		
    	case AE_EV_COMP_n10:  
        	{
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x00b0);  // EV-1
            }
		    break;
		    
		case AE_EV_COMP_00:	
		    {
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0100); // Default Value, 0
            }
    		break;
    		
		case AE_EV_COMP_10:
		    {
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0140);  // EV1
            }
    		break;
    		
    	case AE_EV_COMP_20:
    	    {
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x0180);  // EV2
            }
    		break;		
		default:			
			return KAL_FALSE;
	}
	return KAL_TRUE;
}/* S5K5CAGX_set_param_exposure */


UINT32 S5K5CAGX_SetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("\n****** S5K5CAGX_SetTestPatternMode \n");
    SENSORDB("bEnable = %d \n", bEnable);

	if(bEnable) 	
	{
	S5K5CAGX_write_cmos_sensor(0x0028, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0xB054);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	}
	else        
	{
	S5K5CAGX_write_cmos_sensor(0x0028, 0xD000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0xB054);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);
	}
    return ERROR_NONE;
}
void S5K5CAGX_PV_Mode(kal_uint8 num)
{
       SENSORDB("\n****** S5K5CAGX_PV_Mode \n");
		//================================================================
		// APPLY PREVIEW CONFIGURATION & RUN PREVIEW
		//================================================================
        S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
		S5K5CAGX_write_cmos_sensor(0x002A, 0x168E);
		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0A00);
		
		S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
//		S5K5CAGX_write_cmos_sensor(0x002A, 0x12c8);
//		S5K5CAGX_write_cmos_sensor(0x0F12, 0x08ac);
		S5K5CAGX_write_cmos_sensor(0x002A, 0x023C);
		S5K5CAGX_write_cmos_sensor(0x0F12, num); // #REG_TC_GP_ActivePrevConfig // Select preview configuration_0
		S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
		S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
		S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
		S5K5CAGX_write_cmos_sensor(0x002A, 0x0220);
		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_EnablePreview // Start preview

        S5K5CAGX_write_cmos_sensor(0x0028, 0xD000); 	//Host Interrupt
        S5K5CAGX_write_cmos_sensor(0x002A, 0xB0A0); 
        S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 

		S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
		S5K5CAGX_write_cmos_sensor(0x002A, 0x0222);		

		S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_EnablePreviewChanged
}

void S5K5CAGX_CAP_Mode2(void)
{

    SENSORDB("\n****** S5K5CAGX_CAP_Mode2 \n");
//    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
//    S5K5CAGX_write_cmos_sensor(0x002A, 0x168E);
//    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0700);


	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000); 
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0244); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);  //config select   0 :48, 1, 20,
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0230);  			 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);  				 
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0246);  					 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 		 
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0224);  			 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 

    S5K5CAGX_write_cmos_sensor(0x0028, 0xD000); 	//Host Interrupt
    S5K5CAGX_write_cmos_sensor(0x002A, 0xB0A0); 
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 
 
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0226); 				 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);  			 
	//S5K5CAGX_gPVmode = KAL_FALSE;
}


void S5K5CAGX_CAP_Mode(void)
{

    SENSORDB("\n****** S5K5CAGX_CAP_Mode \n");
 //   S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
 //   S5K5CAGX_write_cmos_sensor(0x002A, 0x168E);
 //   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0700);

	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
//	S5K5CAGX_write_cmos_sensor(0x002A, 0x12c8);
//	S5K5CAGX_write_cmos_sensor(0x0F12, 0x1200);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0244); 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000);  //config select   0 :48, 1, 20,
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0230); 			 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 				 
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0246); 					 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 		 
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0224); 			 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 

    S5K5CAGX_write_cmos_sensor(0x0028, 0xD000); 	//Host Interrupt
    S5K5CAGX_write_cmos_sensor(0x002A, 0xB0A0); 
    S5K5CAGX_write_cmos_sensor(0x0F12, 0x0000); 

	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);	
	S5K5CAGX_write_cmos_sensor(0x002a, 0x0226); 				 
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); 			 
	//S5K5CAGX_gPVmode = KAL_FALSE;
}

void S5K5CAGX_night_mode(kal_bool NightModeEnable)
{
	kal_uint16 video_frame_len = 0;
	kal_uint16 prev_line_len = 0;

	SENSORDB("\n*****S5K5CAGX_night_mode\n");
	spin_lock(&s5k5cagx_drv_lock);
    s5k5cagx_stru.NightMode = NightModeEnable;
	spin_unlock(&s5k5cagx_drv_lock);

/*	
	   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
   S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits    
                                                                                                                                                                                    
   S5K5CAGX_write_cmos_sensor(0x002A,0x0252);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd 
                                                    
   S5K5CAGX_write_cmos_sensor(0x002A,0x020C);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness            
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast                  
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation             
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur  
*/  
    if(S5K5CAGX_MODE_PREVIEW == s5k5cagx_stru.Camco_mode)
    {
    	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0288);	//#REG_0TC_PCFG_usMaxFrTimeMsecMult10 
    	
    	if (NightModeEnable)
    	{
     			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NIT_MAX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NIT_MIN_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10

    	}
    	else
    	{
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NOM_MAX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NOM_MIN_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10

    	}

       

    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
    }
    else if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_VIDEO)
    {
        	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0288);	
       	if (NightModeEnable)
    	{
                S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NIT_FIX_FPS); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NIT_FIX_FPS); //#REG_0TC_PCFG_usMinFrTimeMsecMult10
    	}
    	else
    	{
               S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NOM_FIX_FPS); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NOM_FIX_FPS); //#REG_0TC_PCFG_usMinFrTimeMsecMult10

    	}
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
    }
    else if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_ZSD)
    {
		if (NightModeEnable)
		{ 
			S5K5CAGX_PV_Mode(2);
		}
		else
		{ 
		 	S5K5CAGX_PV_Mode(1);
		}
    }
    else if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
    {

 	   if (NightModeEnable)
   	   { 
			S5K5CAGX_CAP_Mode2();
  	   }
 	   else
	   { 
		 	S5K5CAGX_CAP_Mode();
 	   }
    }
}

void S5K5CAGX_Sport_night_mode(kal_bool NightModeEnable)
{
	kal_uint16 video_frame_len = 0;
	kal_uint16 prev_line_len = 0;

	SENSORDB("\n*****S5K5CAGX_night_mode\n");
	spin_lock(&s5k5cagx_drv_lock);
    s5k5cagx_stru.NightMode = NightModeEnable;
	spin_unlock(&s5k5cagx_drv_lock);

/*	
	   S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
   S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits    
                                                                                                                                                                                    
   S5K5CAGX_write_cmos_sensor(0x002A,0x0252);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd 
                                                    
   S5K5CAGX_write_cmos_sensor(0x002A,0x020C);//                                                                         
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness            
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast                  
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation             
   S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur  
*/  
    if(S5K5CAGX_MODE_PREVIEW == s5k5cagx_stru.Camco_mode)
    {
    	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0288);	//#REG_0TC_PCFG_usMaxFrTimeMsecMult10 
    	
    	if (NightModeEnable)
    	{
     			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NIT_MAX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_CAM_NIT_MIN_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10

    	}
    	else
    	{
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x029A); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
    			S5K5CAGX_write_cmos_sensor(0x0F12, 0x014D); //#REG_0TC_PCFG_usMinFrTimeMsecMult10

    	}
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
    	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
    	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged
    }
    else if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_ZSD)
    {
		if (NightModeEnable)
		{ 
			S5K5CAGX_PV_Mode(2);
		}
		else
		{ 
		 	S5K5CAGX_PV_Mode(1);
		}
    }
}

void S5K5CAGX_SCENE_SUNSET(void)
{

	SENSORDB("\n*****S5K5CAGX_SCENE_SUNSET\n");

       //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
       //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
       //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits 
       S5K5CAGXSetSceneAE();
   	   //S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
       S5K5CAGX_write_cmos_sensor(0x002A,0x0252);//                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd  
                                                        
       S5K5CAGX_write_cmos_sensor(0x002A,0x020C);//                                                                         
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness            
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast                  
       S5K5CAGX_write_cmos_sensor(0x002A,0x0210);               
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation             
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur   
       
       S5K5CAGX_write_cmos_sensor(0x002A,0x0F7E);// 
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_0_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_1_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_2_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_3_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_4_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_5_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_6_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_7_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_8_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_9_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_10_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_11_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_12_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_13_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_14_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_15_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_16_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_17_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_18_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_19_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_20_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_21_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_22_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_23_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_24_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_25_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_26_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_27_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_28_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_29_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_30_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_31_     
       
       S5K5CAGX_write_cmos_sensor(0x002A,0x023C);                                                                          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);
	   S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3, 0);
	   S5K5CAGX_write_cmos_sensor(0x002A, 0x04A0);
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0730); //#REG_SF_USER_Rgain
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0400); //#REG_SF_USER_Ggain
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0540); //#REG_SF_USER_Bgain
	   S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);
	   awbMode_5ca = AWB_MODE_CLOUDY_DAYLIGHT;
}

void S5K5CAGX_SCENE_NIGHT(void)
{

	SENSORDB("\n*****S5K5CAGX_SCENE_NIGHT\n");

       //S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
       //S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
       //S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits    
       S5K5CAGXSetSceneAE();
       //S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);   
        
       S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                                                                                                                                  
       S5K5CAGX_write_cmos_sensor(0x002A,0x0252);//                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd  
                                                        
      // S5K5CAGX_write_cmos_sensor(0x002A,0x020C);//                                                                         
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness            
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast   
      S5K5CAGX_write_cmos_sensor(0x002A,0x0210);               
      S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation             
      S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur   
       
       S5K5CAGX_write_cmos_sensor(0x002A,0x0F7E);// 
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_0_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_1_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_2_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_3_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_4_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_5_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_6_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_7_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_8_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_9_          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_10_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_11_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_12_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_13_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_14_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_15_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_16_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_17_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_18_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_19_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_20_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_21_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_22_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_23_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_24_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_25_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_26_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_27_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_28_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_29_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_30_   
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_31_     
       
       S5K5CAGX_write_cmos_sensor(0x002A,0x023C);                                                                          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);
       S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT3,1);
        awbMode_5ca = AWB_MODE_AUTO;
}

void S5K5CAGX_SCENE_Normal(void)
{

	SENSORDB("\n*****S5K5CAGX_SCENE_Normal\n");


      // S5K5CAGX_write_cmos_sensor(0x0028,0x7000);//                                                                          
      // S5K5CAGX_write_cmos_sensor(0x002A,0x04D2);//                                                                         
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits    
       S5K5CAGXSetSceneAE();
       S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002A,0x0252);//                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd  
                                                        
      // S5K5CAGX_write_cmos_sensor(0x002A,0x020C);//                                                                         
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness            
      // S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast                  
       S5K5CAGX_write_cmos_sensor(0x002A,0x0210);               
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation             
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur   
       S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
       S5K5CAGX_write_cmos_sensor(0x002A,0x023C);                                                                          
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);                                                                         
       S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);

}

void S5K5CAGXSetSceneMode(UINT16 iPara)
{
    SENSORDB("\n****** S5K5CAGXSetSceneMode \n");
	spin_lock(&s5k5cagx_drv_lock);
    s5k5cagx_stru.sceneMode = iPara;
	spin_unlock(&s5k5cagx_drv_lock);

    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return;
    }

	SENSORDB("iPara=%d",iPara);
    switch(iPara)
    {
        case SCENE_MODE_HDR:
             if(s5k5cagx_stru.manualAEStart==KAL_TRUE)
             {
               S5K5CAGXHdr_Part3();
             }
             S5K5CAGX_night_mode(KAL_FALSE); 
             break;
             
        case SCENE_MODE_SUNSET:
            {
                S5K5CAGX_SCENE_SUNSET();
                S5K5CAGX_night_mode(KAL_FALSE); 
            } 
            break;
            
	    case SCENE_MODE_NIGHTSCENE:
            {
                 S5K5CAGX_SCENE_NIGHT();
                 S5K5CAGX_night_mode(KAL_TRUE); 
            }
            break;

	    case SCENE_MODE_PORTRAIT:
	        {
    			S5K5CAGX_PORTRAIT_mode();
    			S5K5CAGX_night_mode(KAL_FALSE);
			}
		    break;
		
	    case SCENE_MODE_LANDSCAPE:
	        {
    			S5K5CAGX_LANDSCAPE_mode();
    			S5K5CAGX_night_mode(KAL_FALSE); 
			}
		    break;
		
	    case SCENE_MODE_SPORTS:
    	    {
    	        S5K5CAGX_SPORTS_mode(); 
    	        S5K5CAGX_Sport_night_mode(KAL_FALSE); 
	        }
		    break;

    	case SCENE_MODE_OFF:
    	default:
           { 
             S5K5CAGX_SCENE_Normal();
             S5K5CAGX_set_param_wb(awbMode_5ca);
             S5K5CAGX_night_mode(KAL_FALSE); 
           }
           break;

    }   
}

void S5K5CAGX_set_ISO(UINT16 iPara)
{ 
    SENSORDB("\n****** S5K5CAGX_set_ISO \n");
    if(S5K5CAGX_YUV_INIT == write_statue)
    {
        return KAL_TRUE;
    }

    switch(iPara)
    {
	    case AE_ISO_100:
	        S5K5CAGX_set_ISO_100();
		break;
	    case AE_ISO_200:
	        S5K5CAGX_set_ISO_200();
		break;
	    case AE_ISO_400:
	        S5K5CAGX_set_ISO_400();
		break;
            case AE_ISO_AUTO:
	     default:
	        S5K5CAGX_set_ISO_Auto();
		break;
    }
    return;
}


void S5K5CAGXset_hue(UINT16 para)
{
	//SENSORDB("\n******* S5K5CAGXset_hue\n");
	//SENSORDB("[Enter]S5K5EAYX S5K5EAYX_set_hue func:para = %d,ISP_HUE_MIDDLE=%d\n",para,ISP_HUE_MIDDLE);
/*
	switch (para)
	{
		case ISP_HUE_LOW:
			         S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
                     S5K5CAGX_write_cmos_sensor(0x002A,0x2474);  //Hue1
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);
                     S5K5CAGX_write_cmos_sensor(0x002A,0x24A2); 
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x01BB); 
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF35);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x00BB);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFFB8);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0122);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFE7D);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF70);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x007C);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0188);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x014E);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x003B);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF7A);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x00A3);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF7D);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x01C8);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF99);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x021E);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0103); 
			break;
			
		case ISP_HUE_HIGH:
    	             S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
                     S5K5CAGX_write_cmos_sensor(0x002A,0x2474); //Hue3
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0000);
                     
                     S5K5CAGX_write_cmos_sensor(0x002A,0x24A2); 
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x01E8); 
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0061);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF62);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFE6D);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0112);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFFD7);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x007C);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF70);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0188);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0072);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0133);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF5F);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x01C8);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFF7D);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x00A2);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0xFFB3);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x00CF);
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0238);
			 break;
			 
		case ISP_HUE_MIDDLE:
		default:
			         S5K5CAGX_write_cmos_sensor(0x0028,0x7000);
                     S5K5CAGX_write_cmos_sensor(0x002A,0x2474); //REG_TC_UserSaturation
                     S5K5CAGX_write_cmos_sensor(0x0F12,0x0001); //Hue 2
			 break;

	}
	*/
	return;
}

void S5K5CAGX_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{

  SENSORDB("\n****** S5K5CAGX_3ACtrl \n");

   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          S5K5CAGX_AE_Lock();
          break;
	  
      case SENSOR_3A_AE_UNLOCK:
          S5K5CAGX_AE_UnLock();
          break;

      case SENSOR_3A_AWB_LOCK:
          S5K5CAGX_AWB_Lock();
          break;

      case SENSOR_3A_AWB_UNLOCK:
          S5K5CAGX_AWB_UnLock();
          break;
      
      default:
      	break;
   }
   return;
}

void S5K5CAGX_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
    SENSORDB("\n****** S5K5CAGX_AutoTestCmd \n");

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
	
}


UINT32 S5K5CAGXYUVSensorSetting(FEATURE_ID iCmd, UINT16 iPara)
{
    SENSORDB("\n****** S5K5CAGXYUVSensorSetting \n");
	SENSORDB("iCmd=%d,iPara=%d\n",iCmd,iPara);

	switch (iCmd)
	{
	case FID_COLOR_EFFECT:
			S5K5CAGX_set_param_effect(iPara);
	     break;

	case FID_AE_EV:	    	    
			S5K5CAGX_set_param_exposure(iPara);
	     break;

	case FID_SCENE_MODE:
			S5K5CAGXSetSceneMode(iPara); 
	     break; 	    
		 
	case FID_AWB_MODE:
			S5K5CAGX_set_param_wb(iPara);
	     break;
		 
	case FID_AE_FLICKER:	    	    	    
			S5K5CAGX_set_param_banding(iPara);
	     break;

    case FID_ISP_SAT:
			S5K5CAGX_set_saturation(iPara);
		break;

	case FID_ISP_CONTRAST:
	        S5K5CAGX_set_contrast(iPara);
		break;
		 
	 case FID_ISP_BRIGHT:
          S5K5CAGX_set_brightness(iPara);
		 break;

	case FID_AE_ISO:
	        S5K5CAGX_set_ISO(iPara);
		break;

	case FID_AE_SCENE_MODE:
			if (iPara == AE_MODE_OFF) 
			{
			    S5K5CAGX_AE_Lock();
	      	}
			else 
			{
			    S5K5CAGX_AE_UnLock();
			}
		break;

	case FID_ISP_HUE:
	        S5K5CAGXset_hue(iPara);
		 break;
		 
	case FID_ZOOM_FACTOR:
			spin_lock(&s5k5cagx_drv_lock);
			zoom_factor = iPara; 		
			spin_unlock(&s5k5cagx_drv_lock);
		break; 

	default:
	     break;
	}
	
	return TRUE;
}   /* S5K5CAGXYUVSensorSetting */

UINT32 S5K5CAGXYUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("\n****** S5K5CAGXYUVSetVideoMode2 \n");
	SENSORDB("u2FrameRate=%d\n",u2FrameRate);
    spin_lock(&s5k5cagx_drv_lock);    //S5K5CAGX_VEDIO_encode_mode = KAL_TRUE; 
    s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_VIDEO;
	spin_unlock(&s5k5cagx_drv_lock);
	
	S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0288);	//#REG_0TC_PCFG_usMaxFrTimeMsecMult10 

    if (u2FrameRate == 30)
    {		
		S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
		S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NOM_FIX_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10
    }
    else if (u2FrameRate == 15)
    {
		S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NIT_FIX_FR_TIME); //#REG_0TC_PCFG_usMaxFrTimeMsecMult10
	    S5K5CAGX_write_cmos_sensor(0x0F12, S5K5CAGX_VID_NIT_FIX_FR_TIME); //#REG_0TC_PCFG_usMinFrTimeMsecMult10
    }
    else 
    {
        SENSORDB("Wrong Frame Rate\n"); 
    }

	SENSORDB("S5K5CAGXYUVSetVideoMode\n");
	
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0240);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevOpenAfterChange
	S5K5CAGX_write_cmos_sensor(0x002A, 0x0230);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_NewConfigSync // Update preview configuration
	S5K5CAGX_write_cmos_sensor(0x002A, 0x023E);
	S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001); // #REG_TC_GP_PrevConfigChanged    
    return 0;
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////AF  Function///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void MiddlewaresizePoint2PreviewsizePoint(
    UINT32 mx,
    UINT32 my,
    UINT32 mw,
    UINT32 mh,
    UINT32 * pvx,
    UINT32 * pvy,
    UINT32 pvw,
    UINT32 pvh
)
{
    *pvx = pvw * mx / mw;
    *pvy = pvh * my / mh;
    SENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}

void S5K5EAYX_MIPI_AF_Init(void)
{
	SENSORDB("\n ****** S5K5EAYX_MIPI_AF_Init \n ");

}

static UINT32 S5K5EAYX_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
	return ERROR_NONE;
}

static void S5K5EAYX_FOCUS_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
    unsigned short state;
    S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);                         
    S5K5CAGX_write_cmos_sensor(0x002c, 0x7000);                         
    S5K5CAGX_write_cmos_sensor(0x002e, 0x26fe);

    state = S5K5CAGX_read_cmos_sensor(0x0F12);
    
    if(0x00 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_IDLE;    
    }
    else if(0x01 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;        
    }
    else if(0x02 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;//SENSOR_AF_FOCUSED;
        //s5k5ca_af_num = 0;   
    }
    else
    {
            *pFeatureReturnPara32 = SENSOR_AF_ERROR;// SENSOR_AF_ERROR;
     }
    
    SENSORDB("lln state = 0x%x, *pFeatureReturnPara32 = %d\n",  state, *pFeatureReturnPara32);  

}

static void S5K5CAGX_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void S5K5CAGX_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 255;
}

static void S5K5CAGX_FOCUS_Constant_Focus(void)
{
	SENSORDB("\n ****** S5K5CAGX_FOCUS_Constant_Focus \n ");
              S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);                       
            S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);                         
            S5K5CAGX_write_cmos_sensor(0x002A,0x0252);
            S5K5CAGX_write_cmos_sensor(0x0F12,0x0006); 

}   

static void S5K5CAGX_FOCUS_Single_Focus()
{
	SENSORDB("\n ****** S5K5CAGX_FOCUS_Single_Focus \n ");
    S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);                                                        
    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);                                                        
    S5K5CAGX_write_cmos_sensor(0x002A,0x0252);
    S5K5CAGX_write_cmos_sensor(0x0F12,0x0005);

	SENSORDB("[S5K5CAGX]exit S5K5CAGX_FOCUS_Single_Focus function:\n ");
}

static void S5K5CAGX_FOCUS_Touch_AF(UINT32 x_start,UINT32 y_start, UINT32 x_end, UINT32 y_end)
{
     UINT32  x_width = 0;
     UINT32  y_height = 0;

	SENSORDB("\n ****** S5K5CAGX_FOCUS_Touch_AF \n ");
}

void S5K5CAGX_MIPI_AF_Init(void)
{
	SENSORDB("\n ****** S5K5CAGX_MIPI_AF_Init \n ");

}

static void S5K5CAGX_FOCUS_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
    unsigned short state;
    S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);                         
    S5K5CAGX_write_cmos_sensor(0x002c, 0x7000);                         
    S5K5CAGX_write_cmos_sensor(0x002e, 0x26fe);

    state = S5K5CAGX_read_cmos_sensor(0x0F12);
    
    if(0x00 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_IDLE;    
    }
    else if(0x01 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;        
    }
    else if(0x02 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;//SENSOR_AF_FOCUSED;
        //s5k5ca_af_num = 0;   
    }
    else
    {
            *pFeatureReturnPara32 = SENSOR_AF_ERROR;// SENSOR_AF_ERROR;
     }
    
    SENSORDB("lln state = 0x%x, *pFeatureReturnPara32 = %d\n",  state, *pFeatureReturnPara32);  

}

static void S5K5CAGX_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone

     UINT32 FD_XS = 4;
     UINT32 FD_YS = 3;  
     UINT32 x0, y0, x1, y1;
     UINT32* zone = (UINT32*)zone_addr;
     UINT32 width_ratio, height_ratio, start_x, start_y;  
     static UINT32 LastPosition=300;
  
     x0 = *zone;
     y0 = *(zone + 1);
     x1 = *(zone + 2);
     y1 = *(zone + 3);       
     FD_XS = *(zone + 4);
     FD_YS = *(zone + 5);
  
  SENSORDB(" [5CA]:[S5K5CAGX]enter S5K5CAGX_AF_Set_Window function:\n ");

#if 0
    //Do CAF
   if(x0==x1)
            return;

   //check if we really need to update AF window
   if((LastPosition<x0 && LastPosition+S5K5CA_FAF_TOLERANCE>x0)||
            (LastPosition>x0 && x0+S5K5CA_FAF_TOLERANCE>LastPosition))
   {
            LastPosition = x0;
            return;
   }

   LastPosition = x0;
    if(s5k5cagx_stru.Camco_mode != S5K5CAGX_MODE_ZSD)
   {
            //width_ratio =  1024*(x1 - x0)/FD_XS;
            //height_ratio = 1024*(y1 - y0)/FD_YS;
        width_ratio = S5K5CA_PRV_RATIO_WIDTH;
            height_ratio = S5K5CA_PRV_RATIO_HEIGHT;

            start_x = x0*S5K5CA_PRE_AFRangeWidth/FD_XS;
            start_y = y0*S5K5CA_PRE_AFRangeHeight/FD_YS;
            
        if(start_x+S5K5CA_PRV_RATIO_WIDTH > S5K5CA_PRE_AFRangeWidth)
                     start_x = S5K5CA_PRE_AFRangeWidth-S5K5CA_PRV_RATIO_WIDTH;
            if(start_y+S5K5CA_PRV_RATIO_HEIGHT>S5K5CA_PRE_AFRangeHeight)
                     start_y = S5K5CA_PRE_AFRangeHeight-S5K5CA_PRV_RATIO_HEIGHT;
   }
   else
   {
         //ZSD
              //width_ratio =  2048*(x1 - x0)/FD_XS;
              //height_ratio = 2048*(y1 - y0)/FD_YS;
              width_ratio = S5K5CA_CAP_RATIO_WIDTH;
              height_ratio = S5K5CA_CAP_RATIO_HEIGHT;

              start_x = x0*S5K5CA_CAP_AFRangeWidth/FD_XS;
              start_y = y0*S5K5CA_CAP_AFRangeHeight/FD_YS;

            if(start_x+S5K5CA_CAP_RATIO_WIDTH > S5K5CA_CAP_AFRangeWidth)
                     start_x = S5K5CA_CAP_AFRangeWidth-S5K5CA_CAP_RATIO_WIDTH;
            if(start_y+S5K5CA_CAP_RATIO_HEIGHT>S5K5CA_CAP_AFRangeHeight)
                     start_y = S5K5CA_CAP_AFRangeHeight-S5K5CA_CAP_RATIO_HEIGHT;

   }
  
     SENSORDB("[5CA]:af x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
                                         x0, y0, x1, y1, FD_XS, FD_YS);                   
     SENSORDB("[5CA]:af start_x=%d,start_y=%d,width_ratio=%d,height_ratio=%d\n",
                                         start_x, start_y, width_ratio, height_ratio); 
  
     S5K5CAGX_write_cmos_sensor(0x002A,                0x025A);
     S5K5CAGX_write_cmos_sensor(0x0F12,          start_x);                   //#REG_TC_AF_FstWinStartX
     S5K5CAGX_write_cmos_sensor(0x0F12,          start_y);                   //#REG_TC_AF_FstWinStartY
     S5K5CAGX_write_cmos_sensor(0x0F12,   width_ratio);                 //#REG_TC_AF_FstWinSizeX
     S5K5CAGX_write_cmos_sensor(0x0F12, height_ratio);                  //#REG_TC_AF_FstWinSizeY
     S5K5CAGX_write_cmos_sensor(0x0F12,          start_x);                   //#REG_TC_AF_ScndWinStartX
     S5K5CAGX_write_cmos_sensor(0x0F12,          start_y);                   //#REG_TC_AF_ScndWinStartY
     S5K5CAGX_write_cmos_sensor(0x0F12,   width_ratio);                 //#REG_TC_AF_ScndWinSizeX
     S5K5CAGX_write_cmos_sensor(0x0F12, height_ratio);                  //#REG_TC_AF_ScndWinSizeY
     S5K5CAGX_write_cmos_sensor(0x0F12, 0x0001);                    //#REG_TC_AF_WinSizesUpdated
      msleep(150);//need 1  fps time to update 
#endif      
}


static void S5K5CAGX_FOCUS_Cancel_Focus()
{
	SENSORDB("\n ****** S5K5CAGX_FOCUS_Cancel_Focus \n ");
    S5K5CAGX_write_cmos_sensor(0xFCFC, 0xD000);                                                        
    S5K5CAGX_write_cmos_sensor(0x0028, 0x7000);                                                                        
    S5K5CAGX_write_cmos_sensor(0x002A,0x0252);
    S5K5CAGX_write_cmos_sensor(0x0F12,0x0001);
}

static void S5K5CAGX_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 
	SENSORDB("\n ****** S5K5CAGX_FOCUS_Get_AF_Max_Num_Focus_Areas \n ");

    *pFeatureReturnPara32 = 1;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void S5K5CAGX_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
	SENSORDB("\n ****** S5K5CAGX_FOCUS_Get_AE_Max_Num_Metering_Areas \n ");
    *pFeatureReturnPara32 = 0;    
    SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	SENSORDB("[S5K5CAGX]exit S5K5CAGX_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}

static void S5K5CAGX_MIPI_AE_Set_Window2HW(
                        unsigned int s_x, 
                        unsigned int s_y, 
                        unsigned int w, 
                        unsigned int h)
{

}


static void S5K5CAGX_FOCUS_Set_AE_Window(
    unsigned int zone_addr,
    unsigned int prevW,
    unsigned int prevH)
{
}

static UINT32 S5K5CAGX_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
	return ERROR_NONE;
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//-------------------------------------------FEATURE   control-----------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
static void S5K5CAGX_InitialPara(void)
{
    SENSORDB("\n****** S5K5CAGX_InitialPara \n");
    spin_lock(&s5k5cagx_drv_lock);
    s5k5cagx_stru.NightMode = 0;
    s5k5cagx_stru.iShutter= 0;
    s5k5cagx_stru.iGain= 0;
    s5k5cagx_stru.manualAEStart=KAL_FALSE;
    s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_PREVIEW;
    ae_awb_state = S5K5CAGX_AEAWB_PRE;
    s5k5cagx_stru.sceneMode = SCENE_MODE_OFF;
    awbMode_5ca = AWB_MODE_AUTO;
    write_statue = S5K5CAGX_YUV_INIT;
    spin_unlock(&s5k5cagx_drv_lock);  
}

static void S5K5CAGXGetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
	//pSensorAEAWBRefStruct is UINT type, but actually a pointer
    PSENSOR_AE_AWB_REF_STRUCT pRef = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;

    SENSORDB("\n****** S5K5CAGXGetEvAwbRef \n");
//sandy check

    //SENSORDB("S5K5CAGXGetEvAwbRef pRef = 0x%x \n", pRef);
	pRef->SensorAERef.AeRefLV05Shutter = 0x8c9c;;
    pRef->SensorAERef.AeRefLV05Gain = 128* 8; /* 7.75x, 128 base */
    pRef->SensorAERef.AeRefLV13Shutter = 0x35f;
    pRef->SensorAERef.AeRefLV13Gain =  128 * 1;  /* 1x, 128 base */
    pRef->SensorAwbGainRef.AwbRefD65Rgain = 179; /* 1.46875x, 128 base */
    pRef->SensorAwbGainRef.AwbRefD65Bgain = 161; /* 1x, 128 base */
    pRef->SensorAwbGainRef.AwbRefCWFRgain = 155; /* 1.25x, 128 base */
    pRef->SensorAwbGainRef.AwbRefCWFBgain = 257; /* 1.28125x, 128 base */
}

static void S5K5CAGXGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
	//pSensorAEAWBCurStruct is UINT type, but actually a pointer
    PSENSOR_AE_AWB_CUR_STRUCT pInfo = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;

    SENSORDB("\n****** S5K5CAGXGetCurAeAwbInfo \n");


    //SENSORDB("S5K5CAGXGetCurAeAwbInfo pInfo = 0x%x \n", pInfo);
    pInfo->SensorAECur.AeCurShutter = S5K5CAGXReadShutter();
    pInfo->SensorAECur.AeCurGain = S5K5CAGXReadGain(); /* 128 base */

   pInfo->SensorAwbGainCur.AwbCurRgain=S5K5CAGXReadAwbRGain(); /* 128 base */
    pInfo->SensorAwbGainCur.AwbCurBgain = S5K5CAGXReadAwbBGain(); /* 128 base */
}

UINT32 S5K5CAGXGetSensorID(UINT32 *sensorID)
{
	int retry=3;
	do{
		S5K5CAGX_write_cmos_sensor(0xFCFC, 0x0000);
		*sensorID=S5K5CAGX_read_cmos_sensor(0x0040);
		SENSORDB("*sensorID=0x%04x",*sensorID);

      //  mt_set_gpio_out(GPIO87,0);
      //  mt_set_gpio_out(GPIO87,1);
      //  mt_set_gpio_out(GPIO88,0);

      //  *sensorID  = S5K5CAGX_SENSOR_ID;
		
		if(*sensorID == S5K5CAGX_SENSOR_ID)
        	break;
		retry--;
	}while(retry>0);


	if(*sensorID!=S5K5CAGX_SENSOR_ID)
	{
		*sensorID=0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	else
	{
		//SENSORDB("S5K5CAGX Read Sendor ID SUCCESS:0x%04x",*sensorID);
		return ERROR_NONE;
	}
}


void S5K5CAGX_GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
    SENSORDB("S5K5CAGX_GetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}

void S5K5CAGX_GetDelayInfo(UINT32 delayAddr)
{
    SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;

	SENSORDB("\n ****** S5K5CAGX_GetDelayInfo \n ");

    pDelayInfo->InitDelay = 5;
    pDelayInfo->EffectDelay = 3;
    pDelayInfo->AwbDelay = 3;
    pDelayInfo->AFSwitchDelayFrame = 50;
}

void S5K5CAGX_GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;

    SENSORDB("\n****** S5K5CAGX_GetExifInfo \n");

    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = AE_ISO_100;
    pExifInfo->AWBMode = AWB_MODE_AUTO;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = AE_ISO_100;
}	


static void S5K5CAGX_SetDummy(kal_bool mode,const kal_uint16 iDummyPixels, const kal_uint16 iDummyLines)
{

	
}   /*  A5141MIPI_SetDummy */


UINT32 S5K5CAGXPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("\n****** S5K5CAGXPreview \n");

	spin_lock(&s5k5cagx_drv_lock);
	s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_PREVIEW;
	write_statue = S5K5CAGX_YUV_NVRAM;
	ae_awb_state = S5K5CAGX_AEAWB_PRE;
	spin_unlock(&s5k5cagx_drv_lock);
	S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,1);
    S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,1);
    S5K5CAGX_PV_Mode(0);
    S5K5CAGX_night_mode(s5k5cagx_stru.NightMode);
	S5K5CAGX_set_mirror(sensor_config_data->SensorImageMirror);
	//image_window->GrabStartX = S5K5CAGX_X_START;
	image_window->GrabStartX = S5K5CAGX_X_START;
	image_window->GrabStartY = S5K5CAGX_Y_START;
	image_window->ExposureWindowWidth = S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight = S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
	
	memcpy(&S5K5CAGXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE; 
}	/* S5K5CAGXPreview() */

UINT32 S5K5CAGXZsdPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    SENSORDB("\n****** S5K5CAGXZsdPreview \n");
	spin_lock(&s5k5cagx_drv_lock);
	s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_ZSD;
	write_statue = S5K5CAGX_YUV_NVRAM;
	ae_awb_state = S5K5CAGX_AEAWB_PRE;
	spin_unlock(&s5k5cagx_drv_lock);
	S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,1);
    S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,1);
	S5K5CAGX_PV_Mode(1);
    S5K5CAGX_night_mode(s5k5cagx_stru.NightMode);
	S5K5CAGX_set_mirror(sensor_config_data->SensorImageMirror);

	image_window->GrabStartX = S5K5CAGX_CAP_X_START;
	image_window->GrabStartY = S5K5CAGX_CAP_Y_START;
    image_window->ExposureWindowWidth = S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
    image_window->ExposureWindowHeight = S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
	// copy sensor_config_data
	memcpy(&S5K5CAGXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  	return ERROR_NONE;
}	/* S5K5CAGXPreview() */

UINT32 S5K5CAGXCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 pv_integration_time = 0; 	// Uinit - us
	kal_uint32 cap_integration_time = 0;
	kal_uint16 PV_line_len = 0;
	kal_uint16 CAP_line_len = 0;

    SENSORDB("\n****** S5K5CAGXCapture \n");
	spin_lock(&s5k5cagx_drv_lock);
	if(s5k5cagx_stru.Camco_mode == S5K5CAGX_MODE_CAPTURE)
	{
		spin_unlock(&s5k5cagx_drv_lock);
       	return;
	}
	s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_CAPTURE;
	ae_awb_state = S5K5CAGX_AEAWB_PRE;
	spin_unlock(&s5k5cagx_drv_lock);
    S5K5CAGX_night_mode(s5k5cagx_stru.NightMode);
	image_window->GrabStartX = S5K5CAGX_CAP_X_START;
	image_window->GrabStartY = S5K5CAGX_CAP_Y_START;
	image_window->ExposureWindowWidth= S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
	image_window->ExposureWindowHeight = S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
	memcpy(&S5K5CAGXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	SENSORDB("exit");
	
    if(S5K5CAGXHdr_CheckHDRstatus())//capture & ZSD mode could get in
    {
        S5K5CAGXHdr_Part1();
    }

	return ERROR_NONE;
}	/* S5K5CAGXCapture() */

UINT32 S5K5CAGXVideoMode(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    SENSORDB("\n****** S5K5CAGXVideoMode1 \n");

	spin_lock(&s5k5cagx_drv_lock);
	s5k5cagx_stru.Camco_mode = S5K5CAGX_MODE_VIDEO;
	ae_awb_state = S5K5CAGX_AEAWB_PRE;
	spin_unlock(&s5k5cagx_drv_lock);
	S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT1,1);
    S5K5CAGX_set_AWB_Bit(S5K5CAGX_BIT2,1);
    S5K5CAGX_PV_Mode(0);
    S5K5CAGX_night_mode(s5k5cagx_stru.NightMode);
	S5K5CAGX_set_mirror(sensor_config_data->SensorImageMirror);
	
	image_window->GrabStartX = S5K5CAGX_X_START;
	image_window->GrabStartY = S5K5CAGX_Y_START;
	image_window->ExposureWindowWidth = S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight = S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
	
	memcpy(&S5K5CAGXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE; 
}


UINT32 S5K5CAGXControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    SENSORDB("\n****** S5K5CAGXControl \n");
	SENSORDB("ScenarioId=%d",ScenarioId);

   spin_lock(&s5k5cagx_drv_lock);
   s5k5cagx_currentScenarioId = ScenarioId;
   spin_unlock(&s5k5cagx_drv_lock);
	
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			S5K5CAGXPreview(pImageWindow, pSensorConfigData);
		    break;

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K5CAGXVideoMode(pImageWindow, pSensorConfigData);
			break;
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			S5K5CAGXCapture(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			S5K5CAGXZsdPreview(pImageWindow, pSensorConfigData);
		break;
		
        default:
            return ERROR_INVALID_SCENARIO_ID;	
	}
	return ERROR_NONE;
}


UINT32 S5K5CAGXGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("\n****** S5K5CAGXGetResolution \n");

	pSensorResolution->SensorFullWidth=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
	pSensorResolution->SensorFullHeight=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=S5K5CAGX_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight=S5K5CAGX_IMAGE_SENSOR_VIDEO_HEIGHT;
	return ERROR_NONE;
}	/* S5K5CAGXGetResolution() */

UINT32 S5K5CAGXGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    SENSORDB("\n****** S5K5CAGXGetInfo \n");
    switch(ScenarioId)
    {
    	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 pSensorInfo->SensorPreviewResolutionX=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=15;
		break;
		default:
			 pSensorInfo->SensorPreviewResolutionX=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
	         pSensorInfo->SensorPreviewResolutionY=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
			 pSensorInfo->SensorCameraPreviewFrameRate=30;
	}
	pSensorInfo->SensorFullResolutionX=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
	//pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;

//	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;//SENSOR_OUTPUT_FORMAT_YUYV;//SENSOR_OUTPUT_FORMAT_VYUY;

	
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	pSensorInfo->SensorInterruptDelayLines = 1;
	#ifdef MIPI_INTERFACE
   		pSensorInfo->SensroInterfaceType        = SENSOR_INTERFACE_TYPE_MIPI;
   	#else
   		pSensorInfo->SensroInterfaceType		= SENSOR_INTERFACE_TYPE_PARALLEL;
   	#endif
	pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   	
	pSensorInfo->CaptureDelayFrame = 3; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 
    pSensorInfo->YUVAwbDelayFrame = 4;    /* The frame of setting sensor gain */
    pSensorInfo->YUVEffectDelayFrame = 2;	
	   
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = S5K5CAGX_X_START; 
			pSensorInfo->SensorGrabStartY = S5K5CAGX_Y_START;  			
			pSensorInfo->SensorPreviewResolutionX=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorFullResolutionX=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorFullResolutionY=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
		#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
		#endif
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = S5K5CAGX_CAP_X_START; 
			pSensorInfo->SensorGrabStartY = S5K5CAGX_CAP_Y_START;  	
			pSensorInfo->SensorPreviewResolutionX=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorFullResolutionX=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorFullResolutionY=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
		#ifdef MIPI_INTERFACE
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
		#endif			
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = S5K5CAGX_X_START; 
			pSensorInfo->SensorGrabStartY = S5K5CAGX_Y_START;  			
			pSensorInfo->SensorPreviewResolutionX=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorFullResolutionX=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorFullResolutionY=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
		break;
	}
	memcpy(pSensorConfigData, &S5K5CAGXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* S5K5CAGXGetInfo() */

UINT32 S5K5CAGXSetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
    SENSORDB("\n****** S5K5CAGXSetMaxFramerateByScenario \n");
	
	return ERROR_NONE;
}

UINT32 S5K5CAGXGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
    SENSORDB("\n****** S5K5CAGXGetDefaultFramerateByScenario \n");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 150;
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

UINT32 S5K5CAGXOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

    SENSORDB("\n****** S5K5CAGXOpen \n");

	for(i=0;i<3;i++)
	{
	    S5K5CAGX_write_cmos_sensor(0xFCFC, 0x0000);
    	sensor_id=S5K5CAGX_read_cmos_sensor(0x0040);
    	SENSORDB("Sensor Read S5K5CAGX ID= 0x%04x\n",sensor_id);
    	if (sensor_id != S5K5CAGX_SENSOR_ID)
    	{
    	    SENSORDB("Sensor Read ByeBye\n");
    		return ERROR_SENSOR_CONNECT_FAIL;
    	}
    }
		
	//Initial sequence setting apply to cmos sensor.
	S5K5CAGX_InitialPara();
	S5K5CAGX_Initialize_Setting();
	return ERROR_NONE;
}	/* S5K5CAGXOpen() */

UINT32 S5K5CAGXClose(void)
{
	return ERROR_NONE;
}

UINT32 S5K5CAGXFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

   //SENSORDB("\n ******* S5K5CAGXFeatureControl\n");

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		     
		case SENSOR_FEATURE_GET_PERIOD:
			switch(s5k5cagx_currentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=S5K5CAGX_IMAGE_SENSOR_FULL_WIDTH;
					*pFeatureReturnPara16=S5K5CAGX_IMAGE_SENSOR_FULL_HEIGHT;
					*pFeatureParaLen=4;
					break;
					
				default:
        			*pFeatureReturnPara16++=S5K5CAGX_IMAGE_SENSOR_PV_WIDTH;
        			*pFeatureReturnPara16=S5K5CAGX_IMAGE_SENSOR_PV_HEIGHT;
        			*pFeatureParaLen=4;
		        break;
		     }
		     break;

		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(s5k5cagx_currentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = S5K5CAGX_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
					
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = S5K5CAGX_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
					
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = S5K5CAGX_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
					
				default:
					*pFeatureReturnPara32 = S5K5CAGX_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;
		
		case SENSOR_FEATURE_SET_ESHUTTER:
		    break;

		     
		case SENSOR_FEATURE_GET_EXIF_INFO:
            S5K5CAGX_GetExifInfo(*pFeatureData32);
            break;
		
		case SENSOR_FEATURE_SET_NIGHTMODE:
			S5K5CAGX_night_mode((kal_bool) *pFeatureData16);
		    break;
		
		case SENSOR_FEATURE_SET_GAIN:
		    break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		    break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
     		break;
     		
		case SENSOR_FEATURE_SET_REGISTER:
			S5K5CAGX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		    break;
		    
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = S5K5CAGX_read_cmos_sensor(pSensorRegData->RegAddr);
		    break;
		
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &S5K5CAGXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
             S5K5CAGX_SetTestPatternMode((BOOL)*pFeatureData16);
             break;

		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=S5K5CA_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;

		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			S5K5CAGXGetSensorID(pFeatureData32);
		    break;
             
		case SENSOR_FEATURE_SET_YUV_CMD:
			S5K5CAGXYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		    break;
		
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			S5K5CAGXYUVSetVideoMode(*pFeatureData16);
		    break; 
		
		case SENSOR_FEATURE_GET_EV_AWB_REF:  //*pFeatureData32 is UINT type, but actually a pointer on this case
			S5K5CAGXGetEvAwbRef(*pFeatureData32);
		    break;

		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            S5K5CAGX_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;			
		
  		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN: //*pFeatureData32 is UINT type, but actually a pointer on this case
			S5K5CAGXGetCurAeAwbInfo(*pFeatureData32);
		    break;

		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			S5K5CAGX_GetAEAWBLock(*pFeatureData32, *(pFeatureData32+1));
			break;
			
		case SENSOR_FEATURE_GET_DELAY_INFO:
			SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			S5K5CAGX_GetDelayInfo(*pFeatureData32);
			break;
			
		case SENSOR_FEATURE_AUTOTEST_CMD:
			SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			S5K5CAGX_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;	

        //below is AF control
#if 0
        case SENSOR_FEATURE_INITIALIZE_AF:
             //
             S5K5CAGX_MIPI_AF_Init();
             break;

        case SENSOR_FEATURE_GET_AF_STATUS:
            S5K5CAGX_FOCUS_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;

		case SENSOR_FEATURE_CONSTANT_AF:
			S5K5CAGX_FOCUS_Constant_Focus();
			 break;

        case SENSOR_FEATURE_SET_AF_WINDOW:       
			S5K5CAGX_FOCUS_Set_AF_Window(*pFeatureData32);
            break;

        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			S5K5CAGX_FOCUS_Single_Focus();
            break;

        case SENSOR_FEATURE_CANCEL_AF:
            S5K5CAGX_FOCUS_Cancel_Focus();
            break;		

        case SENSOR_FEATURE_SET_AE_WINDOW:
            SENSORDB("AE zone addr = 0x%x\n",*pFeatureData32);			
            //S5K5CAGX_FOCUS_Set_AE_Window(*pFeatureData32, S5K5CAGX_Sensor_Driver.Preview_Pixels_In_Line, S5K5CAGX_Sensor_Driver.Preview_Lines_In_Frame);
            break; 

         case SENSOR_FEATURE_GET_AF_MACRO:
            S5K5CAGX_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;           

        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            S5K5CAGX_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;  

        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            S5K5CAGX_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break; 
            
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            // S5K5CAGX_FOCUS_Move_to(*pFeatureData16);
            break;            
 
        case SENSOR_FEATURE_GET_AF_INF:
            S5K5CAGX_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break; 
#endif            

		default:
		   break;			
	}
	return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncS5K5CAGX=
{
	S5K5CAGXOpen,
	S5K5CAGXGetInfo,
	S5K5CAGXGetResolution,
	S5K5CAGXFeatureControl,
	S5K5CAGXControl,
	S5K5CAGXClose
};

UINT32 S5K5CAGX_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncS5K5CAGX;
	return ERROR_NONE;
}	/* SensorInit() */

