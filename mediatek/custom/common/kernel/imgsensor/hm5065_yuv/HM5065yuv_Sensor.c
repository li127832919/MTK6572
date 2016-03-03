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
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
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
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
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
#include <asm/io.h>
#include <asm/system.h>	 
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "HM5065yuv_Sensor.h"
#include "HM5065yuv_Camera_Sensor_para.h"
#include "HM5065yuv_CameraCustomized.h" 
//#define HM5065YUV_DEBUG
#ifdef HM5065YUV_DEBUG
#define HM5065SENSORDB printk
#else
#define HM5065SENSORDB(x,...)
#endif
#define AE_and_AF_FACE
static DEFINE_SPINLOCK(HM5065_drv_lock);
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static kal_uint8 af_pos_h = 0;
static kal_uint8 af_pos_l = 0;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define HM5065_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,HM5065_WRITE_ID)
#define mDELAY(ms)  mdelay(ms)
typedef enum
{
    PRV_W=2592, //1280,
    PRV_H=1944, //960,
		
	ZSD_PRV_W=2592,
	ZSD_PRV_H=1944
}PREVIEW_VIEW_SIZE;
kal_uint16 HM5065_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HM5065_WRITE_ID);
    return get_byte;
}

static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool	  VideoMode;
	kal_uint16  Fps;
	kal_uint16  ShutterStep;
	kal_uint8   IsPVmode;
	kal_uint32  PreviewDummyPixels;
	kal_uint32  PreviewDummyLines;
	kal_uint32  CaptureDummyPixels;
	kal_uint32  CaptureDummyLines;
	kal_uint32  PreviewPclk;
	kal_uint32  CapturePclk;
	kal_uint32  ZsdturePclk;
	kal_uint32  PreviewShutter;
	kal_uint32  PreviewExtraShutter;
	kal_uint32  SensorGain;
	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;
	kal_uint32      currentExposureTime;
    kal_uint32      currentShutter;
	kal_uint32      currentextshutter;
    kal_uint32      currentAxDGain;
	kal_uint32  	sceneMode;
    unsigned char isoSpeed;
	unsigned char zsd_flag;
	HM5065_SENSOR_MODE SensorMode;
	UINT16 wb;
} HM5065Sensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0; 
static kal_int8 HM5065_DELAY_AFTER_PREVIEW = -1;
static kal_uint8 HM5065_Banding_setting = AE_FLICKER_MODE_50HZ; 
static kal_bool HM5065_AWB_ENABLE = KAL_TRUE; 
static kal_bool HM5065_AE_ENABLE = KAL_TRUE; 
MSDK_SENSOR_CONFIG_STRUCT HM5065SensorConfigData;
#define HM5065_TEST_PATTERN_CHECKSUM (0x7ba87eae)
kal_bool HM5065_run_test_pattern=0;

typedef enum
{
    AE_SECTION_INDEX_BEGIN=0, 
    AE_SECTION_INDEX_1=AE_SECTION_INDEX_BEGIN, 
    AE_SECTION_INDEX_2, 
    AE_SECTION_INDEX_3, 
    AE_SECTION_INDEX_4, 
    AE_SECTION_INDEX_5, 
    AE_SECTION_INDEX_6, 
    AE_SECTION_INDEX_7, 
    AE_SECTION_INDEX_8, 
    AE_SECTION_INDEX_9, 
    AE_SECTION_INDEX_10, 
    AE_SECTION_INDEX_11, 
    AE_SECTION_INDEX_12, 
    AE_SECTION_INDEX_13, 
    AE_SECTION_INDEX_14, 
    AE_SECTION_INDEX_15, 
    AE_SECTION_INDEX_16,  
    AE_SECTION_INDEX_MAX
}AE_SECTION_INDEX;
typedef enum
{
    AE_VERTICAL_BLOCKS=4,
    AE_VERTICAL_BLOCKS_MAX,
    AE_HORIZONTAL_BLOCKS=4,
    AE_HORIZONTAL_BLOCKS_MAX
}AE_VERTICAL_HORIZONTAL_BLOCKS;
static UINT32 line_coordinate[AE_VERTICAL_BLOCKS_MAX] = {0};//line[0]=0      line[1]=160     line[2]=320     line[3]=480     line[4]=640
static UINT32 row_coordinate[AE_HORIZONTAL_BLOCKS_MAX] = {0};//line[0]=0       line[1]=120     line[2]=240     line[3]=360     line[4]=480
static BOOL AE_1_ARRAY[AE_SECTION_INDEX_MAX] = {FALSE};
static BOOL AE_2_ARRAY[AE_HORIZONTAL_BLOCKS][AE_VERTICAL_BLOCKS] = {{FALSE},{FALSE},{FALSE},{FALSE}};//how to ....
//=====================touch AE begin==========================//
void HM5065_writeAEReg(void)
{	
}


void HM5065_printAE_1_ARRAY(void)
{
    UINT32 i;
    for(i=0; i<AE_SECTION_INDEX_MAX; i++)
    {
        HM5065SENSORDB("AE_1_ARRAY[%2d]=%d\n", i, AE_1_ARRAY[i]);
    }
}

void HM5065_printAE_2_ARRAY(void)
{
    UINT32 i, j;
    HM5065SENSORDB("\t\t");
    for(i=0; i<AE_VERTICAL_BLOCKS; i++)
    {
        HM5065SENSORDB("      line[%2d]", i);
    }
    printk("\n");
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        HM5065SENSORDB("\trow[%2d]", j);
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {
            //SENSORDB("AE_2_ARRAY[%2d][%2d]=%d\n", j,i,AE_2_ARRAY[j][i]);
            HM5065SENSORDB("  %7d", AE_2_ARRAY[j][i]);
        }
        HM5065SENSORDB("\n");
    }
}

void HM5065_clearAE_2_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        {AE_2_ARRAY[j][i]=FALSE;}
    }
}

void HM5065_mapAE_2_ARRAY_To_AE_1_ARRAY(void)
{
    UINT32 i, j;
    for(j=0; j<AE_HORIZONTAL_BLOCKS; j++)
    {
        for(i=0; i<AE_VERTICAL_BLOCKS; i++)
        { AE_1_ARRAY[j*AE_VERTICAL_BLOCKS+i] = AE_2_ARRAY[j][i];}
    }
}

void HM5065_mapMiddlewaresizePointToPreviewsizePoint(
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
    HM5065SENSORDB("mapping middlware x[%d],y[%d], [%d X %d]\n\t\tto x[%d],y[%d],[%d X %d]\n ",
        mx, my, mw, mh, *pvx, *pvy, pvw, pvh);
}


void HM5065_calcLine(void)
{//line[5]
    UINT32 i;
    UINT32 step = PRV_W / AE_VERTICAL_BLOCKS;
    for(i=0; i<=AE_VERTICAL_BLOCKS; i++)
    {
        *(&line_coordinate[0]+i) = step*i;
        HM5065SENSORDB("line[%d]=%d\t",i, *(&line_coordinate[0]+i));
    }
    HM5065SENSORDB("\n");
}

void HM5065_vcalcRow(void)
{//row[5]
    UINT32 i;
    UINT32 step = PRV_H / AE_HORIZONTAL_BLOCKS;
    for(i=0; i<=AE_HORIZONTAL_BLOCKS; i++)
    {
        *(&row_coordinate[0]+i) = step*i;
        HM5065SENSORDB("row[%d]=%d\t",i,*(&row_coordinate[0]+i));
    }
    HM5065SENSORDB("\n");
}

void HM5065_calcPointsAELineRowCoordinate(UINT32 x, UINT32 y, UINT32 * linenum, UINT32 * rownum)
{
    UINT32 i;
    i = 1;
    while(i<=AE_VERTICAL_BLOCKS)
    {
        if(x<line_coordinate[i])
        {
            *linenum = i;
            break;
        }
        *linenum = i++;
    }
    
    i = 1;
    while(i<=AE_HORIZONTAL_BLOCKS)
    {
        if(y<row_coordinate[i])
        {
            *rownum = i;
            break;
        }
        *rownum = i++;
    }
    HM5065SENSORDB("PV point [%d, %d] to section line coordinate[%d] row[%d]\n",x,y,*linenum,*rownum);
}



MINT32 HM5065_clampSection(UINT32 x, UINT32 min, UINT32 max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void HM5065_mapCoordinate(UINT32 linenum, UINT32 rownum, UINT32 * sectionlinenum, UINT32 * sectionrownum)
{
    *sectionlinenum = HM5065_clampSection(linenum-1,0,AE_VERTICAL_BLOCKS-1);
    *sectionrownum = HM5065_clampSection(rownum-1,0,AE_HORIZONTAL_BLOCKS-1);	
    HM5065SENSORDB("mapCoordinate from[%d][%d] to[%d][%d]\n",
		linenum, rownum,*sectionlinenum,*sectionrownum);
}

void HM5065_mapRectToAE_2_ARRAY(UINT32 x0, UINT32 y0, UINT32 x1, UINT32 y1)
{
    UINT32 i, j;
    HM5065SENSORDB("([%d][%d]),([%d][%d])\n", x0,y0,x1,y1);
    HM5065_clearAE_2_ARRAY();
    x0=HM5065_clampSection(x0,0,AE_VERTICAL_BLOCKS-1);
    y0=HM5065_clampSection(y0,0,AE_HORIZONTAL_BLOCKS-1);
    x1=HM5065_clampSection(x1,0,AE_VERTICAL_BLOCKS-1);
    y1=HM5065_clampSection(y1,0,AE_HORIZONTAL_BLOCKS-1);

    for(j=y0; j<=y1; j++)
    {
        for(i=x0; i<=x1; i++)
        {
            AE_2_ARRAY[j][i]=TRUE;
        }
    }
}

void HM5065_resetPVAE_2_ARRAY(void)
{
    HM5065_mapRectToAE_2_ARRAY(1,1,2,2);
}

//update ae window
//@input zone[] addr
void HM5065_FOCUS_Set_AE_Window(UINT32 zone_addr)
{//update global zone
  #if 1
  	HM5065SENSORDB("[HM5065]enter HM5065_FOCUS_Set_AE_Window function:\n ");
	  UINT32 FD_XS;
	  UINT32 FD_YS;   
	  UINT32 x0, y0, x1, y1;
	  UINT32 pvx0, pvy0, pvx1, pvy1;
	  UINT32 linenum, rownum;
	  UINT32 AF_pvx, AF_pvy;
	  UINT32* zone = (UINT32*)zone_addr;
	  x0 = *zone;
	  y0 = *(zone + 1);
	  x1 = *(zone + 2);
	  y1 = *(zone + 3);   
	  FD_XS = *(zone + 4);
	  FD_YS = *(zone + 5);
  //AE 0x714
	  HM5065SENSORDB("Set_AE_Window x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
	  x0, y0, x1, y1, FD_XS, FD_YS);  
  
	  if(CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD )
	  {
		  HM5065_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, ZSD_PRV_W, ZSD_PRV_H);
		  HM5065_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, ZSD_PRV_W, ZSD_PRV_H);  
	  }  
	  else
	  {
		  HM5065_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
		  HM5065_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);  
	  }
		  
	  HM5065SENSORDB("[HM5065]AE pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	  HM5065SENSORDB("[HM5065]AE pvx1=%d,pvy1=%d\n",pvx1, pvy1);
  //ndef AE_and_AF_FACE
	  if((pvx0==pvx1)&&(pvy0==pvy1))
	  {
	   if(CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD )
	   	{
	   	HM5065_write_cmos_sensor(0x0120,0x01);
		  HM5065_write_cmos_sensor(0x0121,0x04); 
		  HM5065_write_cmos_sensor(0x0122,0x00);
		  HM5065_write_cmos_sensor(0x0123,0xC4);  
		  HM5065_write_cmos_sensor(0x0124,0x02);
		  HM5065_write_cmos_sensor(0x0125,0x09);
		  HM5065_write_cmos_sensor(0x0126,0x01); 
		  HM5065_write_cmos_sensor(0x0127,0x88);
		  
		  HM5065SENSORDB("HM5065_FOCUS_Set_ZSDAE_defalt_Window_to_IC\n");
	   	 }
	   else
	   	{		  
		  HM5065_write_cmos_sensor(0x0120,0x00);
		  HM5065_write_cmos_sensor(0x0121,0x82); 
		  HM5065_write_cmos_sensor(0x0122,0x00);
		  HM5065_write_cmos_sensor(0x0123,0x62);  
		  HM5065_write_cmos_sensor(0x0124,0x01);
		  HM5065_write_cmos_sensor(0x0125,0x04);
		  HM5065_write_cmos_sensor(0x0126,0x00); 		
		  HM5065_write_cmos_sensor(0x0127,0xC4);
		  
		  HM5065SENSORDB("HM5065_FOCUS_Set_AE_defalt_Window_to_IC\n");	
	  	}
	  }
	  else
	  {
		  #if 1
		  HM5065_write_cmos_sensor(0x0120,0x00);
		  HM5065_write_cmos_sensor(0x0121,0x82); 
		  HM5065_write_cmos_sensor(0x0122,0x00);
		  HM5065_write_cmos_sensor(0x0123,0x62);  
		  HM5065_write_cmos_sensor(0x0124,0x01);
		  HM5065_write_cmos_sensor(0x0125,0x04);
		  HM5065_write_cmos_sensor(0x0126,0x00); 		
		  HM5065_write_cmos_sensor(0x0127,0xC4);		  
		  #else
		  HM5065_write_cmos_sensor(0x0120,pvx0>>8);
		  HM5065_write_cmos_sensor(0x0121,pvx0&0xff); 
		  HM5065_write_cmos_sensor(0x0122,pvy0>>8);
		  HM5065_write_cmos_sensor(0x0123,pvy0&0xff);  
		  HM5065_write_cmos_sensor(0x0124,((pvx1-pvx0)/4)>>8);
		  HM5065_write_cmos_sensor(0x0125,((pvx1-pvx0)/4)&0xff);
		  HM5065_write_cmos_sensor(0x0126,((pvy1-pvy0)/4)>>8);		
		  HM5065_write_cmos_sensor(0x0127,((pvy1-pvy0)/4)&0xff);
  		#endif
	 	  HM5065SENSORDB("HM5065_FOCUS_Set_AE_Touch Window_to_IC for center area\n");
   
	 	  HM5065SENSORDB("[HM5065]exit HM5065_FOCUS_Set_AE_Window function:\n ");
	  }  
#endif
}
//=====================touch AE end==========================//
/*************************************************************************
* FUNCTION
*	HM5065_set_dummy
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) & dummy lines(Vertical Blanking), it can be
*	used to adjust the frame rate or gain more time for back-end process.
*	
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : Dummy Lines (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void HM5065initalvariable()
{
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.VideoMode = KAL_FALSE;
	HM5065Sensor.NightMode = KAL_FALSE;
	HM5065Sensor.Fps = 100;
	HM5065Sensor.ShutterStep= 0xde;
	HM5065Sensor.CaptureDummyPixels = 0;
	HM5065Sensor.CaptureDummyLines = 0;
	HM5065Sensor.PreviewDummyPixels = 0;
	HM5065Sensor.PreviewDummyLines = 0;
	HM5065Sensor.SensorMode= SENSOR_MODE_INIT;
	HM5065Sensor.IsPVmode= KAL_TRUE;	
	HM5065Sensor.PreviewPclk= 560;
	HM5065Sensor.CapturePclk= 800;
	HM5065Sensor.ZsdturePclk= 800;
	HM5065Sensor.PreviewShutter=0x0375; //0375
	HM5065Sensor.PreviewExtraShutter=0x00; 
	HM5065Sensor.SensorGain=0x10;
	HM5065Sensor.manualAEStart=0;
	HM5065Sensor.isoSpeed=AE_ISO_100;
	HM5065Sensor.userAskAeLock=KAL_FALSE;
  HM5065Sensor.userAskAwbLock=KAL_FALSE;
	HM5065Sensor.currentExposureTime=0;
  HM5065Sensor.currentShutter=0;
	HM5065Sensor.zsd_flag=0;
	HM5065Sensor.currentextshutter=0;
	spin_unlock(&HM5065_drv_lock);
}
void HM5065GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 20;
    pExifInfo->AEISOSpeed = HM5065Sensor.isoSpeed;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = HM5065Sensor.isoSpeed;
}
static void HM5065SetDummy(kal_uint32 dummy_pixels, kal_uint32 dummy_lines)
{
		HM5065SENSORDB("[HM5065]enter HM5065SetDummy function:\n ");
		if (HM5065Sensor.IsPVmode)  
        {
            dummy_pixels = dummy_pixels+HM5065_PV_PERIOD_PIXEL_NUMS; 
            //HM5065_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            //HM5065_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+HM5065_PV_PERIOD_LINE_NUMS; 
            //HM5065_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            //HM5065_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
        else
        {
            dummy_pixels = dummy_pixels+HM5065_FULL_PERIOD_PIXEL_NUMS; 
            //HM5065_write_cmos_sensor(0x380D,( dummy_pixels&0xFF));         
            //HM5065_write_cmos_sensor(0x380C,(( dummy_pixels&0xFF00)>>8)); 
      
            dummy_lines= dummy_lines+HM5065_FULL_PERIOD_LINE_NUMS; 
            //HM5065_write_cmos_sensor(0x380F,(dummy_lines&0xFF));       
            //HM5065_write_cmos_sensor(0x380E,((dummy_lines&0xFF00)>>8));  
        } 
		HM5065SENSORDB("[HM5065]exit HM5065SetDummy function:\n ");
}    /* HM5065_set_dummy */

/*************************************************************************
* FUNCTION
*	HM5065WriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void HM5065WriteShutter(kal_uint32 shutter)
{
	kal_uint32 extra_exposure_vts = 0;
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	
	//HM5065_write_cmos_sensor(0x17c, ((shutter<<8)&0xff00) );    // shutter h 8bit
	//HM5065_write_cmos_sensor(0x17d, ((shutter)&0xff) );    //  shutter l 8bit

	//SENSORDB("[HM5065]exit HM5065WriteShutter function:\n ");

	HM5065SENSORDB("[HM5065]exit HM5065WriteShutter function:\n ");
}    /* HM5065_write_shutter */

/*************************************************************************
* FUNCTION
*	HM5065ExpWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/

static void HM5065WriteExpShutter(kal_uint32 shutter)
{
#if 0
	shutter*=16;
	HM5065SENSORDB("[HM5065]enter HM5065WriteExpShutter function:\n ");
	//HM5065_write_cmos_sensor(0x3502, (shutter & 0x00FF));           //AEC[7:0]
	//HM5065_write_cmos_sensor(0x3501, ((shutter & 0x0FF00) >>8));  //AEC[15:8]
	//HM5065_write_cmos_sensor(0x3500, ((shutter & 0xFF0000) >> 16));	
	HM5065SENSORDB("[HM5065]exit HM5065WriteExpShutter function:\n ");
#endif
}    /* HM5065_write_shutter */

/*************************************************************************
* FUNCTION
*	HM5065ExtraWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void HM5065WriteExtraShutter(kal_uint32 shutter)
{
	HM5065SENSORDB("[HM5065]enter HM5065WriteExtraShutter function:\n ");
	//HM5065_write_cmos_sensor(0x350D, shutter & 0xFF);          // EXVTS[b7~b0]
	//HM5065_write_cmos_sensor(0x350C, (shutter & 0xFF00) >> 8); // EXVTS[b15~b8]
	HM5065SENSORDB("[HM5065]exit HM5065WriteExtraShutter function:\n ");
}    /* HM5065_write_shutter */

/*************************************************************************
* FUNCTION
*	HM5065WriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void HM5065WriteSensorGain(kal_uint32 gain)
{
	kal_uint16 temp_reg ;
	HM5065SENSORDB("[HM5065]enter HM5065WriteSensorGain function:\n ");
	/*if(gain > 1024)  ASSERT(0);
	temp_reg = 0;
	temp_reg=gain&0x0FF;	*/
	//HM5065_write_cmos_sensor(0x180, ((gain<<8)&0xff00) );    // shutter h 8bit
	//HM5065_write_cmos_sensor(0x181, ((gain)&0xff) );    //  shutter l 8bit

	HM5065SENSORDB("[HM5065]exit HM5065WriteSensorGain function:\n ");
}  /* HM5065_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	HM5065ReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 HM5065ReadShutter(void)
{
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	HM5065SENSORDB("[HM5065]enter HM5065ReadShutter function:\n ");
	temp_reg1 = HM5065_read_cmos_sensor(0x17c);    // shutter h 8bit
	temp_reg2 = HM5065_read_cmos_sensor(0x17d);    //  shutter l 8bit

	HM5065SENSORDB("[HM5065]exit HM5065ReadShutter function:\n ");	

	return ( ((temp_reg1&0xff)<<8) | (temp_reg2&0xff) );

} /* HM5065_read_shutter */

/*************************************************************************
* FUNCTION
*	HM5065ReadExtraShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
static kal_uint32 HM5065ReadExtraShutter(void)
{

} /* HM5065_read_shutter */
/*************************************************************************
* FUNCTION
*	HM5065ReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
static kal_uint32 HM5065ReadSensorGain(void)
{
	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	//kal_uint32 sensor_gain = 0;
	HM5065SENSORDB("[HM5065]enter HM5065ReadSensorGain function:\n ");
 
	temp_reg1 = HM5065_read_cmos_sensor(0x180);    // a gain h 8bit = 0
	temp_reg2 = HM5065_read_cmos_sensor(0x181);    //  a gain  l 8bit

	HM5065SENSORDB("[HM5065]exit HM5065ReadSensorGain function:\n ");
	return ( ((temp_reg1&0xff)<<8) | (temp_reg2&0xff) );
}  /* HM5065ReadSensorGain */
/*************************************************************************
* FUNCTION
*	HM5065_set_AE_mode
*
* DESCRIPTION
*	This function HM5065_set_AE_mode.
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
static void HM5065_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	HM5065SENSORDB("[HM5065]enter HM5065_set_AE_mode function: %d\n ", AE_enable);
    //
    if (AE_enable == KAL_TRUE)
	
		   HM5065_write_cmos_sensor(0x0142,0x00);//enable AE
		else	   
		   HM5065_write_cmos_sensor(0x0142,0x01);//disable AE
		
	HM5065SENSORDB("[HM5065]exit HM5065_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	HM5065_set_AWB_mode
*
* DESCRIPTION
*	This function HM5065_set_AWB_mode.
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
static void HM5065_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	HM5065SENSORDB("[HM5065]enter HM5065_set_AWB_mode function:\n ");
//	  AwbTemp = HM5065_read_cmos_sensor(0x3406);   


	//	if(AWB_enable)
		//	HM5065_write_cmos_sensor(0x01A4,0x00);//open awb
//	else
		//HM5065_write_cmos_sensor(0x01A4,0x04);//freeze
	HM5065SENSORDB("[HM5065]exit HM5065_set_AWB_mode function:\n ");
}


/*************************************************************************
* FUNCTION
*	HM5065_night_mode
*
* DESCRIPTION
*	This function night mode of HM5065.
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
void HM5065_night_mode(kal_bool enable)
{
	HM5065SENSORDB("[HM5065]enter HM5065_night_mode function: %d\n ", enable);
	//kal_uint16 night = HM5065_read_cmos_sensor(0x3A00); 
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.NightMode=enable;
	spin_unlock(&HM5065_drv_lock);

	//if (!HM5065Sensor.MODE_CAPTURE) 
	if((CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_PREVIEW) || (CurrentScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW) )
	{ 
	  if(enable)
	  { 
		 if(HM5065Sensor.VideoMode == KAL_TRUE)
		  {	
				  /* MJPEG or MPEG4 Apps */
			  	printk("HM5065 night mode ture MPEG4_encode--------------\r\n");				  
	
				  HM5065_write_cmos_sensor(0x00E8,0x00);//Static Framerate
				  HM5065_write_cmos_sensor(0x00C8,0x00);
				  HM5065_write_cmos_sensor(0x00C9,0x0F);//15fps
				  HM5065_write_cmos_sensor(0x00CA,0x01);
				  //HM5065_write_cmos_sensor(0x0330,0x00);
	
				  //HM5065_write_cmos_sensor(0x0082,0x74);//Brightness
//				  HM5065_write_cmos_sensor(0x015E,0x41);//Max Dgain 4100=3x ,4000=2x, 3E00=1x
//				  HM5065_write_cmos_sensor(0x015F,0x00);
		 }	  
		  else 
		 {
				printk("HM5065 night mode ture camera preview\r\n");
				  HM5065_write_cmos_sensor(0x00E8,0x01);//AFR
				  HM5065_write_cmos_sensor(0x00ED,0x08);//Min=8fps
				  HM5065_write_cmos_sensor(0x00EE,0x1E);//Max=30fps
				  //HM5065_write_cmos_sensor(0x0143,0x64);			  
//				  HM5065_write_cmos_sensor(0x015E,0x41);//Max Dgain 4100=3x ,4000=2x, 3E00=1x
//				  HM5065_write_cmos_sensor(0x015F,0x00);
		  } 	  
	  }
	  else
	  {
		  if(HM5065Sensor.VideoMode == KAL_TRUE)
		  {		 
				   /* MJPEG or MPEG4 Apps */
				  //HM5065_write_cmos_sensor(0x0030,0x14);	  //  Max.Derate=4
			  printk("HM5065 night_auto mode false MPEG4_encode--------------\r\n"); 			  
				  HM5065_write_cmos_sensor(0x00E8,0x00);//Static Framerate
				  HM5065_write_cmos_sensor(0x00C8,0x00);
				  HM5065_write_cmos_sensor(0x00C9,0x1E);//30fps
				  HM5065_write_cmos_sensor(0x00CA,0x01);
				  //HM5065_write_cmos_sensor(0x0330,0x01);
	
				  //HM5065_write_cmos_sensor(0x0082,0x64);//Brightness
//				  HM5065_write_cmos_sensor(0x015E,0x40);//Max Dgain 4100=3x ,4000=2x, 3E00=1x
//				  HM5065_write_cmos_sensor(0x015F,0x00);
		  }
		  else 
		  {   
			  printk("HM5065 night_auto mode false camera preview--------------\r\n");
				  HM5065_write_cmos_sensor(0x00E8,0x01);//AFR
				  HM5065_write_cmos_sensor(0x00ED,0x0a);//Min=12fps
				  HM5065_write_cmos_sensor(0x00EE,0x1E);//Max=30fps
				  //HM5065_write_cmos_sensor(0x0330,0x01);
	
				  //HM5065_write_cmos_sensor(0x0082,0x5A);//Brightness
				  //HM5065_write_cmos_sensor(0x0143,0x5F);			  
		  } 	 
	  }  
   }
	if((CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD) )
	{ 
		  if(enable)
		  	{ 
					printk("HM5065 night mode ture camera ZSD preview \r\n");
				  HM5065_write_cmos_sensor(0x00E8,0x01);//AFR
				  HM5065_write_cmos_sensor(0x00ED,0x05);//Min=5fps
				  HM5065_write_cmos_sensor(0x00EE,0x0f);//Max=30fps
				  //HM5065_write_cmos_sensor(0x0143,0x61);	//max int = 1/5s		  
				  //HM5065_write_cmos_sensor(0x0144,0x34);
		  	}
		  else
			  {
				  printk("HM5065 night_auto mode false camera ZSD preview--------------\r\n");
				  HM5065_write_cmos_sensor(0x00E8,0x01);//AFR
				  HM5065_write_cmos_sensor(0x00ED,0x08);//Min=12fps
				  HM5065_write_cmos_sensor(0x00EE,0x0f);//Max=30fps
				  //HM5065_write_cmos_sensor(0x0330,0x01);
				  //HM5065_write_cmos_sensor(0x0143,0x5F);	//max int 1/10s
				  //HM5065_write_cmos_sensor(0x0144,0x0D);
			  }  
	  }
	HM5065SENSORDB("[HM5065]exit HM5065_night_mode function:\n ");
}	/* HM5065_night_mode */
/*************************************************************************
* FUNCTION
*	HM5065_GetSensorID
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
//static 
kal_uint32 HM5065_GetSensorID(kal_uint32 *sensorID)
{
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_sccb_addr = 0;
	HM5065SENSORDB("[HM5065]enter HM5065_GetSensorID function:\n ");
	//HM5065_write_cmos_sensor(0x3008,0x82);// Reset sensor
	mDELAY(10);
	for(i=0;i<3;i++)
	{
		sensor_id = (HM5065_read_cmos_sensor(0x0000) << 8) | HM5065_read_cmos_sensor(0x0001);
		HM5065SENSORDB("HM5065 READ ID: %x",sensor_id);
		if(sensor_id != HM5065_SENSOR_ID)
		{	
			*sensorID =0xffffffff;
			return ERROR_SENSOR_CONNECT_FAIL;
		}
		else
			{
			*sensorID=HM5065_SENSOR_ID;
		        break;
			}
	}
	HM5065SENSORDB("[HM5065]exit HM5065_GetSensorID function:\n ");
   return ERROR_NONE;    

}   
UINT32 HM5065SetTestPatternMode(kal_bool bEnable)
{
	HM5065SENSORDB("[HM5065_HM5065SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
		////HM5065_write_cmos_sensor(0x503d,0x80);
		HM5065_run_test_pattern=1;
	}
	else
	{
		////HM5065_write_cmos_sensor(0x503d,0x00);
		HM5065_run_test_pattern=0;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    HM5065InitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static 
void HM5065InitialSetting(void)
{
	//;HM5065 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2 Lane
	HM5065SENSORDB("[HM5065]enter HM5065InitialSetting function:\n ");

		HM5065_write_cmos_sensor(0xffff,0x01);
		HM5065_write_cmos_sensor(0x9000,0x03);
		HM5065_write_cmos_sensor(0xA000,0x90);
		HM5065_write_cmos_sensor(0xA001,0x0C);
		HM5065_write_cmos_sensor(0xA002,0x56);
		HM5065_write_cmos_sensor(0xA003,0xE0);
		HM5065_write_cmos_sensor(0xA004,0xFE);
		HM5065_write_cmos_sensor(0xA005,0xA3);
		HM5065_write_cmos_sensor(0xA006,0xE0);
		HM5065_write_cmos_sensor(0xA007,0xFF);
		HM5065_write_cmos_sensor(0xA008,0x12);
		HM5065_write_cmos_sensor(0xA009,0x42);
		HM5065_write_cmos_sensor(0xA00A,0x85);
		HM5065_write_cmos_sensor(0xA00B,0x90);
		HM5065_write_cmos_sensor(0xA00C,0x01);
		HM5065_write_cmos_sensor(0xA00D,0xB7);
		HM5065_write_cmos_sensor(0xA00E,0xEE);
		HM5065_write_cmos_sensor(0xA00F,0xF0);
		HM5065_write_cmos_sensor(0xA010,0xFC);
		HM5065_write_cmos_sensor(0xA011,0xA3);
		HM5065_write_cmos_sensor(0xA012,0xEF);
		HM5065_write_cmos_sensor(0xA013,0xF0);
		HM5065_write_cmos_sensor(0xA014,0xFD);
		HM5065_write_cmos_sensor(0xA015,0x90);
		HM5065_write_cmos_sensor(0xA016,0x06);
		HM5065_write_cmos_sensor(0xA017,0x05);
		HM5065_write_cmos_sensor(0xA018,0xE0);
		HM5065_write_cmos_sensor(0xA019,0x75);
		HM5065_write_cmos_sensor(0xA01A,0xF0);
		HM5065_write_cmos_sensor(0xA01B,0x02);
		HM5065_write_cmos_sensor(0xA01C,0xA4);
		HM5065_write_cmos_sensor(0xA01D,0x2D);
		HM5065_write_cmos_sensor(0xA01E,0xFF);
		HM5065_write_cmos_sensor(0xA01F,0xE5);
		HM5065_write_cmos_sensor(0xA020,0xF0);
		HM5065_write_cmos_sensor(0xA021,0x3C);
		HM5065_write_cmos_sensor(0xA022,0xFE);
		HM5065_write_cmos_sensor(0xA023,0xAB);
		HM5065_write_cmos_sensor(0xA024,0x07);
		HM5065_write_cmos_sensor(0xA025,0xFA);
		HM5065_write_cmos_sensor(0xA026,0x33);
		HM5065_write_cmos_sensor(0xA027,0x95);
		HM5065_write_cmos_sensor(0xA028,0xE0);
		HM5065_write_cmos_sensor(0xA029,0xF9);
		HM5065_write_cmos_sensor(0xA02A,0xF8);
		HM5065_write_cmos_sensor(0xA02B,0x90);
		HM5065_write_cmos_sensor(0xA02C,0x0B);
		HM5065_write_cmos_sensor(0xA02D,0x4B);
		HM5065_write_cmos_sensor(0xA02E,0xE0);
		HM5065_write_cmos_sensor(0xA02F,0xFE);
		HM5065_write_cmos_sensor(0xA030,0xA3);
		HM5065_write_cmos_sensor(0xA031,0xE0);
		HM5065_write_cmos_sensor(0xA032,0xFF);
		HM5065_write_cmos_sensor(0xA033,0xEE);
		HM5065_write_cmos_sensor(0xA034,0x33);
		HM5065_write_cmos_sensor(0xA035,0x95);
		HM5065_write_cmos_sensor(0xA036,0xE0);
		HM5065_write_cmos_sensor(0xA037,0xFD);
		HM5065_write_cmos_sensor(0xA038,0xFC);
		HM5065_write_cmos_sensor(0xA039,0x12);
		HM5065_write_cmos_sensor(0xA03A,0x0C);
		HM5065_write_cmos_sensor(0xA03B,0x7B);
		HM5065_write_cmos_sensor(0xA03C,0x90);
		HM5065_write_cmos_sensor(0xA03D,0x01);
		HM5065_write_cmos_sensor(0xA03E,0xB9);
		HM5065_write_cmos_sensor(0xA03F,0x12);
		HM5065_write_cmos_sensor(0xA040,0x0E);
		HM5065_write_cmos_sensor(0xA041,0x05);
		HM5065_write_cmos_sensor(0xA042,0x90);
		HM5065_write_cmos_sensor(0xA043,0x01);
		HM5065_write_cmos_sensor(0xA044,0xB9);
		HM5065_write_cmos_sensor(0xA045,0xE0);
		HM5065_write_cmos_sensor(0xA046,0xFC);
		HM5065_write_cmos_sensor(0xA047,0xA3);
		HM5065_write_cmos_sensor(0xA048,0xE0);
		HM5065_write_cmos_sensor(0xA049,0xFD);
		HM5065_write_cmos_sensor(0xA04A,0xA3);
		HM5065_write_cmos_sensor(0xA04B,0xE0);
		HM5065_write_cmos_sensor(0xA04C,0xFE);
		HM5065_write_cmos_sensor(0xA04D,0xA3);
		HM5065_write_cmos_sensor(0xA04E,0xE0);
		HM5065_write_cmos_sensor(0xA04F,0xFF);
		HM5065_write_cmos_sensor(0xA050,0x78);
		HM5065_write_cmos_sensor(0xA051,0x08);
		HM5065_write_cmos_sensor(0xA052,0x12);
		HM5065_write_cmos_sensor(0xA053,0x0D);
		HM5065_write_cmos_sensor(0xA054,0xBF);
		HM5065_write_cmos_sensor(0xA055,0xA8);
		HM5065_write_cmos_sensor(0xA056,0x04);
		HM5065_write_cmos_sensor(0xA057,0xA9);
		HM5065_write_cmos_sensor(0xA058,0x05);
		HM5065_write_cmos_sensor(0xA059,0xAA);
		HM5065_write_cmos_sensor(0xA05A,0x06);
		HM5065_write_cmos_sensor(0xA05B,0xAB);
		HM5065_write_cmos_sensor(0xA05C,0x07);
		HM5065_write_cmos_sensor(0xA05D,0x90);
		HM5065_write_cmos_sensor(0xA05E,0x0B);
		HM5065_write_cmos_sensor(0xA05F,0x49);
		HM5065_write_cmos_sensor(0xA060,0xE0);
		HM5065_write_cmos_sensor(0xA061,0xFE);
		HM5065_write_cmos_sensor(0xA062,0xA3);
		HM5065_write_cmos_sensor(0xA063,0xE0);
		HM5065_write_cmos_sensor(0xA064,0xFF);
		HM5065_write_cmos_sensor(0xA065,0xEE);
		HM5065_write_cmos_sensor(0xA066,0x33);
		HM5065_write_cmos_sensor(0xA067,0x95);
		HM5065_write_cmos_sensor(0xA068,0xE0);
		HM5065_write_cmos_sensor(0xA069,0xFD);
		HM5065_write_cmos_sensor(0xA06A,0xFC);
		HM5065_write_cmos_sensor(0xA06B,0xC3);
		HM5065_write_cmos_sensor(0xA06C,0xEF);
		HM5065_write_cmos_sensor(0xA06D,0x9B);
		HM5065_write_cmos_sensor(0xA06E,0xFF);
		HM5065_write_cmos_sensor(0xA06F,0xEE);
		HM5065_write_cmos_sensor(0xA070,0x9A);
		HM5065_write_cmos_sensor(0xA071,0xFE);
		HM5065_write_cmos_sensor(0xA072,0xED);
		HM5065_write_cmos_sensor(0xA073,0x99);
		HM5065_write_cmos_sensor(0xA074,0xFD);
		HM5065_write_cmos_sensor(0xA075,0xEC);
		HM5065_write_cmos_sensor(0xA076,0x98);
		HM5065_write_cmos_sensor(0xA077,0xFC);
		HM5065_write_cmos_sensor(0xA078,0x78);
		HM5065_write_cmos_sensor(0xA079,0x01);
		HM5065_write_cmos_sensor(0xA07A,0x12);
		HM5065_write_cmos_sensor(0xA07B,0x0D);
		HM5065_write_cmos_sensor(0xA07C,0xBF);
		HM5065_write_cmos_sensor(0xA07D,0x90);
		HM5065_write_cmos_sensor(0xA07E,0x0C);
		HM5065_write_cmos_sensor(0xA07F,0x4A);
		HM5065_write_cmos_sensor(0xA080,0xE0);
		HM5065_write_cmos_sensor(0xA081,0xFC);
		HM5065_write_cmos_sensor(0xA082,0xA3);
		HM5065_write_cmos_sensor(0xA083,0xE0);
		HM5065_write_cmos_sensor(0xA084,0xF5);
		HM5065_write_cmos_sensor(0xA085,0x82);
		HM5065_write_cmos_sensor(0xA086,0x8C);
		HM5065_write_cmos_sensor(0xA087,0x83);
		HM5065_write_cmos_sensor(0xA088,0xC0);
		HM5065_write_cmos_sensor(0xA089,0x83);
		HM5065_write_cmos_sensor(0xA08A,0xC0);
		HM5065_write_cmos_sensor(0xA08B,0x82);
		HM5065_write_cmos_sensor(0xA08C,0x90);
		HM5065_write_cmos_sensor(0xA08D,0x0B);
		HM5065_write_cmos_sensor(0xA08E,0x48);
		HM5065_write_cmos_sensor(0xA08F,0xE0);
		HM5065_write_cmos_sensor(0xA090,0xD0);
		HM5065_write_cmos_sensor(0xA091,0x82);
		HM5065_write_cmos_sensor(0xA092,0xD0);
		HM5065_write_cmos_sensor(0xA093,0x83);
		HM5065_write_cmos_sensor(0xA094,0x75);
		HM5065_write_cmos_sensor(0xA095,0xF0);
		HM5065_write_cmos_sensor(0xA096,0x02);
		HM5065_write_cmos_sensor(0xA097,0x12);
		HM5065_write_cmos_sensor(0xA098,0x0E);
		HM5065_write_cmos_sensor(0xA099,0x45);
		HM5065_write_cmos_sensor(0xA09A,0xEE);
		HM5065_write_cmos_sensor(0xA09B,0xF0);
		HM5065_write_cmos_sensor(0xA09C,0xA3);
		HM5065_write_cmos_sensor(0xA09D,0xEF);
		HM5065_write_cmos_sensor(0xA09E,0xF0);
		HM5065_write_cmos_sensor(0xA09F,0x02);
		HM5065_write_cmos_sensor(0xA0A0,0xBA);
		HM5065_write_cmos_sensor(0xA0A1,0xD8);
		HM5065_write_cmos_sensor(0xA0A2,0x90);
		HM5065_write_cmos_sensor(0xA0A3,0x30);
		HM5065_write_cmos_sensor(0xA0A4,0x18);
		HM5065_write_cmos_sensor(0xA0A5,0xe4);
		HM5065_write_cmos_sensor(0xA0A6,0xf0);
		HM5065_write_cmos_sensor(0xA0A7,0x74);
		HM5065_write_cmos_sensor(0xA0A8,0x3f);
		HM5065_write_cmos_sensor(0xA0A9,0xf0);
		HM5065_write_cmos_sensor(0xA0AA,0x22);
		HM5065_write_cmos_sensor(0xA0BF,0x90);
		HM5065_write_cmos_sensor(0xA0C0,0x00);
		HM5065_write_cmos_sensor(0xA0C1,0x5E);
		HM5065_write_cmos_sensor(0xA0C2,0xE0);
		HM5065_write_cmos_sensor(0xA0C3,0xFF);
		HM5065_write_cmos_sensor(0xA0C4,0x70);
		HM5065_write_cmos_sensor(0xA0C5,0x20);
		HM5065_write_cmos_sensor(0xA0C6,0x90);
		HM5065_write_cmos_sensor(0xA0C7,0x47);
		HM5065_write_cmos_sensor(0xA0C8,0x04);
		HM5065_write_cmos_sensor(0xA0C9,0x74);
		HM5065_write_cmos_sensor(0xA0CA,0x0A);
		HM5065_write_cmos_sensor(0xA0CB,0xF0);
		HM5065_write_cmos_sensor(0xA0CC,0xA3);
		HM5065_write_cmos_sensor(0xA0CD,0x74);
		HM5065_write_cmos_sensor(0xA0CE,0x30);
		HM5065_write_cmos_sensor(0xA0CF,0xF0);
		HM5065_write_cmos_sensor(0xA0D0,0x90);
		HM5065_write_cmos_sensor(0xA0D1,0x47);
		HM5065_write_cmos_sensor(0xA0D2,0x0C);
		HM5065_write_cmos_sensor(0xA0D3,0x74);
		HM5065_write_cmos_sensor(0xA0D4,0x07);
		HM5065_write_cmos_sensor(0xA0D5,0xF0);
		HM5065_write_cmos_sensor(0xA0D6,0xA3);
		HM5065_write_cmos_sensor(0xA0D7,0x74);
		HM5065_write_cmos_sensor(0xA0D8,0xA8);
		HM5065_write_cmos_sensor(0xA0D9,0xF0);
		HM5065_write_cmos_sensor(0xA0DA,0x90);
		HM5065_write_cmos_sensor(0xA0DB,0x47);
		HM5065_write_cmos_sensor(0xA0DC,0xA4);
		HM5065_write_cmos_sensor(0xA0DD,0x74);
		HM5065_write_cmos_sensor(0xA0DE,0x01);
		HM5065_write_cmos_sensor(0xA0DF,0xF0);
		HM5065_write_cmos_sensor(0xA0E0,0x90);
		HM5065_write_cmos_sensor(0xA0E1,0x47);
		HM5065_write_cmos_sensor(0xA0E2,0xA8);
		HM5065_write_cmos_sensor(0xA0E3,0xF0);
		HM5065_write_cmos_sensor(0xA0E4,0x80);
		HM5065_write_cmos_sensor(0xA0E5,0x50);
		HM5065_write_cmos_sensor(0xA0E6,0xEF);
		HM5065_write_cmos_sensor(0xA0E7,0x64);
		HM5065_write_cmos_sensor(0xA0E8,0x01);
		HM5065_write_cmos_sensor(0xA0E9,0x60);
		HM5065_write_cmos_sensor(0xA0EA,0x04);
		HM5065_write_cmos_sensor(0xA0EB,0xEF);
		HM5065_write_cmos_sensor(0xA0EC,0xB4);
		HM5065_write_cmos_sensor(0xA0ED,0x03);
		HM5065_write_cmos_sensor(0xA0EE,0x20);
		HM5065_write_cmos_sensor(0xA0EF,0x90);
		HM5065_write_cmos_sensor(0xA0F0,0x47);
		HM5065_write_cmos_sensor(0xA0F1,0x04);
		HM5065_write_cmos_sensor(0xA0F2,0x74);
		HM5065_write_cmos_sensor(0xA0F3,0x05);
		HM5065_write_cmos_sensor(0xA0F4,0xF0);
		HM5065_write_cmos_sensor(0xA0F5,0xA3);
		HM5065_write_cmos_sensor(0xA0F6,0x74);
		HM5065_write_cmos_sensor(0xA0F7,0x18);
		HM5065_write_cmos_sensor(0xA0F8,0xF0);
		HM5065_write_cmos_sensor(0xA0F9,0x90);
		HM5065_write_cmos_sensor(0xA0FA,0x47);
		HM5065_write_cmos_sensor(0xA0FB,0x0C);
		HM5065_write_cmos_sensor(0xA0FC,0x74);
		HM5065_write_cmos_sensor(0xA0FD,0x03);
		HM5065_write_cmos_sensor(0xA0FE,0xF0);
		HM5065_write_cmos_sensor(0xA0FF,0xA3);
		HM5065_write_cmos_sensor(0xA100,0x74);
		HM5065_write_cmos_sensor(0xA101,0xD4);
		HM5065_write_cmos_sensor(0xA102,0xF0);
		HM5065_write_cmos_sensor(0xA103,0x90);
		HM5065_write_cmos_sensor(0xA104,0x47);
		HM5065_write_cmos_sensor(0xA105,0xA4);
		HM5065_write_cmos_sensor(0xA106,0x74);
		HM5065_write_cmos_sensor(0xA107,0x02);
		HM5065_write_cmos_sensor(0xA108,0xF0);
		HM5065_write_cmos_sensor(0xA109,0x90);
		HM5065_write_cmos_sensor(0xA10A,0x47);
		HM5065_write_cmos_sensor(0xA10B,0xA8);
		HM5065_write_cmos_sensor(0xA10C,0xF0);
		HM5065_write_cmos_sensor(0xA10D,0x80);
		HM5065_write_cmos_sensor(0xA10E,0x27);
		HM5065_write_cmos_sensor(0xA10F,0xEF);
		HM5065_write_cmos_sensor(0xA110,0x64);
		HM5065_write_cmos_sensor(0xA111,0x02);
		HM5065_write_cmos_sensor(0xA112,0x60);
		HM5065_write_cmos_sensor(0xA113,0x04);
		HM5065_write_cmos_sensor(0xA114,0xEF);
		HM5065_write_cmos_sensor(0xA115,0xB4);
		HM5065_write_cmos_sensor(0xA116,0x04);
		HM5065_write_cmos_sensor(0xA117,0x1E);
		HM5065_write_cmos_sensor(0xA118,0x90);
		HM5065_write_cmos_sensor(0xA119,0x47);
		HM5065_write_cmos_sensor(0xA11A,0x04);
		HM5065_write_cmos_sensor(0xA11B,0x74);
		HM5065_write_cmos_sensor(0xA11C,0x02);
		HM5065_write_cmos_sensor(0xA11D,0xF0);
		HM5065_write_cmos_sensor(0xA11E,0xA3);
		HM5065_write_cmos_sensor(0xA11F,0x74);
		HM5065_write_cmos_sensor(0xA120,0x8C);
		HM5065_write_cmos_sensor(0xA121,0xF0);
		HM5065_write_cmos_sensor(0xA122,0x90);
		HM5065_write_cmos_sensor(0xA123,0x47);
		HM5065_write_cmos_sensor(0xA124,0x0C);
		HM5065_write_cmos_sensor(0xA125,0x74);
		HM5065_write_cmos_sensor(0xA126,0x01);
		HM5065_write_cmos_sensor(0xA127,0xF0);
		HM5065_write_cmos_sensor(0xA128,0xA3);
		HM5065_write_cmos_sensor(0xA129,0x74);
		HM5065_write_cmos_sensor(0xA12A,0xEA);
		HM5065_write_cmos_sensor(0xA12B,0xF0);
		HM5065_write_cmos_sensor(0xA12C,0x90);
		HM5065_write_cmos_sensor(0xA12D,0x47);
		HM5065_write_cmos_sensor(0xA12E,0xA4);
		HM5065_write_cmos_sensor(0xA12F,0x74);
		HM5065_write_cmos_sensor(0xA130,0x04);
		HM5065_write_cmos_sensor(0xA131,0xF0);
		HM5065_write_cmos_sensor(0xA132,0x90);
		HM5065_write_cmos_sensor(0xA133,0x47);
		HM5065_write_cmos_sensor(0xA134,0xA8);
		HM5065_write_cmos_sensor(0xA135,0xF0);
		HM5065_write_cmos_sensor(0xA136,0x22);
		HM5065_write_cmos_sensor(0xA137,0x74);
		HM5065_write_cmos_sensor(0xA138,0x04);
		HM5065_write_cmos_sensor(0xA139,0xF0);
		HM5065_write_cmos_sensor(0xA13A,0xA3);
		HM5065_write_cmos_sensor(0xA13B,0x74);
		HM5065_write_cmos_sensor(0xA13C,0x20);
		HM5065_write_cmos_sensor(0xA13D,0xF0);
		HM5065_write_cmos_sensor(0xA13E,0xE4);
		HM5065_write_cmos_sensor(0xA13F,0xF5);
		HM5065_write_cmos_sensor(0xA140,0x22);
		HM5065_write_cmos_sensor(0xA141,0xE5);
		HM5065_write_cmos_sensor(0xA142,0x22);
		HM5065_write_cmos_sensor(0xA143,0xC3);
		HM5065_write_cmos_sensor(0xA144,0x94);
		HM5065_write_cmos_sensor(0xA145,0x40);
		HM5065_write_cmos_sensor(0xA146,0x40);
		HM5065_write_cmos_sensor(0xA147,0x03);
		HM5065_write_cmos_sensor(0xA148,0x02);
		HM5065_write_cmos_sensor(0xA149,0xF1);
		HM5065_write_cmos_sensor(0xA14A,0xFD);
		HM5065_write_cmos_sensor(0xA14B,0x90);
		HM5065_write_cmos_sensor(0xA14C,0x0A);
		HM5065_write_cmos_sensor(0xA14D,0xBA);
		HM5065_write_cmos_sensor(0xA14E,0xE0);
		HM5065_write_cmos_sensor(0xA14F,0xFE);
		HM5065_write_cmos_sensor(0xA150,0xA3);
		HM5065_write_cmos_sensor(0xA151,0xE0);
		HM5065_write_cmos_sensor(0xA152,0xFF);
		HM5065_write_cmos_sensor(0xA153,0xF5);
		HM5065_write_cmos_sensor(0xA154,0x82);
		HM5065_write_cmos_sensor(0xA155,0x8E);
		HM5065_write_cmos_sensor(0xA156,0x83);
		HM5065_write_cmos_sensor(0xA157,0xE0);
		HM5065_write_cmos_sensor(0xA158,0x54);
		HM5065_write_cmos_sensor(0xA159,0x70);
		HM5065_write_cmos_sensor(0xA15A,0xFD);
		HM5065_write_cmos_sensor(0xA15B,0xC4);
		HM5065_write_cmos_sensor(0xA15C,0x54);
		HM5065_write_cmos_sensor(0xA15D,0x0F);
		HM5065_write_cmos_sensor(0xA15E,0xFD);
		HM5065_write_cmos_sensor(0xA15F,0x90);
		HM5065_write_cmos_sensor(0xA160,0x0A);
		HM5065_write_cmos_sensor(0xA161,0xBC);
		HM5065_write_cmos_sensor(0xA162,0xE0);
		HM5065_write_cmos_sensor(0xA163,0xFA);
		HM5065_write_cmos_sensor(0xA164,0xA3);
		HM5065_write_cmos_sensor(0xA165,0xE0);
		HM5065_write_cmos_sensor(0xA166,0xF5);
		HM5065_write_cmos_sensor(0xA167,0x82);
		HM5065_write_cmos_sensor(0xA168,0x8A);
		HM5065_write_cmos_sensor(0xA169,0x83);
		HM5065_write_cmos_sensor(0xA16A,0xED);
		HM5065_write_cmos_sensor(0xA16B,0xF0);
		HM5065_write_cmos_sensor(0xA16C,0x90);
		HM5065_write_cmos_sensor(0xA16D,0x0A);
		HM5065_write_cmos_sensor(0xA16E,0xBD);
		HM5065_write_cmos_sensor(0xA16F,0xE0);
		HM5065_write_cmos_sensor(0xA170,0x04);
		HM5065_write_cmos_sensor(0xA171,0xF0);
		HM5065_write_cmos_sensor(0xA172,0x70);
		HM5065_write_cmos_sensor(0xA173,0x06);
		HM5065_write_cmos_sensor(0xA174,0x90);
		HM5065_write_cmos_sensor(0xA175,0x0A);
		HM5065_write_cmos_sensor(0xA176,0xBC);
		HM5065_write_cmos_sensor(0xA177,0xE0);
		HM5065_write_cmos_sensor(0xA178,0x04);
		HM5065_write_cmos_sensor(0xA179,0xF0);
		HM5065_write_cmos_sensor(0xA17A,0x8F);
		HM5065_write_cmos_sensor(0xA17B,0x82);
		HM5065_write_cmos_sensor(0xA17C,0x8E);
		HM5065_write_cmos_sensor(0xA17D,0x83);
		HM5065_write_cmos_sensor(0xA17E,0xA3);
		HM5065_write_cmos_sensor(0xA17F,0xE0);
		HM5065_write_cmos_sensor(0xA180,0xFF);
		HM5065_write_cmos_sensor(0xA181,0x90);
		HM5065_write_cmos_sensor(0xA182,0x0A);
		HM5065_write_cmos_sensor(0xA183,0xBC);
		HM5065_write_cmos_sensor(0xA184,0xE0);
		HM5065_write_cmos_sensor(0xA185,0xFC);
		HM5065_write_cmos_sensor(0xA186,0xA3);
		HM5065_write_cmos_sensor(0xA187,0xE0);
		HM5065_write_cmos_sensor(0xA188,0xF5);
		HM5065_write_cmos_sensor(0xA189,0x82);
		HM5065_write_cmos_sensor(0xA18A,0x8C);
		HM5065_write_cmos_sensor(0xA18B,0x83);
		HM5065_write_cmos_sensor(0xA18C,0xEF);
		HM5065_write_cmos_sensor(0xA18D,0xF0);
		HM5065_write_cmos_sensor(0xA18E,0x90);
		HM5065_write_cmos_sensor(0xA18F,0x0A);
		HM5065_write_cmos_sensor(0xA190,0xBD);
		HM5065_write_cmos_sensor(0xA191,0xE0);
		HM5065_write_cmos_sensor(0xA192,0x04);
		HM5065_write_cmos_sensor(0xA193,0xF0);
		HM5065_write_cmos_sensor(0xA194,0x70);
		HM5065_write_cmos_sensor(0xA195,0x06);
		HM5065_write_cmos_sensor(0xA196,0x90);
		HM5065_write_cmos_sensor(0xA197,0x0A);
		HM5065_write_cmos_sensor(0xA198,0xBC);
		HM5065_write_cmos_sensor(0xA199,0xE0);
		HM5065_write_cmos_sensor(0xA19A,0x04);
		HM5065_write_cmos_sensor(0xA19B,0xF0);
		HM5065_write_cmos_sensor(0xA19C,0x90);
		HM5065_write_cmos_sensor(0xA19D,0x0A);
		HM5065_write_cmos_sensor(0xA19E,0xBA);
		HM5065_write_cmos_sensor(0xA19F,0xE0);
		HM5065_write_cmos_sensor(0xA1A0,0xFE);
		HM5065_write_cmos_sensor(0xA1A1,0xA3);
		HM5065_write_cmos_sensor(0xA1A2,0xE0);
		HM5065_write_cmos_sensor(0xA1A3,0xFF);
		HM5065_write_cmos_sensor(0xA1A4,0xF5);
		HM5065_write_cmos_sensor(0xA1A5,0x82);
		HM5065_write_cmos_sensor(0xA1A6,0x8E);
		HM5065_write_cmos_sensor(0xA1A7,0x83);
		HM5065_write_cmos_sensor(0xA1A8,0xE0);
		HM5065_write_cmos_sensor(0xA1A9,0x54);
		HM5065_write_cmos_sensor(0xA1AA,0x07);
		HM5065_write_cmos_sensor(0xA1AB,0xFD);
		HM5065_write_cmos_sensor(0xA1AC,0x90);
		HM5065_write_cmos_sensor(0xA1AD,0x0A);
		HM5065_write_cmos_sensor(0xA1AE,0xBC);
		HM5065_write_cmos_sensor(0xA1AF,0xE0);
		HM5065_write_cmos_sensor(0xA1B0,0xFA);
		HM5065_write_cmos_sensor(0xA1B1,0xA3);
		HM5065_write_cmos_sensor(0xA1B2,0xE0);
		HM5065_write_cmos_sensor(0xA1B3,0xF5);
		HM5065_write_cmos_sensor(0xA1B4,0x82);
		HM5065_write_cmos_sensor(0xA1B5,0x8A);
		HM5065_write_cmos_sensor(0xA1B6,0x83);
		HM5065_write_cmos_sensor(0xA1B7,0xED);
		HM5065_write_cmos_sensor(0xA1B8,0xF0);
		HM5065_write_cmos_sensor(0xA1B9,0x90);
		HM5065_write_cmos_sensor(0xA1BA,0x0A);
		HM5065_write_cmos_sensor(0xA1BB,0xBD);
		HM5065_write_cmos_sensor(0xA1BC,0xE0);
		HM5065_write_cmos_sensor(0xA1BD,0x04);
		HM5065_write_cmos_sensor(0xA1BE,0xF0);
		HM5065_write_cmos_sensor(0xA1BF,0x70);
		HM5065_write_cmos_sensor(0xA1C0,0x06);
		HM5065_write_cmos_sensor(0xA1C1,0x90);
		HM5065_write_cmos_sensor(0xA1C2,0x0A);
		HM5065_write_cmos_sensor(0xA1C3,0xBC);
		HM5065_write_cmos_sensor(0xA1C4,0xE0);
		HM5065_write_cmos_sensor(0xA1C5,0x04);
		HM5065_write_cmos_sensor(0xA1C6,0xF0);
		HM5065_write_cmos_sensor(0xA1C7,0x8F);
		HM5065_write_cmos_sensor(0xA1C8,0x82);
		HM5065_write_cmos_sensor(0xA1C9,0x8E);
		HM5065_write_cmos_sensor(0xA1CA,0x83);
		HM5065_write_cmos_sensor(0xA1CB,0xA3);
		HM5065_write_cmos_sensor(0xA1CC,0xA3);
		HM5065_write_cmos_sensor(0xA1CD,0xE0);
		HM5065_write_cmos_sensor(0xA1CE,0xFF);
		HM5065_write_cmos_sensor(0xA1CF,0x90);
		HM5065_write_cmos_sensor(0xA1D0,0x0A);
		HM5065_write_cmos_sensor(0xA1D1,0xBC);
		HM5065_write_cmos_sensor(0xA1D2,0xE0);
		HM5065_write_cmos_sensor(0xA1D3,0xFC);
		HM5065_write_cmos_sensor(0xA1D4,0xA3);
		HM5065_write_cmos_sensor(0xA1D5,0xE0);
		HM5065_write_cmos_sensor(0xA1D6,0xF5);
		HM5065_write_cmos_sensor(0xA1D7,0x82);
		HM5065_write_cmos_sensor(0xA1D8,0x8C);
		HM5065_write_cmos_sensor(0xA1D9,0x83);
		HM5065_write_cmos_sensor(0xA1DA,0xEF);
		HM5065_write_cmos_sensor(0xA1DB,0xF0);
		HM5065_write_cmos_sensor(0xA1DC,0x90);
		HM5065_write_cmos_sensor(0xA1DD,0x0A);
		HM5065_write_cmos_sensor(0xA1DE,0xBD);
		HM5065_write_cmos_sensor(0xA1DF,0xE0);
		HM5065_write_cmos_sensor(0xA1E0,0x04);
		HM5065_write_cmos_sensor(0xA1E1,0xF0);
		HM5065_write_cmos_sensor(0xA1E2,0x70);
		HM5065_write_cmos_sensor(0xA1E3,0x06);
		HM5065_write_cmos_sensor(0xA1E4,0x90);
		HM5065_write_cmos_sensor(0xA1E5,0x0A);
		HM5065_write_cmos_sensor(0xA1E6,0xBC);
		HM5065_write_cmos_sensor(0xA1E7,0xE0);
		HM5065_write_cmos_sensor(0xA1E8,0x04);
		HM5065_write_cmos_sensor(0xA1E9,0xF0);
		HM5065_write_cmos_sensor(0xA1EA,0x90);
		HM5065_write_cmos_sensor(0xA1EB,0x0A);
		HM5065_write_cmos_sensor(0xA1EC,0xBB);
		HM5065_write_cmos_sensor(0xA1ED,0xE0);
		HM5065_write_cmos_sensor(0xA1EE,0x24);
		HM5065_write_cmos_sensor(0xA1EF,0x03);
		HM5065_write_cmos_sensor(0xA1F0,0xF0);
		HM5065_write_cmos_sensor(0xA1F1,0x90);
		HM5065_write_cmos_sensor(0xA1F2,0x0A);
		HM5065_write_cmos_sensor(0xA1F3,0xBA);
		HM5065_write_cmos_sensor(0xA1F4,0xE0);
		HM5065_write_cmos_sensor(0xA1F5,0x34);
		HM5065_write_cmos_sensor(0xA1F6,0x00);
		HM5065_write_cmos_sensor(0xA1F7,0xF0);
		HM5065_write_cmos_sensor(0xA1F8,0x05);
		HM5065_write_cmos_sensor(0xA1F9,0x22);
		HM5065_write_cmos_sensor(0xA1FA,0x02);
		HM5065_write_cmos_sensor(0xA1FB,0xF1);
		HM5065_write_cmos_sensor(0xA1FC,0x41);
		HM5065_write_cmos_sensor(0xA1FD,0x90);
		HM5065_write_cmos_sensor(0xA1FE,0x0A);
		HM5065_write_cmos_sensor(0xA1FF,0xBA);
		HM5065_write_cmos_sensor(0xA200,0x74);
		HM5065_write_cmos_sensor(0xA201,0x0E);
		HM5065_write_cmos_sensor(0xA202,0xF0);
		HM5065_write_cmos_sensor(0xA203,0xA3);
		HM5065_write_cmos_sensor(0xA204,0x74);
		HM5065_write_cmos_sensor(0xA205,0xDC);
		HM5065_write_cmos_sensor(0xA206,0xF0);
		HM5065_write_cmos_sensor(0xA207,0xA3);
		HM5065_write_cmos_sensor(0xA208,0x74);
		HM5065_write_cmos_sensor(0xA209,0x05);
		HM5065_write_cmos_sensor(0xA20A,0xF0);
		HM5065_write_cmos_sensor(0xA20B,0xA3);
		HM5065_write_cmos_sensor(0xA20C,0x74);
		HM5065_write_cmos_sensor(0xA20D,0x61);
		HM5065_write_cmos_sensor(0xA20E,0xF0);
		HM5065_write_cmos_sensor(0xA20F,0x90);
		HM5065_write_cmos_sensor(0xA210,0x0A);
		HM5065_write_cmos_sensor(0xA211,0xBA);
		HM5065_write_cmos_sensor(0xA212,0xE0);
		HM5065_write_cmos_sensor(0xA213,0xFE);
		HM5065_write_cmos_sensor(0xA214,0xA3);
		HM5065_write_cmos_sensor(0xA215,0xE0);
		HM5065_write_cmos_sensor(0xA216,0xAA);
		HM5065_write_cmos_sensor(0xA217,0x06);
		HM5065_write_cmos_sensor(0xA218,0xF9);
		HM5065_write_cmos_sensor(0xA219,0x7B);
		HM5065_write_cmos_sensor(0xA21A,0x01);
		HM5065_write_cmos_sensor(0xA21B,0xC0);
		HM5065_write_cmos_sensor(0xA21C,0x02);
		HM5065_write_cmos_sensor(0xA21D,0xA3);
		HM5065_write_cmos_sensor(0xA21E,0xE0);
		HM5065_write_cmos_sensor(0xA21F,0xFE);
		HM5065_write_cmos_sensor(0xA220,0xA3);
		HM5065_write_cmos_sensor(0xA221,0xE0);
		HM5065_write_cmos_sensor(0xA222,0xAA);
		HM5065_write_cmos_sensor(0xA223,0x06);
		HM5065_write_cmos_sensor(0xA224,0xF8);
		HM5065_write_cmos_sensor(0xA225,0xAC);
		HM5065_write_cmos_sensor(0xA226,0x02);
		HM5065_write_cmos_sensor(0xA227,0x7D);
		HM5065_write_cmos_sensor(0xA228,0x01);
		HM5065_write_cmos_sensor(0xA229,0xD0);
		HM5065_write_cmos_sensor(0xA22A,0x02);
		HM5065_write_cmos_sensor(0xA22B,0x7E);
		HM5065_write_cmos_sensor(0xA22C,0x00);
		HM5065_write_cmos_sensor(0xA22D,0x7F);
		HM5065_write_cmos_sensor(0xA22E,0x04);
		HM5065_write_cmos_sensor(0xA22F,0x12);
		HM5065_write_cmos_sensor(0xA230,0x0F);
		HM5065_write_cmos_sensor(0xA231,0x6F);
		HM5065_write_cmos_sensor(0xA232,0x02);
		HM5065_write_cmos_sensor(0xA233,0x66);
		HM5065_write_cmos_sensor(0xA234,0xD9);
		HM5065_write_cmos_sensor(0xA235,0x90);
		HM5065_write_cmos_sensor(0xA236,0x07);
		HM5065_write_cmos_sensor(0xA237,0xD0);
		HM5065_write_cmos_sensor(0xA238,0x02);
		HM5065_write_cmos_sensor(0xA239,0xA2);
		HM5065_write_cmos_sensor(0xA23A,0x69);
		HM5065_write_cmos_sensor(0xA240,0x02);
		HM5065_write_cmos_sensor(0xA241,0x21);
		HM5065_write_cmos_sensor(0xA242,0x7F);
		HM5065_write_cmos_sensor(0xA243,0x02);
		HM5065_write_cmos_sensor(0xA244,0x21);
		HM5065_write_cmos_sensor(0xA245,0xF4);
		HM5065_write_cmos_sensor(0xA246,0x02);
		HM5065_write_cmos_sensor(0xA247,0xA6);
		HM5065_write_cmos_sensor(0xA248,0x15);
		HM5065_write_cmos_sensor(0xA249,0x60);
		HM5065_write_cmos_sensor(0xA24A,0x0A);
		HM5065_write_cmos_sensor(0xA24B,0xEF);
		HM5065_write_cmos_sensor(0xA24C,0xB4);
		HM5065_write_cmos_sensor(0xA24D,0x01);
		HM5065_write_cmos_sensor(0xA24E,0x16);
		HM5065_write_cmos_sensor(0xA24F,0x90);
		HM5065_write_cmos_sensor(0xA250,0x00);
		HM5065_write_cmos_sensor(0xA251,0x5D);
		HM5065_write_cmos_sensor(0xA252,0xE0);
		HM5065_write_cmos_sensor(0xA253,0x70);
		HM5065_write_cmos_sensor(0xA254,0x10);
		HM5065_write_cmos_sensor(0xA255,0x12);
		HM5065_write_cmos_sensor(0xA256,0x26);
		HM5065_write_cmos_sensor(0xA257,0xC8);
		HM5065_write_cmos_sensor(0xA258,0x90);
		HM5065_write_cmos_sensor(0xA259,0x00);
		HM5065_write_cmos_sensor(0xA25A,0x11);
		HM5065_write_cmos_sensor(0xA25B,0x74);
		HM5065_write_cmos_sensor(0xA25C,0x30);
		HM5065_write_cmos_sensor(0xA25D,0xF0);
		HM5065_write_cmos_sensor(0xA25E,0x90);
		HM5065_write_cmos_sensor(0xA25F,0x00);
		HM5065_write_cmos_sensor(0xA260,0x10);
		HM5065_write_cmos_sensor(0xA261,0x74);
		HM5065_write_cmos_sensor(0xA262,0x01);
		HM5065_write_cmos_sensor(0xA263,0xF0);
		HM5065_write_cmos_sensor(0xA264,0x22);
		HM5065_write_cmos_sensor(0xA265,0x12);
		HM5065_write_cmos_sensor(0xA266,0x25);
		HM5065_write_cmos_sensor(0xA267,0xA8);
		HM5065_write_cmos_sensor(0xA268,0x02);
		HM5065_write_cmos_sensor(0xA269,0x29);
		HM5065_write_cmos_sensor(0xA26A,0xFC);
		HM5065_write_cmos_sensor(0xA26B,0x44);
		HM5065_write_cmos_sensor(0xA26C,0x18);
		HM5065_write_cmos_sensor(0xA26D,0xF0);
		HM5065_write_cmos_sensor(0xA26E,0x90);
		HM5065_write_cmos_sensor(0xA26F,0x72);
		HM5065_write_cmos_sensor(0xA270,0x18);
		HM5065_write_cmos_sensor(0xA271,0xE0);
		HM5065_write_cmos_sensor(0xA272,0x44);
		HM5065_write_cmos_sensor(0xA273,0x18);
		HM5065_write_cmos_sensor(0xA274,0xF0);
		HM5065_write_cmos_sensor(0xA275,0x00);
		HM5065_write_cmos_sensor(0xA276,0x00);
		HM5065_write_cmos_sensor(0xA277,0x00);
		HM5065_write_cmos_sensor(0xA278,0x00);
		HM5065_write_cmos_sensor(0xA279,0x00);
		HM5065_write_cmos_sensor(0xA27A,0x00);
		HM5065_write_cmos_sensor(0xA27B,0x90);
		HM5065_write_cmos_sensor(0xA27C,0x72);
		HM5065_write_cmos_sensor(0xA27D,0x08);
		HM5065_write_cmos_sensor(0xA27E,0xE0);
		HM5065_write_cmos_sensor(0xA27F,0x44);
		HM5065_write_cmos_sensor(0xA280,0x10);
		HM5065_write_cmos_sensor(0xA281,0xF0);
		HM5065_write_cmos_sensor(0xA282,0x90);
		HM5065_write_cmos_sensor(0xA283,0x72);
		HM5065_write_cmos_sensor(0xA284,0x14);
		HM5065_write_cmos_sensor(0xA285,0xE0);
		HM5065_write_cmos_sensor(0xA286,0x54);
		HM5065_write_cmos_sensor(0xA287,0xFD);
		HM5065_write_cmos_sensor(0xA288,0xF0);
		HM5065_write_cmos_sensor(0xA289,0x22);
		HM5065_write_cmos_sensor(0xA29B,0xF0);
		HM5065_write_cmos_sensor(0xA29C,0xD3);
		HM5065_write_cmos_sensor(0xA29D,0x90);
		HM5065_write_cmos_sensor(0xA29E,0x07);
		HM5065_write_cmos_sensor(0xA29F,0x91);
		HM5065_write_cmos_sensor(0xA2A0,0xE0);
		HM5065_write_cmos_sensor(0xA2A1,0x94);
		HM5065_write_cmos_sensor(0xA2A2,0x21);
		HM5065_write_cmos_sensor(0xA2A3,0x90);
		HM5065_write_cmos_sensor(0xA2A4,0x07);
		HM5065_write_cmos_sensor(0xA2A5,0x90);
		HM5065_write_cmos_sensor(0xA2A6,0xE0);
		HM5065_write_cmos_sensor(0xA2A7,0x64);
		HM5065_write_cmos_sensor(0xA2A8,0x80);
		HM5065_write_cmos_sensor(0xA2A9,0x94);
		HM5065_write_cmos_sensor(0xA2AA,0x81);
		HM5065_write_cmos_sensor(0xA2AB,0x40);
		HM5065_write_cmos_sensor(0xA2AC,0x08);
		HM5065_write_cmos_sensor(0xA2AD,0x90);
		HM5065_write_cmos_sensor(0xA2AE,0x07);
		HM5065_write_cmos_sensor(0xA2AF,0xCB);
		HM5065_write_cmos_sensor(0xA2B0,0x74);
		HM5065_write_cmos_sensor(0xA2B1,0xFF);
		HM5065_write_cmos_sensor(0xA2B2,0xF0);
		HM5065_write_cmos_sensor(0xA2B3,0x80);
		HM5065_write_cmos_sensor(0xA2B4,0x06);
		HM5065_write_cmos_sensor(0xA2B5,0x90);
		HM5065_write_cmos_sensor(0xA2B6,0x07);
		HM5065_write_cmos_sensor(0xA2B7,0xCB);
		HM5065_write_cmos_sensor(0xA2B8,0x74);
		HM5065_write_cmos_sensor(0xA2B9,0x01);
		HM5065_write_cmos_sensor(0xA2BA,0xF0);
		HM5065_write_cmos_sensor(0xA2BB,0x02);
		HM5065_write_cmos_sensor(0xA2BC,0xB5);
		HM5065_write_cmos_sensor(0xA2BD,0xC3);
		HM5065_write_cmos_sensor(0xA2BE,0x90);
		HM5065_write_cmos_sensor(0xA2BF,0x08);
		HM5065_write_cmos_sensor(0xA2C0,0x34);
		HM5065_write_cmos_sensor(0xA2C1,0xE0);
		HM5065_write_cmos_sensor(0xA2C2,0xFC);
		HM5065_write_cmos_sensor(0xA2C3,0xA3);
		HM5065_write_cmos_sensor(0xA2C4,0xE0);
		HM5065_write_cmos_sensor(0xA2C5,0xFD);
		HM5065_write_cmos_sensor(0xA2C6,0xA3);
		HM5065_write_cmos_sensor(0xA2C7,0xE0);
		HM5065_write_cmos_sensor(0xA2C8,0xFE);
		HM5065_write_cmos_sensor(0xA2C9,0xA3);
		HM5065_write_cmos_sensor(0xA2CA,0xE0);
		HM5065_write_cmos_sensor(0xA2CB,0xFF);
		HM5065_write_cmos_sensor(0xA2CC,0x90);
		HM5065_write_cmos_sensor(0xA2CD,0x07);
		HM5065_write_cmos_sensor(0xA2CE,0xD0);
		HM5065_write_cmos_sensor(0xA2CF,0xE0);
		HM5065_write_cmos_sensor(0xA2D0,0xF8);
		HM5065_write_cmos_sensor(0xA2D1,0xA3);
		HM5065_write_cmos_sensor(0xA2D2,0xE0);
		HM5065_write_cmos_sensor(0xA2D3,0xF9);
		HM5065_write_cmos_sensor(0xA2D4,0xA3);
		HM5065_write_cmos_sensor(0xA2D5,0xE0);
		HM5065_write_cmos_sensor(0xA2D6,0xFA);
		HM5065_write_cmos_sensor(0xA2D7,0xA3);
		HM5065_write_cmos_sensor(0xA2D8,0xE0);
		HM5065_write_cmos_sensor(0xA2D9,0xFB);
		HM5065_write_cmos_sensor(0xA2DA,0xD3);
		HM5065_write_cmos_sensor(0xA2DB,0x12);
		HM5065_write_cmos_sensor(0xA2DC,0x0D);
		HM5065_write_cmos_sensor(0xA2DD,0xAE);
		HM5065_write_cmos_sensor(0xA2DE,0x40);
		HM5065_write_cmos_sensor(0xA2DF,0x0B);
		HM5065_write_cmos_sensor(0xA2E0,0x12);
		HM5065_write_cmos_sensor(0xA2E1,0xB5);
		HM5065_write_cmos_sensor(0xA2E2,0x49);
		HM5065_write_cmos_sensor(0xA2E3,0x90);
		HM5065_write_cmos_sensor(0xA2E4,0x07);
		HM5065_write_cmos_sensor(0xA2E5,0xA4);
		HM5065_write_cmos_sensor(0xA2E6,0x74);
		HM5065_write_cmos_sensor(0xA2E7,0x02);
		HM5065_write_cmos_sensor(0xA2E8,0xF0);
		HM5065_write_cmos_sensor(0xA2E9,0x80);
		HM5065_write_cmos_sensor(0xA2EA,0x09);
		HM5065_write_cmos_sensor(0xA2EB,0x12);
		HM5065_write_cmos_sensor(0xA2EC,0xB7);
		HM5065_write_cmos_sensor(0xA2ED,0x51);
		HM5065_write_cmos_sensor(0xA2EE,0x90);
		HM5065_write_cmos_sensor(0xA2EF,0x07);
		HM5065_write_cmos_sensor(0xA2F0,0xA4);
		HM5065_write_cmos_sensor(0xA2F1,0x74);
		HM5065_write_cmos_sensor(0xA2F2,0x05);
		HM5065_write_cmos_sensor(0xA2F3,0xF0);
		HM5065_write_cmos_sensor(0xA2F4,0x02);
		HM5065_write_cmos_sensor(0xA2F5,0xA2);
		HM5065_write_cmos_sensor(0xA2F6,0xDA);
		HM5065_write_cmos_sensor(0xA2F7,0x90);
		HM5065_write_cmos_sensor(0xA2F8,0x0E);
		HM5065_write_cmos_sensor(0xA2F9,0xE0);
		HM5065_write_cmos_sensor(0xA2FA,0xE0);
		HM5065_write_cmos_sensor(0xA2FB,0xFD);
		HM5065_write_cmos_sensor(0xA2FC,0xA3);
		HM5065_write_cmos_sensor(0xA2FD,0xE0);
		HM5065_write_cmos_sensor(0xA2FE,0x90);
		HM5065_write_cmos_sensor(0xA2FF,0x02);
		HM5065_write_cmos_sensor(0xA300,0xA2);
		HM5065_write_cmos_sensor(0xA301,0xCD);
		HM5065_write_cmos_sensor(0xA302,0xF0);
		HM5065_write_cmos_sensor(0xA303,0xA3);
		HM5065_write_cmos_sensor(0xA304,0xED);
		HM5065_write_cmos_sensor(0xA305,0xF0);
		HM5065_write_cmos_sensor(0xA306,0x90);
		HM5065_write_cmos_sensor(0xA307,0x0E);
		HM5065_write_cmos_sensor(0xA308,0xE2);
		HM5065_write_cmos_sensor(0xA309,0xE0);
		HM5065_write_cmos_sensor(0xA30A,0xFD);
		HM5065_write_cmos_sensor(0xA30B,0xA3);
		HM5065_write_cmos_sensor(0xA30C,0xE0);
		HM5065_write_cmos_sensor(0xA30D,0x90);
		HM5065_write_cmos_sensor(0xA30E,0x02);
		HM5065_write_cmos_sensor(0xA30F,0xA8);
		HM5065_write_cmos_sensor(0xA310,0xCD);
		HM5065_write_cmos_sensor(0xA311,0xF0);
		HM5065_write_cmos_sensor(0xA312,0xA3);
		HM5065_write_cmos_sensor(0xA313,0xED);
		HM5065_write_cmos_sensor(0xA314,0xF0);
		HM5065_write_cmos_sensor(0xA315,0xE4);
		HM5065_write_cmos_sensor(0xA316,0x90);
		HM5065_write_cmos_sensor(0xA317,0x06);
		HM5065_write_cmos_sensor(0xA318,0x38);
		HM5065_write_cmos_sensor(0xA319,0xF0);
		HM5065_write_cmos_sensor(0xA31A,0x02);
		HM5065_write_cmos_sensor(0xA31B,0x67);
		HM5065_write_cmos_sensor(0xA31C,0x63);
		HM5065_write_cmos_sensor(0xA31D,0x90);
		HM5065_write_cmos_sensor(0xA31E,0x0E);
		HM5065_write_cmos_sensor(0xA31F,0xE8);
		HM5065_write_cmos_sensor(0xA320,0xE0);
		HM5065_write_cmos_sensor(0xA321,0x90);
		HM5065_write_cmos_sensor(0xA322,0x02);
		HM5065_write_cmos_sensor(0xA323,0x62);
		HM5065_write_cmos_sensor(0xA324,0xF0);
		HM5065_write_cmos_sensor(0xA325,0x90);
		HM5065_write_cmos_sensor(0xA326,0x0E);
		HM5065_write_cmos_sensor(0xA327,0xE9);
		HM5065_write_cmos_sensor(0xA328,0xE0);
		HM5065_write_cmos_sensor(0xA329,0x90);
		HM5065_write_cmos_sensor(0xA32A,0x02);
		HM5065_write_cmos_sensor(0xA32B,0x63);
		HM5065_write_cmos_sensor(0xA32C,0xF0);
		HM5065_write_cmos_sensor(0xA32D,0x02);
		HM5065_write_cmos_sensor(0xA32E,0x67);
		HM5065_write_cmos_sensor(0xA32F,0x1F);
		HM5065_write_cmos_sensor(0xA33B,0x90);
		HM5065_write_cmos_sensor(0xA33C,0x0E);
		HM5065_write_cmos_sensor(0xA33D,0x14);
		HM5065_write_cmos_sensor(0xA33E,0xE0);
		HM5065_write_cmos_sensor(0xA33F,0xFE);
		HM5065_write_cmos_sensor(0xA340,0xA3);
		HM5065_write_cmos_sensor(0xA341,0xE0);
		HM5065_write_cmos_sensor(0xA342,0xFF);
		HM5065_write_cmos_sensor(0xA343,0x90);
		HM5065_write_cmos_sensor(0xA344,0x06);
		HM5065_write_cmos_sensor(0xA345,0xD9);
		HM5065_write_cmos_sensor(0xA346,0xEE);
		HM5065_write_cmos_sensor(0xA347,0xF0);
		HM5065_write_cmos_sensor(0xA348,0xA3);
		HM5065_write_cmos_sensor(0xA349,0xEF);
		HM5065_write_cmos_sensor(0xA34A,0xF0);
		HM5065_write_cmos_sensor(0xA34B,0x90);
		HM5065_write_cmos_sensor(0xA34C,0x0E);
		HM5065_write_cmos_sensor(0xA34D,0x18);
		HM5065_write_cmos_sensor(0xA34E,0xE0);
		HM5065_write_cmos_sensor(0xA34F,0xFD);
		HM5065_write_cmos_sensor(0xA350,0x7C);
		HM5065_write_cmos_sensor(0xA351,0x00);
		HM5065_write_cmos_sensor(0xA352,0xC3);
		HM5065_write_cmos_sensor(0xA353,0xEF);
		HM5065_write_cmos_sensor(0xA354,0x9D);
		HM5065_write_cmos_sensor(0xA355,0xEE);
		HM5065_write_cmos_sensor(0xA356,0x9C);
		HM5065_write_cmos_sensor(0xA357,0x50);
		HM5065_write_cmos_sensor(0xA358,0x09);
		HM5065_write_cmos_sensor(0xA359,0xE4);
		HM5065_write_cmos_sensor(0xA35A,0x90);
		HM5065_write_cmos_sensor(0xA35B,0x06);
		HM5065_write_cmos_sensor(0xA35C,0xD7);
		HM5065_write_cmos_sensor(0xA35D,0xF0);
		HM5065_write_cmos_sensor(0xA35E,0xA3);
		HM5065_write_cmos_sensor(0xA35F,0xF0);
		HM5065_write_cmos_sensor(0xA360,0x80);
		HM5065_write_cmos_sensor(0xA361,0x13);
		HM5065_write_cmos_sensor(0xA362,0xC3);
		HM5065_write_cmos_sensor(0xA363,0x90);
		HM5065_write_cmos_sensor(0xA364,0x06);
		HM5065_write_cmos_sensor(0xA365,0xDA);
		HM5065_write_cmos_sensor(0xA366,0xE0);
		HM5065_write_cmos_sensor(0xA367,0x9D);
		HM5065_write_cmos_sensor(0xA368,0xFE);
		HM5065_write_cmos_sensor(0xA369,0x90);
		HM5065_write_cmos_sensor(0xA36A,0x06);
		HM5065_write_cmos_sensor(0xA36B,0xD9);
		HM5065_write_cmos_sensor(0xA36C,0xE0);
		HM5065_write_cmos_sensor(0xA36D,0x9C);
		HM5065_write_cmos_sensor(0xA36E,0x90);
		HM5065_write_cmos_sensor(0xA36F,0x06);
		HM5065_write_cmos_sensor(0xA370,0xD7);
		HM5065_write_cmos_sensor(0xA371,0xF0);
		HM5065_write_cmos_sensor(0xA372,0xA3);
		HM5065_write_cmos_sensor(0xA373,0xCE);
		HM5065_write_cmos_sensor(0xA374,0xF0);
		HM5065_write_cmos_sensor(0xA375,0x90);
		HM5065_write_cmos_sensor(0xA376,0x0E);
		HM5065_write_cmos_sensor(0xA377,0x18);
		HM5065_write_cmos_sensor(0xA378,0xE0);
		HM5065_write_cmos_sensor(0xA379,0xF9);
		HM5065_write_cmos_sensor(0xA37A,0xFF);
		HM5065_write_cmos_sensor(0xA37B,0x90);
		HM5065_write_cmos_sensor(0xA37C,0x06);
		HM5065_write_cmos_sensor(0xA37D,0xC2);
		HM5065_write_cmos_sensor(0xA37E,0xE0);
		HM5065_write_cmos_sensor(0xA37F,0xFC);
		HM5065_write_cmos_sensor(0xA380,0xA3);
		HM5065_write_cmos_sensor(0xA381,0xE0);
		HM5065_write_cmos_sensor(0xA382,0xFD);
		HM5065_write_cmos_sensor(0xA383,0xC3);
		HM5065_write_cmos_sensor(0xA384,0x9F);
		HM5065_write_cmos_sensor(0xA385,0xFF);
		HM5065_write_cmos_sensor(0xA386,0xEC);
		HM5065_write_cmos_sensor(0xA387,0x94);
		HM5065_write_cmos_sensor(0xA388,0x00);
		HM5065_write_cmos_sensor(0xA389,0xFE);
		HM5065_write_cmos_sensor(0xA38A,0x90);
		HM5065_write_cmos_sensor(0xA38B,0x0E);
		HM5065_write_cmos_sensor(0xA38C,0x16);
		HM5065_write_cmos_sensor(0xA38D,0xE0);
		HM5065_write_cmos_sensor(0xA38E,0xFA);
		HM5065_write_cmos_sensor(0xA38F,0xA3);
		HM5065_write_cmos_sensor(0xA390,0xE0);
		HM5065_write_cmos_sensor(0xA391,0xFB);
		HM5065_write_cmos_sensor(0xA392,0xD3);
		HM5065_write_cmos_sensor(0xA393,0x9F);
		HM5065_write_cmos_sensor(0xA394,0xEA);
		HM5065_write_cmos_sensor(0xA395,0x9E);
		HM5065_write_cmos_sensor(0xA396,0x40);
		HM5065_write_cmos_sensor(0xA397,0x0A);
		HM5065_write_cmos_sensor(0xA398,0x90);
		HM5065_write_cmos_sensor(0xA399,0x06);
		HM5065_write_cmos_sensor(0xA39A,0xD5);
		HM5065_write_cmos_sensor(0xA39B,0xEC);
		HM5065_write_cmos_sensor(0xA39C,0xF0);
		HM5065_write_cmos_sensor(0xA39D,0xA3);
		HM5065_write_cmos_sensor(0xA39E,0xED);
		HM5065_write_cmos_sensor(0xA39F,0xF0);
		HM5065_write_cmos_sensor(0xA3A0,0x80);
		HM5065_write_cmos_sensor(0xA3A1,0x0E);
		HM5065_write_cmos_sensor(0xA3A2,0xE9);
		HM5065_write_cmos_sensor(0xA3A3,0x7E);
		HM5065_write_cmos_sensor(0xA3A4,0x00);
		HM5065_write_cmos_sensor(0xA3A5,0x2B);
		HM5065_write_cmos_sensor(0xA3A6,0xFF);
		HM5065_write_cmos_sensor(0xA3A7,0xEE);
		HM5065_write_cmos_sensor(0xA3A8,0x3A);
		HM5065_write_cmos_sensor(0xA3A9,0x90);
		HM5065_write_cmos_sensor(0xA3AA,0x06);
		HM5065_write_cmos_sensor(0xA3AB,0xD5);
		HM5065_write_cmos_sensor(0xA3AC,0xF0);
		HM5065_write_cmos_sensor(0xA3AD,0xA3);
		HM5065_write_cmos_sensor(0xA3AE,0xEF);
		HM5065_write_cmos_sensor(0xA3AF,0xF0);
		HM5065_write_cmos_sensor(0xA3B0,0xE9);
		HM5065_write_cmos_sensor(0xA3B1,0xFB);
		HM5065_write_cmos_sensor(0xA3B2,0x7A);
		HM5065_write_cmos_sensor(0xA3B3,0x00);
		HM5065_write_cmos_sensor(0xA3B4,0x90);
		HM5065_write_cmos_sensor(0xA3B5,0x0E);
		HM5065_write_cmos_sensor(0xA3B6,0x15);
		HM5065_write_cmos_sensor(0xA3B7,0xE0);
		HM5065_write_cmos_sensor(0xA3B8,0x2B);
		HM5065_write_cmos_sensor(0xA3B9,0xFE);
		HM5065_write_cmos_sensor(0xA3BA,0x90);
		HM5065_write_cmos_sensor(0xA3BB,0x0E);
		HM5065_write_cmos_sensor(0xA3BC,0x14);
		HM5065_write_cmos_sensor(0xA3BD,0xE0);
		HM5065_write_cmos_sensor(0xA3BE,0x3A);
		HM5065_write_cmos_sensor(0xA3BF,0x90);
		HM5065_write_cmos_sensor(0xA3C0,0x06);
		HM5065_write_cmos_sensor(0xA3C1,0xE1);
		HM5065_write_cmos_sensor(0xA3C2,0xF0);
		HM5065_write_cmos_sensor(0xA3C3,0xA3);
		HM5065_write_cmos_sensor(0xA3C4,0xCE);
		HM5065_write_cmos_sensor(0xA3C5,0xF0);
		HM5065_write_cmos_sensor(0xA3C6,0xC3);
		HM5065_write_cmos_sensor(0xA3C7,0x90);
		HM5065_write_cmos_sensor(0xA3C8,0x0E);
		HM5065_write_cmos_sensor(0xA3C9,0x17);
		HM5065_write_cmos_sensor(0xA3CA,0xE0);
		HM5065_write_cmos_sensor(0xA3CB,0x9B);
		HM5065_write_cmos_sensor(0xA3CC,0xFE);
		HM5065_write_cmos_sensor(0xA3CD,0x90);
		HM5065_write_cmos_sensor(0xA3CE,0x0E);
		HM5065_write_cmos_sensor(0xA3CF,0x16);
		HM5065_write_cmos_sensor(0xA3D0,0x02);
		HM5065_write_cmos_sensor(0xA3D1,0x20);
		HM5065_write_cmos_sensor(0xA3D2,0xD5);
		HM5065_write_cmos_sensor(0xA3D3,0x90);
		HM5065_write_cmos_sensor(0xA3d4,0x0E);
		HM5065_write_cmos_sensor(0xA3d5,0xE4);
		HM5065_write_cmos_sensor(0xA3d6,0xE0);
		HM5065_write_cmos_sensor(0xA3d7,0x90);
		HM5065_write_cmos_sensor(0xA3d8,0x02);
		HM5065_write_cmos_sensor(0xA3d9,0x66);
		HM5065_write_cmos_sensor(0xA3da,0xF0);
		HM5065_write_cmos_sensor(0xA3DB,0x90);
		HM5065_write_cmos_sensor(0xA3dc,0x0E);
		HM5065_write_cmos_sensor(0xA3dd,0xE5);
		HM5065_write_cmos_sensor(0xA3de,0xE0);
		HM5065_write_cmos_sensor(0xA3df,0x90);
		HM5065_write_cmos_sensor(0xA3e0,0x02);
		HM5065_write_cmos_sensor(0xA3e1,0x64);
		HM5065_write_cmos_sensor(0xA3e2,0xF0);
		HM5065_write_cmos_sensor(0xA3e3,0x90);
		HM5065_write_cmos_sensor(0xA3e4,0x0E);
		HM5065_write_cmos_sensor(0xA3e5,0xE6);
		HM5065_write_cmos_sensor(0xA3e6,0xE0);
		HM5065_write_cmos_sensor(0xA3e7,0x90);
		HM5065_write_cmos_sensor(0xA3e8,0x02);
		HM5065_write_cmos_sensor(0xA3e9,0x65);
		HM5065_write_cmos_sensor(0xA3ea,0xF0);
		HM5065_write_cmos_sensor(0xA3eb,0x02);
		HM5065_write_cmos_sensor(0xA3ec,0x67);
		HM5065_write_cmos_sensor(0xA3ed,0xA5);
		HM5065_write_cmos_sensor(0xA3f0,0x12);
		HM5065_write_cmos_sensor(0xA3f1,0x47);
		HM5065_write_cmos_sensor(0xA3f2,0x59);
		HM5065_write_cmos_sensor(0xA3f3,0x90);
		HM5065_write_cmos_sensor(0xA3f4,0x00);
		HM5065_write_cmos_sensor(0xA3f5,0xB5);
		HM5065_write_cmos_sensor(0xA3f6,0xE0);
		HM5065_write_cmos_sensor(0xA3f7,0xB4);
		HM5065_write_cmos_sensor(0xA3f8,0x02);
		HM5065_write_cmos_sensor(0xA3f9,0x03);
		HM5065_write_cmos_sensor(0xA3fa,0x12);
		HM5065_write_cmos_sensor(0xA3fb,0x47);
		HM5065_write_cmos_sensor(0xA3fc,0x59);
		HM5065_write_cmos_sensor(0xA3fd,0x02);
		HM5065_write_cmos_sensor(0xA3fe,0xC5);
		HM5065_write_cmos_sensor(0xA3ff,0xC3);
		HM5065_write_cmos_sensor(0xA400,0x90);
		HM5065_write_cmos_sensor(0xA401,0x00);
		HM5065_write_cmos_sensor(0xA402,0x3D);
		HM5065_write_cmos_sensor(0xA403,0xF0);
		HM5065_write_cmos_sensor(0xA404,0x90);
		HM5065_write_cmos_sensor(0xA405,0x00);
		HM5065_write_cmos_sensor(0xA406,0x84);
		HM5065_write_cmos_sensor(0xA407,0xE0);
		HM5065_write_cmos_sensor(0xA408,0xFE);
		HM5065_write_cmos_sensor(0xA409,0x90);
		HM5065_write_cmos_sensor(0xA40A,0x00);
		HM5065_write_cmos_sensor(0xA40B,0x3E);
		HM5065_write_cmos_sensor(0xA40C,0xF0);
		HM5065_write_cmos_sensor(0xA40D,0xEF);
		HM5065_write_cmos_sensor(0xA40E,0x70);
		HM5065_write_cmos_sensor(0xA40F,0x03);
		HM5065_write_cmos_sensor(0xA410,0xEE);
		HM5065_write_cmos_sensor(0xA411,0x60);
		HM5065_write_cmos_sensor(0xA412,0x04);
		HM5065_write_cmos_sensor(0xA413,0x7F);
		HM5065_write_cmos_sensor(0xA414,0x01);
		HM5065_write_cmos_sensor(0xA415,0x80);
		HM5065_write_cmos_sensor(0xA416,0x02);
		HM5065_write_cmos_sensor(0xA417,0x7F);
		HM5065_write_cmos_sensor(0xA418,0x00);
		HM5065_write_cmos_sensor(0xA419,0x90);
		HM5065_write_cmos_sensor(0xA41A,0x00);
		HM5065_write_cmos_sensor(0xA41B,0x3F);
		HM5065_write_cmos_sensor(0xA41C,0xEF);
		HM5065_write_cmos_sensor(0xA41D,0xF0);
		HM5065_write_cmos_sensor(0xA41E,0x02);
		HM5065_write_cmos_sensor(0xA41F,0x89);
		HM5065_write_cmos_sensor(0xA420,0xD3);
		HM5065_write_cmos_sensor(0xA421,0x90);
		HM5065_write_cmos_sensor(0xA422,0x00);
		HM5065_write_cmos_sensor(0xA423,0x12);
		HM5065_write_cmos_sensor(0xA424,0xE0);
		HM5065_write_cmos_sensor(0xA425,0xFF);
		HM5065_write_cmos_sensor(0xA426,0x70);
		HM5065_write_cmos_sensor(0xA427,0x0C);
		HM5065_write_cmos_sensor(0xA428,0x90);
		HM5065_write_cmos_sensor(0xA429,0x00);
		HM5065_write_cmos_sensor(0xA42A,0x46);
		HM5065_write_cmos_sensor(0xA42B,0xE0);
		HM5065_write_cmos_sensor(0xA42C,0xC3);
		HM5065_write_cmos_sensor(0xA42D,0x94);
		HM5065_write_cmos_sensor(0xA42E,0x07);
		HM5065_write_cmos_sensor(0xA42F,0x40);
		HM5065_write_cmos_sensor(0xA430,0x03);
		HM5065_write_cmos_sensor(0xA431,0x75);
		HM5065_write_cmos_sensor(0xA432,0x2E);
		HM5065_write_cmos_sensor(0xA433,0x02);
		HM5065_write_cmos_sensor(0xA434,0xEF);
		HM5065_write_cmos_sensor(0xA435,0xB4);
		HM5065_write_cmos_sensor(0xA436,0x01);
		HM5065_write_cmos_sensor(0xA437,0x0C);
		HM5065_write_cmos_sensor(0xA438,0x90);
		HM5065_write_cmos_sensor(0xA439,0x00);
		HM5065_write_cmos_sensor(0xA43A,0x66);
		HM5065_write_cmos_sensor(0xA43B,0xE0);
		HM5065_write_cmos_sensor(0xA43C,0xC3);
		HM5065_write_cmos_sensor(0xA43D,0x94);
		HM5065_write_cmos_sensor(0xA43E,0x07);
		HM5065_write_cmos_sensor(0xA43F,0x40);
		HM5065_write_cmos_sensor(0xA440,0x03);
		HM5065_write_cmos_sensor(0xA441,0x75);
		HM5065_write_cmos_sensor(0xA442,0x2E);
		HM5065_write_cmos_sensor(0xA443,0x02);
		HM5065_write_cmos_sensor(0xA444,0x02);
		HM5065_write_cmos_sensor(0xA445,0xA7);
		HM5065_write_cmos_sensor(0xA446,0x9E);
		HM5065_write_cmos_sensor(0xA447,0xC3);
		HM5065_write_cmos_sensor(0xA448,0x90);
		HM5065_write_cmos_sensor(0xA449,0x0B);
		HM5065_write_cmos_sensor(0xA44A,0x8F);
		HM5065_write_cmos_sensor(0xA44B,0xE0);
		HM5065_write_cmos_sensor(0xA44C,0x94);
		HM5065_write_cmos_sensor(0xA44D,0x00);
		HM5065_write_cmos_sensor(0xA44E,0x90);
		HM5065_write_cmos_sensor(0xA44F,0x0B);
		HM5065_write_cmos_sensor(0xA450,0x8E);
		HM5065_write_cmos_sensor(0xA451,0xE0);
		HM5065_write_cmos_sensor(0xA452,0x94);
		HM5065_write_cmos_sensor(0xA453,0x41); //44
		HM5065_write_cmos_sensor(0xA454,0x40);
		HM5065_write_cmos_sensor(0xA455,0x22);
		HM5065_write_cmos_sensor(0xA456,0x90);
		HM5065_write_cmos_sensor(0xA457,0x0B);
		HM5065_write_cmos_sensor(0xA458,0x91);
		HM5065_write_cmos_sensor(0xA459,0xE0);
		HM5065_write_cmos_sensor(0xA45A,0x94);
		HM5065_write_cmos_sensor(0xA45B,0x00);//80
		HM5065_write_cmos_sensor(0xA45C,0x90);
		HM5065_write_cmos_sensor(0xA45D,0x0B);
		HM5065_write_cmos_sensor(0xA45E,0x90);
		HM5065_write_cmos_sensor(0xA45F,0xE0);
		HM5065_write_cmos_sensor(0xA460,0x94);
		HM5065_write_cmos_sensor(0xA461,0x41);//44
		HM5065_write_cmos_sensor(0xA462,0x40);
		HM5065_write_cmos_sensor(0xA463,0x14);
		HM5065_write_cmos_sensor(0xA464,0x90);
		HM5065_write_cmos_sensor(0xA465,0x0B);
		HM5065_write_cmos_sensor(0xA466,0x93);
		HM5065_write_cmos_sensor(0xA467,0xE0);
		HM5065_write_cmos_sensor(0xA468,0x94);
		HM5065_write_cmos_sensor(0xA469,0x00); //80
		HM5065_write_cmos_sensor(0xA46A,0x90);
		HM5065_write_cmos_sensor(0xA46B,0x0B);
		HM5065_write_cmos_sensor(0xA46C,0x92);
		HM5065_write_cmos_sensor(0xA46D,0xE0);
		HM5065_write_cmos_sensor(0xA46E,0x94);
		HM5065_write_cmos_sensor(0xA46F,0x41);//44
		HM5065_write_cmos_sensor(0xA470,0x40);
		HM5065_write_cmos_sensor(0xA471,0x06);
		HM5065_write_cmos_sensor(0xA472,0x90);
		HM5065_write_cmos_sensor(0xA473,0x01);
		HM5065_write_cmos_sensor(0xA474,0xA4);
		HM5065_write_cmos_sensor(0xA475,0x02);
		HM5065_write_cmos_sensor(0xA476,0x86);
		HM5065_write_cmos_sensor(0xA477,0x57);
		HM5065_write_cmos_sensor(0xA478,0x02);
		HM5065_write_cmos_sensor(0xA479,0x86);
		HM5065_write_cmos_sensor(0xA47A,0x5C);
		HM5065_write_cmos_sensor(0xA500,0xF5);
		HM5065_write_cmos_sensor(0xA501,0x3B);
		HM5065_write_cmos_sensor(0xA502,0x90);
		HM5065_write_cmos_sensor(0xA503,0x06);
		HM5065_write_cmos_sensor(0xA504,0x6C);
		HM5065_write_cmos_sensor(0xA505,0xE0);
		HM5065_write_cmos_sensor(0xA506,0xFF);
		HM5065_write_cmos_sensor(0xA507,0xE5);
		HM5065_write_cmos_sensor(0xA508,0x3B);
		HM5065_write_cmos_sensor(0xA509,0xC3);
		HM5065_write_cmos_sensor(0xA50A,0x9F);
		HM5065_write_cmos_sensor(0xA50B,0x40);
		HM5065_write_cmos_sensor(0xA50C,0x03);
		HM5065_write_cmos_sensor(0xA50D,0x02);
		HM5065_write_cmos_sensor(0xA50E,0xF6);
		HM5065_write_cmos_sensor(0xA50F,0x0E);
		HM5065_write_cmos_sensor(0xA510,0x90);
		HM5065_write_cmos_sensor(0xA511,0x0B);
		HM5065_write_cmos_sensor(0xA512,0xC6);
		HM5065_write_cmos_sensor(0xA513,0xE0);
		HM5065_write_cmos_sensor(0xA514,0x14);
		HM5065_write_cmos_sensor(0xA515,0x60);
		HM5065_write_cmos_sensor(0xA516,0x3C);
		HM5065_write_cmos_sensor(0xA517,0x14);
		HM5065_write_cmos_sensor(0xA518,0x60);
		HM5065_write_cmos_sensor(0xA519,0x6B);
		HM5065_write_cmos_sensor(0xA51A,0x24);
		HM5065_write_cmos_sensor(0xA51B,0x02);
		HM5065_write_cmos_sensor(0xA51C,0x60);
		HM5065_write_cmos_sensor(0xA51D,0x03);
		HM5065_write_cmos_sensor(0xA51E,0x02);
		HM5065_write_cmos_sensor(0xA51F,0xF5);
		HM5065_write_cmos_sensor(0xA520,0xB5);
		HM5065_write_cmos_sensor(0xA521,0x90);
		HM5065_write_cmos_sensor(0xA522,0x0A);
		HM5065_write_cmos_sensor(0xA523,0x9A);
		HM5065_write_cmos_sensor(0xA524,0xE0);
		HM5065_write_cmos_sensor(0xA525,0xFB);
		HM5065_write_cmos_sensor(0xA526,0xA3);
		HM5065_write_cmos_sensor(0xA527,0xE0);
		HM5065_write_cmos_sensor(0xA528,0xFA);
		HM5065_write_cmos_sensor(0xA529,0xA3);
		HM5065_write_cmos_sensor(0xA52A,0xE0);
		HM5065_write_cmos_sensor(0xA52B,0xF9);
		HM5065_write_cmos_sensor(0xA52C,0x85);
		HM5065_write_cmos_sensor(0xA52D,0x3B);
		HM5065_write_cmos_sensor(0xA52E,0x82);
		HM5065_write_cmos_sensor(0xA52F,0x75);
		HM5065_write_cmos_sensor(0xA530,0x83);
		HM5065_write_cmos_sensor(0xA531,0x00);
		HM5065_write_cmos_sensor(0xA532,0x12);
		HM5065_write_cmos_sensor(0xA533,0x0A);
		HM5065_write_cmos_sensor(0xA534,0xB8);
		HM5065_write_cmos_sensor(0xA535,0xFF);
		HM5065_write_cmos_sensor(0xA536,0x74);
		HM5065_write_cmos_sensor(0xA537,0xAB);
		HM5065_write_cmos_sensor(0xA538,0x25);
		HM5065_write_cmos_sensor(0xA539,0x3B);
		HM5065_write_cmos_sensor(0xA53A,0xF5);
		HM5065_write_cmos_sensor(0xA53B,0x82);
		HM5065_write_cmos_sensor(0xA53C,0xE4);
		HM5065_write_cmos_sensor(0xA53D,0x34);
		HM5065_write_cmos_sensor(0xA53E,0x0A);
		HM5065_write_cmos_sensor(0xA53F,0xF5);
		HM5065_write_cmos_sensor(0xA540,0x83);
		HM5065_write_cmos_sensor(0xA541,0xE0);
		HM5065_write_cmos_sensor(0xA542,0xFD);
		HM5065_write_cmos_sensor(0xA543,0xC3);
		HM5065_write_cmos_sensor(0xA544,0xEF);
		HM5065_write_cmos_sensor(0xA545,0x9D);
		HM5065_write_cmos_sensor(0xA546,0xFE);
		HM5065_write_cmos_sensor(0xA547,0xE4);
		HM5065_write_cmos_sensor(0xA548,0x94);
		HM5065_write_cmos_sensor(0xA549,0x00);
		HM5065_write_cmos_sensor(0xA54A,0x90);
		HM5065_write_cmos_sensor(0xA54B,0x0B);
		HM5065_write_cmos_sensor(0xA54C,0xCA);
		HM5065_write_cmos_sensor(0xA54D,0xF0);
		HM5065_write_cmos_sensor(0xA54E,0xA3);
		HM5065_write_cmos_sensor(0xA54F,0xCE);
		HM5065_write_cmos_sensor(0xA550,0xF0);
		HM5065_write_cmos_sensor(0xA551,0x80);
		HM5065_write_cmos_sensor(0xA552,0x62);
		HM5065_write_cmos_sensor(0xA553,0x90);
		HM5065_write_cmos_sensor(0xA554,0x0A);
		HM5065_write_cmos_sensor(0xA555,0x9A);
		HM5065_write_cmos_sensor(0xA556,0xE0);
		HM5065_write_cmos_sensor(0xA557,0xFB);
		HM5065_write_cmos_sensor(0xA558,0xA3);
		HM5065_write_cmos_sensor(0xA559,0xE0);
		HM5065_write_cmos_sensor(0xA55A,0xFA);
		HM5065_write_cmos_sensor(0xA55B,0xA3);
		HM5065_write_cmos_sensor(0xA55C,0xE0);
		HM5065_write_cmos_sensor(0xA55D,0xF9);
		HM5065_write_cmos_sensor(0xA55E,0x85);
		HM5065_write_cmos_sensor(0xA55F,0x3B);
		HM5065_write_cmos_sensor(0xA560,0x82);
		HM5065_write_cmos_sensor(0xA561,0x75);
		HM5065_write_cmos_sensor(0xA562,0x83);
		HM5065_write_cmos_sensor(0xA563,0x00);
		HM5065_write_cmos_sensor(0xA564,0x12);
		HM5065_write_cmos_sensor(0xA565,0x0A);
		HM5065_write_cmos_sensor(0xA566,0xB8);
		HM5065_write_cmos_sensor(0xA567,0xFF);
		HM5065_write_cmos_sensor(0xA568,0x74);
		HM5065_write_cmos_sensor(0xA569,0x9D);
		HM5065_write_cmos_sensor(0xA56A,0x25);
		HM5065_write_cmos_sensor(0xA56B,0x3B);
		HM5065_write_cmos_sensor(0xA56C,0xF5);
		HM5065_write_cmos_sensor(0xA56D,0x82);
		HM5065_write_cmos_sensor(0xA56E,0xE4);
		HM5065_write_cmos_sensor(0xA56F,0x34);
		HM5065_write_cmos_sensor(0xA570,0x0A);
		HM5065_write_cmos_sensor(0xA571,0xF5);
		HM5065_write_cmos_sensor(0xA572,0x83);
		HM5065_write_cmos_sensor(0xA573,0xE0);
		HM5065_write_cmos_sensor(0xA574,0xFD);
		HM5065_write_cmos_sensor(0xA575,0xC3);
		HM5065_write_cmos_sensor(0xA576,0xEF);
		HM5065_write_cmos_sensor(0xA577,0x9D);
		HM5065_write_cmos_sensor(0xA578,0xFE);
		HM5065_write_cmos_sensor(0xA579,0xE4);
		HM5065_write_cmos_sensor(0xA57A,0x94);
		HM5065_write_cmos_sensor(0xA57B,0x00);
		HM5065_write_cmos_sensor(0xA57C,0x90);
		HM5065_write_cmos_sensor(0xA57D,0x0B);
		HM5065_write_cmos_sensor(0xA57E,0xCA);
		HM5065_write_cmos_sensor(0xA57F,0xF0);
		HM5065_write_cmos_sensor(0xA580,0xA3);
		HM5065_write_cmos_sensor(0xA581,0xCE);
		HM5065_write_cmos_sensor(0xA582,0xF0);
		HM5065_write_cmos_sensor(0xA583,0x80);
		HM5065_write_cmos_sensor(0xA584,0x30);
		HM5065_write_cmos_sensor(0xA585,0x90);
		HM5065_write_cmos_sensor(0xA586,0x0A);
		HM5065_write_cmos_sensor(0xA587,0x9A);
		HM5065_write_cmos_sensor(0xA588,0xE0);
		HM5065_write_cmos_sensor(0xA589,0xFB);
		HM5065_write_cmos_sensor(0xA58A,0xA3);
		HM5065_write_cmos_sensor(0xA58B,0xE0);
		HM5065_write_cmos_sensor(0xA58C,0xFA);
		HM5065_write_cmos_sensor(0xA58D,0xA3);
		HM5065_write_cmos_sensor(0xA58E,0xE0);
		HM5065_write_cmos_sensor(0xA58F,0xF9);
		HM5065_write_cmos_sensor(0xA590,0x85);
		HM5065_write_cmos_sensor(0xA591,0x3B);
		HM5065_write_cmos_sensor(0xA592,0x82);
		HM5065_write_cmos_sensor(0xA593,0x75);
		HM5065_write_cmos_sensor(0xA594,0x83);
		HM5065_write_cmos_sensor(0xA595,0x00);
		HM5065_write_cmos_sensor(0xA596,0x12);
		HM5065_write_cmos_sensor(0xA597,0x0A);
		HM5065_write_cmos_sensor(0xA598,0xB8);
		HM5065_write_cmos_sensor(0xA599,0xFF);
		HM5065_write_cmos_sensor(0xA59A,0x74);
		HM5065_write_cmos_sensor(0xA59B,0xA4);
		HM5065_write_cmos_sensor(0xA59C,0x25);
		HM5065_write_cmos_sensor(0xA59D,0x3B);
		HM5065_write_cmos_sensor(0xA59E,0xF5);
		HM5065_write_cmos_sensor(0xA59F,0x82);
		HM5065_write_cmos_sensor(0xA5A0,0xE4);
		HM5065_write_cmos_sensor(0xA5A1,0x34);
		HM5065_write_cmos_sensor(0xA5A2,0x0A);
		HM5065_write_cmos_sensor(0xA5A3,0xF5);
		HM5065_write_cmos_sensor(0xA5A4,0x83);
		HM5065_write_cmos_sensor(0xA5A5,0xE0);
		HM5065_write_cmos_sensor(0xA5A6,0xFD);
		HM5065_write_cmos_sensor(0xA5A7,0xC3);
		HM5065_write_cmos_sensor(0xA5A8,0xEF);
		HM5065_write_cmos_sensor(0xA5A9,0x9D);
		HM5065_write_cmos_sensor(0xA5AA,0xFE);
		HM5065_write_cmos_sensor(0xA5AB,0xE4);
		HM5065_write_cmos_sensor(0xA5AC,0x94);
		HM5065_write_cmos_sensor(0xA5AD,0x00);
		HM5065_write_cmos_sensor(0xA5AE,0x90);
		HM5065_write_cmos_sensor(0xA5AF,0x0B);
		HM5065_write_cmos_sensor(0xA5B0,0xCA);
		HM5065_write_cmos_sensor(0xA5B1,0xF0);
		HM5065_write_cmos_sensor(0xA5B2,0xA3);
		HM5065_write_cmos_sensor(0xA5B3,0xCE);
		HM5065_write_cmos_sensor(0xA5B4,0xF0);
		HM5065_write_cmos_sensor(0xA5B5,0x90);
		HM5065_write_cmos_sensor(0xA5B6,0x07);
		HM5065_write_cmos_sensor(0xA5B7,0x83);
		HM5065_write_cmos_sensor(0xA5B8,0xE0);
		HM5065_write_cmos_sensor(0xA5B9,0xFF);
		HM5065_write_cmos_sensor(0xA5BA,0x7E);
		HM5065_write_cmos_sensor(0xA5BB,0x00);
		HM5065_write_cmos_sensor(0xA5BC,0x90);
		HM5065_write_cmos_sensor(0xA5BD,0x0D);
		HM5065_write_cmos_sensor(0xA5BE,0xF6);
		HM5065_write_cmos_sensor(0xA5BF,0xEE);
		HM5065_write_cmos_sensor(0xA5C0,0xF0);
		HM5065_write_cmos_sensor(0xA5C1,0xA3);
		HM5065_write_cmos_sensor(0xA5C2,0xEF);
		HM5065_write_cmos_sensor(0xA5C3,0xF0);
		HM5065_write_cmos_sensor(0xA5C4,0x90);
		HM5065_write_cmos_sensor(0xA5C5,0x0B);
		HM5065_write_cmos_sensor(0xA5C6,0xCA);
		HM5065_write_cmos_sensor(0xA5C7,0xE0);
		HM5065_write_cmos_sensor(0xA5C8,0xFC);
		HM5065_write_cmos_sensor(0xA5C9,0xA3);
		HM5065_write_cmos_sensor(0xA5CA,0xE0);
		HM5065_write_cmos_sensor(0xA5CB,0xFD);
		HM5065_write_cmos_sensor(0xA5CC,0xD3);
		HM5065_write_cmos_sensor(0xA5CD,0x9F);
		HM5065_write_cmos_sensor(0xA5CE,0x74);
		HM5065_write_cmos_sensor(0xA5CF,0x80);
		HM5065_write_cmos_sensor(0xA5D0,0xF8);
		HM5065_write_cmos_sensor(0xA5D1,0xEC);
		HM5065_write_cmos_sensor(0xA5D2,0x64);
		HM5065_write_cmos_sensor(0xA5D3,0x80);
		HM5065_write_cmos_sensor(0xA5D4,0x98);
		HM5065_write_cmos_sensor(0xA5D5,0x40);
		HM5065_write_cmos_sensor(0xA5D6,0x0C);
		HM5065_write_cmos_sensor(0xA5D7,0x90);
		HM5065_write_cmos_sensor(0xA5D8,0x0B);
		HM5065_write_cmos_sensor(0xA5D9,0xC8);
		HM5065_write_cmos_sensor(0xA5DA,0xE0);
		HM5065_write_cmos_sensor(0xA5DB,0x04);
		HM5065_write_cmos_sensor(0xA5DC,0xF0);
		HM5065_write_cmos_sensor(0xA5DD,0xA3);
		HM5065_write_cmos_sensor(0xA5DE,0xE0);
		HM5065_write_cmos_sensor(0xA5DF,0x04);
		HM5065_write_cmos_sensor(0xA5E0,0xF0);
		HM5065_write_cmos_sensor(0xA5E1,0x80);
		HM5065_write_cmos_sensor(0xA5E2,0x26);
		HM5065_write_cmos_sensor(0xA5E3,0x90);
		HM5065_write_cmos_sensor(0xA5E4,0x0D);
		HM5065_write_cmos_sensor(0xA5E5,0xF6);
		HM5065_write_cmos_sensor(0xA5E6,0xE0);
		HM5065_write_cmos_sensor(0xA5E7,0xFE);
		HM5065_write_cmos_sensor(0xA5E8,0xA3);
		HM5065_write_cmos_sensor(0xA5E9,0xE0);
		HM5065_write_cmos_sensor(0xA5EA,0xFF);
		HM5065_write_cmos_sensor(0xA5EB,0xC3);
		HM5065_write_cmos_sensor(0xA5EC,0xE4);
		HM5065_write_cmos_sensor(0xA5ED,0x9F);
		HM5065_write_cmos_sensor(0xA5EE,0xFF);
		HM5065_write_cmos_sensor(0xA5EF,0xE4);
		HM5065_write_cmos_sensor(0xA5F0,0x9E);
		HM5065_write_cmos_sensor(0xA5F1,0xFE);
		HM5065_write_cmos_sensor(0xA5F2,0xC3);
		HM5065_write_cmos_sensor(0xA5F3,0xED);
		HM5065_write_cmos_sensor(0xA5F4,0x9F);
		HM5065_write_cmos_sensor(0xA5F5,0xEE);
		HM5065_write_cmos_sensor(0xA5F6,0x64);
		HM5065_write_cmos_sensor(0xA5F7,0x80);
		HM5065_write_cmos_sensor(0xA5F8,0xF8);
		HM5065_write_cmos_sensor(0xA5F9,0xEC);
		HM5065_write_cmos_sensor(0xA5FA,0x64);
		HM5065_write_cmos_sensor(0xA5FB,0x80);
		HM5065_write_cmos_sensor(0xA5FC,0x98);
		HM5065_write_cmos_sensor(0xA5FD,0x50);
		HM5065_write_cmos_sensor(0xA5FE,0x0A);
		HM5065_write_cmos_sensor(0xA5FF,0x90);
		HM5065_write_cmos_sensor(0xA600,0x0B);
		HM5065_write_cmos_sensor(0xA601,0xC8);
		HM5065_write_cmos_sensor(0xA602,0xE0);
		HM5065_write_cmos_sensor(0xA603,0x14);
		HM5065_write_cmos_sensor(0xA604,0xF0);
		HM5065_write_cmos_sensor(0xA605,0xA3);
		HM5065_write_cmos_sensor(0xA606,0xE0);
		HM5065_write_cmos_sensor(0xA607,0x04);
		HM5065_write_cmos_sensor(0xA608,0xF0);
		HM5065_write_cmos_sensor(0xA609,0x05);
		HM5065_write_cmos_sensor(0xA60A,0x3B);
		HM5065_write_cmos_sensor(0xA60B,0x02);
		HM5065_write_cmos_sensor(0xA60C,0xF5);
		HM5065_write_cmos_sensor(0xA60D,0x02);
		HM5065_write_cmos_sensor(0xA60E,0x90);
		HM5065_write_cmos_sensor(0xA60F,0x08);
		HM5065_write_cmos_sensor(0xA610,0x58);
		HM5065_write_cmos_sensor(0xA611,0x02);
		HM5065_write_cmos_sensor(0xA612,0x9D);
		HM5065_write_cmos_sensor(0xA613,0x50);
		HM5065_write_cmos_sensor(0x9006,0xBA);
		HM5065_write_cmos_sensor(0x9007,0x75);
		HM5065_write_cmos_sensor(0x9008,0x00);
		HM5065_write_cmos_sensor(0x9009,0x00);
		HM5065_write_cmos_sensor(0x900A,0x02);
		HM5065_write_cmos_sensor(0x900D,0x01);
		HM5065_write_cmos_sensor(0x900E,0xA2);
		HM5065_write_cmos_sensor(0x900F,0x8F);
		HM5065_write_cmos_sensor(0x9010,0x00);
		HM5065_write_cmos_sensor(0x9011,0xCB);
		HM5065_write_cmos_sensor(0x9012,0x03);
		HM5065_write_cmos_sensor(0x9016,0xE6);
		HM5065_write_cmos_sensor(0x9017,0x6B);
		HM5065_write_cmos_sensor(0x9018,0x02);
		HM5065_write_cmos_sensor(0x9019,0x6B);
		HM5065_write_cmos_sensor(0x901A,0x02);
		HM5065_write_cmos_sensor(0x901D,0x01);
		HM5065_write_cmos_sensor(0x901E,0xAC);
		HM5065_write_cmos_sensor(0x901F,0x70);
		HM5065_write_cmos_sensor(0x9020,0x00);
		HM5065_write_cmos_sensor(0x9021,0xC5);
		HM5065_write_cmos_sensor(0x9022,0x03);
		HM5065_write_cmos_sensor(0x9026,0x9C);
		HM5065_write_cmos_sensor(0x9027,0x5B);
		HM5065_write_cmos_sensor(0x9028,0x00);
		HM5065_write_cmos_sensor(0x9029,0xBF);
		HM5065_write_cmos_sensor(0x902A,0x02);
		HM5065_write_cmos_sensor(0x902E,0x60);
		HM5065_write_cmos_sensor(0x902F,0x1C);
		HM5065_write_cmos_sensor(0x9030,0x01);
		HM5065_write_cmos_sensor(0x9031,0x37);
		HM5065_write_cmos_sensor(0x9032,0x02);
		HM5065_write_cmos_sensor(0x9035,0x01);
		HM5065_write_cmos_sensor(0x9036,0xBA);
		HM5065_write_cmos_sensor(0x9037,0x70);
		HM5065_write_cmos_sensor(0x9038,0x00);
		HM5065_write_cmos_sensor(0x9039,0x00);
		HM5065_write_cmos_sensor(0x903A,0x03);
		HM5065_write_cmos_sensor(0x903E,0x21);
		HM5065_write_cmos_sensor(0x903F,0x3F);
		HM5065_write_cmos_sensor(0x9040,0x02);
		HM5065_write_cmos_sensor(0x9041,0x40);
		HM5065_write_cmos_sensor(0x9042,0x02);
		HM5065_write_cmos_sensor(0x9046,0x21);
		HM5065_write_cmos_sensor(0x9047,0xEA);
		HM5065_write_cmos_sensor(0x9048,0x02);
		HM5065_write_cmos_sensor(0x9049,0x43);
		HM5065_write_cmos_sensor(0x904A,0x02);
		HM5065_write_cmos_sensor(0x904E,0xA6);
		HM5065_write_cmos_sensor(0x904F,0x12);
		HM5065_write_cmos_sensor(0x9050,0x02);
		HM5065_write_cmos_sensor(0x9051,0x46);
		HM5065_write_cmos_sensor(0x9052,0x02);
		HM5065_write_cmos_sensor(0x9056,0x29);
		HM5065_write_cmos_sensor(0x9057,0xE3);
		HM5065_write_cmos_sensor(0x9058,0x02);
		HM5065_write_cmos_sensor(0x9059,0x49);
		HM5065_write_cmos_sensor(0x905A,0x02);
		HM5065_write_cmos_sensor(0x905D,0x01);
		HM5065_write_cmos_sensor(0x905E,0x9C);
		HM5065_write_cmos_sensor(0x905F,0x6E);
		HM5065_write_cmos_sensor(0x9060,0x05);
		HM5065_write_cmos_sensor(0x9061,0x00);
		HM5065_write_cmos_sensor(0x9062,0x02);
		HM5065_write_cmos_sensor(0x9065,0x01);
		HM5065_write_cmos_sensor(0x9066,0xA2);
		HM5065_write_cmos_sensor(0x9067,0x66);
		HM5065_write_cmos_sensor(0x9068,0x02);
		HM5065_write_cmos_sensor(0x9069,0x35);
		HM5065_write_cmos_sensor(0x906A,0x02);
		HM5065_write_cmos_sensor(0x906D,0x01);
		HM5065_write_cmos_sensor(0x906E,0xB5);
		HM5065_write_cmos_sensor(0x906F,0xC2);
		HM5065_write_cmos_sensor(0x9070,0x02);
		HM5065_write_cmos_sensor(0x9071,0x9B);
		HM5065_write_cmos_sensor(0x9072,0x02);
		HM5065_write_cmos_sensor(0x9075,0x01);
		HM5065_write_cmos_sensor(0x9076,0xA2);
		HM5065_write_cmos_sensor(0x9077,0xD4);
		HM5065_write_cmos_sensor(0x9078,0x02);
		HM5065_write_cmos_sensor(0x9079,0xBE);
		HM5065_write_cmos_sensor(0x907A,0x02);
		HM5065_write_cmos_sensor(0x907D,0x01);
		HM5065_write_cmos_sensor(0x907E,0xB7);
		HM5065_write_cmos_sensor(0x907F,0xEA);
		HM5065_write_cmos_sensor(0x9080,0x00);
		HM5065_write_cmos_sensor(0x9081,0x02);
		HM5065_write_cmos_sensor(0x9082,0x03);
		HM5065_write_cmos_sensor(0x9086,0x67);
		HM5065_write_cmos_sensor(0x9087,0x31);
		HM5065_write_cmos_sensor(0x9088,0x02);
		HM5065_write_cmos_sensor(0x9089,0xF7);
		HM5065_write_cmos_sensor(0x908A,0x02);
		HM5065_write_cmos_sensor(0x908E,0x66);
		HM5065_write_cmos_sensor(0x908F,0xED);
		HM5065_write_cmos_sensor(0x9090,0x03);
		HM5065_write_cmos_sensor(0x9091,0x1D);
		HM5065_write_cmos_sensor(0x9092,0x02);
		HM5065_write_cmos_sensor(0x9096,0x67);
		HM5065_write_cmos_sensor(0x9097,0x73);
		HM5065_write_cmos_sensor(0x9098,0x03);
		HM5065_write_cmos_sensor(0x9099,0xD3);
		HM5065_write_cmos_sensor(0x909A,0x02);
		HM5065_write_cmos_sensor(0x909E,0x20);
		HM5065_write_cmos_sensor(0x909F,0x40);
		HM5065_write_cmos_sensor(0x90A0,0x03);
		HM5065_write_cmos_sensor(0x90A1,0x3B);
		HM5065_write_cmos_sensor(0x90A2,0x02);
		HM5065_write_cmos_sensor(0x90A6,0xC5);
		HM5065_write_cmos_sensor(0x90A7,0xC0);
		HM5065_write_cmos_sensor(0x90A8,0x03);
		HM5065_write_cmos_sensor(0x90A9,0xF0);
		HM5065_write_cmos_sensor(0x90AA,0x02);
		HM5065_write_cmos_sensor(0x90AE,0x41);
		HM5065_write_cmos_sensor(0x90AF,0xB3);
		HM5065_write_cmos_sensor(0x90B0,0x00);
		HM5065_write_cmos_sensor(0x90B1,0xA2);
		HM5065_write_cmos_sensor(0x90B2,0x02);
		HM5065_write_cmos_sensor(0x90B6,0x44);
		HM5065_write_cmos_sensor(0x90B7,0xBA);
		HM5065_write_cmos_sensor(0x90B8,0x00);
		HM5065_write_cmos_sensor(0x90B9,0xF0);
		HM5065_write_cmos_sensor(0x90BA,0x03);
		HM5065_write_cmos_sensor(0x90BE,0x89);
		HM5065_write_cmos_sensor(0x90BF,0x99);
		HM5065_write_cmos_sensor(0x90C0,0x04);
		HM5065_write_cmos_sensor(0x90C1,0x00);
		HM5065_write_cmos_sensor(0x90C2,0x02);
		HM5065_write_cmos_sensor(0x90C6,0xA7);
		HM5065_write_cmos_sensor(0x90C7,0x91);
		HM5065_write_cmos_sensor(0x90C8,0x04);
		HM5065_write_cmos_sensor(0x90C9,0x21);
		HM5065_write_cmos_sensor(0x90CA,0x02);
		HM5065_write_cmos_sensor(0x90CE,0x3A);
		HM5065_write_cmos_sensor(0x90CF,0x51);
		HM5065_write_cmos_sensor(0x90D0,0x00);
		HM5065_write_cmos_sensor(0x90D1,0xA2);
		HM5065_write_cmos_sensor(0x90D2,0x02);
		HM5065_write_cmos_sensor(0x90D6,0x86);
		HM5065_write_cmos_sensor(0x90D7,0x54);
		HM5065_write_cmos_sensor(0x90D8,0x04);
		HM5065_write_cmos_sensor(0x90D9,0x47);
		HM5065_write_cmos_sensor(0x90DA,0x02);
		HM5065_write_cmos_sensor(0x9000,0x01);
		HM5065_write_cmos_sensor(0xffff,0x00);
	
		mdelay(200);

		HM5065_write_cmos_sensor(0x0009,0x17); 	//	MCLK=26Mhz ,
		HM5065_write_cmos_sensor(0x0085,0x02); 	//	Reg0x0085=0x00(CbYCrY - sequence)for MTK 0:uyvy 2: yuyv
		HM5065_write_cmos_sensor(0x0040,0x00); 	//	binning mode and subsampling mode for frame rate
		HM5065_write_cmos_sensor(0x0016,0x00); 	//	Parallel(00) mode or mipi/1 lane(01) /2 lanes(02)
		HM5065_write_cmos_sensor(0x0046,0x00); 	//	data format (RGB/YUV/JPEG)
		HM5065_write_cmos_sensor(0x0041,0x00); 	//	04 : VGA mode : 0A : self define ; 00 : 5M
		HM5065_write_cmos_sensor(0x0042,0x0A); 	//	X:1298 --> 800 ->2592
		HM5065_write_cmos_sensor(0x0043,0x20); 	//
		HM5065_write_cmos_sensor(0x0044,0x07); 	//	Y:972 -->	600->1944
		HM5065_write_cmos_sensor(0x0045,0x98); 	//
		HM5065_write_cmos_sensor(0x7101,0xC4);
		HM5065_write_cmos_sensor(0x00E8,0x01);//AFR
		HM5065_write_cmos_sensor(0x00ED,0x0A);//Min.Frame Rate
		HM5065_write_cmos_sensor(0x00EE,0x1E);//Max.Framte Rate=30fps
		HM5065_write_cmos_sensor(0x00B2,0x4F);//4e   //50//set PLL output 560MHz
		HM5065_write_cmos_sensor(0x00B3,0xC0);//CA
		HM5065_write_cmos_sensor(0x00B5,0x01); 	//PLL Divider3 //02kai		
		HM5065_write_cmos_sensor(0x7104,0x02);
		HM5065_write_cmos_sensor(0x7105,0x00);
		HM5065_write_cmos_sensor(0x019C,0x4B);
		HM5065_write_cmos_sensor(0x019D,0x20);
		HM5065_write_cmos_sensor(0x0129,0x02);//00:Flat  meter 02:center meter
		HM5065_write_cmos_sensor(0x0130,0xFD);//00(0EV)->FF(-0.2EV)->FD(-0.2EV)
		HM5065_write_cmos_sensor(0x0083,0x00);//0x01
		HM5065_write_cmos_sensor(0x0084,0x00);//0x01
		HM5065_write_cmos_sensor(0x01A1,0x80);
		HM5065_write_cmos_sensor(0x01A2,0x80);
		HM5065_write_cmos_sensor(0x01A3,0x80);
		HM5065_write_cmos_sensor(0x01A0,0x01);
		HM5065_write_cmos_sensor(0x01D2,0x37);
		HM5065_write_cmos_sensor(0x01D3,0x33);
		HM5065_write_cmos_sensor(0x01D4,0x3A);
		HM5065_write_cmos_sensor(0x01D5,0x00);
		HM5065_write_cmos_sensor(0x01D6,0x3A);
		HM5065_write_cmos_sensor(0x01D7,0x00);		
		HM5065_write_cmos_sensor(0x0021,0x00);
		HM5065_write_cmos_sensor(0x0022,0x01);
		HM5065_write_cmos_sensor(0x0060,0x00);
		HM5065_write_cmos_sensor(0x0013,0x00);
		HM5065_write_cmos_sensor(0x0061,0x00);
		HM5065_write_cmos_sensor(0x0066,0x02);
		HM5065_write_cmos_sensor(0x0012,0x00);
		HM5065_write_cmos_sensor(0x7102,0x09);  //default 01 ; 09 : for clear Hsync gating
		HM5065_write_cmos_sensor(0x7103,0x00);
		HM5065_write_cmos_sensor(0x7158,0x00);
		HM5065_write_cmos_sensor(0x7000,0x2C);
		HM5065_write_cmos_sensor(0x5200,0x01);
		HM5065_write_cmos_sensor(0x7000,0x0C);
		HM5065_write_cmos_sensor(0x02C2,0x00);//Max Again 00E0=8x,0080=2x,  00C0=4x
		HM5065_write_cmos_sensor(0x02C3,0xB0);//C0 By Zeroy
		HM5065_write_cmos_sensor(0x015E,0x40);//Max Dgain 4100=3x ,4000=2x, 3E00=1x
		HM5065_write_cmos_sensor(0x015F,0x00);
		HM5065_write_cmos_sensor(0x0390,0x01);
		HM5065_write_cmos_sensor(0x0391,0x00);
		HM5065_write_cmos_sensor(0x0392,0x00);
		HM5065_write_cmos_sensor(0x03A0,0x14);
		HM5065_write_cmos_sensor(0x03A1,0x00);
		HM5065_write_cmos_sensor(0x03A2,0x5A);
		HM5065_write_cmos_sensor(0x03A3,0xEE);
		HM5065_write_cmos_sensor(0x03A4,0x69);
		HM5065_write_cmos_sensor(0x03A5,0x49);
		HM5065_write_cmos_sensor(0x03A6,0x3E);
		HM5065_write_cmos_sensor(0x03A7,0x00);
		HM5065_write_cmos_sensor(0x03A8,0x39);
		HM5065_write_cmos_sensor(0x03A9,0x33);
		HM5065_write_cmos_sensor(0x03B0,0x60);
		HM5065_write_cmos_sensor(0x03B1,0x00);
		HM5065_write_cmos_sensor(0x03B2,0x5A);
		HM5065_write_cmos_sensor(0x03B3,0xEE);
		HM5065_write_cmos_sensor(0x03B4,0x69);
		HM5065_write_cmos_sensor(0x03B5,0x49);
		HM5065_write_cmos_sensor(0x03B6,0x3E);
		HM5065_write_cmos_sensor(0x03B7,0x00);
		HM5065_write_cmos_sensor(0x03B8,0x3D);
		HM5065_write_cmos_sensor(0x03B9,0x20);
		HM5065_write_cmos_sensor(0x03C0,0x10);
		HM5065_write_cmos_sensor(0x03C1,0x00);
		HM5065_write_cmos_sensor(0x03C2,0x5A);
		HM5065_write_cmos_sensor(0x03C3,0xEE);
		HM5065_write_cmos_sensor(0x03C4,0x69);
		HM5065_write_cmos_sensor(0x03C5,0x49);
		HM5065_write_cmos_sensor(0x03C6,0x3A);
		HM5065_write_cmos_sensor(0x03C7,0x80);
		HM5065_write_cmos_sensor(0x03D0,0x64);
		HM5065_write_cmos_sensor(0x03D1,0x00);
		HM5065_write_cmos_sensor(0x03D2,0x5A);
		HM5065_write_cmos_sensor(0x03D3,0xEE);
		HM5065_write_cmos_sensor(0x03D4,0x69);
		HM5065_write_cmos_sensor(0x03D5,0x49);
		HM5065_write_cmos_sensor(0x03D6,0x34);
		HM5065_write_cmos_sensor(0x03D7,0xD1);
		HM5065_write_cmos_sensor(0x004C,0x18);
		HM5065_write_cmos_sensor(0x006C,0x08);
		HM5065_write_cmos_sensor(0x0350,0x00);
		HM5065_write_cmos_sensor(0x0351,0x5A);
		HM5065_write_cmos_sensor(0x0352,0xEE);
		HM5065_write_cmos_sensor(0x0353,0x69);
		HM5065_write_cmos_sensor(0x0354,0x49);
		HM5065_write_cmos_sensor(0x0355,0x39);
		HM5065_write_cmos_sensor(0x0356,0x6D);
		HM5065_write_cmos_sensor(0x0357,0x30);	//19 Zeroy
		HM5065_write_cmos_sensor(0x0358,0x00);
		HM5065_write_cmos_sensor(0x0359,0x3C);
		HM5065_write_cmos_sensor(0x035A,0x5A);
		HM5065_write_cmos_sensor(0x035B,0xEE);
		HM5065_write_cmos_sensor(0x035C,0x69);
		HM5065_write_cmos_sensor(0x035D,0x49);
		HM5065_write_cmos_sensor(0x035E,0x39);
		HM5065_write_cmos_sensor(0x035F,0x85);
		HM5065_write_cmos_sensor(0x0049,0x16);
		HM5065_write_cmos_sensor(0x004A,0x0E);
		HM5065_write_cmos_sensor(0x0069,0x14);
		HM5065_write_cmos_sensor(0x006A,0x0E);
		HM5065_write_cmos_sensor(0x0090,0x5A);
		HM5065_write_cmos_sensor(0x0091,0xEE);
		HM5065_write_cmos_sensor(0x0092,0x3E);
		HM5065_write_cmos_sensor(0x0093,0x00);
		HM5065_write_cmos_sensor(0x0094,0x69);
		HM5065_write_cmos_sensor(0x0095,0x49);
		HM5065_write_cmos_sensor(0x0096,0x39);
		HM5065_write_cmos_sensor(0x0097,0xCF);
		HM5065_write_cmos_sensor(0x0098,0x01);	//CF BY Zeroy 00->01
		HM5065_write_cmos_sensor(0x00A0,0x5A);
		HM5065_write_cmos_sensor(0x00A1,0xEE);
		HM5065_write_cmos_sensor(0x00A2,0x3E);
		HM5065_write_cmos_sensor(0x00A3,0x00);
		HM5065_write_cmos_sensor(0x00A4,0x69);
		HM5065_write_cmos_sensor(0x00A5,0x49);
		HM5065_write_cmos_sensor(0x00A6,0x3B);
		HM5065_write_cmos_sensor(0x00A7,0x80);
		HM5065_write_cmos_sensor(0x00A8,0x01);	//00->01  20131225
				
		HM5065_write_cmos_sensor(0x0420,0x00);//LSC_20120612
		HM5065_write_cmos_sensor(0x0421,0x09);
		HM5065_write_cmos_sensor(0x0422,0xff);
		HM5065_write_cmos_sensor(0x0423,0x9e);
		HM5065_write_cmos_sensor(0x0424,0x00);
		HM5065_write_cmos_sensor(0x0425,0x89);
		HM5065_write_cmos_sensor(0x0426,0x00);
		HM5065_write_cmos_sensor(0x0427,0xab);
		HM5065_write_cmos_sensor(0x0428,0xff);
		HM5065_write_cmos_sensor(0x0429,0xe9);
		HM5065_write_cmos_sensor(0x042a,0xff);
		HM5065_write_cmos_sensor(0x042b,0x8b);
		HM5065_write_cmos_sensor(0x042c,0x00);
		HM5065_write_cmos_sensor(0x042d,0x73);
		HM5065_write_cmos_sensor(0x042e,0xff);
		HM5065_write_cmos_sensor(0x042f,0xb6);
		HM5065_write_cmos_sensor(0x0430,0x00);
		HM5065_write_cmos_sensor(0x0431,0x54);
		HM5065_write_cmos_sensor(0x0432,0xff);
		HM5065_write_cmos_sensor(0x0433,0x43);
		HM5065_write_cmos_sensor(0x0434,0x01);
		HM5065_write_cmos_sensor(0x0435,0x04);
		HM5065_write_cmos_sensor(0x0436,0x01);
		HM5065_write_cmos_sensor(0x0437,0x34);
		HM5065_write_cmos_sensor(0x0438,0xff);
		HM5065_write_cmos_sensor(0x0439,0x7c);
		HM5065_write_cmos_sensor(0x043a,0xfe);
		HM5065_write_cmos_sensor(0x043b,0xd2);
		HM5065_write_cmos_sensor(0x043c,0x00);
		HM5065_write_cmos_sensor(0x043d,0x63);
		HM5065_write_cmos_sensor(0x043e,0xff);
		HM5065_write_cmos_sensor(0x043f,0x15);
		HM5065_write_cmos_sensor(0x0450,0x00);
		HM5065_write_cmos_sensor(0x0451,0x3b);
		HM5065_write_cmos_sensor(0x0452,0xff);
		HM5065_write_cmos_sensor(0x0453,0x98);
		HM5065_write_cmos_sensor(0x0454,0x00);
		HM5065_write_cmos_sensor(0x0455,0x6f);
		HM5065_write_cmos_sensor(0x0456,0x00);
		HM5065_write_cmos_sensor(0x0457,0x93);
		HM5065_write_cmos_sensor(0x0458,0xff);
		HM5065_write_cmos_sensor(0x0459,0xad);
		HM5065_write_cmos_sensor(0x045a,0xff);
		HM5065_write_cmos_sensor(0x045b,0x87);
		HM5065_write_cmos_sensor(0x045c,0x00);
		HM5065_write_cmos_sensor(0x045d,0x52);
		HM5065_write_cmos_sensor(0x045e,0xff);
		HM5065_write_cmos_sensor(0x045f,0xa7);
		HM5065_write_cmos_sensor(0x0440,0xff);
		HM5065_write_cmos_sensor(0x0441,0xfd);
		HM5065_write_cmos_sensor(0x0442,0xff);
		HM5065_write_cmos_sensor(0x0443,0x6c);
		HM5065_write_cmos_sensor(0x0444,0x00);
		HM5065_write_cmos_sensor(0x0445,0x90);
		HM5065_write_cmos_sensor(0x0446,0x00);
		HM5065_write_cmos_sensor(0x0447,0xa1);
		HM5065_write_cmos_sensor(0x0448,0x00);
		HM5065_write_cmos_sensor(0x0449,0x02);
		HM5065_write_cmos_sensor(0x044a,0xff);
		HM5065_write_cmos_sensor(0x044b,0x48);
		HM5065_write_cmos_sensor(0x044c,0x00);
		HM5065_write_cmos_sensor(0x044d,0x5b);
		HM5065_write_cmos_sensor(0x044e,0xff);
		HM5065_write_cmos_sensor(0x044f,0xb4);
		HM5065_write_cmos_sensor(0x0460,0xff);
		HM5065_write_cmos_sensor(0x0461,0x69);
		HM5065_write_cmos_sensor(0x0462,0xff);
		HM5065_write_cmos_sensor(0x0463,0xbb);
		HM5065_write_cmos_sensor(0x0464,0x00);
		HM5065_write_cmos_sensor(0x0465,0x84);
		HM5065_write_cmos_sensor(0x0466,0x00);
		HM5065_write_cmos_sensor(0x0467,0xa3);
		HM5065_write_cmos_sensor(0x0468,0x00);
		HM5065_write_cmos_sensor(0x0469,0x0e);
		HM5065_write_cmos_sensor(0x046a,0x00);
		HM5065_write_cmos_sensor(0x046b,0x76);
		HM5065_write_cmos_sensor(0x046c,0xff);
		HM5065_write_cmos_sensor(0x046d,0xaf);
		HM5065_write_cmos_sensor(0x046e,0xff);
		HM5065_write_cmos_sensor(0x046f,0xf5);
		HM5065_write_cmos_sensor(0x0470,0xff);
		HM5065_write_cmos_sensor(0x0471,0x8a);
		HM5065_write_cmos_sensor(0x0472,0xff);
		HM5065_write_cmos_sensor(0x0473,0x5a);
		HM5065_write_cmos_sensor(0x0474,0x00);
		HM5065_write_cmos_sensor(0x0475,0xef);
		HM5065_write_cmos_sensor(0x0476,0x01);
		HM5065_write_cmos_sensor(0x0477,0x16);
		HM5065_write_cmos_sensor(0x0478,0xff);
		HM5065_write_cmos_sensor(0x0479,0xd4);
		HM5065_write_cmos_sensor(0x047a,0x00);
		HM5065_write_cmos_sensor(0x047b,0x02);
		HM5065_write_cmos_sensor(0x047c,0x00);
		HM5065_write_cmos_sensor(0x047d,0x2c);
		HM5065_write_cmos_sensor(0x047e,0xff);
		HM5065_write_cmos_sensor(0x047f,0x95);
		HM5065_write_cmos_sensor(0x0490,0xff);
		HM5065_write_cmos_sensor(0x0491,0x9b);
		HM5065_write_cmos_sensor(0x0492,0xff);
		HM5065_write_cmos_sensor(0x0493,0x91);
		HM5065_write_cmos_sensor(0x0494,0x00);
		HM5065_write_cmos_sensor(0x0495,0x6f);
		HM5065_write_cmos_sensor(0x0496,0x00);
		HM5065_write_cmos_sensor(0x0497,0x95);
		HM5065_write_cmos_sensor(0x0498,0xff);
		HM5065_write_cmos_sensor(0x0499,0xd5);
		HM5065_write_cmos_sensor(0x049a,0x01);
		HM5065_write_cmos_sensor(0x049b,0x20);
		HM5065_write_cmos_sensor(0x049c,0xff);
		HM5065_write_cmos_sensor(0x049d,0xfb);
		HM5065_write_cmos_sensor(0x049e,0xff);
		HM5065_write_cmos_sensor(0x049f,0xe1);
		HM5065_write_cmos_sensor(0x0480,0xff);
		HM5065_write_cmos_sensor(0x0481,0x5a);
		HM5065_write_cmos_sensor(0x0482,0xff);
		HM5065_write_cmos_sensor(0x0483,0x91);
		HM5065_write_cmos_sensor(0x0484,0x00);
		HM5065_write_cmos_sensor(0x0485,0x8c);
		HM5065_write_cmos_sensor(0x0486,0x00);
		HM5065_write_cmos_sensor(0x0487,0x9f);
		HM5065_write_cmos_sensor(0x0488,0x00);
		HM5065_write_cmos_sensor(0x0489,0x29);
		HM5065_write_cmos_sensor(0x048a,0x00);
		HM5065_write_cmos_sensor(0x048b,0x53);
		HM5065_write_cmos_sensor(0x048c,0xff);
		HM5065_write_cmos_sensor(0x048d,0x80);
		HM5065_write_cmos_sensor(0x048e,0xff);
		HM5065_write_cmos_sensor(0x048f,0xf7);
		HM5065_write_cmos_sensor(0x04a0,0xff);
		HM5065_write_cmos_sensor(0x04a1,0x6c);
		HM5065_write_cmos_sensor(0x04a2,0xff);
		HM5065_write_cmos_sensor(0x04a3,0xb9);
		HM5065_write_cmos_sensor(0x04a4,0x00);
		HM5065_write_cmos_sensor(0x04a5,0x81);
		HM5065_write_cmos_sensor(0x04a6,0x00);
		HM5065_write_cmos_sensor(0x04a7,0x93);
		HM5065_write_cmos_sensor(0x04a8,0x00);
		HM5065_write_cmos_sensor(0x04a9,0x1c);
		HM5065_write_cmos_sensor(0x04aa,0x00);
		HM5065_write_cmos_sensor(0x04ab,0x39);
		HM5065_write_cmos_sensor(0x04ac,0xff);
		HM5065_write_cmos_sensor(0x04ad,0x9f);
		HM5065_write_cmos_sensor(0x04ae,0x00);
		HM5065_write_cmos_sensor(0x04af,0x0e);
		HM5065_write_cmos_sensor(0x04b0,0xff);
		HM5065_write_cmos_sensor(0x04b1,0x96);
		HM5065_write_cmos_sensor(0x04b2,0xff);
		HM5065_write_cmos_sensor(0x04b3,0x7b);
		HM5065_write_cmos_sensor(0x04b4,0x00);
		HM5065_write_cmos_sensor(0x04b5,0xaa);
		HM5065_write_cmos_sensor(0x04b6,0x00);
		HM5065_write_cmos_sensor(0x04b7,0xc8);
		HM5065_write_cmos_sensor(0x04b8,0xff);
		HM5065_write_cmos_sensor(0x04b9,0xe1);
		HM5065_write_cmos_sensor(0x04ba,0x00);
		HM5065_write_cmos_sensor(0x04bb,0x0e);
		HM5065_write_cmos_sensor(0x04bc,0x00);
		HM5065_write_cmos_sensor(0x04bd,0x0b);
		HM5065_write_cmos_sensor(0x04be,0xff);
		HM5065_write_cmos_sensor(0x04bf,0xff);
		HM5065_write_cmos_sensor(0x04d0,0xff);
		HM5065_write_cmos_sensor(0x04d1,0xac);
		HM5065_write_cmos_sensor(0x04d2,0xff);
		HM5065_write_cmos_sensor(0x04d3,0x93);
		HM5065_write_cmos_sensor(0x04d4,0x00);
		HM5065_write_cmos_sensor(0x04d5,0x64);
		HM5065_write_cmos_sensor(0x04d6,0x00);
		HM5065_write_cmos_sensor(0x04d7,0x83);
		HM5065_write_cmos_sensor(0x04d8,0xff);
		HM5065_write_cmos_sensor(0x04d9,0xdb);
		HM5065_write_cmos_sensor(0x04da,0x00);
		HM5065_write_cmos_sensor(0x04db,0xa8);
		HM5065_write_cmos_sensor(0x04dc,0xff);
		HM5065_write_cmos_sensor(0x04dd,0xf5);
		HM5065_write_cmos_sensor(0x04de,0x00);
		HM5065_write_cmos_sensor(0x04df,0x15);
		HM5065_write_cmos_sensor(0x04c0,0xff);
		HM5065_write_cmos_sensor(0x04c1,0x5d);
		HM5065_write_cmos_sensor(0x04c2,0xff);
		HM5065_write_cmos_sensor(0x04c3,0x9c);
		HM5065_write_cmos_sensor(0x04c4,0x00);
		HM5065_write_cmos_sensor(0x04c5,0x82);
		HM5065_write_cmos_sensor(0x04c6,0x00);
		HM5065_write_cmos_sensor(0x04c7,0x96);
		HM5065_write_cmos_sensor(0x04c8,0x00);
		HM5065_write_cmos_sensor(0x04c9,0x33);
		HM5065_write_cmos_sensor(0x04ca,0x00);
		HM5065_write_cmos_sensor(0x04cb,0x07);
		HM5065_write_cmos_sensor(0x04cc,0xff);
		HM5065_write_cmos_sensor(0x04cd,0x71);
		HM5065_write_cmos_sensor(0x04ce,0x00);
		HM5065_write_cmos_sensor(0x04cf,0x11);
		HM5065_write_cmos_sensor(0x04e0,0xff);
		HM5065_write_cmos_sensor(0x04e1,0x6D);//76->6D
		HM5065_write_cmos_sensor(0x04e2,0xff);
		HM5065_write_cmos_sensor(0x04e3,0xb8);
		HM5065_write_cmos_sensor(0x04e4,0x00);
		HM5065_write_cmos_sensor(0x04e5,0x84);
		HM5065_write_cmos_sensor(0x04e6,0x00);
		HM5065_write_cmos_sensor(0x04e7,0x96);
		HM5065_write_cmos_sensor(0x04e8,0xFF);//00->FF
		HM5065_write_cmos_sensor(0x04e9,0xC0);//21->C0
		HM5065_write_cmos_sensor(0x04ea,0x00);
		HM5065_write_cmos_sensor(0x04eb,0x6d);
		HM5065_write_cmos_sensor(0x04ec,0xff);
		HM5065_write_cmos_sensor(0x04ed,0xbb);
		HM5065_write_cmos_sensor(0x04ee,0x00);
		HM5065_write_cmos_sensor(0x04ef,0x00);
		HM5065_write_cmos_sensor(0x04f0,0xff);
		HM5065_write_cmos_sensor(0x04f1,0xA0);//90->A0
		HM5065_write_cmos_sensor(0x04f2,0xff);
		HM5065_write_cmos_sensor(0x04f3,0x95);//6D->95
		HM5065_write_cmos_sensor(0x04f4,0x00);
		HM5065_write_cmos_sensor(0x04f5,0xA7);//D2->A7
		HM5065_write_cmos_sensor(0x04f6,0x00);
		HM5065_write_cmos_sensor(0x04f7,0xC8);//FC->C8
		HM5065_write_cmos_sensor(0x04f8,0xff);
		HM5065_write_cmos_sensor(0x04f9,0xde);
		HM5065_write_cmos_sensor(0x04fa,0x00);
		HM5065_write_cmos_sensor(0x04fb,0x7e);
		HM5065_write_cmos_sensor(0x04fc,0x00);
		HM5065_write_cmos_sensor(0x04fd,0x36);
		HM5065_write_cmos_sensor(0x04fe,0x00);//FF->00
		HM5065_write_cmos_sensor(0x04ff,0x10);//AE->10
		HM5065_write_cmos_sensor(0x0510,0xff);
		HM5065_write_cmos_sensor(0x0511,0xc1);
		HM5065_write_cmos_sensor(0x0512,0xff);
		HM5065_write_cmos_sensor(0x0513,0x9f);
		HM5065_write_cmos_sensor(0x0514,0x00);
		HM5065_write_cmos_sensor(0x0515,0x6a);
		HM5065_write_cmos_sensor(0x0516,0x00);
		HM5065_write_cmos_sensor(0x0517,0x89);
		HM5065_write_cmos_sensor(0x0518,0xff);
		HM5065_write_cmos_sensor(0x0519,0xdc);
		HM5065_write_cmos_sensor(0x051a,0x00);
		HM5065_write_cmos_sensor(0x051b,0x55);
		HM5065_write_cmos_sensor(0x051c,0x00);
		HM5065_write_cmos_sensor(0x051d,0x09);
		HM5065_write_cmos_sensor(0x051e,0x00);
		HM5065_write_cmos_sensor(0x051f,0x0d);
		HM5065_write_cmos_sensor(0x0500,0xff);
		HM5065_write_cmos_sensor(0x0501,0x60);
		HM5065_write_cmos_sensor(0x0502,0xff);
		HM5065_write_cmos_sensor(0x0503,0x9e);
		HM5065_write_cmos_sensor(0x0504,0x00);
		HM5065_write_cmos_sensor(0x0505,0x81);
		HM5065_write_cmos_sensor(0x0506,0x00);
		HM5065_write_cmos_sensor(0x0507,0x9c);
		HM5065_write_cmos_sensor(0x0508,0xFF);//00->FF
		HM5065_write_cmos_sensor(0x0509,0xC0);//36->C0
		HM5065_write_cmos_sensor(0x050a,0x00);		
		HM5065_write_cmos_sensor(0x050b,0x40);
		HM5065_write_cmos_sensor(0x050c,0xff);
		HM5065_write_cmos_sensor(0x050d,0x8e);
		HM5065_write_cmos_sensor(0x050e,0x00);
		HM5065_write_cmos_sensor(0x050f,0x00);
		
		HM5065_write_cmos_sensor(0x0564,0x00);
		HM5065_write_cmos_sensor(0x0324,0x39);
		HM5065_write_cmos_sensor(0x0325,0xAE);
		HM5065_write_cmos_sensor(0x0326,0x3A);
		HM5065_write_cmos_sensor(0x0327,0x29);
		HM5065_write_cmos_sensor(0x0328,0x3B);
		HM5065_write_cmos_sensor(0x0329,0x0A);
		HM5065_write_cmos_sensor(0x032A,0x3B);
		HM5065_write_cmos_sensor(0x032B,0xAE);
		HM5065_write_cmos_sensor(0x0320,0x01);
		HM5065_write_cmos_sensor(0x0321,0x04);
		HM5065_write_cmos_sensor(0x0322,0x01);
		HM5065_write_cmos_sensor(0x0323,0x01);
		HM5065_write_cmos_sensor(0x0330,0xCF);
		HM5065_write_cmos_sensor(0x0384,0x00);
		HM5065_write_cmos_sensor(0x0337,0x01);
		HM5065_write_cmos_sensor(0x03EC,0x39);
		HM5065_write_cmos_sensor(0x03ED,0x85);
		HM5065_write_cmos_sensor(0x03FC,0x3A);
		HM5065_write_cmos_sensor(0x03FD,0x14);
		HM5065_write_cmos_sensor(0x040C,0x3A);
		HM5065_write_cmos_sensor(0x040D,0xF6);
		HM5065_write_cmos_sensor(0x041C,0x3B);
		HM5065_write_cmos_sensor(0x041D,0x9A);
		HM5065_write_cmos_sensor(0x03E0,0xB6);
		HM5065_write_cmos_sensor(0x03E1,0x04);
		HM5065_write_cmos_sensor(0x03E2,0xBB);
		HM5065_write_cmos_sensor(0x03E3,0xE9);
		HM5065_write_cmos_sensor(0x03E4,0xBC);
		HM5065_write_cmos_sensor(0x03E5,0x70);
		HM5065_write_cmos_sensor(0x03E6,0x37);
		HM5065_write_cmos_sensor(0x03E7,0x02);
		HM5065_write_cmos_sensor(0x03E8,0xBC);
		HM5065_write_cmos_sensor(0x03E9,0x00);
		HM5065_write_cmos_sensor(0x03EA,0xBF);
		HM5065_write_cmos_sensor(0x03EB,0x12);
		HM5065_write_cmos_sensor(0x03F0,0xBA);
		HM5065_write_cmos_sensor(0x03F1,0x7B);
		HM5065_write_cmos_sensor(0x03F2,0xBA);
		HM5065_write_cmos_sensor(0x03F3,0x83);
		HM5065_write_cmos_sensor(0x03F4,0xBB);
		HM5065_write_cmos_sensor(0x03F5,0xBC);
		HM5065_write_cmos_sensor(0x03F6,0x38);
		HM5065_write_cmos_sensor(0x03F7,0x2D);
		HM5065_write_cmos_sensor(0x03F8,0xBB);
		HM5065_write_cmos_sensor(0x03F9,0x23);
		HM5065_write_cmos_sensor(0x03FA,0xBD);
		HM5065_write_cmos_sensor(0x03FB,0xAC);
		HM5065_write_cmos_sensor(0x0400,0xBE);
		HM5065_write_cmos_sensor(0x0401,0x96);
		HM5065_write_cmos_sensor(0x0402,0xB9);
		HM5065_write_cmos_sensor(0x0403,0xBE);
		HM5065_write_cmos_sensor(0x0404,0xBB);
		HM5065_write_cmos_sensor(0x0405,0x57);
		HM5065_write_cmos_sensor(0x0406,0x3A);
		HM5065_write_cmos_sensor(0x0407,0xBB);
		HM5065_write_cmos_sensor(0x0408,0xB3);
		HM5065_write_cmos_sensor(0x0409,0x17);
		HM5065_write_cmos_sensor(0x040A,0xBE);
		HM5065_write_cmos_sensor(0x040B,0x66);
		HM5065_write_cmos_sensor(0x0410,0xBB);
		HM5065_write_cmos_sensor(0x0411,0x2A);
		HM5065_write_cmos_sensor(0x0412,0xBA);
		HM5065_write_cmos_sensor(0x0413,0x00);
		HM5065_write_cmos_sensor(0x0414,0xBB);
		HM5065_write_cmos_sensor(0x0415,0x10);
		HM5065_write_cmos_sensor(0x0416,0xB8);
		HM5065_write_cmos_sensor(0x0417,0xCD);
		HM5065_write_cmos_sensor(0x0418,0xB7);
		HM5065_write_cmos_sensor(0x0419,0x5C);
		HM5065_write_cmos_sensor(0x041A,0xBB);
		HM5065_write_cmos_sensor(0x041B,0x6C);
		HM5065_write_cmos_sensor(0x01f8,0x3c);
		HM5065_write_cmos_sensor(0x01f9,0x00);
		HM5065_write_cmos_sensor(0x01fa,0x00);
		HM5065_write_cmos_sensor(0x02a2,0x3e);
		HM5065_write_cmos_sensor(0x02a3,0x00);
		HM5065_write_cmos_sensor(0x02a4,0x3e);
		HM5065_write_cmos_sensor(0x02a5,0x00);
		HM5065_write_cmos_sensor(0x02a6,0x3e);
		HM5065_write_cmos_sensor(0x02a7,0x00);
		HM5065_write_cmos_sensor(0x02a8,0x3f);
		HM5065_write_cmos_sensor(0x02a9,0x66);
		HM5065_write_cmos_sensor(0x056c,0x42);
		HM5065_write_cmos_sensor(0x056d,0x00);
		HM5065_write_cmos_sensor(0x056e,0x42);
		HM5065_write_cmos_sensor(0x056f,0x00);
		HM5065_write_cmos_sensor(0x0570,0x42);
		HM5065_write_cmos_sensor(0x0571,0x00);
		HM5065_write_cmos_sensor(0x0572,0x42);
		HM5065_write_cmos_sensor(0x0573,0x00);
		HM5065_write_cmos_sensor(0x0081,0x64); //	;Saturation 0x75->0x7A
		HM5065_write_cmos_sensor(0x0588,0x00); //	;ColourSaturationDamper fDisable {CompiledExposureTime}
		HM5065_write_cmos_sensor(0x0589,0x5A); //	;ColourSaturationDamper fpLowThreshold {MSB}
		HM5065_write_cmos_sensor(0x058A,0xEE); //	;ColourSaturationDamper fpLowThreshold {LSB}
		HM5065_write_cmos_sensor(0x058B,0x69); //	;ColourSaturationDamper fpHighThreshold {MSB}
		HM5065_write_cmos_sensor(0x058C,0x49); //	;ColourSaturationDamper fpHighThreshold {LSB}
		HM5065_write_cmos_sensor(0x058D,0x3D); //	;ColourSaturationDamper fpMinimumOutput {MSB}
		HM5065_write_cmos_sensor(0x058E,0x3D); //	;ColourSaturationDamper fpMinimumOutput {LSB}
		HM5065_write_cmos_sensor(0x0080,0x6D); //	;Set Contrast
		HM5065_write_cmos_sensor(0x0082,0x64); //	;Set Brightness
		//HM5065_write_cmos_sensor(0x0010,0x01);

		//mdelay(200); // Sleep

		HM5065_write_cmos_sensor(0x4708,0x00); ;//Set horizontal and vertical offsets to zero
		HM5065_write_cmos_sensor(0x4709,0x00);
		HM5065_write_cmos_sensor(0x4710,0x00);
		HM5065_write_cmos_sensor(0x4711,0x00);

		// AF register optimuzation 
		HM5065_write_cmos_sensor(0x0658,0x00); // 2013-01-04
		HM5065_write_cmos_sensor(0x0659,0x00);//01
		HM5065_write_cmos_sensor(0x075A,0x00);//00 ->01
		HM5065_write_cmos_sensor(0x0756,0x00); //03->00,
		HM5065_write_cmos_sensor(0x065A,0x00);
		HM5065_write_cmos_sensor(0x06C9,0x01);
		HM5065_write_cmos_sensor(0x06CD,0x01);
		HM5065_write_cmos_sensor(0x06CE,0xbd);
		HM5065_write_cmos_sensor(0x06CF,0x00);
		HM5065_write_cmos_sensor(0x06D0,0x60);
		HM5065_write_cmos_sensor(0x06D1,0x02);
		HM5065_write_cmos_sensor(0x06D2,0x30);
		HM5065_write_cmos_sensor(0x06D3,0xD4);
		HM5065_write_cmos_sensor(0x06D4,0x01);
		HM5065_write_cmos_sensor(0x06D5,0x01);
		HM5065_write_cmos_sensor(0x06D6,0xbd);
		HM5065_write_cmos_sensor(0x06D7,0x00);
		HM5065_write_cmos_sensor(0x06D8,0x60);
		HM5065_write_cmos_sensor(0x06D9,0x00);
		HM5065_write_cmos_sensor(0x06DA,0x60);		
		HM5065_write_cmos_sensor(0x06DB,0x59);
		HM5065_write_cmos_sensor(0x06DC,0x0d);
		HM5065_write_cmos_sensor(0x0730,0x00);
		HM5065_write_cmos_sensor(0x0731,0x00);
		HM5065_write_cmos_sensor(0x0732,0x03);
		HM5065_write_cmos_sensor(0x0733,0xFF);
		HM5065_write_cmos_sensor(0x0734,0x03);
		HM5065_write_cmos_sensor(0x0735,0x70);
		HM5065_write_cmos_sensor(0x0755,0x01);		
		HM5065_write_cmos_sensor(0x075B,0x01);
		HM5065_write_cmos_sensor(0x075E,0x00);
		HM5065_write_cmos_sensor(0x0764,0x01);
		HM5065_write_cmos_sensor(0x0766,0x01);
		HM5065_write_cmos_sensor(0x0768,0x01);
		HM5065_write_cmos_sensor(0x076A,0x00);
		HM5065_write_cmos_sensor(0x0758,0x01);
		HM5065_write_cmos_sensor(0x075C,0x01);
		HM5065_write_cmos_sensor(0x0770,0x98); // 98->b5
	 	HM5065_write_cmos_sensor(0x0771,0x19);
		HM5065_write_cmos_sensor(0x0772,0x1B);
		HM5065_write_cmos_sensor(0x0774,0x01);
		HM5065_write_cmos_sensor(0x0775,0x4a);
		HM5065_write_cmos_sensor(0x0777,0x00);
		HM5065_write_cmos_sensor(0x0778,0x45);
		HM5065_write_cmos_sensor(0x0779,0x00);
		HM5065_write_cmos_sensor(0x077A,0x02);
		HM5065_write_cmos_sensor(0x077D,0x01);
		HM5065_write_cmos_sensor(0x077E,0x03);
		HM5065_write_cmos_sensor(0x0783,0x10); //10
		HM5065_write_cmos_sensor(0x0785,0x14);
		HM5065_write_cmos_sensor(0x0788,0x04);
		HM5065_write_cmos_sensor(0x0846,0x06);
		HM5065_write_cmos_sensor(0x0847,0x05);
		//END

		HM5065_write_cmos_sensor(0x0561,0x0e);
		HM5065_write_cmos_sensor(0x0562,0x01);
		HM5065_write_cmos_sensor(0x0563,0x01);
		HM5065_write_cmos_sensor(0x0564,0x06);
		
		//MIPI CSI setting
		HM5065_write_cmos_sensor(0xC41A,0x05);
		HM5065_write_cmos_sensor(0xC423,0x11);
		HM5065_write_cmos_sensor(0xC427,0x11);
		HM5065_write_cmos_sensor(0x300B,0x08);
	
		//80M  hkm test 
		HM5065_write_cmos_sensor(0x00B2,0x4F);//4e   //50//set PLL output 560MHz
		HM5065_write_cmos_sensor(0x00B3,0xC0);//CA
		HM5065_write_cmos_sensor(0x00B5,0x01);//02  ZOE
		HM5065_write_cmos_sensor(0x0030,0x11);//Max.Derate//14kai
		//HM5065_write_cmos_sensor(0x0143,0x5D);//Max.Int time_H	0x64->5F
		//HM5065_write_cmos_sensor(0x0144,0x0D);//Max.Int time_L	0x7F->0D		
    HM5065_write_cmos_sensor(0x01d2,0x37);
    HM5065_write_cmos_sensor(0x01d3,0x33);
    HM5065_write_cmos_sensor(0x01d4,0x3A);
    HM5065_write_cmos_sensor(0x01d5,0x00);
    HM5065_write_cmos_sensor(0x01d6,0x3A);
    HM5065_write_cmos_sensor(0x01d7,0x00);
    HM5065_write_cmos_sensor(0x070A,0x01);//ZOE ADD20150707
    
//		HM5065_write_cmos_sensor(0x0010,0x02);ZOE20150707
		mdelay(100);
		HM5065_write_cmos_sensor(0x0010,0x01);
		mdelay(100);
	HM5065SENSORDB("[HM5065]exit HM5065InitialSetting function:\n ");
		
} 
/*****************************************************************
* FUNCTION
*    HM5065PreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void HM5065PreviewSetting(void)
{
	//;HM5065 1280x960,30fps
	//56Mhz, 224Mbps/Lane, 2Lane.
	HM5065SENSORDB("[HM5065]enter HM5065PreviewSetting function:\n ");
	
	HM5065_write_cmos_sensor(0x0142,0x00);//open AE
	//HM5065_write_cmos_sensor(0x01A4,0x00);//open awb
	
	//HM5065_write_cmos_sensor(0x0010,0x00);//sw reset
	//mdelay(100);
	//HM5065_write_cmos_sensor(0x00B5,0x01);//  01=>02	35Mhz  ;01: 70MHz;02:35MHz
	//HM5065_write_cmos_sensor(0x0030,0x11);//14

	HM5065_write_cmos_sensor(0x0040,0x00); 	//	binning mode and subsampling mode for frame rate 2*2 add 2? 4?
	HM5065_write_cmos_sensor(0x0041,0x0A); 	//	04 : VGA mode : 0A : self define ; 00 : 5M ;03:SVGA
	HM5065_write_cmos_sensor(0x0042,0x0A); 	//05	X:800 0x500=1280,0x0320=800
	HM5065_write_cmos_sensor(0x0043,0x20); 	//00
	HM5065_write_cmos_sensor(0x0044,0x07); 	//03	Y:600 0x03c0=960,0x0258=600
	HM5065_write_cmos_sensor(0x0045,0x98); 	//c0
  HM5065_write_cmos_sensor(0x00ED,0x0a); 	//c0
  HM5065_write_cmos_sensor(0x070A,0x01); 	//c0
	HM5065_write_cmos_sensor(0x0251,0x02);//BLC ON	
	//HM5065_write_cmos_sensor(0x0010,0x01);
	//mdelay(100);
	
	if(HM5065_run_test_pattern)
	{
		HM5065_run_test_pattern=0;
		HM5065SetTestPatternMode(1);
	}
	//HM5065_write_cmos_sensor(0x4202, 0x00);//	; open mipi stream
	HM5065WriteExtraShutter(HM5065Sensor.PreviewExtraShutter);
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.SensorMode= SENSOR_MODE_PREVIEW;
	HM5065Sensor.IsPVmode = KAL_TRUE;
	HM5065Sensor.PreviewPclk= 560;	
	spin_unlock(&HM5065_drv_lock);
	HM5065SENSORDB("[HM5065]exit HM5065PreviewSetting function:\n ");
}

/*************************************************************************
* FUNCTION
*     HM5065FullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void HM5065FullSizeCaptureSetting(void)
{	
		HM5065_write_cmos_sensor(0x0142,0x01);//Freeze AE
		
		//HM5065_write_cmos_sensor(0x0010,0x00);
		//mdelay(100);
		//HM5065_write_cmos_sensor(0x00B5,0x01);//
		//HM5065_write_cmos_sensor(0x0030,0x11);	
		//mdelay(200);
		HM5065_write_cmos_sensor(0x0040,0x00); //Full size
		HM5065_write_cmos_sensor(0x0041,0x0A); //00:full size
		HM5065_write_cmos_sensor(0x0042,0x0A); //X:2592
		HM5065_write_cmos_sensor(0x0043,0x20);
		HM5065_write_cmos_sensor(0x0044,0x07); //Y:1944
		HM5065_write_cmos_sensor(0x0045,0x98);
    //HM5065_write_cmos_sensor(0x00ED,0x12);	
    //HM5065_write_cmos_sensor(0x0082,0x4A); // ZOE20150707
    //HM5065_write_cmos_sensor(0x004C,0x18);	
		HM5065_write_cmos_sensor(0x0251,0x01);//BLC off 
	
		// HM5065_write_cmos_sensor(0x0010,0x02);
		// mdelay(200);
		//HM5065_write_cmos_sensor(0x0010,0x01);//Stream on
		//mdelay(100);
	
	if(HM5065_run_test_pattern)
	{
		HM5065_run_test_pattern=0;
		HM5065SetTestPatternMode(1);
	}
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.IsPVmode = KAL_FALSE;
	HM5065Sensor.CapturePclk= 800;	
	//HM5065Sensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&HM5065_drv_lock);
	HM5065SENSORDB("[HM5065]exit HM5065FullSizeCaptureSetting function:\n ");
}

static void HM5065ZSDPreviewSetting(void)
{
		HM5065_write_cmos_sensor(0x0142,0x00);//Release AE
		//HM5065_write_cmos_sensor(0x0010,0x02);
		//mdelay(200);
		//HM5065_write_cmos_sensor(0x00B5,0x01);
		//HM5065_write_cmos_sensor(0x0030,0x11);	
		//mdelay(200);

		HM5065_write_cmos_sensor(0x0040,0x00); //Full size
		HM5065_write_cmos_sensor(0x0041,0x0A); //00:full size
		HM5065_write_cmos_sensor(0x0042,0x0A); //X:2592
		HM5065_write_cmos_sensor(0x0043,0x20);
		HM5065_write_cmos_sensor(0x0044,0x07); //Y:1944
		HM5065_write_cmos_sensor(0x0045,0x98);		
		HM5065_write_cmos_sensor(0x0251,0x02);//BLC on 
	
		//HM5065_write_cmos_sensor(0x0010,0x02);
		//mdelay(200);
		//HM5065_write_cmos_sensor(0x0010,0x01);//Stream on
		//mdelay(200);
	
	if(HM5065_run_test_pattern)
	{
		HM5065_run_test_pattern=0;
		HM5065SetTestPatternMode(1);
	}
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.IsPVmode = KAL_FALSE;
	HM5065Sensor.CapturePclk= 800;	
	//HM5065Sensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&HM5065_drv_lock);
	HM5065SENSORDB("[HM5065]exit HM5065ZSDPreviewetting function:\n ");
}

/*************************************************************************
* FUNCTION
*    HM5065SetHVMirror
*
* DESCRIPTION
*    This function set sensor Mirror
*
* PARAMETERS
*    Mirror
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void HM5065SetHVMirror(kal_uint8 Mirror)
{
  	kal_uint8 mirror= 0, flip=0;
    HM5065SENSORDB("[HM5065]enter HM5065SetHVMirror function:\n ");
		flip = HM5065_read_cmos_sensor(0x0083);
		mirror=HM5065_read_cmos_sensor(0x0084);
	
	if (SENSOR_MODE_PREVIEW)
	{
		switch (Mirror)
		{
		case IMAGE_NORMAL:
			HM5065_write_cmos_sensor(0x0083, 0x00);     
			HM5065_write_cmos_sensor(0x0084, 0x00);
			break;
		case IMAGE_H_MIRROR:
			HM5065_write_cmos_sensor(0x0083, 0x01);     
			HM5065_write_cmos_sensor(0x0084, 0x00);
			break;
		case IMAGE_V_MIRROR: 
			HM5065_write_cmos_sensor(0x0083, 0x00);     
			HM5065_write_cmos_sensor(0x0084, 0x01);
			break;		
		case IMAGE_HV_MIRROR:
			HM5065_write_cmos_sensor(0x0083, 0x01);     
			HM5065_write_cmos_sensor(0x0084, 0x01);
			break; 		
		default:
			ASSERT(0);
		}
	}
/*	else if (Mode== SENSOR_MODE_CAPTURE)
	{
		switch (Mirror)
		{
		case IMAGE_NORMAL:
			//HM5065_write_cmos_sensor(0x3820, flip&0xf9);     
			//HM5065_write_cmos_sensor(0x3821, mirror&0xf9);
			//HM5065_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_H_MIRROR:
			//HM5065_write_cmos_sensor(0x3820, flip&0xf9);     
			//HM5065_write_cmos_sensor(0x3821, mirror|0x06);
			//HM5065_write_cmos_sensor(0x4514, 0x00);
			break;
		case IMAGE_V_MIRROR: 
			//HM5065_write_cmos_sensor(0x3820, flip|0x06);     
			//HM5065_write_cmos_sensor(0x3821, mirror&0xf9);
			//HM5065_write_cmos_sensor(0x4514, 0xaa);
			break;		
		case IMAGE_HV_MIRROR:
			//HM5065_write_cmos_sensor(0x3820, flip|0x06);     
			//HM5065_write_cmos_sensor(0x3821, mirror|0x06);
			//HM5065_write_cmos_sensor(0x4514, 0xbb);
			break; 		
		default:
			ASSERT(0);
		}
	}*/
	HM5065SENSORDB("[HM5065]exit HM5065SetHVMirror function:\n ");
}

void HM5065_Standby(void)
{
	//HM5065_write_cmos_sensor(0x3008,0x42);
}

void HM5065_Wakeup(void)
{
	//HM5065_write_cmos_sensor(0x3008,0x02);
}
/*************************************************************************
* FUNCTION
*   HM5065_FOCUS_OVT_AFC_Init
* DESCRIPTION
*   This function is to load micro code for AF function
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void HM5065_FOCUS_OVT_AFC_Init(void)
{}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Constant_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void HM5065_FOCUS_OVT_AFC_Constant_Focus(void)
{
	HM5065SENSORDB("*********************[HM5065]enter HM5065_FOCUS_OVT_AFC_Constant_Focus function:\n ");
	HM5065_write_cmos_sensor(0x0751,0x00);	
	HM5065_write_cmos_sensor(0x070A,0x01);
	HM5065SENSORDB("###########################3[HM5065]exit HM5065_FOCUS_OVT_AFC_Constant_Focus function:\n ");
}   
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Single_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void HM5065_FOCUS_OVT_AFC_Single_Focus()
{
	HM5065SENSORDB("[HM5065]enter HM5065_FOCUS_OVT_AFC_Single_Focus function:\n ");

    HM5065SENSORDB("HM5065_FOCUS_Single_Focus\n");
    HM5065_write_cmos_sensor(0x0751,0x00);	
    HM5065_write_cmos_sensor(0x070A,0x03);
		mDELAY(50);
    HM5065_write_cmos_sensor(0x070B,0x01);;
		mDELAY(50);
    HM5065_write_cmos_sensor(0x070B,0x02);
		mDELAY(50);

	af_pos_h = HM5065_read_cmos_sensor(0x06F0);
	af_pos_l = HM5065_read_cmos_sensor(0x06F1);
    HM5065SENSORDB("after single focus  \n");
	HM5065SENSORDB("[HM5065]exit HM5065_FOCUS_OVT_AFC_Single_Focus function:\n ");
}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Pause_Focus
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void HM5065_FOCUS_OVT_AFC_Pause_Focus()
{

}
static void HM5065_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	  
    *pFeatureReturnPara32 = 1;    
    HM5065SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void HM5065_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    HM5065SENSORDB("[HM5065]enter HM5065_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
    *pFeatureReturnPara32 = 1;    
    HM5065SENSORDB(" *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
	HM5065SENSORDB("[HM5065]exit HM5065_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}
static void HM5065_FOCUS_OVT_AFC_Touch_AF(UINT32 x,UINT32 y)
{
	 /*HM5065SENSORDB("[HM5065]enter HM5065_FOCUS_OVT_AFC_Touch_AF function:\n ");
	 int x_view,y_view;
     int x_tmp,y_tmp;
     if(x<1)
     {
               x_view=1;
     }
     else if(x>79)
     {
               x_view=79;
     }
     else
     {
               x_view= x;
     }
     
     if(y<1)
     {
               y_view=1;
     }
     else if(y>59)
     {
               y_view=59;
     }
     else
     {
               y_view= y;
     }
	 HM5065SENSORDB("[HM5065]AF x_view=%d,y_view=%d\n",x_view, y_view);
     //HM5065_write_cmos_sensor(0x3024,x_view);
     //HM5065_write_cmos_sensor(0x3025,y_view);   
     x_tmp = HM5065_read_cmos_sensor(0x3024);
	 y_tmp = HM5065_read_cmos_sensor(0x3025);
	 HM5065SENSORDB("[HM5065]AF x_tmp1=%d,y_tmp1=%d\n",x_tmp, y_tmp);
     HM5065SENSORDB("[HM5065]exit HM5065_FOCUS_OVT_AFC_Touch_AF function:\n ");*/
}

#define FACE_LC 			0x0714
#define FACE_START_XH 	0x0715
#define FACE_START_XL 	0x0716
#define FACE_SIZE_XH  	0x0717
#define FACE_SIZE_XL	 0x0718
#define FACE_START_YH	 0x0719
#define FACE_START_YL	 0x071A
#define FACE_SIZE_YH	 0x071B
#define FACE_SIZE_YL 	0x071C

static void HM5065_FOCUS_Set_AF_Window(UINT32 zone_addr)
{//update global zone
	HM5065SENSORDB("[HM5065]enter HM5065_FOCUS_Set_AF_Window function:\n ");
	UINT32 FD_XS;
	UINT32 FD_YS;   
	UINT32 x0, y0, x1, y1;
	UINT32 pvx0, pvy0, pvx1, pvy1;
	UINT32 linenum, rownum;
	UINT32 AF_pvx, AF_pvy;
	UINT32* zone = (UINT32*)zone_addr;
	x0 = *zone;
	y0 = *(zone + 1);
	x1 = *(zone + 2);
	y1 = *(zone + 3);   
	FD_XS = *(zone + 4);
	FD_YS = *(zone + 5);

	HM5065SENSORDB("Set_AF_Window x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
	x0, y0, x1, y1, FD_XS, FD_YS);  

	if(CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_ZSD )
	{
		HM5065_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, ZSD_PRV_W, ZSD_PRV_H);
		HM5065_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, ZSD_PRV_W, ZSD_PRV_H);  
	}  
	else
	{
		HM5065_mapMiddlewaresizePointToPreviewsizePoint(x0,y0,FD_XS,FD_YS,&pvx0, &pvy0, PRV_W, PRV_H);
		HM5065_mapMiddlewaresizePointToPreviewsizePoint(x1,y1,FD_XS,FD_YS,&pvx1, &pvy1, PRV_W, PRV_H);  
	}
		
	HM5065SENSORDB("[HM5065]AF pvx0=%d,pvy0=%d\n",pvx0, pvy0);
	HM5065SENSORDB("[HM5065]AF pvx1=%d,pvy1=%d\n",pvx1, pvy1);

#if 1//def	AE_and_AF_FACE
		//HM5065_write_cmos_sensor(FACE_LC,0x03);//add abc disable//bit 0: AF  bit1: touch AE
#endif

	if((pvx0==pvx1)&&(pvy0==pvy1))
	{
		HM5065_write_cmos_sensor(0x0808,0x01);
		HM5065_write_cmos_sensor(0x0809,0x01); 
		HM5065_write_cmos_sensor(0x080a,0x01);
		HM5065_write_cmos_sensor(0x080b,0x01);	
		HM5065_write_cmos_sensor(0x080c,0x01);
		HM5065_write_cmos_sensor(0x080d,0x01);
		HM5065_write_cmos_sensor(0x080e,0x01); 
		
		HM5065_write_cmos_sensor(0x0751,0x00);	  
		HM5065_write_cmos_sensor(FACE_LC,0x00);//add abc enable//bit 0: AF	bit1: touch AE
/*
       	HM5065_write_cmos_sensor(FACE_START_XH, (pvx0-160)>>8);
		HM5065_write_cmos_sensor(FACE_START_XL, (pvx0-160)&0xff);

		HM5065_write_cmos_sensor(FACE_START_YH, (pvy0-160)>>8);
		HM5065_write_cmos_sensor(FACE_START_YL, (pvy0-160)&0xff);		

		HM5065SENSORDB("HM5065_FOCUS_Set_AF_Center Window_to_IC\n");

		HM5065_write_cmos_sensor(FACE_SIZE_XH,0x01);
		HM5065_write_cmos_sensor(FACE_SIZE_XL,0x40);
		HM5065_write_cmos_sensor(FACE_SIZE_YH,0x01);
		HM5065_write_cmos_sensor(FACE_SIZE_YL,0x40);
*/
		HM5065SENSORDB("HM5065_FOCUS_Set_ normal AF_Center Window_to_IC\n");
		
    }
	else{
		HM5065_write_cmos_sensor(0x0808,0x01);
		HM5065_write_cmos_sensor(0x0809,0x00); 
		HM5065_write_cmos_sensor(0x080a,0x00);
		HM5065_write_cmos_sensor(0x080b,0x00);  
		HM5065_write_cmos_sensor(0x080c,0x00);
		HM5065_write_cmos_sensor(0x080d,0x00);
		HM5065_write_cmos_sensor(0x080e,0x00); 

		HM5065_write_cmos_sensor(0x0751,0x00);	  
		HM5065_write_cmos_sensor(FACE_LC,0x00);

#if 1
		HM5065_write_cmos_sensor(FACE_START_XH, pvx0>>8);
		HM5065_write_cmos_sensor(FACE_START_XL, pvx0&0xff);

		HM5065_write_cmos_sensor(FACE_START_YH, pvy0>>8);
		HM5065_write_cmos_sensor(FACE_START_YL, pvy0&0xff);		

		HM5065SENSORDB("HM5065_FOCUS_Set_AF_Touch Window_to_IC\n");

		HM5065_write_cmos_sensor(FACE_SIZE_XH,(pvx1-pvx0)>>8);
		HM5065_write_cmos_sensor(FACE_SIZE_XL,(pvx1-pvx0)&0xff);
		HM5065_write_cmos_sensor(FACE_SIZE_YH,(pvy1-pvy0)>>8);
		HM5065_write_cmos_sensor(FACE_SIZE_YL,(pvy1-pvy0)&0xff);
#endif
#if 0//test fix window
		HM5065_write_cmos_sensor(FACE_START_XH, 0x00);
		HM5065_write_cmos_sensor(FACE_START_XL, 0x10);

		HM5065_write_cmos_sensor(FACE_START_YH, 0x00);
		HM5065_write_cmos_sensor(FACE_START_YL, 0x10);		

		HM5065SENSORDB("HM5065_FOCUS_Set_AF_Touch Window_to_IC\n");

		HM5065_write_cmos_sensor(FACE_SIZE_XH,0x00);
		HM5065_write_cmos_sensor(FACE_SIZE_XL,0xA0);
		HM5065_write_cmos_sensor(FACE_SIZE_YH,0x00);
		HM5065_write_cmos_sensor(FACE_SIZE_YL,0xA0);

#endif}
	}

	HM5065SENSORDB("[HM5065]exit HM5065_FOCUS_Set_AF_Window function:\n ");
}
static void HM5065_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{

    HM5065SENSORDB("HM5065_FOCUS_Get_AF_Macro\n");
	//lens rang: 0--0x3ff
		HM5065_write_cmos_sensor(0x0751,0x00);	
		HM5065_write_cmos_sensor(0x070A,0x00);//disable af
    HM5065_write_cmos_sensor(0x0700,0x00);//position h
    HM5065_write_cmos_sensor(0x0701,0x05);//position l
		HM5065_write_cmos_sensor(0x070C,0x00);//fix len to 5
		mDELAY(100);
    HM5065_write_cmos_sensor(0x070C,0x07);//fix len to 5

    *pFeatureReturnPara32 = 0;
}
static void HM5065_FOCUS_Get_AF_Inf(UINT32 * pFeatureReturnPara32)
{

    HM5065SENSORDB("HM5065_FOCUS_Get_AF_Inf\n");
    HM5065_write_cmos_sensor(0x0751,0x00);	
		HM5065_write_cmos_sensor(0x070A,0x00);
    HM5065_write_cmos_sensor(0x0700,0x03);
    HM5065_write_cmos_sensor(0x0701,0xFF);
		HM5065_write_cmos_sensor(0x070C,0x00);
		mDELAY(100);
    HM5065_write_cmos_sensor(0x070C,0x07);
		printk("HM5065_FOCUS_Get_AF_Inf: AF POS %02x%02x.\n", HM5065_read_cmos_sensor(0x0700), HM5065_read_cmos_sensor
			(0x0701));

    *pFeatureReturnPara32 = 0;
}
/*************************************************************************
//此函数变量未定义,请根据编译定义数据变量.
//prview 1280*960 
//16的整数倍 ; n*16*80/1280
//16的整数倍 ; n*16*60/960
//touch_x  为对应preview的横坐标[0-1280]
//touch_y  为对应preview的纵坐标[0-960]

*************************************************************************/ 
static UINT32 HM5065_FOCUS_Move_to(UINT32 a_u2MovePosition)//??how many bits for ov3640??
{
}
/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Get_AF_Status
* DESCRIPTION
*   GET af stauts
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/                        
static void HM5065_FOCUS_OVT_AFC_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
	UINT32 state_af=0;
	UINT32 i=0;
	*pFeatureReturnPara32 = SENSOR_AF_IDLE;
	//state_af = HM5065_read_cmos_sensor(0x07ae);
	//state_3029 = HM5065_read_cmos_sensor(0x3029);
	mDELAY(1);

	do
	{
	
		state_af = HM5065_read_cmos_sensor(0x07ae);
		//mDELAY(1);
		if (state_af==1)
		{
			*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;   
			  af_pos_h = HM5065_read_cmos_sensor(0x06F0);
      	     af_pos_l = HM5065_read_cmos_sensor(0x06F1);       
			break;
		}
		else if(state_af==0)
		{
		
					*pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
/*			switch (state_af)
			{
				case 0x70:
					*pFeatureReturnPara32 = SENSOR_AF_IDLE;
					break;
				case 0x00:
					*pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
					break;
				case 0x10:
					*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
					break;
				case 0x20:
					*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
					break;
				default:
					*pFeatureReturnPara32 = SENSOR_AF_SCENE_DETECTING; 
					break;
			}                                 */ 
		} 
		else
		{
			*pFeatureReturnPara32 = SENSOR_AF_ERROR;
		}
		i++;
		if(i>10)
			break;
	}while(1);
    HM5065SENSORDB("HM5065_AF_Status =%x\n",state_af);
	
}

/*************************************************************************
* FUNCTION
*   OV5640_FOCUS_OVT_AFC_Cancel_Focus
* DESCRIPTION
*   cancel af 
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/     
static void HM5065_FOCUS_OVT_AFC_Cancel_Focus()
{
	HM5065_write_cmos_sensor(0x0751,0x01);	

/*
	HM5065_write_cmos_sensor(0x070A,0x00);  
  	HM5065_write_cmos_sensor(0x0700,0x03);
	HM5065_write_cmos_sensor(0x0701,0xFF);
	HM5065_write_cmos_sensor(0x070C,0x00);
	HM5065_write_cmos_sensor(0x070C,0x07); 
 */
//	HM5065_write_cmos_sensor(FACE_LC,0x00);//disable
//	mDELAY(150);    
    HM5065SENSORDB("HM5065_Cancel FOCUS\n");

}

/*************************************************************************
* FUNCTION
*   HM5065WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/	
static void HM5065WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
/*		kal_uint32 color_r_gain_w = 0;
		kal_uint32 color_b_gain_w = 0;
		HM5065SENSORDB("[HM5065]enter HM5065WBcalibattion function:\n ");
		kal_uint8 temp = HM5065_read_cmos_sensor(0x350b); 
		
		if(temp>=0xb0)
		{	
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100;  
		}
		else if (temp>=0x70)
		{
			color_r_gain_w=color_r_gain *97/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else if (temp>=0x30)
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100; 
		}																																																																						
		//HM5065_write_cmos_sensor(0x3400,(color_r_gain_w & 0xff00)>>8);																																														
		//HM5065_write_cmos_sensor(0x3401,color_r_gain_w & 0xff); 			
		//HM5065_write_cmos_sensor(0x3404,(color_b_gain_w & 0xff00)>>8);																																														
		//HM5065_write_cmos_sensor(0x3405,color_b_gain_w & 0xff); 
		HM5065SENSORDB("[HM5065]exit HM5065WBcalibattion function:\n ");*/
}	
/*************************************************************************
* FUNCTION
*	HM5065Open
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
UINT32 HM5065Open(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	HM5065SENSORDB("[HM5065]enter HM5065Open function:\n ");
	//HM5065_write_cmos_sensor(0x3103,0x11);
	//HM5065_write_cmos_sensor(0x3008,0x82);
    mDELAY(10);
	for(i=0;i<3;i++)
	{
		sensor_id = (HM5065_read_cmos_sensor(0x0000) << 8) | HM5065_read_cmos_sensor(0x0001);
		HM5065SENSORDB("HM5065 READ ID :%x",sensor_id);
		if(sensor_id != HM5065_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	HM5065initalvariable();
	HM5065InitialSetting();
	HM5065SENSORDB("[HM5065]exit HM5065Open function:\n ");
	return ERROR_NONE;
}	/* HM5065Open() */

/*************************************************************************
* FUNCTION
*	HM5065Close
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
UINT32 HM5065Close(void)
{
  //CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* HM5065Close() */
/*************************************************************************
* FUNCTION
*	HM5065Preview
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
UINT32 HM5065Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	HM5065SENSORDB("[HM5065]enter HM5065Preview function:\n ");
	kal_uint32 zsdshutter = 0;
#if 0
	switch(CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   HM5065Sensor.zsd_flag=1;
			   HM5065PreviewSetting();
			   HM5065FullSizeCaptureSetting();	
			   zsdshutter=HM5065Sensor.PreviewShutter*2;
			//   HM5065WriteExpShutter(zsdshutter);
			   break;
		default:
			   HM5065PreviewSetting();
			   HM5065Sensor.zsd_flag=0;
			//   HM5065WriteExpShutter(HM5065Sensor.PreviewShutter);
			   break;
	}
	#endif

	if(sensor_config_data->SensorOperationMode==MSDK_SENSOR_OPERATION_MODE_VIDEO){
		HM5065Sensor.VideoMode= KAL_TRUE;  // MPEG4 Encode Mode
	}else{
		HM5065Sensor.VideoMode = KAL_FALSE;  
	}
	HM5065PreviewSetting();

#ifdef IMAGE_SENSOR_HV_MIRROR
	sensor_config_data->SensorImageMirror=IMAGE_HV_MIRROR;
#else
	sensor_config_data->SensorImageMirror=IMAGE_NORMAL;
#endif


	HM5065SetHVMirror(sensor_config_data->SensorImageMirror);

	
	//HM5065_FOCUS_OVT_AFC_Init();
	//HM5065_set_AE_mode(KAL_TRUE);
	//HM5065_set_AWB_mode(KAL_TRUE);
	//mDELAY(30);	
	HM5065_night_mode(HM5065Sensor.NightMode);
	mDELAY(30);	
	HM5065_FOCUS_OVT_AFC_Constant_Focus();
	HM5065SENSORDB("[HM5065]exit HM5065Preview function:\n ");	
	return ERROR_NONE ;
	
}	/* HM5065Preview() */
BOOL HM5065_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 totalGain = 0, exposureTime = 0;
	
	HM5065SENSORDB("[HM5065]HM5065_set_param_exposure_for_HDR para=%d:\n ", para);
    if (0 == HM5065Sensor.manualAEStart)
    {       
        HM5065_set_AE_mode(KAL_FALSE);//Manual AE enable
        spin_lock(&HM5065_drv_lock);	
        HM5065Sensor.manualAEStart = 1;
		spin_unlock(&HM5065_drv_lock);
    }
	totalGain = HM5065Sensor.currentAxDGain;
    exposureTime = HM5065Sensor.currentExposureTime;
	switch (para)
	{
	   case AE_EV_COMP_20:	//+2 EV
       case AE_EV_COMP_10:	// +1 EV
		   totalGain = totalGain<<1;
           exposureTime = exposureTime<<1;
           HM5065SENSORDB("[4EC] HDR AE_EV_COMP_20\n");
		 break;
	   case AE_EV_COMP_00:	// +0 EV
           HM5065SENSORDB("[4EC] HDR AE_EV_COMP_00\n");
		 break;
	   case AE_EV_COMP_n10:  // -1 EV
	   case AE_EV_COMP_n20:  // -2 EV
		   totalGain = totalGain >> 1;
           exposureTime = exposureTime >> 1;
           HM5065SENSORDB("[4EC] HDR AE_EV_COMP_n20\n");
		 break;
	   default:
		 break;//return FALSE;
	}

	if(totalGain > 0xf0)
		totalGain = 0xf0;
    HM5065WriteShutter(totalGain);
	HM5065WriteShutter(exposureTime);
	return TRUE;
}

UINT32 HM5065Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = 0;	
	kal_uint32 extshutter = 0;
	kal_uint32 color_r_gain = 0;
	kal_uint32 color_b_gain = 0;
	kal_uint32 readgain=0;
  UINT8 i=0;
	HM5065SENSORDB("[HM5065]enter HM5065Capture function:\n ");
  //HM5065_FOCUS_OVT_AFC_Cancel_Focus();
	//if(SENSOR_MODE_PREVIEW == HM5065Sensor.SensorMode )
	//{		
	//shutter=HM5065ReadShutter();
	//extshutter=HM5065ReadExtraShutter();
	//readgain=HM5065ReadSensorGain();
	//spin_lock(&HM5065_drv_lock);
	//HM5065Sensor.PreviewShutter=shutter;
	//HM5065Sensor.PreviewExtraShutter=extshutter;	
	//HM5065Sensor.SensorGain=readgain;
	//spin_unlock(&HM5065_drv_lock);	
	//HM5065_set_AE_mode(KAL_FALSE);
	//HM5065_set_AWB_mode(KAL_FALSE);
	//color_r_gain=((HM5065_read_cmos_sensor(0x3401)&0xFF)+((HM5065_read_cmos_sensor(0x3400)&0xFF)*256));  
	//color_b_gain=((HM5065_read_cmos_sensor(0x3405)&0xFF)+((HM5065_read_cmos_sensor(0x3404)&0xFF)*256)); 

	spin_lock(&HM5065_drv_lock);	
	HM5065Sensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&HM5065_drv_lock);
	 #if 1
	do
		{			
			if(HM5065_read_cmos_sensor(0x07ae)==1)
			{	
				break;
			};
			mDELAY(10);	
		}
		while(i++<20);
		af_pos_h = HM5065_read_cmos_sensor(0x06F0);
		af_pos_l = HM5065_read_cmos_sensor(0x06F1);	
	 #endif
   
	HM5065FullSizeCaptureSetting();
	//mDELAY(100);
    //HM5065WBcalibattion(color_r_gain,color_b_gain);      
    //HM5065WBcalibattion(color_r_gain,color_b_gain); 
    #if 1     
	HM5065SENSORDB("[HM5065]Before shutter=%d:\n",shutter);
	HM5065_write_cmos_sensor(0x070A, 0x00);//zoeadd
	HM5065_write_cmos_sensor(0x0734, af_pos_h & 0xFF);
	HM5065_write_cmos_sensor(0x0735, af_pos_l & 0xFF);
	HM5065_write_cmos_sensor(0x070C, 0x00);
	mDELAY(10);
	HM5065_write_cmos_sensor(0x070C, 0x05);
	//mDELAY(100);
	 #endif 
	/*if(HM5065Sensor.zsd_flag==0)
	{
		shutter = shutter*2;
	}*/
	if (SCENE_MODE_HDR == HM5065Sensor.sceneMode)
    {
		HM5065SENSORDB("[HM5065] HDR capture, record shutter gain\n");
        spin_lock(&HM5065_drv_lock);
        HM5065Sensor.currentExposureTime=HM5065ReadShutter();
		//HM5065Sensor.currentextshutter=extshutter;
		HM5065Sensor.currentAxDGain=HM5065ReadSensorGain();
		spin_unlock(&HM5065_drv_lock);
    }

	//mDELAY(200);
	//}
	HM5065SENSORDB("[HM5065]exit HM5065Capture function:\n ");
	return ERROR_NONE; 
}/* HM5065Capture() */

UINT32 HM5065ZSDPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = 0;	
	kal_uint32 extshutter = 0;
	kal_uint32 color_r_gain = 0;
	kal_uint32 color_b_gain = 0;
	kal_uint32 readgain=0;
	HM5065SENSORDB("[HM5065]enter ZSDPreview function:\n ");

	HM5065ZSDPreviewSetting();
	spin_lock(&HM5065_drv_lock);	
	HM5065Sensor.SensorMode= SENSOR_MODE_CAPTURE;
	spin_unlock(&HM5065_drv_lock);
  //HM5065WBcalibattion(color_r_gain,color_b_gain);      
	//HM5065SENSORDB("[HM5065]Before shutter=%d:\n",shutter);
	if (SCENE_MODE_HDR == HM5065Sensor.sceneMode)
    {
    spin_lock(&HM5065_drv_lock);
    HM5065Sensor.currentExposureTime=HM5065ReadShutter();
		//HM5065Sensor.currentextshutter=extshutter;
		HM5065Sensor.currentAxDGain=HM5065ReadSensorGain();
		spin_unlock(&HM5065_drv_lock);
    }
	mDELAY(100);
	HM5065SENSORDB("[HM5065]exit ZSDPreview function:\n ");
	return ERROR_NONE; 
}/* HM5065Capture() */


UINT32 HM5065GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	HM5065SENSORDB("[HM5065]enter HM5065GetResolution function:\n ");
	pSensorResolution->SensorPreviewWidth= HM5065_IMAGE_SENSOR_PREVIEW_WIDTH;
	pSensorResolution->SensorPreviewHeight= HM5065_IMAGE_SENSOR_PREVIEW_HEIGHT;
	pSensorResolution->SensorFullWidth= HM5065_IMAGE_SENSOR_QSXGA_WIDTH; 
	pSensorResolution->SensorFullHeight= HM5065_IMAGE_SENSOR_QSXGA_HEIGHT;
	pSensorResolution->SensorVideoWidth= HM5065_IMAGE_SENSOR_SVGA_WIDTH; 
	pSensorResolution->SensorVideoHeight= HM5065_IMAGE_SENSOR_SVGA_HEIGHT;
	HM5065SENSORDB("[HM5065]exit HM5065GetResolution function:\n ");
	return ERROR_NONE;
}	/* HM5065GetResolution() */

UINT32 HM5065GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	HM5065SENSORDB("[HM5065]enter HM5065GetInfo function:\n ");
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=HM5065_IMAGE_SENSOR_PREVIEW_WIDTH;//HM5065_IMAGE_SENSOR_SVGA_WIDTH;//HM5065_IMAGE_SENSOR_QSXGA_WITDH ;
			pSensorInfo->SensorPreviewResolutionY=HM5065_IMAGE_SENSOR_PREVIEW_HEIGHT;//HM5065_IMAGE_SENSOR_SVGA_HEIGHT;//HM5065_IMAGE_SENSOR_QSXGA_HEIGHT ;
			//pSensorInfo->SensorPreviewResolutionX=HM5065_IMAGE_SENSOR_SVGA_WIDTH;//HM5065_IMAGE_SENSOR_SVGA_WIDTH;//HM5065_IMAGE_SENSOR_QSXGA_WITDH ;
			//pSensorInfo->SensorPreviewResolutionY=HM5065_IMAGE_SENSOR_SVGA_HEIGHT;//HM5065_IMAGE_SENSOR_SVGA_HEIGHT;//HM5065_IMAGE_SENSOR_QSXGA_HEIGHT ;
			pSensorInfo->SensorCameraPreviewFrameRate=10;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=HM5065_IMAGE_SENSOR_PREVIEW_WIDTH ;
			pSensorInfo->SensorPreviewResolutionY=HM5065_IMAGE_SENSOR_PREVIEW_HEIGHT ;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}		 		
	pSensorInfo->SensorFullResolutionX= HM5065_IMAGE_SENSOR_QSXGA_WIDTH;
	pSensorInfo->SensorFullResolutionY= HM5065_IMAGE_SENSOR_QSXGA_HEIGHT;
	//pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=5;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;  
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	pSensorInfo->SensorInterruptDelayLines = 2;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
	pSensorInfo->CaptureDelayFrame = 1;   //4
	pSensorInfo->PreviewDelayFrame = 1;   //6
	pSensorInfo->VideoDelayFrame = 1; 		
	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->YUVAwbDelayFrame = 3;
	pSensorInfo->YUVEffectDelayFrame= 3; 
	pSensorInfo->AEShutDelayFrame= 0;
 	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = HM5065_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = HM5065_PV_GRAB_START_Y;   
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;  	
			pSensorInfo->SensorPacketECCOrder = 1;		
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = HM5065_FULL_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = HM5065_FULL_GRAB_START_Y;             
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =4; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->SensorWidthSampling = 0; 
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = HM5065_PV_GRAB_START_X; 
			pSensorInfo->SensorGrabStartY = HM5065_PV_GRAB_START_Y; 			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 4; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;	
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &HM5065SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));	
	HM5065SENSORDB("[HM5065]exit HM5065GetInfo function:\n ");	
	return ERROR_NONE;
}	/* HM5065GetInfo() */

UINT32 HM5065Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  HM5065SENSORDB("[HM5065]enter HM5065Control function:\n ");
	  spin_lock(&HM5065_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&HM5065_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			 HM5065Preview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 HM5065Capture(pImageWindow, pSensorConfigData);
	  	     break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 HM5065ZSDPreview(pImageWindow, pSensorConfigData);
			break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	HM5065SENSORDB("[HM5065]exit HM5065Control function:\n ");
	return ERROR_NONE;
}	/* HM5065Control() */

/* [TC] YUV sensor */	

//static UINT16 wbsave = AWB_MODE_AUTO;
//static UINT16 effectsave = MEFFECT_OFF;

BOOL HM5065_set_param_wb(UINT16 para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_set_param_wb function:\n ");
//	wbsave = para;
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.wb = para;
	spin_unlock(&HM5065_drv_lock);
    switch (para)
    {
      //  case AWB_MODE_OFF:
			//HM5065_write_cmos_sensor(0x01A4,0x04);
        //    break;

        case AWB_MODE_AUTO:
			HM5065_write_cmos_sensor(0x01A0,0x01);//awb enable
            break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy(13000K)
			HM5065_write_cmos_sensor(0x01A0,0x03);//MWB awb disable
			HM5065_write_cmos_sensor(0x01A1,0xef);//Rgain ff
			HM5065_write_cmos_sensor(0x01A2,0x60);//Ggain 40
			HM5065_write_cmos_sensor(0x01A3,0x00);//Bgain
            break;

        case AWB_MODE_DAYLIGHT: //sunny(7000K)
			HM5065_write_cmos_sensor(0x01A0,0x03);//MWB
			HM5065_write_cmos_sensor(0x01A1,0xC0);//Rgain
			HM5065_write_cmos_sensor(0x01A2,0x40);//Ggain
			HM5065_write_cmos_sensor(0x01A3,0x00);//Bgain
            break;

        case AWB_MODE_INCANDESCENT: //office(5000K)
			HM5065_write_cmos_sensor(0x01A0,0x03);//MWB
			HM5065_write_cmos_sensor(0x01A1,0xB0);//Rgain
			HM5065_write_cmos_sensor(0x01A2,0x40);//Ggain
			HM5065_write_cmos_sensor(0x01A3,0x20);//Bgain
            break;

        case AWB_MODE_TUNGSTEN: //home(2800K)
			HM5065_write_cmos_sensor(0x01A0,0x03);//MWB
			HM5065_write_cmos_sensor(0x01A1,0x90);//Rgain
			HM5065_write_cmos_sensor(0x01A2,0x40);//Ggain
			HM5065_write_cmos_sensor(0x01A3,0x40);//Bgain
            break;

        case AWB_MODE_FLUORESCENT://(4000K)
			HM5065_write_cmos_sensor(0x01A0,0x03);//MWB
			HM5065_write_cmos_sensor(0x01A1,0xA0);//Rgain
			HM5065_write_cmos_sensor(0x01A2,0x40);//Ggain
			HM5065_write_cmos_sensor(0x01A3,0x30);//Bgain
            break;

        default:
            break;
           // return FALSE;
    }
    return TRUE;


	HM5065SENSORDB("[HM5065]exit HM5065_set_param_wb function:\n ");
       return TRUE;
} /* HM5065_set_param_wb */
void HM5065_set_contrast(UINT16 para)
{   
    HM5065SENSORDB("[HM5065]enter HM5065_set_contrast function: %d\n ", para);
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             HM5065_write_cmos_sensor(0x0080,0x5c);
             break; 
        case ISP_CONTRAST_HIGH:
             HM5065_write_cmos_sensor(0x0080,0x7c);
             break; 
        case ISP_CONTRAST_MIDDLE:
             HM5065_write_cmos_sensor(0x0080,0x6c);
        default:
             //HM5065_write_cmos_sensor(0x0080,0x6D);//ZOE20150707
             break; 

    }
    HM5065SENSORDB("[HM5065]exit HM5065_set_contrast function:\n ");
    return;
}

void HM5065_set_brightness(UINT16 para)
{
    HM5065SENSORDB("[HM5065]enter HM5065_set_brightness function: %d\n ", para);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
			  HM5065_write_cmos_sensor(0x0082,0x44);
             break; 
        case ISP_BRIGHT_HIGH:  
			  HM5065_write_cmos_sensor(0x0082,0x84);
             break; 
        case ISP_BRIGHT_MIDDLE:
			 HM5065_write_cmos_sensor(0x0082,0x64);
             break;
        default:
            HM5065_write_cmos_sensor(0x0082,0x64);//ZOE20150707	
             break; 
    }

    HM5065SENSORDB("[HM5065]exit HM5065_set_brightness function:\n ");
    return;
}
void HM5065_set_saturation(UINT16 para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_set_saturation function: %d\n ", para);
    switch (para)
    {
        case ISP_SAT_HIGH:
            HM5065_write_cmos_sensor(0x0081 ,0x74);
            break; 
        case ISP_SAT_LOW:
            HM5065_write_cmos_sensor(0x0081 ,0x54);
            break; 
        case ISP_SAT_MIDDLE:
        default:
             HM5065_write_cmos_sensor(0x0081 ,0x64);  //ZOE20150707
             break; 
    }	

	HM5065SENSORDB("[HM5065]exit HM5065_set_saturation function:\n ");
     return;
}
void HM5065_set_scene_mode(UINT16 para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_set_scene_mode function:\n ");
	spin_lock(&HM5065_drv_lock);
	HM5065Sensor.sceneMode=para;
	spin_unlock(&HM5065_drv_lock);
	HM5065_set_param_wb(HM5065Sensor.wb);
    switch (para)
    { 

		case SCENE_MODE_NIGHTSCENE:
				
          	HM5065_night_mode(KAL_TRUE); 
			break;
        case SCENE_MODE_PORTRAIT:
			HM5065_night_mode(KAL_FALSE);
			//HM5065_write_cmos_sensor(0x004C, 0x08);
			//HM5065_write_cmos_sensor(0x006C, 0x08);
			//HM5065_write_cmos_sensor(0x0080, 0x6C);
			//HM5065_write_cmos_sensor(0x0081, 0x58);
				HM5065_write_cmos_sensor(0x00E8, 0x01);
				HM5065_write_cmos_sensor(0x00ED, 0x05);
				HM5065_write_cmos_sensor(0x0128, 0x00);
			//HM5065_write_cmos_sensor(0x0130, 0x00);
			//	HM5065_write_cmos_sensor(0x0143, 0x5F);
			//	HM5065_write_cmos_sensor(0x0144, 0x0D);
			//HM5065_write_cmos_sensor(0x015E, 0x40);
			//HM5065_write_cmos_sensor(0x015F, 0x00);
				HM5065_write_cmos_sensor(0x01A0, 0x01);
			//HM5065_write_cmos_sensor(0x02C2, 0x00);
			//HM5065_write_cmos_sensor(0x02C3, 0xC0);
			
			 		 
            break;
        case SCENE_MODE_LANDSCAPE:
		
			HM5065_night_mode(KAL_FALSE);
			//HM5065_write_cmos_sensor(0x004C, 0x10);
			//HM5065_write_cmos_sensor(0x006C, 0x10);
			//HM5065_write_cmos_sensor(0x0080, 0x78);
			//HM5065_write_cmos_sensor(0x0081, 0x68);
			HM5065_write_cmos_sensor(0x00E8, 0x01);
			HM5065_write_cmos_sensor(0x00ED, 0x05);
			HM5065_write_cmos_sensor(0x0128, 0x00);
			//HM5065_write_cmos_sensor(0x0130, 0x00);
			//HM5065_write_cmos_sensor(0x0143, 0x5F);
		//	HM5065_write_cmos_sensor(0x0144, 0x0D);
			//HM5065_write_cmos_sensor(0x015E, 0x40);
			//HM5065_write_cmos_sensor(0x015F, 0x00);
			HM5065_write_cmos_sensor(0x01A0, 0x01);
			//HM5065_write_cmos_sensor(0x02C2, 0x00);
			//HM5065_write_cmos_sensor(0x02C3, 0xC0);			 
             break;
        case SCENE_MODE_SUNSET:
	HM5065_night_mode(KAL_FALSE);
			//HM5065_write_cmos_sensor(0x004C, 0x05);
			//HM5065_write_cmos_sensor(0x006C, 0x05);
			//HM5065_write_cmos_sensor(0x0080, 0x6C);
			//HM5065_write_cmos_sensor(0x0081, 0x58);
			HM5065_write_cmos_sensor(0x00E8, 0x01);
			HM5065_write_cmos_sensor(0x00ED, 0x05);
			HM5065_write_cmos_sensor(0x0128, 0x00);
			//HM5065_write_cmos_sensor(0x0130, 0xFC);
			//HM5065_write_cmos_sensor(0x0143, 0x5F);
			//HM5065_write_cmos_sensor(0x0144, 0x0D);
			//HM5065_write_cmos_sensor(0x015E, 0x40);
			//HM5065_write_cmos_sensor(0x015F, 0x00); 		  
			HM5065_write_cmos_sensor(0x01A0, 0x03);//;WB_Cloudy	 
			HM5065_write_cmos_sensor(0x01A1, 0xE0);//;	
			HM5065_write_cmos_sensor(0x01A2, 0x40);//;	
			HM5065_write_cmos_sensor(0x01A3, 0x00);//;	
			//HM5065_write_cmos_sensor(0x02C2, 0x00);
			//HM5065_write_cmos_sensor(0x02C3, 0xC0);					 
            break;
        case SCENE_MODE_SPORTS:
  
			HM5065_night_mode(KAL_FALSE);

			//HM5065_write_cmos_sensor(0x004C, 0x08);
			//HM5065_write_cmos_sensor(0x006C, 0x08);
			//HM5065_write_cmos_sensor(0x0080, 0x6C);
			//HM5065_write_cmos_sensor(0x0081, 0x58);
			HM5065_write_cmos_sensor(0x00E8, 0x01);
			HM5065_write_cmos_sensor(0x00ED, 0x1E);
			HM5065_write_cmos_sensor(0x0128, 0x00);
			//HM5065_write_cmos_sensor(0x0130, 0x00);
			//HM5065_write_cmos_sensor(0x0143, 0x5C);
			//HM5065_write_cmos_sensor(0x0144, 0x09);
			//HM5065_write_cmos_sensor(0x015E, 0x40);
			//HM5065_write_cmos_sensor(0x015F, 0x00);
			HM5065_write_cmos_sensor(0x01A0, 0x01);
			//HM5065_write_cmos_sensor(0x02C2, 0x00);
			//HM5065_write_cmos_sensor(0x02C3, 0xC0);           		 
            break;
      case SCENE_MODE_HDR:
        /*      if (1 == HM5065Sensor.manualAEStart)
            {
                HM5065_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&HM5065_drv_lock);
            	HM5065Sensor.manualAEStart = 0;
                HM5065Sensor.currentExposureTime = 0;
                HM5065Sensor.currentAxDGain = 0;
				spin_unlock(&HM5065_drv_lock);
            }
            */
            break;
        case SCENE_MODE_OFF:
        default:
	
          	HM5065_night_mode(KAL_FALSE); 		 	   
			break;
        //default:
		//	return KAL_FALSE;
         //   break;
    }
	HM5065SENSORDB("[HM5065]exit HM5065_set_scene_mode function:\n ");
	return;
}
void HM5065_set_iso(UINT16 para)
{
    spin_lock(&HM5065_drv_lock);
    HM5065Sensor.isoSpeed = para;
    spin_unlock(&HM5065_drv_lock);   
	 switch (para)
		{
			case AE_ISO_100:
				 //ISO 100
	HM5065_write_cmos_sensor(0x015C, 0x3e);
	HM5065_write_cmos_sensor(0x015D, 0x00);
	HM5065_write_cmos_sensor(0x015E, 0x3e);
	HM5065_write_cmos_sensor(0x015F, 0x00);
	HM5065_write_cmos_sensor(0x02C0, 0x00);
	HM5065_write_cmos_sensor(0x02C1, 0x00);
	HM5065_write_cmos_sensor(0x02C2, 0x00);
	HM5065_write_cmos_sensor(0x02C3, 0x00);
				 break; 
			case AE_ISO_200:
				 //ISO 200
	HM5065_write_cmos_sensor(0x015C, 0x3e);
	HM5065_write_cmos_sensor(0x015D, 0x00);
	HM5065_write_cmos_sensor(0x015E, 0x3e);
	HM5065_write_cmos_sensor(0x015F, 0x00);
	HM5065_write_cmos_sensor(0x02C0, 0x00);
	HM5065_write_cmos_sensor(0x02C1, 0x80);
	HM5065_write_cmos_sensor(0x02C2, 0x00);
	HM5065_write_cmos_sensor(0x02C3, 0x80);
				 break; 
			case AE_ISO_400:
				 //ISO 400
	HM5065_write_cmos_sensor(0x015C, 0x40);
	HM5065_write_cmos_sensor(0x015D, 0x00);
	HM5065_write_cmos_sensor(0x015E, 0x40);
	HM5065_write_cmos_sensor(0x015F, 0x00);
	HM5065_write_cmos_sensor(0x02C0, 0x00);
	HM5065_write_cmos_sensor(0x02C1, 0x80);
	HM5065_write_cmos_sensor(0x02C2, 0x00);
	HM5065_write_cmos_sensor(0x02C3, 0x80);
				 break; 
			default:
			case AE_ISO_AUTO:
				 //ISO Auto
	HM5065_write_cmos_sensor(0x015C, 0x3E);
	HM5065_write_cmos_sensor(0x015D, 0x00);
	HM5065_write_cmos_sensor(0x015E, 0x40);
	HM5065_write_cmos_sensor(0x015F, 0x00);
	HM5065_write_cmos_sensor(0x02C0, 0x00);
	HM5065_write_cmos_sensor(0x02C1, 0x00);
	HM5065_write_cmos_sensor(0x02C2, 0x00);
	HM5065_write_cmos_sensor(0x02C3, 0x80);
				 break; 
		}	

    return;
}

BOOL HM5065_set_param_effect(UINT16 para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_set_param_effect function:\n ");
	 
//		effectsave = para;
	    switch (para)
    {
        case MEFFECT_OFF:
			HM5065_write_cmos_sensor(0x0380,0x00);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x00);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x01);
			HM5065_write_cmos_sensor(0x01A1,0x80);
			HM5065_write_cmos_sensor(0x01A2,0x80);
			HM5065_write_cmos_sensor(0x01A3,0x80);
			HM5065_write_cmos_sensor(0x01A5,0x3e);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x3e);
			HM5065_write_cmos_sensor(0x01A8,0x00);	
			HM5065_write_cmos_sensor(0x0049,0x16);	
#endif
	break;

        case MEFFECT_SEPIA:
			HM5065_write_cmos_sensor(0x0380,0x00);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x06);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x03);
			HM5065_write_cmos_sensor(0x01A1,0x80);
			HM5065_write_cmos_sensor(0x01A2,0x80);
			HM5065_write_cmos_sensor(0x01A3,0x80);
			HM5065_write_cmos_sensor(0x01A5,0x3e);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x3e);
			HM5065_write_cmos_sensor(0x01A8,0x00);
			HM5065_write_cmos_sensor(0x0049,0x14);	
#endif			
            break;

        case MEFFECT_NEGATIVE:
			HM5065_write_cmos_sensor(0x0380,0x01);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x00);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x01);
			HM5065_write_cmos_sensor(0x01A1,0x80);
			HM5065_write_cmos_sensor(0x01A2,0x80);
			HM5065_write_cmos_sensor(0x01A3,0x80);
			HM5065_write_cmos_sensor(0x01A5,0x3e);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x3e);
			HM5065_write_cmos_sensor(0x01A8,0x00);	
			HM5065_write_cmos_sensor(0x0049,0x08);	
#endif
            break;

        case MEFFECT_SEPIAGREEN:
			HM5065_write_cmos_sensor(0x0380,0x00);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x03);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x03);
			HM5065_write_cmos_sensor(0x01A1,0x00);
			HM5065_write_cmos_sensor(0x01A2,0xCF);
			HM5065_write_cmos_sensor(0x01A3,0x00);
			HM5065_write_cmos_sensor(0x01A5,0x00);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x45);
			HM5065_write_cmos_sensor(0x01A8,0x00);	
			HM5065_write_cmos_sensor(0x0049,0x14);	
#endif
            break;

        case MEFFECT_SEPIABLUE:

			HM5065_write_cmos_sensor(0x0380,0x00);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x04);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x00);
			HM5065_write_cmos_sensor(0x01A1,0x00);
			HM5065_write_cmos_sensor(0x01A2,0x00);
			HM5065_write_cmos_sensor(0x01A3,0x3F);
			HM5065_write_cmos_sensor(0x01A5,0x1E);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x3e);
			HM5065_write_cmos_sensor(0x01A8,0x00);
			HM5065_write_cmos_sensor(0x0049,0x14);	
#endif		
            break;
	case MEFFECT_MONO: //B&W
			HM5065_write_cmos_sensor(0x0380,0x00);
			HM5065_write_cmos_sensor(0x0381,0x00);
			HM5065_write_cmos_sensor(0x0382,0x00);
			HM5065_write_cmos_sensor(0x0384,0x05);
#if 0
			HM5065_write_cmos_sensor(0x01A0,0x03);
			HM5065_write_cmos_sensor(0x01A1,0x80);
			HM5065_write_cmos_sensor(0x01A2,0x80);
			HM5065_write_cmos_sensor(0x01A3,0x80);
			HM5065_write_cmos_sensor(0x01A5,0x3e);
			HM5065_write_cmos_sensor(0x01A6,0x00);
			HM5065_write_cmos_sensor(0x01A7,0x3e);
			HM5065_write_cmos_sensor(0x01A8,0x00);
			HM5065_write_cmos_sensor(0x0049,0x14);	
#endif
		break;

        default:
             return KAL_FALSE;
    }
	HM5065SENSORDB("[HM5065]exit HM5065_set_param_effect function:\n ");
    return KAL_TRUE;
} /* HM5065_set_param_effect */

BOOL HM5065_set_param_banding(UINT16 para)
{
	switch (para)
    {
        case AE_FLICKER_MODE_AUTO:
        case AE_FLICKER_MODE_50HZ:
            default:
						spin_lock(&HM5065_drv_lock);
						HM5065_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&HM5065_drv_lock);
						HM5065_write_cmos_sensor(0x0190,0x00);
						HM5065_write_cmos_sensor(0x019C,0x4B);
						HM5065_write_cmos_sensor(0x019D,0x20);
						HM5065SENSORDB("[HM5065]HM5065_set_param_banding selected 50Hz:\n ");
            break;
        case AE_FLICKER_MODE_60HZ:			
						spin_lock(&HM5065_drv_lock);
						HM5065_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&HM5065_drv_lock);
						HM5065_write_cmos_sensor(0x0190,0x00);
						HM5065_write_cmos_sensor(0x019C,0x4B);
						HM5065_write_cmos_sensor(0x019D,0xC0);
						HM5065SENSORDB("[HM5065]HM5065_set_param_banding selected 60Hz:\n ");
            break;
            return FALSE;
    }
        return TRUE;
} /* HM5065_set_param_banding */

BOOL HM5065_set_param_exposure(UINT16 para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_set_param_exposure function:\n ");
	HM5065SENSORDB("[HM5065]para=%d:\n",para);
	//spin_lock(&HM5065_drv_lock);
   if (SCENE_MODE_HDR == HM5065Sensor.sceneMode && 
    SENSOR_MODE_CAPTURE == HM5065Sensor.SensorMode)
   {
   	   //spin_unlock(&HM5065_drv_lock);
    //   HM5065_set_param_exposure_for_HDR(para);
    //   return TRUE;
    
	HM5065SENSORDB("[HM5065]CONTROLFLOW HDR enter HM5065_set_param_exposure function:\n ");
   }
   //spin_unlock(&HM5065_drv_lock);
	switch (para)
    {	
       case AE_EV_COMP_20:	                   
				
				HM5065_write_cmos_sensor(0x0082,0x84);

				break;
		case AE_EV_COMP_10:	                   
		
				HM5065_write_cmos_sensor(0x0082,0x74);

			  break;
		case AE_EV_COMP_00:

                HM5065_write_cmos_sensor(0x0082,0x64);
				//HM5065_write_cmos_sensor(0x3a1f, 0x18);//	; control zone L  
			  break;
   		 case AE_EV_COMP_n10:
			 
			 HM5065_write_cmos_sensor(0x0082,0x54);
			  break;
      	case AE_EV_COMP_n20:  // -2 EV

           	HM5065_write_cmos_sensor(0x0082,0x44);
        	 break;
       default:
           	HM5065_write_cmos_sensor(0x0082,0x64);
			break;//
						// return FALSE;
    }
	HM5065SENSORDB("[HM5065]exit HM5065_set_param_exposure function:\n ");
    return TRUE;
} /* HM5065_set_param_exposure */
#if 1//afc
BOOL HM5065_set_param_afmode(UINT16 para)
{
    switch (para)
    {
			case AF_MODE_AFS:
				HM5065_FOCUS_OVT_AFC_Single_Focus();
			break;
			case AF_MODE_AFC:
				HM5065_FOCUS_OVT_AFC_Constant_Focus();
			break;            
      	default:
      	return FALSE;
    }
        return TRUE;
} /* HM5065_set_param_banding */
#endif
UINT32 HM5065YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	HM5065SENSORDB("HM5065YUVSensorSetting:iCmd=%d,iPara=%d, %d \n",iCmd, iPara);
	HM5065SENSORDB("[HM5065]enter HM5065YUVSensorSetting function:\n ");
	switch (iCmd) {
		case FID_SCENE_MODE:
			HM5065_set_scene_mode(iPara);
	    	break; 	    
		case FID_AWB_MODE:
				HM5065_set_param_wb(iPara);
//				if(AWB_MODE_AUTO==iPara) HM5065_set_param_effect(effectsave);
			  break;
		case FID_COLOR_EFFECT:	    	    
				HM5065_set_param_effect(iPara);
//				if(MEFFECT_OFF==iPara) HM5065_set_param_wb(wbsave);
		 	  break;
		case FID_AE_EV:   
				HM5065_set_param_exposure(iPara);
		    break;
		case FID_AE_FLICKER:    	    	    
				HM5065_set_param_banding(iPara);
		 	  break;
		case FID_AE_SCENE_MODE: 
				if (iPara == AE_MODE_OFF) 
				{
					spin_lock(&HM5065_drv_lock);
		 			HM5065_AE_ENABLE = KAL_FALSE; 
					spin_unlock(&HM5065_drv_lock);
        }
        else 
        {
					spin_lock(&HM5065_drv_lock);
		 			HM5065_AE_ENABLE = KAL_TRUE; 
					spin_unlock(&HM5065_drv_lock);
	     	}
				HM5065_set_AE_mode(HM5065_AE_ENABLE);
        break; 
		case FID_ISP_CONTRAST:
            HM5065_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
            HM5065_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
            HM5065_set_saturation(iPara);
        break; 
    case FID_ZOOM_FACTOR:
   		    HM5065SENSORDB("FID_ZOOM_FACTOR:%d\n", iPara); 	    
					spin_lock(&HM5065_drv_lock);
	        zoom_factor = iPara; 
					spin_unlock(&HM5065_drv_lock);
            break; 
		case FID_AE_ISO:
            HM5065_set_iso(iPara);
            break;
#if 1 //afc
		case FID_AF_MODE:
	    	 HM5065_set_param_afmode(iPara);
					break;     
#endif            
	  default:
		 	      break;
	}
	HM5065SENSORDB("[HM5065]exit HM5065YUVSensorSetting function:\n ");
	  return TRUE;
}   /* HM5065YUVSensorSetting */

UINT32 HM5065YUVSetVideoMode(UINT16 u2FrameRate)
{
	HM5065SENSORDB("[HM5065]enter HM5065YUVSetVideoMode function:\n ");
	//sHM5065Sensor.VideoMode == KAL_TRUE;
	spin_lock(&HM5065_drv_lock);	
	HM5065Sensor.VideoMode= KAL_TRUE;
	spin_unlock(&HM5065_drv_lock);

	if (u2FrameRate == 30)
	{
		//;HM5065 1280x960,30fps
		//56Mhz, 224Mbps/Lane, 2Lane.
		HM5065SENSORDB("[HM5065]HM5065YUVSetVideoMode enter u2FrameRate == 30 setting  :\n ");	

		//HM5065_write_cmos_sensor(0x0040,0x01);	//	binning mode and subsampling mode for frame rate
		//		HM5065_write_cmos_sensor(0x0041,0x0A);	//	04 : VGA mode : 0A : self define ; 00 : 5M ;03:SVGA
		//		HM5065_write_cmos_sensor(0x0042,0x05);	//05	X:800 0x500=1280,0x0320=800
		//			HM5065_write_cmos_sensor(0x0043,0x00);	//00
		//			HM5065_write_cmos_sensor(0x0044,0x03);	//03	Y:600 0x03c0=960,0x0258=600
		//			HM5065_write_cmos_sensor(0x0045,0xc0);	//c0
		
		
				HM5065_write_cmos_sensor(0x00E8,0x00);
				HM5065_write_cmos_sensor(0x00C8,0x00);
				HM5065_write_cmos_sensor(0x00C9,0x1E);//30fps
				HM5065_write_cmos_sensor(0x00CA,0x01);
		

		if(HM5065_run_test_pattern)
		{
			HM5065_run_test_pattern=0;
			HM5065SetTestPatternMode(1);
		}
		HM5065SENSORDB("[HM5065]HM5065YUVSetVideoMode exit u2FrameRate == 30 setting  :\n ");
		}
    else if (u2FrameRate == 15)   
	{
		//;HM5065 1280x960,15fps
		//28Mhz, 112Mbps/Lane, 2Lane.
		HM5065SENSORDB("[HM5065]HM5065YUVSetVideoMode enter u2FrameRate == 15 setting  :\n ");	
		//HM5065_write_cmos_sensor(0x0040,0x01); 	//	binning mode and subsampling mode for frame rate
		//	HM5065_write_cmos_sensor(0x0041,0x0A); 	//	04 : VGA mode : 0A : self define ; 00 : 5M ;03:SVGA
		//		HM5065_write_cmos_sensor(0x0042,0x05); 	//05	X:800 0x500=1280,0x0320=800
		//		HM5065_write_cmos_sensor(0x0043,0x00); 	//00
		//		HM5065_write_cmos_sensor(0x0044,0x03); 	//03	Y:600 0x03c0=960,0x0258=600
		//	HM5065_write_cmos_sensor(0x0045, 0xC0); 	//

	
			HM5065_write_cmos_sensor(0x00E8,0x00);
			HM5065_write_cmos_sensor(0x00C8,0x00);
			HM5065_write_cmos_sensor(0x00C9,0x0F);//15fps
			HM5065_write_cmos_sensor(0x00CA,0x01);


		if(HM5065_run_test_pattern)
		{
			HM5065_run_test_pattern=0;
			HM5065SetTestPatternMode(1);
		}
		HM5065SENSORDB("[HM5065]HM5065YUVSetVideoMode exit u2FrameRate == 15 setting  :\n ");
	}   
    else 
    {
        HM5065SENSORDB("Wrong frame rate setting \n");
    } 
	HM5065SENSORDB("[HM5065]exit HM5065YUVSetVideoMode function:\n ");
    return TRUE; 
}

/**************************/
static void HM5065GetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
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

static void HM5065GetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{

	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=HM5065ReadShutter();
	Info->SensorAECur.AeCurGain=HM5065ReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((HM5065_read_cmos_sensor(0x3401)&&0xff)+((HM5065_read_cmos_sensor(0x3400)&&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((HM5065_read_cmos_sensor(0x3405)&&0xff)+((HM5065_read_cmos_sensor(0x3404)&&0xff)*256));

}
UINT32 HM5065MaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
	{
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		HM5065SENSORDB("HM5065MaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		HM5065SENSORDB("[HM5065]enter HM5065MaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = HM5065_IMAGE_SENSOR_PREVIEW_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - HM5065_IMAGE_SENSOR_PREVIEW_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&HM5065_drv_lock);
				HM5065Sensor.SensorMode= SENSOR_MODE_PREVIEW;
				HM5065Sensor.PreviewDummyLines = dummyLine;
				spin_unlock(&HM5065_drv_lock);
				HM5065SetDummy(HM5065Sensor.PreviewDummyPixels, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = HM5065_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - HM5065_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&HM5065_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&HM5065_drv_lock);
				HM5065SetDummy(0, dummyLine);			
				break;			
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:			
				pclk = 90000000;
				lineLength = HM5065_IMAGE_SENSOR_QSXGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - HM5065_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&HM5065_drv_lock);
				HM5065Sensor.CaptureDummyLines = dummyLine;
				HM5065Sensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&HM5065_drv_lock);
				HM5065SetDummy(HM5065Sensor.CaptureDummyPixels, dummyLine);			
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
		HM5065SENSORDB("[HM5065]exit HM5065MaxFramerateByScenario function:\n ");
		return ERROR_NONE;
	}
UINT32 HM5065GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{
	HM5065SENSORDB("[HM5065]enter HM5065GetDefaultFramerateByScenario function:\n ");
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
	HM5065SENSORDB("[HM5065]exit HM5065GetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void HM5065_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	HM5065SENSORDB("[HM5065]enter HM5065_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	HM5065SENSORDB("[HM5065]HM5065_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	HM5065SENSORDB("[HM5065]exit HM5065_get_AEAWB_lock function:\n ");
}
void HM5065_GetDelayInfo(UINT32 delayAddr)
{
	HM5065SENSORDB("[HM5065]enter HM5065_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=1;
	pDelayInfo->EffectDelay=3;
	pDelayInfo->AwbDelay=3;
	pDelayInfo->AFSwitchDelayFrame=50;
	HM5065SENSORDB("[HM5065]exit HM5065_GetDelayInfo function:\n ");
}
void HM5065_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
	HM5065SENSORDB("[HM5065]enter HM5065_AutoTestCmd function:\n ");
	switch(*cmd)
	{
		case YUV_AUTOTEST_SET_SHADDING:
			HM5065SENSORDB("YUV_AUTOTEST_SET_SHADDING:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAMMA:
			HM5065SENSORDB("YUV_AUTOTEST_SET_GAMMA:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_AE:
			HM5065SENSORDB("YUV_AUTOTEST_SET_AE:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_SHUTTER:
			HM5065SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAIN:
			HM5065SENSORDB("YUV_AUTOTEST_SET_GAIN:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			*para=8228;
			break;
		default:
			HM5065SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	HM5065SENSORDB("[HM5065]exit HM5065_AutoTestCmd function:\n ");
}
void HM5065_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&HM5065_drv_lock);
          HM5065Sensor.userAskAeLock = TRUE;
          spin_unlock(&HM5065_drv_lock);
          HM5065_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&HM5065_drv_lock);
          HM5065Sensor.userAskAeLock = FALSE;
          spin_unlock(&HM5065_drv_lock);
          HM5065_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&HM5065_drv_lock);
          HM5065Sensor.userAskAwbLock = TRUE;
          spin_unlock(&HM5065_drv_lock);
      //    HM5065_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&HM5065_drv_lock);
          HM5065Sensor.userAskAwbLock = FALSE;
          spin_unlock(&HM5065_drv_lock);
      //    HM5065_set_AWB_mode(KAL_TRUE);
      break;
      default:
      	break;
   }
   return;
}

UINT32 HM5065FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
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
	HM5065SENSORDB("[HM5065][HM5065FeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=HM5065_IMAGE_SENSOR_QSXGA_WIDTH;
			*pFeatureReturnPara16=HM5065_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=HM5065_FULL_PERIOD_PIXEL_NUMS + HM5065Sensor.CaptureDummyPixels;
					*pFeatureReturnPara16=HM5065_FULL_PERIOD_LINE_NUMS + HM5065Sensor.CaptureDummyLines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=HM5065_PV_PERIOD_PIXEL_NUMS + HM5065Sensor.PreviewDummyPixels;
					*pFeatureReturnPara16=HM5065_PV_PERIOD_LINE_NUMS + HM5065Sensor.PreviewDummyLines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = HM5065Sensor.ZsdturePclk * 1000 *100;	 //unit: Hz				
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = HM5065Sensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
            HM5065GetExifInfo(*pFeatureData32);
            break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			HM5065_night_mode((BOOL) *pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_GAIN:
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			HM5065_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = HM5065_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &HM5065SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			HM5065SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			HM5065_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=HM5065_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			HM5065YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            HM5065_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    HM5065YUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			HM5065GetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			HM5065GetCurAeAwbInfo(*pFeatureData32);			
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HM5065MaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HM5065GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			HM5065_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			HM5065SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
			HM5065_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_AUTOTEST_CMD:
			HM5065SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			HM5065_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_INITIALIZE_AF:           
             break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            HM5065_FOCUS_Move_to(*pFeatureData16);
            break;
        case SENSOR_FEATURE_GET_AF_STATUS:
            HM5065_FOCUS_OVT_AFC_Get_AF_Status(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_AF_INF:
            HM5065_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
        case SENSOR_FEATURE_GET_AF_MACRO:
            HM5065_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;            
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
			HM5065_FOCUS_OVT_AFC_Constant_Focus();
			 break;
        case SENSOR_FEATURE_SET_AF_WINDOW:       
			HM5065_FOCUS_Set_AF_Window(*pFeatureData32);
            break;
        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			HM5065_FOCUS_OVT_AFC_Single_Focus();
            break;	
        case SENSOR_FEATURE_CANCEL_AF:
            HM5065_FOCUS_OVT_AFC_Cancel_Focus();
            break;					
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            HM5065_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            HM5065_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_SET_AE_WINDOW:
            HM5065SENSORDB("AE zone addr = 0x%x\n",*pFeatureData32);			
            HM5065_FOCUS_Set_AE_Window(*pFeatureData32);
            break; 
		default:
			HM5065SENSORDB("HM5065FeatureControl:default \n");
			break;			
	}
	HM5065SENSORDB("[HM5065]exit HM5065FeatureControl function:\n ");
	return ERROR_NONE;
}	/* HM5065FeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncHM5065=
{
	HM5065Open,
	HM5065GetInfo,
	HM5065GetResolution,
	HM5065FeatureControl,
	HM5065Control,
	HM5065Close,
};

UINT32 HM5065_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncHM5065;
	return ERROR_NONE;
}	/* SensorInit() */



