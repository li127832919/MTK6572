#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "tvp5150_yuv_Sensor.h"
#include "tvp5150_yuv_Camera_Sensor_para.h"
#include "tvp5150_yuv_CameraCustomized.h"

#define TVP5150_DEBUG
#ifdef TVP5150_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);

//static int sensor_id_fail = 0; 
static kal_uint32 zoom_factor = 0; 
static TVP5150_SENSOR_INFO_ST TVP5150_sensor;
static TVP5150_OPERATION_STATE_ST TVP5150_op_state;

static DEFINE_SPINLOCK(tvp5150_drv_lock);

inline TVP5150_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[1] = {(char)(addr & 0xFF)};

	iReadRegI2C(puSendCmd, 1, (u8*)&get_byte, 1, TVP5150_READ_ID_0);

    return get_byte;
}

inline int TVP5150_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(puSendCmd, 2, TVP5150_WRITE_ID_0);
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/
#define WINMO_USE 0

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/


#define	TVP5150_LIMIT_EXPOSURE_LINES				(1253)
#define	TVP5150_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	TVP5150_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	TVP5150_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */



static kal_uint8 TVP5150_exposure_line_h = 0, TVP5150_exposure_line_l = 0,TVP5150_extra_exposure_line_h = 0, TVP5150_extra_exposure_line_l = 0;

static kal_bool TVP5150_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool TVP5150_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool TVP5150_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 TVP5150_dummy_pixels=0, TVP5150_dummy_lines=0;
kal_uint32 TVP5150_FULL_dummy_pixels = 0;
kal_uint32 TVP5150_FULL_dummy_lines = 0;


static kal_uint16 TVP5150_exposure_lines=0, TVP5150_extra_exposure_lines = 0;


static kal_int8 TVP5150_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 TVP5150_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 TVP5150_Capture_Max_Gain16= 6*16;
static kal_uint16 TVP5150_Capture_Gain16=0 ;    
static kal_uint16 TVP5150_Capture_Shutter=0;
static kal_uint16 TVP5150_Capture_Extra_Lines=0;

static kal_uint16  TVP5150_PV_Dummy_Pixels =0, TVP5150_Capture_Dummy_Pixels =0, TVP5150_Capture_Dummy_Lines =0;
static kal_uint16  TVP5150_PV_Gain16 = 0;
static kal_uint16  TVP5150_PV_Shutter = 0;
static kal_uint16  TVP5150_PV_Extra_Lines = 0;

kal_uint16 TVP5150_sensor_gain_base=0,TVP5150_FAC_SENSOR_REG=0,TVP5150_iTVP5150_Mode=0,TVP5150_max_exposure_lines=0;
kal_uint32 TVP5150_capture_pclk_in_M=27,TVP5150_preview_pclk_in_M=27,TVP5150_PV_dummy_pixels=0,TVP5150_PV_dummy_lines=0,TVP5150_isp_master_clock=0;
static kal_uint32  TVP5150_preview_pclk = 27, TVP5150_capture_pclk = 27;
kal_bool TVP5150_Night_mode = KAL_FALSE;
kal_bool TVP5150_Y_Target_L = 64; 
kal_bool TVP5150_Y_Target_H = 72; 


static kal_uint32  TVP5150_sensor_pclk=27;
kal_bool TVP5150_VEDIO_MPEG4 = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4);

kal_bool first_enter_preview = KAL_FALSE;

static kal_bool TVP5150_AWB_ENABLE = KAL_TRUE; 
static kal_bool TVP5150_AE_ENABLE = KAL_TRUE; 


//SENSOR_REG_STRUCT TVP5150SensorCCT[FACTORY_END_ADDR]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
//SENSOR_REG_STRUCT TVP5150SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
//	camera_para.SENSOR.cct	SensorCCT	=> SensorCCT
//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT TVP5150SensorConfigData;
#define SENSOR_CORE_PCLK	83200000	//48M PCLK Output 78000000 

void TVP5150_set_dummy(kal_uint16 pixels, kal_uint16 lines)
	{
		kal_uint32 line_length;
		kal_uint32 frame_rate;
		kal_uint32 base_shutter_50Hz,base_shutter_60Hz;
	
		if(KAL_TRUE == TVP5150_op_state.is_PV_mode)
		{
#if 0
			//for preview
			line_length = TVP5150_PV_PERIOD_PIXEL_NUMS + dummy_pixels;
			
			TVP5150_write_cmos_sensor(0x98E, 0x480A);		//Line Length (A)
			TVP5150_write_cmos_sensor(0x990, line_length);	//		
			
			TVP5150_write_cmos_sensor(0x098E, 0x8400);			// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);			// MCU_DATA_0
			
			frame_rate = SENSOR_CORE_PCLK*10/line_length/(TVP5150_PV_PERIOD_LINE_NUMS + dummy_lines);
			
			base_shutter_50Hz = (frame_rate*TVP5150_PV_PERIOD_LINE_NUMS/50+5)/2/10;;
			base_shutter_60Hz = (frame_rate*TVP5150_PV_PERIOD_LINE_NUMS/60+5)/2/10;;
			TVP5150_write_cmos_sensor(0x98E, 0x481A);				//fd_period_50Hz (A)
			TVP5150_write_cmos_sensor(0x990, base_shutter_50Hz);	//		
			TVP5150_write_cmos_sensor(0x98E, 0x481C);				//fd_period_60Hz (A)
			TVP5150_write_cmos_sensor(0x990, base_shutter_60Hz);	//	
#endif
		}
		else
		{ 
			//for capture
			
			line_length = TVP5150_FULL_PERIOD_PIXEL_NUMS + pixels;
			
			TVP5150_write_cmos_sensor(0x98E, 0x4837);		//Line Length (B)
			TVP5150_write_cmos_sensor(0x990, line_length);	//		
			
			TVP5150_write_cmos_sensor(0x098E, 0x8400);			// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);			// MCU_DATA_0
	
			{
				kal_uint16 temp=0;
				while(temp <60)
				{
					TVP5150_write_cmos_sensor(0x098E, 0x8400);	  // MCU_DATA_0
					if(0==TVP5150_read_cmos_sensor(0x990))
					{
					 break;
					}
					mdelay(4);//DELAY=100
					temp+=1;
				}
			}
			
			frame_rate = SENSOR_CORE_PCLK*10/line_length/(TVP5150_FULL_PERIOD_LINE_NUMS + lines);
			
			base_shutter_50Hz = (frame_rate*TVP5150_FULL_PERIOD_LINE_NUMS/50+5)/2/10;;
			base_shutter_60Hz = (frame_rate*TVP5150_FULL_PERIOD_LINE_NUMS/60+5)/2/10;;
			TVP5150_write_cmos_sensor(0x98E, 0x4847);				//fd_period_50Hz (B)
			TVP5150_write_cmos_sensor(0x990, base_shutter_50Hz);	//		
			TVP5150_write_cmos_sensor(0x98E, 0x4849);				//fd_period_60Hz (B)
			TVP5150_write_cmos_sensor(0x990, base_shutter_60Hz);	//	
		}
		return;
	
	}

 

kal_uint16 TVP5150_read_TVP5150_gain(void)
{
    kal_uint8  temp_reg;
    kal_uint16 sensor_gain;

    temp_reg=TVP5150_read_cmos_sensor(0x3000);  


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
}  /* TVP5150_read_TVP5150_gain */


kal_uint32 TVP5150_read_shutter(void)
{
	kal_uint16 temp_reg1;
return 0;	
	temp_reg1 = TVP5150_read_cmos_sensor(0x3012);    // AEC[b15~b8]
	/* Backup the preview mode last shutter & sensor gain. */
	spin_lock(&tvp5150_drv_lock);
	TVP5150_sensor.pv_shutter = temp_reg1;
	spin_unlock(&tvp5150_drv_lock);
	
return TVP5150_sensor.pv_shutter;
}    /* TVP5150_read_shutter */

void TVP5150_write_TVP5150_gain(kal_uint16 gain)
{    
    kal_uint16 temp_reg;
   
	RETAILMSG(1, (TEXT("TVP5150 write gain: %d\r\n"), gain));
   
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
  
   TVP5150_write_cmos_sensor(0x3000,temp_reg);
}  /* TVP5150_write_TVP5150_gain */

static void TVP5150_write_shutter(kal_uint16 shutter)
{    
}    /* TVP5150_write_shutter */


/*************************************************************************
* FUNCTION
*	TVP5150_NightMode
*
* DESCRIPTION
*	This function night mode of TVP5150.
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
void TVP5150_night_mode(kal_bool enable)
{
	//kal_uint16 night = 0;
	//kal_uint16 temp=TVP5150_read_cmos_sensor(0x3302);
	spin_lock(&tvp5150_drv_lock);
	TVP5150_Night_mode = enable;
	spin_unlock(&tvp5150_drv_lock);

	if (TVP5150_sensor_cap_state == KAL_TRUE) {
		return ;	//If capture mode, return directely.
	}
   if(TVP5150_VEDIO_encode_mode==KAL_FALSE)
   	{
	if (enable) {	
	#ifdef TVP5150_DEBUG
		//SENSORDB("[TVP5150YUV] nightmode \n");
	#endif
	    TVP5150_write_cmos_sensor(0x98E, 0x480A);		//Line Length (A)
	    TVP5150_write_cmos_sensor(0x990, TVP5150_PV_PERIOD_PIXEL_NUMS);	//
		TVP5150_write_cmos_sensor(0x098E, 0x682F);	// MCU_ADDRESS
		TVP5150_write_cmos_sensor(0x0990, 0x0110);	// MCU_DATA_0// gain
	
	    TVP5150_write_cmos_sensor(0x098E, 0x6815);	  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
		TVP5150_write_cmos_sensor(0x0990, 0x0014);	  // MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x6817);	  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
		TVP5150_write_cmos_sensor(0x0990, 0x0018);	  // MCU_DATA_0																	   	    
	}
	else {
     //[Normal mode 10-29.1fps] 
	#ifdef TVP5150_DEBUG
        SENSORDB("[TVP5150YUV] automode \n");
	#endif
	    TVP5150_write_cmos_sensor(0x98E, 0x480A);		//Line Length (A)
	    TVP5150_write_cmos_sensor(0x990, TVP5150_PV_PERIOD_PIXEL_NUMS);	//
		TVP5150_write_cmos_sensor(0x098E, 0x6815);	  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
		TVP5150_write_cmos_sensor(0x0990, 0x000A);	  // MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x6817);	  // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
		TVP5150_write_cmos_sensor(0x0990, 0x000C);	  // MCU_DATA_0
	}
   	}
	
}	/* TVP5150_NightMode */


/* Register setting from capture to preview. */
static void TVP5150_set_VGA_mode(void)
{
	//-----------------------------------
	//From capture to preview
	//-----------------------------------

	//-------------------------------------------------------------------------------
    // PLL MY_OUTPUT clock(fclk)
    // fclk = (0x40 - 0x300E[5:0]) x N x Bit8Div x MCLK / M, where
    //      N = 1, 1.5, 2, 3 for 0x300F[7:6] = 0~3, respectively
    //      M = 1, 1.5, 2, 3 for 0x300F[1:0] = 0~3, respectively
    //      Bit8Div = 1, 1, 4, 5 for 0x300F[5:4] = 0~3, respectively
    // Sys Clk = fclk / Bit8Div / SenDiv
    // Sensor MY_OUTPUT clock(DVP PCLK)
    // DVP PCLK = ISP CLK / DVPDiv, where
    //      ISP CLK =  fclk / Bit8Div / SenDiv / CLKDiv / 2, where
    //          Bit8Div = 1, 1, 4, 5 for 0x300F[5:4] = 0~3, respectively
    //          SenDiv = 1, 2 for 0x3010[4] = 0 or 1 repectively
    //          CLKDiv = (0x3011[5:0] + 1)
    //      DVPDiv = 0x304C[3:0] * (2 ^ 0x304C[4]), if 0x304C[3:0] = 0, use 16 instead
    //
    // Base shutter calculation
    //      60Hz: (1/120) * ISP Clk / QXGA_MODE_WITHOUT_DUMMY_PIXELS
    //      50Hz: (1/100) * ISP Clk / QXGA_MODE_WITHOUT_DUMMY_PIXELS
    //-------------------------------------------------------------------------------

	    //26MHz Mclk,22.5Mhz Pclk 30fps	   

	    TVP5150_write_cmos_sensor(0x300e,0x33); 
	    TVP5150_write_cmos_sensor(0x300f,0x21); 
	    TVP5150_write_cmos_sensor(0x3010,0x20); 
	    TVP5150_write_cmos_sensor(0x3011,0x00); 
	    TVP5150_write_cmos_sensor(0x304c,0x85);      //reserve

	    //size:640x480	

#if 0
	    TVP5150_write_cmos_sensor(0x3012,0x10);					//soft reset,  sensor array resolution:XGA(1024X768) 	
	    
		//TVP5150_write_cmos_sensor(0x3023,0x07);					// ;05 VS[7:0]:Vertical start point of array																										
              TVP5150_write_cmos_sensor(0x3023,0x06);					// ;05 VS[7:0]:Vertical start point of array																												
		TVP5150_write_cmos_sensor(0x3026,0x03);					//VH[15:8]:Vertical height,  																									
		TVP5150_write_cmos_sensor(0x3027,0x04);					//VH[7:0]	VH=772																										
		TVP5150_write_cmos_sensor(0x302a,0x03);					//VTS[15:8]:Vertical total size for 1 frame 																										
		TVP5150_write_cmos_sensor(0x302b,0x10);					//VTS[7:0]	VTS=784																									
		TVP5150_write_cmos_sensor(0x3075,0x24);				  //Vsync start point and width 																										
		TVP5150_write_cmos_sensor(0x300d,0x01);				  // pclk always output,																											
		//TVP5150_write_cmos_sensor(0x30d7,0x90);				  //reserve 																									
		TVP5150_write_cmos_sensor(0x30d7,0x80);				  //reserve 																									
		TVP5150_write_cmos_sensor(0x3069,0x04);				  //;44 ;BLC																									
		TVP5150_write_cmos_sensor(0x303e,0x00);				  //AVH[11:8]:Average window vertical height																											
		TVP5150_write_cmos_sensor(0x303f,0xc0);				  //AVH[7:0]	,AVH=192(192*4=768)
		//need check if need write this register 0x304c
		TVP5150_write_cmos_sensor(0x304c,0x85);				  //reserve 			
		
		TVP5150_write_cmos_sensor(0x3302,0xef);				  //  sde, uv_adj, gam, awb,scale_en																				
		TVP5150_write_cmos_sensor(0x335f,0x34);				  // SIZE_IN_MISC:Vsize_in[10:8],Hsize_in[11:8] 																										
		TVP5150_write_cmos_sensor(0x3360,0x0c);				  // Hsize_in[7:0]	,Hsize_in=1036																									
		TVP5150_write_cmos_sensor(0x3361,0x04);				  // Vsize_in[7:0]	,Vsize_in=772																									
		TVP5150_write_cmos_sensor(0x3362,0x12);				  // SIZE_OUT_MISC:Zoom_out output size:Vsize_out[10:8],Hsize_out[11:8] 																				
		TVP5150_write_cmos_sensor(0x3363,0x88);				  // Hsize_out[7:0] for zoom_out	Hsize_out=648																										
		TVP5150_write_cmos_sensor(0x3364,0xE4);				  // Vsize_out[7:0] for zoom_out	Vsize_out=484																									
		TVP5150_write_cmos_sensor(0x3403,0x42);				  //bit[7:4]:x start=4,	 bit[3:0]:y start	=2																										
		TVP5150_write_cmos_sensor(0x3088,0x12);				  //x_output_size, isp_xout[15:8]:																							
		TVP5150_write_cmos_sensor(0x3089,0x80);				  // isp_xout[7:0]	:640																									
		TVP5150_write_cmos_sensor(0x308a,0x01);				  // y_output_size,isp_yout[15:8]																						
		TVP5150_write_cmos_sensor(0x308b,0xe0);				  // isp_yout[7:0]	:480	
		TVP5150_write_cmos_sensor(0x3366,0x15);		//reserve
#else
              TVP5150_write_cmos_sensor(0x3012, 0x10);
              TVP5150_write_cmos_sensor(0x3023, 0x06);
              TVP5150_write_cmos_sensor(0x3026, 0x03);
              TVP5150_write_cmos_sensor(0x3027, 0x04);
              TVP5150_write_cmos_sensor(0x302a, 0x03);
              TVP5150_write_cmos_sensor(0x302b, 0x10);
              TVP5150_write_cmos_sensor(0x3075, 0x24);
              TVP5150_write_cmos_sensor(0x300d, 0x01);
              TVP5150_write_cmos_sensor(0x30d7, 0x90);
              TVP5150_write_cmos_sensor(0x3069, 0x04);
              TVP5150_write_cmos_sensor(0x303e, 0x00);
              TVP5150_write_cmos_sensor(0x303f, 0xc0);
              TVP5150_write_cmos_sensor(0x3302, 0xef);
              TVP5150_write_cmos_sensor(0x335f, 0x34);
              TVP5150_write_cmos_sensor(0x3360, 0x0c);
              TVP5150_write_cmos_sensor(0x3361, 0x04);
              TVP5150_write_cmos_sensor(0x3362, 0x12);
              TVP5150_write_cmos_sensor(0x3363, 0x88);
              TVP5150_write_cmos_sensor(0x3364, 0xe4);
              TVP5150_write_cmos_sensor(0x3403, 0x42);
              TVP5150_write_cmos_sensor(0x3088, 0x12);
              TVP5150_write_cmos_sensor(0x3089, 0x80);
              TVP5150_write_cmos_sensor(0x308a, 0x01);
              TVP5150_write_cmos_sensor(0x308b, 0xe0);
		
#endif 

    //    TVP5150_write_cmos_sensor(0x308d,0x04);				  //reset block sleep enable																											
	//    TVP5150_write_cmos_sensor(0x3086,0x03);				  //sleep on																											
	 //   TVP5150_write_cmos_sensor(0x3086,0x00);				  // sleep off	
}
void test_pclk_52M(void)
{
	TVP5150_write_cmos_sensor(0x0014, 0x0449);	  //PLL Control: BYPASS PLL = 9541
	TVP5150_write_cmos_sensor(0x0010, 0x0110 ); // PLL_DIVIDERS//MCLK = 26MHzÊ±£¬PCLK = 52MHz

	TVP5150_write_cmos_sensor(0x0012, 0x0070 ); // PLL_P_DIVIDERS
	TVP5150_write_cmos_sensor(0x0014, 0x904A);	  //PLL Control: TEST_BYPASS on = 9541

	TVP5150_write_cmos_sensor(0x002A, 0x7474 ); // PLL_P4_P5_P6_DIVIDERS
	TVP5150_write_cmos_sensor(0x001E, 0x0777 ); // PAD_SLEW_PAD_CONFIG
	TVP5150_write_cmos_sensor(0x001E, 0x0074 ); // PAD_SLEW_PAD_CONFIG
	TVP5150_write_cmos_sensor(0x3B84, 0x019C ); //I2C Master Clock Divider = 412
	TVP5150_write_cmos_sensor(0x0018, 0x4028 ); // STANDBY_CONTROL_AND_STATUS

    mdelay(30);//DELAY=30
    // wait for FW initialization complete

    //POLL_FIELD=STANDBY_CONTROL_AND_STATUS,STANDBY_DONE,==1,DELAY=10,TIMEOUT=100 

    //REGWIZARD OUTPUT
    TVP5150_write_cmos_sensor(0x98E, 0x4800);	//Row Start (A)
    TVP5150_write_cmos_sensor(0x990, 0x0010);	//		= 16
    TVP5150_write_cmos_sensor(0x98E, 0x4802);	//Column Start (A)
    TVP5150_write_cmos_sensor(0x990, 0x0010);	//		= 16
    TVP5150_write_cmos_sensor(0x98E, 0x4804);	//Row End (A)
    TVP5150_write_cmos_sensor(0x990, 0x062D);	//		= 1581
	TVP5150_write_cmos_sensor(0x98E, 0x4806);	//Column End (A)
	TVP5150_write_cmos_sensor(0x990, 0x082D);	//		= 2093
	TVP5150_write_cmos_sensor(0x98E, 0x4808);	//Base Frame Lines (A)
	TVP5150_write_cmos_sensor(0x990, 0x0359);	//		= 857
	TVP5150_write_cmos_sensor(0x98E, 0x480A);	//Line Length (A)
	TVP5150_write_cmos_sensor(0x990, 0x0D08);	//		= 3336
	TVP5150_write_cmos_sensor(0x98E, 0x480C);	//Fine Correction (A)
	TVP5150_write_cmos_sensor(0x990, 0x0399);	//		= 921
	TVP5150_write_cmos_sensor(0x98E, 0x480E);	//Row Speed (A)
	TVP5150_write_cmos_sensor(0x990, 0x0111);	//		= 273
	TVP5150_write_cmos_sensor(0x98E, 0x4810);	//Read Mode (A)
	TVP5150_write_cmos_sensor(0x990, 0x046C);	//		= 1132
	TVP5150_write_cmos_sensor(0x98E, 0x4812);	//Fine IT Min (A)
	TVP5150_write_cmos_sensor(0x990, 0x0510);	//		= 1296
	TVP5150_write_cmos_sensor(0x98E, 0x4814);	//Fine IT Max Margin (A)
	TVP5150_write_cmos_sensor(0x990, 0x01BA);	//		= 442
	TVP5150_write_cmos_sensor(0x98E, 0x482D);	//Row Start (B)
	TVP5150_write_cmos_sensor(0x990, 0x0018);	//		= 24
	TVP5150_write_cmos_sensor(0x98E, 0x482F);	//Column Start (B)
	TVP5150_write_cmos_sensor(0x990, 0x0018);	//		= 24
	TVP5150_write_cmos_sensor(0x98E, 0x4831);	//Row End (B)
	TVP5150_write_cmos_sensor(0x990, 0x0627);	//		= 1575
	TVP5150_write_cmos_sensor(0x98E, 0x4833);	//Column End (B)
	TVP5150_write_cmos_sensor(0x990, 0x0827);	//		= 2087
	TVP5150_write_cmos_sensor(0x98E, 0x4835);	//Base Frame Lines (B)
	TVP5150_write_cmos_sensor(0x990, 0x065D);	//		= 1629
	TVP5150_write_cmos_sensor(0x98E, 0x4837);	//Line Length (B)
	TVP5150_write_cmos_sensor(0x990, 0x1A99);	//		= 6809
	TVP5150_write_cmos_sensor(0x98E, 0x4839);	//Fine Correction (B)
	TVP5150_write_cmos_sensor(0x990, 0x019F);	//		= 415
	TVP5150_write_cmos_sensor(0x98E, 0x483B);	//Row Speed (B)
	TVP5150_write_cmos_sensor(0x990, 0x0111);	//		= 273
	TVP5150_write_cmos_sensor(0x98E, 0x483D);	//Read Mode (B)
	TVP5150_write_cmos_sensor(0x990, 0x0024);	//		= 36
	TVP5150_write_cmos_sensor(0x98E, 0x483F);	//Fine IT Min (B)
	TVP5150_write_cmos_sensor(0x990, 0x0266);	//		= 614
	TVP5150_write_cmos_sensor(0x98E, 0x4841);	//Fine IT Max Margin (B)
	TVP5150_write_cmos_sensor(0x990, 0x010A);	//		= 266
	TVP5150_write_cmos_sensor(0x98E, 0xB81A);	//fd_zone_height

	TVP5150_write_cmos_sensor(0x990, 0x06); //		= 6
	TVP5150_write_cmos_sensor(0x98E, 0x481A);	//fd_period_50Hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x00F9);	//		= 249
	TVP5150_write_cmos_sensor(0x98E, 0x481C);	//fd_period_60Hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x00D0);	//		= 208
	TVP5150_write_cmos_sensor(0x98E, 0xC81E);	//fd_search_f1_50hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x21); //		= 33
	TVP5150_write_cmos_sensor(0x98E, 0xC81F);	//fd_search_f2_50hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x23); //		= 35
	TVP5150_write_cmos_sensor(0x98E, 0xC820);	//fd_search_f1_60hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x28); //		= 40
	TVP5150_write_cmos_sensor(0x98E, 0xC821);	//fd_search_f2_60hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x2A); //		= 42
	TVP5150_write_cmos_sensor(0x98E, 0x4847);	//fd_period_50Hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x007A);	//		= 122
	TVP5150_write_cmos_sensor(0x98E, 0x4849);	//fd_period_60Hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x0069);	//		= 102
	TVP5150_write_cmos_sensor(0x98E, 0xC84B);	//fd_search_f1_50hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x10); //		= 16
	TVP5150_write_cmos_sensor(0x98E, 0xC84C);	//fd_search_f2_50hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x12); //		= 18
	TVP5150_write_cmos_sensor(0x98E, 0xC84D);	//fd_search_f1_60hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x13); //		= 19
	TVP5150_write_cmos_sensor(0x98E, 0xC84E);	//fd_search_f2_60hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x15); //		= 21
	TVP5150_write_cmos_sensor(0x98E, 0x6800);	//Output Width (A)
	TVP5150_write_cmos_sensor(0x990, 0x0400);	//		= 640
	TVP5150_write_cmos_sensor(0x98E, 0x6802);	//Output Height (A)
	TVP5150_write_cmos_sensor(0x990, 0x0300);	//		= 480
	TVP5150_write_cmos_sensor(0x98E, 0x6804);	//FOV Width (A)
	TVP5150_write_cmos_sensor(0x990, 0x0400);	//		= 1024
	TVP5150_write_cmos_sensor(0x98E, 0x6806);	//FOV Height (A)
	TVP5150_write_cmos_sensor(0x990, 0x0300);	//		= 768
	TVP5150_write_cmos_sensor(0x98E, 0xE892);	//JPEG Mode (A)
	TVP5150_write_cmos_sensor(0x990, 0x00); //		= 0
	TVP5150_write_cmos_sensor(0x98E, 0x6C00);	//Output Width (B)
	TVP5150_write_cmos_sensor(0x990, 0x0800);	//		= 2048
	TVP5150_write_cmos_sensor(0x98E, 0x6C02);	//Output Height (B)
	TVP5150_write_cmos_sensor(0x990, 0x0600);	//		= 1536
	TVP5150_write_cmos_sensor(0x98E, 0x6C04);	//FOV Width (B)
	TVP5150_write_cmos_sensor(0x990, 0x0800);	//		= 2048
	TVP5150_write_cmos_sensor(0x98E, 0x6C06);	//FOV Height (B)
	TVP5150_write_cmos_sensor(0x990, 0x0600);	//		= 1536
	TVP5150_write_cmos_sensor(0x98E, 0xEC92);	//JPEG Mode (B)
	TVP5150_write_cmos_sensor(0x990, 0x00); //		= 0
	TVP5150_write_cmos_sensor(0x98E, 0x8400);	//Refresh Sequencer Mode
	TVP5150_write_cmos_sensor(0x990, 0x06); //		= 6

}
void test_pclk_48M(void)
{
	
	//[TVP5150 (SOC3140) Register Wizard Defaults]
	TVP5150_write_cmos_sensor(0x0010, 0x0C60);	//PLL Dividers = 3168
	TVP5150_write_cmos_sensor(0x0012, 0x0070);	//PLL P Dividers = 112
	TVP5150_write_cmos_sensor(0x002A, 0x7464);	//PLL P Dividers 4-5-6 = 29796
	TVP5150_write_cmos_sensor(0x0018, 0x402E);	//Standby:Default = 16430
	//POLL_REG=0x0018,0x4000,==1,DELAY=10,TIME);OUT=100   // Wait for FW initialization complete
	TVP5150_write_cmos_sensor(0x0022, 0x0140);	//Reference clock count for 20 us = 320
	TVP5150_write_cmos_sensor(0x001E, 0x777 );//Pad Slew Rate = 1911
	TVP5150_write_cmos_sensor(0x3B84, 0x017C);	//I2C Master Clock Divider = 380
											
	//LOAD the FW patches and recommended se);ttings here!
										   
	TVP5150_write_cmos_sensor(0x0018, 0x4028);	//Out of Standby = 16424\
	 mdelay(30);//DELAY=3
	TVP5150_write_cmos_sensor(0x98E, 0x4800 );//Row Start (A)
	TVP5150_write_cmos_sensor(0x990, 0x0010 );//	  = 16
	TVP5150_write_cmos_sensor(0x98E, 0x4802 );//Column Start (A)
	TVP5150_write_cmos_sensor(0x990, 0x0010 );//	  = 16
	TVP5150_write_cmos_sensor(0x98E, 0x4804 );//Row End (A)
	TVP5150_write_cmos_sensor(0x990, 0x062D );//	  = 1581
	TVP5150_write_cmos_sensor(0x98E, 0x4806 );//Column End (A)
	TVP5150_write_cmos_sensor(0x990, 0x082D );//	  = 2093
	TVP5150_write_cmos_sensor(0x98E, 0x4808 );//Base Frame Lines (A)
	TVP5150_write_cmos_sensor(0x990, 0x0364 );//	  = 868
	TVP5150_write_cmos_sensor(0x98E, 0x480A );//Line Length (A)
	TVP5150_write_cmos_sensor(0x990, 0x0D07 );//	  = 3335
	TVP5150_write_cmos_sensor(0x98E, 0x480C );//Fine Correction (A)
	TVP5150_write_cmos_sensor(0x990, 0x0399 );//	  = 921
	TVP5150_write_cmos_sensor(0x98E, 0x480E );//Row Speed (A)
	TVP5150_write_cmos_sensor(0x990, 0x0111 );//	  = 273
	TVP5150_write_cmos_sensor(0x98E, 0x4810 );//Read Mode (A)
	TVP5150_write_cmos_sensor(0x990, 0x046C );//	  = 1132
	TVP5150_write_cmos_sensor(0x98E, 0x4812 );//Fine IT Min (A)
	TVP5150_write_cmos_sensor(0x990, 0x0510 );//	  = 1296
	TVP5150_write_cmos_sensor(0x98E, 0x4814 );//Fine IT Max Margin (A)
	TVP5150_write_cmos_sensor(0x990, 0x01BA );//	  = 442
	TVP5150_write_cmos_sensor(0x98E, 0x482D );//Row Start (B)
	TVP5150_write_cmos_sensor(0x990, 0x0018 );//	  = 24
	TVP5150_write_cmos_sensor(0x98E, 0x482F );//Column Start (B)
	TVP5150_write_cmos_sensor(0x990, 0x0018 );//	  = 24
	TVP5150_write_cmos_sensor(0x98E, 0x4831 );//Row End (B)
	TVP5150_write_cmos_sensor(0x990, 0x0627 );//	  = 1575
	TVP5150_write_cmos_sensor(0x98E, 0x4833 );//Column End (B)
	TVP5150_write_cmos_sensor(0x990, 0x0827 );//	  = 2087
	TVP5150_write_cmos_sensor(0x98E, 0x4835 );//Base Frame Lines (B)
	TVP5150_write_cmos_sensor(0x990, 0x065D );//	  = 1629
	TVP5150_write_cmos_sensor(0x98E, 0x4837 );//Line Length (B)
	TVP5150_write_cmos_sensor(0x990, 0x1A4F );//	  = 6735
	TVP5150_write_cmos_sensor(0x98E, 0x4839 );//Fine Correction (B)
	TVP5150_write_cmos_sensor(0x990, 0x019F );//	  = 415
	TVP5150_write_cmos_sensor(0x98E, 0x483B );//Row Speed (B)
	TVP5150_write_cmos_sensor(0x990, 0x0111 );//	  = 273
	TVP5150_write_cmos_sensor(0x98E, 0x483D );//Read Mode (B)
	TVP5150_write_cmos_sensor(0x990, 0x0024 );//	  = 36
	TVP5150_write_cmos_sensor(0x98E, 0x483F );//Fine IT Min (B)
	TVP5150_write_cmos_sensor(0x990, 0x0266 );//	  = 614
	TVP5150_write_cmos_sensor(0x98E, 0x4841 );//Fine IT Max Margin (B)
	TVP5150_write_cmos_sensor(0x990, 0x010A );//	  = 266
	TVP5150_write_cmos_sensor(0x98E, 0xB81A );//fd_zone_height
	TVP5150_write_cmos_sensor(0x990, 0x05	);		//= 5
	TVP5150_write_cmos_sensor(0x98E, 0x481A );//fd_period_50Hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x00E6 );//	  = 230
	TVP5150_write_cmos_sensor(0x98E, 0x481C );//fd_period_60Hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x00C0 );//	  = 192
	TVP5150_write_cmos_sensor(0x98E, 0xC81E );//fd_search_f1_50hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x25	);		//= 37
	TVP5150_write_cmos_sensor(0x98E, 0xC81F );//fd_search_f2_50hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x27	);		//= 39
	TVP5150_write_cmos_sensor(0x98E, 0xC820 );//fd_search_f1_60hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x2D	);		//= 45
	TVP5150_write_cmos_sensor(0x98E, 0xC821 );//fd_search_f2_60hz (A)
	TVP5150_write_cmos_sensor(0x990, 0x2F	);		//= 47
	TVP5150_write_cmos_sensor(0x98E, 0x4847 );//fd_period_50Hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x0072 );//	  = 114
	TVP5150_write_cmos_sensor(0x98E, 0x4849 );//fd_period_60Hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x005F );//	  = 95
	TVP5150_write_cmos_sensor(0x98E, 0xC84B );//fd_search_f1_50hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x12	);		//= 18
	TVP5150_write_cmos_sensor(0x98E, 0xC84C );//fd_search_f2_50hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x14	);		//= 20
	TVP5150_write_cmos_sensor(0x98E, 0xC84D );//fd_search_f1_60hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x15	);	   // = 21
	TVP5150_write_cmos_sensor(0x98E, 0xC84E );//fd_search_f2_60hz (B)
	TVP5150_write_cmos_sensor(0x990, 0x17	);		//= 23
	TVP5150_write_cmos_sensor(0x98E, 0x6800 );//Output Width (A)
	TVP5150_write_cmos_sensor(0x990, 0x0400 );//	  = 1024
	TVP5150_write_cmos_sensor(0x98E, 0x6802 );//Output Height (A)
	TVP5150_write_cmos_sensor(0x990, 0x0300 );//	  = 768
	TVP5150_write_cmos_sensor(0x98E, 0x6804 );//FOV Width (A)
	TVP5150_write_cmos_sensor(0x990, 0x0400 );//	  = 1024
	TVP5150_write_cmos_sensor(0x98E, 0x6806 );//FOV Height (A)
	TVP5150_write_cmos_sensor(0x990, 0x0300 );//	  = 768
	TVP5150_write_cmos_sensor(0x98E, 0xE892 );//JPEG Mode (A)
	TVP5150_write_cmos_sensor(0x990, 0x00	);		//= 0
	TVP5150_write_cmos_sensor(0x98E, 0x6C00 );//Output Width (B)
	TVP5150_write_cmos_sensor(0x990, 0x0800 );//	  = 2048
	TVP5150_write_cmos_sensor(0x98E, 0x6C02 );//Output Height (B)
	TVP5150_write_cmos_sensor(0x990, 0x0600 );//	  = 1536
	TVP5150_write_cmos_sensor(0x98E, 0x6C04 );//FOV Width (B)
	TVP5150_write_cmos_sensor(0x990, 0x0800 );//	  = 2048
	TVP5150_write_cmos_sensor(0x98E, 0x6C06 );//FOV Height (B)
	TVP5150_write_cmos_sensor(0x990, 0x0600 );//	  = 1536
	TVP5150_write_cmos_sensor(0x98E, 0xEC92 );//JPEG Mode (B)
	TVP5150_write_cmos_sensor(0x990, 0x00	);	   // = 0
	TVP5150_write_cmos_sensor(0x98E, 0x8400 );//Refresh Sequencer Mode
	TVP5150_write_cmos_sensor(0x990, 0x06	);	   // = 6
	

}


struct i2c_reg_value {
    unsigned char reg;
    unsigned char value;
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {
    { /* 0x00 */
        TVP5150_VD_IN_SRC_SEL_1,0x00
    },
    { /* 0x01 */
        TVP5150_ANAL_CHL_CTL,0x15
    },
    { /* 0x02 */
        TVP5150_OP_MODE_CTL,0x00
    },
    { /* 0x03 */
        TVP5150_MISC_CTL,0x01
    },
    { /* 0x06 */
        TVP5150_COLOR_KIL_THSH_CTL,0x10
    },
    { /* 0x07 */
        TVP5150_LUMA_PROC_CTL_1,0x60
    },
    { /* 0x08 */
        TVP5150_LUMA_PROC_CTL_2,0x00
    },
    { /* 0x09 */
        TVP5150_BRIGHT_CTL,0x80
    },
    { /* 0x0a */
        TVP5150_SATURATION_CTL,0x80
    },
    { /* 0x0b */
        TVP5150_HUE_CTL,0x00
    },
    { /* 0x0c */
        TVP5150_CONTRAST_CTL,0x80
    },
    { /* 0x0d */
        TVP5150_DATA_RATE_SEL,0x47
    },
    { /* 0x0e */
        TVP5150_LUMA_PROC_CTL_3,0x00
    },
    { /* 0x0f */
        TVP5150_CONF_SHARED_PIN,0x08
    },
    { /* 0x11 */
        TVP5150_ACT_VD_CROP_ST_MSB,0x00
    },
    { /* 0x12 */
        TVP5150_ACT_VD_CROP_ST_LSB,0x00
    },
    { /* 0x13 */
        TVP5150_ACT_VD_CROP_STP_MSB,0x00
    },
    { /* 0x14 */
        TVP5150_ACT_VD_CROP_STP_LSB,0x00
    },
    { /* 0x15 */
        TVP5150_GENLOCK,0x01
    },
    { /* 0x16 */
        TVP5150_HORIZ_SYNC_START,0x80
    },
    { /* 0x18 */
        TVP5150_VERT_BLANKING_START,0x00
    },
    { /* 0x19 */
        TVP5150_VERT_BLANKING_STOP,0x00
    },
    { /* 0x1a */
        TVP5150_CHROMA_PROC_CTL_1,0x0c
    },
    { /* 0x1b */
        TVP5150_CHROMA_PROC_CTL_2,0x14
    },
    { /* 0x1c */
        TVP5150_INT_RESET_REG_B,0x00
    },
    { /* 0x1d */
        TVP5150_INT_ENABLE_REG_B,0x00
    },
    { /* 0x1e */
        TVP5150_INTT_CONFIG_REG_B,0x00
    },
    { /* 0x28 */
        TVP5150_VIDEO_STD,0x00
    },
    { /* 0x2e */
        TVP5150_MACROVISION_ON_CTR,0x0f
    },
    { /* 0x2f */
        TVP5150_MACROVISION_OFF_CTR,0x01
    },
    { /* 0xbb */
        TVP5150_TELETEXT_FIL_ENA,0x00
    },
    { /* 0xc0 */
        TVP5150_INT_STATUS_REG_A,0x00
    },
    { /* 0xc1 */
        TVP5150_INT_ENABLE_REG_A,0x00
    },
    { /* 0xc2 */
        TVP5150_INT_CONF,0x04
    },
    { /* 0xc8 */
        TVP5150_FIFO_INT_THRESHOLD,0x80
    },
    { /* 0xc9 */
        TVP5150_FIFO_RESET,0x00
    },
    { /* 0xca */
        TVP5150_LINE_NUMBER_INT,0x00
    },
    { /* 0xcb */
        TVP5150_PIX_ALIGN_REG_LOW,0x4e
    },
    { /* 0xcc */
        TVP5150_PIX_ALIGN_REG_HIGH,0x00
    },
    { /* 0xcd */
        TVP5150_FIFO_OUT_CTRL,0x01
    },
    { /* 0xcf */
        TVP5150_FULL_FIELD_ENA,0x00
    },
    { /* 0xd0 */
        TVP5150_LINE_MODE_INI,0x00
    },
    { /* 0xfc */
        TVP5150_FULL_FIELD_MODE_REG,0x7f
    },
    { /* end of data */
        0xff,0xff
    }
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
    {
        TVP5150_CONF_SHARED_PIN, 2
    },{ /* Automatic offset and AGC enabled */
        TVP5150_ANAL_CHL_CTL, 0x15
    },{ /* Activate YCrCb output 0x9 or 0xd ? */
        TVP5150_MISC_CTL, 0x6f
    },{ /* Activates video std autodetection for all standards */
        TVP5150_AUTOSW_MSK, 0x0
    },{ /* Default format: 0x47. For 4:2:2: 0x40 */
        TVP5150_DATA_RATE_SEL, 0x47
    },{
        TVP5150_CHROMA_PROC_CTL_1, 0x0c
    },{
        TVP5150_CHROMA_PROC_CTL_2, 0x54
    },{ /* Non documented, but initialized on WinTV USB2 */
        0x27, 0x20
    },{
        0xff,0xff
    }
};

static void TVP5150_YUV_sensor_initial_setting(void)
{
    const struct i2c_reg_value *regs = tvp5150_init_enable;

	SENSORDB("[TVP5150YUV]: sensor initial settings Start!\n");  

    while (regs->reg != 0xff) {
	    TVP5150_write_cmos_sensor(regs->reg, regs->value);
        regs++;
    }
	
	mdelay(100);//DELAY=100
	//STATE= Detect Master Clock, 1

	spin_lock(&tvp5150_drv_lock);
	TVP5150_sensor.preview_pclk = 27;
	spin_unlock(&tvp5150_drv_lock);

	SENSORDB("[TVP5150YUV]: sensor initial settings end!\n");  

} /* TVP5150_YUV_sensor_initial_setting */
static void TVP5150_CAP_setting(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&tvp5150_drv_lock);
	TVP5150_sensor.capture_pclk = 27;
	TVP5150_op_state.is_PV_mode = KAL_FALSE;
	spin_unlock(&tvp5150_drv_lock);

	return;	

} /* TVP5150_CAP_setting */

static void TVP5150_set_AE_mode(kal_bool AE_enable)
{
	kal_uint8 temp_AE_reg = 0;
	return;
	if (AE_enable == KAL_TRUE)
	{
		// turn on AEC/AGC
	}
	else
	{
		// turn off AEC/AGC

	}
}


static void TVP5150_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

	return ;

	if (AWB_enable == KAL_TRUE)
	{
		//enable Auto WB
		
	}
	else
	{
		//turn off AWB

	}
}

static u32 tvp5150_calc_mipiclk(void)
{
       u32 rxpll, val, n, m, bit8div;
       u32 sdiv_inv, mipidiv;
       u32 fclk, mipiclk, mclk = 26000000;
       u8 lut1[4] = {2, 3, 4, 6};
       u8 lut2[4] = {1, 1, 4, 5};

       val = TVP5150_read_cmos_sensor(0x3011); 

       mclk /= (val + 1); 

       /* Calculate fclk */
       val = TVP5150_read_cmos_sensor(0x300E);
       rxpll = val & 0x3F;

      val = TVP5150_read_cmos_sensor(0x300F);
      n = lut1[(val >> 6) & 0x3];
      m = lut1[val & 0x3];
      bit8div = lut2[(val >> 4) & 0x3];
      fclk = (64 - rxpll) * n * bit8div * mclk / m;

      val = TVP5150_read_cmos_sensor(0x3010);
      mipidiv = ((val >> 5) & 1) + 1;
      sdiv_inv = (val & 0xF) * 2;

      if ((val & 0xF) >= 1)
              mipiclk = fclk / sdiv_inv / mipidiv;
      else
              mipiclk = fclk / mipidiv;
     return mipiclk;
}

/*************************************************************************
* FUNCTION
*	TVP5150_Set_Video_Frame_Rate
*
* DESCRIPTION
*	This function set the sensor output frmae to target frame and fix the frame rate for 
*	video encode.
*
* PARAMETERS
*	1. kal_uint32 : Target frame rate to fixed.
*
* RETURNS
*	None
*
*************************************************************************/
static void TVP5150_Set_Video_Frame_Rate(kal_uint32 frame_rate)
{
	kal_uint32 line_length;
	kal_uint32 frame_rate_50Hz,frame_rate_60Hz;
	kal_uint32 base_shutter_50Hz,base_shutter_60Hz;

	line_length = SENSOR_CORE_PCLK*10/frame_rate/(TVP5150_PV_PERIOD_LINE_NUMS + TVP5150_sensor.pv_dummy_lines);
	frame_rate_50Hz		= 50*2*10/frame_rate;
	frame_rate_60Hz		= 60*2*10/frame_rate;


	base_shutter_50Hz = (frame_rate*TVP5150_PV_PERIOD_LINE_NUMS/50+5)/2/10;;
	base_shutter_60Hz = (frame_rate*TVP5150_PV_PERIOD_LINE_NUMS/60+5)/2/10;;
	TVP5150_write_cmos_sensor(0x98E, 0x481A); 				//fd_period_50Hz (A)
	TVP5150_write_cmos_sensor(0x990, base_shutter_50Hz); 	//		
	TVP5150_write_cmos_sensor(0x98E, 0x481C); 				//fd_period_60Hz (A)
	TVP5150_write_cmos_sensor(0x990, base_shutter_60Hz); 	//	

	TVP5150_write_cmos_sensor(0x098E, 0x8400);			// MCU_ADDRESS [SEQ_CMD]
	TVP5150_write_cmos_sensor(0x0990, 0x0006);			// MCU_DATA_0
	{
		kal_uint16 temp=0;
		while(temp <60)
		{
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	  // MCU_DATA_0
			if(0==TVP5150_read_cmos_sensor(0x990))
			{
			 break;
			}
			mdelay(4);//DELAY=100
			temp+=1;
		}
	}

#if 1
	mdelay(400);

	TVP5150_write_cmos_sensor(0x098E, 0x6815);			// MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
	TVP5150_write_cmos_sensor(0x0990, frame_rate_50Hz);	// MCU_DATA_0
	TVP5150_write_cmos_sensor(0x098E, 0x6817);			// MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
	TVP5150_write_cmos_sensor(0x0990, frame_rate_60Hz);	// MCU_DATA_0

	TVP5150_write_cmos_sensor(0x98E, 0x480A);		//Line Length (A)
	TVP5150_write_cmos_sensor(0x990, line_length);	//		

	TVP5150_write_cmos_sensor(0x098E, 0x8400);			// MCU_ADDRESS [SEQ_CMD]
	TVP5150_write_cmos_sensor(0x0990, 0x0006);			// MCU_DATA_0
	{
		kal_uint16 temp=0;
		while(temp <60)
		{
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	  // MCU_DATA_0
			if(0==TVP5150_read_cmos_sensor(0x990))
			{
			 break;
			}
			mdelay(20);//DELAY=100
			temp+=1;
		}
	}
#endif	

	spin_lock(&tvp5150_drv_lock);
	TVP5150_sensor.video_frame_rate = frame_rate;
	spin_unlock(&tvp5150_drv_lock);

	return;

}

static int tvp5150_log_status(void)
{
	SENSORDB("tvp5150: Video input source selection #1 = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VD_IN_SRC_SEL_1));
	SENSORDB("tvp5150: Analog channel controls = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_ANAL_CHL_CTL));
	SENSORDB("tvp5150: Operation mode controls = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_OP_MODE_CTL));
	SENSORDB("tvp5150: Miscellaneous controls = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_MISC_CTL));
	SENSORDB("tvp5150: Autoswitch mask= 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_AUTOSW_MSK));
	SENSORDB("tvp5150: Color killer threshold control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_COLOR_KIL_THSH_CTL));
	SENSORDB("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			TVP5150_read_cmos_sensor(TVP5150_LUMA_PROC_CTL_1),
			TVP5150_read_cmos_sensor(TVP5150_LUMA_PROC_CTL_2),
			TVP5150_read_cmos_sensor(TVP5150_LUMA_PROC_CTL_3));
	SENSORDB("tvp5150: Brightness control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_BRIGHT_CTL));
	SENSORDB("tvp5150: Color saturation control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_SATURATION_CTL));
	SENSORDB("tvp5150: Hue control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_HUE_CTL));
	SENSORDB("tvp5150: Contrast control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_CONTRAST_CTL));
	SENSORDB("tvp5150: Outputs and data rates select = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_DATA_RATE_SEL));
	SENSORDB("tvp5150: Configuration shared pins = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_CONF_SHARED_PIN));
	SENSORDB("tvp5150: Active video cropping start = 0x%02x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_ACT_VD_CROP_ST_MSB),
			TVP5150_read_cmos_sensor(TVP5150_ACT_VD_CROP_ST_LSB));
	SENSORDB("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_ACT_VD_CROP_STP_MSB),
			TVP5150_read_cmos_sensor(TVP5150_ACT_VD_CROP_STP_LSB));
	SENSORDB("tvp5150: Genlock/RTC = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_GENLOCK));
	SENSORDB("tvp5150: Horizontal sync start = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_HORIZ_SYNC_START));
	SENSORDB("tvp5150: Vertical blanking start = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VERT_BLANKING_START));
	SENSORDB("tvp5150: Vertical blanking stop = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VERT_BLANKING_STOP));
	SENSORDB("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			TVP5150_read_cmos_sensor(TVP5150_CHROMA_PROC_CTL_1),
			TVP5150_read_cmos_sensor(TVP5150_CHROMA_PROC_CTL_2));
	SENSORDB("tvp5150: Interrupt reset register B = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_RESET_REG_B));
	SENSORDB("tvp5150: Interrupt enable register B = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_ENABLE_REG_B));
	SENSORDB("tvp5150: Interrupt configuration register B = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INTT_CONFIG_REG_B));
	SENSORDB("tvp5150: Video standard = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VIDEO_STD));
	SENSORDB("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_CB_GAIN_FACT),
			TVP5150_read_cmos_sensor(TVP5150_CR_GAIN_FACTOR));
	SENSORDB("tvp5150: Macrovision on counter = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_MACROVISION_ON_CTR));
	SENSORDB("tvp5150: Macrovision off counter = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_MACROVISION_OFF_CTR));
	SENSORDB("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(TVP5150_read_cmos_sensor(TVP5150_REV_SELECT) & 1) ? 3 : 4);
	SENSORDB("tvp5150: Device ID = %02x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_MSB_DEV_ID),
			TVP5150_read_cmos_sensor(TVP5150_LSB_DEV_ID));
	SENSORDB("tvp5150: ROM version = (hex) %02x.%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_ROM_MAJOR_VER),
			TVP5150_read_cmos_sensor(TVP5150_ROM_MINOR_VER));
	SENSORDB("tvp5150: Vertical line count = 0x%02x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VERT_LN_COUNT_MSB),
			TVP5150_read_cmos_sensor(TVP5150_VERT_LN_COUNT_LSB));
	SENSORDB("tvp5150: Interrupt status register B = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_STATUS_REG_B));
	SENSORDB("tvp5150: Interrupt active register B = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_ACTIVE_REG_B));
	SENSORDB("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
			TVP5150_read_cmos_sensor(TVP5150_STATUS_REG_1),
			TVP5150_read_cmos_sensor(TVP5150_STATUS_REG_2),
			TVP5150_read_cmos_sensor(TVP5150_STATUS_REG_3),
			TVP5150_read_cmos_sensor(TVP5150_STATUS_REG_4),
			TVP5150_read_cmos_sensor(TVP5150_STATUS_REG_5));

	SENSORDB("tvp5150: Teletext filter enable = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_TELETEXT_FIL_ENA));
	SENSORDB("tvp5150: Interrupt status register A = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_STATUS_REG_A));
	SENSORDB("tvp5150: Interrupt enable register A = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_ENABLE_REG_A));
	SENSORDB("tvp5150: Interrupt configuration = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_INT_CONF));
	SENSORDB("tvp5150: VDP status register = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_VDP_STATUS_REG));
	SENSORDB("tvp5150: FIFO word count = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FIFO_WORD_COUNT));
	SENSORDB("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FIFO_INT_THRESHOLD));
	SENSORDB("tvp5150: FIFO reset = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FIFO_RESET));
	SENSORDB("tvp5150: Line number interrupt = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_LINE_NUMBER_INT));
	SENSORDB("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_PIX_ALIGN_REG_HIGH),
			TVP5150_read_cmos_sensor(TVP5150_PIX_ALIGN_REG_LOW));
	SENSORDB("tvp5150: FIFO output control = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FIFO_OUT_CTRL));
	SENSORDB("tvp5150: Full field enable = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FULL_FIELD_ENA));
	SENSORDB("tvp5150: Full field mode register = 0x%02x\n",
			TVP5150_read_cmos_sensor(TVP5150_FULL_FIELD_MODE_REG));

	return 0;
}

static kal_uint32 TVP5150_GetSensorID(kal_uint32 *sensorID)
{

   volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_i2c_addr = 0;
    
	SENSORDB("[Enter]:TVP5150_GetSensorID");
    for (i=0; i<(sizeof(TVP5150_i2c_addr)/sizeof(TVP5150_i2c_addr[0])); i++)
	{
		spin_lock(&tvp5150_drv_lock);
		TVP5150_sensor.i2c_write_id	= TVP5150_i2c_addr[i];
		TVP5150_sensor.i2c_read_id		= (TVP5150_sensor.i2c_write_id | 1);
		spin_unlock(&tvp5150_drv_lock);

		//4 <9>software reset sensor and wait (to sensor)
		//Reset
        /*
		TVP5150_write_cmos_sensor(0x05, 0x01); // STANDBY_CONTROL_AND_STATUS
		mdelay(1);//DELAY=1
		TVP5150_write_cmos_sensor(0x05, 0x00); // STANDBY_CONTROL_AND_STATUS
		mdelay(5);//DELAY=5
        */ 

		sensor_id = TVP5150_read_cmos_sensor(TVP5150_MSB_DEV_ID) << 8 | TVP5150_read_cmos_sensor(TVP5150_LSB_DEV_ID);
		if (sensor_id == TVP5150_SENSOR_ID)
		{
			/* Swap the correct i2c address to first one, it will speed up next time read sensor ID */
			temp_i2c_addr = TVP5150_i2c_addr[0];
			TVP5150_i2c_addr[0] = TVP5150_i2c_addr[i];
			TVP5150_i2c_addr[i] = temp_i2c_addr;		
			break;
		}
	}
	*sensorID = sensor_id;

	SENSORDB("[TVP5150YUV]:Read Sensor ID :0x%x\n", sensor_id);  

	//  Read sensor ID to adjust I2C is OK?
    if (sensor_id != TVP5150_SENSOR_ID)
	{
        *sensorID=0xFFFFFFFF;
	    SENSORDB("[TVP5150YUV]:Read Sensor ID fail:0x%x\n", sensor_id);  
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    return ERROR_NONE;    
}   /* TVP5150Open  */

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	TVP5150Open
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
UINT32 TVP5150Open(void)
{
	volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint8 temp_i2c_addr = 0;

	SENSORDB("[TVP5150YUV]: TVP5150Open Start!\n");  

    TVP5150_GetSensorID(&sensor_id);

	//  Read sensor ID to adjust I2C is OK?
    if (sensor_id != TVP5150_SENSOR_ID)
	{
	    SENSORDB("[TVP5150YUV]:Read Sensor ID fail:0x%x\n", sensor_id);  
		return ERROR_SENSOR_CONNECT_FAIL;
	}

    /*9. Apply sensor initail setting*/
	TVP5150_YUV_sensor_initial_setting();

	spin_lock(&tvp5150_drv_lock);
	TVP5150_sensor.preview_pclk = 27;
	first_enter_preview = KAL_TRUE;
	TVP5150_sensor.pv_shutter = 0x0265;
	TVP5150_sensor.pv_extra_shutter = 0;
	spin_unlock(&tvp5150_drv_lock);

	SENSORDB("[TVP5150YUV]: TVP5150Open Exit!\n");  
     
	return ERROR_NONE;
}	/* TVP5150Open() */

/*************************************************************************
* FUNCTION
*	TVP5150Close
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
UINT32 TVP5150Close(void)
{
//	CISModulePowerOn(FALSE);
	
	return ERROR_NONE;
}	/* TVP5150Close() */


kal_uint16 TVP5150_write_gain(kal_uint16 gain)
{
//Not support

}

static kal_uint32 TVP5150_read_sensor_gain(void)
{
	kal_uint16 temp_reg = 0;
	kal_uint32 sensor_gain = 0;
return 0;	
	temp_reg = TVP5150_read_cmos_sensor(0x3000);  
	
	sensor_gain = (16 + (temp_reg & 0x0F));
	
	if (temp_reg & 0x10)
	{
		sensor_gain <<= 1;
	}
	if (temp_reg & 0x20)
	{
		sensor_gain <<= 1;
	}
	if (temp_reg & 0x40)
	{
		sensor_gain <<= 1;
	}
	if (temp_reg & 0x80)
	{
		sensor_gain <<= 1;
	}
	
	return sensor_gain;
}  /* TVP5150_read_sensor_gain */




static void TVP5150_set_mirror_flip(kal_uint8 image_mirror)
{
    switch (image_mirror) {
    case IMAGE_NORMAL:
		//[Default Orientation]
		TVP5150_write_cmos_sensor(0x098E, 0x4810);	// MCU_ADDRESS [CAM1_CTX_A_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x046C);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x483D);	// MCU_ADDRESS [CAM1_CTX_B_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x0024);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
		TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
        break;
	case IMAGE_H_MIRROR:
		//[Horizontal Mirror]
		TVP5150_write_cmos_sensor(0x098E, 0x4810);	// MCU_ADDRESS [CAM1_CTX_A_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x046D);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x483D);	// MCU_ADDRESS [CAM1_CTX_B_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x0025);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
		TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
        break;
	case IMAGE_V_MIRROR:	//Flip Register 0x04[6] and 0x04[4] (FF = 01)
        //[Vertical Flip]
		TVP5150_write_cmos_sensor(0x098E, 0x4810);	// MCU_ADDRESS [CAM1_CTX_A_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x046E);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x483D);	// MCU_ADDRESS [CAM1_CTX_B_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x0026);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
		TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
        break;

    case IMAGE_HV_MIRROR:
		//[Flip and Mirror]
		TVP5150_write_cmos_sensor(0x098E, 0x4810);	// MCU_ADDRESS [CAM1_CTX_A_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x046F);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x483D);	// MCU_ADDRESS [CAM1_CTX_B_READ_MODE]
		TVP5150_write_cmos_sensor(0x0990, 0x0027);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
		TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
        break;

    default:
        ASSERT(0);
    }
}

/*************************************************************************
* FUNCTION
*	TVP5150_awb_enable
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
static kal_bool TVP5150_awb_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AWB_reg = 0;

	return KAL_TRUE;
}

/*************************************************************************
* FUNCTION
*	TVP5150_ae_enable
*
* DESCRIPTION
*	This function enable or disable the ae (Auto Exposure).
*
* PARAMETERS
*	1. kal_bool : KAL_TRUE - enable ae, KAL_FALSE - disable awb.
*
* RETURNS
*	kal_bool : It means set awb right or not.
*
*************************************************************************/
static kal_bool TVP5150_ae_enable(kal_bool enalbe)
{	 
	kal_uint16 temp_AE_reg = 0;
	
	return KAL_TRUE;
}

/*************************************************************************
* FUNCTION
*	TVP5150Preview
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
UINT32 TVP5150Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)

{	
	
	#ifdef TVP5150_DEBUG
	 SENSORDB("[TVP5150YUV]:preview start\n");  
	#endif
	TVP5150_PV_setting(image_window, sensor_config_data);
	/* After set exposure line, there should be delay for 2~4 frame time, then enable AEC */
	mdelay(65);

	spin_lock(&tvp5150_drv_lock);
	TVP5150_op_state.sensor_cap_state = KAL_FALSE;
	TVP5150_VEDIO_encode_mode=KAL_FALSE;

	//TVP5150_ae_enable(KAL_TRUE);
	//TVP5150_awb_enable(KAL_TRUE);
																
	TVP5150_sensor.pv_dummy_pixels = 0;
    TVP5150_sensor.pv_dummy_lines = 0;
	
	TVP5150_sensor.preview_pclk = 27;
	TVP5150_op_state.is_PV_mode = KAL_TRUE;
	spin_unlock(&tvp5150_drv_lock);
	
	//TVP5150_set_mirror_flip(sensor_config_data->SensorImageMirror);

  	//[Go to preview]
    // TODO
	
	image_window->GrabStartX = TVP5150_PV_GRAB_START_X;
	image_window->GrabStartY = TVP5150_PV_GRAB_START_Y;
	image_window->ExposureWindowWidth = TVP5150_PV_GRAB_WIDTH;
	image_window->ExposureWindowHeight = TVP5150_PV_GRAB_HEIGHT;
    // copy sensor_config_data
	memcpy(&TVP5150SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	#ifdef TVP5150_DEBUG
	 SENSORDB("[TVP5150YUV]:preview exit\n");  
	#endif

  	return ERROR_NONE;
}	/* TVP5150_Preview */

UINT32 TVP5150Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	volatile kal_uint32 shutter = 0, temp_reg = 0;
	kal_uint32 prev_line_len = 0;
	kal_uint32 cap_line_len = 0;
	
	
	TVP5150_ae_enable(KAL_FALSE);
	TVP5150_awb_enable(KAL_FALSE);	
	shutter = TVP5150_read_shutter();
	temp_reg = TVP5150_read_sensor_gain();

	spin_lock(&tvp5150_drv_lock);
	TVP5150_op_state.sensor_cap_state = KAL_TRUE;	
	TVP5150_sensor.pv_sensor_gain = temp_reg;
	spin_unlock(&tvp5150_drv_lock);
	
	if ((image_window->ImageTargetWidth <= TVP5150_IMAGE_SENSOR_PV_WIDTH)
		&& (image_window->ImageTargetHeight <= TVP5150_IMAGE_SENSOR_PV_HEIGHT))
	{		/* Capture Size Less than PV Size */	
			if (zoom_factor >= 3) // DZ >= 3x
			{
			#ifdef TVP5150_DEBUG			
			    SENSORDB("[TVP5150YUV]:capture preview size zoom >=3 \n"); 			
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels =0;//TVP5150_PV_PERIOD_PIXEL_NUMS/4;
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
			}
			else if (zoom_factor >= 2) // DZ >= 2x
			{
            #ifdef TVP5150_DEBUG
			    SENSORDB("[TVP5150YUV]:capture preview size >=2 \n");
			#endif	

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels = 0;
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
				
			}
			else
			{
			
			#ifdef TVP5150_DEBUG
			    SENSORDB("[TVP5150YUV]:capture preview size \n");
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels = 0;
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
			}

			TVP5150_CAP_setting(image_window, sensor_config_data);
			spin_lock(&tvp5150_drv_lock);
			TVP5150_sensor.capture_pclk = TVP5150_sensor.preview_pclk;   //Don't need change the clk for pv capture
			spin_unlock(&tvp5150_drv_lock);
			TVP5150_set_dummy(TVP5150_sensor.cap_dummy_pixels, TVP5150_sensor.cap_dummy_lines);

			//[Go to capture]
            /*
			TVP5150_write_cmos_sensor(0x098E, 0xEC09);	// MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x3400, 0x7A24);	// MIPI_CONTROL

			{
				kal_uint16 temp=0;
				while(temp <60)
				{
					TVP5150_write_cmos_sensor(0x098E, 0x8401);	  // MCU_DATA_0
					if(7==TVP5150_read_cmos_sensor(0x990))
					{
					 break;
					}
					mdelay(20);//DELAY=100
					temp+=1;
				}
			}
            */
		
			image_window->GrabStartX = TVP5150_FULL_GRAB_START_X;
			image_window->GrabStartY = TVP5150_FULL_GRAB_START_Y;
			image_window->ExposureWindowWidth = TVP5150_FULL_GRAB_WIDTH;
			image_window->ExposureWindowHeight = TVP5150_FULL_GRAB_HEIGHT;
		
	}
	else 
	{    /* FULL Size Capture Mode */	
		TVP5150_CAP_setting(image_window, sensor_config_data);
		
		/* Capture Size <= 3M */
		//if ((image_window->image_target_width <= TVP5150_IMAGE_SENSOR_FULL_WIDTH)
		//	&& (image_window->image_target_height <= TVP5150_IMAGE_SENSOR_FULL_HEIGHT))
		{
		
			if (zoom_factor >= 7)
			{
			#ifdef TVP5150_DEBUG
			SENSORDB("[TVP5150YUV]:capture full size >=7 \n");						
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels = 0x3800;//0x80;
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
				
			}
			else if (zoom_factor >= 5)
			{
            #ifdef TVP5150_DEBUG
			SENSORDB("[TVP5150YUV]:capture full size >=5 \n");			
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels = 0x2000;  /*If Capture fail, you can add this dummy*/
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
			}
			else if(zoom_factor >= 3)
			{
			#ifdef TVP5150_DEBUG			
			SENSORDB("[TVP5150YUV]:capture full size >=3 \n");		
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels =0x0800;  /*If Capture fail, you can add this dummy*/
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
			}
			else
			{
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:capture full size \n");
			#endif

				spin_lock(&tvp5150_drv_lock);
				TVP5150_sensor.cap_dummy_pixels =0;  /*If Capture fail, you can add this dummy*/
				TVP5150_sensor.cap_dummy_lines = 0;
				spin_unlock(&tvp5150_drv_lock);
			}
		}
	
		TVP5150_set_dummy(TVP5150_sensor.cap_dummy_pixels, TVP5150_sensor.cap_dummy_lines);
		
		prev_line_len = TVP5150_PV_PERIOD_PIXEL_NUMS + TVP5150_sensor.pv_dummy_pixels;
		cap_line_len = TVP5150_FULL_PERIOD_PIXEL_NUMS + TVP5150_sensor.cap_dummy_pixels;
		shutter = (shutter * TVP5150_sensor.capture_pclk) / TVP5150_sensor.preview_pclk;
		shutter = (shutter * prev_line_len) / cap_line_len;
		shutter *= 2;			/* By sensor design */
		TVP5150_write_shutter(shutter);
		//[Go to capture]
        /*
		TVP5150_write_cmos_sensor(0x098E, 0xEC09);	// MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN]
		TVP5150_write_cmos_sensor(0x0990, 0x0000);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
		TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x3400, 0x7A24);	// MIPI_CONTROL
		{
			kal_uint16 temp=0;
			while(temp <60)
			{
				TVP5150_write_cmos_sensor(0x098E, 0x8401);	  // MCU_DATA_0
				if(7==TVP5150_read_cmos_sensor(0x990))
				{
				 break;
				}
				mdelay(20);//DELAY=100
				temp+=1;
			}
		}
        */
		image_window->GrabStartX = TVP5150_FULL_GRAB_START_X;
		image_window->GrabStartY = TVP5150_FULL_GRAB_START_Y;
		image_window->ExposureWindowWidth = TVP5150_FULL_GRAB_WIDTH;
		image_window->ExposureWindowHeight = TVP5150_FULL_GRAB_HEIGHT;
	}
	memcpy(&TVP5150SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}



UINT32 TVP5150GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=TVP5150_FULL_GRAB_WIDTH;  
	pSensorResolution->SensorFullHeight=TVP5150_FULL_GRAB_HEIGHT;
	pSensorResolution->SensorPreviewWidth=TVP5150_PV_GRAB_WIDTH;
	pSensorResolution->SensorPreviewHeight=TVP5150_PV_GRAB_HEIGHT;

	return ERROR_NONE;
}	/* TVP5150GetResolution() */

UINT32 TVP5150GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=TVP5150_PV_GRAB_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=TVP5150_PV_GRAB_HEIGHT;
	pSensorInfo->SensorFullResolutionX=TVP5150_FULL_GRAB_WIDTH;
	pSensorInfo->SensorFullResolutionY=TVP5150_FULL_GRAB_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;

	

	pSensorInfo->CaptureDelayFrame = 3; 
	pSensorInfo->PreviewDelayFrame = 5; 
	pSensorInfo->VideoDelayFrame = 5; 
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
                     pSensorInfo->SensorGrabStartX = TVP5150_PV_GRAB_START_X; 
                     pSensorInfo->SensorGrabStartY = TVP5150_PV_GRAB_START_Y;     			
			
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
                     pSensorInfo->SensorGrabStartX = TVP5150_FULL_GRAB_START_X; 
                     pSensorInfo->SensorGrabStartY = TVP5150_FULL_GRAB_START_Y;     			
		break;
		default:
			pSensorInfo->SensorClockFreq=26;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
                     pSensorInfo->SensorGrabStartX = TVP5150_PV_GRAB_START_X; 
                     pSensorInfo->SensorGrabStartY = TVP5150_PV_GRAB_START_Y;     			
		break;
	}
	//TVP5150_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	memcpy(pSensorConfigData, &TVP5150SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* TVP5150GetInfo() */


UINT32 TVP5150Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			TVP5150Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			TVP5150Capture(pImageWindow, pSensorConfigData);
		break;
		default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return TRUE;
}	/* TVP5150Control() */



/*************************************************************************
* FUNCTION
*	TVP5150_set_param_wb
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
BOOL TVP5150_set_param_wb(UINT16 para)
{
	mdelay(120);
	switch (para)
	{
	    case AWB_MODE_OFF:
		   #ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB off \n");
			#endif	
			spin_lock(&tvp5150_drv_lock);
	        TVP5150_AWB_ENABLE = KAL_FALSE; 
			spin_unlock(&tvp5150_drv_lock);
	        TVP5150_set_AWB_mode(TVP5150_AWB_ENABLE);
	        break;             
		case AWB_MODE_AUTO:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB auto \n");
			#endif
			spin_lock(&tvp5150_drv_lock);
            TVP5150_AWB_ENABLE = KAL_TRUE; 
			spin_unlock(&tvp5150_drv_lock);
			
            TVP5150_set_AWB_mode(TVP5150_AWB_ENABLE);    
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x003F ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x801F ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x12F7 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x000b ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x000B ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			{
			kal_uint16 temp=0;
			while(temp <60)
			{
				TVP5150_write_cmos_sensor(0x098E, 0x8400);	  // MCU_DATA_0
				if(0==TVP5150_read_cmos_sensor(0x990))
				{
				 break;
				}
				mdelay(28);//DELAY=100
				temp+=1;
			}
		}
		    break;
		case AWB_MODE_CLOUDY_DAYLIGHT:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB Cloudy \n");
			#endif
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			
			TVP5150_write_cmos_sensor(0x098E, 0xAC3B ); // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0034 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xAC3C ); // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0054 ); // MCU_DATA_0
		    break;
		case AWB_MODE_DAYLIGHT:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB Daylight \n");
			#endif
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			
			TVP5150_write_cmos_sensor(0x098E, 0xAC3B ); // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0039 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xAC3C ); // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0054 ); // MCU_DATA_0
		    break;
		case AWB_MODE_INCANDESCENT:	
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB INCANDESCENT \n");
			#endif
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			
			TVP5150_write_cmos_sensor(0x098E, 0xAC3B ); // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0058 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xAC3C ); // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0028 ); // MCU_DATA_0	
		    break;  
		case AWB_MODE_FLUORESCENT:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB FLUORESCENT \n");
			#endif
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			
			TVP5150_write_cmos_sensor(0x098E, 0xAC3B ); // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0044 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xAC3C ); // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0031 ); // MCU_DATA_0
		    break;  
		case AWB_MODE_TUNGSTEN:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:AWB TUNGSTEN \n");
			#endif
		    TVP5150_write_cmos_sensor(0x098E, 0x6848 ); // MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6865 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6867 ); // MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0000 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6881 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x6883 ); // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
			TVP5150_write_cmos_sensor(0x0990, 0x0008 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400 ); // MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006 ); // MCU_DATA_0
			
			TVP5150_write_cmos_sensor(0x098E, 0xAC3B ); // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0058 ); // MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xAC3C ); // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
			TVP5150_write_cmos_sensor(0x0990, 0x0023 ); // MCU_DATA_0
		    break;
		default:
			return FALSE;
	}

	return TRUE;
	
} /* TVP5150_set_param_wb */

/*************************************************************************
* FUNCTION
*	TVP5150_set_param_effect
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
BOOL TVP5150_set_param_effect(UINT16 para)
{
  kal_uint32 ret = KAL_TRUE;
	switch (para)
	{
		case MEFFECT_OFF:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect off \n");
			#endif
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0000);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0000);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
			{
			kal_uint16 temp=0;
			while(temp <60)
			{
				TVP5150_write_cmos_sensor(0x098E, 0x8400);	  // MCU_DATA_0
				if(0==TVP5150_read_cmos_sensor(0x990))
				{
				 break;
				}
				mdelay(28);//DELAY=100
				temp+=1;
			}
		}
	              break;
		case MEFFECT_SEPIA:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect SEPIA \n");
			#endif
			//[Special Effect-Sepia]
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE889);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x001D);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC89); 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
		    TVP5150_write_cmos_sensor(0x0990, 0x001D); 	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE88A);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
			TVP5150_write_cmos_sensor(0x0990, 0x00D8);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC8A);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x00D8);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0	
			break;  
		case MEFFECT_NEGATIVE:		
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect negative \n");
			#endif
			//[Special Effect-Negative]
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0003);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0003);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0
			break; 
		case MEFFECT_SEPIAGREEN:		
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect sepiaGreen \n");
			#endif
			//[Special Effect-green]
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE889);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x00D8);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE88A);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
			TVP5150_write_cmos_sensor(0x0990, 0x00BA);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC89);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x00D8);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC8A);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB]
			TVP5150_write_cmos_sensor(0x0990, 0x00BA);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0	
			break;
		case MEFFECT_SEPIABLUE:	
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect sepiablue \n");
			#endif
			//[Special Effect-Aqua]
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE889);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x00E2);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xE88A);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
			TVP5150_write_cmos_sensor(0x0990, 0x0030);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC89);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
			TVP5150_write_cmos_sensor(0x0990, 0x00E2);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC8A);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB]
			TVP5150_write_cmos_sensor(0x0990, 0x0030);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0	
			break;        
		case MEFFECT_MONO:				
		#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:effect mono \n");
		#endif
			//[Special Effect-Black/White]
			TVP5150_write_cmos_sensor(0x098E, 0xE887);	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0001);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0xEC87);	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
			TVP5150_write_cmos_sensor(0x0990, 0x0001);	// MCU_DATA_0
			TVP5150_write_cmos_sensor(0x098E, 0x8400);	// MCU_ADDRESS [SEQ_CMD]
			TVP5150_write_cmos_sensor(0x0990, 0x0006);	// MCU_DATA_0			  
			break;

		default:
			ret = FALSE;
	}

	return ret;

} /* TVP5150_set_param_effect */

/*************************************************************************
* FUNCTION
*	TVP5150_set_param_banding
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
BOOL TVP5150_set_param_banding(UINT16 para)
{
	kal_uint16 temp_reg = 0;
	/* Some sensor call base shutter as banding filter. */
	kal_uint32 base_shutter = 0, max_shutter_step = 0, exposure_limitation = 0;
	kal_uint32 line_length = 0, sensor_pixel_clock = 0;
	#ifdef TVP5150_DEBUG
		SENSORDB("[TVP5150YUV]:banding\n");  
	#endif
	if (TVP5150_op_state.is_PV_mode == KAL_TRUE)
		{
			line_length = TVP5150_PV_PERIOD_PIXEL_NUMS + TVP5150_sensor.pv_dummy_pixels;
			exposure_limitation = TVP5150_PV_PERIOD_LINE_NUMS + TVP5150_sensor.pv_dummy_lines;
			sensor_pixel_clock = TVP5150_sensor.preview_pclk * 100 * 1000;
		}
		else
		{
			line_length = TVP5150_FULL_PERIOD_PIXEL_NUMS + TVP5150_sensor.cap_dummy_pixels;
			exposure_limitation = TVP5150_FULL_PERIOD_LINE_NUMS + TVP5150_sensor.cap_dummy_lines;
			sensor_pixel_clock = TVP5150_sensor.capture_pclk * 100 * 1000;
		}
	
		line_length = line_length * 2;		/* Multiple 2 is because one YUV422 pixels need two clock. */

		spin_lock(&tvp5150_drv_lock);
		TVP5150_op_state.curr_banding = para;	/* Record current banding setting. */
		spin_unlock(&tvp5150_drv_lock);
		

	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
		#ifdef TVP5150_DEBUG
	       SENSORDB("[TVP5150YUV]:banding 50Hz\n");  
	    #endif	
		    /* + (line_length/2) is used fot base_shutter + 0.5 */
		base_shutter = ((sensor_pixel_clock/100) + (line_length/2)) / line_length;
		max_shutter_step = (exposure_limitation / base_shutter) - 1;

		//[50Hz]
		TVP5150_write_cmos_sensor(0x098E, 0x2003);	// MCU_ADDRESS [FD_ALGO]
		TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0xA005);	// MCU_ADDRESS [FD_FDPERIOD_SELECT]
		TVP5150_write_cmos_sensor(0x0990, 0x0001);	// MCU_DATA_0

			break;

		case AE_FLICKER_MODE_60HZ:
		#ifdef TVP5150_DEBUG
	       SENSORDB("[TVP5150YUV]:banding 60Hz\n");  
	    #endif	
			 /* + (line_length/2) is used fot base_shutter + 0.5 */
		base_shutter = ((sensor_pixel_clock/120) + (line_length/2)) / line_length;
		max_shutter_step = (exposure_limitation / base_shutter) - 1;

		//[60Hz]
		TVP5150_write_cmos_sensor(0x098E, 0x2003);	// MCU_ADDRESS [FD_ALGO]
		TVP5150_write_cmos_sensor(0x0990, 0x0002);	// MCU_DATA_0
		TVP5150_write_cmos_sensor(0x098E, 0xA005);	// MCU_ADDRESS [FD_FDPERIOD_SELECT]
		TVP5150_write_cmos_sensor(0x0990, 0x0000);	// MCU_DATA_0
			break;
	     default:
	          return KAL_FALSE;
	}

	return KAL_TRUE;
} /* TVP5150_set_param_banding */




/*************************************************************************
* FUNCTION
*	TVP5150_set_param_exposure
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
BOOL TVP5150_set_param_exposure(UINT16 para)
{
	switch (para)
	{
		case AE_EV_COMP_13:
			TVP5150_write_cmos_sensor(0x337E, 0x7F00);	  // Y_RGB_OFFSET
			break;  
		case AE_EV_COMP_10:
			TVP5150_write_cmos_sensor(0x337E, 0x5900);		// Y_RGB_OFFSET
			break;    
		case AE_EV_COMP_07:
			TVP5150_write_cmos_sensor(0x337E, 0x3A00);	// Y_RGB_OFFSET
			break;    
		case AE_EV_COMP_03:			
			TVP5150_write_cmos_sensor(0x337E, 0x1C00);	// Y_RGB_OFFSET
		case AE_EV_COMP_00:
			#ifdef TVP5150_DEBUG
				SENSORDB("[TVP5150YUV]:ev 0\n");  
			#endif
			TVP5150_write_cmos_sensor(0x337E, 0x0800);	// Y_RGB_OFFSET
			break;    
		case AE_EV_COMP_n03:
			TVP5150_write_cmos_sensor(0x337E, 0xEA00);	// Y_RGB_OFFSET
			break;    
		case AE_EV_COMP_n07:			
			TVP5150_write_cmos_sensor(0x337E, 0xCC00);	// Y_RGB_OFFSET
			break;    
		case AE_EV_COMP_n10:
			TVP5150_write_cmos_sensor(0x337E, 0xAC00);	// Y_RGB_OFFSET
			break;
		case AE_EV_COMP_n13:
			TVP5150_write_cmos_sensor(0x337E, 0x8000);	// Y_RGB_OFFSET
			break;
		default:
			return FALSE;
	}

	return TRUE;
	
} /* TVP5150_set_param_exposure */


UINT32 TVP5150YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
//   if( TVP5150_sensor_cap_state == KAL_TRUE)
//	   return TRUE;

	switch (iCmd) {
	case FID_SCENE_MODE:	    
//	    SENSORDB("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF)
	    {
	        TVP5150_night_mode(0); 
	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
               TVP5150_night_mode(1); 
	    }	    
	    break; 	    
	case FID_AWB_MODE:
//	    SENSORDB("Set AWB Mode:%d\n", iPara); 	    
           TVP5150_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
//	    SENSORDB("Set Color Effect:%d\n", iPara); 	    	    
           TVP5150_set_param_effect(iPara);
	break;
	case FID_AE_EV:	    
//           SENSORDB("Set EV:%d\n", iPara); 	    	    
           TVP5150_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
//           SENSORDB("Set Flicker:%d\n", iPara); 	    	    	    
           TVP5150_set_param_banding(iPara);
	break;
	case FID_AE_SCENE_MODE: 
	    if (iPara == AE_MODE_OFF) {
				spin_lock(&tvp5150_drv_lock);
                TVP5150_AE_ENABLE = KAL_FALSE; 
				spin_unlock(&tvp5150_drv_lock);
            }
            else {
				spin_lock(&tvp5150_drv_lock);
                TVP5150_AE_ENABLE = KAL_TRUE; 
				spin_unlock(&tvp5150_drv_lock);
	    }
            TVP5150_set_AE_mode(TVP5150_AE_ENABLE);
            break; 

	case FID_ZOOM_FACTOR:
		spin_lock(&tvp5150_drv_lock);
	    zoom_factor = iPara; 		
		spin_unlock(&tvp5150_drv_lock);
	break; 
	default:
	break;
	}
	return TRUE;
}   /* TVP5150YUVSensorSetting */

UINT32 TVP5150YUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    SENSORDB("[TVP5150YUV] Set Video Mode \n"); 

	spin_lock(&tvp5150_drv_lock);
    TVP5150_VEDIO_encode_mode = KAL_TRUE; 
	spin_unlock(&tvp5150_drv_lock);
    //iTemp = TVP5150_read_cmos_sensor(0x3014);
    //TVP5150_write_cmos_sensor(0x3014, iTemp & 0xf7); //Disable night mode

    if (u2FrameRate == 30)
    {
      #ifdef TVP5150_DEBUG
        SENSORDB("[TVP5150YUV] video 30 \n");
	   #endif
	   TVP5150_write_cmos_sensor(0x098E, 0x682F);  // MCU_ADDRESS
	   TVP5150_write_cmos_sensor(0x0990, 0x0110);  // MCU_DATA_0// gain
       u2FrameRate=29;
       TVP5150_Set_Video_Frame_Rate(u2FrameRate*10);
    }
    else if (u2FrameRate == 15)
    {   
	#ifdef TVP5150_DEBUG
        SENSORDB("[TVP5150YUV] video 15 \n");
	#endif
	   TVP5150_write_cmos_sensor(0x098E, 0x682F);  // MCU_ADDRESS
	   TVP5150_write_cmos_sensor(0x0990, 0x0090);  // MCU_DATA_0// gain
       TVP5150_Set_Video_Frame_Rate(u2FrameRate*10);
    }
    else 
    {
    #ifdef TVP5150_DEBUG
        SENSORDB("Wrong Frame Rate \n"); 
	#endif
    }
    
    
    return TRUE;
}

UINT32 TVP5150FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
       UINT16 u2Temp = 0; 
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;


	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=TVP5150_FULL_GRAB_WIDTH;
			*pFeatureReturnPara16=TVP5150_FULL_GRAB_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			//*pFeatureReturnPara16++=TVP5150_PV_PERIOD_PIXEL_NUMS+TVP5150_PV_dummy_pixels;
			//*pFeatureReturnPara16=TVP5150_PV_PERIOD_LINE_NUMS+TVP5150_PV_dummy_lines;
			//*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = TVP5150_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
//			u2Temp = TVP5150_read_shutter(); 
//			SENSORDB("Shutter:%d\n", u2Temp); 			
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			TVP5150_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
//			u2Temp = TVP5150_read_gain(); 
//			SENSORDB("Gain:%d\n", u2Temp); 
//			SENSORDB("y_val:%d\n", TVP5150_read_cmos_sensor(0x301B));
			break; 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&tvp5150_drv_lock);
			TVP5150_isp_master_clock=*pFeatureData32;
			spin_unlock(&tvp5150_drv_lock);
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			TVP5150_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = TVP5150_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &TVP5150SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
		case SENSOR_FEATURE_SET_YUV_CMD:
//		       SENSORDB("TVP5150 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			TVP5150YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;		
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		    TVP5150YUVSetVideoMode(*pFeatureData16);
		    break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			TVP5150_GetSensorID(pFeatureData32); 
			break; 	
		default:
			break;			
	}
	return ERROR_NONE;
}	/* TVP5150FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncTVP5150=
{
	TVP5150Open,
	TVP5150GetInfo,
	TVP5150GetResolution,
	TVP5150FeatureControl,
	TVP5150Control,
	TVP5150Close
};

UINT32 TVP5150_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncTVP5150;

	return ERROR_NONE;
}	/* SensorInit() */


