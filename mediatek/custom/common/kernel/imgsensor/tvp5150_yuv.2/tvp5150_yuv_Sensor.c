#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/system.h>
#include <asm/io.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "tvp5150_yuv_Sensor.h"
#include "tvp5150_yuv_Camera_Sensor_para.h"
#include "tvp5150_yuv_CameraCustomized.h"

#include "tvp5150_reg.h"

#define TVP5150_H_MAX		720
#define TVP5150_V_MAX_525_60	480
#define TVP5150_V_MAX_OTHERS	576
#define TVP5150_MAX_CROP_LEFT	511
#define TVP5150_MAX_CROP_TOP	127
#define TVP5150_CROP_SHIFT	2

#define TVP5150YUV_DEBUG
#ifdef TVP5150YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

typedef struct
{
  kal_uint8   iSensorVersion;
  kal_bool    bNightMode;
  kal_uint16  iWB;
  kal_uint16  iAE;
  kal_uint16  iEffect;
  kal_uint16  iEV;
  kal_uint16  iBanding;
  kal_bool    bFixFrameRate;
  kal_uint8   iMirror;
  kal_uint16  iDummyPixel;  /* dummy pixel for user customization */
  kal_bool    bVideoMode;
  kal_uint8   iPclk;
  kal_uint16  MinFpsNormal; 
  kal_uint16  MinFpsNight; 
  /* Sensor regester backup*/
  kal_uint8   iCurrentPage;
  kal_uint8   iControl;
  kal_uint16  iHblank;     /* dummy pixel for calculating shutter step*/
  kal_uint16  iVblank;     /* dummy line calculated by cal_fps*/
  kal_uint8   iShutterStep;
  kal_uint8   iFrameCount;
} TVP5150Status;

#define Sleep(ms) mdelay(ms)

static DEFINE_SPINLOCK(tvp5150_drv_lock);

/* Global Valuable */
TVP5150Status TVP5150CurrentStatus;
MSDK_SENSOR_CONFIG_STRUCT TVP5150SensorConfigData;

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 tvp5150_write(kal_uint32 addr, kal_uint32 para)
{
  char puSendCmd[2] = {(char)(addr & 0xFF) ,(char)(para & 0xFF)};

  iWriteRegI2C(puSendCmd, 2, TVP5150_WRITE_ID);
  return TRUE;
}

kal_uint16 tvp5150_read(kal_uint32 addr)
{
  char puGetByte=0;
  char puSendCmd = (char)(addr & 0xFF);
  iReadRegI2C(&puSendCmd, 1, &puGetByte, 1, TVP5150_WRITE_ID);
  return puGetByte;
}

static void TVP5150SetPage(kal_uint8 iPage)
{   
#if 0
  if(TVP5150CurrentStatus.iCurrentPage == iPage)
    return ;

  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iCurrentPage = iPage;
  spin_unlock(&tvp5150_drv_lock);
  
  tvp5150_write(0x00,iPage);
#endif
}

static void TVP5150InitialPara(void)
{
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.bNightMode = KAL_FALSE;
  TVP5150CurrentStatus.iWB = AWB_MODE_AUTO;
  TVP5150CurrentStatus.iAE = AE_MODE_AUTO;
  TVP5150CurrentStatus.iEffect = MEFFECT_OFF;
  TVP5150CurrentStatus.iBanding = TVP5150_NUM_50HZ;
  TVP5150CurrentStatus.iEV = AE_EV_COMP_00;
  TVP5150CurrentStatus.bFixFrameRate = KAL_FALSE;
  TVP5150CurrentStatus.iMirror = IMAGE_NORMAL;
  TVP5150CurrentStatus.iDummyPixel = 0x1D;     
  TVP5150CurrentStatus.bVideoMode = KAL_FALSE;
  TVP5150CurrentStatus.iPclk = 27;
  
  TVP5150CurrentStatus.iCurrentPage = 0;
  TVP5150CurrentStatus.iControl = 0x00;
  TVP5150CurrentStatus.iHblank = 0x00;
  TVP5150CurrentStatus.iVblank = 0x00;
  TVP5150CurrentStatus.iShutterStep = 0x00;
  TVP5150CurrentStatus.iFrameCount = 0x0A;

  TVP5150CurrentStatus.MinFpsNormal = TVP5150_FPS(10);
  TVP5150CurrentStatus.MinFpsNight =  TVP5150CurrentStatus.MinFpsNormal >> 1;
  spin_unlock(&tvp5150_drv_lock);
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

static int tvp5150_log_status(void)
{
	SENSORDB("tvp5150: Video input source selection #1 = 0x%02x\n",
			tvp5150_read(TVP5150_VD_IN_SRC_SEL_1));
	SENSORDB("tvp5150: Analog channel controls = 0x%02x\n",
			tvp5150_read(TVP5150_ANAL_CHL_CTL));
	SENSORDB("tvp5150: Operation mode controls = 0x%02x\n",
			tvp5150_read(TVP5150_OP_MODE_CTL));
	SENSORDB("tvp5150: Miscellaneous controls = 0x%02x\n",
			tvp5150_read(TVP5150_MISC_CTL));
	SENSORDB("tvp5150: Autoswitch mask= 0x%02x\n",
			tvp5150_read(TVP5150_AUTOSW_MSK));
	SENSORDB("tvp5150: Color killer threshold control = 0x%02x\n",
			tvp5150_read(TVP5150_COLOR_KIL_THSH_CTL));
	SENSORDB("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp5150_read(TVP5150_LUMA_PROC_CTL_1),
			tvp5150_read(TVP5150_LUMA_PROC_CTL_2),
			tvp5150_read(TVP5150_LUMA_PROC_CTL_3));
	SENSORDB("tvp5150: Brightness control = 0x%02x\n",
			tvp5150_read(TVP5150_BRIGHT_CTL));
	SENSORDB("tvp5150: Color saturation control = 0x%02x\n",
			tvp5150_read(TVP5150_SATURATION_CTL));
	SENSORDB("tvp5150: Hue control = 0x%02x\n",
			tvp5150_read(TVP5150_HUE_CTL));
	SENSORDB("tvp5150: Contrast control = 0x%02x\n",
			tvp5150_read(TVP5150_CONTRAST_CTL));
	SENSORDB("tvp5150: Outputs and data rates select = 0x%02x\n",
			tvp5150_read(TVP5150_DATA_RATE_SEL));
	SENSORDB("tvp5150: Configuration shared pins = 0x%02x\n",
			tvp5150_read(TVP5150_CONF_SHARED_PIN));
	SENSORDB("tvp5150: Active video cropping start = 0x%02x%02x\n",
			tvp5150_read(TVP5150_ACT_VD_CROP_ST_MSB),
			tvp5150_read(TVP5150_ACT_VD_CROP_ST_LSB));
	SENSORDB("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			tvp5150_read(TVP5150_ACT_VD_CROP_STP_MSB),
			tvp5150_read(TVP5150_ACT_VD_CROP_STP_LSB));
	SENSORDB("tvp5150: Genlock/RTC = 0x%02x\n",
			tvp5150_read(TVP5150_GENLOCK));
	SENSORDB("tvp5150: Horizontal sync start = 0x%02x\n",
			tvp5150_read(TVP5150_HORIZ_SYNC_START));
	SENSORDB("tvp5150: Vertical blanking start = 0x%02x\n",
			tvp5150_read(TVP5150_VERT_BLANKING_START));
	SENSORDB("tvp5150: Vertical blanking stop = 0x%02x\n",
			tvp5150_read(TVP5150_VERT_BLANKING_STOP));
	SENSORDB("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp5150_read(TVP5150_CHROMA_PROC_CTL_1),
			tvp5150_read(TVP5150_CHROMA_PROC_CTL_2));
	SENSORDB("tvp5150: Interrupt reset register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_RESET_REG_B));
	SENSORDB("tvp5150: Interrupt enable register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ENABLE_REG_B));
	SENSORDB("tvp5150: Interrupt configuration register B = 0x%02x\n",
			tvp5150_read(TVP5150_INTT_CONFIG_REG_B));
	SENSORDB("tvp5150: Video standard = 0x%02x\n",
			tvp5150_read(TVP5150_VIDEO_STD));
	SENSORDB("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp5150_read(TVP5150_CB_GAIN_FACT),
			tvp5150_read(TVP5150_CR_GAIN_FACTOR));
	SENSORDB("tvp5150: Macrovision on counter = 0x%02x\n",
			tvp5150_read(TVP5150_MACROVISION_ON_CTR));
	SENSORDB("tvp5150: Macrovision off counter = 0x%02x\n",
			tvp5150_read(TVP5150_MACROVISION_OFF_CTR));
	SENSORDB("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp5150_read(TVP5150_REV_SELECT) & 1) ? 3 : 4);
	SENSORDB("tvp5150: Device ID = %02x%02x\n",
			tvp5150_read(TVP5150_MSB_DEV_ID),
			tvp5150_read(TVP5150_LSB_DEV_ID));
	SENSORDB("tvp5150: ROM version = (hex) %02x.%02x\n",
			tvp5150_read(TVP5150_ROM_MAJOR_VER),
			tvp5150_read(TVP5150_ROM_MINOR_VER));
	SENSORDB("tvp5150: Vertical line count = 0x%02x%02x\n",
			tvp5150_read(TVP5150_VERT_LN_COUNT_MSB),
			tvp5150_read(TVP5150_VERT_LN_COUNT_LSB));
	SENSORDB("tvp5150: Interrupt status register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_STATUS_REG_B));
	SENSORDB("tvp5150: Interrupt active register B = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ACTIVE_REG_B));
	SENSORDB("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
			tvp5150_read(TVP5150_STATUS_REG_1),
			tvp5150_read(TVP5150_STATUS_REG_2),
			tvp5150_read(TVP5150_STATUS_REG_3),
			tvp5150_read(TVP5150_STATUS_REG_4),
			tvp5150_read(TVP5150_STATUS_REG_5));

	SENSORDB("tvp5150: Teletext filter enable = 0x%02x\n",
			tvp5150_read(TVP5150_TELETEXT_FIL_ENA));
	SENSORDB("tvp5150: Interrupt status register A = 0x%02x\n",
			tvp5150_read(TVP5150_INT_STATUS_REG_A));
	SENSORDB("tvp5150: Interrupt enable register A = 0x%02x\n",
			tvp5150_read(TVP5150_INT_ENABLE_REG_A));
	SENSORDB("tvp5150: Interrupt configuration = 0x%02x\n",
			tvp5150_read(TVP5150_INT_CONF));
	SENSORDB("tvp5150: VDP status register = 0x%02x\n",
			tvp5150_read(TVP5150_VDP_STATUS_REG));
	SENSORDB("tvp5150: FIFO word count = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_WORD_COUNT));
	SENSORDB("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_INT_THRESHOLD));
	SENSORDB("tvp5150: FIFO reset = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_RESET));
	SENSORDB("tvp5150: Line number interrupt = 0x%02x\n",
			tvp5150_read(TVP5150_LINE_NUMBER_INT));
	SENSORDB("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			tvp5150_read(TVP5150_PIX_ALIGN_REG_HIGH),
			tvp5150_read(TVP5150_PIX_ALIGN_REG_LOW));
	SENSORDB("tvp5150: FIFO output control = 0x%02x\n",
			tvp5150_read(TVP5150_FIFO_OUT_CTRL));
	SENSORDB("tvp5150: Full field enable = 0x%02x\n",
			tvp5150_read(TVP5150_FULL_FIELD_ENA));
	SENSORDB("tvp5150: Full field mode register = 0x%02x\n",
			tvp5150_read(TVP5150_FULL_FIELD_MODE_REG));

	return 0;
}

static int tvp5150_write_inittab(const struct i2c_reg_value *regs)
{
	while (regs->reg != 0xff) {
		tvp5150_write(regs->reg, regs->value);
		regs++;
	}
	return 0;
}


static void TVP5150InitialSetting(void)
{
    const struct i2c_reg_value *regs = tvp5150_init_enable;

	SENSORDB("[TVP5150YUV]: sensor initial settings Start!\n");  

	/* Initializes TVP5150 to its default values */
	tvp5150_write_inittab(tvp5150_init_default);

	/* Initializes TVP5150 to stream enabled values */
	tvp5150_write_inittab(tvp5150_init_enable);


	SENSORDB("[TVP5150YUV]: sensor initial settings end!\n");  
}   /* TVP5150InitialSetting */

/*************************************************************************
* FUNCTION
*    TVP5150HalfAdjust
*
* DESCRIPTION
*    This function dividend / divisor and use round-up.
*
* PARAMETERS
*    dividend
*    divisor
*
* RETURNS
*    [dividend / divisor]
*
* LOCAL AFFECTED
*
*************************************************************************/
__inline static kal_uint32 TVP5150HalfAdjust(kal_uint32 dividend, kal_uint32 divisor)
{
  return (dividend * 2 + divisor) / (divisor * 2); /* that is [dividend / divisor + 0.5]*/
}

/*************************************************************************
* FUNCTION
*   TVP5150SetShutterStep
*
* DESCRIPTION
*   This function is to calculate & set shutter step register .
*
*************************************************************************/
static void TVP5150SetShutterStep(void)
{       
  const kal_uint8 banding = TVP5150CurrentStatus.iBanding == AE_FLICKER_MODE_50HZ ? TVP5150_NUM_50HZ : TVP5150_NUM_60HZ;
  const kal_uint16 shutter_step = TVP5150HalfAdjust(TVP5150CurrentStatus.iPclk * TVP5150_CLK_1MHZ / 2, (TVP5150CurrentStatus.iHblank + TVP5150_PERIOD_PIXEL_NUMS) * banding);

  if(TVP5150CurrentStatus.iShutterStep == shutter_step)
    return ;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iShutterStep = shutter_step;
  spin_unlock(&tvp5150_drv_lock);
  
  ASSERT(shutter_step <= 0xFF);    
  /* Block 1:0x34  shutter step*/
  TVP5150SetPage(1);
  tvp5150_write(0x34,shutter_step);

  SENSORDB("Set Shutter Step:%x\n",shutter_step);
}/* TVP5150SetShutterStep */

/*************************************************************************
* FUNCTION
*   TVP5150SetFrameCount
*
* DESCRIPTION
*   This function is to set frame count register .
*
*************************************************************************/
static void TVP5150SetFrameCount(void)
{    
  kal_uint16 Frame_Count,min_fps = 100;
  kal_uint8 banding = TVP5150CurrentStatus.iBanding == AE_FLICKER_MODE_50HZ ? TVP5150_NUM_50HZ : TVP5150_NUM_60HZ;

  min_fps = TVP5150CurrentStatus.bNightMode ? TVP5150CurrentStatus.MinFpsNight : TVP5150CurrentStatus.MinFpsNormal;
  Frame_Count = banding * TVP5150_FRAME_RATE_UNIT / min_fps;

  if(TVP5150CurrentStatus.iFrameCount == Frame_Count)
    return ;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iFrameCount = Frame_Count;
  spin_unlock(&tvp5150_drv_lock);  

  SENSORDB("min_fps:%d,Frame_Count:%x\n",min_fps/TVP5150_FRAME_RATE_UNIT,Frame_Count);
  /*Block 01: 0x11  Max shutter step,for Min frame rate */
  TVP5150SetPage(1);
  tvp5150_write(0x11,Frame_Count&0xFF);    
}/* TVP5150SetFrameCount */

/*************************************************************************
* FUNCTION
*   TVP5150ConfigBlank
*
* DESCRIPTION
*   This function is to set Blank size for Preview mode .
*
* PARAMETERS
*   iBlank: target HBlank size
*      iHz: banding frequency
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void TVP5150ConfigBlank(kal_uint16 hblank,kal_uint16 vblank)
{    
  SENSORDB("hblank:%x,vblank:%x\n",hblank,vblank);
   /********************************************    
    *   Register :0x20 - 0x22
    *  Block 00
    *  0x20  [7:4]:HBANK[9:8]; 0x20  [3:0]:VBANK[9:8]
    *  0x21 HBANK[7:0]
    *  0x23 VBANK[7:0]  
    ********************************************/
  if((TVP5150CurrentStatus.iHblank == hblank) && (TVP5150CurrentStatus.iVblank == vblank) )
     return ;

  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iHblank = hblank;
  TVP5150CurrentStatus.iVblank = vblank;
  spin_unlock(&tvp5150_drv_lock); 
   
  ASSERT(hblank <= TVP5150_BLANK_REGISTER_LIMITATION && vblank <= TVP5150_BLANK_REGISTER_LIMITATION);
  TVP5150SetPage(0);
  tvp5150_write(0x20,((hblank>>4)&0xF0)|((vblank>>8)&0x0F));
  tvp5150_write(0x21,hblank & 0xFF);
  tvp5150_write(0x23,vblank & 0xFF);
  TVP5150SetShutterStep();
}   /* TVP5150ConfigBlank */

/*************************************************************************
* FUNCTION
*    TVP5150CalFps
*
* DESCRIPTION
*    This function calculate & set frame rate and fix frame rate when video mode
*    MUST BE INVOKED AFTER SIM120C_preview() !!!
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
static void TVP5150CalFps(void)
{
  kal_uint16 Line_length,Dummy_Line,Dummy_Pixel;
  kal_uint16 max_fps = 300;

  Line_length = TVP5150CurrentStatus.iDummyPixel + TVP5150_PERIOD_PIXEL_NUMS; 

  if (TVP5150CurrentStatus.bVideoMode == KAL_TRUE)
  {    
    max_fps = TVP5150CurrentStatus.bNightMode ? TVP5150CurrentStatus.MinFpsNight: TVP5150CurrentStatus.MinFpsNormal;
  }
  else
  {    
    max_fps = TVP5150_FPS(25);//FAE: 30 cause flicker
  }

  Dummy_Line = TVP5150CurrentStatus.iPclk * TVP5150_CLK_1MHZ * TVP5150_FRAME_RATE_UNIT / (2 * Line_length * max_fps) - TVP5150_PERIOD_LINE_NUMS; 
  if(Dummy_Line > TVP5150_BLANK_REGISTER_LIMITATION)
  {
    Dummy_Line = TVP5150_BLANK_REGISTER_LIMITATION;
    Line_length = TVP5150CurrentStatus.iPclk * TVP5150_CLK_1MHZ * TVP5150_FRAME_RATE_UNIT / (2 * (Dummy_Line + TVP5150_PERIOD_LINE_NUMS) * max_fps);
  }
  Dummy_Pixel = Line_length -  TVP5150_PERIOD_PIXEL_NUMS;

  SENSORDB("max_fps:%d\n",max_fps/TVP5150_FRAME_RATE_UNIT);
  SENSORDB("Dummy Pixel:%x,Hblank:%x,Vblank:%x\n",TVP5150CurrentStatus.iDummyPixel,Dummy_Pixel,Dummy_Line);
  TVP5150ConfigBlank((Dummy_Pixel > 0) ? Dummy_Pixel : 0, (Dummy_Line > 0) ? Dummy_Line : 0);
  TVP5150SetShutterStep();
}


/*************************************************************************
* FUNCTION
*   TVP5150FixFrameRate
*
* DESCRIPTION
*   This function fix the frame rate of image sensor.
*
*************************************************************************/
static void TVP5150FixFrameRate(kal_bool bEnable)
{
  if(TVP5150CurrentStatus.bFixFrameRate == bEnable)
    return ;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.bFixFrameRate = bEnable;
  if(bEnable == KAL_TRUE)
  {   //fix frame rate
    TVP5150CurrentStatus.iControl |= 0xC0;
  }
  else
  {        
    TVP5150CurrentStatus.iControl &= 0x3F;
  }
  spin_unlock(&tvp5150_drv_lock);
  
  //TVP5150SetPage(0);
  //tvp5150_write(0x04,TVP5150CurrentStatus.iControl);
}   /* TVP5150FixFrameRate */

/*************************************************************************
* FUNCTION
*   TVP5150HVmirror
*
* DESCRIPTION
*   This function config the HVmirror of image sensor.
*
*************************************************************************/
static void TVP5150HVmirror(kal_uint8 HVmirrorType)
{    
  if(TVP5150CurrentStatus.iMirror == HVmirrorType)
    return ;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iMirror = HVmirrorType;
  TVP5150CurrentStatus.iControl = TVP5150CurrentStatus.iControl & 0xFC;  
  switch (HVmirrorType) 
  {
    case IMAGE_H_MIRROR:
      TVP5150CurrentStatus.iControl |= 0x01;
      break;
    case IMAGE_V_MIRROR:
      TVP5150CurrentStatus.iControl |= 0x02;
      break;
    case IMAGE_HV_MIRROR:
      TVP5150CurrentStatus.iControl |= 0x03;
      break;
    case IMAGE_NORMAL:
    default:
      TVP5150CurrentStatus.iControl |= 0x00;
  }
  spin_unlock(&tvp5150_drv_lock);
  
#if 0
  TVP5150SetPage(0);
  tvp5150_write(0x04,TVP5150CurrentStatus.iControl);
#endif
}   /* TVP5150HVmirror */

/*************************************************************************
* FUNCTION
*  TVP5150NightMode
*
* DESCRIPTION
*  This function night mode of TVP5150.
*
* PARAMETERS
*  none
*
* RETURNS
*  None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void TVP5150NightMode(kal_bool enable)
{
  SENSORDB("NightMode %d\n",enable);

  if (enable == TVP5150CurrentStatus.bNightMode)
    return ;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.bNightMode = enable;
  spin_unlock(&tvp5150_drv_lock);

  if ( TVP5150CurrentStatus.bVideoMode == KAL_TRUE)// camera mode
    return ;
  
  if (TVP5150CurrentStatus.bNightMode == KAL_TRUE)
  {   /* camera night mode */
    SENSORDB("camera night mode\n");
   
#if 0
    TVP5150SetPage(1);
    tvp5150_write(0x40,0x6C); //Max Analog Gain Value @ Shutter step = Max Shutter step  0x7D
    TVP5150SetPage(3);
    tvp5150_write(0xAB,0x10); //Brightness Control 0x11
    tvp5150_write(0xB9,0x18); //Color Suppression Change Start State  0x18           
    tvp5150_write(0xBA,0x32); //Slope
#endif

  }
  else
  {   /* camera normal mode */
    SENSORDB("camera normal mode\n");

#if 0
    TVP5150SetPage(1);
    tvp5150_write(0x40,0x40);// 0x7F
    TVP5150SetPage(3);
    tvp5150_write(0xAB,0x00); //0x04 
    tvp5150_write(0xB9,0x18);            
    tvp5150_write(0xBA,0x34); //Slope
#endif

  }
  //TVP5150SetFrameCount(); 
}  /* TVP5150NightMode */

/*************************************************************************
* FUNCTION
*  TVP5150Open
*
* DESCRIPTION
*  This function initialize the registers of CMOS sensor
*
* PARAMETERS
*  None
*
* RETURNS
*  None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 TVP5150Open(void)
{

  kal_uint16 sensor_id=0;
  kal_uint16 temp_data;
  u8 msb_id, lsb_id, msb_rom, lsb_rom;

  SENSORDB("TVP5150 Sensor open\n");

	msb_id = tvp5150_read(TVP5150_MSB_DEV_ID);
	lsb_id = tvp5150_read(TVP5150_LSB_DEV_ID);
	msb_rom = tvp5150_read(TVP5150_ROM_MAJOR_VER);
	lsb_rom = tvp5150_read(TVP5150_ROM_MINOR_VER);

	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		SENSORDB("[TVP5150YUV]tvp%02x%02xam1 detected.\n", msb_id, lsb_id);

		/* ITU-T BT.656.4 timing */
		tvp5150_write(TVP5150_REV_SELECT, 0);
	} else {
		if (msb_rom == 3 || lsb_rom == 0x21) { /* Is TVP5150A */
			SENSORDB("[TVP5150YUV]tvp%02x%02xa detected.\n", msb_id, lsb_id);
		} else {
			SENSORDB("[TVP5150YUV]*** unknown tvp%02x%02x chip detected.\n",
					msb_id, lsb_id);
			SENSORDB("[TVP5150YUV]*** Rom ver is %d.%d\n", msb_rom, lsb_rom);
		}
	}


  sensor_id = msb_id << 8 | lsb_id;

  SENSORDB("[TVP5150YUV]TVP5150 Sensor Read ID %x\n",sensor_id);
  if (sensor_id != TVP5150_SENSOR_ID) 
  {
    return ERROR_SENSOR_CONNECT_FAIL;
  }
  
  //TVP5150_set_isp_driving_current(3);

  //TVP5150InitialPara();

  TVP5150InitialSetting();

  //TVP5150CalFps();    

  tvp5150_log_status();

  return ERROR_NONE;
}  /* TVP5150Open() */

/*************************************************************************
* FUNCTION
*  TVP5150Close
*
* DESCRIPTION
*  This function is to turn off sensor module power.
*
* PARAMETERS
*  None
*
* RETURNS
*  None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 TVP5150Close(void)
{
//  CISModulePowerOn(FALSE);
  return ERROR_NONE;
}  /* TVP5150Close() */

/*************************************************************************
* FUNCTION
*  TVP5150Preview
*
* DESCRIPTION
*  This function start the sensor preview.
*
* PARAMETERS
*  *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*  None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 TVP5150Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

  SENSORDB("TVP5150Preview\r\n");
    /* ==Camera Preview, MT6516 use 26MHz PCLK, 30fps == */

  //4  <1> preview of capture PICTURE
  TVP5150FixFrameRate(KAL_FALSE);

  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.MinFpsNormal = TVP5150_FPS(10);
  TVP5150CurrentStatus.MinFpsNight =  TVP5150CurrentStatus.MinFpsNormal >> 1;  
  TVP5150CurrentStatus.bVideoMode =  KAL_FALSE;
  spin_unlock(&tvp5150_drv_lock);	
  
  //4 <2> set mirror and flip
  TVP5150HVmirror(sensor_config_data->SensorImageMirror);

  //4 <3> set dummy pixel, dummy line will calculate from frame rate
  /* TVP5150CurrentStatus.iDummyPixel = 0x1d; */    
  
  // copy sensor_config_data
  memcpy(&TVP5150SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));


    // AE shutter 
#if 0
  TVP5150SetPage(1);
  printk("tvp5150  0x11=0x%x\n",tvp5150_read(0x11));
  tvp5150_write(0x11,0x25);
#endif
 
  
    return ERROR_NONE;
}  /* TVP5150Preview() */

UINT32 TVP5150GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
#if 1
  pSensorResolution->SensorFullWidth = TVP5150_IMAGE_SENSOR_PV_WIDTH+2; //add workaround for VGA sensor
  pSensorResolution->SensorFullHeight = TVP5150_IMAGE_SENSOR_PV_HEIGHT;
  pSensorResolution->SensorPreviewWidth = TVP5150_IMAGE_SENSOR_PV_WIDTH;
  pSensorResolution->SensorPreviewHeight = TVP5150_IMAGE_SENSOR_PV_HEIGHT;
#else
  pSensorResolution->SensorFullWidth = TVP5150_IMAGE_SENSOR_PV_WIDTH+2;
  pSensorResolution->SensorFullHeight = TVP5150_IMAGE_SENSOR_PV_HEIGHT;
  pSensorResolution->SensorPreviewWidth = TVP5150_IMAGE_SENSOR_PV_WIDTH;
  pSensorResolution->SensorPreviewHeight = TVP5150_IMAGE_SENSOR_PV_HEIGHT;

#endif
  SENSORDB("TVP5150GetResolution %d, %d, %d, %d\r\n", 
      pSensorResolution->SensorFullWidth,
      pSensorResolution->SensorFullHeight,
      pSensorResolution->SensorPreviewWidth,
      pSensorResolution->SensorPreviewHeight);

  return ERROR_NONE;
}  /* TVP5150GetResolution() */

UINT32 TVP5150GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
            MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
            MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
  SENSORDB("TVP5150GetInfo %d\r\n", ScenarioId);

  pSensorInfo->SensorPreviewResolutionX = TVP5150_IMAGE_SENSOR_PV_WIDTH;
  pSensorInfo->SensorPreviewResolutionY = TVP5150_IMAGE_SENSOR_PV_HEIGHT;
  pSensorInfo->SensorFullResolutionX = TVP5150_IMAGE_SENSOR_PV_WIDTH;
  pSensorInfo->SensorFullResolutionY = TVP5150_IMAGE_SENSOR_PV_HEIGHT;

  pSensorInfo->SensorCameraPreviewFrameRate=30;
  pSensorInfo->SensorVideoFrameRate=30;
  pSensorInfo->SensorStillCaptureFrameRate=30;
  pSensorInfo->SensorWebCamCaptureFrameRate=30;
  pSensorInfo->SensorResetActiveHigh=FALSE;
  pSensorInfo->SensorResetDelayCount=1;
  pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_VYUY;
  pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
  pSensorInfo->SensorInterruptDelayLines = 1;
  pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
  pSensorInfo->SensorMasterClockSwitch = 0; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_4MA;
  pSensorInfo->CaptureDelayFrame = 1; 
  pSensorInfo->PreviewDelayFrame = 3; 
  pSensorInfo->VideoDelayFrame = 5; 
  switch (ScenarioId)
  {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
    default:      
      pSensorInfo->SensorClockFreq=27;
      pSensorInfo->SensorClockDividCount=  3;
      pSensorInfo->SensorClockRisingCount= 0;
      pSensorInfo->SensorClockFallingCount= 2;
      pSensorInfo->SensorPixelClockCount= 3;
      pSensorInfo->SensorDataLatchCount= 2;
      pSensorInfo->SensorGrabStartX = 1; 
      pSensorInfo->SensorGrabStartY = 1;       
      break;
  }
  memcpy(pSensorConfigData, &TVP5150SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  return ERROR_NONE;
}  /* TVP5150GetInfo() */

UINT32 TVP5150Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
            MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

  SENSORDB("TVP5150Control %d\n", ScenarioId);	
  SENSORDB("   camera preview %d\n", MSDK_SCENARIO_ID_CAMERA_PREVIEW);	
  SENSORDB("   video preview %d\n", MSDK_SCENARIO_ID_VIDEO_PREVIEW);	
  SENSORDB("   camera captrue %d\n", MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG);	
  switch (ScenarioId)
  {
    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
    //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
    //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
    default:
      TVP5150Preview(pImageWindow, pSensorConfigData);
      break;
  }
  return ERROR_NONE;
}  /* TVP5150Control() */

UINT32 TVP5150GetSensorID(UINT32 *sensorID) 
{
	
	SENSORDB("TVP5150GetSensorID\n");	

	*sensorID = tvp5150_read(TVP5150_MSB_DEV_ID)<<8| tvp5150_read(TVP5150_LSB_DEV_ID);

	SENSORDB("TVP5150 Sensor Read ID %x\n",*sensorID);
	if (*sensorID != TVP5150_SENSOR_ID) 
	{
      *sensorID=0xFFFFFFFF;
	  return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	return ERROR_NONE;
}
BOOL TVP5150SetParamWB(UINT16 para)
{
  SENSORDB("WB %d\n",para);
  if(TVP5150CurrentStatus.iWB== para)
    return FALSE;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iWB = para;
  spin_unlock(&tvp5150_drv_lock);	
   
  TVP5150SetPage(2);
  switch (para)
  {
    case AWB_MODE_OFF:
      tvp5150_write(0x10, 0x00);
      break;
    case AWB_MODE_AUTO:
      tvp5150_write(0x10, 0xD3);
      break;
    case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
      tvp5150_write(0x10, 0x00);
      tvp5150_write(0x60, 0xD0);
      tvp5150_write(0x61, 0x88);
      break;
    case AWB_MODE_DAYLIGHT: //sunny
      tvp5150_write(0x10, 0x00);
      tvp5150_write(0x60, 0xC2);
      tvp5150_write(0x61, 0x9E);
      break;
    case AWB_MODE_INCANDESCENT: //office
      tvp5150_write(0x10, 0x00);
      tvp5150_write(0x60, 0x98);
      tvp5150_write(0x61, 0xC8);
      break;
    case AWB_MODE_TUNGSTEN: //home
      tvp5150_write(0x10, 0x00);
      tvp5150_write(0x60, 0x90);
      tvp5150_write(0x61, 0xC0);
      break;
    case AWB_MODE_FLUORESCENT:
      tvp5150_write(0x10, 0x00);
      tvp5150_write(0x60, 0xAA);
      tvp5150_write(0x61, 0xBE);
      break;
    default:
      return FALSE;
  }

  return TRUE;
} /* TVP5150SetParamWB */

BOOL TVP5150SetParamAE(UINT16 para)
{
  SENSORDB("AE %d\n",para);
  if(TVP5150CurrentStatus.iAE== para)
    return FALSE;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iAE = para;
  spin_unlock(&tvp5150_drv_lock);
  
  TVP5150SetPage(1);
  if(KAL_TRUE == para)
    tvp5150_write(0x10,0x80);
  else
    tvp5150_write(0x10,0x00);
  return TRUE;
} /* TVP5150SetParamAE */

BOOL TVP5150SetParamEffect(UINT16 para)
{
  SENSORDB("Effect %d\n",para);
  if(TVP5150CurrentStatus.iEffect== para)
    return FALSE;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iEffect = para;
  spin_unlock(&tvp5150_drv_lock);
  
  TVP5150SetPage(3);
  switch (para)
  {
    case MEFFECT_OFF:
      tvp5150_write(0xB6, 0x00);
      break;
    case MEFFECT_SEPIA:
      tvp5150_write(0xB6, 0x80); 
      tvp5150_write(0xB7, 0x60);
      tvp5150_write(0xB8, 0xA0);
      break;
    case MEFFECT_NEGATIVE:
      tvp5150_write(0xB6, 0x20);
      break;
    case MEFFECT_SEPIAGREEN:
      tvp5150_write(0xB6, 0x80); 
      tvp5150_write(0xB7, 0x50);
      tvp5150_write(0xB8, 0x50);
      break;
    case MEFFECT_SEPIABLUE:
      tvp5150_write(0xB6, 0x80); 
      tvp5150_write(0xB7, 0xC0);
      tvp5150_write(0xB8, 0x60);
      break;  
    case MEFFECT_MONO: //B&W
      tvp5150_write(0xB6, 0x40);
      break;
    default:
      return FALSE;
  }
  return TRUE;
} /* TVP5150SetParamEffect */

BOOL TVP5150SetParamBanding(UINT16 para)
{
  SENSORDB("Banding %d\n",para);
  if(TVP5150CurrentStatus.iBanding== para)
    return TRUE;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iBanding = para;
  spin_unlock(&tvp5150_drv_lock);
  
  TVP5150SetShutterStep();
  //TVP5150SetFrameCount(); 
  return TRUE;
} /* TVP5150SetParamBanding */

BOOL TVP5150SetParamEV(UINT16 para)
{
  SENSORDB("Exporsure %d\n",para);
  if(TVP5150CurrentStatus.iEV== para)
    return FALSE;
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.iEV = para;  
  spin_unlock(&tvp5150_drv_lock);

  TVP5150SetPage(3);
  switch (para)
  {
    case AE_EV_COMP_n13:
      tvp5150_write(0xAB,0xE0);
      break;
    case AE_EV_COMP_n10:
      tvp5150_write(0xAB,0xD0);
      break;
    case AE_EV_COMP_n07:
      tvp5150_write(0xAB,0xC0);
      break;
    case AE_EV_COMP_n03:
      tvp5150_write(0xAB,0xB0);
      break;
    case AE_EV_COMP_00:
      tvp5150_write(0xAB,0x00);
      break;
    case AE_EV_COMP_03:
      tvp5150_write(0xAB,0x08);
      break;
    case AE_EV_COMP_07:
      tvp5150_write(0xAB,0x10);
      break;
    case AE_EV_COMP_10:
      tvp5150_write(0xAB,0x20);
      break;
    case AE_EV_COMP_13:
      tvp5150_write(0xAB,0x30);
      break;
    default:
      return FALSE;
  }
  return TRUE;
} /* TVP5150SetParamEV */

UINT32 TVP5150YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
  SENSORDB("TVP5150YUVSensorSetting featrue id %d\n", iCmd);
  switch (iCmd) {
    case FID_SCENE_MODE:      
      if (iPara == SCENE_MODE_OFF)
      {
          //TVP5150NightMode(FALSE); 
      }
      else if (iPara == SCENE_MODE_NIGHTSCENE)
      {
           //TVP5150NightMode(TRUE); 
      }      
      break;      
    case FID_AWB_MODE:
      //TVP5150SetParamWB(iPara);
      break;      
    case FID_COLOR_EFFECT:
      //TVP5150SetParamEffect(iPara);
      break;      
    case FID_AE_EV:
      //TVP5150SetParamEV(iPara);
      break;      
    case FID_AE_FLICKER:
      //TVP5150SetParamBanding(iPara);
      break;      
    case FID_AE_SCENE_MODE: 
      //TVP5150SetParamAE(iPara);
       break;       
    default:
      break;
  }
  return TRUE;
}   /* TVP5150YUVSensorSetting */

UINT32 TVP5150YUVSetVideoMode(UINT16 u2FrameRate)
{
    SENSORDB("TVP5150YUVSetVideoMode SetVideoMode %d\n",u2FrameRate);

  TVP5150FixFrameRate(KAL_TRUE);
  
  spin_lock(&tvp5150_drv_lock);
  TVP5150CurrentStatus.bVideoMode = KAL_TRUE;
  TVP5150CurrentStatus.MinFpsNormal = TVP5150_FPS(25);//FAE: 30 cause flicker
  TVP5150CurrentStatus.MinFpsNight =  TVP5150CurrentStatus.MinFpsNormal >> 1; 
  spin_unlock(&tvp5150_drv_lock);
  
  if (u2FrameRate == 25)
  {
    spin_lock(&tvp5150_drv_lock);
    TVP5150CurrentStatus.bNightMode = KAL_FALSE;
	spin_unlock(&tvp5150_drv_lock);
  }
  else if (u2FrameRate == 12)       
  {
  //VideoCamera.java 
  //(mIsVideoNightMode ? mProfile.videoBitRate / 2 : mProfile.videoBitRate)
    spin_lock(&tvp5150_drv_lock);
    TVP5150CurrentStatus.bNightMode = KAL_TRUE;
	spin_unlock(&tvp5150_drv_lock);
  }
  else 
  {
    printk("Wrong frame rate setting \n");
  return FALSE;
  }   
  //TVP5150SetPage(1);
  //tvp5150_write(0x40,0x58);//Max Analog Gain Value @ Shutter step = Max Shutter step  0x7D
  //TVP5150CalFps();    
  //TVP5150SetFrameCount();    
  return TRUE;
}

UINT32 TVP5150YUVSetSoftwarePWDNMode(kal_bool bEnable)
{
    SENSORDB("[TVP5150YUVSetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);
    
    if(bEnable) {   // enable software power down mode   
	 tvp5150_write(TVP5150_OP_MODE_CTL, 0x00);
    } else {
        tvp5150_write(TVP5150_OP_MODE_CTL, 0x01);  
    }
    return TRUE;
}

void TVP5150GetAFMaxNumFocusAreas(UINT32 *pFeatureReturnPara32)
{	
    *pFeatureReturnPara32 = 0;    
    SENSORDB("TVP5150GetAFMaxNumFocusAreas *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);

}

UINT32 TVP5150FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
               UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
  UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
  UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
  UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
  UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
  MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
  
  SENSORDB("TVP5150FeatrueControl = %d\n",  FeatureId);

  switch (FeatureId)
  {
    case SENSOR_FEATURE_GET_RESOLUTION:
      *pFeatureReturnPara16++=TVP5150_IMAGE_SENSOR_PV_WIDTH;
      *pFeatureReturnPara16=TVP5150_IMAGE_SENSOR_PV_HEIGHT;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_GET_PERIOD:
      *pFeatureReturnPara16++=TVP5150_PERIOD_PIXEL_NUMS+TVP5150CurrentStatus.iHblank;
      *pFeatureReturnPara16=TVP5150_PERIOD_LINE_NUMS+TVP5150CurrentStatus.iVblank;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
      *pFeatureReturnPara32 = TVP5150CurrentStatus.iPclk;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_SET_ESHUTTER:
      break;
    case SENSOR_FEATURE_SET_NIGHTMODE:
      //TVP5150NightMode((BOOL) *pFeatureData16);
      break;
    case SENSOR_FEATURE_SET_GAIN:
    case SENSOR_FEATURE_SET_FLASHLIGHT:
      break;
    case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
      break;
    case SENSOR_FEATURE_SET_REGISTER:
      tvp5150_write(pSensorRegData->RegAddr, pSensorRegData->RegData);
      break;
    case SENSOR_FEATURE_GET_REGISTER:
      pSensorRegData->RegData = tvp5150_read(pSensorRegData->RegAddr);
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
    case SENSOR_FEATURE_GET_GROUP_COUNT:
    case SENSOR_FEATURE_GET_GROUP_INFO:
    case SENSOR_FEATURE_GET_ITEM_INFO:
    case SENSOR_FEATURE_SET_ITEM_INFO:
    case SENSOR_FEATURE_GET_ENG_INFO:
      break;
    case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
      // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
      // if EEPROM does not exist in camera module.
      *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
      *pFeatureParaLen=4;
      break;
    case SENSOR_FEATURE_SET_YUV_CMD:
      TVP5150YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
      break;
    case SENSOR_FEATURE_SET_VIDEO_MODE:
      TVP5150YUVSetVideoMode(*pFeatureData16);
      break;
    case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
      TVP5150YUVSetSoftwarePWDNMode((BOOL)*pFeatureData16);        	        	
      break;
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
	    TVP5150GetSensorID(pFeatureReturnPara32); 
	    break; 
    case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
        TVP5150GetAFMaxNumFocusAreas(pFeatureReturnPara32);            
        *pFeatureParaLen=4;
        break; 
    default:
      break;      
  }
  return ERROR_NONE;
}  /* TVP5150FeatureControl() */

  static SENSOR_FUNCTION_STRUCT  SensorFuncTVP5150=
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
