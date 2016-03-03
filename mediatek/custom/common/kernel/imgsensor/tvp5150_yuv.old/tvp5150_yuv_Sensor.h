/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H


/* DUMMY NEEDS TO BE INSERTED */
/* SETUP TIME NEED TO BE INSERTED */
#include "tvp5150_reg.h"


/* SENSOR READ/WRITE ID */
#define TVP5150_WRITE_ID_0			(0xB8)
#define TVP5150_READ_ID_0			(0xB9)

#define TVP5150_WRITE_ID_1			(0xBA)
#define TVP5150_READ_ID_1			(0xBB)



#define TVP5150_PV_PERIOD_PIXEL_NUMS			(3336)			/* Default preview line length */
#define TVP5150_PV_PERIOD_LINE_NUMS 			(857)		/* Default preview frame length */
#define TVP5150_FULL_PERIOD_PIXEL_NUMS			(6809)		/* Default full size line length */
#define TVP5150_FULL_PERIOD_LINE_NUMS			(1629)		/* Default full size frame length */


/* Sensor Exposure Line Limitation */
#define TVP5150_PV_EXPOSURE_LIMITATION      	(618)
#define TVP5150_FULL_EXPOSURE_LIMITATION    	(1236)

/* Sensor Preview Size (3M: 1024x768 or 640x480, 2M: 800x600, 1,3M: 640x512, VGA: 640x480, CIF: 352x288) */
#define TVP5150_IMAGE_SENSOR_PV_WIDTH   		(720) //(640) //(1024)
#define TVP5150_IMAGE_SENSOR_PV_HEIGHT  		(576) //(480) //(768)

/* Sensor Capture Size (3M: 2048x1536, 2M: 1600x1200, 1.3M: 1280x1024, VGA: 640x480, CIF: 352x288) */
#define TVP5150_IMAGE_SENSOR_FULL_WIDTH     	(720)
#define TVP5150_IMAGE_SENSOR_FULL_HEIGHT    	(576)

/* Config the ISP grab start x & start y, Config the ISP grab width & height */
#define TVP5150_PV_GRAB_START_X 				(8)
#define TVP5150_PV_GRAB_START_Y  				(6)
#define TVP5150_PV_GRAB_WIDTH					(TVP5150_IMAGE_SENSOR_PV_WIDTH - 16)  //(TVP5150_IMAGE_SENSOR_PV_WIDTH - 16)
#define TVP5150_PV_GRAB_HEIGHT					(TVP5150_IMAGE_SENSOR_PV_HEIGHT - 12) //(TVP5150_IMAGE_SENSOR_PV_HEIGHT - 12)

#define TVP5150_FULL_GRAB_START_X   			(16)
#define TVP5150_FULL_GRAB_START_Y	  			(12)
#define TVP5150_FULL_GRAB_WIDTH					(TVP5150_IMAGE_SENSOR_FULL_WIDTH - 32)
#define TVP5150_FULL_GRAB_HEIGHT				(TVP5150_IMAGE_SENSOR_FULL_HEIGHT - 24)

static kal_uint8 TVP5150_i2c_addr[] = 
{
	TVP5150_WRITE_ID_0,			/* Slave address0, Write ID */
	TVP5150_WRITE_ID_1,			/* Slave address1, Write ID */
};

typedef struct
	{
		kal_uint16	video_target_width;
		kal_uint16	video_target_height;

		kal_bool	MJPEG_encode_mode;			/* Motion JPEG */
		kal_bool	MPEG4_encode_mode;			/* MJPEG4 JPEG */
		kal_bool	FULLVIDEO_encode_mode;		/* 3G Video Call */

		kal_bool	sensor_cap_state;			/* Preview or Capture mode */
		kal_bool	is_PV_mode; 				/* PV size or Full size */
		kal_bool	is_panorama_capturing;		/* 3G Video Call */

		kal_uint32	curr_banding;				/* 50Hz/60Hz */
		kal_bool	night_mode;
	} TVP5150_OPERATION_STATE_ST;
	
	typedef struct
	{
		kal_uint8	i2c_write_id;
		kal_uint8	i2c_read_id;

		kal_uint32	pv_shutter;
		kal_uint32	pv_extra_shutter;
		kal_uint32	pv_sensor_gain;

		kal_uint32	pv_dummy_pixels;
		kal_uint32	pv_dummy_lines;
		kal_uint32	cap_dummy_pixels;
		kal_uint32	cap_dummy_lines;

		/* Preview & Capture Pixel Clock, 360 means 36.0MHz. Unit Multiple 10. */
		kal_uint32	preview_pclk;
		kal_uint32	capture_pclk;

		/* Video frame rate 300 means 30.0fps. Unit Multiple 10. */
		kal_uint32	video_frame_rate;	
	}TVP5150_SENSOR_INFO_ST;



	/* SENSOR CHIP VERSION */
//	#define TVP5150_SENSOR_ID    (0x364C)    // rev.2C




//s_add for porting
//s_add for porting
//s_add for porting

//export functions
UINT32 TVP5150Open(void);
UINT32 TVP5150GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 TVP5150GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 TVP5150Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 TVP5150FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 TVP5150Close(void);


//e_add for porting
//e_add for porting
//e_add for porting


#endif /* __SENSOR_H */
