#ifndef __SENSOR_H
#define __SENSOR_H

#include "image_sensor.h"//get IMAGE_SENSOR_DRVNAME

#include "tvp5150_reg.h"

#define IMAGE_SENSOR_DRVNAME SENSOR_DRVNAME_TVP5150_YUV

/* START GRAB PIXEL OFFSET */
#define TVP5150_IMAGE_SENSOR_START_GRAB_X   (1)
#define TVP5150_IMAGE_SENSOR_START_GRAB_Y   (1)

/* SENSOR PV SIZE */
#define TVP5150_IMAGE_SENSOR_PV_WIDTH       (640 - 16)
#define TVP5150_IMAGE_SENSOR_PV_HEIGHT      (480 - 12)


/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define TVP5150_PERIOD_PIXEL_NUMS          (652 + 144 + 3)/* Active + HST + 3;default pixel#(w/o dummy pixels) in VGA mode*/
#define TVP5150_PERIOD_LINE_NUMS           (490 + 9)      /* Active + 9 ;default line#(w/o dummy lines) in VGA mode*/

#define TVP5150_BLANK_REGISTER_LIMITATION   0xFFF

/*50Hz,60Hz*/
#define TVP5150_NUM_50HZ                    (50 * 2)
#define TVP5150_NUM_60HZ                    (60 * 2)
#define TVP5150_CLK_1MHZ                    (1000000)

/* FRAME RATE UNIT */
#define TVP5150_FRAME_RATE_UNIT              10
#define TVP5150_FPS(x)                       (TVP5150_FRAME_RATE_UNIT * (x))

/* SENSOR READ/WRITE ID */
#define TVP5150_WRITE_ID                     0xB9

/* SENSOR CHIP VERSION */
#define TVP5150_SENSOR_VERSION               TVP5150_ROM_MAJOR_VER
#define TVP5150_SENSOR_VERSION               TVP5150_ROM_MINOR_VER

//export functions
UINT32 TVP5150Open(void);
UINT32 TVP5150GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 TVP5150GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 TVP5150Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 TVP5150FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 TVP5150Close(void);
#endif /* __SENSOR_H */
