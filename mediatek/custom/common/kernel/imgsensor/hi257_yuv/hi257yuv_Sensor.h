/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   Anyuan Huang (MTK70663)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H
    
#define HI257_WRITE_ID        0x40

#define HI257_GRAB_START_X    (1)
#define HI257_GRAB_START_Y    (1)
#define HI257_PV_WIDTH        (800 - 8)
#define HI257_PV_HEIGHT       (600 - 6)
#define HI257_FULL_WIDTH      (1600-16)
#define HI257_FULL_HEIGHT     (1200-12)

/* Sesnor Pixel/Line Numbers in One Period */  
#define HI257_PV_PERIOD_PIXEL_NUMS      (816)    /* Default preview line length */
#define HI257_PV_PERIOD_LINE_NUMS       (612)     /* Default preview frame length */
#define HI257_FULL_PERIOD_PIXEL_NUMS    (1620)    /* Default full size line length */
#define HI257_FULL_PERIOD_LINE_NUMS     (1220)    /* Default full size frame length */

/* Sensor Exposure Line Limitation */
#define HI257_PV_EXPOSURE_LIMITATION        (0x750)
#define HI257_FULL_EXPOSURE_LIMITATION      (0xfa0)

#define HI257_FRAME_RATE_UNIT         10
#define HI257_FPS(x)                  (HI257_FRAME_RATE_UNIT * (x))
#define HI257_MAX_FPS                 (HI257_FRAME_RATE_UNIT * 30)
typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_VIDEO,
    SENSOR_MODE_ZSD,
    SENSOR_MODE_CAPTURE
} HI257_SENSOR_MODE;

UINT32 HI257Open(void);
UINT32 HI257GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 HI257GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI257Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 HI257FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 HI257Close(void);
#endif /* __SENSOR_H */
