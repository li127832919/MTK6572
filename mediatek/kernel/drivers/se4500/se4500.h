/*
 * include/media/se4500.h
 *
 * SE4500 sensor driver
 *
 * Copyright (C) 2011 Motorola Solutions, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef	_SE4500_H_
#define	_SE4500_H_

#include <media/v4l2-subdev.h>
#include <media/media-entity.h>

#define USE_SE4500	// Comment this out to revert back to the MT9T111 reference code

#define SE4500_MODULE_NAME			"se4500"		// Name of this module
#define SE4500_MISC_NAME			"moto_sdl"		// Name used to register as 'misc' device

#define SE4500_I2C_ADDR				0x5C

#define SE4500_CLK_MAX				96000000	// 96MHz
#define SE4500_CLK_MIN				6000000		// 6Mhz

#define SE4500_I2C_CONFIG			1

#define SE4500_SENSOR_WIDTH			752
#define SE4500_SENSOR_HEIGHT		480

// SE4500 commands
#define SE45OP_WRITEREGISTER		0x50
#define SE45OP_READREGISTER			0x51
#define SE45OP_AIM					0x55
#define SE45OP_AIMONEXPOSURE		0x56
#define SE45OP_RESET				0x57
#define SE45OP_ARMACQUISITION		0x58
#define SE45OP_ILLUMDURINGEXPOSURE	0x59
#define SE45OP_ACQUISITIONMODE		0x5B
#define SE45OP_FRAMERATE			0x5E
#define SE45OP_GETPARAM				0x70
#define SE45OP_SETPARAM				0x71
#define SE45OP_FATMODE				0x78
#define SE45OP_AUTOPOWERREDUCTION	0x74
#define SE45OP_TIMETOLOWPOWER		0x75
#define SE45OP_WRITESCRIPT			0x76
#define SE45OP_EXECSCRIPT			0x77
#define SE45OP_PICKLIST				0x7B
#define SE45OP_ILLUMPOWER			0xF0
#define SE45OP_EXTILLUMMODE			0xF1
#define SE45OP_AIMPOWER				0xF3

#define SE45_MAX_CMD_LEN			48

// SE4500 command status values
#define SE45STS_ACK					0x80

// SE45OP_RESET modifiers
#define RESET_SENSOR				0
#define RESET_SE4500				1

// SE4500 Parameter ID's
#define SE45PARAM_MODELNO			0
#define SE45PARAM_MODELNO_LEN		18
#define SE45PARAM_SERIALNO			1
#define SE45PARAM_SERIALNO_LEN		16
#define SE45PARAM_DPMCAL			12
#define SE45PARAM_DPMCAL_LEN		16

// SE4500 Sensor Registers
#define SE45REG_AE					0xAF
#define SE45REG_GAIN				0x35
#define SE45REG_EXP					0x0B
#define SE45REG_EXPCAP				0xBD

// SE45OP_ACQUISITIONMODE option values
#define ACQMODE_DECODE				0
#define ACQMODE_IMAGE				1
#define ACQMODE_MOTION				2
#define ACQMODE_AIMCAPTURE			3

struct se4500_platform_data
{
	int (*s_power)(struct v4l2_subdev* subdev, u32 on);
	int (*set_xclk)(struct v4l2_subdev* subdev, u32 hz);
	int (*configure_interface)(struct v4l2_subdev* subdev, u32 pixclk);
};

#endif	// ifndef _SE4500_H_

