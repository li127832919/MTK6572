/* linux/drivers/hwmon/LSM303D.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * LSM303D driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#ifndef LIS3MDL
#define LIS3MDL
#include <linux/ioctl.h>

/* Magnetometer Sensor Full Scale */
// conversion of magnetic data to uT units
#define CONVERT_M                   (1.0f/10.0f)
#define CONVERT_M_X                 (CONVERT_M)
#define CONVERT_M_Y                 (CONVERT_M)
#define CONVERT_M_Z                 (CONVERT_M)

#define	I2C_AUTO_INCREMENT	(0x80)

#define LIS3MDL_I2C_SLAVE_ADDR_WRITE 0x3C    //SDO pin High 0x3c--->SDO GND 0x38

#define LIS3MDL_REG_DEVID			0x0F
#define LIS3MDL_FIXED_DEVID			0x3D

//REG MAPPING
#define	LIS3MDL_REG_CTL1			0x20
#define	LIS3MDL_REG_CTL2			0x21
#define	LIS3MDL_REG_CTL3			0x22
#define	LIS3MDL_REG_CTL4			0x23
#define	LIS3MDL_REG_CTL5			0x24

#define	LIS3MDL_REG_STATUS			0x27

#define LIS3MDL_REG_DATAXL		    0x28
#define LIS3MDL_REG_DATAXH		    0x29
#define LIS3MDL_REG_DATAYL		    0x2A
#define LIS3MDL_REG_DATAYH		    0x2B
#define LIS3MDL_REG_DATAZL		    0x2C
#define LIS3MDL_REG_DATAZH		    0x2D

#define	LIS3MDL_REG_STATUS2			0x30

#define LIS3MDL_REG_INT_SRC			0x31
#define LIS3MDL_REG_INT_THS_L		0x32
#define LIS3MDL_REG_INT_THS_H		0x33

//REG_CTRL1
#define LIS3MDL_XY_MODE_LOW_POWER	0x00
#define LIS3MDL_XY_MODE_MEASURE	    0x60

#define LIS3MDL_MAG_ODR10	  (0x10)  /* 50Hz output data rate */
#define LIS3MDL_MAG_ODR80	  (0x1c)  /* 100Hz output data rate */

//REG_CTRL2
#define LIS3MDL_MAG_FS_4G	(0x00)	/* Full scale 4 gauss */
#define LIS3MDL_MAG_FS_8G	(0x20)	/* Full scale 8 gauss */
#define LIS3MDL_MAG_FS_12G	(0x40)	/* Full scale 12 gauss */
#define LIS3MDL_MAG_FS_16G	(0x60)	/* Full scale 16 gauss */

//REG_CTRL3
#define LIS3MDL_MAG_POWER_ON	  (0x00)  /* POWER ON */
#define LIS3MDL_MAG_POWER_OFF	  (0x03)  /* POWER DOWN */

//REG_CTRL4
#define LIS3MDL_Z_MODE_LOW_POWER	0x00
#define LIS3MDL_Z_MODE_MEASURE	    0x0C

//REG_CTRL5
//donot set,use default,continuous update


#define LIS3MDL_SUCCESS						0
#define LIS3MDL_ERR_I2C						-1
#define LIS3MDL_ERR_STATUS					-3
#define LIS3MDL_ERR_SETUP_FAILURE			-4
#define LIS3MDL_ERR_GETGSENSORDATA			-5
#define LIS3MDL_ERR_IDENTIFICATION			-6
	  
	  
	  
#define LIS3MDL_BUFSIZE				256


#endif


