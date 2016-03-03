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
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
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

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #define print(x...) printf(x)
#else
    #include <linux/string.h>
    #include <mach/mt_gpio.h>
    #define print(x...) printk(x)
#endif

#include <cust_gpio_usage.h>
#include "lcm_drv.h"

#define _P1213_

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifdef _P1213_
	#define FRAME_WIDTH  (240)
	#define FRAME_HEIGHT (320)
	#define _BACKLIGHT_PIN		GPIO134
	//#define HONGJIA_LCM_SEL
#else
	#define FRAME_WIDTH  (800)
	#define FRAME_HEIGHT (480)
	//#define _BACKLIGHT_PIN		GPIO76
	#define _BACKLIGHT_PIN		GPIO67
#endif



#define BL24086_RESET_PIN	GPIO59
#define BL24086_CS_PIN		GPIO60
#define BL24086_SCL_PIN		GPIO21 // GPIO113
#define BL24086_SDA_PIN		GPIO20 // GPIO114

#define BL24086_CLK_PIN		GPIO55
#define BL24086_DE_PIN		GPIO56
#define BL24086_VSYNC_PIN	GPIO57
#define BL24086_HSYNC_PIN	GPIO58

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define HXLCM_SCL_LOW()		mt_set_gpio_out(BL24086_SCL_PIN, GPIO_OUT_ZERO)
#define HXLCM_SCL_HIGH()	mt_set_gpio_out(BL24086_SCL_PIN, GPIO_OUT_ONE)
#define HXLCM_SDA_LOW()		mt_set_gpio_out(BL24086_SDA_PIN, GPIO_OUT_ZERO)
#define HXLCM_SDA_HIGH()	mt_set_gpio_out(BL24086_SDA_PIN, GPIO_OUT_ONE)
#define HXLCM_CS_LOW()		mt_set_gpio_out(BL24086_CS_PIN, GPIO_OUT_ZERO)
#define HXLCM_CS_HIGH()		mt_set_gpio_out(BL24086_CS_PIN, GPIO_OUT_ONE)

//#ifdef _P1213_
//#define BACKLIGHT_HIGH()
//#define BACKLIGHT_LOW()
//#else
#define BACKLIGHT_HIGH()	mt_set_gpio_out(_BACKLIGHT_PIN, GPIO_OUT_ONE)
#define BACKLIGHT_LOW()		mt_set_gpio_out(_BACKLIGHT_PIN, GPIO_OUT_ZERO)
//#endif
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

static void write_command(unsigned char val)
{
	int i;

	//HXLCM_CS_HIGH();
	//UDELAY(1);
    HXLCM_CS_LOW();
    UDELAY(20);
	HXLCM_SCL_LOW();
	UDELAY(20);
	//HXLCM_CS_LOW();
	//UDELAY(1);

	HXLCM_SDA_LOW();
	UDELAY(20);
	HXLCM_SCL_HIGH();
	UDELAY(20);

	for (i=7;i>=0;i--) {
		HXLCM_SCL_LOW();
		UDELAY(20);
		if((val>>i)& 0x01) {
			HXLCM_SDA_HIGH();
			UDELAY(20);
		} else {
			HXLCM_SDA_LOW();
			UDELAY(20);
		}
		HXLCM_SCL_HIGH();
		UDELAY(20);
	}
	
	HXLCM_CS_HIGH();
	UDELAY(20);
}

static void write_data(unsigned char val)
{
	int i;
 
	HXLCM_CS_LOW();
	UDELAY(20);
	HXLCM_SCL_LOW();
	UDELAY(20);
	//HXLCM_CS_LOW();
	//UDELAY(1);

	HXLCM_SDA_HIGH();
	UDELAY(20);
	HXLCM_SCL_HIGH();
	UDELAY(20);

	for (i=7;i>=0;i--) {
		HXLCM_SCL_LOW();
		UDELAY(20);
		if((val>>i)& 0x01) {
			HXLCM_SDA_HIGH();
			UDELAY(20);
		} else {
			HXLCM_SDA_LOW();
			UDELAY(20);
		}

		HXLCM_SCL_HIGH();
		UDELAY(20);
	}

	//HXLCM_SCL_HIGH();
	//UDELAY(1);
	HXLCM_CS_HIGH();
	UDELAY(20);
}

void ST7789SCMI24panelinitialcode(void)
{
    //-----------------------------------ST7789S reset sequence------------------------------------//
    /*
    LCD_RESET=1;
    MDELAY(1); //Delay 1ms
    LCD_RESET=0;
    MDELAY(10); //Delay 10ms
    LCD_RESET=1;
    MDELAY(120); //Delay 120ms
    */
    //---------------------------------------------------------------------------------------------------//
    write_command(0x11);
    MDELAY(120); //Delay 120ms
    //--------------------------------Display and color format setting-------------------
    write_command(0x36);
    write_data(0x00);
    write_command(0x3A);// Pixel format setting of RGB/MPU interface
    write_data(0x66);// 18 bits/pixel
    UDELAY(50);
    write_command(0xb0); //RGB interface signal control , DE SYNC MODE , Polarity of HSYNC VSYNC DOTCLK DEN , display path via memory or not
    write_data(0x11);
    write_data(0xC0);//DEN high enable , dotclk rising edge, HSYNC & VSYNC low level clock .
    UDELAY(50);
    //--------------------------------ST7789S Frame rate setting----------------------------------//
    write_command(0xb2);
    write_data(0x0c);
    write_data(0x0c);
    write_data(0x00);
    write_data(0x33);
    write_data(0x33);
    write_command(0xb7);
    write_data(0x35);
    //---------------------------------ST7789S Power setting--------------------------------------//
    write_command(0xbb);
    write_data(0x2b);
    write_command(0xc0);
    write_data(0x2c);
    write_command(0xc2);
    write_data(0x01);
    write_command(0xc3);
    write_data(0x17);
    write_command(0xc4);
    write_data(0x20);
    write_command(0xc6);
    write_data(0x0f);
    write_command(0xca);
    write_data(0x0f);
    write_command(0xc8);
    write_data(0x08);
    write_command(0x55);
    write_data(0x90);
    write_command(0xd0);
    write_data(0xa4);
    write_data(0xa1);
    //--------------------------------ST7789S gamma setting---------------------------------------//
    write_command(0xe0);
    write_data(0xf0);
    write_data(0x00);
    write_data(0x0a);
    write_data(0x10);
    write_data(0x12);
    write_data(0x1b);
    write_data(0x39);
    write_data(0x44);
    write_data(0x47);
    write_data(0x28);
    write_data(0x12);
    write_data(0x10);
    write_data(0x16);
    write_data(0x1b);
    write_command(0xe1);
    write_data(0xf0);
    write_data(0x00);
    write_data(0x0a);
    write_data(0x10);
    write_data(0x11);
    write_data(0x1a);
    write_data(0x3b);
    write_data(0x34);
    write_data(0x4e);
    write_data(0x3a);
    write_data(0x17);
    write_data(0x16);
    write_data(0x21);
    write_data(0x22);
    write_command(0x29);
}

static void init_lcm_registers(void)
{
  //************* Start Initial Sequence **********//
    write_command(0xCF);
    write_data (0x00);
    write_data (0x81);
    write_data (0X30);

    write_command(0xED);
    write_data (0x64);
    write_data (0x03);
    write_data (0X12);
    write_data (0X81);

    write_command(0xE8);
    write_data (0x85);
    write_data (0x10);
    write_data (0x78);

    write_command(0xCB);
    write_data (0x39);
    write_data (0x2C);
    write_data (0x00);
    write_data (0x34);
    write_data (0x02);

    write_command(0xF7);
    write_data (0x20);

    write_command(0xEA);
    write_data (0x00);
    write_data (0x00);

    write_command(0xB1);
    write_data (0x00);
    write_data (0x18);//0x1b

    write_command(0xB6); // Display Function Control
    write_data (0x0A);
    write_data (0xA2);

    write_command(0xC0); //Power control
    write_data (0x21); //VRH[5:0]

    write_command(0xC1); //Power control
    write_data (0x11); //SAP[2:0];BT[3:0]

    write_command(0xC5); //VCM control
    write_data (0x40);//0x3f//1f
    write_data (0x2a);//0x3c//17

    write_command(0xC7); //VCM control2
    write_data (0Xb6);//0xb5//0xc7

    //********************************

    write_command(0x36); // Memory Access Control
    write_data (0x08);

    write_command(0xF2); // 3Gamma Function Disable
    write_data (0x00);

    write_command(0x26); //Gamma curve selected
    write_data (0x01);

    write_command(0xE0); //Set Gamma
    write_data (0x0F);
    write_data (0x26);
    write_data (0x24);
    write_data (0x0B);
    write_data (0x0E);
    write_data (0x09);
    write_data (0x54);
    write_data (0XA8);
    write_data (0x46);
    write_data (0x0C);
    write_data (0x17);
    write_data (0x09);
    write_data (0x0F);
    write_data (0x07);
    write_data (0x00);
    write_command(0XE1); //Set Gamma
    write_data (0x00);
    write_data (0x19);
    write_data (0x1B);
    write_data (0x04);
    write_data (0x10);
    write_data (0x07);
    write_data (0x2A);
    write_data (0x47);
    write_data (0x39);
    write_data (0x03);
    write_data (0x06);
    write_data (0x06);
    write_data (0x30);
    write_data (0x38);
    write_data (0x0F);

      write_command(0x3A);// Pixel format setting of RGB/MPU interface
    write_data (0x55);// 16 bits/pixel
    UDELAY(50);

      write_command(0x55);// Write Content Adaptive Brightness Control
    write_data (0x00);// Off
    UDELAY(50);

    write_command(0xb0); //RGB interface signal control , DE SYNC MODE , Polarity of HSYNC VSYNC DOTCLK DEN , display path via memory or not
    write_data (0xC0);//DEN high enable , dotclk rising edge, HSYNC & VSYNC low level clock .
    UDELAY(50);

    write_command(0xF6);//Endian setting , RGB/MPU interface select 
    write_data (0x00);//01
    write_data (0x00);
    write_data (0x06);

    write_command(0x11); //Exit Sleep
    UDELAY(50);
    write_command(0x29); //Display on
    write_command(0x2c); //Memory Write
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DPI;
	params->ctrl   = LCM_CTRL_GPIO;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

    /* RGB interface configurations */
    params->dpi.mipi_pll_clk_ref  = 0;
    params->dpi.mipi_pll_clk_div1 = 10;
    params->dpi.mipi_pll_clk_div2 = 15;
    params->dpi.dpi_clk_div       = 2;
    params->dpi.dpi_clk_duty      = 1;

    params->dpi.clk_pol           = LCM_POLARITY_FALLING;
    params->dpi.de_pol            = LCM_POLARITY_RISING;
    params->dpi.vsync_pol         = LCM_POLARITY_RISING;
    params->dpi.hsync_pol         = LCM_POLARITY_RISING;
/*
    params->dpi.hsync_pulse_width = 6;
    params->dpi.hsync_back_porch  = 8;
    params->dpi.hsync_front_porch = 5;
    params->dpi.vsync_pulse_width = 15;
    params->dpi.vsync_back_porch  = 4;
    params->dpi.vsync_front_porch = 5;
*/
    params->dpi.hsync_pulse_width = 6;
    params->dpi.hsync_back_porch  = 8;
    params->dpi.hsync_front_porch = 16;
    params->dpi.vsync_pulse_width = 4;
    params->dpi.vsync_back_porch  = 4;
    params->dpi.vsync_front_porch = 4;

    
    params->dpi.format            = LCM_DPI_FORMAT_RGB666;
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
	params->dpi.is_serial_output  = 0;
	params->dpi.i2x_en			  = 0;
	params->dpi.i2x_edge		  = 0;

    params->dpi.intermediat_buffer_num = 2;

    params->dpi.io_driving_current = LCM_DRIVING_CURRENT_2MA;
}


static void lcm_init(void)
{
    mt_set_gpio_dir(BL24086_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_mode(BL24086_RESET_PIN, GPIO_MODE_00);	//reset
	mt_set_gpio_pull_select(BL24086_RESET_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(BL24086_RESET_PIN, GPIO_PULL_ENABLE);

    mt_set_gpio_dir(BL24086_CS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_mode(BL24086_CS_PIN, GPIO_MODE_00);	//cs
	mt_set_gpio_pull_select(BL24086_CS_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(BL24086_CS_PIN, GPIO_PULL_ENABLE);

    mt_set_gpio_dir(BL24086_SCL_PIN, GPIO_DIR_OUT);	
    mt_set_gpio_mode(BL24086_SCL_PIN, GPIO_MODE_00);	//sclk
	mt_set_gpio_pull_select(BL24086_SCL_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(BL24086_SCL_PIN, GPIO_PULL_ENABLE);

    mt_set_gpio_dir(BL24086_SDA_PIN, GPIO_DIR_OUT);
    mt_set_gpio_mode(BL24086_SDA_PIN, GPIO_MODE_00);	//sda
	mt_set_gpio_pull_select(BL24086_SDA_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(BL24086_SDA_PIN, GPIO_PULL_ENABLE);

    /*
    mt_set_gpio_dir(_BACKLIGHT_PIN, GPIO_DIR_OUT);
    mt_set_gpio_mode(_BACKLIGHT_PIN, GPIO_MODE_00);	//_BACKLIGHT_PIN
	mt_set_gpio_pull_select(_BACKLIGHT_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(_BACKLIGHT_PIN, GPIO_PULL_ENABLE);
    */

	HXLCM_CS_HIGH();HXLCM_SDA_LOW();HXLCM_SCL_LOW();

	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(1);
    //lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ZERO);
    MDELAY(10);
    //lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(120);

#ifdef _P1213_
    //init_lcm_registers();
    ST7789SCMI24panelinitialcode();
#endif
//	clear_panel();
}


static void lcm_suspend(void)
{

	//BACKLIGHT_LOW();

#if 1
	//write_command(0x28);
	//UDELAY(10);
	write_command(0x10);
	MDELAY(5);
#endif

    //mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ZERO);
	//UDELAY(20);

}


static void lcm_resume(void)
{

#if 1
	write_command(0x11);
	MDELAY(5);
	//write_command(0x11);
	//UDELAY(80);
	//write_command(0x29);
#endif
#if 1
    HXLCM_CS_HIGH();HXLCM_SDA_LOW();HXLCM_SCL_LOW();

	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(1);
    //lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ZERO);
    MDELAY(10);
    //lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(BL24086_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(120);

#ifdef _P1213_
    //init_lcm_registers();
#endif
#endif

    //init_lcm_registers();
    ST7789SCMI24panelinitialcode();
    //BACKLIGHT_HIGH();

}




static void lcm_setbacklight(unsigned int level)
{
 
}



LCM_DRIVER st7789s_lcm_drv = 
{
    .name			= "st7789s",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};

