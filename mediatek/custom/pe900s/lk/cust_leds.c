//#include <platform/cust_leds.h>
#include <cust_leds.h>

#include <mt_pwm.h>
//#include <kernel.h>
//#include <pmic_mt6329_hw_bank1.h>
//#include <pmic_mt6329_sw_bank1.h>
//#include <pmic_mt6329_hw.h>
//#include <pmic_mt6329_sw.h>
//#include <upmu_common_sw.h>
//#include <upmu_hw.h>

//#include <asm/arch/mt6577_pwm.h>

//extern int DISP_SetBacklight(int level);

extern int disp_bls_set_backlight(unsigned int level);
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
    {"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
    //{"red",               MT65XX_LED_MODE_NONE, -1, {0}},
	{"green",             MT65XX_LED_MODE_GPIO, 138, {0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1, {0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1, {0}},
	{"keyboard-backlight",MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0, {0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0, {0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_LCD_ISINK, {0}}, 
    {"lcd-backlight",     MT65XX_LED_MODE_PWM, PWM2, {PWM_CLK_NEW_MODE_BLOCK, CLK_DIV128, 64, 16}},
    //{"lcd-backlight",     MT65XX_LED_MODE_PWM, PWM2, {0,0,0,0,0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

