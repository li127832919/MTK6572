/*
TP driver HX8527
*/

#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/module.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>


#include "tpd_custom_HX8527_D40.h"


#include <mach/mt_pm_ldo.h> 
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>

//dma
#include <linux/dma-mapping.h>

#include <cust_eint.h>

#include <linux/bitops.h>
#include <linux/hwmsen_helper.h>
#include <linux/byteorder/generic.h>

//========DEBUG and log ================================

#define MTK_TPD_DEBUG
#ifdef MTK_TPD_DEBUG
	#define MTK_TP_DEBUG(fmt, args ...) printk("mtk-tpd: %5d: " fmt, __LINE__,##args)
#else
	#define MTK_TP_DEBUG(fmt, args ...)
#endif


#define HX_85XX_D_SERIES_PWON

//#define TPD_PROXIMITY_DMESG  printk
#define TPD_PROXIMITY_DMESG(a,arg...) printk("TPD_himax8526" ": " a,##arg)
#define TPD_PROXIMITY_DEBUG(a,arg...) printk("TPD_himax8526" ": " a,##arg)



#define LCT_MTK_CTP_INFO_SUPPORT //
#ifdef LCT_MTK_CTP_INFO_SUPPORT
#include <linux/proc_fs.h>
#define CTP_PROC_FILE "ctp_version"
static struct proc_dir_entry *g_ctp_proc = NULL;
#endif

//===========Define and custom=================
////////////////Himax: Function Define/////////////////////


//#define HIMAX_BUTTON 0
#define HIMAX_DEBUG 1              //Himax TP debug option
#define HIMAX_UPGRADE 1            //Himax TP debug option
#define HIMAX_RAWDATA 1            //Himax TP debug option

#define HIMAX_UPGRADE_WITH_IFILE 0 //Himax CTP  i file update FW 1:open ,default 0
#define HIMAX_CHECKSUM 1
#define ESD_WORKAROUND 0
#define ChangeIref1u 0
#define I2C_MASTER_CLOCK 100

#define _DMA_RW_MODE_

//Himax: Set Point Number and Resolution
#define HIMAX_5POINT_SUPPORT 1

//define ctp device use I2C number 
#define I2C_NUM 0


//#define TPD_PROXIMITY

/******************** Himax: Firmware bin Checksum ***********************/
//Select one,one time
//#define HX_TP_BIN_CHECKSUM_SW
//#define HX_TP_BIN_CHECKSUM_HW
#define HX_TP_BIN_CHECKSUM_CRC 
/******************** Himax: Firmware bin Checksum ***********************/

//Himax: ESD
#if ESD_WORKAROUND
u8 first_pressed = 0;
static u8 reset_activate = 0;
u8 ESD_COUNTER = 0;
#endif

/*----------------Himax customize setting ------------------------------------*/
#if HIMAX_5POINT_SUPPORT
const u8 PT_NUM_MAX = 5;
#else
const u8 PT_NUM_MAX = 2;
#endif

//Himax: Set TP  Resolution
const u16 RESOLUTION_Y = 1326;
const u16 RESOLUTION_X = 720;

//LCM Resolution
#define TMP_LCM_WIDTH  simple_strtoul(LCM_WIDTH, NULL, 0)
#define TMP_LCM_HEIGHT  simple_strtoul(LCM_HEIGHT, NULL, 0)

#define TPD_DELAY                (2*HZ/100)
#define TPD_RES_X                480
#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX  {962,0,0,0,1600,0,0,0};


#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH 100
#define TPD_KEY_COUNT           3
#define TPD_KEYS                { KEY_MENU, KEY_HOME,KEY_BACK}
#define TPD_KEYS_DIM            {{80,850,160,TPD_BUTTON_HEIGH},{240,850,160,TPD_BUTTON_HEIGH},{400,850,160,TPD_BUTTON_HEIGH}}
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

/*----------------Himax customize setting end------------------------------------*/

#ifdef _DMA_RW_MODE_
static uint8_t *g_pDMABuf_va = NULL;
static uint32_t *g_pDMABuf_pa = NULL;
#endif


extern struct tpd_device *tpd;
 
struct i2c_client *hx_i2c_client = NULL;
struct task_struct *hx_thread = NULL;
struct task_struct *hx_update_thread = NULL;

static int probe_fw_flage=0;


#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif


#ifdef TPD_PROXIMITY
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);
 
static void tpd_eint_interrupt_handler(void);
 
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eintno, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eintno, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, 
	   void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);

/*
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
*/

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
void himax_HW_reset(void);

#ifdef _DMA_RW_MODE_
static void dma_buffer_alloct()
{	
g_pDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &g_pDMABuf_pa, GFP_KERNEL);    
if(!g_pDMABuf_va)	
	{        
	TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");    
	}
}
static void dma_buffer_release()
{	
	if(g_pDMABuf_va)	
	{     	
	dma_free_coherent(NULL, 4096, g_pDMABuf_va, g_pDMABuf_pa);        
	g_pDMABuf_va = NULL;        
	g_pDMABuf_pa = NULL;		
	TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");    
	}
	}
#endif



//Himax: Set Flash Pre-Patch
static char PrePatch = 0x06;

//Himax: Define Touch Number
static int tpd_flag = 0;
static int tpd_halt=0;
static int point_num = 0;
static int P_ID = 0;
static int p_soft_key = 0xFF;


#define TPD_OK 0

//Himax: Set FW and CFG Flash Address
#define FW_VER_MAJ_FLASH_ADDR	33	//0x0085 0x0086
#define FW_VER_MAJ_FLASH_LENG	1
#define FW_VER_MIN_FLASH_ADDR	33  //0x0086
#define FW_VER_MIN_FLASH_LENG	1
#define CFG_VER_MAJ_FLASH_ADDR	39//0x009F              173	//0x02B4
#define CFG_VER_MAJ_FLASH_LENG	4 //20130312
#define CFG_VER_MIN_FLASH_ADDR	43//0x00AC             172	//0x02C0 20130312
#define CFG_VER_MIN_FLASH_LENG	3

#ifdef LCT_MTK_CTP_INFO_SUPPORT
//static unsigned char FW_VER_MAJ_FLASH_buff[FW_VER_MAJ_FLASH_LENG * 4];
//static unsigned char FW_VER_MIN_FLASH_buff[FW_VER_MIN_FLASH_LENG * 4];
static unsigned char CFG_VER_MAJ_FLASH_buff[CFG_VER_MAJ_FLASH_LENG * 4];
static unsigned char CFG_VER_MIN_FLASH_buff[CFG_VER_MIN_FLASH_LENG * 4];
#endif

//define kick dog start ***********************************************
/*Android4_1*/
enum wk_wdt_type {	
				   WK_WDT_LOC_TYPE,
				   WK_WDT_EXT_TYPE,
				   WK_WDT_LOC_TYPE_NOLOCK,	
				   WK_WDT_EXT_TYPE_NOLOCK,
                  };


extern void mpcore_wdt_restart(enum wk_wdt_type type);
extern void mtk_wdt_restart(enum wk_wdt_type type);



//kick dog end*********************************************************


#define VELOCITY_CUSTOM_HX8526   

#ifdef VELOCITY_CUSTOM_HX8526
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	void __user *data;
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk(KERN_ERR "tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			printk(KERN_ERR "tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = 
{
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif /* VELOCITY_CUSTOM_HX8526 */

struct touch_info 
{
	#if HIMAX_5POINT_SUPPORT
		int y[5];	// Y coordinate of touch point
		int x[5];	// X coordinate of touch point
		int p[5];	// event flag of touch point
		int id[5];	// touch id of touch point
	#else
		int y[4];	
		int x[4];
		int p[4];
		int id[4];
	#endif
		int count;	// touch counter
};


#if GTP_HAVE_TOUCH_KEY
     //#define GTP_KEY_TAB	 {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEND}
      #define GTP_KEY_TAB { KEY_MENU,  KEY_BACK };
	 //const u16 touch_key_array[] = { KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH };
      const u8 touch_key_array[] = { KEY_MENU,  KEY_BACK };
      #define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )


#endif



static const struct i2c_device_id tpd_i2c_id[] = {{"HX8527",0},{}};
static struct i2c_board_info __initdata HIMAX_TS_i2c_tpd={ I2C_BOARD_INFO("HX8527", (0x90>>1))}; 	



#if ((HIMAX_RAWDATA) | (HIMAX_UPGRADE))
static struct kobject *android_touch_kobj = NULL;
static uint8_t debug_log_level= 0;

static int himax_touch_sysfs_init(void);
static void himax_touch_sysfs_deinit(void);

static uint8_t getDebugLevel(void)
{
    return debug_log_level;
}
#endif

#if HIMAX_RAWDATA
#define DEFAULT_X_CHANNEL            10   /* face the TS, x-axis */
#define DEFAULT_Y_CHANNEL            14   /* face the TS, y-axis */
#define DEFAULT_SELF_CHANNEL         24

static uint8_t himax_command = 0;
static uint8_t x_channel = DEFAULT_X_CHANNEL; /* x asix, when you face the top of TS */
static uint8_t y_channel = DEFAULT_Y_CHANNEL; /* y asix, when you face the top of TS  */
static uint8_t *diag_mutual = NULL;
static uint8_t diag_command = 0;
static uint8_t diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL] = {0};

static uint8_t *getMutualBuffer(void)
{
    return diag_mutual;
}

static void setMutualBuffer(void)
{
    diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

static uint8_t *getSelfBuffer(void)
{
    return &diag_self[0];
}

static uint8_t getDiagCommand(void)
{
    return diag_command;
}

static uint8_t getXChannel(void)
{
    return x_channel;
}

static uint8_t getYChannel(void)
{
    return y_channel;
}

static void setXChannel(uint8_t x)
{
    x_channel = x;
}

static void setYChannel(uint8_t y)
{
    y_channel = y;
}

/* kernel sysfs interface for reading/writing HIMAX register == START */
static ssize_t himax_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data[96] = { 0 }, loop_i;

	printk(KERN_INFO "[TP]%x\n", himax_command);

	if (i2c_smbus_read_i2c_block_data(hx_i2c_client, himax_command, 96, &data[0]) < 0) 
	{
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}

	ret += sprintf(buf, "command: %x\n", himax_command);
	for (loop_i = 0; loop_i < 96; loop_i++)
	{
		ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += sprintf(buf + ret, "\n");
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') 
	{
		if (buf[2] == 'x') 
		{
			uint8_t loop_i;
			uint16_t base = 5;
			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
				himax_command = result;
			for (loop_i = 0; loop_i < 100; loop_i++) 
			{
				if (buf[base] == '\n') 
				{
					if (buf[0] == 'w')
                        i2c_smbus_write_i2c_block_data(hx_i2c_client, himax_command, length, &write_da[0]);
					printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
						write_da[0], length);
					for (veriLen = 0; veriLen < length; veriLen++)
						printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));

					printk(KERN_INFO "\n");
					return count;
				}
				if (buf[base + 1] == 'x') 
				{
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
						write_da[loop_i] = result;
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO|S_IWUGO),
	himax_register_show, himax_register_store);
/* kernel sysfs interface for reading/writing HIMAX register == END */

#if 0
/* kernel sysfs interface for showing firmware version info == START */
static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s_%s\n", FW_VER_MAJ_FLASH_buff, FW_VER_MIN_FLASH_buff);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);
/* kernel sysfs interface for showing firmware version info == END */
#endif

/* kernel sysfs interface for executing HIMAX special command == START*/
static ssize_t himax_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;
	width = x_channel;
	count += sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);

	if (diag_command >= 1 && diag_command <= 6) 
	{
		if (diag_command < 3) 
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) 
				{
					count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) 
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		} else if (diag_command > 4) 
		{
			for (loop_i = 0; loop_i < self_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		} else 
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += sprintf(buf + count, "\n");
			}
		}
	}

	return count;
}

static ssize_t himax_diag_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{	
		const uint8_t command_ec_128_raw_flag = 0x01;
		const uint8_t command_ec_24_normal_flag = 0x00;
	#ifndef HX_85XX_D_SERIES_PWON     
		const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
	#else
		const uint8_t command_ec_128_raw_baseline_flag = 0x02;
		const uint8_t command_ec_128_raw_bank_flag = 0x03;
	#endif    
		uint8_t command_91h[2] = {0x91, 0x00};
		uint8_t command_82h[1] = {0x82};
		uint8_t command_F3h[2] = {0xF3, 0x00};
		uint8_t command_83h[1] = {0x83};
		uint8_t receive[1];

    if (buf[0] == '1')
    {
        command_91h[1] = command_ec_128_raw_baseline_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else if (buf[0] == '2')
    {
        command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else if (buf[0] == '3')
	{
#ifndef HX_85XX_D_SERIES_PWON 	    
    	i2c_smbus_write_i2c_block_data(hx_i2c_client, command_82h[0], 1, &command_82h[0]);
    	msleep(50);
    	
    	i2c_smbus_read_i2c_block_data(hx_i2c_client, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] | 0x80);
    	i2c_smbus_write_i2c_block_data(hx_i2c_client, command_F3h[0], 2, &command_F3h[1]);    	

    	command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
        
    	i2c_smbus_write_i2c_block_data(hx_i2c_client, command_83h[0], 2, &command_83h[0]);
    	msleep(50);
#else
        command_91h[1] = command_ec_128_raw_bank_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
#endif    	        
        diag_command = buf[0] - '0';
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }
    else
    {
#ifndef HX_85XX_D_SERIES_PWON         
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_82h[0], 1, &command_82h[0]);
    	msleep(50);
        
        command_91h[1] = command_ec_24_normal_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
        
    	i2c_smbus_read_i2c_block_data(hx_i2c_client, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] & 0x7F);
    	i2c_smbus_write_i2c_block_data(hx_i2c_client, command_F3h[0], 2, &command_F3h[1]);
    	i2c_smbus_write_i2c_block_data(hx_i2c_client, command_83h[0], 2, &command_83h[0]);
#else
        command_91h[1] = command_ec_24_normal_flag;
        i2c_smbus_write_i2c_block_data(hx_i2c_client, command_91h[0], 1, &command_91h[1]);
#endif    	
        diag_command = 0;
        printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
    }	
	
	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO|S_IWUGO),
	himax_diag_show, himax_diag_dump);
/* kernel sysfs interface for executing HIMAX special command == END*/
#endif /* HIMAX_RAWDATA */


#if HIMAX_UPGRADE
#if HIMAX_UPGRADE_WITH_IFILE
static unsigned char HIMAX_FW[]=   
{
	#include "hx8527_for_MTK_test.i"
};
#endif
								 
static unsigned char upgrade_fw[32*1024];

void himax_ManualMode(int enter)
{
	unsigned char cmd[2];
	cmd[0] = enter;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &cmd[0]);
}

void himax_FlashMode(int enter)
{
	unsigned char cmd[2];
	cmd[0] = enter;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 1, &cmd[0]);
}

void himax_lock_flash(void)
{
	unsigned char cmd[5];
	
	/* lock sequence start */
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]);

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]);

	cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x7D;cmd[3] = 0x03;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x45, 4, &cmd[0]);

	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x4A, 0, &cmd[0]);
	mdelay(50);
	/* lock sequence stop */
}

void himax_unlock_flash(void)
{
	unsigned char cmd[5];
	
	/* unlock sequence start */
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]);

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]);

	cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x3D;cmd[3] = 0x03;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x45, 4, &cmd[0]);

	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x4A, 0, &cmd[0]);
	mdelay(50);
	/* unlock sequence stop */
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)//, int address, int RST)
{
#ifdef HX_TP_BIN_CHECKSUM_SW
	u16 checksum = 0;
	unsigned char cmd[5], last_byte;
	int FileLength, i, readLen, k, lastLength;

	FileLength = fullLength - 2;
	memset(cmd, 0x00, sizeof(cmd));

	//if(himax_modifyIref() == 0)
		//return 0;
	
	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0; i < FileLength; i++) 
	{
	    
		last_byte = 0;
		readLen = 0;

        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1)
            last_byte = 1;
        cmd[1] = (i >> 5) & 0x1F;cmd[2] = (i >> 10) & 0x1F;
        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x46, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x59, 4, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }

		if (i < (FileLength - 1))
		{
			checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
			if (i == 0)
				printk(KERN_ERR "Himax_marked cmd 0 to 3 (first): %d, %d, %d, %d\n", cmd[0], cmd[1], cmd[2], cmd[3]);
		}
		else 
		{
			printk(KERN_ERR "Himax_marked cmd 0 to 3 (last): %d, %d, %d, %d\n", cmd[0], cmd[1], cmd[2], cmd[3]);
			printk(KERN_ERR "Himax_marked, checksum (not last): %d\n", checksum);
			lastLength = (((fullLength - 2) % 4) > 0)?((fullLength - 2) % 4):4;
			
			for (k = 0; k < lastLength; k++) 
				checksum += cmd[k];
			printk(KERN_ERR "Himax_marked, checksum (final): %d\n", checksum);
			
			//Check Success
			if (ImageBuffer[fullLength - 1] == (u8)(0xFF & (checksum >> 8)) && ImageBuffer[fullLength - 2] == (u8)(0xFF & checksum)) 
			{
				himax_FlashMode(0);
				return 1;
			} 
			else //Check Fail
			{
				himax_FlashMode(0);
				return 0;
			}
		}
	}
#endif

#ifdef  HX_TP_BIN_CHECKSUM_HW
    u32 checksum = 0;
    unsigned char cmd[5], last_byte;
    int FileLength, i, readLen, k, lastLength;

    FileLength = fullLength;
    memset(cmd, 0x00, sizeof(cmd));

    //himax_HW_reset(RST);

    //if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0)
        //return 0;
         
    //mdelay(120);
    //printk("himax_marked, Sleep out: %d\n", __LINE__);
    //himax_unlock_flash();
    
    himax_FlashMode(1);

    FileLength = (FileLength + 3) / 4;
    for (i = 0; i < FileLength; i++) 
    {
        last_byte = 0;
        readLen = 0;

        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1)
            last_byte = 1;
        cmd[1] = (i >> 5) & 0x1F;cmd[2] = (i >> 10) & 0x1F;
        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x46, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x59, 4, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }

        if (i < (FileLength - 1))
        {
            checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
            if (i == 0)
                printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
        }
        else 
        {
            printk(KERN_ERR "[TP] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (not last): %d\n", __func__, checksum);

            lastLength = ((fullLength % 4) > 0)?(fullLength % 4):4;
            
            for (k = 0; k < lastLength; k++) 
                checksum += cmd[k];
            printk(KERN_ERR "[TP] %s: himax_marked, checksum (final): %d\n", __func__, checksum);
           
            //Enable HW Checksum function.
            cmd[0] = 0x01;
            if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xE5, 1, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            //Must sleep 5 ms.
            msleep(30);

            //Get HW Checksum. 
	    if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0xAD, 4, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return -1;
            }

            //Compare the checksum.
	    if( cmd[0] == (u8)(0xFFFF & (checksum >> 24)) &&
                cmd[1] == (u8)(0xFFFF & (checksum >> 16)) &&
                cmd[2] == (u8)(0xFFFF & (checksum >> 8)) &&
                cmd[3] == (u8)(0xFFFF & checksum ))
	    {
                himax_FlashMode(0);
                return 1;
	    }            
	    else
            {
                himax_FlashMode(0);
                return 0;
            }
        }
    }
#endif

#ifdef HX_TP_BIN_CHECKSUM_CRC
    unsigned char cmd[5];
    
	//Set Flash Clock Rate
    if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x7F, 5, &cmd[0]))< 0){
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return -1;
    }
    cmd[3] = 0x02;
    
    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x7F, 5, &cmd[0]))< 0){
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    //Enable Flash
    himax_FlashMode(1);

    //Select CRC Mode
    cmd[0] = 0x05;
    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xD2, 1, &cmd[0]))< 0){
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    } 

    //Enable CRC Function
    cmd[0] = 0x01;
    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xE5, 1, &cmd[0]))< 0){
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return 0;
    }

    //Must delay 30 ms
    msleep(30);

    //Read HW CRC
    if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0xAD, 4, &cmd[0]))< 0){
        printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
        return -1;
    }

    if( cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ){
        himax_FlashMode(0);
        return 1;
    }
    else {
        himax_FlashMode(0);
        return 0;
    }
#endif

    return 0;
}

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
    unsigned char* ImageBuffer = fw;//CTPM_FW;
    int fullFileLength = len;//sizeof(CTPM_FW); //Paul Check
    int i, j;
    unsigned char cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;

    //Try 3 Times
    for (j = 0; j < 3; j++) 
    {
        FileLength = fullFileLength - 2;

        himax_HW_reset();

        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }
       
        mdelay(120);

        himax_unlock_flash();  //ok

        cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)     //ok
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x4F, 0, &cmd[0]))< 0)     //ok
        {
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return 0;
        }     
        mdelay(50);

        himax_ManualMode(1); //ok
        himax_FlashMode(1);  //ok

        FileLength = (FileLength + 3) / 4;
        for (i = 0, prePage = 0; i < FileLength; i++) 
        {
            last_byte = 0;
            cmd[0] = i & 0x1F;
            if (cmd[0] == 0x1F || i == FileLength - 1)
                last_byte = 1;
            cmd[1] = (i >> 5) & 0x1F;
            cmd[2] = (i >> 10) & 0x1F;
            if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (prePage != cmd[1] || i == 0) 
            {
                prePage = cmd[1];
                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
            }

            memcpy(&cmd[0], &ImageBuffer[4*i], 4);//Paul
            if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x45, 4, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }
                   
            cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (last_byte == 1) 
            {
                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x05;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                mdelay(10);
                if (i == (FileLength - 1)) 
                {
                    himax_FlashMode(0);
                    himax_ManualMode(0);
                    checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);//, address, RST);
                    //himax_ManualMode(0);
                    himax_lock_flash();
                    
                    if (checksumResult) //Success
                    {
                        himax_HW_reset();
                        return 1;
                    } 
                    else if (/*j == 4 && */!checksumResult) //Fail
                    {
                        himax_HW_reset();
                    } 
                    else //Retry
                    {
                        himax_FlashMode(0);
                        himax_ManualMode(0);
                    }
                }
            }
        }
    }    
    return 0;
}


/* kernel sysfs interface for HIMAX firmware upgrade == START*/
static ssize_t himax_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", debug_log_level);

	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char data[3];
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		debug_log_level = buf[0] - '0';

    struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    char fileName[128];
    
    if(buf[0] == 't')
    {
        memset(fileName, 0, 128);
        snprintf(fileName, count-2, "%s", &buf[2]);
        printk(KERN_INFO "[TP] %s: upgrade from file(%s) start!\n", __func__, fileName);
        filp = filp_open(fileName, O_RDONLY, 0);
        if(IS_ERR(filp)) {
            printk(KERN_ERR "[TP] %s: open firmware file failed\n", __func__);
            return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0) {
            printk(KERN_ERR "[TP] %s: read firmware file failed\n", __func__);
            return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk(KERN_INFO "[TP] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

        if(result > 0)
        {
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
                printk(KERN_INFO "[TP] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
            else
                printk(KERN_INFO "[TP] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
                
      /*      //Himax: Power On Flow
						data[0] =0x02;
						if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &data[0]))< 0)
							goto HimaxErr;
						msleep(1);
					
						data[0] =0x02;
					  if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x35, 1, &data[0]))< 0)
							goto HimaxErr;
						msleep(1);
							
						data[0] =0x0F;
						data[1] =0x53;
					  if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x36, 2, &data[0]))< 0)
							goto HimaxErr;
					  msleep(1);
					  
					  //Himax: Check Flash Pre-Patch
						data[0] =PrePatch; //Himax: Modify by case
					  data[1] =0x02;
						if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xDD, 2, &data[0]))< 0)
							goto HimaxErr;
						msleep(1);
						
						data[0] =0x00;
						if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xE9, 1, &data[0]))< 0)
							goto HimaxErr;
						msleep(1);
					 
						if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x83, 0, &data[0]))< 0)
							goto HimaxErr;
						msleep(100);//msleep(30);  2012-09-21
						
						if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &data[0]))< 0)
							goto HimaxErr;
						msleep(100);
						//Himax: Power On Flow End    */
						
#ifdef HX_85XX_D_SERIES_PWON
    //Himax Touch Controller power-on sequence
	data[0] =0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &data[0]))< 0)
	{
		return -1;
	}
	msleep(1);
	
	data[0] =0x0f;
	data[1] =0x53;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x36, 2, &data[0]))< 0)
	{
		return -1;
	}
	msleep(1);	
	data[0] =0x04;
	data[1] =0x03;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xdd, 2, &data[0]))< 0)
	{
		return -1;
	}
	msleep(1);
	
	data[0] =0x01;
	data[1] =0x36;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xb9, 2, &data[0]))< 0)
	{
		return -1;
	}
	msleep(1);
	
	data[0] =0x01;
	data[1] =0xF5;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xcb, 2, &data[0]))< 0)
	{
		return -1;
	}
	msleep(1);
	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x83, 0, &data[0]))< 0)
	{
		return -1;
	}
	msleep(120);
	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &data[0]))< 0)
	{
		return -1;
	}
	msleep(120);
	
	

#endif //end HX_85XX_D_SERIES_PWON 
						
            
            return count;                    
        }
    }

HimaxErr:
	printk(KERN_ERR "Himax TP: I2C transfer error, line: %d\n", __LINE__);

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO|S_IWUGO),
	himax_debug_level_show, himax_debug_level_dump);
/* kernel sysfs interface for HIMAX firmware upgrade == END*/
#endif /* HIMAX_UPGRADE */


#if ChangeIref1u

#define TarIref 1

		unsigned char SFR_1u_1[16][2] = {{0x18,0x07},{0x18,0x17},{0x18,0x27},{0x18,0x37},{0x18,0x47},
			{0x18,0x57},{0x18,0x67},{0x18,0x77},{0x18,0x87},{0x18,0x97},
			{0x18,0xA7},{0x18,0xB7},{0x18,0xC7},{0x18,0xD7},{0x18,0xE7},
			{0x18,0xF7}};

		unsigned char SFR_2u_1[16][2] = {{0x98,0x06},{0x98,0x16},{0x98,0x26},{0x98,0x36},{0x98,0x46},
			{0x98,0x56},{0x98,0x66},{0x98,0x76},{0x98,0x86},{0x98,0x96},
			{0x98,0xA6},{0x98,0xB6},{0x98,0xC6},{0x98,0xD6},{0x98,0xE6},
			{0x98,0xF6}};

		unsigned char SFR_3u_1[16][2] = {{0x18,0x06},{0x18,0x16},{0x18,0x26},{0x18,0x36},{0x18,0x46},
			{0x18,0x56},{0x18,0x66},{0x18,0x76},{0x18,0x86},{0x18,0x96},
			{0x18,0xA6},{0x18,0xB6},{0x18,0xC6},{0x18,0xD6},{0x18,0xE6},
			{0x18,0xF6}};

		unsigned char SFR_4u_1[16][2] = {{0x98,0x05},{0x98,0x15},{0x98,0x25},{0x98,0x35},{0x98,0x45},
			{0x98,0x55},{0x98,0x65},{0x98,0x75},{0x98,0x85},{0x98,0x95},
			{0x98,0xA5},{0x98,0xB5},{0x98,0xC5},{0x98,0xD5},{0x98,0xE5},
			{0x98,0xF5}};

		unsigned char SFR_5u_1[16][2] = {{0x18,0x05},{0x18,0x15},{0x18,0x25},{0x18,0x35},{0x18,0x45},
			{0x18,0x55},{0x18,0x65},{0x18,0x75},{0x18,0x85},{0x18,0x95},
			{0x18,0xA5},{0x18,0xB5},{0x18,0xC5},{0x18,0xD5},{0x18,0xE5},
			{0x18,0xF5}};

		unsigned char SFR_6u_1[16][2] = {{0x98,0x04},{0x98,0x14},{0x98,0x24},{0x98,0x34},{0x98,0x44},
			{0x98,0x54},{0x98,0x64},{0x98,0x74},{0x98,0x84},{0x98,0x94},
			{0x98,0xA4},{0x98,0xB4},{0x98,0xC4},{0x98,0xD4},{0x98,0xE4},
			{0x98,0xF4}};

		unsigned char SFR_7u_1[16][2] = {{0x18,0x04},{0x18,0x14},{0x18,0x24},{0x18,0x34},{0x18,0x44},
			{0x18,0x54},{0x18,0x64},{0x18,0x74},{0x18,0x84},{0x18,0x94},
			{0x18,0xA4},{0x18,0xB4},{0x18,0xC4},{0x18,0xD4},{0x18,0xE4},
			{0x18,0xF4}};

int ChangeIrefSPP(void)
{
	//struct i2c_client * hx_i2c_client = ts->client ;

#if 1
	unsigned char i;
	//int readLen;
	unsigned char cmd[5];
	//unsigned char Iref[2] = {0x00,0x00};

	unsigned char spp_source[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,		//SPP
						  	        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};		//SPP

	unsigned char spp_target[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,		//SPP
 						  	        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};		//SPP
	unsigned char retry;
	unsigned char spp_ok;

	printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong\n");

	//--------------------------------------------------------------------------
	//Inital
	//--------------------------------------------------------------------------
	cmd[0] = 0x42;
	cmd[1] = 0x02;
	if(i2c_master_send(hx_i2c_client, cmd, 2) < 0)
	{
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong1\n");
		return 0;
	}
	udelay(10);

	cmd[0] = 0x81;
	if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
	{
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong2\n");
		return 0;
	}
	mdelay(160);

	//--------------------------------------------------------------------------
	//read 16-byte SPP to spp_source
	//--------------------------------------------------------------------------
	cmd[0] = 0x43;
	cmd[1] = 0x01;
	cmd[2] = 0x00;
	cmd[3] = 0x1A;
	if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
	{
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong3\n");
		return 0;
	}
	udelay(10);

	//4 words
	for (i = 0; i < 4; i++)
	{
		cmd[0] = 0x44;
		cmd[1] = i;			//word
		cmd[2] = 0x00;		//page
		cmd[3] = 0x00;		//sector
		if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
		{
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong4\n");
			return 0;
		}
		udelay(10);

		cmd[0] = 0x46;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
		{
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong5\n");
			return 0;
		}
		udelay(10);

		cmd[0] = 0x59;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
		{
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong6\n");
			return 0;
		}
		mdelay(5);				//check this

		if(i2c_master_recv(hx_i2c_client, cmd, 4) < 0)
			return 0;
		udelay(10);

		//save data
		spp_source[4*i + 0] = cmd[0];
		spp_source[4*i + 1] = cmd[1];
		spp_source[4*i + 2] = cmd[2];
		spp_source[4*i + 3] = cmd[3];

		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7 cmd0 = 0x%x\n",cmd[0]);
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7 cmd1 = 0x%x\n",cmd[1]);
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7 cmd2 = 0x%x\n",cmd[2]);
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7 cmd3 = 0x%x\n",cmd[3]);
		//printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7 cmd4 = 0x%x\n",cmd[4]);
		printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong7\n");
	}

	//--------------------------------------------------------------------------
	//Search 3u Iref
	//--------------------------------------------------------------------------
	for (i = 0; i < 16; i++)
	{
		if(spp_source[0]==SFR_1u_1[i][0] && spp_source[1]==SFR_1u_1[i][1])
		{
			//found in 1uA
			return (1);				//OK
		}
	}

	spp_ok = 0;
	for (i = 0; i < 16; i++)
	{
		if(spp_source[0]==SFR_3u_1[i][0] && spp_source[1]==SFR_3u_1[i][1])
		{
			//found in 3uA
			spp_ok = 1;

			spp_source[0]= SFR_1u_1[i][0];
			spp_source[1]= SFR_1u_1[i][1];
			break;
		}
	}

	if (spp_ok == 0)
	{
		//no matched pair in SFR_1u_1 or SFR_3u_1
		return 0;
	}

	//--------------------------------------------------------------------------
	//write SPP (retry for 3 times if errors occur)
	//--------------------------------------------------------------------------
	for (retry = 0; retry < 3; retry++)
	{
		himax_unlock_flash();

		//write 16-byte SPP
		cmd[0] = 0x43;
		cmd[1] = 0x01;
		cmd[2] = 0x00;
		cmd[3] = 0x1A;
		if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
			return 0;
		udelay(10);

		cmd[0] = 0x4A;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
			return 0;
		udelay(10);

		for (i = 0; i < 4; i++)
		{
			cmd[0] = 0x44;
			cmd[1] = i;			//word
			cmd[2] = 0x00;		//page
			cmd[3] = 0x00;		//sector
			if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
				return 0;
			udelay(10);

			cmd[0] = 0x45;
			cmd[1] = spp_source[4 * i + 0];
			cmd[2] = spp_source[4 * i + 1];
			cmd[3] = spp_source[4 * i + 2];
			cmd[4] = spp_source[4 * i + 3];
			if(i2c_master_send(hx_i2c_client, cmd, 5) < 0)
				return 0;
			udelay(10);

			cmd[0] = 0x4B;		//write SPP
			if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
				return 0;
			udelay(10);
		}

		cmd[0] = 0x4C;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
			return 0;
		mdelay(10);

		//read 16-byte SPP to spp_target
		cmd[0] = 0x43;
		cmd[1] = 0x01;
		cmd[2] = 0x00;
		cmd[3] = 0x1A;
		if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
		{
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong3\n");
			return 0;
		}
		udelay(10);

		for (i = 0; i < 4; i++)
		{
			cmd[0] = 0x44;
			cmd[1] = i;			//word
			cmd[2] = 0x00;		//page
			cmd[3] = 0x00;		//sector
			if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
			{
				printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong4\n");
				return 0;
			}
			udelay(10);

			cmd[0] = 0x46;
			if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
			{
				printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong5\n");
				return 0;
			}
			udelay(10);

			cmd[0] = 0x59;
			if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
			{
				printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong6\n");
				return 0;
			}
			mdelay(5);		//check this

			if(i2c_master_recv(hx_i2c_client, cmd, 4) < 0)
				return 0;
			udelay(10);

			spp_target[4*i + 0] = cmd[0];
			spp_target[4*i + 1] = cmd[1];
			spp_target[4*i + 2] = cmd[2];
			spp_target[4*i + 3] = cmd[3];

			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9 cmd0 = 0x%x\n",cmd[0]);
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9 cmd1 = 0x%x\n",cmd[1]);
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9 cmd2 = 0x%x\n",cmd[2]);
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9 cmd3 = 0x%x\n",cmd[3]);
			//printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9 cmd4 = 0x%x\n",cmd[4]);
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong9\n");
		}

		//compare source and target
		spp_ok = 1;
		for (i = 0; i < 16; i++)
		{
			if (spp_target[i] != spp_source[i])
			{
				spp_ok = 0;
			}
		}

		if (spp_ok == 1)
		{
			printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong10\n");
			return 1;	//Modify Success
		}

		//error --> reset SFR
		cmd[0] = 0x43;
		cmd[1] = 0x01;
		cmd[2] = 0x00;
		cmd[3] = 0x06;
		if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
			return 0;
		udelay(10);

		//write 16-byte SFR
		for (i = 0; i < 4; i++)
		{
			cmd[0] = 0x44;
			cmd[1] = i;			//word
			cmd[2] = 0x00;		//page
			cmd[3] = 0x00;		//sector
			if(i2c_master_send(hx_i2c_client, cmd, 4) < 0)
				return 0;
			udelay(10);

			cmd[0] = 0x45;
			cmd[1] = spp_source[4 * i + 0];
			cmd[2] = spp_source[4 * i + 1];
			cmd[3] = spp_source[4 * i + 2];
			cmd[4] = spp_source[4 * i + 3];
			if(i2c_master_send(hx_i2c_client, cmd, 5) < 0)
				return 0;
			udelay(10);

			cmd[0] = 0x4A;		//write SFR
			if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
				return 0;
			udelay(10);
		}
	}	//retry

	printk(KERN_ERR "entern Himax ChangeIrefSPP by zhuxinglong8\n");
	return 0;			//No 3u Iref setting

	#else
	ModifyIref(1);
	return 0;
	#endif
}
#endif


#if ((HIMAX_UPGRADE) | (HIMAX_RAWDATA))

static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) 
	{
		printk(KERN_ERR "[TP]TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	#if HIMAX_UPGRADE
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) 
	{
		printk(KERN_ERR "[TP]TOUCH_ERR: create_file debug_level failed\n");
		return ret;
	}
	#endif
	
	#if HIMAX_RAWDATA
	himax_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) 
	{
		printk(KERN_ERR "[TP]TOUCH_ERR: create_file register failed\n");
		return ret;
	}

	//ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	//if (ret) 
	//{
	//	printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
	//	return ret;
	//}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) 
	{
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	#endif
	
	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
	#if HIMAX_UPGRADE
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	#endif

	#if HIMAX_RAWDATA	
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	//sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	#endif
	
	kobject_del(android_touch_kobj);
}
#endif /* ((HIMAX_UPGRADE) | (HIMAX_RAWDATA)) */


#ifdef LCT_MTK_CTP_INFO_SUPPORT
//Himax: Read Falsh Function
/*
static int hx8526_read_flash(unsigned char *buf, unsigned int addr, unsigned int length) //OK
{
	unsigned char index_byte,index_page,index_sector;
	unsigned char buf0[7];	
	unsigned int i;
	unsigned int j = 0;
	
	index_byte = (addr & 0x001F);
	index_page = ((addr & 0x03E0) >> 5);
	index_sector = ((addr & 0x1C00) >> 10);

	for (i = addr; i < addr + length; i++)
	{
		buf0[0] = i&0x1F;
		buf0[1] = (i>>5)&0x1F;
		buf0[2] = (i>>10)&0x1F;

		i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &buf0[0]);//himax_i2c_master_write(slave_address, 4, buf0);
		msleep(1);

		i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x46, 0, &buf0[0]);

		//Himax: Read FW Version to buf
		i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x59, 4, &buf[0+j]);
		msleep(1);
		j += 4;
	}

	return 1;
}
*/   //20130124
static int hx8526_read_flash(unsigned char *buf, unsigned int addr_start, unsigned int length) //OK
{
	u16 i;
	unsigned int j = 0;
	unsigned char add_buf[4];
		
    for (i = addr_start; i < addr_start+length; i++)
    {
        add_buf[0] = i & 0x1F;
        add_buf[1] = (i >> 5) & 0x1F;
        add_buf[2] = (i >> 10) & 0x1F;
        
        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &add_buf[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }
        udelay(10);
        
        if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x46, 0, &add_buf[0]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }
        udelay(10);
        if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x59, 4, &buf[0+j]))< 0){
            printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
            return -1;
        }	
        udelay(10);
        j=j+4;
    }
	
	return 1;
}

u8 himax_read_FW_ver(void) //OK
{
	unsigned int i;
	unsigned char cmd[5];
	unsigned char buffer[20];

	//Himax: Power On Flow
	cmd[0] =0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(1);
	
	cmd[0] =0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x35, 1, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(1);
		
	cmd[0] =0x0F;
	cmd[1] =0x53;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x36, 2, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(1);
	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(120);
	//Himax: Power On Flow End 

	//Himax: Flash Read Enable	
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(1);

	//Himax: Read FW Major Version
	//if (hx8526_read_flash(FW_VER_MAJ_FLASH_buff, FW_VER_MAJ_FLASH_ADDR, FW_VER_MAJ_FLASH_LENG) < 0)	goto HimaxErr;
	//Himax: Read FW Minor Version
	//if (hx8526_read_flash(FW_VER_MIN_FLASH_buff, FW_VER_MIN_FLASH_ADDR, FW_VER_MIN_FLASH_LENG) < 0)	goto HimaxErr;
	//Himax: Read CFG Major Version
	if (hx8526_read_flash(CFG_VER_MAJ_FLASH_buff, CFG_VER_MAJ_FLASH_ADDR, CFG_VER_MAJ_FLASH_LENG) < 0)	goto HimaxErr;
	//Himax: Read CFG Minor Version
	if (hx8526_read_flash(CFG_VER_MIN_FLASH_buff, CFG_VER_MIN_FLASH_ADDR, CFG_VER_MIN_FLASH_LENG) < 0)	goto HimaxErr;

	//{
	//Himax: Check FW Major Version
	//sprintf(buffer, "%s\n", FW_VER_MAJ_FLASH_buff);
	//printk(KERN_ERR "Himax TP: FW_VER_MAJ_FLASH_buff = %x,%x,%x,%x\n", FW_VER_MAJ_FLASH_buff[0],FW_VER_MAJ_FLASH_buff[1],FW_VER_MAJ_FLASH_buff[2],FW_VER_MAJ_FLASH_buff[3]);
	
	//Himax: Read FW Minor Version
	//sprintf(buffer, "%s\n", FW_VER_MIN_FLASH_buff);
	//printk(KERN_ERR "Himax TP: FW_VER_MIN_FLASH_buff = %x,%x,%x,%x\n", FW_VER_MIN_FLASH_buff[0],FW_VER_MIN_FLASH_buff[1],FW_VER_MIN_FLASH_buff[2],FW_VER_MIN_FLASH_buff[3]);
	
	//Himax: Read CFG Major Version
	//sprintf(buffer, "%s\n", CFG_VER_MAJ_FLASH_buff);
	//printk(KERN_ERR "Himax TP: CFG_VER_MAJ_FLASH_buff = %s\n", buffer);
	
	//Himax: Read CFG Minor Version
	//sprintf(buffer, "%s\n", CFG_VER_MIN_FLASH_buff);
	//printk(KERN_ERR "Himax TP: CFG_VER_MIN_FLASH_buff = %s\n", buffer);
	//}
	
	//Himax: Flash Read Disable
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
		goto HimaxErr;
	msleep(1);
	return 1;

HimaxErr:
	return 0;
}
#endif

  

static struct i2c_driver tpd_i2c_driver =
{
	 .driver = 
	 {
		.name = "HX8527",
	 },
	  .probe = tpd_probe,
	  .remove = __devexit_p(tpd_remove),
      .id_table = tpd_i2c_id,
	  .detect = tpd_detect,
};


 
extern struct input_dev *kpd_input_dev;

static int himax_ts_poweron()
{    
    int retval = TPD_OK;
		char data[3];
		#ifdef HX_85XX_D_SERIES_PWON
		    //Himax Touch Controller power-on sequence
			data[0] =0x02;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &data[0]))< 0)
			{
				return -1;
			}
			msleep(1);
			
			data[0] =0x0f;
			data[1] =0x53;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x36, 2, &data[0]))< 0)
			{
				return -1;
			}
			msleep(1);	
			data[0] =0x04;
			data[1] =0x03;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xdd, 2, &data[0]))< 0)
			{
				return -1;
			}
			msleep(1);
			
			data[0] =0x01;
			data[1] =0x36;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xb9, 2, &data[0]))< 0)
			{
				return -1;
			}
			msleep(1);
			
			data[0] =0x01;
			data[1] =0xF5;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xcb, 2, &data[0]))< 0)
			{
				return -1;
			}
			msleep(1);
			
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x83, 0, &data[0]))< 0)
			{
				return -1;
			}
			msleep(120);
			
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &data[0]))< 0)
			{
				return -1;
			}
			msleep(120);
			
			
			#if HIMAX_DEBUG
			printk("hx8526 power on success\n");
		    #endif
			
		
		#endif //end HX_85XX_D_SERIES_PWON 
	
    return retval;   
}




//Himax: Touch Down for Coor.
static  void tpd_down(int x, int y, int p) 
{
	input_report_key(tpd->dev, BTN_TOUCH, 1);	
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 	
	input_mt_sync(tpd->dev);
	printk("tpd_down[%4d %4d %4d]\n ", x, y, p);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 1);  
	}
	TPD_EM_PRINT(x, y, x, y, p-1, 1);
}

//Himax: Touch Up for Coor. and Key  
static  int tpd_up(int x, int y, int *count) 
{	

	printk("tpd_up[%4d %4d]\n ", x, y);

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, 0, 0);
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{   
		tpd_button(x, y, 0); 
	}
}





#ifdef TPD_PROXIMITY
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}
static int tpd_enable_ps(int enable)
{
	int ret = 0;
	char data[1] = {0};
	
	if(enable)	//Proximity On
	{
		data[0] =0x01;
		if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x92, 1, &data[0]))< 0)
			ret = 1;
		else
			tpd_proximity_flag = 1;
	}
	else	//Proximity On
	{
		data[0] =0x00;
		if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x92, 1, &data[0]))< 0)
			ret = 1;
		else
			tpd_proximity_flag = 0;
	}
	msleep(1);

	return ret;
}
int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				TPD_PROXIMITY_DMESG("Set delay parameter error!\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				TPD_PROXIMITY_DMESG("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if((tpd_enable_ps(1) != 0))
					{
						TPD_PROXIMITY_DMESG("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						TPD_PROXIMITY_DMESG("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				TPD_PROXIMITY_DMESG("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				sensor_data->values[0] = tpd_get_ps_value();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;		
			}
			break;
		default:
			TPD_PROXIMITY_DMESG("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	return err;
}
#endif


kal_bool keyflag= false;

#if ESD_WORKAROUND
//HW Reset
void ESD_HW_REST(void)
{
						char data[2] = {0};
		
						reset_activate = 1;
						ESD_COUNTER = 0;
						
						printk(KERN_ERR "Himax TP: ESD - Reset\n");
							
						himax_HW_reset();
						//mdelay(100);

						mutex_lock(&i2c_access);
						
						#ifdef HX_85XX_D_SERIES_PWON
						    //Himax Touch Controller power-on sequence
							data[0] =0x02;
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &data[0]))< 0)
							{
								return -1;
							}
							msleep(1);
							
							data[0] =0x0f;
							data[1] =0x53;
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x36, 2, &data[0]))< 0)
							{
								return -1;
							}
							msleep(1);	
							data[0] =0x04;
							data[1] =0x03;
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xdd, 2, &data[0]))< 0)
							{
								return -1;
							}
							msleep(1);
							
							data[0] =0x01;
							data[1] =0x36;
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xb9, 2, &data[0]))< 0)
							{
								return -1;
							}
							msleep(1);
							
							data[0] =0x01;
							data[1] =0xF5;
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xcb, 2, &data[0]))< 0)
							{
								return -1;
							}
							msleep(1);
							
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x83, 0, &data[0]))< 0)
							{
								return -1;
							}
							msleep(120);
							
							if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &data[0]))< 0)
							{
								return -1;
							}
							msleep(120);
							
							
							#if HIMAX_DEBUG
							printk("hx8526 power on success\n");
						    #endif
							
						
						#endif //end HX_85XX_D_SERIES_PWON 
						
						mutex_unlock(&i2c_access);
}
#endif

static void I2C_dma_Readdata(u8 addr, u8* read_data, u16 size)
{   
//according to your platform.   
	int rc;
	unsigned short temp_addr = 0;
	temp_addr = hx_i2c_client->addr ;

#ifdef _DMA_RW_MODE_	
	if (g_pDMABuf_va == NULL)		
	return;	
#endif	
	struct i2c_msg msgs[] ={		
	{						
		.flags = I2C_M_RD,			
		.len = size,			
		#ifdef _DMA_RW_MODE_			
		.addr = addr & I2C_MASK_FLAG | I2C_DMA_FLAG,			
		.buf = g_pDMABuf_pa,			
		#else			
		.addr = addr,			
		.buf = read_data,			
		#endif		
		},	
	};	
   rc = i2c_transfer(hx_i2c_client->adapter, msgs, 1);	
   if( rc < 0 )    
	{		
	printk("I2C_dma_Readdata error %d\n", rc);	
	}	
#ifdef _DMA_RW_MODE_	
	else	
	{		
	memcpy(read_data, g_pDMABuf_va, size);	
	}	
#endif
    hx_i2c_client->addr = temp_addr;

}

static void I2C_dma_Writedata(u8 addr, u8* data, u16 size)
{    
	//according to your platform.   	
	int rc;	
	unsigned short temp_addr = 0;
	temp_addr = hx_i2c_client->addr ;
#ifdef _DMA_RW_MODE_
	if(g_pDMABuf_va == NULL)		
		return;	
	memcpy(g_pDMABuf_va, data, size);	
#endif		
	struct i2c_msg msgs[] ={		
	{						
		.flags = 0,			
		.len = size,			
#ifdef _DMA_RW_MODE_			
		.addr = addr & I2C_MASK_FLAG | I2C_DMA_FLAG,			
		.buf = g_pDMABuf_pa,			
#else			
		.addr = addr,			
		.buf = data,			
#endif		
		},	
	};	
	rc = i2c_transfer(hx_i2c_client->adapter, msgs, 1);	
	if(rc < 0)
		{		
		printk("I2C_dma_Writedata error %d,addr = %d\n", rc,addr);
		}
	hx_i2c_client->addr = temp_addr;
}

static void I2C_dma_write_Readdata(u8 cmd, u8* data, u16 size)
{    
	//according to your platform.   	
	int rc;
	struct i2c_msg msg[2];
	unsigned short temp_addr = 0;
	temp_addr = hx_i2c_client->addr ;
    #ifdef _DMA_RW_MODE_	
	if(g_pDMABuf_va == NULL)		
		return;		
	#endif		

	msg[0].addr = hx_i2c_client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &cmd;
	msg[0].ext_flag = hx_i2c_client->ext_flag;
    msg[0].timing = I2C_MASTER_CLOCK;
	
	#ifdef _DMA_RW_MODE_			
	msg[1].addr = hx_i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG,			
	msg[1].buf = g_pDMABuf_pa,			
	#else
    msg[1].addr = hx_i2c_client->addr;
	msg[1].buf =data;
	#endif
    msg[1].flags = I2C_M_RD;
    msg[1].len = size;
	msg[1].ext_flag = hx_i2c_client->ext_flag;
    msg[1].timing = I2C_MASTER_CLOCK;

	rc = i2c_transfer(hx_i2c_client->adapter, msg, 2);	
	if(rc < 0)
	{		
		printk("I2C_dma_write_Readdata error %d,addr = %d\n", rc,hx_i2c_client->addr);
	}
	#ifdef _DMA_RW_MODE_	
	else	
	{		
	   memcpy(data, g_pDMABuf_va, size);	
	}	
    #endif
	hx_i2c_client->addr = temp_addr;
}

int i2c_read_bytes(unsigned char cmd,int len,unsigned char* buf)
{
    struct i2c_msg msg[2];
    int ret;
 //printk(KERN_ERR "hx8526 : %s enter\n",__FUNCTION__);
    msg[0].addr = hx_i2c_client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &cmd;
	msg[0].ext_flag = hx_i2c_client->ext_flag;
    msg[0].timing = I2C_MASTER_CLOCK;
    
    msg[1].addr = hx_i2c_client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = len;
    msg[1].buf =buf;
	msg[1].ext_flag = hx_i2c_client->ext_flag;
    msg[1].timing = I2C_MASTER_CLOCK;

    ret = i2c_transfer(hx_i2c_client->adapter, msg, 2);
    //printk(KERN_ERR "hx8526 : %s ret = %d\n",__FUNCTION__,ret);    
    return ret;
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
#if ((HIMAX_RAWDATA) | (HIMAX_UPGRADE))
    //uint8_t *mutual_data;
    //uint8_t *self_data;
    //int mul_num, self_num;
    //int index = 0;
	int read_len = 0;

	
	//Bizzy added for common RawData
    int raw_cnt_max = PT_NUM_MAX/4;
    int raw_cnt_rmd = PT_NUM_MAX%4;
    int hx_touch_info_size, RawDataLen;
    if(raw_cnt_rmd != 0x00)
    {
    #ifdef HX_85XX_D_SERIES_PWON
		RawDataLen = 128 - ((PT_NUM_MAX+raw_cnt_max+3)*4) - 1;
     #else  
		RawDataLen = 128 - ((PT_NUM_MAX+raw_cnt_max+3)*4);
		#endif
		hx_touch_info_size = (PT_NUM_MAX+raw_cnt_max+2)*4;
	}
	else
	{
      #ifdef HX_85XX_D_SERIES_PWON
		RawDataLen = 128 - ((PT_NUM_MAX+raw_cnt_max+2)*4) - 1;
      #else	
		RawDataLen = 128 - ((PT_NUM_MAX+raw_cnt_max+2)*4);
	  #endif 
		hx_touch_info_size = (PT_NUM_MAX+raw_cnt_max+1)*4;
	}
#endif


#ifdef TPD_PROXIMITY
	int err_1;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif

	int i = 0;
	char data[128] = {0};
	u8 check_sum_cal = 0;
	u16 high_byte,low_byte;
#if HIMAX_CHECKSUM
	   #if HIMAX_5POINT_SUPPORT
	   u8 CHECK_SUM_LENGTH = 4*(PT_NUM_MAX+3) ;   	//If PT_NUM_MAX >= 5 && <8 
	   #else
	   u8 CHECK_SUM_LENGTH = 4*(PT_NUM_MAX+2) ;		//If PT_NUM_MAX <= 4
	   #endif
	//const u8 CHECK_SUM_LENGTH = 4*(PT_NUM_MAX+4) ;   	//If PT_NUM_MAX >= 8 && <=10
#endif
	int err[4] = {0};
	
#if HIMAX_UPGRADE
	#if HIMAX_RAWDATA
    if(diag_command) 
        read_len = 128;
    else
	#endif
        read_len = hx_touch_info_size;
	
	//Himax: Receive raw data about Coordinates and
	mutex_lock(&i2c_access);
	if (tpd_halt)
	{
		mutex_unlock(&i2c_access);
		#if HIMAX_DEBUG
		printk(KERN_ERR "Himax TP: tpd_touchinfo return ..\n");
		#endif
		return false;
	}
	if(diag_command)//debug get 128 byte
	{	
#ifdef _DMA_RW_MODE_
		dma_buffer_alloct();
		I2C_dma_write_Readdata(0x86, &data[0],read_len);
#endif
	}
    else
    	{	
			i2c_read_bytes(0x86,8, &data[0]);
			i2c_read_bytes(0x86,8, &data[8]);
			i2c_read_bytes(0x86,8, &data[16]);
			#if HIMAX_5POINT_SUPPORT
			i2c_read_bytes(0x86,8, &data[24]);	
			#endif
    	
    	}
	
	#if 0 //def TPD_PROXIMITY
	TPD_PROXIMITY_DEBUG("RawData: " );
	for(i=0;i<CHECK_SUM_LENGTH;i++)
		TPD_PROXIMITY_DEBUG("%x, \n",data[i]);
		
	TPD_PROXIMITY_DEBUG("\n");
    #endif
	
	mutex_unlock(&i2c_access);
#else
	mutex_lock(&i2c_access);
	if (tpd_halt)
	{
		mutex_unlock(&i2c_access);
		#if HIMAX_DEBUG
		printk(KERN_ERR "Himax TP: tpd_touchinfo return ..\n");
		#endif
		return false;
	}
	/*
	//Himax: Receive raw data about Coordinates and 
	err[0] = hwmsen_read_block(hx_i2c_client, 0x86, &(data[0x00]), 8);	// 1st/2nd layer
	err[1] = hwmsen_read_block(hx_i2c_client, 0x86, &(data[0x08]), 8);	// 3rd/4th layer
	err[2] = hwmsen_read_block(hx_i2c_client, 0x86, &(data[0x10]), 8);	// 5th/6th layer
	#if HIMAX_5POINT_SUPPORT
	err[3] = hwmsen_read_block(hx_i2c_client, 0x86, &(data[0x18]), 8);	// 7th/8th layer  // Modified 20121101
	#endif
	*/
	i2c_read_bytes(0x86,8, &data[0]);
	i2c_read_bytes(0x86,8, &data[8]);
	i2c_read_bytes(0x86,8, &data[16]);
	#if HIMAX_5POINT_SUPPORT
	i2c_read_bytes(0x86,8, &data[24]);	
	#endif
	mutex_unlock(&i2c_access);
#endif
	
	MTK_TP_DEBUG("received raw data from touch panel as following:\r\n");
	MTK_TP_DEBUG("x1:%x,y1:%x\r\n", (u16)((data[0]<<8)|data[1]),(u16)((data[2]<<8)|data[3]));
	MTK_TP_DEBUG("x2:%x,y2:%x\r\n", (u16)((data[4]<<8)|data[5]),(u16)((data[6]<<8)|data[7]));
	MTK_TP_DEBUG("x3:%x,y3:%x\r\n", (u16)((data[8]<<8)|data[9]),(u16)((data[10]<<8)|data[11]));
	MTK_TP_DEBUG("x4:%x,y4:%x\r\n", (u16)((data[12]<<8)|data[13]),(u16)((data[14]<<8)|data[15]));
	MTK_TP_DEBUG("x5:%x,y5:%x\r\n", (u16)((data[16]<<8)|data[17]),(u16)((data[18]<<8)|data[19]));
	MTK_TP_DEBUG("area1:%x,area2:%x,area3:%x,area4:%x\r\n", data[20],data[21],data[22],data[23]);
	MTK_TP_DEBUG("area5:%x,area6:%x,area7:%x,area8:%x\r\n", data[24],data[25],data[26],data[27]);    			
	MTK_TP_DEBUG("point num:%x,ID info1:%x,ID info2:%x,check sum:%x\r\n", data[28],data[29],data[30],data[31]);	


#if HIMAX_RAWDATA  //debug
	if (diag_command >= 1 && diag_command <= 6) 
	{
		int mul_num, self_num;
		int index = 0;
		/* Header: %x, %x, %x, %x\n", buf[24], buf[25], buf[26], buf[27] */
		mul_num = x_channel * y_channel;
		self_num = x_channel + y_channel;

		if (data[hx_touch_info_size] == data[hx_touch_info_size+1] && data[hx_touch_info_size+1] == data[hx_touch_info_size+2] 
			&& data[hx_touch_info_size+2] == data[hx_touch_info_size+3] && data[hx_touch_info_size] > 0) 
		{
			index = (data[hx_touch_info_size] - 1) * RawDataLen;
			for (i = 0; i < RawDataLen; i++) 
			{
			  #ifdef HX_85XX_D_SERIES_PWON
			    if ((index+i) < mul_num) 
              #else				    
				if (index < mul_num) 
              #endif
								{ //mutual
					/*if ((data[i * 2 + (hx_touch_info_size+4)] & 0x80) == 0x80)
						diag_mutual[index + i] = 0 -
							((data[i * 2 + (hx_touch_info_size+4)] << 8 | data[i * 2 + (hx_touch_info_size+4+1)]) & 0x4FFF);
					else
						diag_mutual[index + i] =
							data[i * 2 + (hx_touch_info_size+4)] << 8 | data[i * 2 + (hx_touch_info_size+4+1)];
					*/		
					diag_mutual[index + i] = data[i + hx_touch_info_size+4];	//4: RawData Header
				} 
				else 
				{//self
				 #ifdef HX_85XX_D_SERIES_PWON
				    if ((i+index) >= (self_num+mul_num))
                 #else					    
					if (i >= self_num)
                 #endif
											break;

					/*if ((data[i * 2 + (hx_touch_info_size+4)] & 0x80) == 0x80)
						diag_self[i] = 0 -
							((data[i * 2 + (hx_touch_info_size+4)] << 8 | data[i * 2 + (hx_touch_info_size+4+1)]) & 0x4FFF);
					else
						diag_self[i] =
							data[i * 2 + (hx_touch_info_size+4)] << 8 | data[i * 2 + (hx_touch_info_size+4+1)];
					*/		
				  #ifdef HX_85XX_D_SERIES_PWON
					diag_self[i+index-mul_num] = data[i + hx_touch_info_size+4];	//4: RawData Header
                  #else	
					diag_self[i] = data[i + hx_touch_info_size+4];
                  #endif	
				}
			}
		}
	}
#endif

#if ESD_WORKAROUND
		//u8 hw_reset_check[2];
		for(i = 0; i < CHECK_SUM_LENGTH; i++)
		{
				if(data[i] == 0x00)
						check_sum_cal = 1;
				else
				{
						check_sum_cal = 0;
						i = CHECK_SUM_LENGTH;
				}
		}
		
		if(check_sum_cal != 0 && reset_activate == 0
			#if HIMAX_RAWDATA
			&& diag_command == 0
			#endif
			)	//ESD Check
		{
			#if HIMAX_DEBUG
				printk(KERN_ERR "Himax TP: ESD event check\n");
			#endif
				//hwmsen_read_block(hx_i2c_client, 0x84, &(hw_reset_check[0]), 2);
				
				//if(hw_reset_check[1] == 0x03 && first_pressed > 0)
				//{
				//	printk(KERN_ERR "Himax TP: Not ESD event\n");
				//	return;
				//}
				//else
				{
					ESD_COUNTER++;
						if(ESD_COUNTER > 3)
						ESD_HW_REST();
						
						return false;
				}
		}
		else if(reset_activate) 
		{
			reset_activate = 0;
			#if HIMAX_DEBUG
			printk(KERN_ERR "Himax TP: Back from ESD reset, ready to serve.\n");
			#endif
			return false;
		}
#endif 

#if HIMAX_CHECKSUM
	for(i = 0; i < CHECK_SUM_LENGTH; i++)
		check_sum_cal += data[i];

	#if HIMAX_5POINT_SUPPORT
	  if (check_sum_cal != 0x00 || data[28] & 0xF0 != 0xF0)
	#else
	  if (check_sum_cal != 0x00 || data[20] & 0xF0 != 0xF0)	
	#endif
	{
#if HIMAX_DEBUG 
		printk(KERN_ERR "Himax TP: Coor. Checksum Error\n");
#endif



#if ESD_WORKAROUND
		#if HIMAX_RAWDATA
		if (diag_command == 0)
		#endif
		{
			ESD_COUNTER++;
			if(ESD_COUNTER > 3)
				ESD_HW_REST();
		}
#endif

		return false;
	}
#endif


#ifdef TPD_PROXIMITY
	/*added by bernard*/	
	if (tpd_proximity_flag == 1)
	{
		printk("himax proximity data[14]=%d\n",data[14]);

		for(i = 0; i < CHECK_SUM_LENGTH-1; i++)
		{
			if(data[i] == 0xFF)
				check_sum_cal = 0;
			else
			{
				check_sum_cal = 1;
				i = CHECK_SUM_LENGTH-1;
			}
		}

		
		if(((data[14] & 0x04) == 0) || (check_sum_cal == 0))
			tpd_proximity_detect = 1;	//No Proxi Detect
		else
		{
			tpd_proximity_detect = 0;	//Proxi	Detect
			TPD_PROXIMITY_DEBUG(" ps change tpd_proximity_detect=%d\n",  tpd_proximity_detect);
		}

		//get raw data
		TPD_PROXIMITY_DEBUG(" ps change tpd_proximity_detect:%d\n",  tpd_proximity_detect);
		
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_get_ps_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		
		//let up layer to know
		if((err_1 = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			TPD_PROXIMITY_DMESG("call hwmsen_get_interrupt_data fail = %d\n", err_1);
		
		if(!tpd_proximity_detect)
			return 0;
	}
	/*end of added*/   
#endif



#if HIMAX_BUTTON	
	//Himax: Get Soft Key number
	keyid  = data[22];
#endif
	
	//Himax: Get the number of the touch points
	#if HIMAX_5POINT_SUPPORT
	if (data[28] == 0xFF)
	{
		point_num = 0;
	}
	else
	{
		point_num = data[28] & 0x07;
	}
	#else
	if (data[20] == 0xFF)
	{
		point_num = 0;
	}
	else
	{
		point_num = data[20] & 0x07;
	}
	#endif /* HIMAX_5POINT_SUPPORT */

	//Himax: Check Coor.
	for (i = 0; i < PT_NUM_MAX; i++)
	{ 
		if (data[4*i] != 0xFF)
		{
			/*get the X coordinate, 2 bytes*/
			high_byte = data[4*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[4*i + 1];
			high_byte = high_byte |low_byte;

			MTK_TP_DEBUG("tpd_touchinfo[x=%d]\n ", high_byte);

			
			//Himax: Check X Resolution
			if (high_byte <= RESOLUTION_X)
			{
                high_byte = (high_byte * TMP_LCM_WIDTH)/RESOLUTION_X;//mapping LCM with CTP resolution
				cinfo->x[i] = high_byte;

          #if ESD_WORKAROUND				
				if(first_pressed == 0)
					first_pressed = 1;
          #endif
			}
			else
			{
				cinfo->x[i] = 0xFFFF;
				cinfo->y[i] = 0xFFFF;
				#if HIMAX_DEBUG
				printk(KERN_ERR "Himax TP: X Coor. Error\n");
				#endif

#if ESD_WORKAROUND
				#if HIMAX_RAWDATA
				if(diag_command == 0)
				#endif
				{				
					ESD_COUNTER++;
					if(ESD_COUNTER > 3)
						ESD_HW_REST();
				}
#endif		
		
				continue;
			}
				
			/*get the Y coordinate, 2 bytes*/
			high_byte = data[4*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[4*i+3];
			high_byte = high_byte |low_byte;


	        MTK_TP_DEBUG("tpd_touchinfo[Y=%d]\n ", high_byte);

		
			//Himax: Check Y Resolution
			if (high_byte <= RESOLUTION_Y)
			{
			    high_byte = (high_byte * TMP_LCM_HEIGHT)/RESOLUTION_Y;//mapping LCM with CTP resolution
				cinfo->y[i] = high_byte;
				
#if ESD_WORKAROUND				
				if(first_pressed == 1)
					first_pressed = 2;
#endif
			}
			else
			{
				cinfo->x[i] = 0xFFFF;
				cinfo->y[i] = 0xFFFF;
				#if HIMAX_DEBUG
				printk(KERN_ERR "Himax TP: Y Coor. Error\n");
				#endif
				
#if ESD_WORKAROUND
				#if HIMAX_RAWDATA
				if(diag_command == 0)
				#endif
				{				
					ESD_COUNTER++;
					if(ESD_COUNTER > 3)
						ESD_HW_REST();
				}
#endif		

				continue;
			}

			/*get the point index of touch point*/
			if ((data[29]&0x1F)>>i)
			{
				cinfo->id[i] = i;
			}
						cinfo->count++;

#if ESD_WORKAROUND			
			ESD_COUNTER = 0;
#endif
		}
		else
		{
			cinfo->x[i] = 0xFFFF;
			cinfo->y[i] = 0xFFFF;
			cinfo->id[i] = 0xFFFF;
		}
	}
		  
	return true;
};

static char touch_screen_is_down=0;

static int touch_event_handler(void *unused)
 {
	struct touch_info cinfo, pinfo;
	const u8 PT_LEAVE = 1;
	u8 i;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	MTK_TP_DEBUG("TPD interrupt touch_event_handler\n");

	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
						 
		tpd_flag = 0;

		set_current_state(TASK_RUNNING);
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        MTK_TP_DEBUG("mtk-tpd touch_event_handler mt_eint_mask\n");
		MTK_TP_DEBUG("TPD touch_event_handler tpd_touchinfo\n");
		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
			//TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
#if GTP_HAVE_TOUCH_KEY
                    if(0)
                    	{
				for (i = 0; i < GTP_MAX_KEY_NUM; i++)
				{
					input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01 << i));
				}                   	
                    	}
                     else
#endif			
                       {
				for(i = 0; i < PT_NUM_MAX; i++)
					{
						if (cinfo.x[i] != 0xFFFF)
					{
						//if (cinfo.p[i] != PT_LEAVE)
						tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
						//else
						//tpd_up(cinfo.x[i], cinfo.y[i],i+1);
					}
				}
				if (point_num == 0)
				tpd_up(cinfo.x[0], cinfo.y[0], i + 1);
				input_sync(tpd->dev);
				/*
				if(point_num > 0) 
				{
				tpd_down(cinfo.x[0], cinfo.y[0], 1);
				if(point_num>1)
				{
				tpd_down(cinfo.x[1], cinfo.y[1], 2);
				if(point_num >2) tpd_down(cinfo.x[2], cinfo.y[2], 3);
				if(point_num >3) tpd_down(cinfo.x[3], cinfo.y[3], 4);
				}
				input_sync(tpd->dev);
				//TPD_DEBUG("press --->\n");

				} 
				else  
				{
				tpd_up(cinfo.x[0], cinfo.y[0],1);
				//tpd_up(cinfo.x[0], cinfo.y[0], 0);
				//TPD_DEBUG("release --->\n"); 
				//input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
				}
				*/
			}
		}
	}while(!kthread_should_stop());

	return 0;
 }

static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
 
//static int Check_FW_Version(void);

static void tpd_eint_interrupt_handler(void)
{
	MTK_TP_DEBUG("TPD interrupt has been triggered\n");
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

//Himax: HW_RESET
void himax_HW_reset(void)
{
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
}


u8 isTP_Updated = 0;

#if HIMAX_UPGRADE_WITH_IFILE
//Himax: Check FW Version
static int Check_FW_Version(void)
{
    int tp_ver, i_file_ver;
	unsigned char FW_VER_MAJ_FLASH_buff[FW_VER_MAJ_FLASH_LENG * 4];
	unsigned char FW_VER_MIN_FLASH_buff[FW_VER_MIN_FLASH_LENG * 4];
	unsigned char CFG_VER_MAJ_FLASH_buff[CFG_VER_MAJ_FLASH_LENG * 4];
	unsigned char CFG_VER_MIN_FLASH_buff[CFG_VER_MIN_FLASH_ADDR * 4];
	unsigned int i;
	unsigned char cmd[5];

  /*
	cmd[0] =0x02;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &cmd[0]))< 0)
	{
		return -1;
	}
	msleep(1);
	*/
	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0)
	{
		printk(KERN_ERR "FW_Version", __LINE__);
		return -1;
	}
	msleep(120);

 	

	//Himax: Flash Read Enable
	cmd[0] = 0x01;	cmd[1] = 0x00;	cmd[2] = 0x02;	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)	return 0;	udelay(10);	

	//Himax: Read FW Major Version
	if (hx8526_read_flash(FW_VER_MAJ_FLASH_buff, FW_VER_MAJ_FLASH_ADDR, FW_VER_MAJ_FLASH_LENG) < 0)	goto err;
	//Himax: Read FW Minor Version
	if (hx8526_read_flash(FW_VER_MIN_FLASH_buff, FW_VER_MIN_FLASH_ADDR, FW_VER_MIN_FLASH_LENG) < 0)	goto err;
	//Himax: Read CFG Major Version
	if (hx8526_read_flash(CFG_VER_MAJ_FLASH_buff, CFG_VER_MAJ_FLASH_ADDR, CFG_VER_MAJ_FLASH_LENG) < 0)	goto err;
	//Himax: Read CFG Minor Version
	if (hx8526_read_flash(CFG_VER_MIN_FLASH_buff, CFG_VER_MIN_FLASH_ADDR, CFG_VER_MIN_FLASH_LENG) < 0)	goto err;
	#if 0
	//Himax: Check FW Major Version
	for (i = 0; i < FW_VER_MAJ_FLASH_LENG * 4; i++)	
	{
		if (FW_VER_MAJ_FLASH_buff[i] != *(HIMAX_FW + (FW_VER_MAJ_FLASH_ADDR * 4) + i))	
			return 1;	
	}
	
	//Himax: Read FW Minor Version
	for (i = 0; i < FW_VER_MIN_FLASH_LENG * 4; i++)	
	{
		if (FW_VER_MIN_FLASH_buff[i] != *(HIMAX_FW + (FW_VER_MIN_FLASH_ADDR * 4) + i))	
			return 1;	
	}
	
	//Himax: Read CFG Major Version
	for (i = 0; i < CFG_VER_MAJ_FLASH_LENG * 4; i++)	
	{
		if (CFG_VER_MAJ_FLASH_buff[i] != *(HIMAX_FW + (CFG_VER_MAJ_FLASH_ADDR * 4) + i))	
			return 1;	
	}
	for (i = 0; i < CFG_VER_MIN_FLASH_LENG * 4; i++)	
	{
		if (CFG_VER_MIN_FLASH_buff[i] != *(HIMAX_FW + (CFG_VER_MIN_FLASH_ADDR * 4) + i))	
			return 1;	
	}
	#else
	//Himax: Flash Read Disable
	cmd[0] = 0x00;	cmd[1] = 0x00;	cmd[2] = 0x02;	
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
	{
		printk(KERN_ERR "FW_Version", __LINE__);
		udelay(10);
		return -1;
	}

	//Himax: Read CFG Minor Version
	//if(sscanf(&(CFG_VER_MIN_FLASH_buff[1]), "%x", &tp_ver) == 0) 20130312
	printk("mtk-tpd himax read FW_VER buff[0,1,2,3]=0x%x,0x%x,0x%x,0x%x\n", 
	       CFG_VER_MIN_FLASH_buff[0], 
	       CFG_VER_MIN_FLASH_buff[1], 
	       CFG_VER_MIN_FLASH_buff[2], 
	       CFG_VER_MIN_FLASH_buff[3]);
	
	if(sscanf(&(CFG_VER_MIN_FLASH_buff[1]), "%x", &tp_ver) == 0)//  read  f.w. version shift 1byte to avoid v  buff[1]
	{   
        printk( "mtk-tpd:hx8527 tp_fw=0x%x\n", tp_ver);
        return -1;
	}

	printk("mtk-tpd:hx8527 tp_fw=0x%x,i_file_ver=0x%x\n", tp_ver,i_file_ver);
	//if(sscanf(HIMAX_FW + (CFG_VER_MIN_FLASH_ADDR * 4) , "%x", &i_file_ver) == 0) 20130312 /// for f.w. D01
	if(sscanf(HIMAX_FW + (CFG_VER_MIN_FLASH_ADDR * 4) + 1, "%x", &i_file_ver) == 0)// for f.w. D02
	{   
        printk("mtk-tpd:hx8527 i_file_ver=0x%x\n", tp_ver);
        return -1;
	}
	printk("mtk-tpd:hx8527 tp_fw=0x%x, i_file_ver=0x%x\n", tp_ver, i_file_ver);
	if (tp_ver < i_file_ver)		// enable firmware upgrade procedure with i-file
		return 1;
	else if (tp_ver > i_file_ver)	// disable firmware upgrade procedure with i-file
		return -1;
	else
		return 0;					// firmware upgrade procedure depends on checksume data
	#endif

err:
	printk(KERN_ERR "Himax TP: FW update error exit\n");
	return -1;
}

int himax_read_FW_checksum(void)
{
	int fullFileLength = sizeof(HIMAX_FW); 
	//u16 rem;
	u16 FLASH_VER_START_ADDR =1030;
	u16 FW_VER_END_ADDR = 4120;
	u16 i, j, k;
	unsigned char cmd[4];
	int ret;
	u8 fail_count = 0;
		
	printk("mtk-tpd:hx8527 TP version check start");
	//if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0)	{ret = -1;	goto ret_proc;}	
	//mdelay(120);
	
	//rem = fullFileLength % 4;
	//FLASH_VER_START_ADDR = (fullFileLength / 4) - 3;
	FLASH_VER_START_ADDR = (fullFileLength / 4) - 1;
	//if (rem == 0 && FLASH_VER_START_ADDR > 1)
	//{	
	//	FLASH_VER_START_ADDR--;
	//	rem = 4;
	//}
	FW_VER_END_ADDR = FLASH_VER_START_ADDR * 4;

	himax_FlashMode(1);	

	printk("mtk-tpd:hx8527 version check for loop start\n");
	printk("mtk-tpd:hx8527 FLASH_VER_START_ADDR=%d \n", FLASH_VER_START_ADDR);
	printk("mtk-tpd:hx8527 FW_VER_END_ADDR=%d \n", FW_VER_END_ADDR);
	for (k = 0; k < 3; k++)
	{
		ret = 1;
		j = FW_VER_END_ADDR;
		
		cmd[0] = FLASH_VER_START_ADDR & 0x1F;
		cmd[1] = (FLASH_VER_START_ADDR >> 5) & 0x1F;
		cmd[2] = (FLASH_VER_START_ADDR >> 10) & 0x1F;
		
		if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]))< 0)	{ret = -1;	goto ret_proc;}
		if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x46, 0, &cmd[0]))< 0)	{ret = -1;	goto ret_proc;}
		if((i2c_smbus_read_i2c_block_data(hx_i2c_client, 0x59, 4, &cmd[0]))< 0)	{ret = -1; 	goto ret_proc;}
		
		//for (i = 0; i < rem; i++)
		for (i = 0; i < 4; i++)
		{
			printk("mtk-tpd:hx8527 TP version check, CTPW[%x]:%x, cmd[0]:%x\n", j, HIMAX_FW[j], cmd[i]);
			if (HIMAX_FW[j] != cmd[i])
			{
				ret = 0;
				break;
			}
			j++;
		}

		if (ret == 0)	fail_count++;
		//if (ret == 1)	break;
	}
	
ret_proc:
	himax_FlashMode(0);	
	
	printk("Himax TP version check loop count[%d], fail count[%d]\n", k, fail_count);
	printk("Himax TP version check for loop end, return:%d\n", ret);

	return ret;
}

int himax_fw_upgrade_with_i_file(void)
{
	//Get the Firmware.bin and Length
  unsigned char* ImageBuffer = HIMAX_FW;
  int fullFileLength = sizeof(HIMAX_FW);
  static char data[2];
  int i, j;
	unsigned char cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;

	//Himax: Retry 3 Times
	for (j = 0; j < 3; j++) 
	{
		FileLength = fullFileLength - 2;

		himax_HW_reset();
		printk("mtk-tpd: himax_fw_upgrade_with_i_file start, line:%d\n", __LINE__);
/*
  	data[0] =0x02;
		if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x42, 1, &data[0]))< 0)
			return 0;
		mdelay(1);
*/		 
    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &cmd[0]))< 0)
    	{
	    	printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   return 0; 
    	}
		mdelay(120);

		himax_unlock_flash();  //ok

		cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
    	{
    	printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   return 0; 
    	}

    if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x4F, 0, &cmd[0]))< 0)
    	{
    	printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   return 0; 	
    	}
		mdelay(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) 
		{
		      		
			last_byte = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x44, 3, &cmd[0]))< 0)
				{
				printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   return 0;
				}

			if (prePage != cmd[1] || i == 0) 
			{
				prePage = cmd[1];

				cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}
				cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}

				cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x45, 4, &cmd[0]))< 0)
				{
				printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
				}

			cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
				{
				printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
				}
		   		
			cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
			if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
				{
				printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
				}

			if (last_byte == 1) 
			{
				cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}
		   		
				cmd[0] = 0x01;cmd[1] = 0x05;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}

				cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   		return 0;
					}
		   		
				cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x02;
				if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x43, 3, &cmd[0]))< 0)
					{
					printk(KERN_ERR "himax_fw_upgrade_with_i_file 000, line:%d\n", __LINE__);
		   			return 0;
					}

				mdelay(10);

								
				if (i == (FileLength - 1)) 
				{
					himax_FlashMode(0);
					himax_ManualMode(0);
										
					//update time too long need to kick dog
					mtk_wdt_restart(WK_WDT_EXT_TYPE);//kick external WDT 
					mtk_wdt_restart(WK_WDT_LOC_TYPE);//kick local WDT, CPU0/CPU1 kick 
					
				  					

					//on_each_cpu((smp_call_func_t)mpcore_wdt_restart,0,0); 
					
					checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);
					
					himax_lock_flash();
					if (checksumResult) //Success
					{
						printk("mtk-tpd checksumResult = 1, Success line:%d\n", __LINE__);
						himax_HW_reset();
						return 1;
					} 
					else if (!checksumResult) //Fail
					{
						printk("mtk-tpd checksumResult = 0, Fail line:%d\n", __LINE__);
						himax_HW_reset();
						return 0;
					} 
					else //Retry
					{
						himax_FlashMode(0);
						himax_ManualMode(0);
					}
				}
			}
		}
	}
	printk(KERN_ERR "himax_fw_upgrade_with_i_file end, line:%d\n", __LINE__);
	return 0;
}

static int hx_update_fw_handler(void *unused)
{
	unsigned char cmd[5];
	
	struct sched_param param = { .sched_priority = 4 };
	sched_setscheduler(current, SCHED_RR, &param);

	if(probe_fw_flage == 0)
	//msleep(200);

   	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

   	printk("[mtk-tpd himax start fw update\n");
	
/////////////////////////////////////////////////////////
	if (isTP_Updated == 0)
 	{
		//Inital
		cmd[0] = 0x42;
		cmd[1] = 0x02;
		if(i2c_master_send(hx_i2c_client, cmd, 2) < 0)
		{
			printk("mtk-tpd:himax enter upgrade with ifile cmd 0x42 fail\n");
			return 0;
		}
		udelay(10);

//    data[0] =0x00;data[1] =0x04;data[2]=0x0A;data[3]=0x0A;data[4]=0x02;
//    i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x7D, 5, &data[0);



		cmd[0] = 0x81;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
		{
			printk("mtk-tpd:himax enter upgrade with ifile cmd 0x81 fail\n");
			return 0;
		}
		mdelay(160);
		
	if(// Case-1: upgrade firmware with higher version number
		Check_FW_Version() != 0
		// Case-2: upgrade firmware with the same version number when checksum error founded
		|| (Check_FW_Version() == 0 && himax_read_FW_checksum() == 0))
	   {
		if (himax_fw_upgrade_with_i_file() == 0)
		{
			isTP_Updated = 0;
			printk("mtk-tpd:Himax TP: Upgrade Error, line:%d\n", __LINE__);
		}
		else
		{
			isTP_Updated = 1;
			printk("mtk-tpd:Himax TP: Upgrade OK, line:%d\n", __LINE__);
		}
		
		// Himax: HW_RESET
		himax_HW_reset();
		msleep(100);
	}
 }
/////////////////////////////////////////////////////////

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    printk("[mtk-tpd himax fw update\n");
	
	kthread_should_stop(); 
		return 0;
}


#endif  //// HIMAX_UPGRADE_WITH_IFILE end


#ifdef LCT_MTK_CTP_INFO_SUPPORT
static int ctp_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int cnt= 0;

	himax_HW_reset();	//Hardware Reset
	himax_read_FW_ver(); //OK
	himax_HW_reset();	//Hardware Reset
	cnt = sprintf(page, "vid:truly,firmware:%s\n",CFG_VER_MIN_FLASH_buff);
	return cnt;
}
#endif /* LCT_MTK_CTP_INFO_SUPPORT */

//Himax: Himax Touch Driver Probe and Init
//u8 isTP_Updated = 0;
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	printk("himax tpd_probe start ");
	int retval = TPD_OK;
	char data[5];
	#if HIMAX_UPGRADE_WITH_IFILE
	unsigned char cmd[5];
	#endif
	
	#ifdef TPD_PROXIMITY
	struct hwmsen_object obj_ps;
    #endif
	
		
#ifdef GTP_HAVE_TOUCH_KEY
	s32 idx = 0;
#endif
	   
	#ifdef VELOCITY_CUSTOM_HX8526
	int err=0;
	#endif
	hx_i2c_client = client;

	#if ESD_WORKAROUND	
		reset_activate = 0;
	#endif


#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++)
	{
	   	input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
	}
#endif

   	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_2800, "TP");
    	//hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "TP");      
	msleep(100);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
 	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(100);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(100);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(100);

	#if ChangeIref1u
	retval = ChangeIrefSPP();
	printk("return value = {%d} ChangeIrefSPP\n",retval);
	msleep(20);
	//if(ret > 0)
		himax_HW_reset();	//Hardware Reset
    	msleep(100);
	#endif

//--------------------------------------------------------------------------

	#if HIMAX_UPGRADE_WITH_IFILE
	if (isTP_Updated == 0)
 	{
 	
		//Inital
		cmd[0] = 0x42;
		cmd[1] = 0x02;
		if(i2c_master_send(hx_i2c_client, cmd, 2) < 0)
		{
			printk("mtk-tpd:himax enter upgrade with ifile cmd 0x42 fail\n");
			return 0;
		}
		udelay(10);

//    data[0] =0x00;data[1] =0x04;data[2]=0x0A;data[3]=0x0A;data[4]=0x02;
//    i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x7D, 5, &data[0);



		cmd[0] = 0x81;
		if(i2c_master_send(hx_i2c_client, cmd, 1) < 0)
		{
			printk("mtk-tpd:himax enter upgrade with ifile cmd 0x81 fail\n");
			return 0;
		}
		mdelay(160);
		
 	
 	
	if (// Case-1: upgrade firmware with higher version number
		Check_FW_Version() != 0
		// Case-2: upgrade firmware with the same version number when checksum error founded
		|| (Check_FW_Version() == 0 && himax_read_FW_checksum() == 0))
	{
		if (himax_fw_upgrade_with_i_file() == 0)
		{
			isTP_Updated = 0;
			printk("mtk-tpd:Himax TP: Upgrade Error, line:%d\n", __LINE__);
		}
		else
		{
			isTP_Updated = 1;
			printk("mtk-tpd:Himax TP: Upgrade OK, line:%d\n", __LINE__);
		}
		
		// Himax: HW_RESET
		himax_HW_reset();
		msleep(100);
	}
 }
#endif

//////////////////////////////////////////////////////////////////////
//thread update FW mode, but no poweron  in Himax IC, So No USED  
#if 0//HIMAX_UPGRADE_WITH_IFILE

		 if(1)
		 {
			//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
			hx_update_thread = kthread_run(hx_update_fw_handler, 0, TPD_DEVICE);
			if(IS_ERR(hx_update_thread))
			{
				retval = PTR_ERR(hx_update_thread);
				printk(TPD_DEVICE "failed to create kernel update thread: %ld\n", retval);
			}
		 }
	
		probe_fw_flage = 1;
		//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#endif	
//--------------------------------------------------------------------------

	#ifdef LCT_MTK_CTP_INFO_SUPPORT
	if(himax_read_FW_ver() == 1)
		printk(KERN_ERR "Himax TP: FW Read Pass\n");
		
	//himax_HW_reset();
	#endif
  
  //Himax: SET Interrupt GPIO, no setting PULL LOW or PULL HIGH  
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);


	mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_LOW, tpd_eint_interrupt_handler, 0);					
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	msleep(100);
	
	#ifdef TPD_PROXIMITY
	hwmsen_detach(ID_PROXIMITY);
//	obj_ps.self = NULL;
	obj_ps.polling = 0;//0-interrupt mode, 1-polling mode
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
		TPD_PROXIMITY_DMESG("attach fail = %d\n", err);
	else
		TPD_PROXIMITY_DMESG("attach OK = %d\n", err);
    #endif
	
		
	//Himax: Power On Flow
	   if( himax_ts_poweron() < 0)
    {
        printk("mtk-tpd:Himax TP himax_ts_poweron fail ");
        return -1;
    }
		tpd_load_status = 1;
 
#if HIMAX_RAWDATA
    // Himax: register sysfs node for reading raw data
    setXChannel(DEFAULT_X_CHANNEL); // X channel
    setYChannel(DEFAULT_Y_CHANNEL); // Y channel

    setMutualBuffer();
    if (getMutualBuffer() == NULL) {
       printk(KERN_ERR "Himax TP: mutual buffer allocate fail failed\n");
       return -1; 
    }
#endif

#if ((HIMAX_UPGRADE) | (HIMAX_RAWDATA))
    himax_touch_sysfs_init();
#endif

	#ifdef LCT_MTK_CTP_INFO_SUPPORT
	g_ctp_proc = create_proc_entry(CTP_PROC_FILE, 0444, NULL);
	if (g_ctp_proc == NULL) {
		printk(KERN_ERR "create_proc_entry failed\n");
	} else {
		g_ctp_proc->read_proc = ctp_proc_read;
		g_ctp_proc->write_proc = NULL;
		//g_ctp_proc->owner = THIS_MODULE;
  	printk(KERN_ERR "create_proc_entry success\n");
	}
	#endif
/*	
	#ifdef VELOCITY_CUSTOM_HX8526
	if((err = misc_register(&tpd_misc_device)))
	{
		printk(KERN_ERR "Himax TP: tpd_misc_device register failed\n");
		
	}
	#endif
*/	
	//Himax: Start Touch INT
	hx_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(hx_thread))
	{ 
		retval = PTR_ERR(hx_thread);
		printk(TPD_DEVICE "Himax TP: Failed to create kernel thread: %d\n", retval);
	}
	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	printk("Himax TP: Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	return 0;
	
HimaxErr:
	printk("HimaxErr TP: I2C transfer error, line: %d\n", __LINE__);
	return -1;
}

static int __devexit tpd_remove(struct i2c_client *client)
{   
#if ((HIMAX_UPGRADE) | (HIMAX_RAWDATA))
	himax_touch_sysfs_deinit();
#endif

#ifdef LCT_MTK_CTP_INFO_SUPPORT
	remove_proc_entry(CTP_PROC_FILE, NULL);
#endif
	printk("Himax TP: TPD removed\n");
	return 0;
}
 
 
static int tpd_local_init(void)
{ 
	printk("mtk-tpd:HIMAX hx8527I2C Touchscreen Driver tpd_local_init(Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		printk("unable to add tpd i2c driver.\n");
		return -1;
	}
    if(tpd_load_status == 0) 
    {
		printk("Hx8527 add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
    }
	
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	printk("mtk-tpd:end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
 }


 static int tpd_resume(struct i2c_client *client)
 {
	int retval = TPD_OK;
	static char data[3];
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	return TPD_OK;
#endif	
	data[0] =0x00;
	if((i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xD7, 1, &data[0]))< 0)
	{
	    return -1;
	}        
    msleep(1);    
    if( himax_ts_poweron() < 0)
    {
        return -1;
    }

	//i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x81, 0, &data);  //TP exit sleep mode
   	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	printk( "mtk-tpd:Himax TP: resume\n");
	tpd_halt = 0;
	return retval;
 }

//Himax: Suspend Function 
static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	int retval = TPD_OK;
	static char data[2];
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	return TPD_OK;
#endif
 	tpd_halt = 1;
	printk( "mtk-tpd:Himax TP: Suspend\n");
	
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mutex_lock(&i2c_access);

#ifdef TPD_CLOSE_POWER_IN_SLEEP
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	//Himax: Sense Off
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x82, 0, &data[0]);
	msleep(120);

	//Himax: Sleep In
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0x80, 0, &data[0]);
	msleep(120);

	//Himax: Deep Sleep In
	data[0] =0x01;
	i2c_smbus_write_i2c_block_data(hx_i2c_client, 0xD7, 1, &data[0]);	
	msleep(100);
#endif
	mutex_unlock(&i2c_access);

#if ESD_WORKAROUND	
	first_pressed = 0;
#endif
	
	return retval;
} 


static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "HX8527",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
};


static int __init tpd_driver_init(void) 
{	
	i2c_register_board_info(I2C_NUM, &HIMAX_TS_i2c_tpd, 1);

	if(tpd_driver_add(&tpd_device_driver) < 0){
		printk(KERN_ERR "add HIMAX HX8527 driver failed\n");	
	}
	printk("HX8527 tpd_driver_init");	
	return 0;
}
 
/* should never be called */
static void __exit tpd_driver_exit(void) 
{
	printk("HIMAX HX8527 touch panel driver exit\n");
	
	tpd_driver_remove(&tpd_device_driver);
}
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);
