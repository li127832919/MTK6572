/* 
 * Author: andrew yang <andrew.yang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "capPXS_driver.h"
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[CAPPXS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
/******************************************************************************
 * extern functions
*******************************************************************************/
/*----------------------------------------------------------------------------*/
struct capPXS_priv {
	struct alsps_hw  *hw;
	struct work_struct	eint_work;

	/*misc*/
    atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;
	
	/*data*/
	u8 			ps;             /*latest value*/
	ulong		enable; 		/*enable mask*/
	
	/*early suspend*/
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
};
/*----------------------------------------------------------------------------*/
static struct capPXS_priv *capPXS_obj = NULL;
struct capPXS_callback *capPXS_CB = NULL;
static struct platform_driver capPXS_driver;
/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_PS	   = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;
/********************************************************************/
int capPXS_enable_ps(int enable)
{
	int res;
    int actualout;
    struct capPXS_priv *obj = capPXS_obj;

    APS_FUN(f);
    
	if(!obj)
	{
		return;
	}

    if (NULL != capPXS_CB->capPXS_operate)
	{
		if ((res = capPXS_CB->capPXS_operate(NULL, SENSOR_ENABLE, &enable, sizeof(enable),
			NULL, 0, &actualout)))
		{
			if (enable)
			{
			    APS_ERR("enable ps fail: %d\n", res);
			}
            else
            {
                APS_ERR("disable ps fail: %d\n", res);
            }
			res = -1;
		}
		else
		{
			if (enable)
			{
                atomic_set(&obj->ps_deb_on, 1);
    			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
			}
            else
            {
                atomic_set(&obj->ps_deb_on, 0);
            }
            res = 0;
		}
	}
    else
    {
        APS_ERR("NULL function pointer\n");
        res = -1;
    }
	
	return res;
}
/********************************************************************/
int capPXS_read_ps(u8 *data)
{
	int res = 0;
    int actualout;
    hwm_sensor_data sensor_data;

	APS_FUN(f);

    if (NULL != capPXS_CB->capPXS_operate)
	{
		if ((res = capPXS_CB->capPXS_operate(NULL, SENSOR_GET_DATA, NULL, 0,
			&sensor_data, sizeof(sensor_data), &actualout)))
		{
            APS_ERR("read ps fail: %d\n", res);
			res = -1;
		}
        *data = sensor_data.values[0];
	}
	else
	{
        APS_ERR("NULL function pointer\n");
		res = -1;
	}

    return res;
}
/********************************************************************/
static int capPXS_get_ps_value(struct capPXS_priv *obj, u8 ps)
{
	int val = 0;
	int invalid = 0;

	val = ps;
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
            //if ps is disable do not report value
            APS_DBG("PS: not enable and do not report this value\n");
            return -1;
		}
		else
		{
		   return val;
		}
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t capPXS_show_config(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!capPXS_obj)
	{
		APS_ERR("capPXS_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&capPXS_obj->trace));     
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!capPXS_obj)
	{
		APS_ERR("capPXS_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&capPXS_obj->trace, trace);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!capPXS_obj)
	{
		APS_ERR("capPXS_obj is null!!\n");
		return 0;
	}
	
	if((res = capPXS_read_ps(&capPXS_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", capPXS_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_reg(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t capPXS_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!capPXS_obj)
	{
		APS_ERR("capPXS_obj is null!!\n");
		return 0;
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d\n", atomic_read(&capPXS_obj->ps_suspend));

	return len;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(value,   S_IWUSR | S_IRUGO, capPXS_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, capPXS_show_config,	capPXS_store_config);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, capPXS_show_trace,		capPXS_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, capPXS_show_status, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, capPXS_show_send, capPXS_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, capPXS_show_recv, capPXS_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, capPXS_show_reg, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *capPXS_attr_list[] = {
    &driver_attr_value,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int capPXS_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(capPXS_attr_list)/sizeof(capPXS_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, capPXS_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", capPXS_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int capPXS_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(capPXS_attr_list)/sizeof(capPXS_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, capPXS_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------interrupt functions--------------------------------*/
static void capPXS_eint_work(struct work_struct *work)
{
	hwm_sensor_data sensor_data;
	int res = 0;

    APS_FUN();

    if(!capPXS_obj)
	{
		APS_ERR("capPXS_obj is null!!\n");
		return;
	}

	res = capPXS_read_ps(&(capPXS_obj->ps));

    APS_LOG("capPXS_obj->ps = %d\n", capPXS_obj->ps);

	if(res != 0){
		goto EXIT_INTR_ERR;
	}else{
    	sensor_data.values[0] = capPXS_obj->ps;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	
	if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	{
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
		goto EXIT_INTR_ERR;
	}

	return;
	
	EXIT_INTR_ERR:
	
	APS_ERR("capPXS_eint_work err: %d\n", res);
}
/*----------------------------------------------------------------------------*/
static void capPXS_eint_func(void)
{
	struct capPXS_priv *obj = capPXS_obj;

    APS_FUN();
    
	if(!obj)
	{
		return;
	}
    schedule_work(&obj->eint_work);
}
/*----------------------------------------------------------------------------*/
void capPXS_CB_registration(struct capPXS_callback *CB)
{
    APS_FUN();
    
	capPXS_CB = CB;

	if (NULL != capPXS_CB->capPXS_eint_registration)
	{
		capPXS_CB->capPXS_eint_registration(capPXS_eint_func);
	}
	else
	{
		APS_ERR("null pointer!!\n");
	}
}

/*-------------------------------MISC device related------------------------------------------*/
/************************************************************/
static int capPXS_open(struct inode *inode, struct file *file)
{
	file->private_data = capPXS_obj;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/************************************************************/
static int capPXS_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long capPXS_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct capPXS_priv *obj = (struct capPXS_priv*)file->private_data;
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = capPXS_enable_ps(1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = capPXS_enable_ps(0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = capPXS_read_ps(&obj->ps)))
			{
				goto err_out;
			}
			
			dat = capPXS_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = capPXS_read_ps(&obj->ps)))
			{
				goto err_out;
			}
			
			dat = !obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;
		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if((err = capPXS_read_ps(&obj->ps)))
			{
				goto err_out;
			}
			
			if(copy_to_user(ptr, &obj->ps, sizeof(obj->ps)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;
		/*------------------------------------------------------------------------------------------*/
		
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations capPXS_fops = {
	.owner = THIS_MODULE,
	.open = capPXS_open,
	.release = capPXS_release,
	.unlocked_ioctl = capPXS_unlocked_ioctl,
};

static struct miscdevice capPXS_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ps",
	.fops = &capPXS_fops,
};

/*--------------------------------------------------------------------------------------*/
static void capPXS_early_suspend(struct early_suspend *h)
{
    APS_FUN();
}

static void capPXS_late_resume(struct early_suspend *h) 
{
    APS_FUN();
}
/*--------------------------------------------------------------------------------*/
static int capPXS_init_client()
{
	return 0;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
int capPXS_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
		int err = 0;
		int value;
		hwm_sensor_data* sensor_data;
		struct capPXS_priv *obj = (struct capPXS_priv *)self;		
		APS_FUN(f);
        APS_LOG("capPXS_ps_operate!\n");
		switch (command)
		{
			case SENSOR_DELAY:
				APS_ERR("capPXS ps delay command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Set delay parameter error!\n");
					err = -EINVAL;
				}
				break;
	
			case SENSOR_ENABLE:
				APS_ERR("capPXS ps enable command!\n");
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					APS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					value = *(int *)buff_in;

                    if(value)
        			{
        				if((err = capPXS_enable_ps(1)))
        				{
        					APS_ERR("enable ps fail: %ld\n", err); 
        					err = -1;
        				}
        				else
        				{
        				    set_bit(CMC_BIT_PS, &obj->enable);
        				}
        			}
        			else
        			{
        				if((err = capPXS_enable_ps(0)))
        				{
        					APS_ERR("disable ps fail: %ld\n", err); 
        					err = -1;
        				}
                        else
                        {
        				    clear_bit(CMC_BIT_PS, &obj->enable);
                        }
        			}
				}
				break;
	
			case SENSOR_GET_DATA:
				APS_ERR("capPXS ps get data command!\n");
				if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
				{
					APS_ERR("get sensor data parameter error!\n");
					err = -EINVAL;
				}
				else
				{
					sensor_data = (hwm_sensor_data *)buff_out;

                    if (err = capPXS_read_ps(&(obj->ps)))
                    {
                        err = -1;
                    }
                    else
                    {
                        sensor_data->values[0] = capPXS_get_ps_value(obj, obj->ps);
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                    }
				}
				break;
			default:
				APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
				err = -1;
				break;
		}
		
		return err;

}
/*----------------------------------------------------------------------------*/
static int capPXS_probe(struct platform_device *pdev)
{
	struct capPXS_priv *obj;
	struct hwmsen_object obj_ps;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	capPXS_obj = obj;

	obj->hw = get_cust_alsps_hw();

	INIT_WORK(&obj->eint_work, capPXS_eint_work);

    /*-----------------------------value need to be confirmed-----------------------------------------*/
    atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
    obj->enable = 0;
    /*-----------------------------value need to be confirmed-----------------------------------------*/

	if((err = misc_register(&capPXS_device)))
	{
		APS_ERR("capPXS_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("capPXS_device misc_register OK!\n");

	/*------------------------capPXS attribute file for debug--------------------------------------*/
	if((err = capPXS_create_attr(&capPXS_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------capPXS attribute file for debug--------------------------------------*/

	obj_ps.self = capPXS_obj;
	obj_ps.polling = obj->hw->polling_mode_ps;
	obj_ps.sensor_operate = capPXS_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = capPXS_early_suspend,
	obj->early_drv.resume   = capPXS_late_resume,
	register_early_suspend(&obj->early_drv);
	#endif
    
    return 0;
	
	exit_create_attr_failed:
	exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&capPXS_device);
		kfree(obj);
	exit:
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int capPXS_remove(struct platform_device *pdev)
{
	//APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver capPXS_driver = {
	.probe      = capPXS_probe,
	.remove     = capPXS_remove,
	.driver     = {
		.name  = "ps",
	}
};

/*----------------------------------------------------------------------------*/
static int __init capPXS_init(void)
{
	APS_FUN();
	if(platform_driver_register(&capPXS_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit capPXS_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&capPXS_driver);
}
/*----------------------------------------------------------------------------*/
module_init(capPXS_init);
module_exit(capPXS_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("andrew.yang@mediatek.com");
MODULE_DESCRIPTION("capPXS driver");
MODULE_LICENSE("GPL");

