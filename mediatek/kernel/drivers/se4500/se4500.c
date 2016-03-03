
/*
 * Symbol SE4500 imaging module driver
 *
 * Copyright (C) 2012 Motorola solutions
 * Copyright (C) 2012 MM Solutions
 *
 * Author Stanimir Varbanov <svarbanov@mm-sol.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * Raghav - Restructed se4500 to use se4500_command to remove redudancy.
 * Read back AutoLowPower to actually enter into Low power mode.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include "se4500.h"

#define SE4500_SIZE_WVGA            0
#define SE4500_I2C_DEVICE_ADDR      0x5c
#define SE4500_ACQ                  0x58
#define SE4500_ACQ_ON               0x01
#define SE4500_ACQ_OFF              0x00
#define SE4500_AIM                  0x55
#define SE4500_AIM_ON               0x01
#define SE4500_AIM_OFF              0x00
#define SE4500_AIM_DURING_EXPOSURE  0x56
#define SE4500_ILLUM                0x59
#define SE4500_ILLUM_ON             0x01
#define SE4500_ILLUM_OFF            0x00
#define SE4500_AUTO_POWER           0x74
#define SE4500_AUTO_POWER_EN        0x01
#define SE4500_TIME_TO_LOW_POWER    0x75


#define SE4500_AIM_POWER_FAILURE    0x88


#define SE4500_TIME_TO_LOWPOWER     1000 /* 1 second */

#define USE_SE4500

static u8 *SE4500_I2CDMABuf_va = NULL;
static u32 SE4500_I2CDMABuf_pa = NULL;

static struct i2c_board_info __initdata se4500_i2c_info={ I2C_BOARD_INFO("moto_sdl", (0x5c))};

struct se4500 {
    struct i2c_client * pClient;
    int i_size;
    int i_fmt;
    atomic_t open_excl;
};

struct se4500_i2c_msg {
    __u16 addr; /* slave address            */
    __u16 flags;
#define I2C_M_TEN       0x0010  /* this is a ten bit chip address */
#define I2C_M_RD        0x0001  /* read data, from slave to master */
#define I2C_M_NOSTART       0x4000  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR  0x2000  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK    0x1000  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK     0x0800  /* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN      0x0400  /* length will be first received byte */
    __u16 len;      /* msg length               */
    __u8 *buf;      /* pointer to msg data          */
};

/*
 * struct se4500
 *
 *  Main structure for storage of sensor information
 *
 * @pdata:      Access functions and data for platform level information
 * @ver:        SE4500 chip version TODO: does the SE4500 have this?
 * @model:      Model number returned during detect
 * @power:      Turn the interface ON or OFF
 */
struct se4500_dev
{
    struct se4500_platform_data*    pdata;
    int                             ver;
    char                            model[SE45PARAM_MODELNO_LEN + 1];
    char                            abSN[SE45PARAM_SERIALNO_LEN + 5];
    unsigned int                    power;
};

// WA for se4500_misc dev moto_sdl
static struct se4500* se4500_misc =  NULL;

static struct se4500_dev   SE4500Dev;
static struct se4500_dev*   pSE4500Dev = &SE4500Dev;

static unsigned int GetTickCount()
{
    struct timeval tv;
    do_gettimeofday(&tv);
    return((tv.tv_sec * 1000) + (tv.tv_usec /1000));
}

static u8 se4500_calc_checksum(u8 *data, u16 count)
{
    u16 sum = 0;
    u16 i;
    for (i = 0; i < count; i++)
        sum += data[i];

    sum = ~sum + 1;

    return (sum & 0xff);
}

static int se4500_i2c_write( struct i2c_client *client, char* buf, int dw_len)
{

        int i = 0;
        for(i = 0 ; i < dw_len; i++)
        {
                SE4500_I2CDMABuf_va[i] = buf[i];
        }

        if(dw_len <= 8)
        {
                client->addr = client->addr & I2C_MASK_FLAG;
                //MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
                return i2c_master_send(client, buf, dw_len);
        }
        else
        {
                client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
                //MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
                return i2c_master_send(client, SE4500_I2CDMABuf_pa, dw_len);
        }
}


static int se4500_i2c_read(struct i2c_client *client, char *buf, int len)
{
        int i = 0, err = 0;

        if(len < 8)
        {
                client->addr = client->addr & I2C_MASK_FLAG;
                //MSE_ERR("Sensor non-dma read timing is %x!\r\n", this_client->timing);
                return i2c_master_recv(client, buf, len);
        }
        else
        {
                client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
                //MSE_ERR("Sensor dma read timing is %x!\r\n", this_client->timing);
                err = i2c_master_recv(client, SE4500_I2CDMABuf_pa, len);

                 if(err < 0)
                 {
                        return err;
                }

                for(i = 0; i < len; i++)
                {
                        buf[i] = SE4500_I2CDMABuf_va[i];
                }
        }
}


static int se4500_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int num)
{

    client->timing = 400 ;
    if(msgs->flags &I2C_M_RD)
    {
        se4500_i2c_read(client, msgs->buf, msgs->len);
    }
    else
    {
        se4500_i2c_write(client, msgs->buf,msgs->len);
    }
}


static int se4500_write(struct i2c_client *client, u8 reg, u8 val, u8 len)
{
    struct i2c_msg msg;
    u8 data[4];
    int i;
    if (len > 3)
    {
        return -EINVAL;
    }
    data[0] = reg;
    data[1] = val;
    data[2] = se4500_calc_checksum(data, 2);
    for(i=0; i<3;i++)
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = len;
    msg.buf = data;

    return se4500_i2c_transfer(client, &msg, 1);
}





static int se4500_read(struct i2c_client *client, u8 reg, u8 *val, u8 len)
{
    struct i2c_msg msg;
    u8 data[4];
    int ret;

    if (len > 3)
        return -EINVAL;

    data[0] = reg;
    data[1] = 0;
    data[2] = se4500_calc_checksum(data, 2);
    msg.addr = client->addr;
    msg.flags = I2C_M_RD;
    msg.len = len;
    msg.buf = data;

    ret = se4500_i2c_transfer(client, &msg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "%s: i2c (line: %d)\n", __func__,
            __LINE__);
        goto out;
    }

    *val = data[1];
out:
    return ret;
}

/*
 * As per the block buster support, only mutltiples of 1, 10 and 100 is
 * supported with a max of 23500 sec. For example if want to set 1 sec,
 * we need to send value 0x15
 * 0x01 - 0x0A = 10-100 ms, 10 ms increments
 * 0x0B - 0x14 = 100-900 ms, 100 ms  increments
 * 0x15 - 0xFF = 1s - 235 s, 1 s increments
 * 0x00 = 5 ms
 */
static u8 se4500_convert_to_low_power(int time)
{
    if ((time >= 1000) && (time <= 235000))
        return (u8) ((time / 1000) + 20);
    else if( (time >= 100) &&  (time < 1000) )
        return (u8) ((time / 100) + 10);
    else if(time >= 10)
        return (u8) ((time / 10) + 1);
    else
        return (u8) time;

    return 0;
}

static int se4500_AcqCtrl(struct i2c_client *pClient, u8 cmd, u8 value)
{
    int ret;
    u8 val;
    int retry = 10;
    
    printk(KERN_INFO "%s: Enter cmd 0x%x value %d\n", __func__,cmd,value);
    
retry:
    ret = se4500_write(pClient, cmd, value, 3);
    if (ret < 0 && retry) {
        dev_err(&pClient->dev, "%s: i2c (line: %d, client addr %x)\n", __func__,__LINE__, pClient->addr);
        msleep(100);
        retry--;
        goto retry;     
    }
    if(ret < 0){
        printk(KERN_INFO "%s: cmd 0x%x value %d Failed retry %d\n", __func__,cmd,value,retry);
        goto out;
    }
    msleep(1);
    val = 0;
    ret = se4500_read(pClient, cmd, &val,3);
    if(ret < 0){
        dev_err(&pClient->dev, "%s: i2c (line: %d, client addr %x)\n", __func__,__LINE__, pClient->addr);       
        goto out;
    }else{
        printk(KERN_INFO "Read status val %d\n",val);
    }
out:    
    printk(KERN_INFO "%s: Exit cmd 0x%x value %d\n", __func__,cmd,value);
    return ret;
}

static int se4500_command(struct i2c_client *pClient, u8 cmd, u8 value)
{
    int ret;
    u8 val;
    int retry = 1;
    printk(KERN_INFO "%s: Enter cmd 0x%x value %d %u\n", __func__,cmd,value,GetTickCount());
    
retry:
    printk("\n######i2c write retry %s##########\n", pClient->name);
    ret = se4500_write(pClient, cmd, value, 3);
    if (ret < 0 && retry) {
        dev_err(&pClient->dev, "%s: i2c (line: %d, client addr %x)\n", __func__,__LINE__, pClient->addr);
        msleep(100);
        retry--;
        goto retry;     
    }
    if(ret < 0){
        printk(KERN_INFO "%s: cmd 0x%x value %d Failed retry %d\n", __func__,cmd,value,retry);
        goto out;
    }
    if(SE4500_ACQ != cmd){
        msleep(1);
        val = 0;
        ret = se4500_read(pClient, cmd, &val,3);
        if(ret < 0){
            dev_err(&pClient->dev, "%s: i2c (line: %d, client addr %x)\n", __func__,__LINE__, pClient->addr);       
            goto out;
        }else{
            printk(KERN_INFO "Read status val %d\n",val);
            if((SE4500_AIM == cmd) && (val == SE4500_AIM_POWER_FAILURE)){
                //Restart Acq
                se4500_AcqCtrl(pClient,SE4500_ACQ,SE4500_ACQ_OFF);
                mdelay(1);
                se4500_AcqCtrl(pClient,SE4500_ACQ,SE4500_ACQ_ON);
            }
        }
    }
out:    
    printk(KERN_INFO "%s: Exit cmd 0x%x value %d %u\n", __func__,cmd,value,GetTickCount());
    return ret;
}

static int se4500_auto_power(struct i2c_client *pClient)
{
    int ret;
    u8 val;
    printk(KERN_INFO "%s: Enter\n", __func__);
    
    //Set Auto Power
    ret  = se4500_command(pClient,SE4500_AUTO_POWER,SE4500_AUTO_POWER_EN);
    if(ret >= 0){
        val = se4500_convert_to_low_power(SE4500_TIME_TO_LOWPOWER);
        //Set Time to Low Power
        ret = se4500_command(pClient,SE4500_TIME_TO_LOW_POWER,val); 
        if(ret < 0){
            printk(KERN_INFO "%s: SE4500_TIME_TO_LOW_POWER Failed\n", __func__);
            goto out;
        }       
    }else{
        printk(KERN_INFO "%s: Turning On Auto Power Failed\n", __func__);   
        goto out;
    }
out:
    printk(KERN_INFO "%s: sucess\n", __func__);
    return ret;
}


static int se4500_start_sensor(struct i2c_client *pClient, int enable)
{
    int ret;

    if (enable) {
        ret = se4500_command(pClient,SE4500_ILLUM,SE4500_ILLUM_ON);
        if(ret >= 0){
            ret = se4500_command(pClient,SE4500_AIM,SE4500_AIM_ON);
            if(ret >= 0){
                ret = se4500_command(pClient,SE4500_ACQ,SE4500_ACQ_ON);
                if(ret < 0){
                    printk(KERN_INFO "Error %s Turn ON ACQ Aim Failed \n",__func__);
                    goto out;
                }
            }else{
                printk(KERN_INFO "Error %s Turn ON Aim Failed \n",__func__);
                goto out;
            }
        }else{
            printk(KERN_INFO "Error %s Turn On Illumination Failed \n",__func__);
            goto out;
        }
    }else{
        ret = se4500_command(pClient,SE4500_ACQ,SE4500_ACQ_OFF);
        if(ret < 0){
            dev_err(&pClient->dev, "%s: Turn Off Acq Failed; ret = %d", __func__, ret);
            goto out;
        }
    }
out:
    if (ret < 0)
        dev_err(&pClient->dev, "%s: %s (ret = %d)\n", __func__,
            enable ? "start" : "stop",
            ret);

    return ret;
}


static int se4500_read_ext(struct i2c_client* pClient, u8* data, int data_length)
{
    struct i2c_msg  msg;
    int             err;

    msg.addr   = pClient->addr;
    msg.flags  = I2C_M_RD;
    msg.len    = data_length;
    msg.buf    = data;
    err         = se4500_i2c_transfer(pClient, &msg, 1);
    if ( err >= 0 )
    {
        err = 0;    // Success
    }
    return(err);
}

/*
 * se4500_write - Write a command to the SE4500 device
 * @pClient:        i2c driver client structure
 * @data:           pointer to data to write
 * @data_length:    length of data to write
 *
 * Write a command to the SE4500 device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int se4500_write_ext(struct i2c_client* pClient, u8* data, int data_length)
{
    struct i2c_msg  msg;
    int             err;

    msg.addr   = pClient->addr;
    msg.flags  = 0;
    msg.len    = data_length;
    msg.buf    = data;
    err         = se4500_i2c_transfer(pClient, &msg, 1);
    if ( err >= 0 )
    {
        err = 0;    // Non-negative indicates success
    }

    return(err);
}

static int se4500_command_ext(struct i2c_client* pClient, u8 bCmd, u8* pParam, int num_param, u8* pResp, int resp_len)
{
    int retVal;
    int iIdx;
    int iCmdLen;
    u8  abCmd[SE45_MAX_CMD_LEN];
    u8  bCkSum;
    u8  bTmp;

    // Make sure command, params and checksum will fit in buffer
    if ( (num_param >= (sizeof(abCmd) - 2)) || (num_param < 0) )
    {
        //v4l_err(pClient, "se4500_command: invalid param count: %d\n", num_param);
        return(-EINVAL);
    }

    // Build command and calculate checksum
    abCmd[0] = bCkSum = bCmd;
    for ( iIdx = 0; iIdx < num_param; )
    {
        bTmp = pParam[iIdx++];
        abCmd[iIdx] = bTmp;
        bCkSum += bTmp;
    }
    abCmd[++iIdx] = -bCkSum;    // Store checksum

    iCmdLen = num_param + 2;
    retVal = -EIO;

    //v4l_info(pClient, "se4500_command: %d %02X:%02X:%02X\n", iCmdLen, abCmd[0], abCmd[1], abCmd[2]);

    // Try up to 3 times to send the command
    for ( iIdx = 0; iIdx < 3; ++iIdx )
    {
        retVal = se4500_write_ext(pClient, abCmd, iCmdLen);
        if ( 0 == retVal )
        {
            // Command successfully sent
            // Try up to 3 times to read the response
            for ( iIdx = 0; iIdx < 3; ++iIdx )
            {
                msleep(5);
                retVal = se4500_read_ext(pClient, pResp, resp_len);
                if ( 0 == retVal )
                {
                    // TODO: Should we check for ACK?
                    //v4l_info(pClient, "se4500_command: resp=%d %02X:%02X\n", retVal, pResp[0], pResp[1]);
                    return(resp_len);
                }
            }
            //v4l_err(pClient, "Read %02X response failed, err=%d\n", bCmd, retVal);
            return(retVal);
        }
    }
    //v4l_err(pClient, "Write %02X failed, err=%d\n", bCmd, retVal);
    return(retVal);
}


/*
 * se4500_detect - Detect if an SE4500 is present, and if so get the model number
 * @pSubdev:    pointer to the V4L2 sub-device driver structure
 *
 * Detect if an SE4500 is present
 * Returns a negative error number if no device is detected, or 0x99
 * if the model number is successfully read.
 */
int se4500_detect(void)
{
    struct se4500_dev*  pSE4500 = pSE4500Dev;
    struct i2c_client*  pClient = NULL;
    int retVal;
    int numParam;
    u8  abParam[2];
    u8  abResp[SE45PARAM_MODELNO_LEN + 4];


    if (se4500_misc != NULL)
        pClient = se4500_misc->pClient;
    else
        return 0;

    // Start and stop acquisition as a workaround for the AIM not working first time problem
    numParam = 1;
    abParam[0] = 1;
    // Uncomment the following to get a visual confirmation that I2C is working at startup
    // retVal = se4500_command(pClient, SE45OP_ILLUMDURINGEXPOSURE, abParam, numParam, abResp, 2);
    retVal = se4500_command_ext(pClient, SE45OP_ARMACQUISITION, abParam, numParam, abResp, 2);
    abParam[0] = 0;
    retVal = se4500_command_ext(pClient, SE45OP_ARMACQUISITION, abParam, numParam, abResp, 2);

    // Try to get the model number from the sensor
    numParam = 2;
    abParam[0] = (SE45PARAM_MODELNO & 0x00FF) >> 0;
    abParam[1] = (SE45PARAM_MODELNO & 0xFF00) >> 8;
    retVal = se4500_command_ext(pClient, SE45OP_GETPARAM, abParam, numParam, abResp, sizeof(abResp));
    if ( retVal > 0 )
    {
        memcpy(pSE4500->model, abResp + 4, sizeof(pSE4500->model) - 1);
        printk("SE4500 model=%s\n", (char*)pSE4500->model);
        // SE-4500DL-I000R
        return retVal;
    }
    else
    {
        u8  abSN[SE45PARAM_SERIALNO_LEN + 5];

        abParam[0] = (SE45PARAM_SERIALNO & 0x00FF) >> 0;
        abParam[1] = (SE45PARAM_SERIALNO & 0xFF00) >> 8;
        memset(abSN, 0, sizeof(abSN));
        retVal = se4500_command_ext(pClient, SE45OP_GETPARAM, abParam, numParam, abSN, sizeof(abSN) - 1);
        if ( retVal > 0 )
        {
            printk("SE4500 S/N=%s\n", (char*) abSN);
            memcpy(pSE4500->abSN, abSN, sizeof(pSE4500->abSN) - 1);
        }
    }

    printk("SE4500 model=%s\n", (char*)pSE4500->model);
    return -1;
}

int se4500_s_stream(int streaming){
    struct i2c_client*  pClient = se4500_misc->pClient;

    // If the 'misc' device is open, do not attempt to enable or disable acquisition
    int retVal;
    int numParam;
    u8  abResp[2];
    u8  abParam[1];

    numParam = 1;
    if ( streaming ){
        abParam[0] = 0x00; //0x00=60fps, 0x01=30fps, 0x02=15fps, 0x03=10fps
        retVal = se4500_command_ext(pClient, SE45OP_FRAMERATE, abParam, numParam, abResp, sizeof(abResp));

        // If streaming is being turned on, set the acquisition mode for imaging and enable illumination
        abParam[0] = ACQMODE_IMAGE;
        retVal = se4500_command_ext(pClient, SE45OP_ACQUISITIONMODE, abParam, numParam, abResp, sizeof(abResp));
        abParam[0] = 1;
        retVal = se4500_command_ext(pClient, SE45OP_ILLUMDURINGEXPOSURE, abParam, numParam, abResp, sizeof(abResp));
    }
    // Turn acquisition on or off
    abParam[0] = streaming ? 1 : 0;
    retVal = se4500_command_ext(pClient, SE45OP_ARMACQUISITION, abParam, numParam, abResp, sizeof(abResp));

    printk("SE4500 set stream %s called\n", streaming ? "on" : "off");

    return(0);
}

static int se4500_init(struct i2c_client *client)
{
    dev_dbg(&client->dev, "Sensor initialized\n");

    return 0;
}

void ToggleCamRestGPIO()
{
    int error = 0;
    printk(KERN_INFO "ToggleCamRestGPIO ++\n");

    printk(KERN_INFO "ToggleCamRestGPIO --\n");
}

// Support for moto_sdl to be exposed to the IAL
static int se4500_misc_open(struct inode* node, struct file* file)
{
    int i ;
    
    if ( atomic_inc_return(&se4500_misc->open_excl) != 1 )
    {
        atomic_dec(&se4500_misc->open_excl);
        return -EBUSY;
    }
  
    file->private_data = se4500_misc;

    return(0);
}

static long se4500_misc_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    struct se4500* se4500;
    struct i2c_rdwr_ioctl_data rdwr_data;
    struct se4500_i2c_msg msg;
    u8 __user* usr_data;
    int ret = 0;
    

    se4500 = file->private_data;
    
    if ( (se4500 == NULL) || (cmd != I2C_RDWR) || !arg ) {
        return -EINVAL;
    }

    if ( copy_from_user(&rdwr_data, (struct i2c_rdwr_ioctl_data __user*) arg, sizeof(rdwr_data)) ) {
        return -EFAULT;
    }

    if ( rdwr_data.nmsgs != 1 ) {
        return -EINVAL;
    }

    if ( copy_from_user(&msg, rdwr_data.msgs, sizeof(struct se4500_i2c_msg)) ) {
        return -EFAULT;
    }

    // Only allow transfers to the SE4500, limit the size of the message and don't allow received length changes
    if ( (msg.addr != SE4500_I2C_DEVICE_ADDR) || (msg.len > 256) || (msg.flags & I2C_M_RECV_LEN) ) {
        return -EINVAL;
    }

    // Map the data buffer from user-space to kernel space
    // WA reuse same structure for message
    usr_data = (u8 __user*) msg.buf;
    msg.buf = memdup_user(usr_data, msg.len);
    if ( IS_ERR(msg.buf) )
    {
        return(PTR_ERR(msg.buf));
    }

    ret = se4500_i2c_transfer(se4500->pClient, &msg, 1);
    if ( (ret >= 0) && (msg.flags & I2C_M_RD) ) {
        // Successful read, copy data to user-space
        if ( copy_to_user(usr_data, msg.buf, msg.len) ) {
            ret = -EFAULT;
        }
    }
    kfree(msg.buf);
    return ret;
}

static int se4500_misc_release(struct inode* node, struct file* file)
{
    atomic_dec(&se4500_misc->open_excl);
    return(0);
}

static const struct file_operations se4500_misc_fops =
{
    .owner = THIS_MODULE,
    .unlocked_ioctl = se4500_misc_ioctl,
    .open = se4500_misc_open,
    .release = se4500_misc_release,
};

static struct miscdevice se4500_misc_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "moto_sdl",
    .fops = &se4500_misc_fops,
};


static int se4500_suspend (struct i2c_client *client, pm_message_t mesg){
    printk(KERN_INFO "%s ++\n",__func__);
    printk(KERN_INFO "%s --\n",__func__);
    return 0;
}

static int se4500_resume (struct i2c_client *client){
    printk(KERN_INFO "%s ++\n",__func__);   
    ToggleCamRestGPIO();
    se4500_auto_power(client);
    printk(KERN_INFO "%s --\n",__func__);
    return 0;
}

static int se4500_probe(struct i2c_client *client,
            const struct i2c_device_id *did)
{
    struct se4500 *se4500;
    int ret;

     SE4500_I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &SE4500_I2CDMABuf_pa, GFP_KERNEL);
        if(!SE4500_I2CDMABuf_va)
        {
                printk("[TSP] dma_alloc_coherent error\n");
          return -ENOMEM;
        }


    se4500 = kzalloc(sizeof(*se4500), GFP_KERNEL);
    if (!se4500)
    {
        printk("\r\n se4500 allocation fails\n");
        return -ENOMEM;
    }
    
    se4500->i_size = SE4500_SIZE_WVGA;
    se4500->i_fmt = 0; /* First format in the list */
    se4500->pClient = client ;

    ret = se4500_init(client);
    if (ret) {
        dev_err(&client->dev, "Failed to initialize sensor\n");
        ret = -EINVAL;
    }

    ToggleCamRestGPIO();

    se4500_misc = se4500;
    atomic_set(&se4500_misc->open_excl, 0);

    misc_register(&se4500_misc_device);

    printk(KERN_ERR "%s: sucess\n", __func__);

    return ret;
}

static int se4500_remove(struct i2c_client *client)
{
    struct se4500 *se4500 = se4500_misc ;
    misc_deregister(&se4500_misc_device);

    client->driver = NULL;
    kfree(se4500);

    if(SE4500_I2CDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, SE4500_I2CDMABuf_va, SE4500_I2CDMABuf_pa);
        SE4500_I2CDMABuf_va = NULL;
        SE4500_I2CDMABuf_pa = 0;
    }
    se4500_misc= NULL;

    return 0;
}

static const struct i2c_device_id se4500_id[] = {
    { "moto_sdl", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, se4500_id);

static struct i2c_driver se4500_i2c_driver = {
    .driver = {
        .name = "moto_sdl",
    },
    .probe = se4500_probe,
    .remove = se4500_remove,
    .suspend = se4500_suspend,
    .resume = se4500_resume,
    .id_table = se4500_id,
};

static int __init se4500_mod_init(void)
{
    printk("se4500 :  MediaTek  se4500  driver init\n");
    i2c_register_board_info(0, &se4500_i2c_info, 1);
    return i2c_add_driver(&se4500_i2c_driver);
}

static void __exit se4500_mod_exit(void)
{
    i2c_del_driver(&se4500_i2c_driver);
}

module_init(se4500_mod_init);
module_exit(se4500_mod_exit);

MODULE_DESCRIPTION("Symbol SE4500 Imaging module driver");
MODULE_AUTHOR("Stanimir Varbanov <svarbanov@mm-sol.com>");
MODULE_LICENSE("GPL v2");

