/*
 *  stk3x3x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x
 *  , and stk3410 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2016 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sensors.h>
#ifdef CONFIG_SAVE_AWAKEN_EVENT
#include <linux/awaken_sys_event.h>
#endif /* CONFIG_SAVE_AWAKEN_EVENT */

#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif
#include <linux/productinfo.h>
#include "stk3x3x.h"
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#define DRIVER_VERSION  "3.11.0"


/* Driver Settings */
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#define STK_ALS_CHANGE_THD  1
#define STK_INT_PS_MODE     1       /* 1, 2, or 3   */


//#define STK_TUNE0
#define CALI_EVERY_TIME

#define CALI_PS_EVERY_TIME
//#define STK_DEBUG_PRINTF
//#define STK_ALS_FIR
// #define STK_IRS
#define STK_CHK_REG
#define NEAR_CODE   1
#define FAR_CODE    0

static uint8_t confuse_i = 0;

/* Define Register Map */
#define STK_STATE_REG           0x00
#define STK_PSCTRL_REG          0x01
#define STK_ALSCTRL_REG         0x02
#define STK_LEDCTRL_REG         0x03
#define STK_INT_REG             0x04
#define STK_WAIT_REG            0x05
#define STK_THDH1_PS_REG        0x06
#define STK_THDH2_PS_REG        0x07
#define STK_THDL1_PS_REG        0x08
#define STK_THDL2_PS_REG        0x09
#define STK_THDH1_ALS_REG       0x0A
#define STK_THDH2_ALS_REG       0x0B
#define STK_THDL1_ALS_REG       0x0C
#define STK_THDL2_ALS_REG       0x0D
#define STK_FLAG_REG            0x10
#define STK_DATA1_PS_REG        0x11
#define STK_DATA2_PS_REG        0x12
#define STK_DATA1_RESERVED1_REG         0x15
#define STK_DATA2_RESERVED1_REG         0x16
#define STK_DATA1_ALS_REG       0x13
#define STK_DATA2_ALS_REG       0x14
#define STK_DATA1_RESERVED2_REG         0x17
#define STK_DATA2_RESERVED2_REG         0x18
#define STK_DATA1_RESERVED3_REG         0x19
#define STK_DATA2_RESERVED3_REG         0x1a

#define STK_DATA1_C_REG         0x1B
#define STK_DATA2_C_REG         0x1C
#define STK_DATA1_OFFSET_REG    0x1D
#define STK_DATA2_OFFSET_REG    0x1E
#define STK_PDT_ID_REG          0x3E
#define STK_RSRVD_REG           0x3F
#define STK_SW_RESET_REG        0x80
#define STK_INT2_REG            0xA4
#define STK_ALSCTRL2_REG        0x4E
#define STK_REG_INTELLI_WAIT_PS_REG 0x1E
#define STK_BGIR_REG            0xA0
#define STK_CI_REG              0xDB

/* Define state reg */
#define STK_STATE_EN_INTELLPRST_SHIFT   3
#define STK_STATE_EN_WAIT_SHIFT     2
#define STK_STATE_EN_ALS_SHIFT      1
#define STK_STATE_EN_PS_SHIFT       0

#define STK_STATE_EN_INTELLPRST_MASK    0x08
#define STK_STATE_EN_WAIT_MASK  0x04
#define STK_STATE_EN_ALS_MASK   0x02
#define STK_STATE_EN_PS_MASK    0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT        6
#define STK_PS_GAIN_SHIFT       4
#define STK_PS_IT_SHIFT         0

#define STK_PS_PRS_MASK         0xC0
#define STK_PS_GAIN_MASK        0x30
#define STK_PS_IT_MASK          0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT       6
#define STK_ALS_GAIN_SHIFT      4
#define STK_ALS_IT_SHIFT        0

#define STK_ALS_PRS_MASK        0xC0
#define STK_ALS_GAIN_MASK       0x30
#define STK_ALS_IT_MASK         0x0F

/* Define LED ctrl reg */
#define STK_LED_IRDR_SHIFT      5
#define STK_LED_DT_SHIFT        0

#define STK_LED_IRDR_MASK       0xE0

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT      7
#define STK_INT_ENIVALIDPS_SHIFT        5
#define STK_INT_ALS_SHIFT       3
#define STK_INT_PS_SHIFT        0

#define STK_INT_CTRL_MASK       0x80
#define STK_INT_ENIVALIDPS_MASK     0x20
#define STK_INT_ALS_MASK        0x08
#define STK_INT_PS_MASK         0x07

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT         7
#define STK_FLG_PSDR_SHIFT          6
#define STK_FLG_ALSINT_SHIFT        5
#define STK_FLG_PSINT_SHIFT         4
#define STK_FLG_ALSSAT_SHIFT        2
#define STK_FLG_INVALIDPS_SHIFT         1
#define STK_FLG_NF_SHIFT        0

#define STK_FLG_ALSDR_MASK      0x80
#define STK_FLG_PSDR_MASK       0x40
#define STK_FLG_ALSINT_MASK     0x20
#define STK_FLG_PSINT_MASK      0x10
#define STK_FLG_ALSSAT_MASK     0x04
#define STK_FLG_INVALIDPS_MASK      0x02
#define STK_FLG_NF_MASK         0x01

/* misc define */
#define MIN_ALS_POLL_DELAY_NS   60000000

#ifdef STK_TUNE0
    #define STK_MAX_MIN_DIFF    200
    #define STK_LT_N_CT 100
    #define STK_HT_N_CT 150
#endif  /* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE        20000
#define STK_IRC_MIN_ALS_CODE        25
#define STK_IRC_MIN_IR_CODE     50
#define STK_IRC_ALS_DENOMI      2
#define STK_IRC_ALS_NUMERA      5
#define STK_IRC_ALS_CORREC      850

#define STK_IRS_IT_REDUCE           2
#define STK_ALS_READ_IRS_IT_REDUCE  5
#define STK_ALS_THRESHOLD           30

#define DEVICE_NAME "stk_ps"

#define ALS_NAME        "light"
#define PS_NAME     "proximity"

#define STK3X3X_VDD_MIN_UV  2800000
#define STK3X3X_VDD_MAX_UV  2950000
#define STK3X3X_VIO_MIN_UV  1750000
#define STK3X3X_VIO_MAX_UV  1950000
#define STK3338_PID                             0x58
#define STK3337_PID                             0x57
#define STK3335_PID                             0x51
#define STK3332_PID                             0x52
#define STK3331_PID                             0x53


static struct sensors_classdev stk_light_cdev = {
    .name = ALS_NAME,
    .vendor = "Sensortek",
    .version = 1,
    .handle = SENSOR_HANDLE_LIGHT,
    .type = SENSOR_TYPE_LIGHT,
    .max_range = "65535",
    .resolution = "0.0625",
    .sensor_power = "0.09",
    .min_delay = 0, /* us */
    .max_delay = 0,
    .delay_msec = 200,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .flags = 2,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_calibrate = NULL,
    .sensors_write_cal_params = NULL,
    .params = NULL,
};
static struct sensors_classdev stk_prox_cdev = {
    .name = PS_NAME,
    .vendor = "Sensortek",
    .version = 1,
    .handle = SENSOR_HANDLE_PROXIMITY,
    .type = SENSOR_TYPE_PROXIMITY,
    .max_range = "1.0",
    .resolution = "1.0",
    .sensor_power = "0.1",
    .min_delay = 0,
    .max_delay = 0,
    .delay_msec = 200,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 1,
    .enabled = 0,
    .flags = 3,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_calibrate = NULL,
    .sensors_write_cal_params = NULL,
    .params = NULL,
};

#ifdef STK_ALS_FIR
#define STK_FIR_LEN 8
#define MAX_FIR_LEN 32



struct data_filter
{
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif

uint8_t device_addr[] ={
    0x67,
    0x47,
};
const char * alps_device_info[][2] ={
    {"stk3338-v","sensortek"},
    {"stk3338","sensortek"}, 
};
struct stk3x3x_data
{
    struct i2c_client *client;
    struct stk3x3x_platform_data *pdata;
    struct sensors_classdev als_cdev;
    struct sensors_classdev ps_cdev;
    int32_t irq;

    uint16_t ir_code;
    uint16_t als_correct_factor;
    uint8_t flag_reg;
    uint8_t alsctrl_reg;
    uint8_t psctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t state_reg;
    int     int_pin;
    uint8_t wait_reg;
    uint8_t int_reg;
    uint16_t ps_thd_h;
    uint16_t ps_thd_l;
    bool bgir_update;
    bool ps_report;
    bool first_als;
#ifdef CALI_PS_EVERY_TIME
    //uint16_t ps_high_thd_boot;
    //uint16_t ps_low_thd_boot;
#endif
    struct mutex io_lock;
    struct input_dev *ps_input_dev;
    int32_t ps_distance_last;
    bool ps_enabled;
    bool re_enable_ps;
    //struct wake_lock ps_wakelock;
    struct wakeup_source wake_src;
    struct input_dev *als_input_dev;
    uint32_t als_transmittance;
    bool als_enabled;
    bool re_enable_als;
    ktime_t ps_poll_delay;
    ktime_t als_poll_delay;
    int als_root;
    int als_numerator;
    int als_fittness;
    struct work_struct stk_als_work;
    struct hrtimer als_timer;
    struct workqueue_struct *stk_als_wq;

    struct work_struct stk_ps_work;
    struct hrtimer ps_timer;  
    struct workqueue_struct *stk_ps_wq;

    struct work_struct stk_work;
    struct workqueue_struct *stk_wq;

    uint16_t last_ps_canc;
    int stk_lt_n_ct;
    int stk_ht_n_ct;
#ifdef STK_TUNE0
    uint16_t psa;
    uint16_t psi;
    uint16_t psi_set;
    //uint16_t ps_crosstalk_max;
    
    struct hrtimer ps_tune0_timer;
    struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
    ktime_t ps_tune0_delay;
    bool tune_zero_init_proc;
    uint32_t ps_stat_data[3];
    int data_count;
    int stk_max_min_diff;
    
#endif
#ifdef STK_ALS_FIR
    struct data_filter      fir;
    atomic_t                firlength;
#endif
    atomic_t    recv_reg;

struct regulator *vdd;
struct regulator *vio;
bool power_enabled;
#ifdef STK_IRS
    int als_data_index;
#endif
    uint8_t pid;
    uint32_t als_code_last;
    bool als_en_hal;
    uint32_t ps_code_last;
    uint8_t boot_cali;
};

#if( !defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] =
{
    3,
    10,
    40,
    65,
    145,
    300,
    550,
    930,
    1250,
    1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE + 1];
#endif

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *ps_data, uint8_t enable, uint8_t validate_reg);
static int32_t stk3x3x_enable_als(struct stk3x3x_data *ps_data, uint8_t enable);
static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l);
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h);
static void stk3x3x_ps_regs_show_in_log(struct stk3x3x_data *stk_data);

//static uint32_t stk_alscode2lux(struct stk3x3x_data *ps_data, uint32_t alscode);


//static uint32_t stk_lux2alscode(struct stk3x3x_data *ps_data, uint32_t lux);
//static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l);
//static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h);
/*
#ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
    static void stk_als_set_new_thd(struct stk3x3x_data *ps_data, uint16_t alscode);
#endif
*/

    static int stk_ps_val(struct stk3x3x_data *ps_data);
#ifdef STK_TUNE0
    static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *ps_data);
#endif
#ifdef STK_CHK_REG
    static int stk3x3x_validate_n_handle(struct i2c_client *client);
#endif


static int stk3x3x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
    int err = i2c_smbus_read_i2c_block_data(client,command,length,values);
    if (err < 0)
        return err;
    return 0;
    /*
    uint8_t retry;
    int err;
    struct i2c_msg msgs[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &command,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = values,
        },
    };

    for (retry = 0; retry < 5; retry++)
    {
        err = i2c_transfer(client->adapter, msgs, 2);

        if (err == 2)
            break;
        else
            mdelay(5);
    }

    if (retry >= 5)
    {
        printk(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
        return -EIO;
    }
*/
    
}

static int stk3x3x_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
    int retry;
    int err;
    unsigned char data[11];
    struct i2c_msg msg;
    int index;

    if (!client)
        return -EINVAL;
    else if (length >= 10)
    {
        printk(KERN_ERR "%s:length %d exceeds 10\n", __func__, length);
        return -EINVAL;
    }

    data[0] = command;

    for (index = 1; index <= length; index++)
        data[index] = values[index - 1];

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = length + 1;
    msg.buf = data;

    for (retry = 0; retry < 5; retry++)
    {
        err = i2c_transfer(client->adapter, &msg, 1);

        if (err == 1)
            break;
        else
            mdelay(5);
    }

    if (retry >= 5)
    {
        printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);
        return -EIO;
    }

    return 0;
}

static int stk3x3x_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
    unsigned char value;
    int err;
    err = stk3x3x_i2c_read_data(client, command, 1, &value);

    if (err < 0)
        return err;

    return value;
}

static int stk3x3x_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
    int err;
    err = stk3x3x_i2c_write_data(client, command, 1, &value);
    return err;
}
/*
static uint32_t stk_alscode2lux(struct stk3x3x_data *stk_data, uint32_t alscode)
{
    alscode += ((alscode << 7) + (alscode << 3) + (alscode >> 1));
    alscode <<= 3;
    alscode /= stk_data->als_transmittance;
    return alscode;
}

*/
/*
static uint32_t stk_lux2alscode(struct stk3x3x_data *ps_data, uint32_t lux)
{
    lux *= ps_data->als_transmittance;
    lux /= 1100;

    if (unlikely(lux >= (1 << 16)))
        lux = (1 << 16) - 1;

    return lux;
}
*/
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void stk_init_code_threshold_table(struct stk3x3x_data *ps_data)
{
    uint32_t i, j;
    uint32_t alscode;
    code_threshold_table[0] = 0;
#ifdef STK_DEBUG_PRINTF
    printk(KERN_INFO "alscode[0]=%d\n", 0);
#endif

    for (i = 1, j = 0; i < LUX_THD_TABLE_SIZE; i++, j++)
    {
        alscode = stk_lux2alscode(ps_data, lux_threshold_table[j]);
        printk(KERN_INFO "alscode[%d]=%d\n", i, alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }

    code_threshold_table[i] = 0xffff;
    printk(KERN_INFO "alscode[%d]=%d\n", i, alscode);
}

static uint32_t stk_get_lux_interval_index(uint16_t alscode)
{
    uint32_t i;

    for (i = 1; i <= LUX_THD_TABLE_SIZE; i++)
    {
        if ((alscode >= code_threshold_table[i - 1]) && (alscode < code_threshold_table[i]))
        {
            return i;
        }
    }

    return LUX_THD_TABLE_SIZE;
}
#else
/*
static void stk_als_set_new_thd(struct stk3x3x_data *ps_data, uint16_t alscode)
{
    int32_t high_thd, low_thd;
    high_thd = alscode + stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    low_thd = alscode - stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);

    if (high_thd >= (1 << 16))
        high_thd = (1 << 16) - 1;

    if (low_thd < 0)
        low_thd = 0;

    stk3x3x_set_als_thd_h(ps_data, (uint16_t)high_thd);
    stk3x3x_set_als_thd_l(ps_data, (uint16_t)low_thd);
}
*/
#endif // CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD


static void stk3x3x_proc_plat_data(struct stk3x3x_data *stk_data, struct stk3x3x_platform_data *plat_data)
{
    uint8_t w_reg;
    stk_data->state_reg = plat_data->state_reg;
    stk_data->psctrl_reg = plat_data->psctrl_reg;

    stk_data->alsctrl_reg = plat_data->alsctrl_reg;
    stk_data->ledctrl_reg = plat_data->ledctrl_reg;

    if (stk_data->pid != STK3338_PID)
        stk_data->ledctrl_reg = 0x60;

    stk_data->wait_reg = plat_data->wait_reg;

    if (stk_data->wait_reg < 2)
    {
        printk(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
        stk_data->wait_reg = 2;
    }

    stk_data->int_reg = 0x01;

    //#ifndef STK_TUNE0
    if (stk_data->ps_thd_h == 0 && stk_data->ps_thd_l == 0)
    {
        stk_data->ps_thd_h = plat_data->ps_thd_h;
        stk_data->ps_thd_l = plat_data->ps_thd_l;
    }

    //#endif
#ifdef CALI_PS_EVERY_TIME
    //stk_data->ps_high_thd_boot = plat_data->ps_thd_h;
    //stk_data->ps_low_thd_boot = plat_data->ps_thd_l;
#endif
    w_reg = 0;

    w_reg |= STK_INT_PS_MODE;

    stk_data->int_reg = w_reg;
    return;
}

static int32_t stk3x3x_init_all_reg(struct stk3x3x_data *ps_data)
{
    int32_t ret;
    uint8_t reg;
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    reg = 0x20;//gain 16
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL2_REG, reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
// 隐藏寄存器，重置远近门限不重置中断
    reg = 0x01;
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xfa, reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write register 0xfa  error \n", __func__);
        return ret;
    }

    //ALS count(G)
    //ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xA0, 0x10);
    //if (ret < 0)
    //{
    //    printk(KERN_ERR "%s: write i2c error\n", __func__);
    //    return ret;
    //}
    //A Gain
    //ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, 0xDB, 0x15);
    //if (ret < 0)
    //{
    //    printk(KERN_ERR "%s: write i2c error\n", __func__);
    //    return ret;
    //}
    //BGIR
    //reg = 0x10;
    //ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_BGIR_REG, reg);
    //
    //if (ret < 0)
    //{
    //    printk(KERN_ERR "%s: write i2c error\n", __func__);
    //    return ret;
    //}
#ifdef STK_TUNE0
    ps_data->psa = 0x0;
    ps_data->psi = 0xFFFF;
#endif
    stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
    stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);

    return 0;
}

static int32_t stk3x3x_check_pid(struct stk3x3x_data *stk_data)
{
    unsigned char value[2];
    int err;
    bool adapt_device = false;
    int8_t i  =0;
//  printk("slave address %x\n",stk_data->client->addr);
    while(adapt_device == false && i < ARRAY_SIZE(device_addr)){
        stk_data->client->addr = device_addr[i];
        err = stk3x3x_i2c_read_data(stk_data->client, STK_PDT_ID_REG, 2, &value[0]);
        if (err < 0)
        {   
            printk(KERN_ERR "%s:addr 0x%x fail, ret=%d \n", __func__,stk_data->client->addr, err);
            ++i; 
        }else{
            adapt_device = true;
            printk(KERN_ERR "%s:addr 0x%x success \n", __func__,stk_data->client->addr);
        }
        
    }
    if(i >= ARRAY_SIZE(device_addr)){
        printk(KERN_ERR "all devie addr adapt failed\n");
        return -1;
    }
    printk(KERN_INFO "%s: PID=0x%x, RID=0x%x\n", __func__, value[0], value[1]);
    stk_data->pid = value[0];

    if (value[0] == 0)
    {
        printk(KERN_ERR "PID=0x0, please make sure the chip is stk3x3x!\n");
        return -2;
    }

    return 0;
}


static int32_t stk3x3x_software_reset(struct stk3x3x_data *ps_data)
{
    int32_t r;
    uint8_t w_reg;
    w_reg = 0x7F;
    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, w_reg);

    if (r < 0)
    {
        printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
        return r;
    }

    r = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_WAIT_REG);

    if (w_reg != r)
    {
        printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        return -1;
    }

    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_SW_RESET_REG, 0);

    if (r < 0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }

    usleep_range(13000, 15000);
    return 0;
}

/*
static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_l & 0xFF00) >> 8;
    val[1] = thd_l & 0x00FF;
    ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDL1_ALS_REG, 2, val);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_h & 0xFF00) >> 8;
    val[1] = thd_h & 0x00FF;
    ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDH1_ALS_REG, 2, val);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
*/
static int stk3x3x_clear_ps_thds(struct stk3x3x_data *stk_data)
{
	int ret = 0;
    ret = stk3x3x_set_ps_thd_h(stk_data, 0xffff);
    ret = stk3x3x_set_ps_thd_l(stk_data, 0x00);
    if (ret < 0)
        printk(KERN_ERR "%s(): set thds fail, ret = %d\n", __func__, ret);
	return ret;
}

static int32_t stk3x3x_set_ps_thd(struct stk3x3x_data *stk_data)
{
    int32_t err =0;
    printk("%s set thd_h %u thd_l %u\n",__func__,stk_data->ps_thd_h,stk_data->ps_thd_l);
    err = stk3x3x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
    if (err < 0){
        printk(KERN_ERR "%s: set thd high fail, err=%d\n", __func__, err);
        return err;
    }
        
    err =stk3x3x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
    if (err < 0)
        printk(KERN_ERR "%s: set thd low fail, err=%d\n", __func__, err);
        
    return err;
}

static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_l & 0xFF00) >> 8;
    val[1] = thd_l & 0x00FF;
    ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDL1_PS_REG, 2, val);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_h & 0xFF00) >> 8;
    val[1] = thd_h & 0x00FF;
    ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDH1_PS_REG, 2, val);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}

static int32_t stk3x3x_get_ps_reading(struct stk3x3x_data *stk_data)
{
    unsigned char value[2];
    int err = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_PS_REG, 2, &value[0]);
    if (err < 0) {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
        return err;
    }

    stk_data->ps_code_last = ((value[0] << 8) | value[1]);
	//printk(KERN_ERR "%s(): get ps_raw = %d\n", __func__, stk_data->ps_code_last);
	//printk(KERN_ERR "%s(): show ps_thd: thd_h = %d; thd_l = %d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
    return ((value[0] << 8) | value[1]);
}

#if 0//((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))

static int32_t stk3x3x_set_flag(struct stk3x3x_data *stk_data, uint8_t org_flag_reg, uint8_t clr)
{
    uint8_t w_flag;
    int ret;
    w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_ALSSAT_MASK | STK_FLG_INVALIDPS_MASK);
    w_flag &= (~clr);
    //printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);
    ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_FLAG_REG, w_flag);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
#endif

static int32_t stk3x3x_get_flag(struct stk3x3x_data *ps_data)
{
    int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_FLAG_REG);
    if (ret < 0) {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	} 
    return ret;
}

static int stk3x3x_flush_flag_reg(struct stk3x3x_data *stk_data, uint8_t clr)
{
	int ret = stk3x3x_get_flag(stk_data);
	if (ret >= 0) {
		stk_data->flag_reg = ret & 0xFF;
		printk("%s flag %x w flag %x\n",
			__func__, stk_data->flag_reg, (stk_data->flag_reg & (~clr)));
		ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client,
			STK_FLAG_REG, (stk_data->flag_reg & (~clr)));
	}
	return ret;
}

static int32_t stk3x3x_set_state(struct stk3x3x_data *ps_data, uint8_t state)
{
    int ret;
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, state);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}

static int32_t stk3x3x_get_state(struct stk3x3x_data *ps_data)
{
    int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);

    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}

//----
#if 0
static void stk_ps_bgir_update(struct stk3x3x_data *ps_data, int nf);
static void stk_ps_bgir_judgement(struct stk3x3x_data *ps_data, int nf);
static void stk_ps_bgir_task(struct stk3x3x_data *ps_data, int nf);

static void stk_ps_bgir_update(struct stk3x3x_data *ps_data)
{
    ps_data->bgir_update = false;
    stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
    stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
}

static void stk_ps_bgir_judgement(struct stk3x3x_data *ps_data)
{
    int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xA5);

    if (ret == 0x28)
    {
        uint8_t bgir_data[2];
        bgir_data[0] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xB0);
        bgir_data[1] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xB1);

        if ((bgir_data[0] < 100) && (bgir_data[1] < 100) && (ps_data->ps_code_last == 0))
        {
            if (ps_data->ps_report)
            {
                ps_data->ps_report = false;
            }

            ps_data->ps_code_last = 0xFFFF;
            stk3x3x_set_ps_thd_h(ps_data, 0x01);
            stk3x3x_set_ps_thd_l(ps_data, 0x01);
            ps_data->bgir_update = true;
        }
    }
}

static void stk_ps_bgir_task(struct stk3x3x_data *ps_data, int nf)
{
    if (ps_data->bgir_update)
    {
        stk_ps_bgir_update(ps_data);
    }

    if (nf == 1)
    {
        stk_ps_bgir_judgement(ps_data);
    }
}
//----
#endif

static void stk_ps_report(struct stk3x3x_data *stk_data, int nf)
{
    stk_data->ps_report = true;
    //stk_ps_bgir_task(ps_data, nf);

    if (stk_data->ps_report)
    {   
        //if(stk_data->ps_distance_last)
        stk_data->ps_distance_last = nf;
        sensors_report_1(stk_data->ps_input_dev, nf);
        //when report near, print some info to sdcard 
        if(NEAR_CODE == nf)
            stk3x3x_ps_regs_show_in_log(stk_data);
        if (!(stk_data->wake_src.active) && (false == stk_data->pdata->ps_choose_poll)) {
            __pm_wakeup_event(&(stk_data->wake_src), jiffies_to_msecs(HZ));
        }
    }
}

static void stk_als_report(struct stk3x3x_data *stk_data, int als)
{

    if(stk_data->first_als == true){
		stk_data->first_als = false;
		printk(KERN_INFO "stk3x3x lux value %d\n",als);
	}
    
    sensors_report_2(stk_data->als_input_dev,
                als, 0, 1, &confuse_i);
    
#ifdef STK_DEBUG_PRINTF
    printk(KERN_INFO "%s: als input event %d lux\n", __func__, als);
#endif
}

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *stk_data,
		uint8_t enable, uint8_t validate_reg)
{
    int32_t ret =0;
    uint8_t w_state_reg,count=0;
    uint8_t curr_ps_enable;
    int32_t reading = 0;
   sensors_marker("stk3x3x_enable_ps()",
		"receive", "ps_en", enable ? "1" : "0");
#ifdef STK_CHK_REG
	if (validate_reg) {
        ret = stk3x3x_validate_n_handle(stk_data->client);

        if (ret < 0)
            printk(KERN_ERR "stk3x3x_validate_n_handle fail: %d\n", ret);
    }

#endif /* #ifdef STK_CHK_REG */
    curr_ps_enable = stk_data->ps_enabled ? 1 : 0;

    if (curr_ps_enable == enable)
        return 0;

    ret = stk3x3x_get_state(stk_data);
    if (ret < 0)
        return ret;

	w_state_reg = ret;
    
	if (w_state_reg != stk_data->state_reg) {
		pr_err("%s(): pre-stored state reg(0x%x) not equal to current get(0x%x), Warning!!!\n",
			__func__, stk_data->state_reg, w_state_reg);
	}
    w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);
    stk_data->state_reg = w_state_reg;
    
	if (enable) {
        w_state_reg |= STK_STATE_EN_PS_MASK;
        if (!(stk_data->als_enabled))
            w_state_reg |= STK_STATE_EN_WAIT_MASK;
        //打开距离时，先设置远近门限为0,0xffff
        ret = stk3x3x_clear_ps_thds(stk_data);
        //打开,关闭距离传感器，清除标志位
        ret = stk3x3x_flush_flag_reg(stk_data,
        	(STK_FLG_PSDR_MASK | STK_FLG_PSINT_MASK | STK_FLG_INVALIDPS_MASK));
		ret = stk3x3x_set_state(stk_data, w_state_reg);
        if (ret < 0)
            return -EIO;
		mdelay(8);;
        while (!(stk3x3x_get_flag(stk_data) & STK_FLG_PSDR_MASK) && count < 8) {
            printk("%s count %u \n",__func__,count);
            msleep(ktime_to_ms(stk_data->als_poll_delay)>>3);
            count++;   
        }
        
        if(count < 8){
            reading = stk3x3x_get_ps_reading(stk_data);
            if(reading < 0){
                printk("%s read ps data failed\n",__func__);
                ret = -EIO;
            } else {
				//每次打开距离传感器均会更新底噪值
				if (reading < stk_data->pdata->ps_crosstalk_max && 0 == stk_ps_val(stk_data)) {
					stk_data->last_ps_canc = reading;
				} else {
					pr_err("%s(): ps data(%d) is too large(MAX = %d), try last_canc!\n",
						__func__, reading, stk_data->pdata->ps_crosstalk_max);
				}

				if (0xFFFF != stk_data->last_ps_canc) {
					stk_data->ps_thd_h =
						stk_data->last_ps_canc + stk_data->stk_ht_n_ct ;
					stk_data->ps_thd_l =
						stk_data->last_ps_canc + stk_data->stk_lt_n_ct ; 
					ret = stk3x3x_set_ps_thd(stk_data);
					//report ps value when enable ps
					if (reading < stk_data->ps_thd_h){
						stk_ps_report(stk_data, FAR_CODE);
						pr_err("%s(): ps input event=%d, ps_code=%d\n",
							__func__, FAR_CODE, reading);
					}
					    ret = stk3x3x_flush_flag_reg(stk_data,
						    (STK_FLG_PSDR_MASK | STK_FLG_PSINT_MASK | STK_FLG_INVALIDPS_MASK));
					if (true == stk_data->pdata->ps_choose_poll){
					    if (reading > stk_data->ps_thd_h){
							stk_ps_report(stk_data, NEAR_CODE);
							pr_err("%s(): ps input event=%d, ps_code=%d\n",
								__func__, NEAR_CODE, reading);
					    }
					}else {
					    if (stk_data->flag_reg & STK_FLG_PSINT_MASK) {
						    int32_t near_far_state =
								    (stk_data->flag_reg & STK_FLG_NF_MASK) ? FAR_CODE : NEAR_CODE;
						    stk_ps_report(stk_data, near_far_state);
						    pr_err("%s(): ps input event=%d, ps_code=%d\n",
							    __func__, near_far_state, reading);
					    }
					}
					ret = 0;
				} else {
                    pr_err("%s(): There is no vaild ps canc yet, cease PS function this time!\n",
						__func__);
				}
                // true for using poll
                if(true == stk_data->pdata->ps_choose_poll){
				    hrtimer_start(&stk_data->ps_timer, stk_data->ps_poll_delay, HRTIMER_MODE_REL);
                }else {
                    enable_irq(stk_data->irq);
                }

				stk_data->ps_enabled = true;
                
			}
        } else {
            pr_err("ps data can't ready, some wrong during this process, cease the PS funciton!\n");
			ret = -EAGAIN;
		}
		if (!(stk_data->wake_src.active) && (true == stk_data->pdata->ps_choose_poll)) {
            __pm_stay_awake(&(stk_data->wake_src));
            printk(KERN_ERR "%s using poll: system stay awake while enable ps", __func__);
        }   
    } else {
		// Disable PS route
		 ret = stk3x3x_flush_flag_reg(stk_data,
        	(STK_FLG_PSDR_MASK | STK_FLG_PSINT_MASK | STK_FLG_INVALIDPS_MASK));
		ret = stk3x3x_set_state(stk_data, w_state_reg);
        if (ret < 0)
            return -EIO;
		if(stk_data->ps_enabled ==true){
			//true for using poll while false for using interrupt
			if(true == stk_data->pdata->ps_choose_poll){
                hrtimer_cancel(&stk_data->ps_timer);
		        cancel_work_sync(&stk_data->stk_ps_work);
			}else {
			disable_irq(stk_data->irq);
			}
		}
        stk_data->ps_enabled = false;
		ret = 0;
        sensors_report_1(stk_data->ps_input_dev, FAR_CODE);	
		if (stk_data->wake_src.active && (true == stk_data->pdata->ps_choose_poll)) {
            __pm_relax(&(stk_data->wake_src));
			printk(KERN_ERR "%s using poll: wake_src relax while disable ps", __func__);
        }
    }
    return ret;
}

static int32_t stk3x3x_enable_als(struct stk3x3x_data *stk_data, uint8_t enable)
{
    int32_t ret;
    uint8_t w_state_reg;
    uint8_t curr_als_enable = (stk_data->als_enabled) ? 1 : 0;
    sensors_marker("stk3x3x_enable_als()",
		"receive", "als_en", enable ? "1" : "0");
    if (curr_als_enable == enable)
        return 0;

    ret = stk3x3x_get_state(stk_data);

    if (ret < 0)
        return ret;

    w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)));

    if (enable)
        w_state_reg |= STK_STATE_EN_ALS_MASK;
    else if (stk_data->ps_enabled)
        w_state_reg |= STK_STATE_EN_WAIT_MASK;
    //打开,关闭光传感器，清除标志位
    ret = stk3x3x_flush_flag_reg(stk_data, STK_FLG_ALSDR_MASK);
    ret = stk3x3x_set_state(stk_data, w_state_reg);

    if (ret < 0)
        return ret;

    stk_data->state_reg = w_state_reg;

    if (enable)
    {
        stk_data->als_enabled = true;
        stk_data->first_als = true;
        hrtimer_start(&stk_data->als_timer, stk_data->als_poll_delay, HRTIMER_MODE_REL);

#ifdef STK_IRS
        stk_data->als_data_index = 0;
#endif
    }
    else
    {
        stk_data->als_enabled = false;

        hrtimer_cancel(&stk_data->als_timer);
        cancel_work_sync(&stk_data->stk_als_work);

    }

    return ret;
}


static int32_t stk3x3x_get_als_reading(struct stk3x3x_data *ps_data)
{
    int32_t als_data[5];
//    int32_t ir_data = 0;
#ifdef STK_ALS_FIR
    int index;
    int firlen = atomic_read(&ps_data->firlength);
#endif
    unsigned char value[10];
    int ret;
#ifdef STK_IRS
    const int ir_enlarge = 1 << (STK_ALS_READ_IRS_IT_REDUCE - STK_IRS_IT_REDUCE);
#endif
    ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_RESERVED1_REG, 2, &value[0]);
    ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[2]);
    ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_RESERVED2_REG, 2, &value[4]);
    ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_RESERVED3_REG, 2, &value[6]);
    ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_C_REG, 2, &value[8]);

    if (ret < 0)
    {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    als_data[0] = (value[0] << 8 | value[1]);
    als_data[1] = (value[2] << 8 | value[3]);
    als_data[2] = (value[4] << 8 | value[5]);
    als_data[3] = (value[6] << 8 | value[7]);
    als_data[4] = (value[8] << 8 | value[9]);
/*  
    printk(KERN_INFO "%s: als_data=%d, als_code_last=%d,ir_data=%d\n", __func__, als_data[1], ps_data->als_code_last,
           ir_data);
    printk(KERN_INFO "%s: zhangxuguang als_data[0]=%d, als_data[1]=%d,als_data[2]=%d,data[3]=%d data[4]=%d\n", __func__, als_data[0], als_data[1],
           als_data[2], als_data[3],als_data[4]);
*/  
    ps_data->als_code_last = als_data[1];
#ifdef STK_ALS_FIR

    if (ps_data->fir.number < firlen)
    {
        ps_data->fir.raw[ps_data->fir.number] = als_data[1];
        ps_data->fir.sum += als_data[1];
        ps_data->fir.number++;
        ps_data->fir.idx++;
    }
    else
    {
        index = ps_data->fir.idx % firlen;
        ps_data->fir.sum -= ps_data->fir.raw[index];
        ps_data->fir.raw[index] = als_data[1];
        ps_data->fir.sum += als_data[1];
        ps_data->fir.idx++;
        als_data[1] = ps_data->fir.sum / firlen;
    }

#endif
    return als_data[1];
}
static int  als_sensors_enable(struct sensors_classdev *sensors_cdev,
                    unsigned int enabled)
{
    struct stk3x3x_data *stk_data =
        container_of(sensors_cdev, struct stk3x3x_data, als_cdev);
    int err =0;
    err = stk3x3x_enable_als(stk_data,enabled);
    if (err < 0) {
        pr_err(" als enable/disable fail, err=%d\n", err);
        return err;
    }
    return 0;
}
static int stk3x3x_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct stk3x3x_data *stk_data =
        container_of(sensors_cdev, struct stk3x3x_data, ps_cdev);
	sensors_marker("stk3x3x_ps_flush()",
		"conducted", "#", "#");
    sensors_report_1(stk_data->ps_input_dev, stk_data->ps_distance_last);

	return 0;
}

static int  ps_sensors_enable(struct sensors_classdev *sensors_cdev,
                    unsigned int enabled)
{
    struct stk3x3x_data *stk_data =
        container_of(sensors_cdev, struct stk3x3x_data, ps_cdev);
    int err =0;
    err = stk3x3x_enable_ps(stk_data,enabled,1);
    if (err < 0) {
        pr_err(" ps enable/disable fail, err=%d\n", err);
        return err;
    }
    return 0;
}

#ifdef STK_CHK_REG
static int stk3x3x_chk_reg_valid(struct stk3x3x_data *ps_data)
{
    unsigned char value[9];
    int err;
    /*
    uint8_t cnt;

    for(cnt=0;cnt<9;cnt++)
    {
        value[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt+1));
        if(value[cnt] < 0)
        {
            printk(KERN_ERR "%s fail, ret=%d", __func__, value[cnt]);
            return value[cnt];
        }
    }
    */
    err = stk3x3x_i2c_read_data(ps_data->client, STK_PSCTRL_REG, 9, &value[0]);

    if (err < 0)
    {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
        return err;
    }

    if (value[0] != ps_data->psctrl_reg)
    {
        printk(KERN_ERR "%s: invalid reg 0x01=0x%2x\n", __func__, value[0]);
        return 0xFF;
    }

#ifdef STK_IRS

    if ((value[1] != ps_data->alsctrl_reg) && (value[1] != (ps_data->alsctrl_reg - STK_IRS_IT_REDUCE))
            && (value[1] != (ps_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#else
    if ((value[1] != ps_data->alsctrl_reg) && (value[1] != (ps_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#endif
    {
        printk(KERN_ERR "%s: invalid reg 0x02=0x%2x\n", __func__, value[1]);
        return 0xFF;
    }

    if (value[2] != ps_data->ledctrl_reg)
    {
        printk(KERN_ERR "%s: invalid reg 0x03=0x%2x\n", __func__, value[2]);
        return 0xFF;
    }

    if (value[3] != ps_data->int_reg)
    {
        printk(KERN_ERR "%s: invalid reg 0x04=0x%2x\n", __func__, value[3]);
        return 0xFF;
    }

    if (value[4] != ps_data->wait_reg)
    {
        printk(KERN_ERR "%s: invalid reg 0x05=0x%2x\n", __func__, value[4]);
        return 0xFF;
    }

    if (value[5] != ((ps_data->ps_thd_h & 0xFF00) >> 8))
    {
        printk(KERN_ERR "%s: invalid reg 0x06=0x%2x\n", __func__, value[5]);
        return 0xFF;
    }

    if (value[6] != (ps_data->ps_thd_h & 0x00FF))
    {
        printk(KERN_ERR "%s: invalid reg 0x07=0x%2x\n", __func__, value[6]);
        return 0xFF;
    }

    if (value[7] != ((ps_data->ps_thd_l & 0xFF00) >> 8))
    {
        printk(KERN_ERR "%s: invalid reg 0x08=0x%2x\n", __func__, value[7]);
        return 0xFF;
    }

    if (value[8] != (ps_data->ps_thd_l & 0x00FF))
    {
        printk(KERN_ERR "%s: invalid reg 0x09=0x%2x\n", __func__, value[8]);
        return 0xFF;
    }

    return 0;
}

static int stk3x3x_validate_n_handle(struct i2c_client *client)
{
    struct stk3x3x_data *ps_data = i2c_get_clientdata(client);
    int err;
    err = stk3x3x_chk_reg_valid(ps_data);

    if (err < 0)
    {
        printk(KERN_ERR "stk3x3x_chk_reg_valid fail: %d\n", err);
        return err;
    }

    if (err == 0xFF)
    {
        printk(KERN_ERR "%s: Re-init chip\n", __func__);
        err = stk3x3x_software_reset(ps_data);

        if (err < 0)
            return err;

        err = stk3x3x_init_all_reg(ps_data);

        if (err < 0)
            return err;

        //ps_data->psa = 0;
        //ps_data->psi = 0xFFFF;
        stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
        stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
#ifdef STK_ALS_FIR
        memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
#endif
        return 0xFF;
    }

    return 0;
}
#endif /* #ifdef STK_CHK_REG */

static ssize_t stk_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    int32_t reading;

    reading = stk_data->als_code_last;
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}
/*
static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t ret;
    ret = stk3x3x_get_state(ps_data);

    if (ret < 0)
        return ret;

    ret = (ret & STK_STATE_EN_ALS_MASK) ? 1 : 0;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct stk3x3x_data *ps_data = dev_get_drvdata(dev);
    uint8_t en;

    if (sysfs_streq(buf, "1"))
        en = 1;
    else if (sysfs_streq(buf, "0"))
        en = 0;
    else
    {
        printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }

    printk(KERN_INFO "%s: Enable ALS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    stk3x3x_enable_als(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    ps_data->als_en_hal = en ? true : false;
    return size;
}
*/
static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
        struct stk3x3x_data, als_cdev);
    int32_t als_reading;
    //uint32_t als_lux;
    als_reading = stk3x3x_get_als_reading(stk_data);
    //als_lux = stk_alscode2lux(stk_data, als_reading);
    printk("%s als value %d\n",__func__,als_reading);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_reading);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
    struct stk3x3x_data, als_cdev);

    unsigned long value = 0;
    int ret;
    ret = kstrtoul(buf, 16, &value);

    if (ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_als_report(stk_data, value);
    return size;
}


static ssize_t stk_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
            struct stk3x3x_data, als_cdev);

    int32_t transmittance;
    transmittance = stk_data->als_transmittance;
    return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t stk_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf,
        size_t size)
{
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
            struct stk3x3x_data, als_cdev);

    unsigned long value = 0;
    int ret;
    ret = kstrtoul(buf, 10, &value);

    if (ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->als_transmittance = value;
    return size;
}

#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
            struct stk3x3x_data, als_cdev);

    int len = atomic_read(&stk_data->firlength);
    printk(KERN_INFO "%s: len = %2d, idx = %2d\n", __func__, len, stk_data->fir.idx);
    printk(KERN_INFO "%s: sum = %5d, ave = %5d\n", __func__, stk_data->fir.sum, stk_data->fir.sum / len);
    return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}


static ssize_t stk_als_firlen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;
    int ret;
    struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(als_cdev,
            struct stk3x3x_data, als_cdev);

    ret = kstrtoull(buf, 10, &value);

    if (ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    if (value > MAX_FIR_LEN)
    {
        printk(KERN_ERR "%s: firlen exceed maximum filter length\n", __func__);
    }
    else if (value < 1)
    {
        atomic_set(&stk_data->firlength, 1);
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    }
    else
    {
        atomic_set(&stk_data->firlength, value);
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    }

    return size;
}
#endif  /* #ifdef STK_ALS_FIR */
static ssize_t stk_ps_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    int32_t reading_sub_canc =0 ,reading =0;
    int count =0;
    if (stk_data->ps_enabled){
        if(true == stk_data->pdata->ps_choose_poll)
        {
            reading_sub_canc = stk_data->ps_code_last - stk_data->last_ps_canc;
            if(reading_sub_canc < 0)
                reading_sub_canc = 0;
            return scnprintf(buf, PAGE_SIZE, "%d\n", reading_sub_canc);
        }else {
            while(!(stk3x3x_get_flag(stk_data) & STK_FLG_PSDR_MASK) && count < 4 ){
                printk("%s count %u \n",__func__,count);
                msleep(ktime_to_ms(stk_data->als_poll_delay)>>2);
                count++;   
            }
            if(count < 4){

                reading = stk3x3x_get_ps_reading(stk_data);
                if(reading < 0){
                    printk("%s read ps data failed\n",__func__);
                    return -1;
                }
                if(stk_data->last_ps_canc == 0xffff)
                    reading_sub_canc =  reading;
                else
                    reading_sub_canc = reading - stk_data->last_ps_canc;
                printk("%s, %d \n", __func__, reading);
                if (reading_sub_canc <=0)
                    reading_sub_canc = 0;
                return scnprintf(buf, PAGE_SIZE, "%d\n",reading_sub_canc);
            } else{
                printk("%s ps data can't ready",__func__);
                return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
            }
        }  //end else for if(true == stk_data->pdata->ps_choose_poll)
    }else{
        printk("ps disable  ps data is 0\n");
        return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
    }
}

static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    int32_t reading;
    reading = stk3x3x_get_ps_reading(stk_data);
    if(reading < 0)
        return -1;
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}
/*
static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
    struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    ret = stk3x3x_get_state(ps_data);

    if (ret < 0)
        return ret;

    ret = (ret & STK_STATE_EN_PS_MASK) ? 1 : 0;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    uint8_t en;

    if (sysfs_streq(buf, "1"))
        en = 1;
    else if (sysfs_streq(buf, "0"))
        en = 0;
    else
    {
        printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }

    printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    stk3x3x_enable_ps(ps_data, en, 0);
    mutex_unlock(&ps_data->io_lock);
    return size;
}
*/
static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    ret = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_STATE_REG);
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    //struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    //uint8_t en;
    //int32_t ret;
    //uint8_t w_state_reg;
    //if (sysfs_streq(buf, "1"))
    //    en = 1;
    //else if (sysfs_streq(buf, "0"))
    //    en = 0;
    //else
    //{
    //    printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
    //    return -EINVAL;
    //}
    //printk(KERN_INFO "%s: Enable PS ASO : %d\n", __func__, en);
    //ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    //if (ret < 0)
    //{
    //    printk(KERN_ERR "%s: write i2c error\n", __func__);
    //    return ret;
    //}
    //w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK));
    //if (en)
    //    w_state_reg |= STK_STATE_EN_ASO_MASK;
    //ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    //if (ret < 0)
    //{
    //    printk(KERN_ERR "%s: write i2c error\n", __func__);
    //    return ret;
    //}
    //return size;
    return 0;
}


static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    int32_t word_data;
    unsigned char value[2];
    int ret;
    ret = stk3x3x_i2c_read_data(stk_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);

    if (ret < 0)
    {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data = (value[0] << 8) | value[1];
    return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    unsigned long offset = 0;
    int ret;
    unsigned char val[2];
    ret = kstrtoul(buf, 10, &offset);

    if (ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    if (offset > 65535)
    {
        printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, offset);
        return -EINVAL;
    }

    val[0] = (offset & 0xFF00) >> 8;
    val[1] = offset & 0x00FF;
    ret = stk3x3x_i2c_write_data(stk_data->client, STK_DATA1_OFFSET_REG, 2, val);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    int32_t dist = 1;
    int32_t ret;
    ret = stk3x3x_get_flag(stk_data);

    if (ret < 0)
        return ret;

    dist = (ret & STK_FLG_NF_MASK) ? 1 : 0;
    stk_ps_report(stk_data, dist);
    printk(KERN_INFO "%s: ps input event=%d\n", __func__, dist);
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    unsigned long value = 0;
    int ret;
    ret = kstrtoul(buf, 10, &value);

    if (ret < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_ps_report(stk_data, value);
    printk(KERN_INFO "%s: ps input event=%d\n", __func__, (int)value);
    return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_lt_n_ct);
}

static ssize_t stk_ps_code_thd_l_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
    if( value < 65535 ){
        stk_data->stk_lt_n_ct = value;
        stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct;
    }else{
        dev_err(&stk_data->client->dev,
			"%s(): It is an illegal para, %lu!", __func__, value);
		ret = -EINVAL;
    }
    return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_ht_n_ct);
}

static ssize_t stk_ps_code_thd_h_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
    if( value < 65535 ){
        stk_data->stk_ht_n_ct = value;
        stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct;
    }else{
        dev_err(&stk_data->client->dev,
			"%s(): It is an illegal para, %lu!", __func__, value);
		ret = -EINVAL;
    }
    return size;
}



static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[0x23];
    uint8_t cnt;
    int len = 0;
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    
    for (cnt = 0; cnt < 0x20; cnt++)
    {
        ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, (cnt));

        if (ps_reg[cnt] < 0)
        {
            printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
            return -EINVAL;
        }
        else
        {
            printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
            len += scnprintf(buf + len, PAGE_SIZE - len, "[%2X]%2X,", cnt, ps_reg[cnt]);
        }
    }

    ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);

    if (ps_reg[cnt] < 0)
    {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);

    if (ps_reg[cnt] < 0)
    {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk(KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, 0xE0);

    if (ps_reg[cnt] < 0)
    {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk(KERN_INFO "reg[0xE0]=0x%2X\n", ps_reg[cnt]);
    len += scnprintf(buf + len, PAGE_SIZE - len, "[3E]%2X,[3F]%2X,[E0]%2X\n", ps_reg[cnt - 2], ps_reg[cnt - 1],
                     ps_reg[cnt]);
    return len;
    /*
        return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n",
            ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
            ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
            ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
            */
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[27];
    uint8_t cnt;
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    for (cnt = 0; cnt < 25; cnt++)
    {
        ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, (cnt));

        if (ps_reg[cnt] < 0)
        {
            printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
            return -EINVAL;
        }
        else
        {
            printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
        }
    }

    ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);

    if (ps_reg[cnt] < 0)
    {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);

    if (ps_reg[cnt] < 0)
    {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);
    return scnprintf(buf, PAGE_SIZE,
                     "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n",
                     ps_reg[0] & 0x01, (ps_reg[0] & 0x02) >> 1, ((ps_reg[0] & 0x04) >> 2) * ps_reg[5] * 6, (ps_reg[0] & 0x20) >> 5,
                     (ps_reg[0] & 0x40) >> 6, ps_reg[16] & 0x01, (ps_reg[16] & 0x04) >> 2, (ps_reg[16] & 0x10) >> 4,
                     (ps_reg[16] & 0x20) >> 5);
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk_data->recv_reg));
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
    int ret;
    int32_t recv_data;
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    if ((ret = kstrtoul(buf, 16, &value)) < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    recv_data = stk3x3x_i2c_smbus_read_byte_data(stk_data->client, value);
    //  printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
    atomic_set(&stk_data->recv_reg, recv_data);
    return size;
}


static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return 0;
}


static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int addr, cmd;
    int32_t ret, i;
    char *token[10];
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    if ((ret = kstrtoul(token[0], 16, (unsigned long *) & (addr))) < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    if ((ret = kstrtoul(token[1], 16, (unsigned long *) & (cmd))) < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);
    ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, (unsigned char)addr, (unsigned char)cmd);

    if (0 != ret)
    {
        printk(KERN_ERR "%s: stk3x3x_i2c_smbus_write_byte_data fail\n", __func__);
        return ret;
    }

    return size;
}

#ifdef STK_TUNE0
#if 0
static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data;
    unsigned char value[2];
    int ret;
    ret = stk3x3x_i2c_read_data(ps_data->client, 0x20, 2, &value[0]);

    if (ret < 0)
    {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data = (value[0] << 8) | value[1];
    ret = stk3x3x_i2c_read_data(ps_data->client, 0x22, 2, &value[0]);

    if (ret < 0)
    {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data += ((value[0] << 8) | value[1]);
    printk("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __func__,
           ps_data->psi_set, ps_data->psa, ps_data->psi, word_data);
#ifdef CALI_PS_EVERY_TIME
    //printk("%s: boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
#endif
    return 0;
}
#endif


static ssize_t stk_ps_maxdiff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
            struct stk3x3x_data, ps_cdev);

    unsigned long value = 0;
    int ret;

    if ((ret = kstrtoul(buf, 10, &value)) < 0)
    {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->stk_max_min_diff = (int) value;
    return size;
}


static ssize_t stk_ps_maxdiff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
            struct stk3x3x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_max_min_diff);
}

#endif  /* #ifdef STK_TUNE0 */
static ssize_t stk_ps_canc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->last_ps_canc);
}
static ssize_t stk_ps_crosstalk_max_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->pdata->ps_crosstalk_max);
}

static ssize_t stk_ps_crosstalk_max_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
    struct stk3x3x_data *stk_data = container_of(ps_cdev,
        struct stk3x3x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
    stk_data->pdata->ps_crosstalk_max= value;
    return size;
}


//static struct device_attribute als_enable_attribute = __ATTR(enable, 0664, stk_als_enable_show, stk_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(lux, 0664, stk_als_lux_show, stk_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(code, 0444, stk_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(transmittance, 0664, stk_als_transmittance_show,
        stk_als_transmittance_store);
#ifdef STK_ALS_FIR
    static struct device_attribute als_firlen_attribute = __ATTR(firlen, 0664, stk_als_firlen_show, stk_als_firlen_store);
#endif

static struct attribute *stk_als_attrs [] =
{
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
#ifdef STK_ALS_FIR
    &als_firlen_attribute.attr,
#endif
    NULL
};

static struct attribute_group stk_als_attribute_group =
{
    .attrs = stk_als_attrs,
};

//static struct device_attribute ps_enable_attribute = __ATTR(enable, 0664, stk_ps_enable_show, stk_ps_enable_store);
static struct device_attribute ps_enable_aso_attribute = __ATTR(enableaso, 0664, stk_ps_enable_aso_show,
        stk_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance, 0664, stk_ps_distance_show,
        stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset, 0664, stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, stk_ps_code_show, NULL);
static struct device_attribute ps_data_attribute = __ATTR(ps_data, 0440, stk_ps_data_show, NULL);

static struct device_attribute ps_code_thd_l_attribute = __ATTR(ps_thd_away, 0664, stk_ps_code_thd_l_show,
        stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(ps_thd_close, 0664, stk_ps_code_thd_h_show,
        stk_ps_code_thd_h_store);
static struct device_attribute ps_recv_attribute = __ATTR(recv, 0664, stk_recv_show, stk_recv_store);
static struct device_attribute ps_send_attribute = __ATTR(send, 0664, stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, stk_all_reg_show, NULL);
static struct device_attribute status_attribute =  __ATTR(status, 0444, stk_status_show, NULL);
static struct device_attribute ps_canc_attribute = __ATTR(ps_canc, 0440, stk_ps_canc_show, NULL);
static struct device_attribute ps_crosstalk_maxthd_attribute = __ATTR(ps_crosstalk_maxthd, 0640, stk_ps_crosstalk_max_show, stk_ps_crosstalk_max_store);


#ifdef STK_TUNE0
    
    static struct device_attribute ps_maxdiff_attribute = __ATTR(maxdiff, 0664, stk_ps_maxdiff_show, stk_ps_maxdiff_store);
    
#endif

static struct attribute *stk_ps_attrs [] =
{
    &ps_enable_aso_attribute.attr,
    &ps_distance_attribute.attr,
    &ps_offset_attribute.attr,
    &ps_code_attribute.attr,
    &ps_data_attribute.attr,
    &ps_code_thd_l_attribute.attr,
    &ps_code_thd_h_attribute.attr,
    &ps_recv_attribute.attr,
    &ps_send_attribute.attr,
    &all_reg_attribute.attr,
    &status_attribute.attr,
    &ps_canc_attribute.attr,
    &ps_crosstalk_maxthd_attribute.attr,
#ifdef STK_TUNE0
    &ps_maxdiff_attribute.attr,
#endif
    NULL
};

static struct attribute_group stk_ps_attribute_group =
{
    .attrs = stk_ps_attrs,
};

static void stk3x3x_ps_regs_show_in_log(struct stk3x3x_data *stk_data)
{
    uint8_t ps_reg[20];
	int err,i;

	err = stk3x3x_i2c_read_data(stk_data->client, STK_STATE_REG, 19, ps_reg);
    if(err < 0)
    {
    printk(KERN_INFO "%s: ps regs can not be read\n", __func__);
	return;
    }
    for(i = 0; i < 19; i++)
    {
        pr_buf_err(KERN_ERR "%s: regs_0x%x: %d \n", __func__, i, ps_reg[i]);
        //printk(KERN_INFO "%s: regs_0x%x: %d \n", __func__, i, ps_reg[i]);
    }
}


static int stk_ps_val(struct stk3x3x_data *ps_data)
{
    uint8_t ps_invalid_flag;
    uint8_t bgir_raw_data[4];
    int ret;
    ret = stk3x3x_i2c_read_data(ps_data->client, 0xA7, 1, &ps_invalid_flag);

    if (ret != 0)
    {
        return ret;
    }

    ret = stk3x3x_i2c_read_data(ps_data->client, 0x34, 4, &bgir_raw_data[0]);

    if (ret != 0)
    {
        return ret;
    }
/*
    printk(KERN_INFO
           "%s: ps_invalid_flag=%d, bgir_raw_data[0]=%d, bgir_raw_data[1]=%d, bgir_raw_data[2]=%d, bgir_raw_data[3]=%d\n",
           __func__, ps_invalid_flag, bgir_raw_data[0], bgir_raw_data[1], bgir_raw_data[2], bgir_raw_data[3]);
*/
    if (((ps_invalid_flag >> 5) & 0x1) || ((bgir_raw_data[0] & 0x7f) >= 100) ||
            ((bgir_raw_data[1] & 0x7f) >= 100) || ((bgir_raw_data[2] & 0x7f) >= 100) || ((bgir_raw_data[3] & 0x7f) >= 100))
    {
        return -1;
    }

    return 0;
}

#ifdef STK_TUNE0
static int stk_ps_tune_zero_final(struct stk3x3x_data *stk_data)
{
    int ret;
   
    ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_INT_REG, stk_data->int_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    ret = stk3x3x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, 0);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    if (stk_data->data_count == -1)
    {
        printk(KERN_INFO "%s: exceed limit\n", __func__);
        hrtimer_cancel(&stk_data->ps_tune0_timer);
        return 0;
    }

    stk_data->psa = stk_data->ps_stat_data[0]; // 5 次最大值
    stk_data->psi = stk_data->ps_stat_data[2]; // 5 次最新值
#ifdef CALI_PS_EVERY_TIME
    printk("%s average value %d\n",__func__,stk_data->ps_stat_data[1]);
    if(stk_data->ps_stat_data[1] <  stk_data->pdata->ps_crosstalk_max ){
        stk_data->last_ps_canc = stk_data->ps_stat_data[1];
        //stk_data->ps_high_thd_boot = stk_data->last_ps_canc + stk_data->stk_ht_n_ct ;
        //stk_data->ps_low_thd_boot = stk_data->last_ps_canc + stk_data->stk_lt_n_ct ;
        stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct ;
        stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct ;
    //   开机时采集5次距离值60ms 采集一次，之后600ms采集一次
        stk_data->tune_zero_init_proc = false;
        stk_data->ps_tune0_delay = ns_to_ktime(600 * NSEC_PER_MSEC);
    }else{

        printk("%s, crosstalk is too large and no last canc!!!\n", __func__);
        stk_data->last_ps_canc = 0x00;
        //stk_data->ps_high_thd_boot =  0x0fff; 
        //stk_data->ps_low_thd_boot =  0x00;  
        stk_data->ps_thd_h = 0xffff ;
        stk_data->ps_thd_l = 0x00 ;
    }
#else
    stk_data->ps_thd_h = stk_data->ps_stat_data[1] + stk_data->stk_ht_n_ct;
    stk_data->ps_thd_l = stk_data->ps_stat_data[1] + stk_data->stk_lt_n_ct;
#endif
    ret = stk3x3x_set_ps_thd(stk_data);
    if (ret < 0)
        return ret;
    stk_data->boot_cali = 1;
    printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, stk_data->ps_thd_h,  stk_data->ps_thd_l);
    hrtimer_cancel(&stk_data->ps_tune0_timer);
    return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x3x_data *stk_data)
{
    int32_t ps_adc;
    int ret;
    ret = stk_ps_val(stk_data);

    if (ret == 0xFFFF)
    {
        stk_data->data_count = -1;
        stk_ps_tune_zero_final(stk_data);
        return 0;
    }

    ps_adc = stk3x3x_get_ps_reading(stk_data);
    if(ps_adc < 0)
        return -1;
    printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, stk_data->data_count, ps_adc);


    stk_data->ps_stat_data[1]  +=  ps_adc;

    if (ps_adc > stk_data->ps_stat_data[0])
        stk_data->ps_stat_data[0] = ps_adc;

    if (ps_adc < stk_data->ps_stat_data[2])
        stk_data->ps_stat_data[2] = ps_adc;

    stk_data->data_count++;

    if (stk_data->data_count == 5)
    {
        
        stk_data->ps_stat_data[1]  /= stk_data->data_count;
        stk_ps_tune_zero_final(stk_data);
        stk_data->ps_stat_data[1] = 0;
		stk_data->data_count = 0;
    }

    return 0;
}

static int stk_ps_tune_zero_init(struct stk3x3x_data *ps_data)
{
    int32_t ret = 0;
    uint8_t w_state_reg;
#ifdef CALI_EVERY_TIME
    /*ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
    ps_data->ps_low_thd_boot = ps_data->ps_thd_l;

    if (ps_data->ps_high_thd_boot <= 0)
    {
        ps_data->ps_high_thd_boot = ps_data->stk_ht_n_ct * 3;
        ps_data->ps_low_thd_boot = ps_data->stk_lt_n_ct * 3;
    }*/

#endif
    ps_data->psi_set = 0;
    ps_data->ps_stat_data[0] = 0;
    ps_data->ps_stat_data[2] = 9999;
    ps_data->ps_stat_data[1] = 0;
    ps_data->data_count = 0;
    ps_data->boot_cali = 0;
    ps_data->tune_zero_init_proc = true;
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);

    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);
    return 0;
}

static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *stk_data)
{
#ifdef CTTRACKING
    uint16_t ct_value = 0;
#endif
    int32_t word_data;
    int ret, diff;
    unsigned char value[2];
#ifdef CALI_PS_EVERY_TIME

    if (!(stk_data->ps_enabled))
#else
    if (stk_data->psi_set || !(stk_data->ps_enabled))
#endif
    {
        return 0;
    }

    ret = stk3x3x_get_flag(stk_data);

    if (ret < 0)
        return ret;

    if (!(ret & STK_FLG_PSDR_MASK))
    {
        printk(KERN_INFO "%s: ps data is not ready yet\n", __func__);
        return 0;
    }

    ret = stk_ps_val(stk_data);
    if (ret != 0)
        return ret;

    ret = stk3x3x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
    if (ret < 0)
    {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data = (value[0] << 8) | value[1];
    
#ifdef STK_DEBUG_PRINTF  
    printk(KERN_INFO "%s: word_data=%d\n", __func__, word_data);
#endif/*STK_DEBUG_PRINTF*/

    if (word_data == 0)
    {
        printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
        return 0xFFFF;
    }

#ifdef CTTRACKING
    //Skin
    if (stk_data->psi_set != 0)
    {
        if (near_far_state == 0)//near
        {
            if ((word_data - stk_data->psi) > STK_SKIN_TH)
            {
                atomic_set(&stk_data->ps_thd_h, stk_data->psi + stk_data->stk_ht_n_ct + 300);
                atomic_set(&stk_data->ps_thd_l, stk_data->psi + stk_data->stk_lt_n_ct + 300);
                stk3x3x_set_ps_thd(stk_data);
                stk_data->ps_thd_update = 1;
                printk(KERN_INFO "%s: Set skin thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
            }
        }
        else
        {
            if (ps_thd_update == 1)
            {
                stk_data->psi = word_data;
                atomic_set(&stk_data->ps_thd_h, stk_data->psi + stk_data->stk_ht_n_ct);
                atomic_set(&stk_data->ps_thd_l, stk_data->psi + stk_data->stk_lt_n_ct);
                stk3x3x_set_ps_thd(stk_data);
                stk_data->ps_thd_update = 0;
                printk(KERN_INFO "%s: Reset no skin thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
            }
        }
    }

    //tracking
    if ((stk_data->psi_set != 0))
    {
        if ((stk_data->ps_distance_last == 1))
        {
            ret = stk3x3x_ps_val();
            if (ret == 0)
            {
                if (word_data > 0)
                {
                    ct_value = atomic_read(&stk_data->ps_high_thd_val) - stk_data->stk_ht_n_ct;
                    ret = stk3x3x_read_flag(stk_data->client, &flag);
                    if ((word_data < ct_value) && ((ct_value - word_data) > 2)
                        //&& ((flag&STK_FLG_PSINT_MASK) == 0)
                        && (stk_data->ps_distance_last == 1)
                        && (stk_data->ps_thd_update == false))
                    {
                        stk_data->last_ps_canc = word_data;
                        atomic_set(&stk_data->ps_thd_h, word_data + stk_data->stk_ht_n_ct);
                        atomic_set(&stk_data->ps_thd_l, word_data + stk_data->stk_lt_n_ct);
                        stk3x3x_set_ps_thd(stk_data);
                        
                        APS_LOG("%s: CTTRACKING set HT=%d, LT=%d\n", __func__,
                            atomic_read(&stk_data->ps_thd_h),
                            atomic_read(&stk_data->ps_thd_l));
                        stk_data->psi = word_data;
                        APS_LOG("%s: CTTRACKING update psi=%d\n",
                            __func__, stk_data->psi);                       
                    }
                }
            }
        }
        return 0;
    }
#endif
     if (word_data > stk_data->psa)
     {
         stk_data->psa = word_data;
         printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, stk_data->psa, stk_data->psi);
     }

     if (word_data < stk_data->psi)
     {
         stk_data->psi = word_data;
         printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, stk_data->psa, stk_data->psi);
     }
    
    diff = stk_data->psa - stk_data->psi;

    if (diff > stk_data->stk_max_min_diff)
    {
        stk_data->psi_set = stk_data->psi;
#ifdef CALI_PS_EVERY_TIME

        if (((stk_data->psi + stk_data->stk_ht_n_ct) > (stk_data->ps_thd_h + stk_data->pdata->ps_crosstalk_max )) && (stk_data->ps_thd_h != 0))
        {
            printk(KERN_INFO "%s: no update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
        }
        else
        {
            stk_data->last_ps_canc = stk_data->psi;
            stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
            stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
            printk(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
        }
#else
        stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
        stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
        printk(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
#endif
        ret =stk3x3x_set_ps_thd(stk_data);
        if(ret < 0)
            return ret;
        printk("%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);

#ifndef CTTRACKING
        hrtimer_cancel(&stk_data->ps_tune0_timer);
#endif
    }

    return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
    struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_ps_tune0_work);

    if (stk_data->tune_zero_init_proc)
        stk_tune_zero_get_ps_data(stk_data);
    else
        stk_ps_tune_zero_func_fae(stk_data);

    return;
}

static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
    struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, ps_tune0_timer);
    queue_work(stk_data->stk_ps_tune0_wq, &stk_data->stk_ps_tune0_work);
    hrtimer_forward_now(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay);
    return HRTIMER_RESTART;
}
#endif


static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
    struct stk3x3x_data *stk_data = container_of(timer, struct stk3x3x_data, als_timer);
    queue_work(stk_data->stk_als_wq, &stk_data->stk_als_work);
// 虏脡脩霉脢卤录盲脦陋碌脷脪禄脰隆赂霉戮脻禄媒路脰脢卤录盲脡猫脰脙拢卢脰庐潞贸脡猫脰脙脦陋500ms
    hrtimer_forward_now(&stk_data->als_timer, stk_data->als_poll_delay);
    return HRTIMER_RESTART;
}

static void stk_als_poll_work_func(struct work_struct *work)
{
    struct stk3x3x_data *stk_data = container_of(work, struct stk3x3x_data, stk_als_work);
    int32_t reading = 0, flag_reg;
#ifdef STK_IRS
    int ret;
#endif
    
    flag_reg = stk3x3x_get_flag(stk_data);

    if (flag_reg < 0)
        return;

    if (!(flag_reg & STK_FLG_ALSDR_MASK))
    {
        printk(KERN_INFO "%s: als is not ready\n", __func__);
        return;
    }

#ifdef STK_IRS
    ret = stk_als_ir_skip_als(stk_data);

    if (ret == 1)
        return;

#endif
    reading = stk3x3x_get_als_reading(stk_data);

    if (reading < 0)
        return;

    //printk("%s: als_data_index=%d, als_data=%d\n", __func__, ps_data->als_data_index, reading);
#ifdef STK_IRS
    stk_als_ir_get_corr(stk_data, reading);
    reading = reading * ps_data->als_correct_factor / 1000;
#endif
    //reading_lux = stk_alscode2lux(stk_data, reading);

    
    stk_als_report(stk_data,reading);
    

#ifdef STK_IRS
    stk_als_ir_run(stk_data);
#endif
}


static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
    struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_timer);
    queue_work(ps_data->stk_ps_wq, &ps_data->stk_ps_work);

    hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
    return HRTIMER_RESTART;
}


static void stk_ps_poll_work_func(struct work_struct *work)
{
    struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_work);
    int32_t reading = 0, flag_reg;
    int32_t ps_distance_now;
    
    flag_reg = stk3x3x_get_flag(ps_data);

    if (flag_reg < 0)
        return;

    if (!(flag_reg & STK_FLG_PSDR_MASK))
    {
        printk(KERN_INFO "%s: ps is not ready\n", __func__);
        return;
    }

    ps_distance_now = (flag_reg & STK_FLG_NF_MASK) ? FAR_CODE : NEAR_CODE;// 1 is far and 0 is near
    reading = stk3x3x_get_ps_reading(ps_data);

    if (reading < 0)
        return;

	if (ps_data->ps_distance_last != ps_distance_now)
			{
				
				stk_ps_report(ps_data, ps_distance_now);
				printk(KERN_ERR "%s: ps input event=%d, ps=%d\n", __func__, ps_distance_now, reading);
			}

}


#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
static void stk_ps_int_handle_int_mode_2_3(struct stk3x3x_data *ps_data)
{
    int32_t reading;
    int32_t near_far_state;
#if (STK_INT_PS_MODE    == 0x03)
    near_far_state = gpio_get_value(ps_data->int_pin);
#elif   (STK_INT_PS_MODE == 0x02)
    near_far_state = !(gpio_get_value(ps_data->int_pin));
#endif
    reading = stk3x3x_get_ps_reading(ps_data);
    if(reading < 0)
        return -1;
    stk_ps_report(stk_data, near_far_state);
    printk(KERN_INFO "%s: ps input event=%d, ps code=%d\n", __func__, near_far_state, reading);
}
#endif

static void stk_ps_int_handle(struct stk3x3x_data *ps_data, uint32_t ps_reading, int32_t nf_state)
{
    stk_ps_report(ps_data, nf_state);
    pr_err( "%s: ps input event=%d, ps code=%d\n", __func__, nf_state, ps_reading);
}
/*
static int stk_als_int_handle(struct stk3x3x_data *stk_data, uint32_t als_reading)
{
    int32_t als_comperator;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
    uint32_t nLuxIndex;
#endif
    //int lux;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
    nLuxIndex = stk_get_lux_interval_index(als_reading);
    stk3x3x_set_als_thd_h(stk_data, code_threshold_table[nLuxIndex]);
    stk3x3x_set_als_thd_l(stk_data, code_threshold_table[nLuxIndex - 1]);
#else
    stk_als_set_new_thd(stk_data, als_reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

    if (stk_data->ir_code)
    {
        if (als_reading < STK_IRC_MAX_ALS_CODE && als_reading > STK_IRC_MIN_ALS_CODE &&
                stk_data->ir_code > STK_IRC_MIN_IR_CODE)
        {
            als_comperator = als_reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;

            if (stk_data->ir_code > als_comperator)
                stk_data->als_correct_factor = STK_IRC_ALS_CORREC;
            else
                stk_data->als_correct_factor = 1000;
        }

        // printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_reading, ps_data->ir_code, ps_data->als_correct_factor);
        stk_data->ir_code = 0;
    }

    als_reading = als_reading * stk_data->als_correct_factor / 1000;
    stk_data->als_code_last = als_reading;
    //lux = stk_alscode2lux(stk_data, als_reading);
    stk_als_report(stk_data,als_reading);
    return 0;
}
*/
static void stk_work_func(struct stk3x3x_data *ps_data)
{
    int32_t ret = 0;

    sensors_marker("stk_work_func()",
		"irq", "triggered with ps",
		ps_data->ps_enabled ? "enabled" : "disabled");
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
    stk_ps_int_handle_int_mode_2_3(ps_data);
#else
	ret = stk3x3x_flush_flag_reg(ps_data,
		(STK_FLG_PSINT_MASK | STK_FLG_PSDR_MASK | STK_FLG_INVALIDPS_MASK));
	if (ret < 0) {
        printk(KERN_INFO "%s(): failed to fresh reg FLAG!\n", __func__);
		msleep(30);
	} else {
		if (ps_data->flag_reg & STK_FLG_PSINT_MASK) {
			int32_t reading = stk3x3x_get_ps_reading(ps_data);
			int32_t near_far_state =
				(ps_data->flag_reg & STK_FLG_NF_MASK) ? FAR_CODE : NEAR_CODE;
			ps_data->ps_code_last = reading;
			stk_ps_int_handle(ps_data, reading, near_far_state);
		}
	}
#endif
    mdelay(3);
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
    struct stk3x3x_data *pData = data;
    disable_irq_nosync(irq);
    stk_work_func(pData);
    enable_irq(irq);
    return IRQ_HANDLED;
}


static void stk3x3x_als_set_poll_delay(struct stk3x3x_data *ps_data)
{
    uint8_t als_it = ps_data->alsctrl_reg & 0x0F;

    if (als_it == 0x0)
    {
        ps_data->als_poll_delay = ns_to_ktime(38 * NSEC_PER_MSEC);
    }
    else if (als_it == 0x1)
    {
        ps_data->als_poll_delay = ns_to_ktime(75 * NSEC_PER_MSEC);
    }
    else if (als_it == 0x2)
    {
        ps_data->als_poll_delay = ns_to_ktime(150 * NSEC_PER_MSEC);
    }
    else if (als_it == 0x3)
    {
        ps_data->als_poll_delay = ns_to_ktime(300 * NSEC_PER_MSEC);
    }
    else if (als_it == 0x4)
    {
        ps_data->als_poll_delay = ns_to_ktime(600 * NSEC_PER_MSEC);
    }
    else
    {
        ps_data->als_poll_delay = ns_to_ktime(150 * NSEC_PER_MSEC);
        printk(KERN_INFO "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
    }
}


static int32_t stk3x3x_init_all_setting(struct i2c_client *client, struct stk3x3x_platform_data *plat_data)
{
    int32_t ret;
    struct stk3x3x_data *stk_data = i2c_get_clientdata(client);
    ret = stk3x3x_software_reset(stk_data);

    if (ret < 0)
        return ret;

    stk3x3x_proc_plat_data(stk_data, plat_data);
    ret = stk3x3x_init_all_reg(stk_data);

    if (ret < 0)
        return ret;


    stk3x3x_als_set_poll_delay(stk_data);

    stk_data->als_enabled = false;
    stk_data->ps_enabled = false;
    stk_data->re_enable_als = false;
    stk_data->re_enable_ps = false;
    stk_data->ir_code = 0;
    stk_data->als_correct_factor = 1000;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
    stk_init_code_threshold_table(stk_data);
#endif
#ifdef STK_TUNE0
    stk_ps_tune_zero_init(stk_data);
#endif
#ifdef STK_ALS_FIR
    memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    atomic_set(&stk_data->firlength, STK_FIR_LEN);
#endif
    atomic_set(&stk_data->recv_reg, 0);
#ifdef STK_IRS
    stk_data->als_data_index = 0;
#endif
    stk_data->ps_distance_last = -1;
    stk_data->als_code_last = 100;
    stk_data->last_ps_canc = 0xFFFF;
    stk_data->ps_thd_h = 0xffff ;
    stk_data->ps_thd_l = 0x00 ;
    return 0;
}


static int stk3x3x_setup_irq(struct i2c_client *client)
{
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);
    int err = 0;

    err = gpio_request(ps_data->int_pin, "stk-int");
    if (err < 0) {
        printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
        return err;
    }

    err = gpio_direction_input(ps_data->int_pin);
    if (err < 0) {
        printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
        goto err_request_any_context_irq;
    }

	ps_data->irq = gpio_to_irq(ps_data->int_pin);
#ifdef STK_DEBUG_PRINTF
    printk(KERN_INFO "%s: int pin #=%d, irq=%d\n", __func__, ps_data->int_pin, irq);
#endif
    if (ps_data->irq <= 0) {
        printk(KERN_ERR "irq number is not specified, int pin=%d\n",
			ps_data->int_pin);
        goto err_request_any_context_irq;
    }

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE  == 0x02))
    err = request_any_context_irq(irq,
    					stk_oss_irq_handler,
    					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
    					DEVICE_NAME,
    					ps_data);
#else
	err = request_threaded_irq(ps_data->irq,
						NULL,
						stk_oss_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						DEVICE_NAME,
						ps_data);
#endif

    if (err < 0) {
        printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n",
			__func__, ps_data->irq, err);
        goto err_1;
    }
#ifdef CONFIG_SAVE_AWAKEN_EVENT
	printk(KERN_ERR "%s: register_irqnum_and_event\n", __func__);
    register_irqnum_and_event(ps_data->irq, AWAKEN_EVENT_PSENSOR);
#endif /* CONFIG_SAVE_AWAKEN_EVENT */
   
    disable_irq(ps_data->irq);
    return 0;
err_1:
	free_irq(ps_data->irq, ps_data);
err_request_any_context_irq:
    gpio_free(ps_data->int_pin);
    return err;
}


static int stk3x3x_suspend(struct device *dev)
{
    struct stk3x3x_data *stk_data = dev_get_drvdata(dev);

    int err;


    struct i2c_client *client = to_i2c_client(dev);

    sensors_marker("stk3x3x_suspend()",
		"do", "suspend", "done");
    mutex_lock(&stk_data->io_lock);
#ifdef STK_CHK_REG
    err = stk3x3x_validate_n_handle(stk_data->client);

    if (err < 0)
    {
        printk(KERN_ERR "stk3x3x_validate_n_handle fail: %d\n", err);
    }
    else if (err == 0xFF)
    {
        if (stk_data->ps_enabled)
            stk3x3x_enable_ps(stk_data, 1, 0);
    }

#endif /* #ifdef STK_CHK_REG */

    if (stk_data->als_enabled)
    {
        printk(KERN_INFO "%s: Enable ALS : 0\n", __func__);
        stk3x3x_enable_als(stk_data, 0);
        stk_data->re_enable_als = true;
    }

    if (stk_data->ps_enabled && (false == stk_data->pdata->ps_choose_poll))
    {
        if (device_may_wakeup(&client->dev))
        {
            err = enable_irq_wake(stk_data->irq);

            if (err)
                printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, stk_data->irq, err);
        }
        else
        {
            printk(KERN_ERR "%s: not support wakeup source\n", __func__);
        }


    }
   
    //disable_irq_nosync(stk_data->client->irq);
    mutex_unlock(&stk_data->io_lock);
    return 0;
}

static int stk3x3x_resume(struct device *dev)
{
    struct stk3x3x_data *stk_data = dev_get_drvdata(dev);

    int err;


    struct i2c_client *client = to_i2c_client(dev);

    sensors_marker("stk3x3x_resume()",
		"do", "resume", "done");
    mutex_lock(&stk_data->io_lock);
#ifdef STK_CHK_REG
    err = stk3x3x_validate_n_handle(stk_data->client);

    if (err < 0)
    {
        printk(KERN_ERR "stk3x3x_validate_n_handle fail: %d\n", err);
    }
    else if (err == 0xFF)
    {
        if (stk_data->ps_enabled)
            stk3x3x_enable_ps(stk_data, 1, 0);
    }

#endif /* #ifdef STK_CHK_REG */

    if (stk_data->re_enable_als)
    {
        printk(KERN_INFO "%s: Enable ALS : 1\n", __func__);
        stk3x3x_enable_als(stk_data, 1);
        stk_data->re_enable_als = false;
    }

    if (stk_data->ps_enabled)
    {


        if (device_may_wakeup(&client->dev))
        {
            err = disable_irq_wake(stk_data->irq);

            if (err)
                printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, stk_data->irq, err);
        }


    }
    
    //enable_irq(stk_data->client->irq);
    mutex_unlock(&stk_data->io_lock);
    return 0;
}

static const struct dev_pm_ops stk3x3x_pm_ops =
{
    SET_SYSTEM_SLEEP_PM_OPS(stk3x3x_suspend, stk3x3x_resume)
};

static int stk3x3x_power_ctl(struct stk3x3x_data *data, bool on)
{
    int ret = 0;

    if (!on && data->power_enabled) {
        ret = regulator_disable(data->vdd);
        if (ret) {
            pr_err("Regulator vdd disable failed ret=%d\n", ret);
            return ret;
        }

        ret = regulator_disable(data->vio);
        if (ret) {
            pr_err("Regulator vio disable failed ret=%d\n", ret);
            ret = regulator_enable(data->vdd);
            if (ret) {
                pr_err("Regulator vdd enable failed ret=%d\n", ret);
            }

            return ret;
        }

        pr_info("%s: disable stk3x3x power", __func__);
        pr_info("stk3x3x_power_ctl on=%d\n", on);
    } else if (on && !data->power_enabled) {
        ret = regulator_enable(data->vdd);
        if (ret) {
            pr_err("Regulator vdd enable failed ret=%d\n", ret);
            return ret;
        }

        ret = regulator_enable(data->vio);
        if (ret) {
            pr_err("Regulator vio enable failed ret=%d\n", ret);
            regulator_disable(data->vdd);
            return ret;
        }
        msleep(3);
        pr_info("%s: enable stk3x3x power", __func__);
        pr_info("stk3x3x_power_ctl on=%d\n", on);
    } else {
        pr_info("invalid!!! Power on=%d. enabled=%d\n",
            on, data->power_enabled);
    }
    data->power_enabled = on;

    return ret;
}

static int stk3x3x_power_init(struct stk3x3x_data *data, bool on)
{
    int ret;

    if (on) {
        data->vdd = regulator_get(&data->client->dev, "vdd");
        if (IS_ERR(data->vdd)) {
            ret = PTR_ERR(data->vdd);
            pr_err("Regulator get failed vdd ret=%d\n", ret);
            return ret;
        }

        if (regulator_count_voltages(data->vdd) > 0) {
                ret = regulator_set_voltage(data->vdd,
                        STK3X3X_VDD_MIN_UV,STK3X3X_VDD_MAX_UV);
            if (ret) {
                pr_err("Regulator set failed vdd ret=%d\n", ret);
                goto reg_vdd_put;
            }
        }

        data->vio = regulator_get(&data->client->dev, "vio");
        if (IS_ERR(data->vio)) {
            ret = PTR_ERR(data->vio);
            pr_err("Regulator get failed vio ret=%d\n", ret);
            goto reg_vdd_set;
        }

        if (regulator_count_voltages(data->vio) > 0) {
            ret = regulator_set_voltage(data->vio,
                                        STK3X3X_VIO_MIN_UV,
                                        STK3X3X_VIO_MAX_UV);
            if (ret) {
                pr_err("Regulator set failed vio ret=%d\n", ret);
                goto reg_vio_put;
            }
        }
    } else {
        if (regulator_count_voltages(data->vdd) > 0)
            regulator_set_voltage(data->vdd, 0, STK3X3X_VDD_MAX_UV);
        regulator_put(data->vdd);
        
        if (regulator_count_voltages(data->vio) > 0)
            regulator_set_voltage(data->vio, 0, STK3X3X_VIO_MAX_UV);
        regulator_put(data->vio);
    }

    return 0;
reg_vio_put:
    regulator_put(data->vio);
reg_vdd_set:

    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, STK3X3X_VDD_MAX_UV);

reg_vdd_put:
    regulator_put(data->vdd);
    return ret;
}

#ifdef CONFIG_OF
static int stk3x3x_parse_dt(struct device *dev,
                            struct stk3x3x_platform_data *pdata)
{
#if 0
    pdata->int_pin = 65;
    pdata->transmittance = 500;
    pdata->state_reg = 0;
    pdata->psctrl_reg = 0x72;
    pdata->alsctrl_reg = 0x31;
    pdata->ledctrl_reg = 0x60;
    pdata->wait_reg = 0x20;
    pdata->ps_thd_h = 1700;
    pdata->ps_thd_l = 1500;
#else
    int rc;
    struct device_node *np = dev->of_node;
    u32 temp_val;
    pdata->int_pin = of_get_named_gpio_flags(np, "stk,irq-gpio",
                     0, &pdata->int_flags);

    if (pdata->int_pin < 0)
    {
        dev_err(dev, "Unable to read irq-gpio\n");
        return pdata->int_pin;
    }

    rc = of_property_read_u32(np, "stk,als_root", &temp_val);
    if (!rc)
        pdata->als_root = temp_val;
    else {
        dev_err(dev, "Unable to read als_root\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,als_numerator", &temp_val);
    if (!rc)
        pdata->als_numerator = temp_val;
    else {
        dev_err(dev, "Unable to read als_numerator\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,als_transmittance", &temp_val);

    if (!rc)
        pdata->transmittance = temp_val;
    else
    {
        dev_err(dev, "Unable to read transmittance\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,state-reg", &temp_val);

    if (!rc)
        pdata->state_reg = temp_val;
    else
    {
        dev_err(dev, "Unable to read state-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);

    if (!rc)
        pdata->psctrl_reg = (u8)temp_val;
    else
    {
        dev_err(dev, "Unable to read psctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);

    if (!rc)
        pdata->alsctrl_reg = (u8)temp_val;
    else
    {
        dev_err(dev, "Unable to read alsctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);

    if (!rc)
        pdata->ledctrl_reg = (u8)temp_val;
    else
    {
        dev_err(dev, "Unable to read ledctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);

    if (!rc)
        pdata->wait_reg = (u8)temp_val;
    else
    {
        dev_err(dev, "Unable to read wait-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);

    if (!rc)
        pdata->ps_thd_h = (u16)temp_val;
    else
    {
        dev_err(dev, "Unable to read ps-thdh\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);

    if (!rc)
        pdata->ps_thd_l = (u16)temp_val;
    else
    {
        dev_err(dev, "Unable to read ps-thdl\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ps_crosstalk_max", &temp_val);
    if (!rc)
        pdata->ps_crosstalk_max= (u16)temp_val;
    else {
        dev_err(dev, "Unable to read ps-thdl\n");
        return rc;
    }

    //add for deciding that using interrupt or poll of ps by wangdong24
    rc = of_property_read_u32(np, "stk,ps_choose_poll", &temp_val);
    if (!rc)
        pdata->ps_choose_poll= (u16)temp_val;
    else {
        dev_err(dev, "Unable to read ps-choose-poll\n");
        return rc;
    }

    pdata->use_fir = of_property_read_bool(np, "stk,use-fir");
#endif
    return 0;
}
#else
static int stk3x3x_parse_dt(struct device *dev,
                            struct stk3x3x_platform_data *pdata)
{
    return -ENODEV;
}
#endif /* !CONFIG_OF */


static int stk3x3x_set_wq(struct stk3x3x_data *ps_data)
{

    ps_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
    INIT_WORK(&ps_data->stk_als_work, stk_als_poll_work_func);
    hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
    ps_data->als_timer.function = stk_als_timer_func;


#ifdef STK_TUNE0
    ps_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
    INIT_WORK(&ps_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
    hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ps_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
    ps_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

    if(true == ps_data->pdata->ps_choose_poll){
        ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
        INIT_WORK(&ps_data->stk_ps_work, stk_ps_poll_work_func);
        hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ps_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
        ps_data->ps_timer.function = stk_ps_timer_func;
    }else {
        ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
        //INIT_WORK(&ps_data->stk_work, stk_work_func);
    }
	
    return 0;
}


static int stk3x3x_set_input_devices(struct stk3x3x_data *  stk_data)
{
    int err;
    stk_data->als_input_dev = input_allocate_device();

    if (stk_data->als_input_dev == NULL)
    {
        printk(KERN_ERR "%s: could not allocate als device\n", __func__);
        err = -ENOMEM;
        return err;
    }

    stk_data->als_input_dev->name = ALS_NAME;
    set_bit(EV_ABS, stk_data->als_input_dev->evbit);
    input_set_abs_params(stk_data->als_input_dev, ABS_MISC, 0, (1 << 16) - 1, 0, 0);
    err = input_register_device(stk_data->als_input_dev);

    if (err < 0)
    {
        printk(KERN_ERR "%s: can not register als input device\n", __func__);
        goto err_als_input_register;
    }
    input_set_drvdata(stk_data->als_input_dev, stk_data);
    
    stk_data->ps_input_dev = input_allocate_device();

    if (stk_data->ps_input_dev == NULL)
    {
        printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
        err = -ENOMEM;
        goto err_ps_input_allocate;
    }

    stk_data->ps_input_dev->name = PS_NAME;
    set_bit(EV_ABS, stk_data->ps_input_dev->evbit);
    input_set_abs_params(stk_data->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    err = input_register_device(stk_data->ps_input_dev);

    if (err < 0)
    {
        printk(KERN_ERR "%s: can not register ps input device\n", __func__);
        goto err_ps_input_register;
    }
    input_set_drvdata(stk_data->ps_input_dev, stk_data);
    return 0;
       
err_ps_input_register:
    input_unregister_device(stk_data->ps_input_dev);

err_ps_input_allocate:
    
    input_free_device(stk_data->als_input_dev);
err_als_input_register:
    
    input_unregister_device(stk_data->als_input_dev);
    return err;
}

static int stk3x3x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int err = -ENODEV;
    int i =0;
    struct stk3x3x_data *stk_data;
    struct stk3x3x_platform_data *plat_data ={0} ;
	
   	printk(KERN_ERR "%s: probe()!", __func__);
    printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
        return -ENODEV;
    }

    stk_data = kzalloc(sizeof(struct stk3x3x_data), GFP_KERNEL);

    if (!stk_data)
    {
        printk(KERN_ERR "%s: failed to allocate stk3x3x_data\n", __func__);
        return -ENOMEM;
    }

    stk_data->client = client;
    i2c_set_clientdata(client, stk_data);
    mutex_init(&stk_data->io_lock);
    //wake_lock_init(&stk_data->ps_wakelock, WAKE_LOCK_SUSPEND, "stk_input_wakelock");
    wakeup_source_init(&(stk_data->wake_src), "stk_input_wakelock");


#ifdef CTTRACKING
    obj->ps_thd_update = false;
#endif

    if (client->dev.of_node)
    {
        printk(KERN_INFO "%s: probe with device tree\n", __func__);
        plat_data = devm_kzalloc(&client->dev,
                                 sizeof(struct stk3x3x_platform_data), GFP_KERNEL);

        if (!plat_data)
        {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }

        err = stk3x3x_parse_dt(&client->dev, plat_data);

        if (err)
        {
            dev_err(&client->dev,
                    "%s: stk3x3x_parse_dt ret=%d\n", __func__, err);
            return err;
        }
    }
    else
    {
        printk(KERN_INFO "%s: probe with platform data\n", __func__);
        stk_data = client->dev.platform_data;
    }

    if (!plat_data)
    {
        dev_err(&client->dev,
                "%s: no stk3x3x platform data!\n", __func__);
        goto err_als_input_allocate;
    }

    stk_data->als_transmittance = plat_data->transmittance;
    stk_data->int_pin = plat_data->int_pin;
    stk_data->pdata = plat_data;
    

    if (stk_data->als_transmittance == 0)
    {
        dev_err(&client->dev,
                "%s: Please set als_transmittance\n", __func__);
        goto err_als_input_allocate;
    }
    err = stk3x3x_power_init(stk_data, true);
    if (err)
        goto err_als_input_allocate;
    
    err = stk3x3x_power_ctl(stk_data, true);
    if (err)
        goto err_power_init;

    err = stk3x3x_check_pid(stk_data);
    if (err < 0)
        goto err_power_on;

    stk3x3x_set_wq(stk_data);
    stk_data->stk_lt_n_ct = plat_data->ps_thd_l;
    stk_data->stk_ht_n_ct = plat_data->ps_thd_h;
#ifdef STK_TUNE0
    stk_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
    stk_data->stk_lt_n_ct = plat_data->ps_thd_l;
    stk_data->stk_ht_n_ct = plat_data->ps_thd_h;
#endif

    stk_data->als_enabled = false;
    stk_data->ps_enabled = false;
    err = stk3x3x_init_all_setting(client, plat_data);

    if (err < 0)
        goto err_init_all_setting;

    err = stk3x3x_set_input_devices(stk_data);

    if (err < 0)
        goto err_setup_input_device;

    //false for using interrupt of ps
    if(false == plat_data->ps_choose_poll){
        err = stk3x3x_setup_irq(client);
        if (err < 0)
            goto err_setup_input_device;
    }

    /* make sure everything is ok before registering the class device */
    stk_data->als_cdev = stk_light_cdev;

    stk_light_cdev.min_delay = ktime_to_us(stk_data->als_poll_delay);

    stk_data->als_cdev.sensors_enable = als_sensors_enable;
    err = sensors_classdev_register(&stk_data->als_input_dev->dev,
        &stk_data->als_cdev);
    if (err)
        goto err_setup_irq;
    err = sysfs_create_group(&((stk_data->als_cdev.dev)->kobj),
        &stk_als_attribute_group);
    if (err < 0) {
        pr_err("%s:could not create sysfs group for als\n", __func__);
        goto err_als_cdev_regi;
    }

    stk_data->ps_cdev = stk_prox_cdev;
    stk_data->ps_cdev.sensors_enable = ps_sensors_enable;
    stk_data->ps_cdev.sensors_flush = stk3x3x_ps_flush;
    err = sensors_classdev_register(&stk_data->ps_input_dev->dev,
        &stk_data->ps_cdev);
    if (err)
        goto err_als_class_sysfs;
    err = sysfs_create_group(&((stk_data->ps_cdev.dev)->kobj),
        &stk_ps_attribute_group);
    if (err < 0) {
        pr_err("%s:could not create sysfs group for ps\n", __func__);
        goto err_ps_cdev_regi;
    }

    device_init_wakeup(&client->dev, true);
    printk(KERN_INFO "%s: probe successfully", __func__);
    for(i=0;i<ARRAY_SIZE(device_addr);i++){
        if(stk_data->client->addr == device_addr[i]){
    	    productinfo_register(PRODUCTINFO_SENSOR_ALSPS_ID,
    		    alps_device_info[i][0],alps_device_info[i][1]);
            break;
        }
    }
    return 0;
    //device_init_wakeup(&client->dev, false);


err_ps_cdev_regi:
    sensors_classdev_unregister(&stk_data->ps_cdev);    
err_als_class_sysfs:
    sysfs_remove_group(&((stk_data->als_cdev.dev)->kobj), &stk_als_attribute_group);
err_als_cdev_regi:
    sensors_classdev_unregister(&stk_data->als_cdev);


err_setup_irq:

	if(false == plat_data->ps_choose_poll){
        free_irq(stk_data->irq, stk_data);
        gpio_free(stk_data->int_pin);
	}

err_setup_input_device:
err_init_all_setting:

    hrtimer_try_to_cancel(&stk_data->als_timer);
    destroy_workqueue(stk_data->stk_als_wq);

#ifdef STK_TUNE0
    destroy_workqueue(stk_data->stk_ps_tune0_wq);
#endif

    if(true == plat_data->ps_choose_poll){
        destroy_workqueue(stk_data->stk_ps_wq);
    }else {
        destroy_workqueue(stk_data->stk_wq);
    }

err_power_on:
    stk3x3x_power_ctl(stk_data, false);
err_power_init:
    stk3x3x_power_init(stk_data, false);

err_als_input_allocate:

   // wake_lock_destroy(&stk_data->ps_wakelock);
    wakeup_source_trash(&(stk_data->wake_src));
    mutex_destroy(&stk_data->io_lock);
    kfree(stk_data);
    return err;
}


static int stk3x3x_remove(struct i2c_client *client)
{
    struct stk3x3x_data *stk_data = i2c_get_clientdata(client);
    device_init_wakeup(&client->dev, false);

    free_irq(stk_data->irq, stk_data);
    gpio_free(stk_data->int_pin);

    sysfs_remove_group(&stk_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
    sysfs_remove_group(&stk_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
    input_unregister_device(stk_data->ps_input_dev);
    input_unregister_device(stk_data->als_input_dev);

    hrtimer_try_to_cancel(&stk_data->als_timer);
    destroy_workqueue(stk_data->stk_als_wq);

#ifdef STK_TUNE0
    destroy_workqueue(stk_data->stk_ps_tune0_wq);
#endif

    if(true == stk_data->pdata->ps_choose_poll){
		hrtimer_try_to_cancel(&stk_data->ps_timer);
		destroy_workqueue(stk_data->stk_ps_wq);
    }

    //wake_lock_destroy(&ps_data->ps_wakelock);
    wakeup_source_trash(&(stk_data->wake_src));
    mutex_destroy(&stk_data->io_lock);
    kfree(stk_data);
    return 0;
}

static const struct i2c_device_id stk_ps_id[] =
{
    { "stk_ps", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] =
{
    { .compatible = "stk,stk3x3x", },
    { },
};

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = stk_match_table,
#endif
        .pm = &stk3x3x_pm_ops,
    },
    .probe = stk3x3x_probe,
    .remove = stk3x3x_remove,
    .id_table = stk_ps_id,
};


static int __init stk3x3x_init(void)
{
    int ret;
    ret = i2c_add_driver(&stk_ps_driver);

    if (ret)
    {
        i2c_del_driver(&stk_ps_driver);
        return ret;
    }

    return 0;
}

static void __exit stk3x3x_exit(void)
{
    i2c_del_driver(&stk_ps_driver);
}

module_init(stk3x3x_init);
module_exit(stk3x3x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x3x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
