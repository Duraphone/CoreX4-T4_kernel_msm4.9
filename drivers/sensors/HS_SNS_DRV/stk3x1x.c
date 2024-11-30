/*
 *  stk3x1x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x
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
#define pr_fmt(fmt) "stk3x1x: " fmt

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
#include <linux/productinfo.h>


#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include "stk3x1x.h"


#define DRIVER_VERSION  "3.12.2"

/* Driver Settings */
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

/* The threshold to trigger ALS interrupt, unit: lux */
#define STK_ALS_CHANGE_THD  1

/* 1, 2, or 3	*/
#define STK_INT_PS_MODE     1
/* ALS interrupt is valid only when STK_INT_PS_MODE = 1 or 4*/
//#define STK_POLL_ALS
#define CTTRACKING
#define STK_TUNE0
#define CALI_PS_EVERY_TIME
#define STK_DEBUG_PRINTF
//#define STK_ALS_FIR
//#define QUALCOMM_PLATFORM


#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#define STK_QUALCOMM_POWER_CTRL


#define NEAR_CODE	1
#define FAR_CODE	0

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
#define STK_DATA1_ALS_REG       0x13
#define STK_DATA2_ALS_REG       0x14
#define STK_DATA1_OFFSET_REG    0x15
#define STK_DATA2_OFFSET_REG    0x16
#define STK_DATA1_IR_REG        0x17
#define STK_DATA2_IR_REG        0x18
#define STK_PDT_ID_REG          0x3E
#define STK_RSRVD_REG           0x3F
#define STK_SW_RESET_REG        0x80

#define STK_GSCTRL_REG          0x1A
#define STK_FLAG2_REG           0x1C

/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT      7
#define STK_STATE_EN_AK_SHIFT       6
#define STK_STATE_EN_ASO_SHIFT      5
#define STK_STATE_EN_IRO_SHIFT      4
#define STK_STATE_EN_WAIT_SHIFT     2
#define STK_STATE_EN_ALS_SHIFT      1
#define STK_STATE_EN_PS_SHIFT       0

#define STK_STATE_EN_IRS_MASK   0x80
#define STK_STATE_EN_AK_MASK    0x40
#define STK_STATE_EN_ASO_MASK   0x20
#define STK_STATE_EN_IRO_MASK   0x10
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
#define STK_LED_IRDR_SHIFT      6
#define STK_LED_DT_SHIFT        0

#define STK_LED_IRDR_MASK       0xC0
#define STK_LED_DT_MASK         0x3F

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT      7
#define STK_INT_OUI_SHIFT       4
#define STK_INT_ALS_SHIFT       3
#define STK_INT_PS_SHIFT        0

#define STK_INT_CTRL_MASK       0x80
#define STK_INT_OUI_MASK        0x10
#define STK_INT_ALS_MASK        0x08
#define STK_INT_PS_MASK         0x07

#define STK_INT_ALS             0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT         7
#define STK_FLG_PSDR_SHIFT          6
#define STK_FLG_ALSINT_SHIFT        5
#define STK_FLG_PSINT_SHIFT         4
#define STK_FLG_OUI_SHIFT       2
#define STK_FLG_IR_RDY_SHIFT        1
#define STK_FLG_NF_SHIFT        0

#define STK_FLG_ALSDR_MASK      0x80
#define STK_FLG_PSDR_MASK       0x40
#define STK_FLG_ALSINT_MASK     0x20
#define STK_FLG_PSINT_MASK      0x10
#define STK_FLG_OUI_MASK        0x04
#define STK_FLG_IR_RDY_MASK     0x02
#define STK_FLG_NF_MASK         0x01

/* Define flag2 reg */
#define STK_FLG2_INT_GS_SHIFT   6
#define STK_FLG2_GS10_SHIFT     5
#define STK_FLG2_GS01_SHIFT     4

#define STK_FLG2_INT_GS_MASK    0x40
#define STK_FLG2_GS10_MASK      0x20
#define STK_FLG2_GS01_MASK      0x10

/* misc define */
#define MIN_ALS_POLL_DELAY_NS   60000000

#ifdef STK_TUNE0    //dts
    #define STK_MAX_MIN_DIFF    80
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
#ifdef CTTRACKING

#define PS_GESTURE_DIRTY    100
#endif

#define DEVICE_NAME "stk_ps"

#define ALS_NAME        "light"
#define PS_NAME     "proximity"

#ifdef STK_QUALCOMM_POWER_CTRL
    /* POWER SUPPLY VOLTAGE RANGE */
    #define STK3X1X_VDD_MIN_UV  2750000
    #define STK3X1X_VDD_MAX_UV  2950000
    #define STK3X1X_VIO_MIN_UV  1750000
    #define STK3X1X_VIO_MAX_UV  1950000
#endif
#define STK3310SA_PID       0x17
#define STK3311SA_PID       0x1E
#define STK3311WV_PID       0x1D
#define STK3311X_PID        0x12
#define STK33119_PID        0x11

static uint8_t confuse_i = 0;


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
    .max_range = "5.0",
    .resolution = "5.0",
    .sensor_power = "0.1",
    .min_delay = 0,
    .max_delay = 0,
    .delay_msec = 200,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .flags = 3,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
    .sensors_calibrate = NULL,
    .sensors_write_cal_params = NULL,
    .params = NULL,
};


#ifdef STK_ALS_FIR
#define STK_FIR_LEN 4
#define MAX_FIR_LEN 32

struct data_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif

struct stk3x1x_data {
    struct i2c_client *client;
    struct stk3x1x_platform_data *pdata;

    struct sensors_classdev als_cdev;
    struct sensors_classdev ps_cdev;

    int32_t irq;
    struct work_struct stk_work;
    struct workqueue_struct *stk_wq;
    uint16_t ir_code;
    uint16_t als_correct_factor;
    uint8_t alsctrl_reg;
    uint8_t psctrl_reg;
    uint8_t ledctrl_reg;
    uint8_t state_reg;
    int     int_pin;
    uint8_t wait_reg;
    uint8_t int_reg;
    uint16_t ps_thd_h;
    uint16_t ps_thd_l;
	uint16_t ps_factory_flag;
	bool first_als;
	atomic_t als_refresh_flag;
#ifdef CTTRACKING
	uint16_t  ps_greasy_dirty;
	bool ps_thd_update;
#endif
	
    struct mutex io_lock;
    struct input_dev *ps_input_dev;
	int32_t ps_distance_now;
    bool ps_enabled;
    bool re_enable_ps;
    //struct wake_lock ps_wakelock;
    struct wakeup_source wake_src;
    struct input_dev *als_input_dev;
    int32_t als_lux_last;
    //uint32_t als_transmittance;
    bool als_enabled;
    bool re_enable_als;
    ktime_t ps_poll_delay;
    uint64_t als_poll_delay;
    int als_root;
    int als_numerator;
    int als_fittness;

    struct work_struct stk_als_work;
    struct hrtimer als_timer;
    struct workqueue_struct *stk_als_wq;

  	
#ifdef STK_TUNE0
    uint16_t psa;
    uint16_t psi;
    uint16_t psi_set;
	uint16_t ps_crosstalk_max;
	uint16_t last_ps_canc;
	uint16_t boot_ps_canc; //每次ps使能时使用的ps canc
    struct hrtimer ps_tune0_timer;
    struct workqueue_struct *stk_ps_tune0_wq;
    struct work_struct stk_ps_tune0_work;
    ktime_t ps_tune0_delay;
	ktime_t als_work_delay;
    bool tune_zero_init_proc;
    uint32_t ps_stat_data[3];
    int data_count;
    int stk_max_min_diff;
    uint16_t stk_lt_n_ct;
    uint16_t stk_ht_n_ct;
#endif
#ifdef STK_ALS_FIR
    struct data_filter      fir;
    atomic_t                firlength;
#endif
    atomic_t    recv_reg;

#ifdef STK_QUALCOMM_POWER_CTRL
    struct regulator *vdd;
    struct regulator *vio;
    bool power_enabled;
#endif
    uint8_t pid;
    uint8_t p_wv_r_bd_with_co;
    uint32_t als_code_last;
    bool als_en_hal;
    uint8_t boot_cali;
    uint8_t p_1x_r_bd_with_co;
    uint8_t p_19_r_bc;
    bool als_take_effect;
    bool ps_take_effect;
	
};

#if (!defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] = {
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

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table) / sizeof(uint32_t) + 1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE + 1];
#endif

static int stk_ps_tune_zero_init(struct stk3x1x_data *stk_data);

static int stk3x1x_i2c_read_data(struct i2c_client *client,
    unsigned char command, int length,unsigned char *values)
{
    uint8_t retry;
    int err;
    struct i2c_msg msgs[] = {
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

    for (retry = 0; retry < 5; retry++) {
        err = i2c_transfer(client->adapter, msgs, 2);
        if (err == 2)
            break;
        else
            mdelay(5);
    }

    if (retry >= 5) {
        printk(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
        return -EIO;
    }

    return 0;
}

static int stk3x1x_i2c_write_data(struct i2c_client *client,
    unsigned char command, int length,unsigned char *values)
{
    int retry;
    int err;
    unsigned char data[11];
    struct i2c_msg msg;
    int index;

    if (!client) {
        return -EINVAL;
    } else if (length >= 10) {
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

    for (retry = 0; retry < 5; retry++) {
        err = i2c_transfer(client->adapter, &msg, 1);

        if (err == 1)
            break;
        else
            mdelay(5);
    }

    if (retry >= 5) {
        printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);
        return -EIO;
    }

    return 0;
}

static int stk3x1x_i2c_smbus_read_byte_data(struct i2c_client *client,
    unsigned char command)
{
    unsigned char value;
    int err;
    err = stk3x1x_i2c_read_data(client, command, 1, &value);
    if (err < 0)
        return err;

    return value;
}

static int stk3x1x_i2c_smbus_write_byte_data(struct i2c_client *client,
    unsigned char command, unsigned char value)
{
    return stk3x1x_i2c_write_data(client, command, 1, &value);
}
/*
uint32_t stk_alscode2lux(struct stk3x1x_data *stk_data, uint32_t alscode)
{
    alscode += ((alscode << 7) + (alscode << 3) + (alscode >> 1));
    alscode <<= 3;
    alscode /= stk_data->pdata->als_transmittance;
    return alscode;
}
*/
uint32_t stk_lux2alscode(struct stk3x1x_data *stk_data, uint32_t lux)
{
    lux *= stk_data->pdata->als_transmittance;
    lux /= 1100;

    if (unlikely(lux >= (1 << 16)))
        lux = (1 << 16) - 1;

    return lux;
}

static int32_t stk3x1x_set_als_thd_l(struct stk3x1x_data *stk_data,
    uint16_t thd_l)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_l & 0xFF00) >> 8;
    val[1] = thd_l & 0x00FF;
    ret = stk3x1x_i2c_write_data(stk_data->client, STK_THDL1_ALS_REG, 2, val);
    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
static int32_t stk3x1x_set_als_thd_h(struct stk3x1x_data *stk_data,
    uint16_t thd_h)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_h & 0xFF00) >> 8;
    val[1] = thd_h & 0x00FF;
    ret = stk3x1x_i2c_write_data(stk_data->client, STK_THDH1_ALS_REG, 2, val);
    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}


static int32_t stk3x1x_set_ps_thd_l(struct stk3x1x_data *stk_data, uint16_t thd_l)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_l & 0xFF00) >> 8;
    val[1] = thd_l & 0x00FF;
    ret = stk3x1x_i2c_write_data(stk_data->client, STK_THDL1_PS_REG, 2, val);
    if (ret < 0)
        pr_err("%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
static int32_t stk3x1x_set_ps_thd_h(struct stk3x1x_data *stk_data, uint16_t thd_h)
{
    unsigned char val[2];
    int ret;
    val[0] = (thd_h & 0xFF00) >> 8;
    val[1] = thd_h & 0x00FF;
    ret = stk3x1x_i2c_write_data(stk_data->client, STK_THDH1_PS_REG, 2, val);
    if (ret < 0)
        pr_err("%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
static int32_t stk3x1x_set_ps_thd(struct stk3x1x_data *stk_data)
{
    int err =0;

    err = stk3x1x_set_ps_thd_h(stk_data, stk_data->ps_thd_h);
    if (err < 0){
        printk(KERN_ERR "%s: set thd high fail, err=%d\n", __func__, err);
    return err;
    }
    
    err =stk3x1x_set_ps_thd_l(stk_data, stk_data->ps_thd_l);
    if (err < 0)
        printk(KERN_ERR "%s: set thd low fail, err=%d\n", __func__, err);
    return err;
}
static int stk_ps_val(struct stk3x1x_data *stk_data)
{
    int mode;
    int32_t word_data, lii;
    unsigned char value[4];
    int ret;
    ret = stk3x1x_i2c_read_data(stk_data->client, 0x20, 4, value);
    if (ret < 0) {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data = (value[0] << 8) | value[1];
    word_data += ((value[2] << 8) | value[3]);
    mode = (stk_data->psctrl_reg) & 0x3F;

    if (mode == 0x30)
        lii = 100;
    else if (mode == 0x31)
        lii = 200;
    else if (mode == 0x32)
        lii = 400;
    else if (mode == 0x33)
        lii = 800;
    else {
        printk(KERN_ERR "%s: unsupported PS_IT(0x%x)\n", __func__, mode);
        return -1;
    }

    if (word_data > lii) {
        printk(KERN_INFO "%s: word_data=%d, lii=%d\n", __func__, word_data, lii);
        return 0xFFFF;
    }

    return 0;
}

static uint16_t stk3x1x_get_ps_reading(struct stk3x1x_data *stk_data)
{
    unsigned char value[2];
    int err;
    err = stk3x1x_i2c_read_data(stk_data->client, STK_DATA1_PS_REG, 2, &value[0]);
    if (err < 0) {
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
        return err;
    }

    return ((value[0] << 8) | value[1]);
}


static int32_t stk3x1x_set_flag(struct stk3x1x_data *stk_data,
    uint8_t org_flag_reg, uint8_t clr)
{
    uint8_t w_flag;
    int ret;
    w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
    w_flag &= (~clr);
    //printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_FLAG_REG, w_flag);
    if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

    return ret;
}

static int32_t stk3x1x_get_flag(struct stk3x1x_data *stk_data)
{
	int ret = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_FLAG_REG);
	if (ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);

	//printk("%s flag %x\n",__func__,ret);
	return ret;
}
static int32_t stk3x1x_clear_flag(struct stk3x1x_data *stk_data, uint8_t clr)
{
	int32_t ret =0;	
	uint8_t w_flag =0;
	ret = stk3x1x_get_flag(stk_data);
	if(ret < 0)
		return ret;
	
	w_flag |=ret;
	w_flag &= (~clr);
	printk("%s flag %x w flag %x\n",__func__,ret,w_flag);
	ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_FLAG_REG, w_flag);
	 if (ret < 0)
        printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	 return ret;
}

/*
static int32_t stk3x1x_read_register(struct stk3x1x_data *stk_data, uint8_t reg_addr,uint8_t count)
{
	unsigned char value[2] ={0};
	int ret =0;
	ret = stk3x1x_i2c_read_data(stk_data->client, reg_addr, count, &value[0]);
	if (ret < 0){
        pr_err("%s: fail, ret=%d\n", __func__, ret);
	}
	
	return ((value[0] << 8) | value[1]);
}
*/
static int32_t stk3x1x_set_state(struct stk3x1x_data *stk_data, uint8_t state)
{
    int ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_STATE_REG, state);
    if (ret < 0)
        pr_err("%s: fail, ret=%d\n", __func__, ret);

    return ret;
}

static int32_t stk3x1x_get_state(struct stk3x1x_data *stk_data)
{
    int ret = stk3x1x_i2c_smbus_read_byte_data(stk_data->client,
		STK_STATE_REG);
    if (ret < 0)
        pr_err("%s: fail, ret=%d\n", __func__, ret);

    return ret;
}
/*
static void stk_als_report(struct stk3x1x_data *stk_data, int als)
{

	ktime_t timestamp = ktime_get_boottime();

	stk_data->als_lux_last = als;
	input_report_abs(stk_data->als_input_dev, ABS_MISC, als);

	input_event(stk_data->als_input_dev,
		EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(stk_data->als_input_dev,
		EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);

	input_sync(stk_data->als_input_dev);
#ifdef STK_DEBUG_PRINTF
    //printk(KERN_INFO "%s: als input event %d lux\n", __func__, als);
#endif
}

static void stk_ps_report(struct stk3x1x_data *stk_data, int nf)
{

    ktime_t timestamp = ktime_get_boottime();

    stk_data->ps_distance_last = nf;
    input_report_abs(stk_data->ps_input_dev, ABS_DISTANCE, nf? 1: 1000);
    input_report_abs(stk_data->ps_input_dev, ABS_DISTANCE, nf? 0: 1023);

    input_event(stk_data->ps_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
    input_event(stk_data->ps_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);

    input_sync(stk_data->ps_input_dev);
    wake_lock_timeout(&stk_data->ps_wakelock, 3 * HZ);
}


static void stk3x1x_als_set_poll_delay(struct stk3x1x_data *stk_data)
{
	uint8_t als_it = stk_data->alsctrl_reg & 0x0F;

	if (als_it == 0x8) {
        stk_data->als_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
    } else if (als_it == 0x9) {
        stk_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
    } else if (als_it == 0xA) {
        stk_data->als_poll_delay = ns_to_ktime(220 * NSEC_PER_MSEC);
    } else if (als_it == 0xB) {
        stk_data->als_poll_delay = ns_to_ktime(440 * NSEC_PER_MSEC);
    } else if (als_it == 0xC) {
        stk_data->als_poll_delay = ns_to_ktime(880 * NSEC_PER_MSEC);
    } else {
        stk_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
        printk(KERN_INFO "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
    }
}

*/

static int32_t stk3x1x_init_all_reg(struct stk3x1x_data *stk_data)
{
    int32_t ret;
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_STATE_REG, stk_data->state_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_STATE_REG error\n", __func__);
        return ret;
    }

    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_PSCTRL_REG, stk_data->psctrl_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_PSCTRL_REG error\n", __func__);
        return ret;
    }

    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_ALSCTRL_REG, stk_data->alsctrl_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_ALSCTRL_REG error\n", __func__);
        return ret;
    }

    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_LEDCTRL_REG, stk_data->ledctrl_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_LEDCTRL_REG error\n", __func__);
        return ret;
    }

    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_WAIT_REG, stk_data->wait_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_WAIT_REG error\n", __func__);
        return ret;
    }

#ifdef STK_TUNE0
    stk_data->psa = 0x0;
    stk_data->psi = 0xFFFF;
#endif
	ret = stk3x1x_set_ps_thd(stk_data);
    if (ret < 0) {
        printk(KERN_ERR "%s: set ps thd error\n", __func__);
        return ret;
    }
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_INT_REG, stk_data->int_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write STK_INT_REG error\n", __func__);
        return ret;
    }

    return 0;
}

static void stk3x1x_proc_plat_data(struct stk3x1x_data *stk_data,
		struct stk3x1x_platform_data *plat_data)
{
    uint8_t w_reg;
    stk_data->state_reg = plat_data->state_reg;
    stk_data->psctrl_reg = plat_data->psctrl_reg;
    stk_data->alsctrl_reg = plat_data->alsctrl_reg;
    stk_data->ledctrl_reg = plat_data->ledctrl_reg;
    stk_data->als_take_effect = plat_data->als_take_effect;
    stk_data->ps_take_effect = plat_data->ps_take_effect;

    if ((stk_data->pid == STK3310SA_PID) || (stk_data->pid == STK3311SA_PID))
        stk_data->ledctrl_reg &= 0x3F;

    stk_data->wait_reg = plat_data->wait_reg;

    if (stk_data->wait_reg < 2) {
        printk(KERN_WARNING
			"%s: wait_reg should be larger than 2, force to write 2\n", __func__);
        stk_data->wait_reg = 2;
    } else if (stk_data->wait_reg > 0xFF) {
        printk(KERN_WARNING
			"%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
        stk_data->wait_reg = 0xFF;
    }

    if (stk_data->ps_thd_h == 0 && stk_data->ps_thd_l == 0) {
        stk_data->ps_thd_h = plat_data->ps_thd_h;
        stk_data->ps_thd_l = plat_data->ps_thd_l;
    }


    w_reg = 0;
    w_reg |= STK_INT_PS_MODE;
    stk_data->int_reg = w_reg;
    return;
}

static int32_t stk3x1x_software_reset(struct stk3x1x_data *stk_data)
{
    uint8_t w_reg = 0x7F;
    int32_t r = stk3x1x_i2c_smbus_write_byte_data(stk_data->client, STK_WAIT_REG, w_reg);
    if (r < 0) {
        printk(KERN_ERR
			"%s(): write STK_WAIT_REG error, ret=%d\n", __func__, r);
        return r;
    }

    r = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_WAIT_REG);
    if (w_reg != r) {
        printk(KERN_ERR
			"%s(): read-back STK_WAIT_REG is not the same\n", __func__);
        return -EINVAL;
    }

    r = stk3x1x_i2c_smbus_write_byte_data(stk_data->client, STK_SW_RESET_REG, 0);
    if (r < 0) {
        printk(KERN_ERR "%s(): write STK_SW_RESET_REG reset\n", __func__);
        return r;
    }

    usleep_range(13000, 15000);
    return 0;
}

static int32_t stk3x1x_init_all_setting(struct i2c_client *client,
		struct stk3x1x_platform_data *plat_data)
{
    struct stk3x1x_data *stk_data = i2c_get_clientdata(client);
	int32_t ret = stk3x1x_software_reset(stk_data);
    if (ret < 0) {
		pr_err("%s(): Failed to reset device!\n", __func__);
		return ret;
	}

	
    ret = stk3x1x_init_all_reg(stk_data);
    if (ret < 0)
        return ret;


    //stk3x1x_als_set_poll_delay(stk_data);

    //stk_data->als_enabled = false;
    //stk_data->ps_enabled = false;
    //stk_data->re_enable_als = false;
    //stk_data->re_enable_ps = false;
    stk_data->ir_code = 0;
    //stk_data->als_correct_factor = 1000;
   
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
    stk_init_code_threshold_table(stk_data);
#endif
#ifdef STK_TUNE0

    stk_data->psi_set = 0;
    if(stk_data->ps_take_effect){

        stk_ps_tune_zero_init(stk_data);
    }
#endif
 #ifdef STK_ALS_FIR
    memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    atomic_set(&stk_data->firlength, STK_FIR_LEN);
#endif 
    atomic_set(&stk_data->recv_reg, 0);
    stk_data->als_code_last = 100;
    return 0;
}



static int32_t stk3x1x_ps_switch(struct stk3x1x_data *stk_data,
		uint8_t enable, uint8_t validate_reg)
{
    int32_t ret;
    uint8_t w_state_reg;
    uint8_t curr_ps_enable;
    uint16_t reading;
	uint8_t count =0;
	uint8_t i = 0;

    curr_ps_enable = stk_data->ps_enabled ? 1 : 0;
    if (curr_ps_enable == enable)
        return 0;

#ifdef STK_TUNE0
#ifndef CTTRACKING
	if (!(stk_data->psi_set) && !enable)
    {
        hrtimer_cancel(&stk_data->ps_tune0_timer);
        cancel_work_sync(&stk_data->stk_ps_tune0_work);
    }
#endif
#endif
    ret = stk3x1x_get_state(stk_data);
    if (ret < 0)
        return ret;
    w_state_reg = ret;
    w_state_reg &=
		~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK);


	if(w_state_reg & STK_STATE_EN_ALS_MASK)
	{

//enable ps after alsp get first data because state register change would reflash state machine
		while(atomic_read(&stk_data->als_refresh_flag) == 0 && count < (1<<3)){
			pr_err("%s ps enable count %d\n",__func__,count);
			msleep(stk_data->als_poll_delay >> 3);
			++count;
		}
    }
    if (enable) {
		//使用之前的远近门限
		ret = stk3x1x_set_ps_thd(stk_data);
		if(ret < 0)
			return ret;
		
        w_state_reg |= STK_STATE_EN_PS_MASK;

        if (!(w_state_reg & STK_STATE_EN_ALS_MASK)){
			// 没有使能als 使能等待，避免ps灯没有冷却降低寿命及数据准确性
            w_state_reg |= STK_STATE_EN_WAIT_MASK;
        }
		// disable ps slove interrupt signal can not continue trigger
		ret = stk3x1x_set_state(stk_data, STK_STATE_EN_ALS_MASK); 
		if(ret < 0)
			return ret;	
    }
	//由于改变state ，状态机将重新积分，因此光距离标志都需要清标志
	stk3x1x_clear_flag(stk_data,STK_FLG_PSDR_MASK | STK_FLG_PSINT_MASK | STK_FLG_ALSDR_MASK);
	
    ret = stk3x1x_set_state(stk_data, w_state_reg);
    if (ret < 0)
        return ret;
	//pr_err("set state \n");
    stk_data->state_reg = w_state_reg;
	
		
    if (enable) {
        enable_irq(stk_data->irq);
#ifdef STK_TUNE0
#ifdef CTTRACKING
		stk_data->ps_thd_update = false;
#endif
#ifndef CALI_PS_EVERY_TIME
		if (!(stk_data->psi_set))
			hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay, HRTIMER_MODE_REL);
#else
		stk_data->psi_set = 0;
		stk_data->psa = 0;
		stk_data->psi = 0xFFFF;
		if(!stk_data->ps_factory_flag)
			hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay, HRTIMER_MODE_REL);
		
#endif
#endif
		
        usleep_range(4000, 5000);
		count =0;
		while(!(stk3x1x_get_flag(stk_data) & STK_FLG_PSDR_MASK) && count < 4 ){
		    printk("%s count %u \n",__func__,count);
		    msleep(stk_data->als_poll_delay>>2);
		    count++;   
		}
        
        if(count < 4){
		msleep(200);
            	reading = stk3x1x_get_ps_reading(stk_data);
		printk(KERN_ERR "delay 200ms ps_code %d\n", reading);
		if (reading < 0){
                	disable_irq(stk_data->irq);
           		return reading;
            	}
			//pr_err("read data\n");
			//每次打开光距离均会更新底噪及远近门限
			if(reading < stk_data->ps_crosstalk_max &&  stk_ps_val(stk_data) == 0 && reading != 0){
				stk_data->last_ps_canc = reading;
				stk_data->psi = stk_data->last_ps_canc;
				stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct ;
				stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct ;
				
                ret =stk3x1x_set_ps_thd(stk_data);
                if(ret < 0)
                {
                    disable_irq(stk_data->irq);
                    return ret;
                }
				pr_err("%s set thd_h %u thd_l %u\n",__func__,stk_data->ps_thd_h,stk_data->ps_thd_l);
			}else{
				pr_err("%s ps data is too large or strong light,so user last ps canc %u\n",__func__,stk_data->last_ps_canc);
			}
			
			if (reading < stk_data->ps_thd_h){
				stk_data->ps_distance_now = FAR_CODE;
				sensors_report_1(stk_data->ps_input_dev, stk_data->ps_distance_now);
				pr_err("%s: ps input event=%d, ps_code=%d\n",
				__func__, FAR_CODE, reading);
				
			}else{
				pr_err("%s: ps_code=%d\n",__func__,reading);
			}
        }else{
			pr_err("ps data can't ready\n");
		}

        stk_data->ps_enabled = true;


	} else {
#ifdef CTTRACKING
		if(!stk_data->ps_factory_flag)
			hrtimer_cancel(&stk_data->ps_tune0_timer);
#endif
        disable_irq(stk_data->irq);
        stk_data->ps_enabled = false;
		sensors_report_1(stk_data->ps_input_dev, FAR_CODE);

    }
    return 0;
}

static int32_t stk3x1x_als_switch(struct stk3x1x_data *stk_data,
		uint8_t enable)
{
    int32_t ret;
    uint8_t w_state_reg;
    uint8_t curr_als_enable = (stk_data->als_enabled) ? 1 : 0;
    if (curr_als_enable == enable)
        return 0;
#ifdef STK_ALS_FIR
    if (enable) {
		stk_data->fir.number = 0;
		stk_data->fir.idx = 0;
    }	
#endif
    ret = stk3x1x_get_state(stk_data);
    if (ret < 0)
        return ret;
    w_state_reg =
		(uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)));
	
    if (enable){
        w_state_reg |= STK_STATE_EN_ALS_MASK;
		atomic_set(&stk_data->als_refresh_flag,0);	
		
	
    }else if (w_state_reg & STK_STATE_EN_PS_MASK){
		atomic_set(&stk_data->als_refresh_flag,1);
        w_state_reg |= STK_STATE_EN_WAIT_MASK;
    }else{
		atomic_set(&stk_data->als_refresh_flag,1);
	}
	//由于改变state ，状态机将重新开始，因此光距离标志都需要清标志
	stk3x1x_clear_flag(stk_data,STK_FLG_PSDR_MASK  | STK_FLG_ALSDR_MASK);	 
    ret = stk3x1x_set_state(stk_data, w_state_reg);
    if (ret < 0)
        return ret;
	
    stk_data->state_reg = w_state_reg;

    if (enable) {
        stk_data->als_enabled = true;
		stk_data->first_als = true;
        hrtimer_start(&stk_data->als_timer,
        	stk_data->als_work_delay, HRTIMER_MODE_REL);

    } else {
        stk_data->als_enabled = false;
        hrtimer_cancel(&stk_data->als_timer);
        cancel_work_sync(&stk_data->stk_als_work);

    }
    return 0;
}

static int32_t stk3x1x_set_irs_it_slp(struct stk3x1x_data *stk_data,
		uint16_t *slp_time, int32_t ials_it_reduce)
{
    uint8_t irs_alsctrl;
    int32_t ret;
    irs_alsctrl = (stk_data->alsctrl_reg & 0x0F) - ials_it_reduce;

    switch (irs_alsctrl) {
    case 2:
        *slp_time = 1;
        break;
    case 3:
        *slp_time = 2;
        break;
    case 4:
        *slp_time = 3;
        break;
    case 5:
        *slp_time = 6;
        break;
    case 6:
        *slp_time = 12;
        break;
    case 7:
        *slp_time = 24;
        break;
    case 8:
        *slp_time = 48;
        break;
    case 9:
        *slp_time = 96;
        break;
    case 10:
        *slp_time = 192;
        break;
    default:
        printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
        ret = -EINVAL;
        return ret;
    }

    irs_alsctrl |= (stk_data->alsctrl_reg & 0xF0);
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_ALSCTRL_REG, irs_alsctrl);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    return 0;
}

static int32_t stk3x1x_get_ir_reading(struct stk3x1x_data *stk_data,
		int32_t als_it_reduce)
{
    int32_t word_data, ret;
    uint8_t w_reg, retry = 0;
    uint16_t irs_slp_time = 100;
    unsigned char value[2];
    ret = stk3x1x_set_irs_it_slp(stk_data, &irs_slp_time, als_it_reduce);
    if (ret < 0)
        goto irs_err_i2c_rw;

    ret = stk3x1x_get_state(stk_data);
    if (ret < 0)
        goto irs_err_i2c_rw;

    w_reg = ret | STK_STATE_EN_IRS_MASK;
    ret = stk3x1x_set_state(stk_data, w_reg);
    if (ret < 0)
        goto irs_err_i2c_rw;

    msleep(irs_slp_time);

    do {
        usleep_range(3000, 4000);
        ret = stk3x1x_get_flag(stk_data);
        if (ret < 0)
            goto irs_err_i2c_rw;

        retry++;
    } while ((retry < 10) && ((ret & STK_FLG_IR_RDY_MASK) == 0));

    if (retry == 10) {
        printk(KERN_ERR "%s: ir data is not ready for a long time\n", __func__);
        ret = -EINVAL;
        goto irs_err_i2c_rw;
    }

    ret = stk3x1x_set_flag(stk_data, ret, STK_FLG_IR_RDY_MASK);
    if (ret < 0)
        goto irs_err_i2c_rw;

    ret = stk3x1x_i2c_read_data(stk_data->client, STK_DATA1_IR_REG, 2, &value[0]);
    if (ret < 0) {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        goto irs_err_i2c_rw;
    }

    word_data = ((value[0] << 8) | value[1]);
    //printk(KERN_INFO "%s: ir=%d\n", __func__, word_data);
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
    	STK_ALSCTRL_REG, stk_data->alsctrl_reg );
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        goto irs_err_i2c_rw;
    }

    return word_data;
irs_err_i2c_rw:
    return ret;
}

static int32_t stk3x1x_get_als_reading(struct stk3x1x_data *stk_data)
{
    int32_t als_data, ir_data = 0;
#ifdef STK_ALS_FIR
    int index;
    int firlen = atomic_read(&stk_data->firlength);
#endif
    unsigned char value[2];
    int ret;
    ret = stk3x1x_i2c_read_data(stk_data->client,
    	STK_DATA1_ALS_REG, 2, &value[0]);
    if (ret < 0) {
        pr_err("%s fail, ret=0x%x\n", __func__, ret);
        return ret;
    }
    als_data = (value[0] << 8) | value[1];
    if (stk_data->p_1x_r_bd_with_co == 0x07 || stk_data->p_19_r_bc == 0x03) {
        als_data = als_data * 16 / 10;
        if (als_data > 65535)
            als_data = 65535;
    }
    if (stk_data->p_wv_r_bd_with_co & 0b010) {
        if (als_data < STK_ALS_THRESHOLD && stk_data->als_code_last > 10000) {
            ir_data =
				stk3x1x_get_ir_reading(stk_data, STK_ALS_READ_IRS_IT_REDUCE);
            // printk(KERN_INFO "%s: als_data=%d, als_code_last=%d,ir_data=%d\n",
            // __func__, als_data, stk_data->als_code_last, ir_data);
            if (ir_data > (STK_ALS_THRESHOLD * 3)) {
                als_data = stk_data->als_code_last;
            }
        }
    }

    stk_data->als_code_last = als_data;
#ifdef STK_ALS_FIR
    if (stk_data->fir.number < firlen) {
        stk_data->fir.raw[stk_data->fir.number] = als_data;
        stk_data->fir.sum += als_data;
        stk_data->fir.number++;
        stk_data->fir.idx++;
    } else {
        index = stk_data->fir.idx % firlen;
        stk_data->fir.sum -= stk_data->fir.raw[index];
        stk_data->fir.raw[index] = als_data;
        stk_data->fir.sum += als_data;
        stk_data->fir.idx++;
        als_data = stk_data->fir.sum / firlen;
    }
#endif
    return als_data;
}

static ssize_t stk_als_code_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
	int32_t reading;

    reading = stk_data->als_code_last;

    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}
static struct device_attribute als_code_attribute =
	__ATTR(code, 0440, stk_als_code_show, NULL);


static int stk_als_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enabled)
{
    struct stk3x1x_data *stk_data =
		container_of(sensors_cdev, struct stk3x1x_data, als_cdev);
    int err = 0;
	sensors_marker("stk_als_enable_set()",
		"receive", "als_en", enabled ? "1" : "0");
    mutex_lock(&stk_data->io_lock);
    err = stk3x1x_als_switch(stk_data, enabled);
    mutex_unlock(&stk_data->io_lock);
    if (err < 0) {
		pr_err(" als enable/disable fail, err=%d\n", err);
        return err;
	}
    return 0;
}


static ssize_t stk_als_lux_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    int32_t als_reading;
    //uint32_t als_lux;
    als_reading = stk3x1x_get_als_reading(stk_data);
    //als_lux = stk_alscode2lux(stk_data, als_reading);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_reading);
}
static struct device_attribute als_lux_attribute =
	__ATTR(lux, 0440, stk_als_lux_show, NULL);

static ssize_t stk_als_transmittance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
		stk_data->pdata->als_transmittance);
}

static ssize_t stk_als_transmittance_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->pdata->als_transmittance = value;
    return size;
}
static struct device_attribute als_transmittance_attribute =
	__ATTR(transmittance, 0640, stk_als_transmittance_show, stk_als_transmittance_store);

static ssize_t stk_als_ir_code_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    int32_t reading = stk3x1x_get_ir_reading(stk_data, STK_IRS_IT_REDUCE);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}
static struct device_attribute als_ir_code_attribute =
	__ATTR(ircode, 0440, stk_als_ir_code_show, NULL);

#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    int len = atomic_read(&stk_data->firlength);
    printk(KERN_INFO "%s: len = %2d, idx = %2d\n",
		__func__, len, stk_data->fir.idx);
    printk(KERN_INFO "%s: sum = %5d, ave = %5d\n",
		__func__, stk_data->fir.sum, stk_data->fir.sum / len);
    return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

static ssize_t stk_als_firlen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
    uint64_t value = 0;
    int ret = kstrtoull(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    if (value > MAX_FIR_LEN) {
        printk(KERN_ERR "%s: firlen exceed maximum filter length\n", __func__);
    } else if (value < 1) {
        atomic_set(&stk_data->firlength, 1);
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    } else {
        atomic_set(&stk_data->firlength, value);
        memset(&stk_data->fir, 0x00, sizeof(stk_data->fir));
    }

    return size;
}
static struct device_attribute als_firlen_attribute =
	__ATTR(firlen, 0640, stk_als_firlen_show, stk_als_firlen_store);
#endif  /* #ifdef STK_ALS_FIR */

static ssize_t als_fittness_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", stk_data->als_fittness);
}
static ssize_t als_fittness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(als_cdev,
		struct stk3x1x_data, als_cdev);
	int value = 0;
	sscanf(buf, "%d", &value);
	stk_data->als_fittness = value;
	return count;
}
static struct device_attribute als_fittness_attribute =
	__ATTR(als_fittness, 0660, als_fittness_show, als_fittness_store);

static struct attribute *stk_als_attrs[] = {
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
    &als_ir_code_attribute.attr,
    &als_fittness_attribute.attr,
#ifdef STK_ALS_FIR
    &als_firlen_attribute.attr,
#endif
    NULL
};

static struct attribute_group stk_als_attribute_group = {
    .attrs = stk_als_attrs,
};

static ssize_t stk_ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	int count =0;
	int32_t reading_sub_canc =0 ,reading =0;
	if (stk_data->ps_enabled){
		
		while(!(stk3x1x_get_flag(stk_data) & STK_FLG_PSDR_MASK) && count < 4 ){
            printk("%s count %u \n",__func__,count);
            msleep(stk_data->als_poll_delay>>2);
            count++;   
        }
        if(count < 4){

            reading = stk3x1x_get_ps_reading(stk_data);
            if(reading < 0){
                pr_err("%s read ps data failed\n",__func__);
                return -1;
            }    
            reading_sub_canc = reading - stk_data->last_ps_canc;
            printk("%s, %d \n", __func__, reading);
            if (reading_sub_canc <=0)
                reading_sub_canc = 0;
            return scnprintf(buf, PAGE_SIZE, "%d\n",reading_sub_canc);
        } else{
            pr_err("%s ps data can't ready",__func__);
            return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
        }
	}else{
		pr_err("%s ps disable\n",__func__);
		return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
	}
}
static struct device_attribute ps_data_attribute =
	__ATTR(ps_data, 0440, stk_ps_data_show, NULL);

static ssize_t stk_ps_code_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
      uint16_t reading;
      reading = stk3x1x_get_ps_reading(stk_data);
      return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static struct device_attribute ps_code_attribute =
	__ATTR(ps_code, 0440, stk_ps_code_show, NULL);


static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enabled)
{
    struct stk3x1x_data *stk_data =
		container_of(sensors_cdev, struct stk3x1x_data, ps_cdev);
    int err;
	sensors_marker("stk_ps_enable_set()",
		"receive", "ps_en", enabled ? "1" : "0");
	printk("%s factory flag %u\n",__func__,stk_data->ps_factory_flag);
    mutex_lock(&stk_data->io_lock);
    err = stk3x1x_ps_switch(stk_data, enabled, 0);
    mutex_unlock(&stk_data->io_lock);
    if (err < 0)
        return err;

    return 0;
}

static ssize_t stk_ps_enable_aso_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    int32_t ret =
		stk3x1x_i2c_smbus_read_byte_data(stk_data->client,
										STK_STATE_REG);
    ret = (ret & STK_STATE_EN_ASO_MASK) ? 1 : 0;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_aso_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    uint8_t en;
    int32_t ret;
    uint8_t w_state_reg;

    if (sysfs_streq(buf, "1"))
        en = 1;
    else if (sysfs_streq(buf, "0"))
        en = 0;
    else {
        printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }

    printk(KERN_INFO "%s: Enable PS ASO : %d\n", __func__, en);
    ret = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_STATE_REG);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK));

    if (en)
        w_state_reg |= STK_STATE_EN_ASO_MASK;

    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_STATE_REG, w_state_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    return size;
}
static struct device_attribute ps_enable_aso_attribute =
	__ATTR(enableaso, 0640, stk_ps_enable_aso_show, stk_ps_enable_aso_store);

static ssize_t stk_ps_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    int32_t word_data;
    unsigned char value[2];
    int ret = stk3x1x_i2c_read_data(stk_data->client,
		STK_DATA1_OFFSET_REG, 2, &value[0]);
    if (ret < 0) {
        printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
        return ret;
    }

    word_data = (value[0] << 8) | value[1];
    return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}

static ssize_t stk_ps_offset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long offset = 0;
    unsigned char val[2];
    int ret = kstrtoul(buf, 10, &offset);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    if (offset > 65535) {
        printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, offset);
        return -EINVAL;
    }

    val[0] = (offset & 0xFF00) >> 8;
    val[1] = offset & 0x00FF;
    ret = stk3x1x_i2c_write_data(stk_data->client, STK_DATA1_OFFSET_REG, 2, val);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    return size;
}
static struct device_attribute ps_offset_attribute =
	__ATTR(offset, 0640, stk_ps_offset_show, stk_ps_offset_store);

static ssize_t stk_ps_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    int32_t dist = 1;
    int32_t ret;
    ret = stk3x1x_get_flag(stk_data);
    if (ret < 0)
        return ret;

    dist = (ret & STK_FLG_NF_MASK) ? FAR_CODE : NEAR_CODE;
    sensors_report_1(stk_data->ps_input_dev, dist);
    printk(KERN_INFO "%s: ps input event=%d\n", __func__, dist);
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}

static ssize_t stk_ps_distance_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    sensors_report_1(stk_data->ps_input_dev, value);
    printk(KERN_INFO "%s: ps input event=%d\n", __func__, (int)value);
    return size;
}
static struct device_attribute ps_distance_attribute =
	__ATTR(distance, 0640, stk_ps_distance_show, stk_ps_distance_store);

static ssize_t stk_ps_code_thd_l_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//int value =0;
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	//value = stk3x1x_read_register(stk_data,STK_THDL1_PS_REG,2);
	//return scnprintf(buf, PAGE_SIZE, "%d\n", value);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_lt_n_ct);
}

static ssize_t stk_ps_code_thd_l_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
	stk_data->stk_lt_n_ct = value;
    return size;
}
static struct device_attribute ps_code_thd_l_attribute =
	__ATTR(ps_thd_away, 0640, stk_ps_code_thd_l_show, stk_ps_code_thd_l_store);

static ssize_t stk_ps_code_thd_h_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//int value =0;
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	//value = stk3x1x_read_register(stk_data,STK_THDH1_PS_REG,2);
	//return scnprintf(buf, PAGE_SIZE, "%d\n", value);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_ht_n_ct);
}

static ssize_t stk_ps_code_thd_h_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret = kstrtoul(buf, 10, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
	stk_data->stk_ht_n_ct= value;
    return size;
}
static struct device_attribute ps_code_thd_h_attribute =
	__ATTR(ps_thd_close, 0640, stk_ps_code_thd_h_show, stk_ps_code_thd_h_store);


//
static ssize_t stk_ps_canc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->last_ps_canc);
}

static struct device_attribute ps_canc_attribute =
	__ATTR(ps_canc, 0440, stk_ps_canc_show, NULL);


static ssize_t stk_ps_crosstalk_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->pdata->ps_crosstalk_max);
}

static ssize_t stk_ps_crosstalk_max_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	unsigned long value = 0;
	int ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;
	}
	stk_data->pdata->ps_crosstalk_max= value;
	return size;
}
static struct device_attribute ps_crosstalk_maxthd_attribute =
	__ATTR(ps_crosstalk_maxthd, 0640, stk_ps_crosstalk_max_show, stk_ps_crosstalk_max_store);
//



static ssize_t stk_all_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[0x23];
    uint8_t cnt;
    int len = 0;
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);

    for (cnt = 0; cnt < 0x20; cnt++) {
        ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, (cnt));
        if (ps_reg[cnt] < 0) {
            printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
            return -EINVAL;
        } else {
            printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
            len += scnprintf(buf + len, PAGE_SIZE - len,
				"[%2X]%2X,", cnt, ps_reg[cnt]);
        }
    }

    ps_reg[cnt] =
		stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);
    if (ps_reg[cnt] < 0) {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk(KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);
    if (ps_reg[cnt] < 0) {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk(KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, 0xE0);
    if (ps_reg[cnt] < 0) {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk(KERN_INFO "reg[0xE0]=0x%2X\n", ps_reg[cnt]);
    len +=
		scnprintf(buf + len, PAGE_SIZE - len,
		"[3E]%2X,[3F]%2X,[E0]%2X\n",
		ps_reg[cnt - 2], ps_reg[cnt - 1], ps_reg[cnt]);
    return len;
    /*
	return scnprintf(buf, PAGE_SIZE,
		"[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n",
            ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8],
            ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17],
            ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
	*/
}
static struct device_attribute all_reg_attribute =
	__ATTR(allreg, 0440, stk_all_reg_show, NULL);

static ssize_t stk_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    int32_t ps_reg[27];
    uint8_t cnt;
    for (cnt = 0; cnt < 25; cnt++) {
        ps_reg[cnt] =
			stk3x1x_i2c_smbus_read_byte_data(stk_data->client, (cnt));
        if (ps_reg[cnt] < 0) {
            printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
            return -EINVAL;
        } else {
            printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
        }
    }

    ps_reg[cnt] =
		stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_PDT_ID_REG);
    if (ps_reg[cnt] < 0) {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);
    cnt++;
    ps_reg[cnt] =
		stk3x1x_i2c_smbus_read_byte_data(stk_data->client, STK_RSRVD_REG);
    if (ps_reg[cnt] < 0) {
        printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);
        return -EINVAL;
    }

    printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);
    return scnprintf(buf, PAGE_SIZE,
		"[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n",
         ps_reg[0] & 0x01, (ps_reg[0] & 0x02) >> 1, ((ps_reg[0] & 0x04) >> 2) * ps_reg[5] * 6, (ps_reg[0] & 0x20) >> 5,
         (ps_reg[0] & 0x40) >> 6, ps_reg[16] & 0x01, (ps_reg[16] & 0x04) >> 2, (ps_reg[16] & 0x10) >> 4, (ps_reg[16] & 0x20) >> 5);
}
static struct device_attribute status_attribute =
	__ATTR(status, 0440, stk_status_show, NULL);

static ssize_t stk_recv_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct stk3x1x_data *stk_data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE,
		"0x%04X\n", atomic_read(&stk_data->recv_reg));
}

static ssize_t stk_recv_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int32_t recv_data;
	int ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    recv_data = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, value);
    //  printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
    atomic_set(&stk_data->recv_reg, recv_data);
    return size;
}
static struct device_attribute ps_recv_attribute =
	__ATTR(recv, 0640, stk_recv_show, stk_recv_store);

static ssize_t stk_send_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t stk_send_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
    int addr, cmd;
    int32_t ret, i;
    char *token[10];
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");
	ret = kstrtoul(token[0], 16, (unsigned long *) & (addr));
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }
	ret = kstrtoul(token[1], 16, (unsigned long *) & (cmd));
    if (ret < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);
    ret =
		stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		(unsigned char)addr, (unsigned char)cmd);
    if (0 != ret) {
        printk(KERN_ERR "%s: stk3x1x_i2c_smbus_write_byte_data fail\n", __func__);
        return ret;
    }

    return size;
}
static struct device_attribute ps_send_attribute =
	__ATTR(send, 0640, stk_send_show, stk_send_store);

static ssize_t stk_ps_factory_flag_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	unsigned long value = 0;
	int ret = kstrtoul(buf, 10, &value);
	if (ret < 0) {
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;
	}
	stk_data->ps_factory_flag = value;
	return size;
}

static struct device_attribute ps_factory_flag_attribute =
	__ATTR(factory_flag, 0440, NULL, stk_ps_factory_flag_store);

#ifdef STK_TUNE0
static ssize_t stk_ps_cali_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
	int32_t word_data;
	unsigned char value[2];
	int ret = stk3x1x_i2c_read_data(stk_data->client, 0x20, 2, &value[0]);
	if (ret < 0) {
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	word_data = (value[0] << 8) | value[1];
	ret = stk3x1x_i2c_read_data(stk_data->client, 0x22, 2, &value[0]);
	if (ret < 0) {
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	word_data += ((value[0] << 8) | value[1]);
	printk("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __func__,
		stk_data->psi_set, stk_data->psa, stk_data->psi, word_data);

	return 0;
}
static struct device_attribute ps_cali_attribute =
	__ATTR(cali, 0440, stk_ps_cali_show, NULL);

static ssize_t stk_ps_maxdiff_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret;

    if ((ret = kstrtoul(buf, 10, &value)) < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->stk_max_min_diff = (int)value;
    return size;
}

static ssize_t stk_ps_maxdiff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->stk_max_min_diff);
}
static struct device_attribute ps_maxdiff_attribute =
	__ATTR(maxdiff, 0640, stk_ps_maxdiff_show, stk_ps_maxdiff_store);

static ssize_t stk_ps_ltnct_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret;

    if ((ret = kstrtoul(buf, 10, &value)) < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->ps_thd_l = (int) value;
    return size;
}

static ssize_t stk_ps_ltnct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->ps_thd_l);
}
static struct device_attribute ps_ltnct_attribute =
	__ATTR(ltnct, 0640, stk_ps_ltnct_show, stk_ps_ltnct_store);

static ssize_t stk_ps_htnct_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret;

    if ((ret = kstrtoul(buf, 10, &value)) < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->ps_thd_h = (int)value;
    return size;
}

static ssize_t stk_ps_htnct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->ps_thd_h);
}
static struct device_attribute ps_htnct_attribute =
	__ATTR(htnct, 0640, stk_ps_htnct_show, stk_ps_htnct_store);

#ifdef CTTRACKING
static ssize_t stk_ps_greasy_dirty_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    unsigned long value = 0;
    int ret;

    if ((ret = kstrtoul(buf, 10, &value)) < 0) {
        printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
        return ret;
    }

    stk_data->ps_greasy_dirty = (int)value;
    return size;
}

static ssize_t stk_ps_greasy_dirty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct stk3x1x_data *stk_data = container_of(ps_cdev,
		struct stk3x1x_data, ps_cdev);
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk_data->ps_greasy_dirty);
}
static struct device_attribute ps_greasy_dirty_attribute =
	__ATTR(ps_greasy_dirty, 0640, stk_ps_greasy_dirty_show, stk_ps_greasy_dirty_store);
#endif //#ifdef CTTRACKING

#endif

static struct attribute *stk_ps_attrs [] = {
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
    &ps_factory_flag_attribute.attr,
#ifdef CTTRACKING
    &ps_greasy_dirty_attribute.attr,
#endif
#ifdef STK_TUNE0
    &ps_cali_attribute.attr,
    &ps_maxdiff_attribute.attr,
    &ps_ltnct_attribute.attr,
    &ps_htnct_attribute.attr,
#endif
    NULL
};

static struct attribute_group stk_ps_attribute_group = {
    .attrs = stk_ps_attrs,
};


#ifdef CTTRACKING

static int stk_skin_update_thd(struct stk3x1x_data *stk_data ,int32_t word_data)
{
	int ret =0;
	int flag =0;
	uint16_t ct_value = 0;
		//Skin
	if (stk_data->psi_set != 0)
	{
		if (stk_data->ps_distance_now == NEAR_CODE && !stk_data->ps_thd_update)//near
		{
			if ((word_data - stk_data->last_ps_canc) > STK_SKIN_TH)
			{
		
				stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct + stk_data->ps_greasy_dirty;
				stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct + stk_data->ps_greasy_dirty;
				ret = stk3x1x_set_ps_thd(stk_data);
				if(ret  < 0){
					printk(KERN_ERR "Set ps thd err, use old thd value\n");
					return ret;
				}
				stk_data->ps_thd_update = true;
				printk(KERN_INFO "%s: Set skin thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
			}
		}
		else
		{
			if (stk_data->ps_thd_update == true && word_data < stk_data->boot_ps_canc + stk_data->ps_greasy_dirty)
			{
				stk_data->last_ps_canc = word_data;
				stk_data->psi = word_data;
				stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct;
				stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct;

				ret = stk3x1x_set_ps_thd(stk_data);
				if(ret  < 0){
					printk(KERN_ERR "Set ps thd err, use old thd value\n");
					return ret;
				}
				stk_data->ps_thd_update = false;
				printk(KERN_INFO "%s: Reset no skin thd, HT=%d, LT=%d\n", 
					__func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
			}
		}
	}

		//tracking
	if ((stk_data->psi_set != 0))
	{
		if (stk_data->ps_distance_now == FAR_CODE && stk_data->ps_thd_update == false)
		{
			ret = stk_ps_val(stk_data);
			
			if (ret == 0)
			{
				if (word_data > 0)
				{
					// ct_value = psi 
					ct_value = stk_data->ps_thd_h - stk_data->stk_ht_n_ct;
					
					flag = stk3x1x_get_flag(stk_data);
					if(flag < 0){
						printk(KERN_ERR "get flag failed\n");
						return -1;
					}
					
					if ((word_data < ct_value) && ((ct_value - word_data) > 2)
						&& ((flag&STK_FLG_PSINT_MASK) == 0))
					{
						stk_data->last_ps_canc = word_data;
						stk_data->psi = word_data;
						
						stk_data->ps_thd_h = stk_data->last_ps_canc + stk_data->stk_ht_n_ct;
						stk_data->ps_thd_l = stk_data->last_ps_canc + stk_data->stk_lt_n_ct;
						
						ret = stk3x1x_set_ps_thd(stk_data);
						if(ret < 0){
							printk(KERN_ERR "set ps thd failed\n");
							return ret;
						}
						printk(KERN_ERR "%s: CTTRACKING set HT=%d, LT=%d\n", __func__,
							stk_data->ps_thd_h,stk_data->ps_thd_l);	
					}
				}
			}
		}
	}
	return 0;
}
#endif



#ifdef STK_TUNE0
static int stk_ps_tune_zero_final(struct stk3x1x_data *stk_data)
{
	int ret = 0;

    //stk_data->tune_zero_init_proc = false;
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_INT_REG, stk_data->int_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
/*
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client, STK_STATE_REG, 0);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
*/
    if (stk_data->data_count == -1) {
        printk(KERN_INFO "%s: exceed limit\n", __func__);
        hrtimer_cancel(&stk_data->ps_tune0_timer);
        return 0;
    }

    stk_data->psa = stk_data->ps_stat_data[0];
    stk_data->psi = stk_data->ps_stat_data[2];
	
#ifdef CALI_PS_EVERY_TIME
    printk("stk3x3x ps average %d\n",stk_data->ps_stat_data[1]);
	if(stk_data->ps_stat_data[1] <= stk_data->ps_crosstalk_max ){
		stk_data->last_ps_canc = stk_data->ps_stat_data[1];
		stk_data->ps_thd_h = stk_data->ps_stat_data[1] + stk_data->stk_ht_n_ct; //5次数据的平均值+ 60
		stk_data->ps_thd_l = stk_data->ps_stat_data[1] + stk_data->stk_lt_n_ct;  //5次数据的平均值+ 30
		
	//成功获取底噪初始值，之后去污算法200ms执行一次
		
	}else{
		printk("%s, crosstalk is too large and no last canc!!!\n", __func__);
		stk_data->last_ps_canc = 0x00;
		stk_data->ps_thd_h =  0xffff; 
		stk_data->ps_thd_l =  0x00;  
		
	}
#else
    stk_data->ps_thd_h = stk_data->ps_stat_data[1] + stk_data->stk_ht_n_ct;
    stk_data->ps_thd_l = stk_data->ps_stat_data[1] + stk_data->stk_lt_n_ct;
#endif
	ret = stk3x1x_set_ps_thd(stk_data);
	if (ret < 0) {
        printk(KERN_ERR "%s: write STK_INT_REG error\n", __func__);
        return ret;
    }
    stk_data->boot_cali = 1;
	stk_data->tune_zero_init_proc = false;
	stk_data->ps_tune0_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
    printk(KERN_INFO "%s: set HT=%d,LT=%d\n",
		__func__, stk_data->ps_thd_h,  stk_data->ps_thd_l);
    hrtimer_cancel(&stk_data->ps_tune0_timer);
    return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x1x_data *stk_data)
{
    uint16_t ps_adc;
    int ret = 0;
	
	ret	= stk_ps_val(stk_data);
    if (ret == 0xFFFF) {
        stk_data->data_count = -1;
        stk_ps_tune_zero_final(stk_data);
        return 0;
    }

    ps_adc = stk3x1x_get_ps_reading(stk_data);
    printk(KERN_INFO "%s: ps_adc #%d=%d\n",
		__func__, stk_data->data_count, ps_adc);

    if (ps_adc < 0)
        return ps_adc;

    stk_data->ps_stat_data[1]  +=  ps_adc;

    if (ps_adc > stk_data->ps_stat_data[0])
        stk_data->ps_stat_data[0] = ps_adc;

    if (ps_adc < stk_data->ps_stat_data[2])
        stk_data->ps_stat_data[2] = ps_adc;

    stk_data->data_count++;

    if (stk_data->data_count == 5) {
        stk_data->ps_stat_data[1]  /= stk_data->data_count;
        stk_ps_tune_zero_final(stk_data);
		stk_data->ps_stat_data[1] = 0;
		stk_data->data_count = 0;
    }

    return 0;
}

static int stk_ps_tune_zero_func_fae(struct stk3x1x_data *stk_data)
{
    int32_t word_data;
    int ret;
	int diff =0;
    unsigned char value[2];
#ifdef CALI_PS_EVERY_TIME
    if (!(stk_data->ps_enabled))
#else
    if (stk_data->psi_set || !(stk_data->ps_enabled))
#endif
    {
        return 0;
    }

    ret = stk3x1x_get_flag(stk_data);

    if (ret < 0)
        return ret;

    if (!(ret & STK_FLG_PSDR_MASK)) {
		pr_err("%s: ps data is not ready yet\n", __func__);
        return 0;
    }

    ret = stk_ps_val(stk_data); // strong light judge
	if (ret != 0)
		return ret;
	ret = stk3x1x_i2c_read_data(stk_data->client, 0x11, 2, &value[0]);
	if (ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	word_data = (value[0] << 8) | value[1];
	//printk(KERN_INFO "%s: word_data=%d\n", __func__, word_data);

	if (word_data == 0)
	{
		//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
		return 0xFFFF;
	}
#ifdef CTTRACKING
 	ret = stk_skin_update_thd(stk_data ,word_data);
	if (ret  < 0)
	{
		return ret;
	}
#endif
	if(stk_data->psi_set == 0){
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

	    if (diff > stk_data->stk_max_min_diff )
	    {
	        stk_data->psi_set = stk_data->psi;
#ifdef CTTRACKING
			printk("first %u,second %u\n",(stk_data->psi + stk_data->stk_ht_n_ct),(stk_data->ps_thd_h + stk_data->ps_greasy_dirty));
#else
            printk("first %u,second %u\n",(stk_data->psi + stk_data->stk_ht_n_ct),(stk_data->ps_thd_h));
#endif
#ifdef CALI_PS_EVERY_TIME
#ifdef CTTRACKING
			//允许底噪最坏的情况是stk_data->ps_crosstalk_max + stk_data->ps_greasy_dirty
	        if ((((stk_data->psi + stk_data->stk_ht_n_ct) > (stk_data->ps_thd_h + stk_data->ps_greasy_dirty)) 
				&& (stk_data->ps_thd_h != 0)) || stk_data->psi > stk_data->ps_crosstalk_max + stk_data->ps_greasy_dirty)
#else
            if ((((stk_data->psi + stk_data->stk_ht_n_ct) > (stk_data->ps_thd_h)) 
				&& (stk_data->ps_thd_h != 0)) || stk_data->psi > stk_data->ps_crosstalk_max)
#endif //#ifdef CTTRACKING
	        {
	            printk(KERN_INFO "%s: no update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
	        }
	        else
	        {
	        	stk_data->last_ps_canc = stk_data->psi ;
	            stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
	            stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
	            printk(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
	        }
#else
			stk_data->last_ps_canc = stk_data->psi;
	        stk_data->ps_thd_h = stk_data->psi + stk_data->stk_ht_n_ct;
	        stk_data->ps_thd_l = stk_data->psi + stk_data->stk_lt_n_ct;
	        printk(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, stk_data->ps_thd_h, stk_data->ps_thd_l);
#endif
			stk_data->boot_ps_canc =stk_data->last_ps_canc; 
	        ret =stk3x1x_set_ps_thd(stk_data);
			if(ret < 0)
				return ret;
	        printk("%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);

#ifndef CTTRACKING
			hrtimer_cancel(&stk_data->ps_tune0_timer);
#endif
	    }
	}
    return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
    struct stk3x1x_data *stk_data =
		container_of(work, struct stk3x1x_data, stk_ps_tune0_work);

    if (stk_data->tune_zero_init_proc)
        stk_tune_zero_get_ps_data(stk_data);
    else
        stk_ps_tune_zero_func_fae(stk_data);

    return;
}

static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
    struct stk3x1x_data *stk_data =
		container_of(timer, struct stk3x1x_data, ps_tune0_timer);
    queue_work(stk_data->stk_ps_tune0_wq, &stk_data->stk_ps_tune0_work);
    hrtimer_forward_now(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay);
    return HRTIMER_RESTART;
}
#endif


static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{

    struct stk3x1x_data *stk_data =
		container_of(timer, struct stk3x1x_data, als_timer);
    queue_work(stk_data->stk_als_wq, &stk_data->stk_als_work);
	//第一次采样时间为实际时间的1.2倍，之后为500ms
    hrtimer_forward_now(&stk_data->als_timer, stk_data->als_work_delay);
    return HRTIMER_RESTART;
}

static void stk_als_poll_work_func(struct work_struct *work)
{
    struct stk3x1x_data *stk_data =
		container_of(work, struct stk3x1x_data, stk_als_work);
    int32_t reading = 0,  flag_reg;
    uint16_t res = 1;
	
    flag_reg = stk3x1x_get_flag(stk_data);
    if (flag_reg < 0)
        return;
    if (!(flag_reg & STK_FLG_ALSDR_MASK)) {
        pr_info("%s: als is not ready\n", __func__);
        return;
    }

    reading = stk3x1x_get_als_reading(stk_data);
    if (reading < 0)
        return;

    //reading_lux = stk_alscode2lux(stk_data, reading);
	if(stk_data->first_als == true){
		stk_data->first_als = false;
		printk(KERN_INFO "stk3x1x lux value %d\n",reading);
	}
	atomic_set(&stk_data->als_refresh_flag,1);
	sensors_report_2(stk_data->als_input_dev,
				reading, 0, res, &confuse_i);
	
}


static void stk_ps_int_handle(struct stk3x1x_data *stk_data)
{
	sensors_report_1(stk_data->ps_input_dev, stk_data->ps_distance_now);
    pr_err("%s: ps input event=%d\n",
		__func__,stk_data->ps_distance_now );
	if (!(stk_data->wake_src.active)) {
            __pm_wakeup_event(&(stk_data->wake_src), jiffies_to_msecs(HZ));
    } else {
		dev_err(&stk_data->client->dev,
			"wake_lock PROX_NEAR_TO_FAR_WLOCK be locked,\
			do nothing!\n");
	}
}

void stk_als_set_new_thd(struct stk3x1x_data *stk_data, uint16_t alscode)
{
    int32_t high_thd, low_thd;
    high_thd = alscode + stk_lux2alscode(stk_data, STK_ALS_CHANGE_THD);
    low_thd = alscode - stk_lux2alscode(stk_data, STK_ALS_CHANGE_THD);

    if (high_thd >= (1 << 16))
        high_thd = (1 << 16) - 1;

    if (low_thd < 0)
        low_thd = 0;

    stk3x1x_set_als_thd_h(stk_data, (uint16_t)high_thd);
    stk3x1x_set_als_thd_l(stk_data, (uint16_t)low_thd);
}
/*
static int stk_als_int_handle(struct stk3x1x_data *stk_data,
		uint32_t als_reading)
{
    int32_t als_comperator;
    //int lux;
    stk_als_set_new_thd(stk_data, als_reading);

    if (stk_data->ir_code) {
        if ((als_reading < STK_IRC_MAX_ALS_CODE)
			&& (als_reading > STK_IRC_MIN_ALS_CODE)
			&& (stk_data->ir_code > STK_IRC_MIN_IR_CODE)) {
            als_comperator = als_reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;

            if (stk_data->ir_code > als_comperator)
                stk_data->als_correct_factor = STK_IRC_ALS_CORREC;
            else
                stk_data->als_correct_factor = 1000;
        }

        // printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_reading, stk_data->ir_code, stk_data->als_correct_factor);
        stk_data->ir_code = 0;
    }

    als_reading = als_reading * stk_data->als_correct_factor / 1000;
    stk_data->als_code_last = als_reading;
    //lux = stk_alscode2lux(stk_data, als_reading);
    sensors_report_2(stk_data->als_input_dev,
				als_reading, 0, 1, &confuse_i);
    return 0;
}
*/
static void stk_work_func(struct work_struct *work)
{
    struct stk3x1x_data *stk_data =
		container_of(work, struct stk3x1x_data, stk_work);
	int32_t ret;
	uint8_t disable_flag = 0;
	int32_t org_flag_reg;
	//uint16_t reading;
	pr_err("%s\n", __func__);
	/* mode 0x01 or 0x04 */
    org_flag_reg = stk3x1x_get_flag(stk_data);
    if (org_flag_reg < 0)
        goto err_i2c_rw;

#ifdef STK_DEBUG_PRINTF
    pr_err("%s: flag=0x%x\n", __func__, org_flag_reg);
#endif
/*
	if (org_flag_reg & STK_FLG_ALSINT_MASK) {
        disable_flag |= STK_FLG_ALSINT_MASK;
        reading = stk3x1x_get_als_reading(stk_data);
        if (reading < 0)
            goto err_i2c_rw;

        ret = stk_als_int_handle(stk_data, reading);
        if (ret < 0)
            goto err_i2c_rw;
    }
*/
	if (org_flag_reg & STK_FLG_PSINT_MASK) {
        disable_flag |= STK_FLG_PSINT_MASK;
        stk_data->ps_distance_now = (org_flag_reg & STK_FLG_NF_MASK) ? FAR_CODE : NEAR_CODE;// 1 is far and 0 is near.
        stk_ps_int_handle(stk_data);
    }

    if (disable_flag) {
        ret = stk3x1x_set_flag(stk_data, org_flag_reg, disable_flag);
        if (ret < 0)
            goto err_i2c_rw;
    }

    usleep_range(1000, 2000);
    enable_irq(stk_data->irq);
    return;
err_i2c_rw:
    sensors_report_1(stk_data->ps_input_dev, FAR_CODE);
    msleep(30);
    enable_irq(stk_data->irq);
    return;
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
    struct stk3x1x_data *pData = data;
	pr_err("%s\n", __func__);
    disable_irq_nosync(irq);
    queue_work(pData->stk_wq, &pData->stk_work);
    return IRQ_HANDLED;
}

static int stk3x1x_setup_irq(struct i2c_client *client)
{
    struct stk3x1x_data *stk_data = i2c_get_clientdata(client);
    int err = -EIO;

    err = gpio_request(stk_data->int_pin, "stk-int");
    if (err < 0) {
		pr_err("%s: gpio_request, err=%d\n", __func__, err);
        return err;
    }

    err = gpio_direction_input(stk_data->int_pin);
    if (err < 0) {
		pr_err("%s: gpio_direction_input, err=%d\n", __func__, err);
        return err;
    }

     stk_data->irq = gpio_to_irq(stk_data->int_pin);
    if (stk_data->irq <= 0) {
		pr_err("irq number is not specified, irq # = %d, int pin=%d\n", stk_data->irq, stk_data->int_pin);
        return stk_data->irq;
    }
	err = request_threaded_irq(stk_data->irq,
						NULL,
						stk_oss_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						DEVICE_NAME,
						stk_data);
    if (err < 0) {
		pr_err("%s: request_any_context_irq(%d) failed for (%d)\n", __func__, stk_data->irq, err);
        goto err_request_any_context_irq;
    }

    disable_irq(stk_data->irq);
    device_init_wakeup(&client->dev, true);
    return 0;
err_request_any_context_irq:
    gpio_free(stk_data->int_pin);
    return err;
}

static int stk3x1x_set_input_devices(struct stk3x1x_data *stk_data)
{
	int err = 0;
	if(stk_data->als_take_effect){
		stk_data->als_input_dev = input_allocate_device();
		if (stk_data->als_input_dev == NULL) {
			pr_err("%s: could not allocate als input device\n", __func__);
	        err = -ENOMEM;
	        return err;
	    }

	    stk_data->als_input_dev->name = ALS_NAME;
	    set_bit(EV_ABS, stk_data->als_input_dev->evbit);
	    input_set_abs_params(stk_data->als_input_dev,
			ABS_MISC, 0, (1 << 16) - 1, 0, 0);
	    input_set_drvdata(stk_data->als_input_dev, stk_data);
	    err = input_register_device(stk_data->als_input_dev);
	    if (err < 0) {
			pr_err("%s: can not register als input device\n", __func__);
	        goto err_als_input_register;
	    }
	}
	if(stk_data->ps_take_effect){
		stk_data->ps_input_dev = input_allocate_device();
	    if (stk_data->ps_input_dev == NULL) {
			pr_err("%s: could not allocate ps input device\n", __func__);
	        err = -ENOMEM;
	        goto err_ps_input_allocate;
	    }

		stk_data->ps_input_dev->name = PS_NAME;
	    set_bit(EV_ABS, stk_data->ps_input_dev->evbit);
	    input_set_abs_params(stk_data->ps_input_dev, ABS_DISTANCE, 0, 1023, 0, 0);
	    input_set_drvdata(stk_data->ps_input_dev, stk_data);
	    err = input_register_device(stk_data->ps_input_dev);
	    if (err < 0) {
			pr_err("%s: can not register ps input device\n", __func__);
	        goto err_ps_input_register;
	    }
	}
    return 0;
err_ps_input_register:
    input_free_device(stk_data->ps_input_dev);
err_ps_input_allocate:
    input_unregister_device(stk_data->ps_input_dev);
err_als_input_register:
    input_free_device(stk_data->als_input_dev);
    return err;
}


static int stk_ps_tune_zero_init(struct stk3x1x_data *stk_data)
{
    int32_t ret = 0;
    uint8_t w_state_reg;
    stk_data->psi_set = 0;
    stk_data->ps_stat_data[0] = 0;
    stk_data->ps_stat_data[2] = 9999;
    stk_data->ps_stat_data[1] = 0;
    stk_data->data_count = 0;
    stk_data->boot_cali = 0;
    stk_data->tune_zero_init_proc = true;
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client, STK_INT_REG, 0);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);
    ret = stk3x1x_i2c_smbus_write_byte_data(stk_data->client,
		STK_STATE_REG, w_state_reg);
    if (ret < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }

    hrtimer_start(&stk_data->ps_tune0_timer,
		stk_data->ps_tune0_delay, HRTIMER_MODE_REL);
    return 0;
}


#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void stk_init_code_threshold_table(struct stk3x1x_data *stk_data)
{
    uint32_t i, j;
    uint32_t alscode;
    code_threshold_table[0] = 0;
#ifdef STK_DEBUG_PRINTF
    printk(KERN_INFO "alscode[0]=%d\n", 0);
#endif

    for (i = 1, j = 0; i < LUX_THD_TABLE_SIZE; i++, j++) {
        alscode = stk_lux2alscode(stk_data, lux_threshold_table[j]);
        printk(KERN_INFO "alscode[%d]=%d\n", i, alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }

    code_threshold_table[i] = 0xffff;
    printk(KERN_INFO "alscode[%d]=%d\n", i, alscode);
}
#endif

static int stk3x1x_otp_read_byte_data(struct i2c_client *client,
		unsigned char command)
{
    unsigned char value;
    int err;
    err = stk3x1x_i2c_smbus_write_byte_data(client, 0x0, 0x2);
    if (err < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return err;
    }

    err = stk3x1x_i2c_smbus_write_byte_data(client, 0x90, command);
    if (err)
        return err;

    err = stk3x1x_i2c_smbus_write_byte_data(client, 0x92, 0x82);
    if (err)
        return err;

    usleep_range(2000, 4000);
    err = stk3x1x_i2c_smbus_read_byte_data(client, 0x91);
    if (err < 0)
        return err;

    value = err;
    printk("%s:read OTP 0x%x=0x%x", __func__, command, value);
    err = stk3x1x_i2c_smbus_write_byte_data(client, 0x0, 0x0);
    if (err < 0) {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return err;
    }

    return value;
}

static int32_t stk3x1x_check_pid(struct stk3x1x_data *stk_data)
{
    unsigned char value[3], pid_msb;
    int err;
    int otp25;
    stk_data->p_wv_r_bd_with_co = 0;
    stk_data->p_1x_r_bd_with_co = 0;
    stk_data->p_19_r_bc = 0;
    err = stk3x1x_i2c_read_data(stk_data->client,
		STK_PDT_ID_REG, 2, &value[0]);
    if (err < 0) {
		pr_err("%s: fail to read STK_PDT_ID_REG, ret=%d\n",
			__func__, err);
        return err;
    }

    err = stk3x1x_i2c_smbus_read_byte_data(stk_data->client, 0xE0);
	if (err < 0){
		pr_err("%s: fail to read 0xE0, ret=%d\n",
			__func__, err);
        return err;
	}

    value[2] = err;
	pr_info("%s: PID=0x%x, RID=0x%x, 0x90=0x%x\n",
		__func__, value[0], value[1], value[2]);
    stk_data->pid = value[0];

    if (value[0] == STK3311WV_PID)
        stk_data->p_wv_r_bd_with_co |= 0b100;
    else if (value[0] == STK3311X_PID)
        stk_data->p_1x_r_bd_with_co |= 0b100;
    else if (value[0] == STK33119_PID)
        stk_data->p_19_r_bc |= 0b10;

    if (value[1] == 0xC3) {
        stk_data->p_wv_r_bd_with_co |= 0b010;
        stk_data->p_1x_r_bd_with_co |= 0b010;
    } else if (value[1] == 0xC2) {
        stk_data->p_19_r_bc |= 0b01;
    }

    err = stk3x1x_otp_read_byte_data(stk_data->client, 0x25);
	if (err < 0){
		pr_err("%s: fail to read 0x25, ret=%d\n",
			__func__, err);
        return err;
	}
    otp25 = err;

    if (otp25 & 0x80)
        stk_data->p_wv_r_bd_with_co |= 0b001;

	pr_info("%s: p_wv_r_bd_with_co = 0x%x\n",
		__func__, stk_data->p_wv_r_bd_with_co);

    if (otp25 & 0x40)
        stk_data->p_1x_r_bd_with_co |= 0b001;

	pr_info("%s: p_1x_r_bd_with_co = 0x%x\n",
		__func__, stk_data->p_1x_r_bd_with_co);
	pr_info("%s: p_19_r_bc = 0x%x\n",
		__func__, stk_data->p_19_r_bc);

    if (value[0] == 0) {
		pr_info("PID=0x0, please make sure the chip is stk3x1x!\n");
        return -2;
    }

    pid_msb = value[0] & 0xF0;
    switch (pid_msb) {
    case 0x10:
    case 0x20:
    case 0x30:
        return 0;

    default:
			pr_err("%s: invalid PID(%#x)\n", __func__, value[0]);
        return -1;
    }

    return 0;
}

#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3x1x_power_ctl(struct stk3x1x_data *data, bool on)
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

		pr_info("%s: disable stk3x1x power", __func__);
		pr_info("stk3x1x_power_ctl on=%d\n", on);
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
		pr_info("%s: enable stk3x1x power", __func__);
		pr_info("stk3x1x_power_ctl on=%d\n", on);
	} else {
		pr_info("invalid!!! Power on=%d. enabled=%d\n",
			on, data->power_enabled);
	}
	data->power_enabled = on;

	return ret;
}

static int stk3x1x_power_init(struct stk3x1x_data *data, bool on)
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
										STK3X1X_VDD_MIN_UV,
										STK3X1X_VDD_MAX_UV);
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
										STK3X1X_VIO_MIN_UV,
										STK3X1X_VIO_MAX_UV);
			if (ret) {
				pr_err("Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	} else {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, STK3X1X_VDD_MAX_UV);
		regulator_put(data->vdd);
		
		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, STK3X1X_VIO_MAX_UV);
		regulator_put(data->vio);
	}

    return 0;
reg_vio_put:
    regulator_put(data->vio);
reg_vdd_set:

    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, STK3X1X_VDD_MAX_UV);

reg_vdd_put:
    regulator_put(data->vdd);
    return ret;
}
#endif  /* #ifdef STK_QUALCOMM_POWER_CTRL */

static int stk3x1x_set_wq(struct stk3x1x_data *stk_data)
{
    if(stk_data->als_take_effect){
        stk_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
        INIT_WORK(&stk_data->stk_als_work, stk_als_poll_work_func);
        hrtimer_init(&stk_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        stk_data->als_work_delay = ns_to_ktime(stk_data->als_poll_delay * NSEC_PER_MSEC);
        stk_data->als_timer.function = stk_als_timer_func;
    }	
    if(stk_data->ps_take_effect){
#ifdef STK_TUNE0
        stk_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
        INIT_WORK(&stk_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
        hrtimer_init(&stk_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        stk_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
        stk_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif
        stk_data->stk_wq = create_singlethread_workqueue("stk_wq");
        INIT_WORK(&stk_data->stk_work, stk_work_func);
    }
    return 0;
}

#ifdef CONFIG_OF
static int stk3x1x_parse_dt(struct device *dev,
		struct stk3x1x_platform_data *pdata)
{
    int rc;
    struct device_node *np = dev->of_node;
    u32 temp_val;
    pdata->int_pin =
		of_get_named_gpio_flags(np, "stk,irq-gpio", 0, &pdata->int_flags);
    if (pdata->int_pin < 0) {
        dev_err(dev, "Unable to read irq-gpio\n");
        return -EIO;
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
	rc = of_property_read_u32(np, "stk,als_fittness", &temp_val);
    if (!rc)
        pdata->als_fittness = temp_val;
    else {
        dev_err(dev, "Unable to read als_fittness\n");
        return rc;
    }
    rc = of_property_read_u32(np, "stk,als_transmittance", &temp_val);
    if (!rc)
        pdata->als_transmittance= temp_val;
    else {
        dev_err(dev, "Unable to read transmittance\n");
        return rc;
    }
    rc = of_property_read_u32(np, "stk,state-reg", &temp_val);
    if (!rc)
        pdata->state_reg = temp_val;
    else {
        dev_err(dev, "Unable to read state-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);
    if (!rc)
        pdata->psctrl_reg = (u8)temp_val;
    else {
        dev_err(dev, "Unable to read psctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);
    if (!rc)
        pdata->alsctrl_reg = (u8)temp_val;
    else {
        dev_err(dev, "Unable to read alsctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);
    if (!rc)
        pdata->ledctrl_reg = (u8)temp_val;
    else {
        dev_err(dev, "Unable to read ledctrl-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);
    if (!rc)
        pdata->wait_reg = (u8)temp_val;
    else {
        dev_err(dev, "Unable to read wait-reg\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
    if (!rc)
        pdata->ps_thd_h = (u16)temp_val;
    else {
        dev_err(dev, "Unable to read ps-thdh\n");
        return rc;
    }

    rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
    if (!rc)
        pdata->ps_thd_l = (u16)temp_val;
    else {
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
    rc = of_property_read_u32(np, "stk,als_take_effect", &temp_val);
    if (!rc)
        pdata->als_take_effect= (bool)temp_val;
    else {
        dev_err(dev, "Unable to read stk,als_take_effect set to true \n");
       	pdata->als_take_effect = true;
    }
    rc = of_property_read_u32(np, "stk,ps_take_effect", &temp_val);
    if (!rc)
        pdata->ps_take_effect= (bool)temp_val;
    else {
        dev_err(dev, "Unable to read stk,ps_take_effect set to true\n");
        pdata->ps_take_effect = true;
    }
 #ifdef CTTRACKING    
	rc = of_property_read_u32(np, "stk,ps_greasy_dirty", &temp_val);

	if (rc) {
		pdata->ps_greasy_dirty = PS_GESTURE_DIRTY;
		dev_err(dev, "ps_greasy_dirty do not set and use default value%d,\n",
				pdata->ps_greasy_dirty);
	} else {
		pdata->ps_greasy_dirty = (u16)temp_val;
	}
#endif	
    pdata->use_fir = of_property_read_bool(np, "stk,use-fir");
    return 0;
}
#else
static int stk3x1x_parse_dt(struct device *dev,
		struct stk3x1x_platform_data *pdata)
{
    return -ENODEV;
}
#endif /* !CONFIG_OF */

static int stk3x1x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    struct stk3x1x_data *stk_data = NULL;
    struct stk3x1x_platform_data *plat_data = NULL;
    int err = -ENODEV;
	uint integral_time =0;

	pr_info( "probe start, driver version = %s\n", DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("No Support for I2C_FUNC_I2C\n");
        return -ENODEV;
    }

    stk_data = kzalloc(sizeof(struct stk3x1x_data), GFP_KERNEL);
    if (!stk_data) {
		pr_err("failed to allocate stk3x1x_data\n");
        return -ENOMEM;
    }

    stk_data->client = client;
    i2c_set_clientdata(client, stk_data);
    mutex_init(&stk_data->io_lock);
    if (client->dev.of_node) {
        plat_data = devm_kzalloc(&client->dev,
			sizeof(struct stk3x1x_platform_data), GFP_KERNEL);
        if (!plat_data) {
			pr_err("Failed to allocate memory\n");
            err = -ENOMEM;
			goto quit_0;
        }
        err = stk3x1x_parse_dt(&client->dev, plat_data);
        if (err) {
			pr_err("%s: stk3x1x_parse_dt ret=%d\n", __func__, err);
            goto quit_1;
        }
    } else {
		pr_err("%s(): no device tree supported! Quit!\n", __func__);
		err = -ESRCH;
		goto quit_1;
    }
	stk3x1x_proc_plat_data(stk_data, plat_data);
	stk_data->ps_crosstalk_max = plat_data->ps_crosstalk_max;
    //stk_data->als_root = plat_data->transmittance;
    stk_data->als_root = plat_data->als_root;
    stk_data->als_numerator = plat_data->als_numerator;
    stk_data->als_fittness= plat_data->als_fittness;
    stk_data->int_pin = plat_data->int_pin;
#ifdef CTTRACKING
	stk_data->ps_greasy_dirty = plat_data->ps_greasy_dirty;
#endif
    stk_data->pdata = plat_data;
	stk_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
    stk_data->last_ps_canc = 0;
	stk_data->ps_thd_h =  0xffff;
	stk_data->ps_thd_l =  0x00;
	//默认非factory模式
	stk_data->ps_factory_flag =0;
    stk_data->als_enabled = false;
    stk_data->ps_enabled = false;
    stk_data->re_enable_als = false;
	
	//设置als第一次积分时间，该时间需要根据寄存器设置的具体值计算得出
	integral_time=((1 << (stk_data->alsctrl_reg & 0x0f)) * 185/1000) + 1;
	stk_data->als_poll_delay = integral_time * 6 / 5 + 1;
	printk("als  poll delay %llu\n",stk_data->als_poll_delay);
    if (stk_data->pdata->als_transmittance == 0) {
		pr_err("%s: Please set als_transmittance\n", __func__);
        goto quit_1;
    }
	
    stk_data->stk_lt_n_ct = plat_data->ps_thd_l;
    stk_data->stk_ht_n_ct = plat_data->ps_thd_h;

 #ifdef STK_QUALCOMM_POWER_CTRL
    err = stk3x1x_power_init(stk_data, true);
    if (err)
        goto quit_1;
    err = stk3x1x_power_ctl(stk_data, true);
    if (err)
        goto err_power_init;

    err = stk3x1x_check_pid(stk_data);
    if (err < 0)
        goto err_power_on;

    stk_data->als_enabled = false;
    stk_data->ps_enabled = false;
	err = stk3x1x_set_wq(stk_data);
    if (err < 0)
        goto err_power_on;
	err = stk3x1x_init_all_setting(client, plat_data);
	if (err < 0){
		pr_err("%s:init setting err=%d\n", __func__, err);
        goto err_setup_wq;
	}	
   
#endif


	err = stk3x1x_set_input_devices(stk_data);
    if (err < 0)
        goto err_setup_wq;

    if(stk_data->ps_take_effect){
        err = stk3x1x_setup_irq(client);
        if (err < 0)
            goto err_setup_input_device;

    }
    /* make sure everything is ok before registering the class device */
    if(stk_data->als_take_effect){
        stk_data->als_cdev = stk_light_cdev;

        stk_light_cdev.min_delay  = stk_data->als_poll_delay;
    	do_div(stk_light_cdev.min_delay,NSEC_PER_MSEC);
    	
        stk_data->als_cdev.sensors_enable = stk_als_enable_set;
        err = sensors_classdev_register(&stk_data->als_input_dev->dev,
    		&stk_data->als_cdev);
        if (err)
            goto err_stk3x1x_setup_irq;
        err = sysfs_create_group(&((stk_data->als_cdev.dev)->kobj),
    		&stk_als_attribute_group);
        if (err < 0) {
    		pr_err("%s:could not create sysfs group for als\n", __func__);
            goto err_als_cdev_regi;
        }
    }
    if(stk_data->ps_take_effect){
        stk_data->ps_cdev = stk_prox_cdev;
        stk_data->ps_cdev.sensors_enable = stk_ps_enable_set;
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

        //wake_lock_init(&stk_data->ps_wakelock, WAKE_LOCK_SUSPEND, "stk_input_wakelock");
    	wakeup_source_init(&(stk_data->wake_src), "stk_input_wakelock");
    }
	productinfo_register(PRODUCTINFO_SENSOR_ALSPS_ID,
		"stk3321", "sensortek");
	pr_err("%s: probe successfully", __func__);
    return 0;
    //device_init_wakeup(&client->dev, false);
err_ps_cdev_regi:
    if(stk_data->ps_take_effect)
	    sensors_classdev_unregister(&stk_data->ps_cdev);
err_als_class_sysfs:
    if(stk_data->als_take_effect)
	    sysfs_remove_group(&((stk_data->als_cdev.dev)->kobj), &stk_als_attribute_group);
err_als_cdev_regi:
    if(stk_data->als_take_effect)
	    sensors_classdev_unregister(&stk_data->als_cdev);
err_stk3x1x_setup_irq:
    if(stk_data->ps_take_effect){
        device_init_wakeup(&client->dev, false);
        free_irq(stk_data->irq, stk_data);
        gpio_free(stk_data->int_pin);
    }   
err_setup_input_device:
    if(stk_data->ps_take_effect){
    	input_free_device(stk_data->ps_input_dev);
    	input_unregister_device(stk_data->ps_input_dev);
    }
    if(stk_data->als_take_effect){
    	input_free_device(stk_data->als_input_dev);
    	input_unregister_device(stk_data->als_input_dev);
    }    
err_setup_wq:
    if(stk_data->als_take_effect)
		destroy_workqueue(stk_data->stk_als_wq);
    if(stk_data->ps_take_effect){    
		destroy_workqueue(stk_data->stk_ps_tune0_wq);
		destroy_workqueue(stk_data->stk_wq);
    }    

#ifdef STK_QUALCOMM_POWER_CTRL
err_power_on:
    stk3x1x_power_ctl(stk_data, false);
err_power_init:
    stk3x1x_power_init(stk_data, false);
#endif
quit_1:
	devm_kfree(&client->dev, plat_data);
quit_0:
	kfree(stk_data);
    return err;
}

static int stk3x1x_remove(struct i2c_client *client)
{
    struct stk3x1x_data *stk_data = i2c_get_clientdata(client);
    device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL
    stk3x1x_power_ctl(stk_data, false);
#endif

    sensors_classdev_unregister(&stk_data->ps_cdev);
    sensors_classdev_unregister(&stk_data->als_cdev);

#ifdef STK_QUALCOMM_POWER_CTRL
    stk3x1x_power_init(stk_data, false);
#endif
    if(stk_data->ps_take_effect){
        free_irq(stk_data->irq, stk_data);
        gpio_free(stk_data->int_pin);
        sysfs_remove_group(&stk_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
        input_unregister_device(stk_data->ps_input_dev);
#ifdef STK_TUNE0
        destroy_workqueue(stk_data->stk_ps_tune0_wq);
#endif
    //wake_lock_destroy(&stk_data->ps_wakelock);
	    wakeup_source_trash(&(stk_data->wake_src));
    }
    if(stk_data->als_take_effect){
        sysfs_remove_group(&stk_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
        input_unregister_device(stk_data->als_input_dev);
        hrtimer_try_to_cancel(&stk_data->als_timer);
        destroy_workqueue(stk_data->stk_als_wq);

    }
    mutex_destroy(&stk_data->io_lock);
    kfree(stk_data);
    return 0;
}

static int stk3x1x_suspend(struct device *dev)
{
    struct stk3x1x_data *stk_data = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);
    int err;
	sensors_marker("stk3x3x_suspend()",
		"do", "suspend", "done");
    mutex_lock(&stk_data->io_lock);
    if (stk_data->als_enabled) {
        printk(KERN_ERR "%s: Enable ALS : 0\n", __func__);
        stk3x1x_als_switch(stk_data, 0);

        //hrtimer_cancel(&stk_data->als_timer);
        //cancel_work_sync(&stk_data->stk_als_work);

        stk_data->re_enable_als = true;
    }

    if (stk_data->ps_enabled && stk_data->ps_take_effect) {
        if (device_may_wakeup(&client->dev)) {
            err = enable_irq_wake(stk_data->irq);
            if (err)
                printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, stk_data->irq, err);
        }
#ifdef CTTRACKING
	if(!stk_data->ps_factory_flag)
		hrtimer_cancel(&stk_data->ps_tune0_timer);
	
#endif
    }
    mutex_unlock(&stk_data->io_lock);
    return 0;
}

static int stk3x1x_resume(struct device *dev)
{
    struct stk3x1x_data *stk_data = dev_get_drvdata(dev);
    struct i2c_client *client = to_i2c_client(dev);
    int err;

	sensors_marker("stk3x3x_resume()",
		"do", "resume", "done");
    mutex_lock(&stk_data->io_lock);
    if (stk_data->re_enable_als) {
        printk(KERN_ERR "%s: Enable ALS : 1\n", __func__);
        stk3x1x_als_switch(stk_data, 1);
        stk_data->re_enable_als = false;
    }

    if (stk_data->ps_enabled && stk_data->ps_take_effect) {
        if (device_may_wakeup(&client->dev)) {
            err = disable_irq_wake(stk_data->irq);
            if (err)
                printk(KERN_WARNING
	                "%s: disable_irq_wake(%d) failed, err=(%d)\n",
	                __func__, stk_data->irq, err);
        }
#ifdef CTTRACKING
	if(!stk_data->ps_factory_flag)	
		hrtimer_start(&stk_data->ps_tune0_timer, stk_data->ps_tune0_delay, HRTIMER_MODE_REL);

#endif
    }

    mutex_unlock(&stk_data->io_lock);
    return 0;
}

static const struct dev_pm_ops stk3x1x_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(stk3x1x_suspend, stk3x1x_resume)
};

static const struct i2c_device_id stk_ps_id[] = {
	{"stk_ps", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{.compatible = "stk,stk3x1x",},
	{},
};

static struct i2c_driver stk_ps_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = stk_match_table,
#endif
		.pm = &stk3x1x_pm_ops,
	},
	.probe = stk3x1x_probe,
	.remove = stk3x1x_remove,
	.id_table = stk_ps_id,
};

static int __init stk3x1x_init(void)
{
	printk(KERN_ERR "enter %s", __func__);
    return i2c_add_driver(&stk_ps_driver);
}

static void __exit stk3x1x_exit(void)
{
    i2c_del_driver(&stk_ps_driver);
}

module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x1x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
