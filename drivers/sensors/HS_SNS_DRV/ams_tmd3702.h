/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* proximity detection (prox), Gesture, and Beam functionality within the
* AMS-TAOS TMD3702 family of devices.
*/

#ifndef __TMD3702_H
#define __TMD3702_H

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#ifdef CONFIG_SAVE_AWAKEN_EVENT
#include <linux/awaken_sys_event.h>
#endif /* CONFIG_SAVE_AWAKEN_EVENT */

#define AMS_MUTEX_DEBUG 0
#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
		printk(KERN_ERR "%s: Mutex Lock\n", __func__); \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		printk(KERN_ERR "%s: Mutex Unlock\n", __func__); \
		mutex_unlock(m); \
	}
#else
#define AMS_MUTEX_LOCK(m) { \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		mutex_unlock(m); \
	}
#endif

#define TMD3702_PRATE_TIME_U_BASE	90
#define TMD3702_ATIME_U_BASE	2800
#define TMD3702_WTIME_U_BASE	2800
#define ALS_INTEGRATION_CYCLE	2780

#define TMD3702_ITIME_U_BASE	3500
#define TMD3702_ITIME_RAW_DATA_STEP	1024

enum tmd3702_regs {
	TMD3702_REG_ENABLE     = 0x80,
	TMD3702_REG_ATIME      = 0x81,
	TMD3702_REG_PTIME      = 0x82,
	TMD3702_REG_WTIME      = 0x83,
	TMD3702_REG_AILT       = 0x84,
	TMD3702_REG_AILT_HI    = 0x85,
	TMD3702_REG_AIHT       = 0x86,
	TMD3702_REG_AIHT_HI    = 0x87,
	TMD3702_REG_PILT       = 0x88,
	TMD3702_REG_PIHT       = 0x8A,
	TMD3702_REG_PERS       = 0x8C,
	TMD3702_REG_CFG0       = 0x8D,
	TMD3702_REG_PGCFG0     = 0x8E,
	TMD3702_REG_PGCFG1     = 0x8F,

	TMD3702_REG_CFG1       = 0x90,
	TMD3702_REG_REVID      = 0x91,
	TMD3702_REG_ID         = 0x92,
	TMD3702_REG_STATUS     = 0x93,
	TMD3702_REG_CH0DATA    = 0x94,
	TMD3702_REG_CH0DATA_HI = 0x95,
	TMD3702_REG_CH1DATA    = 0x96,
	TMD3702_REG_CH1DATA_HI = 0x97,
	TMD3702_REG_PDATA      = 0x9C,

	TMD3702_REG_ADCDATA_L  = 0x9D,
	TMD3702_REG_AUXID      = 0x9E,
	TMD3702_REG_CFG2       = 0x9F,

	TMD3702_REG_SOFTRST    = 0xA0,

	TMD3702_REG_CFG3       = 0xAB,
	TMD3702_REG_CFG4       = 0xAC,
	TMD3702_REG_CFG5       = 0xAD,
	TMD3702_REG_CFG6       = 0xAE,

	TMD3702_REG_POFFSET_L  = 0xC0,
	TMD3702_REG_POFFSET_H  = 0xC1,

	TMD3702_REG_AZ_CONFIG  = 0xD6,
	TMD3702_REG_CALIB      = 0xD7,
	TMD3702_REG_CALIBCFG   = 0xD9,
	TMD3702_REG_CALIBSTAT  = 0xDC,
	TMD3702_REG_INTENAB    = 0xDD,
};

enum tmd3702__reg {
	TMD3702_MASK_BINSRCH_TARGET = 0x70,
	TMD3702_SHIFT_BINSRCH_TARGET = 4,

	TMD3702_MASK_START_OFFSET_CALIB = 0x01,
	TMD3702_SHIFT_START_OFFSET_CALIB = 0,

	TMD3702_MASK_PROX_PERS = 0xf0,
	TMD3702_SHIFT_PROX_PERS = 4,

	TMD3702_MASK_PDRIVE = 0x1f,
	TMD3702_SHIFT_PDRIVE = 0,

	TMD3702_MASK_PGAIN = 0xC0,
	TMD3702_SHIFT_PGAIN = 6,

	TMD3702_MASK_AGAIN = 0x1F,
	TMD3702_SHIFT_AGAIN = 0,

	TMD3702_MASK_APERS = 0x0f,
	TMD3702_SHIFT_APERS = 0,

	TMD3702_MASK_WLONG = 0x04,
	TMD3702_SHIFT_WLONG = 2,

	TMD3702_MASK_POFFSET_H = 0x01,
	TMD3702_SHIFT_POFFSET_H = 0,

	TMD3702_MASK_PROX_DATA_AVG = 0x07,
	TMD3702_SHIFT_PROX_DATA_AVG = 0,

	TMD3702_MASK_PROX_AUTO_OFFSET_ADJUST = 0x08,
	TMD3702_SHIFT_PROX_AUTO_OFFSET_ADJUST = 3,
};

enum tmd3702_en_reg {
	TMD3702_PON  = (1 << 0),
	TMD3702_AEN  = (1 << 1),
	TMD3702_PEN  = (1 << 2),
	TMD3702_WEN  = (1 << 3),
	TMD3702_EN_ALL = (TMD3702_AEN | TMD3702_PEN | TMD3702_WEN),
};

enum tmd3702_status {
	TMD3702_ST_PGSAT_AMBIENT  = (1 << 0),
	TMD3702_ST_PGSAT_RELFLECT = (1 << 1),
	TMD3702_ST_ZERODET    = (1 << 2),
	TMD3702_ST_CAL_IRQ    = (1 << 3),
	TMD3702_ST_ALS_IRQ    = (1 << 4),
	TMD3702_ST_PRX_IRQ    = (1 << 5),
	TMD3702_ST_PRX_SAT    = (1 << 6),
	TMD3702_ST_ALS_SAT    = (1 << 7),
};

#define TMD3702_PS_IRQ_CHECK_MASK	(TMD3702_ST_PGSAT_AMBIENT | TMD3702_ST_PGSAT_RELFLECT | TMD3702_ST_PRX_IRQ | TMD3702_ST_PRX_SAT)
#define TMD3702_ALS_IRQ_CHECK_MASK	(TMD3702_ST_ALS_IRQ | TMD3702_ST_ALS_SAT)
#define TMD3702_USELESS_IRQ_CHECK_MASK	(TMD3702_ST_ZERODET | TMD3702_ST_CAL_IRQ)

enum tmd3702_intenab_reg {
	TMD3702_ZIEN = (1 << 2),
	TMD3702_CIEN = (1 << 3),
	TMD3702_AIEN = (1 << 4),
	TMD3702_PIEN = (1 << 5),
	TMD3702_PSIEN = (1 << 6),
	TMD3702_ASIEN = (1 << 7),
	TMD3702_IEN_ALL = (TMD3702_ZIEN | TMD3702_CIEN | TMD3702_AIEN | TMD3702_PIEN | TMD3702_PSIEN | TMD3702_ASIEN),
};
#define TMD3702_ALS_INT_MASK (~(TMD3702_ASIEN | TMD3702_AIEN))

#define MAX_REGS 256

enum tmd3702_pwr_state {
	POWER_ON,
	POWER_OFF,
	POWER_STANDBY,
};

enum tmd3702_prox_state {
	PROX_STATE_NONE = 0,
	PROX_STATE_INIT,
	PROX_STATE_CALIB,
	PROX_STATE_WAIT_AND_CALIB
};

enum tmd3702_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
	PGAIN_1        = (0 << TMD3702_SHIFT_PGAIN),
	PGAIN_2        = (1 << TMD3702_SHIFT_PGAIN),
	PGAIN_4        = (2 << TMD3702_SHIFT_PGAIN),
	PGAIN_8        = (3 << TMD3702_SHIFT_PGAIN),
	PG_PULSE_4US   = (0 << 6),
	PG_PULSE_8US   = (1 << 6),
	PG_PULSE_16US  = (2 << 6),
	PG_PULSE_32US  = (3 << 6),
};

// pldrive
#define PDRIVE_MA(p)   (((u8)((p) / 6) - 1) & 0x1f)
#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define INTEGRATION_CYCLE 2816
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define WAIT_TIME_MS(p) ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

// lux
#define INDOOR_LUX_TRIGGER		6000
#define OUTDOOR_LUX_TRIGGER		10000
#define TMD3702_MAX_LUX			0xffff
#define TMD3702_MAX_ALS_VALUE	0xffff
#define TMD3702_MIN_ALS_VALUE	10

#define TMD3702_VDD_MIN_UV 3200000
#define TMD3702_VDD_MAX_UV 3300000
#define TMD3702_VI2C_MIN_UV 1750000
#define TMD3702_VI2C_MAX_UV 1950000

struct tmd3702_lux_segment {
	u32 ch0_coef;
	u32 ch1_coef;
};

struct tmd3702_als_info {
	u16 als_ch0; // clear_raw
	u16 als_ch1; // red_raw
	u16 als_ch2; // green_raw
	u16 als_ch3; // blue_raw
	u32 cpl;
	u32 saturation;
	u16 lux;
	u16 cct;
    s16 ir;
};

struct tmd3702_prox_info {
	u16 raw;
	int near_detected;
};

struct als_threshold_info {
	int thd_h;
	int thd_l;
};

struct tmd3702_chip {
	struct i2c_client *client;
	struct mutex mlock;
    struct wakeup_source wake_src;
	struct workqueue_struct *workqueue;
	struct work_struct ps_work;

	atomic_t in_suspend;
	int wake_irq;
	unsigned int irq_gpio;
	unsigned int ps_irq_handle;
	unsigned int als_irq_handle;
	bool unpowered;
	u8 device_index;
	int als_take_effect;
	int ps_take_effect;

	u8 reg_enable;
	u8 reg_inten;

	struct input_dev *p_idev;
	struct sensors_classdev ps_cdev;
	struct tmd3702_prox_info ps_inf;
	int ps_enabled;
	bool ps_functioned;
	bool detected_change;

	u8 ps_th_min;
	u8 ps_th_max;
	u16 ps_thd_away;
	u16 ps_thd_close;
	u8 poffset;
	u8 ps_crosstalk_max;

	u8 reg_wtime;

	u8 reg_ptime;
	u8 reg_pcfg0;
	u8 ps_pulse_cnt;
	u8 ps_pulse_len;
	u8 reg_pcfg1;
	u8 ps_gain;
	u8 ps_drive;
	u8 persist;

	u8 wait_time;

	struct input_dev *a_idev;
	struct sensors_classdev als_cdev;
	struct tmd3702_als_info als_inf;
	struct delayed_work als_dwork;
	int als_enabled;
	bool als_functioned;
	struct als_threshold_info als_curr_thd;
	int range_idx;
	bool als_gain_auto;
	int als_u_delay;
	int als_raw_max;
	u8 reg_atime;
	u8 reg_cfg1;
	u8 reg_cfg6;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;

	struct regulator *vdd;
	struct regulator *vio;
	
	int als_with_irq;
	int als_irq_functioned;
	struct als_threshold_info curr_thd;
    u32 als_c_coef;
    u32 als_dgf;
};

#endif /* __TMD3702_H */
