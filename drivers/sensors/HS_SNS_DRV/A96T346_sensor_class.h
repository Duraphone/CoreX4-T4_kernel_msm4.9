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
* AMS-TAOS TMD2725 family of devices.
*/

#ifndef __A96T346_H
#define __A96T346_H

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
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>


/*
 *  I2C Registers
 */
#define ABOV_IRQSTAT_REG			0x00
#define ABOV_VERSION_REG		    0x01
#define ABOV_MODELNO_REG		    0x02
#define ABOV_VENDOR_ID_REG		    0x03
#define ABOV_SOFTRESET_REG  		0x06
#define ABOV_CTRL_MODE_REG			0x07
#define ABOV_CTRL_CHANNEL_REG		0x08
#define ABOV_CH0_RAW_MSB_REG		0x14
#define ABOV_CH0_RAW_LSB_REG		0x15
#define ABOV_CH1_RAW_MSB_REG		0x16
#define ABOV_CH1_RAW_LSB_REG		0x17
#define ABOV_CH0_BASE_MSB_REG		0x18
#define ABOV_CH0_BASE_LSB_REG		0x19
#define ABOV_CH1_BASE_MSB_REG		0x1A
#define ABOV_CH1_BASE_LSB_REG		0x1B
#define ABOV_CH0_DIFF_MSB_REG		0x1C
#define ABOV_CH0_DIFF_LSB_REG		0x1D
#define ABOV_CH1_DIFF_MSB_REG		0x1E
#define ABOV_CH1_DIFF_LSB_REG		0x1F
#define ABOV_CH0_CAP_MSB_REG		0x20
#define ABOV_CH0_CAP_LSB_REG		0x21
#define ABOV_CH1_CAP_MSB_REG		0x22
#define ABOV_CH1_CAP_LSB_REG		0x23
#define ABOV_REF_CAP_MSB_REG		0x24
#define ABOV_REF_CAP_LSB_REG		0x25
#define ABOV_CTRL_TH_LEVEL_REG		0x2B
#define ABOV_RECALI_REG				0xFB

/* for I2C transmit */
struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};

typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data abov_i2c_reg_setup[] = {
	{
		.reg = ABOV_CTRL_MODE_REG,
		.val = 0x00,
	},
	{
		.reg = ABOV_CTRL_CHANNEL_REG,
		.val = 0x07,
	},
	{
		.reg = ABOV_RECALI_REG,
		.val = 0x01,
	},
};



struct a96t346_chip {
	struct i2c_client *client;
	struct mutex mlock;
	 struct wakeup_source wlock;
	struct workqueue_struct *workqueue;
	struct work_struct sar_work;

	struct work_struct fw_update_work;

	atomic_t in_suspend;
	int irq;
	unsigned int irq_gpio;
	bool unpowered;
	u8 device_index;

	u8 reg_enable;
	u8 reg_inten;
	int i2c_reg_num;
	struct smtc_reg_data *pi2c_reg;
	const char *fw_name;
	int last_state;

	char irq_disabled;

	struct input_dev *sar_idev;
	struct sensors_classdev sar_cdev;
	int enabled;
	bool functioned;
	bool detected_change;
	bool loading_fw;
	
	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;

	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
};

#endif /* __A96T346_H */

