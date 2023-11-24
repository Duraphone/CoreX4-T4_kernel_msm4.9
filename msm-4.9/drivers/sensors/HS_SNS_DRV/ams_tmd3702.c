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
* proximity detection (prox) functionality within the
* AMS-TAOS TMD3702 family of devices.
*/

#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/freezer.h>
#include <linux/regulator/consumer.h>
#include <linux/productinfo.h>

#include "ams_tmd3702.h"
#include "ams_i2c.h"

/* TMD3702 Identifiers */
static u8 const tmd3702_ids[] = {
//	ID,	AUXID,	REV
	0x10,	0x07,	0x01
};

/* TMD3702 Device Names */
static char const *tmd3702_names[] = {
	"tmd3702"
};

static char tmd3702_ps_name_base[32] = {0};
static char tmd3702_als_name_base[32] = {0};

struct sensors_classdev tmd3702_ps_cdev = {
	.vendor = "ams",
	.version = 1,
	.handle = SENSOR_HANDLE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 30000, /* in microseconds */
	.max_delay = 8393,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 1,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE | SENSOR_FLAG_WAKE_UP,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

struct sensors_classdev tmd3702_als_cdev = {
	.vendor = "ams",
	.version = 1,
	.handle = SENSOR_HANDLE_LIGHT,
	.max_range = "65535.0",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 30000, /* in microseconds */
	.max_delay = 8393,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 2,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
	.sensors_calibrate = NULL,
};

/* Registers to restore */
static u8 const restorable_regs[] = {
	TMD3702_REG_PILT,
	TMD3702_REG_PIHT,
	TMD3702_REG_PERS,
	TMD3702_REG_PGCFG0,
	TMD3702_REG_PGCFG1,
	TMD3702_REG_CFG1,
	TMD3702_REG_PTIME,
	TMD3702_REG_ATIME,
	TMD3702_REG_WTIME,
};

#define THD_MAX_H	((65536 / 8) * 7)
#define THD_MIN_LUX	(3)
#define THD_STEP_LUX	(8)

static u16 const als_gains[] = {
    0,    1,
    0,    4,
    0,    16,
    0,    64,
    128,  256,
    512,
};

struct tmd3702_lux_segment lux_segment[2];
static u32 d_factor = 0;

static uint8_t confuse_i = 0;
static int cm3702_find_a_threshold_idx_pair(struct tmd3702_chip *lpi);

static int tmd3702_read_als(struct tmd3702_chip *chip)
{
	u8 temp[8] = {0};
	int ret = ams_i2c_blk_read(chip->client,
							TMD3702_REG_CH0DATA,
							temp,
							8 * sizeof(u8));
	if (ret >= 0) {
		chip->als_inf.als_ch0 = le16_to_cpu(*((const __le16 *) &temp[0])); // clear_raw
		chip->als_inf.als_ch1 = le16_to_cpu(*((const __le16 *) &temp[2])); // red_raw
		chip->als_inf.als_ch2 = le16_to_cpu(*((const __le16 *) &temp[4])); // green_raw
		chip->als_inf.als_ch3 = le16_to_cpu(*((const __le16 *) &temp[6])); // blue_raw
		chip->als_inf.ir =
			(chip->als_inf.als_ch1 + chip->als_inf.als_ch2 +
			chip->als_inf.als_ch3 - chip->als_inf.als_ch0) >> 1;
		if (chip->als_inf.ir < 0)
			chip->als_inf.ir = 0;
		ret = 0;
	}

	return ret;
}

static int tmd3702_als_update_thresholds(struct tmd3702_chip *chip,
		int high_thd, int low_thd)
{
	int ret = 0;
	u8 u_LL, u_LH, u_HL, u_HH;
	u_LL = (u8)(low_thd & 0xFF);
	u_LH = (u8)((low_thd >> 8) & 0xFF);
	u_HL = (u8)(high_thd & 0xFF);
	u_HH = (u8)((high_thd >> 8) & 0xFF);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_AILT, u_LL);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_AILT_HI, u_LH);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_AIHT, u_HL);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_AIHT_HI, u_HH);
	dev_info(&chip->client->dev, "%s(): u_LL: 0x%02x, u_LH: 0x%02x, u_HL: 0x%02x, u_HH: 0x%02x,!\n", __func__,
		u_LL, u_LH, u_HL, u_HH);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps thresholds!\n", __func__);
	}
	return ret;
}

static void tmd3702_als_calc_mdelay(struct tmd3702_chip *chip)
{
	chip->als_u_delay = TMD3702_PRATE_TIME_U_BASE * (chip->reg_ptime + 1) +
		TMD3702_ATIME_U_BASE * (chip->reg_atime + 1) +
		TMD3702_WTIME_U_BASE * (chip->wait_time + 1);
}

static void tmd3702_als_calc_max_raw(struct tmd3702_chip *chip)
{
	chip->als_raw_max =
		TMD3702_ITIME_RAW_DATA_STEP * (chip->reg_atime + 1) - 1;
	if (chip->als_raw_max > 65535)
		chip->als_raw_max = 65535;
}

static int tmd3702_als_init_paras(struct tmd3702_chip *chip)
{
	int ret = 0;
	tmd3702_als_calc_mdelay(chip);
	tmd3702_als_calc_max_raw(chip);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ATIME, chip->reg_atime);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_CFG1, chip->reg_cfg1);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to flush als regs!\n", __func__);
	}
	return ret;
}

/**add the management of ps and als irq enable and disable sequence in time by wangdong**/
static void tmd3702_irq_enable_status_handler(struct tmd3702_chip *chip)
{
    mutex_lock(&chip->mlock);
	if (chip->als_irq_handle){
	    if (chip->als_enabled){
	        if (chip->als_irq_functioned && !chip->ps_enabled)
	            enable_irq(chip->wake_irq);
        }else{
            if (chip->als_irq_functioned && !chip->ps_enabled)
                disable_irq(chip->wake_irq);
        }
	}
	if (chip->ps_irq_handle){
	    if (chip->ps_enabled){
	        if (!chip->als_irq_functioned || !chip->als_enabled)
			    enable_irq(chip->wake_irq);
        }else{
            if (!chip->als_irq_functioned || !chip->als_enabled)
                disable_irq(chip->wake_irq);
        }
	}
	mutex_unlock(&chip->mlock);
}

static int tmd3702_als_function_on(struct tmd3702_chip *chip)
{
	int ret = 0;

	// ALS Interrupt Enable
	if (chip->als_with_irq) {
		ret = tmd3702_als_update_thresholds(chip, 0xFFFF, 0);
		chip->reg_inten |= (TMD3702_AIEN | TMD3702_ASIEN);
		ret = ams_i2c_write_direct(chip->client, TMD3702_REG_INTENAB, chip->reg_inten);
		if (0 > ret)
			dev_err(&chip->client->dev, "%s(): TMD3702_AIEN enabl failed, return = %d!\n", __func__, ret);
	}

	// Turning on ALS
	chip->reg_enable |= (TMD3702_AEN | TMD3702_PON);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ENABLE, chip->reg_enable);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to start ALS function!\n", __func__);
	} else {
		chip->als_functioned = true;
	}

	return ret;
}

static int tmd3702_als_function_off(struct tmd3702_chip *chip)
{
	int ret  = 0;

	chip->reg_enable &= (~TMD3702_AEN);
	chip->reg_enable |= TMD3702_PON;
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ENABLE, chip->reg_enable);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to cease the ALS int!\n", __func__);
	} else {
		chip->als_functioned = false;
	}
	// ALS Interrupt disable
	if (chip->als_with_irq) {
		chip->reg_inten &= (~TMD3702_AIEN);
		ret = ams_i2c_write_direct(chip->client, TMD3702_REG_INTENAB, chip->reg_inten);
		if (0 > ret)
			dev_err(&chip->client->dev, "%s(): TMD3702_AIEN disable failed, return = %d!\n", __func__, ret);
	}

	return ret;
}

static int tmd3702_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct tmd3702_chip *chip =
		container_of(sensors_cdev, struct tmd3702_chip, als_cdev);
	int ret = 0;

	sensors_marker("tmd3702_als_set_enable()",
		"receive", "als_en", enable ? "1" : "0");

	if (enable != chip->als_enabled) {
		if (enable) {
			if (false == chip->ps_functioned) {
				ret = tmd3702_als_init_paras(chip);
				ret = tmd3702_als_function_on(chip);
			}
			if (chip->als_irq_functioned) {
				ret = tmd3702_als_update_thresholds(chip, 0xFFFF, 0);
				enable_irq(chip->wake_irq);
			}
			else
			{
				queue_delayed_work(chip->workqueue, &chip->als_dwork,
					usecs_to_jiffies(chip->als_u_delay));
			}
		} else {
			cancel_delayed_work(&chip->als_dwork);
			if (false == chip->ps_functioned) {
				ret = tmd3702_als_function_off(chip);
			}
		}
		chip->als_enabled = enable;
		chip->als_irq_handle = 1;
		tmd3702_irq_enable_status_handler(chip);
		chip->als_irq_handle = 0;
	} 

	return ret;
}

static int tmd3702_get_lux(struct tmd3702_chip *chip)
{
	uint64_t ch0 = (uint64_t) chip->als_inf.als_ch0;
	uint64_t lux = 0;

	lux = ch0 * (uint64_t)chip->als_c_coef * (uint64_t)chip->als_dgf;
	lux /= (uint64_t)(chip->reg_atime * als_gains[(chip->reg_cfg1 & TMD3702_MASK_AGAIN)] * ALS_INTEGRATION_CYCLE);

	chip->als_inf.lux = (u16) lux;
	/*
	dev_err(&chip->client->dev,
		"%s(): lux = %d, ch0 = %lld.\n", __func__, chip->als_inf.lux, ch0);
	*/

	return 0;
}

static void tmd3702_als_dwork_handler(struct work_struct *work)
{
	struct tmd3702_chip *chip =
		container_of((struct delayed_work *)work,
			struct tmd3702_chip, als_dwork);
	struct i2c_client *client = chip->client;

	if (0 == chip->als_enabled) {
		dev_err(&client->dev,
			"%s(): als not enabled, do not re-schedule als_dwork!\n", __func__);
	} else {
		int ret = tmd3702_read_als(chip);
		if (0 > ret) {
			dev_err(&client->dev,
				"%s(): tmd3702_read_als() failed! Do not re-schedule als_dwork!\n",
				__func__);
		} else {
			tmd3702_get_lux(chip);
			sensors_report_2(chip->a_idev,
				chip->als_inf.lux, chip->als_inf.als_ch1,
				1, &confuse_i);

			queue_delayed_work(chip->workqueue, &chip->als_dwork,
				usecs_to_jiffies(chip->als_u_delay));
		}
	}
	return;
}

/*****************/
/* ABI Functions */
/*****************/
static ssize_t tmd3702_auto_gain_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd3702_auto_gain_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;
	chip->als_gain_auto = (value) ? true : false;
	return size;
}
static DEVICE_ATTR(als_auto_gain, 0640,
	tmd3702_auto_gain_enable_show, tmd3702_auto_gain_enable_store);

static ssize_t tmd3702_als_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->reg_cfg1 & TMD3702_MASK_AGAIN);
}

static ssize_t tmd3702_als_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	unsigned long gain;
	int rc = kstrtoul(buf, 10, &gain);
	if (rc)
		return -EINVAL;

	//chip->reg_cfg1 = gain;
	chip->reg_cfg1 = gain | 0x20;//Bit 5 must set to 1, or the VCSEL current will be 2X
	return size;
}
static DEVICE_ATTR(als_gain, 0640,
	tmd3702_als_gain_show, tmd3702_als_gain_store);

static ssize_t tmd3702_als_atime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->reg_atime);
}

static ssize_t tmd3702_als_atime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	unsigned long atime;
	int rc = kstrtoul(buf, 10, &atime);
	if (rc)
		return -EINVAL;

	chip->reg_atime = atime;
	return size;
}
static DEVICE_ATTR(als_atime, 0640,
	tmd3702_als_atime_show, tmd3702_als_atime_store);

static ssize_t tmd3702_als_coef_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_c_coef);
}

static ssize_t tmd3702_als_coef_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	unsigned long coef;
	int rc = kstrtoul(buf, 10, &coef);
	if (rc)
		return -EINVAL;

	chip->als_c_coef = coef;
	return size;
}
static DEVICE_ATTR(als_coef, 0640,
	tmd3702_als_coef_show, tmd3702_als_coef_store);

static ssize_t tmd3702_als_itime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->reg_atime);
}

static ssize_t tmd3702_als_itime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);
	long itime = 0;
	int rc = kstrtoul(buf, 10, &itime);
	if (rc) {
		rc = -EINVAL;
	} else {
		chip->reg_atime = (u8)itime;
	}

	return rc ? rc : size;
}
static DEVICE_ATTR(als_Itime, 0640,
	tmd3702_als_itime_show, tmd3702_als_itime_store);

static ssize_t tmd3702_als_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);

	u8 rEnable = 0;
	u8 rAtime = 0;
	u8 rWtime = 0;
	u8 rCfg1 = 0;
	u8 rStatus = 0;
	u8 rCfg2 = 0;
	u8 rCfg3 = 0;
	u8 rIntenab = 0;
	u8 u_LL = 0;
	u8 u_LH = 0;
	u8 u_HL = 0;
	u8 u_HH = 0;

	ams_i2c_read(chip->client, TMD3702_REG_ENABLE, &rEnable);
	ams_i2c_read(chip->client, TMD3702_REG_ATIME, &rAtime);
	ams_i2c_read(chip->client, TMD3702_REG_WTIME, &rWtime);
	ams_i2c_read(chip->client, TMD3702_REG_CFG1, &rCfg1);
	ams_i2c_read(chip->client, TMD3702_REG_STATUS, &rStatus);
	ams_i2c_read(chip->client, TMD3702_REG_CFG2, &rCfg2);
	ams_i2c_read(chip->client, TMD3702_REG_CFG3, &rCfg3);
	ams_i2c_read(chip->client, TMD3702_REG_INTENAB, &rIntenab);
	
	ams_i2c_read(chip->client, TMD3702_REG_AILT, &u_LL);
	ams_i2c_read(chip->client, TMD3702_REG_AILT_HI, &u_LH);
	ams_i2c_read(chip->client, TMD3702_REG_AIHT, &u_HL);
	ams_i2c_read(chip->client, TMD3702_REG_AIHT_HI, &u_HH);

	return snprintf(buf, PAGE_SIZE,
		"ENABLE =   %2x\nATIME  =   %2x\nWTIME  =   %2x\n"
		"CFG1   =   %2x\nSTATUS =   %2x\nCFG2   =   %2x\n"
		"CFG3   =   %2x\nINTENAB=   %2x\nu_LH   =   %2x\n"
		"u_LL   =   %2x\nu_HH   =   %2x\nu_HL   =   %2x\n",
			rEnable, rAtime, rWtime,
			rCfg1, rStatus, rCfg2,
			rCfg3, rIntenab,
			u_LH, u_LL, u_HH, u_HL);
}
static DEVICE_ATTR(als_regs, 0440, tmd3702_als_regs_show, NULL);

static ssize_t tmd3702_als_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *als_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(als_cdev, struct tmd3702_chip, als_cdev);

	tmd3702_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "LUX: %d CH0: %d CH1:%d\n",
		chip->als_inf.lux,chip->als_inf.als_ch0, chip->als_inf.als_ch1);
}
static DEVICE_ATTR(als_adc, 0440, tmd3702_als_adc_show, NULL);

static ssize_t with_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *lpi =
		container_of(sensors_cdev, struct tmd3702_chip, als_cdev);
	return snprintf(buf, 20, "%s\n",
		(1 == lpi->als_with_irq) ? "Yes" : "No");
}
static DEVICE_ATTR(with_irq, 0440, with_irq_show, NULL);

static ssize_t irq_functioned_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *lpi = container_of(sensors_cdev,
			struct tmd3702_chip, als_cdev);
	return snprintf(buf, 20, "%d\n", lpi->als_irq_functioned);
}

static ssize_t irq_functioned_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *lpi = container_of(sensors_cdev,
			struct tmd3702_chip, als_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;

	lpi->als_irq_functioned = (temp == 0) ? 0 : 1;
	return size;
}
static DEVICE_ATTR(irq_functioned, 0640,
		irq_functioned_show, irq_functioned_store);

struct attribute *tmd3702_als_attributes[] = {
	&dev_attr_with_irq.attr,
	&dev_attr_irq_functioned.attr,
	&dev_attr_als_Itime.attr,
	&dev_attr_als_gain.attr,
	&dev_attr_als_atime.attr,
	&dev_attr_als_coef.attr,
	&dev_attr_als_auto_gain.attr,
	&dev_attr_als_regs.attr,
	&dev_attr_als_adc.attr,
    NULL
};

const struct attribute_group tmd3702_als_attr_group = {
	.attrs = tmd3702_als_attributes,
};

static int tmd3702_parse_als_dt(struct device *dev,
		struct tmd3702_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;

	int rc = of_property_read_u32(np, "als,extend_num", &tmp);
	if (rc) {
		dev_err(dev,
			"%s(): Unable to read extend_num! Treat as 1 ALS sensor!\n",
			__func__);
		tmp = 0;
	}

	if (tmp > 0) {
		// Find a extend als sensor now, try to parse the description.
		const char *position;
		rc = of_property_read_string_index(np, "als,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "%s(): position can not be parsed!\n", __func__);
			goto quit_parse;
		}
		sensors_construct_extend_name(tmd3702_als_name_base,
			LIGHT_NAME, position, tmp);
		tmd3702_als_cdev.type =
			sensors_construct_extend_type_num(SENSOR_TYPE_LIGHT,
			position, tmp);
		dev_err(dev, "%s(): get ALS dev type: %d\n",
			__func__, tmd3702_als_cdev.type);
	} else {
		strncpy(tmd3702_als_name_base, LIGHT_NAME, strlen(LIGHT_NAME));
		tmd3702_als_cdev.type = SENSOR_TYPE_LIGHT;
	}
	tmd3702_als_cdev.name = tmd3702_als_name_base;
	dev_err(dev, "%s(): get ALS dev name: %s\n",
		__func__, tmd3702_als_cdev.name);

	/* ALS tuning value */
	rc = of_property_read_u32(np, "ams,als_time", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read als_time, use default!\n", __func__);
		chip->reg_atime = 64;
	} else {
		chip->reg_atime = tmp & 0xFF;
	}

	rc = of_property_read_u32(np, "ams,wait_time", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read wait_time, use default!\n", __func__);
		chip->reg_wtime = 32;
	} else {
		chip->reg_wtime = tmp & 0xFF;
	}

	rc = of_property_read_u32(np, "ams,als_gain", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read als_gain, use default!\n",
			__func__);
		chip->reg_cfg1 = 1 | 0x20; //Bit 5 must set to 1, or the VCSEL current will be 2X
	} else {
		chip->reg_cfg1 = tmp | 0x20; //Bit 5 must set to 1, or the VCSEL current will be 2X
	}

	rc = of_property_read_u32(np, "ams,ps_apc", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read ps_apc, use default!\n",
			__func__);
		chip->reg_cfg6 = 0x3F;//0x3F: enable apc; 0x7F: disable apc
	} else {
		chip->reg_cfg6 = tmp & 0xFF;
	}

	rc = of_property_read_u32(np, "ams,d_factor", &tmp);
	if (rc) {
		dev_err(dev, "%s(): Unable to read d_factor, use default!\n",
			__func__);
		d_factor = 581;
	} else {
		d_factor = tmp;
	}

	rc = of_property_read_u32(np, "ams,lux_seg0_ch0_coef", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,lux_seg0_ch0_coef\n");
		lux_segment[0].ch0_coef = 1000;
	} else {
		lux_segment[0].ch0_coef = tmp;
	}

	rc = of_property_read_u32(np, "ams,lux_seg0_ch1_coef", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,lux_seg0_ch1_coef\n");
		lux_segment[0].ch1_coef = 192;
	} else {
		lux_segment[0].ch1_coef = tmp;
	}

	rc = of_property_read_u32(np, "ams,lux_seg1_ch0_coef", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,lux_seg1_ch0_coef\n");
		lux_segment[1].ch0_coef = 769;
	} else {
		lux_segment[1].ch0_coef = tmp;
	}

	rc = of_property_read_u32(np, "ams,lux_seg1_ch1_coef", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,lux_seg1_ch1_coef\n");
		lux_segment[1].ch1_coef = 86;
	} else {
		lux_segment[1].ch1_coef = tmp;
	}

	rc = of_property_read_u32(np, "ams,als_c_coef", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,als_c_coef\n");
		chip->als_c_coef = 110;
	} else {
		chip->als_c_coef = tmp;
	}

	rc = of_property_read_u32(np, "ams,als_dgf", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,als_dgf\n");
		chip->als_dgf = 833;
	} else {
		chip->als_dgf = tmp;
	}

	return 0;
quit_parse:

	return rc;
}

static int tmd3702_als_constructor(struct tmd3702_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret = tmd3702_parse_als_dt(&client->dev, chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to parse als dts!\n", __func__);
		return ret;
	}
	chip->als_irq_functioned = 0;
	chip->a_idev =
		sensors_construct_input_dev(&client->dev,
			tmd3702_als_name_base, 2, chip);
	if (chip->a_idev == NULL) {
		dev_err(&client->dev,
			"%s(): failed to construct als_input_dev!\n", __func__);
		goto quit_0;
	}

	/*Register  Als class */
	chip->als_cdev = tmd3702_als_cdev;
	chip->als_cdev.sensors_enable = &tmd3702_als_set_enable;
	chip->als_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&chip->a_idev->dev, &chip->als_cdev);
	if (ret) {
		dev_err(&client->dev,
			"Unable to register to sensors class light.\n");
		goto quit_1;
    }

	ret = sysfs_create_group(&((chip->als_cdev.dev)->kobj), &tmd3702_als_attr_group);
	if (ret) {
		dev_err(&client->dev,
			"Unable to creat sysfs als.\n");
		goto quit_2;
	}

	INIT_DELAYED_WORK(&chip->als_dwork, tmd3702_als_dwork_handler);
	chip->als_enabled = 0;
	return 0;
quit_2:
	sensors_classdev_unregister(&chip->als_cdev);
quit_1:
	input_unregister_device(chip->a_idev);
	//input_free_device(chip->a_idev);
quit_0:
	return ret;
}

static void tmd3702_als_clean_up(struct tmd3702_chip *chip)
{
	sysfs_remove_group(&((chip->als_cdev.dev)->kobj), &tmd3702_als_attr_group);
	sensors_classdev_unregister(&chip->als_cdev);
	input_unregister_device(chip->a_idev);
	//input_free_device(chip->a_idev);
}

/**************************/
/* General Prox Functions */
/**************************/

static int tmd3702_read_ps(struct tmd3702_chip *chip)
{
	s32 ret = i2c_smbus_read_byte_data(chip->client, TMD3702_REG_PDATA);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to read TMD3702_REG_PDATA!\n", __func__);
	} else {
		chip->ps_inf.raw = ret & 0xFF;
		ret = 0;
		dev_err(&chip->client->dev,
			"%s(): get PS raw %d!\n", __func__, chip->ps_inf.raw);
	}
	return ret;
}

static int tmd3702_ps_update_thresholds(struct tmd3702_chip *chip,
		u8 high_thd, u8 low_thd)
{
	int ret = 0;
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PILT, low_thd);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PIHT, high_thd);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps thresholds!\n", __func__);
	}
	return ret;
}
/*
static int tmd3702_ps_update_offset(struct tmd3702_chip *chip, u8 offset)
{
	int ret =
		ams_i2c_write_direct(chip->client, TMD3702_REG_POFFSET_L, offset);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps offset!\n", __func__);
	}
	return ret;
}
*/
static int tmd3702_ps_init_paras(struct tmd3702_chip *chip)
{
	int ret = 0;
	chip->reg_pcfg0 = chip->ps_pulse_cnt | (chip->ps_pulse_len << 6);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PGCFG0, chip->reg_pcfg0);
	chip->reg_pcfg1 = ((chip->ps_gain << 6) | chip->ps_drive);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PGCFG1, chip->reg_pcfg1);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PTIME, chip->reg_ptime);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_WTIME, chip->reg_wtime);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_CFG6, chip->reg_cfg6); //0x3F: enable apc; 0x7F: disable apc
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to write ps paras!\n", __func__);
	}
	return ret;
}

static void tmd3702_get_ps(struct tmd3702_chip *chip)
{
	chip->detected_change = false;
	if (chip->ps_inf.near_detected == 0) {
		// We got a Far last time, and looking for a near now.
		if (chip->ps_inf.raw > chip->ps_thd_close) {
			chip->ps_inf.near_detected = 1;
			chip->detected_change = true;
		}
	} else {
		// We got a Near last time, and looking for a Far now.
		if (chip->ps_inf.raw < chip->ps_thd_away) {
			chip->ps_inf.near_detected = 0;
			chip->detected_change = true;
		}
	}
}

static int tmd3702_clear_irq_status(struct tmd3702_chip *chip)
{
	// Simplify write 1s to clear all state in TMD3702_REG_STATUS.
	return ams_i2c_write_direct(chip->client, TMD3702_REG_STATUS, 0xFF);
}

static int tmd3702_offset_calibration(struct tmd3702_chip *chip)
{
	int ret = 0;
	int stat_reg = 0;
	int count = 0;

	chip->ps_thd_away = 0;
	chip->ps_thd_close = 0;
	ret = tmd3702_ps_update_thresholds(chip, 0xFF, 0);
	ret = tmd3702_clear_irq_status(chip);
	ret = tmd3702_ps_init_paras(chip);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PERS, (chip->persist & 0x0F));
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to init paras!\n", __func__);
		return ret;
	}

	ret = ams_i2c_write_direct(chip->client,
			TMD3702_REG_ENABLE, chip->reg_enable | (TMD3702_PEN | TMD3702_PON));
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to start ps only!\n", __func__);
		return ret;
	}

	do {
		count++;
		msleep(10);
		ret = i2c_smbus_read_word_data(chip->client, TMD3702_REG_STATUS);
		stat_reg = ret & 0xFF;
	} while ((0 == (stat_reg & TMD3702_ST_PRX_IRQ)) && (count < 10));

	ret = ams_i2c_write_direct(chip->client,
		TMD3702_REG_ENABLE,
		chip->reg_enable & (~TMD3702_PEN));
	ret = tmd3702_clear_irq_status(chip);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_PERS, chip->persist);
	dev_err(&chip->client->dev, "%s(): chip->persist %d!\n", __func__, chip->persist);
	if (0 == (stat_reg & TMD3702_ST_PRX_IRQ)) {
		dev_err(&chip->client->dev,
			"%s(): Check STATE_REG for 10 times but not set!\n", __func__);
		return -EIO;
	}

	ret = tmd3702_read_ps(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to get a ps data!\n", __func__);
		return ret;
	}

	dev_err(&chip->client->dev,
		"%s(): Notice! get ps raw = %d, read state_reg %d times!\n",
		__func__, chip->ps_inf.raw, count);

	if (chip->ps_inf.raw < chip->ps_crosstalk_max) {
		chip->poffset = chip->ps_inf.raw;
		dev_err(&chip->client->dev,
			"%s(): ps calib OK, ps_canc = %d!\n", __func__, chip->poffset);
	} else {
		dev_err(&chip->client->dev,
			"%s(): corsstalk-(%d) is larger than MAX-(%d)!\n", __func__,
			chip->ps_inf.raw, chip->ps_crosstalk_max);
	}

	if (0xFF != chip->poffset) {
		chip->ps_thd_close = chip->ps_th_max + chip->poffset;
		chip->ps_thd_away = chip->ps_th_min + chip->poffset;
		if (chip->ps_thd_close >= 255) {
			chip->ps_thd_close = 128 + (chip->ps_thd_away / 2);
			dev_err(&chip->client->dev,
				"%s(): fix ps_thd_close = %d!\n",
				__func__, chip->ps_thd_close);
		}
		dev_err(&chip->client->dev,
			"%s() end: get ps_thd_away = %d, ps_thd_close = %d!\n",
			__func__, chip->ps_thd_away, chip->ps_thd_close);
	} else {
		/* First time use prox after power on and crosstalk too large */
	}
	
	return 0;
}

void tmd3702_ps_thread(struct work_struct *work)
{
	struct tmd3702_chip *chip
		= container_of(work, struct tmd3702_chip, ps_work);
	int ret = tmd3702_offset_calibration(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): tmd3702_offset_calibration IO failed when bootup\n", __func__);
	} else if (0xFF == chip->poffset) {
		dev_err(&chip->client->dev,
			"%s(): Something cover the lens caused ps calib failed when bootup\n",
			__func__);
	}
}

static int tmd3702_ps_function_on(struct tmd3702_chip *chip)
{
	int ret = 0;
	chip->ps_inf.near_detected = 0;
	if (0 == chip->als_enabled) {
		ret = tmd3702_als_init_paras(chip);
	}

	ret = tmd3702_offset_calibration(chip);
	if (0 > ret) {
		dev_err(&chip->client->dev,
			"%s(): failed to execute offset calibration!\n", __func__);
		goto quit_0;
	}

	if (0xFF != chip->poffset) {
		ret = tmd3702_ps_update_thresholds(chip, chip->ps_thd_close, 0);
		chip->ps_functioned = true;
		chip->reg_inten |= TMD3702_PIEN;
		ret = ams_i2c_write_direct(chip->client,
				TMD3702_REG_INTENAB, chip->reg_inten);

		chip->reg_enable |= (TMD3702_PEN | TMD3702_PON);
		ret = ams_i2c_write_direct(chip->client,
			TMD3702_REG_ENABLE, chip->reg_enable);
		if (0 > ret) {
		    dev_err(&chip->client->dev,
			    "%s(): failed to write ps_enable register!\n", __func__);
			chip->ps_functioned = false;
		    goto quit_0;
	    }

		if (chip->ps_inf.raw < chip->ps_thd_close) {
			dev_err(&chip->client->dev,
				"%s(): raw = %d, thd_close = %d!\n",
				__func__, chip->ps_inf.raw, chip->ps_thd_close);
			sensors_report_1(chip->p_idev, 0);
		}

		if (0 == chip->als_enabled) {
			ret = tmd3702_als_function_on(chip);
		}

	} else {
		dev_err(&chip->client->dev,
			"%s(): Since can't got a valid ps calib, cease the ps function!\n",
			__func__);
		ret = ams_i2c_write_direct(chip->client,
			TMD3702_REG_ENABLE, chip->reg_enable);
		sensors_report_1(chip->p_idev, 0);
	}

quit_0:
	return ret;
}

static int tmd3702_ps_function_off(struct tmd3702_chip *chip)
{
	int ret = 0;

	// Disable Proximity and Proximity Interrupt
	chip->reg_enable &= (~TMD3702_PEN);
	chip->reg_enable |= TMD3702_PON;
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ENABLE, chip->reg_enable);
	chip->reg_inten &= (~TMD3702_PIEN);
	ret = ams_i2c_write_direct(chip->client, TMD3702_REG_INTENAB, chip->reg_inten);
	ret = tmd3702_clear_irq_status(chip);

	sensors_report_1(chip->p_idev, 0);
	
	chip->ps_functioned = false;
	return ret;
}

static int tmd3702_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct tmd3702_chip *chip =
		container_of(sensors_cdev, struct tmd3702_chip, ps_cdev);
	int ret = 0;

	sensors_marker("tmd3702_ps_set_enable()",
		"receive", "ps_en", enable ? "1" : "0");

	if (enable != chip->ps_enabled) {
		ret = enable ?
			tmd3702_ps_function_on(chip) :
			tmd3702_ps_function_off(chip);
		chip->ps_enabled = enable;
		chip->ps_irq_handle = 1;
		tmd3702_irq_enable_status_handler(chip);
		chip->ps_irq_handle = 0;
	}

	return ret;
}

static int tmd3702_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct tmd3702_chip *chip =
		container_of(sensors_cdev, struct tmd3702_chip, ps_cdev);
	int ret = 0;
	sensors_marker("tmd3702_ps_flush()",
		"conducted", "#", "#");

	if (chip->ps_functioned) {
		ret = tmd3702_read_ps(chip);
		tmd3702_get_ps(chip);
		dev_err(&chip->client->dev,
			"%s(): %s reported while PS enabled!\n", __func__,
			(chip->ps_inf.near_detected) ? "FAR_TO_NEAR" : "NEAR_TO_FAR");
	} else {
		dev_err(&chip->client->dev,
			"%s(): %s reported while PS disabled!\n", __func__,
			(chip->ps_inf.near_detected) ? "FAR_TO_NEAR" : "NEAR_TO_FAR");
	}
	sensors_report_1(chip->p_idev, chip->ps_inf.near_detected);

	return 0;
}

/*****************/
/* ABI Functions */
/*****************/

static ssize_t tmd3702_device_ps_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	int ret = -EPERM;
	int val = 0;
	if (chip->ps_enabled) {
		ret = tmd3702_read_ps(chip);
		val = ((chip->ps_inf.raw - chip->poffset) > 0) ?
			(chip->ps_inf.raw - chip->poffset) : 0;
	}

	return ret ? ret : snprintf(buf, PAGE_SIZE, "%d\n", val);
}
static DEVICE_ATTR(ps_data, 0440, tmd3702_device_ps_raw, NULL);

static ssize_t tmd3702_device_ps_detected(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return (chip->ps_enabled) ?
		snprintf(buf, PAGE_SIZE, "%s\n", chip->ps_inf.near_detected ? "Near" : "Far") : -EPERM;
}
static DEVICE_ATTR(ps_state, 0440, tmd3702_device_ps_detected, NULL);

static ssize_t tmd3702_ps_crosstalk_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_crosstalk_max);
}

static ssize_t tmd3702_ps_crosstalk_max_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_crosstalk_max = (u8)val;
	return size;
}
static DEVICE_ATTR(ps_crosstalk_maxthd, 0640,
	tmd3702_ps_crosstalk_max_show, tmd3702_ps_crosstalk_max_store);

static ssize_t tmd3702_ps_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_gain);
}

static ssize_t tmd3702_ps_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_gain = (u8)(val & 0x3);
	return size;
}
static DEVICE_ATTR(ps_gain, 0640,
	tmd3702_ps_gain_show, tmd3702_ps_gain_store);

static ssize_t tmd3702_ps_plen_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_pulse_len);//"%dus\n", (2 ^ (chip->ps_pulse_len)) * 4
}

static ssize_t tmd3702_ps_plen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_pulse_len = (u8)(val & 0x3);
	return size;
}
static DEVICE_ATTR(ps_plen, 0640,
	tmd3702_ps_plen_show, tmd3702_ps_plen_store);

static ssize_t tmd3702_ps_pnum_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", (chip->ps_pulse_cnt + 1));
}

static ssize_t tmd3702_ps_pnum_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_pulse_cnt = (u8)(val & 0x3F);
	return size;
}
static DEVICE_ATTR(ps_pnum, 0640,
	tmd3702_ps_pnum_show, tmd3702_ps_pnum_store);

static ssize_t tmd3702_ps_pdrv_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_drive);//"%dmA\n", (chip->ps_drive + 1) * 2
}

static ssize_t tmd3702_ps_pdrv_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->ps_drive = (u8)(val & 0xF);
	return size;
}
static DEVICE_ATTR(ps_pdrv, 0640,
	tmd3702_ps_pdrv_show, tmd3702_ps_pdrv_store);

static ssize_t tmd3702_ps_ptime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->reg_ptime);
}

static ssize_t tmd3702_ps_ptime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long val;
	int rc = kstrtoul(buf, 10, &val);
	if (rc)
		return -EINVAL;
	chip->reg_ptime = (u8)(val & 0xFF);
	return size;
}
static DEVICE_ATTR(ps_ptime, 0640,
	tmd3702_ps_ptime_show, tmd3702_ps_ptime_store);

static ssize_t tmd3702_ps_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->poffset);
}
static DEVICE_ATTR(ps_canc, 0440, tmd3702_ps_offset_show, NULL);

static ssize_t tmd3702_ps_threshold_high_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_th_max);
}

static ssize_t tmd3702_ps_threshold_high_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	if (temp <= 0) {
		return -EINVAL;
	}
	chip->ps_th_max = (temp > 255) ? 255 : temp;
	return size;
}
static DEVICE_ATTR(ps_thd_close, 0640,
	tmd3702_ps_threshold_high_show, tmd3702_ps_threshold_high_store);

static ssize_t tmd3702_ps_threshold_low_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ps_th_min);
}

static ssize_t tmd3702_ps_threshold_low_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);
	unsigned long temp;
	int rc = kstrtoul(buf, 10, &temp);
	if (rc)
		return -EINVAL;
	if (temp <= 0) {
		return -EINVAL;
	}
	chip->ps_th_min = (temp > 255) ? 255 : temp;

	return size;
}
static DEVICE_ATTR(ps_thd_away, 0640,
	tmd3702_ps_threshold_low_show, tmd3702_ps_threshold_low_store);

static ssize_t tmd3702_ps_regs_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sensors_classdev *ps_cdev = dev_get_drvdata(dev);
	struct tmd3702_chip *chip =
		container_of(ps_cdev, struct tmd3702_chip, ps_cdev);

	u8 rEnable = 0;
	u8 rPtime = 0;
	u8 rPiltl = 0;
	u8 rPihtl = 0;
	u8 rPers = 0;
	u8 rPgcfg0 = 0;
	u8 rPgcfg1 = 0;
	u8 rCfg1 = 0;
	u8 rStatus = 0;
	u8 rCfg2 = 0;
	u8 rIntenab = 0;

	ams_i2c_read(chip->client, TMD3702_REG_ENABLE, &rEnable);
	ams_i2c_read(chip->client, TMD3702_REG_PTIME, &rPtime);
	ams_i2c_read(chip->client, TMD3702_REG_PILT, &rPiltl);
	ams_i2c_read(chip->client, TMD3702_REG_PIHT, &rPihtl);
	ams_i2c_read(chip->client, TMD3702_REG_PERS, &rPers);
	ams_i2c_read(chip->client, TMD3702_REG_PGCFG0, &rPgcfg0);
	ams_i2c_read(chip->client, TMD3702_REG_PGCFG1, &rPgcfg1);
	ams_i2c_read(chip->client, TMD3702_REG_CFG1, &rCfg1);
	ams_i2c_read(chip->client, TMD3702_REG_STATUS, &rStatus);
	ams_i2c_read(chip->client, TMD3702_REG_CFG2, &rCfg2);
	ams_i2c_read(chip->client, TMD3702_REG_INTENAB, &rIntenab);

	return snprintf(buf, PAGE_SIZE,
		"ENABLE =   %2x\nPTIME  =   %2x\nPILT   =   %2x\nPIHT   =   %2x\n"
		"PERS   =   %2x\nPGCFG0 =   %2x\nPGCFG1 =   %2x\nCFG1   =   %2x\n"
		"CFG2   =   %2x\nSTATUS =   %2x\nINTENAB=   %2x\n"
		"%s\n",
			rEnable,
			rPtime,
			rPiltl,
			rPihtl,
			rPers,
			rPgcfg0,
			rPgcfg1,
			rCfg1,
			rCfg2,
			rStatus,
			rIntenab,
			chip->ps_inf.near_detected ? "Prox Near" : "Prox Far");
}
static DEVICE_ATTR(ps_regs, 0440, tmd3702_ps_regs_show, NULL);

struct attribute *tmd3702_ps_attributes[] = {
	&dev_attr_ps_state.attr,
	&dev_attr_ps_regs.attr,
	&dev_attr_ps_thd_close.attr,
	&dev_attr_ps_thd_away.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_crosstalk_maxthd.attr,
	&dev_attr_ps_gain.attr,
	&dev_attr_ps_plen.attr,
	&dev_attr_ps_pnum.attr,
	&dev_attr_ps_pdrv.attr,
	&dev_attr_ps_ptime.attr,
	NULL
};

const struct attribute_group tmd3702_ps_attr_group = {
	.attrs = tmd3702_ps_attributes,
};

static int tmd3702_irq_process(struct tmd3702_chip *chip)
{
	struct i2c_client *client = chip->client;
	bool ps_emergence = false;
	bool als_emergence = false;
	u8 status = 0;

	int ret = i2c_smbus_read_byte_data(chip->client, TMD3702_REG_STATUS);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to get TMD3702_REG_STATUS reg!\n", __func__);
		ps_emergence = (chip->ps_functioned) ? true : false;
		als_emergence = (chip->als_irq_functioned) ? true : false;
		goto err_process_0;
	}
	
	status = ret & 0xFF;
	dev_err(&client->dev, "%s(): Got irq state = 0x%02x\n", __func__, status);
	if (status == 0) {
		dev_err(&client->dev, "%s(): Nothing happend, but irq triggered!\n", __func__);
		ps_emergence = (chip->ps_functioned) ? true : false;
		als_emergence = (chip->als_irq_functioned) ? true : false;
		goto err_process_0;
	}

	// Clear the interrupts we'll process
	ret = tmd3702_clear_irq_status(chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to clear TMD3702_REG_STATUS reg!\n", __func__);
		ps_emergence = (chip->ps_functioned) ? true : false;
		als_emergence = (chip->als_irq_functioned) ? true : false;
		goto err_process_0;
	}

	// Check useless irq
	if (0 != (status & TMD3702_USELESS_IRQ_CHECK_MASK)) {
		dev_err(&client->dev, "%s(): useless irq triggered, deal with it later!\n", __func__);
	}

	if (chip->ps_functioned) {
		// Check PS irq status
		if (0 != (status & TMD3702_PS_IRQ_CHECK_MASK)) {
			// Read Prox, determine detect/release, report results
			ret = tmd3702_read_ps(chip);
			if (0 > ret) {
				dev_err(&client->dev,
					"%s(): tmd3702_read_ps() failed! Force to light on screen!\n",
					__func__);
				ps_emergence = true;
			} else {
				tmd3702_get_ps(chip);
				if (chip->detected_change) {
					dev_err(&chip->client->dev, "%s(): %s detected!\n", __func__,
						(chip->ps_inf.near_detected) ? "FAR_TO_NEAR" : "NEAR_TO_FAR");
					sensors_report_1(chip->p_idev, chip->ps_inf.near_detected);
					if (chip->ps_inf.near_detected == 1) {
						tmd3702_ps_update_thresholds(chip, 0xFF, chip->ps_thd_away);
					} else {
						tmd3702_ps_update_thresholds(chip, chip->ps_thd_close, 0);
					}
				} else {
					dev_err(&client->dev,
						"%s(): No valid ps state identified, err!\n",
						__func__);
					ps_emergence = true;
				}
			}
		}
	}

//check_als:
	if (chip->als_irq_functioned) {
		// Check ALS irq status
		if (0 != (status & TMD3702_ALS_IRQ_CHECK_MASK)) {
			if (TMD3702_ST_ALS_IRQ != (status & TMD3702_ALS_IRQ_CHECK_MASK)) {
				dev_err(&client->dev, "%s(): ALS irq triggered, due to SAT!\n", __func__);
				als_emergence = true;
			} else {
				ret = tmd3702_read_als(chip);
				if (0 > ret) {
					dev_err(&client->dev,
						"%s(): tmd3702_read_als() failed! use the last value!\n",
						__func__);
				} else {
					tmd3702_get_lux(chip);
				}

				sensors_report_2(chip->a_idev,
					chip->als_inf.lux, chip->als_inf.als_ch1,
					1, &confuse_i);

				// ALS Interrupt disable
				chip->reg_inten &= (~(TMD3702_AIEN | TMD3702_ASIEN));
				ret = ams_i2c_write_direct(chip->client, TMD3702_REG_INTENAB, chip->reg_inten);
				tmd3702_als_update_thresholds(chip, 0xFFFF, 0);
			}
		}
	}

	goto quit;

err_process_0:
	if (ps_emergence)
		sensors_report_1(chip->p_idev, 0);
	if (als_emergence)
		sensors_report_2(chip->a_idev,
			chip->als_inf.lux, chip->als_inf.als_ch1,
			1, &confuse_i);
quit:
	if (!(chip->wake_src.active)) {
		__pm_wakeup_event(&(chip->wake_src), jiffies_to_msecs(HZ));
	}
	return 0;  // we handled the interrupt
}

irqreturn_t tmd3702_irq_handler(int irq, void *handle)
{
	struct tmd3702_chip *chip = handle;
	struct i2c_client *client = chip->client;
	int ret = 0;

	sensors_marker("tmd3702_irq()", "triggered with",
		chip->ps_functioned ? "ps enabled" : "ps disabled",
		chip->als_irq_functioned ? "& als irq enabled" : "& als irq disabled");

	if (chip->ps_functioned || chip->als_irq_functioned) {
		disable_irq_nosync(chip->wake_irq);
		ret = tmd3702_irq_process(chip);
		enable_irq(chip->wake_irq);
	} else {
		dev_err(&client->dev,
			"%s(): Neither prox nor als function was enabled!\n", __func__);
	}

	return IRQ_HANDLED;
}

static int tmd3702_pinctrl_init(struct tmd3702_chip *chip)
{
	struct i2c_client *client = chip->client;

	chip->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		dev_err(&client->dev,
			"%s(): Failed to get pinctrl\n", __func__);
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default =
		pinctrl_lookup_state(chip->pinctrl, "default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		dev_err(&client->dev,
			"%s(): Failed to look up default state\n", __func__);
		return PTR_ERR(chip->pin_default);
	}

	if (pinctrl_select_state(chip->pinctrl, chip->pin_default)) {
		dev_err(&client->dev,
			"%s(): Can't select pinctrl default state.\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

static int tmd3702_irq_init(struct device *dev, struct tmd3702_chip *chip)
{
	struct i2c_client *client = chip->client;
	struct device_node *np = dev->of_node;
	int ret = 0;
	/* irq gpio */
	ret = of_get_named_gpio_flags(np, "ams,interrupt-gpio", 0, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read irq gpio\n");
		return ret;
	}
	chip->irq_gpio = ret;

	ret = tmd3702_pinctrl_init(chip);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Can't initialize pinctrl.\n", __func__);
		goto pinctrl_failed;
	}
	ret = gpio_request(chip->irq_gpio, tmd3702_ps_name_base);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): gpio %d request failed (%d)\n", __func__,
			chip->irq_gpio, ret);
		goto pinctrl_failed;
	}

	ret = gpio_direction_input(chip->irq_gpio);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): fail to set gpio %d as input (%d)\n", __func__,
			chip->irq_gpio, ret);
		goto fail_free_intr_pin;
	}

	chip->wake_irq = client->irq;
	ret = request_threaded_irq(chip->wake_irq,
							NULL,
							tmd3702_irq_handler,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							tmd3702_ps_name_base,
							chip);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Failed to request irq %d\n", __func__, chip->wake_irq);
		goto fail_free_intr_pin;
	}

	ret = enable_irq_wake(chip->wake_irq);
	if (0 != ret) {
		dev_err(&client->dev,
			"%s(): enable_irq_wake failed for %s.\n",
			__func__, tmd3702_ps_name_base);
		goto fail_set_irq_wake;
	}
	disable_irq(chip->wake_irq);
	return 0;
fail_set_irq_wake:
	free_irq(chip->wake_irq, chip);
fail_free_intr_pin:
	gpio_free(chip->irq_gpio);
pinctrl_failed:
	return ret;
}

static int tmd3702_parse_ps_dt(struct device *dev,
		struct tmd3702_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;
	int rc = of_property_read_u32(np, "ps,extend_num", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ps_extend_num! It is only one PS sensor!\n");
		tmp = 0;
	}

	if (tmp > 0) {
		// Find a extend ps sensor now, try to parse the description.
		const char *position;
		rc = of_property_read_string_index(np, "ps,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "position can not be parsed!\n");
			goto quit_parse;
		}
		sensors_construct_extend_name(tmd3702_ps_name_base,
			PROXIMITY_NAME, position, tmp);
		tmd3702_ps_cdev.type =
			sensors_construct_extend_type_num(SENSOR_TYPE_PROXIMITY,
			position, tmp);
	} else {
		strncpy(tmd3702_ps_name_base, PROXIMITY_NAME, strlen(PROXIMITY_NAME));
		tmd3702_ps_cdev.type = SENSOR_TYPE_PROXIMITY;
	}
	tmd3702_ps_cdev.name = tmd3702_ps_name_base;
	dev_err(dev, "get dev name: %s\n", tmd3702_ps_cdev.name);

	/* ps tuning data*/
	rc = of_property_read_u32(np, "ams,ps_th_max", &tmp);
	if (rc)
		dev_err(dev, "Unable to read ams,ps_th_max\n");
	else
		chip->ps_th_max = tmp & 0xFF;

	rc = of_property_read_u32(np, "ams,ps_th_min", &tmp);
	if (rc)
		dev_err(dev, "Unable to read ams,ps_th_min\n");
	else
		chip->ps_th_min = tmp & 0xFF;

	rc = of_property_read_u32(np, "ams,persist", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,persist\n");
		chip->persist = 1;
	} else {
		chip->persist = tmp;
	}

	rc = of_property_read_u32(np, "ams,ptime", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,ptime\n");
		chip->reg_ptime = 31;
	} else {
		chip->reg_ptime = tmp;
	}

	rc = of_property_read_u32(np, "ams,ps_pulse_cnt", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,ps_pulse_cnt\n");
		chip->ps_pulse_cnt = 15;
	} else {
		chip->ps_pulse_cnt = tmp;
	}

	rc = of_property_read_u32(np, "ams,ps_gain", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,ps_gain\n");
		chip->ps_gain = 0;
	} else {
		chip->ps_gain = tmp;
	}

	rc = of_property_read_u32(np, "ams,ps_drive", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,ps_drive\n");
		chip->ps_drive = 16;
	} else {
		chip->ps_drive = tmp;
	}

	rc = of_property_read_u32(np, "ams,ps_pulse_len", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read ams,ps_pulse_len\n");
		chip->ps_pulse_len = 1;
	} else {
		chip->ps_pulse_len = tmp;
	}

	rc = of_property_read_u32(np, "ams,ps_crosstalk_max", &tmp);
	if (rc) {
		chip->ps_crosstalk_max = 100;
		dev_err(dev,
			"ps_crosstalk_max do not set and use default value%d\n",
				chip->ps_crosstalk_max);
	} else {
		chip->ps_crosstalk_max = (u8)tmp;
	}
	return 0;
quit_parse:
	return rc;
}

static int tmd3702_ps_constructor(struct tmd3702_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret = tmd3702_parse_ps_dt(&client->dev, chip);
	if (0 > ret) {
		dev_err(&client->dev,
			"%s(): failed to parse ps dts!\n", __func__);
		goto quit_0;
	}

	chip->p_idev =
		sensors_construct_input_dev(&client->dev,
			tmd3702_ps_name_base, 1, chip);
	if (chip->p_idev == NULL) {
		dev_err(&client->dev,
			"%s(): failed to construct ps_input_dev!\n", __func__);
		goto quit_0;
	}

	/* Register  Prox class */
	chip->ps_cdev = tmd3702_ps_cdev;
	chip->ps_cdev.sensors_enable = tmd3702_ps_set_enable;
	chip->ps_cdev.sensors_flush = tmd3702_ps_flush;
	chip->ps_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&chip->p_idev->dev, &chip->ps_cdev);
	if (ret) {
		dev_err(&client->dev,
			"%s(): Unable to register to sensors class.proximity\n", __func__);
		goto quit_1;
	}

	/* Create Prox sysfs */	
	ret = sysfs_create_group(&((chip->ps_cdev.dev)->kobj), &tmd3702_ps_attr_group);
	if (ret) {
		dev_err(&client->dev, "%s(): Unable to creat sysfs ps.\n", __func__);
		goto quit_2;
	}

	chip->poffset = 0xFF;
	chip->ps_enabled = 0;
	chip->ps_functioned = false;

	INIT_WORK(&chip->ps_work, tmd3702_ps_thread);
	
	queue_work(chip->workqueue, &chip->ps_work);
	return 0;

quit_2:
	sensors_classdev_unregister(&chip->ps_cdev);
quit_1:
	input_unregister_device(chip->p_idev);
	//input_free_device(chip->p_idev);
quit_0:
	return ret;
}

static void tmd3702_ps_clean_up(struct tmd3702_chip *chip)
{
	//wake_lock_destroy(&(chip->wlock));
	//disable_irq_wake(chip->wake_irq);
	//disable_irq(chip->wake_irq);
	//free_irq(chip->wake_irq, chip);
	//gpio_free(chip->irq_gpio);
	sysfs_remove_group(&((chip->ps_cdev.dev)->kobj), &tmd3702_ps_attr_group);
	sensors_classdev_unregister(&chip->ps_cdev);
	input_unregister_device(chip->p_idev);
	//input_free_device(chip->p_idev);
}

/********************************************************************/
/* Validate the appropriate ams device is available for this driver */
/********************************************************************/
static int tmd3702_check_chip_id(struct tmd3702_chip *chip)
{
	s32 id = 0, rev = 0, auxid = 0;
	auxid = i2c_smbus_read_byte_data(chip->client, TMD3702_REG_AUXID);
	rev = i2c_smbus_read_byte_data(chip->client, TMD3702_REG_REVID);
	id = i2c_smbus_read_byte_data(chip->client, TMD3702_REG_ID);
	if ((0 > auxid) || (0 > rev) ||(0 > id)) {
		dev_err(&chip->client->dev,
			"%s(): failed to read out ChipIDs!\n", __func__);
		return -EIO;
	} else {
		int i = 0;
		dev_info(&chip->client->dev,
			"%s(): id = 0x%02x, aux_id = 0x%02x, rev = 0x%02x\n",
			__func__, id, auxid, rev);
		/* clear the 2 LSbits, they indicate the bus voltage */
		id &= 0xfc;
		/* clear all but fuse bits */
		rev &= 0x07;
		for (i = 0; i < ARRAY_SIZE(tmd3702_ids)/3; i++) {
			if (id == (tmd3702_ids[i * 3 + 0]))
				if (auxid == (tmd3702_ids[i * 3 + 1]))
					if (rev == (tmd3702_ids[i * 3 + 2]))
						break;
		}

		if (i < ARRAY_SIZE(tmd3702_names)) {
			dev_info(&chip->client->dev,
				"%s(): '%s rev. 0x%x' detected\n",
				__func__, tmd3702_names[i], rev);
			chip->device_index = i;
		} else {
			dev_err(&chip->client->dev,
				"%s(): not supported chip id\n", __func__);
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static int tmd3702_power_switch(struct tmd3702_chip *chip, bool on)
{
	int rc;

	if (on && chip->unpowered) {
		rc = regulator_enable(chip->vdd);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}
		rc = regulator_enable(chip->vio);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}
		chip->unpowered = false;
		msleep(80);
		dev_err(&chip->client->dev,
			"%s(): regulator switch ON.\n", __func__);
	} else if (!on && !chip->unpowered) {
		rc = regulator_disable(chip->vdd);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(chip->vio);
		if (rc) {
			dev_err(&chip->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		chip->unpowered = true;
		dev_err(&chip->client->dev,
			"%s(): regulator switch OFF.\n", __func__);
	}
	return 0;
err_vio_ena:
	regulator_disable(chip->vdd);
err_vdd_ena:
	return rc;
}

static int tmd3702_power_init(struct tmd3702_chip *chip, bool on)
{
	int rc;

	if (on) {
		chip->vdd = regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR(chip->vdd)) {
			rc = PTR_ERR(chip->vdd);
			dev_err(&chip->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}
		if (regulator_count_voltages(chip->vdd) > 0) {
			rc = regulator_set_voltage(chip->vdd,
					TMD3702_VDD_MIN_UV, TMD3702_VDD_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}
		chip->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR(chip->vio)) {
			rc = PTR_ERR(chip->vio);
			dev_err(&chip->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}
		if (regulator_count_voltages(chip->vio) > 0) {
			rc = regulator_set_voltage(chip->vio,
				TMD3702_VI2C_MIN_UV, TMD3702_VI2C_MAX_UV);
			if (rc) {
				dev_err(&chip->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
	} else {
		if (regulator_count_voltages(chip->vdd) > 0)
			regulator_set_voltage(chip->vdd, 0, TMD3702_VDD_MAX_UV);
		regulator_put(chip->vdd);
		if (regulator_count_voltages(chip->vio) > 0)
			regulator_set_voltage(chip->vio, 0, TMD3702_VI2C_MAX_UV);
		regulator_put(chip->vio);
	}
	return 0;
err_vio_set_vtg:
	regulator_put(chip->vio);
err_vio_get:
	if (regulator_count_voltages(chip->vdd) > 0)
		regulator_set_voltage(chip->vdd, 0, TMD3702_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(chip->vdd);
err_vdd_get:
	return rc;
}

static int tmd3702_parse_platform_dt(struct device *dev,
		struct tmd3702_chip *chip)
{
	struct device_node *np = dev->of_node;
	u32 tmp;
	int rc = 0;

	rc = of_property_read_u32(np, "ams,ps_take_effect", &tmp);
	if (rc) {
		dev_info(dev, "Unable to read ams,ps_take_effect, set default 1\n");
		chip->ps_take_effect = 1;
	} else {
		chip->ps_take_effect = tmp;
	}

	rc = of_property_read_u32(np, "ams,als_take_effect", &tmp);
	if (rc) {
		dev_info(dev, "Unable to read ams,als_take_effect, set default 1\n");
		chip->als_take_effect = 0;
	} else {
		chip->als_take_effect = tmp;
	}

	return 0;
}

static int tmd3702_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	struct tmd3702_chip *chip = NULL;
	int ret = 0;

	printk(KERN_ERR "\nTMD3702: probe()\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"%s(): i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

	chip = kzalloc(sizeof(struct tmd3702_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto init_failed;
	}
	chip->reg_inten = 0;
	chip->als_irq_functioned = 0;
	chip->als_with_irq = 0;

	if (&client->dev.of_node) {
		ret = tmd3702_parse_platform_dt(&client->dev, chip);
		if (ret) {
			dev_err(&client->dev, "tmd3702_parse_platform_dt() err.\n");
			goto parse_dt_failed;
		}
	} else {
		goto init_failed;
	}
	
	chip->client = client;
	i2c_set_clientdata(client, chip);

	/* power on chip */
	chip->unpowered = true;
	ret = tmd3702_power_init(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): tmd3702 power on error!\n", __func__);
		goto power_on_failed;
	}

	ret = tmd3702_power_switch(chip, true);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): tmd3702 power switch error!\n", __func__);
		goto power_switch_failed;
	}

	ret = tmd3702_check_chip_id(chip);
	if (ret < 0) {
		dev_err(&client->dev, "Not a valid chip ID.\n");
		goto verify_chip_failed;
	}

	mutex_init(&chip->mlock);

	chip->workqueue = create_workqueue("tmd3702");
	if (NULL == chip->workqueue) {
		dev_err(&client->dev, "%s(): out of memory\n", __func__);
		goto wq_quit;
	}

	if (chip->als_take_effect > 0) {
		ret = tmd3702_als_constructor(chip);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s(): Can not construct als function.\n", __func__);
			goto als_construct_quit;
		}
	}

	if (chip->ps_take_effect > 0) {
		ret = tmd3702_ps_constructor(chip);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s(): Can not construct prox function.\n", __func__);
			goto ps_construct_quit;
		}
	}

	if ((1 == chip->ps_take_effect) || (2 == chip->als_take_effect)) {
		ret = tmd3702_irq_init(&client->dev, chip);
		if (0 > ret) {
			dev_err(&client->dev, "%s(): Init irq failed.\n", __func__);
			goto irq_init_quit;
		}
#ifdef CONFIG_SAVE_AWAKEN_EVENT
		register_irqnum_and_event(chip->wake_irq, AWAKEN_EVENT_PSENSOR);
		wakeup_source_init(&(chip->wake_src), "tmd_input_wakelock");
#endif /* CONFIG_SAVE_AWAKEN_EVENT */
	}

	productinfo_register(PRODUCTINFO_SENSOR_ALSPS_ID,
		"tmd3702", "AMS");
	dev_info(&client->dev, "Probe ok.\n");
	return 0;

irq_init_quit:
	if (chip->als_take_effect > 0) {
		tmd3702_als_clean_up(chip);
	}
als_construct_quit:
	if (chip->ps_take_effect) {
		tmd3702_ps_clean_up(chip);
	}
ps_construct_quit:
	destroy_workqueue(chip->workqueue);
wq_quit:
verify_chip_failed:
	tmd3702_power_switch(chip, false);	
power_switch_failed:
	tmd3702_power_init(chip, false);
power_on_failed:
parse_dt_failed:
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);
init_failed:
	kfree(chip);
	dev_err(&client->dev, "Probe failed.\n");
	return ret;
}

static int cm3702_cal_raw_from_lux(struct tmd3702_chip *chip, int lux)
{
	int raw = lux * (chip->reg_atime * als_gains[(chip->reg_cfg1 & TMD3702_MASK_AGAIN)] * ALS_INTEGRATION_CYCLE)
			/ (chip->als_c_coef * chip->als_dgf);

	if (raw < 0)
		raw = 0;

	if (raw > 65535)
		raw = 65535;

	return raw;
}

static int cm3702_find_a_threshold_idx_pair(struct tmd3702_chip *lpi)
{
	int curr_thd_min_l = cm3702_cal_raw_from_lux(lpi, THD_MIN_LUX);
	int curr_div_root = cm3702_cal_raw_from_lux(lpi, THD_STEP_LUX);
	if (lpi->als_inf.lux > THD_MAX_H) {
		lpi->curr_thd.thd_l = THD_MAX_H;
		lpi->curr_thd.thd_h = 0xFFFF;
	} else if (lpi->als_inf.lux < curr_thd_min_l) {
		lpi->curr_thd.thd_l = 0;
		lpi->curr_thd.thd_h = curr_thd_min_l;
	} else if (lpi->als_inf.lux < curr_div_root) {
		lpi->curr_thd.thd_l = curr_thd_min_l;
		lpi->curr_thd.thd_h = cm3702_cal_raw_from_lux(lpi, lpi->als_inf.lux) + curr_div_root;
	} else {
		lpi->curr_thd.thd_l = cm3702_cal_raw_from_lux(lpi, lpi->als_inf.lux) - curr_div_root;
		lpi->curr_thd.thd_h = cm3702_cal_raw_from_lux(lpi, lpi->als_inf.lux) + curr_div_root;
	}
	dev_err(&lpi->client->dev,
			"%s(): thd_h= %d, thd_l= %d\n", __func__, lpi->curr_thd.thd_h, lpi->curr_thd.thd_l);
	return tmd3702_als_update_thresholds(lpi,
		lpi->curr_thd.thd_h, lpi->curr_thd.thd_l);
}

static int tmd3702_suspend(struct device *dev)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	dev_err(&chip->client->dev, "%s(): enter !\n", __func__);
	//atomic_set(chip->in_suspend, 1);
	sensors_marker("tmd3702_suspend()",
		"do", "suspend", "begin");
	if (chip->als_irq_functioned) {
		int ret = 0;
		disable_irq_nosync(chip->wake_irq);
		cancel_delayed_work_sync(&chip->als_dwork);
		ret = tmd3702_get_lux(chip);

		chip->reg_enable &= (~TMD3702_AEN);
		chip->reg_enable |= TMD3702_PON;
		ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ENABLE, chip->reg_enable);

		ret = cm3702_find_a_threshold_idx_pair(chip);
		// ALS Interrupt Enable
		chip->reg_inten |= TMD3702_AIEN;
		ret = ams_i2c_write_direct(chip->client, TMD3702_REG_INTENAB, chip->reg_inten);
		dev_err(&chip->client->dev, "%s(): TMD3702_AIEN enable 0x%02x!\n", __func__, chip->reg_inten);

		chip->reg_enable |= (TMD3702_AEN | TMD3702_PON);
		ret = ams_i2c_write_direct(chip->client, TMD3702_REG_ENABLE, chip->reg_enable);
	}

	sensors_marker("tmd3702_suspend()",
		"do", "suspend", "done");
	return 0;
}

static int tmd3702_resume(struct device *dev)
{
	struct tmd3702_chip *chip = dev_get_drvdata(dev);
	//atomic_set(chip->in_suspend, 0);
	if (chip->als_irq_functioned) {
		enable_irq(chip->wake_irq);
	}
	sensors_marker("tmd3702_resume()",
		"do", "resume", "done");
	return 0;
}

static const struct i2c_device_id tmd3702_idtable[] = {
	{ "tmd3702", 0 },
	{}
};

static struct of_device_id tmd3702_match_table[] = {
	{.compatible = "ams,tmd3702",},
	{ },
};

static UNIVERSAL_DEV_PM_OPS(tmd3702_pm_ops, tmd3702_suspend, tmd3702_resume, NULL);

static struct i2c_driver tmd3702_driver = {
	.id_table = tmd3702_idtable,
	.probe = tmd3702_probe,
	.driver = {
		.name = "tmd3702",
		.owner = THIS_MODULE,
		.pm = &tmd3702_pm_ops,
		.of_match_table = tmd3702_match_table,
	},
};

static int __init tmd3702_init(void)
{
	return i2c_add_driver(&tmd3702_driver);
}

static void __exit tmd3702_exit(void)
{
	i2c_del_driver(&tmd3702_driver);
}

module_init(tmd3702_init);
module_exit(tmd3702_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmd3702 ALS, Prox, Gesture, Beam, RemCon sensor driver");
MODULE_LICENSE("GPL");

