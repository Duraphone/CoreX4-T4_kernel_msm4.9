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
* AMS-TAOS TMD2725 family of devices.
*/

#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/freezer.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h> 
#include <linux/productinfo.h>

#include "A96T346_sensor_class.h"

#define SLEEP(x)	mdelay(x)

#define C_I2C_FIFO_SIZE 8

#define I2C_MASK_FLAG	(0x00ff)

#define IDLE 0
#define ACTIVE 1

#define ABOV_DEBUG 1
#define LOG_TAG "ABOV"

#if ABOV_DEBUG
#define LOG_INFO(fmt, args...)  pr_err(LOG_TAG fmt, ##args)
#define LOG_DBG(fmt, args...)	pr_err(LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)   pr_err(LOG_TAG fmt, ##args)
#else
#define LOG_INFO(fmt, args...)
#define LOG_DBG(fmt, args...)	
#define LOG_ERR(fmt, args...)  
#endif

static int last_val;
//static int mEnabled;
static int programming_done;
struct a96t346_chip *abov_sar_ptr;
#define A96T346_VDD_MIN_UV	2800000
#define A96T346_VDD_MAX_UV	2950000
#define A96T346_VIO_MIN_UV	1750000
#define A96T346_VIO_MAX_UV	1950000
//#define DEVICE_NAME "abov_sar"

static u8 checksum_h;
static u8 checksum_h_bin;
static u8 checksum_l;
static u8 checksum_l_bin;

static uint8_t confuse_i = 0;


struct sensors_classdev a96t346_sar_cdev = {
	.name = SAR_NAME,
	.vendor = "abov",
	.version = 1,
	.handle = SENSOR_HANDLE_SAR,
	.type = PYSUEDO_SENSOR_TYPE_SAR | SENSOR_TYPE_DEVICE_PRIVATE_BASE,
	.max_range = "65535.0",
	.resolution = "1.0",
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

static int abov_tk_fw_mode_enter(struct i2c_client *client);
static int abov_get_nirq_state(unsigned irq_gpio);
static int initialize(struct a96t346_chip *abov_data);




/**
 * fn static int write_register(a96t346_chip abov_data, u8 address, u8 value)
 * brief Sends a write register to the device
 * param abov_data Pointer to main parent struct
 * param address 8-bit register address
 * param value   8-bit register value to write to address
 * return Value from i2c_master_send
 */
static int write_register(struct a96t346_chip *abov_data, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (abov_data && abov_data->client) {
		i2c = abov_data->client;

		returnValue = i2c_master_send(i2c, buffer, 2);
		LOG_INFO("write_register Addr: 0x%02x Val: 0x%02x Return: %d\n",
				address, value, returnValue);
	}
	if (returnValue < 0) {
		LOG_DBG("Write_register failed!\n");
	}
	return returnValue;
}

/**
 * fn static int read_register(a96t346_chip abov_data, u8 address, u8 *value)
 * brief Reads a register's value from the device
 * param abov_data Pointer to main parent struct
 * param address 8-Bit address to read from
 * param value Pointer to 8-bit value to save register value to
 * return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int read_register(struct a96t346_chip *abov_data, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;

	if (abov_data && value && abov_data->client) {
		i2c = abov_data->client;
		returnValue = i2c_smbus_read_byte_data(i2c, address);
		LOG_INFO("read_register Addr: 0x%02x Return: 0x%02x\n",
				address, returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	LOG_INFO("read_register failed!\n");
	return -ENOMEM;
}

/**
 * detect if abov exist or not
 * return 1 if chip exist else return 0
 */
static int abov_detect(struct i2c_client *client)
{
	s32 returnValue = 0, i;
	u8 address = ABOV_VENDOR_ID_REG;
	u8 value = 0xAB;

	if (client) {
		for (i = 0; i < 3; i++) {
			returnValue = i2c_smbus_read_byte_data(client, address);
			LOG_INFO("abov read_register for %d time Addr: 0x%02x Return: 0x%02x\n",
					i, address, returnValue);
			if (returnValue >= 0) {
				if (value == returnValue) {
					LOG_INFO("abov detect success!\n");
					return 1;
				}
			}
		}
/*
		for (i = 0; i < 3; i++) {
		     if(abov_tk_fw_mode_enter(client) == 0) {
		         LOG_INFO("abov boot detect success!\n");
		         return 2;
			 }
		}*/
	}
	LOG_INFO("abov detect failed!!!\n");
	return 0;
}

static int _i2c_adapter_block_write(struct i2c_client *client, u8 *data, u8 len)
{
	u8 buffer[C_I2C_FIFO_SIZE];
	u8 left = len;
	u8 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = {
		.addr = client->addr & I2C_MASK_FLAG,
		.flags = 0,
		.buf = buffer,
	};

	if (data == NULL || len < 1) {
		LOG_ERR("Invalid : data is null or len=%d\n", len);
		return -EINVAL;
	}

	while (left > 0) {
		retry = 0;
		if (left >= C_I2C_FIFO_SIZE) {
			msg.buf = &data[offset];
			msg.len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE;
			offset += C_I2C_FIFO_SIZE;
		} else {
			msg.buf = &data[offset];
			msg.len = left;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;
			if (retry > 10) {
			    LOG_ERR("OUT : fail - addr:%#x len:%d \n", client->addr, msg.len);
				return -EIO;
			}
		}
	}
	return 0;
}

static int abov_tk_check_busy(struct i2c_client *client)
{
	int ret, count = 0;
	unsigned char val = 0x00;

	do {
		ret = i2c_master_recv(client, &val, sizeof(val));
		if (val & 0x01) {
			count++;
			if (count > 1000) {
				LOG_INFO("%s: val = 0x%x\r\n", __func__, val);
				return ret;
			}
		} else {
			break;
		}
	} while (1);

	return ret;
}

static int abov_tk_fw_write(struct i2c_client *client, unsigned char *addrH,
						unsigned char *addrL, unsigned char *val)
{
	int length = 36, ret = 0;
	unsigned char buf[40] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0x7A;
	memcpy(&buf[2], addrH, 1);
	memcpy(&buf[3], addrL, 1);
	memcpy(&buf[4], val, 32);
	ret = _i2c_adapter_block_write(client, buf, length);
	if (ret < 0) {
		LOG_ERR("Firmware write fail ...\n");
		return ret;
	}

	SLEEP(3);
	abov_tk_check_busy(client);

	return 0;
}

static int abov_tk_reset_for_bootmode(struct i2c_client *client)
{
	int ret, retry_count = 10;
	unsigned char buf[16] = {0, };

retry:
	buf[0] = 0xF0;
	buf[1] = 0xAA;
	ret = i2c_master_send(client, buf, 2);
	if (ret < 0) {
		LOG_INFO("write fail(retry:%d)\n", retry_count);
		if (retry_count-- > 0) {
			goto retry;
		}
		return -EIO;
	} else {
		LOG_INFO("success reset & boot mode\n");
		return 0;
	}
}

static int abov_tk_fw_mode_enter(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[40] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0x5B;
	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:%#x data:%#x %#x... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}
	LOG_INFO("SEND : succ - addr:%#x data:%#x %#x... ret:%d\n", client->addr, buf[0], buf[1], ret);
	SLEEP(5);

	ret = i2c_master_recv(client, buf, 1);
	if (ret < 0) {
		LOG_ERR("Enter fw downloader mode fail ...\n");
		return -EIO;
	}

	LOG_INFO("succ ... device id = %#x\n", buf[0]);

	return 0;
}

static int abov_tk_fw_mode_exit(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[40] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0xE1;
	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:%#x data:%#x %#x ... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}
	LOG_INFO("SEND : succ - addr:%#x data:%#x %#x ... ret:%d\n", client->addr, buf[0], buf[1], ret);

	return 0;
}

static int abov_tk_flash_erase(struct i2c_client *client)
{
	int ret = 0;
	unsigned char buf[16] = {0, };

	buf[0] = 0xAC;
	buf[1] = 0x2E;


	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		LOG_ERR("SEND : fail - addr:%#x data:%#x %#x ... ret:%d\n", client->addr, buf[0], buf[1], ret);
		return -EIO;
	}

	LOG_INFO("SEND : succ - addr:%#x data:%#x %#x ... ret:%d\n", client->addr, buf[0], buf[1], ret);

	return 0;
}

static int abov_tk_i2c_read_checksum(struct i2c_client *client)
{
	unsigned char checksum[6] = {0, };
	unsigned char buf[16] = {0, };
	int ret;

	checksum_h = 0;
	checksum_l = 0;

	buf[0] = 0xAC;
	buf[1] = 0x9E;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = checksum_h_bin;
	buf[5] = checksum_l_bin;
	ret = i2c_master_send(client, buf, 6);

	if (ret != 6) {
		LOG_ERR("SEND : fail - addr:%#x len:%d... ret:%d\n", client->addr, 6, ret);
		return -EIO;
	}
	SLEEP(5);

	buf[0] = 0x00;
	ret = i2c_master_send(client, buf, 1);
	if (ret != 1) {
		LOG_ERR("SEND : fail - addr:%#x data:%#x ... ret:%d\n", client->addr, buf[0], ret);
		return -EIO;
	}
	SLEEP(5);

	ret = i2c_master_recv(client, checksum, 6);
	if (ret < 0) {
		LOG_ERR("Read raw fail ... \n");
		return -EIO;
	}

	checksum_h = checksum[4];
	checksum_l = checksum[5];

	return 0;
}

static int _abov_fw_update(struct i2c_client *client, const u8 *image, u32 size)
{
	int ret, ii = 0;
	int count;
	unsigned short address;
	unsigned char addrH, addrL;
	unsigned char data[32] = {0, };

	LOG_INFO("%s: call in\r\n", __func__);

	if (abov_tk_reset_for_bootmode(client) < 0) {
		LOG_ERR("don't reset(enter boot mode)!");
		return -EIO;
	}

	SLEEP(45);

	for (ii = 0; ii < 10; ii++) {
		if (abov_tk_fw_mode_enter(client) < 0) {
			LOG_ERR("don't enter the download mode! %d", ii);
			SLEEP(40);
			continue;
		}
		break;
	}

	if (10 <= ii) {
		return -EAGAIN;
	}

	if (abov_tk_flash_erase(client) < 0) {
		LOG_ERR("don't erase flash data!");
		return -EIO;
	}

	SLEEP(1400);

	address = 0x800;
	count = size / 32;

	for (ii = 0; ii < count; ii++) {
		/* first 32byte is header */
		addrH = (unsigned char)((address >> 8) & 0xFF);
		addrL = (unsigned char)(address & 0xFF);
		memcpy(data, &image[ii * 32], 32);
		ret = abov_tk_fw_write(client, &addrH, &addrL, data);
		if (ret < 0) {
			LOG_INFO("fw_write.. ii = 0x%x err\r\n", ii);
			return ret;
		}

		address += 0x20;
		memset(data, 0, 32);
	}

	ret = abov_tk_i2c_read_checksum(client);
	ret = abov_tk_fw_mode_exit(client);
	if ((checksum_h == checksum_h_bin) && (checksum_l == checksum_l_bin)) {
		LOG_INFO("checksum successful. checksum_h:0x%02x=0x%02x && checksum_l:0x%02x=0x%02x\n",
			checksum_h, checksum_h_bin, checksum_l, checksum_l_bin);
	} else {
		LOG_INFO("checksum error. checksum_h:0x%02x!=0x%02x && checksum_l:0x%02x!=0x%02x\n",
			checksum_h, checksum_h_bin, checksum_l, checksum_l_bin);
		ret = -1;
	}
	SLEEP(100);

	return ret;
}

static int abov_fw_update(bool force)
{
	int update_loop;
	struct a96t346_chip *abov_data = abov_sar_ptr;
	struct i2c_client *client = abov_data->client;
	int rc;
	bool fw_upgrade = false;
	u8 fw_version = 0, fw_file_version = 0;
	u8 fw_modelno = 0, fw_file_modeno = 0;
	const struct firmware *fw = NULL;
	char fw_name[64] = {0};

	strlcpy(fw_name, abov_data->fw_name, 64);
	strlcat(fw_name, ".BIN", 64);
	rc = request_firmware(&fw, fw_name, &client->dev);
	if (rc < 0) {
		LOG_INFO("Request firmware failed - %s (%d)\n",
				abov_data->fw_name, rc);
		return rc;
	}
    if (force == false) {
	    read_register(abov_data, ABOV_VERSION_REG, &fw_version);
	    read_register(abov_data, ABOV_MODELNO_REG, &fw_modelno);
    }

	fw_file_modeno = fw->data[1];
	fw_file_version = fw->data[5];
	checksum_h_bin = fw->data[8];
	checksum_l_bin = fw->data[9];

	if ((force) || (fw_version < fw_file_version) || (fw_modelno != fw_file_modeno))
		fw_upgrade = true;
	else {
		LOG_INFO("Exiting fw upgrade...\n");
		fw_upgrade = false;
		rc = -EIO;
		goto rel_fw;
	}

	if (fw_upgrade) {
		for (update_loop = 0; update_loop < 10; update_loop++) {
			rc = _abov_fw_update(client, &fw->data[32], fw->size-32);
			if (rc < 0)
				LOG_INFO("retry : %d times!\n", update_loop);
			else {
				initialize(abov_data);
				break;
			}
			SLEEP(400);
		}
		if (update_loop >= 10)
			rc = -EIO;
	}
	rc = write_register(abov_data, ABOV_CTRL_MODE_REG, 0x02);
	if(rc < 0)
			LOG_ERR("%s error!", __func__);
	LOG_ERR("%s turn off sar sensor!", __func__);
rel_fw:
	release_firmware(fw);
	return rc;
}
static void a96t346_capsense_update_work(struct work_struct *work)
{
	struct a96t346_chip *abov_data = container_of(work, struct a96t346_chip, fw_update_work);

	LOG_INFO("%s: start update firmware\n", __func__);
	
	mutex_lock(&abov_data->mlock);
	abov_data->loading_fw = true;
	abov_fw_update(false);
	abov_data->loading_fw = false;
	mutex_unlock(&abov_data->mlock);

	LOG_INFO("%s: update firmware end\n", __func__);
}

static void a96t346_capsense_fore_update_work(struct work_struct *work)
{
	struct a96t346_chip *abov_data = container_of(work, struct a96t346_chip, fw_update_work);

	LOG_INFO("%s: start force update firmware\n", __func__);
	
	mutex_lock(&abov_data->mlock);
	abov_data->loading_fw = true;
	abov_fw_update(true);
	abov_data->loading_fw = false;
	mutex_unlock(&abov_data->mlock);

	LOG_INFO("%s: force update firmware end\n", __func__);

}

static ssize_t a96t346_capsense_fw_ver_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	u8 fw_version = 0;
	struct a96t346_chip *abov_data = abov_sar_ptr;

	read_register(abov_data, ABOV_VERSION_REG, &fw_version);

	return snprintf(buf, 16, "ABOV:0x%02x\n", fw_version);
}

static ssize_t a96t346_capsense_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct a96t346_chip *abov_data = abov_sar_ptr;
	unsigned long val;
	int rc;

	if (count > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	abov_data->irq_disabled = 1;
	disable_irq(abov_data->irq);
	mutex_lock(&abov_data->mlock);

	if (!abov_data->loading_fw  && val) {
		abov_data->loading_fw = true;
		abov_fw_update(false);
		abov_data->loading_fw = false;
	}
	mutex_unlock(&abov_data->mlock);

	enable_irq(abov_data->irq);
	abov_data->irq_disabled = 0;

	return count;
}
static DEVICE_ATTR(update_fw, 0660, a96t346_capsense_fw_ver_show, a96t346_capsense_update_fw_store);


/**
 * brief Perform a manual offset calibration
 * param abov_data Pointer to main parent struct
 * return Value return value from the write register
 */
static int manual_offset_calibration(struct a96t346_chip *abov_data)
{
	s32 returnValue = 0;

	returnValue = write_register(abov_data, ABOV_RECALI_REG, 0x01);
	return returnValue;
}

/**
 * fn static int read_regStat(struct a96t346_chip *abov_data)
 * brief Shortcut to read what caused interrupt.
 * details This is to keep the drivers a unified
 * function that will read whatever register(s)
 * provide information on why the interrupt was caused.
 * param abov_data Pointer to main parent struct
 * return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(struct a96t346_chip *abov_data)
{
	u8 data = 0;

	if (abov_data) {
		if (read_register(abov_data, ABOV_IRQSTAT_REG, &data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

static void a96t346_sar_report(struct a96t346_chip *abov_data, int value_1, int value_2)
{
	int value = 0;
	value |= value_1 << 0;
	value |= value_2 << 1;
	if(value != abov_data->last_state){
		abov_data->last_state = value;
		sensors_report_3(abov_data->sar_idev, value_1, value_2, 0, 0, &confuse_i);
	}else{
		LOG_INFO("%s state is same with last_state", __func__);
	}
}

/**
 * brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	struct sensors_classdev *sar_classdev = dev_get_drvdata(dev);
	struct a96t346_chip *abov_data =
		container_of(sar_classdev, struct a96t346_chip, sar_cdev);

	LOG_INFO("Reading IRQSTAT_REG\n");
	read_register(abov_data, ABOV_IRQSTAT_REG, &reg_value);
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", reg_value);
}

/* brief sysfs store function for manual calibration */
static ssize_t manual_offset_calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long val;
	struct sensors_classdev *sar_classdev = dev_get_drvdata(dev);
	struct a96t346_chip *abov_data =
		container_of(sar_classdev, struct a96t346_chip, sar_cdev);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val) {
		LOG_INFO("Performing manual_offset_calibration()\n");
		manual_offset_calibration(abov_data);
	}
	return count;
}

static DEVICE_ATTR(sar_calibrate, 0644, manual_offset_calibration_show,
		manual_offset_calibration_store);
/*
static ssize_t a96t346_reset_show(struct device *dev,
		struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", programming_done);
}
*/

static ssize_t a96t346_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct a96t346_chip *abov_data = abov_sar_ptr;

	if (!count || (abov_data == NULL))
		return -EINVAL;

	if (!strncmp(buf, "reset", 5) || !strncmp(buf, "1", 1))
		write_register(abov_data, ABOV_SOFTRESET_REG, 0x10);
	LOG_ERR("manual reset the chip!");
    last_val = 0;
	return count;
}

static DEVICE_ATTR(reset, 0220, NULL, a96t346_reset_store);

static ssize_t a96t346_reg_dump_show(struct device *dev,
  		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0, i;
	u16 diff_value_ch0 = 0,diff_value_ch1 = 0,cap_value_ch0 = 0,cap_value_ch1 = 0,cap_value_ref = 0;
	struct a96t346_chip *abov_data = abov_sar_ptr;
	char *p = buf;

	for (i = 0; i < 0x26; i++) {
		read_register(abov_data, i, &reg_value);
		p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n",
				i, reg_value);
	}

	for (i = 0x80; i < 0x8C; i++) {
		read_register(abov_data, i, &reg_value);
		p += snprintf(p, PAGE_SIZE, "(0x%02x)=0x%02x\n",
				i, reg_value);
	}
	
	/* diff value ch0*/
	read_register(abov_data, 0x1C, &reg_value);
	diff_value_ch0 = reg_value;
	diff_value_ch0 <<= 8;
	read_register(abov_data, 0x1D, &reg_value);
	diff_value_ch0 += reg_value;
		
	/* diff value ch1 */
	read_register(abov_data, 0x1E, &reg_value);
	diff_value_ch1 = reg_value;
	diff_value_ch1 <<= 8;
	read_register(abov_data, 0x1F, &reg_value);
	diff_value_ch1 += reg_value;

	/* cap value ch0 */
	read_register(abov_data, 0x20, &reg_value);
	cap_value_ch0 = reg_value;
	cap_value_ch0 <<= 8;
	read_register(abov_data, 0x21, &reg_value);
	cap_value_ch0 += reg_value;
	

	/* cap value ch1 */
	read_register(abov_data, 0x22, &reg_value);
	cap_value_ch1 = reg_value;
	cap_value_ch1 <<= 8;
	read_register(abov_data, 0x23, &reg_value);
	cap_value_ch1 += reg_value;

	/* cap value ch1 */
	read_register(abov_data, 0x24, &reg_value);
	cap_value_ref = reg_value;
	cap_value_ref <<= 8;
	read_register(abov_data, 0x25, &reg_value);
	cap_value_ref += reg_value;
	
	p += snprintf(p, PAGE_SIZE, "diff_value_ch0=%d\n",
			diff_value_ch0);
	p += snprintf(p, PAGE_SIZE, "diff_value_ch1=%d\n",
			diff_value_ch1);
	p += snprintf(p, PAGE_SIZE, "cap_value_ch0=%d\n",
			cap_value_ch0);
	p += snprintf(p, PAGE_SIZE, "cap_value_ch1=%d\n",
			cap_value_ch1);
	p += snprintf(p, PAGE_SIZE, "cap_value_ref=%d\n",
			cap_value_ref);

	return (p-buf);
}

static ssize_t a96t346_reg_dump_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct a96t346_chip *abov_data = abov_sar_ptr;
	unsigned int val, reg;
    if (sscanf(buf, "%x,%x", &reg, &val) == 2) {
		LOG_INFO("%s,reg = 0x%02x, val = 0x%02x\n",
				__func__, *(u8 *)&reg, *(u8 *)&val);
		write_register(abov_data, *((u8 *)&reg), *((u8 *)&val));
	}

	return count;
}

static DEVICE_ATTR(reg, 0660, a96t346_reg_dump_show, a96t346_reg_dump_store);

static ssize_t a96t346_ch_raw_data_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	u8 reg_value = 0;
	u16 raw_value_ch0 = 0,raw_value_ch1 = 0;
	struct a96t346_chip *abov_data = abov_sar_ptr;
/* diff value ch0*/
	read_register(abov_data, ABOV_CH0_RAW_MSB_REG, &reg_value);
	raw_value_ch0 = reg_value;
	raw_value_ch0 <<= 8;
	read_register(abov_data, ABOV_CH0_RAW_LSB_REG, &reg_value);
	raw_value_ch0 += reg_value;
		
	/* diff value ch1 */
	read_register(abov_data, ABOV_CH1_RAW_MSB_REG, &reg_value);
	raw_value_ch1 = reg_value;
	raw_value_ch1 <<= 8;
	read_register(abov_data, ABOV_CH1_RAW_LSB_REG, &reg_value);
	raw_value_ch1 += reg_value;
	
	return scnprintf(buf, PAGE_SIZE, "%d %d\n",raw_value_ch0,raw_value_ch1);
}

static DEVICE_ATTR(ch_raw_data, 0440, a96t346_ch_raw_data_show, NULL);


static ssize_t a96t346_ch_base_data_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	u8 reg_value = 0;
	u16 base_value_ch0 = 0,base_value_ch1 = 0;
	struct a96t346_chip *abov_data = abov_sar_ptr;
/* diff value ch0*/
	read_register(abov_data, ABOV_CH0_BASE_MSB_REG, &reg_value);
	base_value_ch0 = reg_value;
	base_value_ch0 <<= 8;
	read_register(abov_data, ABOV_CH0_BASE_LSB_REG, &reg_value);
	base_value_ch0 += reg_value;
		
	/* diff value ch1 */
	read_register(abov_data, ABOV_CH1_BASE_MSB_REG, &reg_value);
	base_value_ch1 = reg_value;
	base_value_ch1 <<= 8;
	read_register(abov_data, ABOV_CH1_BASE_LSB_REG, &reg_value);
	base_value_ch1 += reg_value;
	
	return scnprintf(buf, PAGE_SIZE, "%d %d\n",base_value_ch0,base_value_ch1);
}

static DEVICE_ATTR(ch_base_data, 0440, a96t346_ch_base_data_show, NULL);

static ssize_t a96t346_ch_diff_data_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	u8 reg_value = 0;
	u16 diff_value_ch0 = 0,diff_value_ch1 = 0;
	struct a96t346_chip *abov_data = abov_sar_ptr;
/* diff value ch0*/
	read_register(abov_data, ABOV_CH0_DIFF_MSB_REG, &reg_value);
	diff_value_ch0 = reg_value;
	diff_value_ch0 <<= 8;
	read_register(abov_data, ABOV_CH0_DIFF_LSB_REG, &reg_value);
	diff_value_ch0 += reg_value;
		
	/* diff value ch1 */
	read_register(abov_data, ABOV_CH1_DIFF_MSB_REG, &reg_value);
	diff_value_ch1 = reg_value;
	diff_value_ch1 <<= 8;
	read_register(abov_data, ABOV_CH1_DIFF_LSB_REG, &reg_value);
	diff_value_ch1 += reg_value;
	return scnprintf(buf, PAGE_SIZE, "%d %d\n",diff_value_ch0,diff_value_ch1);
}
static DEVICE_ATTR(ch_diff_data, 0440, a96t346_ch_diff_data_show, NULL);

static struct attribute *a96t346_sar_attributes[] = {
	&dev_attr_sar_calibrate.attr,
	&dev_attr_reset.attr,
	&dev_attr_reg.attr,
	&dev_attr_ch_raw_data.attr,
	&dev_attr_ch_base_data.attr,
	&dev_attr_ch_diff_data.attr,
	&dev_attr_update_fw.attr,
	NULL,
};
const struct attribute_group a96t346_sar_attr_group = {
	.attrs = a96t346_sar_attributes,
};


/**
 * brief  Initialize I2C config from platform data
 * param abov_data Pointer to main parent struct
 *
 */
static void hw_init(struct a96t346_chip *abov_data)
{
	int i = 0;
	
	if (abov_data) {
		while (i < abov_data->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			LOG_INFO("Going to Write Reg: 0x%02x Value: 0x%02x\n",
					abov_data->pi2c_reg[i].reg, abov_data->pi2c_reg[i].val);
			write_register(abov_data, abov_data->pi2c_reg[i].reg,
					abov_data->pi2c_reg[i].val);
			i++;
		}
	} else {
		LOG_DBG("ERROR! a96t346_chip data 0x%p\n", abov_data);
		return;
	}
}

/**
 * fn static int initialize(struct a96t346_chip *abov_data)
 * brief Performs all initialization needed to configure the device
 * param abov_data Pointer to main parent struct
 * return Last used command's return value (negative if error)
 */
static int initialize(struct a96t346_chip *abov_data)
{
	int ret;

	if (abov_data) {
		/* prepare reset by disabling any irq handling */
		abov_data->irq_disabled = 1;
		disable_irq(abov_data->irq_gpio);
		/* perform a reset */
		ret = write_register(abov_data, ABOV_SOFTRESET_REG, 0x10);
		if (ret < 0)
			goto error_exit;
		/* wait until the reset has finished by monitoring NIRQ */
		LOG_INFO("Software Reset. Waiting device back to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(300);
		/*
		 *    while(abov_data->get_nirq_low && abov_data->get_nirq_low()) { read_regStat(abov_data); }
		 */
		LOG_INFO("Device back from the reset, continuing. NIRQ = %d\n",
				abov_get_nirq_state(abov_data->irq_gpio));
		hw_init(abov_data);
		msleep(100); /* make sure everything is running */
		ret = manual_offset_calibration(abov_data);
		if (ret < 0)
			goto error_exit;
		/* re-enable interrupt handling */
		enable_irq(abov_data->irq);
		abov_data->irq_disabled = 0;

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		ret = read_regStat(abov_data);
		LOG_INFO("Exiting initialize(). current state = %d\n", ret);
		programming_done = ACTIVE;
		return 0;
	}
	return -ENOMEM;

error_exit:
	programming_done = IDLE;
	return ret;
}

static int a96t346_function_on(struct a96t346_chip *abov_data)
{
	int ret = 0;
	if(abov_data->functioned == true){
		LOG_INFO("a96t346 sar is already enabled!");
		return 0;
	}else{
		ret = initialize(abov_data);
		if(ret < 0)
			LOG_ERR("%s error!", __func__);
		abov_data->functioned = true;
	}
	return ret;
}

static int a96t346_function_off(struct a96t346_chip *abov_data)
{
	int ret = 0;
	if(abov_data->functioned == false){
		LOG_INFO("a96t346 sar is already disabled!");
		return 0;
	}else{
		ret = write_register(abov_data, ABOV_CTRL_MODE_REG, 0x02);
		if(ret < 0)
			LOG_ERR("%s error!", __func__);
		disable_irq(abov_data->irq);
		abov_data->functioned = false;
	}
	return ret;
}

static int a96t346_sar_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct a96t346_chip *abov_data =
		container_of(sensors_cdev, struct a96t346_chip, sar_cdev);
	int ret = 0;

	sensors_marker("a96t346_ps_set_enable()",
		"receive", "sar_en", enable ? "1" : "0");

	if (enable != abov_data->enabled) {
		ret = enable ?
			a96t346_function_on(abov_data) :
			a96t346_function_off(abov_data);
		abov_data->enabled = enable;
	}
	return ret;
}

static int a96t346_sar_flush(struct sensors_classdev *sensors_cdev)
{
	struct a96t346_chip *abov_data =
		container_of(sensors_cdev, struct a96t346_chip, sar_cdev);
	int value = 0;
	int value_1 = 0;
	int value_2 = 0;
	sensors_marker("a96t346_sar_flush()",
		"conducted", "#", "#");

	if (abov_data->functioned){
		value = read_regStat(abov_data);
		value_1 = value & (1 << 0);
		value_2 = value & (1 << 1);
		a96t346_sar_report(abov_data, value_1, value_2);
		SLEEP(500);
	} else {
		LOG_ERR("%s: reported while sar disabled!\n", __func__);
	}
	//sensors_report_1(abov_data->p_idev, abov_data->ps_inf.near_detected); report_3
	return 0;
}


static int abov_get_nirq_state(unsigned irq_gpio)
{
	if (irq_gpio) {
		return !gpio_get_value(irq_gpio);
	} else {
		LOG_INFO("abov irq_gpio is not set.");
		return -EINVAL;
	}
}

irqreturn_t a96t346_irq_handler(int irq, void *handle)
{
	struct a96t346_chip *abov_data = handle;
	//struct i2c_client *client = abov_data->client;
	int value = 0;
	int value_1 = 0;
	int value_2 = 0;
	sensors_marker("a96t346_irq()",
		"irq", "triggered with sar",
		abov_data->enabled ? "enabled" : "disabled");

	if (abov_data->enabled) {
		disable_irq_nosync(abov_data->irq);
		value = read_regStat(abov_data);
		value_1 = value & (1 << 0);
		value_2 = value & (1 << 1);
		a96t346_sar_report(abov_data, value_1, value_2);
		SLEEP(500);
		enable_irq(abov_data->irq);
	} else {
		LOG_ERR("%s(): sar function was not enabled!\n", __func__);
	}

	return IRQ_HANDLED;
}


static int a96t346_pinctrl_init(struct a96t346_chip *abov_data)
{
	struct i2c_client *client = abov_data->client;

	abov_data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(abov_data->pinctrl)) {
		LOG_ERR("%s(): Failed to get pinctrl\n", __func__);
		return PTR_ERR(abov_data->pinctrl);
	}

	abov_data->pin_default =
		pinctrl_lookup_state(abov_data->pinctrl, "default");
	if (IS_ERR_OR_NULL(abov_data->pin_default)) {
		LOG_ERR("%s(): Failed to look up default state\n", __func__);
		return PTR_ERR(abov_data->pin_default);
	}

	if (pinctrl_select_state(abov_data->pinctrl, abov_data->pin_default)) {
		LOG_ERR("%s(): Can't select pinctrl default state.\n", __func__);
		return -EAGAIN;
	}

	return 0;
}

static int a96t346_irq_init(struct a96t346_chip *abov_data)
{
	struct i2c_client *client = abov_data->client;
	int ret = a96t346_pinctrl_init(abov_data);
	if (ret) {
		LOG_ERR("%s(): Can't initialize pinctrl.\n", __func__);
		goto pinctrl_failed;
	}
	ret = gpio_request(abov_data->irq_gpio, "abov_int");
	if (ret < 0) {
		LOG_ERR("%s(): gpio %d request failed (%d)\n", __func__,
			abov_data->irq_gpio, ret);
		goto pinctrl_failed;
	}

	ret = gpio_direction_input(abov_data->irq_gpio);
	if (ret < 0) {
		LOG_ERR("%s(): fail to set gpio %d as input (%d)\n", __func__,
			abov_data->irq_gpio, ret);
		goto fail_free_intr_pin;
	}

	abov_data->irq = client->irq;
	ret = request_threaded_irq(abov_data->irq,
							NULL,
							a96t346_irq_handler,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							SAR_NAME,
							abov_data);
	if (ret) {
		LOG_ERR("%s(): Failed to request irq %d\n", __func__, abov_data->irq);
		goto fail_free_intr_pin;
	}
	ret = enable_irq_wake(abov_data->irq);
	if (0 != ret) {
		LOG_ERR("%s(): enable_irq_wake failed for %s.\n", __func__, SAR_NAME);
		goto fail_set_irq_wake;
	}
	disable_irq(abov_data->irq);
	return 0;
fail_set_irq_wake:
	free_irq(abov_data->irq, abov_data);
fail_free_intr_pin:
	gpio_free(abov_data->irq_gpio);
pinctrl_failed:
	return ret;
}


/**
 *fn static void abov_reg_setup_init(struct i2c_client *client)
 *brief read reg val form dts
 *      reg_array_len for regs needed change num
 *      data_array_val's format <reg val ...>
 */
 
static void abov_reg_setup_init(struct i2c_client *client)
{
	u32 data_array_len = 0;
	u32 *data_array = NULL;
	int ret, i, j;
	size_t sz;
	struct device *dev;
	struct device_node *np = client->dev.of_node;

	dev = &(client->dev);
	ret = of_property_read_u32(np, "reg_array_len", &data_array_len);
	if (ret < 0) {
		LOG_DBG("data_array_len read error");
		return;
	}

	LOG_DBG("data_array_len = %d", data_array_len);
	
	sz = data_array_len / sizeof(*data_array);
	data_array = devm_kzalloc(dev, sz * sizeof(*data_array), GFP_KERNEL);
	
	ret = of_property_read_u32_array(np, "reg_array_val",
			data_array,
			sz);
	if (ret < 0) {
		LOG_DBG("data_array_val read error, ret = %d", ret);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(abov_i2c_reg_setup); i++) {
		for (j = 0; j < data_array_len*2; j += 2) {
			if (data_array[j] == abov_i2c_reg_setup[i].reg) {
				abov_i2c_reg_setup[i].val = data_array[j+1];
				LOG_DBG("read dtsi 0x%02x:0x%02x set reg\n",
					data_array[j], data_array[j+1]);  
			}
		}
	}
	devm_kfree(dev,data_array);
}

static int a96t346_parse_dt(struct i2c_client *client,
		struct a96t346_chip *abov_data)
{
	struct device_node *np = client->dev.of_node;
	int ret;

	//client->irq = of_get_gpio(np, 0);
	abov_data->irq_gpio  = of_get_named_gpio_flags(np, "abovsar,interrupt-gpio", 0, NULL);
	

	abov_reg_setup_init(client);
	abov_data->pi2c_reg = abov_i2c_reg_setup;
	abov_data->i2c_reg_num = ARRAY_SIZE(abov_i2c_reg_setup);


	ret = of_property_read_string(np, "label", &abov_data->fw_name);
	if (ret < 0) {
		LOG_DBG("firmware name read error!\n");
		//return ret;
	}
	return 0;
}

static int a96t346_sar_constructor(struct a96t346_chip *abov_data)
{
	struct i2c_client *client = abov_data->client;
	int ret =0;
	/*
	int ret = a96t346_parse_dt(client, abov_data);
	if (0 > ret) {
		LOG_ERR("%s(): failed to parse sar dts!\n", __func__);
		goto quit_0;
	}
*/
	abov_data->sar_idev =
		sensors_construct_input_dev(&client->dev,
			SAR_NAME, 3, abov_data);
	if (abov_data->sar_idev == NULL) {
		LOG_ERR("%s(): failed to construct sar_input_dev!\n", __func__);
		goto quit_0;
	}

	/* Register  sar class */
	abov_data->sar_cdev = a96t346_sar_cdev;
	abov_data->sar_cdev.sensors_enable = a96t346_sar_set_enable;
	abov_data->sar_cdev.sensors_flush = a96t346_sar_flush;
	abov_data->sar_cdev.sensors_poll_delay = NULL;

	//ret = sensors_classdev_register(&abov_data->sar_idev->dev, &abov_data->sar_cdev);
	ret = sensors_classdev_register(&client->dev, &abov_data->sar_cdev);
	if (ret) {
		LOG_ERR("%s(): Unable to register to sensors class.sar\n", __func__);
		goto quit_1;
	}

	/* Create sar sysfs */	
	ret = sysfs_create_group(&((abov_data->sar_cdev.dev)->kobj),
							&a96t346_sar_attr_group);
	if (ret) {
		LOG_ERR("%s(): Unable to creat sysfs sar.\n", __func__);
		goto quit_2;
	}

	ret = a96t346_irq_init(abov_data);
	if (ret) {
		LOG_ERR("%s(): Init irq failed.\n", __func__);
		goto quit_3;
	}
	//wake_lock_init(&(abov_data->wlock),
	//	WAKE_LOCK_SUSPEND, DEVICE_NAME);
	wakeup_source_init(&(abov_data->wlock), SAR_NAME);
	//INIT_WORK(&abov_data->sar_work, a96t346_sar_thread);
	//abov_data->poffset = 0xFF;
	abov_data->enabled = 0;
	abov_data->functioned = false;
	return 0;
quit_3:
	sysfs_remove_group(&((abov_data->sar_cdev.dev)->kobj), &a96t346_sar_attr_group);
quit_2:
	sensors_classdev_unregister(&abov_data->sar_cdev);
quit_1:
	input_unregister_device(abov_data->sar_idev);
	//input_free_device(abov_data->sar_idev);
quit_0:	
	return ret;
}

static int a96t346_power_switch(struct a96t346_chip *data, bool on)
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

		pr_info("%s: disable a96t346 power", __func__);
		pr_info("a96t346_power_ctl on=%d\n", on);
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
		pr_info("%s: enable a96t346 power", __func__);
		pr_info("a96t346_power_ctl on=%d\n", on);
	} else {
		pr_info("invalid!!! Power on=%d. enabled=%d\n",
			on, data->power_enabled);
	}
	data->power_enabled = on;

	return ret;
}


static int a96t346_power_init(struct a96t346_chip *data, bool on)
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
						A96T346_VDD_MIN_UV,A96T346_VDD_MAX_UV);
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
										A96T346_VIO_MIN_UV,
										A96T346_VIO_MAX_UV);
			if (ret) {
				pr_err("Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	} else {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, A96T346_VDD_MAX_UV);
		regulator_put(data->vdd);
		
		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, A96T346_VIO_MAX_UV);
		regulator_put(data->vio);
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, A96T346_VDD_MAX_UV);

reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int a96t346_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	struct a96t346_chip *abov_data = NULL;
	int ret = 0;
	bool isForceUpdate = false;

	printk(KERN_ERR "\na96t346: probe()\n");

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_WORD_DATA))
	{
		ret = -EIO;
		goto init_failed;
	}

	abov_data = kzalloc(sizeof(struct a96t346_chip), GFP_KERNEL);
	if (!abov_data) {
		ret = -ENOMEM;
		goto init_failed;
	}

	if (&client->dev.of_node) {
		ret = a96t346_parse_dt(client, abov_data);
		if (ret) {
			LOG_ERR("a96t346_parse_dt() err.\n");
			goto parse_dt_failed;
		}
	} else {
		goto init_failed;
	}
	abov_sar_ptr = abov_data;
	abov_data->client = client;
	i2c_set_clientdata(client, abov_data);

	/* power on chip */
	abov_data->unpowered = true;
	ret = a96t346_power_init(abov_data, true);
    if (ret){
		
		goto err_power_init;
    }	
	
    ret = a96t346_power_switch(abov_data, true);
    if (ret){
		ret = -ENOMEM;
		goto err_power_ctl;
    }
	msleep(200);
	LOG_INFO("abov_probe() delay 200ms!\n");
	/* detect if abov exist or not */
	ret = abov_detect(client);
	if (ret == 0) {
		ret =-1;
		goto err_abov_detect;
	}

	mutex_init(&abov_data->mlock);
	
	abov_data->workqueue = create_workqueue("a96t346");
	if (NULL == abov_data->workqueue) {
		dev_err(&client->dev, "%s(): out of memory\n", __func__);
		goto wq_quit;
	}
	
	ret = a96t346_sar_constructor(abov_data);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s(): Can not construct sar function.\n", __func__);
		goto sar_construct_quit;
	}
	
	//queue_work(abov_data->workqueue, &abov_data->sar_work);
	productinfo_register(PRODUCTINFO_SENSOR_SAR_ID,
		"a96t346", "ABOV");
	abov_data->loading_fw = false;
	if (isForceUpdate == true) {	  
	    INIT_WORK(&abov_data->fw_update_work, a96t346_capsense_fore_update_work);
	} else {
		INIT_WORK(&abov_data->fw_update_work, a96t346_capsense_update_work);
	}

	schedule_work(&abov_data->fw_update_work);
	ret = write_register(abov_data, ABOV_CTRL_MODE_REG, 0x02);
		if(ret < 0)
			LOG_ERR("%s error!", __func__);
	LOG_ERR("%s turn off sar sensor!", __func__);
	dev_info(&client->dev, "Probe ok.\n");
	return 0;

sar_construct_quit:
	destroy_workqueue(abov_data->workqueue);
wq_quit:
err_abov_detect:
	a96t346_power_switch(abov_data, false);	
err_power_ctl:
	a96t346_power_init(abov_data, false);	
err_power_init:
parse_dt_failed:
	kfree(abov_data);
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);
init_failed:
	dev_err(&client->dev, "Probe failed.\n");
	return ret;
}

static int a96t346_suspend(struct device *dev)
{
	struct a96t346_chip *chip = dev_get_drvdata(dev);
	sensors_marker("a96t346_suspend()",
		"do", "suspend", "done");
	a96t346_sar_set_enable(&chip->sar_cdev, 0);
	return 0;
}

static int a96t346_resume(struct device *dev)
{
	struct a96t346_chip *chip = dev_get_drvdata(dev);
	sensors_marker("a96t346_resume()",
		"do", "resume", "done");
	if(chip->enabled == 1)
		a96t346_sar_set_enable(&chip->sar_cdev, 1);
	return 0;
}

static const struct i2c_device_id a96t346_idtable[] = {
	{ "a96t346", 0 },
	{}
};

static struct of_device_id a96t346_match_table[] = {
	{.compatible = "abov,a96t346",},
	{ },
};

static UNIVERSAL_DEV_PM_OPS(a96t346_pm_ops, a96t346_suspend, a96t346_resume, NULL);

static struct i2c_driver a96t346_driver = {
	.id_table = a96t346_idtable,
	.probe = a96t346_probe,
	.driver = {
		.name = "a96t346",
		.owner = THIS_MODULE,
		.pm = &a96t346_pm_ops,
		.of_match_table = a96t346_match_table,
	},
};

static int __init a96t346_init(void)
{
	printk(KERN_ERR "a96t346_init\n");
	return i2c_add_driver(&a96t346_driver);
}

static void __exit a96t346_exit(void)
{
	i2c_del_driver(&a96t346_driver);
}

module_init(a96t346_init);
module_exit(a96t346_exit);

MODULE_AUTHOR("wangdong <wangdong24@hisense.com>");
MODULE_DESCRIPTION("ABOV a96t346 sar sensor driver");
MODULE_LICENSE("GPL");


