/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/sensors.h>
#include <linux/string.h>

#define APPLY_MASK	0x00000001

#define CMD_W_L_MASK 0x00
#define CMD_W_H_MASK 0x10
#define CMD_W_H_L	0x10
#define CMD_MASK	0xF
#define DATA_MASK	0xFFFF0000
#define DATA_AXIS_SHIFT	17
#define DATA_APPLY_SHIFT	16
/*
 * CMD_GET_PARAMS(BIT, PARA, DATA) combine high 16 bit and low 16 bit
 * as one params
 */

#define CMD_GET_PARAMS(BIT, PARA, DATA)	\
	((BIT) ?	\
		((DATA) & DATA_MASK)	\
		: ((PARA) \
		| (((DATA) & DATA_MASK) >> 16)))


/*
 * CMD_DO_CAL sensor do calibrate command, when do sensor calibrate must use
 * this.
 * AXIS_X,AXIS_Y,AXIS_Z write axis params to driver like accelerometer
 * magnetometer,gyroscope etc.
 * CMD_W_THRESHOLD_H,CMD_W_THRESHOLD_L,CMD_W_BIAS write theshold and bias
 * params to proximity driver.
 * CMD_W_FACTOR,CMD_W_OFFSET write factor and offset params to light
 * sensor driver.
 * CMD_COMPLETE when one sensor receive calibrate parameters complete, it
 * must use this command to end receive the parameters and send the
 * parameters to sensor.
 */

enum {
	CMD_DO_CAL = 0x0,
	CMD_W_OFFSET_X,
	CMD_W_OFFSET_Y,
	CMD_W_OFFSET_Z,
	CMD_W_THRESHOLD_H,
	CMD_W_THRESHOLD_L,
	CMD_W_BIAS,
	CMD_W_OFFSET,
	CMD_W_FACTOR,
	CMD_W_RANGE,
	CMD_COMPLETE,
	CMD_COUNT
};

int cal_map[] = {
	0,
	offsetof(struct cal_result_t, offset_x),
	offsetof(struct cal_result_t, offset_y),
	offsetof(struct cal_result_t, offset_z),
	offsetof(struct cal_result_t, threshold_h),
	offsetof(struct cal_result_t, threshold_l),
	offsetof(struct cal_result_t, bias),
	offsetof(struct cal_result_t, offset[0]),
	offsetof(struct cal_result_t, offset[1]),
	offsetof(struct cal_result_t, offset[2]),
	offsetof(struct cal_result_t, factor),
	offsetof(struct cal_result_t, range),
};

static struct class *sensors_class;

DECLARE_RWSEM(sensors_list_lock);
LIST_HEAD(sensors_list);

static ssize_t sensors_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->name);
}

static DEVICE_ATTR(name, 0440, sensors_name_show, NULL);

static ssize_t sensors_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->vendor);
}

static DEVICE_ATTR(vendor, 0440, sensors_vendor_show, NULL);

static ssize_t sensors_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->version);
}

static DEVICE_ATTR(version, 0440, sensors_version_show, NULL);

static ssize_t sensors_handle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->handle);
}

static DEVICE_ATTR(handle, 0440, sensors_handle_show, NULL);

static ssize_t sensors_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->type);
}

static DEVICE_ATTR(type, 0440, sensors_type_show, NULL);

static ssize_t sensors_max_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->max_delay);
}

static DEVICE_ATTR(max_delay, 0440, sensors_max_delay_show, NULL);

static ssize_t sensors_flags_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->flags);
}

static DEVICE_ATTR(flags, 0440, sensors_flags_show, NULL);

static ssize_t sensors_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->max_range);
}

static DEVICE_ATTR(max_range, 0440, sensors_max_range_show, NULL);

static ssize_t sensors_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->resolution);
}

static DEVICE_ATTR(resolution, 0440, sensors_resolution_show, NULL);

static ssize_t sensors_power_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->sensor_power);
}

static DEVICE_ATTR(sensor_power, 0440, sensors_power_show, NULL);

static ssize_t sensors_min_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->min_delay);
}

static DEVICE_ATTR(min_delay, 0440, sensors_min_delay_show, NULL);

static ssize_t sensors_fifo_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			sensors_cdev->fifo_reserved_event_count);
}

static DEVICE_ATTR(fifo_reserved_event_count, 0440, sensors_fifo_event_show, NULL);

static ssize_t sensors_fifo_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			sensors_cdev->fifo_max_event_count);
}

static DEVICE_ATTR(fifo_max_event_count, 0440, sensors_fifo_max_show, NULL);

static ssize_t sensors_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if (data > 1) {
		dev_err(dev, "Invalid value of input, input=%ld\n", data);
		return -EINVAL;
	}

	if (sensors_cdev->sensors_enable == NULL) {
		dev_err(dev, "Invalid sensor class enable handle\n");
		return -EINVAL;
	}
	ret = sensors_cdev->sensors_enable(sensors_cdev, data);
	if (ret)
		return ret;

	sensors_cdev->enabled = data;
	return size;
}


static ssize_t sensors_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n",
			sensors_cdev->enabled);
}

static DEVICE_ATTR(enable, 0660, sensors_enable_show, sensors_enable_store);

static ssize_t sensors_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	/* The data unit is millisecond, the min_delay unit is microseconds. */
	if ((data * 1000) < sensors_cdev->min_delay) {
		dev_err(dev, "Invalid value of delay, delay=%ld\n", data);
		return -EINVAL;
	}
	if (sensors_cdev->sensors_poll_delay == NULL) {
		dev_err(dev, "Invalid sensor class delay handle\n");
		return -EINVAL;
	}
	ret = sensors_cdev->sensors_poll_delay(sensors_cdev, data);
	if (ret)
		return ret;

	sensors_cdev->delay_msec = data;
	return size;
}

static ssize_t sensors_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u\n",
			sensors_cdev->delay_msec);
}

static DEVICE_ATTR(poll_delay, 0660, sensors_delay_show, sensors_delay_store);

static ssize_t sensors_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	int ret;
	int Show_test[4] ;

	if (sensors_cdev->sensors_self_test == NULL) {
		dev_err(dev, "Invalid sensor class self test handle\n");
		return -EINVAL;
	}

	ret = sensors_cdev->sensors_self_test(sensors_cdev,Show_test);
	if (ret)
		dev_warn(dev, "self test failed.(%d)\n", ret);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			ret ? "fail" : "pass");
}

static DEVICE_ATTR(self_test, 0440, sensors_test_show, NULL);

static ssize_t sensors_max_latency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	unsigned long latency;
	int ret = -EINVAL;

	ret = kstrtoul(buf, 10, &latency);
	if (ret)
		return ret;

	if (latency > sensors_cdev->max_delay) {
		dev_err(dev, "max_latency(%lu) is greater than max_delay(%u)\n",
				latency, sensors_cdev->max_delay);
		return -EINVAL;
	}

	if (sensors_cdev->sensors_set_latency == NULL) {
		dev_err(dev, "Invalid sensor calss set latency handle\n");
		return -EINVAL;
	}

	/* Disable batching for this sensor */
	if ((latency < sensors_cdev->delay_msec) && (latency != 0)) {
		dev_err(dev, "max_latency is less than delay_msec\n");
		return -EINVAL;
	}

	ret = sensors_cdev->sensors_set_latency(sensors_cdev, latency);
	if (ret)
		return ret;

	sensors_cdev->max_latency = latency;

	return size;
}

static ssize_t sensors_max_latency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
		"%u\n", sensors_cdev->max_latency);
}

static DEVICE_ATTR(max_latency, 0660, sensors_max_latency_show, sensors_max_latency_store);

static ssize_t sensors_flush_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long data = 0;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if (data != 1) {
		dev_err(dev, "Flush: Invalid value of input, input=%ld\n",
				data);
		return -EINVAL;
	}

	if (sensors_cdev->sensors_flush == NULL) {
		dev_err(dev, "Invalid sensor class flush handle\n");
		return -EINVAL;
	}
	ret = sensors_cdev->sensors_flush(sensors_cdev);
	if (ret)
		return ret;

	return size;
}

static ssize_t sensors_flush_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
		"Flush handler %s\n",
			(sensors_cdev->sensors_flush == NULL)
				? "not exist" : "exist");
}

static DEVICE_ATTR(flush, 0660, sensors_flush_show, sensors_flush_store);

static ssize_t sensors_enable_wakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	unsigned long enable;

	if (sensors_cdev->sensors_enable_wakeup == NULL) {
		dev_err(dev, "Invalid sensor class enable_wakeup handle\n");
		return -EINVAL;
	}

	ret = kstrtoul(buf, 10, &enable);
	if (ret)
		return ret;

	enable = enable ? 1 : 0;
	ret = sensors_cdev->sensors_enable_wakeup(sensors_cdev, enable);
	if (ret)
		return ret;

	sensors_cdev->wakeup = enable;

	return size;
}

static ssize_t sensors_enable_wakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", sensors_cdev->wakeup);
}

static DEVICE_ATTR(enable_wakeup, 0660, sensors_enable_wakeup_show, sensors_enable_wakeup_store);

static ssize_t sensors_calibrate_show(struct device *dev,
		struct device_attribute *atte, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	if (sensors_cdev->params == NULL) {
		dev_err(dev, "Invalid sensor params\n");
		return -EINVAL;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", sensors_cdev->params);
}

static ssize_t sensors_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	long data;
	int axis, apply_now;
	int cmd, bit_h;

	ret = kstrtol(buf, 0, &data);
	if (ret)
		return ret;
	dev_dbg(dev, "data = %lx\n", data);
	cmd = data & CMD_MASK;
	if (cmd == CMD_DO_CAL) {
		if (sensors_cdev->sensors_calibrate == NULL) {
			dev_err(dev, "Invalid calibrate handle\n");
			return -EINVAL;
		}
		/* parse the data to get the axis and apply_now value*/
		apply_now = (int)(data >> DATA_APPLY_SHIFT) & APPLY_MASK;
		axis = (int)data >> DATA_AXIS_SHIFT;
		dev_dbg(dev, "apply_now = %d, axis = %d\n", apply_now, axis);
		ret = sensors_cdev->sensors_calibrate(sensors_cdev,
				axis, apply_now);
		if (ret)
			return ret;
	} else {
		if (sensors_cdev->sensors_write_cal_params == NULL) {
			dev_err(dev,
					"Invalid write_cal_params handle\n");
			return -EINVAL;
		}
		bit_h = (data & CMD_W_H_L) >> 4;
		if (cmd > CMD_DO_CAL && cmd < CMD_COMPLETE) {
			char *p = (char *)(&sensors_cdev->cal_result)
					+ cal_map[cmd];
			*(int *)p = CMD_GET_PARAMS(bit_h, *(int *)p, data);
		} else if (cmd == CMD_COMPLETE) {
			ret = sensors_cdev->sensors_write_cal_params
				(sensors_cdev, &sensors_cdev->cal_result);
		} else {
			dev_err(dev, "Invalid command\n");
			return -EINVAL;
		}
	}
	return size;
}

static DEVICE_ATTR(calibrate, 0660, sensors_calibrate_show, sensors_calibrate_store);

static struct attribute *sensors_class_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_vendor.attr,
	&dev_attr_version.attr,
	&dev_attr_handle.attr,
	&dev_attr_type.attr,
	&dev_attr_max_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_sensor_power.attr,
	&dev_attr_min_delay.attr,
	&dev_attr_fifo_reserved_event_count.attr,
	&dev_attr_fifo_max_event_count.attr,
	&dev_attr_max_delay.attr,
	&dev_attr_flags.attr,
	&dev_attr_enable.attr,
	&dev_attr_enable_wakeup.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_self_test.attr,
	&dev_attr_max_latency.attr,
	&dev_attr_flush.attr,
	&dev_attr_calibrate.attr,
	NULL,
};

static const struct attribute_group sensors_class_group = {
	.attrs = sensors_class_attrs,
};

static const struct attribute_group *sensor_groups[] = {
	&sensors_class_group,
	NULL,
};

/**
 * sensors_classdev_register - register a new object of sensors_classdev class.
 * @parent: The device to register.
 * @sensors_cdev: the sensors_classdev structure for this device.
*/
int sensors_classdev_register(struct device *parent,
				struct sensors_classdev *sensors_cdev)
{
	sensors_cdev->dev = device_create(sensors_class, parent, 0,
				      sensors_cdev, "%s", sensors_cdev->name);
	if (IS_ERR(sensors_cdev->dev))
		return PTR_ERR(sensors_cdev->dev);

	down_write(&sensors_list_lock);
	list_add_tail(&sensors_cdev->node, &sensors_list);
	up_write(&sensors_list_lock);

	pr_debug("Registered sensors device: %s\n",
			sensors_cdev->name);
	return 0;
}
EXPORT_SYMBOL(sensors_classdev_register);

/**
 * sensors_classdev_unregister - unregister a object of sensors class.
 * @sensors_cdev: the sensor device to unregister
 * Unregister a previously registered via sensors_classdev_register object.
*/
void sensors_classdev_unregister(struct sensors_classdev *sensors_cdev)
{
	device_unregister(sensors_cdev->dev);
	down_write(&sensors_list_lock);
	list_del(&sensors_cdev->node);
	up_write(&sensors_list_lock);
}
EXPORT_SYMBOL(sensors_classdev_unregister);

void sensors_marker(char *src_func,
	char *action, char *object, char *para)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("%s: %s %s %s. @%d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		src_func, action, object, para,
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}
EXPORT_SYMBOL(sensors_marker);

void sensors_report_1(struct input_dev *idev, int val_1)
{
	ktime_t timestamp;

	timestamp = ktime_get_boottime();
	input_report_abs(idev, ABS_DISTANCE, val_1 ? 1000 : 1);
	input_report_abs(idev, ABS_DISTANCE, val_1 ? 1023 : 0);
	input_event(idev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(idev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(idev);
}
EXPORT_SYMBOL(sensors_report_1);

void sensors_report_2(struct input_dev *idev,
		int val_1, int val_2, int sensi, uint8_t *c_i)
{
	ktime_t timestamp = ktime_get_boottime();
	input_report_abs(idev, ABS_MISC, ((val_1 << 16) | (sensi << 4) | (*c_i & 0xF)));
	input_report_abs(idev, ABS_X, ((val_2 << 16)| (sensi << 4) | (*c_i & 0xF)));
	input_event(idev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(idev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(idev);
	if (*c_i < 15)
		*c_i = *c_i + 1;
	else
		*c_i = 0;
}
EXPORT_SYMBOL(sensors_report_2);

void sensors_report_3(struct input_dev *idev,		int val_1, int val_2, int val_3, int sensi, uint8_t *c_i)
{
	ktime_t timestamp = ktime_get_boottime();
	input_report_abs(idev, ABS_X, ((val_1 << 16) | (sensi << 4) | (*c_i & 0xF)));
	input_report_abs(idev, ABS_Y, ((val_2 << 16) | (sensi << 4) | (*c_i & 0xF)));
	input_report_abs(idev, ABS_Z, ((val_3 << 16) | (sensi << 4) | (*c_i & 0xF)));
	input_event(idev, EV_SYN, SYN_TIME_SEC,
		ktime_to_timespec(timestamp).tv_sec);
	input_event(idev, EV_SYN, SYN_TIME_NSEC,
		ktime_to_timespec(timestamp).tv_nsec);
	input_sync(idev);
	if (*c_i < 15)
		*c_i = *c_i + 1;
	else
		*c_i = 0;
}
EXPORT_SYMBOL(sensors_report_3);

void sensors_construct_extend_name(char *base,
		char *type_name, const char *position, int index)
{
	int count_back = 0;
	memset(base, 0, 32);
	strncpy(base, type_name, strlen(type_name));
	base += strlen(type_name);
	count_back += strlen(type_name);
	strncpy(base, "_", 1);
	base++;
	count_back++;
	strncpy(base, EXTEND_SENSOR_NAME_PREFIX, strlen(EXTEND_SENSOR_NAME_PREFIX));
	base += strlen(EXTEND_SENSOR_NAME_PREFIX);
	count_back += strlen(EXTEND_SENSOR_NAME_PREFIX);
	strncpy(base, position, strlen(position));
	base += strlen(position);
	count_back += strlen(position);
	if (index > 0) {
		char idx_char = index + 48;
		strncpy(base, "_", 1);
		base++;
		count_back++;
		// It is only support 1~9 with ASCII process,
		// bigger number will be done in the future.
		strncpy(base, &idx_char, 1);
	}
	base -= count_back;
}
EXPORT_SYMBOL(sensors_construct_extend_name);

int sensors_construct_extend_type_num(int base_type_id,
		const char *position, int index)
{
	int new_type_num = SENSOR_TYPE_DEVICE_PRIVATE_BASE;
	printk("%s(): step1, base-num = %d, posi is %s, target = %d\n",
		__func__, new_type_num, position, base_type_id);
	if (0 == strcmp(position, "front")) {
		new_type_num |= POSI_FRONT;
	} else if (0 == strcmp(position, "back")) {
		new_type_num |= POSI_BACK;
	} else if (0 == strcmp(position, "top")) {
		new_type_num |= POSI_TOP;
	} else if (0 == strcmp(position, "butt")) {
		new_type_num |= POSI_BUTT;
	} else if (0 == strcmp(position, "left")) {
		new_type_num |= POSI_LEFT;
	} else if (0 == strcmp(position, "right")) {
		new_type_num |= POSI_RIGHT;
	} else {
		// wrong position string.
	}
	printk("%s(): step2, base-num = %d, posi is %s, target = %d\n",
		__func__, new_type_num, position, base_type_id);
	new_type_num |= (index << 8);
	new_type_num += base_type_id;
	printk("%s(): step3, base-num = %d, posi is %s, target = %d\n",
		__func__, new_type_num, position, base_type_id);
	return new_type_num;
}
EXPORT_SYMBOL(sensors_construct_extend_type_num);

struct input_dev *sensors_construct_input_dev(struct device *parent_dev,
		char *name, int axes_num, void *priv_data)
{
	struct input_dev *i_dev = devm_input_allocate_device(parent_dev);
	if (!i_dev) {
		dev_err(parent_dev,
			"%s(): no memory for input_dev(%s)\n", __func__, name);
		return NULL;
	} else {
		i_dev->name = name;
		//i_dev->id.bustype = BUS_I2C;
		i_dev->dev.parent = parent_dev;
		set_bit(EV_ABS, i_dev->evbit);
		switch (axes_num) {
		case 1:
			input_set_abs_params(i_dev, ABS_DISTANCE, 0, 65535, 0, 0);
			break;
		case 2:
			input_set_abs_params(i_dev, ABS_MISC, 0, 65535, 0, 0);
			input_set_abs_params(i_dev, ABS_X, 0, 65535, 0, 0);
			break;
		case 3:
			input_set_abs_params(i_dev, ABS_X, -65535, 65535, 0, 0);
			input_set_abs_params(i_dev, ABS_Y, -65535, 65535, 0, 0);
			input_set_abs_params(i_dev, ABS_Z, -65535, 65535, 0, 0);
			break;
		}
		input_set_drvdata(i_dev, priv_data);
		if (input_register_device(i_dev)) {
			input_free_device(i_dev);
			dev_err(parent_dev,
				"%s: cant register input light\n", __func__);
		}
	}

	return i_dev;
}
EXPORT_SYMBOL(sensors_construct_input_dev);

static int __init sensors_init(void)
{
	sensors_class = class_create(THIS_MODULE, "sensors");
	if (IS_ERR(sensors_class))
		return PTR_ERR(sensors_class);
	sensors_class->dev_groups = sensor_groups;
	return 0;
}

static void __exit sensors_exit(void)
{
	class_destroy(sensors_class);
}

subsys_initcall(sensors_init);
module_exit(sensors_exit);
