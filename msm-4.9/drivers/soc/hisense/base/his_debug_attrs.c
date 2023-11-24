/*
 * Copyright (C) 2005-2016 Hisense, Inc.
 *
 * Author:
 *   wangyongqing <wangyongqing1@hisense.com>
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

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/his_debug_base.h>
#include <hiconfig/flag_parti_layout.h>

#define HISENSE_DEBUG_ATTR(attr_name, attr_offset) \
	static ssize_t attr_name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t len) \
	{ \
		int ret; \
		long value; \
					\
		ret = kstrtol(buf, 10, &value); \
		if (ret) { \
			pr_err("%s: kstrtol is wrong!\n", __func__); \
			return -EINVAL; \
		} \
		  \
		pr_info("%s : enable %ld\n", __func__, value); \
		hs_store_attr_value(attr_offset, value); \
		\
		return len; \
	} \
	static ssize_t attr_name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		int flag = 0; \
		\
		his_read_file(SYS_FLAG_PARTITION, \
				attr_offset, &flag, sizeof(flag)); \
		\
		return snprintf(buf, PAGE_SIZE, "%d\n", flag); \
	} \
	static DEVICE_ATTR(attr_name, S_IWUSR | S_IWGRP | S_IRUGO, \
		attr_name##_show, attr_name##_store);

/* Debug cmdline early setup macro */
#define DEFINED_DEBUG_EARLY_PARAM(cmdline, setted_bit)  \
	static int __init early_param_##cmdline(char *p) { \
		boot_debug_flag |= setted_bit; \
		pr_err("%s: set %s\n", __func__, #setted_bit); \
		return 0; \
	} \
	early_param(#cmdline, early_param_##cmdline)

static uint32_t boot_debug_flag;
void set_debug_flag_bit(int set_bit)
{
	boot_debug_flag |= set_bit;
}

void clear_debug_flag_bit(int set_bit)
{
	boot_debug_flag &= ~set_bit;
}

bool get_debug_flag_bit(int get_bit)
{
	if (boot_debug_flag & get_bit)
		return true;
	else
		return false;
}

DEFINED_DEBUG_EARLY_PARAM(enable_debug, DEBUG_ENABLE_BIT);
DEFINED_DEBUG_EARLY_PARAM(print_active_ws, PRINT_WAKELOCK_BIT);
DEFINED_DEBUG_EARLY_PARAM(serial_enable, SERIAL_ENABLE_BIT);

/* common function */
static void hs_store_attr_value(int offset, int value)
{
	int flag;
	int old_value;

	his_read_file(SYS_FLAG_PARTITION, offset,
			&flag, sizeof(flag));
	old_value = flag;
	if (old_value == !!value) {
		pr_info("%s: already set to %d\n", __func__, value);
		return;
	}

	if (value == 1)
		flag = 1;
	else
		flag = 0;

	his_write_file(SYS_FLAG_PARTITION, offset,
			&flag, sizeof(flag));
}

void hs_set_rtb_flag(int value)
{
    hs_store_attr_value(FLAG_HS_MSM_RTB_ENABLE_OFFSET, value);
}
EXPORT_SYMBOL(hs_set_rtb_flag);

/* sysfs attributes for debug, In /sys/debug_control/ directory */
HISENSE_DEBUG_ATTR(serial_console_ctrl, FLAG_ENABLE_SERIAL_CONSOLE_OFFSET);
HISENSE_DEBUG_ATTR(print_sleep_gpio, FLAG_PRINT_SLEEP_GPIO_OFFSET);
HISENSE_DEBUG_ATTR(print_active_ws, FLAG_PRINT_ACTIVE_WS_OFFSET);
HISENSE_DEBUG_ATTR(hs_msm_rtb_enable, FLAG_HS_MSM_RTB_ENABLE_OFFSET);
HISENSE_DEBUG_ATTR(hs_disable_avb, FLAG_HS_DISABLE_AVB_OFFSET);

static struct attribute *his_debug_ctrl_attrs[] = {
	&dev_attr_serial_console_ctrl.attr,
	&dev_attr_print_sleep_gpio.attr,
	&dev_attr_print_active_ws.attr,
	&dev_attr_hs_msm_rtb_enable.attr,
	&dev_attr_hs_disable_avb.attr,
	NULL,
};

static struct attribute_group his_debug_ctrl_attr_group = {
	.attrs = his_debug_ctrl_attrs,
};

static int __init hisense_debug_ctrl_init(void)
{
	int ret;

	ret = his_register_sysfs_attr_group(&his_debug_ctrl_attr_group);
	if (ret < 0) {
		pr_err("Error creating poweroff sysfs group, ret=%d\n", ret);
		return ret;
	}

	pr_info("%s: create sysfs ok\n", __func__);

	return 0;
}
late_initcall(hisense_debug_ctrl_init);

