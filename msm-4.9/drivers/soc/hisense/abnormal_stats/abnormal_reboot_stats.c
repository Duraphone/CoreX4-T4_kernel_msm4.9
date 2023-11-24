/*
 * Copyright (C) 2016 Hisense, Inc.
 *
 * Author:
 *   wangxufeng <wangxufeng@hisense.com>
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
#define pr_fmt(fmt) "subsys_report: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <asm/setup.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/his_debug_base.h>
#include <hiconfig/kdebug_parti_layout.h>
#include "subsys_dbg_internal.h"

#define ALL_SUBSYSTEM_NUM             12
#define REBOOT_STATS_SAVE_PATH        "/kdebuginfo/rs_recorder/reboot_reason"

#define FULL_ABNORMAL_REASON_LEN      256
static int abnormal_stats_ready = 0;
static struct mutex rw_mlock;
static char last_abnormal_reason[FULL_ABNORMAL_REASON_LEN] = {0};
static struct subsys_trap_info abnormal_data[ALL_SUBSYSTEM_NUM];

static char modem_reset_time[64] = {0};
void set_modem_subsys_reset(char *reset_time)
{
	strlcpy(modem_reset_time, reset_time, sizeof(modem_reset_time));
}

int abnormal_need_append_time(const char *name)
{
	int i = 0;
	char *knowed_name[] = {"kernel", "tz", "modem"};
	int item_num = sizeof(knowed_name)/sizeof(char *);

	for (i = 0; i < item_num; i++)
		if (!strcmp(knowed_name[i], name))
			return 0;

	return 1;
}

static int save_reboot_stats_reason(const char *name, const char *reason)
{
	int ret = 0;
#ifdef CONFIG_ABNORMAL_STATS_SAVE_FILE
	int reason_len = 0;
	struct file *filp = NULL;
	char *path = REBOOT_STATS_SAVE_PATH;
#endif /* CONFIG_ABNORMAL_STATS_SAVE_FILE */

	mutex_lock(&rw_mlock);
	memset(last_abnormal_reason, 0, FULL_ABNORMAL_REASON_LEN);
	snprintf(last_abnormal_reason, FULL_ABNORMAL_REASON_LEN,
			"%s\t\t%s\n", name, reason);

#ifdef CONFIG_ABNORMAL_STATS_SAVE_FILE
	reason_len = strlen(last_abnormal_reason);

	filp = filp_open(path, O_CREAT | O_WRONLY | O_APPEND, 0644);
	if (IS_ERR(filp)) {
		pr_err("Failed to open %s\n", path);
		mutex_unlock(&rw_mlock);
		return PTR_ERR(filp);
	}

	ret = kernel_write(filp, last_abnormal_reason, reason_len, filp->f_pos);
	filp_close(filp, NULL);
#endif /* CONFIG_ABNORMAL_STATS_SAVE_FILE */
	mutex_unlock(&rw_mlock);

	return ret;
}

void update_abnormal_stats(const char *name, const char *reason)
{
	int i = 0;
	char time_str[32] = {0};
	int size = 0;
	int need_append_time = 0;

	if (0 == abnormal_stats_ready)
		return;

	pr_info("system abnormal, update stats\n");
	for (i = 0; i < ALL_SUBSYSTEM_NUM; i++) {
		if (abnormal_data[i].saved == KDEBUG_SAVED_MAGIC) {
			if (!strcmp(abnormal_data[i].name, name)) {
				abnormal_data[i].count++;
				break;
			}
		} else {
			strlcpy(abnormal_data[i].name, name, SUBSYS_NAME_LEN);
			abnormal_data[i].count = 1;
			/* set saved flag */
			abnormal_data[i].saved = KDEBUG_SAVED_MAGIC;
			break;
		}
	}

	if ((i < ALL_SUBSYSTEM_NUM) && (NULL != reason)) {
		need_append_time = abnormal_need_append_time(name);
		if (need_append_time == 1) {
			his_get_current_time(time_str, sizeof(time_str));
			size = strlcpy(abnormal_data[i].last_reason,
					time_str, REASON_STR_LEN);
		}

		snprintf(abnormal_data[i].last_reason + size, REASON_STR_LEN - size,
				"  %s", reason);
	}

	/* save data to emmc */
	his_write_file(SYS_DEBUG_PARTITION, STATS_INFO_OFFSET_IN_KDEBUG,
			(void *)abnormal_data, sizeof(abnormal_data));

	save_reboot_stats_reason(name, reason);
}

void reload_abnormal_stats_work(void)
{
	int i = 0, ret;
	char *buff;
	int size;

	size = sizeof(abnormal_data);
	buff = kzalloc(size, GFP_KERNEL);
	if (buff == NULL) {
		pr_err("No memory for malloc\n");
		return;
	}

	ret = his_read_file(SYS_DEBUG_PARTITION, STATS_INFO_OFFSET_IN_KDEBUG,
			buff, size);
	if (ret <= 0) {
		kfree(buff);
		return;
	}

	memcpy((void *)abnormal_data, buff, size);
	kfree(buff);

	pr_info("subsystem      count     last_reason\n");
	for (i = 0; i < ALL_SUBSYSTEM_NUM; i++) {
		if (abnormal_data[i].saved == KDEBUG_SAVED_MAGIC)
			pr_info("%s\t\t %d\t %s\n",
					abnormal_data[i].name,
					abnormal_data[i].count,
					abnormal_data[i].last_reason);
	}

	abnormal_stats_ready = 1;
}

static ssize_t abnormal_data_ready_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d", abnormal_stats_ready);
}

static ssize_t abnormal_reboot_stats_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0;
	int size = 0;

	size = snprintf(buf, PAGE_SIZE,
			"subsystem      count     last_reason\n");

	for (i = 0; i < ALL_SUBSYSTEM_NUM; i++) {
		if (abnormal_data[i].saved == KDEBUG_SAVED_MAGIC)
			size += snprintf(buf+size, PAGE_SIZE, "%s\t\t %d\t %s\n",
						abnormal_data[i].name,
						abnormal_data[i].count,
						abnormal_data[i].last_reason);
	}

	return size;
}

static ssize_t abnormal_reboot_stats_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int ret;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("debug_store: sscanf is wrong!\n");
		return -EINVAL;
	}

	if (1 == value) {
		pr_info("%s : clear the abnormal stats data\n", __func__);
		memset(abnormal_data, 0, sizeof(abnormal_data));
		his_write_file(SYS_DEBUG_PARTITION, STATS_INFO_OFFSET_IN_KDEBUG,
				(void *)abnormal_data, sizeof(abnormal_data));
	} else {
		pr_info("Error parameter for abnormal stats");
	}

	return len;
}

static ssize_t last_abnormal_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;
	int size = 0;

	mutex_lock(&rw_mlock);
	len = strlen(last_abnormal_reason);
	if (len > 0)
		size = snprintf(buf, PAGE_SIZE, "%s", last_abnormal_reason);
	else
		size = snprintf(buf, PAGE_SIZE, "NULL");

	memset(last_abnormal_reason, 0, FULL_ABNORMAL_REASON_LEN);
	mutex_unlock(&rw_mlock);

	return size;
}

static ssize_t modem_reset_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;
	int size = 0;

	len = strlen(modem_reset_time);
	if (len > 0) {
		size = snprintf(buf, PAGE_SIZE, "1_%s\n", modem_reset_time);
		memset(modem_reset_time, 0, sizeof(modem_reset_time));
	} else {
		size = snprintf(buf, PAGE_SIZE, "0_0\n");
	}

	return size;
}
static DEVICE_ATTR(data_ready, S_IRUGO, abnormal_data_ready_show, NULL);
static DEVICE_ATTR(reboot_stats, S_IRUGO, abnormal_reboot_stats_show, NULL);
static DEVICE_ATTR(last_abnormal, S_IRUGO, last_abnormal_info_show, NULL);
static DEVICE_ATTR(clear_stats, S_IWUSR, NULL, abnormal_reboot_stats_store);
static DEVICE_ATTR(modem_reset, S_IRUGO, modem_reset_status_show, NULL);

static struct attribute *abnormal_attrs[] = {
	&dev_attr_data_ready.attr,
	&dev_attr_reboot_stats.attr,
	&dev_attr_last_abnormal.attr,
	&dev_attr_clear_stats.attr,
	&dev_attr_modem_reset.attr,
	NULL,
};

static struct attribute_group abnormal_attr_group = {
	.attrs = abnormal_attrs,
};

static int __init abnormal_reboot_stats_init(void)
{
	int ret;
	struct kobject *root_kobjs;

	mutex_init(&rw_mlock);
	root_kobjs = his_register_sysfs_dir("abnormal_stats");
	if (NULL == root_kobjs)
		return -ENOMEM;

	ret = sysfs_create_group(root_kobjs, &abnormal_attr_group);
	if (ret < 0) {
		pr_err("Error create abnormal_attr_group %d\n", ret);
		return ret;
	}

	return 0;
}
late_initcall(abnormal_reboot_stats_init);

