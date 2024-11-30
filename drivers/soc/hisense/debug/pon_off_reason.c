/*
 * Copyright (C) 2017 Hisense, Inc.
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
#define pr_fmt(fmt) "HS_PON: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/his_debug_base.h>
#include <hiconfig/kdebug_parti_layout.h>

#define POWER_REASON_FILE     "/kdebuginfo/power_reason.txt"

extern const char *qpnp_get_pon_off_reason(int is_power_on, int reg_val);
extern const char *qpnp_get_current_power_reason(int is_power_on, int *reg_val);

#define RELOAD_DATA_TIMEOUT   (40 * 1000)

#define POWER_STATS_NUM       64
#define POWER_REASON_MAGIC    0x504F4646

#define PON_INDEX_START       0
#define POFF_INDEX_START      8
/*
 * pon: 1 -> power on reason
 *      0 -> power off reason
 *
 * reg_val: The pmic register value
 * count  : The number of current value
 */
struct pon_off_data {
	int pon;
	int reg_val;
	u64 count;
};

struct power_stats {
	int magic;
	int num;
	struct pon_off_data pon_data[POWER_STATS_NUM];
};
static struct power_stats s_power_stats = {0};
/* reload power stats data work_struct */
static struct delayed_work reload_data_work;

static ssize_t pon_off_show_stats(char *buf)
{
	int i = 0;
	int num = 0;
	size_t size = 0;
	const char *str_ptr = NULL;

	size += snprintf(buf, PAGE_SIZE, "\nPON Stats:\n");
	num = s_power_stats.num & 0xFFFF;
	pr_info("PON reason num: %d\n", num);
	for (i = 0; i < num; i++) {
		int reg = 0;

		reg = s_power_stats.pon_data[i + PON_INDEX_START].reg_val;
		str_ptr = qpnp_get_pon_off_reason(1, reg);
		size += snprintf(buf + size, PAGE_SIZE, "  %s: %lld\n",
				str_ptr,
				s_power_stats.pon_data[i + PON_INDEX_START].count);
	}

	size += snprintf(buf + size, PAGE_SIZE, "\nPOFF Stats:\n");
	num = (s_power_stats.num & 0xFFFF0000) >> 16;
	pr_info("POFF reason num: %d\n", num);
	for (i = 0; i < num; i++) {
		int reg = 0;

		reg = s_power_stats.pon_data[i + POFF_INDEX_START].reg_val;
		str_ptr = qpnp_get_pon_off_reason(0, reg);
		size += snprintf(buf + size, PAGE_SIZE, "  %s: %lld\n",
				str_ptr,
				s_power_stats.pon_data[i + POFF_INDEX_START].count);
	}

	return size;
}

static ssize_t pon_off_reason_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	const char *str_ptr = NULL;
	int temp = 0;

	size += snprintf(buf + size, PAGE_SIZE, "Current power reason:\n");
	str_ptr = qpnp_get_current_power_reason(1, &temp);
	if (str_ptr == NULL)
		str_ptr = "Unknown";
	size += snprintf(buf + size, PAGE_SIZE, "  Power on: %s\n", str_ptr);

	str_ptr = qpnp_get_current_power_reason(0, &temp);
	if (str_ptr == NULL)
		str_ptr = "Unknown";
	size += snprintf(buf + size, PAGE_SIZE, "  Power off: %s\n", str_ptr);

	size += pon_off_show_stats(buf + size);

	return size;
}

static DEVICE_ATTR(pon_off_reason, S_IRUGO, pon_off_reason_show, NULL);

static int update_pon_data_by_index(struct power_stats *pstats,
		int num, int start, int reg_val)
{
	int i = 0;

	for (i = 0; i < num; i++) {
		int idx = start + i;

		if (pstats->pon_data[idx].reg_val == reg_val) {
			pstats->pon_data[idx].count ++;
			return 1;
		}
	}

	if (i == num) {
		pstats->pon_data[start + i].reg_val = reg_val;
		pstats->pon_data[start + i].count ++;
	}

	return 0;
}

static void update_power_reason_file(int pon, int poff)
{
	int str_len = 0;
	struct file *filp = NULL;
	const char *pon_str = NULL;
	const char *poff_str = NULL;
	char time_str[32] = {0};
	char curr_reason_str[160] = {0};

	his_get_current_time(time_str, sizeof(time_str));
	pon_str = qpnp_get_pon_off_reason(1, pon);
	poff_str = qpnp_get_pon_off_reason(0, poff);
	str_len = snprintf(curr_reason_str, sizeof(curr_reason_str),
			"%s    OFF: %s    ON: %s\n",
			time_str, poff_str, pon_str);

	filp = filp_open(POWER_REASON_FILE, O_CREAT | O_WRONLY | O_APPEND, 0644);
	if (IS_ERR(filp)) {
		pr_err("Failed to open %s\n", POWER_REASON_FILE);
		return;
	}

	kernel_write(filp, curr_reason_str, str_len, filp->f_pos);
	filp_close(filp, NULL);
}

static void pon_reason_update_stats(struct power_stats *pstats)
{
	int ret = 0;
	int num = 0;
	int pon = 0, poff = 0;

	qpnp_get_current_power_reason(1, &pon);
	qpnp_get_current_power_reason(0, &poff);

	if (pstats->magic != POWER_REASON_MAGIC) {
		pr_err("The pon magic is error, reinit it\n");
		memset(pstats, 0, sizeof(struct power_stats));
		pstats->magic = POWER_REASON_MAGIC;
	}

	num = pstats->num & 0xFFFF;
	ret = update_pon_data_by_index(pstats, num, PON_INDEX_START, pon);
	if (ret == 0)
		pstats->num += 0x1;

	num = (pstats->num & 0xFFFF0000) >> 16;
	ret = update_pon_data_by_index(pstats, num, POFF_INDEX_START, poff);
	if (ret == 0)
		pstats->num += 0x10000;

	update_power_reason_file(pon, poff);
}

static void reload_and_update_power_stats(struct work_struct *work)
{
	int ret = 0;
	void *pdata = &s_power_stats;

	ret = his_read_file(SYS_DEBUG_PARTITION, PON_REASON_OFFSET_IN_KDEBUG,
			pdata, sizeof(struct power_stats));
	if (ret <= 0) {
		pr_err("read power stats error %d\n", ret);
		return;
	}

	pr_info("pdata->magic = 0x%x\n", s_power_stats.magic);
	pr_info("pdata->num = 0x%x\n", s_power_stats.num);
	pon_reason_update_stats(&s_power_stats);

	his_write_file(SYS_DEBUG_PARTITION, PON_REASON_OFFSET_IN_KDEBUG,
			pdata, sizeof(struct power_stats));
	if (ret < 0)
		pr_err("flush data to flash error %d\n", ret);
}

static int __init pon_off_reason_init(void)
{
	int ret;

	ret = his_register_sysfs_attr(&dev_attr_pon_off_reason.attr);
	if (ret < 0) {
		pr_err("Error create pon_off_reason file %d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&reload_data_work, reload_and_update_power_stats);
	if (dev_bi.bootmode != BOOT_CHARGER_MODE)
		schedule_delayed_work(&reload_data_work,
				msecs_to_jiffies(RELOAD_DATA_TIMEOUT));

	return 0;
}
late_initcall(pon_off_reason_init);

