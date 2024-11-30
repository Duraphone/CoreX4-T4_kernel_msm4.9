/*
 * Copyright (C) 2016 Hisense, Inc.
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/compiler.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/his_debug_base.h>
#include <hiconfig/kdebug_parti_layout.h>
#include "subsys_dbg_internal.h"

static bool subsys_probe_ok = false;
static char *pstore_info_addr;
static struct subsys_trap_info trap_info;
static struct platform_device *report_dev;
static struct delayed_work report_delay_work;
static struct delayed_work read_delay_work;

#define READ_DATA_DELAY_TIME      (30 * 1000)
#define SUBSYS_REASON_READSIZE    (256)

#define REPORT_ATTR(_name) \
	static struct kobj_attribute _name##_attr = {   \
		.attr = {                           \
			.name = __stringify(_name),     \
			.mode = 0644,                   \
		},                                  \
		.show   = _name##_show,             \
		.store  = _name##_store,            \
	}

static void save_subsys_err_to_mem(const char *name, const char *reason)
{
	struct subsys_trap_info *pinfo;

	if (NULL == pstore_info_addr)
		return;

	pinfo = (struct subsys_trap_info *)pstore_info_addr;
	snprintf(pinfo->name, SUBSYS_NAME_LEN, "%s", name);
	snprintf(pinfo->last_reason, REASON_STR_LEN, "%s", reason);

	pinfo->saved = KDEBUG_SAVED_MAGIC;
}

void subsystem_report(const char *subsys, const char *err_log, bool flag)
{
	static bool report_flag = false;
	char time_str[32] = {0};

	if (report_flag || !subsys_probe_ok)
		return;

	/* The flag avoid to repeat report kernel panic */
	report_flag = flag;
	his_get_current_time(time_str, sizeof(time_str));
	snprintf(trap_info.name, SUBSYS_NAME_LEN, "%s", subsys);
	snprintf(trap_info.last_reason, REASON_STR_LEN, "%s %s",
			time_str, err_log);

	if (!strcmp(trap_info.name, "kernel")
			|| get_debug_flag_bit(DEBUG_ENABLE_BIT)) {
		pr_info("ap panic, save it to memory\n");
		save_subsys_err_to_mem(trap_info.name, trap_info.last_reason);
	} else {
		pr_info("modem subsystem crash, set node status\n");
		if (!strcmp(trap_info.name, "modem"))
			set_modem_subsys_reset(time_str);
		schedule_delayed_work(&report_delay_work, msecs_to_jiffies(100));
	}
}

static void read_subsys_err_work(struct work_struct *work)
{
	int ret = 0;
	struct subsys_trap_info *pinfo;
#ifdef CONFIG_ABNORMAL_STATS
	char time_str[32] = {0};
	char reason_str[REASON_STR_LEN] = {0};

	reload_abnormal_stats_work();
#endif /* CONFIG_ABNORMAL_STATS */

	if (pstore_info_addr == NULL)
		return;

	ret = his_read_file(SYS_DEBUG_PARTITION, CRASH_INFO_OFFSET_IN_KDEBUG,
			pstore_info_addr, SUBSYS_REASON_READSIZE);
	if (ret <= 0) {
		pr_err("read buf err %d\n", ret);
		return;
	}

	pinfo = (struct subsys_trap_info *)pstore_info_addr;
	pr_info("the pinfo->saved is 0x%x\n", pinfo->saved);
	if (pinfo->saved != KDEBUG_SAVED_MAGIC)
		return;

	pr_info("the subsys name is %s\n", pinfo->name);
	pr_info("the report reason is %s\n", pinfo->last_reason);
#ifdef CONFIG_ABNORMAL_STATS
	his_get_current_time(time_str, sizeof(time_str));
	snprintf(reason_str, REASON_STR_LEN, "%s %s", time_str, pinfo->last_reason);

	update_abnormal_stats(pinfo->name, reason_str);
#endif /* CONFIG_ABNORMAL_STATS */

	/* clear recorder info in emmc */
	memset(pinfo, 0, sizeof(struct subsys_trap_info));
	his_write_file(SYS_DEBUG_PARTITION, CRASH_INFO_OFFSET_IN_KDEBUG,
			pinfo, sizeof(struct subsys_trap_info));
}

static void report_subsys_err_work(struct work_struct *work)
{
#ifdef CONFIG_ABNORMAL_STATS
	pr_info("system exception, save it to emmc\n");
	update_abnormal_stats(trap_info.name, trap_info.last_reason);
#endif /* CONFIG_ABNORMAL_STATS */
	/* clean the buffer */
	memset(&trap_info, 0, sizeof(struct subsys_trap_info));
}

static ssize_t test_report_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t test_report_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t n)
{
	static int test_time;
	char test_buf[30] = {0};
	int ret;
	long value;

	ret = kstrtol(buf, 10, &value);
	if (!ret && (value == 1)) {
		pr_info("run test error report %d\n", test_time);
		snprintf(test_buf, sizeof(test_buf), "dump time is %d", test_time++);
		subsystem_report("test", (const char *)test_buf, false);
	}

	return n;
}
REPORT_ATTR(test_report);

static struct attribute *test_attrs[] = {
	&test_report_attr.attr,
	NULL,
};

static struct attribute_group test_attr_group = {
	.attrs = test_attrs,
};

static int subsys_err_parse_dt(struct device *dev, u32 *data)
{
	struct device_node *pnode;
	const u32 *addr;
	u64 size;

	pnode = of_parse_phandle(dev->of_node, "linux,contiguous-region", 0);
	if (pnode == NULL) {
		pr_err("mem reserve for subsys-err not present\n");
		return -EINVAL;
	}

	addr = of_get_address(pnode, 0, &size, NULL);
	if (!addr) {
		pr_err("failed to parse the reserve memory address\n");
		of_node_put(pnode);
		return -EINVAL;
	}

	data[0] = (u32) of_read_ulong(addr, 2);
	data[1] = (u32) size;

	return 0;
}

static int subsys_err_report_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 offsets[2];

	if (!pdev->dev.of_node) {
		pr_err("can not find of_node of device\n");
		return -EINVAL;
	}

	ret = subsys_err_parse_dt(&pdev->dev, offsets);
	if (ret < 0) {
		pr_err("subsys err report parse dts failed %d\n", ret);
		return -EINVAL;
	}

	pstore_info_addr = ioremap_wc(offsets[0], offsets[1]);
	pr_info("The phys_base=0x%x, size=0x%x vaddr=0x%llx\n",
			offsets[0], offsets[1], (u64)pstore_info_addr);

	report_dev = pdev;

	INIT_DELAYED_WORK(&report_delay_work, report_subsys_err_work);
	INIT_DELAYED_WORK(&read_delay_work, read_subsys_err_work);

	ret = sysfs_create_group(&pdev->dev.kobj, &test_attr_group);
	if (ret)
		pr_err("subsys_err_report register attr failed\n");

	if (BOOT_NORMAL_MODE == dev_bi.bootmode ||
				BOOT_FACTORY_MODE == dev_bi.bootmode)
		schedule_delayed_work(&read_delay_work,
				msecs_to_jiffies(READ_DATA_DELAY_TIME));

	/* set module ready to report */
	subsys_probe_ok = true;

	return ret;
}

static int subsys_err_report_remove(struct platform_device *pdev)
{
	int ret = 0;

	sysfs_remove_group(&pdev->dev.kobj, &test_attr_group);
	return ret;
}

static struct of_device_id subsys_err_match_table[] = {
	{
		.compatible = "subsys-err-report"
	},
	{},
};

static struct platform_driver subsys_err_report_driver = {
	.probe = subsys_err_report_probe,
	.remove = subsys_err_report_remove,
	.driver = {
		.name = "subsys-err-report",
		.owner	= THIS_MODULE,
		.of_match_table = subsys_err_match_table,
	},
};

static int __init subsys_err_report_init(void)
{
	int ret;

	ret = platform_driver_register(&subsys_err_report_driver);
	if (ret) {
		pr_err("%s: register driver error\n", __func__);
		return ret;
	}

	return ret;
}

static void __exit subsys_err_report_exit(void)
{
	platform_driver_unregister(&subsys_err_report_driver);
}

late_initcall(subsys_err_report_init);
module_exit(subsys_err_report_exit);
