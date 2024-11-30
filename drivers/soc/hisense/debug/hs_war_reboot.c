#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/timekeeping.h>
#include <linux/his_debug_base.h>

#define HS_RB_MIN_TIMEOUT_SECS 30  /* 30s  */

struct hs_reboot_war {
	bool is_set;
	bool is_reboot;
	long  rb_time;
	struct workqueue_struct *wq;
	struct delayed_work rb_work;
	struct mutex rb_lock;
};

static struct hs_reboot_war *hs_rb_if;

static ssize_t reboot_war_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	if (!hs_rb_if)
		return s - buf;

	s += snprintf(s, PAGE_SIZE, "%d\n", hs_rb_if->is_set);

	return s - buf;
}

static ssize_t reboot_war_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	long value = 0;
	int ret = 0;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("%s: kstrtol is wrong!\n", __func__);
		return -EINVAL;
	}

	/* Sync with system/core/init/reboot.c
	   1 SHUTDOWN  2 REBOOT */

	if ((value != 1) && (value != 2)){
		pr_err("%s: invalid value %ld\n", __func__, value);
		return n;
	}

	if (!hs_rb_if)
		return n;

	/*if (hs_rb_if->is_set) {
		pr_info("%s: hs_rb_if already set\n", __func__);
		return n;
	}*/

	mutex_lock(&hs_rb_if->rb_lock);
	if (hs_rb_if->is_set) {
		pr_info("%s: hs_rb_if already set, re-sched work\n", __func__);
		cancel_delayed_work_sync(&hs_rb_if->rb_work);
	}
	if (value == 2) {
		hs_rb_if->is_reboot = true;
		pr_info("%s: reboot confirm!\n", __func__);
  	} else
		pr_info("%s: poweroff confirm!\n", __func__);
	queue_delayed_work(hs_rb_if->wq, &hs_rb_if->rb_work, HZ * hs_rb_if->rb_time);
	hs_rb_if->is_set = true;
	mutex_unlock(&hs_rb_if->rb_lock);
	return n;
}
DEVICE_ATTR_RW(reboot_war);

static ssize_t reboot_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	if (!hs_rb_if)
		return s - buf;

	s += snprintf(s, PAGE_SIZE, "%ld\n", hs_rb_if->rb_time);

	return s - buf;
}

static ssize_t reboot_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	long value = 0;
	int ret = 0;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("%s: kstrtol is wrong!\n", __func__);
		return -EINVAL;
	}

	/* Default/Min value is HS_RB_MIN_TIMEOUT_SECS  *
	 * New value only applied before reboot_war set */

	if (value < HS_RB_MIN_TIMEOUT_SECS){
		pr_err("%s:invalid value %ld, reset\n", __func__, value);
		value = HS_RB_MIN_TIMEOUT_SECS;
	}

	if (!hs_rb_if)
		return n;

	/*if (hs_rb_if->is_set) {
		pr_info("%s: hs_rb_if already set\n", __func__);
		return n;
	}*/

	mutex_lock(&hs_rb_if->rb_lock);
	hs_rb_if->rb_time = value;
	if (hs_rb_if->is_set) {
		pr_info("%s: hs_rb_if already set, re-sched work\n", __func__);
		cancel_delayed_work_sync(&hs_rb_if->rb_work);
		queue_delayed_work(hs_rb_if->wq, &hs_rb_if->rb_work, HZ * hs_rb_if->rb_time);
	}
	mutex_unlock(&hs_rb_if->rb_lock);
	return n;
}
DEVICE_ATTR_RW(reboot_time);

static struct attribute *hs_reboot_attrs[] = {
	&dev_attr_reboot_war.attr,
	&dev_attr_reboot_time.attr,
	NULL,
};

static struct attribute_group hs_reboot_attr_group = {
	.attrs = hs_reboot_attrs,
};

extern void hs_reboot_war_action(bool is_reboot);

static void reboot_work(struct work_struct *work)
{

	if (!hs_rb_if)
		return;

	pr_info("hs reboot war work\n");

	hs_reboot_war_action(hs_rb_if->is_reboot);
}

static int __init init_hs_reboot_war(void)
{
	int ret = 0;
	struct hs_reboot_war *reboot_info = NULL;
	reboot_info = kzalloc(sizeof(*reboot_info), GFP_KERNEL);
	if (!reboot_info) {
		ret = -ENOMEM;
		pr_err("%s create kzalloc failed\n", __func__);
		goto out1;
	}

	reboot_info->wq = create_singlethread_workqueue("reboot_war");
	if (!reboot_info->wq){
		ret = -ENOMEM;
		pr_err("%s create wq failed\n", __func__);
		goto out2;
	}
	INIT_DELAYED_WORK(&reboot_info->rb_work, reboot_work);
	mutex_init(&reboot_info->rb_lock);
	reboot_info->is_set = false;
	reboot_info->is_reboot = false;
	reboot_info->rb_time = HS_RB_MIN_TIMEOUT_SECS * 2;
	hs_rb_if = reboot_info;
	ret = his_register_sysfs_attr_group(&hs_reboot_attr_group);
	if (ret) {
		pr_err("%s: register hs reboot war attr failed\n", __func__);
		goto out3;
	}
	return 0;

out3:
	destroy_workqueue(reboot_info->wq);
out2:
	kfree(reboot_info);
	hs_rb_if = NULL;
out1:
	return ret;
}
late_initcall(init_hs_reboot_war);

