#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/his_debug_base.h>

#define HS_BLOCK_BL_TIMEOUT 300  /* 5 min */

struct hs_sil_info {
	long nonblock_time_sec;
	bool nonblock_bl;
	struct mutex sil_lock;
};

static struct hs_sil_info *hs_sil_if;

static bool hs_silence_reboot;

static void hs_sil_bootmode_init(void)
{
	if (dev_bi.bootmode == BOOT_SILENCE_MODE)
		hs_silence_reboot = true;
	else
		hs_silence_reboot = false;
	pr_info("hs silence reboot: %d\n", hs_silence_reboot);
}

bool hs_should_block_bl(int level)
{
	struct timespec uptime;
	bool block_bl = true;

	if (!hs_sil_if)
		return false;
	if (!level)
		return false;

	mutex_lock(&hs_sil_if->sil_lock);

	if (hs_sil_if->nonblock_bl) {
		block_bl = false;
		goto out;
	}

	if (!hs_silence_reboot) {
		block_bl = false;
		goto out;
	}

	get_monotonic_boottime(&uptime);
	/*Protect for lcd bl ops*/
	if (uptime.tv_sec > HS_BLOCK_BL_TIMEOUT) {
		pr_info("%s(): boot time %ld reset nonblock_bl true\n",
				__func__, uptime.tv_sec);
		hs_sil_if->nonblock_bl = true;
		block_bl = false;
		goto out;
	}

	pr_info_ratelimited("silence mode:%d nonblock_bl:%d  origin lvl %d\n",
		hs_silence_reboot, hs_sil_if->nonblock_bl, level);

out:
	mutex_unlock(&hs_sil_if->sil_lock);
	return block_bl;
}
EXPORT_SYMBOL(hs_should_block_bl);

static ssize_t hs_nonblock_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	if (!hs_sil_if)
		return s - buf;

	s += snprintf(s, PAGE_SIZE, "%ld\n", hs_sil_if->nonblock_time_sec);

	return s - buf;
}
DEVICE_ATTR_RO(hs_nonblock_time);

static ssize_t hs_nonblock_bl_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	if (!hs_sil_if)
		return s - buf;

	s += snprintf(s, PAGE_SIZE, "%d\n", hs_sil_if->nonblock_bl);

	return s - buf;
}

static ssize_t hs_nonblock_bl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	long value = 0;
	int ret = 0;
	struct timespec uptime;

	ret = kstrtol(buf, 10, &value);
	if (ret) {
		pr_err("%s: kstrtol is wrong!\n", __func__);
		return -EINVAL;
	}

	if (!hs_sil_if)
		return n;

	/*no need set non_block_bl for non silence reboot */
	if (!hs_silence_reboot)
		return n;

	if (hs_sil_if->nonblock_bl) {
		pr_info("%s(): nonblock_bl already set\n", __func__);
		return n;
	}
	mutex_lock(&hs_sil_if->sil_lock);
	if (value != 0) {
		pr_info("%s(): set nonblock_bl value: %ld\n", __func__, value);
		hs_sil_if->nonblock_bl = true;
		get_monotonic_boottime(&uptime);
		hs_sil_if->nonblock_time_sec = uptime.tv_sec;
	} else
	pr_err("%s():set nonblock_bl wrong: %ld\n", __func__, value);
	mutex_unlock(&hs_sil_if->sil_lock);
	return n;
}
DEVICE_ATTR_RW(hs_nonblock_bl);

static struct attribute *hs_nonblock_attrs[] = {
	&dev_attr_hs_nonblock_time.attr,
	&dev_attr_hs_nonblock_bl.attr,
	NULL,
};

static struct attribute_group hs_nonblock_attr_group = {
	.attrs = hs_nonblock_attrs,
};

static int __init init_hs_nonblock_bl(void)
{
	int ret = 0;
	struct hs_sil_info *sil_info = NULL;
	sil_info = kzalloc(sizeof(*sil_info), GFP_KERNEL);
	if (!sil_info) {
		ret = -ENOMEM;
		pr_err("%s create kzalloc failed\n", __func__);
		goto out1;
	}

	hs_sil_bootmode_init();

	ret = his_register_sysfs_attr_group(&hs_nonblock_attr_group);
	if (ret) {
		pr_err("%s: register hs nonblock bl attr failed\n", __func__);
		goto out2;
	}
	mutex_init(&sil_info->sil_lock);
	sil_info->nonblock_time_sec = 0;
	sil_info->nonblock_bl = false;
	hs_sil_if = sil_info;

	return 0;

out2:
	kfree(sil_info);
	hs_sil_if = NULL;
out1:
	return ret;
}
late_initcall(init_hs_nonblock_bl);

