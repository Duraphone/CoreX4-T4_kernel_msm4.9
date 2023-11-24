/*
 * Copyright (C) 2008-2014 Hisense, Inc.
 *
 * Author:
 *   kongzhiqiang <kongzhiqiang@hisense.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <asm/setup.h>
#include <linux/his_debug_base.h>
#include <linux/awaken_sys_event.h>

#define DEV_MAX_KEY_NUM     10
struct device_keymap {
	int key_num;

	struct keymap {
		int code;
		const char *name;
	} keymap[DEV_MAX_KEY_NUM];
};

#define BOOTINFO_ATTR(_name) \
static struct kobj_attribute _name##_attr = { \
	.attr   = {	                              \
		.name = __stringify(_name),	          \
		.mode = 0444,                         \
	},                                        \
	.show   = _name##_show,	                  \
	.store  = NULL,	                          \
}

#define POWERON_REASON_LEN          20
static char poweron_reason[POWERON_REASON_LEN];

#define LCD_ID_LEN 32
static char mdss_panel_lcd_id[LCD_ID_LEN];

struct device_bootinfo dev_bi;
static struct device_keymap dev_kmap;

static void of_parse_device_info(void)
{
	int i = 0;
	struct device_node *np = NULL;
	int size;

	np = of_find_node_by_path("/soc/his_devinfo");
	if (!np) {
		pr_err("Can not find his_devinfo node\n");
		return;
	}

	size = of_property_count_strings(np, "dev,keymap-names");
	if ((size <= 0) || (size > DEV_MAX_KEY_NUM)) {
		pr_err("device keymap size is error\n");
		return;
	}

	pr_err("found Key num: %d\n", size);
	dev_kmap.key_num = size;
	for (i = 0; i < dev_kmap.key_num; i++) {
		of_property_read_string_index(np, "dev,keymap-names",
				i, &dev_kmap.keymap[i].name);

		of_property_read_u32_index(np, "dev,keymap-values",
				i, &dev_kmap.keymap[i].code);
	}
}

void input_print_keyevent_log(u32 type, u32 code, int value)
{
	int i = 0;

	if (type != EV_KEY)
		return;

	if (dev_kmap.key_num == 0) {
		pr_err("%d key %s\n", code,
				value ? "pressed" : "released");
		return;
	}

	for (i = 0; i < dev_kmap.key_num; i++) {
		if (dev_kmap.keymap[i].code == code) {
			pr_err("%s key %s\n", dev_kmap.keymap[i].name,
					value ? "pressed" : "released");
			break;
		}
	}
}

inline void his_set_curr_backlight_state(u32 bl_level)
{
	static int last_level;

	/* print lcd backlight control log when turn on or off */
	if ((last_level == 0) || (bl_level == 0)) {
		pr_err("Backlight: set lcd backlight from %d to %d\n",
				last_level, bl_level);
		last_level = bl_level;

		if (bl_level == 0) {
			dev_bi.backlight_on = 0;
		} else {
			dev_bi.backlight_on = 1;
			update_backlight_on_time();
		}
	}
}

int __init alarm_normal_pon_setup(char *s)
{
	if (!strcmp(s, "1"))
		dev_bi.alarm_mode = 1;

	pr_err("%s: alarm_ahead_enable = %d\n", __func__, dev_bi.alarm_mode);

	return 1;
}
__setup("alarm_ahead.enable=", alarm_normal_pon_setup);

int __init boot_charger_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		dev_bi.bootmode = BOOT_CHARGER_MODE;
	else if (!strcmp(s, "factory2"))
		dev_bi.bootmode = BOOT_FACTORY_MODE;
	else if (!strcmp(s, "recovery"))
		dev_bi.bootmode = BOOT_RECOVERY_MODE;
	else if (!strcmp(s, "silence"))
		dev_bi.bootmode = BOOT_SILENCE_MODE;
	else
		dev_bi.bootmode = BOOT_NORMAL_MODE;

	pr_err("%s: buffer= %s bootmode= %d\n", __func__, s, dev_bi.bootmode);

	return 1;
}
__setup("androidboot.mode=", boot_charger_mode_init);

static int __init lcd_get_id(char *p)
{
	strlcpy(mdss_panel_lcd_id, p, LCD_ID_LEN);
	pr_err("%s: lcd_id= %s\n", __func__, mdss_panel_lcd_id);

	return 0;
}
__setup("lcd_id=", lcd_get_id);

static int __init powerup_reason_setup(char *p)
{
	strlcpy(poweron_reason, p, POWERON_REASON_LEN);
	pr_err("%s: poweron_reason= %s\n", __func__, poweron_reason);

	return 0;
}
early_param("powerup_reason", powerup_reason_setup);

static ssize_t powerup_reason_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, POWERON_REASON_LEN, "%s\n", poweron_reason);

	return ret;
}
BOOTINFO_ATTR(powerup_reason);

static int __init meid_status_setup(char *p)
{
	if (!strcmp(p, "0"))
		dev_bi.meid_is_null = 1;
	else
		dev_bi.meid_is_null = 0;

	pr_err("%s: meid_status= %d\n", __func__, dev_bi.meid_is_null);

	return 0;
}
early_param("androidboot.meid", meid_status_setup);

static ssize_t meid_is_null_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	char *meid_is_null_text[] = {
		"is_null",
		"is_not_null",
	};

	if (dev_bi.meid_is_null)
		s += snprintf(s, 15, "%s", meid_is_null_text[0]);
	else
		s += snprintf(s, 15, "%s", meid_is_null_text[1]);

	return s-buf;
}
BOOTINFO_ATTR(meid_is_null);

static int __init encrypt_status_setup(char *p)
{
	if (!strcmp(p, "0"))
		dev_bi.phone_is_encrypt = 0;
	else
		dev_bi.phone_is_encrypt = 1;

	pr_err("%s: encrypt_status= %d\n", __func__, dev_bi.phone_is_encrypt);

	return 0;
}
early_param("androidboot.encrypt", encrypt_status_setup);

static ssize_t encrypt_status_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, 16, "%d\n", dev_bi.phone_is_encrypt);

	return ret;
}
BOOTINFO_ATTR(encrypt_status);

static struct attribute *bootattr[] = {
	&powerup_reason_attr.attr,
	&meid_is_null_attr.attr,
	&encrypt_status_attr.attr,
	NULL,
};

static struct attribute_group bootattr_group = {
	.attrs = bootattr,
};

static void print_device_bootinfo(void)
{
	pr_err("===========================\n");
	pr_err("Boot mode:   %d\n", dev_bi.bootmode);
	pr_err("Meid null:   %d\n", dev_bi.meid_is_null);
	pr_err("Encrypt st:  %d\n", dev_bi.phone_is_encrypt);
	pr_err("Alarm mode:  %d\n", dev_bi.alarm_mode);
	pr_err("DDR size:    %lld\n", dev_bi.ddr_size);
	pr_err("sector num:  %lld\n", dev_bi.sectors_num);
	pr_err("Sector size: %d\n", dev_bi.sector_size);
	pr_err("===========================\n");
}

static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;
	struct kobject *bootinfo_kobj = NULL;

	of_parse_device_info();

	bootinfo_kobj = kobject_create_and_add("bootinfo", NULL);
	if (bootinfo_kobj == NULL) {
		pr_err("bootinfo_init: bootinfo kobject create failed\n");
		return ret;
	}

	ret = sysfs_create_group(bootinfo_kobj, &bootattr_group);
	if (ret) {
		pr_err("bootinfo_init: bootattr group create failed\n");
		goto sys_fail;
	}

	print_device_bootinfo();
	return ret;

sys_fail:
	kobject_del(bootinfo_kobj);
	return ret;
}
late_initcall(bootinfo_init);
