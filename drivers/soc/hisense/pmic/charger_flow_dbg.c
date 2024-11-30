/*
 * Copyright (C) 2020 Hisense, Inc.
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

#define pr_fmt(fmt) "his-chg: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/pm.h>
#include <linux/his_debug_base.h>

void charger_log_start_charge(int curr_cap)
{
	char time_str[32] = {0};

	his_get_current_time(time_str, sizeof(time_str));
	pr_buf_err("%s start charge, capacity: %d\n", time_str, curr_cap);
}

void charger_log_charger_type(char *chgr_type)
{
	pr_buf_err("charger type: %s\n", chgr_type);
}

void charger_log_finish_charge(void)
{
	char time_str[32] = {0};

	his_get_current_time(time_str, sizeof(time_str));
	pr_buf_err("%s finish charge.\n", time_str);
}

