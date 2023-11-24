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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/awaken_sys_event.h>
#include <linux/his_debug_base.h>

#define DEBUGFS_FILE_CREATE(name) \
	static int name##_open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, name##_show, inode->i_private); \
	} \
	  \
	static const struct file_operations name##_ops = { \
		.open = name##_open, \
		.read =	seq_read, \
		.llseek = seq_lseek, \
		.release = single_release, \
	};


static int registered_event[AWAKEN_EVENT_MAX];
/* last awaken system event and time(ns) */
static struct awaken_event last_event;

int register_irqnum_and_event(int irq_number, int event)
{
	if (event >= AWAKEN_EVENT_MAX)
		return -1;

	registered_event[event] = irq_number;
	return 0;
}

void set_awaken_event_by_irqnum(int curr_irq)
{
	last_event.event = curr_irq;
	last_event.recv_nsec = local_clock();
}

void clear_awaken_system_event(void)
{
	last_event.event = AWAKEN_EVENT_NONE;
	last_event.recv_nsec = 0;
}

int get_current_awaken_event(void)
{
	int i = 0;

	for (i = 1; i < AWAKEN_EVENT_MAX; i++) {
		if ((registered_event[i] != 0)
				&& (last_event.event == registered_event[i])) {
			return i;
		}
	}

	return AWAKEN_EVENT_UNKNOWN;
}

void update_backlight_on_time(void)
{
	static int record_index;

	last_event.bl_on_nsec = local_clock();

	if (last_event.recv_nsec != 0) {
		last_event.timebank[record_index] =
			last_event.bl_on_nsec - last_event.recv_nsec;
		record_index ++;
		if (record_index == WAKEUP_TIME_COUNT)
			record_index = 0;
	}
}

void update_screen_init_time(int start)
{
	if (start == 1)
		last_event.lcd_start_nsec = local_clock();
	else
		last_event.lcd_end_nsec = local_clock();
}

static const char *get_wakeup_irqname(int irq_num)
{
	struct irq_desc *desc;
	const char *name = "null";

	desc = irq_to_desc(irq_num);
	if (desc == NULL)
		name = "stray irq";
	else if (desc->action && desc->action->name)
		name = desc->action->name;

	return name;
}

static int events_list_show(struct seq_file *s, void *data)
{
	int i = 0;
	const char *irqname = NULL;

	seq_puts(s, "Register events list:\n");
	for (i = 1; i < AWAKEN_EVENT_MAX; i++) {

		if (registered_event[i] == 0)
			continue;

		irqname = get_wakeup_irqname(registered_event[i]);
		seq_printf(s, "  %d: %s\n", i, irqname);
	}

	return 0;
}

static int curr_event_show(struct seq_file *s, void *data)
{
	int i = 0;
	u64 sec = 0;
	unsigned long rem_nsec;

	seq_printf(s, "Current awaken infomation:\n");
	seq_printf(s, "\tLast awaken IRQ: %s\n", get_wakeup_irqname(last_event.event));

	sec = last_event.recv_nsec;
	rem_nsec = do_div(sec, 1000000000);
	seq_printf(s, "\tReceive wake time  : [%5lu.%06lu]\n",
			(unsigned long)sec, rem_nsec / 1000);

	sec = last_event.lcd_start_nsec;
	rem_nsec = do_div(sec, 1000000000);
	seq_printf(s, "\tLcd init start time: [%5lu.%06lu]\n",
			(unsigned long)sec, rem_nsec / 1000);

	sec = last_event.lcd_end_nsec;
	rem_nsec = do_div(sec, 1000000000);
	seq_printf(s, "\tLcd init end time  : [%5lu.%06lu]\n",
			(unsigned long)sec, rem_nsec / 1000);

	sec = last_event.bl_on_nsec;
	rem_nsec = do_div(sec, 1000000000);
	seq_printf(s, "\tBacklight on time  : [%5lu.%06lu]\n",
			(unsigned long)sec, rem_nsec / 1000);

	seq_printf(s, "\tLast 5 wake time(ns): \n\t");
	for (i = 0; i < WAKEUP_TIME_COUNT; i++)
		seq_printf(s, "\t%lu", (unsigned long)last_event.timebank[i]);

	seq_printf(s, "\n");

	return 0;
}

DEBUGFS_FILE_CREATE(events_list);
DEBUGFS_FILE_CREATE(curr_event)

static int __init awaken_event_init(void)
{
	struct dentry *debugfs_dir;

	debugfs_dir = his_register_debugfs_dir("awaken_events");
	if (!debugfs_dir)
		return -ENOMEM;

	debugfs_create_file("events_list", S_IRUGO, debugfs_dir, NULL,
			&events_list_ops);
	debugfs_create_file("curr_event", S_IRUGO, debugfs_dir, NULL,
			&curr_event_ops);

	return 0;
}
late_initcall(awaken_event_init);
