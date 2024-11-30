/*
 * Copyright (C) 2018 Hisense, Inc.
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
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/pm_wakeup.h>
#include <linux/syscalls.h>
#include <linux/suspend.h>

#define SUSPEND_SYS_SYNC_TIMEOUT (HZ/4)

static void suspend_sys_sync_handler(unsigned long arg);
static void suspend_sys_sync(struct work_struct *work);

static int suspend_sys_sync_count;
static bool suspend_sys_sync_abort;
static struct workqueue_struct *suspend_sys_sync_workqueue;

static DEFINE_SPINLOCK(suspend_sys_sync_lock);
static DECLARE_COMPLETION(suspend_sys_sync_comp);
static void suspend_sys_sync_handler(unsigned long);
static DEFINE_TIMER(suspend_sys_sync_timer, suspend_sys_sync_handler, 0, 0);

static void suspend_sys_sync(struct work_struct *work)
{
	pr_info("PM: Syncing filesystems ... ");
	sys_sync();
	pr_cont("done.\n");

	spin_lock(&suspend_sys_sync_lock);
	suspend_sys_sync_count--;
	spin_unlock(&suspend_sys_sync_lock);
}
static DECLARE_WORK(suspend_sys_sync_work, suspend_sys_sync);

void suspend_sys_sync_queue(void)
{
	int ret = 0;

	spin_lock(&suspend_sys_sync_lock);
	ret = queue_work(suspend_sys_sync_workqueue, &suspend_sys_sync_work);
	if (ret)
		suspend_sys_sync_count++;
	spin_unlock(&suspend_sys_sync_lock);
}
EXPORT_SYMBOL(suspend_sys_sync_queue);

static void suspend_sys_sync_handler(unsigned long arg)
{
	if (suspend_sys_sync_count == 0) {
		complete(&suspend_sys_sync_comp);
	} else if (pm_wakeup_pending()) {
		suspend_sys_sync_abort = true;
		complete(&suspend_sys_sync_comp);
	} else {
		mod_timer(&suspend_sys_sync_timer, jiffies +
				SUSPEND_SYS_SYNC_TIMEOUT);
	}
}

int suspend_sys_sync_wait(void)
{
	suspend_sys_sync_abort = false;

	if (suspend_sys_sync_count != 0) {
		mod_timer(&suspend_sys_sync_timer, jiffies +
				SUSPEND_SYS_SYNC_TIMEOUT);
		wait_for_completion(&suspend_sys_sync_comp);
	}

	if (suspend_sys_sync_abort) {
		pr_err("suspend aborted....while waiting for sys_sync\n");
		return -EBUSY;
	}

	return 0;
}
EXPORT_SYMBOL(suspend_sys_sync_wait);

static int __init sys_sync_queue_init(void)
{
	int ret = 0;

	init_completion(&suspend_sys_sync_comp);

	suspend_sys_sync_workqueue =
			create_singlethread_workqueue("suspend_sys_sync");
	if (suspend_sys_sync_workqueue == NULL)
		ret = -ENOMEM;

	return ret;
}

core_initcall(sys_sync_queue_init);
