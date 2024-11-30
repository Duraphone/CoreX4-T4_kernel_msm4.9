/*
 * EX(Exception) Monitor Module
 *
 * Copyright (C) 2017 Hisense, Inc.
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

#define pr_fmt(fmt) "EX-MON: " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/notifier.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>

struct exmlog_hr {
	u64 ts_nsec;	/* timestamp in nanoseconds */
	u16 len;		/* length of entire record */
	u16 text_len;	/* length of text buffer */
}
#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
__packed __aligned(4)
#endif
;

#define MSG_LINE_MAX      (512)
#define EXMLOG_ALIGN      __alignof__(struct exmlog_hr)
#define EXM_BUF_SIZE      (1 << CONFIG_LOG_BUF_SHIFT)
static char __exmlog_buf[EXM_BUF_SIZE] __aligned(EXMLOG_ALIGN);
static char *exmlog_buf   = __exmlog_buf;
static u32 exmlog_buf_len = EXM_BUF_SIZE;

static u64 proclog_seq;
static u32 proclog_idx;

/* index and sequence number of the first record stored in the buffer */
static u64 log_first_seq;
static u32 log_first_idx;

/* index and sequence number of the next record to store in the buffer */
static u64 log_next_seq;
static u32 log_next_idx;

/* The exmlog_lock protects log buffer, indices, counters. */
static DEFINE_RAW_SPINLOCK(exmlog_lock);

static DECLARE_WAIT_QUEUE_HEAD(exmlog_wait);

static struct work_struct wakelog_work;

/* human readable text of the record */
static char *log_text(const struct exmlog_hr *log_hr)
{
	return (char *)log_hr + sizeof(struct exmlog_hr);
}

static char *log_padding(const struct exmlog_hr *log_hr)
{
	return (char *)log_hr + sizeof(struct exmlog_hr) + log_hr->text_len;
}

/* get record by index; idx must point to valid msg */
static struct exmlog_hr *log_from_idx(u32 idx)
{
	struct exmlog_hr *msg = (struct exmlog_hr *)(exmlog_buf + idx);

	/*
	 * A length == 0 record is the end of buffer marker. Wrap around and
	 * read the message at the start of the buffer.
	 */
	if (!msg->len)
		return (struct exmlog_hr *)exmlog_buf;

	return msg;
}

static size_t exm_print_time(u64 ts, char *buf)
{
	unsigned long rem_nsec;

	if (!IS_ENABLED(CONFIG_PRINTK_TIME))
		return 0;

	rem_nsec = do_div(ts, 1000000000);

	if (!buf)
		return snprintf(NULL, 0, "[%5lu.000000] ", (unsigned long)ts);

	return sprintf(buf, "[%5lu.%06lu] ",
		       (unsigned long)ts, rem_nsec / 1000);
}

/* compute the message size including the padding bytes */
static u32 msg_used_size(u16 text_len, u32 *pad_len)
{
	u32 size;

	size = sizeof(struct exmlog_hr) + text_len;
	*pad_len = (-size) & (EXMLOG_ALIGN - 1);
	size += *pad_len;

	return size;
}

/* get next record; idx must point to valid log_hr */
static u32 log_next(u32 idx)
{
	struct exmlog_hr *log_hr = (struct exmlog_hr *)(exmlog_buf + idx);

	/* length == 0 indicates the end of the buffer; wrap */
	/*
	 * A length == 0 record is the end of buffer marker. Wrap around and
	 * read the message at the start of the buffer as *this* one, and
	 * return the one after that.
	 */
	if (!log_hr->len) {
		log_hr = (struct exmlog_hr *)exmlog_buf;
		return log_hr->len;
	}

	return idx + log_hr->len;
}

static int logbuf_has_space(u32 msg_size, bool empty)
{
	u32 free;

	if (log_next_idx > log_first_idx || empty)
		free = max(exmlog_buf_len - log_next_idx, log_first_idx);
	else
		free = log_first_idx - log_next_idx;

	/*
	 * We need space also for an empty header that signalizes wrapping
	 * of the buffer.
	 */
	return free >= msg_size + sizeof(struct exmlog_hr);
}

static int log_make_free_space(u32 msg_size)
{
	while (log_first_seq < log_next_seq) {
		if (logbuf_has_space(msg_size, false))
			return 0;
		/* drop old messages until we have enough contiguous space */
		log_first_idx = log_next(log_first_idx);
		log_first_seq++;
	}

	/* sequence numbers are equal, so the log buffer is empty */
	if (logbuf_has_space(msg_size, true))
		return 0;

	return -ENOMEM;
}

static int exmlog_store(const char *text, u16 text_len)
{
	struct exmlog_hr *log_hr;
	u32 size, pad_len;

	/* number of '\0' padding bytes to next message */
	size = msg_used_size(text_len, &pad_len);
	
	if (log_make_free_space(size))
		return 0;

	if (log_next_idx + size + sizeof(struct exmlog_hr) > exmlog_buf_len) {
		/*
		 * This message + an additional empty header does not fit
		 * at the end of the buffer. Add an empty header with len == 0
		 * to signify a wrap around.
		 */
		memset(exmlog_buf + log_next_idx, 0, sizeof(struct exmlog_hr));
		log_next_idx = 0;
	}

	/* fill message */
	log_hr = (struct exmlog_hr *)(exmlog_buf + log_next_idx);
	memcpy(log_text(log_hr), text, text_len);
	log_hr->text_len = text_len;

	/* only save clock, convert time when cat log */
	log_hr->ts_nsec = local_clock();
	memset(log_padding(log_hr), 0, pad_len);
	log_hr->len = size;

	/* insert message */
	log_next_idx += log_hr->len;
	log_next_seq++;

	return log_hr->text_len;
}

int print_exmlog(const char *fmt, ...)
{
	int msg_len;
	va_list args;
	static char tmpbuf[MSG_LINE_MAX];

	raw_spin_lock(&exmlog_lock);
	va_start(args, fmt);
	msg_len = vscnprintf(tmpbuf, sizeof(tmpbuf), fmt, args);
	va_end(args);

	msg_len = exmlog_store(tmpbuf, msg_len);
	raw_spin_unlock(&exmlog_lock);

	if (waitqueue_active(&exmlog_wait))
		schedule_work(&wakelog_work);

	return msg_len;
}
EXPORT_SYMBOL(print_exmlog);

static size_t msg_print_text(const struct exmlog_hr *msg,
		char *buf, size_t size)
{
	const char *text = log_text(msg);
	size_t text_size = msg->text_len;
	size_t len = 0;

	len += exm_print_time(msg->ts_nsec, buf);
	/* check buffer overflow, reserve +1 for '\0' */
	if ((len + text_size) > size)
		text_size = size - len - 1;
	memcpy(buf + len, text, text_size);
	len += text_size;

	return len;
}

static ssize_t exmlog_read(struct file *file, char __user *buf,
			 size_t count, loff_t *ppos)
{
	char *text;
	struct exmlog_hr *msg;
	int len = 0;
	int ret = 0;

	pr_info("enter %s\n", __func__);
	if (!buf || count <= 0) {
		pr_err("the buf or count is err\n");
		return -EINVAL;
	}

	raw_spin_lock_irq(&exmlog_lock);
	while (proclog_seq == log_next_seq) {
		if (file->f_flags & O_NONBLOCK) {
			len = -EAGAIN;
			raw_spin_unlock_irq(&exmlog_lock);
			pr_err("the file f_flags is non-block\n");
			goto out;
		}

		raw_spin_unlock_irq(&exmlog_lock);
		ret = wait_event_interruptible(exmlog_wait,
					proclog_seq != log_next_seq);
		if (ret) {
			len = ret;
			pr_err("wait event error: %d\n", ret);
			goto out;
		}
		raw_spin_lock_irq(&exmlog_lock);
	}
	raw_spin_unlock_irq(&exmlog_lock);

	text = kzalloc(MSG_LINE_MAX, GFP_KERNEL);
	if (!text) {
		pr_err("get memory failed\n");
		return -ENOMEM;
	}

	while (count > 0) {
		size_t n;

		raw_spin_lock_irq(&exmlog_lock);
		if (proclog_seq < log_first_seq) {
			/* messages are gone, move to first one */
			proclog_seq = log_first_seq;
			proclog_idx = log_first_idx;
		}

		if (proclog_seq == log_next_seq) {
			raw_spin_unlock_irq(&exmlog_lock);
			break;
		}

		msg = log_from_idx(proclog_idx);
		n = msg_print_text(msg, text, MSG_LINE_MAX);
		proclog_idx = log_next(proclog_idx);
		proclog_seq++;
		raw_spin_unlock_irq(&exmlog_lock);

		if (copy_to_user(buf, text, n))
			break;

		len += n;
		count -= n;
		buf += n;
	}

	kfree(text);
out:
	return len;
}

static const struct file_operations proc_exmlog_operations = {
	.read		= exmlog_read,
};

static void wakeup_logread_work_func(struct work_struct *work)
{
	wake_up_interruptible(&exmlog_wait);
}

static int __init proc_exmlog_init(void)
{
	proc_create("kexmlog", S_IRUGO, NULL, &proc_exmlog_operations);
	INIT_WORK(&wakelog_work, wakeup_logread_work_func);

	return 0;
}
fs_initcall(proc_exmlog_init);

