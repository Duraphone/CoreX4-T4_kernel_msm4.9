#ifndef __HI_SMART_SHRINKER_H__
#define __HI_SMART_SHRINKER_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mutex.h>

struct smart_shrinker {
	bool                enable_swap;
	bool                enable_shrinker;

	/* kthread handler, this also is enable status flag. */
	struct task_struct	*ktask;
	/* wakeup/wait for working thread. */
	struct semaphore	wait;
	/* sysfs node */
	struct kobject		*kobj;

	/**
	 * swap dirty pages to zram
	 */
	/* counter for recording wakeup times. */
	unsigned int		swap_count;
	/* backlight status */
	bool                backlight_off;
	/* cpu load threshld in percent for compress action. */
	int					idle_threshold;
	/* min free pages to keep in our system. */
	int					free_pages_min;
	/* min anon pages kept uncompress. */
	int					anon_pages_min;
	/* counter for total normal clean pages. */
	unsigned int		nr_swap_pages;
	/* jiffies counter for all swap time. */
	unsigned int		total_swap_time;
	/* last cpu load stat, for calculate cpu idle. */
	u64                 last_idle_time;
	u64                 last_timestamp;
	/* cpu load in percent. */
	int					cpu_load[3];
	/* last run the swap jeffies */
	unsigned long       last_swap_jiffies;

	/**
	 * compact the swap memory when over the watermark(%)
	 */
	unsigned int        compact_threshold;
	unsigned int        compact_count;
	unsigned int        compact_finished;
	unsigned int        last_compact_jiffies;

	/**
	 * shrink system memory when over the watermark
	 */
	unsigned int        shrink_min_vmpr;
	unsigned int        shrink_max_vmpr;
	unsigned int        shrink_count;
	/* last run the shrinker jeffies */
	unsigned long       last_shrink_jiffies;
};

extern void shrink_mem_by_lmk_rpt(int percent);
extern void run_compact_zram(void);
extern void lmk_get_shrink_count(u32 *kill_cnt, u32 *shrink_cnt);

#endif /* __HI_SMART_SHRINKER_H__ */
