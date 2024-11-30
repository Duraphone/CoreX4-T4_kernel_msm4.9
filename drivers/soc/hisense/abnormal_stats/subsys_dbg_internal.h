
#ifndef __SUBSYS_DBG_INTERNAL_H__
#define __SUBSYS_DBG_INTERNAL_H__

#define SUBSYS_NAME_LEN      8
#define REASON_STR_LEN       128
#define KDEBUG_SAVED_MAGIC   0x5353ACAC

struct subsys_trap_info {
	char name[SUBSYS_NAME_LEN];
	char last_reason[REASON_STR_LEN];
	int count;
	int saved;
};

/* Do not record the subsys exception when debug version */
extern int hs_get_debug_flag(void);

#ifdef CONFIG_ABNORMAL_STATS
extern void update_abnormal_stats(const char *name, const char *reason);
extern void reload_abnormal_stats_work(void);
extern void set_modem_subsys_reset(char *reset_time);
#else  /* CONFIG_ABNORMAL_STATS */
static void set_modem_subsys_reset(char *reset_time) {}
#endif /* CONFIG_ABNORMAL_STATS */

#endif /* __SUBSYS_DBG_INTERNAL_H__ */

