#ifndef __HIS_DEBUG_CONTROL_NODE_H__
#define __HIS_DEBUG_CONTROL_NODE_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/fs.h>

enum {
	BOOT_NORMAL_MODE,
	BOOT_CHARGER_MODE,
	BOOT_FACTORY_MODE,
	BOOT_RECOVERY_MODE,
	BOOT_SILENCE_MODE
};

struct device_bootinfo {
	u8 bootmode;
	u8 meid_is_null;
	u8 alarm_mode;
	u8 backlight_on;
	u32 sector_size;
	u64 sectors_num;
	u64 ddr_size;
	u8 phone_is_encrypt;
};
extern struct device_bootinfo dev_bi;

#define PRINT_OUT(m, x...) \
	do { \
		if (m) \
			seq_printf(m, x); \
		else \
			pr_err(x); \
	} while (0)

/* debug flag defined */
enum {
	DEBUG_ENABLE_BIT       = 1U << 1,
	PRINT_WAKELOCK_BIT     = 1U << 2,
	SERIAL_ENABLE_BIT      = 1U << 3,
};

void set_debug_flag_bit(int set_bit);
void clear_debug_flag_bit(int set_bit);
bool get_debug_flag_bit(int get_bit);

/* sysfs node interface */
extern int his_register_sysfs_attr(struct attribute *attr);
extern int his_register_sysfs_attr_group(struct attribute_group *attr_group);
extern struct kobject *his_register_sysfs_dir(const char *name);

/* debugfs node interface */
extern int his_register_debugfs_file(const char *name, umode_t mode, void *data,
		const struct file_operations *fops);
extern struct dentry *his_register_debugfs_dir(const char *name);

/* procfs node interface */
extern int his_create_procfs_file(const char *name, umode_t mode,
		const struct file_operations *fops);
extern struct proc_dir_entry *his_create_procfs_dir(const char *name);

/* read/write file interface */
extern int his_read_file(const char *path, loff_t offset, void *buf, u32 size);
extern int his_write_file(const char *path, loff_t offset, void *buf, u32 size);

/* get current time */
extern void his_get_current_time(char *ptime, int size);

/* key and backlight log */
extern void input_print_keyevent_log(u32 type, u32 code, int value);
extern void his_set_curr_backlight_state(u32 bl_level);

/* memory info */
extern unsigned long get_hs_total_ram(void);

/* run fs_sync when system to die */
extern void run_fs_sync_work(void);

/* charger action log, about duration etc */
extern void charger_log_start_charge(int curr_cap);

extern void charger_log_charger_type(char *chgr_type);

extern void charger_log_finish_charge(void);

/* Append RTB by debug_flag */
extern void hs_set_rtb_flag(int value);


#endif	/* __HIS_DEBUG_CONTROL_NODE_H__ */
