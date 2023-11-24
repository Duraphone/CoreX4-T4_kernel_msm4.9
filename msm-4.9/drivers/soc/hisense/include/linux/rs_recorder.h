/*
 * rs_recorder (Runtime State Recorder) Module
 *
 * Copyright (C) 2015 Hisense, Inc.
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

#ifndef __RS_RECORDER_H__
#define __RS_RECORDER_H__

extern void rs_exec_dump_task(const char *reason, int level, int line,
		int is_panic);

#ifdef CONFIG_RS_RECORDER_SUPPORT
struct rs_recorder_log;
typedef int (*dump_func)(struct rs_recorder_log *rs_log, char *fname);

/*
 * register module special dump operations
 * name:     module name
 * level:    dump level(1~3), 1 is highest priority
 * savename: save file name
 * dump_ops: dump callback function
 */
extern int rs_register_dumpops(char *name, int level, char *savename,
		dump_func dump_ops);
#endif /* CONFIG_RS_RECORDER_SUPPORT */

#endif /* __RS_RECORDER_H__ */

