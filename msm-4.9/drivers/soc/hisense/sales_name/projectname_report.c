/*
 * Copyright (C) 2018 Hisense, Inc.
 *
 * Author:
 *   zhengfengxia <zhengfengxia@hisense.com>
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
#define pr_fmt(fmt) "project-name: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/his_debug_base.h>

#define PRJNAME_STR_LEN   30
#define DIAG_PARTITION    "/dev/block/bootdevice/by-name/diag"
#define PARTNO_OFFSET     1288
#define PARTNO_LENGTH     10
#define MAX_PROJECTS_NUM  10
#define SIZE_1G           (1024*1024*1024)

struct support_projects {
	int read_via_diag;
	int hwinfo_ver;
	int hw_boot_prd;
	int num;
	const char *all_projects[MAX_PROJECTS_NUM][2];
};

static char projectname[PRJNAME_STR_LEN];
static struct support_projects curr_projects;



static int __init boot_prd_setup(char *p)
{
	if (!strcmp(p, "1"))
		curr_projects.hw_boot_prd = 1;
	else
		curr_projects.hw_boot_prd = 0;

	pr_err("%s: hw_boot_prd= %d\n", __func__, curr_projects.hw_boot_prd);

	return 0;
}
early_param("boot_prd", boot_prd_setup);


static int project_get_emmc_size(void)
{
	int emmc_size = 0;

	emmc_size = dev_bi.sectors_num * dev_bi.sector_size / SIZE_1G;
	pr_err("Emmc sectors num: %lld emmc size: %dGB\n",
			dev_bi.sectors_num, emmc_size);

	if (emmc_size <= 8)
		return 8;
	else if (emmc_size <= 16)
		return 16;
	else if (emmc_size <= 32)
		return 32;
	else if (emmc_size <= 64)
		return 64;
	else if (emmc_size <= 128)
		return 128;
	else if (emmc_size <= 256)
		return 256;

	pr_err("There is no proper EMMC size found BUG\n");
	return 0;
}

static int project_get_ddr_size(void)
{
	int ddr_size;
	long total_ram_size;

	total_ram_size = get_hs_total_ram();
	ddr_size = total_ram_size / SIZE_1G;
	pr_err("Total DDR: %lu Size: %dGB\n", total_ram_size, ddr_size);

	return ddr_size;
}

static int get_project_name_via_hardware(void)
{
	int ddr_size = 0;
	int emmc_size = 0;
	char str[PRJNAME_STR_LEN] = {0};
	int i = 0;
	int cur_len = 0;

	pr_info("enter %s\n", __func__);
	ddr_size = project_get_ddr_size();
	emmc_size = project_get_emmc_size();
	cur_len = sprintf(str, "%d+%d", ddr_size, emmc_size);

	pr_info("product hardware is %s, num is %d\n",
			str, curr_projects.num);
	for (i = 0; i < curr_projects.num; i++) {
		const char *hwinfo = NULL;
		const char *name_ptr = NULL;

		hwinfo = curr_projects.all_projects[i][0];
		name_ptr = curr_projects.all_projects[i][1];
		if ((cur_len == strlen(hwinfo)) && (!strcmp(str, hwinfo))) {
			strcpy(projectname, name_ptr);
			break;
		}
	}
	pr_buf_info("current project name is %s\n", projectname);

	return 0;
}

static int get_project_name_via_hardware_v1(void)
{
	int ddr_size = 0;
	int emmc_size = 0;
	char str[PRJNAME_STR_LEN] = {0};
	int i = 0;
	int cur_len = 0;

	pr_info("enter %s\n", __func__);
	ddr_size = project_get_ddr_size();
	emmc_size = project_get_emmc_size();
	cur_len = sprintf(str, "%d+%d+%d", ddr_size, emmc_size,
          curr_projects.hw_boot_prd);

	pr_info("product hardware is %s, num is %d\n",
			str, curr_projects.num);
	for (i = 0; i < curr_projects.num; i++) {
		const char *hwinfo = NULL;
		const char *name_ptr = NULL;

		hwinfo = curr_projects.all_projects[i][0];
		name_ptr = curr_projects.all_projects[i][1];
		if ((cur_len == strlen(hwinfo)) && (!strcmp(str, hwinfo))) {
			strcpy(projectname, name_ptr);
			break;
		}
	}
	pr_buf_info("current project name is %s\n", projectname);

	return 0;
}

static int read_partno_from_diag_partition(char *buff)
{
	int ret = 0;

	ret = his_read_file(DIAG_PARTITION, PARTNO_OFFSET,
			buff, PARTNO_LENGTH);
	if (ret <= 0)
		return -EIO;

	pr_info("The phone partNO is %s\n", buff);

	return 0;
}

static void get_project_name_via_diag(void)
{
	int i = 0;
	int ret = 0;
	char partNO[PARTNO_LENGTH + 1] = {0};

	pr_info("enter %s\n", __func__);
	ret = read_partno_from_diag_partition(partNO);
	if (ret < 0) {
		pr_err("Error to read PartNo in diag partition!\n");
		return;
	}

	for (i = 0; i < curr_projects.num; i++) {
		const char *serial = NULL;
		const char *name_ptr = NULL;

		serial = curr_projects.all_projects[i][0];
		name_ptr = curr_projects.all_projects[i][1];
		ret = memcmp(partNO, serial, PARTNO_LENGTH);
		if (ret == 0) {
			strlcpy(projectname, name_ptr, sizeof(projectname));
			pr_buf_info("Project name is %s\n", projectname);
			break;
		}
	}
}

static int projectname_proc_show(struct seq_file *m, void *v)
{
	static int first_time = 1;

	if (first_time) {
		if (curr_projects.read_via_diag == 1)
			get_project_name_via_diag();
		else if (curr_projects.hwinfo_ver == 1)
			get_project_name_via_hardware_v1();
    else
			get_project_name_via_hardware();
		first_time = 0;
	}
	seq_write(m, projectname, strlen(projectname));

	return 0;
}

static int projectname_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, projectname_proc_show, NULL);
}

static const struct file_operations projectname_proc_fops = {
	.open       = projectname_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

int of_parse_projectname_config(void)
{
	int i = 0, j = 0;
	struct device_node *np = NULL;
	int size;
	int rc, temp;

	np = of_find_node_by_path("/soc/his_projectname");
	if (!np) {
		pr_err("Can not find his_projectname node\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "dev,read_via_diag", &temp);
	if (!rc) {
		curr_projects.read_via_diag = (u32)temp;
	} else if (rc != -EINVAL) {
		pr_err("Unable to read read_via_diag rc=%d\n", rc);
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "dev,hwinfo_version", &temp);
	if (!rc) {
		curr_projects.hwinfo_ver = (u32)temp;
	} else if (rc != -EINVAL) {
		pr_err("Unable to read read_via_diag rc=%d\n", rc);
		return -EINVAL;
	}

	size = of_property_count_strings(np, "dev,projectname-list");
	if ((size <= 0) || ((size/2) > MAX_PROJECTS_NUM)) {
		pr_err("the project list size is error\n");
		return -EINVAL;
	}

	curr_projects.num = size / 2;
	pr_err("found projectname num: %d\n", curr_projects.num);
	for (i = 0, j = 0; i < size; i++) {

		if ((i % 2) == 0) {
			of_property_read_string_index(np, "dev,projectname-list",
				i, &curr_projects.all_projects[j][0]);
		} else {
			of_property_read_string_index(np, "dev,projectname-list",
				i, &curr_projects.all_projects[j][1]);
			j++;
		}
	}

	return 0;
}

static int __init projectname_report_init(void)
{
	int ret = 0;

	ret = of_parse_projectname_config();
	if (ret < 0)
		return -EINVAL;

	ret = his_create_procfs_file("projectname", 0444,
			&projectname_proc_fops);
	if (ret < 0)
		return -ENOMEM;

	return 0;
}
module_init(projectname_report_init);
