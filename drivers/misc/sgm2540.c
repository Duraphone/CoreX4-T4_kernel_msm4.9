/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/qpnp/qpnp-adc.h>
#include <../power/supply/qcom/smb-lib.h>

struct sgm2540_chip {
	struct platform_device *pdev;
	struct delayed_work chg_redet_work;
	struct power_supply		*batt_psy;
	//struct power_supply		*usb_psy;
	struct notifier_block	nb;
	struct delayed_work	usb_init_work;
	u32             sgm2540_af_gpio;
	u32             sgm2540_bf_gpio;
	int             sgm2540_af_irq;
	int             sgm2540_bf_irq;
	u32             b_channel_gpio;
	u32             usb_switch_gpio;
	int				a_otg_present;
	int				b_otg_present;
	int battery_fcc_ma;
	unsigned long inputs;
	bool	magconn_poweron;
#define AF_BUS_VLD	0
#define BF_BUS_VLD	1
#define A_OTG_VLD	2
#define B_OTG_VLD	3
#define SGM_INIT	7
};

enum sgm2540_mode
{
    sgm2540_default_mode = 0,
    magcon_otg_mode,
    usb_otg_mode,
    magcon_chg_mode,
    usb_chg_mode,
    magcon_usb_chg_mode,
};



static struct sgm2540_chip *global_chip = NULL;
static char *doubleinput[2] = { "AF_BF_EVENT=AF_BF_EVENT", NULL };

bool sgm2540_af_charger_ok(void)
{
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return 0;
	}

	return !!test_bit(AF_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_af_charger_ok);

bool sgm2540_bf_charger_ok(void)
{
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return 0;
	}

	return !!test_bit(BF_BUS_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_bf_charger_ok);

bool sgm2540_af_otg_ok(void)
{
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return 0;
	}

	return !!test_bit(A_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_af_otg_ok);

bool sgm2540_bf_otg_ok(void)
{
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return 0;
	}

	return !!test_bit(B_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_bf_otg_ok);

bool sgm2540_double_charger_ok(void)
{
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return 0;
	}

	return !!(test_bit(AF_BUS_VLD, &global_chip->inputs)
		&& test_bit(BF_BUS_VLD, &global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm2540_double_charger_ok);

static irqreturn_t sgm2540_af_irq(int irq, void *data)
{
	struct sgm2540_chip *chip = data;

	pr_info("sgm2540:%s\n", __func__);
		if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return -EINVAL;
	}
	if(test_bit(A_OTG_VLD, &(global_chip->inputs))
		|| test_bit(B_OTG_VLD, &(global_chip->inputs)))
		return IRQ_HANDLED;

	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static irqreturn_t sgm2540_bf_irq(int irq, void *data)
{
	struct sgm2540_chip *chip = data;

	pr_info("sgm2540:%s\n", __func__);
	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return -EINVAL;
	}
	if(test_bit(A_OTG_VLD, &(global_chip->inputs))
		|| test_bit(B_OTG_VLD, &(global_chip->inputs)))
		return IRQ_HANDLED;

	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

int sgm2540_a_otg_vbus_enable(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_err("sgm2540:%s err,no chip\n", __func__);
		return -EINVAL;
	}

	if (enable)
		global_chip->a_otg_present = true;
	else
		global_chip->a_otg_present = false;

	if (test_bit(BF_BUS_VLD, &(global_chip->inputs))) {
		pr_info("sgm2540:%s B charger work, a_otg_present = %d\n",
			__func__, global_chip->a_otg_present);
		return ret;
	}

	if (test_bit(B_OTG_VLD, &(global_chip->inputs))) {
		pr_info("sgm2540:%s B OTG work, a_otg_present = %d\n",
			__func__, global_chip->a_otg_present);
		return ret;
	}

	if (enable) {
		free_irq(global_chip->sgm2540_af_irq, global_chip);
		disable_irq(global_chip->sgm2540_bf_irq);
		ret = gpio_direction_output(global_chip->sgm2540_af_gpio, 1);
		clear_bit(AF_BUS_VLD, &global_chip->inputs);
		clear_bit(BF_BUS_VLD, &global_chip->inputs);
	} else {
		ret = gpio_direction_input(global_chip->sgm2540_af_gpio);
		ret |= request_irq(global_chip->sgm2540_af_irq,
					  sgm2540_af_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING,
					  "sgm2540_af_irq", global_chip);
		enable_irq_wake(global_chip->sgm2540_af_irq);
		enable_irq(global_chip->sgm2540_bf_irq);
		sgm2540_af_irq(0, global_chip);
	}

	ret = gpio_direction_output(global_chip->usb_switch_gpio, enable);
	enable ? set_bit(A_OTG_VLD, &global_chip->inputs) : clear_bit(A_OTG_VLD, &global_chip->inputs);

	pr_info("sgm2540:%s magconn otg vbus channel %s\n",
		enable?"enable":"disable", !ret?"success":"fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm2540_a_otg_vbus_enable);

int sgm2540_b_otg_vbus_enable(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_info("sgm2540:%s err, no chip\n", __func__);
		return -EINVAL;
	}

	if (enable)
		global_chip->b_otg_present = true;
	else
		global_chip->b_otg_present = false;

	if (test_bit(AF_BUS_VLD, &(global_chip->inputs))) {
		pr_info("sgm2540:%s A charger work, b_otg_present =%d\n",
			__func__, global_chip->b_otg_present);
		kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
		return ret;
	}

	if (test_bit(A_OTG_VLD, &(global_chip->inputs))) {
		pr_info("sgm2540:%s A OTG work, b_otg_present = %d\n",
			__func__, global_chip->b_otg_present);
		kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
		return ret;
	}

	if (enable){
		free_irq(global_chip->sgm2540_bf_irq, global_chip);
		disable_irq(global_chip->sgm2540_bf_irq);
		ret = gpio_direction_output(global_chip->b_channel_gpio, 1);
	}
	else{
		ret = gpio_direction_output(global_chip->b_channel_gpio, 0);
		ret |= request_irq(global_chip->sgm2540_bf_irq,
					  sgm2540_bf_irq,
					  IRQF_TRIGGER_RISING |
					  IRQF_TRIGGER_FALLING,
					  "sgm2540_bf_irq", global_chip);
		enable_irq_wake(global_chip->sgm2540_bf_irq);
		}

	enable ? set_bit(B_OTG_VLD, &global_chip->inputs) : clear_bit(B_OTG_VLD, &global_chip->inputs);
	kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);

	pr_info("sgm2540:%s typec otg vbus channel %s\n", enable?"enable":"disable", !ret?"success":"fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm2540_b_otg_vbus_enable);

static void sgm2540_chg_redet_work(struct work_struct *w)
{
	struct sgm2540_chip *chip = container_of(w, struct sgm2540_chip,
		chg_redet_work.work);
	bool af_status = !!gpio_get_value(chip->sgm2540_af_gpio);
	bool bf_status = !!gpio_get_value(chip->sgm2540_bf_gpio);
	bool need_redet = false;
	union power_supply_propval pval = {3000000, };
	union power_supply_propval val = {1, };

	pr_info("sgm2540:input 0x%lx, af=%d, bf=%d\n", chip->inputs, af_status, bf_status);

	if(((af_status << AF_BUS_VLD) | (bf_status << BF_BUS_VLD))
		== (chip->inputs & (BIT(AF_BUS_VLD) | BIT(BF_BUS_VLD)))) {
		pr_info("sgm2540:the same as last, do nothing\n");
		return;
	}

#if 0
	if (!chip->usb_psy) {
		chip->usb_psy = power_supply_get_by_name("usb");
		if (!chip->usb_psy) {
			dev_err(&chip->pdev->dev, "Could not get usb power_supply\n");
			return;
		}
	}
#endif

	if (!chip->batt_psy) {
		chip->batt_psy = power_supply_get_by_name("battery");

		if (!chip->batt_psy) {
			dev_err(&chip->pdev->dev, "Could not get battery power_supply\n");
			return;
		}
	}

	gpio_direction_output(chip->b_channel_gpio, bf_status && (!af_status));

	if(test_bit(AF_BUS_VLD, &chip->inputs) && test_bit(BF_BUS_VLD, &chip->inputs) 
		&& ((!af_status && bf_status) || (af_status && !bf_status))) {
		/*double charger to single charger*/
		need_redet = true;
		pr_info("sgm2540:need to redet charger type\n");
		goto set_afbf_flag;
	} else if(af_status && bf_status && 
		((test_bit(AF_BUS_VLD, &chip->inputs) && !test_bit(BF_BUS_VLD, &chip->inputs)) || 
		(!test_bit(AF_BUS_VLD, &chip->inputs) && test_bit(BF_BUS_VLD, &chip->inputs)))) {
		/*single charger to double charger*/
		pval.intval = 500 * 1000;
		pr_info("sgm2540:double input, data not switch to new bus\n");
		goto set_afbf_flag;
 	} else {
 		gpio_direction_output(chip->usb_switch_gpio, af_status && (!bf_status));
set_afbf_flag:
		af_status ? set_bit(AF_BUS_VLD, &chip->inputs) : clear_bit(AF_BUS_VLD, &chip->inputs);
		bf_status ? set_bit(BF_BUS_VLD, &chip->inputs) : clear_bit(BF_BUS_VLD, &chip->inputs);

		if(need_redet || chip->magconn_poweron) {
			val.intval = 1;
			chip->magconn_poweron = false;
			gpio_direction_output(chip->usb_switch_gpio, af_status && (!bf_status));
			chip->batt_psy->desc->set_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CHARGER_REDET, &val);
		}

		chip->batt_psy->desc->set_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
		kobject_uevent_env(&chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);

		pr_info("sgm2540:set fcc %d ma\n", pval.intval / 1000);	
	}
}

static void sgm2540_hw_init(struct device *dev, struct sgm2540_chip *chip)
{
	bool af_status = !!gpio_get_value(chip->sgm2540_af_gpio);
	bool bf_status = !!gpio_get_value(chip->sgm2540_bf_gpio);

	pr_info("@@@@%s:af_status=%d, bf_status=%d\n", __func__, af_status, bf_status);
	if (af_status && !bf_status) 
		chip->magconn_poweron = true;
	schedule_delayed_work(&chip->chg_redet_work, msecs_to_jiffies(5000));
}


static int sgm2540_parse_dt(struct device *dev,
		struct sgm2540_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/* sgm2540_af_gpio */
	ret = of_get_named_gpio(np, "sgmicro,sgm2540-af-gpio", 0);
	chip->sgm2540_af_gpio = ret;
	if(!gpio_is_valid(chip->sgm2540_af_gpio)) {
		dev_err(dev, "invalid sgm2540_af gpio. ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->sgm2540_af_gpio, "sgm2540-af-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->sgm2540_af_gpio, ret);
		return ret;
	}

	/* sgm2540_bf_gpio */
	ret = of_get_named_gpio(np, "sgmicro,sgm2540-bf-gpio", 0);
	chip->sgm2540_bf_gpio = ret;
	if (!gpio_is_valid(chip->sgm2540_bf_gpio)) {
		dev_err(dev,
			"invalid sgm2540-bf gpio: ret = %d\n", ret);
	}

	ret = gpio_request(chip->sgm2540_bf_gpio, "sgm2540-bf-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->sgm2540_bf_gpio, ret);
		return ret;
	}

	/*b_channel_gpio*/
	ret = of_get_named_gpio(np, "sgmicro,b-channel-gpio", 0);
	chip->b_channel_gpio = ret;
	if(!gpio_is_valid(chip->b_channel_gpio)) {
		dev_err(dev, "invalid b_channel gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->b_channel_gpio, "sgm2540-b-channel-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->b_channel_gpio, ret);
		return ret;
	}

	/*usb_switch_gpio*/
	ret = of_get_named_gpio(np, "sgmicro,usb-switch-gpio", 0);
	chip->usb_switch_gpio = ret;
	if(!gpio_is_valid(chip->usb_switch_gpio)) {
		dev_err(dev, "invalid usb_switch gpio: %d\n", ret);
		return ret;
	}
	printk("usb_switch_gpio is %d\n", chip->usb_switch_gpio);
    //gpio_free(chip->usb_switch_gpio);
	ret = gpio_request(chip->usb_switch_gpio, "sgm2540-usb-switch-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->usb_switch_gpio, ret);
		return ret;
	}

	return 0;
}

/*
static int sgm2540_notifier_call(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct sgm2540_chip *chip = container_of(nb, struct sgm2540_chip, nb);
	struct power_supply *psy = data;
	//union power_supply_propval pval = {0, };

	if (!chip->usb_psy && (strcmp(psy->desc->name, "usb") == 0)){
		pr_info("sgm2540:usb psy\n");
		chip->usb_psy = psy;
	}

	if (!chip->batt_psy && (strcmp(psy->desc->name, "battery") == 0)){
		pr_info("sgm2540:battery psy\n");
		//chip->batt_psy = psy;
		//psy->desc->get_property(psy,
			//POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
		//chip->battery_fcc_ma = pval.intval / 1000;
		pr_info("sgm2540:battery fcc %d ma\n", chip->battery_fcc_ma);
	}

	if(chip->usb_psy && chip->batt_psy ){
		pr_info("sgm2540:schedule usb_init_work\n");
		schedule_delayed_work(&chip->usb_init_work, 0);
	}

	return NOTIFY_OK;
}

static void sgm2540_usb_init_work(struct work_struct *work)
{
	struct sgm2540_chip *chip = container_of(work, struct sgm2540_chip,
					    usb_init_work.work);

	power_supply_unreg_notifier(&chip->nb);
	sgm2540_enable_otg_vbus(is_otg_present(chip));
	set_bit(SGM_INIT, &chip->inputs);
}
*/

static int sgm2540_mode_show(struct seq_file *s, void *unused)
{
    int mode = 0;
    struct sgm2540_chip *chip = s->private;

    if (test_bit(A_OTG_VLD, &global_chip->inputs))
		mode = magcon_otg_mode;
	else if (test_bit(B_OTG_VLD, &global_chip->inputs))
		mode = usb_otg_mode;
	else if ((test_bit(AF_BUS_VLD, &global_chip->inputs))
		&& (test_bit(BF_BUS_VLD, &global_chip->inputs)))
		mode = magcon_usb_chg_mode;
	else if (test_bit(AF_BUS_VLD, &global_chip->inputs))
		mode = magcon_chg_mode;
	else if (test_bit(BF_BUS_VLD, &global_chip->inputs))
		mode = usb_chg_mode;
	else
		mode = sgm2540_default_mode;
	dev_info(&chip->pdev->dev, "curent_mode = 0x%x\n", mode);

	seq_printf(s, "%d\n", mode);

	return 0;
}

static int sgm2540_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, sgm2540_mode_show, inode->i_private);
}

const struct file_operations sgm2540_mode_fops = {
	.open = sgm2540_mode_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int typec_attached_state_show(struct seq_file *s, void *unused)
{
    int attached_state = 0;
    struct sgm2540_chip *chip = s->private;
	bool bf_status = !!gpio_get_value(chip->sgm2540_bf_gpio);

	if (global_chip->b_otg_present || bf_status)
		attached_state = 1;
	else
		attached_state = 0;

	dev_info(&chip->pdev->dev, "attached_state = 0x%x\n", attached_state);
	seq_printf(s, "%d\n", attached_state);

	return 0;
}

static int typec_attached_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, typec_attached_state_show, inode->i_private);
}

const struct file_operations typec_attached_state_fops = {
	.open = typec_attached_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int sgm2540_debugfs_init(struct sgm2540_chip *chip)
{
	struct dentry *sgm2540_dentry;
    struct dentry *sgm2540_dbg_root;
	struct dentry *typec_dentry;
    struct dentry *typec_dbg_root;

	sgm2540_dbg_root = debugfs_create_dir("sgm2540", NULL);

	if (!sgm2540_dbg_root || IS_ERR(sgm2540_dbg_root))
		return -ENODEV;

	sgm2540_dentry = debugfs_create_file("mode", S_IRUGO,
		sgm2540_dbg_root, chip,
		&sgm2540_mode_fops);

	if (!sgm2540_dentry) {
		debugfs_remove_recursive(sgm2540_dbg_root);
		return -ENODEV;
	}

	typec_dbg_root = debugfs_create_dir("type_c", NULL);

	if (!typec_dbg_root || IS_ERR(typec_dbg_root))
		return -ENODEV;

	typec_dentry = debugfs_create_file("attached_state", S_IRUGO,
		typec_dbg_root, chip,
		&typec_attached_state_fops);

	if (!typec_dentry) {
		debugfs_remove_recursive(typec_dbg_root);
		return -ENODEV;
	}

	return 0;
}

static int sgm2540_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm2540_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip); 
	if (pdev->dev.of_node) {
		ret = sgm2540_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"%s: sgm2540_parse_dt() err\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		goto err_parse_dt;
	}

	global_chip = chip;
	INIT_DELAYED_WORK(&chip->chg_redet_work, sgm2540_chg_redet_work);

	chip->sgm2540_af_irq = gpio_to_irq(chip->sgm2540_af_gpio);
	chip->sgm2540_bf_irq = gpio_to_irq(chip->sgm2540_bf_gpio);

	ret = request_irq(chip->sgm2540_af_irq,
				  sgm2540_af_irq,
				  IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "sgm2540_af_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for af\n");
		goto err_parse_dt;
	}

	ret |= request_irq(chip->sgm2540_bf_irq,
				  sgm2540_bf_irq,
				  IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
				  "sgm2540_bf_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for bf\n");
		goto err_parse_dt;
	}

    enable_irq_wake(chip->sgm2540_af_irq);
    enable_irq_wake(chip->sgm2540_bf_irq);

	set_bit(SGM_INIT, &chip->inputs);
	sgm2540_hw_init(&pdev->dev, chip);

	sgm2540_debugfs_init(chip);

	pr_info("sgm2540:sgm2540_probe success\n");
	return 0;

err_parse_dt:
	devm_kfree(&pdev->dev,chip);
	return ret;
}

static void sgm2540_shutdown(struct platform_device *pdev)
{
	struct sgm2540_chip *chip = platform_get_drvdata(pdev);

	global_chip = NULL;
	pr_info("sgm2540:%s:b_channel_gpio is %d\n", __func__,
		gpio_get_value(chip->b_channel_gpio));
}

static int sgm2540_remove(struct platform_device *pdev)
{
	struct sgm2540_chip *chip = platform_get_drvdata(pdev);

	pr_info("sgm2540:%s:b_channel_gpio is %d\n", __func__,
		gpio_get_value(chip->b_channel_gpio));
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm2540_suspend(struct device *dev)
{
	struct sgm2540_chip *chip  = dev_get_drvdata(dev);

	pr_info("sgm2540:%s:b_channel_gpio is %d\n", __func__,
		gpio_get_value(chip->b_channel_gpio));
	return 0;
}

static int sgm2540_resume(struct device *dev)
{
	struct sgm2540_chip *chip  = dev_get_drvdata(dev);

	pr_info("sgm2540:%s:b_channel_gpio is %d\n", __func__,
		gpio_get_value(chip->b_channel_gpio));
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm2540_pm_ops, sgm2540_suspend,
			  sgm2540_resume);

static struct of_device_id sgm2540_match_table[] = {
	{	.compatible = "sgmicro,sgm2540",
	},
	{}
};

static struct platform_driver sgm2540_driver = {
	.driver		= {
		.name		= "sgm2540",
		.owner		= THIS_MODULE,
		.of_match_table	= sgm2540_match_table,
		.pm	= &sgm2540_pm_ops,
	},
	.probe		= sgm2540_probe,
	.remove		= sgm2540_remove,
	.shutdown 	= sgm2540_shutdown,
};

module_platform_driver(sgm2540_driver);
MODULE_DESCRIPTION("sgm2540");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm2540");
