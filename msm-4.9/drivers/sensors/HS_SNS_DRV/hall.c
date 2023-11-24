/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>

#define HALL_VDD_MIN_UV 1750000
#define HALL_VDD_MAX_UV 1950000

struct hall_descriptor {
	struct platform_device *p_dev;
	struct input_dev *input_dev;
	struct sensors_classdev h_cdev;

	struct regulator *pldo;
	bool power_on;
	struct wakeup_source wlock;
	int hall_enable;
	int gpio_num;
	int irq;
	char hall_state;
	struct pinctrl *pinctrl_hall;
	struct pinctrl_state *pin_active;
};

static struct sensors_classdev hall_cdev = {
	.vendor = "oem",
	.version = 1,
	.handle = SENSOR_HANDLE_HALL,
	.flags = (SENSOR_FLAG_ON_CHANGE_MODE), // | SENSOR_FLAG_WAKE_UP
	.max_range = "1",
	.resolution = "1",
	.sensor_power = "0.05",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 500,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static char hall_name_base[32] = {0};

static void hall_process(struct hall_descriptor *hall_info)
{
	int curr_state = gpio_get_value(hall_info->gpio_num) ? 0 : 1;
	if (curr_state != hall_info->hall_state) {
		hall_info->hall_state = curr_state;
		sensors_marker("hall_process()",
			"get", "hall_state", hall_info->hall_state ? "1" : "0");
		if (0 == hall_info->hall_state) {
			if (!(hall_info->wlock.active)) {
            	__pm_wakeup_event(&(hall_info->wlock), jiffies_to_msecs(HZ));
    		} 
		}
	} else {
		sensors_marker("hall_process()",
			"get equal", "hall_state", hall_info->hall_state ? "1" : "0");
	}
	sensors_report_1(hall_info->input_dev, hall_info->hall_state);
}

static irqreturn_t hall_interrupt_handler(int irq, void *data)
{
	struct hall_descriptor *hall_info = data;

	sensors_marker("hall_interrupt_handler()",
		"irq", "triggered with hall",
		hall_info->hall_enable ? "enabled" : "disabled");

	disable_irq_nosync(hall_info->irq);
	if (hall_info->hall_enable)
		hall_process(hall_info);
	enable_irq(hall_info->irq);
	return IRQ_HANDLED;
}

static int hall_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct hall_descriptor *hall_info =
		container_of(sensors_cdev, struct hall_descriptor, h_cdev);

	sensors_marker("hall_set_enable()",
		"receive", "hall_en", enable ? "1" : "0");

	if (enable != hall_info->hall_enable) {
		enable ?
			enable_irq(hall_info->irq) :
			disable_irq(hall_info->irq);
		hall_info->hall_enable = enable;
	}

	return 0;
}

static ssize_t hall_int_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct hall_descriptor *hall_info = container_of(sensors_cdev,
			struct hall_descriptor, h_cdev);
	char state = gpio_get_value(hall_info->gpio_num) ? 0 : 1;
	return snprintf(buf, 20, "int_state = %d\n", state);
}
static DEVICE_ATTR(hall_int, 0400, hall_int_show, NULL);

static struct attribute *attributes_hall[] = {
	&dev_attr_hall_int.attr,
	NULL
};

static struct attribute_group attribute_group_hall = {
	.attrs = attributes_hall
};

static int hall_pinctrl_init(struct hall_descriptor *hall_info)
{
	struct platform_device *pdev = hall_info->p_dev;
	int err = 0;
	hall_info->pinctrl_hall = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(hall_info->pinctrl_hall)) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): Failed to get pinctrl.\n", __func__);
		return PTR_ERR(hall_info->pinctrl_hall);
	}
	hall_info->pin_active =
		pinctrl_lookup_state(hall_info->pinctrl_hall, "hall_int_active");
	if (IS_ERR_OR_NULL(hall_info->pin_active)) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): Failed to look up active state.\n", __func__);
		return PTR_ERR(hall_info->pin_active);
	} else {
		err = pinctrl_select_state(hall_info->pinctrl_hall, hall_info->pin_active);
		if (err) {
			dev_err(&hall_info->p_dev->dev,
				"%s(): Can't select pinctrl active state.\n", __func__);
			return err;
		}
	}
	return 0;
}

static int hall_setup_interrupt(struct hall_descriptor *hall_info)
{
	int ret = hall_pinctrl_init(hall_info);
	if (ret) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): Can't initialize pinctrl.\n", __func__);
		goto quit_0;
	}

	ret = gpio_request(hall_info->gpio_num, hall_name_base);
	if (ret < 0) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): gpio request failed, %d!\n", __func__, ret);
		goto quit_0;
	}	

	ret = gpio_direction_input(hall_info->gpio_num);
	if (ret < 0) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): set gpio direction failed, %d!\n", __func__, ret);
		goto quit_1;
	}
	
	hall_info->hall_state = gpio_get_value(hall_info->gpio_num) ? 0 : 1;
	dev_err(&hall_info->p_dev->dev,
		"%s(): init hall state, %d!\n", __func__, hall_info->hall_state);
	
	hall_info->irq = gpio_to_irq(hall_info->gpio_num);
	ret = request_threaded_irq(hall_info->irq,
							NULL,
							hall_interrupt_handler,
							IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
							hall_name_base,
							hall_info);
	if (ret < 0) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): request irq failed, %d!\n", __func__, ret);
		goto quit_1;
	}

	disable_irq(hall_info->irq);
	
	enable_irq_wake(hall_info->irq);
	//wake_lock_init(&(hall_info->wlock), WAKE_LOCK_SUSPEND, hall_name_base);
	wakeup_source_init(&(hall_info->wlock), hall_name_base);
	return 0;
quit_1:
	gpio_free(hall_info->gpio_num);
quit_0:
	return ret;
}
/*
static void tmd2725_ps_clean_up(struct hall_info *lpi)
{
	if (lpi->with_irq) {
		wake_lock_destroy(&(lpi->wlock));
		disable_irq_wake(lpi->irq);
		disable_irq(lpi->irq);
		free_irq(lpi->irq, lpi);
		gpio_free(lpi->intr_pin);
	}
	sysfs_remove_group(&((lpi->als_cdev.dev)->kobj), &hall_attribute_group);
	sensors_classdev_unregister(&lpi->als_cdev);
	input_unregister_device(lpi->input_dev);
	input_free_device(lpi->input_dev);
}
*/
static int hall_power_init(struct hall_descriptor *hall_info, bool on)
{
	int rc;

	if (on) {
		hall_info->pldo = regulator_get(&hall_info->p_dev->dev, "vdd");
		if (IS_ERR(hall_info->pldo)) {
			rc = PTR_ERR(hall_info->pldo);
			dev_err(&hall_info->p_dev->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}
		if (regulator_count_voltages(hall_info->pldo) > 0) {
			rc = regulator_set_voltage(hall_info->pldo,
					HALL_VDD_MIN_UV, HALL_VDD_MAX_UV);
			if (rc) {
				dev_err(&hall_info->p_dev->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}
		
	} else {
		if (regulator_count_voltages(hall_info->pldo) > 0)
			regulator_set_voltage(hall_info->pldo, 0, HALL_VDD_MAX_UV);
		regulator_put(hall_info->pldo);
	}
	return 0;

err_vdd_set_vtg:
	regulator_put(hall_info->pldo);
err_vdd_get:
	return rc;
}

static int hall_power_switch(struct hall_descriptor *hall_info, bool on)
{
	int rc;

	if (on && !hall_info->power_on) {
		rc = regulator_enable(hall_info->pldo);
		if (rc) {
			dev_err(&hall_info->p_dev->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}

		hall_info->power_on = true;
		msleep(80);
		dev_err(&hall_info->p_dev->dev,
			"%s(): regulator switch ON.\n", __func__);
	} else if (!on && hall_info->power_on) {
		rc = regulator_disable(hall_info->pldo);
		if (rc) {
			dev_err(&hall_info->p_dev->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		hall_info->power_on = false;
		dev_err(&hall_info->p_dev->dev,
			"%s(): regulator switch OFF.\n", __func__);
	}
	return 0;
err_vdd_ena:
	return rc;
}

static int hall_constructor(struct hall_descriptor *hall_info)
{
	int ret = 0;

	hall_info->hall_enable = 0;

	hall_info->input_dev =
		sensors_construct_input_dev(&hall_info->p_dev->dev,
			hall_name_base, 1, hall_info);
	if (hall_info->input_dev == NULL) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): failed to construct hall_input_dev!\n", __func__);
		goto quit_1;
	}

	hall_info->h_cdev = hall_cdev;
	hall_info->h_cdev.sensors_enable = hall_set_enable;
	hall_info->h_cdev.sensors_poll_delay = NULL;

	ret = sensors_classdev_register(&hall_info->input_dev->dev, &hall_info->h_cdev);
	if (ret) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): Unable to register to sensors class.als\n", __func__);
		goto quit_2;
	}
	ret = sysfs_create_group(&((hall_info->h_cdev.dev)->kobj),
		&attribute_group_hall);
	if (ret) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): could not create sysfs group\n", __func__);
		goto quit_3;
	}

	ret = hall_setup_interrupt(hall_info);
	if (0 > ret) {
		dev_err(&hall_info->p_dev->dev,
			"%s(): setup interrupt failed, %d!\n", __func__, ret);
		goto quit_4;
	}
	
	return 0;
quit_4:
	sysfs_remove_group(&((hall_info->h_cdev.dev)->kobj),
		&attribute_group_hall);
quit_3:
	sensors_classdev_unregister(&hall_info->h_cdev);
quit_2:
	input_unregister_device(hall_info->input_dev);
quit_1:
	return ret;
}

#ifdef CONFIG_OF
static int hall_parse_dt(struct device *dev,
		struct hall_descriptor *hall_info)
{
	struct device_node *np = dev->of_node;
	int rc = 0;
	int temp_val = 0;

	const char *position = NULL;
	rc = of_property_read_u32(np, "hall,extend_num", &temp_val);
	if (rc) {
		dev_err(dev,
			"Unable to read hall_extend_num! It is only one HALL sensor!\n");
		rc = 0;
		temp_val = 0;
	}

	if (temp_val > 0) {
		// Find a extend als sensor, try to parse the description.
		rc = of_property_read_string_index(np, "hall,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "position can not be parsed!\n");
			goto quit_parse;
		}
		sensors_construct_extend_name(hall_name_base,
			HALL_NAME, position, temp_val);
		hall_cdev.type =
			sensors_construct_extend_type_num(PYSUEDO_SENSOR_TYPE_HALL,
			position, temp_val);
	} else {
		strncpy(hall_name_base, HALL_NAME, strlen(HALL_NAME));
		hall_cdev.type =
			SENSOR_TYPE_DEVICE_PRIVATE_BASE | PYSUEDO_SENSOR_TYPE_HALL;
	}
	hall_cdev.name = hall_name_base;
	hall_info->gpio_num = of_get_named_gpio_flags(np, "hall-gpio", 0, NULL);
	if (hall_info->gpio_num < 0) {
		dev_err(dev,
			"Unable to read interrupt pin number, Use polling instead!\n");
		rc = -1;
		goto quit_parse;
	}

quit_parse:
	return rc;
}
#endif

static int hall_probe(struct platform_device *pdev)
{
	struct hall_descriptor *hall_info = NULL;
	int rc = 0;

	hall_info = kzalloc(sizeof(struct hall_descriptor), GFP_KERNEL);
	if (NULL == hall_info)
		return -ENOMEM;

	hall_info->p_dev = pdev;
	dev_set_drvdata(&pdev->dev, hall_info);

	rc = hall_parse_dt(&pdev->dev, hall_info);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s(): failed to parse dts, %d!\n", __func__, rc);
		goto fail;
	}

	rc = hall_power_init(hall_info, true);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s(): failed to init power, %d!\n", __func__, rc);
		goto fail;
	}

	rc = hall_power_switch(hall_info, true);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s(): failed to turn on power, %d!\n", __func__, rc);
		goto fail_1;  
	}

	rc = hall_constructor(hall_info);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s(): failed to construct chip info, %d!\n", __func__, rc);
		goto fail_2;  
	}

	printk("hall driver probed\n");
	return 0;

fail_2:  
	hall_power_switch(hall_info, false);
fail_1:
	hall_power_init(hall_info, false);
fail:
	kfree(hall_info);
	return rc;
}

static struct of_device_id hall_match_table[] = {
	{
		.compatible = "hisense,hall-device",
	}
};

static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.driver = {
		.name = "hall_driver",
		.owner = THIS_MODULE,
		.of_match_table = hall_match_table,
	},
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver to report hall state.");
MODULE_VERSION("1.00");
MODULE_AUTHOR("lizengbo@hisensecom.com");

module_init(hall_init);
module_exit(hall_exit);

