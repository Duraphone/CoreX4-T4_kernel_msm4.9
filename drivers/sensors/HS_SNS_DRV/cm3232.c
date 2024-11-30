#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/jiffies.h>
#include <linux/of_gpio.h>
#include <linux/productinfo.h>
#include <linux/sensors.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/productinfo.h>

#include "cm3232.h"

static char cm3232_als_name_base[32] = {0};

static struct sensors_classdev cm3232_als_cdev = {
	.vendor = "capella",
	.version = 1,
	.handle = SENSOR_HANDLE_LIGHT,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "65535",
	.resolution = "0.018",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = SENSOR_FLAG_ON_CHANGE_MODE,
	.enabled = 0,
	.delay_msec = 500,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

struct cm3232_private_data {
	struct i2c_client		*i2c_client;
	struct input_dev		*als_input_dev;

	bool	als_enable;
	int		poll_delay;

	bool	chip_function_on;
	bool	power_on;

	u8		ls_cmd;
	int		it_time_index;
	int		sens_index;
	int		als_fittness;
	u16		ldata;
	unsigned int als_extend_num;

	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work ldwork;

	struct sensors_classdev als_cdev;
};
static uint8_t confuse_i = 0;
static int als_delay_adjust_number= 0;

static int cm3232_write_cmd_byte(struct i2c_client *client,
		u8 *txData, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
	
	if (i2c_transfer(client->adapter, msg, 1) < 0) {
		printk("%s() err!\n", __func__);
		return -EIO;
	}
	return 0;
}

static int cm3232_chip_reset(struct cm3232_private_data *info)
{
	u8 reg[2] = {0, 0};
	reg[1] = info->ls_cmd  | CM3232_CHIP_RESET;
	cm3232_write_cmd_byte(info->i2c_client, reg, 2);
	reg[1] = info->ls_cmd;
	return cm3232_write_cmd_byte(info->i2c_client, reg, 2);
}

static int cm3232_function_switch(struct cm3232_private_data *info, bool on)
{
	u8 reg[2] = {0, 0};
	int ret = 0;
	if (on){
		info->ls_cmd &= ~CM3232_FUNC_OFF;
	}else{
		info->ls_cmd |= CM3232_FUNC_OFF;
	}
	reg[1] = info->ls_cmd;
	ret = cm3232_write_cmd_byte(info->i2c_client, reg, 2);
	if (ret < 0) {
		dev_err(&info->i2c_client->dev,
			"%s(): write ALS_CONF error!\n", __func__);
		return -EIO;
    } else
		info->chip_function_on = on ? true : false;

	printk("%s() get function on = %d, return %d\n", __func__,
		on, info->chip_function_on);
	als_delay_adjust_number = 0;
	return ret;
}

static bool cm3232_check_chip_function(struct cm3232_private_data *info)
{
	if (!info->power_on)
		return false;
	else if (!info->chip_function_on)
		return cm3232_function_switch(info, true) ? false : true;
	else
		return true;
}

static void alsensor_delay_work_handler(struct work_struct *work)
{
	struct cm3232_private_data *info = 
		container_of((struct delayed_work *)work, struct cm3232_private_data, ldwork);

	u32 als_value = 0;
	s32 temp_val = 0;
	uint16_t res;
	if (0 == info->als_enable) {
		dev_err(&info->i2c_client->dev,
				"%s(): als not enabled, do not re-schedule als_dwork!\n",
				__func__);
		return;
	}

	temp_val = i2c_smbus_read_word_data(info->i2c_client, 0x50);
    if (temp_val < 0) {
        dev_err(&info->i2c_client->dev,
            "%s: CM3232 read ALS_DATA error!\n", __func__);
        return;
    }
	als_value = temp_val;
	res = (uint16_t)(light_2_lux_table[info->sens_index][info->it_time_index]);

	sensors_report_2(info->als_input_dev,
		als_value, als_value, res, &confuse_i);

	info->ldata = (u16)temp_val;
	if (als_delay_adjust_number < 50){
		queue_delayed_work(system_freezable_wq,
			&info->ldwork, msecs_to_jiffies(CM3232_LS_IT_100_POLL_DELAY));
		als_delay_adjust_number++;
	}else{
		queue_delayed_work(system_freezable_wq,
			&info->ldwork, msecs_to_jiffies(CM3232_LS_IT_400_POLL_DELAY));
	}
}

static int cm3232_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct cm3232_private_data *info =
		container_of(sensors_cdev, struct cm3232_private_data, als_cdev);
	struct input_dev *input_dev = info->als_input_dev;

	int error = 0;

	if ((enable < 0) || (enable > 1)) {
    	dev_err(&info->i2c_client->dev,
			"%s(): It is an illegal para %d!\n",
			__func__, enable);
    	return -EINVAL;
	}
	dev_err(&info->i2c_client->dev, "receive als_en is %d\n", enable);

	mutex_lock(&input_dev->mutex);
	if (enable != info->als_enable) {
    	if (enable) {
            if (!cm3232_check_chip_function(info)) {
                dev_err(&info->i2c_client->dev,
    				"%s(): Chip not functioned, report nothing!\n",
    				__func__);
            } else {
			queue_delayed_work(system_freezable_wq,
				&info->ldwork, msecs_to_jiffies(CM3232_LS_IT_400_POLL_DELAY));
			info->als_enable = true;
            }
    	} else {
            cancel_delayed_work_sync(&info->ldwork);
    		error = cm3232_function_switch(info, false);
            if(!error)
    		    info->als_enable = false;
    	}
    }
	mutex_unlock(&input_dev->mutex);

	return error;
}

static ssize_t ldata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct cm3232_private_data *info = container_of(sensors_cdev,
		struct cm3232_private_data, als_cdev);
	if (info->chip_function_on){
		return snprintf(buf, 10, "%d\n", info->ldata);
	} else {
        dev_err(dev,
            "%s: Light sensor has not been enabled, no data now!\n", __func__);
        return -EPERM;
    }
}
static DEVICE_ATTR(ldata, 0664, ldata_show, NULL);

static ssize_t als_fittness_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct cm3232_private_data *info = container_of(sensors_cdev,
		struct cm3232_private_data, als_cdev);
	return snprintf(buf, 20,
		"%d\n", info->als_fittness);
}
static ssize_t als_fittness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct sensors_classdev *sensors_cdev = dev_get_drvdata(dev);
	struct cm3232_private_data *info = container_of(sensors_cdev,
		struct cm3232_private_data, als_cdev);
	int value = 0;
	sscanf(buf, "%d", &value);
	info->als_fittness = value;
	return count;
}
static DEVICE_ATTR(als_fittness, 0660, als_fittness_show, als_fittness_store);

static struct attribute *cm3232_attributes[] = {
	&dev_attr_ldata.attr,
	&dev_attr_als_fittness.attr,
	NULL
};

static struct attribute_group cm3232_attribute_group = {
	.attrs = cm3232_attributes
};

static int cm3232_als_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{ 
	return 0;
}

static int cm3232_init_input(struct cm3232_private_data *info)
{
	int ret;

	info->als_input_dev = input_allocate_device();
	if (!info->als_input_dev) {
		pr_err("[CM3232 error]%s(): could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	
	input_set_drvdata(info->als_input_dev, info);
	
	info->als_input_dev->name = cm3232_als_name_base;
	set_bit(EV_ABS, info->als_input_dev->evbit);

	input_set_abs_params(info->als_input_dev, ABS_MISC, 0, 65535, 0, 0);

	ret = input_register_device(info->als_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM3232 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_als_input_device;
	}

	return ret;

err_free_als_input_device:
	input_free_device(info->als_input_dev);
	return ret;
}

static int cm3232_parse_dt(struct device *dev,
				struct cm3232_private_data *priv_data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	
	rc = of_property_read_u32(np, "capella,it_time", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read it_time\n");
		return rc;
	} else {
		priv_data->it_time_index = temp_val;
	}
	
	rc = of_property_read_u32(np, "capella,sensitivity", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read sensitivity\n");
		return rc;
	} else {
		priv_data->sens_index = temp_val;
	}

    rc = of_property_read_u32(np, "capella,als_fittness", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read als_fittness\n");
		return rc;
	} else {
		priv_data->als_fittness = (int)temp_val;
	}

	rc = of_property_read_u32(np, "als,extend_num", &temp_val);
	if (rc) {
		dev_err(dev,
			"Unable to read als_extend_num! It is only one ALS sensor!\n");
		temp_val = 0;
		rc=0;
	}

	if (temp_val > 0) {
		// Find a extend als sensor, try to parse the description.
		const char *position = NULL;
		rc = of_property_read_string_index(np,
			"als,position", 0, &position);
		if (0 > rc) {
			dev_err(dev, "position can not be parsed!\n");
			goto quit_parse;
		}
		sensors_construct_extend_name(cm3232_als_name_base,
			LIGHT_NAME, position, temp_val);
		cm3232_als_cdev.type =
			sensors_construct_extend_type_num(SENSOR_TYPE_LIGHT,
			position, temp_val);
	} else {
		strncpy(cm3232_als_name_base, LIGHT_NAME, strlen(LIGHT_NAME));
		cm3232_als_cdev.type = SENSOR_TYPE_LIGHT;
	}
	cm3232_als_cdev.name = cm3232_als_name_base;
	priv_data->ls_cmd = (priv_data->it_time_index << CM3232_IT_TIME_SHIFTER)
						| (priv_data->sens_index << CM3232_SENS_SHIFTER);

quit_parse:
	return rc;
}


static int cm3232_power_init(struct cm3232_private_data *info, bool on)
{
	int rc;

	if (on) {
		info->vdd = regulator_get(&info->i2c_client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			rc = PTR_ERR(info->vdd);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			goto err_vdd_get;
		}
		if (regulator_count_voltages(info->vdd) > 0) {
			rc = regulator_set_voltage(info->vdd,
					CM3232_VDD_MIN_UV, CM3232_VDD_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
					"Regulator set failed vdd rc=%d\n", rc);
				goto err_vdd_set_vtg;
			}
		}
		info->vio = regulator_get(&info->i2c_client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->i2c_client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}
		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				CM3232_VI2C_MIN_UV, CM3232_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->i2c_client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
	} else {
		if (regulator_count_voltages(info->vdd) > 0)
			regulator_set_voltage(info->vdd, 0, CM3232_VDD_MAX_UV);
		regulator_put(info->vdd);
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0, CM3232_VI2C_MAX_UV);
		regulator_put(info->vio);
	}
	return 0;
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
	if (regulator_count_voltages(info->vdd) > 0)
		regulator_set_voltage(info->vdd, 0, CM3232_VDD_MAX_UV);
err_vdd_set_vtg:
	regulator_put(info->vdd);
err_vdd_get:
	return rc;
}

static int cm3232_power_switch(struct cm3232_private_data *info, bool on)
{
	int rc;

	if (on && !info->power_on) {
		rc = regulator_enable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_ena;
		}
		rc = regulator_enable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_ena;
		}
		info->power_on = true;
		msleep(80);
		dev_err(&info->i2c_client->dev,
			"%s(): regulator switch ON.\n", __func__);
	} else if (!on && info->power_on) {
		rc = regulator_disable(info->vdd);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(info->vio);
		if (rc) {
			dev_err(&info->i2c_client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			return rc;
		}
		info->power_on = false;
		dev_err(&info->i2c_client->dev,
			"%s(): regulator switch OFF.\n", __func__);
	}
	return 0;
err_vio_ena:
	regulator_disable(info->vdd);
err_vdd_ena:
	return rc;
}


static int cm3232_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct cm3232_private_data *info = NULL;
	int ret = 0;
	als_delay_adjust_number = 0;
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	info = kzalloc(sizeof(struct cm3232_private_data), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = cm3232_parse_dt(&client->dev, info);
	if (ret) {
		dev_err(&client->dev, "Failed to parsing device tree\n");
		goto err_free_mem;
	}

	info->i2c_client = client;
	i2c_set_clientdata(client, info);

    info->power_on = false;
	ret = cm3232_power_init(info, true);
	if (ret) {
		dev_err(&client->dev, "Failed to request regulators\n");
		goto err_free_mem;
	}

	ret = cm3232_power_switch(info, true);
	if (ret) {
		dev_err(&client->dev, "Failed to switch on regulators\n");
		goto err_free_power;
	}

	ret = cm3232_chip_reset(info);
	if (ret < 0) {
		dev_err(&client->dev, "cm3232 reset error!\n");
		goto err_power_off;
	}

	ret = cm3232_init_input(info);
	if (ret < 0) {
		dev_err(&client->dev, "cm3232_init_input error!\n");
		goto err_power_off;
	}

	/* Register to sensors class */
	info->als_cdev = cm3232_als_cdev;
	info->als_cdev.sensors_enable = cm3232_als_set_enable;
	info->als_cdev.sensors_poll_delay = cm3232_als_poll_delay_set;

	ret = sensors_classdev_register(&info->als_input_dev->dev, &info->als_cdev);
    if(ret < 0) {
        dev_err(&client->dev, "cm3232 classdev register error!\n");
        goto err_destroy_input;
    } 
	ret = sysfs_create_group(&((info->als_cdev.dev)->kobj), &cm3232_attribute_group);
	if (ret < 0) {
        dev_err(&client->dev, "cm3232 create group error!\n");
		goto err_unregister_cdev;
    }
	info->chip_function_on = false;
	INIT_DELAYED_WORK(&info->ldwork, alsensor_delay_work_handler);
	productinfo_register(PRODUCTINFO_SENSOR_ALSPS_ID,
		"cm3232", "capella");
	printk("%s() done!\n", __func__);
	return ret;

err_unregister_cdev:
	sensors_classdev_unregister(&info->als_cdev);
err_destroy_input:
	input_unregister_device(info->als_input_dev);
err_power_off:
	cm3232_power_switch(info, false);
err_free_power:
	cm3232_power_init(info, false);
err_free_mem:
	kfree(info);
	dev_err(&client->dev, "%s:error exit! ret = %d\n", __func__, ret);

	return ret;
}

static int cm3232_remove(struct i2c_client *client)
{
	struct cm3232_private_data *info = i2c_get_clientdata(client);
	if (info->als_enable) {
		cancel_delayed_work_sync(&info->ldwork);
		cm3232_function_switch(info, false);
	}
    sysfs_remove_group(&((info->als_cdev.dev)->kobj), &cm3232_attribute_group);
    sensors_classdev_unregister(&info->als_cdev);
    input_unregister_device(info->als_input_dev);
	cm3232_power_switch(info, false);
	cm3232_power_init(info, false);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cm3232_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm3232_private_data *info = i2c_get_clientdata(client);

	if (info->als_enable) {
        cancel_delayed_work_sync(&info->ldwork);
		cm3232_function_switch(info, false);
	}
	dev_err(&client->dev, "%s().\n", __func__);
	return 0;
}

static int cm3232_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cm3232_private_data *info = i2c_get_clientdata(client);

	dev_err(&client->dev, "%s()\n", __func__);
	if (info->als_enable) {
        if (!cm3232_check_chip_function(info)) {
            dev_err(&client->dev,
				"%s(): Chip not functioned, report nothing!\n",
				__func__);
        } else {
            queue_delayed_work(system_freezable_wq,
                &info->ldwork, msecs_to_jiffies(CM3232_LS_IT_400_POLL_DELAY));
        }
	}
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cm3232_pm, cm3232_suspend, cm3232_resume);

static const struct i2c_device_id cm3232_i2c_id[] = {
	{CM3232_I2C_NAME, 0},
	{}
};

static struct of_device_id cm3232_match_table[] = {
	{ .compatible = "capella,cm3232",},
	{ },
};

static struct i2c_driver cm3232_driver = {
	.id_table = cm3232_i2c_id,
	.probe = cm3232_probe,
	.remove = cm3232_remove,
	.driver = {
		.name = CM3232_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cm3232_pm,
		.of_match_table = cm3232_match_table,
	},
};

static int __init cm3232_init(void)
{
	return i2c_add_driver(&cm3232_driver);
}

static void __exit cm3232_exit(void)
{
	i2c_del_driver(&cm3232_driver);
}

module_init(cm3232_init);
module_exit(cm3232_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3232 Driver");
MODULE_AUTHOR("Frank Hsieh <pengyueh@gmail.com>");
