/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/suspend.h>
#include <linux/poll.h>
#include <linux/nfc/pn547.h>
#include <linux/nfcinfo.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>

#include <linux/productinfo.h>
#define MAX_BUFFER_SIZE	512
#define PN547_VI2C_MIN_UV	1650000
#define PN547_VI2C_MAX_UV	1950000

struct pn547_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	union  nqx_uinfo	nqx_info;
	//struct wake_lock	read_wake;
	spinlock_t		irq_enabled_lock;
	bool			irq_enabled;
	unsigned int		ven_gpio;
	unsigned int		firm_gpio;
	unsigned int		irq_gpio;
	/* CLK_REQ IRQ to signal the state has changed */
	unsigned int		irq_gpio_clk_req;
	unsigned int		clkreq_gpio;
	/* CLK control */
	unsigned int		clk_src_gpio;
	const	char		*clk_src_name;
	struct	clk		*s_clk;
	bool			clk_run;
	unsigned int    irq_wakeup_state;
	bool			use_sys_clk;
	u8 *kbuf;
	struct regulator *vio;
};

static const char *nfc_name = NULL;
#define PN547_DRVNAME		"pn544"

static struct of_device_id msm_match_table[] = {
	{.compatible = "nxp,nfc-nci"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

static int pn547_clock_select(struct pn547_dev *pn547_dev)
{
	int ret = 0;

	if ((!strcmp(pn547_dev->clk_src_name, "BBCLK2")) || (!strcmp(pn547_dev->clk_src_name, "BBCLK3"))) {
		printk("nfc clk_src is BBCLK2/BBCLK3\n");
		pn547_dev->s_clk  =
			clk_get(&pn547_dev->client->dev, "ref_clk");
		if (pn547_dev->s_clk == NULL)
			goto err_invalid_dis_gpio;
	} else {
		pn547_dev->s_clk = NULL;
		goto err_invalid_dis_gpio;
	}
	if (pn547_dev->clk_run == false) {
		printk("pn547 clk_run = true\n");
		ret = clk_prepare_enable(pn547_dev->s_clk);
		if (ret)
			goto err_invalid_clk;
		pn547_dev->clk_run = true;
	}
	ret = 0;
	return ret;

err_invalid_clk:
	ret = -1;
	return ret;
err_invalid_dis_gpio:
	ret = -2;
	return ret;
}

static int pn547_clock_deselect(struct pn547_dev *pn547_dev)
{
	int ret = -1;

	if (pn547_dev->s_clk != NULL) {
		if (pn547_dev->clk_run == true) {
			clk_disable_unprepare(pn547_dev->s_clk);
			pn547_dev->clk_run = false;
		}
		return 0;
	}
	return ret;
}

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;

	//printk("pn547_dev_irq_handler\n");
	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn547_disable_irq(pn547_dev);

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	int r;

	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware\n", __func__);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			printk("%s power on\n", __func__);
			if (pn547_dev->irq_wakeup_state == 0) {
				printk("%s power on, wakeup_state=%d, need setup wakeup\n", __func__, pn547_dev->irq_wakeup_state);
				irq_set_irq_wake(pn547_dev->client->irq, 1);
				pn547_dev->irq_wakeup_state = 1;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			if (pn547_dev->use_sys_clk){
				r = pn547_clock_select(pn547_dev);
				if (r < 0)
					printk( "unable to enable clock\n");
			}
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			printk("%s power off\n", __func__);
			if (pn547_dev->irq_wakeup_state == 1) {
				printk("%s power off, wakeup_state=%d, need setup not wakeup\n", __func__,pn547_dev->irq_wakeup_state);
				irq_set_irq_wake(pn547_dev->client->irq, 0);
				pn547_dev->irq_wakeup_state = 0;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			if (pn547_dev->use_sys_clk){
				r = pn547_clock_deselect(pn547_dev);
				if (r < 0)
					printk( "unable to disable clock\n");
			}
			msleep(10);
		} else {
			printk("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static int nfc_parse_dt(struct device *dev, struct pn547_i2c_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "reg", &pdata->reg);
	if (ret)
		return -EINVAL;

	ret = of_property_read_string(np, "qcom,clk-src", &pdata->clk_src_name);
	if (ret) {
		printk("%s: no system clock set!\n", __func__);
		pdata->use_sys_clk = false;
	}else
		pdata->use_sys_clk = true;

	if (pdata->use_sys_clk) {
		pdata->clkreq_gpio = of_get_named_gpio(np, "qcom,clk-gpio", 0);
		if (!gpio_is_valid(pdata->clkreq_gpio))
			return -EINVAL;
	}

	pdata->ven_gpio = of_get_named_gpio(np, "qcom,en-gpio", 0);
	if ((!gpio_is_valid(pdata->ven_gpio)))
		return -EINVAL;

	pdata->irq_gpio = of_get_named_gpio(np, "qcom,irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "qcom,firm-gpio", 0);
	if ((!gpio_is_valid(pdata->firm_gpio)))
		return -EINVAL;

	if (pdata->use_sys_clk) {
		pr_info("GPIO ven,irq,firm,clk_req = %d, %d, %d, %d",
			pdata->ven_gpio, pdata->irq_gpio, pdata->firm_gpio, pdata->clkreq_gpio);
	}else{
		pr_info("GPIO ven,irq,firm = %d, %d, %d",
			pdata->ven_gpio, pdata->irq_gpio, pdata->firm_gpio);	
	}

	return 0;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	unsigned char *tmp = NULL;
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	//printk("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
		ret = wait_event_interruptible(pn547_dev->read_wq,
				gpio_get_value(pn547_dev->irq_gpio));

		pn547_disable_irq(pn547_dev);

		if (ret)
			goto fail;
	}

	tmp = pn547_dev->kbuf;
	if (!tmp) {
		printk("%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto fail;
	}
	memset(tmp, 0x00, count);

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);
	mutex_unlock(&pn547_dev->read_mutex);

	if (ret < 0) {
		printk("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		printk("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		printk("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547dev;
	char *tmp = NULL;
	int ret;

	pn547dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	//printk("pn547 i2c addr client->addr = 0x%x\n", pn547dev->client->addr);
	tmp = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!tmp) {
		printk("%s: failed to allocate memory for pn547_dev->write\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	if (copy_from_user(tmp, buf, count)) {
		printk("%s : failed to copy from user space\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	//printk("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn547dev->client, tmp, count);
	if (ret != count) {
		printk("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}
exit:
	kfree(tmp);
	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);

	filp->private_data = pn547_dev;

	printk("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn547_dev_release(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = filp->private_data;

	pn547_dev = NULL;

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
    .release	= pn547_dev_release,
	.unlocked_ioctl  = pn547_dev_ioctl,
};

/* Check for availability of NQ_ NFC controller hardware */
static int nfcc_hw_check(struct i2c_client *client, struct pn547_dev *pn547_dev)
{
	int ret = 0;

	unsigned char raw_nci_reset_cmd[] =  {0x20, 0x00, 0x01, 0x00};
	unsigned char raw_nci_init_cmd[] =   {0x20, 0x01, 0x00};
	unsigned char raw_nci_getversion_cmd[] = {0x00, 0x04, 0xF1,
					0x00, 0x00, 0x00, 0x6E, 0xEF};

	unsigned char nci_init_rsp[28];
	unsigned char nci_reset_rsp[6];
	unsigned char nci_getversion_rsp[20];
	unsigned char init_rsp_len = 0;
	unsigned int enable_gpio = pn547_dev->ven_gpio;
	/* making sure that the NFCC starts in a clean state. */


	gpio_set_value(enable_gpio, 1);/* HPD : Enable*/
	/* hardware dependent delay */
	usleep_range(10000, 10100);
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	/* hardware dependent delay */
	usleep_range(10000, 10100);
	gpio_set_value(enable_gpio, 1);/* HPD : Enable*/
	/* hardware dependent delay */
	usleep_range(10000, 10100);

	/* send NCI CORE RESET CMD with Keep Config parameters */
	ret = i2c_master_send(client, raw_nci_reset_cmd,
						sizeof(raw_nci_reset_cmd));

	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send core reset Error\n", __func__);

		gpio_set_value(pn547_dev->ven_gpio, 1);
		usleep_range(10000, 10100);
		if (gpio_is_valid(pn547_dev->firm_gpio))
			gpio_set_value(pn547_dev->firm_gpio, 1);
		usleep_range(10000, 10100);
		gpio_set_value(pn547_dev->ven_gpio, 0);
		usleep_range(10000, 10100);
		gpio_set_value(pn547_dev->ven_gpio, 1);
		usleep_range(10000, 10100);

		ret = i2c_master_send(client, raw_nci_getversion_cmd,
						sizeof(raw_nci_getversion_cmd));

		if (ret < 0) {
			dev_err(&client->dev,
			"%s: - i2c_master_send get version Error\n", __func__);
			goto err_nfcc_hw_check;
		}
		/* hardware dependent delay */
		msleep(30);

		ret = i2c_master_recv(client, nci_getversion_rsp,
						sizeof(nci_getversion_rsp));
		if (ret < 0) {
			dev_err(&client->dev,
			"%s: - i2c_master_recv Error\n", __func__);
			goto err_nfcc_hw_check;
		} else {
			pn547_dev->nqx_info.info.chip_type =
				nci_getversion_rsp[3];
			pn547_dev->nqx_info.info.rom_version =
				nci_getversion_rsp[4];
			pn547_dev->nqx_info.info.fw_minor =
				nci_getversion_rsp[10];
			pn547_dev->nqx_info.info.fw_major =
				nci_getversion_rsp[11];
		}
		goto err_nfcc_reset_failed;
	}
	/* hardware dependent delay */
	msleep(30);

	/* Read Response of RESET command */
	ret = i2c_master_recv(client, nci_reset_rsp,
		sizeof(nci_reset_rsp));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	dev_info(&client->dev,
	"%s: - nq - reset cmd answer : NfcNciRx %02x %02x %02x %02x %02x %02x\n",
	__func__, nci_reset_rsp[0], nci_reset_rsp[1],
	nci_reset_rsp[2], nci_reset_rsp[3],
	nci_reset_rsp[4], nci_reset_rsp[5]);
	ret = i2c_master_send(client, raw_nci_init_cmd,
		sizeof(raw_nci_init_cmd));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send failed for core init\n", __func__);
		goto err_nfcc_hw_check;
	}
	/* hardware dependent delay */
	msleep(30);
	/* Read Response of INIT command */
	ret = i2c_master_recv(client, nci_init_rsp,
		sizeof(nci_init_rsp));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	init_rsp_len = 2 + nci_init_rsp[2]; /*payload + len*/
	if (init_rsp_len > PAYLOAD_HEADER_LENGTH) {
		pn547_dev->nqx_info.info.chip_type =
				nci_init_rsp[init_rsp_len - 3];
		pn547_dev->nqx_info.info.rom_version =
				nci_init_rsp[init_rsp_len - 2];
		pn547_dev->nqx_info.info.fw_major =
				nci_init_rsp[init_rsp_len - 1];
		pn547_dev->nqx_info.info.fw_minor =
				nci_init_rsp[init_rsp_len];
	}
err_nfcc_reset_failed:
	dev_info(&pn547_dev->client->dev, "NQ NFCC chip_type = 0x%x\n",
		pn547_dev->nqx_info.info.chip_type);
	dev_info(&pn547_dev->client->dev, "NQ fw version = %x.%x.%x\n",
		pn547_dev->nqx_info.info.rom_version,
		pn547_dev->nqx_info.info.fw_major,
		pn547_dev->nqx_info.info.fw_minor);

	switch (pn547_dev->nqx_info.info.chip_type) {
	case NFCC_NQ_210:
		nfc_name="NQ210";
		dev_info(&client->dev,
		"%s: ## NFCC == NQ210 ##\n", __func__);
		break;
	case NFCC_NQ_220:
		nfc_name="NQ220";
		dev_info(&client->dev,
		"%s: ## NFCC == NQ220 ##\n", __func__);
		break;
	case NFCC_NQ_310:
		nfc_name="NQ310";
		dev_info(&client->dev,
		"%s: ## NFCC == NQ310 ##\n", __func__);
		break;
	case NFCC_NQ_330:
		nfc_name="NQ330";
		dev_info(&client->dev,
		"%s: ## NFCC == NQ330 ##\n", __func__);
		break;
	case NFCC_PN66T:
		nfc_name="PN66T";
		dev_info(&client->dev,
		"%s: ## NFCC == PN66T ##\n", __func__);
		break;
	case NFCC_PN553:
		nfc_name="PN553";
		dev_info(&client->dev,
		"%s: ## NFCC == PN553 ##\n", __func__);
		break;
	default:
		dev_err(&client->dev,
		"%s: - NFCC ChipID can't be identified\n", __func__);
		break;
	}

	/*Disable NFC by default to save power on boot*/
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	if(gpio_get_value(pn547_dev->firm_gpio)){
		gpio_set_value(pn547_dev->firm_gpio, 0);
	}
	ret = 0;
	goto done;

err_nfcc_hw_check:
	ret = -ENXIO;
	dev_err(&client->dev,
		"%s: - NFCC HW not available\n", __func__);
done:
	return ret;
}

static void nfc_register_production(void)
{
	char nfc_info_show[20];
	snprintf(nfc_info_show, sizeof(nfc_info_show),
			"Chip: %s ", nfc_name);
	productinfo_register(PRODUCTINFO_NFC_ID, NULL, nfc_info_show);
}

static int pn547_power_init(struct pn547_dev *info, bool on)
{
	int rc;

	if (on) {
		info->vio = regulator_get(&info->client->dev, "vio");
		if (IS_ERR(info->vio)) {
			rc = PTR_ERR(info->vio);
			dev_err(&info->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto err_vio_get;
		}
		if (regulator_count_voltages(info->vio) > 0) {
			rc = regulator_set_voltage(info->vio,
				PN547_VI2C_MIN_UV,PN547_VI2C_MAX_UV);
			if (rc) {
				dev_err(&info->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set_vtg;
			}
		}
	} else {
		if (regulator_count_voltages(info->vio) > 0)
			regulator_set_voltage(info->vio, 0, PN547_VI2C_MAX_UV);
		regulator_put(info->vio);
	}
	return 0;
err_vio_set_vtg:
	regulator_put(info->vio);
err_vio_get:
	return rc;
}

static int pn547_power_switch(struct pn547_dev *info, bool on)
{
	int rc;

	if (on) {
		rc=regulator_is_enabled(info->vio);
		dev_err(&info->client->dev, "%s(): VDDIO=%d.\n", __func__,rc);
		if(!rc){
			rc = regulator_enable(info->vio);
			if (rc) {
				dev_err(&info->client->dev,
					"Regulator vio enable failed rc=%d\n", rc);
				goto err_vio_ena;
			}
			msleep(80);
			dev_err(&info->client->dev,
				"%s(): regulator switch ON.\n", __func__);
		}
	} else {
		rc=regulator_is_enabled(info->vio);
		dev_err(&info->client->dev, "%s(): VDDIO=%d.\n", __func__,rc);
		if(rc>0){
			rc = regulator_disable(info->vio);
			if (rc) {
				dev_err(&info->client->dev,
					"Regulator vio disable failed rc=%d\n", rc);
				return rc;
			}
			dev_err(&info->client->dev,
				"%s(): regulator switch OFF.\n", __func__);
		}
	}
	return 0;
err_vio_ena:
	return rc;
}

static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	int irqn = 0;
	struct pn547_i2c_platform_data *platform_data;
	struct pn547_dev *pn547_dev;

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
				sizeof(struct pn547_i2c_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev,
					"nfc-pn547 probe: Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = nfc_parse_dt(&client->dev, platform_data);
		if (ret) {
			dev_err(&client->dev,
				"%s parse node error. please check the dts file \n", __func__);
			return -EINVAL;
			}
	} else {
		platform_data = client->dev.platform_data;
	}

	dev_info(&client->dev,
		"%s, probing pn547 driver flags = %x\n", __func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc probe fail\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,
			"%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
		}
	pn547_dev->client=client;

	pn547_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!pn547_dev->kbuf) {
		dev_err(&client->dev,
			"failed to allocate memory for pn547_dev->kbuf\n");
		ret = -ENOMEM;
		goto err_mem;
	}

	/*2.power on*/  //lws add by lws
	ret = pn547_power_init(pn547_dev, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): pn547 power on error!\n",
				__func__);
		goto err_power_init;
	}
	ret = pn547_power_switch(pn547_dev, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): pn547 power switch error!\n",
				__func__);
		goto err_power_switch;
	}

	/**********nfc gpio operation**********/
	if (gpio_is_valid(platform_data->ven_gpio)) {
		ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
		if (ret) {
			pr_err("%s nfc_ven gpio error ret = %d \n", __func__, ret);
			dev_err(&client->dev, "%s: unable to request gpio [%d]\n",
				__func__, platform_data->ven_gpio);
			goto err_power;
			}
		ret = gpio_direction_output(platform_data->ven_gpio, 0); // 1 /0
		if (ret) {
			dev_err(&client->dev, "%s: unable to set direction for gpio [%d]\n",
				__func__, platform_data->ven_gpio);
			goto err_ven;
			}
		} else {
				dev_err(&client->dev, "%s: enable gpio isn't provided\n",
					__func__);
				goto err_power;
		}
	if (gpio_is_valid(platform_data->firm_gpio)) {
		ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
		if (ret) {
			pr_err("%s nfc_firm gpio error ret = %d\n", __func__, ret);
			dev_err(&client->dev, "%s: unable to request gpio [%d]\n",
				__func__, platform_data->firm_gpio);
			goto err_ven;
			}
		ret = gpio_direction_output(platform_data->firm_gpio, 0); // 1 /0
		if (ret) {
			dev_err(&client->dev, "%s: unable to set direction for gpio [%d]\n",
				__func__, platform_data->firm_gpio);
			goto err_firm;
			}
		} else {
				dev_err(&client->dev, "%s: firm download gpio isn't provided\n",
					__func__);
				goto err_ven;
		}
	if (gpio_is_valid(platform_data->irq_gpio)) {
		ret = gpio_request(platform_data->irq_gpio, "nfc_irq_gpio");
		if (ret) {
			pr_err("%s nfc_irq gpio error ret = %d\n", __func__, ret);
			dev_err(&client->dev, "%s: unable to request gpio [%d]\n",
				__func__, platform_data->irq_gpio);
			goto err_firm;
			}
		ret = gpio_direction_input(platform_data->irq_gpio);
		if (ret) {
			dev_err(&client->dev, "%s: unable to set direction for gpio [%d]\n",
				__func__, platform_data->irq_gpio);
			goto err_irq;
			}
		irqn = gpio_to_irq(platform_data->irq_gpio);
		if (irqn < 0) {
			ret = irqn;
			goto err_irq;
		}
		client->irq = irqn;
		} else {
				dev_err(&client->dev, " %s: interupt gpio isn't provided\n",
					__func__);
				goto err_firm;
		}
	if (platform_data->use_sys_clk) {
		if (gpio_is_valid(platform_data->clkreq_gpio)) {
			ret = gpio_request(platform_data->clkreq_gpio, "nfc_clkreq_gpio");
			if (ret) {
				pr_err("%s nfc_clkreq gpio error ret = %d\n", __func__, ret);
				dev_err(&client->dev, "%s: unable to request gpio [%d]\n",
					__func__, platform_data->clkreq_gpio);
				goto err_irq;
				}
			ret = gpio_direction_input(platform_data->clkreq_gpio);
			if (ret) {
				dev_err(&client->dev, "%s: unable to set direction for gpio [%d]\n",
					__func__, platform_data->clkreq_gpio);
				goto err_clkreq;
				}
			} else {
					dev_err(&client->dev, "%s: clock request gpio isn't provided\n",
						__func__);
					goto err_irq;
			}
	}

	printk("%s All the nfc gpio request ok!\n", __func__);

	pn547_dev->clk_src_name = platform_data->clk_src_name;
	pn547_dev->ven_gpio = platform_data->ven_gpio;
	pn547_dev->firm_gpio = platform_data->firm_gpio;
	pn547_dev->irq_gpio = platform_data->irq_gpio;
	if (platform_data->use_sys_clk) {
		pn547_dev->clkreq_gpio = platform_data->clkreq_gpio;
	}
	pn547_dev->use_sys_clk = platform_data->use_sys_clk;
	pn547_dev->clk_run = false;
	pn547_dev->irq_wakeup_state = 0;
	pn547_dev->irq_enabled = true;

	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);

	ret = request_irq(client->irq, pn547_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn547_dev);
	if (ret) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}
	pn547_disable_irq(pn547_dev);

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = PN547_DRVNAME;
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	i2c_set_clientdata(client, pn547_dev);
	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		dev_err(&client->dev, "%s: misc_register failed\n", __FILE__);
		goto err_misc_register;
		}

	/*
	 * To be efficient we need to test whether nfcc hardware is physically
	 * present before attempting further hardware initialisation.
	 *
	 */
	ret= nfcc_hw_check(client, pn547_dev);
	if (ret) {
		/* make sure NFCC is not enabled */
		gpio_set_value(platform_data->ven_gpio, 0);
		if(gpio_get_value(platform_data->firm_gpio)){
			gpio_set_value(platform_data->firm_gpio, 0);
		}
		/* We don't think there is hardware switch NFC OFF */
		//goto err_request_hw_check_failed;
	}

	if( nfc_name != NULL){
		nfc_register_production();
	}

	dev_info(&client->dev,
			"%s, probing pn547 driver exited successfully\n",
			__func__);

	return 0;

//err_request_hw_check_failed:
	//misc_deregister(&pn547_dev->pn547_device);
err_misc_register:
	free_irq(client->irq, client);
err_request_irq_failed:
	mutex_destroy(&pn547_dev->read_mutex);
err_clkreq:
	if (platform_data->use_sys_clk){
		gpio_free(platform_data->clkreq_gpio);
	}
err_irq:
	gpio_free(platform_data->irq_gpio);
err_firm:
	gpio_free(platform_data->firm_gpio);
err_ven:
	gpio_free(platform_data->ven_gpio);
err_power:
	pn547_power_switch(pn547_dev,false);
err_power_switch:
	pn547_power_init(pn547_dev,false);
err_power_init:
	kfree(pn547_dev->kbuf);
err_mem:
	kfree(pn547_dev);
err_exit:
	dev_err(&client->dev, "%s(): error exit! ret = %d\n", __func__, ret);
	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(client);
	misc_deregister(&pn547_dev->pn547_device);
	free_irq(client->irq, pn547_dev);
	mutex_destroy(&pn547_dev->read_mutex);
	if (pn547_dev->use_sys_clk){
		gpio_free(pn547_dev->clkreq_gpio);
	}
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	kfree(pn547_dev->kbuf);
	kfree(pn547_dev);

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ PN547_NAME, 0 },
	{ }
};

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN547_NAME,
		.of_match_table = msm_match_table,
	},
};

static int __init pn547_dev_init(void)
{
	pr_info("Loading pn547 driver\n");
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	pr_info("Unloading pn547 driver\n");
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
