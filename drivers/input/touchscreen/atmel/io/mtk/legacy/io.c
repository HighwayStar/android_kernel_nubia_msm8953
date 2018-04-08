/*
 * Atmel maXTouch Touchscreen driver Hardware interface
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Pitter.liao <pitter.liao@atmel.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/*-----------------------------------------------------------------*/
#define DRIVER_VERSION 0x0001
/*----------------------------------------------------------------
*	write version log here
*/
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/i2c.h>

#include "io/io.h"

#define MXT_HW_RESET_TIME		200	/* msec */

static void *mxt_g_data;
static struct device *t_dev;

extern void *mxt_g_data;
extern const struct dev_pm_ops mxt_pm_ops;

void mxt_active_proc_thread(void *dev_id, unsigned int event);
int mxt_probe(struct i2c_client *client,const struct i2c_device_id *id);
int mxt_remove(struct i2c_client *client);
void mxt_shutdown(struct i2c_client *client);

void board_pulse_irq_thread(void)
{
	void *dev_id = mxt_g_data;
	struct device *dev = t_dev;

	if (!mxt_g_data || !dev) {
		printk(KERN_ERR "MXT_EXTERNAL_TRIGGER_IRQ: mxt_g_data/dev is not prepared \n");
		return;
	}

	device_disable_irq(dev, __func__);
	mxt_active_proc_thread(dev_id,MXT_EVENT_IRQ);
}

int device_wait_irq_state(struct device *dev, int pin_level, long interval)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int state;
	unsigned long start_wait_jiffies = jiffies;
	unsigned long timeout;

	if (!pdata)
		return -ENODEV;

	/* Reset completion indicated by asserting CHG  */
	/* Wait for CHG asserted or timeout after 200ms */
	timeout = start_wait_jiffies + msecs_to_jiffies(interval);
	do {
		state = mt_get_gpio_in(pdata->gpio_irq);
		if (state == pin_level)
			break;

		//usleep_range(1000, 1000); 
		msleep(1);

		if (time_after_eq(jiffies, timeout)) {
			state = mt_get_gpio_in(pdata->gpio_irq);
			break;
		}
	} while (1);

	if (state == pin_level) {
		dev_dbg(dev, "irq took (%ld) %ums\n",
			interval, jiffies_to_msecs(jiffies - start_wait_jiffies));
		return 0;
	}else {
		dev_warn(dev, "timeout waiting(%ld) for idle %ums\n",
			interval, jiffies_to_msecs(jiffies - start_wait_jiffies));
		return -ETIME;
	}
}

void device_regulator_enable(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int error;

	if (!pdata)
		return;

	dev_info(dev, "Regulator on\n");

	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ZERO);

	error = hwPowerOn(TPD_LDO, VOL_3300, "touch_vdd");
	if (error) {
		dev_err(dev, "Error %d enabling vdd regulator\n", error);
		return;
	}

#if defined(TPD_LDO_AVDD)
	if (pdata->common_vdd_supply == 0) {
		error = hwPowerOn(TPD_LDO_AVDD, VOL_3300, "touch_avdd");
		if (error) {
			dev_err(dev, "Error %d enabling avdd regulator\n",
				error);
			return;
		}
	}
#endif

	msleep(20);

	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ONE);

	msleep(20);
	
	device_wait_irq_state(dev, 0, MXT_HW_RESET_TIME);
}

void device_regulator_disable(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return;

	dev_info(dev, "Regulator off\n");

	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ZERO);

	hwPowerDown(TPD_LDO, "touch_vdd");

#if defined(TPD_LDO_AVDD)
	if (pdata->common_vdd_supply == 0)
		hwPowerDown(TPD_LDO_AVDD, "touch_avdd");
#endif
}

int device_gpio_configure(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	//Config GPIO : Reset (Pin Name : CE)
	mt_set_gpio_mode(pdata->gpio_reset, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(pdata->gpio_reset, GPIO_DIR_OUT);
	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ONE);
	
	//Config GPIO : Interrupt (Pin Name : CHG)
	mt_set_gpio_mode(pdata->gpio_irq, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(pdata->gpio_irq, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(pdata->gpio_irq, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(pdata->gpio_irq, GPIO_PULL_UP);

	return 0;
}

void device_gpio_free(struct device *dev)
{

}

int device_power_init(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return -ENODEV;


	/*
		VDD: for T series: VDD 3.1~3.3v, for U series VDD 2.7~3.3v
		AVDD: suggest 3.3v for higer SNR
		suggest default value 3.3v both VDD and AVDD
	*/
	device_regulator_enable(dev);

	return 0;
}

void device_power_deinit(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return;

	dev_info(dev, "device regulator release\n");
}


int device_hw_reset(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return -EACCES;

	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ZERO);
	udelay(1500);
	mt_set_gpio_out(pdata->gpio_reset, GPIO_OUT_ONE);

	device_wait_irq_state(dev, 0, MXT_HW_RESET_TIME);

	return 0;

}

int device_por_reset(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return -EACCES;

	device_regulator_disable(dev);
	msleep(100);
	device_regulator_enable(dev);

	device_wait_irq_state(dev, 0, MXT_HW_RESET_TIME);
	return 0;
}

void device_disable_irq(struct device *dev, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_info(dev, "irq disabled ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	atomic_dec(&pdata->depth);

	WARN_ON(atomic_read(&pdata->depth) < -1);

	dev_dbg(dev, "irq disabled --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_disable_irq_nosync(struct device *dev, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_dbg(dev, "irq disabled nosync ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	atomic_dec(&pdata->depth);

	WARN_ON(atomic_read(&pdata->depth) < -1);

	dev_dbg(dev, "irq disabled nosync --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_disable_irq_wake(struct device *dev)
{

}

void device_enable_irq(struct device *dev, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_dbg(dev, "irq enabled ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	atomic_inc(&pdata->depth);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	WARN_ON(atomic_read(&pdata->depth) > 0);

	dev_dbg(dev, "irq enabled --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_enable_irq_wake(struct device *dev)
{

}

void device_free_irq(struct device *dev, void *dev_id, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (pdata && pdata->irq) {
		dev_info(dev, "irq free, %d %s\n", pdata->irq, name_str);
		mt_eint_unregistration(CUST_EINT_TOUCH_PANEL_NUM);
		atomic_set(&pdata->depth, 0);
	}
}

int device_register_irq(struct device *dev, irq_handler_t handler,
			irq_handler_t thread_fn, const char *devname, void *dev_id)
{
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_LOW, board_pulse_irq_thread, 0);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	return 0;
}

static char *device_find_text(char *head, char *delimiters, char **next)
{
	char *patch = head;

	for (; patch; patch = *next) {
		*next = strpbrk(patch, delimiters);
		if (*next)
			*(*next)++ = '\0';

		patch = skip_spaces(patch);
		if (!patch || !*patch || *patch == '#')
			continue;
		else
			break;
	}

	return patch;
}

static int device_parse_content(struct device *dev, int object, int instance, char *query,
		struct list_head *head, bool is_dig)
{
	int i, error;
	char *next, *value_p, *pair = query;
	long index_v, value_v;
	struct obj_link *ln;
	struct obj_container *con;

	for (i = 0; pair; pair = next, i++) {
		pair = device_find_text(pair, ",", &next);

		value_p = strpbrk(pair, "=");
		if (!value_p) {
			dev_err(dev, "T%d[%d]: invalid syntax '%s'\n",
				object, i, pair);
			continue;
		}

		/* make sure string is null terminated */
		*value_p = '\0';

		if (is_dig) {
			error = kstrtol(pair, 10, &index_v);
			if (error) {
				dev_err(dev, "T%d[%d]: dec conversion error %d\n", object, i, error);
				continue;
			}

			/* primitive data validation */
			if (index_v < 0 || index_v > 255) {
				dev_err(dev, "T%d[%d]: invalid index_v %ld\n", object, i, index_v);
				continue;
			}
			
		}else {
			index_v = *pair;
		}

		error = kstrtol(++value_p, 10, &value_v);
		if (error) {
			dev_err(dev, "T%d[%d]: hex conversion error\n", object, i);
			continue;
		}

		con = kzalloc(sizeof(*con), GFP_KERNEL);
		if (!con) {
			dev_err(dev, "failed to alloc mem container\n");
			return -ENOMEM;
		}

		INIT_LIST_HEAD(&con->node);
		con->id = (char)index_v;
		con->value = (int)value_v;

		ln = get_obj_link(head, object);
		if (!ln) {
			ln = kzalloc(sizeof(*ln), GFP_KERNEL);
			if (!ln) {
				dev_err(dev, "failed to alloc mem\n");
				return -ENOMEM;
			}

			ln->object = object;
			INIT_LIST_HEAD(&ln->sublist);
			INIT_LIST_HEAD(&ln->node);
			list_add_tail(&ln->node, head);
		}

		list_add_tail(&con->node, &ln->sublist);

		dev_info(dev, "T%d, id %d val %d(%c)\n",
			ln->object,con->id, con->value, (char)con->value);
	}

	return 0;
}


/*
 * Special settings can be passed to the driver via kernel cmd line
 * Example: atmxt="T100@0=1f,1=a0;T72@47=b2;T110-3@26=a"
 *   Where:
 *      T100    - decimal object number
 *      @       - delimits object number and following patch sets
 *      0=1f    - patch set decimal offset and hex value
 *      110-3   - object number and instance
 */
static void device_parse_setup_string(struct device *dev,
		const char *patch_ptr, struct list_head *head)
{
	long number_v, instance_v;
	char *patch_string;
	char *config_p, *instance_p, *next, *patch_set;
	int i, error;
	bool digital = true;

	if (!patch_ptr)
		return;

	patch_string = kstrdup(patch_ptr, GFP_KERNEL);
	for (i = 0, patch_set = patch_string; patch_set; patch_set = next) {
		patch_set = device_find_text(patch_set, ";\n", &next);
		if (!patch_set)
			break;

		dev_dbg(dev, "patch set %d: \"%s\"\n", i, patch_set);

		config_p = strpbrk(patch_set, "@$");

		if ((*patch_set != 'T' && *patch_set != 't') || !config_p) {
			dev_err(dev, "invalid syntax '%s'\n", patch_set);
			continue;
		}

		//charactor '@' mean digital for index ; '#' mean charactor for index
		if ((*config_p) == '$')
			digital = false;

		// strip non digits
		*config_p++ = '\0';

		instance_v = 0L;
		instance_p = strpbrk(patch_set, "-");
		if (instance_p) {
			*instance_p++ = '\0';
			error = kstrtol(instance_p, 10, &instance_v);
			if (error)
				dev_err(dev, "kstrtol error %d\n", error);
		}

		error = kstrtol(++patch_set, 10, &number_v);
		if (error) {
			dev_err(dev, "kstrtol error %d\n", error);
			continue;
		}

		error = device_parse_content(dev, (int)number_v, (int)instance_v,
				config_p, head, digital);
		if (error < 0) {
			dev_err(dev, "invalid patch; parse error %d\n", error);
			continue;
		}

		i++;
	}
	kfree(patch_string);
}

static void device_release_setup_string(struct device *dev, struct list_head *head)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	struct list_head *l,*c;
	struct obj_link *ln;
	struct obj_container *con;

	if (!pdata)
		return;

	dev_info(dev, "device release setup string\n");

	list_for_each(l, head) {
		ln = list_entry(l, struct obj_link, node);
		list_for_each(c, &ln->sublist) {
			con = list_entry(c, struct obj_container, node);
			dev_dbg(dev, "release T%d, id %d val %d(%c)\n",
				ln->object,con->id, con->value, (char)con->value);
			list_del(c);
			kfree(con);
			c = &ln->sublist;
		}
		list_del(l);
		kfree(ln);
		l = head;
	}
}


static const char key_button[] =
		"T15@0=139,1=172,2=158;";
static const char key_gesture[] =
		"T81@0=61;T93@0=61;T115@0=61,1=62;T116$v=64,o=65";

/**
 *	device_parse_dt - parse device tree for all hardware resource
 *	@dev: current device of driver
 *
 *	This call allocates IRQ number / GPIOs / Regulator and initilize the key event lists
 *
 */
static bool device_parse_platform(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int error;

	if (!pdata)
		return NULL;

	pdata->common_vdd_supply = 1;
	pdata->use_regulator = 0;
	if (pdata->use_regulator)
		dev_info(dev, "using suspend method: power off\n");

	/* key list */
	INIT_LIST_HEAD(&pdata->keylist);
	device_parse_setup_string(dev, key_button, &pdata->keylist);
	device_parse_setup_string(dev, key_gesture, &pdata->keylist);

	/* reset, irq gpio info */
	pdata->gpio_irq = GPIO_CTP_EINT_PIN;
	pdata->gpio_reset = GPIO_CTP_RST_PIN;
	pdata->irq = -1 /*CUST_EINT_TOUCH_PANEL_NUM*/;
	//----- here write irq io number
	//transfer to IRQ
	//---- here write reset io number

	error = device_gpio_configure(dev);
	if (error) {
		dev_err(dev, "failed to config gpio");
		return error;
	}

	error = device_power_init(dev);
	if (error) {
		dev_err(dev, "failed to init power\n");
		goto exit_parser;
	}

	dev_err(dev, "device parse dt successful\n");
	return 0;

exit_parser:
	dev_err(dev, "device parse dt failed\n");
	device_gpio_free(dev);

	return error;
}


static void device_release_platform(struct device *dev)
{
	device_regulator_disable(dev);
	device_power_deinit(dev);
	device_gpio_free(dev);
}

/**
 *	device_parse_default_dts - parse device tree for all hardware resource
 *	@dev: current device of driver
 *
 *	This is a packet of device_parse_dt()
 *
 */

int device_parse_default_chip(void *dev_id, struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int error = 0;

	if (!pdata)
		return -ENODEV;

	error = device_parse_platform(dev);
	if (!error) {
		mxt_g_data = dev_id;
		t_dev = dev;
	}
	return error;
}

void device_release_chip(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return;

	dev_err(dev, "device release dt\n");

	device_release_platform(dev);

	device_release_setup_string(dev, &pdata->keylist);

	mxt_g_data =NULL;
	t_dev = NULL;
}

static struct i2c_board_info mxt_i2c_tpd={
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4a),
		.platform_data = NULL};

static int tpd_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	
	pr_info("[mxt] tpd_probe\n");

	ret = mxt_probe(client,id);
	if(ret){
		pr_info("[mxt] tpd_probe failed %d\n", ret);
		return ret;
	}

	tpd_load_status = 1;

	return 0;
}

static int tpd_remove(struct i2c_client *client)
{
	tpd_load_status = 0;

	pr_info("[mxt] tpd_remove\n");

	return mxt_remove(client);
}

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ "Atmel MXT336T", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver tpd_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#if !(defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB))
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= tpd_probe,
	.remove		= tpd_remove,
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

static int tpd_local_init(void)
{
	int ret;
	
	pr_info("[mxt] Atmel MXT I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

	ret = i2c_add_driver(&tpd_driver);
	if(ret) {
		pr_err("[mxt] error unable to add i2c driver.\n");
		return ret;
	}

	if(tpd_load_status == 0) {  // disable auto load touch driver for linux3.0 porting
		pr_err("[mxt] atmel add error touch panel driver\n");
		i2c_del_driver(&tpd_driver);
		return -ENODEV;
	}

	pr_info("[mxt] %s, success %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;
	
	return 0;
}

static void tpd_suspend(struct device *h)
{
	// here you should call mxt_early_suspend() above if you don't use stand power interface
}

static void tpd_resume(struct device *h)
{
	// here you should call mxt_late_resume() above if you don't use stand power interface
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "atmel_mxt_ts",
	.tpd_local_init = tpd_local_init,
	.suspend =tpd_suspend,
	.resume = tpd_resume,
	//.tpd_have_button = 0,
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	int ret;
	
	pr_info("[mxt] tpd_driver_init\n");

	i2c_register_board_info(0, &mxt_i2c_tpd, 1);

	ret = tpd_driver_add(&tpd_device_driver);
	if(ret)
		pr_err("[mxt] tpd_driver_init failed %d\n", ret);

	return ret;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	pr_info("[mxt] tpd_driver_exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


