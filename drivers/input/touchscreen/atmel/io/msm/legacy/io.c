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
#include "include/config.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/kthread.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include <linux/i2c.h>

#include "io/io.h"

#define MXT_HW_RESET_TIME		200	/* msec */

static void *mxt_g_data;
static struct device *t_dev;

extern void *mxt_g_data;
extern const struct dev_pm_ops mxt_pm_ops;
int mxt_probe(struct i2c_client *client,const struct i2c_device_id *id);
int mxt_remove(struct i2c_client *client);
void mxt_shutdown(struct i2c_client *client);

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
		state = gpio_get_value(pdata->gpio_irq);
		if (state == pin_level)
			break;

		//usleep_range(1000, 1000); 
		msleep(1);

		if (time_after_eq(jiffies, timeout)) {
			state = gpio_get_value(pdata->gpio_irq);
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

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_set_value(pdata->gpio_reset, 0);

	if (pdata->reg_vdd) {
		error = regulator_enable(pdata->reg_vdd);
		if (error) {
			dev_err(dev, "Error %d enabling vdd regulator\n", error);
			return;
		}
	}

	if (pdata->common_vdd_supply == 0) {
		if (pdata->reg_avdd) {
			error = regulator_enable(pdata->reg_avdd);
			if (error) {
				regulator_disable(pdata->reg_vdd);
				dev_err(dev, "Error %d enabling avdd regulator\n",
					error);
				return;
			}
		}
	}

	msleep(20);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_set_value(pdata->gpio_reset, 1);

	msleep(20);
	
	device_wait_irq_state(dev, 0, MXT_HW_RESET_TIME);
}

void device_regulator_disable(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return;

	dev_info(dev, "Regulator off\n");

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_set_value(pdata->gpio_reset, 0);

	if (pdata->reg_vdd)
		regulator_disable(pdata->reg_vdd);

	if (pdata->common_vdd_supply == 0) {
		if (pdata->reg_avdd)
			regulator_disable(pdata->reg_avdd);
	}
}

int device_gpio_configure(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
#ifdef CONFIG_OF
	struct pinctrl *pinctrl;
#endif
	int error = -EINVAL;

	if (!pdata)
		return error;

#ifdef CONFIG_OF
	pinctrl = devm_pinctrl_get_select(dev, "active");
	if (IS_ERR(pinctrl)) {
		error = PTR_ERR(pinctrl);
		dev_err(dev, "pinctrl failed err %d\n", error);
		return error;
	}
#endif
	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */

	dev_info(dev, "config gpio: irq[%d] reset[%d]\n", 
		pdata->gpio_irq, pdata->gpio_reset);
	 
	if (gpio_is_valid(pdata->gpio_reset)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(pdata->gpio_reset, "mxt_reset_gpio");
		if (error) {
			dev_err(dev, "unable to request gpio [%u] error %d\n",
				pdata->gpio_reset, error);
			goto fail;
		}

		error = gpio_direction_output(pdata->gpio_reset, 0);
		if (error) {
			dev_err(dev, "unable to set direction for gpio [%u] error %d\n",
				pdata->gpio_reset, error);
			goto fail_release_reset_gpio;
		}
	} else {
		dev_err(dev, "reset gpio not provided\n");
		error = -EIO;
		goto fail;
	}

	if (gpio_is_valid(pdata->gpio_irq)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_irq, "mxt_irq_gpio");
		if (error) {
			dev_err(dev, "unable to request gpio [%u] error %d\n",
				pdata->gpio_irq, error);
			goto fail_release_reset_gpio;
		}
		error = gpio_direction_input(pdata->gpio_irq);
		if (error) {
			dev_err(dev, "unable to set direction for gpio [%u] error %d\n",
				pdata->gpio_irq, error);
			goto fail_release_gpios;
		}

		//may set pullup ...
		
	} else {
		dev_err(dev, "irq gpio not provided\n");
		error = -EIO;
		goto fail_release_reset_gpio;
	}
	dev_err(dev, "irq pin config success\n");

	return 0;

fail_release_gpios:
	gpio_free(pdata->gpio_irq);
fail_release_reset_gpio:
	gpio_free(pdata->gpio_reset);
fail:
	dev_err(dev, "gpio config failed\n");

	return error;
}

void device_gpio_free(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
#ifdef CONFIG_OF
	struct pinctrl *pinctrl;
#endif

	if (!pdata)
		return;

	dev_info(dev, "device gpio free\n");

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);

	if (gpio_is_valid(pdata->gpio_irq))
		gpio_free(pdata->gpio_irq);

#ifdef CONFIG_OF
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_err(dev, "pinctrl failed err %ld\n", PTR_ERR(pinctrl));
#endif
}

int device_power_init(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int error;

	if (!pdata)
		return -ENODEV;

	pdata->reg_vdd = regulator_get(dev, "touch_vdd");
	if (IS_ERR(pdata->reg_vdd)) {
		error = PTR_ERR(pdata->reg_vdd);
		dev_err(dev, "Error %d getting vdd regulator\n", error);
		goto fail;
	}

	/*
		VDD: for T series: VDD 3.1~3.3v, for U series VDD 2.7~3.3v
		AVDD: suggest 3.3v for higer SNR
		suggest default value 3.3v both VDD and AVDD
	*/

	/*
		NUBIA note: we use vdd to set vddio. No warries~ since vdd is tied to avdd.
	*/
	 if (regulator_count_voltages(pdata->reg_vdd) > 0) {
		error = regulator_set_voltage(pdata->reg_vdd, 1800000,
							1800000);
		if (error) {
			dev_err(dev,
				"regulator couldn't set reg_vdd, error=%d\n", error);
			goto fail_release_vdd;
		}
	}

	if (pdata->common_vdd_supply == 0) {
		pdata->reg_avdd = regulator_get(
					dev, "touch_avdd");
		if (IS_ERR(pdata->reg_avdd)) {
			error = PTR_ERR(pdata->reg_avdd);
			dev_err(dev, "Error %d avdd regulator get\n", error);
			goto fail_release_vdd;
		}
		if (regulator_count_voltages(pdata->reg_avdd) > 0) {
			error = regulator_set_voltage(pdata->reg_avdd, 3300000,
								3300000);
			if (error) {
				dev_err(dev,
					"regulator couldn't set reg_avdd, error=%d\n", error);
				//goto fail_release_avdd;
			}
		}
	}

	device_regulator_enable(dev);

	dev_info(dev, "device power config success\n");
	return 0;

//fail_release_avdd:
	regulator_put(pdata->reg_avdd);
fail_release_vdd:
	regulator_put(pdata->reg_vdd);
fail:
	dev_err(dev, "device power config failed\n");
	pdata->reg_vdd = NULL;
	pdata->reg_avdd = NULL;
	return error;
}

void device_power_deinit(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return;

	dev_info(dev, "device regulator release\n");

	regulator_put(pdata->reg_vdd);

	if (pdata->common_vdd_supply == 0)
		regulator_put(pdata->reg_avdd);

}

int device_hw_reset(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata)
		return -EACCES;

	gpio_set_value(pdata->gpio_reset, 0);
	udelay(1500);
	gpio_set_value(pdata->gpio_reset, 1);

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

	dev_dbg(dev, "irq disabled ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	disable_irq(pdata->irq);
	atomic_dec(&pdata->depth);

	WARN_ON(atomic_read(&pdata->depth) < -1);

	dev_dbg(dev, "irq disabled --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_disable_irq_nosync(struct device *dev, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_dbg(dev, "irq disabled nosync ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	disable_irq_nosync(pdata->irq);
	atomic_dec(&pdata->depth);

	WARN_ON(atomic_read(&pdata->depth) < -1);

	dev_dbg(dev, "irq disabled nosync --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_disable_irq_wake(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	disable_irq_wake(pdata->irq);

	dev_info(dev, "irq wake disable, depth %d \n", atomic_read(&pdata->depth));
}

void device_enable_irq(struct device *dev, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_dbg(dev, "irq enabled ++, depth %d, %s\n", atomic_read(&pdata->depth), name_str);

	atomic_inc(&pdata->depth);
	enable_irq(pdata->irq);

	WARN_ON(atomic_read(&pdata->depth) > 0);

	dev_dbg(dev, "irq enabled --, depth %d, %s\n", atomic_read(&pdata->depth), name_str);
}

void device_enable_irq_wake(struct device *dev)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	dev_info(dev, "irq wake enable, depth %d \n", atomic_read(&pdata->depth));

	enable_irq_wake(pdata->irq);
}

void device_free_irq(struct device *dev, void *dev_id, const char * name_str)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);

	if (pdata && pdata->irq) {
		dev_info(dev, "irq free, %d %s\n", pdata->irq, name_str);
		free_irq(pdata->irq, dev_id);
		atomic_set(&pdata->depth, 0);
	}
}

int device_register_irq(struct device *dev, irq_handler_t handler,
			irq_handler_t thread_fn, const char *devname, void *dev_id)
{
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	int ret;
	
	dev_info(dev, "register irq %d hanlder %p thread_fn %p\n",pdata->irq, handler, thread_fn);

	if (handler)
		ret = request_irq(pdata->irq, handler, pdata->irqflags, devname, dev_id);
	else
		ret = request_threaded_irq(pdata->irq, NULL, thread_fn, pdata->irqflags | IRQF_ONESHOT, devname, dev_id);

	return ret;
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

#ifdef CONFIG_OF

int device_dt_parse_state(struct device *dev, struct device_node *np_config,
		struct list_head *head)
{
	const char *patch_data;
	struct device_node *np_state;
	int err;

	np_state = of_node_get(np_config);
	err = of_property_read_string(np_config, "list",
				(const char **)&patch_data);
	if (err < 0) {
		dev_err(dev, "unable to read data\n");
		return err;
	}

	dev_dbg(dev, "processing state: %s\n", patch_data);
	device_parse_setup_string(dev, patch_data, head);

	return 0;
}

#define MAX_NUM_STATES	2
int device_dt_parse_mode(struct device *dev, const char *name,
		struct list_head *head)
{
	struct device_node *np = dev->of_node;
	struct device_node *np_modes;
	int ret;
	char *propname;
	struct property *prop;
	const __be32 *list;
	int size, config;
	phandle phandle;
	struct device_node *np_config;

	propname = kasprintf(GFP_KERNEL, "%s_modes", name);
	dev_dbg(dev, "processing mode %s\n", propname);
	np_modes = of_find_node_by_name(np, propname);
	kfree(propname);
	if (!np_modes) {
		dev_err(dev, "can't find node %s\n", propname);
		ret = -EINVAL;
		goto err;
	}

	propname = kasprintf(GFP_KERNEL, "%s-mode", name);
	dev_dbg(dev, "processing mode %s\n", propname);
	prop = of_find_property(np_modes, propname, &size);
	kfree(propname);
	of_node_put(np_modes);
	if (!prop) {
		dev_err(dev, "can't find mode %s\n", propname);
		ret = -EINVAL;
		goto err;
	}
	list = prop->value;
	size /= sizeof(*list);

	if (size > MAX_NUM_STATES) {
		dev_err(dev, "unexpected number of states %d\n", size);
		ret = -EINVAL;
		goto err;
	}

	for (config = 0; config < size; config++) {
		phandle = be32_to_cpup(list++);

		/* Look up the touchstate configuration node */
		np_config = of_find_node_by_phandle(phandle);
		if (!np_config) {
			dev_err(dev,
				"prop %s index %i invalid phandle\n",
				prop->name, config);
			ret = -EINVAL;
			goto err;
		}

		/* Parse the node */
		ret = device_dt_parse_state(dev, np_config,
				 head);
		of_node_put(np_config);
		if (ret < 0)
			goto err;
	}
err:
	return ret;
}

/**
 *	device_parse_dt - parse device tree for all hardware resource
 *	@dev: current device of driver
 *
 *	This call allocates IRQ number / GPIOs / Regulator and initilize the key event lists
 *
 */
static bool device_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct mxt_platform_data *pdata = dev_get_platdata(dev);
	u32 out_value[2];
	unsigned long flags;
	int error;

	if (!pdata)
		return NULL;

	pdata->common_vdd_supply = of_property_read_bool(np,
					"atmel,common-vdd-supply");
	pdata->use_regulator = of_property_read_bool(np,
					"atmel,suspend-power-off");
	if (pdata->use_regulator)
		dev_info(dev, "using suspend method: power off\n");
	dev_info(dev, "common %d, regulator %d\n", 
		pdata->common_vdd_supply, pdata->use_regulator);

	/* key list */
	INIT_LIST_HEAD(&pdata->keylist);

	error = device_dt_parse_mode(dev, "keyevent", &pdata->keylist);
	if (error) {
		dev_err(dev, "failed to load default mode\n");
		return error;
	}

	/* reset, irq gpio info */

	pdata->gpio_irq = of_get_gpio(np, 0);
	//irqflags
	flags = 0;
	if (pdata->irq) {
		error = of_property_read_u32_array(np, "interrupts", out_value, 2);
		if (error == 0) {
			flags = out_value[1];
			dev_info(dev, "get of irqflags 0x%lx\n", flags);
		}
	}else {
		//transfer to IRQ
		pdata->irq = gpio_to_irq(pdata->gpio_irq);
	}
	if (!flags)
		flags = IRQF_TRIGGER_LOW;
	pdata->irqflags = flags;

	pdata->gpio_reset = of_get_gpio(np, 1);

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

static void device_release_dt(struct device *dev)
{
	device_regulator_disable(dev);
	device_power_deinit(dev);
	device_gpio_free(dev);
}
#else

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
	unsigned long flags;
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

	pdata->gpio_irq = 0;	//----- here write irq io number
	//transfer to IRQ
	pdata->irq = gpio_to_irq(pdata->gpio_irq);

	if (!flags)
		flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	pdata->irqflags = flags;

	pdata->gpio_reset = 0;  //---- here write reset io number

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

#endif

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

#ifdef CONFIG_OF
	if (dev->of_node)
		error = device_parse_dt(dev);
#else
	error = device_parse_platform(dev);
#endif

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
#ifdef CONFIG_OF
	device_release_dt(dev);
#else
	device_release_platform(dev);
#endif
	device_release_setup_string(dev, &pdata->keylist);
	
	mxt_g_data =NULL;
	t_dev = NULL;
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

#ifdef CONFIG_OF
static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,mxt-ts",},
	{ },
};
#else
#define mxt_match_table NULL
#endif

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mxt_match_table,
#endif
#if !(defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB))
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

module_i2c_driver(mxt_driver);

