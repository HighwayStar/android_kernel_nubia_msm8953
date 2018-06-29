/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Version: 1.0
 * Revision record:

 * 	V1.0:
 * 		Add version information. 2016/10/25
 *
 */

#include "external_key.h"

#define LOG_TAG "HOMEKEY"
#define DEBUG_ON

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif


static int  external_key_parse_dt(struct external_key_platform *pdata)
{
	struct device_node *np = pdata->pdev->dev.of_node;
	int retval;
	u32 value;

	pdata->external_key_gpio = of_get_named_gpio_flags(np,"nubia,external_key-gpio", 0, NULL);

	retval = of_property_read_u32(np, "nubia,external_key-flags", &value);
	if (retval < 0)
		return retval;
	else
		pdata->external_key_irq_flags = value;

	retval = of_property_read_u32(np, "nubia,external_key_press-state",&value);
	if (retval < 0)
		return retval;
	else
		pdata->external_key_press_state = value;

	retval = of_property_read_u32(np, "nubia,external_key_old_statte",&value);
	if (retval < 0)
		return retval;
	else
		pdata->external_key_old_state = value;

	retval = of_property_read_u32(np, "nubia,external_key_button",&value);
	if (retval < 0)
		return retval;
	else
		pdata->external_key_button = value;

	SENSOR_LOG_INFO("%s: gpio=%d,irq_flag=%d,press_state=%d,old_state=%d,button=%d\n", __func__,pdata->external_key_gpio,pdata->external_key_irq_flags,pdata->external_key_press_state,pdata->external_key_old_state,pdata->external_key_button);

	return 0;
}

static int external_key_pinctrl_init(struct external_key_platform *pdata)
{
	pdata->pinctrl_info.pinctrl = devm_pinctrl_get(&pdata->pdev->dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl_info.pinctrl)) {
		SENSOR_LOG_ERROR("%s:external_key get pinctrl info error.\n", __func__);
		return -EINVAL;
	}

	pdata->pinctrl_info.pin_active = pinctrl_lookup_state(
					pdata->pinctrl_info.pinctrl,
					EXTERNAL_KEY_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(pdata->pinctrl_info.pin_active)) {
		SENSOR_LOG_ERROR("%s:external_key get pin_active info error.\n", __func__);
		return -EINVAL;
	}

	pdata->pinctrl_info.pin_suspend = pinctrl_lookup_state(
					pdata->pinctrl_info.pinctrl,
					EXTERNAL_KEY_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(pdata->pinctrl_info.pin_suspend)) {
		SENSOR_LOG_ERROR("%s : external_key get pin_suspend info error.\n",__func__);
		return -EINVAL;
	}

	return 0;
}

static int external_key_pinctrl_set_state(struct external_key_platform *pdata,bool active)
{
	int ret = -1;

	if (!pdata->pinctrl_info.pinctrl ||!pdata->pinctrl_info.pin_active ||!pdata->pinctrl_info.pin_suspend) {
		SENSOR_LOG_ERROR("%s : pinctrl is invalid, skip.\n",__func__);
		return ret;
	}
	if (active) {
		ret = pinctrl_select_state(pdata->pinctrl_info.pinctrl,
				pdata->pinctrl_info.pin_active);
	} else {
		ret = pinctrl_select_state(pdata->pinctrl_info.pinctrl,
				pdata->pinctrl_info.pin_suspend);
	}
	SENSOR_LOG_DEBUG("%s : set pinctrl to [%s], ret = %d.\n", __func__,active ? "active" : "suspend", ret);

	return ret;
}

static irqreturn_t external_key_irq_handler(int irq, void *data)
{
	  struct external_key_platform *pdata = data;
         int press_state;

         press_state =  gpio_get_value(pdata->external_key_gpio);
         press_state = !press_state;

	if (!pdata->external_key_old_state && !press_state) {
	    input_report_key(pdata->input_dev,pdata->external_key_button, 1);
	    input_sync(pdata->input_dev);
	}

	input_report_key(pdata->input_dev,pdata->external_key_button, press_state);
	input_sync(pdata->input_dev);

	pdata->external_key_old_state = !!press_state;

	return IRQ_HANDLED;
}

static int external_key_register_input_device(struct external_key_platform *pdata)
{
	int ret = 0;

	pdata->input_dev = input_allocate_device();
	if (!pdata->input_dev) {
		SENSOR_LOG_ERROR("no memory for idev\n");
		ret = -ENODEV;
		goto input_allocate_fail;
	}
	pdata->input_dev->name = EXTERNAL_KEY_DRV_NAME;
	pdata->input_dev->id.bustype = BUS_VIRTUAL;

	set_bit(EV_KEY,	pdata->input_dev->evbit);
	set_bit(pdata->external_key_button,pdata->input_dev->keybit);
	input_set_capability(pdata->input_dev,EV_KEY, pdata->external_key_button);

	ret = input_register_device(pdata->input_dev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n", pdata->input_dev->name);
		goto input_register_failed;
	}
	return 0;

input_register_failed:
	input_free_device(pdata->input_dev);
input_allocate_fail:
	return ret;
}


static int external_key_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[32];

	if (config) {
		retval = snprintf(buf, ARRAY_SIZE(buf), "external_key_gpio_%u\n", gpio);
		if (retval >= 32)
			return -EINVAL;

		retval = gpio_request(gpio, buf);
		if (retval) {
			SENSOR_LOG_ERROR("%s: Failed to get gpio %d (code: %d)",__func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			pr_err("%s: Failed to set gpio %d direction",
					__func__, gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

static int external_key_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct external_key_platform *pdata = NULL;

	SENSOR_LOG_INFO("probe start\n");

	pdata = kzalloc(sizeof(struct external_key_platform), GFP_KERNEL);
	if (!pdata) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	pdata->pdev = pdev;
	platform_set_drvdata(pdev, pdata);

	ret =  external_key_parse_dt(pdata);
	if (ret) {
		SENSOR_LOG_ERROR("parse dt failed\n");
		goto parse_dt_failed;
	}

	ret = external_key_pinctrl_init(pdata);
	if (ret < 0) {
		SENSOR_LOG_ERROR("%s: external_key pinctrl init failed.\n", __func__);
		goto pinctrl_init_failed;
	}
	ret = external_key_pinctrl_set_state(pdata,true);
	if (ret) {
		SENSOR_LOG_ERROR("%s: external_key pinctrl set state failed.\n", __func__);
		goto pinctrl_set_failed;
	}
	SENSOR_LOG_INFO("%s: external_key pinctrl inited, set state to active successfully.\n", __func__);

	ret = external_key_register_input_device(pdata);
	if (ret) {
		SENSOR_LOG_ERROR("%s: register_input_device failed.\n", __func__);
		goto input_register_failed;
	}

	ret = external_key_gpio_setup(pdata->external_key_gpio,true, 0, 0);
	if (ret) {
		SENSOR_LOG_ERROR("%s: external_key_set_gpio failed.\n", __func__);
		goto gpio_setup_fail;
	}

      pdata->external_key_irq = gpio_to_irq(pdata->external_key_gpio);
	if (pdata->external_key_irq < 0) {
		SENSOR_LOG_ERROR("gpio_to_irq fail '%d'\n", pdata->external_key_irq);
		goto gpio_to_irq_fail;
	}
	ret = request_irq(pdata->external_key_irq, external_key_irq_handler,pdata->external_key_irq_flags,EXTERNAL_KEY_DRV_NAME, (void*)pdata);
	if (ret < 0)
	{
		SENSOR_LOG_ERROR("%s: Failed to create homekey irq thread\n",__func__);
		goto request_irq_faile;
	}
	enable_irq_wake(pdata->external_key_irq);
	SENSOR_LOG_INFO("probe finished\n");

	return 0;

request_irq_faile:
	free_irq(pdata->external_key_irq,pdata);
gpio_to_irq_fail:
	external_key_gpio_setup(pdata->external_key_gpio,false, 0, 0);
gpio_setup_fail:
	input_unregister_device(pdata->input_dev);
input_register_failed:
pinctrl_set_failed:
	devm_pinctrl_put(pdata->pinctrl_info.pinctrl);
pinctrl_init_failed:
	pdata->pinctrl_info.pinctrl = NULL;
parse_dt_failed:
	if(pdata){
		kfree(pdata);
		pdata = NULL;
	}
malloc_failed:
	return ret;
}

static int external_key_remove(struct platform_device *pdev)
{
	struct external_key_platform *pdata = platform_get_drvdata(pdev);
	SENSOR_LOG_INFO("hall_device_remove\n");
	disable_irq(pdata->external_key_irq);
	free_irq(pdata->external_key_irq,pdata);
	devm_pinctrl_put(pdata->pinctrl_info.pinctrl);
	input_unregister_device(pdata->input_dev);
	if(pdata){
		kfree(pdata);
		pdata = NULL;
	}
	return 0;
}

static int external_key_resume(struct device *dev)
{
	struct external_key_platform *pdata = dev_get_drvdata(dev);
	if (external_key_pinctrl_set_state(pdata, true)) {
		SENSOR_LOG_ERROR("%s: synaptics pinctrl set active failed.\n", __func__);
	}
      disable_irq_wake(pdata->external_key_irq);
	SENSOR_LOG_INFO("%s: resume finished.\n", __func__);
	return 0 ;
}

static int external_key_suspend(struct device *dev)
{

	struct external_key_platform *pdata = dev_get_drvdata(dev);
	if (external_key_pinctrl_set_state(pdata, false)) {
		SENSOR_LOG_ERROR("%s: synaptics pinctrl set suspend failed.\n", __func__);
	}
	enable_irq_wake(pdata->external_key_irq);
	SENSOR_LOG_INFO("%s: suspend start.\n", __func__);
	return 0 ;
}

static struct of_device_id of_external_key_idtable[] = {
	{.compatible = "nubia,external_key",},
	{}
};

static const struct dev_pm_ops external_key_pm_ops = {
	.suspend	= external_key_suspend,
	.resume	= external_key_resume,
};

static struct platform_driver external_key_plat_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "external_key",
		.of_match_table = of_external_key_idtable,
		.pm = &external_key_pm_ops,
	},
	.probe = external_key_probe,
	.remove = external_key_remove,
};

static int __init external_key_init(void)
{
	return platform_driver_register(&external_key_plat_driver);
}

static void __exit external_key_exit(void)
{
	platform_driver_unregister(&external_key_plat_driver);
}

module_init(external_key_init);
module_exit(external_key_exit);

MODULE_DESCRIPTION("external_key driver");
MODULE_AUTHOR("NUBIA");
MODULE_LICENSE("GPL");

