#ifndef __EXTERNAL_KEY_H__
#define __EXTERNAL_KEY_H__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>


#define EXTERNAL_KEY_DRV_NAME "homekey"
#define EXTERNAL_KEY_PINCTRL_STATE_SUSPEND 	"external_key_pin_suspend"
#define EXTERNAL_KEY_PINCTRL_STATE_ACTIVE 	"external_key_pin_active"

struct external_key_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_active;
	struct pinctrl_state *pin_suspend;
};

struct external_key_platform {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct external_key_pinctrl pinctrl_info;

	int external_key_button;
       int external_key_gpio;
	int external_key_irq;
	int external_key_irq_flags;
	int external_key_press_state;
	int external_key_old_state ;
};


#endif /* __EXTERNAL_KEY_H__ */
