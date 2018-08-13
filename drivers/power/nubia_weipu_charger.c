/* Copyright (c) 2014-2015 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "WPCHG: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include "nubia_weipu_charger.h"
#include <linux/qpnp/qpnp-adc.h>

#define REF_MIN_UV		1800000
#define REF_MAX_UV		1800000
#define WP_MIN_CHG_VOL	3500000
#define USB_VALID_MIN_MV	    3000
#define USB_REMOVE_CHECK_MS     500
#define DELAY_TIME_MS	        10000
#define WP_MAX_CHG_SEC          4 * 3600
#define WP_VOTE_MASK1           0x3F    //not include timeout mask
#define WP_VOTE_MASK_TOUT       0x20

#define WP_CHG_MIN_MA    2000
#define STEP_MA          200
#define CHECK_CNT        6

#define WP_OV_MV         4550
#define WP_OC_MA         5400

#define USB_CONN_DETECT  0

struct weipu_plat
{
	struct device	*dev;
	int weipu_en_gpio;
	int weipu_st_gpio;
    int weipu_st_irq;
    int weipu_chg_st;    //wp charging status
	int is_wp_present;   //wp charger is present whenever it is wp charging or wp charger but pmi charging
	int en_st;
	int batt_warm;
	int cv_chg;
	int first_check;
	int chg_cfg;
	int chg_time_out;
	int batt_cool_temp;
	int batt_warm_temp;
	int chg_limit_temp;
	int usb_hot_temp;
	int batt_cv_soc;
	int fake_charging;   // stay charging status when switch wp to pmi
	int current_ma;
	int pre_current;
	int max_chg_ma;
	int check_cnt;
	int lcd_on;
	unsigned int vote_mask;
	struct qpnp_vadc_chip  *vadc_dev;
	struct mutex			en_cntl_lock;
	struct mutex	        current_lock;
	spinlock_t			    mask_lock;
	struct pinctrl		   *wp_pinctrl;
    struct pinctrl_state   *wp_gpio_state;	
	struct wake_lock        wlock;
	struct wake_lock        check_lock;
	struct delayed_work	    wp_chg_work;
	struct delayed_work	    wp_fake_chg_work;
	struct delayed_work	    wp_switch_check_work;
	struct delayed_work	    usb_rm_check_work;
	struct work_struct      timeout_work;
	struct power_supply	   *batt_psy;
	struct power_supply	   *parallel_psy;
	struct power_supply	   *usb_psy;
	struct power_supply		wp_psy;
	struct regulator *ref_vdd;
	struct hrtimer    wp_hrtimer;
};

#if USB_CONN_DETECT
static const struct vadc_map_pt usb_ntc_map[] = {
	{-200, 1560},
	{-150, 1506},
	{-100, 1446},
	{-50, 1379},
	{0, 1306},
	{50, 1228},
	{100, 1147},
	{150, 1065},
	{200, 981},
	{250, 900},
	{300, 816},
	{350, 741},
	{400, 667},
	{450, 600},
	{500, 541},
	{550, 486},
	{600, 425},
	{650, 382},
	{700, 336},
	{750, 300},
	{800, 274},
	{850, 234},
	{900, 221},
	{950, 192},
	{1000, 163},
	{1150, 148},
	{1200, 133},
	{1250, 117},
};
#endif

#define WPCHG_DEBUG    3
#define WPCHG_INFO     5
//ztelog > wpchg_level will show
static int wpchg_level = 4;
module_param(wpchg_level, int, 0644);
#define wpchg_debug(x...) do {if (WPCHG_DEBUG > wpchg_level) pr_err(x); } while (0)
#define wpchg_info(x...)  do {if (WPCHG_INFO  > wpchg_level) pr_err(x); } while (0)

extern int set_usb_chg_current(int usb_current);

struct weipu_plat *wchip;

static enum power_supply_property wp_chg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static char *wp_chg_power_supplied_to[] = {
	"battery",
};

static int weipu_pinctrl_select(struct weipu_plat *wp_chip)
{
	 struct pinctrl_state *pins_state;
	 int ret;

	 if(!wp_chip->wp_pinctrl){
		pr_err( "wp_pinctrl not init\n");
		return -1;
	 }
	 
	 pins_state = wp_chip->wp_gpio_state;
	 
	 if (!IS_ERR_OR_NULL(pins_state)) {
		 ret = pinctrl_select_state(wp_chip->wp_pinctrl, pins_state);
		 if (ret) {
			 pr_err( "can not set gpio pins(%d)\n",ret );
			 return ret;
		 }
	 } else{ 
		 pr_err("not a valid gpio pinstate\n");
		 return -1;
	 }

	 pr_info( "wp_pinctrl config ok\n");
	 return 0;
}

static int weipu_pinctrl_init(struct weipu_plat *wp_chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	wp_chip->wp_pinctrl = devm_pinctrl_get(wp_chip->dev);
	if (IS_ERR_OR_NULL(wp_chip->wp_pinctrl)) {
		pr_err(	"Target does not use pinctrl! \n");
		retval = PTR_ERR(wp_chip->wp_pinctrl);
		wp_chip->wp_pinctrl = NULL;
		return retval;
	}

	wp_chip->wp_gpio_state
		= pinctrl_lookup_state(wp_chip->wp_pinctrl,"default");
	if (IS_ERR_OR_NULL(wp_chip->wp_gpio_state)) {
		pr_err("Can not get ts default pinstate! \n");
		retval = PTR_ERR(wp_chip->wp_gpio_state);
		wp_chip->wp_pinctrl = NULL;
		return retval;
	}

	pr_info("WP PinCtrl Init Success!\n");
	return 0;
}

static int weipu_gpio_enable(struct weipu_plat *wp_chip)
{
	int rc;

	if(wp_chip->wp_pinctrl){
		rc = weipu_pinctrl_select(wp_chip);
		if(rc < 0)
			return rc;
	}

	//weipu enable gpio
	if (gpio_is_valid(wp_chip->weipu_en_gpio)) {
		rc = gpio_request(wp_chip->weipu_en_gpio,"weipu-en-gpio");
		if (rc < 0){ 
			pr_err("Unable to request gpio=%d rc = %d\n",wp_chip->weipu_en_gpio,rc);
			return -EINVAL;
		}

    	rc = gpio_direction_output(wp_chip->weipu_en_gpio, 0); 
		if (rc < 0) {
			pr_err("Unable to request gpio=%d rc = %d\n",wp_chip->weipu_en_gpio, rc);
			return -EINVAL;
		}
	}

    //weipu chg status gpio
	if (gpio_is_valid(wp_chip->weipu_st_gpio)) {
		rc = gpio_request(wp_chip->weipu_st_gpio,"weipu-st-gpio");
		if (rc < 0){ 
			pr_err("Unable to request gpio=%d rc = %d\n",wp_chip->weipu_st_gpio,rc);
			return -EINVAL;
		}
			
		rc = gpio_direction_input(wp_chip->weipu_st_gpio); 
		if (rc < 0){ 						
			dev_err(wp_chip->dev, "Unable to config weipu-chg-irq input rc=%d\n",rc);
			return -EINVAL;
		}
	}

	return 0;
}

static int test_vote(unsigned int vote_status, enum chg_voter voter)
{
	return vote_status & voter;
}

static int set_vote(unsigned int *vote_status, enum chg_voter voter)
{
	return *vote_status |= voter;
}

static int clear_vote(unsigned int *vote_status, enum chg_voter voter)
{
	return *vote_status &= (!voter);
}

#define DEFAULT_USB_VOLTAGE	 0
static int get_usbin_voltage_mv(struct weipu_plat *wp_chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	
	if(!wp_chip->vadc_dev){
	    wp_chip->vadc_dev = qpnp_get_vadc(wp_chip->dev, "usbin");
	    if (IS_ERR(wp_chip->vadc_dev)) {
			pr_err(" vadc_dev NULL ! \n");
			return DEFAULT_USB_VOLTAGE;
	    }
	}

	if(wp_chip->vadc_dev){
	    rc = qpnp_vadc_read(wp_chip->vadc_dev, USBIN, &results);
	    if (rc) {
			pr_err("Unable to read usbin rc=%d\n", rc);
			return DEFAULT_USB_VOLTAGE ;
	    }else{
			return results.physical/1000;
	    }
	}else{
	    return DEFAULT_USB_VOLTAGE;
    }
}  

#if USB_CONN_DETECT
static int32_t wp_adc_map_temp_voltage(const struct vadc_map_pt *pts,
		int tablesize, int input, int *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

#define DEFAULT_USB_NTC_VOLTAGE	 0
#define MPP1_CHAN	 0x10
static int get_usb_temp_sensor_mv(struct weipu_plat *wp_chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	
	if(!wp_chip->vadc_dev){
	    wp_chip->vadc_dev = qpnp_get_vadc(wp_chip->dev, "usbin");
	    if (IS_ERR(wp_chip->vadc_dev)) {
			pr_err(" vadc_dev NULL ! \n");
			return DEFAULT_USB_NTC_VOLTAGE;
	    }
	}

	if(wp_chip->vadc_dev){
	    rc = qpnp_vadc_read(wp_chip->vadc_dev, MPP1_CHAN, &results);
	    if (rc) {
			pr_err("Unable to read usbin rc=%d\n", rc);
			return DEFAULT_USB_NTC_VOLTAGE ;
	    }else{
			return results.physical/1000;
	    }
	}else{
	    return DEFAULT_USB_NTC_VOLTAGE;
    }
}  

static int wp_get_usb_conn_temp(struct weipu_plat *wp_chip)
{
	int sensor_mv;
	int usb_conn_temp;
    int rc;

	sensor_mv = get_usb_temp_sensor_mv(wp_chip);

	wpchg_debug("get sensor_mv=%d\n", sensor_mv);

	rc = wp_adc_map_temp_voltage(usb_ntc_map, 
		                    ARRAY_SIZE(usb_ntc_map),
		                    sensor_mv,
		                    &usb_conn_temp);
	if(rc){
		pr_err("get usb temp err rc=%d\n",rc);
		return rc;
	}

	return usb_conn_temp;
}

static int wp_usb_ntc_regulator_enable(struct weipu_plat *wp_chip, int enable)
{
	int ret = 0;
	
	if(!wp_chip->ref_vdd){
		pr_err("wp do not init\n");
		return -1;
	}

	wpchg_debug("ref_vdd regulator enable=%d\n", enable);

	if(enable){
		ret = regulator_enable(wp_chip->ref_vdd);
		if (ret) {
			pr_err("unable to enable ref_vdd\n");
			return -1;
		}
	}else{
		ret = regulator_disable(wp_chip->ref_vdd);
		if (ret) {
			pr_err("unable to disable ref_vdd\n");
			return -1;
		}
	}

	return ret;
}
#endif

int is_wp_chg_present(void)
{
    if(!wchip){
		pr_err("weipu chg is not inited\n");
		return -1;
    }

	return wchip->is_wp_present;
}

int notify_lcd_on(int lcd_on)
{
    if(!wchip){
		pr_err("weipu chg is not inited\n");
		return -1;
    }

    wchip->lcd_on = lcd_on;
	return 0;
}

int is_wp_fake_chging(void)
{
    if(!wchip){
		pr_err("weipu chg is not inited\n");
		return 0;
    }
	
	wpchg_debug("fake_charging=%d\n",wchip->fake_charging);

	return wchip->fake_charging;
}

static void set_wp_chg_present(struct weipu_plat *wp_chip, int present)
{
	wp_chip->is_wp_present = present;
	return;
}

static int wp_get_batt_prop(struct weipu_plat *wp_chip, enum power_supply_property prop, int *val)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (wp_chip->batt_psy == NULL)
		wp_chip->batt_psy = power_supply_get_by_name("battery");
	
	if (wp_chip->batt_psy) {
		rc = wp_chip->batt_psy->get_property(wp_chip->batt_psy, prop, &ret);
		if(rc){
			pr_err("fail to get battery prop rc = %d\n",rc);
			return rc;
		}
		*val = ret.intval;
	}
	
	return rc;
}

static int wp_get_usb_prop(struct weipu_plat *wp_chip, enum power_supply_property prop, int *val)
{
	union power_supply_propval ret = {0, };
	int rc = -1;

	if (wp_chip->usb_psy == NULL)
		wp_chip->usb_psy = power_supply_get_by_name("usb");
	
	if (wp_chip->usb_psy) {
		rc = wp_chip->usb_psy->get_property(wp_chip->usb_psy, prop, &ret);
		if(rc){
			pr_err("fail to get usb_psy prop rc = %d\n",rc);
			return rc;
		}
		*val = ret.intval;
	}
	
	return rc;
}

static int wp_set_usb_prop(struct weipu_plat *wp_chip, enum power_supply_property prop, int val)
{
	const union power_supply_propval ret = {val,};
	int rc = -1;

	if (wp_chip->usb_psy == NULL)
		wp_chip->usb_psy = power_supply_get_by_name("usb");
	
	if (wp_chip->usb_psy) {
		rc = wp_chip->usb_psy->set_property(wp_chip->usb_psy, prop, &ret);
		if(rc){
			pr_err("fail to get usb_psy prop rc = %d\n",rc);
			return rc;
		}
	}
	
	return rc;
}

static int pmi_charging_enable(struct weipu_plat *wp_chip, int enable)
{
	const union power_supply_propval ret = {enable,};
	int rc = -1;

	if (wp_chip->batt_psy == NULL)
		wp_chip->batt_psy = power_supply_get_by_name("battery");
	
	if (wp_chip->batt_psy) {
		wpchg_info("set pmi:%s\n",enable ? "Charging" : "Discharging");
		rc = wp_chip->batt_psy->set_property(wp_chip->batt_psy,
			 POWER_SUPPLY_PROP_CHARGING_ENABLED,
			 &ret);
	}

    //control parallel charger
    if (wp_chip->parallel_psy == NULL)
		wp_chip->parallel_psy = power_supply_get_by_name("usb-parallel");
	
	if (wp_chip->parallel_psy) {
		wpchg_info("set parallel_chg:%s\n",enable ? "Charging" : "Discharging");
		rc = wp_chip->parallel_psy->set_property(wp_chip->parallel_psy,
			 POWER_SUPPLY_PROP_CHARGING_ENABLED,
			 &ret);
	}
	
	return rc;
}

static int wp_charging_enable(struct weipu_plat *wp_chip, int enable)
{
	int rc;

	wpchg_debug("set gpio[%d]=%d\n",wp_chip->weipu_en_gpio, enable);

	mutex_lock(&wp_chip->en_cntl_lock);

	wp_chip->en_st = enable;
	rc = gpio_direction_output(wp_chip->weipu_en_gpio, !!enable);
    if(rc < 0)
		pr_err("Unable to request gpio=%d rc = %d\n",wp_chip->weipu_en_gpio, rc);
	
	mutex_unlock(&wp_chip->en_cntl_lock);
	return rc;
}

static int wp_update_power_changed(struct weipu_plat *wp_chip)
{
	if (wp_chip->batt_psy == NULL)
		wp_chip->batt_psy = power_supply_get_by_name("battery");

    if (wp_chip->batt_psy)
		power_supply_changed(wp_chip->batt_psy);
	return 0;
}

static int wp_notify_usb_status(struct weipu_plat *wp_chip, int usb_status)
{
	if (wp_chip->usb_psy == NULL)
		wp_chip->usb_psy = power_supply_get_by_name("usb");
	
	if (wp_chip->usb_psy) {
		if(usb_status == USB_INSERT_EVENT){
            power_supply_set_present(wp_chip->usb_psy, 1);
		    power_supply_set_online(wp_chip->usb_psy, 1);
		    power_supply_set_health_state(wp_chip->usb_psy, POWER_SUPPLY_HEALTH_GOOD);
		    power_supply_set_supply_type(wp_chip->usb_psy, POWER_SUPPLY_TYPE_USB_DCP);
	    }else{
		    power_supply_set_present(wp_chip->usb_psy, 0);
		    power_supply_set_online(wp_chip->usb_psy, 0);
		    power_supply_set_health_state(wp_chip->usb_psy, POWER_SUPPLY_HEALTH_UNKNOWN);
		    power_supply_set_supply_type(wp_chip->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);
	    }
			
		power_supply_changed(wp_chip->usb_psy);
	}

	return 0;
}

int weipu_is_charging_status(void)
{
    if(!wchip){
		pr_err("weipu chg is not inited\n");
		return -1;
    }
	
	return gpio_get_value(wchip->weipu_st_gpio);
}

static int wp_charging_enable_cntl(struct weipu_plat *wp_chip, int enable)
{
	int rc;
	
	if(wp_chip->vote_mask && enable){
		//this condition is that wp charging swith to pmi charging, then update the usb status
		wp_notify_usb_status(wp_chip, USB_INSERT_EVENT);
		pr_err("vote_mask=0x%x, wp chg is switch out.\n",wp_chip->vote_mask);
		return 0;
	}

	wpchg_info("Control wp charger:%s\n",enable ? "Enable" : "Disable");
	if(enable){
		wpchg_info("start wp chg work\n");
		wp_chip->first_check = 1;
		//set pmi charging current to 1000mA before switch to wp charging
		set_usb_chg_current(1000);
		#if USB_CONN_DETECT
		wp_usb_ntc_regulator_enable(wp_chip, 1);
		#endif
		cancel_delayed_work_sync(&wp_chip->wp_chg_work);
		schedule_delayed_work(&wp_chip->wp_chg_work, msecs_to_jiffies(6000));
	}else{
		wpchg_info("canel wp chg work\n");
		#if USB_CONN_DETECT
		wp_usb_ntc_regulator_enable(wp_chip, 0);
		#endif
		cancel_delayed_work_sync(&wp_chip->wp_chg_work);
		wp_chip->vote_mask = 0;
	}

	rc = wp_charging_enable(wp_chip, enable);

	if(!enable && wake_lock_active(&wp_chip->wlock))
		wake_unlock(&wp_chip->wlock);
	
	return rc;
}

static void weipu_chg_vote(struct weipu_plat *wp_chip, unsigned int voters, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&wp_chip->mask_lock, flags);
	wp_chip->vote_mask &= ~mask;
	wp_chip->vote_mask |= voters & mask;
	spin_unlock_irqrestore(&wp_chip->mask_lock, flags);
	
	return;
}

static void weipu_chg_cfg(struct weipu_plat *wp_chip, enum chg_cfg chg_select)
{
	wp_chip->chg_cfg = chg_select;

	pr_err("Switch charger:-->%s vote=0x%x cool=%d warm=%d cv=%d hot=%d\n",
		chg_select == CHG_CFG_WP?"WeiPu":"PMI",
		wp_chip->vote_mask,
		test_vote(wp_chip->vote_mask, WP_CHG_COOL),
		test_vote(wp_chip->vote_mask, WP_CHG_WARM),
		test_vote(wp_chip->vote_mask, WP_CHG_CV),
		test_vote(wp_chip->vote_mask, WP_CHG_USB_HOT)
	);
	
	if(chg_select == CHG_CFG_WP){
		//use weipu charging
		pmi_charging_enable(wp_chip, WP_CHG_DISABLE);
		wp_charging_enable(wp_chip, WP_CHG_ENABLE);
	}else if(chg_select == CHG_CFG_PMI){
	    //use pmi charging
		wp_charging_enable(wp_chip, WP_CHG_DISABLE);
		pmi_charging_enable(wp_chip, WP_CHG_ENABLE);
	}else{
		pr_err("wp chg cfg invalid\n");
	}
}

static void wp_fake_chg_worker(struct work_struct *work)
{
	struct weipu_plat *wp_chip = container_of(work, struct weipu_plat, wp_fake_chg_work.work);

	wp_chip->fake_charging = 0;
	return;
}

//check for when wp chg switch to pm chg,and remove the usb
static void usb_rm_check_worker(struct work_struct *work)
{
	struct weipu_plat *wp_chip = container_of(work, struct weipu_plat, usb_rm_check_work.work);
	int vbus_mv;

	if(!is_wp_chg_present()){
		pr_err("usb is remove, not need check\n");
		return;
	}

	vbus_mv = get_usbin_voltage_mv(wp_chip);
	if(vbus_mv < USB_VALID_MIN_MV){
		wpchg_info("usb is removed, vbus_mv=%d\n",vbus_mv);
		wp_chip->fake_charging = 0;
		wp_charging_enable_cntl(wp_chip, WP_CHG_DISABLE);
		set_wp_chg_present(wp_chip, WP_CHG_REMOVE);
		wp_notify_usb_status(wp_chip, USB_REMOVE_EVENT);
	}else if(vbus_mv >= USB_VALID_MIN_MV){
		wpchg_info("usb is still online, vbus_mv=%d\n",vbus_mv);
	}

	return;
}

int wp_charger_remove_check(void)
{
	if(!wchip){
		pr_err("wp do not init\n");
		return -1;
	}
	
	if( is_wp_chg_present() ){
		cancel_delayed_work(&wchip->usb_rm_check_work);
		schedule_delayed_work(&wchip->usb_rm_check_work, msecs_to_jiffies(600));
	}

	return 0;
}

#define MA_TO_HZ       100
#define HZ_TO_MS       1000
#define HOLD_TIME_MS   3000
static int wp_set_current(struct weipu_plat *wp_chip, int current_ma)
{
    int hz = current_ma / MA_TO_HZ;
    int cycle_us = HZ_TO_MS * 1000 / hz;
	int cycles = HOLD_TIME_MS * 1000 / cycle_us;
	int i, rc, flag = 0;

    mutex_lock(&wp_chip->current_lock);
	
	wp_chip->current_ma = current_ma;
	pr_err("set current=%d hz=%d cyc_us=%d cycles=%d\n",current_ma, hz, cycle_us, cycles);
	
	for(i=0; i<cycles * 2; i++){
		if(wp_chip->weipu_chg_st == 0){
			pr_err("charger switch out\n");
			gpio_direction_output(wp_chip->weipu_en_gpio, wp_chip->en_st);
			mutex_unlock(&wp_chip->current_lock);
			return 1;
		}
			
		flag = flag ? 0 : 1;
		rc = gpio_direction_output(wp_chip->weipu_en_gpio, flag);
    	if(rc < 0)
			pr_err("Unable to request gpio=%d rc = %d\n",wp_chip->weipu_en_gpio, rc);
		udelay(cycle_us / 2);
	}
	gpio_direction_output(wp_chip->weipu_en_gpio, 1);
	
	mutex_unlock(&wp_chip->current_lock);
    pr_err("send pwd end\n");
	return 0;
}

#define DEFAULT_BATT_TEMP	 200
#define TEMP_DELTA		     20
static void wp_chg_worker(struct work_struct *work)
{
	struct weipu_plat *wp_chip = container_of(work, struct weipu_plat, wp_chg_work.work);
	int batt_temp,batt_soc,batt_mv,batt_ma;
	int set_limit_temp;
	int current_chg_ma;
	int rc;
	static int ov_cnt = 0;
	static int oc_cnt = 0;
	int vbus_mv = get_usbin_voltage_mv(wp_chip);
	#if USB_CONN_DETECT
	int usb_temp = wp_get_usb_conn_temp(wp_chip);
	#endif
	unsigned int vote_status = wp_chip->vote_mask & WP_VOTE_MASK1;
	
	wp_chip->weipu_chg_st = gpio_get_value(wp_chip->weipu_st_gpio);
	
	if(wp_chip->first_check){
		wp_chip->first_check = 0;
		if(!wp_chip->weipu_chg_st && wp_chip->en_st){
			pr_err("weipu not charging, stop work\n");
			weipu_chg_cfg(wp_chip, CHG_CFG_PMI);
			set_usb_chg_current(1500);
			return;
		}

		ov_cnt = 0;
		oc_cnt = 0;		
	}
	
	rc = wp_get_batt_prop(wp_chip, POWER_SUPPLY_PROP_TEMP, &batt_temp);
	if(rc){
		batt_temp = DEFAULT_BATT_TEMP;
	}

	rc = wp_get_batt_prop(wp_chip, POWER_SUPPLY_PROP_CAPACITY, &batt_soc);
	if(rc){
		batt_soc = 100;
	}

	rc = wp_get_batt_prop(wp_chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_mv);
	if(rc){
		batt_mv = 0;
	}
	batt_mv = batt_mv/1000;

	rc = wp_get_batt_prop(wp_chip, POWER_SUPPLY_PROP_CURRENT_NOW, &batt_ma);
	if(rc){
		batt_ma = 0;
	}
	batt_ma = batt_ma/1000;

    //battery cool temperature check
    if(batt_temp <= wp_chip->batt_cool_temp)
		set_vote(&vote_status, WP_CHG_COOL);
	else if((batt_temp > wp_chip->batt_cool_temp+TEMP_DELTA) && test_vote(vote_status, WP_CHG_COOL))
		clear_vote(&vote_status, WP_CHG_COOL);

	//battery warm temperature check
	if(batt_temp >= wp_chip->batt_warm_temp)
		set_vote(&vote_status, WP_CHG_WARM);
	else if((batt_temp < wp_chip->batt_warm_temp-TEMP_DELTA) && test_vote(vote_status, WP_CHG_WARM))
		clear_vote(&vote_status, WP_CHG_WARM);

	//battery soc check
	if(batt_soc >= wp_chip->batt_cv_soc)
		set_vote(&vote_status, WP_CHG_CV);
	else if(batt_soc < wp_chip->batt_cv_soc && test_vote(vote_status, WP_CHG_CV))
		clear_vote(&vote_status, WP_CHG_CV);

    #if USB_CONN_DETECT
    //usb connecter temperature check
    if(usb_temp >= wp_chip->usb_hot_temp)
		set_vote(&vote_status, WP_CHG_USB_HOT);
	else if((usb_temp < wp_chip->usb_hot_temp-TEMP_DELTA) && test_vote(vote_status, WP_CHG_USB_HOT))
		clear_vote(&vote_status, WP_CHG_USB_HOT);
	#endif

	//over voltage check
    if(batt_mv >= WP_OV_MV && !test_vote(vote_status, WP_CHG_OV)){
		if(ov_cnt++ >= 2)
		    set_vote(&vote_status, WP_CHG_OV);
	}else if(batt_mv < WP_OV_MV){ 
	    ov_cnt = 0;
	    if( test_vote(vote_status, WP_CHG_OV) )
			clear_vote(&vote_status, WP_CHG_OV);
	}

	//over current check
    if(abs(batt_ma) >= WP_OC_MA && !test_vote(vote_status, WP_CHG_OC)){
		if(oc_cnt++ >= 2)
			set_vote(&vote_status, WP_CHG_OC);
	}else if(abs(batt_ma) < WP_OC_MA){ 
        oc_cnt = 0;
		if( test_vote(vote_status, WP_CHG_OC) )
		    clear_vote(&vote_status, WP_CHG_OC);
	}

	//check vote_status changes
	if((wp_chip->vote_mask & WP_VOTE_MASK1) != vote_status)
		weipu_chg_vote(wp_chip, vote_status, WP_VOTE_MASK1);

    //wp vote_mask check
	if(wp_chip->vote_mask && wp_chip->chg_cfg==CHG_CFG_WP){
		wp_chip->fake_charging = 1;
		cancel_delayed_work(&wp_chip->wp_fake_chg_work);
		schedule_delayed_work(&wp_chip->wp_fake_chg_work, msecs_to_jiffies(5000));
		weipu_chg_cfg(wp_chip, CHG_CFG_PMI);
	}else if(!wp_chip->vote_mask && wp_chip->chg_cfg==CHG_CFG_PMI)
		weipu_chg_cfg(wp_chip, CHG_CFG_WP);

    //wp charging current cntl
	if(wp_chip->chg_cfg == CHG_CFG_WP){
        wp_chip->check_cnt = (wp_chip->check_cnt+1) % CHECK_CNT;

		if(wp_chip->current_ma == 0)
			wp_set_current(wp_chip, wp_chip->max_chg_ma);

		if(wp_chip->pre_current == 0){
			current_chg_ma = abs(batt_ma);			
		}else{
			current_chg_ma = ((abs(batt_ma) + wp_chip->pre_current)/200) * 100;
		}
		pr_debug("batt_ma=%ld pre_ma=%d chg_ma=%d\n",abs(batt_ma), wp_chip->pre_current, current_chg_ma);
		wp_chip->pre_current = abs(batt_ma);	
		
		if(wp_chip->check_cnt == 0){
			if(wp_chip->lcd_on)
				set_limit_temp = wp_chip->chg_limit_temp - 10;
			else
				set_limit_temp = wp_chip->chg_limit_temp;
			
			if((batt_temp > set_limit_temp) && (wp_chip->current_ma != WP_CHG_MIN_MA)){
	
				wp_chip->current_ma = current_chg_ma - STEP_MA;
				if(wp_chip->current_ma < WP_CHG_MIN_MA)
					wp_chip->current_ma = WP_CHG_MIN_MA;
					
				pr_err("batt temp up,lcd_on=%d current=%d\n",wp_chip->lcd_on, wp_chip->current_ma);
				wp_set_current(wp_chip, wp_chip->current_ma);
			}else if((batt_temp < set_limit_temp-TEMP_DELTA) && (wp_chip->current_ma != wp_chip->max_chg_ma)){
	
                wp_chip->current_ma += STEP_MA;
				
				if((wp_chip->current_ma - current_chg_ma) >= 2000)
					wp_chip->current_ma = wp_chip->max_chg_ma;
				
				if(wp_chip->current_ma > wp_chip->max_chg_ma)
					wp_chip->current_ma = wp_chip->max_chg_ma;
				
				pr_err("batt temp down,lcd_on=%d current=%d\n",wp_chip->lcd_on, wp_chip->current_ma);
				wp_set_current(wp_chip, wp_chip->current_ma);
			}
		}
		
	}
	
	wpchg_info("->En=%d St=%d Soc=%d Tb=%d Vb=%d Ib=%d Vu=%d Vote=0x%x\n",
		wp_chip->en_st,
		wp_chip->weipu_chg_st,
		batt_soc,
		batt_temp,
		batt_mv,
		batt_ma,
		vbus_mv,
		wp_chip->vote_mask);
	
	schedule_delayed_work(&wp_chip->wp_chg_work, msecs_to_jiffies(DELAY_TIME_MS));
	return;
}

static void wp_switch_check_worker(struct work_struct *work)
{
	struct weipu_plat *wp_chip = container_of(work, struct weipu_plat, wp_switch_check_work.work);
	int batt_uv;
	int usb_present;

    if( wp_get_usb_prop(wp_chip, POWER_SUPPLY_PROP_PRESENT, &usb_present) ){
		usb_present = 0;
	}
	
	if( !usb_present ){
		pr_err("usb is remove, not need check\n");
		wake_unlock(&wp_chip->check_lock);
		return;
	}

	if( wp_get_batt_prop(wp_chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_uv) ){
		batt_uv = 0;
	}

	wpchg_info("check batt_mv=%d\n",batt_uv);
	if( batt_uv > WP_MIN_CHG_VOL ){
		wpchg_info("start enable wp chg\n");
		wp_charging_enable_cntl(wp_chip, WP_CHG_ENABLE);
		wake_unlock(&wp_chip->check_lock);
		return;
	}

	schedule_delayed_work(&wp_chip->wp_switch_check_work, msecs_to_jiffies(DELAY_TIME_MS));
	return;
}

int wp_charger_insert_check(void)
{
	int rc = 0;
	int batt_uv;

	if(!wchip){
		pr_err("wp do not init\n");
		return -1;
	}
	
	rc = wp_get_batt_prop(wchip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &batt_uv);
	if(rc){
		batt_uv = 0;
	}

	wpchg_info("batt_mv=%d\n",batt_uv);
	if(batt_uv > WP_MIN_CHG_VOL){
		wpchg_info("start enable wp chg\n");
		wp_charging_enable_cntl(wchip, WP_CHG_ENABLE);
	}else{
		wpchg_info("start check batt vol\n");
		wake_lock(&wchip->check_lock);
		cancel_delayed_work_sync(&wchip->wp_switch_check_work);
		schedule_delayed_work(&wchip->wp_switch_check_work, msecs_to_jiffies(DELAY_TIME_MS));
	}

	return 0;
}

static int weipu_en;
static int weipu_charging_enable_set(const char *val, struct kernel_param *kp)
{
	int rc;
	
	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("error setting value %d\n", rc);
		return rc;
	}

	if(!wchip)
		return -1;
	
	wpchg_info("user set weipu_en=%d\n", weipu_en);
	wp_charging_enable_cntl(wchip, weipu_en);
	return 0;
}
module_param_call(weipu_en, weipu_charging_enable_set, param_get_uint,
					      &weipu_en, 0644);

static void weipu_chg_timeout_worker(struct work_struct *work)
{
	struct weipu_plat *wp_chip = container_of(work, struct weipu_plat, timeout_work);
	int wp_st = gpio_get_value(wp_chip->weipu_st_gpio);

	if(wp_st){
		weipu_chg_vote(wp_chip, WP_CHG_TOUT, WP_VOTE_MASK_TOUT);
		wpchg_info("wp max chg timer out! switch to pmi chg\n");
	}

	return;
}

static enum hrtimer_restart wp_hrtimer_func(struct hrtimer *hrtimer)
{
	struct weipu_plat *wp_chip = container_of(hrtimer, struct weipu_plat, wp_hrtimer);

	schedule_work(&wp_chip->timeout_work);
    return HRTIMER_NORESTART;
}

static irqreturn_t weipu_chg_handler(int irq, void *_chip)
{
    struct weipu_plat *wp_chip = _chip;
	int vbus_mv;

	wp_chip->weipu_chg_st = gpio_get_value(wp_chip->weipu_st_gpio);
	vbus_mv = get_usbin_voltage_mv(wp_chip);

	wpchg_info("-->en=%d st=%d[%s] vbus=%d\n",
		wp_chip->en_st,
		wp_chip->weipu_chg_st, 
		wp_chip->weipu_chg_st? "Charging" : "Discharging",
		vbus_mv);

	if( wp_chip->weipu_chg_st ){
		wake_lock(&wp_chip->wlock);

		wp_chip->chg_cfg = CHG_CFG_WP;
		pmi_charging_enable(wp_chip, WP_CHG_DISABLE);
		
		set_wp_chg_present(wp_chip, WP_CHG_PRESENT);
		wp_set_usb_prop(wp_chip, POWER_SUPPLY_PROP_TYPE, POWER_SUPPLY_TYPE_USB_HVDCP_3);
		wp_set_usb_prop(wp_chip, POWER_SUPPLY_PROP_ONLINE, 1);

		//start chg timeout check
		hrtimer_start(&wp_chip->wp_hrtimer, ktime_set(WP_MAX_CHG_SEC,0), HRTIMER_MODE_REL);
		wp_update_power_changed(wp_chip);
	}else{
		if(wp_chip->en_st){
			wpchg_info("Weipu chg is removed\n");
			set_wp_chg_present(wp_chip, WP_CHG_REMOVE);
			wp_notify_usb_status(wp_chip, USB_REMOVE_EVENT);
		}else{
			msleep(USB_REMOVE_CHECK_MS);
			vbus_mv = get_usbin_voltage_mv(wp_chip);
			if(vbus_mv > USB_VALID_MIN_MV){
				wpchg_info("Weipu swicth to Pmi chg, vbus_mv=%d\n",vbus_mv);
				set_wp_chg_present(wp_chip, WP_CHG_PRESENT);
			}else{
				wpchg_info("Weipu chg is removed by recheck, vbus_mv=%d\n",vbus_mv);
				set_wp_chg_present(wp_chip, WP_CHG_REMOVE);
				wp_notify_usb_status(wp_chip, USB_REMOVE_EVENT);
			}
		}
		pmi_charging_enable(wp_chip, WP_CHG_ENABLE);
		hrtimer_cancel(&wp_chip->wp_hrtimer);
		wp_chip->current_ma = 0;
		wp_chip->check_cnt = 0;
		wp_chip->pre_current = 0;
	}

	if(!is_wp_chg_present()){
		wp_charging_enable_cntl(wp_chip, WP_CHG_DISABLE);
	}

	return IRQ_HANDLED;
}

static int wp_chg_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{	
    struct weipu_plat *wp_chip = container_of(psy, struct weipu_plat, wp_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if( weipu_is_charging_status() )
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_wp_chg_present();
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = wp_chip->en_st;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int wp_chg_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
    //struct weipu_plat *wp_chip = container_of(psy, struct weipu_plat, wp_psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		//wp_set_current(wp_chip, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int wp_chg_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int wp_parse_dt(struct weipu_plat *wp_chip)
{
	int rc = 0;
	struct device_node *dev_node = wp_chip->dev->of_node;
	
	wp_chip->weipu_st_gpio = of_get_named_gpio(dev_node, "weipu-st-gpio", 0);
	wp_chip->weipu_en_gpio = of_get_named_gpio(dev_node, "weipu-en-gpio", 0);	

    of_property_read_u32(dev_node,
						 "weipu,batt-cool-temp",
						 &wp_chip->batt_cool_temp);
	
	of_property_read_u32(dev_node,
						 "weipu,batt-warm-temp",
						 &wp_chip->batt_warm_temp);

	of_property_read_u32(dev_node,
						 "weipu,chg-limit-temp",
						 &wp_chip->chg_limit_temp);

	of_property_read_u32(dev_node,
						 "weipu,batt-cv-soc",
						 &wp_chip->batt_cv_soc);

	of_property_read_u32(dev_node,
						 "weipu,usb-hot-temp",
						 &wp_chip->usb_hot_temp);

	of_property_read_u32(dev_node,
						 "weipu,max-chg-ma",
						 &wp_chip->max_chg_ma);

	wpchg_info("batt_cool=%d batt_warm=%d limit_temp=%d cv_soc=%d usb_hot=%d max_ma=%d\n",
		wp_chip->batt_cool_temp, wp_chip->batt_warm_temp, wp_chip->chg_limit_temp,
		wp_chip->batt_cv_soc,wp_chip->usb_hot_temp,wp_chip->max_chg_ma);

	return rc;
}

static int wp_chg_probe(struct platform_device *pdev)
{	
    int rc;
	struct weipu_plat *wp_chip;
	
	wpchg_info("wp chg enter\n");	
   
	wp_chip = devm_kzalloc(&pdev->dev, sizeof(*wp_chip), GFP_KERNEL);
	if (!wp_chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	wp_chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, wp_chip);
	INIT_DELAYED_WORK(&wp_chip->wp_chg_work, wp_chg_worker);
	INIT_DELAYED_WORK(&wp_chip->wp_fake_chg_work, wp_fake_chg_worker);
	INIT_DELAYED_WORK(&wp_chip->usb_rm_check_work, usb_rm_check_worker);
	INIT_DELAYED_WORK(&wp_chip->wp_switch_check_work, wp_switch_check_worker);
	INIT_WORK(&wp_chip->timeout_work, weipu_chg_timeout_worker);
	wake_lock_init(&wp_chip->wlock, WAKE_LOCK_SUSPEND, "wp-chg");
	wake_lock_init(&wp_chip->check_lock, WAKE_LOCK_SUSPEND, "wp-switch-chg");
	mutex_init(&wp_chip->en_cntl_lock);
	mutex_init(&wp_chip->current_lock);
	spin_lock_init(&wp_chip->mask_lock);
    hrtimer_init(&wp_chip->wp_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    wp_chip->wp_hrtimer.function = wp_hrtimer_func;
	wp_chip->chg_cfg = CHG_CFG_INVALID;
	wp_chip->vote_mask = 0;

	rc = wp_parse_dt(wp_chip);
	if(rc < 0)
		goto init_fail;

	rc = weipu_pinctrl_init(wp_chip);
	if(rc)
		dev_err(&pdev->dev, "Fail to init pinctrl\n");
	
	rc = weipu_gpio_enable(wp_chip);
	if(rc){
		dev_err(&pdev->dev, "Fail to init pinctrl\n");
		goto init_fail;
	}

	wp_chip->ref_vdd = devm_regulator_get(wp_chip->dev, "usb-ntc");
	if (IS_ERR(wp_chip->ref_vdd)) {
		pr_err("Unable to get vbus_otg\n");
	}
	if(wp_chip->ref_vdd){
		rc = regulator_set_voltage(wp_chip->ref_vdd, REF_MIN_UV, REF_MAX_UV);
		if (rc) 
			pr_err("Regulator set_vtg failed vdd rc=%d\n",	rc);
	}

	wp_chip->weipu_st_irq = gpio_to_irq(wp_chip->weipu_st_gpio);
	rc = request_threaded_irq(wp_chip->weipu_st_irq, 
		NULL, 
		weipu_chg_handler, 
		(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT), 
		"weipu-chg-irq",	
		wp_chip);
	if (rc < 0){						
		dev_err(wp_chip->dev, "Unable to request weipu-chg-irq irq: %d\n",rc);
		goto init_fail;
	}
	enable_irq_wake(wp_chip->weipu_st_irq);

	wp_chip->wp_psy.name		= "wp_chg";
	wp_chip->wp_psy.type		= POWER_SUPPLY_TYPE_USB_HVDCP_3;
	wp_chip->wp_psy.supplied_to = wp_chg_power_supplied_to;
	wp_chip->wp_psy.get_property	= wp_chg_get_property;
	wp_chip->wp_psy.set_property	= wp_chg_set_property;
	wp_chip->wp_psy.properties	= wp_chg_properties;
	wp_chip->wp_psy.num_properties	= ARRAY_SIZE(wp_chg_properties);
	wp_chip->wp_psy.property_is_writeable = wp_chg_is_writeable;

	rc = power_supply_register(wp_chip->dev, &wp_chip->wp_psy);
	if (rc < 0) {
		pr_err("Unable to register wp_psy rc = %d\n", rc);
		goto init_fail;
	}

    wchip = wp_chip;
	wpchg_info("wp chg probe successfully.\n");	
	
	return 0;
	
init_fail:
	return rc;
}

static int wp_chg_remove(struct platform_device *pdev)
{	
	struct weipu_plat *wp_chip = platform_get_drvdata(pdev);
	
	wpchg_info("wp chg remove \n");
	
	if(gpio_is_valid(wp_chip->weipu_en_gpio))
		gpio_free(wp_chip->weipu_en_gpio);

	if(gpio_is_valid(wp_chip->weipu_st_gpio))
		gpio_free(wp_chip->weipu_st_gpio);

	kfree(wp_chip);
	return 0;
}

static struct of_device_id wp_dt_match[] = {
	{
		.compatible     = "weipu,charger",
	},
	{ },
};

static struct platform_driver wp_chg_driver = {	
	.probe	= wp_chg_probe,	
	.remove	= wp_chg_remove,	
	.driver	= {		
		.name	= "wp_chg_driver",		
		.owner	= THIS_MODULE,		
		.of_match_table = wp_dt_match,	
	},
};  

static int __init wp_chg_init(void) 
{    
	int ret; 	
		
	ret = platform_driver_register(&wp_chg_driver);	
	return ret; 
}

static void __exit wp_chg_exit(void)
{
    printk( "%s:exit...\n", __func__);	
	return;
}

module_init(wp_chg_init);

module_exit(wp_chg_exit);

MODULE_AUTHOR("ztemt-wang.shuai");
MODULE_DESCRIPTION("ztemt weipu charger driver");
MODULE_LICENSE("GPL");

