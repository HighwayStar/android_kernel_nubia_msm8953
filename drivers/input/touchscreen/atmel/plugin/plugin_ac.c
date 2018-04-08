/*
 * Atmel maXTouch Touchscreen driver Plug in
 *
 * Copyright (C) 2013 Atmel Co.Ltd
 *
 * Author: Pitter Liao <pitter.liao@atmel.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/****************************************************************  
	Pitter Liao add for macro for the global platform
		email:  pitter.liao@atmel.com 
		mobile: 13244776877
-----------------------------------------------------------------*/
#define PLUG_AC_VERSION 0x0001
/*----------------------------------------------------------------

*/

//first version for AC Plug: just watch T72 status

#include "plug.h"

#define AC_FLAG_RESETING				P_FLAG_EV_RESETING
#define AC_FLAG_CALING					P_FLAG_EV_CALING
#define AC_FLAG_WORKAROUND_HALT			P_FLAG_EV_HALT

#define AC_FLAG_RESET					P_FLAG_EV_RESET
#define AC_FLAG_CAL						P_FLAG_EV_CAL
#define AC_FLAG_RESUME					P_FLAG_EV_RESUME

#define AC_FLAG_OLD_STATE_VERY_NOISE			(1<<8)
#define AC_FLAG_EXT_EVENT_HANDLED			(1<<9)

#define AC_FLAG_STABLE					(1<<16)
#define AC_FLAG_NOISE					(1<<17)
#define AC_FLAG_VERY_NOISE				(1<<18)
#define AC_FLAG_STATE_CHANGE			(1<<19)

#define AC_FLAG_EVENT_SHIFT			(20)
#define AC_FLAG_EVENT_AC_REMOVED		(1<<20)
#define AC_FLAG_EVENT_AC_DETECTED		(1<<21)

#define AC_FLAG_FUNC_SHIFT			(24)
#define AC_FLAG_FUNC_VERY_NOISE_EXIT_RECAL		(1<<24)
#define AC_FLAG_FUNC_EXTERN_FUNCTION			(1<<25)

#define AC_FLAG_MASK_LOW			P_FLAG_EV_DONE_MASK
#define AC_FLAG_MASK_NORMAL			(0x0ff00)
#define AC_STATE_MASK				(0x00f0000)
#define AC_EXT_EVENT_MASK			(0x0f00000)
#define AC_FLAG_MASK_FUNC			(0xf000000)
#define AC_FLAG_MASK				(-1)

struct ac_observer{
	unsigned long flag;
	unsigned long touch_id_list;
	unsigned long ticks_last_vnoise;
};

/* T72 */
struct t72_config {
	u8 vnoilownlthr;
	u8 rsv;
	u8 vnoicnt;
}__packed;

enum{
	INTERVAL_VNOISE_EXIT_RECAL = 0,
	NUM_AC_INTERVAL,
};

struct ac_config{
	struct t72_config t72;
	unsigned long interval[NUM_AC_INTERVAL];
};

static void plugin_ac_hook_t6(struct plugin_ac *p, u8 status)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct ac_observer *obs = p->obs;

	plugin_p_hook_t6(status, &obs->flag,AC_FLAG_MASK_NORMAL);

	dev_dbg(dev, "mxt ac flag=0x%lx %x\n",
		 obs->flag, status);
}

static int plugin_ac_hook_t9_t100(struct plugin_ac *p, int id, int x, int y, struct ext_info *in)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct ac_observer *obs = p->obs;

	if (!test_flag(AC_FLAG_MASK_FUNC, &obs->flag))
		return 0;

	if (id >= MAX_TRACE_POINTS)
		return 0;

	if (in->status & MXT_T9_T100_DETECT)
		set_bit(id, &obs->touch_id_list);
	else
		clear_bit(id, &obs->touch_id_list);

	dev_dbg(dev,  "[mxt] tch status %x\n",in->status);

	return 0;
}

static void plugin_ac_hook_t72(struct plugin_ac *p, u8 *msg)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;

	struct ac_observer *obs = (struct ac_observer *)p->obs;
	int state,dualx;
	
	state = msg[2] & T72_NOISE_STATE_MASK;
	dualx = msg[2] & T72_NOISE_DUALX_MASK;

	dev_dbg(dev, "mxt hook ac %d(%d,%d,%d)\n",
		state,NOISE_STABLE,NOISE_NOISY,NOISE_VERY_NOISY);

	if (state == NOISE_STABLE) {
		if (!test_flag(AC_FLAG_STABLE, &obs->flag)) {
			dev_info(dev, "mxt stable state\n");
			set_and_clr_flag(AC_FLAG_STABLE | AC_FLAG_STATE_CHANGE, AC_STATE_MASK, &obs->flag);
			p->set_and_clr_flag(p->dev, 0, PL_STATUS_FLAG_NOISE_MASK);
		}
	}else if (state == NOISE_NOISY) {
		if (!test_flag(AC_FLAG_NOISE, &obs->flag)) {
			dev_info(dev, "mxt noise state\n");
			set_and_clr_flag(AC_FLAG_NOISE | AC_FLAG_STATE_CHANGE, AC_STATE_MASK, &obs->flag);
			p->set_and_clr_flag(p->dev, PL_STATUS_FLAG_NOISE, PL_STATUS_FLAG_NOISE_MASK);
		}
	}else if (state == NOISE_VERY_NOISY) {
		if (!test_flag(AC_FLAG_VERY_NOISE, &obs->flag)) {
			dev_info(dev, "mxt very noise state\n");
			set_and_clr_flag(AC_FLAG_VERY_NOISE | AC_FLAG_STATE_CHANGE, AC_STATE_MASK, &obs->flag);
			p->set_and_clr_flag(p->dev, PL_STATUS_FLAG_VERY_NOISE, PL_STATUS_FLAG_NOISE_MASK);
		}
	}else{
		dev_info(dev, "mxt hook ac unknow status %d\n",state);
	}

	if (dualx) {
		dev_dbg(dev, "mxt hook ac dualx %d\n",dualx);
		p->set_and_clr_flag(p->dev, PL_STATUS_FLAG_DUALX, 0);
	}else{
		p->set_and_clr_flag(p->dev, 0, PL_STATUS_FLAG_DUALX);
	}
}

static void plugin_ac_reset_slots_hook(struct plugin_ac *p)
{
	struct ac_observer *obs = p->obs;

	memset(&obs->touch_id_list, 0, sizeof(*obs) - offsetof(struct ac_observer,touch_id_list));  //clear data after touch_id_list
}

static long mxt_proc_noise_msg(struct plugin_ac *p,unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct ac_observer *obs = p->obs;
	struct ac_config *cfg = p->cfg;
	long interval = MAX_SCHEDULE_TIMEOUT;	
	unsigned long ticks = jiffies;

	int handled = 1;

	dev_dbg(dev, "mxt ac at mxt_proc_noise_msg flag 0x%lx pl_flag 0x%lx\n",obs->flag,pl_flag);
	
	//very noise
	if (test_flag(AC_FLAG_VERY_NOISE,&obs->flag)) {
		dev_dbg(dev, "mxt ac enter very noise state\n");
		set_flag(AC_FLAG_OLD_STATE_VERY_NOISE, &obs->flag);
		obs->ticks_last_vnoise = jiffies;
	//noise / stable
	}else {
		dev_dbg(dev, "mxt ac enter %s state\n", test_flag(AC_FLAG_NOISE,&obs->flag) ? "noise" : "stable");	
		if (!obs->touch_id_list || 
			time_after_eq(ticks, obs->ticks_last_vnoise + cfg->interval[INTERVAL_VNOISE_EXIT_RECAL])) {
			if (test_and_clear_flag(AC_FLAG_OLD_STATE_VERY_NOISE,&obs->flag)) {
				dev_info(dev, "mxt old very noise to stable set cal\n");
				p->set_t6_cal(p->dev);
			}
		}else {
			interval = obs->ticks_last_vnoise + cfg->interval[INTERVAL_VNOISE_EXIT_RECAL] - ticks;
			handled = 0;
		}
	}

	if (handled)
		clear_flag(AC_FLAG_STATE_CHANGE, &obs->flag);

	p->set_and_clr_flag(p->dev, PL_STATUS_FLAG_NOISE_CHANGE, 0);

	return (long)interval;
}

static int mxt_proc_ext_event_msg(struct plugin_ac *p, unsigned long flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct ac_config *cfg = p->cfg;
	struct ac_observer *obs = p->obs;
	int set = 1;

	struct reg_config t72_reg =
		{.reg = MXT_PROCG_NOISESUPPRESSION_T72, .instance = 0,
			.offset = 77,.buf = {3, 0, 0xff}, .len = 3, .mask = 0, .flag = 0, .sleep = 0};

	if (test_flag(AC_FLAG_EVENT_AC_DETECTED, &flag)) {
		dev_info(dev, "[mxt] ext event ac set t72 %d %x\n", t72_reg.buf[0],  t72_reg.buf[2]);

		t72_reg.buf[0] = 3;
		t72_reg.buf[1] = 0;
		t72_reg.buf[2] = 0xff;
		set = 1;
	} else {
		if (!test_flag(AC_FLAG_RESET, &obs->flag)) {
			dev_info(dev, "[mxt] ext event clr t72 %d %x\n", t72_reg.buf[0], t72_reg.buf[2]);
			memcpy(t72_reg.buf, &cfg->t72, sizeof(cfg->t72));

			set = 0;
		}
	}

	if (set)
		return p->set_obj_cfg(p->dev, &t72_reg, NULL, 0);

	return 0;
}

static void plugin_ac_start(struct plugin_ac *p, bool resume)
{
	struct ac_observer *obs = p->obs;

	clear_flag(AC_FLAG_WORKAROUND_HALT, &obs->flag);

	if (resume)
		set_flag(AC_FLAG_RESUME, &obs->flag);
}

static void plugin_ac_stop(struct plugin_ac *p)
{
	struct ac_observer *obs = p->obs;

	set_and_clr_flag(AC_FLAG_WORKAROUND_HALT,AC_FLAG_RESUME, &obs->flag);
}

static int get_extern_ac_status(void)
{
	return -EIO;
}

static void plugin_ac_pre_process_messages(struct plugin_ac *p, unsigned long pl_flag)
{
	struct ac_observer *obs = p->obs;
	int det = get_extern_ac_status();

	if (det == 1) {
		if (!test_flag(AC_FLAG_EVENT_AC_DETECTED, &obs->flag))
			set_and_clr_flag(AC_FLAG_EVENT_AC_DETECTED , AC_FLAG_EXT_EVENT_HANDLED | AC_EXT_EVENT_MASK, &obs->flag);
	}else if (det == 0) {
		if (!test_flag(AC_FLAG_EVENT_AC_REMOVED, &obs->flag))
			set_and_clr_flag(AC_FLAG_EVENT_AC_REMOVED , AC_FLAG_EXT_EVENT_HANDLED | AC_EXT_EVENT_MASK, &obs->flag);
	}else {
		// unused
	}
}

static long plugin_ac_post_process_messages(struct plugin_ac *p, unsigned long pl_flag)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct ac_observer *obs = p->obs;	
	struct device *dev = dcfg->dev;
	long interval = MAX_SCHEDULE_TIMEOUT;

	dev_info(dev, "[mxt]status: Flag=0x%08lx\n",
		obs->flag);

	if (test_flag(AC_FLAG_WORKAROUND_HALT, &obs->flag))
		return interval;

	if (test_flag(AC_FLAG_RESETING|AC_FLAG_CALING, &obs->flag))
		return interval;

	if (test_flag(AC_FLAG_FUNC_EXTERN_FUNCTION, &obs->flag)) {
		if (test_flag(AC_EXT_EVENT_MASK, &obs->flag)) {
			if (!test_and_set_flag(AC_FLAG_EXT_EVENT_HANDLED, &obs->flag)) {
				mxt_proc_ext_event_msg(p, obs->flag);
			}
		}
	}

	if (test_flag(AC_FLAG_FUNC_VERY_NOISE_EXIT_RECAL, &obs->flag)) {
		if (test_flag(AC_FLAG_STATE_CHANGE, &obs->flag)) {
			interval = mxt_proc_noise_msg(p, pl_flag);
		}
	}
	
	clear_flag(AC_FLAG_MASK_LOW,&obs->flag);

	return interval;
}

static int plugin_ac_show(struct plugin_ac *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;
	struct ac_config *cfg = p->cfg;
	struct ac_observer * obs = p->obs;
	int i;

	dev_info(dev, "[mxt]PLUG_AC_VERSION: 0x%x\n",PLUG_AC_VERSION);
	
	if (!p->init)
		return 0;

	dev_info(dev, "[mxt]AC cfg :\n");
	dev_info(dev, "[mxt]\n");

	dev_info(dev, "[mxt]AC obs :\n");
	dev_info(dev, "[mxt]status: Flag=0x%08lx\n",
		obs->flag);

	for (i = 0; i < NUM_AC_INTERVAL; i++)
		dev_info(dev, "ac interval[%d] = %ld\n", i, cfg->interval[i]);
	dev_info(dev, "[mxt]\n");

	return 0;
}

static int plugin_ac_store(struct plugin_ac *p, const char *buf, size_t count)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;	
	struct ac_config *cfg = p->cfg;
	struct ac_observer * obs = p->obs;
	int config[2];
	char name[255];

	dev_info(dev, "[mxt]ac store:%s\n",buf);

	if (!p->init)
		return 0;

	if (sscanf(buf, "status: Flag=0x%lx\n",
		&obs->flag) > 0) {
		dev_info(dev, "[mxt] OK\n");
	}else if (sscanf(buf, "enable %x\n",
		&config[0]) > 0) {
		config[0] <<= AC_FLAG_FUNC_SHIFT;
		config[0] &= AC_FLAG_MASK_FUNC;
		set_and_clr_flag(config[0], AC_FLAG_MASK_FUNC, &obs->flag);
		dev_info(dev, "[mxt] set func %x\n", config[0]);
	}else {
		if (count > 4 && count < sizeof(name)) {
			if (sscanf(buf, "interval[%d]: %d", &config[0],&config[1]) == 2) {
				if (config[0] < NUM_AC_INTERVAL)
					cfg->interval[config[0]] = config[1];
			}else {
				dev_err(dev, "Unknow ac command: %s\n",buf);
			}
		}
	}
	
	return 0;
}

ssize_t plugin_ac_extern_event_show(struct plugin_ac *p, char *buf, size_t count)
{
	struct ac_observer *obs = p->obs;
	ssize_t offset = 0;

	if (!p->init)
		return 0;

	offset += scnprintf(buf + offset, count - offset, "%ld %s\n",
		(obs->flag & AC_EXT_EVENT_MASK) >> AC_FLAG_EVENT_SHIFT,
		(obs->flag & AC_FLAG_EXT_EVENT_HANDLED) ? "done" : "");

	return offset;
}

int plugin_ac_extern_event_store(struct plugin_ac *p, const char *buf, size_t count)
{
	struct ac_observer *obs = p->obs;
	int config;

	if (!p->init)
		return -EIO;

	if (count) {
		config = buf[0];
		config <<= AC_FLAG_EVENT_SHIFT;
		config &= AC_EXT_EVENT_MASK;

		set_and_clr_flag(config, AC_EXT_EVENT_MASK | AC_FLAG_EXT_EVENT_HANDLED, &obs->flag);
	}
	
	return count;
}

static int init_ac_object(struct plugin_ac *p)
{ 
	struct ac_config *cfg = p->cfg;

	struct reg_config t72_reg =
		{.reg = MXT_PROCG_NOISESUPPRESSION_T72, .instance = 0,
			.offset = 77,.buf = {0}, .len = 3, .mask = 0, .flag = 0, .sleep = 0, .ext_buf = (u8 *)&cfg->t72};

	cfg->interval[INTERVAL_VNOISE_EXIT_RECAL] = 3 * HZ;

	return p->get_obj_cfg(p->dev, &t72_reg, FLAG_REG_DATA_IN_EXT_BUF);
}

static int deinit_ac_object(struct plugin_ac *p)
{
	return 0;
}

static int plugin_ac_init(struct plugin_ac *p)
{
	const struct mxt_config *dcfg = p->dcfg;
	struct device *dev = dcfg->dev;

	dev_info(dev, "%s: plugin ac version 0x%x\n", 
			__func__,PLUG_AC_VERSION);

	p->obs = kzalloc(sizeof(struct ac_observer), GFP_KERNEL);
	if (!p->obs) {
		dev_err(dev, "Failed to allocate memory for ac observer\n");
		return -ENOMEM;
	}

	p->cfg = kzalloc(sizeof(struct ac_config), GFP_KERNEL);
	if (!p->cfg) {
		dev_err(dev, "Failed to allocate memory for ac cfg\n");
		kfree(p->obs);
		p->obs =NULL;
		return -ENOMEM;
	}

	if (init_ac_object(p) != 0) {
		dev_err(dev, "Failed to allocate memory for ac cfg\n");
		kfree(p->obs);
		p->obs = NULL;
		kfree(p->cfg);
		p->cfg = NULL;
	}
	
	return  0;
}

static void plugin_ac_deinit(struct plugin_ac *p)
{
	if (p->obs) {
		deinit_ac_object(p);
		kfree(p->obs);
	}
	if (p->cfg)
		kfree(p->cfg);
}

static struct plugin_ac mxt_plugin_ac_if = 
{
	.init = plugin_ac_init,
	.deinit = plugin_ac_deinit,
	.start = plugin_ac_start,
	.stop = plugin_ac_stop,
	.hook_t6 = plugin_ac_hook_t6,
	.hook_t100 = plugin_ac_hook_t9_t100,
	.hook_t72 = plugin_ac_hook_t72,
	.pre_process = plugin_ac_pre_process_messages,
	.post_process = plugin_ac_post_process_messages,
	.hook_reset_slots = plugin_ac_reset_slots_hook,
	.show = plugin_ac_show,
	.store = plugin_ac_store,
};

int plugin_interface_ac_init(struct plugin_ac *p)
{
	memcpy(p, &mxt_plugin_ac_if, sizeof(struct plugin_ac));

	return 0;
}

void plugin_interface_ac_deinit(struct plugin_ac *p)
{

}

