/*
 * Nubia TouchPad driver common file
 *
 * Copyright (C) 2016 Nubia Co.Ltd
 *
 * Author: Irven <ai.haifeng9@zte.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include "nubia_tp_cmn.h"


extern void nubia_tp_resume_work_func(struct work_struct *work);

/*
void nubia_input_palm_parse(device_node *np )
{
}
*/

// donnot using device tree to parse key, we define it directly
void nubia_input_set_palm_cap( struct input_dev * input_dev )
{
    set_bit(KEY_PALM_LOCK, input_dev->keybit);
    input_set_capability(input_dev, EV_KEY, KEY_PALM_LOCK);
 }

void nubia_input_report_palm( struct input_dev * input_dev )
{
    input_report_key(input_dev, KEY_PALM_LOCK, 1);
    input_sync(input_dev);
    input_report_key(input_dev, KEY_PALM_LOCK, 0);
    input_sync(input_dev);
}

static void nubia_tp_resume_work_func_hook(struct work_struct *work)
{
    nubia_tp_resume_work_func(work);
}

void nubia_tp_resume_work_init(struct work_struct *work)
{
    INIT_WORK(work, nubia_tp_resume_work_func_hook);
}
void nubia_tp_resume_work_deinit(struct work_struct *work)
{
    cancel_work_sync(work);
}
