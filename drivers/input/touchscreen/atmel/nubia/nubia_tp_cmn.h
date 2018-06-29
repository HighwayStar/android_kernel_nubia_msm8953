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
 */

#ifndef __LINUX_NUBIA_TP_CMN_H
#define __LINUX_NUBIA_TP_CMN_H

#include <linux/input/mt.h>
#include <linux/module.h>

#define CONFIG_ZTEMT_TOUCHSCREEN_SYSFS
#define NUBIA_DEFERRED_RESUME_TP
#define NUBIA_REPORT_F9_AS_PALM
#define NUBIA_PALM_KEY_CAN_DISABLE

#if defined (NUBIA_DEFERRED_RESUME_TP)
extern void nubia_tp_resume_work_init(struct work_struct *work);
extern void nubia_tp_resume_work_deinit(struct work_struct *work);
#endif

#endif /* __LINUX_NUBIA_TP_CMN_H */
