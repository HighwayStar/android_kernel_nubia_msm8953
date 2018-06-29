/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
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
#define DRIVER_VERSION 0xB509
/*----------------------------------------------------------------
0.50:
1 support selfcap recal workaround
0.48
1 fixed T7 value still old after updating
2 fixed depth and irq mismatch in thread
3 support wdc plugin
4 support fw postfix

0.47
1 fixed read/write len limit of 255
2 cmd interface
3 T81 msg bugs
4 T100 ext/scr message parse
5 Palm detect message

0.44
1 support gesture object
2 support irq nested(but not working perfect)
3 fixed bug in object table and msg_bufs alloc
4 use key array instead of key list
5 use each object indivadual write when config update
6 support family id check when firmware update
0.42
1 add self tune support in T102
2 change MXT_MAX_BLOCK_WRITE to 128
3 change T24/T61/T92/T93 wakeup support algorithm
4 change bootloader DMA address
5 delete pm supend if earlysuspend or fb call back support

0.41
1 fixed bugs at alternative config
0.40
1 add fb callback
2 and alternative chip support
0.39
1 t65 support
0.386:
1 fixed bug don't report message when enable_report is disable, which cause crash in update cfg
2 delete call back in pluging force stop
0.385
0.384
1 compatible bootloader mode
0.383
1 t80 support
0.382
1 fixed bugs for deinit plugin at update cfg
0.381
1 add mxt_plugin_hook_reg_init support(support t38 protect)
0.38
1 fixed bug at deinit
0.37
1 modify plug resume order, independ the check and calibration function
1 support no key project
0.343
0.342:
1 fixed t15 report virtual key

0.34
1 add t15 individual key to virtual key
2 config failed contiue to work

0.33
1 add t55 address
2 do calibration when crc match, this will useful for stable POR calibration
3 add update fw name check
4 add suspend/resume input_dev check, this is for bootup bootloader mode

0.32
1 add t6 address
2 modify kthread exit contidion
0.31
1 version for simple workaround without debug message
0.3
1 Add plugin support
0.2
1 Add early suspend/resume
2 Add MTK supprot without test
*/
#include "include/config.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/kthread.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include "plugin/plug.h"
#include "io/io.h"

#if defined(CONFIG_MXT_I2C_DMA)
#include <linux/dma-mapping.h>
#endif

#if defined(CONFIG_ZTEMT_TOUCHSCREEN_ATMEL_MXTS)
#include "nubia/nubia_tp_cmn.h"
#endif

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
//Block size (should more than T5 size(10), less than 255)
#define MXT_MAX_BLOCK_READ	250
#define MXT_MAX_BLOCK_WRITE	128
#define MXT_DMA_BUFFER_SIZE PAGE_SIZE		//must algn for DMA use

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
//#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN	  (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1
#define MXT_TOUCH_MAJOR_T15_VIRTUAL_KEY 2

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS	(1 << 0)
#define MXT_T63_STYLUS_RELEASE	(1 << 1)
#define MXT_T63_STYLUS_MOVE		(1 << 2)
#define MXT_T63_STYLUS_SUPPRESS	(1 << 3)

#define MXT_T63_STYLUS_DETECT	(1 << 4)
#define MXT_T63_STYLUS_TIP		(1 << 5)
#define MXT_T63_STYLUS_ERASER	(1 << 6)
#define MXT_T63_STYLUS_BARREL	(1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK	0x3F

/* Define for NOISE SUPPRESSION T72 */
#define MXT_T72_NOISE_SUPPRESSION_NOISELVCHG	(1 << 4)

/* T100 Multiple Touch Touchscreen */

//#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_SCRAUX		2
#define MXT_T100_TCHAUX		3
#define MXT_T100_XSIZE		9
#define MXT_T100_XPITCH		10
#define MXT_T100_XRANGE		13
#define MXT_T100_YSIZE		20
#define MXT_T100_YPITCH		21
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_SCRAUX_NUMTCH	(1 << 0)
#define MXT_T100_SCRAUX_TCHAREA	(1 << 1)
#define MXT_T100_SCRAUX_ATCHAREA	(1 << 2)
#define MXT_T100_SCRAUX_INTTCHAREA		(1 << 3)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)
#define MXT_T100_TCHAUX_HW		(1 << 3)
#define MXT_T100_TCHAUX_PEAK	(1 << 4)
#define MXT_T100_TCHAUX_AREAHW	(1 << 5)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_STYLUS	0x20

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		200	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

#ifndef INIT_COMPLETION
#define INIT_COMPLETION(x)	((x).done = 0)
#endif

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int max_x;
	unsigned int max_y;

#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	unsigned int max_y_t;	//touch max y
#endif
	bool in_bootloader;
	u16 mem_size;

	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
#if defined(CONFIG_MXT_I2C_DMA)
	unsigned short bootloader_addr;
	void *i2c_dma_va;
	dma_addr_t i2c_dma_pa;
	size_t dma_buffer_size;		//W/R use half individul
#else
	u8 bootloader_addr;
#endif
	struct mutex bus_access_mutex;

	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	u8 tchcfg[4];

	char suffix_pid_name[16];
	char *fw_name;
	char *cfg_name;
#define ALT_CHIP_BIT_FW  (1<<8)
	unsigned long alt_chip;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u16 T9_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T15_address;
	u16 T18_address;
	u16 T19_address;
	u8 T19_reportid;
	u8  t19_msg[1];
	u16 T24_address;
	u8 T24_reportid;
	u16 T25_address;
	u8  T25_reportid;
	u8  t25_msg[6];
	u16 T37_address;
	u16 T38_address;
	u16 T40_address;
	u16 T42_address;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u16 T46_address;
	u16 T47_address;
	u8 T48_reportid;
	u16 T55_address;
	u16 T56_address;
	u8 T61_reportid_min;
	u8 T61_reportid_max;
	u16 T61_address;
	u16 T61_instances;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T65_address;
	u16 T68_address;
	u8 T68_reportid_min;
	u8 T68_reportid_max;
	u16 T71_address;
	u16 T72_address;
	u8 T72_reportid_min;
	u8 T72_reportid_max;
	u16 T78_address;
	u16 T80_address;
	u16 T81_address;
	u8 T81_reportid_min;
	u8 T81_reportid_max;
	u16 T92_address;
	u8 T92_reportid;
	u16 T93_address;
	u8 T93_reportid;
	u8 T97_reportid_min;
	u8 T97_reportid_max;
	u16 T96_address;
	u16 T97_address;
	u16 T99_address;
	u8 T99_reportid;
	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T102_address;
	u8  T102_reportid;
	u16 T104_address;
	u16 T113_address;
	u16 T115_address;
	u8 T115_reportid;

	/* for reset handling */
	struct completion reset_completion;

	/* for reset handling */
	struct completion crc_completion;

	/* Enable reporting of input events */
	bool enable_reporting;

#define MXT_WK_ENABLE 1
#define MXT_WK_DETECTED 2
	unsigned long enable_wakeup;

	/* Indicates whether device is in suspend */
	bool suspended;

	struct mutex access_mutex;

#if defined(CONFIG_MXT_FW_UPDATE_EXAMPLE_BY_KTHREAD)
	struct task_struct *tsk_handle_update;
#endif
#if defined(CONFIG_MXT_IRQ_WORKQUEUE)
	struct task_struct *tsk_handle_msg;
	wait_queue_head_t wait;
#endif
	unsigned long busy;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;

#if defined(NUBIA_DEFERRED_RESUME_TP)
       struct work_struct nubia_tp_resume_work;
#endif

#if defined( NUBIA_PALM_KEY_CAN_DISABLE )
       int nubia_pk_enable;
#endif

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	struct plug_interface plug;
#endif

#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	struct kobject *properties_kobj;
#endif
};

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_late_resume(struct early_suspend *es);
#endif

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
static void mxt_plugin_pre_process(struct plug_interface *pl, bool in_boot);
static long mxt_plugin_post_process(struct plug_interface *pl, bool in_boot);
static void mxt_plugin_hook_t6(struct plug_interface *pl, u8 status);
static int mxt_plugin_hook_t42(struct plug_interface *pl, u8 status);
static void mxt_plugin_hook_t61(struct plug_interface *pl, int id, u8 status);
static void mxt_plugin_hook_t68(struct plug_interface *pl, u8* msg);
static void mxt_plugin_hook_t72(struct plug_interface *pl, u8* msg);
static int mxt_plugin_get_pid_name(struct plug_interface *pl, char *name, int len);

static int mxt_plugin_hook_gesture_msg(struct plug_interface *pl, u8 type, u8 *msg);

static int mxt_plugin_hook_reg_init(struct plug_interface *pl, u8 *config_mem, size_t config_mem_size, int cfg_start_ofs);
static int mxt_plugin_hook_reg_access(struct plug_interface *pl, u16 addr, u16 reg, u16 len, const void *val, unsigned long flag, int result, bool is_w);
static void mxt_plugin_hook_reset_slots(struct plug_interface *pl);
static int mxt_plugin_hook_t100(struct plug_interface *pl, int id, int x, int y, struct  ext_info *in);
static int mxt_plugin_hook_t100_scraux(struct plug_interface *pl, struct scr_info *in);
static int mxt_plugin_hook_set_t7(struct plug_interface *pl, bool sleep);
static int mxt_plugin_cal_t37_check_and_calibrate(struct plug_interface *pl, bool check_sf, bool resume);
static void mxt_plugin_thread_stopped(struct plug_interface *pl);
static int mxt_plugin_force_stop(struct plug_interface *pl);
static int mxt_plugin_wakeup_enable(struct plug_interface *pl);
static int mxt_plugin_wakeup_disable(struct plug_interface *pl);
static int mxt_plugin_start(struct plug_interface *pl,bool resume);
static int mxt_plugin_stop(struct plug_interface *pl,bool suspend);
static int mxt_plugin_pre_init(struct mxt_data *data);
static int mxt_plugin_init(struct mxt_data *data);
static void mxt_plugin_deinit(struct plug_interface *pl);
static ssize_t mxt_plugin_show(struct device *dev,	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_tag_show(struct device *dev, struct device_attribute *attr, char *buf);
#	if defined(CONFIG_MXT_PLIGIN_CAL)
static ssize_t mxt_plugin_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_cal_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_PI)
static ssize_t mxt_plugin_glove_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_glove_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_stylus_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_stylus_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_wakeup_gesture_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_wakeup_gesture_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_gesture_list_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_gesture_list_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_gesture_trace_show(struct device *dev,
	struct device_attribute *attr, char *buf);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_MISC)
static ssize_t mxt_plugin_misc_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_misc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_CLIP)
static ssize_t mxt_plugin_clip_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_clip_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_plugin_clip_tag_show(struct device *dev,
	struct device_attribute *attr, char *buf);
#	endif

#	if defined(CONFIG_MXT_PLIGIN_AC)
static ssize_t mxt_plugin_ac_extern_event_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t mxt_plugin_ac_extern_event_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
#	endif

#endif


#if defined(CONFIG_FB)
#if defined(NUBIA_DEFERRED_RESUME_TP)
static int mxt_resume(struct device *dev);
void nubia_tp_resume_work_func(struct work_struct *work)
{
      struct mxt_data *mxt = container_of(work, struct mxt_data, nubia_tp_resume_work);
       dev_info(&mxt->client->dev, "%s: ztemt atmel resume work func\n", __func__);
       if (mxt_resume(&mxt->client->dev) != 0)
               dev_err(&mxt->client->dev, "%s: failed\n", __func__);
}
#endif
#endif

static bool keylist_empty(struct mxt_platform_data *pdata, u8 type)
{
	return sublist_empty(&pdata->keylist, type);
}

static int get_key_value(struct mxt_platform_data *pdata, u8 type, u8 id)
{
	return get_list_value(&pdata->keylist, type, id, 0);
}

static inline int mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline int mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}
static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;
	if (!data->object_table)
		return NULL;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);

	return NULL;
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
				message, data->T5_msg_size, false);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data) {
		dev_err(&data->client->dev, "Failed to allocate buffer\n");
		return;
	}

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_info(dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	dev_info(dev, "disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	if (data->debug_msg_data) {
		kfree(data->debug_msg_data);
		data->debug_msg_data = NULL;
	}
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_info(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	mutex_lock(&data->debug_msg_lock);

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data + data->debug_msg_count * data->T5_msg_size,
				msg,
				data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data)
		return -EIO;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	mutex_lock(&data->debug_msg_lock);

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{

	if (!data->debug_msg_attr.attr.name) {
		sysfs_bin_attr_init(&data->debug_msg_attr);
		data->debug_msg_attr.attr.name = kstrdup("debug_msg", GFP_KERNEL);
		data->debug_msg_attr.attr.mode = 0666;
		data->debug_msg_attr.read = mxt_debug_msg_read;
		data->debug_msg_attr.write = mxt_debug_msg_write;
		data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

		if (sysfs_create_bin_file(&data->client->dev.kobj,
					  &data->debug_msg_attr) < 0) {
			dev_err(&data->client->dev, "Failed to create %s\n",
				data->debug_msg_attr.attr.name);
			return -EINVAL;
		}
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name) {
		sysfs_remove_bin_file(&data->client->dev.kobj,
					  &data->debug_msg_attr);
		kfree(data->debug_msg_attr.attr.name);
		data->debug_msg_attr.attr.name = NULL;
	}
}

static int mxt_wait_for_completion(struct mxt_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

#define I2C_ACCESS_R_REG_FIXED   (1 << 0)   //don't mov reg address if read len is too long

#define I2C_ACCESS_NO_REG   (1 << 4)  // no reg address, directly access i2c reg
#define I2C_ACCESS_NO_CACHE   (1 << 5)  //no dma cache need

static int __mxt_cache_read(struct i2c_client *client,u16 addr,
				u16 reg, u16 len, void *val, u8 *r_cache, u8 *r_cache_pa, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg *msgs;
	int num;

	struct i2c_msg xfer[2];
	char buf[2];
	u16 transferred;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
		w_cache = w_cache_pa = buf;
		r_cache = r_cache_pa = val;
	}

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		msgs = &xfer[1];
		num = 1;
	}else{
		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;

		msgs = &xfer[0];
		num = ARRAY_SIZE(xfer);

		/* Write register */
		xfer[0].addr = addr;
		xfer[0].flags = 0;
		xfer[0].len = 2;
		xfer[0].buf = w_cache_pa;
	}

	/* Read data */
	xfer[1].addr = addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = r_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer[1].ext_flag = xfer[0].ext_flag = client->addr & 0xff00;
	xfer[1].timing = xfer[0].timing = 100;
	dev_dbg(&client->dev, "%s: i2c transfer(r)  (addr %x extflag %x) reg %d len %d\n",
		__func__, client->addr, xfer[0].ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		if (!test_flag(I2C_ACCESS_NO_REG | I2C_ACCESS_R_REG_FIXED,&flag)) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			xfer[1].buf = r_cache_pa + transferred;
		xfer[1].len = len - transferred;
		if (xfer[1].len > MXT_MAX_BLOCK_READ)
			xfer[1].len = MXT_MAX_BLOCK_READ;
	retry_read:
		ret = i2c_transfer(client->adapter, msgs, num);
		if (ret != num) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(r) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_read;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(r) failed (%d) reg %d len %d transferred %d\n",
					__func__, ret, reg, len, transferred);
				return -EIO;
			}
		}
		if (!test_flag(I2C_ACCESS_NO_CACHE,&flag))
			memcpy(val + transferred, r_cache, xfer[1].len);
		transferred += xfer[1].len;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(r) reg %d len %d current %d transferred %d\n",
			reg, len, xfer[1].len, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] r:", DUMP_PREFIX_NONE, 16, 1,
					test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer[1].buf : r_cache, xfer[1].len, false);
#endif
	}
	return 0;
}

static int __mxt_cache_write(struct i2c_client *client,u16 addr,
				u16 reg, u16 len, const void *val, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg xfer;
	void *buf = NULL;
	u16 transferred,extend;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		extend = 0;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			w_cache = w_cache_pa = (u8 *)val;
	}else {
		extend = 2;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
			buf = kmalloc(len + extend, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;
			w_cache = w_cache_pa = buf;
		}

		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;
	}

	/* Write register */
	xfer.addr = addr;
	xfer.flags = 0;
	xfer.buf = w_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer.ext_flag = client->addr & 0xff00;
	xfer.timing = 100;
	dev_dbg(&client->dev, "%s: i2c transfer(w) (addr %x extflag %x) reg %d len %d\n",
		__func__, client->addr , xfer.ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		xfer.len = len - transferred+ extend;
		if (xfer.len> MXT_MAX_BLOCK_WRITE)
			xfer.len = MXT_MAX_BLOCK_WRITE;

		if (test_flag(I2C_ACCESS_NO_CACHE,&flag) &&
			test_flag(I2C_ACCESS_NO_REG,&flag))
			xfer.buf = w_cache_pa + transferred;
		else
			memcpy(w_cache + extend, val + transferred, xfer.len - extend);

		if (extend) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

	retry_write:
		ret = i2c_transfer(client->adapter, &xfer, 1);
		if (ret != 1) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(w) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_write;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(w) failed (%d) reg %d len %d transferred %d\n",
					__func__, ret, reg, len, transferred);
				if (buf)
					kfree(buf);
				return -EIO;
			}
		}

		transferred += xfer.len -extend;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(w) reg %d len %d current %d transferred %d\n",
			reg, len, xfer.len -extend, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] w:", DUMP_PREFIX_NONE, 16, 1,
					test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer.buf : w_cache, xfer.len, false);
#endif
	}

	if (buf)
		kfree(buf);
	return 0;
}

static int __mxt_read_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len,
				void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	u8 *r_cache,*r_cache_pa,*w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)
	r_cache = data->i2c_dma_va;
	r_cache_pa = (void *)data->i2c_dma_pa;

	w_cache = data->i2c_dma_va + (data->dma_buffer_size >> 1);
	w_cache_pa = (void *)data->i2c_dma_pa + (data->dma_buffer_size >> 1);
#else
	r_cache_pa = r_cache = NULL;
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif

	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_read(client, addr, reg, len, val, r_cache, r_cache_pa, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_reg_access(&data->plug, addr, reg, len, val, flag, ret, false);
#endif
	return ret;
}


static int __mxt_write_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len,
				const void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	u8 *w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)
	w_cache = data->i2c_dma_va + (data->dma_buffer_size >> 1);
	w_cache_pa = (void *)data->i2c_dma_pa + (data->dma_buffer_size >> 1);

#else
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif
	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_write(client, addr, reg, len, val, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_reg_access(&data->plug, addr, reg, len, val, flag, ret, true);
#endif
	return ret;
}

static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len,
				void *val)
{
	return __mxt_read_reg_ext(client, client->addr, reg, len, val, 0);
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
				const void *val)
{
	return __mxt_write_reg_ext(client, client->addr, reg, len, val, 0);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static int mxt_write_reg_cfg(struct mxt_data *data, struct reg_config *config, u8 *stack_buf, unsigned long flag)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	u16 address;
	u8 old_buf[255],*buf;
	int ret;
	int i;

	dev_dbg(dev, "mxt set t%d offset %d, len %d mask 0x%lx, flag 0x%lx\n",
		config->reg,
		config->offset,
		config->len,
		config->mask,
		config->flag);

	if (test_flag(FLAG_REG_DATA_IN_EXT_BUF, &flag)) {
		dev_dbg(dev, "mxt set use ext buffer 0x%p len %d\n",
				config->ext_buf,config->len);
		buf = config->ext_buf;
	}else {
		buf = config->buf;
	}

	if (flag && config->flag) {
		if (!(flag & config->flag)) {
			dev_dbg2(dev, "Skip request: reg %d off %d len %d flag(0x%lx,0x%lx,0x%lx)\n",
				config->reg,config->offset,config->len,
				flag, config->flag, flag & config->flag);
			return 0;
		}
	}

	if (config->len == 0) {
		dev_dbg(dev, "Skip zero request: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return 0;
	}
	object = mxt_get_object(data, config->reg);
	if (!object) {
		dev_err(dev, "Object not found: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return -ENODATA;
	}
	if (!object->start_address) {
		dev_err(dev, "Object address not found: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return -EINVAL;
	}
	if (object->instances_minus_one < config->instance) {
		dev_err(dev, "Tried to write outside object T%d instance %d not exist",
			config->reg,config->instance);
		return -EINVAL;
	}
	if (config->offset >= mxt_obj_size(object)) {
		dev_err(dev, "Tried to write outside object T%d"
			" offset:%d, size:%d\n", config->reg, config->offset, mxt_obj_size(object));
		return -EINVAL;
	}
	if (config->len > sizeof(old_buf)) {
		dev_err(dev, "Config data too long: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return -EINVAL;
	}

	address = object->start_address + mxt_obj_size(object) * config->instance + config->offset;
	dev_dbg2(dev, "mxt set address %d\n",address);
	if (stack_buf || config->mask) {
		ret = __mxt_read_reg(data->client, address, config->len, old_buf);
		if (ret) {
			dev_err(dev, "Config read reg failed: reg %d off %d len %d\n",
				config->reg,config->offset,config->len);
			return ret;
		}
		if (config->mask) {
			for (i = 0; i < config->len; i++) {
				buf[i] &= config->mask;
				buf[i] |= old_buf[i] & ~config->mask;
			}
		}
	}

#if (DBG_LEVEL > 1)
	dev_info(dev, "mxt set t%d offset %d, len %d mask 0x%lx, flag 0x%lx\n",
		config->reg,
		config->offset,
		config->len,
		config->mask,
		config->flag);

	print_hex_dump(KERN_INFO, "[mxt]", DUMP_PREFIX_NONE, 16, 1,
			buf, config->len, false);
#endif
	ret = __mxt_write_reg(data->client, address, config->len, buf);

	if (ret == 0) {
		if (stack_buf)
			memcpy(stack_buf, old_buf, config->len);
		if (config->sleep)
			msleep(config->sleep);
	}

	return ret;
}

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
static int mxt_read_reg_cfg(struct mxt_data *data, struct reg_config *config, unsigned long flag)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;
	u8 *buf;
	u16 address;
	int ret;
	int i;

	dev_dbg(dev, "mxt get t%d ins %d offset %d, len %d mask 0x%lx, flag 0x%lx\n",
		config->reg,
		config->instance,
		config->offset,
		config->len,
		config->mask,
		config->flag);

	if (test_flag(FLAG_REG_DATA_IN_EXT_BUF, &flag)) {
		dev_dbg(dev, "mxt set use ext buffer 0x%p len %d\n",
				config->ext_buf,config->len);
		buf = config->ext_buf;
	}else {
		buf = config->buf;
		if (config->len > sizeof(config->buf))
			config->len = sizeof(config->buf);
	}
	memset(buf,0,config->len);

	if (flag && config->flag) {
		if (!(flag & config->flag)) {
			dev_dbg2(dev, "Skip request: reg %d off %d len %d flag(0x%lx,0x%lx,0x%lx)\n",
				config->reg,config->offset,config->len,
				flag, config->flag, flag & config->flag);
			return 0;
		}
	}

	object = mxt_get_object(data, config->reg);
	if (!object) {
		dev_dbg(dev, "Object not found: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return -ENODATA;
	}
	if (!object->start_address) {
		dev_err(dev, "Object address not found: reg %d off %d len %d\n",
			config->reg,config->offset,config->len);
		return -EINVAL;
	}
	if (object->instances_minus_one < config->instance) {
		dev_err(dev, "Tried to read outside object T%d instance %d not exist",
			config->reg,config->instance);
		return -EINVAL;
	}
	if (config->offset >= mxt_obj_size(object)) {
		dev_err(dev, "Tried to read outside object T%d"
			" offset:%d, size:%d\n", config->reg, config->offset, mxt_obj_size(object));
		return -EINVAL;
	}

	if (config->len) {
		address = object->start_address + mxt_obj_size(object) * config->instance + config->offset;
		ret = __mxt_read_reg(data->client, address, config->len, buf);
		if (ret) {
			dev_err(dev, "Config read reg failed: reg %d off %d len %d\n",
				config->reg,config->offset,config->len);
			return ret;
		}
		if (config->mask) {
			for (i = 0; i < config->len; i++) {
				buf[i] &= config->mask;
				buf[i] |= buf[i] & ~config->mask;
			}
		}
	}
	config->reg_len = mxt_obj_size(object);

#if (DBG_LEVEL > 1)
	dev_info(dev, "mxt get t%d ins %d offset %d, len %d mask 0x%lx, flag 0x%lx\n",
		config->reg,
		config->instance,
		config->offset,
		config->len,
		config->mask,
		config->flag);

	print_hex_dump(KERN_INFO, "[mxt]", DUMP_PREFIX_NONE, 16, 1,
			buf, config->len, false);
#endif

	return ret;
}
#endif

static int mxt_bootloader_read(struct mxt_data *data, u8 *val, unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_read_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}

static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val,
	unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_write_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}


static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr & 0x7F;
	u8 bootloader;
	u8 family_id = 0;

	if (data->info)
		family_id = data->info->family_id;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
#if defined(CONFIG_MXT_I2C_DMA)
	data->bootloader_addr |= MXT_I2C_DMA_ADDR_FLAG;
#endif

	dev_dbg(&data->client->dev,
			"Appmode i2c address 0x%02x, bootloader 0x%02x\n",appmode,bootloader);

	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool retry)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;

	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return -EIO;
		}

		dev_info(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_info(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
				bool wait)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
	if (wait) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = device_wait_irq_state(dev, 0, MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -EINTR better by terminating fw update
			 * process before returning to userspace by writing
			 * length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			dev_err(dev, "Update wait error %d\n", ret);
			// don't return false if there is interrupt issue
			return ret;
		}
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

#if defined(CONFIG_MXT_PROBE_ALTERNATIVE_CHIP)
static unsigned short mxt_lookup_chip_address(struct mxt_data *data, int retry)
{
	unsigned short address = data->client->addr & 0x7F;  //7 bit address

	if (retry && !(retry & 0x1)) {
		address++;
	}

	dev_err(&data->client->dev, "[mxt] try %d chip address 0x%x\n",retry,address);

	return address;
}
#endif

static void mxt_reset_slots(struct mxt_data *data);

static int mxt_proc_gesture_messages(struct mxt_data *data, u8 type, u8 *msg)
{
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev = data->input_dev;
	int value, id;

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return -ENODEV;

	if (keylist_empty(pdata, type))
		return -ENOKEY;

	id = mxt_plugin_hook_gesture_msg(&data->plug, type, msg);
	value = get_key_value(pdata, type, id);
	if (value > 0) {
#if defined ( NUBIA_PALM_KEY_CAN_DISABLE )
	if (((value == KEY_PALM_LOCK) || (value == KEY_F9) ) && !data->nubia_pk_enable) return 0;
#endif
#if defined ( NUBIA_REPORT_F9_AS_PALM )
		input_event(input_dev, EV_KEY, KEY_F9, 1);
		input_sync(input_dev);
		input_event(input_dev, EV_KEY, KEY_F9, 0);
		input_sync(input_dev);
		if ( value == KEY_F9 ) return 0;
#endif
		input_event(input_dev, EV_KEY, value, 1);
		input_sync(input_dev);
		input_event(input_dev, EV_KEY, value, 0);
		input_sync(input_dev);

		return 0;
	}
#endif

	return -ENOKEY;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_info(dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Detect transition out of reset */
	if (data->t6_status & MXT_T6_STATUS_RESET) {
		if (!(status & MXT_T6_STATUS_RESET)) {
			/* T7 config may have changed */
			mxt_init_t7_power_cfg(data);

			complete(&data->reset_completion);
		}
		data->enable_wakeup = 0;
	}

	/* Output debug if status has changed */
	if (status != data->t6_status) {
		dev_info(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

		if (status & MXT_T6_STATUS_CAL)
			mxt_reset_slots(data); //release all points in calibration for safe

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		mxt_plugin_hook_t6(&data->plug, status);
#endif
	}
	/* Save current status */
	data->t6_status = status;
}

#if defined(CONFIG_MXT_VENDOR_ID_BY_T19)
static void mxt_proc_t19_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	/* Output debug if status has changed */
	dev_info(dev, "T19 Status 0x%x\n",
		status);

	/* Save current status */
	memcpy(&data->t19_msg[0], &msg[1], sizeof(data->t19_msg));
}
#endif

static void mxt_proc_t24_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	int ret;

	dev_info(dev, "T24 Status 0x%x Info: %x %x %x %x %x %x\n",
		status,
		msg[2],
		msg[3],
		msg[4],
		msg[5],
		msg[6],
		msg[7]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key message, ret %d\n", ret);
	}
}

static void mxt_proc_t25_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	/* Output debug if status has changed */
	dev_info(dev, "T25 Status 0x%x Info: %x %x %x %x %x\n",
		status,
		msg[2],
		msg[3],
		msg[4],
		msg[5],
		msg[6]);

	/* Save current status */
	memcpy(&data->t25_msg[0], &msg[1], sizeof(data->t25_msg));
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	if (!input_dev) {
		printk(KERN_ERR "mxt_input_sync input dev is NULL!\n");
		return;
	}

	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

void parse_t100_ext_message(const u8 *message, const u8 *tchcfg, struct ext_info *in)
{
	u8 aux = 6;
	u8 exp;

	memset(in, 0, sizeof(struct ext_info));
	in->status = message[1];

	if (!(in->status & MXT_T100_DETECT))
		return;

	if (test_flag_8bit(MXT_T100_TCHAUX_VECT, &tchcfg[MXT_T100_TCHAUX]))
		in->vec = message[aux++];

	if (aux < 10) {
		if (test_flag_8bit(MXT_T100_TCHAUX_AMPL, &tchcfg[MXT_T100_TCHAUX])) {
			in->amp = message[aux++];
			if (in->amp < 0xff)
				in->amp++;
		}
	}

	if (aux < 10) {
		if (test_flag_8bit(MXT_T100_TCHAUX_AREA, &tchcfg[MXT_T100_TCHAUX]))
			in->area = message[aux++];
	}

	if (aux < 9) {
		if (test_flag_8bit(MXT_T100_TCHAUX_HW, &tchcfg[MXT_T100_TCHAUX])) {
				in->height = message[aux++];
				in->width = message[aux++];
			if (test_flag_8bit(MXT_T100_CFG_SWITCHXY, &tchcfg[MXT_T100_CFG1]))
				swap(in->height,in->width);
		}
	}

	if (aux < 10) {
		if (test_flag_8bit(MXT_T100_TCHAUX_PEAK, &tchcfg[MXT_T100_TCHAUX]))
			in->peak = message[aux++];
	}

	if (aux < 9) {
		if (test_flag_8bit(MXT_T100_TCHAUX_AREAHW, &tchcfg[MXT_T100_TCHAUX])) {
			exp = (message[aux] >> 5) & 0x3;
			in->area = (message[aux] & 0x1f) << exp;
			in->height = (message[aux + 1] & 0xf)  << exp;
			in->width = (message[aux + 1] >> 4)  << exp;
			if (test_flag_8bit(MXT_T100_CFG_SWITCHXY, &tchcfg[MXT_T100_CFG1]))
				swap(in->height,in->width);
		}
	}
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int tool;
//	s8 comp[2];
//	int tan;
	struct ext_info info;

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return;

	if (!data->enable_reporting)
		return;

	id = message[0] - data->T100_reportid_min - 2;
	/* ignore SCRSTATUS events */
	if (id < 0) {
		dev_dbg(dev,
			"T100 [%d] SCRSTATUS : 0x%x\n",
			id,
			message[1]);
		return;
	}

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];
	parse_t100_ext_message(message, data->tchcfg, &info);
	dev_dbg(dev,
		"[%u] status:%02X x:%u y:%u [amp]:%02X [vec]:%02X [area]:%02X [peak]:%02X [width]:%02X [height]:%02X\n",
		id,
		status,
		x, y,
		info.amp,
		info.vec,
		info.area,
		info.peak,
		info.width,
		info.height);

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	if (mxt_plugin_hook_t100(&data->plug, id, x, y, &info) != 0) {
		status = info.status;
		dev_dbg(dev,
			"[%u] status:%02X x:%u y:%u [amp]:%02X [vec]:%02X [area]:%02X [peak]:%02X [width]:%02X [height]:%02X *\n",
			id,
			status,
			x, y,
			info.amp,
			info.vec,
			info.area,
			info.peak,
			info.width,
			info.height);
	}
#endif

	input_mt_slot(input_dev, id);

	if (status & MXT_T100_DETECT) {
		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (test_flag_8bit(MXT_T100_TCHAUX_AMPL, &data->tchcfg[MXT_T100_TCHAUX]))
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 info.amp);

		if (test_flag_8bit(MXT_T100_TCHAUX_AREA | MXT_T100_TCHAUX_AREAHW, &data->tchcfg[MXT_T100_TCHAUX])) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 info.area);
		}

		if (test_flag_8bit(MXT_T100_TCHAUX_VECT, &data->tchcfg[MXT_T100_TCHAUX]))
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					 info.vec);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
void parse_t100_scr_message(const u8 *message, unsigned long scraux, struct scr_info *in)
{
	u8 aux = 1;

	memset(in, 0, sizeof(struct scr_info));
	in->status = message[aux++];

	if (test_flag(MXT_T100_SCRAUX_NUMTCH, &scraux))
		in->num_tch = message[aux++];

	if (test_flag(MXT_T100_SCRAUX_TCHAREA, &scraux)) {
		in->area_tch = MAKEWORD(message[aux], message[aux + 1]);
		aux += 2;
	}

	if (test_flag(MXT_T100_SCRAUX_ATCHAREA, &scraux)) {
		in->area_atch = MAKEWORD(message[aux], message[aux + 1]);
		aux += 2;
	}

	if (test_flag(MXT_T100_SCRAUX_INTTCHAREA, &scraux)) {
		in->area_inttch = MAKEWORD(message[aux], message[aux + 1]);
		aux += 2;
	}
}

static void mxt_proc_t100_scr_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	struct scr_info info;

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return;

	if (!data->enable_reporting)
		return;

	id = message[0] - data->T100_reportid_min - 2;

	if (id != -2) {
		dev_dbg(dev,
			"T100 [%d] msg : 0x%x\n",
			id,
			message[1]);
		return;
	}

	status = message[1];
	parse_t100_scr_message(message, data->tchcfg[MXT_T100_SCRAUX], &info);
	dev_dbg(dev,
		"[scr] status:%02X  [num]:%d [area]:%d %d %d\n",
		status,
		info.num_tch,
		info.area_tch,
		info.area_atch,
		info.area_inttch);

	mxt_plugin_hook_t42(&data->plug, status);

	if (mxt_plugin_hook_t100_scraux(&data->plug, &info) == 0)
		return;

	status = info.status;

#define EXAMPLE_FOR_KEY_CONTROLLED_EVENT
#if defined(EXAMPLE_FOR_KEY_CONTROLLED_EVENT)
	if (status & MXT_SCRAUX_STS_SUP) {
		dev_info(dev, "supression event (t42)\n");
		mxt_proc_gesture_messages(data, MXT_PROCI_TOUCHSUPPRESSION_T42, NULL);
	}
#elif defined(EXAMPLE_FOR_PRESSURE_CONTROLLED_SUSPEND_EVENT)
	if (status & MXT_SCRAUX_STS_SUP) {
		//e.g for pressure controlled suspend gesture
		if ((test_flag_8bit(MXT_T100_TCHAUX_AMPL, &data->tchcfg[MXT_T100_TCHAUX]))) {
			dev_info(dev, "supression event (pressure)\n");

			data->update_input = false;   //disable current report

#define ABS_TOOL_SURRESS_PRESSURE 1000
			mxt_reset_slots(data);

			input_mt_slot(input_dev, 0);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, 0);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, 0);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					ABS_TOOL_SURRESS_PRESSURE);
			/*
			input_report_abs(input_dev, ABS_PRESSURE,
						ABS_TOOL_SURRESS_PRESSURE);
			*/
			mxt_input_sync(input_dev);

			mxt_reset_slots(data);
		}
	}
#endif
}
#endif


static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev = data->input_dev;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);
	unsigned long key_chg;
	int i,value;


	/* do not report events if input device not yet registered */
	if (!input_dev)
		return;

	if (!data->enable_reporting)
		return;

	if (keylist_empty(pdata, MXT_TOUCH_KEYARRAY_T15))
		return;

	key_chg = keystates ^ data->t15_keystatus;
	if (key_chg) {
		for (i = 0;; i++) {
			if(key_chg & 0x1) {
				value = get_key_value(pdata, MXT_TOUCH_KEYARRAY_T15, i);
				if (value > 0) {
					input_event(input_dev, EV_KEY, value, (keystates >> i) & 0x1);
					sync = true;
				}
			}

			key_chg >>= 1;
			if (!key_chg)
				break;
		}

		if (sync)
			input_sync(input_dev);

		data->t15_keystatus = keystates;
	}
}

static void mxt_proc_t97_messages(struct mxt_data *data, u8 *msg)
{
	mxt_proc_t15_messages(data,msg);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_t42(&data->plug, status);
#endif
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
static void mxt_proc_t61_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 reportid = msg[0];
	int ret;

	dev_dbg(dev, "T61 status 0x%x 0x%x\n",
			msg[0],
			msg[1]);

	msg[0] -= data->T61_reportid_min;
	mxt_plugin_hook_t61(&data->plug, msg[0], msg[1]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_SPT_TIMER_T61, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key message, ret %d\n", ret);
	}
	msg[0] = reportid;
}
#endif

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return;

	if (!data->enable_reporting)
		return;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	dev_dbg(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		(msg[1] & MXT_T63_STYLUS_SUPPRESS) ? 'S' : '.',
		(msg[1] & MXT_T63_STYLUS_MOVE)	 ? 'M' : '.',
		(msg[1] & MXT_T63_STYLUS_RELEASE)  ? 'R' : '.',
		(msg[1] & MXT_T63_STYLUS_PRESS)	? 'P' : '.',
		x, y, pressure,
		(msg[2] & MXT_T63_STYLUS_BARREL) ? 'B' : '.',
		(msg[2] & MXT_T63_STYLUS_ERASER) ? 'E' : '.',
		(msg[2] & MXT_T63_STYLUS_TIP)	? 'T' : '.',
		(msg[2] & MXT_T63_STYLUS_DETECT) ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			 (msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			 (msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(input_dev);
}

static void mxt_proc_t68_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	dev_info(dev, "T68 state = 0x%x\n" ,
		msg[1]);

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_t68(&data->plug, msg);
#endif
}

static void mxt_proc_t72_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	dev_info(dev, "T72 noise state1 = 0x%x state2 = 0x%x\n" ,
		msg[1],
		msg[2]);

	if (msg[1] & MXT_T72_NOISE_SUPPRESSION_NOISELVCHG) {
		dev_info(dev, "T72 noise change, state = %d, peak = %d, level = %d\n" ,
			msg[2] & 0x7,
			msg[4],
			msg[5]);
	}
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_t72(&data->plug, msg);
#endif
}

static void mxt_proc_T81_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 reportid = msg[0];
 	u8 status = msg[1];
	int ret;

	dev_info(dev, "T81 Status 0x%x Info: %x %x %x %x\n",
		status,
		msg[2],
		msg[3],
		msg[4],
		msg[5]);

	msg[0] -= data->T81_reportid_min;
	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_PROCI_UNLOCKGESTURE_T81, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key message, ret %d\n", ret);
	}
	msg[0] = reportid;
}

static void mxt_proc_T92_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	int ret;

	dev_info(dev, "T92 %s 0x%x Info: %x %x %x %x %x %x\n",
		(status & 0x80) ? "stroke" : "symbol",
		status,
		msg[2],
		msg[3],
		msg[4],
		msg[5],
		msg[6],
		msg[7]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_PROCI_GESTURE_T92, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key message, ret %d\n", ret);
	}
}

static void mxt_proc_T93_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	int ret;

	dev_info(dev, "T93 Status 0x%x Info: %x\n",
		status,
		msg[1]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_PROCI_TOUCHSEQUENCELOGGER_T93, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key 93 message, ret %d\n", ret);
	}
}

static void mxt_proc_T99_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	int ret;

	dev_info(dev, "T99 Status 0x%x Event: %d Index %d\n",
		status,
		msg[1] & 0xF,
		msg[2]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		ret = mxt_proc_gesture_messages(data, MXT_PROCI_KEYGESTUREPROCESSOR_T99, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key 99 message, ret %d\n", ret);
	}
}

static void mxt_proc_t102_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	dev_info(dev, "msg for t102 = 0x%x 0x%x 0x%x 0x%x\n",
		msg[2], msg[3], msg[4], msg[5]);
}

static void mxt_proc_T115_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 type,status = msg[1];
	int ret;

	dev_info(dev, "T115 Status 0x%x Info: %x\n",
		status,
		msg[1]);

	/* do not report events if input device not yet registered */
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		if (status & 0x80) {
			type = MXT_PROCI_SYMBOLGESTURE_T115;
		}else {
			type = MXT_SPT_SYMBOLGESTURECONFIG_T116;
		}
		ret = mxt_proc_gesture_messages(data, type, msg);
		if (!ret)
			set_bit(MXT_WK_DETECTED,&data->enable_wakeup);
		else
			dev_info(dev, "Unhandled key 115 message, ret %d\n", ret);
	}
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T100_reportid_min
		&& report_id <= data->T100_reportid_max) {
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		if (report_id < data->T100_reportid_min + 2)
			mxt_proc_t100_scr_message(data, message);
		else
#endif
			mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
#if defined(CONFIG_MXT_VENDOR_ID_BY_T19)
		mxt_proc_t19_messages(data, message);
#endif
	} else if (report_id == data->T25_reportid) {
		mxt_proc_t25_messages(data, message);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	} else if (report_id >= data->T61_reportid_min
			&& report_id <= data->T61_reportid_max) {
		mxt_proc_t61_messages(data, message);
#endif
	} else if (report_id >= data->T63_reportid_min
			&& report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
			&& report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
			&& report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else if ( report_id == data->T24_reportid) {
		mxt_proc_t24_messages(data, message);
	} else if (report_id >= data->T68_reportid_min
			&& report_id <= data->T68_reportid_max) {
		mxt_proc_t68_messages(data, message);
	} else if (report_id >= data->T72_reportid_min
			&& report_id <= data->T72_reportid_max) {
		mxt_proc_t72_messages(data, message);
	} else if (report_id >= data->T81_reportid_min
			&& report_id <= data->T81_reportid_max) {
		mxt_proc_T81_messages(data, message);
	} else if (report_id == data->T92_reportid) {
		mxt_proc_T92_messages(data, message);
	} else if (report_id == data->T93_reportid) {
		mxt_proc_T93_messages(data, message);
	} else if (report_id >= data->T97_reportid_min
			&& report_id <= data->T97_reportid_max) {
		mxt_proc_t97_messages(data, message);
	} else if (report_id == data->T99_reportid) {
		mxt_proc_T99_messages(data, message);
	} else if (report_id == data->T102_reportid) {
		mxt_proc_t102_messages(data, message);
	} else if (report_id == data->T115_reportid) {
		mxt_proc_T115_messages(data, message);
	} else {
		dump = true;
	}

	if (dump || report_id > data->max_reportid)
		mxt_dump_message(data, message);

	if (data->debug_v2_enabled && report_id <= data->max_reportid)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg_ext(data->client, data->client->addr, data->T5_address,
				data->T5_msg_size * count, data->msg_buf, I2C_ACCESS_R_REG_FIXED);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		/*dev_warn*/dev_dbg(dev, "Interrupt triggered but zero messages\n");
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		dev_err(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}

end:
	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, 1);
		if (read < 1)
			return 0;
	} while (--count);

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	irqreturn_t ret;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_NONE;

	if (!data->msg_buf)
		return IRQ_NONE;

	if (data->T44_address) {
		ret = mxt_process_messages_t44(data);
	} else {
		ret = mxt_process_messages(data);
	}

	return ret;
}

#if defined(CONFIG_MXT_FW_UPDATE_EXAMPLE_BY_KTHREAD)
static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static int mxt_process_fw_upate_example_thread(void *dev_id)
{
	struct device *dev = dev_id;
	struct mxt_data *data;
	int s_wait = 8;		//seconds, must more than android start time

	const char *fw_name = "A4_2F_1.5_AA.fw";
	const char *cfg_name = "A4_2F.raw";

	while(s_wait-- > 0) {
		msleep(1000);

		dev_info(dev, "s_wait %d\n", s_wait);

		if (kthread_should_stop())
			return -ESRCH;
	}

	data = dev_get_drvdata(dev);
	if (!data)
		return -EIO;

	dev_info(dev, "mxt_process_fw_upate_example_thread: fw %s\n", fw_name);
	mxt_update_fw_store(dev, NULL, fw_name, strlen(fw_name));

	msleep(100);
	if (kthread_should_stop())
		return -ESRCH;

	dev_info(dev, "mxt_process_fw_upate_example_thread: cfg %s \n", cfg_name);
	mxt_update_cfg_store(dev, NULL, cfg_name, strlen(cfg_name));

	data->tsk_handle_update =NULL;
	return 0;
}
#endif

#if defined(CONFIG_MXT_IRQ_WORKQUEUE)

void mxt_active_proc_thread(void *dev_id, unsigned int event)
{
	struct mxt_data *data;

	data = (struct mxt_data *)dev_id;
	if (!data)
		return;

	set_bit(event,&data->busy);
	if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
		if (event == MXT_EVENT_IRQ)
			set_bit(MXT_EVENT_IRQ_FLAG,&data->busy);
	}
	wake_up_interruptible(&data->wait);
}

static irqreturn_t mxt_interrupt_workqueue_handler(int irq, void *dev_id)
{
	struct mxt_data *data = (struct mxt_data *)dev_id;
	struct device *dev = &data->client->dev;

	dev_dbg(dev, "irq workqueue\n");

	device_disable_irq_nosync(dev, __func__);
	if (data) {
		mxt_active_proc_thread(data,MXT_EVENT_IRQ);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

//warning: don't put any write/read code in this function
static int mxt_process_message_thread(void *dev_id)
{
	struct device *dev = dev_id;
	struct mxt_data *data;
	struct mxt_platform_data *pdata;
	long interval = MAX_SCHEDULE_TIMEOUT;
	int post = false;
	irqreturn_t iret;

	data = dev_get_drvdata(dev);
	if (!data)
		return -EIO;

	pdata = data->pdata;

	/*
		you could create a new thread to update fw/cfg here for skip selinux check by shell
	*/
#if defined(CONFIG_MXT_FW_UPDATE_EXAMPLE_BY_KTHREAD)
	data->tsk_handle_update = kthread_run(mxt_process_fw_upate_example_thread, dev,
						"Atmel_mxt_ts_fw_update_eg");
#endif

	while (!kthread_should_stop()) {

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		mxt_plugin_pre_process(&data->plug,data->in_bootloader);
#endif
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible_timeout(
			data->wait,
			test_bit(MXT_EVENT_IRQ,&data->busy)||test_bit(MXT_EVENT_EXTERN,&data->busy)||
				kthread_should_stop(),
			interval);

		set_current_state(TASK_RUNNING);

		dev_dbg(dev, "mxt_process_message_thread busy %lx suspend %d  boot %d interval %ld(0x%lx)\n",
			data->busy,
			data->suspended,
			data->in_bootloader,
			interval,interval);

		if (kthread_should_stop()) {
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
			mxt_plugin_thread_stopped(&data->plug);
#endif
			break;
		}

		if (test_and_clear_bit(MXT_EVENT_IRQ,&data->busy)) {
			iret = mxt_interrupt(pdata->irq, (void *)data);
			if(iret == IRQ_NONE) {
				if (data->pdata->irqflags & IRQF_TRIGGER_LOW) {
					dev_err(dev, "Invalid irq: busy 0x%lx depth %d\n",
						data->busy,atomic_read(&pdata->depth));
						msleep(MXT_WAKEUP_TIME);
				}
			}
			device_enable_irq(dev, __func__);
		}

		if (data->suspended) {
			interval = MAX_SCHEDULE_TIMEOUT;
			if (test_bit(MXT_EVENT_EXTERN,&data->busy))
				post = true;
		}
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		else{
			post = true;
		}

		if (post) {
			clear_bit(MXT_EVENT_EXTERN,&data->busy);
			interval = (long)mxt_plugin_post_process(&data->plug,data->in_bootloader);
		}
#endif
	}

#if defined(CONFIG_MXT_FW_UPDATE_EXAMPLE_BY_KTHREAD)
	if (data->tsk_handle_update) {
		kthread_stop(data->tsk_handle_update);
		data->tsk_handle_update = NULL;
	}
#endif

	data->tsk_handle_msg = NULL;

	return 0;
}
#else
	#define mxt_interrupt_workqueue_handler NULL
#endif

static int mxt_acquire_irq(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct i2c_client *client = data->client;

	int error;

	error = device_register_irq(dev, mxt_interrupt_workqueue_handler, mxt_interrupt, client->name, data);
	if (error) {
		dev_err(dev, "Failed to register interrupt error %d\n", error);
		return error;
	}

	//mxt_process_messages_until_invalid(data);

	return 0;
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(MXT_WAKEUP_TIME);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting chip(soft)\n");

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	msleep(MXT_RESET_TIME);

	mxt_process_messages_until_invalid(data);

	ret = mxt_wait_for_completion(data, &data->reset_completion,
					  MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	msleep(MXT_WAKEUP_TIME);

	return 0;
}

static int mxt_set_reset(struct mxt_data *data, int por)
{
	struct device *dev = &data->client->dev;
	int ret = -EBUSY;

	if (por) {
		dev_info(dev, "mxt set POR reset\n");
		ret = device_por_reset(dev);
	}

	if (ret) {
		//hardware reset or soft reset, return 0 if success
		dev_info(dev, "mxt set HW reset\n");
		ret = device_hw_reset(dev);
	}

	if (ret) {
		dev_info(dev, "mxt set SOFT reset\n");
		ret = mxt_soft_reset(data);
	}

	return ret;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	struct device *dev = &data->client->dev;
	dev_info(dev, "Getting crc\n");

	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/* Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);

	mxt_process_messages_until_invalid(data);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

/*
 * mxt_check_reg_init - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *	<TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *	<TYPE> - 2-byte object type as hex
 *	<INSTANCE> - 2-byte object instance number as hex
 *	<SIZE> - 2-byte object size as hex
 *	<CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_check_reg_init(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	struct mxt_object *object;
	const struct firmware *cfg = NULL;
	int ret;
	int offset;
	int data_pos;
	int byte_offset;
	int i;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;
	unsigned int type, instance, size;
	u8 val;
	u16 reg;
#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
	u8 *object_mem,*object_offset;
#endif

	if (!data->cfg_name) {
		dev_dbg(dev, "Skipping cfg download\n");
		return 0;
	}

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			data->cfg_name);

		return 0;
	}

	if (data->config_crc == 0)
		mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
				 (unsigned char *)&cfg_info + i,
				 &offset);
	#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		print_trunk(cfg->data, data_pos, offset);
	#endif
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	if (cfg_info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	print_trunk(cfg->data, data_pos, offset);
#endif
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	print_trunk(cfg->data, data_pos, offset);
#endif
	data_pos += offset;

	/* The Info Block CRC is calculated over mxt_info and the object table
	 * If it does not match then we are trying to load the configuration
	 * from a different chip or firmware version, so the configuration CRC
	 * is invalid anyway. */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_info(dev, "Config CRC 0x%06X: OK\n",
				 data->config_crc);
#if defined(CONFIG_MXT_CAL_TRIGGER_CAL_WHEN_CFG_MATCH)
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		/* Recalibrate since chip has been in deep sleep */
		if (mxt_plugin_cal_t37_check_and_calibrate(&data->plug, false,false) != 0)
#endif
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
#endif
			ret = 0;
			goto release;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START
		+ data->info->object_num * sizeof(struct mxt_object)
		+ MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
	config_mem_size <<= 1;
#endif
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		dev_err(dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}
#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
	object_mem = config_mem + (config_mem_size>>1);
#endif

	while (data_pos < cfg->size - 16) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
				 &type, &instance, &size, &offset);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		print_trunk(cfg->data, data_pos, offset);
#endif
#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
		object_offset = object_mem;
#endif
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			/*ret = -EINVAL;
			goto release_mem;*/
			break;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
						 &val,
						 &offset);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
				print_trunk(cfg->data, data_pos, offset);
#endif

				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/* Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited */
			dev_warn(dev, "Discarding %ud byte(s) in T%d\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/* If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration. */
			dev_warn(dev, "Zeroing %d byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			ret = -EINVAL;
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n",
					 &val,
					 &offset);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
			print_trunk(cfg->data, data_pos, offset);
#endif
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d\n", type);
				ret = -EINVAL;
				goto release_mem;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if ((byte_offset >= 0)
				&& (byte_offset <= config_mem_size)) {
				*(config_mem + byte_offset) = val;
		#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
				*(object_offset++)=val ;
		#endif
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				ret = -EINVAL;
				goto release_mem;
			}
		}

#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
		ret = __mxt_write_reg(data->client,reg,size,object_mem);
		if(ret!=0){
			dev_err(dev,"write object[%d] error\n",object->type);
			goto release_mem ;
		}
#endif
	}

	/* calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		dev_err(dev, "Bad T7 address, T7addr = %x, config offset %x\n",
			data->T7_address, cfg_start_ofs);
		ret = 0;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem,
						data->T7_address - cfg_start_ofs,
						config_mem_size);

	if (config_crc > 0 && (config_crc != calculated_crc))
		dev_warn(dev, "Config CRC error, calculated=%06X, file=%06X\n",
			 calculated_crc, config_crc);

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_reg_init(&data->plug, config_mem, config_mem_size, cfg_start_ofs);
#endif
	/* Write configuration as blocks */
	byte_offset = 0;

#if defined(CONFIG_MXT_UPDATE_BY_OBJECT)
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		ret = __mxt_write_reg(data->client,
					  cfg_start_ofs + byte_offset,
					  size, config_mem + byte_offset);
		if (ret != 0) {
			dev_err(dev, "Config write error, ret=%d\n", ret);
			goto release_mem;
		}

		byte_offset += size;
	}
#endif

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	dev_info(dev, "Config written\n");

release_mem:
	kfree(config_mem);
release:
	release_firmware(cfg);

	mxt_set_reset(data, 0);

	return ret;
}

static int mxt_set_smartscan_sync_cfg(struct mxt_data *data, u8 sleep)
{
	int i;
	int ret;
	struct reg_config reg_list[] =  {
		{.reg = MXT_SPT_SMARTSCAN_T124, .instance = 0,
				.offset = 0,.buf = {1}, .len = 1, .mask = 0x1, .flag = 0, .sleep = 0},
		/*
		{.reg = MXT_SPT_CTECONFIG_T46,
			.offset = 4,.buf = {1}, .len = 1, .mask = 0, .flag = 0, .sleep = 0},
		{.reg = MXT_SPT_SELFCAPCONFIG_T111, .instance = 0,
			.offset = 25,.buf = {1}, .len = 1, .mask = 0, .flag = 0, .sleep = 0},
		{.reg = MXT_SPT_SELFCAPCONFIG_T111, .instance = 1,
			.offset = 25,.buf = {1}, .len = 1, .mask = 0, .flag = 0, .sleep = 0},
		*/
	};

	for (i = 0; i < ARRAY_SIZE(reg_list); i++) {
		if (sleep == MXT_POWER_CFG_DEEPSLEEP)
			reg_list[i].buf[0] = 0;

		ret = mxt_write_reg_cfg(data, &reg_list[i], NULL, 0);
		if (ret)
			break;
	}

	return ret;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	struct mxt_platform_data *pdata = data->pdata;
#endif
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP) {
		new_config = &deepsleep;
		mxt_set_smartscan_sync_cfg(data, sleep);
	}else
		new_config = &data->t7_cfg;

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	error = mxt_plugin_hook_set_t7(&data->plug, sleep == MXT_POWER_CFG_DEEPSLEEP);
	if (error) {	//T6 have flags not clear
		WARN_ON(atomic_read(&pdata->depth) < 0);
		//device_enable_irq(dev, __func__);
	}
#endif
	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg),
			new_config);
	if (error)
		return error;

	if (sleep == MXT_POWER_CFG_RUN)
		mxt_set_smartscan_sync_cfg(data, sleep);

	dev_dbg(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct t7_config cfg;
	int error;

	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(cfg), &cfg);
	if (error) {
		dev_info(dev, "Failed read T7 power config, set free run\n");
		data->t7_cfg.active = data->t7_cfg.idle = 255;
		return error;
	}

	if (cfg.active == 0 || cfg.idle == 0)
		dev_err(dev, "T7 cfg zero after reset\n");
	 else {
		dev_info(dev, "Initialised power cfg: ACTV %d, IDLE %d\n",
				cfg.active, cfg.idle);
		memcpy(&data->t7_cfg, &cfg, sizeof(cfg));
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	//mxt_debug_msg_remove(data);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_force_stop(&data->plug);
	mxt_plugin_deinit(&data->plug);
#endif
	if (data->raw_info_block) {
		kfree(data->raw_info_block);
		data->raw_info_block = NULL;
	}
	data->object_table = NULL;
	data->info = NULL;
	data->raw_info_block = NULL;
	if (data->msg_buf) {
		kfree(data->msg_buf);
		data->msg_buf = NULL;
	}
	mxt_debug_msg_remove(data);
	mxt_free_input_device(data);

	data->enable_reporting = false;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T8_address = 0;
	data->T9_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_address = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_address = 0;
	data->T19_reportid = 0;
	data->T24_address = 0;
	data->T24_reportid = 0;
	data->T25_address = 0;
	data->T25_reportid = 0;
	data->T37_address = 0;
	data->T38_address = 0;
	data->T40_address = 0;
	data->T42_address = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T46_address = 0;
	data->T47_address = 0;
	data->T48_reportid = 0;
	data->T55_address = 0;
	data->T56_address = 0;
	data->T61_address = 0;
	data->T61_reportid_min = 0;
	data->T61_reportid_max = 0;
	data->T61_instances = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T65_address = 0;
	data->T68_address = 0;
	data->T71_address = 0;
	data->T72_address = 0;
	data->T72_reportid_min = 0;
	data->T72_reportid_max = 0;
	data->T78_address = 0;
	data->T80_address = 0;
	data->T81_address = 0;
	data->T81_reportid_min= 0;
	data->T81_reportid_max= 0;
	data->T92_address = 0;
	data->T92_reportid = 0;
	data->T93_address = 0;
	data->T93_reportid = 0;
	data->T96_address = 0;
	data->T97_address = 0;
	data->T97_reportid_min = 0;
	data->T97_reportid_max = 0;
	data->T99_address = 0;
	data->T99_reportid = 0;
	data->T100_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->T102_address = 0;
	data->T102_reportid = 0;
	data->T104_address = 0;
	data->T113_address = 0;
	data->T115_address = 0;
	data->T115_reportid = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGEPROCESSOR_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
		case MXT_GEN_COMMANDPROCESSOR_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWERCONFIG_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_GEN_ACQUISITIONCONFIG_T8:
			data->T8_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->T9_address = object->start_address;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			data->T15_address = object->start_address;
			break;
		case MXT_SPT_COMCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_address = object->start_address;
			data->T19_reportid = min_id;
			break;
		case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
			data->T24_address = object->start_address;
			data->T24_reportid = min_id;
			break;
		case MXT_SPT_SELFTEST_T25:
			data->T25_address = object->start_address;
			data->T25_reportid = min_id;
			break;
		case MXT_DEBUG_DIAGNOSTIC_T37:
			data->T37_address = object->start_address;
			break;
		case MXT_SPT_USERDATA_T38:
			data->T38_address = object->start_address;
			break;
		case MXT_PROCI_GRIPSUPPRESSION_T40:
			data->T40_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_address = object->start_address;
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPARE_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_CTECONFIG_T46:
			data->T46_address = object->start_address;
			break;
		case MXT_PROCI_STYLUS_T47:
			data->T47_address = object->start_address;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_ADAPTIVE_T55:
			data->T55_address = object->start_address;
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			data->T56_address = object->start_address;
			break;
		case MXT_SPT_TIMER_T61:
			/* Only handle messages from first T63 instance */
			data->T61_address = object->start_address;
			data->T61_reportid_min = min_id;
			data->T61_reportid_max = max_id;
			data->T61_instances = mxt_obj_instances(object);
			break;
		case MXT_PROCI_ACTIVESTYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		case MXT_PROCI_LENSBENDING_T65:
			data->T65_address = object->start_address;
			break;
		case MXT_SPARE_T68:
			data->T68_address = object->start_address;
			data->T68_reportid_min = min_id;
			data->T68_reportid_max = max_id;
			break;
		case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
			data->T71_address = object->start_address;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T72:
			data->T72_address = object->start_address;
			data->T72_reportid_min = min_id;
			data->T72_reportid_max = max_id;
			break;
		case MXT_PROCI_GLOVEDETECTION_T78:
			data->T78_address = object->start_address;
			break;
		case MXT_PROCI_RETRANSMISSIONCOMPENSATION_T80:
			data->T80_address = object->start_address;
			break;
		case MXT_PROCI_UNLOCKGESTURE_T81:
			data->T81_address = object->start_address;
			data->T81_reportid_min = min_id;
			data->T81_reportid_max = max_id;
			break;
		case MXT_PROCI_GESTURE_T92:
			data->T92_address = object->start_address;
			data->T92_reportid = min_id;
			break;
		case MXT_PROCI_TOUCHSEQUENCELOGGER_T93:
			data->T93_address = object->start_address;
			data->T93_reportid = min_id;
			break;
		case MXT_TOUCH_SPT_PTC_TUNINGPARAMS_T96:
			data->T96_address = object->start_address;
			break;
		case MXT_TOUCH_PTC_KEYS_T97:
			data->T97_reportid_min = min_id;
			data->T97_reportid_max = max_id;
			data->T97_address = object->start_address;
			break;
		case MXT_PROCI_KEYGESTUREPROCESSOR_T99:
			data->T99_address = object->start_address;
			data->T99_reportid = min_id;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			data->T100_address = object->start_address;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_SPT_SELFCAPHOVERCTECONFIG_T102:
			data->T102_address = object->start_address;
			data->T102_reportid = min_id;
			break;
		case MXT_PROCI_AUXTOUCHCONFIG_T104:
			data->T104_address = object->start_address;
			break;
		case MXT_SPT_SELFCAPMEASURECONFIG_T113:
			data->T113_address = object->start_address;
			break;
		case MXT_PROCI_SYMBOLGESTURE_T115:
			data->T115_address = object->start_address;
			data->T115_reportid = min_id;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
					size - MXT_OBJECT_START,
					buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;
	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
						size - MXT_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);

		dev_err(&client->dev, "info block size %zd\n",size);
		print_hex_dump(KERN_ERR, "[mxt] INFO:", DUMP_PREFIX_NONE, 16, 1,
			buf, size, false);

		error = -EIO;
		goto err_free_mem;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	dev_info(&client->dev,
		 "T5 message size %d\n", data->T5_msg_size);

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);
	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static int mxt_read_t100_config(struct mxt_data *data, u16 *rx, u16 *ry, u8 *cfg)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
					object->start_address + MXT_T100_XRANGE,
					sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
					object->start_address + MXT_T100_YRANGE,
					sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error = __mxt_read_reg(client,
				object->start_address,
				sizeof(cfg), cfg);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (test_flag_8bit(MXT_T100_CFG_SWITCHXY, &cfg[MXT_T100_CFG1]))
		swap(range_x, range_y);

	*rx = range_x;
	*ry = range_y;

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	struct obj_link *ln;
	struct obj_container *con;
	u16 range_x, range_y;
	u8 cfg[4];
	int error;

	error = mxt_read_t100_config(data, &range_x, &range_y, cfg);
	if (error == 0) {
		if (data->max_x != range_x ||
			data->max_y != range_y ||
			memcmp(data->tchcfg, cfg, sizeof(data->tchcfg))) {	//release input device if resolution not match

			mxt_free_input_device(data);
		}
		data->max_x = range_x;
		data->max_y = range_y;
		/* allocate aux bytes */
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
		data->max_y_t = data->pdata->max_y_t;
		if (data->max_y_t > range_y)
			data->max_y_t = range_y;
#endif
		memcpy(data->tchcfg, cfg, sizeof(data->tchcfg));
	}

	if (data->input_dev) {
		dev_info(dev, "Already initialized T100 input devices\n");
		return 0;
	}

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "atmel_mxt_ts_T100_touchscreen";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
#if defined(INPUT_PROP_DIRECT)
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids, INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	//input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
				 0, data->max_x, 0, 0);
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				 0, data->max_y_t, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				 0, data->max_y, 0, 0);
#endif

	if (test_flag_8bit(MXT_T100_TCHAUX_AREA | MXT_T100_TCHAUX_AREAHW, &data->tchcfg[MXT_T100_TCHAUX]))
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
					 0, MXT_MAX_AREA, 0, 0);

	if (test_flag_8bit(MXT_T100_TCHAUX_AMPL, &data->tchcfg[MXT_T100_TCHAUX]))
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
					 0, 255, 0, 0);

	if (test_flag_8bit(MXT_T100_TCHAUX_VECT, &data->tchcfg[MXT_T100_TCHAUX]))
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
					 0, 255, 0, 0);

	/* For T15 key array */
	data->t15_keystatus = 0;

	/* For Key register */
	list_for_each_entry(ln, &pdata->keylist, node) {
		list_for_each_entry(con, &ln->sublist, node) {
			set_bit(con->value, input_dev->keybit);
			input_set_capability(input_dev, EV_KEY,con->value);
		}
	}
#if defined ( NUBIA_REPORT_F9_AS_PALM )
	input_set_capability(input_dev, EV_KEY, KEY_F9);
#endif

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}
static int mxt_configure_objects(struct mxt_data *data);

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	bool alt_bootloader_addr = false;
	bool retry = false;
#if defined(CONFIG_MXT_PROBE_ALTERNATIVE_CHIP)
	unsigned short addr_bak = 0;
	int probe_retry = 0;
#endif
retry_info:
	error = mxt_read_info_block(data);
#if defined(CONFIG_MXT_PROBE_ALTERNATIVE_CHIP)
	while(error) {
#	if defined(CONFIG_MXT_POWER_CONTROL_SUPPORT_AT_PROBE)
		device_por_reset(dev);
#	endif
		client->addr = mxt_lookup_chip_address(data, probe_retry);
#if defined(CONFIG_MXT_I2C_DMA)
		client->addr |= MXT_I2C_DMA_ADDR_FLAG;
#endif
		if (!addr_bak)
			addr_bak = client->addr;
		error = mxt_read_info_block(data);

		if (++probe_retry > CONFIG_MXT_PROBE_ALTERNATIVE_CHIP)
			break;
	}
	if (error)
		client->addr = addr_bak;
#endif
	if (error) {
retry_bootloader:
		error = mxt_probe_bootloader(data, alt_bootloader_addr);
		if (error) {
			if (alt_bootloader_addr) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}

			dev_info(&client->dev, "Trying alternate bootloader address\n");
			alt_bootloader_addr = true;
			goto retry_bootloader;
		} else {
			if (retry) {
				dev_err(&client->dev,
						"Could not recover device from "
						"bootloader mode\n");
				/* this is not an error state, we can reflash
				 * from here */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			retry = true;
			goto retry_info;
		}
	}

#if defined(CONFIG_MXT_FORCE_RESET_AT_POWERUP)
	dev_warn(&client->dev, "board force a reset after gpio init\n");
	mxt_set_reset(data, 0);
#endif

	error = mxt_configure_objects(data);
	if (error)
		return error;

	return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = mxt_debug_msg_init(data);
	if (error)
		return error;

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	error = mxt_plugin_pre_init(data);
	if (error) {
		dev_err(&client->dev, "Error %d plugin init\n",
			error);
		return error;
	}
#endif
	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error) {
		dev_err(&client->dev, "Error %d initialising configuration\n",
			error);
		//don't return error whether config failed, just send a reset command
	}

	error = mxt_initialize_t100_input_device(data);
	if (error)
		return error;

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	error = mxt_plugin_init(data);
	if (error) {
		dev_err(&client->dev, "Error %d plugin init\n",
			error);
		return error;
	}
#endif

#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	data->properties_kobj = create_virtual_key_object(&client->dev);
#endif
	data->enable_reporting = true;

	dev_info(&client->dev, "configure objects finished\r\n");

	return 0;
}

static int mxt_t19_command(struct mxt_data *data, bool enable, bool wait)
{
	u16 reg;
	int timeout_counter = 0;
	int ret;
	u8  val[1];

	reg = data->T19_address;
	val[0] = 60;

	ret = __mxt_write_reg(data->client, reg + 3, sizeof(val), val);
	if (ret)
		return ret;

	val[0] = 7;
	ret = __mxt_write_reg(data->client, reg, sizeof(val), val);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(MXT_WAKEUP_TIME);
		ret = __mxt_read_reg(data->client, reg, 1, &val[0]);
		if (ret)
			return ret;
	} while ((val[0] & 0x4) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static ssize_t mxt_t19_gpio_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data)
		return -EIO;

	return scnprintf(buf, PAGE_SIZE, "%02x (GPIO 0x%02x)\n",
			 data->t19_msg[0],
			 (data->t19_msg[0]>>2) & 0xf);
}

static ssize_t mxt_t19_gpio_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u8 cmd;
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	char suffix[16];
	int ret;
#endif

	if (!data)
		return -EIO;

	if (sscanf(buf, "%hhd", &cmd) == 1) {
		if (cmd == 0) {
			data->alt_chip = 0;
			return count;
		}else if (cmd == 1) {
			if (mxt_t19_command(data,!!cmd,1) == 0) {
				data->alt_chip = cmd;
				return count;
			}
			dev_dbg(dev, "mxt_t19_cmd_store write cmd %x error\n",cmd);
		}else if (cmd == 2){
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
			suffix[0] = '\0';
			ret = mxt_plugin_get_pid_name(&data->plug, suffix, sizeof(suffix));
			if (ret == 0) {
				if (suffix[0] != '\0') {
					data->alt_chip = cmd;
					scnprintf(data->suffix_pid_name, sizeof(data->suffix_pid_name), "%s", suffix);
				}
			}
#endif
		}
		return -EINVAL;
	} else {
		dev_dbg(dev, "mxt_t19_cmd_store write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_irq_depth_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_platform_data *pdata;

	if (!data)
		return -EIO;

	pdata = data->pdata;

	return scnprintf(buf, PAGE_SIZE, "depth %d\n",
				 atomic_read(&pdata->depth));
}

static ssize_t mxt_irq_depth_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_platform_data *pdata;
	int cmd;

	if (!data)
		return -EIO;

	pdata = data->pdata;

	if (sscanf(buf, "%d", &cmd) == 1) {
		atomic_set(&pdata->depth,cmd);
	}

	return count;
}

static int mxt_t25_command(struct mxt_data *data, u8 cmd, bool wait)
{
	u16 reg;
	int timeout_counter = 0;
	int ret;
	u8  val[2];

	reg = data->T25_address;
	val[0] = 0x3;
	val[1] = cmd;

	ret = __mxt_write_reg(data->client, reg, sizeof(val), val);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(MXT_WAKEUP_TIME);
		ret = __mxt_read_reg(data->client, reg + 1, 1, &val[1]);
		if (ret)
			return ret;
	} while ((val[1] != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_t25_selftest_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	ssize_t offset = 0;

	if (!data)
		return -EIO;

	if (data->t25_msg[0] == 0xFE)
		offset += scnprintf(buf, PAGE_SIZE, "PASS\n");
	else
		offset += scnprintf(buf, PAGE_SIZE, "FAILED\n");

	offset += scnprintf(buf + offset, PAGE_SIZE, "%x %x %x %x %x %x\n",
		 data->t25_msg[0],
		 data->t25_msg[1],
		 data->t25_msg[2],
		 data->t25_msg[3],
		 data->t25_msg[4],
		 data->t25_msg[5]);
	return offset;
}

static ssize_t mxt_t25_selftest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u32 cmd;

	if (!data)
		return -EIO;

	if (sscanf(buf, "%x", &cmd) == 1) {
		if (mxt_t25_command(data,(u8)cmd,1) == 0)
			return count;

		dev_dbg(dev, "mxt_t25_cmd_store write cmd %x error\n",cmd);
		return -EINVAL;
	} else {
		dev_dbg(dev, "mxt_t25_cmd_store write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	u32 cmd;

	const char *command_list[] = {
		"# reset(por)",
		"# reset(hw)",
		"# reset (soft)",
		"# enable irq",
		"# disable irq"
		"msc ptc tune 3",
	};

	if (!data)
		return -EIO;

	dev_info(dev, "[mxt]%s\n",buf);

	if (sscanf(buf, "%d\n", &cmd) >= 1) {
		dev_info(dev, "[mxt] cmd %d (%zd): %s\n",cmd,ARRAY_SIZE(command_list),command_list[cmd]);
		if (cmd >=0 && cmd < ARRAY_SIZE(command_list)) {
			if (cmd == 0) {
				mxt_set_reset(data, 1);
			}else if (cmd == 1) {
				mxt_set_reset(data, 0);
			}else if (cmd == 2) {
				mxt_soft_reset(data);
			}else if (cmd == 3) {
				device_enable_irq(dev, __func__);
			}else if (cmd == 4) {
				device_disable_irq(dev, __func__);
			}else {
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
				mxt_plugin_store(dev, attr, command_list[cmd], strlen(command_list[cmd]));
#endif
			}
			return count;
		}
		return -EINVAL;
	} else {
		dev_dbg(dev, "mxt_t19_cmd_store write error\n");
		return -EINVAL;
	}
}


/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data)
		return -EIO;

	if (!data->info)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data)
		return -EIO;

	if (!data->info)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%02x.%02x\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
					"Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	if (!data)
		return -EIO;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
					 const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -1;
}

static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_platform_data *pdata;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;

	if (!data)
		return -EIO;

	pdata = data->pdata;

	if (!data->fw_name)
		return -EEXIST;

	dev_info(dev, "mxt_load_fw %s\n",data->fw_name);

	ret = request_firmware(&fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		return ret;
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (pdata->use_regulator)
			device_regulator_enable(dev);
	}

	if (data->in_bootloader) {
		ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
		if(ret) {
			dev_err(dev, "false bootloader check %d\n", ret);
			data->in_bootloader = false;
		}
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
					 MXT_BOOT_VALUE, false);
		if (ret) {
			dev_err(dev, "reset to boot loader mode return %d\n", ret);
			//don't return failed, maybe it's in bootloader mode
			//goto release_firmware;
		}
		msleep(MXT_RESET_TIME);

		mxt_lookup_bootloader_address(data, 0);

		/* At this stage, do not need to scan since we know
		 * family ID */
		do {
			ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
			if(ret == 0)
				break;
			dev_err(dev, "mxt_bootloader_read failed %d retry %d\n", ret,retry);
			mxt_lookup_bootloader_address(data, retry);
		}while(++retry <= 3);

		if (ret) {
			data->in_bootloader = false;
			goto release_firmware;
		}
	}

	device_free_irq(dev, data, __func__);

	mxt_free_object_table(data);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
	if (ret) {
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
		if (ret) {
			data->in_bootloader = false;
			goto disable_irq;
		}
	} else {
		dev_info(dev, "Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret) {
			data->in_bootloader = false;
			goto disable_irq;
		}
	}

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
		//print_hex_dump(KERN_INFO, "[mxt] ", DUMP_PREFIX_OFFSET, 16, 1, fw->data+pos, frame_size, false);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0)
			dev_info(dev, "Sent %d frames, %d/%zd bytes\n",
				 frame, pos, fw->size);
	}

	dev_info(dev, "Sent %ud frames, %ud bytes\n", frame, pos);

	/* Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore error */
	ret = device_wait_irq_state(dev, 0, MXT_FW_RESET_TIME);
	data->in_bootloader = false;

disable_irq:
	data->busy = 0;
release_firmware:
	release_firmware(fw);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count, unsigned long alternative)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char *file_name_tmp;
	char suffix[16];
	int size;
	u8 cmd = alternative & 0xF;
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	char *sc;
 	int ret;
#endif

	if (!data)
		return -EIO;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	size = count + 1 + sizeof(suffix);
	file_name_tmp = krealloc(*file_name,size , GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	if (cmd == 1) {
		snprintf(suffix, sizeof(suffix), ".%02X.%02X.cfg",
			(u8)(data->client->addr & 0x7f),
			(data->t19_msg[0]>>2) & 0xf);
		strncat((*file_name),suffix,size);
	}else if (cmd == 2){
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		suffix[0] = '\0';
		if (data->suffix_pid_name[0] == '\0') {
			ret = mxt_plugin_get_pid_name(&data->plug, suffix, sizeof(suffix));
			if (ret == 0) {
				if (suffix[0] != '\0') {
					scnprintf(data->suffix_pid_name, sizeof(data->suffix_pid_name), "%s", suffix);
				}
			}
		}else {
			strncpy(suffix, data->suffix_pid_name, sizeof(suffix));
		}

		if (suffix[0] != '\0') {
			sc = strpbrk(suffix, ".");
			if (test_flag(ALT_CHIP_BIT_FW, &alternative)) {
				if (sc) {
					if (*sc != '\0') {
						strncat((*file_name), sc, size);
					}
				}
			}else {
				if(sc) {
					if (sc != suffix) {  //may no cfg name
						*sc = '\0';
						strncat((*file_name), ".", size);
						strncat((*file_name), suffix, size);
					}
				}else { // all is cfg name
					strncat((*file_name), ".", size);
					strncat((*file_name), suffix, size);
				}
			}
		}
#endif
	}

	return 0;
}

static int mxt_update_cfg_name_by_fw_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	if (count > 3) {
		if (!strcmp(&buf[count - 3], ".fw"))
			count -= 3;
	}

	file_name_tmp = krealloc(*file_name, count + 1 + 4, GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);
	memcpy(*file_name + count, ".cfg", 4);
	count += 4;

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

int mxt_check_firmware_version(struct mxt_data *data, const char *version_str)
{
	struct device *dev = &data->client->dev;
	char firmware_version[64];
	u8 family_id,variant_id,version,version2,build;

	if (data->info == NULL)
		return 0;

	snprintf(firmware_version, 64, "%02X_%02X_%u.%u_%02X.fw",
			data->info->family_id,
			data->info->variant_id,
			(data->info->version & 0xF0) >> 4,
			(data->info->version & 0x0F),
			data->info->build);

	dev_info(dev, "[mxt] firmware version %s - %s\n",version_str, firmware_version);

	if (!strncmp(firmware_version, version_str,strlen(firmware_version)))
		return -EEXIST;

	family_id = data->info->family_id;
	variant_id = data->info->variant_id;
	if(sscanf(version_str,"%02hhX_%02hhX_%hhu.%hhu_%02hhX.fw",
		&family_id,
		&variant_id,
		&version,
		&version2,
		&build) >= 2) {
		if(data->info->family_id != family_id ||
			data->info->variant_id != variant_id) {
			dev_info(dev, "Check chip version mismatch: %02X %02X %u.%u.%02X\n",
				family_id, variant_id,version,version2,build);
			return -ENXIO;
		}
	}

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	if (!data)
		return -EIO;

	dev_info(dev, "mxt_update_fw_store\n");

	error = mxt_update_file_name(dev, &data->fw_name, buf, count, /*false*/data->alt_chip | ALT_CHIP_BIT_FW);
	if (error)
		return error;

	error = mxt_check_firmware_version(data,data->fw_name);
	if (error)
		return error;

	error = mxt_update_cfg_name_by_fw_name(dev, &data->cfg_name, data->fw_name, strlen(data->fw_name));
	if (error)
		return error;

	//lock it for disable outside access
	mutex_lock(&data->access_mutex);
	error = mxt_load_fw(dev);
	mutex_unlock(&data->access_mutex);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = mxt_initialize(data);
		if (error)
			return error;

		error = mxt_acquire_irq(data);
		if (error)
			return error;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_platform_data *pdata;
	int ret;

	if (!data)
		return -EIO;
	pdata = data->pdata;

	dev_info(dev, "mxt_update_cfg_store\n");

	if (data->in_bootloader) {
		dev_err(dev, "Not in appmode\n");
		return -EINVAL;
	}

	if (!data->object_table) {
		dev_err(dev, "Not initialized\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count, data->alt_chip);
	if (ret)
		return ret;

	data->enable_reporting = false;

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_force_stop(&data->plug);
	mxt_plugin_deinit(&data->plug);
#endif

	if (data->suspended) {
		if (pdata->use_regulator)
			device_regulator_enable(dev);
	}

	ret = mxt_configure_objects(data);
	if (ret)
		goto out;

	ret = count;

out:
	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	if (!data)
		return -EIO;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	if (!data)
		return -EIO;

	if (data->debug_msg_data)
		return sprintf(buf, "%d\n", data->debug_msg_count);
	else
		return sprintf(buf, "disable\n");

	return sprintf(buf, "1\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (!data)
		return -EIO;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		return count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (!data)
		return -EIO;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_bootloader_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	if (!data)
		return -EIO;

	c = data->in_bootloader ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_bootloader_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (!data)
		return -EIO;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->in_bootloader = (i == 1);

		dev_dbg(dev, "%s\n", i ? "in bootloader" : "app mode");
		return count;
	} else {
		dev_dbg(dev, "in_bootloader write error\n");
		return -EINVAL;
	}
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
						size_t *count, int max_size)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > max_size)
		*count = max_size;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (!data)
		return -EIO;

	ret = mxt_check_mem_access_params(data, off, &count, MXT_MAX_BLOCK_READ);
	if (ret < 0)
		return ret;

	mutex_lock(&data->access_mutex);

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	mutex_unlock(&data->access_mutex);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (!data)
		return -EIO;

	ret = mxt_check_mem_access_params(data, off, &count, MXT_MAX_BLOCK_WRITE);
	if (ret < 0)
		return ret;

	mutex_lock(&data->access_mutex);

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	mutex_unlock(&data->access_mutex);

	return ret == 0 ? count : 0;
}

#if defined (CONFIG_ZTEMT_TOUCHSCREEN_SYSFS)
/* these interfaces can be moved to custum files like plugin.if */
static ssize_t mxt_manual_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int rc = 0;
    //rc =mxt_cmd_store(dev, attr, "5", 1);   //msc ptc tune 5
    //this touch pad no need to do cali, just keep this interface
    dev_info(dev, "mxt_manual_cali_show write, ret=%d\n",rc);
    if(rc < 0)
        return -EINVAL;

    msleep(300);
    return snprintf(buf, PAGE_SIZE, "0\n");//PASS
}

static ssize_t mxt_ic_detect_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	ssize_t offset = 0;
    int rc = 0;
    rc = mxt_t25_selftest_store(dev, attr, "0xFE", 4);
    dev_info(dev, "mxt_t25_selftest_store write, ret=%d\n", rc);
    if(rc < 0)
		return -EINVAL;

    msleep(300);

	if (data->t25_msg[0] == 0xFE)
		offset += scnprintf(buf, PAGE_SIZE, "0\n");//PASS
	else
		offset += scnprintf(buf, PAGE_SIZE, "1\n");//FAILED
	return offset;
}

static ssize_t mxt_easy_wakeup_gesture_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    //struct mxt_data *data = dev_get_drvdata(dev);
   // struct plug_interface *pl = &data->plug;
    int rc = 0;
    u32 cmd;
    int i;
    //dev_info(dev, "set mxt_easy_wakeup_gesture \n");

#define RE_TRY 2
	if (sscanf(buf, "%x", &cmd) == 1) {
            dev_info(dev, "mxt_easy_wakeup_gesture_store write cmd %x \n",cmd);
		if(cmd == 1){
                for( i = 0; i < RE_TRY ; i++ )
                {
                    rc = mxt_plugin_gesture_list_store(dev, attr, "TAP 1;", 6);
                    if( rc > 0 ) break;
                    msleep(50);
                 }
                dev_info(dev, "mxt_plugin_gesture_list_store return %d  \n",rc);
                if( rc < 0 ) return rc;
                msleep(500);

                for( i = 0; i < RE_TRY ; i++ )
                {
                    rc = mxt_plugin_wakeup_gesture_store(dev, attr, "1", 1);
                    if( rc > 0 ) break;
                    msleep(100);
                }
                dev_info(dev, "mxt_plugin_wakeup_gesture_store return %d  \n",rc);
                if( rc < 0 ) return rc;
        }else{
                for( i = 0; i < RE_TRY ; i++ )
                {
                    rc = mxt_plugin_gesture_list_store(dev, attr, "TAP 2;", 6);
                    if( rc > 0 ) break;
                    msleep(50);
                 }
                dev_info(dev, "mxt_plugin_gesture_list_store return %d  \n",rc);
                if( rc < 0 ) return rc;
                msleep(500);

                for( i = 0; i < RE_TRY ; i++ )
                {
                    rc = mxt_plugin_wakeup_gesture_store(dev, attr, "0", 1);
                    if( rc > 0 ) break;
                    msleep(100);
                }
                dev_info(dev, "mxt_plugin_wakeup_gesture_store return %d  \n",rc);
                if( rc < 0 ) return rc;
        }
        return count;
	} else {
		dev_info(dev, "mxt_easy_wakeup_gesture_store cmd format err\n");
		return -EINVAL;
	}
}


#if defined ( NUBIA_PALM_KEY_CAN_DISABLE )
static ssize_t mxt_easy_sleep_palm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    u32 cmd;

    if (!data ) return -ENODEV;
    dev_info(dev, "mxt_easy_sleep_palm_store %s \n",buf);
    if (sscanf(buf, "%x", &cmd) == 1) {
        cmd = !!cmd;
       if(cmd)  data->nubia_pk_enable = 1;
       else  data->nubia_pk_enable = 0;
       return count;
    }
    return -EINVAL;
}
#endif

static int tm_value = -1;
static ssize_t mxt_touch_mode_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",tm_value);
}

static ssize_t mxt_touch_mode_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int inv = -1,ret = -1;
    ret = sscanf(buf,"%d",&inv);
    if( ret == 1 )
    {
       switch(inv)
       {
           case 0:
           case 1:
               ret = mxt_plugin_glove_store(dev,attr,"0",1);
               if( ret == 1)
                   tm_value = inv ;
               break;
           case 2:
               ret = mxt_plugin_glove_store(dev,attr,"1",1);
               if ( ret == 1 )
                   tm_value = inv ;
                break;
           default:
                break;
       }
    }
    dev_info(dev, "set touch_mode in: %d, ret: %d, tm: %d \n", inv, ret, tm_value);

    return count;
}
#endif
static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, mxt_debug_v2_enable_show, mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
			mxt_debug_enable_store);
static DEVICE_ATTR(bootloader, S_IWUSR | S_IRUSR, mxt_bootloader_show,
			mxt_bootloader_store);
static DEVICE_ATTR(t19, S_IWUSR | S_IRUSR, mxt_t19_gpio_show,
			mxt_t19_gpio_store);
static DEVICE_ATTR(t25, S_IWUSR | S_IRUSR, mxt_t25_selftest_show,
			mxt_t25_selftest_store);
static DEVICE_ATTR(cmd, S_IWUSR, NULL,
			mxt_cmd_store);
static DEVICE_ATTR(depth, S_IWUSR | S_IRUSR, mxt_irq_depth_show,
			mxt_irq_depth_store);
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
static DEVICE_ATTR(plugin, S_IWUSR | S_IRUSR, mxt_plugin_show,
			mxt_plugin_store);
static DEVICE_ATTR(plugin_tag, S_IRUGO, mxt_plugin_tag_show,
			NULL);
#	if defined(CONFIG_MXT_PLIGIN_CAL)
	static DEVICE_ATTR(cal, S_IWUSR | S_IRUSR, mxt_plugin_cal_show,
				mxt_plugin_cal_store);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_PI)
static DEVICE_ATTR(en_glove, S_IWUSR | S_IRUSR, mxt_plugin_glove_show,
			mxt_plugin_glove_store);
static DEVICE_ATTR(en_stylus, S_IWUSR | S_IRUSR, mxt_plugin_stylus_show,
			mxt_plugin_stylus_store);
static DEVICE_ATTR(en_gesture, S_IWUSR | S_IRUSR, mxt_plugin_wakeup_gesture_show,
			mxt_plugin_wakeup_gesture_store);
static DEVICE_ATTR(gesture_list, S_IWUSR | S_IRUSR, mxt_plugin_gesture_list_show,
			mxt_plugin_gesture_list_store);
static DEVICE_ATTR(gesture_trace, S_IRUSR, mxt_plugin_gesture_trace_show,
			NULL);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_MISC)
	static DEVICE_ATTR(misc, S_IWUSR | S_IRUSR, mxt_plugin_misc_show,
				mxt_plugin_misc_store);
#	endif
#	if defined(CONFIG_MXT_PLIGIN_CLIP)
	static DEVICE_ATTR(clip, S_IWUSR | S_IRUSR, mxt_plugin_clip_show,
				mxt_plugin_clip_store);
	static DEVICE_ATTR(clip_tag, S_IRUSR, mxt_plugin_clip_tag_show, NULL);
#	endif

#	if defined(CONFIG_MXT_PLIGIN_AC)
static DEVICE_ATTR(ac, S_IWUSR | S_IRUSR, mxt_plugin_ac_extern_event_show,
			mxt_plugin_ac_extern_event_store);
#	endif
#endif

#if defined (CONFIG_ZTEMT_TOUCHSCREEN_SYSFS)
static DEVICE_ATTR(ic_detect, S_IRUSR|S_IRGRP|S_IROTH, mxt_ic_detect_show,NULL);
static DEVICE_ATTR(manual_cali, S_IRUSR|S_IRGRP|S_IROTH, mxt_manual_cali_show,NULL);
static DEVICE_ATTR(easy_wakeup_gesture, S_IWUSR|S_IWGRP, NULL, mxt_easy_wakeup_gesture_store);
#if defined( NUBIA_PALM_KEY_CAN_DISABLE )
static DEVICE_ATTR(easy_sleep_palm, S_IWUSR|S_IWGRP, NULL, mxt_easy_sleep_palm_store);
#endif
static DEVICE_ATTR(touch_mode, S_IRUSR| S_IWUSR | S_IRGRP | S_IWGRP, mxt_touch_mode_show, mxt_touch_mode_store);
#endif

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_bootloader.attr,
	&dev_attr_t19.attr,
	&dev_attr_t25.attr,
	&dev_attr_cmd.attr,
	&dev_attr_depth.attr,
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	&dev_attr_plugin.attr,
	&dev_attr_plugin_tag.attr,
#	if defined(CONFIG_MXT_PLIGIN_CAL)
	&dev_attr_cal.attr,
#     endif
#	if defined(CONFIG_MXT_PLIGIN_PI)
	&dev_attr_en_glove.attr,
	&dev_attr_en_stylus.attr,
	&dev_attr_en_gesture.attr,
	&dev_attr_gesture_list.attr,
	&dev_attr_gesture_trace.attr,
#	endif
#	if defined(CONFIG_MXT_PLIGIN_MISC)
	&dev_attr_misc.attr,
#     endif
#	if defined(CONFIG_MXT_PLIGIN_CLIP)
	&dev_attr_clip.attr,
	&dev_attr_clip_tag.attr,
#     endif
#	if defined(CONFIG_MXT_PLIGIN_AC)
	&dev_attr_ac.attr,
#	endif
#endif
#if defined (CONFIG_ZTEMT_TOUCHSCREEN_SYSFS)
	&dev_attr_ic_detect.attr,
	&dev_attr_easy_wakeup_gesture.attr,
#if defined ( NUBIA_PALM_KEY_CAN_DISABLE )
	&dev_attr_easy_sleep_palm.attr,
#endif
	&dev_attr_manual_cali.attr,
	&dev_attr_touch_mode.attr,
#endif
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;

	if (!input_dev)
		return;

	num_mt_slots = data->num_touchids + data->num_stylusids;
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_hook_reset_slots(&data->plug);
#endif
	for (id = 0; id < num_mt_slots; id++) {
		input_mt_slot(input_dev, id);
		if (test_flag_8bit(MXT_T100_TCHAUX_AMPL, &data->tchcfg[MXT_T100_TCHAUX]))
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 0);  //add for point release
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	mxt_input_sync(input_dev);

}

static void mxt_start(struct mxt_data *data, bool resume)
{
	struct device *dev = &data->client->dev;
	struct mxt_platform_data *pdata = data->pdata;
	int ret = 0;

	dev_info(dev, "mxt_start %d %d\n",
		data->suspended,
		data->in_bootloader);

	if (!data->suspended || data->in_bootloader)
		return;

	dev_info(dev, "mxt_start\n");

	if (pdata->use_regulator) {
		device_regulator_enable(dev);
	} else {
		/* Discard any messages still in message buffer from before
		 * chip went to sleep */
		if (test_bit(MXT_WK_ENABLE,&data->enable_wakeup)) {
			if (test_and_clear_bit(MXT_EVENT_IRQ_FLAG,&data->busy)) {
				mxt_process_messages_until_invalid(data);
				if (!test_bit(MXT_WK_DETECTED,&data->enable_wakeup)) {
					dev_info(dev, "detect a invalid wakeup signal\n");
					//mxt_acquire_irq(data);
#if !(defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB))
					return;
#endif
				}
			}
		}
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
		ret = mxt_plugin_wakeup_disable(&data->plug);
#endif
		mxt_process_messages_until_invalid(data);

		//mxt_set_reset(data, 0);
		mxt_soft_reset(data);
	}

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_start(&data->plug, resume);
#endif

	mxt_reset_slots(data);  //it's strange some platform will resverse some points in last touch

	data->enable_wakeup = 0;
	data->enable_reporting = true;
	data->suspended = false;

	dev_info(dev, "ret %d\n", ret);
	if (ret != -EBUSY) {
		device_enable_irq(dev, __func__);
	}else{
		device_disable_irq_wake(dev);
	}
}

static void mxt_stop(struct mxt_data *data,bool suspend)
{
	struct device *dev = &data->client->dev;
	struct mxt_platform_data *pdata = data->pdata;
	int ret = 0;

	if (data->suspended || data->in_bootloader)
		return;

	dev_info(dev, "mxt_stop\n");

	device_disable_irq(dev, __func__);

	data->enable_reporting = false;

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
	mxt_plugin_stop(&data->plug,suspend);
#endif

	if (pdata->use_regulator)
		device_regulator_disable(dev);
	else{
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);

		if (suspend) {
#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
			ret = mxt_plugin_wakeup_enable(&data->plug);
#endif
			if (ret == -EBUSY) {
				dev_info(dev, "mxt_stop: set wakeup enable\n");
				mxt_process_messages_until_invalid(data);
				device_enable_irq(dev, __func__);
				device_enable_irq_wake(dev);
				set_bit(MXT_WK_ENABLE,&data->enable_wakeup);
				clear_bit(MXT_EVENT_IRQ_FLAG,&data->busy);
			}
		}
	}
	mxt_reset_slots(data);

	data->suspended = true;
}

static int mxt_input_open(struct input_dev *input_dev)
{
	struct mxt_data *data = input_get_drvdata(input_dev);
	struct device *dev = &data->client->dev;

	dev_info(dev, "mxt_input_open\n");

	mxt_start(data,false);

	return 0;
}

static void mxt_input_close(struct input_dev *input_dev)
{
	struct mxt_data *data = input_get_drvdata(input_dev);
	struct device *dev = &data->client->dev;

	dev_info(dev, "mxt_input_close\n");

	mxt_stop(data,false);
}

static int mxt_handle_pdata(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	struct mxt_platform_data *pdata;

	pdata = dev_get_platdata(dev);
	dev_dbg(dev, "mxt pdata %p\n", pdata);
	if (!pdata) {
		dev_info(dev, "mxt alloc pdata\n");
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "Failed to allocate pdata\n");
			return -ENOMEM;
		}
		pdata->mem_allocated = true;
		dev->platform_data = pdata;
	}
	dev_info(dev, "client irq %d\n", client->irq);

	pdata->irq = client->irq;
	data->pdata = pdata;

	return device_parse_default_chip(data, dev);
}

static void mxt_free_pdata(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_platform_data *pdata = data->pdata;

	device_release_chip(dev);

	if (pdata) {
		if(pdata->mem_allocated) {
			dev_info(dev, "mxt free pdata\n");
			kfree(data->pdata);
			data->pdata = NULL;
			dev->platform_data = NULL;
		}
	}
}

int mxt_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct mxt_data *data;
	int error;

	dev_info(dev, "%s: driver version 0x%x\n",
			__func__,DRIVER_VERSION);

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->bus_access_mutex);
#if defined(CONFIG_MXT_I2C_DMA)
	client->addr |= MXT_I2C_DMA_ADDR_FLAG;
	dev->coherent_dma_mask = DMA_BIT_MASK(32);
	data->dma_buffer_size = (MXT_DMA_BUFFER_SIZE << 1);
	data->i2c_dma_va = (u8 *)dma_alloc_coherent(dev, data->dma_buffer_size, &data->i2c_dma_pa, GFP_KERNEL);
	if (!data->i2c_dma_va) {
		error = -ENOMEM;
		dev_err(dev, "Allocate DMA I2C Buffer failed!\n");
		goto err_free_mem;
	}
#endif

	error = mxt_handle_pdata(data);
	if (error) {
		dev_err(dev, "mxt handle pdata failed!\n");
		goto err_free_pdata;
	}

	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);
	mutex_init(&data->access_mutex);

#if defined(CONFIG_MXT_IRQ_WORKQUEUE)
	init_waitqueue_head(&data->wait);
	data->tsk_handle_msg = kthread_run(mxt_process_message_thread, dev,
						"Atmel_mxt_ts");
	if (!data->tsk_handle_msg) {
		dev_err(dev, "Error %d Can't create handle message thread\n",
			error);
		error = -ESRCH;
		goto err_free_workqueue;
	}
#endif
	error = mxt_initialize(data);
	if (error)
		goto err_free_object;

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object;

	error = sysfs_create_group(&dev->kobj, &mxt_attr_group);
	if (error) {
		dev_err(dev, "Failure %d creating sysfs group\n",
			error);
		goto err_free_irq;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRWXUGO /*S_IRUGO | S_IWUSR*/;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&dev->kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&data->fb_notif);
	if (error) {
		dev_err(dev,
			"Unable to register fb_notifier: %d\n",
			error);
		goto err_remove_mem_access_attr;
	}

#if defined(NUBIA_DEFERRED_RESUME_TP)
	nubia_tp_resume_work_init(&data->nubia_tp_resume_work);
#endif

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	dev_info(dev, "Mxt probe finished\n");

	return 0;

#if defined(CONFIG_FB)
err_remove_mem_access_attr:
	sysfs_remove_bin_file(&dev->kobj,
				  &data->mem_access_attr);
#endif
err_remove_sysfs_group:
	sysfs_remove_group(&dev->kobj, &mxt_attr_group);
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
		if (data->properties_kobj) {
			kobject_put(data->properties_kobj);
			data->properties_kobj = NULL;
		}
#endif
err_free_irq:
	device_free_irq(dev, data, __func__);
err_free_object:
	mxt_free_object_table(data);
#if defined(CONFIG_MXT_IRQ_WORKQUEUE)
err_free_workqueue:
	if (data->tsk_handle_msg) {
		kthread_stop(data->tsk_handle_msg);
		data->tsk_handle_msg = NULL;
	}
#endif
	mutex_destroy(&data->access_mutex);
	mutex_destroy(&data->debug_msg_lock);
err_free_pdata:
	mxt_free_pdata(data);
#if defined(CONFIG_MXT_I2C_DMA)
	if (data->i2c_dma_va) {
		dma_free_coherent(dev, data->dma_buffer_size, data->i2c_dma_va, data->i2c_dma_pa);
		data->i2c_dma_va = NULL;
	}
err_free_mem:
#endif
	kfree(data);
	i2c_set_clientdata(client, NULL);
	return error;
}

int mxt_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mxt_data *data = i2c_get_clientdata(client);

	dev_info(dev, "mxt_remove\n");

#if defined(CONFIG_FB)
	fb_unregister_client(&data->fb_notif);

#if defined(NUBIA_DEFERRED_RESUME_TP)
	nubia_tp_resume_work_deinit(&data->nubia_tp_resume_work);
#endif

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
#if defined(CONFIG_MXT_IRQ_WORKQUEUE)
	if (data->tsk_handle_msg) {
		kthread_stop(data->tsk_handle_msg);
		data->tsk_handle_msg = NULL;
	}
#endif
	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&dev->kobj,
					  &data->mem_access_attr);

	sysfs_remove_group(&dev->kobj, &mxt_attr_group);
#if defined(CONFIG_MXT_I2C_DMA)
	if (data->i2c_dma_va) {
		dma_free_coherent(dev, data->dma_buffer_size, data->i2c_dma_va, data->i2c_dma_pa);
		data->i2c_dma_va = NULL;
	}
#endif
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	if (data->properties_kobj) {
		kobject_del(data->properties_kobj);
		data->properties_kobj = NULL;
	}
#endif
	mxt_free_object_table(data);
	mutex_destroy(&data->access_mutex);
	mutex_destroy(&data->debug_msg_lock);

	device_free_irq(dev, data, __func__);
	mxt_free_pdata(data);

	kfree(data);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#if defined(CONFIG_PM_SLEEP)
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(dev, "mxt_suspend\n");

	if (!input_dev)  //maybe bootup in bootloader mode
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data,true);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(dev, "mxt_resume\n");

	if (!input_dev)  //maybe bootup in bootloader mode
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data,true);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct mxt_data *mxt =
		container_of(self, struct mxt_data, fb_notif);
	int *blank;
	int ret;

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
				mxt && mxt->client) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
#if defined (NUBIA_DEFERRED_RESUME_TP)
				dev_info(&mxt->client->dev, "%s: ztemt atmel FB_BLANK_UNBLANK, deferred\n", __func__);
				if (!work_pending(&mxt->nubia_tp_resume_work))
					schedule_work(&mxt->nubia_tp_resume_work);
#else
				ret = mxt_resume(&mxt->client->dev);
				if (ret)
					dev_err(&mxt->client->dev, "%s: resume failed %d\n", __func__, ret);
#endif
			}else if (*blank == FB_BLANK_POWERDOWN) {
#if defined (NUBIA_DEFERRED_RESUME_TP)
				nubia_tp_resume_work_deinit(&mxt->nubia_tp_resume_work);
#endif
				ret = mxt_suspend(&mxt->client->dev);
				if (ret)
					dev_err(&mxt->client->dev, "%s: suspend failed %d\n", __func__, ret);
			}
	}

	return 0;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt = container_of(es, struct mxt_data, early_suspend);
	int ret;

	ret = mxt_suspend(&mxt->client->dev);
	if (ret)
		dev_err(&mxt->client->dev, "%s: failed %d\n", __func__, ret);
}

static void mxt_late_resume(struct early_suspend *es)
{
	struct mxt_data *mxt = container_of(es, struct mxt_data, early_suspend);
	int ret;

	ret = mxt_resume(&mxt->client->dev);
	if (ret)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__, ret);
}
#else
SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);
#endif
#endif

void mxt_shutdown(struct i2c_client *client)
{
	device_disable_irq(&client->dev, __func__);
}

#if defined(CONFIG_MXT_PLUGIN_SUPPORT)
#include "plugin/plug.if"
#endif

#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
#include "virtualkey.if"
#endif

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

