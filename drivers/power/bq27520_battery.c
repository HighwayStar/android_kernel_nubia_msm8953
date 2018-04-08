/*
 * BQ27520 battery driver
 *
 * Copyright (C) 2013¡rwangshuai <wang.shuai12@zte.com.cn>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#define pr_fmt(fmt)	"FG:%s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#define BQ27520_UPDATER

#ifdef BQ27520_UPDATER
#include "nubia-bq-batterydata-3560mAh.h"
#endif 

//NUBIA_BATT
#include "nubia_fuelgauge_cntl.h"

#define BQ27520_NAME		"bq27520_battery"

/* Bq27520 standard data commands */
#define BQ27520_REG_CNTL		0x00
#define BQ27520_REG_AR			0x02
#define BQ27520_REG_ARTTE		0x04
#define BQ27520_REG_TEMP		0x06
#define BQ27520_REG_VOLT		0x08
#define BQ27520_REG_FLAGS		0x0A
#define BQ27520_REG_NAC			0x0C
#define BQ27520_REG_FAC			0x0e
#define BQ27520_REG_RM			0x10
#define BQ27520_REG_FCC			0x12
#define BQ27520_REG_AI			0x14
#define BQ27520_REG_TTE			0x16
#define BQ27520_REG_SI			0x18
#define BQ27520_REG_STTE	    0x1a
#define BQ27520_REG_SOH		    0x1c
#define BQ27520_REG_CC			0x1e
#define BQ27520_REG_SOC		    0x20
#define BQ27520_REG_INSTC	    0x22
#define BQ27520_REG_INTTEMP		0x28
#define BQ27520_REG_RSCL	    0x2a
#define BQ27520_REG_OPCFIG		0x2c
#define BQ27520_REG_DESCAP		0x2e
#define BQ27520_REG_UFRM		0x6c
#define BQ27520_REG_FRM 		0x6e
#define BQ27520_REG_DFFCC		0x70
#define BQ27520_FLAG_FFCC		0x72
#define BQ27520_FLAG_UFSOC		0x74

#define BQ27520_REG_DFCLASS         0x3E
#define BQ27520_REG_DFBLOCK         0x3F
#define BQ27520_REG_BLOCKDATA       0x40

#define BQ27520_REG_BLKCHKSUM       0x60
#define BQ27520_REG_BLKDATCTL       0x61

/* Control subcommands */
#define BQ27520_SUBCMD_CTNL_STATUS  0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27520_SUBCMD_FW_VER       0x0002
#define BQ27520_SUBCMD_HW_VER       0x0003
#define BQ27520_SUBCMD_DF_CSUM      0x0004
#define BQ27520_SUBCMD_PREV_MACW    0x0007
#define BQ27520_SUBCMD_CHEM_ID      0x0008
#define BQ27520_SUBCMD_BD_OFFSET    0x0009
#define BQ27520_SUBCMD_INT_OFFSET   0x000a
#define BQ27520_SUBCMD_CC_VER       0x000b
#define BQ27520_SUBCMD_OCV          0x000c
#define BQ27520_SUBCMD_BAT_INS      0x000d
#define BQ27520_SUBCMD_BAT_REM      0x000e
#define BQ27520_SUBCMD_SET_HIB      0x0011
#define BQ27520_SUBCMD_CLR_HIB      0x0012
#define BQ27520_SUBCMD_SET_SLP      0x0013
#define BQ27520_SUBCMD_CLR_SLP      0x0014
#define BQ27520_SUBCMD_FCT_RES      0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27520_SUBCMD_SEALED       0x0020
#define BQ27520_SUBCMD_ENABLE_IT    0x0021
#define BQ27520_SUBCMD_DISABLE_IT   0x0023
#define BQ27520_SUBCMD_CAL_MODE     0x0040
#define BQ27520_SUBCMD_RESET        0x0041
#define BQ27520_SUBCMD_ENTER_ROM    0x0F00


#define BQ27520_SECURITY_SEALED     0x03
#define BQ27520_SECURITY_UNSEALED   0x02
#define BQ27520_SECURITY_FA         0x01
#define BQ27520_SECURITY_MASK       0x03

#define BQ27520_UNSEAL_KEY          0x36720414
#define BQ27520_FA_KEY              0xFFFFFFFF

#define I2C_RETRY_CNT    3
#define BQGAUGE_I2C_ROM_ADDR    (0x16 >> 1)
#define BQGAUGE_I2C_DEV_ADDR    (0xAA >> 1)

#define RETRY_CNT		3
#define LEARNED_PARAM_LEN	50
#define CUST_MODE_LEN		48

#define START_MONITOR_MS	3000

#define BQFG_SHUTDOWN_VALID_LIMIT  15
#define BQFG_DEFAULT_SOC    100
#define BQ_DEFAULT_TEMP		250
#define BQ_BATT_MIN_TEMP   -300

//uart debug
#define ZTEMT_UART_DEBUG_ENABLE

#define FG_INFO 1
#define FG_DEBUG 4
//log level < bqfg_log_level will show
static int bqfg_log_level = 3;  
module_param(bqfg_log_level, int, 0644);

#define FGLOG_DEBUG(x...) do {if (FG_DEBUG < bqfg_log_level) pr_info(x); } while (0)
#define FGLOG_INFO(x...)  do {if (FG_INFO  < bqfg_log_level) pr_info(x); } while (0)

enum bq_batt_func {
	BQ_PROP_BATT_STATUS = 0,
	BQ_PROP_BATT_HEALTH,
	BQ_PROP_BATT_PRESENT,
	BQ_PROP_CHG_TYPE,
	BQ_PROP_CHG_ENABLE,
};

enum battery_type_id {
	BATT_ID_SDI = 470,  //Samsung
	BATT_ID_ATL = 15,   //ATL
	BATT_ID_UNKOWN = 0,
};

struct bq27520_chip {
	struct device		  *dev;
	struct i2c_client     *i2c;
	struct work_struct     soc_work;
	struct power_supply	  *batt_psy;
	struct power_supply	  *usb_psy;
	struct power_supply	  *bms_psy;
	struct power_supply	   bq27520_batt_psy;
	struct device_node    *dev_node;
	struct dentry		*dent;
	struct pinctrl      *pinctrl;
	struct pinctrl_state   *gpio_state_active;	
	struct pinctrl_state   *gpio_state_suspend;
	struct qpnp_vadc_chip  *vadc_dev;
	struct wake_lock soc_wlock;
	char *bqfs_version;
	const bqfs_cmd_t *bqfs_image;
	int bqfs_image_size;
	unsigned long suspend_at_tm;
	unsigned long sleep_secs;
	int low_vol_thrhld;
	int low_int_gpio;
	int soc_int_gpio;
	int soc_irq;
	int low_irq;
	int batt_soc;
	int vbatt_mv;
	int shutdown_soc;
	int shutdown_vbatt;
	int batt_status;
	int batt_health;
	int batt_id;
	char *battery_type;
	bool batt_por;
	bool is_sleep;
	int batt_temp;

    //NUBIA_BATT
	struct nubia_fg_cntl *batt_cntl;
	int batt_flags;
	int rm_mah;
	int full_mah;
	int qmax_mah;
	int cntl_status;
};

static struct bq27520_chip   *bqfg_chip = NULL;

enum power_supply_property bq27520_batt_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
};

static int bq27520_update_bqfs(struct bq27520_chip *chip);
static int bq27520_unseal(struct bq27520_chip *chip);
static bool bq27520_check_rom_mode(struct bq27520_chip *chip);
extern int qpnp_read_shutdown_ocv_soc(int *soc, int *ocv_mv);

static DEFINE_MUTEX(battery_mutex);

/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/
static int bq27520_i2c_readb(struct bq27520_chip *chip, u8 reg, u8 *val)
{
	int ret;
	int i;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_read_byte_data(chip->i2c, reg);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);

	if(ret < 0){
        pr_err("fail to read reg[0x%x],ret=%d\n",reg,ret);
        return ret;
	}
		
	*val = ret;

	pr_debug("reg[%x]=0x%x,ret=%d\n",reg,*val,ret);

	return ret;
}

static int bq27520_i2c_writeb(struct bq27520_chip *chip, u8 reg, u8 val)
{
	int ret;
	int i;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_write_byte_data(chip->i2c, reg, val);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);

	if(ret < 0){
        pr_err("fail to write reg[0x%x],ret=%d\n",reg,ret);
        return ret;
	}

	return ret;
}

static inline int bq27520_i2c_write_word(struct bq27520_chip *chip, u8 reg, u16 val)
{
    int ret;
	int i;

	val = __cpu_to_le16(val);

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_write_i2c_block_data(chip->i2c, reg, sizeof(u16), (u8 *)&val);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);
	
	if(ret < 0)
		pr_err("fail to write reg[0x%x],ret=%d\n",reg,ret);

	return ret;
}

static inline int bq27520_i2c_read_word(struct bq27520_chip *chip, u8 reg, u16 *val)
{
	int ret;
	u16 tmp;
	int i;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_smbus_read_i2c_block_data(chip->i2c, reg, sizeof(u16), (u8 *)&tmp);
        if(ret >= 0) 
			break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);

	if(ret < 0)
		pr_err("fail to read reg[0x%x],ret=%d\n",reg,ret);
	
	*val = __le16_to_cpu(tmp);

	pr_debug("reg[%x]=0x%x,ret=%d\n",reg,*val,ret);

	return ret;
}

static inline int bq27520_read_i2c_blk(struct bq27520_chip *chip, u8 reg, u8 *data, u8 len)
{

    struct i2c_client *client = chip->i2c;
    struct i2c_msg msg[2];
    int ret;
    int i = 0;

    if (!client->adapter)
        return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = data;
    msg[1].len = len;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);    

    if (ret < 0)
        return ret;

    return ret;
}

static inline int bq27520_write_i2c_blk(struct bq27520_chip *chip, u8 reg, u8 *data, u8 sz)
{
    struct i2c_client *client = chip->i2c;
    struct i2c_msg msg;
    int ret;
    int i = 0;
    u8 buf[200];

    if (!client->adapter)
        return -ENODEV;

    buf[0] = reg;
    memcpy(&buf[1], data, sz);

    msg.buf = buf;
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = sz + 1;

    mutex_lock(&battery_mutex);
    for(i = 0; i < I2C_RETRY_CNT; i++){
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret >= 0) break;
        msleep(5);
    }
    mutex_unlock(&battery_mutex);
    if (ret < 0)
        return ret;

    return 0;
}

static inline int bq27520_write_verify_reg(struct bq27520_chip *bq27520, u8 reg, u16 val)
{
	int i;
	int ret;
	u16 tmp;

	for (i = 0; i < RETRY_CNT; i++) {
		ret = bq27520_i2c_write_word(bq27520, reg, val);
		if (unlikely(ret < 0))
			return ret;
			
		ret = bq27520_i2c_read_word(bq27520, reg, &tmp);
		if (unlikely(ret < 0))
			return ret;

		if (likely(val == tmp))
			return 0;
	}
	
	return -EIO;
}

int get_property_from_bms(struct bq27520_chip *chip,	enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy)
		chip->bms_psy =	power_supply_get_by_name("bms");
	if (!chip->bms_psy) {
		pr_err("no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_err("bms psy doesn't support reading prop %d rc = %d\n",	prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

static int get_property_from_batt(struct bq27520_chip *chip,	enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->batt_psy) {
		pr_err("no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->batt_psy->get_property(chip->batt_psy, prop, &ret);
	if (rc) {
		pr_err("bms psy doesn't support reading prop %d rc = %d\n", prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

/******************************************************** 
 *					  functions 				                     *
 *********************************************************/
static u8 bq27520_checksum(u8 *data, u8 len)
{
    u16 sum = 0;
    int i;

    for (i = 0; i < len; i++)
        sum += data[i];

    sum &= 0xFF;

    return 0xFF - sum;
}

static int bq27520_get_cntl_status(struct bq27520_chip *chip)
{
    int ret;
    u16 buf;

	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, 0x00);
    if(ret < 0){
        dev_err(chip->dev,"Failed to send read fw version command\n");
        return ret;
    }
	
    mdelay(2);
    ret = bq27520_i2c_read_word(chip, BQ27520_REG_CNTL, &buf);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read read fw version \n");
        return ret;
    }
 
    return buf;
}

static int bq27520_read_fw_version(struct bq27520_chip *chip)
{
    int ret;
    u16 buf;
    
    ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_FW_VER);
    if(ret < 0){
        dev_err(chip->dev,"Failed to send read fw version command\n");
        return ret;
    }
	
    mdelay(2);
    ret = bq27520_i2c_read_word(chip, BQ27520_REG_CNTL, &buf);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read read fw version \n");
        return ret;
    }
 
    return buf;
  
}

static int bq27520_get_batt_flags(struct bq27520_chip *chip)
{
    int ret;
    u16 buf;
    
    ret = bq27520_i2c_read_word(chip, BQ27520_REG_FLAGS, &buf);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read read batt flags \n");
        return ret;
    }
 
    return buf;
}

static int bq27520_reset_fuel_gauge(struct bq27520_chip *chip)
{
    int ret;
	
    //1. unseal
	ret = bq27520_unseal(chip);
	if(ret < 0){
		pr_err("fail to unseal,ret=%d\n",ret);
		return ret;
	}
	
	//2. reset the fuel gauge
    mdelay(2);
	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_RESET);
	if(ret < 0)
		pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);

	//3. sealed
    mdelay(10);
	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_SEALED);
	if(ret < 0)
		pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);

	return ret;
}

static int bq27520_enter_rom_subclass(struct bq27520_chip *chip, int classid)
{
    int ret;
	
    //1. unseal
	ret = bq27520_unseal(chip);
	if(ret == 0){
		pr_err("fail to unseal,ret=%d\n",ret);
		return -1;
	}
	//2. reset
    ret = bq27520_i2c_writeb(chip, BQ27520_REG_BLKDATCTL, 0);
    if(ret < 0) 
		return ret;
	
	//3. access the registers subclass
    mdelay(2);
    ret = bq27520_i2c_writeb(chip, BQ27520_REG_DFCLASS, classid);
    if(ret < 0) 
	    return ret;

	return 0;
}

static int bq27520_modify_opconfigb_wrtemp(struct bq27520_chip *chip)
{
    int ret = 0;
	u8 op_confib,new_confib;
	u8 checksum;
	u8 temp;

	//1,2,3 enter rom subclass
	ret = bq27520_enter_rom_subclass(chip, 0x40);
	if(ret < 0){
		pr_err("fail to enter rom subclass,ret=%d\n",ret);
		return ret;
	}
	
    //4. write the block offset location
    mdelay(2);
	ret = bq27520_i2c_writeb(chip, BQ27520_REG_DFBLOCK, 0x00);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x3f, ret);
		return ret;
	}
	
    //5. read the data of a specific offset
    mdelay(2);
    ret = bq27520_i2c_readb(chip, 0x4b, &op_confib);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x4b,ret);
		return ret;
	}
    pr_debug("read reg[0x%x]=0x%x\n",0x4b,op_confib);

	//6. read 1-byte checksum
    ret = bq27520_i2c_readb(chip, BQ27520_REG_BLKCHKSUM, &checksum);
	if(ret < 0){
		pr_err("fail to write reg %d,ret=%d\n",0x60,ret);
		return ret;
	}
    pr_debug("read reg[0x%x]=0x%x\n",0x60,checksum);

	//7. set new WRTEMP 
    new_confib = op_confib | 0x80;

    temp = (255-checksum-op_confib) % 256;
	
    checksum = 255 - (temp + new_confib) % 256;
		
	//8. write new opconfigB
	ret = bq27520_i2c_writeb(chip, 0x4b, new_confib);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x4b,ret);
		return ret;
	}
	
	//9. write new checksum
	mdelay(2);
	ret = bq27520_i2c_writeb(chip, BQ27520_REG_BLKCHKSUM, checksum);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x60,ret);
		return ret;
	}

	//10. reset
	mdelay(2);
	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_RESET);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);
		return ret;
	}
	
	//11. sealed
	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_SEALED);
	if(ret < 0){
		pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);
		return ret;
	}
	
    return ret;
}

static int bq27520_read_df(struct bq27520_chip *chip,u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    u8 crc;
    u8 crc_calc = 0;
    
    if (offset % 32 + len > 32) 
		return -EINVAL; // less than one block boundary one time
	
	ret = bq27520_enter_rom_subclass(chip, classid);
	if(ret < 0){
		pr_err("fail to enter rom subclass,ret=%d\n",ret);
		return ret;
	}
	
    mdelay(2);
    ret = bq27520_i2c_writeb(chip, BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0) 
		return ret;
    
    mdelay(2);
    ret = bq27520_read_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0) 
		return ret;
    
    bq27520_i2c_readb(chip, BQ27520_REG_BLKCHKSUM, &crc);
    crc_calc = bq27520_checksum(tmp_buf,32);
    if(crc != crc_calc) 
		return -2;
	
    for(i = 0; i < len; i++){
        buf[i] =  tmp_buf[offset % 32 + i];
    }

    return len;
} 

static int bq27520_write_df(struct bq27520_chip *chip,u8 classid, u8 offset, u8* buf, u8 len)
{
    int ret;
    u8 tmp_buf[40];
    int i;
    u8 crc;
    u8 crc_calc = 0;
    
    if (offset % 32 + len > 32)
		return -EINVAL; // less than one block one time

	ret = bq27520_enter_rom_subclass(chip, classid);
	if(ret < 0){
		pr_err("fail to enter rom subclass,ret=%d\n",ret);
		return ret;
	}
    mdelay(2);
    ret = bq27520_i2c_writeb(chip, BQ27520_REG_DFBLOCK, offset / 32);
    if(ret < 0) 
		return ret;
    
    mdelay(2);
    ret = bq27520_read_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0) 
		return ret;
    
    mdelay(2);
    bq27520_i2c_readb(chip, BQ27520_REG_BLKCHKSUM, &crc);
    crc_calc = bq27520_checksum(tmp_buf,32);
    if(crc != crc_calc)
		return -2;
    //update with new value
    for(i = 0; i < len; i++)
        tmp_buf[offset % 32 + i] = buf[i];
    // calculate new crc 
    crc_calc = bq27520_checksum(tmp_buf,32);
	
    mdelay(2);
    ret = bq27520_write_i2c_blk(chip, BQ27520_REG_BLOCKDATA, tmp_buf, 32);
    if(ret < 0) 
		return ret;
    
    mdelay(2);
    ret = bq27520_i2c_writeb(chip, BQ27520_REG_BLKCHKSUM, crc_calc);
    return ret;
} 

//#define BQFG_SHUTDOWN_MODE
#ifdef BQFG_SHUTDOWN_MODE
static int  bq27520_masked_write(struct bq27520_chip *chip, u8 reg,
							u16 mask, u16 val)
{
	int rc;
	u16 buf;

	rc = bq27520_i2c_read_word(chip,reg,&buf);
	if (rc<0) {
		pr_err("bq27520_i2c_read_word failed: reg=0x%x, rc=%d\n", reg, rc);
		return rc;
	}
	
	buf &= ~mask;
	buf |= val & mask;

	rc = bq27520_i2c_write_word(chip,reg,buf);
	if (rc<0) {
		pr_err("bq27520_i2c_write_word failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	
	return 0;
}

#define BQFG_SHURDOWN_TIME_MASK     0xe000
#define BQFG_SHURDOWN_MODE_MASK     0x0080
#define BQFG_SHURDOWN_SHIT  7
int bq27520_shutdown_mode(int enable)
{
    u16 temp;
	int rc;
	
    if (!bqfg_chip) {
		pr_err("called before init\n");
		return 0;
	}
	if(enable)
        temp = 1 << BQFG_SHURDOWN_SHIT;
	else
		temp = 0;
  
    rc = bq27520_masked_write(bqfg_chip,BQ27520_REG_CNTL,BQFG_SHURDOWN_MODE_MASK,temp);
	if (rc) {
		pr_err("bq27520_masked_write failed to modify shdn, rc=%d\n", rc);
		return rc;
	}

	rc = bq27520_masked_write(bqfg_chip,BQ27520_REG_SHDNTIMER,BQFG_SHURDOWN_TIME_MASK,0);
	if (rc) {
		pr_err("bq27520_masked_write failed to modify shdn, rc=%d\n", rc);
		return rc;
	}
	mdelay(10);

	return 0;	
}
EXPORT_SYMBOL_GPL(bq27520_shutdown_mode);
static int maxin_shutdown_notify(struct notifier_block *this, unsigned long code,
			  void *unused)
{
    printk("%s: go to shutdown...\n",__func__);
	bq27520_shutdown_mode(1);
	
	return NOTIFY_DONE;
}
static struct notifier_block maxin_notifier = {
	.notifier_call = maxin_shutdown_notify,
};
#endif

//------------------------------------------------------------------------

#define DEFAUT_BATT_TEMP 250
static int bq_get_batt_temp(struct bq27520_chip *chip)
{
	int temp, rc;

	rc = get_property_from_batt(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_err("Couldn't get temperature rc = %d\n", rc);
		return DEFAUT_BATT_TEMP;
	}
	return temp;
}

int bq27520_get_batt_voltage(struct bq27520_chip *chip)
{
    int ret;
	u16 value;

	ret = bq27520_i2c_read_word(chip, BQ27520_REG_VOLT, &value);
	if (ret<0) {
		pr_err("fail to read batt vol\n");
		return 0;
	}

	return value;
}

int bq27520_get_batt_temp(struct bq27520_chip *chip)
{
    int ret;
	u16 value;

    ret = bq27520_i2c_read_word(chip, BQ27520_REG_TEMP, &value);
	if (ret<0) {
		pr_err("fail to read batt temp\n");
		return BQ_DEFAULT_TEMP;
	}

	value = value - 2730;
	
	return (int)value;
}

static int bq27520_set_batt_temp(struct bq27520_chip *chip,int batt_temp)
{
    int ret;

	batt_temp = batt_temp + 2730;
    
	ret = bq27520_i2c_write_word(chip,BQ27520_REG_TEMP,(u16)batt_temp);
	if (ret<0) {
		pr_err("fail to write batt temp\n");
		return -EINVAL;
	}
	
	return 0;
}
static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

static int bq27520_get_batt_soc(struct bq27520_chip *chip, int *battery_soc)
{
    int ret;
	u16 value=0;
   
    ret = bq27520_i2c_read_word(chip, BQ27520_REG_SOC, &value);
	if (ret<0) {
		pr_err("fail to read batt soc\n");
		*battery_soc = BQFG_DEFAULT_SOC;
		return ret;
	}
	
    pr_debug("get soc value=%d ret=%d\n", value, ret);
	
	*battery_soc = bound_soc(value);
	return ret;
}

static int bq27520_get_ibatt_now(struct bq27520_chip *chip)
{
    int ret;
	u16 value = 0;

	ret = bq27520_i2c_read_word(chip, BQ27520_REG_INSTC, &value);
	if (ret<0) {
		pr_err("fail to get batt current\n");
		return 0;
	}
	
	return (s16)(value*(-1));
}

static int bq27520_get_ibatt_avg(struct bq27520_chip *chip)
{
    int ret;
	u16 value = 0;

	ret = bq27520_i2c_read_word(chip, BQ27520_REG_AI, &value);
	if (ret<0) {
		pr_err("fail to get batt current\n");
		return 0;
	}
	
	return (s16)(value*(-1));
}

static int bq27520_get_rm_mah(struct bq27520_chip *chip)
{
    int ret;
	u16 value = 0;

	ret = bq27520_i2c_read_word(chip, BQ27520_REG_RM, &value);
	if (ret<0) {
		pr_err("fail to rm mah\n");
		return 0;
	}
	
	return (s16)value;
}

static int bq27520_get_full_mah(struct bq27520_chip *chip)
{
    int ret;
	u16 value = 0;

	ret = bq27520_i2c_read_word(chip, BQ27520_REG_FCC, &value);
	if (ret<0) {
		pr_err("fail to full mah\n");
		return 0;
	}
	
	return (s16)value;
}

#define BQ27520_QMAX_MAH_CLASSID     82
#define BQ27520_QMAX_MAH_OFFSET      1
#define BQ27520_QMAX_MAH_LENGTH      2
static int bq27520_get_qmax_mah(struct bq27520_chip *chip)
{
    u8 buf[2] = {0};
    int ret;
	int val;

    ret = bq27520_read_df(chip,
		                  BQ27520_QMAX_MAH_CLASSID,
		                  BQ27520_QMAX_MAH_OFFSET, 
		                  buf, 
		                  BQ27520_QMAX_MAH_LENGTH);
	
    if( ret != BQ27520_QMAX_MAH_LENGTH){
		pr_info("read df err,ret=%d\n",ret);
        return -1;
    }
	
	val = (buf[0]<<8) | buf[1];
	pr_debug("get qmax_mah=%d \n",val);
	
	return val;
}

static int bq27520_get_batt_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct bq27520_chip *chip = container_of(psy, struct bq27520_chip, bq27520_batt_psy);
	u16 value;
	int ret = 0;
	
    if(chip->is_sleep){
		pr_info("chip call before ready\n");
		return -EINVAL;
	}
		
	switch (psp) {
    case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
		
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27520_i2c_read_word(chip, BQ27520_REG_CC, &value);
		if (likely(ret >= 0)) 
			val->intval = value;
		break;	
		
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4400 * 1000;
		break;	
		
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27520_get_batt_voltage(chip) * 1000;
		break;
		
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27520_get_ibatt_now(chip) * 1000;
		break;
		
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = bq27520_get_ibatt_avg(chip) * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->qmax_mah;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27520_get_full_mah(chip);
		if (likely(ret >= 0)) 
			val->intval = (int)ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27520_get_rm_mah(chip);
		if (likely(ret >= 0)) 
			val->intval = (int)ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_AVG:
		ret = bq27520_i2c_read_word(chip, BQ27520_REG_NAC, &value);
		if (likely(ret >= 0)) 
			val->intval = (int)value;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
	    val->intval = nubia_report_batt_capacity(chip->batt_cntl);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27520_i2c_read_word(chip, BQ27520_REG_TTE, &value);
		if (likely(ret >= 0)) 
			val->intval = value;
		break;
		
	default:
		return -EINVAL;
	}

	return 0;
}

static int
bq27520_set_batt_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	int rc = 0;

	pr_debug("%s psp=%d intval=%d\n",__func__,psp,val->intval);
	return rc;
}

static void update_power_supply(struct bq27520_chip *chip)
{
	if (chip->batt_psy)
		power_supply_changed(chip->batt_psy);
}

static void bq27520_dump_regs(struct bq27520_chip *chip)
{
    int i;
	u16 val;
	static int first_flag = 1;

	if(first_flag){
		first_flag = 0;
		printk("MDREG ");
	}
	
	for(i=0;i<=0x2e;i+=2)
	{
		bq27520_i2c_read_word(chip, i, &val);
        printk("reg[0x%x]=0x%4x \n",i,val);
	}

}

#if 1
static int bq27520_debug;
static int bq27520_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    int value = 0;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!bqfg_chip) {
		pr_err("called before init\n");
		return 1;
	}
	
	printk("__%s: bq27520_debug=%d\n",__func__,bq27520_debug);
	switch(bq27520_debug){
	case 1:
		value = bq27520_get_batt_temp(bqfg_chip);
		printk("__%s: batt_soc=%d\n",__func__,value);
		break;
	case 2:
		bq27520_modify_opconfigb_wrtemp(bqfg_chip);
		break;
	case 3:
		value = (int)bq27520_check_rom_mode(bqfg_chip);
		printk("__%s: rom_mode=%d\n",__func__,value);
		break;
	case 4:
		ret = bq27520_reset_fuel_gauge(bqfg_chip);
		if(ret < 0)
			pr_err("fail to write reg[0x%x],ret=%d\n",0x00,ret);
		break;
	default:
		break;
	};
	
	return 0;
}
module_param_call(bq27520_debug, bq27520_debug_mode, param_get_uint,
					&bq27520_debug, 0644);
#endif

static int bq27520_battery_read_dt_props(struct bq27520_chip *chip)
{
    int rc;
	
	chip->soc_int_gpio = of_get_named_gpio_flags(chip->dev_node,"bq-soc-int-gpio", 0, NULL);
	if (chip->soc_int_gpio < 0){
		pr_err( "Unable to parse 'soc_int_gpio'\n");
		return chip->soc_int_gpio;
	}
	
	chip->low_int_gpio = of_get_named_gpio_flags(chip->dev_node, "bq-low-int-gpio",0, NULL);
	if (chip->low_int_gpio < 0){
		pr_err( "Unable to parse 'low_int_gpio'\n");
		//return chip->low_int_gpio;
	}
	
	rc = of_property_read_u32(chip->dev_node, "low-voltage-threshold", &chip->low_vol_thrhld);
	if (rc) {
		pr_err( "Unable to parse 'low-voltage-threshold'\n");
		return rc;
	}
		
	FGLOG_DEBUG("soc_int=%d low_int=%d low_vol_thrhld=%d\n",chip->soc_int_gpio,chip->low_int_gpio,chip->low_vol_thrhld);

	return 0;
}

static ssize_t bq27520_show_power_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27520_chip *bq27520 = dev_get_drvdata(dev);
	
	return sprintf(buf, "%d\n", bq27520->batt_por);
}
static ssize_t bq27520_store_power_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq27520_chip *bq27520 = dev_get_drvdata(dev);
	
	bq27520->batt_por = *buf;
	return 1;
}

const static DEVICE_ATTR(batt_por, S_IRUGO | S_IWUSR,
	            bq27520_show_power_status, bq27520_store_power_status);

static irqreturn_t bq27520_soc_isr(int irq, void *irq_data)
{
	struct bq27520_chip *chip = irq_data;
	int batt_soc;
    u16 batt_flags;
	int batt_mv;
	
	batt_flags = bq27520_get_batt_flags(chip);
	batt_mv = bq27520_get_batt_voltage(chip);
	
	if( bq27520_get_batt_soc(chip, &batt_soc) < 0 );
		batt_soc = nubia_report_batt_capacity(chip->batt_cntl); //NUBIA_BATT
	
	if(!wake_lock_active(&chip->soc_wlock) && (batt_soc == 0 || batt_mv < chip->low_vol_thrhld))
    	wake_lock(&chip->soc_wlock);
	
	FGLOG_INFO("batt_flags=0x%x batt_mv=%d soc=%d\n",batt_flags,batt_mv,batt_soc);
	update_power_supply(chip);

	return IRQ_HANDLED;
}

 static int bq27520_pinctrl_select(struct bq27520_chip *chip, struct pinctrl_state *pins_state)
 {
	 int ret;
		 
	 if (!IS_ERR_OR_NULL(pins_state)) {
		 ret = pinctrl_select_state(chip->pinctrl, pins_state);
		 if (ret) {
			 pr_err( "can not set gpio pins(%d)\n",ret );
			 return ret;
		 }
	 } else 
		 pr_err("not a valid gpio pinstate\n");
	 
	 return 0;
 }

static int bq27520_pinctrl_init(struct bq27520_chip *chip)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	chip->pinctrl = devm_pinctrl_get(&(chip->i2c->dev));
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		pr_err(	"Target does not use pinctrl! \n");
		retval = PTR_ERR(chip->pinctrl);
		chip->pinctrl = NULL;
		return retval;
	}

	chip->gpio_state_active	= pinctrl_lookup_state(chip->pinctrl,"default");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		pr_err("Can not get ts default pinstate! \n");
		retval = PTR_ERR(chip->gpio_state_active);
		chip->pinctrl = NULL;
		return retval;
	}

	pr_debug("BQ FG PinCtrl Init Success! \n");
	return 0;
}

/***********************************************************
*  for debug reg , path: sys/kernel/debug/bq27520
*  reg:   the reg to read or write
*  data: 'echo x > data' to write the reg  and  'cat data' to read the reg
************************************************************/
static u8 bqfg_fs_reg;
static int get_reg_addr(void *data, u64 * val)
{
	*val = bqfg_fs_reg;
	return 0;
}

static int set_reg_addr(void *data, u64 val)
{
	bqfg_fs_reg = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(set_reg_fops, get_reg_addr, set_reg_addr, "0x%02llx\n");

static int get_reg_data(void *data, u64 * val)
{
	struct bq27520_chip *chip = (struct bq27520_chip *)data;
	int ret;
	u16 value;

    ret = bq27520_i2c_read_word(chip, bqfg_fs_reg, &value);
	if (ret<0) {
		pr_err("fail to read reg[0x%x]\n",bqfg_fs_reg);
		return ret;
	}

	*val = value;
	return 0;
}

static int set_reg_data(void *data, u64 val)
{
	struct bq27520_chip *chip = (struct bq27520_chip *)data;
	int ret;
	u16  value = val;

	ret = bq27520_i2c_write_word(chip, bqfg_fs_reg, value);
	if (ret<0) {
	   pr_err("fail to write reg[0x%x]\n",bqfg_fs_reg);
	   return ret;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rw_reg_fops, get_reg_data, set_reg_data, "0x%02llx\n");

static void bq27520_create_debugfs_entries(struct bq27520_chip *chip)
{

	chip->dent = debugfs_create_dir("bq27520", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("bq27520 couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("reg", 0644, chip->dent, chip, &set_reg_fops);
	debugfs_create_file("data", 0644, chip->dent, chip, &rw_reg_fops);
	return;
}

#ifdef BQ27520_UPDATER 
/* the following routines are for bqfs/dffs update purpose, can be removed if not used*/
static int bq27520_check_seal_state(struct bq27520_chip *chip)
{
    int status;
	int ret;
	u16 buf;
	
    bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_CTNL_STATUS);
    mdelay(2);
	
	ret = bq27520_i2c_read_word(chip, BQ27520_REG_CNTL, &buf);
    if(ret < 0) 
		return ret;
    
    if((buf & 0x6000) == 0) //FA and SS neither set
        status = BQ27520_SECURITY_FA;
    else if((buf & 0x2000) == 0) // SS not set
        status = BQ27520_SECURITY_UNSEALED;
    else    
        status = BQ27520_SECURITY_SEALED;

    return status;
}

static int bq27520_unseal(struct bq27520_chip *chip)
{
    int ret;
    
	bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_UNSEAL_KEY & 0xFFFF);
    mdelay(2);
	bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, (BQ27520_UNSEAL_KEY >> 16)& 0xFFFF);
    mdelay(5);
    
    ret = bq27520_check_seal_state(chip);	
    if(ret == BQ27520_SECURITY_UNSEALED || ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;
}

static int bq27520_unseal_full_access(struct bq27520_chip *chip)
{
    int ret;
    
	bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_FA_KEY & 0xFFFF);
    mdelay(2);
	bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, (BQ27520_FA_KEY >> 16)& 0xFFFF);
    mdelay(5);
    
    ret = bq27520_check_seal_state(chip);
    if(ret == BQ27520_SECURITY_FA)
        return 1;
    else
        return 0;
}

static bool bq27520_check_rom_mode(struct bq27520_chip *chip)
{
    struct i2c_client *client = to_i2c_client(chip->dev);
    int ret;
	u8 val;
	u16 value=0;

    client->addr = BQGAUGE_I2C_DEV_ADDR;
	ret = bq27520_i2c_read_word(chip, BQ27520_REG_SOC, &value);
	if (ret >= 0) {
		pr_err("it is in normal mode\n");
		return false;
	}
	
    client->addr = BQGAUGE_I2C_ROM_ADDR;
	ret = bq27520_i2c_readb(chip, 0x66, &val);
    mdelay(2);
    client->addr = BQGAUGE_I2C_DEV_ADDR;//restore address
    if(ret < 0 ){ 
        pr_info("it is not in rom mode\n");
        return false;
    }
	pr_info("it is in rom mode\n");
    return true;
}

static bool bq27520_enter_rom_mode(struct bq27520_chip *chip)
{
    int ret;
    
	ret = bq27520_i2c_write_word(chip, BQ27520_REG_CNTL, BQ27520_SUBCMD_ENTER_ROM);
    mdelay(2);
    if(ret < 0) 
		return false;
    
    return bq27520_check_rom_mode(chip);
}

#define BQ27520_DEVICE_NAME_CLASSID     48
#define BQ27520_DEVICE_NAME_OFFSET      17
#define BQ27520_DEVICE_NAME_LENGTH      7

static bool bq27520_check_update_necessary(struct bq27520_chip *chip)
{
    // this is application specific, return true if need update firmware or data flash
    u8 buf[40] = {0};
    int ret;

    ret = bq27520_read_df(chip,
		                  BQ27520_DEVICE_NAME_CLASSID,
		                  BQ27520_DEVICE_NAME_OFFSET, 
		                  buf, 
		                  BQ27520_DEVICE_NAME_LENGTH);
	
    if( ret != BQ27520_DEVICE_NAME_LENGTH){
		pr_info("read df err,ret=%d\n",ret);
        return false;
    }
	
	pr_info("get now bqfs_version = %s\n",buf);
	
    if(strncmp(buf, chip->bqfs_version, BQ27520_DEVICE_NAME_LENGTH) == 0) {
        return false;
    } else{    
  		pr_info("need update to new bqfs_version = %s\n",chip->bqfs_version);
        return true;
    }
}

static bool bq27520_mark_as_updated(struct bq27520_chip *chip)
{
    // this is application specific
    int ret;
    ret = bq27520_write_df(chip,
		                   BQ27520_DEVICE_NAME_CLASSID,
		                   BQ27520_DEVICE_NAME_OFFSET, 
		                   chip->bqfs_version, 
		                   BQ27520_DEVICE_NAME_LENGTH);
    if(ret < 0) 
        return false;
    else    
        return true;
}

static bool bq27520_update_execute_cmd(struct bq27520_chip *chip, const bqfs_cmd_t *cmd)
{
    int ret;
    uint8_t tmp_buf[CMD_MAX_DATA_SIZE];

    switch (cmd->cmd_type) {
    case CMD_R:
        ret = bq27520_read_i2c_blk(chip, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
        if( ret < 0){ 
			pr_err("CMD_R:fail to read block reg=0x%x,ret=%d",cmd->reg,ret);
			return false;
        }
        return true;

    case CMD_W:
		ret = bq27520_write_i2c_blk(chip, cmd->reg, (u8 *)&cmd->data.bytes, cmd->data_len);
        if(ret < 0){
			pr_err("CMD_W:fail to write block reg=0x%x,ret=%d",cmd->reg,ret);
			return false;
        }
        return true;
		
    case CMD_C:
		ret = bq27520_read_i2c_blk(chip, cmd->reg, tmp_buf, cmd->data_len);
        if (ret < 0){
            pr_err("CMD_C:fail to read block reg=0x%x,ret=%d",cmd->reg,ret);
			return false;
        }
        if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len)) {
            dev_err(chip->dev, "\nCommand C failed at line %d: reg[0x%x] data[0x%x], temp[0x%x]\n",
				cmd->line_num,cmd->reg,*(cmd->data.bytes),*tmp_buf);
            return false;
        }
        return true;

    case CMD_X:
        mdelay(cmd->data.delay);
        return true;

    default:
        dev_err(chip->dev, "Unsupported command at line %d\n",
            cmd->line_num);
        return false;
    }
}

static int bq27520_update_bqfs(struct bq27520_chip *chip)
{
    struct i2c_client *client = to_i2c_client(chip->dev);
    u16 i;
	int ret;

	chip->bqfs_version = BQFS_DATA_VERSION;
	chip->bqfs_image = bqfs_image;
	chip->bqfs_image_size = ARRAY_SIZE(bqfs_image);

	pr_info("load bqfs_version=%s\n",chip->bqfs_version);
	
    if(bq27520_check_rom_mode(chip)) 
		goto update;// already in rom mode
	
    // check if needed update 
    if(!bq27520_check_update_necessary(chip)){
		pr_info("not need update\n");
		return -EINVAL;
    }

    if (bq27520_check_seal_state(chip) != BQ27520_SECURITY_FA){
        if(!bq27520_unseal(chip)) 
            return -EINVAL;
        mdelay(10);
        if(!bq27520_unseal_full_access(chip)) 
            return -EINVAL;
    }

    if(!bq27520_enter_rom_mode(chip)){ 
		pr_err("fail to enter rom mode\n");
		return -EINVAL;
    }
	
update:    
    client->addr = BQGAUGE_I2C_ROM_ADDR;
    dev_info(chip->dev,"BQFS Updating");
    for(i = 0; i < chip->bqfs_image_size; i++){
        dev_dbg(chip->dev,". i=%d",i);
        if(!bq27520_update_execute_cmd(chip,&chip->bqfs_image[i])){
            dev_err(chip->dev,"%s: Failed at command=%d addr=0x%x reg=0x%x\n",__func__,
				    i,chip->bqfs_image[i].addr,chip->bqfs_image[i].reg);
            return -EINVAL;
        }
    }
    dev_info(chip->dev,"Done!\n");

    client->addr = BQGAUGE_I2C_DEV_ADDR;    
    // mark as updated
    ret = bq27520_mark_as_updated(chip);
	if(!ret)
		pr_err("fail to mark update\n");
    
    return 0;
}
#endif

static ssize_t bq27520_show_batt_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct power_supply	 *bq27520_psy = dev_get_drvdata(dev);
	struct bq27520_chip *chip = container_of(bq27520_psy, struct bq27520_chip, bq27520_batt_psy);

	return sprintf(buf, "%s\n", chip->battery_type);
}

const static DEVICE_ATTR(batt_type, S_IRUGO , bq27520_show_batt_type, NULL);

//NUBIA_BATT
static int bq27520_nubia_get_initial_soc(struct bq27520_chip *chip)
{

	int batt_temp,batt_mv,batt_soc,initial_soc;
	
	batt_mv = bq27520_get_batt_voltage(chip);
	batt_temp = bq_get_batt_temp(chip);
	
	if( bq27520_set_batt_temp(chip,batt_temp) < 0 ){
		pr_err("fail to write batt temp\n");
		goto INIT_FAIL;
	}
	
	mdelay(5);
	
	if( bq27520_get_batt_soc(chip, &batt_soc) < 0 ){
		pr_err("fail to get batt soc\n");
		goto INIT_FAIL;
	}

	initial_soc = batt_soc;
	pr_info("shdn_soc=%d shdn_vbatt=%d pon_soc=%d pon_mv=%d batt_temp=%d\n", 
		chip->shutdown_soc,chip->shutdown_vbatt,batt_soc,batt_mv,batt_temp);
	return initial_soc;

INIT_FAIL:
	return BQFG_DEFAULT_SOC;
	
}

static int bq27520_nubia_get_fg_soc(struct nubia_fg_cntl *batt_cntl, int *batt_soc)
{
	struct bq27520_chip *chip = (struct bq27520_chip *)batt_cntl->fg_pri_data;
	int batt_temp;
	int rc;
	
	batt_temp = bq_get_batt_temp(chip);
	if( bq27520_set_batt_temp(chip, batt_temp) < 0)
		pr_err("fail to write batt temp\n");

	mdelay(5);

	rc = bq27520_get_batt_soc(chip, batt_soc);
	if(rc < 0){
		pr_err("fail to write batt temp\n");
		return rc;
	}

	return  0;
}

static int bq27520_nubia_update_data(struct nubia_fg_cntl *batt_cntl)
{
	struct bq27520_chip *chip = (struct bq27520_chip *)batt_cntl->fg_pri_data;
	
	chip->batt_flags = bq27520_get_batt_flags(chip);
	chip->cntl_status = bq27520_get_cntl_status(chip);
	chip->rm_mah = bq27520_get_rm_mah(chip);
	chip->full_mah = bq27520_get_full_mah(chip);
	chip->qmax_mah = bq27520_get_qmax_mah(chip);

	return 0;
}

static int bq27520_print_info(struct nubia_fg_cntl *batt_cntl)
{
	struct nubia_fg_params *pbatt = &batt_cntl->batt_data;
	struct bq27520_chip *chip = (struct bq27520_chip *)batt_cntl->fg_pri_data;

	bq27520_nubia_update_data(batt_cntl);
	
	pr_info("bsoc=%d T=%d mA=%d mV=%d U=%d St=%d [0x%x 0x%x %d %d %d %d]\n",
		pbatt->batt_soc,pbatt->batt_temp,pbatt->batt_ma,pbatt->batt_mv,pbatt->usb_in,pbatt->batt_status,
		chip->batt_flags,chip->cntl_status,pbatt->fg_soc,chip->rm_mah,chip->full_mah,chip->qmax_mah);
	
	if (bqfg_log_level > FG_DEBUG)
		bq27520_dump_regs(chip);

	return 0;
}

static int bq27520_nubia_batt_init(struct bq27520_chip *chip)
{
	struct nubia_fg_cntl *batt_cntl;
	int rc;

	batt_cntl = kzalloc(sizeof(*batt_cntl), GFP_KERNEL);
	if (!batt_cntl) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->batt_cntl = batt_cntl;

	//--Part start. These 6 datas must be inited.
	/**  fg_pri_data  **/
	batt_cntl->fg_pri_data = chip;
	/**  nubia_update_fg_data  **/
	batt_cntl->nubia_update_fg_data = bq27520_nubia_update_data;
	/**  nubia_print_info  **/
	batt_cntl->nubia_print_info = bq27520_print_info;
	/**  nubia_get_soc  **/
	batt_cntl->nubia_get_soc = bq27520_nubia_get_fg_soc;
	/**  soc_wlock  **/
	batt_cntl->soc_wlock = &chip->soc_wlock;
	/**  low_vol_thrhld  **/
	batt_cntl->low_vol_thrhld = chip->low_vol_thrhld;
	/**  init  batt_soc  **/
	batt_cntl->batt_data.batt_soc = bq27520_nubia_get_initial_soc(chip);
    //--Part end. 
		
    rc = nubia_batt_cntl_init(batt_cntl);
	if (rc) {
		pr_err("Couldn't nubia_chg_cntl_init\n");
		return -EINVAL;
	}

    return 0;
}
//NUBIA_BATT end

static int  bq27520_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct bq27520_chip *chip;
	struct power_supply *battery_psy;
	int fw_version;
	int ret;
	int batt_cap;
	
	pr_info("enter..\n");

	chip = kzalloc(sizeof(struct bq27520_chip),GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate bq27520_chip\n");
		return -ENOMEM;
	}

    battery_psy = power_supply_get_by_name("battery");
	if (!battery_psy) {
		pr_err("battery supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}
	
	chip->i2c = client;
	chip->dev = &client->dev;
	chip->dev_node = client->dev.of_node;
	chip->batt_psy = battery_psy;
	chip->batt_temp = BQ_DEFAULT_TEMP;
	i2c_set_clientdata(client, chip);	
	
	bq27520_battery_read_dt_props(chip);

	ret = bq27520_update_bqfs(chip);
    if(ret < 0)
		pr_info("do not update bqfs\n");

	wake_lock_init(&chip->soc_wlock, WAKE_LOCK_SUSPEND, "bq27520_soc");

	chip->bq27520_batt_psy.name = "bq27520-fg";
	chip->bq27520_batt_psy.type = POWER_SUPPLY_TYPE_FG;
	chip->bq27520_batt_psy.properties = bq27520_batt_props;
	chip->bq27520_batt_psy.num_properties = ARRAY_SIZE(bq27520_batt_props);
	chip->bq27520_batt_psy.get_property = bq27520_get_batt_property;
	chip->bq27520_batt_psy.set_property = bq27520_set_batt_property;
	ret = power_supply_register(chip->dev, &chip->bq27520_batt_psy);
	if (ret < 0) {
		pr_err("power_supply_register bq27520 failed rc = %d\n", ret);
		goto fail_psy;
	}

	ret = device_create_file(chip->bq27520_batt_psy.dev, &dev_attr_batt_type);
	if (unlikely(ret < 0)) 
		pr_err("failed: cannot create batt_type. ret=%d\n",ret);
	
	//----------------------------------------------
	ret = bq27520_pinctrl_init(chip);
	if (ret) {
		pr_err("failed init gpio ret=%d\n", ret);
	}else{
		bq27520_pinctrl_select(chip, chip->gpio_state_active);
	}
		
	if (gpio_is_valid(chip->soc_int_gpio)) {
		ret = gpio_request(chip->soc_int_gpio, "bq27520_soc_gpio");
		if (unlikely(ret < 0)) {
			dev_err(chip->dev, "gpio request failed");
		}
		ret = gpio_direction_input(chip->soc_int_gpio);
		 
		chip->soc_irq = gpio_to_irq(chip->soc_int_gpio);	
		if (unlikely(chip->soc_irq < 0)) {
			dev_err(chip->dev, "gpio request to isr failed");
		}

		ret = request_threaded_irq(chip->soc_irq, 
		      	NULL, 
				bq27520_soc_isr, 
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT), 
				"bq27520_soc_irq",	
				chip);
		if (unlikely(ret < 0)) {
			dev_err(chip->dev, "request_irq failed\n");	
		}else{
		    enable_irq_wake(chip->soc_irq);
		}
	}
	//-----------------------------------------------
	
	bq27520_create_debugfs_entries(chip);
	
	bq27520_modify_opconfigb_wrtemp(chip);

	bqfg_chip = chip;
	
	//NUBIA_BATT
	ret = bq27520_nubia_batt_init(chip);
	if(ret < 0)
		pr_err("bq27520_nubia_batt_init fail!\n");	

	fw_version = bq27520_read_fw_version(chip);

	bq27520_get_batt_soc(chip, &batt_cap);
    pr_info("bq27520 successfully probed. fw_version=0x%x batt_cap=%d\n",fw_version,batt_cap);

    return 0;

fail_psy:
	dev_set_drvdata(chip->dev, NULL);
	return ret;


}

static int bq27520_battery_remove(struct i2c_client *client)
{	
	struct bq27520_chip *bq27520 = i2c_get_clientdata(client);

	kfree(bq27520);
	bq27520 = NULL;
	bqfg_chip = NULL;

	return 0;
}

static int bq27520_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	struct bq27520_chip *chip = i2c_get_clientdata(cl);
	
	FGLOG_DEBUG("goto suspend.\n");
	chip->is_sleep = 1;	
	//NUBIA_BATT
	nubia_batt_suspend(chip->batt_cntl);

	return 0;
};

static int bq27520_resume(struct i2c_client *cl)
{
	struct bq27520_chip *chip = i2c_get_clientdata(cl);

	FGLOG_DEBUG("goto resume.\n");
	chip->is_sleep = 0; 
	//NUBIA_BATT
	nubia_batt_resume(chip->batt_cntl);

	return 0;
};

static struct of_device_id  bq27520_match_table[] = {
	{ .compatible = "ti,bq27520_fg",},
	{}
};

static const struct i2c_device_id  bq27520_id[] = {
	{ "bq27520_fg", 1 },
	{},
};

static struct i2c_driver bq27520_battery_driver = {
	.driver = {
		.name = "bq27520_fg",
		.of_match_table = bq27520_match_table,
	},
	.id_table 	= bq27520_id,
	.probe 		= bq27520_battery_probe,
	.remove 	= bq27520_battery_remove,

	.suspend	= bq27520_suspend,
	.resume 	= bq27520_resume,
};


static int __init bq27520_battery_init(void)
{
	printk( "%s:enter...\n", __func__);

	return i2c_add_driver(&bq27520_battery_driver);
}

static void __exit bbq27520_battery_exit(void)
{
	printk( "%s:bq27520 is exiting\n", __func__);

	i2c_del_driver(&bq27520_battery_driver);
}

module_init(bq27520_battery_init);

module_exit(bbq27520_battery_exit);

MODULE_AUTHOR("ztemt-swang");
MODULE_DESCRIPTION("bq27520 battery driver");
MODULE_LICENSE("GPL");
