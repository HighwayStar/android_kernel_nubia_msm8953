#ifndef __LINUX_ATMEL_MXT_TS_DUMMY_H
#define __LINUX_ATMEL_MXT_TS_DUMMY_H

#include <linux/types.h>
#include "../../../mediatek/tpd.h"

struct mt_i2c_data {
	u16 pdn;	/* MTK clock id */
	u16 speed;	/* bus speed in kHz */
	u32 flags;
};

struct mt_i2c_msg {
	u16 addr;		/* slave address                        */
	u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_STOP		0x8000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_NOSTART */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	u16 len;		/* msg length                           */
	u8 *buf;		/* pointer to msg data                  */
	u32 timing;
	u32 ext_flag;
};

#define I2C_A_FILTER_MSG	0x8000	/* filer out error messages     */
#define I2C_A_CHANGE_TIMING	0x4000	/* change timing paramters      */
#define I2C_MASK_FLAG	(0x00ff)
#define I2C_DMA_FLAG	(0xdead2000)
#define I2C_WR_FLAG		(0x1000)
#define I2C_RS_FLAG		(0x0800)
#define I2C_HS_FLAG   (0x0400)
#define I2C_ENEXT_FLAG (0x0200)
#define I2C_DISEXT_FLAG (0x0000)
#define I2C_POLL_FLAG (0x4000)
#define I2C_CH2_FLAG	(0x8000)

#define I2C_PUSHPULL_FLAG (0x00000002)
#define I2C_3DCAMERA_FLAG (0x00000004)
#define I2C_DIRECTION_FLAG (0x00000008)

#define GPIO_CTP_EINT_PIN 0
#define GPIO_CTP_RST_PIN 0
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0
#define TPD_LDO 1
#define TPD_LDO_AVDD 2
#define GPIO_CTP_RST_PIN_M_GPIO 0
#define GPIO_DIR_OUT 1
#define GPIO_CTP_EINT_PIN_M_EINT 1
#define GPIO_DIR_IN 1
#define GPIO_PULL_ENABLE 1
#define GPIO_PULL_UP 0

static inline int mt_get_gpio_in(unsigned gpio)
{
	WARN_ON(1);
	return 0;
}

static inline void mt_set_gpio_out(unsigned gpio, int value)
{
	/* GPIO can never have been requested or set as output */
	WARN_ON(1);
}

static inline void mt_set_gpio_mode(unsigned gpio, int value)
{
	/* GPIO can never have been requested or set as output */
	WARN_ON(1);
}

static inline void mt_set_gpio_dir(unsigned gpio, int value)
{
	/* GPIO can never have been requested or set as output */
	WARN_ON(1);
}

static inline void mt_set_gpio_pull_enable(unsigned gpio, int value)
{
	/* GPIO can never have been requested or set as output */
	WARN_ON(1);
}

static inline void mt_set_gpio_pull_select(unsigned gpio, int value)
{
	/* GPIO can never have been requested or set as output */
	WARN_ON(1);
}

#define VOL_3300 0
#define PMIC_RG_VLDO3_3_EN 1
#define PMIC_RG_VLDO3_3_ON_CTRL 1
#define PMIC_RG2_VLDO3_3_EN 2
#define PMIC_RG2_VLDO3_3_ON_CTRL 1
static inline int hwPowerOn(int reg, int voltage, const char * name)
{
	WARN_ON(1);
	return 0;
}

static inline int hwPowerDown(int reg, const char * name)
{
	WARN_ON(1);
	return 0;
}

static inline void pmic_set_register_value(int reg, int val)
{
	WARN_ON(1);
}

#define CUST_EINT_TOUCH_PANEL_NUM 1
#define CUST_EINTF_TRIGGER_LOW 0
#define MT_LEVEL_SENSITIVE 1
#define MT_EINT_POL_NEG 0
#define EINTF_TRIGGER_LOW IRQF_TRIGGER_LOW

static inline void mt_eint_mask(unsigned irq)
{
	WARN_ON(1);
}

static inline void mt_eint_unmask(unsigned irq)
{
	WARN_ON(1);
}

static inline void mt_eint_unregistration(unsigned irq)
{
	WARN_ON(1);
}

static inline int mt_eint_registration(unsigned irq, int irqflags, void (*handler)(void), int auto_mask)
{
	WARN_ON(1);
	return 0;
}

static inline void mt_eint_set_polarity(unsigned irq, int flag)
{
	WARN_ON(1);
}

static inline void mt_eint_set_sens(unsigned irq, int flag)
{
	WARN_ON(1);
}

static inline int mt_gpio_to_irq(unsigned irq)
{
	WARN_ON(1);
	return 0;
}

#endif /* __LINUX_ATMEL_MXT_TS_DUMMY_H */
