#ifndef __NUBIA_WP_CHG_H__
#define __NUBIA_WP_CHG_H__


enum{
	WP_CHG_DISABLE = 0,
	WP_CHG_ENABLE,
};

enum{
	WP_CHG_REMOVE = 0,
	WP_CHG_PRESENT,
};

enum{
	USB_REMOVE_EVENT = 0,
	USB_INSERT_EVENT,
};

enum chg_cfg {
	CHG_CFG_PMI = 0,
	CHG_CFG_WP,
	CHG_CFG_INVALID,
};

enum chg_voter {
	WP_CHG_INIT = 0,
	WP_CHG_COOL = 1<<0, //val: 1
	WP_CHG_WARM = 1<<1, //val: 2
	WP_CHG_CV =   1<<2, //val: 4
	WP_CHG_OV =   1<<3, //val: 8
	WP_CHG_OC =   1<<4, //val: 16
	WP_CHG_USB_HOT = 1<<5, //val: 32
	WP_CHG_TOUT =    1<<6, //val: 64
};

struct vadc_map_pt {
	int x;
	int y;
};

int weipu_is_charging_status(void);

int is_wp_chg_present(void);

int wp_charger_remove_check(void);

int wp_charger_insert_check(void);

int is_wp_fake_chging(void);

int notify_lcd_on(int lcd_on);

#endif
