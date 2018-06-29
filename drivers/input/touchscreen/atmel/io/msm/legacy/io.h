#ifndef __LINUX_ATMEL_MXT_IO
#define __LINUX_ATMEL_MXT_IO

#include <linux/types.h>

#define MXT_I2C_DMA_ADDR_FLAG	0

#define MXT_EVENT_IRQ 1
#define MXT_EVENT_EXTERN 2
#define MXT_EVENT_IRQ_FLAG 5

struct obj_container{
	u8 id;		//container pair: ID value for search
	int value;	//container value
	struct list_head node;
};

struct obj_link{
	u16 object;	//object
	struct list_head sublist;	//container head
	struct list_head node;	//next link
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	atomic_t depth;		/* irq nested counter*/
	int irq;			/* irq issued by device 	*/

	unsigned gpio_reset;
	unsigned gpio_irq;

	bool use_retrigen_workaround;
	bool use_regulator;
	bool common_vdd_supply;

	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	
	const char *cfg_name;

	struct list_head keylist;

	bool mem_allocated;
};

static inline struct obj_link *get_obj_link(struct list_head *link_list, u8 type) 
{
	struct obj_link *ln;

	WARN_ON(!link_list);

	list_for_each_entry(ln, link_list, node) {
		WARN_ON(!ln);
		if (ln->object == type)
			return ln;
	}

	return NULL;
}

static inline bool sublist_empty(struct list_head *link_list, u8 type)
{
	struct obj_link *link;

	link = get_obj_link(link_list, type);
	if (link) {
		if(!list_empty(&link->sublist))
			return false;
	}

	return true;
}

// def: 0xff for default value
static inline int get_sublist_value(struct list_head *con_list, u8 idx, u8 def)
{
	struct obj_container *con;
	int code = -ENOKEY;

	WARN_ON(!con_list);

	list_for_each_entry(con, con_list, node){
		WARN_ON(!con);
		if (con->id == idx)
			return con->value;

		if (def != 0xff) {
			if (con->id == def)		//get default code value
				code = con->value;
		}
	}

	return code;
}

static inline int get_list_value(struct list_head *link_list, u8 type, u8 idx, u8 def) 
{
	struct obj_link *ln;
	int code = -ENOKEY;

	WARN_ON(!link_list);

	list_for_each_entry(ln, link_list, node) {
		WARN_ON(!ln);
		if (ln->object == type) {
			code = get_sublist_value(&ln->sublist,idx, def);
			break;
		}
	}

	return code;
}


int device_wait_irq_state(struct device *dev, int val, long interval);
void device_regulator_enable(struct device *dev);
void device_regulator_disable(struct device *dev);
int device_gpio_configure(struct device *dev);
void device_gpio_free(struct device *dev);
int device_power_init(struct device *dev);
void device_power_deinit(struct device *dev);
int device_hw_reset(struct device *dev);
int device_por_reset(struct device *dev);
void device_disable_irq(struct device *dev, const char * name_str);
void device_disable_irq_nosync(struct device *dev, const char * name_str);
void device_disable_irq_wake(struct device *dev);
void device_enable_irq(struct device *dev, const char * name_str);
void device_enable_irq_wake(struct device *dev);
void device_free_irq(struct device *dev, void *dev_id, const char * name_str);
int device_register_irq(struct device *dev, irq_handler_t handler,
			irq_handler_t thread_fn, const char *devname, void *dev_id);
int device_parse_default_chip(void *dev_id, struct device *dev);
void device_release_chip(struct device *dev);

#endif

