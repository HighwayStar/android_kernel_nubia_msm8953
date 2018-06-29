
#ifndef __LINUX_ATMEL_MXT_PLUG_AC
#define __LINUX_ATMEL_MXT_PLUG_AC

#include <linux/types.h>

#define T72_NOISE_STATE_MASK 0x7
#define T72_NOISE_DUALX_MASK (1<<3)
enum{
	NOISE_RSV = 0,
	NOISE_OFF,
	NOISE_STABLE,
	NOISE_NOISY,
	NOISE_VERY_NOISY,
};
#define T72_NOISE_STATE_TYPES (NOISE_VERY_NOISY - NOISE_OFF)

int plugin_interface_ac_init(struct plugin_ac *p);
void plugin_interface_ac_deinit(struct plugin_ac *p);

ssize_t plugin_ac_extern_event_show(struct plugin_ac *p, char *buf, size_t count);
int plugin_ac_extern_event_store(struct plugin_ac *p, const char *buf, size_t count);

#endif


