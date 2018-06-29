ccflags-y += -I$(srctree)/drivers/input/touchscreen/atmel/

obj-$(CONFIG_MXT_PLIGIN_CAL) += plugin_cal.o
obj-$(CONFIG_MXT_PLIGIN_AC) += plugin_ac.o
obj-$(CONFIG_MXT_PLIGIN_PI) += plugin_pi.o
obj-$(CONFIG_MXT_PLIGIN_MISC) += plugin_misc.o
obj-$(CONFIG_MXT_PLIGIN_CLIP) += plugin_clip.o
obj-$(CONFIG_MXT_PLIGIN_WDG) += plugin_wdg.o

ifdef CONFIG_MXT_DEBUG_ENABLE
KBUILD_CFLAGS	+= -DDEBUG
endif

#MAKEFLAGS += --include-dir=

