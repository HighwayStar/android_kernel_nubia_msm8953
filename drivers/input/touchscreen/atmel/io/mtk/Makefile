# Some platform lik MTK6755 android 5.1 need this declaration
#include $(srcTree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/input/touchscreen/atmel/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

ifeq ('$(CONFIG_TOUCHSCREEN_ATMEL_MXTS)', 'y')

obj-y = atmel_mxt_ts.o
obj-y += io/mtk/l/
obj-y += plugin/

else ifeq ('$(CONFIG_TOUCHSCREEN_ATMEL_MXTS)', 'm')

obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXTS) += mxt.o

mxt-objs := atmel_mxt_ts.o
mxt-objs += io/mtk/l/io.o

ifdef CONFIG_MXT_PLIGIN_CAL
mxt-objs += plugin/plugin_cal.o
endif

ifdef CONFIG_MXT_PLIGIN_AC
mxt-objs += plugin/plugin_ac.o
endif

ifdef CONFIG_MXT_PLIGIN_PI
mxt-objs += plugin/plugin_pi.o
endif

ifdef CONFIG_MXT_PLIGIN_MISC
mxt-objs += plugin/plugin_misc.o
endif

ifdef CONFIG_MXT_PLIGIN_CLIP
mxt-objs += plugin/plugin_clip.o
endif

ifdef CONFIG_MXT_PLIGIN_WDG
mxt-objs += plugin/plugin_wdg.o
endif
endif


ifdef CONFIG_MXT_DEBUG_ENABLE
KBUILD_CFLAGS	+= -DDEBUG
endif

#MAKEFLAGS += --include-dir=

