Porting guide:

cd atmel/

1  Makefile
	<a> move from 'io/<arch>/Makefile' to atmel root directory:
		mv io/mtk/Makefile .
	<b> check content whether match your directory
		include $(srcTree)/drivers/misc/mediatek/Makefile.custom
		ccflags-y += -I$(srctree)/drivers/input/touchscreen/atmel/
		ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
		ccflags-y += ....

		obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXTS) += mxt.o

		mxt-objs := atmel_mxt_ts.o
		mxt-objs += io/mtk/legacy/io.o

2	add building directory:
	include this Makefile and Kconfig in your parent directory

3	add defconfig
	CONFIG_TOUCHSCREEN_ATMEL_MXTS
	CONFIG_MXT_PLUGIN_SUPPORT
	CONFIG_MXT_PLIGIN_CAL
	CONFIG_MXT_PLIGIN_AC
	CONFIG_MXT_PLIGIN_PI
	CONFIG_MXT_PLIGIN_MISC
	CONFIG_MXT_PLIGIN_CLIP
	CONFIG_MXT_PLIGIN_WDG

4	config.h
	#define CONFIG_MXT_I2C_DMA
	#define CONFIG_MXT_I2C_EXTFLAG
	//-- if not use DTS  --
	#undef CONFIG_OF
	//-- if not use fb_callback  --
	#undef CONFIG_FB

5  io.h
	move from 'io/<arch>/<platform>/io.h' to 'io' directory('io'):
	mv io/mtk/legacy/io.h io/

6  io.c 
	check <your platform>/<dts or legacy>/io.c
	1 Reset pin
	2 interrupt pin
	3 interrupt number
	4 interrupt flags
	5 regulator/LDO power (VDDIO 1.8V first, then VDD 3.3V)
	6 i2c bus/device number

7  modify DTS if you use DTS file

