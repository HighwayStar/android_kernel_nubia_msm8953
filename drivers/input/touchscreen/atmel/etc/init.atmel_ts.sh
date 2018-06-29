#!/system/bin/sh

sleep 2

insmod /data/temp/mxt.ko

cd /sys/bus/i2c/devices/3-0047

#chown root root *
#chmod 666 t19
#chmod 666 update_cfg
#chmod 666 update_fw

#format: [family id]_[variant id]_[version]_[build].fw
#format: [xxx].raw


#use GPIO 19 for diffferent hw identify
#1 : GPIO+I2C address for config match
#	format: [xxx].raw.[hex_i2c_address].[hex_gpio_t19].cfg
#2 : UID for config match
#format: [xxx].raw.[UID TAG]
#format: [xxx].fw.[UID TAG]
echo "2" > t19

#for pid (pid name = 01)
#echo "A4_15_2.2_E0.fw" > update_fw
#    switch to "A4_15_2.2_E0.fw.01"
#for pid (pid name = 02)
#echo "A4_15_3.0_AB.fw" > update_fw
#    switch to "A4_15_3.0_AB.fw.02"

#update new config
#echo "A4_15.raw" > update_cfg
#for gpio 19 alternative(i2c address 0x4b, gpio=01)
#    switch to "A4_15.raw.4B.01.cfg"
#for pid (pid name = 01)
#    switch to "A4_15.raw.01"

#send self tune command for 540s
#0 : no backup
#1 : backup
#echo 0 > self_tune
#dmesg > /cache/atmel_ts.log

sleep 1

#enable plugin
#
#format: pl enable [hex]
#[0] : CAL
#[1] : MSC
#[2] : PI
#[3] : CLIP
#[4] : WDG
#[7] : PLUG PAUSE
#echo "pl enable 2" > plugin

#PTC auto tune (should enable MSC above,sleep 5s for tune complete)
#[0] : tune not store
#[1] : tune and store
#[2] : re-tune and not store
#[3] : re-tune and store
#[other value] : report tune status 
#echo "msc ptc tune 1" > plugin
#sleep 7

#set gesture list
#format: <name> <val>;<name> <val>;...
#you could run command "cat gesture_list" for current config list
#<val>: bit[0]: enable
#	bit[1]: disable, 
#	bit[3]  status (1: excuted)
echo "LEFT 1;UNLOCK0 1;e 1;" > gesture_list

#enable gesture feature
echo 1 > en_gesture

#set clip plugin
# set edge clip parameters
#echo "clp cl area[0]: 0,0 33,1920" > plugin
#echo "clp cl area[1]: 0,0 1080,0" > plugin
#echo "clp cl area[2]: 1046,0 1080,1920" > plugin
#echo "clp cl area[3]: 0,1920 1080,1920" > plugin
#echo "clp cl dist: 0,300" > plugin
# set palm event parameters
#echo "clp pa numtch: 0" > plugin
#echo "clp pa thld: 55 30 0 35" > plugin
#echo "clp enable 3" >  plugin
#set watch dog plugin
#	bit[0]: PALM
#	bit[1]  CLIP
#	bit[2]  SUP
#	bit[3]  POS
#echo "wd enable 3" > plugin
echo "pl enable 1e" > plugin
#chmod 440 t19
#chmod 440 update_cfg
#chmod 440 update_fw

