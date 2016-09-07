#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# This is specifically for Debian Jessie, tested on the following images:
# 2016-05-13

INSTALL_DIR="/root"
BOOTSCRIPT="Auto_Run_Script.sh"
OVERLAY="RoboticsCape-00A0"
CAPENAME="RoboticsCape"
KERNEL="$(uname -r)"
DEBIAN="$(cat /etc/debian_version)"
UENV_TXT="/boot/uEnv.txt"
AM335_DTB="/boot/dtbs/$KERNEL/am335x-boneblack.dtb"
CONFIG_DIR="/etc/robotics"
AUTO_RUN_DIR="/root/Auto_Run_Programs"


echo " "
echo "Detected Linux kernel $KERNEL"
echo "Detected Debian version $DEBIAN"
echo " "

####################
# Sanity Checks
####################

# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to install this."
	exit 1
fi

# make sure the release is really jessie
if ! grep -q "8." /etc/debian_version ; then
	echo "ERROR: This is not Debian Jessie."
	echo "Flash the latest Jessie image to your BBB"
	echo "or use the Wheezy branch of this installer."
	exit 1
fi

#check that the remoteproc driver is there
if modprobe -n remoteproc | grep -q "not found" ; then
	echo "ERROR: remoteproc module not found"
	echo "Use a standard TI kernel with remoteproc instead."
	exit
fi

# make sure the user really wants to install
echo "This script will install all Robotics Cape supporting software."
read -r -p "Continue? [y/n] " response
case $response in
    [yY]) echo " " ;;
    *) echo "cancelled"; exit;;
esac
echo " "


###############
# install
###############

# # touch everything since the BBB clock is probably wrong
# find . -exec touch {} \;

echo "Installing Device Tree Overlay"
cp install_files/$OVERLAY.dtbo /lib/firmware/$OVERLAY.dtbo

# make a backup of the original uEnv.txt file
# if it doesn't already exist
if [ -a "$UENV_TXT.old" ];then
	echo "backup of $UENV_TXT already exists"
else
	echo "making backup copy of $UENV_TXT"
	cp $UENV_TXT $UENV_TXT.old
fi

# disable cape-universal
sed -i '/cape_universal=enable/ s??#cape_universal=enable?' $UENV_TXT

# disable HDMI
echo "optargs=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN" >> $UENV_TXT

# set Robotics Cape as the only cape to load
echo "Setting Capemgr to Load $CAPENAME Overlay by Default"
echo "CAPE=$CAPENAME" > /etc/default/capemgr

# also add to uEnv.txt even though this doesn't work until the cape overlay is 
# pushed upstream. here now in anticipation of that
echo "cape_enable=capemgr.enable_partno=$CAPENAME" >> $UENV_TXT

# copy precompiles pru binaries to firmware
echo "Installing PRU Binaries"
# cp install_files/pru/pru_1_servo.bin /usr/bin
# cp install_files/pru/pru_0_encoder.bin /usr/bin

# compile and install robotics_cape.so
echo "Installing Supporting Libraries"
cd libraries
make clean > /dev/null
make install > /dev/null
make clean
cd ../

# compile all examples, this is slow
echo "Installing examples, this will take a minute."
find examples/ -exec touch {} \;
cd examples
make clean > /dev/null
make install > /dev/null
make clean > /dev/null
cd ../

# make a config directory if it doesn't exist
if [ ! -d "$CONFIG_DIR" ]; then
	echo "Making config directory $CONFIG_DIR"
	mkdir $CONFIG_DIR
fi

# install the battery_monitor service
cd battery_monitor_service
bash install.sh
cd ../


#################################
# set up the startup service
#################################

echo " "
echo "Which program should run on boot?"
echo "Select 'existing' to keep current configuration."
echo "Select blink if you are unsure."
echo "type 1-4 then enter"
select bfn in "blink" "balance" "none" "existing"; do
    case $bfn in
		blink ) PROG="blink"; break;;
        balance ) PROG="balance"; break;;
		none ) PROG="bare_minimum"; break;;
		existing ) PROG="existing"; break;;
    esac
done


# copy the service file over but don't replace if it's already there
# this will preserve an existing one if 

# if user selected existing, copy the service file with -n option
# to preserve it if it exists but still put it there if not
# otherwise force it
if [ "$PROG" == "existing" ]; then
	cp -n install_files/robot.service /etc/systemd/system/
else
	cp install_files/robot.service /etc/systemd/system/
fi

# now put in the right value for blink or balance
# if 'none' was selected then leave default as bare_minimum (does nothing)
if [ "$PROG" == "blink" ]; then
	sed -i "/ExecStart=/c\ExecStart=/usr/bin/blink" /etc/systemd/system/robot.service 
elif  [ "$PROG" == "balance" ]; then
	sed -i "/ExecStart=/c\ExecStart=/usr/bin/balance" /etc/systemd/system/robot.service 
fi


# reload the daemon and enable the service
systemctl daemon-reload
systemctl enable battery_monitor




############
# all done
############
echo " "
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo "After Rebooting we suggest running calibrate_gyro and calibrate_mag"
echo " " 

