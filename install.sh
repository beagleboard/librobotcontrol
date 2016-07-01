#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# tested on follwing beagleboard.org Debian releases 
# 2014-05-14, 2015-03-01, 2015-11-12

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


# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to install this"
	exit
fi

# check what image and kernel is being used as the install differs
if grep -q "2015-03-01" /etc/dogtag; then
	echo "using Debian release 2015-03-01"
    IMG="2015-03-01"
elif grep -q "2015-11-12" /etc/dogtag; then
	echo "using Debian release 2015-11-12"
	IMG="2015-11-12"
else
	echo "please use one of the following Debian images"
	echo "2015-03-01 (Wheezy)"
	echo "2015-11-12 (Wheezy)"
	exit
fi 
echo "using linux kernel $KERNEL"

# warn the user if using a 'ti' kernel instead of a 'bone' kernel
if uname -r | grep -q "ti"; then
	echo "WARNING: the 'ti' kernels do not necessarily"
	echo "support the uio-pruss PRU driver."
	echo "we suggest using a 'bone' kernel"
fi

#check that the uio-pruss driver is available
if modprobe -n uio-pruss | grep -q "not found"; then
	echo "ERROR: uio-pruss driver missing."
	echo "We suggest using the latest Debian Wheezy image."
	echo "The Debian Jessie image does not yet support the PRU."
	exit
fi

#check dependencies
if [ ! -f /usr/bin/make ]; then
	echo " "
    echo "error: dependency 'make' not installed"
	echo "use apt-get install build-essentials"
	echo "OR, if you are using a console-only image:"
	echo "bash upgrade_console_only_image.sh"
	exit
fi

if [ ! -f /usr/bin/gcc ]; then
	echo " "
    echo "error: dependency 'gcc' not installed"
	echo "use apt-get install build-essentials"
	echo "OR, if you are using a console-only image:"
	echo "bash upgrade_console_only_image.sh"
	exit
fi

if [ ! -f /usr/lib/libprussdrv.so ]; then
	if [ ! -f /usr/lib/local/libprussdrv.so ]; then
		echo " "
		echo "error: libprussdrv missing"
		exit
	fi
fi

# make sure the user really wants to install
echo ""
echo "This script will install all Robotics Cape supporting software."
read -r -p "Continue? [y/n] " response
case $response in
    [yY]) echo " " ;;
    *) echo "cancelled"; exit;;
esac
echo " "


# touch everything since the BBB clock is probably wrong
find . -exec touch {} \;

echo "Installing Device Tree Overlay"
# newer images ship with dtc compiler
dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ install_files/$OVERLAY.dts


# make a backup of the original uEnv.txt file
# if it doesn't already exist
if [ -a "$UENV_TXT.old" ];then
	echo "backup of $UENV_TXT already exists"
else
	echo "making backup copy of $UENV_TXT"
	cp $UENV_TXT $UENV_TXT.old
fi

# disable cape-universal in 2015-11-12 image and newer
# 2015-03-01 was before cape-universal so not needed
if [ "$IMG" != "2015-03-01" ]; then
	sed -i '/cape_universal=enable/ s??#cape_universal=enable?' $UENV_TXT
fi

# all images need HDMI disabled
echo "optargs=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN" >> $UENV_TXT


# Now we must increase the I2C bus speed, this is done by tweaking the 
# am335x-boneblack.dtb file. Modified versions are included with the installer
#make a backup if it hasn't been made by a previous installation
if [ -a "$AM335_DTB.old" ];then
	echo "backup of $AM335_DTB already exists"
else
	echo "making backup copy of $AM335_DTB"
	cp $AM335_DTB $AM335_DTB.old
fi
#copy the right file over
echo "installing new am335x-boneblack.dtb"
if [ $IMG == "2015-03-01" ]; then
	cp install_files/$IMG/am335x-boneblack.dtb $AM335_DTB
elif [ $IMG == "2015-11-12" ]; then
	cp install_files/$IMG/$DEBIAN/am335x-boneblack.dtb $AM335_DTB
else
	echo "invalid IMG variable value $IMG"
fi

# set Robotics Cape as the only cape to load
echo "Setting Capemgr to Load $CAPENAME Overlay by Default"
echo "CAPE=$CAPENAME" > /etc/default/capemgr

# also add to uEnv.txt even though this doesn't work until
# the cape is pushed upstream. here now in anticipation of that
echo "cape_enable=capemgr.enable_partno=$CAPENAME" >> $UENV_TXT

echo "Installing PRU Binaries"
cp install_files/pru/pru_1_servo.bin /usr/bin
cp install_files/pru/pru_0_encoder.bin /usr/bin


echo "Installing Supporting Libraries"
cd libraries
make clean > /dev/null
make install > /dev/null
make clean
cd ../


echo "Installing examples, this will take a minute."
find examples/ -exec touch {} \;
cd examples
make clean > /dev/null
make install > /dev/null
make clean > /dev/null
cd ../

# make sure config diectory exists
if [ ! -a "$CONFIG_DIR" ]; then
	mkdir $CONFIG_DIR
fi

# make sure Auto Run directory exists
if [ ! -a "$AUTO_RUN_DIR" ]; then
	echo "creating directory " $AUTO_RUN_DIR
	mkdir $AUTO_RUN_DIR
fi

# set up a script to run things on boot
echo "Enabling Boot Script"
cp install_files/$BOOTSCRIPT /etc/init.d/
chmod 755 /etc/init.d/$BOOTSCRIPT
update-rc.d $BOOTSCRIPT defaults 

echo " "
echo "Which program should run on boot?"
echo "This will overwrite any program in " $AUTO_RUN_DIR
echo "Select 'existing' to keep current configuration."
echo "type 1-5 then enter"
select bfn in "blink" "balance" "none" "existing"; do
    case $bfn in
		blink ) PROG="blink"; break;;
        balance ) PROG="balance"; break;;
		none ) PROG="none"; break;;
		existing ) PROG="existing"; break;;
    esac
done

# if user didn't select existing, delete everything in auto run folder
# and replace with new program
if [ "$PROG" != "existing" ]; then
	rm -f $AUTO_RUN_DIR/*
	if [ "$PROG" != "none" ]; then
		cp /usr/bin/$PROG $AUTO_RUN_DIR/
	fi
fi

# all done
echo " "
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo "After Rebooting we suggest running calibrate_gyro"
echo " " 

