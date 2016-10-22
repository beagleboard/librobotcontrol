#!/bin/bash

OVERLAY=RoboticsCape-00A0
CAPE=RoboticsCape
UENV=/boot/uEnv.txt

KERNEL="$(uname -r)"
UNAME="$(cat /boot/uEnv.txt | grep "uname_r=")"
DEBIAN="$(cat /etc/debian_version)"
UUID="$(blkid -c /dev/null -s UUID -o value /dev/mmcblk*)"
# get the model number so we can set the right device tree
MODEL="$(cat /proc/device-tree/model)"


################################################################################
# Sanity Checks
################################################################################

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

# # make sure the user really wants to install
# echo "This will configure uEnv.txt and install the Robotics Cape overlay."
# echo "This step is not needed on the BeagleBone Blue."
# read -r -p "Continue? [y/n] " response
# case $response in
#     [yY]) echo " " ;;
#     *) echo "cancelled"; exit;;
# esac
# echo " "

################################################################################
# off we go!
################################################################################


if   [ "$MODEL" == "TI AM335x BeagleBone Black" ]; then
	 DTB="am335x-boneblack-emmc-overlay.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Black Wireless" ]; then
	 DTB="am335x-boneblack-wireless.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Green" ]; then
	 DTB="am335x-bonegreen.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Green Wireless" ]; then
	 DTB="am335x-bonegreen-wireless.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Blue" ]; then
	 echo "No overlay needed on the Blue!"
	 exit 1
else
	 echo "unknown or unsupported BB model"
	 exit 1
fi

# put overlay into /lib/firmware if it is missing
# comes with newer images so this isn't really needed anymore
if [ ! -f "/lib/firmware/$OVERLAY.dtbo" ]; then
	cp /etc/robotics/$OVERLAY.dtbo /lib/firmware/
fi

# make backup if not already one
if [ -a "$UENV.backup" ];then
	echo "backup of $UENV already exists"
else
	echo "making backup copy of $UENV"
	cp $UENV $UENV.backup
fi

# wipe the file clean with an echo
echo " " > $UENV
echo "# this uEnv.txt created by configure_robotics_overlay.sh" >> $UENV
echo " " >> $UENV

# write in kernel name from last UENV file
# if it's empty use currently booted kernel instead
if [ "$UNAME" == "uname_r=" ]; then
	echo "uname_r=$KERNEL" >> $UENV
	echo "Setting kernel $KERNEL to load on boot"
else
	echo "Using previously listed kernel $UNAME"
	echo $UNAME >> $UENV
fi

# write in the device tree name
echo dtb=$DTB >> $UENV

# standard entries
echo cmdline=coherent_pool=1M >> $UENV
echo cape_enable=bone_capemgr.enable_partno=RoboticsCape >> $UENV

#uuid of emmc to boot from
echo uuid=$UUID >> $UENV
echo " " >> $UENV

# modify default cape to load
echo CAPE=$CAPE > /etc/default/capemgr

echo "Robotics Cape Overlay Configured and Installed"
