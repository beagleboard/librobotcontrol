#!/bin/bash

OVERLAY=RoboticsCape
TREE=am335x-boneblack-roboticscape.dtb
UENV=/boot/uEnv.txt

KERNEL="$(uname -r)"
UNAME="$(sed -n -e '/uname_r=/ s/.*\= *//p' /boot/uEnv.txt)"
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


################################################################################
# off we go!
################################################################################


# use custom boneblack-roboticscape dtb if it's there
# otherwise we will have to load the overlay later
if   [ "$MODEL" == "TI AM335x BeagleBone Black" ]; then
	 
	 if [ -a "/boot/dtbs/$UNAME/$TREE" ]; then
	 	DTB="$TREE"
	 else
	 	DTB="am335x-boneblack-emmc-overlay.dtb"
	 fi

elif [ "$MODEL" == "TI AM335x BeagleBone Black Wireless" ]; then
	 DTB="am335x-boneblack-wireless.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Green" ]; then
	 DTB="am335x-bonegreen.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Green Wireless" ]; then
	 DTB="am335x-bonegreen-wireless.dtb"
elif [ "$MODEL" == "TI AM335x BeagleBone Blue" ]; then
	 echo "No overlay needed on the Blue!"
	 exit 0
else
	 echo "unknown or unsupported BB model"
	 exit 1
fi

echo "Using $DTB"


# make backup if not already one
if [ -a "$UENV.backup" ]; then
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
if [ ! "$UNAME" ]; then
	echo "uname_r=$KERNEL" >> $UENV
	echo "Setting kernel $KERNEL to load on boot"
else
	echo "Using previously listed kernel $UNAME"
	echo "uname_r=$UNAME" >> $UENV
fi

# write in the device tree name
echo dtb=$DTB >> $UENV

# standard entry
echo cmdline=coherent_pool=1M >> $UENV

# if not using custom device tree, load the overlay
if [ "$DTB" != "$TREE" ]; then
	echo cape_enable=bone_capemgr.enable_partno=$OVERLAY >> $UENV
	echo "enabling overlay"
fi

#uuid of emmc to boot from
echo uuid=$UUID >> $UENV
echo " " >> $UENV

# modify default cape to load
echo CAPE=$OVERLAY > /etc/default/capemgr

echo "Robotics Cape Device Tree Configured and Installed"

exit 0
