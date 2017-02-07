#!/bin/bash

OVERLAY=RoboticsCape
TREE_BLACK_RC=am335x-boneblack-roboticscape.dtb
TREE_BW_RC=am335x-boneblack-wireless-roboticscape.dtb
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
# check for the -f force flag
################################################################################
FORCE=NO
while [[ $# -gt 0 ]]; do
	key="$1"
	case $key in
		-f|--force)
		FORCE=YES
		echo "force enabled"
		shift
		;;
		*)
		 echo "Unknown Argument, continuing anyway"
		;;
	esac
	shift
done

################################################################################
# off we go!
################################################################################


# here we decide what device tree to use
# Blue needs no modification
if [ "$MODEL" == "TI AM335x BeagleBone Blue" ]; then
	echo "No overlay needed on the Blue!"
	exit 0
	
# if black and black wireless already have the DT installed, nothing to do
elif   [ "$MODEL" == "TI AM335x BeagleBone Black RoboticsCape" ]; then
	echo "Detected BB Black with RoboticsCape device tree already installed\n"
	echo "No changes required\n"
	exit 0
	
elif   [ "$MODEL" == "TI AM335x BeagleBone Black Wireless RoboticsCape" ]; then
	echo "Detected BB Black Wireless with RoboticsCape device tree already installed\n"
	echo "No changes required\n"
	exit 0


# test for BBB wireless
elif   [ "$MODEL" == "TI AM335x BeagleBone Black Wireless" ]; then

	# if the roboticscape tree is available, use that
	if [ -a "/boot/dtbs/$UNAME/$TREE_BW_RC" ]; then
		DTB="$TREE_BW_RC"
	else
		echo "ERROR, can't find $TREE_BW_RC for this kernel."
		echo "no changes made to uEnv.txt"
	fi

# test for BBB
elif   [ "$MODEL" == "TI AM335x BeagleBone Black" ]; then

	# if the roboticscape tree is available, use that
	if [ -a "/boot/dtbs/$UNAME/$TREE_BLACK_RC" ]; then
		DTB="$TREE_BLACK_RC"
	else
		echo "ERROR, can't find $TREE_BLACK_RC for this kernel."
		echo "no changes made to uEnv.txt"
	fi
	
# for all others (green, etc) make sure the force argument was given, 
# otherwise use the black_rc_overlay
else  
	if [ "$FORCE" == "NO" ]; then
		echo " "
		echo "RoboticsCape library only designed to work with Black, Black wireless, and Blue"
		echo "At your own risk, you can try the normal BB Black RoboticsCape device tree."
		echo "Please run 'configure_robotics_dt.sh -f' manually to force this operation."
		echo " "
		exit 0
	else
		# if the roboticscape tree is available, use that
		if [ -a "/boot/dtbs/$UNAME/$TREE_BLACK_RC" ]; then
			DTB="$TREE_BLACK_RC"
			echo "Forcing use of $TREE_BLACK_RC on untested board!"
		else
			echo "ERROR, can't find $TREE_BLACK_RC for this kernel."
			echo "no changes made to uEnv.txt"
		fi
	fi
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
echo "# this uEnv.txt created by configure_robotics_dt.sh" >> $UENV
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
if [ "$DTB" != "$TREE_BLACK_RC" ] && [ "$DTB" != "$TREE_BW_RC" ]; then
	echo cape_enable=bone_capemgr.enable_partno=$OVERLAY >> $UENV
	# modify default cape to load in case missing from initramfs
	echo CAPE=$OVERLAY > /etc/default/capemgr
	echo "enabling overlay"
fi

#uuid of emmc to boot from
echo uuid=$UUID >> $UENV
echo " " >> $UENV


echo "Robotics Cape Device Tree Configured and Installed"

exit 0
