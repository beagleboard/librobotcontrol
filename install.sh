#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# This is specifically for Debian Jessie, tested on the following images:
# 2016-05-13


################################################################################
# Variables collected here for convenience
################################################################################
INSTALL_DIR="/root"
OVERLAY="RoboticsCape-00A0"
CAPENAME="RoboticsCape"
UNAME="$(uname -r)"
UUID="$(blkid -c /dev/null -s UUID -o value /dev/mmcblk*)"
DEBIAN="$(cat /etc/debian_version)"
UENV_TXT="/boot/uEnv.txt"
CONFIG_DIR="/etc/robotics"


echo " "
echo "Detected Linux kernel $KERNEL"
echo "Detected Debian version $DEBIAN"
echo " "

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

# # touch everything since the BBB clock is probably wrong
# find . -exec touch {} \;


################################################################################
# install cape overlay stuff
################################################################################

# make a backup of the original uEnv.txt file
# if it doesn't already exist
if [ -a "$UENV_TXT.original" ];then
	echo "backup of $UENV_TXT already exists"
else
	echo "making backup copy of $UENV_TXT"
	cp $UENV_TXT $UENV_TXT.original
fi

# copy new uEnv.txt file over and plug in uname and uuid
cp install_files/uEnv.txt $UENV_TXT
sed -i "s/^uuid=.*\$/uuid=$UUID/" $UENV_TXT
sed -i "s/^uname_r=.*\$/uname_r=$UNAME/" $UENV_TXT

# set Robotics Cape as the only cape to load
echo "Setting Capemgr to Load $CAPENAME Overlay by Default"
echo "CAPE=$CAPENAME" > /etc/default/capemgr



################################################################################
# Compile and install library, then examples, then battery monitor service
################################################################################

make install
make clean


#################################################
# Prompt user for desired startup program
#################################################

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

# now make a link to the right program
# if 'none' was selected then leave default as bare_minimum (does nothing)
if [ "$PROG" == "blink" ]; then
	ln -s /usr/bin/blink /etc/robotics/link_to_startup_program
elif  [ "$PROG" == "balance" ]; then
	ln -s /usr/bin/balance /etc/robotics/link_to_startup_program
elif  [ "$PROG" == "none" ]; then
	ln -s /usr/bin/bare_minimum /etc/robotics/link_to_startup_program
fi





############
# all done
############
echo " "
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo "After Rebooting we suggest running calibrate_gyro and calibrate_mag"
echo " " 

