#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# This is specifically for Debian Jessie


################################################################################
# Variables collected here for convenience
################################################################################

KERNEL="$(uname -r)"
UNAME="$(cat /boot/uEnv.txt | grep "uname_r")"
DEBIAN="$(cat /etc/debian_version)"
UUID = "$(shell blkid -c /dev/null -s UUID -o value /dev/mmcblk*)"

echo " "
echo "Detected Linux kernel $KERNEL"
echo "Detected Debian version $DEBIAN"
cat /etc/dogtag
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

echo "This script will install all Robotics Cape supporting software."
echo "Enter 1 to install on a BeagleBone Black"
echo "Enter 2 to install on Beaglebone Blue"
echo "Enter 3 to abort and exit"
select bfn in "black" "blue" "quit"; do
    case $bfn in
		black ) BOARD="black"; break;;
        blue ) BOARD="blue"; break;;
		quit ) exit;;
    esac
done


################################################################################
# Run Beaglebone Black specific installation steps
################################################################################
if [$BOARD == "black"]; then
	
	echo "Installing Cape Overlay"
	install -m 644 black_install_files/RoboticsCape-00A0.dtbo /lib/firmware/
	
	echo "Updating uEnv.txt"
	install -m 644 --backup=numbered ./uEnv.txt /boot/ 
	sed -i s/^uuid=.*\$$/uuid=$(UUID)/ $(UENV)
	sed -i s/^uname_r=.*\$$/$(UNAME)/ $(UENV)
	
	echo  "Setting Default Cape"
	echo "CAPE=RoboticsCape" > /etc/default/capemgr
fi

################################################################################
# Compile and install library, examples, and services
# This works for Black and Blue
################################################################################

#make install
#make clean


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

