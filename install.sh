#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# This is specifically for Debian Jessie


################################################################################
# Variables collected here for convenience
################################################################################

KERNEL="$(uname -r)"
DEBIAN="$(cat /etc/debian_version)"
MODEL="$(cat /proc/device-tree/model)"
DOGTAG="$(cat /etc/dogtag)"

echo " "
echo "Detected Linux kernel: $KERNEL"
echo "Detected Debian version: $DEBIAN"
echo "Detected Model: $MODEL"
echo "Detected $DOGTAG"
echo " "
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
	exit 1
fi

# make sure the user really wants to install
echo "This script will install all Robotics Cape supporting software"
read -r -p "Continue? [y/n] " response
case $response in
    [yY]) echo " " ;;
    *) echo "cancelled"; exit;;
esac
echo " "



################################################################################
# Compile and install library, examples, and services
# This works for Black and Blue
################################################################################
find . -exec touch {} \;
bash debian/preinst
make clean
make install

# enable services
echo "systemctl daemon-reload"
systemctl daemon-reload
echo "Enabling battery_monitor Service"
systemctl enable battery_monitor
echo "Starting battery_monitor Service"
systemctl start battery_monitor
echo "Enabling roboticscape Service"
systemctl enable roboticscape

# set up overlay if not on the blue
if [ "$MODEL" != "TI AM335x BeagleBone Blue" ]; then
	echo "Configuring Device Tree Overlay"
	/usr/bin/configure_robotics_overlay.sh
fi


#################################################
# Prompt user for desired startup program
#################################################

echo " "
echo "Which program should run on boot?"
echo "Select 'blink' if you are unsure."
echo "Select 'balance' for BeagleMiP"
echo "Select 'none' to start nothing on boot"
echo "Select 'existing' to keep current configuration."

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
	ln -s -f /usr/bin/blink /etc/roboticscape/link_to_startup_program
elif  [ "$PROG" == "balance" ]; then
	ln -s -f /usr/bin/balance /etc/roboticscape/link_to_startup_program
elif  [ "$PROG" == "none" ]; then
	ln -s -f /usr/bin/bare_minimum /etc/roboticscape/link_to_startup_program
fi


#################################################
# Prompt user for desired startup program
#################################################

echo " "
echo " "
echo " "
echo "Robotics Cape Package Installed"
echo " "

# set up overlay if not on the blue
if [ "$MODEL" != "TI AM335x BeagleBone Blue" ]; then
	echo "Reboot to load device tree."
fi

