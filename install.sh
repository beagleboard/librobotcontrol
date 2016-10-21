#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# This is specifically for Debian Jessie


################################################################################
# Variables collected here for convenience
################################################################################

KERNEL="$(uname -r)"
DEBIAN="$(cat /etc/debian_version)"

echo " "
echo "Detected Linux kernel $KERNEL"
echo "Detected Debian version $DEBIAN"
cat /etc/dogtag
cat /proc/device-tree/model
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


# enable services
systemctl daemon-reload
systemctl enable battery_monitor
systemctl start battery_monitor
systemctl enable roboticscape


#################################################
# Prompt user for desired startup program
#################################################

echo " "
echo " "
echo " "
echo "roboticscape Package Installed"
echo "If you are not on a Bealgebone Blue, please"
echo "run configure_robotics_overlay.sh once to configure the"
echo "overlay, then reboot to load device tree. After rebooting"
echo "we suggest running calibrate_gyro and calibrate_mag."
