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

# kernel version check
MINDEBIAN=8.6
MINKERNEL="4.4.32"
function version_lt() { test "$(echo "$@" | tr " " "\n" | sort -rV | head -n 1)" != "$1"; }

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

# check that the remoteproc driver is there
if modprobe -n remoteproc | grep -q "not found" ; then
	echo "ERROR: remoteproc module not found"
	echo "Use a standard TI kernel with remoteproc instead."
	exit 1
fi

# debian version check
if version_lt $DEBIAN $MINDEBIAN; then
	echo "WARNING: Debian version $MINDEBIAN or newer is required"
	exit 1
fi

# kernel version check
if version_lt $KERNEL $MINKERNEL; then
	echo "WARNING: Kernel $MINKERNEL or newer is required for full functionality"
	echo "Motor 1, PINMUX, and PRU functions will not work as-is"
	echo "You may still continue the installation"
	echo " "
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
ldconfig

###############################################################################
# Normally the package manager would call postinst here, but we don't want
# to invoke debconf when installing from source, so we re-implement the
# postinst script here
##############################################################################
# enable services
echo "systemctl daemon-reload"
systemctl daemon-reload
echo "Enabling roboticscape Service"
systemctl enable roboticscape
# don't enable battery_monitor on BB Green Wireless
if [ ! "$MODEL" == "TI AM335x BeagleBone Green Wireless" ]; then
	echo "Enabling rc_battery_monitor Service"
	systemctl enable rc_battery_monitor
	echo "Starting rc_battery_monitor Service"
	systemctl start rc_battery_monitor
fi

echo "Configuring Device Tree"
bash /usr/bin/configure_robotics_dt.sh


#################################################
# Prompt user for desired startup program
#################################################

echo " "
echo "Which program should run on boot?"
echo "Select 'rc_blink' if you are unsure."
echo "Select 'rc_balance' for BeagleMiP"
echo "Select 'none' to start nothing on boot"
echo "Select 'existing' to keep current configuration."

echo "type 1-4 then enter"
select bfn in "rc_blink" "rc_balance" "none" "existing"; do
	case $bfn in
		rc_blink ) PROG="rc_blink"; break;;
		rc_balance ) PROG="rc_balance"; break;;
		none ) PROG="rc_bare_minimum"; break;;
		existing ) PROG="existing"; break;;
	esac
done

# now make a link to the right program
# if 'none' was selected then leave default as bare_minimum (does nothing)
if [ "$PROG" == "rc_blink" ]; then
	ln -s -f /usr/bin/rc_blink /etc/roboticscape/link_to_startup_program
elif  [ "$PROG" == "rc_balance" ]; then
	ln -s -f /usr/bin/rc_balance /etc/roboticscape/link_to_startup_program
elif  [ "$PROG" == "none" ]; then
	ln -s -f /usr/bin/rc_bare_minimum /etc/roboticscape/link_to_startup_program
fi



echo " "
echo " "
echo " "
echo "Robotics Cape package installation complete."
echo "Please reboot now."
echo " "
