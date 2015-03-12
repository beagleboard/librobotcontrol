#!/bin/bash

# Bash script to install supporting software for the Robotics Cape
# tested on beagleboard.org Debian release 2014-05-14
# and 2015-03-01

INSTALL_DIR="/root"
BOOTSCRIPT="Auto_Run_Script.sh"

touch *
echo " "

# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to install this"
	exit
fi

# check what image is being used as the install differs
if grep -q "2015-03-01" /etc/dogtag; then
	echo "using Debian release 2015-03-01"
    IMG="2015-03-01"
elif grep -q "2014-05-14" /etc/dogtag; then
	echo "using Debian release 2014-05-14"
	IMG="2014-05-14"
else
	echo "please use Debian 2015-03-01 or 2014-05-14"
	exit
fi

# print kernel version, mostly for fun
KERN="$(uname -r)"
echo "using linux kernel $KERN"

echo " "
echo "Installing Device Tree Overlay"
cp install_files/SD-101C-00A0.dtbo /lib/firmware/SD-101C-00A0.dtbo

# if you want to recompile yourself use this
#dtc -O dtb -o /lib/firmware/SD-101C-00A0.dtbo -b 0 -@ install_files/SD-101C-00A0.dts

# HDMI must be disabled to open pins for cape use
echo "modifying uEnv.txt to Disable HDMI"
# uEnv.txt is located differently between releases
# set a variable with the path

if [ "$IMG" == "2014-05-14" ]; then
	ENVPATH="/boot/uboot/uEnv.txt"
elif [ "$IMG" == "2015-03-01" ]; then
	ENVPATH="/boot/uEnv.txt"
fi

# make a backup of the original file if it hasn't already been 
# done before by a previous installation
if [ -a "$ENVPATH.old" ];then
	echo "backup of $ENVPATH already exists"
else
	echo "making backup copy of $ENVPATH"
	cp $ENVPATH $ENVPATH.old
fi

# copy the appropriate line to disable HDMI
# if the user reinstalls the line is duplicated, but still works
echo "optargs=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN" >> $ENVPATH


# Now we must increase the I2C bus speed, this is done by tweaking the 
# am335x-boneblack.dtb file. Modified versions are included with the installer
# first make a backup copy, then pick which one to copy
if [ "$IMG" == "2014-05-14" ]; then
	AMPATH="/boot/uboot/dtbs/am335x-boneblack.dtb"
elif [ "$IMG" == "2015-03-01" ]; then
	AMPATH="/boot/dtbs/3.8.13-bone70/am335x-boneblack.dtb"
else
	echo "invalid IMG variable value $IMG"
fi
#make a backup if it hasn't been made by a previous installation
if [ -a "$AMPATH.old" ];then
	echo "backup of $AMPATH already exists"
else
	echo "making backup copy of $AMPATH"
	cp $AMPATH $AMPATH.old
fi
#copy the right file over
echo "installing new am335x-boneblack.dtb"
if [ "$IMG" == "2014-05-14" ]; then
	cp install_files/2014-05-14/am335x-boneblack.dtb $AMPATH
elif [ "$IMG" == "2015-03-01" ]; then
	cp install_files/2015-03-01/am335x-boneblack.dtb $AMPATH
else
	echo "invalid IMG variable value $IMG"
fi

# set SD-101C as the only cape to load besides eMMC
# single ">" means previous contents will be erased
echo "Setting Capemgr to Load Robotics Overlay by Default"
echo "CAPE=SD-101C" > /etc/default/capemgr


echo "Installing Supporting Libraries"
cd libraries
make clean > /dev/null
make install > /dev/null
make clean
cd ../

echo "Installing PRU Binaries and Assembler"
cp install_files/pru_servo.bin /usr/bin
cp install_files/pasm /usr/bin


echo "Installing examples, this may take a minute."
find examples/ -exec touch {} \;
cd examples
make clean > /dev/null
make install > /dev/null
make clean > /dev/null
cd ../


echo "Installing Default Calibration Files"
cp -r robot_config/ $INSTALL_DIR

mkdir /root/robot_logs


#the led_aging script causes problems in older images so this is a modified version
if [ "$IMG" == "2014-05-14" ]; then
	echo "upgrading /etc/init.d/led_aging.sh"
	cp install_files/led_aging.sh /etc/init.d/
fi

echo "Enabling Boot Script"
cp install_files/$BOOTSCRIPT $INSTALL_DIR
rm -f /etc/init.d/$BOOTSCRIPT
ln $INSTALL_DIR/$BOOTSCRIPT /etc/init.d/$BOOTSCRIPT 
chmod 755 $INSTALL_DIR/$BOOTSCRIPT
chmod 755 /etc/init.d/$BOOTSCRIPT
update-rc.d $BOOTSCRIPT defaults 

echo " "
echo "which program should run on boot?"
echo "type 1-5 then enter"
select bfn in "blink" "drive" "balance" "fly" "none"; do
    case $bfn in
		blink ) PROG="blink"; break;;
		drive ) PROG="drive"; break;;
        balance ) PROG="balance"; break;;
		fly ) PROG="fly"; break;;
		none ) PROG=" "; break;;
    esac
done

# put the right program in the auto run script
sed "s/#INSERT/$PROG/" install_files/Auto_Run_Script.sh > $INSTALL_DIR/Auto_Run_Script.sh

echo
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo

