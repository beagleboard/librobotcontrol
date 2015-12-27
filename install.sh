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
elif grep -q "2014-05-14" /etc/dogtag; then
	echo "using Debian release 2014-05-14"
	IMG="2014-05-14"
elif grep -q "2015-11-12" /etc/dogtag; then
	echo "using Debian release 2015-11-12"
	IMG="2015-11-12"
else
	echo "please use one of the following Debian images"
	echo "2014-05-14"
	echo "2015-03-01"
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

# change location of uEnv.txt and sitara  file based on image
if [ "$IMG" == "2014-05-14" ]; then
	UENV_TXT="/boot/uboot/uEnv.txt"
	AM335_DTB="/boot/uboot/dtbs/am335x-boneblack.dtb"
else
	UENV_TXT="/boot/uEnv.txt"
	AM335_DTB="/boot/dtbs/$KERNEL/am335x-boneblack.dtb"
fi

# detect if preemptive scheduling is enabled
# not necessary
if [ ! -f /sys/kernel/realtime ]; 
then
	echo "installing without realtime preemption"	
else
	echo "installing with realtime preemption"
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
		echo "error: PRU library not installed"
		echo "use the following script with an internet connection:"
		echo "bash upgrade_console_only_image.sh"
		exit
	fi
fi

# make sure the user really wants to install
echo "This script will install all Robotics Cape supporting software."
echo "This is for SD-101C Revision C capes ONLY!!!!"
read -r -p "Continue? [y/n] " response
case $response in
    [yY]) echo " " ;;
    *) echo "cancelled"; exit;;
esac
echo " "


# touch everything since the BBB clock is probably wrong
find . -exec touch {} \;

echo "Installing Device Tree Overlay"
if [ "$IMG" == "2015-11-12" ]; #  image includes dtc compiler
then dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ install_files/$OVERLAY.dts
# older images need pre-compiled dtbo
else cp install_files/$OVERLAY.dtbo /lib/firmware/$OVERLAY.dtbo
fi

# make a backup of the original uEnv.txt file
# if it doesn't already exist
if [ -a "$UENV_TXT.old" ];then
	echo "backup of $UENV_TXT already exists"
else
	echo "making backup copy of $UENV_TXT"
	cp $UENV_TXT $UENV_TXT.old
fi

# disable cape-universal in 2015-11-12 image
if [ "$IMG" == "2015-11-12" ]; then
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
if [ "$IMG" == "2014-05-14" ]; then
	cp install_files/$IMG/am335x-boneblack.dtb $AM335_DTB
elif [ "$IMG" == "2015-03-01" ]; then
	cp install_files/$IMG/am335x-boneblack.dtb $AM335_DTB
elif [ "$IMG" == "2015-11-12" ]; then
	cp install_files/$IMG/$DEBIAN/am335x-boneblack.dtb $AM335_DTB
else
	echo "invalid IMG variable value $IMG"
fi

# set Robotics Cape as the only cape to load
echo "Setting Capemgr to Load $CAPENAME Overlay by Default"
echo "CAPE=$CAPENAME" > /etc/default/capemgr

# also add to uEnv.txt even though this doesn't work until
# the cape is pushed upstream. here now in anticipation of that
echo "cape_enable=capemgr.enable_partno=SD-101C" >> $UENV_TXT


echo "Installing Supporting Libraries"
cd libraries
make clean > /dev/null
make install > /dev/null
make clean
cd ../

echo "Installing PRU Binaries and Assembler"
cp install_files/pru/pru_1_servo.bin /usr/bin
cp install_files/pru/pru_0_encoder.bin /usr/bin
cp install_files/pasm /usr/bin


echo "Installing examples, this will take a few minutes."
find examples/ -exec touch {} \;
cd examples
make clean > /dev/null
make install > /dev/null
make clean > /dev/null
cd ../

# install calibration files 
if [ ! -d "$INSTALL_DIR/robot_config" ]; then
	echo "Installing Default Calibration Files"
	cp -r robot_config/ $INSTALL_DIR
else
	#okay, the config folder exists, check status of each config file
	# STAT=0 > files are the same as default
	# STAT=1 > user has modified a file
	# STAT=2 > file doesn't exist yet
	cmp -s robot_config/gyro.cal $INSTALL_DIR/robot_config/gyro.cal
	GYRO_STAT=$?
	cmp -s robot_config/dsm2.cal $INSTALL_DIR/robot_config/dsm2.cal
	DSM_STAT=$?
	
	if [ $GYRO_STAT -eq "1" ]; then
		echo " "
		read -r -p "Would you like to keep your old gyro calibration file? [y/n] " response
		case $response in
			[yY]) 
				echo " "
				;;
			*)
				echo "writing new default gyro calibration file"
				cp robot_config/gyro.cal $INSTALL_DIR/robot_config/
				;;
		esac
	else
		echo "writing new default gyro calibration file"
		cp robot_config/gyro.cal $INSTALL_DIR/robot_config/
	fi
	
	if [ $DSM_STAT -eq "1" ]; then
		echo " "
		read -r -p "Would you like to keep your old DSM2 calibration file? [y/n] " response
		case $response in
			[yY]) 
				echo " "
				;;
			*)
				echo "writing new default DSM2 calibration file"
				cp robot_config/dsm2.cal $INSTALL_DIR/robot_config/
				;;
		esac
	else
		echo "writing new default DSM2 calibration file"
		cp robot_config/dsm2.cal $INSTALL_DIR/robot_config/
	fi
		
fi


# make a robot_logs directory if it doesn't exist
if [ -d $INSTALL_DIR/robot_logs ]; then
	echo "$INSTALL_DIR/robot_logs already exists"
else
	echo "creating log directory $INSTALL_DIR/robot_logs"
	mkdir $INSTALL_DIR/robot_logs
fi

#the led_aging script causes problems in old image
#install modified version
if [ "$IMG" == "2014-05-14" ]; then
	echo "upgrading /etc/init.d/led_aging.sh"
	cp install_files/2015-05-14/led_aging.sh /etc/init.d/
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

echo " "
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo "After Rebooting we suggest running 'calibrate_gyro'"
echo " " 

