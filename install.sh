#!/bin/bash

#Bash script to install supporting software for the Robotics Cape

INSTALL_DIR="/home/root"

touch *

echo "Flashing Cape EEPROM"
cat install_files/data.eeprom > /sys/bus/i2c/devices/1-0054/eeprom 


echo "Compiling and Installing Device Tree Overlay"
dtc -O dtb -o /lib/firmware/SD-101B-00A0.dtbo -b 0 -@ install_files/SD-101B-00A0.dts
cp /boot/am335x-boneblack.dtb /boot/am335x-boneblack.dtb.old 
cp install_files/am335x-boneblack.dtb /boot/



echo "Copying uEnv.txt to Disable HDMI"
mount /dev/mmcblk0p1 /media/BEAGLEBONE >/dev/null
cp /media/BEAGLEBONE/uEnv.txt /media/BEAGLEBONE/uEnv.txt.old
cp install_files/uEnv.txt /media/BEAGLEBONE/

echo "Copying new pwm_test kernel Module"
cp /lib/modules/3.8.13/kernel/drivers/pwm/pwm_test.ko /lib/modules/3.8.13/kernel/drivers/pwm/pwm_test.ko.old
cp install_files/pwm_test.ko /lib/modules/3.8.13/kernel/drivers/pwm/
cp install_files/tieqep.ko /usr/bin/



echo "Enabling Boot Script"
#cp -r startup /home/root/
cp -f install_files/bootscript.sh /usr/bin/
chmod u+x /usr/bin/bootscript.sh
cp install_files/bootscript.service /lib/systemd/
rm /etc/systemd/system/bootscript.service
ln /lib/systemd/bootscript.service /etc/systemd/system/bootscript.service
systemctl daemon-reload 
systemctl enable bootscript.service 

echo "Installing Supporting Libraries"
cd libraries
make install
make clean
cd ../
cp -r libraries/ $INSTALL_DIR

echo "Installing Examples"
cp -r examples/ $INSTALL_DIR
cd examples
cp balance/balance 					/usr/bin/
cp bare_minimum/bare_minimum 		/usr/bin/
cp battery_monitor/battery_monitor  /usr/bin/
cp calibrate_esc/calibrate_esc		/usr/bin/
cp calibrate_spektrum/calibrate_spektrum 	/usr/bin/
cp fly/fly							/usr/bin/
cp test_buttons/test_buttons 		/usr/bin/
cp test_encoders/test_encoders 		/usr/bin/
cp test_esc/test_esc				/usr/bin/
cp test_imu/test_imu				/usr/bin/
cp test_motors/test_motors 			/usr/bin/
cp test_spektrum/test_spektrum 		/usr/bin/


cd ../
chmod 755 /usr/bin/*

echo "Installing Default Calibration Files"
cp -r cape_calibration/ $INSTALL_DIR


echo "which program should run on boot?"
select bfn in "balance" "fly" "none"; do
    case $bfn in
        balance ) echo "balance" >> /usr/bin/bootscript.sh; break;;
        fly ) echo "fly" >> /usr/bin/bootscript.sh; break;;
		none ) exit;;
    esac
done

echo
echo "Robotics Cape Configured and Installed"
echo "Reboot to complete installation."
echo

