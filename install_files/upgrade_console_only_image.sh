#!/bin/bash

# this script installs the missing components to make the 2015-03-01
# console-only image work with the Robotics Cape software package
# the following packages are installed:
# - build-essential for make and gcc
# - PRU module library
# - usb network service

echo " "
echo "This script will install necessary dependencies"
echo "that are not included with the console-only images"
echo "Make sure your BBB is connected to the internet."
echo " "

read -r -p "Continue? [y/n] " response
case $response in
    [yY]) 
        echo " "
        ;;
    *)
		echo "cancelled"
        exit
        ;;
esac

apt-get update
apt-get install build-essential
apt-get install git

git clone https://github.com/beagleboard/am335x_pru_package.git
cd am335x_pru_package/
make 
make install

apt-get install udhcpd

echo " "
echo "all done!"
echo " "