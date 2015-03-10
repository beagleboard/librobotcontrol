#!/bin/bash

# this script installs the missing components to make the 2015-03-01
# console-only image work with the Robotics Cape software package
# the following packages are installed:
# - build-essential for make and gcc
# - PRU module library
# - usb network service


apt-get update
apt-get install build-essential

git clone https://github.com/beagleboard/am335x_pru_package.git
cd am335x_pru_package/
make 
make install

apt-get install udhcpd
 