Robotics Cape Supporting Libraries
James Strawson - 2013

Contains libraries to assist in the use of your Robotics Cape.


Installation Instructions:

Log in as root and execute the install script and reboot
root@beaglebone:~#cd libraries
root@beaglebone:~/libraries# make clean
root@beaglebone:~/libraries# make install
If all went well, you now have shared libraries
located in /usr/lib and /usr/include


This is done for you in the BeagleMIP_Installer. 
If you modify or add to these libraries you can
recompile and install them with install.sh