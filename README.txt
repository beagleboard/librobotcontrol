Robotics Cape Rev D Installer for BeagleBone Black
James Strawson - 2015

For older Revision C installer see the Rev C Branch
Latest version will always be the master branch.

Installation Instructions:

1) Flash your BBB to the latest version using the eMMC image.
http://beagleboard.org/latest-images
This package is tested on the following Debian Wheezy images
2014-05-14 
2015-03-01
2015-11-12

2) Copy the Robotics_Cape_Installer folder to /root/
This is usually done through SFTP using a program like WinSCP


3) SSH into your Beaglebone through USB or LAN.
If you have difficulty connecting consult BeagleBoard.org
http://beagleboard.org/Getting%20Started/


4) Make sure the Robotics cape is plugged into your BBB


5) Log in as root, execute the install.sh script and reboot
You will be prompted if you wish to run the BeagleMIP balance
or the BeagleQuad fly programs on boot. Type the number of the
item you wish to select and hit enter.

root@beaglebone:~#cd Robotics_Cape_Installer
root@beaglebone:~/Robotics_Cape_Installer# bash install.sh
root@beaglebone:~/Robotics_Cape_Installer# reboot


6) If all went well, restarting will result in the selected example
program loading on boot. For the Balance and Fly examples, the red LED 
will turn on once the BBB has completely booted. To indicate the
balance program has started on boot.



7) Located at /root/Auto_Run_Script.sh is a bash script that is 
configured to run on boot. By default it includes whatever program 
you selected during the install process in step 5. Modify this
bootscript to include any other boot time configuration.
