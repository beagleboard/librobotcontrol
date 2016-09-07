Robotics Cape Installer for BeagleBone Black
James Strawson - 2016

This is an unstable development branch for Debian Jessie only. 
For now use Wheezy and the master branch.


Installation Instructions:

1) Flash your BBB to the latest stable DebianvJessie version.
   Images and instructions avialabe here: http://beagleboard.org/latest-images
   
   This branch is tested on the following Jessie release: 2016-05-13

  
2) Copy the Robotics_Cape_Installer folder to /root/
   This is usually done through SFTP using a program like WinSCP


3) SSH into your Beaglebone through USB or LAN.
   If you have difficulty connecting consult BeagleBoard.org
   http://beagleboard.org/Getting%20Started/


4) Make sure the Robotics cape is plugged into your BBB


5) Log in as root and execute the install.sh script. You will be prompted with
   further instructions. Reboot when complete.

   root@beaglebone:~#cd Robotics_Cape_Installer
   root@beaglebone:~/Robotics_Cape_Installer# bash install.sh
   root@beaglebone:~/Robotics_Cape_Installer# reboot


6) If all went well, restarting will result in the selected example
   program loading on boot. For the balance example, a red LED will turn on 
   once the BBB has completely booted beofre turning green a few seconds later
   to indicate it is ready to be picked up to start balancing.
   If the blink example was selected, both red and green LEDs should be
   responsive to button presses. 


7) There should be a new folder located at /root/Auto_Run_Programs containing 
   the program you elected to run on boot. Any binaries placed in this folder 
   will be started automatically on boot once the robotics cape overlay and all
   drivers necessary for robotics cape function are ready.
