Robotics Cape Rev D Installer for BeagleBone Black
James Strawson - 2015

For older Revision C installer see the Rev C Branch
Latest version will always be the master branch.

Installation Instructions:

1) Flash your BBB to one of the following stable Wheezy images.
	2015-03-01  or  2015-11-12
	http://beagleboard.org/latest-images

2) Copy the Robotics_Cape_Installer folder to /root/
This is usually done through SFTP using a program like WinSCP


3) SSH into your Beaglebone through USB or LAN.
If you have difficulty connecting consult BeagleBoard.org
http://beagleboard.org/Getting%20Started/


4) Log in as root, execute the install.sh script and reboot.
You will be prompted if you wish to run one of the main example programs. 
Type the number of the item you wish to select and hit enter.

root@beaglebone:~#cd Robotics_Cape_Installer
root@beaglebone:~/Robotics_Cape_Installer# bash install.sh
root@beaglebone:~/Robotics_Cape_Installer# reboot


5) If all went well, restarting will result in the selected example
program loading on boot. For the 'balance' and 'drive' examples, the red LED 
will turn on once the BBB has completely booted. For the 'blink' example, both
LEDs and buttons should be responsive.


6) Your BeagleBone will now run any program contained in
/root/Auto_Run_Programs automatically on boot. The install script will have
already placed a program in there based on your selection in the last step of
the install process. While you may put multiple programs in there, note that
only one program using the Robotics Cape library will run at a time.