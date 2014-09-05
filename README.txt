Robotics Cape Installer with BeagleMIP balance code
James Strawson - 2013

Installation Instructions:

1) Flash your BBB to the latest version using the eMMC image.
http://beagleboard.org/latest-images
This package is tested on 2013-09-04 images.

2) Copy the BeagleMIP_Installer folder to /home/root/
This is usually done through SFTP using a program like WinSCP

3) SSH into your Beaglebone through USB or LAN.
If you have difficulty connecting consult BeagleBoard.org
http://beagleboard.org/Getting%20Started/

3) Log in as root, execute the install.sh script and reboot
You will be prompted if you wish to run the BeagleMIP balance
or the BeagleQuad fly programs on boot. Type the number of the
item you wish to select and hit enter.

root@beaglebone:~#cd Robotics_Cape_Installer
root@beaglebone:~/Robotics_Cape_Installer# bash install.sh
root@beaglebone:~/Robotics_Cape_Installer# reboot

4) If all went well, restarting will result in the red LED 
turning on once the BBB has completely booted. To indicate the
balance program has started on boot.


BeagleMIP Use

1) Slide the motor driver switch to the 'on' position. Turn 
the motor drivers off when testing new code to prevent 
possible damage.

2) Lift your BeagleMIP to the upright position to start balancing. 

3) Hold the 'Select' button for 3 seconds to exit the balance program.

A copy of the balance program is in /usr/bin along with several
example programs. Any executable in this directory can be run from 
anywhere. Once your project is stable, you can run 'make install'
to place a copy of your program here. Only one cape project should 
be executed at a time to avoid conflicts. 

Located /home/root/startup is a bash script that is configured to
run on boot. By default it includes the balance program. Modify this
bootscript to include any other boot time configuration. Only run
'make install' on stable projects, especially if you have configured
them to run automatically on boot.

