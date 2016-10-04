Robotics Cape Library Installer 
For BeagleBone Black and BeagleBone Blue

This is for Debian Jessie ONLY. 
See the wheezy_archive branch for Wheezy-compatible code.


Installation Instructions and Black or Blue

1) 	Flash your BBB to the latest stable Debian Jessie version.
   	Images and instructions avialabe here: 
	http://beagleboard.org/latest-images
   
   	For working encoder counting you will also need to install a 'bone' kernel
	root@beaglebone:~# apt-get install linux-image-4.4.23-bone14
  

2) SSH into your Beaglebone as the root user through USB or LAN.
   If you have difficulty connecting consult BeagleBoard.org
   http://beagleboard.org/Getting%20Started/


3) Download the Robotics_Cape_Installer folder to /root/
   
	Option 1: If you don't have an ethernet/wifi internet connection configured 
	on your BeagleBone Black or Blue then you must download the librobotics-cape
	source package to your host computer and then transfer to your beaglebone 
	over USB, usually to its address at 192.168.7.2 through SFTP. 
	
	Download from here:
	https://github.com/StrawsonDesign/Robotics_Cape_Installer/archive/master.zip

	Option 2: If your BeagleBone Black or Blue does have an internet connection
	configured then you can clone the source directly:
	
	root@beaglebone:~#git clone https://github.com/StrawsonDesign/Robotics_Cape_Installer.git


4) 	If you are installing on a BeagleBone Black, you may plug the cape in now.


5) 	Log in as root and execute the install.sh script. You will be prompted with
   	further instructions during installation. Reboot when complete.

   	root@beaglebone:~# cd Robotics_Cape_Installer
   	root@beaglebone:~/Robotics_Cape_Installer# bash install.sh
   	root@beaglebone:~/Robotics_Cape_Installer# reboot


6) 	If all went well, restarting will result in the selected example
  	program loading on boot. If the blink example was selected, both red and 
	green LEDs should be responsive to button presses. 



Package Installation for Black or Blue

If you are using a BeagleBone Blue with a working internet connection then you
may elect to pull the librobotics-cape package from a repository with apt.
This will also allow updates to the package for BeagleBone Black users with a
wifi dongle or ethernet connection. Note that BeagleBone Black users must first
run the install.sh script as described above to configure the cape overlay.
After that all updates to the library binary itself may be pulled using apt-get.

echo "deb http://strawsondesign.com debian/" > /etc/apt/sources.list
apt-get update
apt-get install librobotics-cape




