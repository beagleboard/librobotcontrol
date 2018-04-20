Robotics Cape Library                   {#mainpage}
===============================

This package contains the C library and example/testing programs for the BeagleBone Black with Robotics Cape and BeagleBone Blue. The master branch is always the most current but not necessarily stable. See other branches and [releases](https://github.com/StrawsonDesign/Robotics_Cape_Installer/releases) page for older stable versions or install from BeagleBoard.org repositories as described below.

Full API documentation and examples at <http://strawsondesign.com/docs/roboticscape/>.


## Installation
#### From Clean BeagleBoard.org Image
Stable RoboticsCape library releases are pre-installed with the BeagleBoard stable images. We currently recommend BeagleBone Black and Black Wireless users flash the 'BBB-blank' Debian flasher image from 3-7-2017 available [here](https://rcn-ee.com/rootfs/bb.org/testing/2017-03-07/iot/) BeagleBone Blues come from the factory with the roboticscape package pre-installed as well.

On first boot after flashing a clean image the package will be installed but not configured yet. Manually configure the package with:

```
sudo dpkg-reconfigure roboticscape
```

This will let you configure a program to run on boot and enbale the two systemd services

#### From Repository
BeagleBoard.org graciously hosts the latest stable RoboticsCape package in their repositories. On a BeagleBoard product, install like any other package with:

```
sudo apt update && sudo apt install roboticscape
```

The package can easily be updated from there with:

```
sudo apt update && sudo apt upgrade roboticscape
```

#### From Source

To help beta-test and to stay with the latest library version you can either clone this repo and run 'make && sudo make install', or download and install the latest debian package from the [releases](https://github.com/StrawsonDesign/Robotics_Cape_Installer/releases) page. A debian package is the safer option.

The debian package installation also enables the systemd services which you will have to do manually if installing from source:

```
sudo systemctl enable roboticscape
sudo systemctl enable rc_battery_monitor
```

If installing from source, BeagleBone Black and Black Wireless users will need to enable the Robotics Cape device tree to enable all features of the roboticscape package. This is not necessary for the BeagleBone Blue or when installing a debian package.

```
sudo configure_robotics_dt.sh
sudo reboot
```

To revert to the standard device tree, replace uEnv.txt with the backup that was created by the previous step.

```
sudo cp /boot/uEnv.txt.backup /boot/uEnv.txt
sudo reboot
```

## Version
V0.3.4 is the current stable release. V0.4 is in beta testing and can be installed from source as described above or from the github releases page.

You can check which version of the package is currently installed with the rc_version program.

```
debian@beaglebone:~$ rc_version
0.3.4
```

## Setting Up Networking
Follow the instructions on [BeagleBoard.org](http://beagleboard.org/getting-started) at <http://strawsondesign.com/#!manual-usb> to ssh into your BeagleBone over USB.

Set up wifi networking with the instructions at <http://strawsondesign.com/#!manual-wifi>.

## Services
The RoboticsCape package sets up two systemd services which the user can enable/disable at will.
BeagleBoard images have these services installed but disabled by default. Enable manually like any systemd service or with `sudo dpkg-reconfigure roboticscape` as described above.

#### rc_battery_monitor service
This service runs in the background and illuminates the 4 battery monitor LEDs on the Robotics Cape and BeagleBone Blue based on the state of a 2-cell lithium battery connected to the white balance connector, or a 3 or 4 cell pack connected to the DC input jack.

If you wish to control those LEDs manually (probably through the <rc/led.h> interface) then please disable this service first to avoid conflicts.


#### roboticscape service
This service does some preliminary startup checks on boot to ensure the hardware is configured correctly. A short log file of this process is written to /var/log/roboticscape/startup_log.txt on boot.

After completing the startup checks, this service will then start any program the user as elected to run automatically on boot. If the user's startup program has a problem, closes, or doesn't exist then the roboticscape service will show up as failed when calling `systemctl status roboticscape`. This allows the user to treat their own robot control program like a systemd service, starting and stopping it with `systemctl`. See the next section for how to configure this startup program.

## Making a Robotics Project Run on Boot

Simply make a symbolic link to your program with the name /etc/roboticscape/link_to_startup_program. For example, to set the eduMiP rc_balance program example to run on boot, run:

```
sudo ln -s -f /usr/bin/rc_balance /etc/roboticscape/link_to_startup_program
```

If using the makefile from the rc_project_template (see below) you can also do the same linking with

```
sudo make runonboot
```

After making the symbolic link, you can start, stop, restart, and check the status of your robot program with the roboticscape systemd service just like any other service!

```
debian@beaglebone:~$ sudo systemctl status roboticscape
● roboticscape.service - roboticscape
   Loaded: loaded (/lib/systemd/system/roboticscape.service; enabled; vendor preset: enabled)
   Active: active (running) since Fri 2018-04-20 07:56:28 UTC; 35min ago
  Process: 453 ExecStartPre=/usr/bin/rc_startup_routine (code=exited, status=0/SUCCESS)
 Main PID: 578 (link_to_startup)
    Tasks: 7 (limit: 4915)
   CGroup: /system.slice/roboticscape.service
           └─578 /etc/roboticscape/link_to_startup_program

debian@beaglebone:~$ sudo systemctl stop roboticscape
debian@beaglebone:~$ sudo systemctl start roboticscape
```

Note that while your program is running in the background it is using system resources. If you try doing something simple like rc_test_leds while a project is running in the background then you WILL get IO errors or Resource Busy errors. Make sure to stop the background service as described above.

Both the rc_blink and rc_balance example programs which are offered as startup options during the package installation process make a PID file at /run/roboticscape/roboticscape.pid. You can have your program make a PID file too with rc_make_pid_file() and automatically stop the background process when it starts with rc_kill_existing_process(timeout). If your program takes advantage of these features, you can also stop it from the command line with sudo rc_kill.

## Project Template

We highly suggest using or at least studying the rc_project_template available at <https://github.com/StrawsonDesign/Robotics_Cape_Installer/tree/master/rc_project_template>.

This contains a generic makefile which links to the roboticscape library and provides the following common make commands:

```
make
make clean
make install
make uninstall
make runonboot
```

`make runonboot` sets the program to run automatically on boot if the roboticscape systemd service is enabled, see the services section above for more details.

The project template source is also included as part of the roboticscape package and in installed to /usr/share/roboticscape/rc_project_template so it can be easily copied and modified. We suggest copying the project and modifying as follows

1. `cp -r /usr/share/roboticscape/rc_project_template /home/debian/new_project_name`
2. `cd new_project_name`
3. `mv rc_project_template.c new_project_name.c`
4. Edit the makefile, changing the TARGET variable to your new project name
5. Try compiling with `make` and run the new executable.
6. Work from there!








