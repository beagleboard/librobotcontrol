Robot Control Library                   {#mainpage}
===============================

This package contains the C library and example/testing programs for the Robot Control project. This project began as a hardware interface for the Robotics Cape and later the BeagleBone Blue and was originally called Robotics_Cape_Installer. It grew to include an extensive math library for discrete time feedback control, as well as a plethora of POSIX-compliant functions for timing, threads, program flow, and lots more, all aimed at developing robot control software on embedded computers.

This package ships with official BeagleBone images and is used in the Ardupilot and PX4 ports to the BeagleBone Blue hardware. A python interface to this package is also available at <https://github.com/mcdeoliveira/rcpy>.

The master branch is always the most current but not necessarily stable. See [releases](https://github.com/StrawsonDesign/Robotics_Cape_Installer/releases) page for older stable versions or install from BeagleBoard.org repositories as described below.

Full API documentation and examples at <http://strawsondesign.com/docs/librobotcontrol/>.


## Installation
#### From Clean BeagleBoard.org Image
Stable Robot Control library releases are pre-installed with the BeagleBoard stable images. We recommend BeagleBone Black and Black Wireless users run the latest stable Stretch IOT image available [here](http://beagleboard.org/latest-images). Currently the latest stable Debian Stretch IOT image is 2018-06-17 which includes the previously-named robticscape package, although the API is the same.

On first boot after flashing a clean 2018-06-17 image the package will be installed but not configured yet. Manually configure the package and reboot with:

```
sudo dpkg-reconfigure roboticscape
sudo reboot
```

This will let you configure a program to run on boot and enable the two systemd services. In future testing images that include the librobotcontrol debian package, these services will be enbaled by default.

Finally check that the right device tree is loaded with the rc_model program which will print something like this:

```
debian@beaglebone:~$ rc_model

Currently running on a:
BB_BLACK_RC

```

On the BeagleBone Blue, expect it to say BB_BLUE. If you are using the RoboticsCape on a BealgeBone Black or Black Wireless, make sure it says BB_BLACK_RC or BB_BLACK_W_RC. If it doesn't show the RC suffix then you either did not reboot after reconfiguring the package, or something weird happened and you should manually set the device tree with the following. Note, this is for BeagleBone Black and Black Wireless board using the Robotics Cape, not the BeagleBone Blue.

```
debian@beaglebone:~$ sudo configure_robotics_dt
debian@beaglebone:~$ sudo reboot
```

#### Upgrade From Repository
BeagleBoard.org graciously hosts the latest stable librobotcontrol package in their repositories. On a BeagleBoard product, install like any other package with:

```
sudo apt update && sudo apt install librobotcontrol
```

The package can easily be updated from there with:

```
sudo apt update && sudo apt upgrade librobotcontrol
```

#### From Source

To help beta-test and to stay with the latest library version you can either clone this repo and run 'make && sudo make install', or download and install the latest debian package from the [releases](https://github.com/StrawsonDesign/Robotics_Cape_Installer/releases) page. A debian package is the safer option.

An installer script exists to help compile and install from source.

```
debian@beaglebone:~/librobotcontrol$ ./install.sh

This install script is just to help install from source.
If you are running this on a BeagleBone we recommend running
the version which comes packaged and preinstalled in the
official BeagleBone images. Please visit this link for details:

http://strawsondesign.com/docs/robotcontrol/

To continue, enter 1 to compile and install on BeagleBone
enter 2 for other platforms (Raspberry Pi, x86, etc..)
```

Option 2 will only install the library and examples, whereas option 1 will also do the following beaglebone-specific steps: compile and install PRU binaries, enable systemd services, setup uEnv.txt to enable the Robotics Cape device tree if running on a beaglebone black or black wireless.

You can manually setup the device tree outside of this script with the following command, although this is not necessary when using the install.sh helper script. This is not necessary for the BeagleBone Blue or when installing a debian package.

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
You can check which version of the package is currently installed with the rc_version program. The 2018-06-17 stable image ships with 0.4.4

```
debian@beaglebone:~$ rc_version
0.4.4
```

## Setting Up Networking
Follow the instructions on [BeagleBoard.org](http://beagleboard.org/getting-started) at <http://strawsondesign.com/#!manual-usb> to ssh into your BeagleBone over USB.

Set up wifi networking with the instructions at <http://strawsondesign.com/#!manual-wifi>.

## Services
The librobotcontrol package sets up two systemd services which the user can enable/disable at will.
The BeagleBoard 2018-06-17 image has these services installed but disabled by default. Enable manually like any systemd service or with `sudo dpkg-reconfigure librobotcontrol` as described above.

#### rc_battery_monitor service
This service runs in the background and illuminates the 4 battery monitor LEDs on the Robotics Cape and BeagleBone Blue based on the state of a 2-cell lithium battery connected to the white balance connector, or a 3 or 4 cell pack connected to the DC input jack.

If you wish to control those LEDs manually (probably through the <rc/led.h> interface) then please disable this service first to avoid conflicts.


#### librobotcontrol service
This service does some preliminary startup checks on boot to ensure the hardware is configured correctly. A short log file of this process is written to /var/log/librobotcontrol/startup_log.txt on boot.

After completing the startup checks, this service will then start any program the user as elected to run automatically on boot. If the user's startup program has a problem, closes, or doesn't exist then the librobotcontrol service will show up as failed when calling `systemctl status librobotcontrol`. This allows the user to treat their own robot control program like a systemd service, starting and stopping it with `systemctl`. See the next section for how to configure this startup program.

## Making a Robotics Project Run on Boot

Simply make a symbolic link to your program with the name /etc/librobotcontrol/link_to_startup_program. For example, to set the eduMiP rc_balance program example to run on boot, run:

```
sudo ln -s -f /usr/bin/rc_balance /etc/librobotcontrol/link_to_startup_program
```

If using the makefile from the rc_project_template (see below) you can also do the same linking with

```
sudo make runonboot
```

After making the symbolic link, you can start, stop, restart, and check the status of your robot program with the librobotcontrol systemd service just like any other service!

```
debian@beaglebone:~$ sudo systemctl status robotcontrol
● librobotcontrol.service - robotcontrol
   Loaded: loaded (/lib/systemd/system/robotcontrol.service; enabled; vendor preset: enabled)
   Active: active (running) since Fri 2018-04-20 07:56:28 UTC; 35min ago
  Process: 453 ExecStartPre=/usr/bin/rc_startup_routine (code=exited, status=0/SUCCESS)
 Main PID: 578 (link_to_startup)
    Tasks: 7 (limit: 4915)
   CGroup: /system.slice/robotcontrol.service
           └─578 /etc/robotcontrol/link_to_startup_program

debian@beaglebone:~$ sudo systemctl stop robotcontrol
debian@beaglebone:~$ sudo systemctl start robotcontrol
```

Note that while your program is running in the background it is using system resources. If you try doing something simple like rc_test_leds while a project is running in the background then you WILL get IO errors or Resource Busy errors. Make sure to stop the background service as described above.

Both the rc_blink and rc_balance example programs which are offered as startup options during the package installation process make a PID file at /run/librobotcontrol/librobotcontrol.pid. You can have your program make a PID file too with rc_make_pid_file() and automatically stop the background process when it starts with rc_kill_existing_process(timeout). If your program takes advantage of these features, you can also stop it from the command line with sudo rc_kill.

## Project Template

We highly suggest using or at least studying the rc_project_template available at <https://github.com/StrawsonDesign/librobotcontrol/tree/master/rc_project_template>.

This contains a generic makefile which links to the librobotcontrol library and provides the following common make commands:

```
make
make clean
make install
make uninstall
make runonboot
```

`make runonboot` sets the program to run automatically on boot if the librobotcontrol systemd service is enabled, see the services section above for more details.

The project template source is also included as part of the librobotcontrol package and in installed to /usr/share/robotcontrol/rc_project_template so it can be easily copied and modified. We suggest copying the project and modifying as follows

1. `cp -r /usr/share/robotcontrol/rc_project_template ~/new_project_name`
2. `cd new_project_name`
3. `mv rc_project_template.c new_project_name.c`
4. Edit the makefile, changing the TARGET variable to your new project name
5. Try compiling with `make` and run the new executable.
6. Work from there!








