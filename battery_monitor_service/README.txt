Battery Monitor for Robotics Cape
James Strawson - 2016


This battery monitor service is started automatically on boot by 
battery_monitor.service in order to illuminate the set of 4 battery indicator 
LEDS on the Robotics Cape to display battery charge level. It is primarily 
intended to monitor the charge status of a 2-cell lithium battery plugged into 
the white 3-pin JST-XH connector and light the indicator LEDs appropriately. 

However, if a 2,3 or 4 cell lithium battery is connected to the DC input jack 
and the white 3-pin JST-XH connector is left disconnected then it will 
illuminate the battery indicator LEDs to reflect the charge status of the 
battery connected to the DC input jack.

This is a service managed by systemd and should be stopped and started like
other linux services as follows:


root@beaglebone:~# systemctl stop battery_monitor
root@beaglebone:~# systemctl start battery_monitor
root@beaglebone:~# systemctl status battery_monitor
● battery_monitor.service - battery_monitor
   Loaded: loaded (/etc/systemd/system/battery_monitor.service; enabled)
   Active: active (running) since Fri 2016-05-13 18:44:19 UTC; 4s ago
  Process: 16797 ExecStop=/usr/bin/battery_monitor -k (code=exited, status=0/SUCCESS)
  Process: 16855 ExecStartPre=/usr/bin/battery_monitor -k (code=exited, status=0/SUCCESS)
 Main PID: 16859 (battery_monitor)
   CGroup: /system.slice/battery_monitor.service
           └─16859 /usr/bin/battery_monitor


Use the install.sh and uninstall.sh scripts to install and uninstall accordingly.
After calling install.sh the service should start after the next reboot.
The main robotics cape installer will install the battery_monitor service too.
