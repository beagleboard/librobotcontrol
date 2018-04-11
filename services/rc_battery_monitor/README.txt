Battery Monitor for Robotics Cape
James Strawson - 2016


This battery monitor service is started automatically on boot by 
rc_battery_monitor.service in order to illuminate the set of 4 battery indicator 
LEDS on the Robotics Cape to display battery charge level. It is primarily 
intended to monitor the charge status of a 2-cell lithium battery plugged into 
the white 3-pin JST-XH connector and light the indicator LEDs appropriately. 

However, if a 2,3 or 4 cell lithium battery is connected to the DC input jack 
and the white 3-pin JST-XH connector is left disconnected then it will 
illuminate the battery indicator LEDs to reflect the charge status of the 
battery connected to the DC input jack.

This is a service managed by systemd and should be stopped and started like
other linux services as follows:


root@beaglebone:~# systemctl stop rc_battery_monitor
root@beaglebone:~# systemctl start rc_battery_monitor
root@beaglebone:~# systemctl status rc_battery_monitor
● rc_battery_monitor.service - rc_battery_monitor
   Loaded: loaded (/etc/systemd/system/rc_battery_monitor.service; enabled)
   Active: active (running) since Fri 2016-05-13 18:44:19 UTC; 4s ago
  Process: 16797 ExecStop=/usr/bin/rc_battery_monitor -k (code=exited, status=0/SUCCESS)
  Process: 16855 ExecStartPre=/usr/bin/rc_battery_monitor -k (code=exited, status=0/SUCCESS)
 Main PID: 16859 (rc_battery_monitor)
   CGroup: /system.slice/rc_battery_monitor.service
           └─16859 /usr/bin/rc_battery_monitor


