Battery Monitor for Robotics Cape
James Strawson - 2016


This battery monitor service is started automatically on boot by 
Auto_Run_Script.sh in order to illuminate the set of 4 battery indicator LEDS 
on the Robotics Cape to display battery charge level. It is primarily intended
to monitor the charge status of a 2-cell lithium battery plugged into the white
3-pin JST-XH connector and illuminate the indicator LEDs appropriately. 

However, if a 2,3 or 4 cell lithium battery is connected to the DC input jack 
and the white 3-pin JST-XH connector is left disconnected then it will 
illuminate the battery indicator LEDs to reflect the charge status of the 
battery connected to the DC input jack.


