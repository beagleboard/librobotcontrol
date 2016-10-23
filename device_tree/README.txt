Robotics Cape Device Tree

The overlay and device tree source files here don't actually get compiled
or installed since they are part of the beagleboard image now. However,
They live here for completeness. 

The configure_robotics_dt.sh script does get installed to /usr/bin
and sets up uEnv.txt to load either the overlay or the full device tree based
on which is present as some may be using older images without the device tree.