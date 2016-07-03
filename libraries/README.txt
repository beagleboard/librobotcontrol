Robotics Cape Supporting Libraries
James Strawson - 2016

Contains libraries to assist in the use of your Robotics Cape.
The included Makefile will compile a single shared object robotics_cape.so
and install it to /usr/lib. It will also move all header files to
/usr/include.

To use the .so you only need to #include <robotics_cape.h>
We also recommend using #include <useful_includes.h>

If you wish to recompile the lirbary without going through the entire Robotics
Cape installation procedure, just execute the following make command as root.

root@beaglebone:~/Robotics_Cape_Installer# cd libraries/
root@beaglebone:~/Robotics_Cape_Installer/libraries# make clean install

