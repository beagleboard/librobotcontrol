#!/bin/bash

# this is not part of the roboticscape package, just a helper script
# to speed up compiling and installing new device trees while fiddling

# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to run this."
	exit 1
fi

echo "enter 1 to install for 4.9 kernel"
echo "enter 2 to install for 4.14 kernel"
echo "anything else to exit"

read n
case $n in
1)
	cp -r dtb-4.9-ti/* /opt/source/dtb-4.9-ti/src/arm/
	cd /opt/source/dtb-4.9-ti
	;;
2)
	cp -r dtb-4.14-ti/* /opt/source/dtb-4.14-ti/src/arm/
	cd /opt/source/dtb-4.14-ti
	;;

*)
	exit;;
esac


make src/arm/am335x-boneblack-roboticscape.dtb
make src/arm/am335x-boneblack-wireless-roboticscape.dtb
make src/arm/am335x-boneblue.dtb
make src/arm/
make install

echo "DONE"

