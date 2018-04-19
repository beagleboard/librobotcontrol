#!/bin/bash

# this is not part of the roboticscape package, just a helper script
# to speed up compiling and installing new device trees while fiddling

# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to run this."
	exit 1
fi

cp am335x-boneblack-roboticscape.dts /opt/source/dtb-4.9-ti/src/arm/
cp am335x-boneblack-wireless-roboticscape.dts /opt/source/dtb-4.9-ti/src/arm/
cp am335x-roboticscape.dtsi /opt/source/dtb-4.9-ti/src/arm/
cd /opt/source/dtb-4.9-ti
make src/arm/am335x-boneblack-roboticscape.dtb
make src/arm/am335x-boneblack-wireless-roboticscape.dtb
make install



