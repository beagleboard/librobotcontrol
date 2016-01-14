#!/bin/bash

OVERLAY="RoboticsCape-00A0"
CAPENAME="RoboticsCape"

echo "Installing Device Tree Overlay"

dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ $OVERLAY.dts


