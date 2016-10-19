#!/bin/bash


echo "rebooting pru core 0"
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind
echo "pru core 0 is now loaded"

echo "rebooting pru core 1"
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind 2> /dev/null
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind
echo "pru core 1 is now loaded"
