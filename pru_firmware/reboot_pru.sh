#!/bin/bash


echo "rebooting pru core 0"
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a334000.pru0" > /sys/bus/platform/drivers/pru-rproc/bind

echo "rebooting pru core 1"
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/unbind
echo "4a338000.pru1" > /sys/bus/platform/drivers/pru-rproc/bind
