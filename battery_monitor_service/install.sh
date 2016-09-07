#! /bin/bash

# installs and starts the battery monitor service for use with the Robotics Cape


cd src
make clean install
make clean
cd ../

cp battery_monitor.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable battery_monitor

echo "finished"