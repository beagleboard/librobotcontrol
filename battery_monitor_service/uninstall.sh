#! /bin/bash

systemctl stop battery_monitor
systemctl disable battery_monitor
rm /etc/systemd/system/battery_monitor.service
rm /usr/bin/battery_monitor
systemctl daemon-reload


echo "finished"