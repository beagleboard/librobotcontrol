#!/bin/bash

# this is not part of the roboticscape package, just a helper script for
# myself to speed up boot times on my own robots.

# make sure the user is root
if [ `whoami` != 'root' ]; then
	echo "You must be root to run this."
	exit 1
fi

echo "Disabling Avahi"
systemctl disable avahi-daemon

echo "Disabling bonescript"
systemctl disable bonescript

echo "Disabling bonescript-autorun"
systemctl disable bonescript-autorun

echo "Disabling cloud9"
systemctl disable cloud9

echo "Disabling apache2"
systemctl disable apache2

echo "DONE"