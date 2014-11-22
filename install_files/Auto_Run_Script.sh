#! /bin/sh
### BEGIN INIT INFO
# Provides:          robotics
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: robotics
# Description:       robotics boot script
### END INIT INFO


case "$1" in
	stop)
		echo "shutting down"
		 #cleanly exit currently running robot program
		kill_robot
    ;;
	start)
		# wait for for the cape to load
		sleep 3
		battery_monitor &
		#anything else you want loaded on boot below
		