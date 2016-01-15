#! /bin/sh
### BEGIN INIT INFO
# Provides:          robotics
# Required-Start:    $all
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
		
		# keep checking the test-initialization example program
		# until it loads the cape hardware correctly
		RETURNCODE=-1
		until [ $RETURNCODE -eq "0" ]; do
			test_initialization
			RETURNCODE=$?
            sleep 1
        done
		
		# everything seems to be loaded now, start the battery monitor program in the background
		battery_monitor &
		
		# now put anything else you want loaded on boot HERE
		#INSERT
		
	;;
esac
exit 0
