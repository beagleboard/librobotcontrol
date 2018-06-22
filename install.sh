# basic install script to help when installing from source
# please use the debian package unless you know what you are doing!


echo ""
echo "This install script is just to help install from source."
echo "If you are running this on a BeagleBone we recommend running"
echo "the version which comes packaged and preinstalled in the "
echo "official BeagleBone images. Please visit this link for details:"
echo ""
echo "http://strawsondesign.com/docs/librobotcontrol/"
echo ""
echo "To continue, enter 1 to compile and install on BeagleBone"
echo "enter 2 for other platforms (Raspberry Pi, x86, etc..)"
echo ""



read n
case $n in
	1)
		mode=BB
		echo "ready to install on BeagleBone, press enter to continue"
		;;
	2)
		mode=OTHER
		echo "ready to install generic library, press enter to continue"
		;;

	*)
		echo "invalid option"
		exit;;
esac

# wait for keypress
read n


case $mode in
	BB)
		# for BBB, compile and install everything
		sudo debian/preinst
		make
		sudo make install
		sudo ldconfig
		sudo systemctl daemon-reload
		sudo systemctl enable robotcontrol
		sudo systemctl enable rc_battery_monitor
		sudo configure_robotics_dt.sh
		;;
	OTHER)
		# for other platforms just install libary and examples
		make -C library --no-print-directory
		make -C examples --no-print-directory
		sudo make -C library -s install
		sudo make -C examples -s install
		sudo ldconfig
		;;
	*)
		echo "invalid option"
		exit;;
esac

echo "finished!"




