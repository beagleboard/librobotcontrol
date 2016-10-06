prefix := /usr
RM := rm -r -f 

all:
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C roboticscape_service --no-print-directory
	@make -C black_install_files --no-print-directory

install:
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install
	@make -C black_install_files -s install
	@echo "roboticscape Package Installed"
	@echo "If you are running this on a beaglebone black, please"
	@echo "run configure_black_overlay.sh once to configure the"
	@echo "device tree overlay, then reboot to load device tree."
	@echo "After rebooting we suggest running calibrate_gyro and calibrate_mag."

clean:
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C project_template -s clean
	@make -C black_install_files -s clean
	@$(RM) debian/roboticscape
	@$(RM) debian/roboticscape.debhelper.log
	@$(RM) debian/debhelper-build-stamp
	@$(RM) debian/files
	@$(RM) debian/roboticscape.debhelper.log
	@$(RM) debian/roboticscape.postrm.debhelper
	@$(RM) debian/roboticscape.substvars
	@echo "All Directories Cleaned"


uninstall:
	@make -C libraries -s uninstall
	@make -C examples -s uninstall
	@make -C battery_monitor_service -s uninstall
	@make -C roboticscape_service -s uninstall
	@make -C project_template -s uninstall
	@make -C black_install_files -s uninstall
	@echo "roboticscape Uninstalled"

