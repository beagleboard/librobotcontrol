prefix := /usr
RM := rm -r -f 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 
ETC_DIR := /etc/roboticscape

all:
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C roboticscape_service --no-print-directory
	@make -C overlay --no-print-directory

install:
	@$(INSTALLDIR) $(DEST_DIR)/$(ETC_DIR)
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install
	@make -C overlay -s install
	@cp -r -f  project_template/ $(DEST_DIR)/$(ETC_DIR)/
	@echo " "
	@echo "roboticscape Package Installed"
	@echo "If you are not on a Bealgebone Blue, please"
	@echo "run configure_robotics_overlay.sh once to configure the"
	@echo "overlay, then reboot to load device tree. After rebooting"
	@echo "we suggest running calibrate_gyro and calibrate_mag."

clean:
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C project_template -s clean
	@make -C overlay -s clean
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
	@make -C overlay -s uninstall
	@$(RM) $(DEST_DIR)/$(ETC_DIR)
	@echo "Robotics Cape Uninstalled"

