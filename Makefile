prefix := /usr
RM := rm -r -f 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 
ETC_DIR := /etc/roboticscape

CONFIG_SH := configure_robotics_overlay.sh

all:
	@make -C pru_firmware --no-print-directory
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C roboticscape_service --no-print-directory

install:
	@$(INSTALLDIR) $(DEST_DIR)/$(ETC_DIR)
	@$(INSTALLDIR) $(DEST_DIR)/usr/bin
	@make -C pru_firmware -s install
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install
	@$(install) device_tree/$(CONFIG_SH) $(DESTDIR)/usr/bin
	@cp -r -f  project_template/ $(DEST_DIR)/$(ETC_DIR)/

clean:
	@make -C pru_firmware -s clean
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C project_template -s clean
	@$(RM) debian/roboticscape
	@$(RM) debian/roboticscape.debhelper.log
	@$(RM) debian/debhelper-build-stamp
	@$(RM) debian/files
	@$(RM) debian/roboticscape.debhelper.log
	@$(RM) debian/roboticscape.postrm.debhelper
	@$(RM) debian/roboticscape.substvars
	@echo "All Directories Cleaned"


uninstall:
	@make -C pru_firmware -s uninstall
	@make -C libraries -s uninstall
	@make -C examples -s uninstall
	@make -C battery_monitor_service -s uninstall
	@make -C roboticscape_service -s uninstall
	@make -C project_template -s uninstall
	@$(RM) $(ETC_DIR)
	@$(RM) /usr/bin/$(CONFIG_SH)
	@echo "Robotics Cape Uninstalled"

