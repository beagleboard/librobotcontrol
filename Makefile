prefix := /usr
RM := rm -r -f 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 

CONFIG_SH := configure_robotics_dt.sh

all:
	@make -C pru_firmware --no-print-directory
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C roboticscape_service --no-print-directory

install:
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/share/roboticscape
	@$(INSTALLDIR) $(DESTDIR)/var/lib/roboticscape
	@make -C pru_firmware -s install
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install
	@$(INSTALL) device_tree/$(CONFIG_SH) $(DESTDIR)$(prefix)/bin
	@cp -r -f  robot_template/ $(DESTDIR)$(prefix)/share/roboticscape/

clean:
	@make -C pru_firmware -s clean
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C robot_template -s clean
	@$(RM) debian/roboticscape
	@echo "All Directories Cleaned"


uninstall:
	@make -C pru_firmware -s uninstall
	@make -C libraries -s uninstall
	@make -C examples -s uninstall
	@make -C battery_monitor_service -s uninstall
	@make -C roboticscape_service -s uninstall
	@$(RM) $(DESTDIR)$(prefix)/share/roboticscape
	@$(RM) $(DESTDIR)$(prefix)/bin/$(CONFIG_SH)
	@$(RM) $(DESTDIR)/var/lib/roboticscape
	@echo "Robotics Cape Uninstalled"

