prefix := /usr
RM := rm -r -f 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 755 
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
	@$(INSTALLDIR) $(DESTDIR)/var/log/roboticscape
	@$(INSTALLDIR) $(DESTDIR)/etc/roboticscape
	@$(INSTALL) device_tree/$(CONFIG_SH) $(DESTDIR)$(prefix)/bin
	@cp -r -f  rc_project_template $(DESTDIR)$(prefix)/share/roboticscape/
	@make -C pru_firmware -s install
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install
	

clean:
	@make -C pru_firmware -s clean
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C rc_project_template -s clean
	@$(RM) debian/roboticscape
	@$(RM) debian/roboticscape.postrm.debhelper
	@$(RM) debian/roboticscape.substvars
	@$(RM) debian/files
	@$(RM) debian/*.debhelper.log
	@$(RM) debian/debhelper-build-stamp
	@echo "All Directories Cleaned"


uninstall:
	@make -C pru_firmware -s uninstall
	@make -C libraries -s uninstall
	@make -C examples -s uninstall
	@make -C battery_monitor_service -s uninstall
	@make -C roboticscape_service -s uninstall
	@$(RM) $(DESTDIR)$(prefix)/bin/$(CONFIG_SH)
	@$(RM) $(DESTDIR)$(prefix)/share/roboticscape
	@$(RM) $(DESTDIR)/var/lib/roboticscape
	@$(RM) $(DESTDIR)/var/log/roboticscape
	@echo "Robotics Cape Uninstalled"

package:
	debuild -us -uc

