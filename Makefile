prefix		:= /usr
RM		:= rm -r -f
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
INSTALLDIRWRITE	:= install -d -m 777
INSTALLNONEXEC	:= install -m 644

CONFIG_SH	:= configure_robotics_dt.sh

all:
	@make -C pru_firmware --no-print-directory
	@make -C library --no-print-directory
	@make -C examples --no-print-directory
	@make -C services/rc_battery_monitor --no-print-directory
	@make -C services/roboticscape --no-print-directory

install:
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/share/roboticscape
	@cp -r -f  rc_project_template $(DESTDIR)$(prefix)/share/roboticscape/
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) device_tree/$(CONFIG_SH) $(DESTDIR)$(prefix)/bin
	@make -C pru_firmware -s install
	@make -C library -s install
	@make -C examples -s install
	@make -C services/rc_battery_monitor -s install
	@make -C services/roboticscape -s install


clean:
	@make -C pru_firmware -s clean
	@make -C library -s clean
	@make -C examples -s clean
	@make -C services/rc_battery_monitor -s clean
	@make -C services/roboticscape -s clean
	@make -C rc_project_template -s clean
	@$(RM) debian/roboticscape
	@$(RM) debian/roboticscape.postrm.debhelper
	@$(RM) debian/roboticscape.substvars
	@$(RM) debian/files
	@$(RM) debian/*.debhelper.log
	@$(RM) debian/debhelper-build-stamp
	@$(RM) docs/html
	@echo "All Directories Cleaned"


uninstall:
	@make -C pru_firmware -s uninstall
	@make -C library -s uninstall
	@make -C examples -s uninstall
	@make -C services/rc_battery_monitor -s uninstall
	@make -C services/roboticscape -s uninstall
	@$(RM) $(DESTDIR)$(prefix)/bin/$(CONFIG_SH)
	@$(RM) $(DESTDIR)$(prefix)/share/roboticscape
	@$(RM) $(DESTDIR)/var/lib/roboticscape
	@$(RM) $(DESTDIR)/var/log/roboticscape
	@echo "Robotics Cape Uninstalled"

package:
	debuild -us -uc

