SERVICE := roboticscape

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 



all:
	@$(MAKE) -C robot_startup_routine -s --no-print-directory
	@echo "robot_startup_routine Make Complete"

install: $(all)
	@$(MAKE) -C robot_startup_routine -s install
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLDIR) $(DESTDIR)/etc/modules-load.d
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@$(INSTALLNONEXEC) roboticscape_modules.conf $(DESTDIR)/etc/modules-load.d/
	@echo "roboticscape Service Install Complete"
	
clean:
	@$(MAKE) -C robot_startup_routine -s clean

uninstall:
	@$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	@$(RM) $(DESTDIR)/etc/modules-load.d/roboticscape_modules.conf
	@$(MAKE) -C robot_startup_routine -s uninstall
	@echo "roboticscape Service Uninstall Complete"


