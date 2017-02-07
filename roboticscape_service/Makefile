SERVICE := roboticscape

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 755
INSTALLNONEXEC := install -m 644 



all:
	@$(MAKE) -C rc_startup_routine -s --no-print-directory
	@echo "rc_startup_routine Make Complete"

install: $(all)
	@$(MAKE) -C rc_startup_routine -s install
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@$(INSTALLDIR) $(DESTDIR)/etc/modules-load.d/
	@$(INSTALLNONEXEC) roboticscape_modules.conf $(DESTDIR)/etc/modules-load.d/
	@echo "roboticscape Service Install Complete"
	
clean:
	@$(MAKE) -C rc_startup_routine -s clean

uninstall:
	@$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	@$(RM) $(DESTDIR)/etc/modules-load.d/roboticscape_modules.conf
	@$(MAKE) -C rc_startup_routine -s uninstall
	@echo "roboticscape Service Uninstall Complete"


