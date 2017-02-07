SERVICE := rc_battery_monitor

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 755 
INSTALLNONEXEC := install -m 644 



all:
	@$(MAKE) -C rc_battery_monitor -s --no-print-directory
	@echo "rc_battery_monitor Make Complete"

install: $(all)
	@$(MAKE) -C rc_battery_monitor -s install
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@echo "rc_battery_monitor Service Install Complete"
	
clean:
	@$(MAKE) -C rc_battery_monitor -s clean

uninstall:
	$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	$(MAKE) -C rc_battery_monitor -s uninstall
	@echo "rc_battery_monitor Service Uninstall Complete"


