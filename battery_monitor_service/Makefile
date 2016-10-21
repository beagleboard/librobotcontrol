SERVICE := battery_monitor

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 



all:
	@$(MAKE) -C battery_monitor -s --no-print-directory
	@echo "battery_monitor Make Complete"

install: $(all)
	@$(MAKE) -C battery_monitor -s install
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@echo "battery_monitor Service Install Complete"
	
clean:
	@$(MAKE) -C battery_monitor -s clean

uninstall:
	$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	$(MAKE) -C battery_monitor -s uninstall
	@echo "Battery Monitor Service Uninstall Complete"


