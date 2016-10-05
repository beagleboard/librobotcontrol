SERVICE := battery_monitor

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 



all:
	@$(MAKE) -C battery_monitor -s --no-print-directory
	@echo "battery_monitor Make Complete"

install: $(all)
	@$(MAKE) --no-print-directory
	@echo "Installing battery_monitor Binary"
	@$(MAKE) -C battery_monitor -s install
	@echo "Installing battery_monitor Service File"
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALL) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@echo "Enabling battery_monitor Service"
	-systemctl daemon-reload
	-systemctl enable $(SERVICE)
	-systemctl start $(SERVICE)
	@echo "battery_monitor Service Install Complete"
	
clean:
	@$(MAKE) -C battery_monitor -s clean

uninstall:
	-systemctl -q stop --now $(SERVICE)
	-systemctl -f -q --now disable $(SERVICE)
	$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	$(MAKE) -C battery_monitor -s uninstall
	-systemctl -q daemon-reload
	@echo "Battery Monitor Service Uninstall Complete"


