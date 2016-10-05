
SERVICE := roboticscape

PREFIX := /usr
RM := rm -f
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 


all:
	@$(MAKE) -C robot_startup_routine -s --no-print-directory
	@echo "robot_startup_routine Make Complete"

install: all
	@echo "Installing startup_routine Binary"
	@$(MAKE) -C robot_startup_routine install -s --no-print-directory
	@echo "Installing systemd roboticscape Service"
	@$(INSTALLDIR) $(DESTDIR)/etc/roboticscape
	@$(INSTALLDIR) $(DESTDIR)/etc/systemd/system
	@$(INSTALL) $(SERVICE).service $(DESTDIR)/etc/systemd/system/
	@echo "Enabling roboticscape Service"
	@systemctl daemon-reload
	@systemctl enable $(SERVICE)
	@echo "roboticscape Service Install Complete"

clean:
	@$(MAKE) -C robot_startup_routine -s clean

uninstall:
	-systemctl -f -q --now disable $(SERVICE)
	@$(RM) $(DESTDIR)/etc/systemd/system/$(SERVICE).service
	@$(MAKE) -C robot_startup_routine -s uninstall
	@echo "roboticscape Service Uninstall Complete"
