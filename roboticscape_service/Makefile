SERVICE := roboticscape

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 



all:
	@$(MAKE) -C robot_startup_routine -s --no-print-directory
	@echo "robot_startup_routine Make Complete"

install: $(all)
	@$(MAKE) --no-print-directory
	@echo "Installing robot_startup_routine Binary"
	@$(MAKE) -C robot_startup_routine -s install
	@echo "Installing roboticscape Service File"
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALL) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@echo "Enabling roboticscape Service"
	-systemctl daemon-reload
	-systemctl enable $(SERVICE)
	@echo "roboticscape Service Install Complete"
	
clean:
	@$(MAKE) -C robot_startup_routine -s clean
	@echo "roboticscape Service Cleanup Complete"

uninstall:
	-systemctl -q stop $(SERVICE)
	-systemctl -f -q disable $(SERVICE)
	$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	$(MAKE) -C robot_startup_routine -s uninstall
	-systemctl -q daemon-reload
	@echo "roboticscape Service Uninstall Complete"


