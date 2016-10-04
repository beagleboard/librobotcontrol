
all:
	@make -C battery_monitor

install: $(all)
	@$(MAKE)
	@echo "Installing battery_monitor Binary"
	@make -C battery_monitor -s install
	@echo "Installing battery_monitor Service File"
	@install -d -m 755 $(DESTDIR)/etc/systemd/system
	@install -m 0644 battery_monitor.service $(DESTDIR)/etc/systemd/system/
	@echo "Enabling battery_monitor Service"
	@systemctl daemon-reload
	@systemctl enable battery_monitor
	@systemctl start battery_monitor
	
clean:
	@make -C battery_monitor -s clean


