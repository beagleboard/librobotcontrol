

all:
	@make -C startup_routine

install: all
	@echo "Installing startup_routine Binary"
	@make -C startup_routine -s install
	@echo "Installing systemd Robot Service"
	@install -d -m 755 $(DESTDIR)/etc/robotics
	@install -d -m 755 $(DESTDIR)/etc/systemd/system
	@install -m 644 robot.service $(DESTDIR)/etc/systemd/system/
	@echo "Enabling Robot Service"
	@systemctl daemon-reload
	@systemctl enable robot

clean:
	@make -C startup_routine -s clean
