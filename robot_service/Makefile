

all:
	

install: all
	@echo "Installing Systemd Robot Service"
	@install -d -m 755 $(DESTDIR)/etc/robotics
	@install -d -m 755 $(DESTDIR)/etc/systemd/system
	@install -m 644 robot.service $(DESTDIR)/etc/systemd/system/
	@systemctl daemon-reload
	@systemctl enable robot

clean:
