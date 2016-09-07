prefix := /usr

all:
	make -C libraries
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C examples
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C battery_monitor_service/src

install:
	install -d -m 755 $(DESTDIR)/lib/firmware
	install -m 644 install_files/RoboticsCape-00A0.dtbo $(DESTDIR)/lib/firmware/
	install -m 644 install_files/pru/*.bin $(DESTDIR)/lib/firmware/
	install -d -m 755 $(DESTDIR)/etc/systemd/system
	install -m 644 install_files/robot.service $(DESTDIR)/etc/systemd/system
	install -m 644 battery_monitor_service/battery_monitor.service $(DESTDIR)/etc/systemd/system
	install -d -m 755 $(DESTDIR)$(prefix)/lib
	install -m 644 libraries/librobotics_cape.so $(DESTDIR)$(prefix)/lib
	install -d -m 755 $(DESTDIR)$(prefix)/include
	install -m 644 libraries/*.h $(DESTDIR)$(prefix)/include
