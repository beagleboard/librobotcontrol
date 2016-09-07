prefix := /usr

all:
	make -C libraries
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C examples
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C battery_monitor_service/src
	make -C install_files

install: all
	make -C install_files -s install
	install -d -m 755 $(DESTDIR)$(prefix)/lib
	install -m 644 libraries/librobotics_cape.so $(DESTDIR)$(prefix)/lib
	install -d -m 755 $(DESTDIR)$(prefix)/include
	install -m 644 libraries/*.h $(DESTDIR)$(prefix)/include
	make -C examples -s install
	make -C battery_monitor_service/src -s install
	install -d -m 755 $(DESTDIR)/etc/systemd/system
	install -m 644 battery_monitor_service/battery_monitor.service $(DESTDIR)/etc/systemd/system

