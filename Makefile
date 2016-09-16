prefix := /usr

all:
	make -C libraries
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C examples
	CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C battery_monitor_service

install: all
	make -C install_files -s install
	make -C examples -s install
	make -C battery_monitor_service -s install
	install -d -m 755 $(DESTDIR)/etc/systemd/system
	install -m 644 battery_monitor_service/battery_monitor.service $(DESTDIR)/etc/systemd/system

clean:
	make -C examples -s clean
	make -C libraries -s clean
	make -C battery_monitor_service -s clean

