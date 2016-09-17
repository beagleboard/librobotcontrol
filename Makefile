prefix := /usr

all:
	@make -C install_files
	@make -C libraries
	@CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C examples
	@CPATH=$(PWD)/libraries LIBRARY_PATH=$(PWD)/libraries make -C battery_monitor_service

install:
	@make -C install_files -s install
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install


clean:
	@make -C examples -s clean
	@make -C libraries -s clean
	@make -C battery_monitor_service -s clean
	@echo "All Directories Cleaned"

