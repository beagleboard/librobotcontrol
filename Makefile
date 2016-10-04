prefix := /usr

all:
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C robot_service --no-print-directory

install:
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C robot_service -s install

clean:
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C robot_service -s clean
	@echo "All Directories Cleaned"

