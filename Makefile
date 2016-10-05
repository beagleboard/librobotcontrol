prefix := /usr
RM := rm -r -f 

all:
	@make -C libraries --no-print-directory
	@make -C examples --no-print-directory
	@make -C battery_monitor_service --no-print-directory
	@make -C roboticscape_service --no-print-directory

install:
	@make -C libraries -s install
	@make -C examples -s install
	@make -C battery_monitor_service -s install
	@make -C roboticscape_service -s install

clean:
	@make -C libraries -s clean
	@make -C examples -s clean
	@make -C battery_monitor_service -s clean
	@make -C roboticscape_service -s clean
	@make -C project_template -s clean
	@$(RM) debian/libroboticscape
	@$(RM) debian/libroboticscape.debhelper.log
	@echo "All Directories Cleaned"


uninstall:
	@make -C libraries -s uninstall
	@make -C examples -s uninstall
	@make -C battery_monitor_service -s uninstall
	@make -C roboticscape_service -s uninstall
	@make -C project_template -s uninstall
	@echo "libroboticscape Uninstalled"

