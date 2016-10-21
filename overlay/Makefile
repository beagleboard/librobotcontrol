
OVERLAY := "RoboticsCape-00A0"
OVERLAY_SCRIPT := "configure_robotics_overlay.sh"
ROBOTICS_DIR := "/etc/roboticscape"

PREFIX := /usr
RM := rm -f -r 
INSTALL := install -m 755 
INSTALLNONEXEC := install -m 644 
INSTALLDIR := install -d -m 644 

all:
	@echo "Compiling Overlay"
	@dtc -O dtb -o $(OVERLAY).dtbo -b 0 -@ $(OVERLAY).dts

install:
	$(MAKE)
	@$(INSTALLDIR) $(DESTDIR)/usr/bin
	@$(INSTALL) $(OVERLAY_SCRIPT) $(DESTDIR)/usr/bin
	@$(INSTALLDIR) $(DESTDIR)$(ROBOTICS_DIR)
	@$(INSTALLNONEXEC) $(OVERLAY).dtbo $(DESTDIR)$(ROBOTICS_DIR)
	@echo "Overlay Install Complete"

clean:
	@rm -f $(OVERLAY).dtbo
	@echo "Overlay Cleanup Complete"

uninstall:
	@$(RM) $(DESTDIR)$(ROBOTICS_DIR)/$(OVERLAY).dtbo 
	@$(RM) $(DESTDIR)/usr/bin/$(OVERLAY_SCRIPT)
	@echo "RoboticsCape Overlay Uninstall Complete"
