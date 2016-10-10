
OVERLAY := "RoboticsCape-00A0"
OVERLAY_SCRIPT := "configure_robotics_overlay.sh"

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
	@$(INSTALLDIR) $(DESTDIR)/lib/firmware
	@$(INSTALLNONEXEC) $(OVERLAY).dtbo $(DESTDIR)/lib/firmware
	@echo "Overlay Install Complete"
	@echo "run configure_robotics_overlay.sh to apply it in uEnv.txt"

clean:
	@rm -f $(OVERLAY).dtbo
	@echo "Overlay Cleanup Complete"

uninstall:
	@$(RM) $(DESTDIR)/lib/firmware/$(OVERLAY).dtbo 
	@$(RM) $(DESTDIR)/usr/bin/$(OVERLAY_SCRIPT)
	@echo "RoboticsCape Overlay Uninstall Complete"
