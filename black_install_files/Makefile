
OVERLAY := "RoboticsCape-00A0"
CAPE := "RoboticsCape"
BLACKDIR := /etc/roboticscape/black_install_files

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
	@$(INSTALLDIR) $(DESTDIR)/$(BLACKDIR)
	@$(INSTALLDIR) $(DESTDIR)/usr/bin
	@$(INSTALLNONEXEC) $(OVERLAY).dtbo $(DESTDIR)/$(BLACKDIR)
	@$(INSTALLNONEXEC) uEnv.txt $(DESTDIR)/$(BLACKDIR)
	@$(INSTALL) configure_black_overlay.sh $(DESTDIR)/usr/bin
	@echo "black_install_files Install Complete"

clean:
	@rm -f $(OVERLAY).dtbo
	@echo "Overlay Cleanup Complete"

uninstall:
	@$(RM) $(DESTDIR)/$(BLACKDIR)/$(OVERLAY).dtbo 
	@$(RM) $(DESTDIR)/$(BLACKDIR)/uEnv.txt
	@$(RM) $(DESTDIR)/usr/bin/configure_black_overlay.sh
	@echo "black_install_files Uninstall Complete"
