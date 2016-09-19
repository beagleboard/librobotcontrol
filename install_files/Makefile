
OVERLAY = "RoboticsCape-00A0"
UENV = "/boot/uEnv.txt"
UNAME := "$(shell uname -r)"
UUID := "$(shell blkid -c /dev/null -s UUID -o value /dev/mmcblk*)"

all:
	@echo "Compiling Overlay"
	@dtc -O dtb -o $(OVERLAY).dtbo -b 0 -@ $(OVERLAY).dts

install: all
	@echo "Installing Device Tree Overlay"
	@install -d -m 755 $(DESTDIR)/lib/firmware
	@install -m 644 $(OVERLAY).dtbo $(DESTDIR)/lib/firmware/
	@echo "Installing PRU binaries"
	@install -m 644 pru/am335x-pru0-fw $(DESTDIR)/lib/firmware/
	@install -m 644 pru/am335x-pru1-fw $(DESTDIR)/lib/firmware/
	@echo "Installing modprobe blacklist"
	@install -d -m 755 $(DESTDIR)/etc/modprobe.d
	@install -m 644 pruss-blacklist.conf $(DESTDIR)/etc/modprobe.d/
	@echo "Installing generic makefile"
	@install -m 644 robotics.mk $(DESTDIR)/usr/include/
	@echo "Updating uEnv.txt"
	@install -m 644 --backup=numbered ./uEnv.txt /boot/ 
	@sed -i s/^uuid=.*\$$/uuid=$(UUID)/ $(UENV)
	@sed -i s/^uname_r=.*\$$/uname_r=$(UNAME)/ $(UENV)
	@echo  "Setting Default Cape"
	@echo "CAPE=$OVERYLAY" > /etc/default/capemgr

clean:
	@rm -f $(OVERLAY).dtbo
	@echo "Overlay Cleanup Complete"