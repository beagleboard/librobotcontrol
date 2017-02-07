# 
# Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
#
# much thanks to Zubeen Tolani and Mark Yoder for porting this pru code 
# to remoteproc. 


BIN_DIR := bin

PRU1_FW		=$(BIN_DIR)/main-pru1-fw.out
PRU0_FW		=$(BIN_DIR)/main-pru0-fw.out

TARGETS		=$(PRU1_FW) $(PRU0_FW)
LINK_PRU1_FW=$(BIN_DIR)/pru1-servo.object
LINK_PRU0_FW=$(BIN_DIR)/pru0-encoder.object

RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 755
INSTALLNONEXEC := install -m 644 


PRU_CGT=/usr/share/ti/cgt-pru
LNKPRU=/usr/bin/lnkpru
CLPRU=/usr/bin/clpru

# if the pru-software-support-package is installed to /opt/ use that
# otherwise look in /usr/lib/ti/ which is where the BBB images puts it
ifneq ("$(wildcard /opt/source/pru-software-support-package/lib/rpmsg_lib.lib)","")
LIBS=--library=/opt/source/pru-software-support-package/lib/rpmsg_lib.lib
INCLUDE=--include_path=/opt/source/pru-software-support-package/include \
--include_path=/opt/source/pru-software-support-package/include/am335x \
--include_path=$(PRU_CGT)/include
else
LIBS=--library=/usr/lib/ti/pru-software-support-package/lib/rpmsg_lib.lib
INCLUDE=--include_path=/usr/lib/ti/pru-software-support-package/include \
--include_path=/usr/lib/ti/pru-software-support-package/include/am335x \
--include_path=$(PRU_CGT)/include
endif


LINKER_COMMAND_FILE=./AM335x_PRU.cmd
STACK_SIZE=0x100
HEAP_SIZE=0x100


CFLAGS=-v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(BIN_DIR) --pp_directory=$(BIN_DIR) -ppd -ppa --asm_listing --c_src_interlist # --absolute_listing
LFLAGS=--reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE) -m file.map



all: $(TARGETS)
	@echo "Generated: $^"


$(PRU0_FW): $(BIN_DIR)/main_pru0.object $(LINK_PRU0_FW)
	@echo 'LD	$^' 
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(PRU1_FW): $(BIN_DIR)/main_pru1.object $(LINK_PRU1_FW)
	@echo 'LD	$^'
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^


$(BIN_DIR)/main_pru0.object: main_pru0.c 
	@mkdir -p $(BIN_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<


$(BIN_DIR)/main_pru1.object: main_pru1.c
	@mkdir -p $(BIN_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<


$(BIN_DIR)/pru1-servo.object: pru1-servo.asm
	@mkdir -p $(BIN_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<

$(BIN_DIR)/pru0-encoder.object: pru0-encoder.asm
	@mkdir -p $(BIN_DIR)
	@echo 'CC	$<'
	@$(CLPRU) $(INCLUDE) $(CFLAGS) -fe $@ $<



install:
	$(MAKE)
	@$(INSTALLDIR) $(DESTDIR)/lib/firmware
	@$(INSTALLDIR) $(DESTDIR)/etc/modprobe.d
	@$(INSTALLNONEXEC) $(PRU0_FW) $(DESTDIR)/lib/firmware/am335x-pru0-fw
	@$(INSTALLNONEXEC) $(PRU1_FW) $(DESTDIR)/lib/firmware/am335x-pru1-fw
	@$(INSTALLNONEXEC) pruss-blacklist.conf $(DESTDIR)/etc/modprobe.d/
	@echo 'PRU Firmware Install Complete'


clean:
	@$(RM) $(BIN_DIR)
	@$(RM) main_pru1.asm main_pru0.asm file.map
	@echo 'PRU Firmware Cleanup Complete'

uninstall:
	@$(RM) /lib/firmware/am335x-pru0-fw
	@$(RM) /lib/firmware/am335x-pru1-fw
	@$(RM) /etc/modprobe.d/pruss-blacklist.conf
	@echo "PRU Firmware Uninstall Complete"
