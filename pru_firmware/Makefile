# 
# Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
#
# much thanks to Zubeen Tolani and Mark Yoder for porting this pru code 
# to remoteproc. 



PRU_CGT=/usr/share/ti/cgt-pru
LNKPRU=/usr/bin/lnkpru
CLPRU=/usr/bin/clpru


LINKER_COMMAND_FILE=./AM335x_PRU.cmd
LIBS=--library=/opt/source/pru-software-support-package/lib/rpmsg_lib.lib
INCLUDE=--include_path=/opt/source/pru-software-support-package/include --include_path=/opt/source/pru-software-support-package/include/am335x
STACK_SIZE=0x100
HEAP_SIZE=0x100


CFLAGS=-v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(GEN_DIR) --pp_directory=$(GEN_DIR) -ppd -ppa --asm_listing --c_src_interlist # --absolute_listing
LFLAGS=--reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE) -m file.map


GEN_DIR := build
FINAL_DIR := final_bins

PRU1_FW		=$(FINAL_DIR)/main_pru1_fw.out
PRU0_FW		=$(FINAL_DIR)/main_pru0_fw.out

TARGETS		=$(PRU1_FW) $(PRU0_FW)
LINK_PRU1_FW=$(GEN_DIR)/pru1-servo.object
LINK_PRU0_FW=$(GEN_DIR)/pru0-encoder.object

RM := rm -f -r 
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644 
INSTALLNONEXEC := install -m 644 



all: $(TARGETS)
	@echo "Generated: $^"


$(PRU0_FW): $(GEN_DIR)/main_pru0.object $(LINK_PRU0_FW)
	@echo 'LD	$^' 
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(PRU1_FW): $(GEN_DIR)/main_pru1.object $(LINK_PRU1_FW)
	@echo 'LD	$^'
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^


$(GEN_DIR)/main_pru0.object: main_pru0.c 
	@mkdir -p $(GEN_DIR)
	@mkdir -p $(FINAL_DIR)
	@echo 'CC	$<'
	@$(CLPRU) --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<


$(GEN_DIR)/main_pru1.object: main_pru1.c
	@mkdir -p $(GEN_DIR)
	@mkdir -p $(FINAL_DIR)
	@echo 'CC	$<'
	@$(CLPRU) --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<


$(GEN_DIR)/pru1-servo.object: pru1-servo.asm
	@mkdir -p $(GEN_DIR)
	@mkdir -p $(FINAL_DIR)
	@echo 'CC	$<'
	@$(CLPRU) --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<

$(GEN_DIR)/pru0-encoder.object: pru0-encoder.asm
	@mkdir -p $(GEN_DIR)
	@mkdir -p $(FINAL_DIR)
	@echo 'CC	$<'
	@$(CLPRU) --include_path=$(PRU_CGT)/include $(INCLUDE) $(CFLAGS) -fe $@ $<





install:
	@echo "copying firmware to /lib/firmware/am335x_pru0_fw"
	@$(INSTALLNONEXEC) $(PRU0_FW) $(DESTDIR)/lib/firmware/am335x_pru0_fw
	@echo 'copying firmware to /lib/firmware/am335x_pru1_fw'
	@$(INSTALLNONEXEC) $(PRU1_FW) $(DESTDIR)/lib/firmware/am335x_pru1_fw
	@$(INSTALLNONEXEC) pruss-blacklist.conf $(DESTDIR)/etc/modprobe.d/


clean:
	@$(RM) $(GEN_DIR)
	@$(RM) main_pru1.asm main_pru0.asm file.map
	@echo 'PRU Firmware Cleanup Complete'

uninstall:
	@$(RM) $(DESTDIR)/lib/firmware/am335x-pru0-fw
	@$(RM) $(DESTDIR)/lib/firmware/am335x-pru1-fw
	@$(RM) $(DESTDIR)/etc/modprobe.d/pruss-blacklist.conf
	@echo "PRU Firmware Uninstall Complete"
