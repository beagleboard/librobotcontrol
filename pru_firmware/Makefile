#
# Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
#
# much thanks to Zubeen Tolani and Mark Yoder for porting this pru code
# to remoteproc.


BIN_DIR		:= fw
SRC_DIR		:= src
BUILD_DIR	:= build

PRU0_FW		:= am335x-pru0-rc-encoder-fw
PRU1_FW		:= am335x-pru1-rc-servo-fw

TARGET0		:= $(BIN_DIR)/$(PRU0_FW)
TARGET1		:= $(BIN_DIR)/$(PRU1_FW)
TARGETS		:= $(TARGET0) $(TARGET1)
LINK_PRU1_FW	:= $(BUILD_DIR)/pru1-servo.object
LINK_PRU0_FW	:= $(BUILD_DIR)/pru0-encoder.object

RM		:= rm -f -r
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
INSTALLNONEXEC	:= install -m 644


PRU_CGT		:= /usr/share/ti/cgt-pru
LNKPRU		:= /usr/bin/lnkpru
CLPRU		:= /usr/bin/clpru

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


LINKER_COMMAND_FILE=./$(SRC_DIR)/AM335x_PRU.cmd
STACK_SIZE=0x100
HEAP_SIZE=0x100


CFLAGS=-v3 -O2 --display_error_number --endian=little --hardware_mac=on --obj_directory=$(BUILD_DIR) --pp_directory=$(BUILD_DIR) -ppd -ppa --asm_listing --c_src_interlist # --absolute_listing
LFLAGS=--reread_libs --warn_sections --stack_size=$(STACK_SIZE) --heap_size=$(HEAP_SIZE) -m file.map



all: $(TARGETS)
	@echo "Generated: $^"


$(TARGET0): $(BUILD_DIR)/main_pru0.object $(LINK_PRU0_FW)
	@echo 'LD	$^'
	@mkdir -p $(BIN_DIR)
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(TARGET1): $(BUILD_DIR)/main_pru1.object $(LINK_PRU1_FW)
	@echo 'LD	$^'
	@mkdir -p $(BIN_DIR)
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^


$(BUILD_DIR)/main_pru0.object: $(SRC_DIR)/main_pru0.c
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<


$(BUILD_DIR)/main_pru1.object: $(SRC_DIR)/main_pru1.c
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<


$(BUILD_DIR)/pru1-servo.object: $(SRC_DIR)/pru1-servo.asm
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<

$(BUILD_DIR)/pru0-encoder.object: $(SRC_DIR)/pru0-encoder.asm
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU) $(INCLUDE) $(CFLAGS) -fe $@ $<



install:
	@$(INSTALLDIR) $(DESTDIR)/lib/firmware
	@$(INSTALLNONEXEC) $(TARGET0) $(DESTDIR)/lib/firmware/
	@$(INSTALLNONEXEC) $(TARGET1) $(DESTDIR)/lib/firmware/
	@echo 'PRU Firmware Install Complete'


clean:
	@$(RM) $(BUILD_DIR)
	@$(RM) main_pru1.asm main_pru0.asm file.map
	@echo 'PRU Firmware Cleanup Complete'

uninstall:
	@$(RM) $(DESTDIR)/lib/firmware/$(PRU0_FW)
	@$(RM) $(DESTDIR)$(DESTDIR)/lib/firmware/$(PRU1_FW)
	@echo "PRU Firmware Uninstall Complete"
