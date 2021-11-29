#
# Copyright (c) 2016 Zubeen Tolani <ZeekHuge - zeekhuge@gmail.com>
#
# much thanks to Zubeen Tolani and Mark Yoder for porting this pru code
# to remoteproc.

# /sys/bus/platform/devices/b034000.pru/remoteproc/remoteproc0
# j7-pru0_0-fw


BIN_DIR		:= fw
SRC_DIR		:= src
BUILD_DIR	:= build

TARGETS		:= j7-pru0_0-rc-pwm \
		   am335x-pru0-rc-encoder \
		   am335x-pru1-rc-servo \
		   am57xx-pru1_1-rc-encoder \
		   am57xx-pru2_0-rc-servo

FW		:= $(addprefix $(BIN_DIR)/, $(addsuffix -fw, $(TARGETS)))
OBJS		:= $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(TARGETS)))

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



all: $(FW)
	@echo "Generated: $^"

$(FW): $(BIN_DIR)/%-fw: $(BUILD_DIR)/%.o $(BUILD_DIR)/main_pru.o
	@echo 'LD	$^'
	@mkdir -p $(BIN_DIR)
	@$(LNKPRU) -i$(PRU_CGT)/lib -i$(PRU_CGT)/include $(LFLAGS) -o $@ $^  $(LINKER_COMMAND_FILE) --library=libc.a $(LIBS) $^

$(OBJS): $(BUILD_DIR)/%.o: $(SRC_DIR)/%.asm
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<

$(BUILD_DIR)/main_pru.o: $(SRC_DIR)/main_pru.c
	@mkdir -p $(BUILD_DIR)
	@echo 'CC	$<'
	@$(CLPRU)  $(INCLUDE) $(CFLAGS) -fe $@ $<

install:
	@$(INSTALLDIR) $(DESTDIR)/lib/firmware
	@$(INSTALLNONEXEC) $(TARGETS) $(DESTDIR)/lib/firmware/
	@echo 'PRU Firmware Install Complete'

clean:
	# TODO
	@$(RM) $(BUILD_DIR)
	@$(RM) main_pru1.asm main_pru0.asm file.map
	@echo 'PRU Firmware Cleanup Complete'

uninstall:
	# TODO
	@$(RM) $(DESTDIR)/lib/firmware/{$(TARGETS)}
	@echo "PRU Firmware Uninstall Complete"
