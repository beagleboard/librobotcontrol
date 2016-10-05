TARGET = libroboticscape.so

SUBDIRS = $(shell find * -type d -print)
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fpic -march=armv7-a -mtune=cortex-a8
LFLAGS	:= -lm -lrt -lpthread -shared

SOURCES := $(shell find ./ -name '*.c')
INCLUDES := $(shell find ./ -name '*.h')
OBJECTS  := $(SOURCES:$%.c=$%.o)

PREFIX := /usr
RM := rm -f -r
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644
INSTALLPRU := install -m 644


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) $(DEFS) -c $< -o $(@)
	@echo "Compiled: "$<


all:
	$(TARGET)
	@echo "$(TARGET) Make Complete"

install:
	$(MAKE)
	@# includes
	$(INSTALLDIR) $(DESTDIR)$(PREFIX)/include
	$(INSTALL) roboticscape.h $(DESTDIR)$(PREFIX)/include/
	$(INSTALL) roboticscape-defs.h $(DESTDIR)$(PREFIX)/include/
	$(INSTALL) usefulincludes.h $(DESTDIR)$(PREFIX)/include/
	@# library .so
	$(INSTALLDIR) $(DESTDIR)$(PREFIX)/lib
	$(INSTALL) $(TARGET) $(DESTDIR)$(PREFIX)/lib
	@# calibration directory
	$(INSTALLDIR) $(DESTDIR)/etc/roboticscape/
	@# install pru binaries
	$(INSTALLDIR) $(DESTDIR)/lib/firmware/
	$(INSTALLPRU) pru/am335x-pru0-fw $(DESTDIR)/lib/firmware/
	$(INSTALLPRU) pru/am335x-pru1-fw $(DESTDIR)/lib/firmware/
	@#uipruss balcklist
	@$(INSTALLDIR) $(DESTDIR)/etc/modprobe.d
	@$(INSTALL) pruss-blacklist.conf $(DESTDIR)/etc/modprobe.d/
	@# ldconfig to update library index
	ldconfig
	@echo "$(TARGET) Install Complete"
	

clean:
	@for dir in $(SUBDIRS) ; do \
		$(RM) $$dir/*.o ; \
	done
	@$(RM) *.o
	@$(RM) *.so
	@echo "$(TARGET) Clean Complete"


uninstall:
	$(RM) $(DESTDIR)$(PREFIX)/lib/$(TARGET)
	$(RM) $(DESTDIR)$(PREFIX)/include/roboticscape.h
	$(RM) $(DESTDIR)$(PREFIX)/include/roboticscape-defs.h
	$(RM) $(DESTDIR)$(PREFIX)/include/usefulincludes.h
	$(RM) $(DESTDIR)/etc/roboticscape/
	$(RM) $(DESTDIR)/etc/modprobe.d/pruss-blacklist.conf
	@echo "$(TARGET) Uninstall Complete"

	



