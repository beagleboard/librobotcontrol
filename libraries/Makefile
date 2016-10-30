TARGET = libroboticscape.so


SUBDIRS = $(shell find * -type d -print)
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fpic -march=armv7-a -mtune=cortex-a8 
LFLAGS	:= -lm -lrt -lpthread -shared -Wl,-soname,$(TARGET)

SOURCES := $(shell find ./ -name '*.c')
INCLUDES := $(shell find ./ -name '*.h')
OBJECTS  := $(SOURCES:$%.c=$%.o)

prefix := /usr
RM := rm -f -r
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644
INSTALLNOEXEC := install -m 644


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
	$(INSTALLDIR) $(DESTDIR)$(prefix)/include
	$(INSTALL) roboticscape.h $(DESTDIR)$(prefix)/include/
	$(INSTALL) roboticscape-defs.h $(DESTDIR)$(prefix)/include/
	$(INSTALL) roboticscape-usefulincludes.h $(DESTDIR)$(prefix)/include/
	@# library .so
	$(INSTALLDIR) $(DESTDIR)$(prefix)/lib
	$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/lib
	@# ldconfig to update library index
	ldconfig
	@echo "$(TARGET) Install Complete"
	

clean:
	@$(RM) */*.o
	@$(RM) *.o
	@$(RM) *.so
	@echo "$(TARGET) Clean Complete"


uninstall:
	$(RM) $(DESTDIR)$(prefix)/lib/$(TARGET)
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape.h
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape-defs.h
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape-usefulincludes.h
	@echo "$(TARGET) Uninstall Complete"

	



