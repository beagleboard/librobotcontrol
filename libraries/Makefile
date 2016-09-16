TARGET = librobotics_cape.so

SUBDIRS = $(shell find * -type d -print)
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fpic -march=armv7-a -mtune=cortex-a8
LFLAGS	:= -lm -lrt -lpthread -shared

#DEFS = 

prefix := /usr

SOURCES := $(shell find ./ -name '*.c')
INCLUDES := $(shell find ./ -name '*.h')
OBJECTS  := $(SOURCES:$%.c=$%.o)
RM := rm -f
CP := cp
INSTALL_DIR = $(DESTDIR)$(prefix)/lib/
INCLUDE_DIR = $(DESTDIR)$(prefix)/include/

all:$(TARGET)
	
	
# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	@echo
	@echo "Linking Complete"
	
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) $(DEFS) -c $< -o $(@)
	@echo "Compiled "$<" successfully!"




# install to /usr/bin
$(phony all) : $(TARGET)
.PHONY: install


install:
	$(MAKE)
	install -d -m 755 $(DESTDIR)$(prefix)/lib
	install -m 644 librobotics_cape.so $(DESTDIR)$(prefix)/lib
	install -d -m 755 $(DESTDIR)$(prefix)/include
	install -m 644 *.h $(DESTDIR)$(prefix)/include
	install -m 644 ../install_files/robotics.mk $(DESTDIR)$(prefix)/include
	@ldconfig
	@echo "$(TARGET) installed to $(INSTALL_DIR)"
	@echo
	

clean:
	@for dir in $(SUBDIRS) ; do \
		$(RM) $$dir/*.o ; \
	done
	
	@$(RM) *.o
	@$(RM) *.so
	@echo "Cleanup complete!"
