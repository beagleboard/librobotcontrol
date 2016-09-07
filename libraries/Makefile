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
	@echo "moving headers files to /usr/include"
	@for f in $(INCLUDES) ; do \
		$(CP) --parents $$f $(INCLUDE_DIR) ; \
	done
	
	$(MAKE)
	@$(CP) $(TARGET) $(INSTALL_DIR)
	@$(CP) $(OBJECTS) $(INSTALL_DIR)
	@$(CP) $(INSTALL_DIR)/$(TARGET) $(INSTALL_DIR)/lib$(TARGET)
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
