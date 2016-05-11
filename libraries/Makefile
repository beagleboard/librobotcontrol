TARGET = robotics_cape.so

SUBDIRS = $(shell find * -type d -print)
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fPIC
LFLAGS	:= -lm -lrt -lpthread -shared -lprussdrvd

# add -DI2C_DEBUG for debugging
#DEFS = -DEMPL_TARGET_LINUX -DMPU9150 -DAK8975_SECONDARY

SOURCES := $(shell find ./ -name '*.c')
INCLUDES := $(shell find ./ -name '*.h')
OBJECTS  := $(SOURCES:$%.c=$%.o)
RM := rm -f
CP := cp
INSTALL_DIR = /usr/lib/
INCLUDE_DIR = /usr/include/

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
	@for f in $(includes) ; do \
		$(CP) $$f $(INCLUDE_DIR) ; \
	done
	@ $(CP) *.h $(INCLUDE_DIR)
	
	$(MAKE)
	@$(CP) $(TARGET) $(INSTALL_DIR)
	@$(CP) $(OBJECTS) $(INSTALL_DIR)
	@$(CP) pru/prussdrv.h $(INCLUDE_DIR)
	@$(CP) pru/pruss_intc_mapping.h $(INCLUDE_DIR)
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
