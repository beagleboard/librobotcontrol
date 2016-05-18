# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGET = uart_loopback




TOUCH 	 := $(shell touch *)
CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -lm -lrt -lpthread -lrobotics_cape

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)
RM := rm -f

INSTALL_DIR = /usr/bin/

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)
	@echo
	@echo "Linking Complete"


# compiling command
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Compiled "$<" successfully!"


# install to /usr/bin
$(phony all) : $(TARGET)
.PHONY: install

install: $(all)
	@$(MAKE)
	@install -m 0755 $(TARGET) $(INSTALL_DIR)
	@echo
	@echo "Project "$(TARGET)" installed to $(INSTALL_DIR)"
	@echo
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "Cleanup complete!"

	