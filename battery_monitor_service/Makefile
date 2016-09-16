# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGET = battery_monitor

CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -lm -lrt -lpthread -llibrobotics_cape

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
	@echo "Installing battery_monitor binary"
	@install -m 0755 $(TARGET) $(INSTALL_DIR)
	@install -d -m 755 $(DESTDIR)/etc/systemd/system
	@echo "Installing battery_monitor service"
	@install -m 0644 battery_monitor.service $(DESTDIR)/etc/systemd/system/
	@echo "Enabling battery_monitor service"
	@systemctl daemon-reload
	@systemctl enable battery_monitor
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "Cleanup complete!"

