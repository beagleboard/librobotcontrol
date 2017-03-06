# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGET = rc_battery_monitor

CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -L ../../libraries -lm -lrt -lpthread -lroboticscape

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)

PREFIX := /usr
RM := rm -f
INSTALL := install -m 755 
INSTALLDIR := install -d -m 755 


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	$(CC) $(CFLAGS) -c $< -o $(@)


all: 
	$(TARGET)
	@echo "$(TARGET) Make Complete"

install: 
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(PREFIX)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(PREFIX)/bin
	@echo "$(TARGET) Install Complete"
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(PREFIX)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"


	
