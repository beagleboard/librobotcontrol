# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGET = project_template


TOUCH 	 := $(shell touch *)
CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -lm -lrt -lpthread -lrobotics-cape

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)
RM := rm -f

PREFIX := /usr


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)


# install to /usr/bin
$(phony all) : $(TARGET)
.PHONY: install

install: $(all)
	@$(MAKE) --no-print-directory
	@install -m 0755 $(TARGET) $(DESTDIR)$(PREFIX)/bin
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)

	
