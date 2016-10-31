TOUCH 	 := $(shell touch *)
CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -L ../../libraries -lm -lrt -lpthread -lroboticscape

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)

prefix := /usr
RM := rm -f
INSTALL := install -m 755 
INSTALLDIR := install -d -m 644


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)



all: 
	$(TARGET)

install: 
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin

clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)

	
