# shared makefile for all the examples

CC		:= gcc
LINKER		:= gcc -o
CFLAGS		:= -c -Wall -O2
LFLAGS		:= -L ../../libraries -lm -lrt -lpthread -lroboticscape

SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

prefix		:= /usr
RM		:= rm -f
INSTALL		:= install -o root -g root -m 4755 
INSTALLDIR	:= install -d -m 755

LINK		:= ln -s -f
LINKDIR		:= /etc/roboticscape
LINKNAME	:= link_to_startup_program

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) -c $< -o $(@)



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

runonboot:
	@$(MAKE) install --no-print-directory
	@$(LINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(LINKDIR)/$(LINKNAME)
	@echo "$(TARGET) Set to Run on Boot"

	
