TARGET = rc_startup_routine

CC			:= gcc
LINKER		:= gcc -o
CFLAGS		:= -c -Wall -g
LFLAGS		:= -L ../../libraries -lm -lrt -lpthread -lroboticscape

SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

PREFIX		:= /usr
RM			:= rm -f
INSTALL		:= install -m 755 
INSTALLDIR	:= install -d -m 755


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	$(CC) $(CFLAGS) -c $< -o $(@)

all:
	$(TARGET)

install:
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(PREFIX)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(PREFIX)/bin
	@echo "$(TARGET) Installation Complete"
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Cleanup Complete"

uninstall:
	@$(RM) $(DESTDIR)$(PREFIX)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"
