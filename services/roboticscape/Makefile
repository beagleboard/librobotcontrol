# makefile for roboticscape service

SERVICE		:= roboticscape

# directories
SRCDIR		:= src
BINDIR		:= bin
INCLUDEDIR	:= ../../library/include
LIBDIR		:= ../../library/lib

# file definitions for rules
TARGET		:= $(BINDIR)/rc_startup_routine
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')
SOURCES		:= $(shell find $(SRCDIR) -name '*.c')

# compiler and linker binaries
CC		:= gcc
LINKER		:= gcc

# compiler and linker flags
WFLAGS		:= -Wall -Wextra
CFLAGS		:= -pthread -I $(INCLUDEDIR)
LFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -lroboticscape

# commands
RM		:= rm -rf
INSTALL		:= install -m 4755
INSTALLNONEXEC	:= install -m 644
INSTALLDIR	:= install -d -m 755

# prefix variable for making debian package
prefix		?= /usr/


all: $(TARGET)

install:
	@$(MAKE)
	@$(INSTALL) $(TARGET) $(prefix)/bin/
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@$(INSTALLDIR) $(DESTDIR)/etc/modules-load.d/
	@$(INSTALLNONEXEC) roboticscape_modules.conf $(DESTDIR)/etc/modules-load.d/
	@systemctl daemon-reload
	@echo "roboticscape Service Install Complete"

clean:
	@$(RM) $(BINDIR)
	@echo "roboticscape Service Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	@$(RM) $(DESTDIR)/etc/modules-load.d/roboticscape_modules.conf
	@echo "roboticscape Service Uninstall Complete"


$(BINDIR)/% : $(SRCDIR)/%.c
	@mkdir -p $(BINDIR)
	@$(CC) -o $@ $< $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $(LFLAGS)
	@echo "made: $@"
