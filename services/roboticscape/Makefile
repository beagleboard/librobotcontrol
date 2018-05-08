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
CFLAGS		:= -g -pthread -I $(INCLUDEDIR)
LDFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -lroboticscape

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLNONEXEC	:= install -m 644
INSTALLDIR	:= install -d -m 755

# prefix variable for making debian package
prefix		?= /usr


all: $(TARGET)

install:
	@$(MAKE)
	@$(INSTALLDIR) $(DESTDIR)/etc/roboticscape
	@$(INSTALLDIR) $(DESTDIR)/var/log/roboticscape
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin/
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@$(INSTALLDIR) $(DESTDIR)/etc/modules-load.d/
	@$(INSTALLNONEXEC) roboticscape_modules.conf $(DESTDIR)/etc/modules-load.d/
	@echo ""
	@echo "roboticscape Service Install Complete"
	@echo "run the following to manually enable:"
	@echo "sudo systemctl daemon-reload"
	@echo "sudo systemctl enable roboticscape"
	@echo ""

clean:
	@$(RM) $(BINDIR)
	@echo "roboticscape Service Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	@$(RM) $(DESTDIR)/etc/modules-load.d/roboticscape_modules.conf
	@$(RM) $(DESTDIR)$(prefix)/$(TARGET)
	@echo "roboticscape Service Uninstall Complete"


$(BINDIR)/% : $(SRCDIR)/%.c
	@mkdir -p $(BINDIR)
	@$(CC) -g -o $@ $< $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $(LDFLAGS)
	@echo "made: $@"
