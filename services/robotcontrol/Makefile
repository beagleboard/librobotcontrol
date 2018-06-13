# makefile for robotcontrol service

SERVICE		:= robotcontrol

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
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized -Wunused-variable -Wdouble-promotion
CFLAGS		:= -g -pthread -I $(INCLUDEDIR)
LDFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -l:librobotcontrol.so.1

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
	@$(INSTALLDIR) $(DESTDIR)/etc/robotcontrol
	@$(INSTALLDIR) $(DESTDIR)/var/log/robotcontrol
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin/
	@$(INSTALLDIR) $(DESTDIR)/lib/systemd/system
	@$(INSTALLNONEXEC) $(SERVICE).service $(DESTDIR)/lib/systemd/system/
	@$(INSTALLDIR) $(DESTDIR)/etc/modules-load.d/
	@$(INSTALLNONEXEC) robotcontrol_modules.conf $(DESTDIR)/etc/modules-load.d/
	@echo "robotcontrol Service Install Complete"


clean:
	@$(RM) $(BINDIR)
	@echo "robotcontrol Service Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)/lib/systemd/system/$(SERVICE).service
	@$(RM) $(DESTDIR)/etc/modules-load.d/robotcontrol_modules.conf
	@$(RM) $(DESTDIR)$(prefix)/$(TARGET)
	@echo "robotcontrol Service Uninstall Complete"


$(BINDIR)/% : $(SRCDIR)/%.c
	@mkdir -p $(BINDIR)
	@$(CC) -g -o $@ $< $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $(LDFLAGS)
	@echo "made: $@"
