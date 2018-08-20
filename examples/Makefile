# makefile for rc_mpu, builds example programs

# directories
SRCDIR		:= src
BINDIR		:= bin
INCLUDEDIR	:= ../library/include
LIBDIR		:= ../library/lib

# basic definitions for rules
EXAMPLES	:= $(shell find $(SRCDIR) -type f -name *.c)
TARGETS		:= $(EXAMPLES:$(SRCDIR)/%.c=$(BINDIR)/%)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)

# compiler and linker programs
CC		:= gcc
LINKER		:= gcc

# compile flags
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -pedantic -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef
CFLAGS		:= -g -pthread -I $(INCLUDEDIR)
LDFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -l:librobotcontrol.so.1

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
LINK		:= ln -s -f
LINKDIR		:= /etc/librobotcontrol
LINKNAME	:= link_to_startup_program

# prefix variable in case this is used to make a deb package
prefix		?= /usr




all : $(TARGETS)

debug :
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "Make Debug Complete"
	@echo " "

clean :
	@$(RM) $(BINDIR)
	@echo "Examples Clean Complete"

install:
	$(MAKE)
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@for f in $(TARGETS); do $(INSTALL) $$f $(DESTDIR)$(prefix)/bin/; done
	@echo "Examples Install complete"

uninstall:
	@for f in $(TARGETS); do $(RM) $(DESTDIR)$(prefix)/$$f; done
	@echo "Examples Uninstall complete"


$(BINDIR)/% : $(SRCDIR)/%.c
	@mkdir -p $(BINDIR)
	@$(CC) -o $@ $< $(CFLAGS) $(WFLAGS)  $(DEBUGFLAG) $(LDFLAGS)
	@echo "made: $@"
