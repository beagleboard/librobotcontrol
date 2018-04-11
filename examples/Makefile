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
WFLAGS		:= -Wall -Wextra
CFLAGS		:= -pthread -I $(INCLUDEDIR)
LFLAGS		:= -lm -lrt -pthread -L $(LIBDIR) -lroboticscape

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
LINK		:= ln -s -f
LINKDIR		:= /etc/roboticscape
LINKNAME	:= link_to_startup_program

# prefix variable in case this is used to make a deb package
prefix		:= /usr




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
	@$(INSTALL) $(TARGETS) $(prefix)/bin/
	@echo "Examples Install complete"

uninstall:
	@for f in $(TARGETS); do $(RM) $(prefix)/$$f; done
	@echo "Examples Uninstall complete"


$(BINDIR)/% : $(SRCDIR)/%.c
	@mkdir -p $(BINDIR)
	@$(CC) -o $@ $< $(CFLAGS) $(WFLAGS)  $(DEBUGFLAG)  $(LFLAGS)
	@echo "made: $@"
