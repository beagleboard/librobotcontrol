# directories
SRCDIR		:= src
LIBDIR		:= lib
BUILDDIR	:= build
INCLUDEDIR	:= include
SHORTNAME	:= librobotcontrol.so
SONAME		:= librobotcontrol.so.1
FULLNAME	:= librobotcontrol.so.1.0.5
TARGET		:= $(LIBDIR)/$(FULLNAME)
RC_VAR_DIR	:= var/lib/robotcontrol

# file definitions for rules
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')

# compiler and linker binaries
CC		:= gcc
LINKER		:= gcc

# general compiler flags
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -pedantic -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef
CFLAGS		:= $(EXT_CFLAGS) -g -fPIC -I $(INCLUDEDIR)
OPT_FLAGS	:= -O1
LDFLAGS		:= -lm -lrt -pthread -shared -Wl,-soname,$(SONAME)

# different compile flags for math libs
MATH_OPT_FLAGS	:= -O3 -ffast-math -ftree-vectorize

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755
INSTALLDIRWRITE	:= install -d -m 777

# prefix variable for making debian package
prefix		?= /usr



# linking rules
# symlink only for local examples compilation, not for install
$(TARGET): $(OBJECTS)
	@mkdir -p $(LIBDIR)
	@$(LINKER) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	@ln -sf $(FULLNAME) $(LIBDIR)/$(SONAME)
	@echo "Done making $(TARGET)"

# rule for math libs
$(BUILDDIR)/math/%.o : $(SRCDIR)/math/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(WFLAGS) $(MATH_OPT_FLAGS) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"

# rule for all other objects
$(BUILDDIR)/%.o : $(SRCDIR)/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(WFLAGS) $(OPT_FLAGS) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"




# rule for target
all:	$(TARGET)

# compile with debug symbols and DEBUG defined
debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo "$(TARGET) Make Debug Complete"


install:
	$(MAKE)
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/include
	@cp -r include/* $(DESTDIR)$(prefix)/include
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/lib
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/lib
	@ln -sf $(FULLNAME) $(DESTDIR)$(prefix)/lib/$(SONAME)
	@ln -sf $(FULLNAME) $(DESTDIR)$(prefix)/lib/$(SHORTNAME)
	@ln -sf $(FULLNAME) $(DESTDIR)$(prefix)/lib/libroboticscape.so
	@$(INSTALLDIRWRITE) $(DESTDIR)/$(RC_VAR_DIR)
	@echo "Library Install Complete"

# cleanup local binaries
clean:
	@$(RM) $(LIBDIR)
	@$(RM) $(BUILDDIR)
	@echo "Library Clean Complete"

# uninstall completely from system
uninstall:
	$(RM) $(DESTDIR)$(prefix)/lib/$(FULLNAME)
	$(RM) $(DESTDIR)$(prefix)/lib/$(SONAME)
	$(RM) $(DESTDIR)$(prefix)/lib/$(SHORTNAME)
	$(RM) $(DESTDIR)$(prefix)/lib/libroboticscape.so
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape.h
	$(RM) $(DESTDIR)$(prefix)/include/robotcontrol.h
	$(RM) $(DESTDIR)$(prefix)/include/rc_usefulincludes.h
	$(RM) $(DESTDIR)$(prefix)/include/rc
	@echo "Library Uninstall Complete"




