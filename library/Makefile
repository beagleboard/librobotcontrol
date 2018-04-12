# makefile for libroboticscape.so

# directories
SRCDIR		:= src
LIBDIR		:= lib
BUILDDIR	:= build
INCLUDEDIR	:= include
SONAME		:= libroboticscape.so
TARGET		:= $(LIBDIR)/$(SONAME)

# file definitions for rules
SOURCES		:= $(shell find $(SRCDIR) -type f -name *.c)
OBJECTS		:= $(SOURCES:$(SRCDIR)/%.c=$(BUILDDIR)/%.o)
INCLUDES	:= $(shell find $(INCLUDEDIR) -name '*.h')

# compiler and linker binaries
CC		:= gcc
LINKER		:= gcc

# general compiler flags
WFLAGS		:= -Wall -Wextra
CFLAGS		:= -fPIC -I $(INCLUDEDIR)
OPT_FLAGS	:= -O1
LDFLAGS		:= -lm -lrt -pthread -shared -Wl,-soname,$(SONAME)

# different compile flags for math libs
MATH_OPT_FLAGS	:= -O3 -ffast-math -ftree-vectorize
#ARCFLAGS	:= -mfpu=neon -march=armv7-a -mtune=cortex-a8
#FLOAT_FLAG	:= -mfloat-abi=hard

# commands
RM		:= rm -rf
INSTALL		:= install -m 755
INSTALLDIR	:= install -d -m 755

# prefix variable for making debian package
prefix		?= /usr



# linking rules
$(TARGET): $(OBJECTS)
	@mkdir -p $(LIBDIR)
	@$(LINKER) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	@echo "Done making $(TARGET)"

# rule for math libs
$(BUILDDIR)/math/%.o : $(SRCDIR)/math/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(ARCFLAGS) $(MATH_OPT_FLAGS) $(FLOAT_FLAG) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"

# rule for all other objects
$(BUILDDIR)/%.o : $(SRCDIR)/%.c $(INCLUDES)
	@mkdir -p $(dir $(@))
	@$(CC) -c $(CFLAGS) $(ARCFLAGS) $(OPT_FLAGS) $(FLOAT_FLAG) $(DEBUGFLAG) $< -o $(@)
	@echo "made: $(@)"




# rule for target
all:
	$(TARGET)

# compile with soft floating point math for labview
softfloat:
	$(MAKE) $(MAKEFILE) FLOAT_FLAG="-mfloat-abi=soft"
	@echo "$(TARGET) Make softfloat Complete"

# compile with debug symbols and DEBUG defined
debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo "$(TARGET) Make Debug Complete"

# make and install everything
install:
	$(MAKE)
	$(INSTALLDIR) $(DESTDIR)$(prefix)/include
	@cp -r include/* $(DESTDIR)$(prefix)/include
	$(INSTALLDIR) $(DESTDIR)$(prefix)/lib
	$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/lib
	@echo "Library Install Complete"

# cleanup local binaries
clean:
	@$(RM) $(LIBDIR)
	@$(RM) $(BUILDDIR)
	@echo "Library Clean Complete"

# uninstall completely from system
uninstall:
	$(RM) $(DESTDIR)$(prefix)/$(TARGET)
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape.h
	$(RM) $(DESTDIR)$(prefix)/include/rc
	@echo "Library Uninstall Complete"




