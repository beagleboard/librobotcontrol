TARGET = libroboticscape.so


SUBDIRS		:=	$(shell find * -type d -print)
CC			:= gcc
LINKER		:= gcc
TOUCH		:= $(shell touch *)
CWARNINGS	:= -Wall
CFLOATFLAGS	:= -frounding-math -fsignaling-nans
CFLAGS		:= -fPIC -march=armv7-a -mtune=cortex-a8 
LFLAGS		:= -lm -lrt -lpthread -shared -Wl,-soname,$(TARGET)

SOURCES		:= $(shell find ./ -name '*.c')
INCLUDES	:= $(shell find ./ -name '*.h')
OBJECTS		:= $(SOURCES:$%.c=$%.o)

prefix		:= /usr
RM			:= rm -f -r
INSTALL		:= install -m 755 
INSTALLDIR	:= install -d -m 644
INSTALLNOEXEC := install -m 644


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	@echo " "
	@echo "Linked $(TARGET)"
	@echo " "
	
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) $(CWARNINGS) $(CFLOATFLAGS) $(DEBUGFLAG) $(DEFS) -c $< -o $(@)
	@echo "Compiled: "$<

all:
	$(TARGET)


debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install:
	$(MAKE)
	@# includes
	$(INSTALLDIR) $(DESTDIR)$(prefix)/include
	$(INSTALL) roboticscape.h $(DESTDIR)$(prefix)/include/
	$(INSTALL) roboticscape-defs.h $(DESTDIR)$(prefix)/include/
	$(INSTALL) roboticscape-usefulincludes.h $(DESTDIR)$(prefix)/include/
	@# library .so
	$(INSTALLDIR) $(DESTDIR)$(prefix)/lib
	$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/lib
	@echo " "
	@echo "$(TARGET) Install Complete"
	@echo " "
	

clean:
	@$(RM) */*.o
	@$(RM) *.o
	@$(RM) *.so
	@echo " "
	@echo "$(TARGET) Clean Complete"
	@echo " "


uninstall:
	$(RM) $(DESTDIR)$(prefix)/lib/$(TARGET)
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape.h
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape-defs.h
	$(RM) $(DESTDIR)$(prefix)/include/roboticscape-usefulincludes.h
	@echo " "
	@echo "$(TARGET) Uninstall Complete"
	@echo " "

	



