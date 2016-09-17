TARGET = librobotics_cape.so

SUBDIRS = $(shell find * -type d -print)
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fpic -march=armv7-a -mtune=cortex-a8
LFLAGS	:= -lm -lrt -lpthread -shared


PREFIX := /usr

SOURCES := $(shell find ./ -name '*.c')
INCLUDES := $(shell find ./ -name '*.h')
OBJECTS  := $(SOURCES:$%.c=$%.o)

all:$(TARGET)
	
	
# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(LFLAGS) -o $(TARGET) $(OBJECTS)
	
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) $(DEFS) -c $< -o $(@)
	@echo "Compiled: "$<



# install to /usr/bin
$(phony all) : $(TARGET)
.PHONY: install


install:
	$(MAKE)
	install -d -m 755 $(DESTDIR)$(PREFIX)/lib
	install -m 644 librobotics_cape.so $(DESTDIR)$(PREFIX)/lib
	install -d -m 755 $(DESTDIR)$(PREFIX)/include
	install -m 644 *.h $(DESTDIR)$(PREFIX)/include
	@ldconfig
	@echo "$(TARGET) Installed to $(DESTDIR)$(PREFIX)/lib"
	

clean:
	@for dir in $(SUBDIRS) ; do \
		rm -f $$dir/*.o ; \
	done
	
	@rm -f *.o
	@rm -f *.so
	@echo "librobotics_cape Cleanup Complete!"
