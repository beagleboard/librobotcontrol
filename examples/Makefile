# makefile to make all example programs

CWD = $(PWD)
SUBDIRS = $(shell find * -type d -print)
PREFIX := /usr

all:
	@for dir in $(SUBDIRS) ; do \
		echo "Making Example: $$dir" ; \
		$(MAKE) -C $$dir -s --no-print-directory; \
	done
	
clean:
	@for dir in $(SUBDIRS) ; do \
		echo "Cleaning Example: $$dir" ; \
		$(MAKE) -C $$dir -s clean; \
	done
	
install:
	@install -d -m 755 $(DESTDIR)$(PREFIX)/bin
	@for dir in $(SUBDIRS) ; do \
		echo "Installing Example: $$dir" ; \
		$(MAKE) -C $$dir -s install ;\
	done

uninstall:
	@for dir in $(SUBDIRS) ; do \
		echo "Uninstalling Example: $$dir" ; \
		$(MAKE) -C $$dir -s uninstall ;\
	done



