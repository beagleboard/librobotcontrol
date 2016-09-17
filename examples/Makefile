# makefile to make all example programs

CWD = $(PWD)
SUBDIRS = $(shell find * -type d -print)

all:
	@for dir in $(SUBDIRS) ; do \
		echo "making example: $$dir" ; \
		$(MAKE) -C $$dir -s --no-print-directory; \
	done
	
clean:
	@for dir in $(SUBDIRS) ; do \
		echo "cleaning example: $$dir" ; \
		$(MAKE) -C $$dir -s clean; \
	done
	
install:
	@for dir in $(SUBDIRS) ; do \
		echo "installing example: $$dir" ; \
		$(MAKE) -C $$dir -s install ;\
	done
