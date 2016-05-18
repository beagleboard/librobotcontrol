# makefile to make all example programs

CWD = $(PWD)
SUBDIRS = $(shell find * -type d -print)

all:
	@for dir in $(SUBDIRS) ; do \
		echo "making example: $$dir" ; \
		cd "$$dir" ; \
		$(MAKE) -s; \
		cd ../ ; \
	done
	
clean:
	@for dir in $(SUBDIRS) ; do \
		echo "cleaning example: $$dir" ; \
		cd "$$dir" ; \
		$(MAKE) -s clean; \
		cd ../ ; \
	done
	
install:
	@for dir in $(SUBDIRS) ; do \
		echo "installing example: $$dir" ; \
		cd "$$dir" ; \
		$(MAKE) -s install; \
		cd ../ ; \
	done