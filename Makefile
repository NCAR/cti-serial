# MAKEFILE =      Makefile
# ALL:    build
PREFIX = /usr

KERNELDIR ?= $(shell find /usr/src/linux-header* -maxdepth 1 -mindepth 1 -type d -print | head -n 1)
# KERNELDIR := /usr/src/linux-headers-4.15.18-vortex86dx3 

$(info DESTDIR=$(DESTDIR))
$(info KERNELDIR=$(KERNELDIR))
$(info DEB_BUILD_GNU_TYPE=$(DEB_BUILD_GNU_TYPE))
$(info DEB_HOST_GNU_TYPE=$(DEB_HOST_GNU_TYPE))
$(info DEB_HOST_MULTIARCH=$(DEB_HOST_MULTIARCH))

.PHONY: build
build:
	@echo "Building cti-serial..."
	$(MAKE) -C driver KERNELDIR=$(KERNELDIR)
	$(MAKE) -C utilities

.PHONY: install
install:
	@echo "Installing cti-serial..."
	$(MAKE) -C driver KERNELDIR=$(KERNELDIR) INSTALL_MOD_PATH=$(DESTDIR) modules_install
	$(MAKE) -C utilities DESTDIR=$(DESTDIR) PREFIX=$(PREFIX) install

.PHONY: clean
clean:
	@echo "Clean cti-serial..."
	$(MAKE) -C driver clean
	$(MAKE) -C utilities clean
