# MAKEFILE =      Makefile
# ALL:    build
PREFIX = /usr

KERNEL_DIR ?= $(shell find /usr/src -maxdepth 1 -mindepth 1 -type d -print | head -n 1)
# KERNEL_DIR := /usr/src/linux-headers-4.15.18-vortex86dx3 

$(info DESTDIR=$(DESTDIR))
$(info DEB_BUILD_GNU_TYPE=$(DEB_BUILD_GNU_TYPE))
$(info DEB_HOST_GNU_TYPE=$(DEB_HOST_GNU_TYPE))
$(info DEB_HOST_MULTIARCH=$(DEB_HOST_MULTIARCH))

.PHONY: build
build:
	@echo "Building cti-serial..."
	$(MAKE) -C driver KERNEL_DIR=$(KERNEL_DIR)
	$(MAKE) -C utilities

.PHONY: install
install:
	@echo "Installing cti-serial..."
	$(MAKE) -C driver KERNEL_DIR=$(KERNEL_DIR) INSTALL_MOD_PATH=$(DESTDIR) modules_install
	$(MAKE) -C utilities DESTDIR=$(DESTDIR) PREFIX=$(PREFIX) install

.PHONY: clean
clean:
	@echo "Clean cti-serial..."
	$(MAKE) -C driver clean
	$(MAKE) -C utilities clean
