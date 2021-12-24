# Comment/uncomment the following line to enable/disable debugging
# DEBUG = y 

# ifeq ($(DEBUG),y)
# DEBFLAGS = -O -g -DDEBUG # "-O" is needed to expand inlines
# else
# DEBFLAGS = -O2
# endif

# EXTRA_CFLAGS += $(DEBFLAGS) -I$(LDDINC) -DLINUX

# get_variable = $(strip $(shell grep "^[[:space:]]*$(1)[[:space:]]*=\{1\}[[:space:]]*" $(2) | cut -d= -f2))

# NOTE: In the call there can be *no* whitespace after the comma
# VERSION := $(call get_variable,VERSION,$(KERNEL_DIR)/Makefile)
# PATCHLEVEL := $(call get_variable,PATCHLEVEL,$(KERNEL_DIR)/Makefile)
# SUBLEVEL := $(call get_variable,SUBLEVEL,$(KERNEL_DIR)/Makefile)
# EXTRAVERSION := $(call get_variable,EXTRAVERSION,$(KERNEL_DIR)/Makefile)
# LOCALVERSION := $(call get_variable,LOCALVERSION,$(KERNEL_DIR)/.config)
# KERNELRELEASE := $(VERSION).$(PATCHLEVEL).$(SUBLEVEL)$(EXTRAVERSION)$(LOCALVERSION)

ifneq ($(KERNELRELEASE),)

$(info KERNELRELEASE is not empty)

cti_serial_core-objs := serial_core.o 8250_core.o 
cti_8250_pci-objs :=  8250_pci.o 

obj-m	:= cti_serial_core.o 
obj-m	+= cti_8250_pci.o 

# Kbuild syntax, rather than above -objs
# cti_serial_core-y := cti_serial_core.o
# cti_8250_pci-y := 8250_pci.o

else

$(info ifneq KERNELRELEASE is empty)

# if KERNEL_DIR is empty, look on /usr/src
ifeq ($(KERNEL_DIR),)
KERNEL_DIR := $(shell find /usr/src -maxdepth 1 -mindepth 1 -type d -print | head -n 1)
endif

# if KERNEL_DIR is still empty, get current kernel version - assumes we're building
# for this host system.
ifeq ($(KERNEL_DIR),)
KERNEL_DIR := /lib/modules/$(shell uname -r)/build
endif

$(info KERNEL_DIR=$(KERNEL_DIR))

default:
	$(MAKE) -C $(KERNEL_DIR) M=$$PWD

endif

all: modules

install: modules_install

modules modules_install clean::
	$(RM) *.o.ur-safe
	@$(MAKE) -C $(KERNEL_DIR) M=$$PWD $@

