# cti-serial
Driver, utilities and configuration for Connect Tech Xtreme 104 serial cards.

The **cti_serial_core** driver module is identical to version 143 from Connect Tech.

The driver creates device files, called **/dev/ttyCTI\***.  Also see below about the **cti-serial.service** which creates device files with the old naming convention: **/dev/ttyS\***.

This repository contains a debian directory for creating a native debian package, and a **build_dpkg.sh** script to build it, typically in a container.

## Supported Serial Cards

The cti-serial package has been tested for the [Connect Tech Xtreme 104 Isolated](https://connecttech.com/product/xtreme104-isolated/) serial card.

This is a PC/104 card, not PCI.  With slight modifications it should work for other Connect Tech cards, but modifications (simplifications) for PCI will be more significant.

## Contents of cti-serial package

### Kernel modules
- cti_serial_core.ko

### Executable binaries on /usr/bin
- arb_setbaud
- setmodem
- set485

### Configuration files
- /etc/modules-load.d/cti-serial.conf: enables loading the **cti_serial_core** module at boot
- /etc/modprobe.d/cti-serial.conf: options for cti_serial_core
  - nr_uarts: the number of ports on the CTI card. 12 for the Xtreme/104 Isolated card.
- /etc/serial.conf: created or modified by the postinst script of cti-serial

    The postinst script for this package does the following:
    1. If /etc/serial.conf doesn't exist, it is copied from /var/lib/setserial/autoserial.conf
    2. Adds setserial entries for **/dev/ttyCTI\*** files to /etc/serial.conf.

#### setserial services in Ubuntu
There are two setserial services in Ubuntu bionic (and maybe other distributions):

- setserial.service

The setserial.service invokes /etc/init.d/setserial.  If /etc/serial.conf exists, then this
script does nothing, otherwise it runs setserial on the entries in /var/lib/setserial/autoserial.conf.

- etc-setserial.service

The etc-setserial.service invokes /etc/init.d/etc-setserial, which runs setserial on the entires in /etc/serial.conf.

So after cti-serial is installed, /etc/serial.conf should exist
and contain entries for the on-board serial ports
(/dev/ttyS[0-3] on Vortex) and for the CTI ports (/dev/ttyCTI[0-11]) (assuming 12 ports).
These entries will then be read by /etc/init.d/etc-setserial.

### cti-serial.service
- /lib/systemd/system/cti-serial.service
- /lib/systemd/cti-serial-service.sh, script run by cti-serial.service

/lib/systemd/cti-serial-service.sh creates **/dev/ttyS[4-15]** which can be used instead of **/dev/ttyCTI[0-11]**.
        
### Number of UARTs

The Connect Tech Xtreme 104 Isolated card has 12 UARTs. So in **/etc/modprobe.d/cti-serial.conf** nr_uarts is set to 12.  In cti-serial/driver/8205_core.c, nr_uarts defaults to 16. It doesn't hurt to leave it at 16, except that unnecessary files /dev/ttyCTI12- ttyCTI15 are created, which might mislead some users to think they are functional.

The number of UARTs on the motherboard is a kernel configuration parameter:

    grep "CONFIG_SERIAL_8250_.*UARTS" /boot/config-4.15.18-vortex86dx3 

On the Vortex, with 4 UARTs on the motherboard, but CONFIG_SERIAL_8250_RUNTIME_UARTS=32, then 32 /dev/ttyS\* ports are created when the 8250 serial driver is loaded, even though devices 4-31 are not functional.

To avoid conflict between the /dev/ttyS device files created by the 8250 serial driver, and the files created for backward compatiblity by cti-serial.service, the cti-serial package alters the GRUB boot line, **GRUB_CMDLINE_LINUX_DEFAULT** in **/etc/default/grub** to set **8250.nr_uarts=4**. Then the 8250 driver will only create device files **/dev/ttyS0-ttyS3** when it is loaded.

## Building cti-serial package

### KERNEL_DIR
To build the cti_serial_core module, make needs to be able to find the Linux kernel headers matching the kernel version of the target system.

The headers are typically found in **/usr/src**, in a directory such as **linux-headers-4.15.18-vortex86dx3**.

To build On Debian you need to install a **linux-headers** package corresponding to the kernel of the target system.  

For example, the header package for a Vortex DX3 running Ubuntu bionic, kernel 4.15.18:

    apt-get install linux-headers-4.15.18-vortex86dx3

Assuming builds are only done in containers, not on a Vortex, the **linux-headers** package only needs to be installed in the container image.

The Makefile will set KERNEL_DIR to the first directory that is found on /usr/src.  If there are more than one, you should move or uninstall the extra ones, or hand edit the Makefile and set the value of KERNEL_DIR. This can also be done if the headers are in some other directory than **/usr/src**:

    KERNEL_DIR := /my/location/linux-headers-4.15.18-vortex86dx3 

### Building in a Debian container
The nidas-devel package contains a **start_podman** script in **/opt/nidas/bin** which can be used to run docker/podman images for building software.

1. Clone the cti-serial repository:

        git clone https://github.com/NCAR/cti-serial.git
        cd cti-serial

1. Run a podman image interactively.  An interactive session may be necessary if you're installing
the package and the gpg-agent needs to prompt for the password to the \<eol-prog@ucar.edu\> signing key.

    start_podman bionic

    1. The Ubuntu bionic image on docker.io/ncar contains the linux-headers-4.15.18-vortex86dx3 package.

       If you need to install a different header package for the target system:

            apt-get update 
            apt-get install linux-headers-x.y.z

    1. Edit the Makefile if you need to override the default search for KERNEL_DIR on **/usr/src**:

            vi /root/current/Makefile

    1. Build the package, which will be placed in the parent directory (/root in this example):

            cd /root/current
            ./build_dpkg.sh i386

    1. Add the -I option to install the package with reprepo to the EOL debian repository on /net/ftp

            ./build_dpkg.sh i386 -I bionic

1. To build the package non-interacively

        start_podman bionic /root/current/build_dpkg.sh i386 -I bionic
