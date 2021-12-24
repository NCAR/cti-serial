# cti-serial
Driver, utilities and configuration for Connect Tech Xtreme 104 serial cards.

The cti_serial_core driver is identical to version 143 from Connect Tech.

The driver creates device files, called **/dev/ttyCTI\***.

This repository contains a debian directory for creating a native debian package, and a **build_dpkg.sh** script to build it, typically in a container.

The package contains the following configuration files:

## Contents of cti-serial package

### Kernel modules
- cti_serial_core.ko

### Executable binaries on /usr/bin
- arb_setbaud
- setmodem
- set485

### Configuration files
- /etc/modules-load.d/cti-serial.conf: enables loading cti_serial_core module at boot
- /etc/modprobe.d/cti-serial.conf: options for cti_serial_core module

- /etc/serial.conf: created or modified by the postinst script of cti-serial

    The postinst script for this package does the following:
    1. If /etc/serial.conf doesn't exist, it is copied from /var/lib/setserial/autoserial.conf
    2. adds setserial entries for **/dev/ttyCTI\*** files to /etc/serial.conf.

#### setserial services in Ubuntu

    There are two setserial services in Ubuntu bionic (and maybe other distributions):

    1. setserial.service, runs /etc/init.d/setserial.

       If /etc/serial.conf exists, then this script does nothing, otherwise it
       runs setserial on the entries in /var/lib/setserial/autoserial.conf.
    2. etc-setserial.service, runs /etc/init.d/etc-setserial.
       
       Runs setserial on the entires in /etc/serial.conf.

    So after this package is installed, /etc/serial.conf should exist
    and contain entries for the on-board serial ports,
    (/dev/ttyS[0-3] on Vertex) and for the CTI ports (/dev/ttyCTI[0-7]).
    These entries will be read by the etc-setserial.service, /etc/init.d/etc-setserial.

### cti-serial.service
- /lib/systemd/system/cti-serial.service
- /lib/systemd/cti-serial-service.sh, script run by cti-serial.service

/lib/systemd/cti-serial-service.sh creates **/dev/ttyS[4-11]** backward compatible serial device files.
        
## Building cti-serial package

### Building in a Debian container
The nidas-devel package contains a **start_podman** script in **/opt/nidas/bin** which can be used to start docker/podman images for building software.

    # clone this repository
    git clone https://github.com/NCAR/cti-serial.git
    # run a podman image, mounting the current directory
    cd cti-serial
    # build the package, install to codename-bionic repository.
    start_podman bionic /root/current/build_dpkg.sh i386 -I bionic
