Linux CTI UART driver for BlueStorm, BlueHeat, Xtreme/104,
    Xtreme/104-Plus, Titan and Xtreme/104-Express families

File:	readme.txt
Date:	2019-04-01 ver 1.43

Kernel versions supported are 3.9.0 and up.

Build the driver source using 'make'. The build process will produce
cti_serial_core.ko and cti_8250_pci.ko. cti_serial_core.ko is
the driver core that manages the main method functions and cti_8250_pci.ko
detects CTI's PCI and PCI Express based boards.

The driver is a standalone module and can be loaded by insmod or modprobe
Run 'make install' after 'make' to load driver automatically on boot. 

CTI boards are also detected by standard '8250_pci' driver, therefore, 
make sure standard '8250_pci' driver does NOT
install them. You may have to restrict the number of ports the standard driver
maps to achieve this by passing it the 'nr_uarts=' option. Although the
modprobe tool should resolve load time module dependencies, the modules are
dependent on being loaded in the following order:
cti_serial_core.ko
cti_8250_pci.ko

For example to load the modules in a system that has multiple CTI boards, some
of which are PCI or PCI-Express, and the
total number of ports is 24.
'modprobe cti_serial_core nr_uarts=24' 
'modprobe cti_8250_pci'

The driver will produce device files prefixed with 'ttyCTI', whereas the
standard driver uses 'ttyS' as the prefix.

Note that on Ubuntu systems (and possibly others too) it is observed that the
'make install' phase does not complete all operations as expected. If at this
point modprobe complains that it cannot find the module you will need to
execute the following additional command as root:
'depmod -a'

If the driver is configured to load automatically on boot you can provide the nr_uarts
argument to cti_serial_core module by creating a custom file under /etc/modprobe.d

For example:
/etc/modprobe.d/cti.conf
-------------------------------------------------
options cti_serial_core nr_uarts=24

Due to the behaviour of the standard serial driver PCI detection, you will
need to ensure that the ports of your CTI board are not already taken by the
standard serial driver before you load the CTI driver. To check for a conflict
please verify that none of the MMIO listed in /proc/tty/driver/serial are
found in /proc/tty/driver/ctipu. In the case where the standard serial driver
is built into your kernel image you will need to append an "8250.nr_uarts=??"
to your kernel command-line in order to limit the number of ports that it will
map to less than the number that would cause it to map any of the ports on
the CTI board. When there is no conflict over port resources, the ports will
function correctly.

To install Xtreme/104 boards (NOT PC/104-Plus, PCI-104, PCI/104-Express, or
PCIe/104) use 'setserial' to assign resources to unassigned ports. The
following is a template that can be changed for use by direct invocation of
setserial, or a setserial configuration file as used by your distribution.
/dev/ttyCTI[[port_index]] port [[io]] baud_base [[baud_base]] auto_irq ^skip_test autoconfig

A specific example for one port will look like this for direct invocation:
setserial /dev/ttyCTI0 port 0x300 baud_base 480600 auto_irq ^skip_test autoconfig
or this for use in a configuration file:
/dev/ttyCTI0 port 0x300 baud_base 480600 auto_irq ^skip_test autoconfig

The driver will log useful information to /proc/tty/driver/ctipu.


RECENT RELEASES and ENHANCEMENTS TO THE DRIVER:

Revision 1.01 July 12, 2010
The following is a summary of the changes in this release:
1. 1st release.

Revision 1.02 July 16, 2010
The following is a summary of the changes in this release:
1. Documentation update about conflict with standard serial driver.
2. Tweak certain printk() messages.
3. Updates to support future products.

Revision 1.03 August 09, 2010
1. tweaks to support down to 2.6.26
2. Makefile tweaks

Revision 1.04 September 24, 2010
1. Update driver to support XIG family

Revision 1.04b September 28, 2010
1. Documentation updates.

Revision 1.05 October 05, 2010
1. Further accomodations for configurations with XR16V79x UARTs.
2. Clean up certain code that deals with port indices for boards
   where they matter.

Revision 1.06 January 28, 2011
1. Ported back to 2.6.18

Revision 1.07 March 3, 2011
1. Modified Readme example
 
Revision 1.08 March 14, 2011
1. Set485 fixed for 64 bit.

Revision 1.09 March 24, 2011
1. lock acquisition in 485 settings fixed.

Revision 1.11 April 08, 2011
1. Upgraded to 2.6.37

Revision 1.12 April 14, 2011
1. Fixed Xtreme/104 for default clock prescaler

Revision 1.14, 1.13 Feb 08, 2012
1. Support for 2.6.39 and up


Revision 1.15, April 10, 2012
1. Support for 2.6.39 and up; updated to 3.2

Revision 1.16, 1.17, April 20, 2012
1. Support for 2.6.39 and up; updated to 3.3

Revision 1.18, 1.19, Feb 01, 2013
1. Fractional baud support fixed.
2. Half duplex fixed for bluestorm and titan

Revision 1.20, 1.21 April 15, 2013
1. Half duplex fixed for Xtreme/104-Express 

Revision 1.22, August 11, 2013
1. Fixed half duplex operation with power on tristate

Revision 1.23, August 21, 2013
1. Added support for version 3.8 of Linux

Revision 1.24, August 23, 2013
1. Fixed Oops with version 1.23

Revision 1.25, Sept 30, 2013
1. Plx bridge 8511 support for some UART fixed.

Revision 1.26, Mar 18, 2014
1. Added support for version 3.11 of Linux

Revision 1.27, Apr 11, 2014
1. Added support for version 3.9 to 3.11 of Linux

Revision 1.28, Jun 19, 2014
1. Fixed UART detection bug
2. Fixed interrupt issues for XR17XX5X UART

Revision 1.29, Jul 11, 2014
1. Fixed bug causing kernel panic in PCI IO mapped boards

Revision 1.30, Jul 31, 2014
1. Fixed problems with  unhandled interrupts 

Revision 1.31, Aug 11, 2014
1. Added support for boards that use MPIO for line mode settings

Revision 1.32, Mar 26, 2014
1. Fixed bug which was causing communication issues with BlueStorm/Express boards 

Revision 1.33, April 21, 2015
1. Fixed fractional baud rate support

Revision 1.34, Jan 22, 2016
1. Updated to fix compilation issues in kernel 4.2+

Revision 1.35, May 26, 2016
1. Fixed compilation issues seen when au_serial_in() and au_serial_out() was defined 

Revision 1.36, Oct 17, 2016
1. Added support for XIG01X boards

Revision 1.37, Dec 06, 2016
1. Defined pointer to .compat_ioctl for calling IOCTL's from 32 bit userspace 
   on 64 bit kernel 
 
Revision 1.38, Jan 17, 2017
1. Removed obscure wakeup call

Revision 1.39, July 27, 2017
1. Updated to fix compilation issues in kernel 4.11

Revision 1.40, October 20, 2017
1. Fixed userspace pointer bug that was causing kernel panic 

Revision 1.41, January 05, 2018
1. Fixed issue where kernel pointer was being used as userspace pointer

Revision 1.42, June 11, 2018
1. Kernel 4.15 support added

Revision 1.43, April 01, 2019
1. Kernel 4.18 support added

CONTACTING TECHNICAL SUPPORT.

   If you have any problems, questions or suggestions regarding this driver
   and the products it supports please feel free to contact Connect Tech Inc.
   Connect Tech can be reached in a variety of
   ways:

   MAIL: Connect Tech Inc.
         42 Arrow Road
         Guelph, Ontario, Canada
         N1K 1S6

   TEL 1 519 836 1291 or 1-800-426-8979 in North America
   FAX 1 519 836 4878

   INTERNET:
   Sales:        sales@connecttech.com
   Tech Support: support@connecttech.com
   WWW:          http://www.connecttech.com

   Be sure to check the support section of our home page for answers to
   technical questions at http://www.connecttech.com. Also be sure to
   browse the knowledge data base. If you do not find what you are
   looking for, please contact the support department and let us know.
   We will be glad to help you.

 --- End of README ---

