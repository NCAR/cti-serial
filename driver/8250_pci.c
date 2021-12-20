/*
 *  Probe module for 8250/16550-type PCI serial ports.
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial_reg.h>
#include "serial_core.h"
#include "pci_local.h"
#include <linux/8250_pci.h>
#include <linux/bitops.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "8250.h"

int cti_serial8250_register_8250_port(struct uart_8250_port *up);
void cti_serial8250_unregister_port(int line);
void cti_serial8250_suspend_port(int line);
void cti_serial8250_resume_port(int line);

int pci_cti485_bl(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_bl_6(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_bl_4(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_bl_2(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_bl_1(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_4(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_2(struct uart_port *port, int ioctl, unsigned int *value);
int pci_cti485_tn(struct uart_port *port, int ioctl, unsigned int *value);
int pci_ctiptm(struct uart_port *port, int ioctl, unsigned int *value);

#undef SERIAL_DEBUG_PCI

/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*probe)(struct pci_dev *dev);
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct serial_private *,
			 const struct pciserial_board *,
			 struct uart_8250_port *, int);
	void	(*exit)(struct pci_dev *dev);
};

#define PCI_NUM_BAR_RESOURCES	6

struct serial_private {
	struct pci_dev		*dev;
	unsigned int		nr;
	void __iomem		*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	int			line[0];
};

#define MAX_CTI_PCI_BOARDS 16

struct pci_dev *pci_pdev[MAX_CTI_PCI_BOARDS];

static int num_cti_pci_uart_board = 0;

static int pci_default_setup(struct serial_private*,
	  const struct pciserial_board*, struct uart_8250_port *, int);

static void moan_device(const char *str, struct pci_dev *dev)
{
	printk(KERN_WARNING
	       "%s: %s\n"
	       "Please send the output of lspci -vv, this\n"
	       "message (0x%04x,0x%04x,0x%04x,0x%04x), the\n"
	       "manufacturer and name of serial board or\n"
	       "modem board to rmk+serial@arm.linux.org.uk.\n",
	       pci_name(dev), str, dev->vendor, dev->device,
	       dev->subsystem_vendor, dev->subsystem_device);
}

#define XR17C15x_REGB		0x8e
#define XR17C15x_REGB_EECK	0x10
#define XR17C15x_REGB_EECS	0x20
#define XR17C15x_REGB_EEDI	0x40
#define XR17C15x_REGB_EEDO	0x80
#define XR17C15x_REGB_EE_ADDR_SIZE	6
#define XR17C15x_REGB_EE_DATA_SIZE	16

static inline void xr17c15x_ee_setpin(void *mem_base, int pin, int bit)
{
	static unsigned char regb = 0;

	if (bit)
		regb |= pin;
	else
		regb &= ~pin;

	writeb(regb, mem_base + XR17C15x_REGB);
}

static inline void xr17c15x_ee_select(void *mem_base)
{
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EEDI, 0);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 0);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECS, 1);
	udelay(2);
}

static inline void xr17c15x_ee_deselect(void *mem_base)
{
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECS, 0);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 0);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EEDI, 0);
	udelay(2);
}

static inline void xr17c15x_ee_clockin(void *mem_base, int bit)
{
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EEDI, bit);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 0);
	udelay(2);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 1);
	udelay(2);
}

static inline int xr17c15x_ee_clockout(void *mem_base)
{
	int regb;

	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 0);
	udelay(2);
	xr17c15x_ee_setpin(mem_base, XR17C15x_REGB_EECK, 1);
	udelay(2);

	regb = readb(mem_base + XR17C15x_REGB);

	return (regb & XR17C15x_REGB_EEDO ? 1 : 0);
}

static int xr17c15x_ee_read(void *mem_base, int ee_addr)
{
	int i;
	int data;

	xr17c15x_ee_select(mem_base);

	/* send command read */
	xr17c15x_ee_clockin(mem_base, 1);
	xr17c15x_ee_clockin(mem_base, 1);
	xr17c15x_ee_clockin(mem_base, 0);

	/* send address */
	for (i = 1 << (XR17C15x_REGB_EE_ADDR_SIZE - 1); i; i >>= 1)
		xr17c15x_ee_clockin(mem_base, ee_addr & i);

	/* read data */
	for (i = 0, data = 0; i <= XR17C15x_REGB_EE_DATA_SIZE; i++) {
		data <<= 1;
		data |= xr17c15x_ee_clockout(mem_base);
	}

	xr17c15x_ee_deselect(mem_base);

	return data;
}


static int
setup_port(struct serial_private *priv, struct uart_8250_port *port,
	   int bar, int offset, int regshift)
{
	struct pci_dev *dev = priv->dev;
	unsigned long base, len;

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	base = pci_resource_start(dev, bar);

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap_nocache(base, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		port->port.iotype = UPIO_MEM;
		port->port.iobase = 0;
		port->port.mapbase = base + offset;
		port->port.membase = priv->remapped_bar[bar] + offset;
		port->port.regshift = regshift;
	} else {
		port->port.iotype = UPIO_PORT;
		port->port.iobase = base + offset;
		port->port.mapbase = 0;
		port->port.membase = NULL;
		port->port.regshift = 0;
	}
	return 0;
}

/*
 * Oxford Semiconductor Inc.
 * Check that device is part of the Tornado range of devices, then determine
 * the number of ports available on the device.
 */
static int pci_oxsemi_tornado_init(struct pci_dev *dev, struct pciserial_board *board)
{
	u8 __iomem *p;
	unsigned long deviceID;
	unsigned int  number_uarts;

	/* OxSemi Tornado devices are all 0xCxxx */
	if (dev->vendor == PCI_VENDOR_ID_OXSEMI &&
	    (dev->device & 0xF000) != 0xC000)
		return 0;

	p = pci_iomap(dev, 0, 5);
	if (p == NULL)
		return -ENOMEM;

	deviceID = ioread32(p);
	/* Tornado device */
	if (deviceID == 0x07000200) {
		number_uarts = ioread8(p + 4);
		board->num_ports = number_uarts;
		printk(KERN_DEBUG
			"%d ports detected on Oxford PCI Express device\n",
								number_uarts);
	}
	pci_iounmap(dev, p);
	return 0;
}

static int pci_default_setup(struct serial_private *priv,
		  const struct pciserial_board *board,
		  struct uart_8250_port *port, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;

	bar = FL_GET_BASE(board->flags);
	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else
		offset += idx * board->uart_offset;

	maxnr = (pci_resource_len(priv->dev, bar) - board->first_offset) >>
		(board->reg_shift + 3);

	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;

	return setup_port(priv, port, bar, offset, board->reg_shift);
}


/*
 * Connect Tech cards
 */
static int cti_get_clock(struct pci_dev *dev,
						const struct pciserial_board *board)
{
	unsigned long start;
	unsigned char *mem_base;
	int eeprom_offset;
	int osc_freq;

	start = pci_resource_start(dev, 0);
	mem_base = ioremap_nocache(start, board->uart_offset);

	switch (dev->device) {
		default:
		case PCI_DEVICE_ID_EXAR_XR17C152:
		case PCI_DEVICE_ID_EXAR_XR17C154:
		case PCI_DEVICE_ID_EXAR_XR17C158:
			eeprom_offset = 0x4;
			break;
		case PCI_DEVICE_ID_EXAR_XR17V252:
		case PCI_DEVICE_ID_EXAR_XR17V254:
		case PCI_DEVICE_ID_EXAR_XR17V258:
			eeprom_offset = 0x8;
			break;
	}

	osc_freq = xr17c15x_ee_read(mem_base, eeprom_offset);
	osc_freq |= xr17c15x_ee_read(mem_base, eeprom_offset+1) << 16;

	iounmap(mem_base);

	return osc_freq;
}

static unsigned int cti_get_35x_port_flags_from_eeprom(struct pci_dev *dev,
			const struct pciserial_board *board, struct uart_port *port, int idx)
{
	unsigned long start;
	unsigned char *mem_base;
	unsigned int  port_flags;
	const unsigned int port_offset  = 0x14;

	start = pci_resource_start(dev, 0);
	mem_base = ioremap_nocache(start, board->uart_offset);

	
	port_flags  = xr17c15x_ee_read(mem_base, port_offset + idx);

	iounmap(mem_base);

	if( port_flags ) {
		unsigned int tmp_masked = port_flags & 0xf;
		printk(KERN_INFO "CTIserial: Port 0x%02x is: ", idx);
		port->use_485_3t_mpio = 1;
		port->use_mpio_input = 0;
		port->use_clr_rts =0;
		if (tmp_masked == 6) {
			printk("Switchable (HW & SW) (2 bits per port)");
			port->lmode_fn = pci_cti485_bl;
			port->use_mpio_input=2;
		}
		else if (tmp_masked == 5) {
			printk("Switchable (HW & SW) (4 bits per port)");
			port->lmode_fn = pci_cti485_bl;
			port->use_mpio_input=4;
		}
		else if( tmp_masked == 4 ) {
			printk("Switchable (S/W) RS-232/422/485");
			port->lmode_fn = pci_cti485_bl;
		}
		else if ( tmp_masked == 3 ) {
			printk("Switchable (H/W) RS-232/422/485");
			port->lmode_fn = pci_cti485_bl;
		}
		else if ( tmp_masked == 2 ) {
			printk("RS-422/485");
			port->lmode_fn = pci_cti485_bl;
		}
		else if ( tmp_masked == 1 ) {
			printk("RS-232");
			port->use_485_3t_mpio = 0;
		}
		else
			printk("Bogus line mode!");

		tmp_masked = (port_flags >> 8) & 0xf;
		if( tmp_masked == 3 )
			printk(", Electro-Isolated\n");
		else if ( tmp_masked == 2 )
			printk(", Opto-Isolated\n");
		else if ( tmp_masked == 1 )
			printk(", NON-Isolated\n");
		else
			printk(", Bogus Iso type!\n");

	} else {
		printk(KERN_INFO "Invalid port flags setting found in EEPROM!\n");
	}

	return port_flags;
}

static int pci_cti_setup(struct serial_private *priv,
		const struct pciserial_board *board, struct uart_8250_port *port, int idx)
{
	port->port.pci_dev_id = priv->dev->device;
	/* set the line mode handling function */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH081101V1:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH041101V1:
			port->port.lmode_fn = pci_cti485;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485_4_4:
			port->port.lmode_fn = pci_cti485_4;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_485_2_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485_2_6:
			port->port.lmode_fn = pci_cti485_2;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_1_1:
			port->port.lmode_fn = pci_cti485_bl_1;
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_6_SP:
			port->port.lmode_fn = pci_cti485_bl_2;
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_XPRS_OPTO:
			port->port.lmode_fn = pci_cti485_bl_4;
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_6_2_SP:
			port->port.lmode_fn = pci_cti485_bl_6;
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_SP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_LEFT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_RIGHT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_485:
			port->port.lmode_fn = pci_cti485_bl;
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_232_NS:
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17XX5X;
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_8:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_12:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_16:
			port->port.flags |= UPF_TYPE_PREDEF;
			port->port.type = PORT_XR17V35X;
			cti_get_35x_port_flags_from_eeprom(priv->dev, board, &port->port, idx);
			break;
		case PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_8:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_XEG001:
			port->port.lmode_fn = pci_cti485_tn;
		default:
			break;
	}
	switch( priv->dev->device ) {
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16:
			port->port.lmode_fn = pci_cti485_bl;
                        port->port.flags |= UPF_TYPE_PREDEF;
                        port->port.type = PORT_XR17XX5X;
			{
				u16 val;
				pci_read_config_word(priv->dev, 0x48, &val);
				pci_write_config_word(priv->dev, 0x48, val|0x8000);
			}
			//RS-485 gate needs to be enabled; otherwise RTS/CTS won't work
			{
				unsigned long start;
				unsigned char *mem_base;

				start = pci_resource_start(priv->dev, 0);
				mem_base = ioremap_nocache(start + 0x2008, 1);
				*mem_base  = 1;
				iounmap(mem_base);
			}
			break;
		default:
			break;
	}
	/* set the base baud rate */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_1_1:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_SP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_6_2_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_6_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_232_NS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_LEFT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_RIGHT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_232:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_232_NS:
			port->port.uartclk = cti_get_clock(priv->dev, board);
			break;
		default:
			break;
	}
	switch (priv->dev->device) {
		case PCI_DEVICE_ID_EXAR_XR17V352:
		case PCI_DEVICE_ID_EXAR_XR17V354:
		case PCI_DEVICE_ID_EXAR_XR17V358:
			port->port.uartclk = 125000000;
			break;
		case PCI_DEVICE_ID_EXAR_XR17V4354:
		case PCI_DEVICE_ID_EXAR_XR17V8354:
			port->port.uartclk = ( ( (idx+1) > 4 ) ? 62500000 : 125000000 );
			break;
		case PCI_DEVICE_ID_EXAR_XR17V4358:
		case PCI_DEVICE_ID_EXAR_XR17V8358:
			port->port.uartclk = ( ( (idx+1) > 8 ) ? 62500000 : 125000000 );
			break;
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16:
			port->port.uartclk = 33333333;
                        break;
		default:
			break;
	}

	/* set the PTM handling function */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_PTM:
			port->port.ptm_fn = pci_ctiptm;
			break;
		default:
			break;
	}

	/* setup Device Config Registers (DCR) */
	switch (priv->dev->device) {
		case PCI_DEVICE_ID_EXAR_XR17C152:
		case PCI_DEVICE_ID_EXAR_XR17C154:
		case PCI_DEVICE_ID_EXAR_XR17C158:
		case PCI_DEVICE_ID_EXAR_XR17V252:
		case PCI_DEVICE_ID_EXAR_XR17V254:
		case PCI_DEVICE_ID_EXAR_XR17V258:
		case PCI_DEVICE_ID_EXAR_XR17V352:
		case PCI_DEVICE_ID_EXAR_XR17V354:
		case PCI_DEVICE_ID_EXAR_XR17V4354:
		case PCI_DEVICE_ID_EXAR_XR17V8354:
		case PCI_DEVICE_ID_EXAR_XR17V358:
		case PCI_DEVICE_ID_EXAR_XR17V4358:
		case PCI_DEVICE_ID_EXAR_XR17V8358:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16:
			port->port.has_dcr = 1;
			break;
		default:
			break;
	}

	/* setup fractional divisor */
	switch (priv->dev->device) {
		case PCI_DEVICE_ID_EXAR_XR17V252:
		case PCI_DEVICE_ID_EXAR_XR17V254:
		case PCI_DEVICE_ID_EXAR_XR17V258:
		case PCI_DEVICE_ID_EXAR_XR17V352:
		case PCI_DEVICE_ID_EXAR_XR17V354:
		case PCI_DEVICE_ID_EXAR_XR17V4354:
		case PCI_DEVICE_ID_EXAR_XR17V8354:
		case PCI_DEVICE_ID_EXAR_XR17V358:
		case PCI_DEVICE_ID_EXAR_XR17V4358:
		case PCI_DEVICE_ID_EXAR_XR17V8358:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X:
		case PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16:
			port->port.use_frac_div = 1;
			break;
		default:
			break;
	}

	/* setup PLX fix for PCIe cards */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_B:
			port->port.use_plx_fix = 1;
			break;
		default:
			break;
	}

	/* setup 485 Tri-state */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_1_1:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_SP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_B:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_A:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_B:
			port->port.use_485_3t = 1;
			break;
		default:
			break;
	}
	/* setup 485 Tri-state via MPIO */
	switch (priv->dev->subsystem_device) {
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_485:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_6_2_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_6_SP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_LEFT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_RIGHT:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XP_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_XPRS_OPTO:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP:
		case PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_485:
			port->port.use_485_3t_mpio = 1;
			break;
		default:
			break;
	}

	return pci_default_setup(priv, board, port, idx);
}


#define PCI_VENDOR_ID_SBSMODULARIO	0x124B
#define PCI_SUBVENDOR_ID_SBSMODULARIO	0x124B
#define PCI_DEVICE_ID_OCTPRO		0x0001
#define PCI_SUBDEVICE_ID_OCTPRO232	0x0108
#define PCI_SUBDEVICE_ID_OCTPRO422	0x0208
#define PCI_SUBDEVICE_ID_POCTAL232	0x0308
#define PCI_SUBDEVICE_ID_POCTAL422	0x0408
#define PCI_SUBDEVICE_ID_SIIG_DUAL_00	0x2500
#define PCI_SUBDEVICE_ID_SIIG_DUAL_30	0x2530
#define PCI_VENDOR_ID_ADVANTECH		0x13fe
#define PCI_DEVICE_ID_INTEL_CE4100_UART 0x2e66
#define PCI_DEVICE_ID_ADVANTECH_PCI3620	0x3620
#define PCI_DEVICE_ID_TITAN_200I	0x8028
#define PCI_DEVICE_ID_TITAN_400I	0x8048
#define PCI_DEVICE_ID_TITAN_800I	0x8088
#define PCI_DEVICE_ID_TITAN_800EH	0xA007
#define PCI_DEVICE_ID_TITAN_800EHB	0xA008
#define PCI_DEVICE_ID_TITAN_400EH	0xA009
#define PCI_DEVICE_ID_TITAN_100E	0xA010
#define PCI_DEVICE_ID_TITAN_200E	0xA012
#define PCI_DEVICE_ID_TITAN_400E	0xA013
#define PCI_DEVICE_ID_TITAN_800E	0xA014
#define PCI_DEVICE_ID_TITAN_200EI	0xA016
#define PCI_DEVICE_ID_TITAN_200EISI	0xA017
#define PCI_DEVICE_ID_TITAN_400V3	0xA310
#define PCI_DEVICE_ID_TITAN_410V3	0xA312
#define PCI_DEVICE_ID_TITAN_800V3	0xA314
#define PCI_DEVICE_ID_TITAN_800V3B	0xA315
#define PCI_DEVICE_ID_OXSEMI_16PCI958	0x9538
#define PCIE_DEVICE_ID_NEO_2_OX_IBM	0x00F6
#define PCI_DEVICE_ID_PLX_CRONYX_OMEGA	0xc001
#define PCI_DEVICE_ID_INTEL_PATSBURG_KT 0x1d3d
#define PCI_VENDOR_ID_WCH		0x4348
#define PCI_DEVICE_ID_WCH_CH352_2S	0x3253
#define PCI_DEVICE_ID_WCH_CH353_4S	0x3453
#define PCI_DEVICE_ID_WCH_CH353_2S1PF	0x5046
#define PCI_DEVICE_ID_WCH_CH353_2S1P	0x7053
#define PCI_VENDOR_ID_AGESTAR		0x5372
#define PCI_DEVICE_ID_AGESTAR_9375	0x6872
#define PCI_VENDOR_ID_ASIX		0x9710
#define PCI_DEVICE_ID_COMMTECH_4224PCIE	0x0020
#define PCI_DEVICE_ID_COMMTECH_4228PCIE	0x0021
#define PCI_DEVICE_ID_COMMTECH_4222PCIE	0x0022
#define PCI_DEVICE_ID_BROADCOM_TRUMANAGE 0x160a

#define PCI_VENDOR_ID_SUNIX		0x1fd4
#define PCI_DEVICE_ID_SUNIX_1999	0x1999


/* Unknown vendors/cards - this should not be in linux/pci_ids.h */
#define PCI_SUBDEVICE_ID_UNKNOWN_0x1584	0x1584
#define PCI_SUBDEVICE_ID_UNKNOWN_0x1588	0x1588

/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] __refdata = {
	/*
	 * Connect Tech cards
	 */
	{
		.vendor		= PCI_VENDOR_ID_V3,
		.device		= PCI_DEVICE_ID_V3_V351,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17C152,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17C154,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17C158,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V258,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V352,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V354,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V4354,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V8354,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V358,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V4358,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_EXAR,
		.device		= PCI_DEVICE_ID_EXAR_XR17V8358,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_OXSEMI,
		.device		= PCI_DEVICE_ID_OXSEMI_16PCI954,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_OXSEMI,
		.device		= PCI_DEVICE_ID_OXSEMI_16PCEI958,
		.subvendor	= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},

	{
		.vendor		= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.device		= PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.device		= PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	{
		.vendor		= PCI_SUBVENDOR_ID_CONNECT_TECH,
		.device		= PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_cti_setup,
	},
	/*
         * Default "match everything" terminator entry
         */
        {
                .vendor         = PCI_ANY_ID,
                .device         = PCI_ANY_ID,
                .subvendor      = PCI_ANY_ID,
                .subdevice      = PCI_ANY_ID,
                .setup          = pci_default_setup,
        }
};

static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device) &&
		    quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
		    quirk_id_matches(quirk->subdevice, dev->subsystem_device))
			break;
	return quirk;
}

static inline int get_pci_irq(struct pci_dev *dev,
				const struct pciserial_board *board)
{
	if (board->flags & FL_NOIRQ)
		return 0;
	else
		return dev->irq;
}

/*
 * This is the configuration table for all of the PCI serial boards
 * which we support.  It is directly indexed by the pci_board_num_t enum
 * value, which is encoded in the pci_device_id PCI probe table's
 * driver_data member.
 *
 * The makeup of these names are:
 *  pbn_bn{_bt}_n_baud{_offsetinhex}
 *
 *  bn		= PCI BAR number
 *  bt		= Index using PCI BARs
 *  n		= number of serial ports
 *  baud	= baud rate
 *  offsetinhex	= offset for each sequential port (in hex)
 *
 * This table is sorted by (in order): bn, bt, baud, offsetindex, n.
 *
 * Please note: in theory if n = 1, _bt infix should make no difference.
 * ie, pbn_b0_1_115200 is the same as pbn_b0_bt_1_115200
 */
enum pci_board_num_t {
	pbn_default = 0,

	pbn_b0_1_115200,
	pbn_b0_2_115200,
	pbn_b0_4_115200,
	pbn_b0_5_115200,
	pbn_b0_8_115200,

	pbn_b0_1_921600,
	pbn_b0_2_921600,
	pbn_b0_4_921600,

	pbn_b0_2_1130000,

	pbn_b0_4_1152000,

	pbn_b0_2_1843200,
	pbn_b0_4_1843200,
	pbn_b0_8_1843200,

	pbn_b0_2_1843200_200,
	pbn_b0_2_1843200_200_400,
	pbn_b0_4_1843200_200,
	pbn_b0_8_1843200_200,
	pbn_b0_2_7812500_400,
	pbn_b0_4_7812500_400,
	pbn_b0_8_7812500_400,
	pbn_b0_12_7812500_400,
	pbn_b0_16_7812500_400,

	pbn_b0_8_3906250_200_1000,

	pbn_b0_1_4000000,

	pbn_b0_bt_1_115200,
	pbn_b0_bt_2_115200,
	pbn_b0_bt_8_115200,

	pbn_b0_bt_1_460800,
	pbn_b0_bt_2_460800,
	pbn_b0_bt_4_460800,

	pbn_b0_bt_1_921600,
	pbn_b0_bt_2_921600,
	pbn_b0_bt_4_921600,
	pbn_b0_bt_8_921600,

	pbn_b1_1_115200,
	pbn_b1_2_115200,
	pbn_b1_4_115200,
	pbn_b1_8_115200,

	pbn_b0_12_2083333_200,
	pbn_b0_16_2083333_200,

	pbn_b1_1_921600,
	pbn_b1_2_921600,
	pbn_b1_4_921600,
	pbn_b1_8_921600,

	pbn_b1_2_1250000,

	pbn_b1_bt_1_115200,
	pbn_b1_bt_2_921600,

	pbn_b1_1_1382400,
	pbn_b1_2_1382400,
	pbn_b1_4_1382400,
	pbn_b1_8_1382400,

	pbn_b2_1_115200,
	pbn_b2_2_115200,
	pbn_b2_4_115200,
	pbn_b2_8_115200,

	pbn_b2_1_460800,
	pbn_b2_4_460800,
	pbn_b2_8_460800,
	pbn_b2_16_460800,

	pbn_b2_1_921600,
	pbn_b2_4_921600,
	pbn_b2_8_921600,

	pbn_b2_bt_1_115200,
	pbn_b2_bt_2_115200,
	pbn_b2_bt_4_115200,

	pbn_b2_bt_2_921600,
	pbn_b2_bt_4_921600,

	pbn_b3_2_115200,
	pbn_b3_4_115200,
	pbn_b3_8_115200,

	/*
	 * Board-specific versions.
	 */
	pbn_panacom,
	pbn_panacom2,
	pbn_panacom4,
	pbn_exsys_4055,
	pbn_plx_romulus,
	pbn_oxsemi,
	pbn_oxsemi_1_4000000,
	pbn_oxsemi_2_4000000,
	pbn_oxsemi_4_4000000,
	pbn_oxsemi_8_4000000,
	pbn_intel_i960,
	pbn_sgi_ioc3,
	pbn_computone_4,
	pbn_computone_6,
	pbn_computone_8,
	pbn_sbsxrsio,
	pbn_exar_XR17C152,
	pbn_exar_XR17C154,
	pbn_exar_XR17C158,
	pbn_pasemi_1682M,
};

/*
 * uart_offset - the space between channels
 * reg_shift   - describes how the UART registers are mapped
 *               to PCI memory by the card.
 * For example IER register on SBS, Inc. PMC-OctPro is located at
 * offset 0x10 from the UART base, while UART_IER is defined as 1
 * in include/linux/serial_reg.h,
 * see first lines of serial_in() and serial_out() in 8250.c
*/

static struct pciserial_board pci_boards[]  = {
	[pbn_default] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_1_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_2_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_4_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_5_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 5,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_8_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_1_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_2_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_4_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b0_2_1130000] = {
		.flags          = FL_BASE0,
		.num_ports      = 2,
		.base_baud      = 1130000,
		.uart_offset    = 8,
	},

	[pbn_b0_4_1152000] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 1152000,
		.uart_offset	= 8,
	},

	[pbn_b0_2_1843200] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 1843200,
		.uart_offset	= 8,
	},
	[pbn_b0_4_1843200] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 1843200,
		.uart_offset	= 8,
	},
	[pbn_b0_8_1843200] = {
                .flags          = FL_BASE0,
                .num_ports      = 8,
                .base_baud      = 1843200,
                .uart_offset    = 8,
        },

	[pbn_b0_2_1843200_200] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 1843200,
		.uart_offset	= 0x200,
	},
	[pbn_b0_2_1843200_200_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 1843200,
		.uart_offset	= 0x200,
		.first_offset	= 0x400,
	},
	[pbn_b0_4_1843200_200] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 1843200,
		.uart_offset	= 0x200,
	},
	[pbn_b0_8_1843200_200] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 1843200,
		.uart_offset	= 0x200,
	},
	[pbn_b0_2_7812500_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 7812500,
		.uart_offset	= 0x400,
	},
	[pbn_b0_4_7812500_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 7812500,
		.uart_offset	= 0x400,
	},
	[pbn_b0_8_7812500_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 7812500,
		.uart_offset	= 0x400,
	},
	[pbn_b0_12_7812500_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 12,
		.base_baud	= 7812500,
		.uart_offset	= 0x400,
	},
	[pbn_b0_16_7812500_400] = {
		.flags		= FL_BASE0,
		.num_ports	= 16,
		.base_baud	= 7812500,
		.uart_offset	= 0x400,
	},
	[pbn_b0_8_3906250_200_1000] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 3906250,
		.uart_offset	= 0x200,
		.first_offset	= 0x1000,
	},
	[pbn_b0_12_2083333_200] {
		.flags		= FL_BASE0,
		.num_ports	= 12,
		.base_baud	= 2083333,
		.uart_offset	= 0x200,
	},
	[pbn_b0_16_2083333_200] {
		.flags          = FL_BASE0,
		.num_ports      = 16,
		.base_baud	= 2083333,
		.uart_offset    = 0x200,
	},
	[pbn_b0_1_4000000] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 4000000,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_1_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_2_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_4_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_8_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b1_1_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_2_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_4_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_8_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_2_1250000] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 1250000,
		.uart_offset	= 8,
	},

	[pbn_b1_bt_1_115200] = {
		.flags		= FL_BASE1|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b1_bt_2_921600] = {
		.flags		= FL_BASE1|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_1_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_2_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_4_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_8_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},

	[pbn_b2_1_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_2_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_4_115200] = {
		.flags          = FL_BASE2,
		.num_ports      = 4,
		.base_baud      = 115200,
		.uart_offset    = 8,
	},
	[pbn_b2_8_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_1_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_4_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_8_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_16_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 16,
		.base_baud	= 460800,
		.uart_offset	= 8,
	 },

	[pbn_b2_1_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_4_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_8_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_1_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_2_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_2_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b3_2_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b3_4_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b3_8_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	/*
	 * Entries following this are board-specific.
	 */

	/*
	 * Panacom - IOMEM
	 */
	[pbn_panacom] = {
		.flags		= FL_BASE2,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 0x400,
		.reg_shift	= 7,
	},
	[pbn_panacom2] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 0x400,
		.reg_shift	= 7,
	},
	[pbn_panacom4] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 0x400,
		.reg_shift	= 7,
	},

	[pbn_exsys_4055] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	/* I think this entry is broken - the first_offset looks wrong --rmk */
	[pbn_plx_romulus] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8 << 2,
		.reg_shift	= 2,
		.first_offset	= 0x03,
	},

	/*
	 * This board uses the size of PCI Base region 0 to
	 * signal now many ports are available
	 */
	[pbn_oxsemi] = {
		.flags		= FL_BASE0|FL_REGION_SZ_CAP,
		.num_ports	= 32,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_oxsemi_1_4000000] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 3906250,
		.uart_offset	= 0x200,
		.first_offset	= 0x1000,
	},
	[pbn_oxsemi_2_4000000] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 3906250,
		.uart_offset	= 0x200,
		.first_offset	= 0x1000,
	},
	[pbn_oxsemi_4_4000000] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 3906250,
		.uart_offset	= 0x200,
		.first_offset	= 0x1000,
	},
	[pbn_oxsemi_8_4000000] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 3906250,
		.uart_offset	= 0x200,
		.first_offset	= 0x1000,
	},


	/*
	 * EKF addition for i960 Boards form EKF with serial port.
	 * Max 256 ports.
	 */
	[pbn_intel_i960] = {
		.flags		= FL_BASE0,
		.num_ports	= 32,
		.base_baud	= 921600,
		.uart_offset	= 8 << 2,
		.reg_shift	= 2,
		.first_offset	= 0x10000,
	},
	[pbn_sgi_ioc3] = {
		.flags		= FL_BASE0|FL_NOIRQ,
		.num_ports	= 1,
		.base_baud	= 458333,
		.uart_offset	= 8,
		.reg_shift	= 0,
		.first_offset	= 0x20178,
	},

	/*
	 * Computone - uses IOMEM.
	 */
	[pbn_computone_4] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 0x40,
		.reg_shift	= 2,
		.first_offset	= 0x200,
	},
	[pbn_computone_6] = {
		.flags		= FL_BASE0,
		.num_ports	= 6,
		.base_baud	= 921600,
		.uart_offset	= 0x40,
		.reg_shift	= 2,
		.first_offset	= 0x200,
	},
	[pbn_computone_8] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 0x40,
		.reg_shift	= 2,
		.first_offset	= 0x200,
	},
	[pbn_sbsxrsio] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 460800,
		.uart_offset	= 256,
		.reg_shift	= 4,
	},
	/*
	 * Exar Corp. XR17C15[248] Dual/Quad/Octal UART
	 *  Only basic 16550A support.
	 *  XR17C15[24] are not tested, but they should work.
	 */
	[pbn_exar_XR17C152] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
	},
	[pbn_exar_XR17C154] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
	},
	[pbn_exar_XR17C158] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 0x200,
	},
	/*
	 * PA Semi PWRficient PA6T-1682M on-chip UART
	 */
	[pbn_pasemi_1682M] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 8333333,
	},
};

static const struct pci_device_id blacklist[] = {
	/* softmodems */
	{ PCI_VDEVICE(AL, 0x5457), }, /* ALi Corporation M5457 AC'97 Modem */
	{ PCI_VDEVICE(MOTOROLA, 0x3052), }, /* Motorola Si3052-based modem */
	{ PCI_DEVICE(0x1543, 0x3052), }, /* Si3052-based modem, default IDs */

	/* multi-io cards handled by parport_serial */
	{ PCI_DEVICE(0x4348, 0x7053), }, /* WCH CH353 2S1P */
};

/*
 * Given a complete unknown PCI device, try to use some heuristics to
 * guess what the configuration might be, based on the pitiful PCI
 * serial specs.  Returns 0 on success, 1 on failure.
 */
static int
serial_pci_guess_board(struct pci_dev *dev, struct pciserial_board *board)
{
	const struct pci_device_id *bldev;
	int num_iomem, num_port, first_port = -1, i;

	/*
	 * If it is not a communications device or the programming
	 * interface is greater than 6, give up.
	 *
	 * (Should we try to make guesses for multiport serial devices
	 * later?)
	 */
	if ((((dev->class >> 8) != PCI_CLASS_COMMUNICATION_SERIAL) &&
	     ((dev->class >> 8) != PCI_CLASS_COMMUNICATION_MODEM)) ||
	    (dev->class & 0xff) > 6)
		return -ENODEV;

	/*
	 * Do not access blacklisted devices that are known not to
	 * feature serial ports or are handled by other modules.
	 */
	for (bldev = blacklist;
	     bldev < blacklist + ARRAY_SIZE(blacklist);
	     bldev++) {
		if (dev->vendor == bldev->vendor &&
		    dev->device == bldev->device)
			return -ENODEV;
	}

	num_iomem = num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
		if (pci_resource_flags(dev, i) & IORESOURCE_MEM)
			num_iomem++;
	}

	/*
	 * If there is 1 or 0 iomem regions, and exactly one port,
	 * use it.  We guess the number of ports based on the IO
	 * region size.
	 */
	if (num_iomem <= 1 && num_port == 1) {
		board->flags = first_port;
		board->num_ports = pci_resource_len(dev, first_port) / 8;
		return 0;
	}

	/*
	 * Now guess if we've got a board which indexes by BARs.
	 * Each IO BAR should be 8 bytes, and they should follow
	 * consecutively.
	 */
	first_port = -1;
	num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO &&
		    pci_resource_len(dev, i) == 8 &&
		    (first_port == -1 || (first_port + num_port) == i)) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
	}

	if (num_port > 1) {
		board->flags = first_port | FL_BASE_BARS;
		board->num_ports = num_port;
		return 0;
	}

	return -ENODEV;
}

static inline int
serial_pci_matches(const struct pciserial_board *board,
		   const struct pciserial_board *guessed)
{
	return
	    board->num_ports == guessed->num_ports &&
	    board->base_baud == guessed->base_baud &&
	    board->uart_offset == guessed->uart_offset &&
	    board->reg_shift == guessed->reg_shift &&
	    board->first_offset == guessed->first_offset;
}

struct serial_private *
cti_pciserial_init_ports(struct pci_dev *dev,  struct pciserial_board *board)
{
	struct uart_8250_port uart;
	struct serial_private *priv;
	struct pci_serial_quirk *quirk;
	int rc, nr_ports, i, curr_index;
	unsigned char prev_DVID, curr_DVID;

    prev_DVID = 0;
    /*
	 * Find number of ports on board
	 */
	if (dev->vendor == PCI_VENDOR_ID_OXSEMI)
		pci_oxsemi_tornado_init(dev, board);

	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0) {
			priv = ERR_PTR(rc);
			goto err_out;
		}
		if (rc)
			nr_ports = rc;
	}

	priv = kzalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto err_deinit;
	}

	priv->dev = dev;
	priv->quirk = quirk;

	memset(&uart, 0, sizeof(uart));
	uart.port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
	uart.port.uartclk = board->base_baud * 16;
	uart.port.irq = get_pci_irq(dev, board);
	uart.port.dev = &dev->dev;
	for (i = curr_index = 0; i < nr_ports; i++, curr_index++) {
		if (quirk->setup(priv, board, &uart, i))
			break;
		uart.port.index_on_board = i;
		uart.port.index_on_dev   = uart.port.index_on_board;
		if( ( ( uart.port.pci_dev_id & 0xFF00 ) == 0x1100 ) ||
		    ( ( uart.port.pci_dev_id & 0x0FF0 ) == 0x0350 ) ) {
			// get the number of ports in this UART chip
			curr_DVID = ( readb( uart.port.membase + 0x80 + 0xD ) & 0x0F );
			// is the current index greater than the previous density?
			if( curr_index >= prev_DVID ) {
				// reset the current index
				curr_index = 0;
				// current density is the new previous density
				prev_DVID = curr_DVID;
			}
			uart.port.index_on_dev = curr_index;
		}

#ifdef SERIAL_DEBUG_PCI
		printk(KERN_DEBUG "Setup PCI port: port %lx, irq %d, type %d\n",
		       uart.port.iobase, uart.port.irq, uart.port.iotype);
#endif
		priv->line[i] = cti_serial8250_register_8250_port(&uart);
		if (priv->line[i] < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), priv->line[i]);
			break;
		}
	}
	priv->nr = i;
	return priv;

err_deinit:
	if (quirk->exit)
		quirk->exit(dev);
err_out:
	return priv;
}
EXPORT_SYMBOL_GPL(cti_pciserial_init_ports);

void cti_pciserial_remove_ports(struct serial_private *priv)
{
	struct pci_serial_quirk *quirk;
	int i;

	for (i = 0; i < priv->nr; i++)
		cti_serial8250_unregister_port(priv->line[i]);

	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (priv->remapped_bar[i])
			iounmap(priv->remapped_bar[i]);
		priv->remapped_bar[i] = NULL;
	}

	/*
	 * Find the exit quirks.
	 */
	quirk = find_quirk(priv->dev);
	if (quirk->exit)
		quirk->exit(priv->dev);

	kfree(priv);
}
EXPORT_SYMBOL_GPL(cti_pciserial_remove_ports);

void cti_pciserial_suspend_ports(struct serial_private *priv)
{
	int i;

	for (i = 0; i < priv->nr; i++)
		if (priv->line[i] >= 0)
			cti_serial8250_suspend_port(priv->line[i]);

	/*
	 * Ensure that every init quirk is properly torn down
	 */
	if (priv->quirk->exit)
		priv->quirk->exit(priv->dev);
}
EXPORT_SYMBOL_GPL(cti_pciserial_suspend_ports);

void cti_pciserial_resume_ports(struct serial_private *priv)
{
	int i;

	/*
	 * Ensure that the board is correctly configured.
	 */
	if (priv->quirk->init)
		priv->quirk->init(priv->dev);

	for (i = 0; i < priv->nr; i++)
		if (priv->line[i] >= 0)
			cti_serial8250_resume_port(priv->line[i]);
}
EXPORT_SYMBOL_GPL(cti_pciserial_resume_ports);

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int
cti_pciserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{

	struct serial_private *priv;
	struct pciserial_board *board;
	struct pciserial_board tmp;
	int rc;

	if (ent->driver_data >= ARRAY_SIZE(pci_boards)) {
		printk(KERN_ERR "pci_init_one: invalid driver_data: %ld\n",
			ent->driver_data);
		return -EINVAL;
	}

	board = &pci_boards[ent->driver_data];

	rc = pci_enable_device(dev);
	pci_save_state(dev);
	if (rc)
		return rc;

	if (ent->driver_data == pbn_default) {
		/*
		 * Use a copy of the pci_board entry for this;
		 * avoid changing entries in the table.
		 */
		memcpy(&tmp, board, sizeof(struct pciserial_board));
		board = &tmp;

		/*
		 * We matched one of our class entries.  Try to
		 * determine the parameters of this board.
		 */
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc)
			goto disable;
	} else {
		/*
		 * We matched an explicit entry.  If we are able to
		 * detect this boards settings with our heuristic,
		 * then we no longer need this entry.
		 */
		memcpy(&tmp, &pci_boards[pbn_default],
		       sizeof(struct pciserial_board));
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc == 0 && serial_pci_matches(board, &tmp))
			moan_device("Redundant entry in serial pci_table.",
				    dev);
	}

	priv = cti_pciserial_init_ports(dev, board);
	if (!IS_ERR(priv)) {
		pci_set_drvdata(dev, priv);
		return 0;
	}

	rc = PTR_ERR(priv);

 disable:
	pci_disable_device(dev);
	return rc;
}


static void cti_pciserial_remove_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);

	cti_pciserial_remove_ports(priv);

	pci_disable_device(dev);
}

static void cti_pciserial_remove(void)
{
	int i;

	for (i=0; i<num_cti_pci_uart_board; i++)
		cti_pciserial_remove_one(pci_pdev[i]);
}

#ifdef CONFIG_PM
static int pciserial_suspend_one(struct pci_dev *dev, pm_message_t state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv)
		cti_pciserial_suspend_ports(priv);

	pci_save_state(dev);
	pci_set_power_state(dev, pci_choose_state(dev, state));
	return 0;
}

static int pciserial_resume_one(struct pci_dev *dev)
{
	int err;
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);

	if (priv) {
		/*
		 * The device may have been disabled.  Re-enable it.
		 */
		err = pci_enable_device(dev);
		/* FIXME: We cannot simply error out here */
		if (err)
			printk(KERN_ERR "pciserial: Unable to re-enable ports, trying to continue.\n");
		cti_pciserial_resume_ports(priv);
	}
	return 0;
}
#endif

static struct pci_device_id cti_serial_pci_tbl[] = {
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V960,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_232, 0, 0,
		pbn_b1_8_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V960,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_232, 0, 0,
		pbn_b1_4_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V960,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_232, 0, 0,
		pbn_b1_2_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_232, 0, 0,
		pbn_b1_8_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_232, 0, 0,
		pbn_b1_4_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_232, 0, 0,
		pbn_b1_2_1382400 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_PTM, 0, 0,
		pbn_b1_2_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485, 0, 0,
		pbn_b1_8_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485_4_4, 0, 0,
		pbn_b1_8_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_485, 0, 0,
		pbn_b1_4_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH4_485_2_2, 0, 0,
		pbn_b1_4_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_485, 0, 0,
		pbn_b1_2_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH8_485_2_6, 0, 0,
		pbn_b1_8_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH081101V1, 0, 0,
		pbn_b1_8_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH041101V1, 0, 0,
		pbn_b1_4_921600 },
	{	PCI_VENDOR_ID_V3, PCI_DEVICE_ID_V3_V351,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_BH2_20MHZ, 0, 0,
		pbn_b1_2_1250000 },
	{	PCI_VENDOR_ID_OXSEMI, PCI_DEVICE_ID_OXSEMI_16PCI954,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_2, 0, 0,
		pbn_b0_2_1843200 },
	{	PCI_VENDOR_ID_OXSEMI, PCI_DEVICE_ID_OXSEMI_16PCI954,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_4, 0, 0,
		pbn_b0_4_1843200 },
	{	PCI_VENDOR_ID_OXSEMI, PCI_DEVICE_ID_OXSEMI_16PCI954,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_TITAN_8, 0, 0,
		pbn_b0_8_1843200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_232, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_232, 0, 0,
		pbn_b0_4_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_232, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_1_1, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_2, 0, 0,
		pbn_b0_4_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4, 0, 0,
		pbn_b0_4_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_485, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_485, 0, 0,
		pbn_b0_4_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_485, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_SP_OPTO, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_A, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_SP_OPTO_B, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_A, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_B, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_A, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_16_XPRS_B, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XPRS_OPTO, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_A, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C152,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XPRS_OPTO_B, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_232, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_485, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_SP, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_6_2_SP, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_6_SP, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C158,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_SP_232_NS, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_LEFT, 0, 0,
		pbn_b0_2_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_2_XP_OPTO_RIGHT, 0, 0,
		pbn_b0_2_1843200_200_400 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17C154,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_XP_OPTO, 0, 0,
		pbn_b0_4_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_4_4_XPRS_OPTO, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_232, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_485, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_DEVICE_ID_EXAR_XR17V258,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCI_UART_8_XPRS_LP_232_NS, 0, 0,
		pbn_b0_8_1843200_200 },
	{	PCI_VENDOR_ID_EXAR, PCI_ANY_ID,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_2, 0, 0,
		pbn_b0_2_7812500_400 },
	{	PCI_VENDOR_ID_EXAR, PCI_ANY_ID,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_4, 0, 0,
		pbn_b0_4_7812500_400 },
	{	PCI_VENDOR_ID_EXAR, PCI_ANY_ID,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_8, 0, 0,
		pbn_b0_8_7812500_400 },
	{	PCI_VENDOR_ID_EXAR, PCI_ANY_ID,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_12, 0, 0,
		pbn_b0_12_7812500_400 },
	{	PCI_VENDOR_ID_EXAR, PCI_ANY_ID,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_PCIE_XR35X_16, 0, 0,
		pbn_b0_16_7812500_400 },
	{	PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG00X,
		PCI_ANY_ID,
		PCI_ANY_ID, 0, 0,
		pbn_b0_12_2083333_200 },
	{	PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_12_XIG01X,
		PCI_ANY_ID,
		PCI_ANY_ID, 0, 0,
		pbn_b0_12_2083333_200 },
	{	PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_DEVICE_ID_CONNECT_TECH_PCI_XR79X_16,
		PCI_ANY_ID,
		PCI_ANY_ID, 0, 0,
		pbn_b0_16_2083333_200 },
	{	PCI_VENDOR_ID_OXSEMI, PCI_DEVICE_ID_OXSEMI_16PCEI958,
		PCI_SUBVENDOR_ID_CONNECT_TECH,
		PCI_SUBDEVICE_ID_CONNECT_TECH_XEG001, 0, 0,
		pbn_b0_8_3906250_200_1000 },
	{ 0, }
};

static pci_ers_result_t serial8250_io_error_detected(struct pci_dev *dev,
						pci_channel_state_t state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (priv)
		cti_pciserial_suspend_ports(priv);

	pci_disable_device(dev);

	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t serial8250_io_slot_reset(struct pci_dev *dev)
{
	int rc;

	rc = pci_enable_device(dev);

	if (rc)
		return PCI_ERS_RESULT_DISCONNECT;

	pci_restore_state(dev);
	pci_save_state(dev);

	return PCI_ERS_RESULT_RECOVERED;
}

static void serial8250_io_resume(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv)
		cti_pciserial_resume_ports(priv);
}

static const struct pci_error_handlers serial8250_err_handler = {
	.error_detected = serial8250_io_error_detected,
	.slot_reset = serial8250_io_slot_reset,
	.resume = serial8250_io_resume,
};

//static struct pci_driver serial_pci_driver = {
//	.name		= "serial",
//	.probe		= pciserial_init_one,
//	.remove		= pciserial_remove_one,
//#ifdef CONFIG_PM
//	.suspend	= pciserial_suspend_one,
//	.resume		= pciserial_resume_one,
//#endif
//	//.id_table	= serial_pci_tbl,
//	.err_handler	= &serial8250_err_handler,
//};
//
//module_pci_driver(serial_pci_driver);

static void find_cti_pci_uart(void)
{
	int		b;
	struct pci_dev	*pdev=NULL;
	int n = (sizeof(cti_serial_pci_tbl) / sizeof(struct pci_device_id)) - 1;
	b = 0;

	while (b < n) {
		pdev = pci_get_subsys(cti_serial_pci_tbl[b].vendor,cti_serial_pci_tbl[b].device, cti_serial_pci_tbl[b].subvendor, cti_serial_pci_tbl[b].subdevice, pdev);
		if(pdev==NULL){
			b++;
			continue;
		}
		printk(KERN_INFO "CTIserial: Found CTI board BusNo=%d, DevNo=%d\n",pdev->bus->number, PCI_SLOT(pdev->devfn));
		num_cti_pci_uart_board++;
		if ( num_cti_pci_uart_board >= MAX_CTI_PCI_BOARDS) {
			printk(KERN_INFO "CTIserial: Too many CTI family boards found, maximum allowed %d\n",MAX_CTI_PCI_BOARDS);
			num_cti_pci_uart_board--;
			return;
		}
		else {
			pci_pdev[num_cti_pci_uart_board - 1] = pdev;
			cti_pciserial_init_one(pdev, &cti_serial_pci_tbl[b] );
		}
	}
}

//static int
//cti_pciserial_init_one2(struct pci_dev *dev, const struct pci_device_id *ent)
//{
//    // dummy
//    return 0;
//
//}

static struct pci_driver cti_serial_pci_driver = {
	.name		= "ctipciuart",
	//.probe		= cti_pciserial_init_one2,
	//.remove		= __devexit_p(cti_pciserial_remove_one),
#ifdef CONFIG_PM
	.suspend	= pciserial_suspend_one,
	.resume		= pciserial_resume_one,
#endif
	//.id_table	= cti_serial_pci_tbl,
};

static int __init cti_serial8250_pci_init(void)
{
	find_cti_pci_uart();
	return pci_register_driver(&cti_serial_pci_driver);
}

static void __exit cti_serial8250_pci_exit(void)
{
	cti_pciserial_remove();
	pci_unregister_driver(&cti_serial_pci_driver);
}

module_init(cti_serial8250_pci_init);
module_exit(cti_serial8250_pci_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Generic 8250/16x50 PCI serial probe module");
MODULE_DEVICE_TABLE(pci, cti_serial_pci_tbl);
