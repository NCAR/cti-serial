/*
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * A note about mapbase / membase
 *
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 */
/*	Copyright (c) 2013 CTI, Connect Tech Inc. All Rights Reserved.*/

#if defined(CONFIG_SERIAL_8250_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/ratelimit.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial.h> /* for serial_state and serial_icounter_struct */
//#include <linux/serial_core.h>
#include "serial_core.h"
//#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#ifdef CONFIG_SPARC
#include <linux/sunserialcore.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>

#include "8250.h"

#include "cti485.h"

#define MAX_PORTS 128

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

#define CTI_TTY_MINOR 0


void cti_serial8250_tx_chars(struct uart_8250_port *up);
void cti_uart_insert_char(struct uart_port *port, unsigned int status,
		 unsigned int overrun, unsigned int ch, unsigned int flag);
void cti_uart_write_wakeup(struct uart_port *port);
void cti_uart_handle_dcd_change(struct uart_port *uport, unsigned int status);
void cti_uart_handle_cts_change(struct uart_port *uport, unsigned int status);
unsigned int cti_uart_get_divisor(struct uart_port *port, unsigned int baud);
unsigned int cti_uart_get_baud_rate(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old, unsigned int min, unsigned int max);
void cti_uart_update_timeout(struct uart_port *port, unsigned int cflag,
		    unsigned int baud);
int cti_uart_add_one_port(struct uart_driver *drv, struct uart_port *uport);
void cti_uart_console_write(struct uart_port *port, const char *s,
			unsigned int count,
			void (*putchar)(struct uart_port *, int));
void cti_uart_parse_options(char *options, int *baud, int *parity, int *bits, int *flow);
int cti_uart_set_options(struct uart_port *port, struct console *co,
		 int baud, int parity, int bits, int flow);
int cti_uart_match_port(struct uart_port *port1, struct uart_port *port2);
int cti_uart_suspend_port(struct uart_driver *drv, struct uart_port *uport);
int cti_serial8250_register_8250_port(struct uart_8250_port *up);
void cti_serial8250_unregister_port(int line);
int cti_uart_remove_one_port(struct uart_driver *drv, struct uart_port *uport);
int cti_uart_register_driver(struct uart_driver *drv);
void cti_uart_unregister_driver(struct uart_driver *drv);
int cti_uart_resume_port(struct uart_driver *drv, struct uart_port *uport);

/*
 * Configuration:
 *   share_irqs - whether we pass IRQF_SHARED to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = 0; //SERIAL8250_SHARE_IRQS;

static unsigned int disable_msr_int = 0;

static unsigned int nr_uarts = 16; //CONFIG_SERIAL_8250_RUNTIME_UARTS;

static struct uart_driver serial8250_reg;

static int serial_index(struct uart_port *port)
{
	return (serial8250_reg.minor - CTI_TTY_MINOR) + port->line;
}

static unsigned int skip_txen_test; /* force skip of txen test at init time */

/*
 * Debugging.
 */
#if 0
#define DEBUG_AUTOCONF(fmt...)	printk(fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	1024

#define BOTH_EMPTY 	(UART_LSR_TEMT | UART_LSR_THRE)

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

#ifdef CONFIG_SERIAL_8250_DETECT_IRQ
#define CONFIG_SERIAL_DETECT_IRQ 1
#endif
#ifdef CONFIG_SERIAL_8250_MANY_PORTS
#define CONFIG_SERIAL_MANY_PORTS 1
#endif

/*
 * HUB6 is always on.  This will be removed once the header
 * files have been cleaned.
 */
#define CONFIG_HUB6 1

#include <asm/serial.h>
/*
 * SERIAL_PORT_DFNS tells us about built-in ports that have no
 * standard enumeration mechanism.   Platforms that can find all
 * serial ports via mechanisms like ACPI or PCI need not supply it.
 */
#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

/*struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	unsigned int flags;
	unsigned char hub6;
	unsigned char io_type;
	unsigned char *iomem_base;
	unsigned short iomem_reg_shift;
	unsigned long irqflags;
};*/

static const struct old_serial_port old_serial_port;

#define UART_NR	CONFIG_SERIAL_8250_NR_UARTS

#ifdef CONFIG_SERIAL_8250_RSA


#define PORT_RSA_MAX 4
static unsigned long probe_rsa[PORT_RSA_MAX];
static unsigned int probe_rsa_count;
#endif /* CONFIG_SERIAL_8250_RSA  */

struct irq_info {
	struct			hlist_node node;
	int			irq;
	spinlock_t		lock;	/* Protects list not the hash */
	struct list_head	*head;
};

#define NR_IRQ_HASH		32	/* Can be adjusted later */
static struct hlist_head irq_lists[NR_IRQ_HASH];
static DEFINE_MUTEX(hash_mutex);	/* Used to walk the hash */

#define SA485(softauto485, bit)	(softauto485 ? 0 : (bit))

#ifndef UART_DLD
#define UART_DLD    2       // Fractional Divisor
#endif  //UART_DLD

#ifndef UART_FLAT_TXCNT
#define UART_FLAT_FCTR_TX_INT	0x20
#define UART_FLAT_FCTR_TRGMASK	0xC0
#define UART_FLAT_FCTR_TRGA	0x0
#define UART_FLAT_FCTR_TRGB	0x40
#define UART_FLAT_FCTR_TRGC	0x80
#define UART_FLAT_FCTR_TRGD	0xC0
#define UART_FLAT_FCTR		8
#define UART_FLAT_EFR		9
#define UART_FLAT_TXCNT 10      /* Out: Transmit FIFO Count */
#define UART_FLAT_TXTRG 10      /* In: Transmit FIFO Trigger Level */
#define UART_FLAT_RXCNT 11      /* Out: Receive FIFO Count */
#define UART_FLAT_RXTRG 11      /* In: Receive FIFO Trigger Level */
#endif // UART_FLAT_TXCNT

#ifndef UART_17XX5X_RX
#define UART_17XX5X_RX		0x100
#define UART_17XX5X_TX		0x100
#define UART_17XX5X_RXLSR	0x180
#define UART_17V35X_RXLSR	0x200
#endif // UART_17XX5X_RX

#ifndef UART_IIR_RDTI
#define UART_IIR_RDTI		0x0C /* Receiver data timeout interrupt */
#define UART_IIR_EMS		0x20 /* Extended modem status interrupt */
#endif //UART_IIR_RDTI

#ifndef UART_LSR_FIFO_ERRORS
#define UART_LSR_FIFO_ERRORS	0x80 /* At least one parity error, framing error or break indication is in the FIFO data. */
#endif // UART_LSR_FIFO_ERRORS

#ifndef UART_ACR_DTR_MASK
#define UART_ACR_DTR_MASK	0x18	/* DTR mode mask */
#define UART_ACR_DTR_485_HIGH	0x18	/* DTR 485 active high mode */
#endif // UART_ACR_DTR_MASK

/*
 * Here we define the default xmit fifo size used for each type of UART.
 */
static const struct serial8250_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unassigned",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_8250] = {
		.name		= "8250",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.name		= "16450",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.name		= "16550",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550A] = {
		.name		= "16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO,
	},
	[PORT_16650] = {
		.name		= "ST16650",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16650V2] = {
		.name		= "ST16650V2",
		.fifo_size	= 32,
		.tx_loadsz	= 16,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01 |
				  UART_FCR_T_TRIG_00,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},

	[PORT_16C950] = {
		.name		= "16C95x",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO  | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16654] = {
		.name		= "ST16654",
		.fifo_size	= 64,
		.tx_loadsz	= 32,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_01 |
				  UART_FCR_T_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},
	[PORT_16850] = {
		.name		= "XR16850",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR | UART_CAP_SLEEP,
	},

	[PORT_XR17XX5X] = {
		.name		= "XR17xx5x",
		.fifo_size	= 64,
		.tx_loadsz	= 64,
		.fcr		= UART_FCR_ENABLE_FIFO,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR,
	},
	[PORT_XR17V35X] = {
		.name		= "XR17V35x",
		.fifo_size	= 256,
		.tx_loadsz	= 256,
		.fcr		= UART_FCR_ENABLE_FIFO,
 		.flags		= UART_CAP_FIFO | UART_CAP_EFR,
 	},
};

/* sane hardware needs no mapping */
#define map_8250_in_reg(up, offset) (offset)
#define map_8250_out_reg(up, offset) (offset)

/* Uart divisor latch read */
static int default_serial_dl_read(struct uart_8250_port *up)
{
	return serial_in(up, UART_DLL) | serial_in(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static void default_serial_dl_write(struct uart_8250_port *up, int value)
{
	serial_out(up, UART_DLL, value & 0xff);
	serial_out(up, UART_DLM, value >> 8 & 0xff);
}


#if defined(CONFIG_MIPS_ALCHEMY) || defined(CONFIG_SERIAL_8250_RT288X)

/* Au1x00/RT288x UART hardware has a weird register layout */
static const u8 au_io_in_map[] = {
	[UART_RX]  = 0,
	[UART_IER] = 2,
	[UART_IIR] = 3,
	[UART_LCR] = 5,
	[UART_MCR] = 6,
	[UART_LSR] = 7,
	[UART_MSR] = 8,
};

static const u8 au_io_out_map[] = {
	[UART_TX]  = 1,
	[UART_IER] = 2,
	[UART_FCR] = 4,
	[UART_LCR] = 5,
	[UART_MCR] = 6,
};



static unsigned int au_serial_in(struct uart_port *p, int offset)
{
	offset = au_io_in_map[offset] << p->regshift;
	return __raw_readl(p->membase + offset);
}

static void au_serial_out(struct uart_port *p, int offset, int value)
{
	offset = au_io_out_map[offset] << p->regshift;
	__raw_writel(value, p->membase + offset);
}

/* Au1x00 haven't got a standard divisor latch */
static int au_serial_dl_read(struct uart_8250_port *up)
{
	return __raw_readl(up->port.membase + 0x28);
}

static void au_serial_dl_write(struct uart_8250_port *up, int value)
{
	__raw_writel(value, up->port.membase + 0x28);
}

#endif

static unsigned int hub6_serial_in(struct uart_port *p, int offset)
{
	offset = offset << p->regshift;
	outb(p->hub6 - 1 + offset, p->iobase);
	return inb(p->iobase + 1);
}

static void hub6_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	outb(p->hub6 - 1 + offset, p->iobase);
	outb(value, p->iobase + 1);
}

static unsigned int mem_serial_in(struct uart_port *p, int offset)
{
	offset = offset << p->regshift;
	return readb(p->membase + offset);
}

static void mem_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	writeb(value, p->membase + offset);
}

static void mem32_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	writel(value, p->membase + offset);
}

static unsigned int mem32_serial_in(struct uart_port *p, int offset)
{
	offset = offset << p->regshift;
	return readl(p->membase + offset);
}

static unsigned int io_serial_in(struct uart_port *p, int offset)
{
	offset = offset << p->regshift;
	return inb(p->iobase + offset);
}

static void io_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	outb(value, p->iobase + offset);
}

/* Save the LCR value so it can be re-written when a Busy Detect IRQ occurs. */
static inline void dwapb_save_out_value(struct uart_port *p, int offset,
					int value)
{
	struct uart_8250_port *up =
		container_of(p, struct uart_8250_port, port);

	if (offset == UART_LCR)
		up->lcr = value;
}

/* Read the IER to ensure any interrupt is cleared before returning from ISR. */
static inline void dwapb_check_clear_ier(struct uart_port *p, int offset)
{
	if (offset == UART_TX || offset == UART_IER)
		p->serial_in(p, UART_IER);
}

static void dwapb_serial_out(struct uart_port *p, int offset, int value)
{
	int save_offset = offset;
	offset = map_8250_out_reg(p, offset) << p->regshift;
	dwapb_save_out_value(p, save_offset, value);
	writeb(value, p->membase + offset);
	dwapb_check_clear_ier(p, save_offset);
}

static void dwapb32_serial_out(struct uart_port *p, int offset, int value)
{
	int save_offset = offset;
	offset = map_8250_out_reg(p, offset) << p->regshift;
	dwapb_save_out_value(p, save_offset, value);
	writel(value, p->membase + offset);
	dwapb_check_clear_ier(p, save_offset);
}


static int serial8250_default_handle_irq(struct uart_port *port);
//static int exar_handle_irq(struct uart_port *port);

static void set_io_from_upio(struct uart_port *p)
{
	struct uart_8250_port *up =
		container_of(p, struct uart_8250_port, port);

	up->dl_read = default_serial_dl_read;
	up->dl_write = default_serial_dl_write;
	switch (p->iotype) {
	case UPIO_HUB6:
		p->serial_in = hub6_serial_in;
		p->serial_out = hub6_serial_out;
		break;

	case UPIO_MEM:
		p->serial_in = mem_serial_in;
		p->serial_out = mem_serial_out;
		break;

	case UPIO_MEM32:
		p->serial_in = mem32_serial_in;
		p->serial_out = mem32_serial_out;
		break;

#if defined(CONFIG_MIPS_ALCHEMY) || defined(CONFIG_SERIAL_8250_RT288X)
	case UPIO_AU:
		p->serial_in = au_serial_in;
		p->serial_out = au_serial_out;
		up->dl_read = au_serial_dl_read;
		up->dl_write = au_serial_dl_write;
		break;
#endif

    case UPIO_DWAPB:
		p->serial_in = mem_serial_in;
		p->serial_out = dwapb_serial_out;
		break;

	case UPIO_DWAPB32:
		p->serial_in = mem32_serial_in;
		p->serial_out = dwapb32_serial_out;
		break;

    default:
		p->serial_in = io_serial_in;
		p->serial_out = io_serial_out;
		break;
	}
	/* Remember loaded iotype */
	up->cur_iotype = p->iotype;
	p->handle_irq = serial8250_default_handle_irq;
}

static void
serial_port_out_sync(struct uart_port *p, int offset, int value)
{
	switch (p->iotype) {
	case UPIO_MEM:
	case UPIO_MEM32:
	case UPIO_AU:
	case UPIO_DWAPB:
	case UPIO_DWAPB32:
		p->serial_out(p, offset, value);
		p->serial_in(p, UART_LCR);	/* safe, no side-effects */
		break;
	default:
		p->serial_out(p, offset, value);
	}
}

#define serial_in(up, offset)		\
	(up->port.serial_in(&(up)->port, (offset)))
#define serial_out(up, offset, value)	\
	(up->port.serial_out(&(up)->port, (offset), (value)))


#define serial_inp(up, offset)		serial_in(up, offset)
#define serial_outp(up, offset, value)	serial_out(up, offset, value)

static unsigned int
serial_dcr_in(struct uart_8250_port *up, int offset)
{
	return serial_in(up, offset + 0x80);
}

static void
serial_dcr_out(struct uart_8250_port *up, int offset, int value)
{
	return serial_out(up, offset + 0x80, value);
}

static void check_mpio(struct uart_8250_port *up){
	unsigned short val, line_mask;
	
	if (up->port.use_mpio_input){
		/*set MPIO
		MPIOSEL[0:7] = 0xff
		MPIOSEL[15:8] = 0xff
		*/
		serial_dcr_out(up, 0x13, 0xff);
		serial_dcr_out(up, 0x19, 0xff);
		/*read MPIO
		MPIOLVL[7:0]
		MPIOLVL[15:8]
		*/
		val = serial_dcr_in(up, 0x10);
		val |= (serial_dcr_in(up, 0x16)<<8) & 0xff00;
		/*only need use_clr_rts flag in RS422 line mode*/
		line_mask = 0x03 << ((up->port.index_on_dev) * (up->port.use_mpio_input));
		if ((val & line_mask) == line_mask)
			up->port.use_clr_rts = 1;
		else 
			up->port.use_clr_rts = 0;
	}
}

/*
 * For the 16C950
 */
static void serial_icr_write(struct uart_8250_port *up, int offset, int value)
{
	serial_out(up, UART_SCR, offset);
	serial_out(up, UART_ICR, value);
}

static unsigned int serial_icr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_out(up, UART_SCR, offset);
	value = serial_in(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);

	return value;
}

static void SHRE_poll(unsigned long arg)
{
	struct uart_8250_port *up = (struct uart_8250_port *)arg;

	while (!(serial_in(up, UART_LSR) & UART_LSR_TEMT));

	up->mcr &= ~UART_MCR_RTS;
	serial_out(up, UART_MCR, up->mcr);
}


/*
 * FIFO support.
 */
static void serial8250_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(p, UART_FCR, 0);
	}
}

void cti_serial8250_clear_and_reinit_fifos(struct uart_8250_port *p)
{
	unsigned char fcr;

	serial8250_clear_fifos(p);
	fcr = uart_config[p->port.type].fcr;
	serial_out(p, UART_FCR, fcr);
}
EXPORT_SYMBOL_GPL(cti_serial8250_clear_and_reinit_fifos);

/*
 * IER sleep support.  UARTs which have EFRs need the "extended
 * capability" bit enabled.  Note that on XR16C850s, we need to
 * reset LCR to write to IER.
 */
static void serial8250_set_sleep(struct uart_8250_port *p, int sleep)
{
	/*
	 * Exar UARTs have a SLEEP register that enables or disables
	 * each UART to enter sleep mode separately.  On the XR17V35x the
	 * register is accessible to each UART at the UART_EXAR_SLEEP
	 * offset but the UART channel may only write to the corresponding
	 * bit.
	 */
	/*if ((p->port.type == PORT_XR17V35X) ||
	   (p->port.type == PORT_XR17XX5X)) {
		serial_out(p, UART_EXAR_SLEEP, 0x00);
		return;
	} *//*Generating interrupts when irq handler is not registered - MG*/
	

	if (p->capabilities & UART_CAP_SLEEP) {
		if (p->capabilities & UART_CAP_EFR) {
			serial_out(p, UART_LCR, UART_LCR_CONF_MODE_B);
			serial_out(p, UART_EFR, UART_EFR_ECB);
			serial_out(p, UART_LCR, 0);
		}
		serial_out(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
		if (p->capabilities & UART_CAP_EFR) {
			serial_out(p, UART_LCR, UART_LCR_CONF_MODE_B);
			serial_out(p, UART_EFR, 0);
			serial_out(p, UART_LCR, 0);
		}
	}
}

#ifdef CONFIG_SERIAL_8250_RSA
/*
 * Attempts to turn on the RSA FIFO.  Returns zero on failure.
 * We set the port uart clock rate if we succeed.
 */
static int __enable_rsa(struct uart_8250_port *up)
{
	unsigned char mode;
	int result;

	mode = serial_in(up, UART_RSA_MSR);
	result = mode & UART_RSA_MSR_FIFO;

	if (!result) {
		serial_out(up, UART_RSA_MSR, mode | UART_RSA_MSR_FIFO);
		mode = serial_in(up, UART_RSA_MSR);
		result = mode & UART_RSA_MSR_FIFO;
	}

	if (result)
		up->port.uartclk = SERIAL_RSA_BAUD_BASE * 16;

	return result;
}

static void enable_rsa(struct uart_8250_port *up)
{
	if (up->port.type == PORT_RSA) {
		if (up->port.uartclk != SERIAL_RSA_BAUD_BASE * 16) {
			spin_lock_irq(&up->port.lock);
			__enable_rsa(up);
			spin_unlock_irq(&up->port.lock);
		}
		if (up->port.uartclk == SERIAL_RSA_BAUD_BASE * 16)
			serial_out(up, UART_RSA_FRR, 0);
	}
}

/*
 * Attempts to turn off the RSA FIFO.  Returns zero on failure.
 * It is unknown why interrupts were disabled in here.  However,
 * the caller is expected to preserve this behaviour by grabbing
 * the spinlock before calling this function.
 */
static void disable_rsa(struct uart_8250_port *up)
{
	unsigned char mode;
	int result;

	if (up->port.type == PORT_RSA &&
	    up->port.uartclk == SERIAL_RSA_BAUD_BASE * 16) {
		spin_lock_irq(&up->port.lock);

		mode = serial_in(up, UART_RSA_MSR);
		result = !(mode & UART_RSA_MSR_FIFO);

		if (!result) {
			serial_out(up, UART_RSA_MSR, mode & ~UART_RSA_MSR_FIFO);
			mode = serial_in(up, UART_RSA_MSR);
			result = !(mode & UART_RSA_MSR_FIFO);
		}

		if (result)
			up->port.uartclk = SERIAL_RSA_BAUD_BASE_LO * 16;
		spin_unlock_irq(&up->port.lock);
	}
}
#endif /* CONFIG_SERIAL_8250_RSA */

static inline void serial_dlf_read(struct uart_8250_port *up, unsigned short *dl, unsigned short *frac)
{
	*dl = serial_inp(up, UART_DLL) | serial_inp(up, UART_DLM) << 8;
	*frac = serial_inp(up, UART_DLD);
}

static inline void serial_dlf_write(struct uart_8250_port *up, int dl, int frac, unsigned char prescaler, unsigned char sample_rate)
{
	unsigned char index;

	index = up->port.index_on_dev;
	

	serial_outp(up, UART_DLL, dl & 0xff);
	serial_outp(up, UART_DLM, dl >> 8 & 0xff);
	serial_outp(up, UART_DLD, frac);

	if(sample_rate==8)
		serial_dcr_out(up, 0x8, serial_dcr_in(up, 0x8) | (0x01 << index));
	else
		serial_dcr_out(up, 0x8, serial_dcr_in(up, 0x8) & ~(0x01 << index));
	
	up->mcr = ((up->mcr & ~UART_MCR_CLKSEL) | ((prescaler == 1) ? 0 : UART_MCR_CLKSEL));
	
	serial_outp(up, UART_MCR, up->mcr);
	
}


/*
 * This is a quickie test to see how big the FIFO is.
 * It doesn't work at all the time, more's the pity.
 */
static int size_fifo(struct uart_8250_port *up)
{
	unsigned char old_fcr, old_mcr, old_lcr;
	unsigned short old_dl, old_frac = 0;
	int count;

	old_lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, 0);
	old_fcr = serial_in(up, UART_FCR);
	old_mcr = serial_in(up, UART_MCR);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
		    UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_MCR, UART_MCR_LOOP);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	if (up->port.use_frac_div) {
		serial_dlf_read(up, &old_dl, &old_frac);
		serial_dlf_write(up, 0x0001, 0x0, 1, 16);
    }
    else
    {
	    old_dl = serial_dl_read(up);
	    serial_dl_write(up, 0x0001);
    }
	serial_out(up, UART_LCR, 0x03);
	for (count = 0; count < 256; count++)
		serial_out(up, UART_TX, count);
	mdelay(20);/* FIXME - schedule_timeout */
	for (count = 0; (serial_in(up, UART_LSR) & UART_LSR_DR) &&
	     (count < 256); count++)
		serial_in(up, UART_RX);
	serial_out(up, UART_FCR, old_fcr);
	serial_out(up, UART_MCR, old_mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	if (up->port.use_frac_div)
        serial_dlf_write(up, old_dl, old_frac, 1, 16);
    else
        serial_dl_write(up, old_dl);
	serial_out(up, UART_LCR, old_lcr);

	return count;
}

/*
 * Read UART ID using the divisor method - set DLL and DLM to zero
 * and the revision will be in DLL and device type in DLM.  We
 * preserve the device state across this.
 */
static unsigned int autoconfig_read_divisor_id(struct uart_8250_port *p)
{
	unsigned char old_dll, old_dlm, old_lcr;
	unsigned int id;

	old_lcr = serial_in(p, UART_LCR);
	serial_out(p, UART_LCR, UART_LCR_CONF_MODE_A);

	old_dll = serial_in(p, UART_DLL);
	old_dlm = serial_in(p, UART_DLM);

	serial_out(p, UART_DLL, 0);
	serial_out(p, UART_DLM, 0);

	id = serial_in(p, UART_DLL) | serial_in(p, UART_DLM) << 8;

	serial_out(p, UART_DLL, old_dll);
	serial_out(p, UART_DLM, old_dlm);
	serial_out(p, UART_LCR, old_lcr);

	return id;
}

/*
 * This is a helper routine to autodetect StarTech/Exar/Oxsemi UART's.
 * When this function is called we know it is at least a StarTech
 * 16650 V2, but it might be one of several StarTech UARTs, or one of
 * its clones.  (We treat the broken original StarTech 16650 V1 as a
 * 16550, and why not?  Startech doesn't seem to even acknowledge its
 * existence.)
 *
 * What evil have men's minds wrought...
 */
static void autoconfig_has_efr(struct uart_8250_port *up)
{
	unsigned int id1, id2, id3, rev;

	/*
	 * Everything with an EFR has SLEEP
	 */
	up->capabilities |= UART_CAP_EFR | UART_CAP_SLEEP;

	/*
	 * First we check to see if it's an Oxford Semiconductor UART.
	 *
	 * If we have to do this here because some non-National
	 * Semiconductor clone chips lock up if you try writing to the
	 * LSR register (which serial_icr_read does)
	 */

	/*
	 * Check for Oxford Semiconductor 16C950.
	 *
	 * EFR [4] must be set else this test fails.
	 *
	 * This shouldn't be necessary, but Mike Hudson (Exoray@isys.ca)
	 * claims that it's needed for 952 dual UART's (which are not
	 * recommended for new designs).
	 */
	up->acr = 0;
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x00);
	id1 = serial_icr_read(up, UART_ID1);
	id2 = serial_icr_read(up, UART_ID2);
	id3 = serial_icr_read(up, UART_ID3);
	rev = serial_icr_read(up, UART_REV);

	DEBUG_AUTOCONF("950id=%02x:%02x:%02x:%02x ", id1, id2, id3, rev);

	if (id1 == 0x16 && id2 == 0xC9 &&
	    (id3 == 0x50 || id3 == 0x52 || id3 == 0x54)) {
		up->port.type = PORT_16C950;

		/*
		 * Enable work around for the Oxford Semiconductor 952 rev B
		 * chip which causes it to seriously miscalculate baud rates
		 * when DLL is 0.
		 */
		if (id3 == 0x52 && rev == 0x01)
			up->bugs |= UART_BUG_QUOT;
		return;
	}

	/*
	 * We check for a XR16C850 by setting DLL and DLM to 0, and then
	 * reading back DLL and DLM.  The chip type depends on the DLM
	 * value read back:
	 *  0x10 - XR16C850 and the DLL contains the chip revision.
	 *  0x12 - XR16C2850.
	 *  0x14 - XR16C854.
	 */
	id1 = autoconfig_read_divisor_id(up);
	DEBUG_AUTOCONF("850id=%04x ", id1);

	id2 = id1 >> 8;
	if (id2 == 0x10 || id2 == 0x12 || id2 == 0x14) {
		up->port.type = PORT_16850;
		return;
	}

	/*
	 * It wasn't an XR16C850.
	 *
	 * We distinguish between the '654 and the '650 by counting
	 * how many bytes are in the FIFO.  I'm using this for now,
	 * since that's the technique that was sent to me in the
	 * serial driver update, but I'm not convinced this works.
	 * I've had problems doing this in the past.  -TYT
	 */
	if (size_fifo(up) == 64)
		up->port.type = PORT_16654;
	else
		up->port.type = PORT_16650V2;
}

/*
 * We detected a chip without a FIFO.  Only two fall into
 * this category - the original 8250 and the 16450.  The
 * 16450 has a scratch register (accessible with LCR=0)
 */
static void autoconfig_8250(struct uart_8250_port *up)
{
	unsigned char scratch, status1, status2;

	up->port.type = PORT_8250;

	scratch = serial_in(up, UART_SCR);
	serial_out(up, UART_SCR, 0xa5);
	status1 = serial_in(up, UART_SCR);
	serial_out(up, UART_SCR, 0x5a);
	status2 = serial_in(up, UART_SCR);
	serial_out(up, UART_SCR, scratch);

	if (status1 == 0xa5 && status2 == 0x5a)
		up->port.type = PORT_16450;
}

static int broken_efr(struct uart_8250_port *up)
{
	/*
	 * Exar ST16C2550 "A2" devices incorrectly detect as
	 * having an EFR, and report an ID of 0x0201.  See
	 * http://linux.derkeiler.com/Mailing-Lists/Kernel/2004-11/4812.html 
	 */
	if (autoconfig_read_divisor_id(up) == 0x0201 && size_fifo(up) == 16)
		return 1;

	return 0;
}

static inline int ns16550a_goto_highspeed(struct uart_8250_port *up)
{
	unsigned char status;

	status = serial_in(up, 0x04); /* EXCR2 */
#define PRESL(x) ((x) & 0x30)
	if (PRESL(status) == 0x10) {
		/* already in high speed mode */
		return 0;
	} else {
		status &= ~0xB0; /* Disable LOCK, mask out PRESL[01] */
		status |= 0x10;  /* 1.625 divisor for baud_base --> 921600 */
		serial_out(up, 0x04, status);
	}
	return 1;
}

/*
 * We know that the chip has FIFOs.  Does it have an EFR?  The
 * EFR is located in the same register position as the IIR and
 * we know the top two bits of the IIR are currently set.  The
 * EFR should contain zero.  Try to read the EFR.
 */
static void autoconfig_16550a(struct uart_8250_port *up)
{
	unsigned char status1, status2;
	unsigned int iersave;

	up->port.type = PORT_16550A;
	up->capabilities |= UART_CAP_FIFO;

	/*
	 * XR17V35x UARTs have an extra divisor register, DLD
	 * that gets enabled with when DLAB is set which will
	 * cause the device to incorrectly match and assign
	 * port type to PORT_16650.  The EFR for this UART is
	 * found at offset 0x09. Instead check the Deice ID (DVID)
	 * register for a 2, 4 or 8 port UART.
	 */
	if (up->port.flags & UPF_EXAR_EFR) {
		status1 = serial_in(up, UART_EXAR_DVID);
		if (status1 == 0x82 || status1 == 0x84 || status1 == 0x88) {
			DEBUG_AUTOCONF("Exar XR17V35x ");
			up->port.type = PORT_XR17V35X;
			up->capabilities |= UART_CAP_AFE | UART_CAP_EFR |
						UART_CAP_SLEEP;

			return;
		}

	}

	/*
	 * Check for presence of the EFR when DLAB is set.
	 * Only ST16C650V1 UARTs pass this test.
	 */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	if (serial_in(up, UART_EFR) == 0) {
		serial_out(up, UART_EFR, 0xA8);
		if (serial_in(up, UART_EFR) != 0) {
			DEBUG_AUTOCONF("EFRv1 ");
			up->port.type = PORT_16650;
			up->capabilities |= UART_CAP_EFR | UART_CAP_SLEEP;
		} else {
			DEBUG_AUTOCONF("Motorola 8xxx DUART ");
		}
		serial_out(up, UART_EFR, 0);
		return;
	}

	/*
	 * Maybe it requires 0xbf to be written to the LCR.
	 * (other ST16C650V2 UARTs, TI16C752A, etc)
	 */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	if (serial_in(up, UART_EFR) == 0 && !broken_efr(up)) {
		DEBUG_AUTOCONF("EFRv2 ");
		autoconfig_has_efr(up);
		return;
	}

	/*
	 * Check for a National Semiconductor SuperIO chip.
	 * Attempt to switch to bank 2, read the value of the LOOP bit
	 * from EXCR1. Switch back to bank 0, change it in MCR. Then
	 * switch back to bank 2, read it from EXCR1 again and check
	 * it's changed. If so, set baud_base in EXCR2 to 921600. -- dwmw2
	 */
	serial_out(up, UART_LCR, 0);
	status1 = serial_in(up, UART_MCR);
	serial_out(up, UART_LCR, 0xE0);
	status2 = serial_in(up, 0x02); /* EXCR1 */

	if (!((status2 ^ status1) & UART_MCR_LOOP)) {
		serial_out(up, UART_LCR, 0);
		serial_out(up, UART_MCR, status1 ^ UART_MCR_LOOP);
		serial_out(up, UART_LCR, 0xE0);
		status2 = serial_in(up, 0x02); /* EXCR1 */
		serial_out(up, UART_LCR, 0);
		serial_out(up, UART_MCR, status1);

		if ((status2 ^ status1) & UART_MCR_LOOP) {
			unsigned short quot;

			serial_out(up, UART_LCR, 0xE0);

			quot = serial_dl_read(up);
			quot <<= 3;

			if (ns16550a_goto_highspeed(up))
				serial_dl_write(up, quot);

			serial_out(up, UART_LCR, 0);

			up->port.uartclk = 921600*16;
			up->port.type = PORT_NS16550A;
			up->capabilities |= UART_NATSEMI;
			return;
		}
	}

	/*
	 * No EFR.  Try to detect a TI16750, which only sets bit 5 of
	 * the IIR when 64 byte FIFO mode is enabled when DLAB is set.
	 * Try setting it with and without DLAB set.  Cheap clones
	 * set bit 5 without DLAB set.
	 */
	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status1 = serial_in(up, UART_IIR) >> 5;
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status2 = serial_in(up, UART_IIR) >> 5;
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_LCR, 0);

	DEBUG_AUTOCONF("iir1=%d iir2=%d ", status1, status2);

	if (status1 == 6 && status2 == 7) {
		up->port.type = PORT_16750;
		up->capabilities |= UART_CAP_AFE | UART_CAP_SLEEP;
		return;
	}

	/*
	 * Try writing and reading the UART_IER_UUE bit (b6).
	 * If it works, this is probably one of the Xscale platform's
	 * internal UARTs.
	 * We're going to explicitly set the UUE bit to 0 before
	 * trying to write and read a 1 just to make sure it's not
	 * already a 1 and maybe locked there before we even start start.
	 */
	iersave = serial_in(up, UART_IER);
	serial_out(up, UART_IER, iersave & ~UART_IER_UUE);
	if (!(serial_in(up, UART_IER) & UART_IER_UUE)) {
		/*
		 * OK it's in a known zero state, try writing and reading
		 * without disturbing the current state of the other bits.
		 */
		serial_out(up, UART_IER, iersave | UART_IER_UUE);
		if (serial_in(up, UART_IER) & UART_IER_UUE) {
			/*
			 * It's an Xscale.
			 * We'll leave the UART_IER_UUE bit set to 1 (enabled).
			 */
			DEBUG_AUTOCONF("Xscale ");
			up->port.type = PORT_XSCALE;
			up->capabilities |= UART_CAP_UUE | UART_CAP_RTOIE;
			return;
		}
	} else {
		/*
		 * If we got here we couldn't force the IER_UUE bit to 0.
		 * Log it and continue.
		 */
		DEBUG_AUTOCONF("Couldn't force IER_UUE to 0 ");
	}
	serial_out(up, UART_IER, iersave);

	/*
	 * Exar uarts have EFR in a weird location
	 */
	if (up->port.flags & UPF_EXAR_EFR) {
		DEBUG_AUTOCONF("Exar XR17D15x ");
		up->port.type = PORT_XR17XX5X;
		up->capabilities |= UART_CAP_AFE | UART_CAP_EFR |
				    UART_CAP_SLEEP;

		return;
	}

	/*
	 * We distinguish between 16550A and U6 16550A by counting
	 * how many bytes are in the FIFO.
	 */
	if (up->port.type == PORT_16550A && size_fifo(up) == 64) {
		up->port.type = PORT_U6_16550A;
		up->capabilities |= UART_CAP_AFE;
	}
}

/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct uart_8250_port *up, unsigned int probeflags)
{
	unsigned char status1, scratch, scratch2, scratch3;
	unsigned char save_lcr, save_mcr;
	struct uart_port *port = &up->port;
	unsigned long flags;
	unsigned int old_capabilities;

	DEBUG_AUTOCONF("IN ttyCTI%d: autoconf (0x%04lx, 0x%p): ",
		       serial_index(port), port->iobase, port->membase);

	if (!port->iobase && !port->mapbase && !port->membase)
		return;

	DEBUG_AUTOCONF("ttyCTI%d: autoconf (0x%04lx, 0x%p): ",
		       serial_index(port), port->iobase, port->membase);

	/*
	 * We really do need global IRQs disabled here - we're going to
	 * be frobbing the chips IRQ enable register to see if it exists.
	 */
	spin_lock_irqsave(&port->lock, flags);

	up->capabilities = 0;
	up->bugs = 0;

	if (!(port->flags & UPF_BUGGY_UART)) {
		/*
		 * Do a simple existence test first; if we fail this,
		 * there's no point trying anything else.
		 *
		 * 0x80 is used as a nonsense port to prevent against
		 * false positives due to ISA bus float.  The
		 * assumption is that 0x80 is a non-existent port;
		 * which should be safe since include/asm/io.h also
		 * makes this assumption.
		 *
		 * Note: this is safe as long as MCR bit 4 is clear
		 * and the device is in "PC" mode.
		 */
		scratch = serial_in(up, UART_IER);
		serial_out(up, UART_IER, 0);
#ifdef __i386__
		outb(0xff, 0x080);
#endif
		/*
		 * Mask out IER[7:4] bits for test as some UARTs (e.g. TL
		 * 16C754B) allow only to modify them if an EFR bit is set.
		 */
		scratch2 = serial_in(up, UART_IER) & 0x0f;
		serial_out(up, UART_IER, 0x0F);
#ifdef __i386__
		outb(0, 0x080);
#endif
		scratch3 = serial_in(up, UART_IER) & 0x0f;
		serial_out(up, UART_IER, scratch);
		if (scratch2 != 0 || scratch3 != 0x0F) {
			/*
			 * We failed; there's nothing here
			 */
			spin_unlock_irqrestore(&port->lock, flags);
			DEBUG_AUTOCONF("IER test failed (%02x, %02x) ",
				       scratch2, scratch3);
			goto out;
		}
	}

	save_mcr = serial_in(up, UART_MCR);
	save_lcr = serial_in(up, UART_LCR);

	/*
	 * Check to see if a UART is really there.  Certain broken
	 * internal modems based on the Rockwell chipset fail this
	 * test, because they apparently don't implement the loopback
	 * test mode.  So this test is skipped on the COM 1 through
	 * COM 4 ports.  This *should* be safe, since no board
	 * manufacturer would be stupid enough to design a board
	 * that conflicts with COM 1-4 --- we hope!
	 */
	if (!(port->flags & UPF_SKIP_TEST)) {
		serial_out(up, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = serial_in(up, UART_MSR) & 0xF0;
		serial_out(up, UART_MCR, save_mcr);
		if (status1 != 0x90) {
			spin_unlock_irqrestore(&port->lock, flags);
			DEBUG_AUTOCONF("LOOP test failed (%02x) ",
				       status1);
			goto out;
		}
	}

	/*
	 * We're pretty sure there's a port here.  Lets find out what
	 * type of port it is.  The IIR top two bits allows us to find
	 * out if it's 8250 or 16450, 16550, 16550A or later.  This
	 * determines what we test for next.
	 *
	 * We also initialise the EFR (if any) to zero for later.  The
	 * EFR occupies the same register location as the FCR and IIR.
	 */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_in(up, UART_IIR) >> 6;

	switch (scratch) {
	case 0:
		autoconfig_8250(up);
		break;
	case 1:
        DEBUG_AUTOCONF("PORT_UNKNOWN" );
		port->type = PORT_UNKNOWN;
		break;
	case 2:
		port->type = PORT_16550;
		break;
	case 3:
		autoconfig_16550a(up);
		break;
	}

#ifdef CONFIG_SERIAL_8250_RSA
	/*
	 * Only probe for RSA ports if we got the region.
	 */
	if (port->type == PORT_16550A && probeflags & PROBE_RSA) {
		int i;

		for (i = 0 ; i < probe_rsa_count; ++i) {
			if (probe_rsa[i] == port->iobase && __enable_rsa(up)) {
				port->type = PORT_RSA;
				break;
			}
		}
	}
#endif

	serial_out(up, UART_LCR, save_lcr);

	port->fifosize = uart_config[up->port.type].fifo_size;
	old_capabilities = up->capabilities; 
	up->capabilities = uart_config[port->type].flags;
	up->tx_loadsz = uart_config[port->type].tx_loadsz;

	if (port->type == PORT_UNKNOWN)
		goto out_lock;

	/*
	 * Reset the UART.
	 */
#ifdef CONFIG_SERIAL_8250_RSA
	if (port->type == PORT_RSA)
		serial_out(up, UART_RSA_FRR, 0);
#endif
	serial_out(up, UART_MCR, save_mcr);
	serial8250_clear_fifos(up);
	serial_in(up, UART_RX);
	if (up->capabilities & UART_CAP_UUE)
		serial_out(up, UART_IER, UART_IER_UUE);
	else
		serial_out(up, UART_IER, 0);

out_lock:
	spin_unlock_irqrestore(&port->lock, flags);
	if (up->capabilities != old_capabilities) {
		printk(KERN_WARNING
		       "ttyCTI%d: detected caps %08x should be %08x\n",
		       serial_index(port), old_capabilities,
		       up->capabilities);
	}
out:
	DEBUG_AUTOCONF("iir=%d ", scratch);
	DEBUG_AUTOCONF("type=%s\n", uart_config[port->type].name);
}

static void autoconfig_irq(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	if (port->flags & UPF_FOURPORT) {
		ICP = (port->iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_in(up, UART_MCR);
	save_ier = serial_in(up, UART_IER);
	serial_out(up, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_out(up, UART_MCR, 0);
	udelay(10);
	if (port->flags & UPF_FOURPORT) {
   		if(up->use_xtreme)
   			serial_outp(up, UART_MCR,
			    SA485(up->softauto485, UART_MCR_DTR | UART_MCR_RTS));
        else
    		serial_out(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS);
	} else {
		if(up->use_xtreme)
			serial_outp(up, UART_MCR,
			    SA485(up->softauto485, UART_MCR_DTR | UART_MCR_RTS) | UART_MCR_OUT2);
		else
    		serial_out(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_out(up, UART_IER, 0x0f);	/* enable all intrs */
	serial_in(up, UART_LSR);
	serial_in(up, UART_RX);
	serial_in(up, UART_IIR);
	serial_in(up, UART_MSR);
	serial_out(up, UART_TX, 0xFF);
	udelay(20);
	irq = probe_irq_off(irqs);

	serial_out(up, UART_MCR, save_mcr);
	serial_out(up, UART_IER, save_ier);

	if (port->flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	port->irq = (irq > 0) ? irq : 0;
}

static inline void __stop_tx(struct uart_8250_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void serial8250_stop_tx(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	__stop_tx(up);

	/*
	 * We really want to stop the transmitter from sending.
	 */
	if (port->type == PORT_16C950) {
		up->acr |= UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void serial8250_start_tx(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	if (up->dma && !serial8250_tx_dma(up)) {
		return;
	} else if (!(up->ier & UART_IER_THRI)) {
		if (up->softauto485) {
			up->mcr |= UART_MCR_RTS;
			serial_out(up, UART_MCR, up->mcr);
		}
		up->ier |= UART_IER_THRI;
		serial_port_out(port, UART_IER, up->ier);

		if (up->bugs & UART_BUG_TXEN) {
			unsigned char lsr;
			lsr = serial_in(up, UART_LSR);
			up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
			if (lsr & UART_LSR_TEMT)
				cti_serial8250_tx_chars(up);
		}
	}

	/*
	 * Re-enable the transmitter if we disabled it.
	 */
	if (port->type == PORT_16C950 && up->acr & UART_ACR_TXDIS) {
		up->acr &= ~UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void serial8250_stop_rx(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_port_out(port, UART_IER, up->ier);
}

static void serial8250_enable_ms(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	/* no MSR capabilities */
	if (up->bugs & UART_BUG_NOMSR)
		return;

    if(!up->disable_msr_int && !disable_msr_int) {
    	up->ier |= UART_IER_MSI;
	    serial_port_out(port, UART_IER, up->ier);
    }
}

/*
 * cti_serial8250_rx_chars: processes according to the passed in LSR
 * value, and returns the remaining LSR bits not handled
 * by this Rx routine.
 */
unsigned char
cti_serial8250_rx_chars(struct uart_8250_port *up, unsigned char lsr)
{
	struct uart_port *port = &up->port;
	unsigned char ch;
	int max_count = 256;
	char flag;

	do {
		if (likely(lsr & UART_LSR_DR))
			ch = serial_in(up, UART_RX);
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;

		flag = TTY_NORMAL;
		port->icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				port->icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				port->icount.parity++;
			else if (lsr & UART_LSR_FE)
				port->icount.frame++;
			if (lsr & UART_LSR_OE)
				port->icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= port->read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		cti_uart_insert_char(port, lsr, UART_LSR_OE, ch, flag);

ignore_char:
		lsr = serial_in(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
	return lsr;
}
EXPORT_SYMBOL_GPL(cti_serial8250_rx_chars);

static __always_inline void*
serial8250_memcpy(char* dst, const char* src, size_t n)
{
	void* ret = (void*)dst;

	while (n >= 4) {
#ifdef CONFIG_64BIT
		*((unsigned int*)dst) = *((unsigned int*)src);
#else
		*((unsigned long*)dst) = *((unsigned long*)src);
#endif
		dst += 4;
		src += 4;
		n -= 4;
	}
	while (n >= 2) {
		*((unsigned short*)dst) = *((unsigned short*)src);
		dst += 2;
		src += 2;
		n -= 2;
	}
	if (n) {
		*((unsigned char*)dst) = *((unsigned char*)src);
	}

	return ret;
}

static unsigned int serial_asr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ASREN);
	value = serial_in(up, offset);
	serial_icr_write(up, UART_ACR, up->acr);

	return value;
}

void cti_serial8250_tx_chars(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	struct circ_buf *xmit = &port->state->xmit;
	int count;
    unsigned int    tmpcount1, tmp, tmpcount2;

	if (port->x_char) {
		serial_out(up, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	if (uart_tx_stopped(port)) {
		serial8250_stop_tx(port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		if (up->softauto485)
			tasklet_schedule(&up->tlet);
		__stop_tx(up);
		return;
	}

	if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X) ) {
		count = up->tx_loadsz;
		if( up->port.type == PORT_16C950 ) {
			tmpcount1 = serial_asr_read( up, UART_TFL );
			for (tmp=0; tmp < 2; tmp++) {
				tmpcount2 = serial_asr_read( up, UART_TFL );
				if(tmpcount2 > tmpcount1)
					tmpcount1 = tmpcount2;
			}
			count -= tmpcount1;
			count = MIN(count, uart_circ_chars_pending(xmit));
		}
		do {
			serial_out(up, UART_TX, xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			up->port.icount.tx++;
			if (uart_circ_empty(xmit))
				break;
		} while (--count > 0);
	} else {
		count = up->port.fifosize;

		/* read the TX count 3 times and take the maximum
		 * see Exar's DAN119
		 */
		tmpcount1 = serial_in(up, UART_FLAT_TXCNT);
		for (tmp=0; tmp < 2; tmp++) {
			tmpcount2 = serial_in(up, UART_FLAT_TXCNT);
			if(tmpcount2 > tmpcount1)
				tmpcount1 = tmpcount2;
		}
		count -= tmpcount1;
		/* is the data to be written
		 * smaller than the available buffer?
		 */
		count = MIN(count, uart_circ_chars_pending(xmit));
		do {
			if(((xmit->tail + count) & (UART_XMIT_SIZE-1)) < xmit->tail) {
				tmp = UART_XMIT_SIZE - xmit->tail;
				serial8250_memcpy(up->port.membase + UART_17XX5X_TX,
					(xmit->buf + xmit->tail),
					tmp);
				count -= tmp;
				xmit->tail += tmp;

				xmit->tail &= (UART_XMIT_SIZE-1);
				up->port.icount.tx += tmp;
			} else {
				serial8250_memcpy(up->port.membase + UART_17XX5X_TX,
					(xmit->buf + xmit->tail),
					count);
				xmit->tail += count;
				up->port.icount.tx += count;
				count = 0;
				xmit->tail &= (UART_XMIT_SIZE-1);
			}
		} while(count > 0);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		cti_uart_write_wakeup(port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit)){
		if (up->softauto485)
			tasklet_schedule(&up->tlet);
		__stop_tx(up);
	}
}
EXPORT_SYMBOL_GPL(cti_serial8250_tx_chars);

static inline void
uart_handle_cts_change_ctibh(struct uart_8250_port *up, unsigned int status)
{
	 struct uart_port *uport = &up->port;
	 struct tty_port *port = &uport->state->port;
    struct tty_struct *tty = port->tty;


	uport->icount.cts++;

	if (uport->flags &  ASYNC_CTS_FLOW) {
		if (tty->hw_stopped) {
			if (status) {
				tty->hw_stopped = 0;
				if (up->softauto485) {
					up->mcr |= UART_MCR_RTS;
					serial_out(up, UART_MCR, up->mcr);
				}
				uport->ops->start_tx(uport);
				cti_uart_write_wakeup(uport);
			}
		} else {
			if (!status) {
				tty->hw_stopped = 1;
				uport->ops->stop_tx(uport);
			}
		}
	}
 }


unsigned int cti_serial8250_modem_status(struct uart_8250_port *up)
{
	struct uart_port *port = &up->port;
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    port->state != NULL) {
		if (status & UART_MSR_TERI)
			port->icount.rng++;
		if (status & UART_MSR_DDSR)
			port->icount.dsr++;
		if (status & UART_MSR_DDCD)
			cti_uart_handle_dcd_change(port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
        {
			if(up->use_xtreme)
	        	uart_handle_cts_change_ctibh(up, status & UART_MSR_CTS);
   	        else
			    cti_uart_handle_cts_change(port, status & UART_MSR_CTS);
        }
		wake_up_interruptible(&port->state->port.delta_msr_wait);
	}

	return status;
}
EXPORT_SYMBOL_GPL(cti_serial8250_modem_status);

/*
 * This handles the interrupt from one port.
 */
int cti_serial8250_handle_irq(struct uart_port *port, unsigned int iir)
{
	unsigned char status;
	unsigned long flags;
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	int dma_err = 0;

	if (iir & UART_IIR_NO_INT)
		return 0;

	spin_lock_irqsave(&port->lock, flags);

	status = serial_port_in(port, UART_LSR);

	DEBUG_INTR("status = %x...", status);

	if (status & (UART_LSR_DR | UART_LSR_BI)) {
		if (up->dma)
			dma_err = serial8250_rx_dma(up, iir);

		if (!up->dma || dma_err)
			status = cti_serial8250_rx_chars(up, status);
	}
	cti_serial8250_modem_status(up);
	if (status & UART_LSR_THRE)
		cti_serial8250_tx_chars(up);

	spin_unlock_irqrestore(&port->lock, flags);
	return 1;
}
EXPORT_SYMBOL_GPL(cti_serial8250_handle_irq);

static int serial8250_default_handle_irq(struct uart_port *port)
{
	unsigned int iir = serial_port_in(port, UART_IIR);

	return cti_serial8250_handle_irq(port, iir);
}

/*
 * These Exar UARTs have an extra interrupt indicator that could
 * fire for a few unimplemented interrupts.  One of which is a
 * wakeup event when coming out of sleep.  Put this here just
 * to be on the safe side that these interrupts don't go unhandled.
 */
/*static int exar_handle_irq(struct uart_port *port)
{
	unsigned char int0, int1, int2, int3;
	unsigned int iir = serial_port_in(port, UART_IIR);
	int ret;

	ret = cti_serial8250_handle_irq(port, iir);

	if ((port->type == PORT_XR17V35X) ||
	   (port->type == PORT_XR17XX5X)) {
		int0 = serial_port_in(port, 0x80);
		int1 = serial_port_in(port, 0x81);
		int2 = serial_port_in(port, 0x82);
		int3 = serial_port_in(port, 0x83);
	}

	return ret;
}*/

static void
receive_chars(struct uart_8250_port *up, unsigned int *status)
{
	struct tty_port *tty = &up->port.state->port;
	unsigned char ch, lsr = (unsigned char)*status;
	int max_count = 256;
	char flag;

	do {
		if (lsr & UART_LSR_DR)
			ch = serial_inp(up, UART_RX);
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & UART_LSR_BRK_ERROR_BITS)) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		cti_uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

ignore_char:
		lsr = serial_inp(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
	*status = lsr;
}



static void
receive_chars_buf(struct uart_8250_port *up, unsigned char *buf, int count)
{
	struct tty_port *tty = &up->port.state->port;
	unsigned char lsr;
	char flag;

	do {
		lsr = buf[0];
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
				    UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (lsr & UART_LSR_PE)
				up->port.icount.parity++;
			else if (lsr & UART_LSR_FE)
				up->port.icount.frame++;
			if (lsr & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			lsr &= up->port.read_status_mask;

			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (lsr & UART_LSR_FE)
				flag = TTY_FRAME;
		}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
		if (uart_handle_sysrq_char(&up->port, buf[1], regs))
			goto ignore_char;
#else
		if (uart_handle_sysrq_char(&up->port, buf[1]))
			goto ignore_char;
#endif

		cti_uart_insert_char(&up->port, lsr, UART_LSR_OE, buf[1], flag);

	ignore_char:
		buf += 2;
		count--;
	} while (count);
	spin_unlock(&up->port.lock);
	tty_flip_buffer_push(tty);
	spin_lock(&up->port.lock);
}

static void transmit_chars(struct uart_8250_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	unsigned int count, tmpcount1, tmpcount2;
	int tmp;
	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		serial8250_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		if (up->softauto485)
			tasklet_schedule(&up->tlet);
		__stop_tx(up);
		return;
	}
	if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X) ) {
		count = up->tx_loadsz;
		if( up->port.type == PORT_16C950 ) {
			tmpcount1 = serial_asr_read( up, UART_TFL );
			for (tmp=0; tmp < 2; tmp++) {
				tmpcount2 = serial_asr_read( up, UART_TFL );
				if(tmpcount2 > tmpcount1)
					tmpcount1 = tmpcount2;
			}
			count -= tmpcount1;
			count = MIN(count, uart_circ_chars_pending(xmit));
		}
		do {
			serial_out(up, UART_TX, xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			up->port.icount.tx++;
			if (uart_circ_empty(xmit))
				break;
		} while (--count > 0);
	} else {
		count = up->port.fifosize;

		/* read the TX count 3 times and take the maximum
		 * see Exar's DAN119
		 */
		tmpcount1 = serial_in(up, UART_FLAT_TXCNT);
		for (tmp=0; tmp < 2; tmp++) {
			tmpcount2 = serial_in(up, UART_FLAT_TXCNT);
			if(tmpcount2 > tmpcount1)
				tmpcount1 = tmpcount2;
		}
		count -= tmpcount1;
		/* is the data to be written
		 * smaller than the available buffer?
		 */
		count = MIN(count, uart_circ_chars_pending(xmit));
		do {
			if(((xmit->tail + count) & (UART_XMIT_SIZE-1)) < xmit->tail) {
				tmp = UART_XMIT_SIZE - xmit->tail;
				serial8250_memcpy(up->port.membase + UART_17XX5X_TX,
					(xmit->buf + xmit->tail),
					tmp);
				count -= tmp;
				xmit->tail += tmp;

				xmit->tail &= (UART_XMIT_SIZE-1);
				up->port.icount.tx += tmp;
			} else {
				serial8250_memcpy(up->port.membase + UART_17XX5X_TX,
					(xmit->buf + xmit->tail),
					count);
				xmit->tail += count;
				up->port.icount.tx += count;
				count = 0;
				xmit->tail &= (UART_XMIT_SIZE-1);
			}
		} while(count > 0);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		cti_uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit)) {
		if (up->softauto485)
			tasklet_schedule(&up->tlet);
 		__stop_tx(up);
	}
}

static unsigned int check_modem_status(struct uart_8250_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);
	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
		{
			if(up->use_xtreme)
	        	uart_handle_cts_change_ctibh(up, status & UART_MSR_CTS);
	      else
				uart_handle_cts_change(&up->port, status & UART_MSR_CTS);
		}

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

static inline int
serial8250_handle_xr17xx5x(struct uart_8250_port *up)
{
	int            retval = 0;
	unsigned int   pass_count = 0;
	unsigned long  flags;
	unsigned char  i;
	unsigned char iir;
	unsigned int   count1;
	unsigned int   count2;
	unsigned char  reset_fifo;
	unsigned short fifo_size;
	unsigned char  dump[512];
	unsigned int status;

	spin_lock_irqsave(&up->port.lock, flags);
	
	iir = serial_in(up, UART_IIR);

	fifo_size = uart_config[up->port.type].fifo_size;

	//DEBUG_INTR("INTR= %d, ier=0x%x iir = 0x%x\n", !(iir & UART_IIR_NO_INT), ier, iir);
	while (!(iir & UART_IIR_NO_INT)) {
		retval = 1;
		if(pass_count++ > 10) {
			/*printk("%d-pch,%#x;",
				serial_index(up),
				iir);*/
			break;
		}

		switch (iir & 0x3f) {
		case UART_IIR_RDI:  // RX
		case UART_IIR_RDTI:  // RXTO
			/* read the RX count 3 times and take
			 * the minimum
			 * see Exar's DAN119
			 */
            DEBUG_INTR(" RDI \n");
			count1 = serial_in(up,
				UART_FLAT_RXCNT);
			for (i = 0; i < 2; i++) {
				count2 = serial_in(up,
					UART_FLAT_RXCNT);
				if (count2 < count1)
					count1 = count2;
			}
			reset_fifo = 0;
			if (count1 > fifo_size) {
				/* hmm, we appear to have hit
				 * the overrun error failure
				 * condition, reset the FIFO
				 */
				count1 = fifo_size;
				reset_fifo = 1;
			}
			if (count1 > 0) {
				/* it's possible that the count
				 * is too high due to the "overrun-read
				 * race condition bug"
				 * to account for it we will only
				 * trust the FIFO count minus 1 and
				 * poll LSR for the last byte
				 */
				if(serial_in(up, UART_LSR) & UART_LSR_FIFO_ERRORS)
				{
					count2 = (count1 - 1) * 2;
					if (count2 > 0) {
						serial8250_memcpy(dump,
							up->port.membase + ( (up->port.type == PORT_XR17XX5X) ? UART_17XX5X_RXLSR : UART_17V35X_RXLSR ),
							count2);
					}
					dump[count2] = serial_in(up, UART_LSR);
					if (dump[count2] & UART_LSR_DR) {
						dump[count2+1] = serial_in(up, UART_RX);
					} else {
						count1--;
					}
					if (count1 > 0) {
						receive_chars_buf(up, dump, count1);
					}
				}
				else
				{
					unsigned int size;
					// no error in fifo so pass buffer to tty
					struct tty_port *tty = &up->port.state->port;
					
					serial8250_memcpy(dump, up->port.membase + UART_17XX5X_RX, count1);
					size = tty_insert_flip_string(tty, dump, count1);
					if(count1 != size)
						up->port.icount.overrun += (count1 - size);
					up->port.icount.rx += size;

					spin_unlock(&up->port.lock);
					tty_flip_buffer_push(tty);
					spin_lock(&up->port.lock);
				}

				/* we don't need to worry
				 * about reset_fifo being set
				 * if count1 is less than 1,
				 * since reset_fifo can only be
				 * set if count1 is greater
				 * than fifo_size
				 */
				if (reset_fifo) {
					up->port.icount.overrun++;
					serial_out(up, UART_FCR,
						UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR);
				}
			} else {
				/* huh, we got an RX interrupt
				 * with an RX FIFO count of
				 * zero???  this is one of the
				 * signs that we are stuck in
				 * an infinite RX timeout
				 * interrupt, so we're going to
				 * reset the FIFO
				 */
				serial_out(up, UART_FCR,
					UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR);
			}
			break;
		case UART_IIR_THRI:  // TX
            DEBUG_INTR(" TX \n");
			transmit_chars(up);
			break;
		case UART_IIR_MSI:  // MS
		case UART_IIR_EMS:  // EMS
            DEBUG_INTR(" MS \n");
            cti_serial8250_modem_status(up);
			check_modem_status(up);
			break;
		case UART_IIR_RLSI:
            DEBUG_INTR(" RLSI \n");
			status = serial_inp(up, UART_LSR);
			if (status & (UART_LSR_DR | UART_LSR_BI))
	                	receive_chars(up, &status);
			break;
		default:
			printk("unknown or unexpected int "
				"(0x%x);", iir);
			break;
		}

		iir = serial_in(up, UART_IIR);
	}

	/* we know there are no more irqs;
	 * check LSR for empty & IER_THRI non-zero and simulate an irq
	 */
	if (up->ier & UART_IER_THRI) {
		if (serial_in(up, UART_LSR) & UART_LSR_THRE) {
			transmit_chars(up);
		}
	}
	
	/* Exar extra interrupt fix from exar_handle_irq() - MG*/
	serial_in(up, 0x80);
	serial_in(up, 0x81);
	serial_in(up, 0x82);
	serial_in(up, 0x83);

	spin_unlock_irqrestore(&up->port.lock, flags);
	
	DEBUG_INTR("retval: %d\n", retval);

	return retval;
}

static void serial8250_plx_fix(struct list_head *l, char begin_end)
{
	struct list_head *head = l;
	struct uart_8250_port *up;

	if (l == NULL)
		return;

	do {
		up = list_entry(l, struct uart_8250_port, list);
		if (up->port.use_plx_fix) {
			if (begin_end) {
				/* disallow interrupt generation
				 * MPIOLVL[0] = 1
				 */
				serial_dcr_out(up, 0x10, 0xff);
				up->port.plx_fix_applied = 1;
			} else {
				if (up->port.plx_fix_applied) {
					/* allow interrupt generation
					 * MPIOLVL[0] = 0
					 */
					serial_dcr_out(up, 0x10, 0xfe);
					up->port.plx_fix_applied = 0;
				}
			}
		}
		l = l->next;
	} while ((l != NULL) && (l != head));
}



/*
 * This is the serial driver's interrupt routine.
 *
 * Arjan thinks the old way was overly complex, so it got simplified.
 * Alan disagrees, saying that need the complexity to handle the weird
 * nature of ISA shared interrupts.  (This is a special exception.)
 *
 * In order to handle ISA shared interrupts properly, we need to check
 * that all ports have been serviced, and therefore the ISA interrupt
 * line has been de-asserted.
 *
 * This means we need to loop through all ports. checking that they
 * don't have an interrupt pending.
 */
static irqreturn_t serial8250_interrupt(int irq, void *dev_id)
{
	struct irq_info *i = dev_id;
	struct list_head *l, *end = NULL;
	int pass_counter = 0, handled = 0;

	DEBUG_INTR("serial8250_interrupt(%d)...", irq);

	spin_lock(&i->lock);

    serial8250_plx_fix(i->head, 1);

	l = i->head;
	do {
		struct uart_8250_port *up;
		struct uart_port *port;

		up = list_entry(l, struct uart_8250_port, list);
		//ier = serial_in(up, UART_IER);
		//iir = serial_in(up, UART_IIR);
		//DEBUG_INTR("iir1=0x%x,  ier1=0x%x\n", iir, ier);
		port = &up->port;
        if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X) ) {
		    if (port->handle_irq(port)) {
			    handled = 1;
			    end = NULL;
		    } else if (end == NULL)
			    end = l;
        }
        else
        {
 			if (serial8250_handle_xr17xx5x(up)) {
				handled = 1;
				end = NULL;
			} else if (end == NULL){
				end = l;
			}
       }

		l = l->next;

		if (l == i->head && pass_counter++ > PASS_LIMIT) {
			/* If we hit this, we're dead. */
			printk_ratelimited(KERN_ERR
				"serial8250: too much work for irq%d\n", irq);
			serial_out(up, UART_IER, 0);
			break;
		}
	} while (l != end);

    serial8250_plx_fix(i->head, 0);

	spin_unlock(&i->lock);
	
	DEBUG_INTR("end, handled= %d\n", handled);
	
	return IRQ_RETVAL(handled);
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_8250_port *up)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &up->list)
			i->head = i->head->next;
		list_del(&up->list);
	} else {
		BUG_ON(i->head != &up->list);
		i->head = NULL;
	}
	spin_unlock_irq(&i->lock);
	/* List empty so throw away the hash node */
	if (i->head == NULL) {
		hlist_del(&i->node);
		kfree(i);
	}
}

static int serial_link_irq_chain(struct uart_8250_port *up)
{
	struct hlist_head *h;
	struct hlist_node *n;
	struct irq_info *i;
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
	mutex_lock(&hash_mutex);

	h = &irq_lists[up->port.irq % NR_IRQ_HASH];

	hlist_for_each(n, h) {
		i = hlist_entry(n, struct irq_info, node);
		if (i->irq == up->port.irq)
			break;
	}

	if (n == NULL) {
		i = kzalloc(sizeof(struct irq_info), GFP_KERNEL);
		if (i == NULL) {
			mutex_unlock(&hash_mutex);
			return -ENOMEM;
		}
		spin_lock_init(&i->lock);
		i->irq = up->port.irq;
		hlist_add_head(&i->node, h);
	}
	mutex_unlock(&hash_mutex);

	spin_lock_irq(&i->lock);

	if (i->head) {
		list_add(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);
		irq_flags |= up->port.irqflags;
		ret = request_irq(up->port.irq, serial8250_interrupt,
				  irq_flags, "serial", i);
		if (ret < 0){
			serial_do_unlink(i, up);
		}
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_8250_port *up)
{
	struct irq_info *i;
	struct hlist_node *n;
	struct hlist_head *h;
	mutex_lock(&hash_mutex);

	h = &irq_lists[up->port.irq % NR_IRQ_HASH];

	hlist_for_each(n, h) {
		i = hlist_entry(n, struct irq_info, node);
		if (i->irq == up->port.irq)
			break;
	}

	BUG_ON(n == NULL);
	BUG_ON(i->head == NULL);

	if (list_empty(i->head))
		free_irq(up->port.irq, i);

	serial_do_unlink(i, up);
	mutex_unlock(&hash_mutex);
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
static void serial8250_timeout(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *)data;
#else
static void serial8250_timeout(struct timer_list *tl)
{
	struct uart_8250_port *up = from_timer(up,tl,timer);
#endif

	up->port.handle_irq(&up->port);
	mod_timer(&up->timer, jiffies + uart_poll_timeout(&up->port));
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
static void serial8250_backup_timeout(unsigned long data)
{
	struct uart_8250_port *up = (struct uart_8250_port *)data;
#else
static void serial8250_backup_timeout(struct timer_list *tl)
{
	struct uart_8250_port *up = from_timer(up,tl,timer);
#endif
	unsigned int iir, ier = 0, lsr;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Must disable interrupts or else we risk racing with the interrupt
	 * based handler.
	 */
	if (up->port.irq) {
		ier = serial_in(up, UART_IER);
		serial_out(up, UART_IER, 0);
	}

	iir = serial_in(up, UART_IIR);

	/*
	 * This should be a safe test for anyone who doesn't trust the
	 * IIR bits on their UART, but it's specifically designed for
	 * the "Diva" UART used on the management processor on many HP
	 * ia64 and parisc boxes.
	 */
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	if ((iir & UART_IIR_NO_INT) && (up->ier & UART_IER_THRI) &&
	    (!uart_circ_empty(&up->port.state->xmit) || up->port.x_char) &&
	    (lsr & UART_LSR_THRE)) {
		iir &= ~(UART_IIR_ID | UART_IIR_NO_INT);
		iir |= UART_IIR_THRI;
	}

	if (!(iir & UART_IIR_NO_INT))
		cti_serial8250_tx_chars(up);

	if (up->port.irq)
		serial_out(up, UART_IER, ier);

	spin_unlock_irqrestore(&up->port.lock, flags);

	/* Standard timer interval plus 0.2s to keep the port running */
	mod_timer(&up->timer,
		jiffies + uart_poll_timeout(&up->port) + HZ / 5);
}

static unsigned int serial8250_tx_empty(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&port->lock, flags);
	lsr = serial_port_in(port, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&port->lock, flags);

	return (lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int serial8250_get_mctrl(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned int status;
	unsigned int ret;

	status = cti_serial8250_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial8250_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned char mcr = 0;

	if(up->port.use_clr_rts == 0){
		if (mctrl & TIOCM_RTS) {
			if(up->use_xtreme)
				mcr |= SA485(up->softauto485, UART_MCR_RTS);
			else
				mcr |= UART_MCR_RTS;
		}
	}
	if (mctrl & TIOCM_DTR) {
			if(up->use_xtreme)
				mcr |= SA485(up->softauto485, UART_MCR_DTR);
			else
				mcr |= UART_MCR_DTR;
	}
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_port_out(port, UART_MCR, mcr);
}

static int softauto485(struct uart_8250_port *up, int onoff)
{
	if (onoff) {
		/* turn on special mode */
		if (!up->is_saved_mcr) {
			up->saved_mcr = up->mcr;
			up->is_saved_mcr = 1;
		}
		if (onoff == TIOCSOFTAUTO485_DTRLOW)
			up->mcr &= ~UART_MCR_DTR;
		if (up->ier & UART_IER_THRI)
			up->mcr |= UART_MCR_RTS;
		else
			up->mcr &= ~UART_MCR_RTS;
	} else {
		/* turn off mode */
		up->mcr = up->saved_mcr;
		up->is_saved_mcr = 0;
	}
	serial_out(up, UART_MCR, up->mcr);

	return 0;
}


static int do_softauto485(struct uart_8250_port *up, int *value)
{
	int onoff = 0;

	if (copy_from_user(&onoff, value, sizeof(int)))
		return -EFAULT;

	if (onoff < 0 || onoff > TIOCSOFTAUTO485_DTRNORM)
		return -EINVAL;

	if (onoff == up->softauto485)
		return 0;

	up->softauto485 = onoff;

	softauto485(up, onoff);

	return 0;
}

static void serial8250_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_port_out(port, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&port->lock, flags);
}

/*
 *	Wait for transmitter & holding register to empty
 */
static void wait_for_xmitr(struct uart_8250_port *up, int bits)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	for (;;) {
		status = serial_in(up, UART_LSR);

		up->lsr_saved_flags |= status & LSR_SAVE_FLAGS;

		if ((status & bits) == bits)
			break;
		if (--tmout == 0)
			break;
		udelay(1);
	}

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		unsigned int tmout;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);
			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
			touch_nmi_watchdog();
		}
	}
}

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for writing and reading from the uart while
 * in an interrupt or debug context.
 */

static int serial8250_get_poll_char(struct uart_port *port)
{
	unsigned char lsr = serial_port_in(port, UART_LSR);

	if (!(lsr & UART_LSR_DR))
		return NO_POLL_CHAR;

	return serial_port_in(port, UART_RX);
}


static void serial8250_put_poll_char(struct uart_port *port,
			 unsigned char c)
{
	unsigned int ier;
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_port_in(port, UART_IER);
	if (up->capabilities & UART_CAP_UUE)
		serial_port_out(port, UART_IER, UART_IER_UUE);
	else
		serial_port_out(port, UART_IER, 0);

	wait_for_xmitr(up, BOTH_EMPTY);
	/*
	 *	Send the character out.
	 *	If a LF, also do CR...
	 */
	serial_port_out(port, UART_TX, c);
	if (c == 10) {
		wait_for_xmitr(up, BOTH_EMPTY);
		serial_port_out(port, UART_TX, 13);
	}

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up, BOTH_EMPTY);
	serial_port_out(port, UART_IER, ier);
}

#endif /* CONFIG_CONSOLE_POLL */

static int serial8250_startup(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned long flags;
	unsigned char lsr, iir;
	int retval;
	if (port->type == PORT_8250_CIR)
		return -ENODEV;

	if (!port->fifosize)
		port->fifosize = uart_config[port->type].fifo_size;
	if (!up->tx_loadsz)
		up->tx_loadsz = uart_config[port->type].tx_loadsz;
	if (!up->capabilities)
		up->capabilities = uart_config[port->type].flags;
	// xtreme/104 MCR bit 7=1 if jumpered; this condition will be cleared otherwise
	up->mcr = serial_inp(up, UART_MCR) & 0xe0;
    //up->mcr = 0;

	if (port->iotype != up->cur_iotype)
		set_io_from_upio(port);

	if (port->type == PORT_16C950) {
		/* Wake up and initialize UART */
		up->acr = 0;
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_IER, 0);
		serial_port_out(port, UART_LCR, 0);
		serial_icr_write(up, UART_CSR, 0); /* Reset the UART */
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_LCR, 0);
	}

    //reset uart
	if ( (port->type == PORT_XR17XX5X) || (port->type == PORT_XR17V35X) )
	{
		unsigned char index;
		index = port->index_on_dev;

		//reset test
		//serial_out( up, UART_SCR, up->port.index_on_board );
		//dump_uart( port );

		serial_dcr_out( up, 0x0a, (1 << index) );
	}
	


#ifdef CONFIG_SERIAL_8250_RSA
	/*
	 * If this is an RSA port, see if we can kick it up to the
	 * higher speed clock.
	 */
	enable_rsa(up);
#endif

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial8250_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(port->flags & UPF_BUGGY_UART) &&
	    (serial_port_in(port, UART_LSR) == 0xff)) {
		printk_ratelimited(KERN_INFO "ttyCTI%d: LSR safety check engaged!\n",
				   serial_index(port));
		return -ENODEV;
	}

	/*
	 * For a XR16C850, we need to set the trigger levels
	 */
	if (port->type == PORT_16850) {
		unsigned char fctr;

		serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

		fctr = serial_in(up, UART_FCTR) & ~(UART_FCTR_RX|UART_FCTR_TX);
		serial_port_out(port, UART_FCTR,
				fctr | UART_FCTR_TRGD | UART_FCTR_RX);
		serial_port_out(port, UART_TRG, UART_TRG_96);
		serial_port_out(port, UART_FCTR,
				fctr | UART_FCTR_TRGD | UART_FCTR_TX);
		serial_port_out(port, UART_TRG, UART_TRG_96);

		serial_port_out(port, UART_LCR, 0);
	}

	if (is_real_interrupt(up->port.irq)) {
		unsigned char iir1;
		/*
		 * Test for UARTs that do not reassert THRE when the
		 * transmitter is idle and the interrupt has already
		 * been cleared.  Real 16550s should always reassert
		 * this interrupt whenever the transmitter is idle and
		 * the interrupt is enabled.  Delays are necessary to
		 * allow register changes to become visible.
		 */
		spin_lock_irqsave(&port->lock, flags);
		if (up->port.irqflags & IRQF_SHARED)
			disable_irq_nosync(port->irq);

		wait_for_xmitr(up, UART_LSR_THRE);
		serial_port_out_sync(port, UART_IER, UART_IER_THRI);
		udelay(1); /* allow THRE to set */
		iir1 = serial_port_in(port, UART_IIR);
		serial_port_out(port, UART_IER, 0);
		serial_port_out_sync(port, UART_IER, UART_IER_THRI);
		udelay(1); /* allow a working UART time to re-assert THRE */
		iir = serial_port_in(port, UART_IIR);
		serial_port_out(port, UART_IER, 0);

		if (port->irqflags & IRQF_SHARED)
			enable_irq(port->irq);
		spin_unlock_irqrestore(&port->lock, flags);

		/*
		 * If the interrupt is not reasserted, or we otherwise
		 * don't trust the iir, setup a timer to kick the UART
		 * on a regular basis.
		 */
		if ((!(iir1 & UART_IIR_NO_INT) && (iir & UART_IIR_NO_INT)) ||
		    up->port.flags & UPF_BUG_THRE) {
			up->bugs |= UART_BUG_THRE;
			pr_debug("ttyCTI%d - using backup timer\n",
				 serial_index(port));
		}
	}

	/*
	 * The above check will only give an accurate result the first time
	 * the port is opened so this value needs to be preserved.
	 */
	if (up->bugs & UART_BUG_THRE) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
		up->timer.function = serial8250_backup_timeout;
		up->timer.data = (unsigned long)up;
#else
		timer_setup(&up->timer,serial8250_backup_timeout,0);
#endif
		mod_timer(&up->timer, jiffies +
			uart_poll_timeout(port) + HZ / 5);
	}

	if ( (up->port.type == PORT_XR17XX5X) || (up->port.type == PORT_XR17V35X) ) {
 		unsigned char fctr;
		serial_outp(up, UART_FLAT_EFR, serial_inp(up, UART_FLAT_EFR) | UART_EFR_ECB);
		if (up->port.lmode_fn) {
                        up->port.lmode_fn((struct uart_port *)up, TIOCSER485SET, &up->port.lmode);
                }

 		fctr = serial_inp(up, UART_FLAT_FCTR) & ~(UART_FLAT_FCTR_TRGMASK);
		//set also RTS/DTR hysteresis 8 bytes
 		serial_outp(up, UART_FLAT_FCTR, fctr | UART_FLAT_FCTR_TRGD | UART_FCTR_RTS_8DELAY);
 		serial_outp(up, UART_FLAT_TXTRG, (uart_config[up->port.type].fifo_size / 8) * 6 );
 		serial_outp(up, UART_FLAT_RXTRG, (uart_config[up->port.type].fifo_size / 8) * 6 );
 	}

	if (up->port.type == PORT_16C950) {
		// 750 mode trigger level
		serial_outp(up, UART_FCR, 0xff);
		if (up->port.lmode_fn) {
                        up->port.lmode_fn((struct uart_port *)up, TIOCSER485SET, &up->port.lmode);
                }
 	}

 	if(up->softauto485 && up->port.type == PORT_16654)
 		softauto485(up, up->softauto485);

    
    /*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
	if (!is_real_interrupt(up->port.irq)) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
		up->timer.data = (unsigned long)up;
#endif
		mod_timer(&up->timer, jiffies + uart_poll_timeout(port));
	} else {
		retval = serial_link_irq_chain(up);
		if (retval)
			return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_port_out(port, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&port->lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		if (!up->port.irq)
			up->port.mctrl |= TIOCM_OUT1;
	} else
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT2;

	serial8250_set_mctrl(port, port->mctrl);

	/* Serial over Lan (SoL) hack:
	   Intel 8257x Gigabit ethernet chips have a
	   16550 emulation, to be used for Serial Over Lan.
	   Those chips take a longer time than a normal
	   serial device to signalize that a transmission
	   data was queued. Due to that, the above test generally
	   fails. One solution would be to delay the reading of
	   iir. However, this is not reliable, since the timeout
	   is variable. So, let's just don't test if we receive
	   TX irq. This way, we'll never enable UART_BUG_TXEN.
	 */
	if (skip_txen_test || up->port.flags & UPF_NO_TXEN_TEST)
		goto dont_test_tx_en;

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_port_out(port, UART_IER, UART_IER_THRI);
	lsr = serial_port_in(port, UART_LSR);
	iir = serial_port_in(port, UART_IIR);
	serial_port_out(port, UART_IER, 0);

	if (lsr & UART_LSR_TEMT && iir & UART_IIR_NO_INT) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			pr_debug("ttyCTI%d - enabling bad tx status workarounds\n",
				 serial_index(port));
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

dont_test_tx_en:
	spin_unlock_irqrestore(&port->lock, flags);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Request DMA channels for both RX and TX.
	 */
	/*if (up->dma) {
		retval = serial8250_request_dma(up);
		if (retval) {
			pr_warn_ratelimited("ttyCTI%d - failed to request DMA\n",
					    serial_index(port));
			up->dma = NULL;
		}
	}*/

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RDI;
	if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X))
		up->ier |= UART_IER_RLSI;
	serial_port_out(port, UART_IER, up->ier);

	
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);
	
	//if (port->flags & UPF_FOURPORT) {
		//unsigned int icp;
		/*
		 * Enable interrupts on the AST Fourport board
		 */
	//	icp = (port->iobase & 0xfe0) | 0x01f;
	//	outb_p(0x80, icp);
	//	inb_p(icp);
//	}
	check_mpio(up);
	
	return 0;
}

static void serial8250_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned long flags;

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_port_out(port, UART_IER, 0);

	if (up->dma)
		serial8250_release_dma(up);

	spin_lock_irqsave(&port->lock, flags);
	if (port->flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((port->iobase & 0xfe0) | 0x1f);
		port->mctrl |= TIOCM_OUT1;
	} else
		port->mctrl &= ~TIOCM_OUT2;

	serial8250_set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_port_out(port, UART_LCR,
		serial_port_in(port, UART_LCR) & ~UART_LCR_SBC);
	serial8250_clear_fifos(up);

#ifdef CONFIG_SERIAL_8250_RSA
	/*
	 * Reset the RSA board back to 115kbps compat mode.
	 */
	disable_rsa(up);
#endif

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	serial_port_in(port, UART_RX);

	del_timer_sync(&up->timer);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
	up->timer.function = serial8250_timeout;
#else
	timer_setup(&up->timer,serial8250_timeout,0);
#endif
	if (port->irq){
		serial_unlink_irq_chain(up);
	}
}

static inline unsigned long TruncX1000(unsigned long val)
{
	return (val/1000)*1000;
}

static inline unsigned long RoundX1000(unsigned long val)
{
	return ((val+500)/1000)*1000;
}

static inline unsigned long AbsDiff(unsigned long val1, unsigned long val2)
{
	if (val1 > val2)
		return val1-val2;
	return val2-val1;
}

//filter the bounds on the divisor
unsigned long DivisorBoundsFilter( unsigned long divisor, unsigned long prescaler )
{
	// limit the closest divisor to 0xffff * 1000 * prescaler because bigger than that overruns the possible divisor value
	unsigned long upper_limit = (0xFFFFL * 1000 * prescaler);
	if( divisor >  upper_limit )
		divisor = upper_limit;

	// can't go with a divisor that is less than 1?
	if( divisor < 1000L )
		divisor = 1000L;

	return divisor;
}


void
cti_uart_get_divisorf(struct uart_port *port, unsigned int baud, unsigned int *quot, unsigned int *frac, unsigned char *prescaler, unsigned char *sample_rate)
{


	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST) {
		*quot = port->custom_divisor;
		*frac = 0;
	} else {
		unsigned long   req_divisorX1000, closest_baud = 0, divisor = 16;
		unsigned long   closest_divisorX1000 = 0;
		unsigned long   closest_baud_prescale4X = 0;
		unsigned long   req_divisor_prescale4X1000;
		unsigned long   closest_divisor_prescale4X1000 = 0;

		unsigned long   lower_baud_limit_prescale4X,
				upper_baud_limit_prescale4X;
		unsigned long   upper_baud_limit_sample8X;
		
				

		unsigned char	prescaler_value = 1;

		if(baud == 0)
			closest_divisorX1000 = 0;
		else {
			// note: in order to work out some calculations here many values are worked out with a 1000x
			// factor and near the end this is removed.  This allows us to do the appropriate rounding
			// and 2% error calculations without using floating point math

			// divisor is the sample rate (typically 16)

			// according to Exar if the baud is greater than (clock/16)/2 set the samble clock (divisor) to 8
			// else set it to 16

			upper_baud_limit_prescale4X = (port->uartclk/4/16);
			lower_baud_limit_prescale4X = upper_baud_limit_prescale4X / (64*1024);
			upper_baud_limit_sample8X   = (port->uartclk/8);
			
			if ( baud < lower_baud_limit_prescale4X ) {
				//truncate to bottom end
				//should print a log message
				baud = lower_baud_limit_prescale4X;
			} else {
				//first make sure that the board can hit the baud
				if( baud > upper_baud_limit_sample8X ) {
					baud = upper_baud_limit_sample8X;
				}
			}

			if( baud > ((port->uartclk/16)/2)){
				divisor = 8;
			}
			else {
				divisor = 16;
			}

			prescaler_value = 1;

			// work out the required divisor
			//req_divisorX1000 = ((int64)(ttp->clk/divisor)*1000)/baud;
			req_divisorX1000 = (port->uartclk/divisor)/baud;
			req_divisorX1000 *= 1000;
			// note baud rates with 125Mhz clock above 7812500 will overflow the unsigned long
			// seeing as this is beyond a reasonable speed for serial comunications this is not supported
			req_divisorX1000 += (((port->uartclk/divisor)%baud)*1000)/baud;

			// if the UART supports (like the Exar 17X285X) fractional baud rate divisor make appropriate calculations

			closest_divisorX1000 = req_divisorX1000 - TruncX1000(req_divisorX1000);
			closest_divisorX1000 *= 16;
			closest_divisorX1000 = RoundX1000( closest_divisorX1000 )/16;
			closest_divisorX1000 += TruncX1000(req_divisorX1000);
			closest_divisorX1000 = DivisorBoundsFilter( closest_divisorX1000, 1 );
			//closest_baudX1000 = ((int64)(ttp->clk/ttp->divisor)*1000*1000)/closest_divisorX1000;
			closest_baud = (port->uartclk/divisor)/closest_divisorX1000;
			closest_baud *= 1000;
			closest_baud += (((port->uartclk/divisor)%closest_divisorX1000)*1000)/closest_divisorX1000;


			if(( ((AbsDiff( closest_baud, baud)*1000)/baud) > 20)
				&& ((AbsDiff( closest_baud, baud)*1000)/baud)  /* fractional support and non-zero error */
				&& ( baud < (port->uartclk/4/divisor) )) /* baud less than the max we can hit with 4x prescaler */
			{
				// work out an alternate divisor/prescaler on fractional baud support if it is not perfect (0% error)

				// not 0% error with 1x prescaler so lets try prescaler of 4
				req_divisor_prescale4X1000 = (((port->uartclk/divisor)/4)*1000)/baud;
				closest_divisor_prescale4X1000 = req_divisor_prescale4X1000 - TruncX1000(req_divisor_prescale4X1000);
				closest_divisor_prescale4X1000 *= 16;
				closest_divisor_prescale4X1000 = RoundX1000( closest_divisor_prescale4X1000 )/16;
				closest_divisor_prescale4X1000 += TruncX1000(req_divisor_prescale4X1000);
				closest_divisor_prescale4X1000 = DivisorBoundsFilter( closest_divisor_prescale4X1000, 4 );


				//closest_baud_prescale4X1000 = (((int64)(ttp->clk/divisor)/4)*1000*1000)/closest_divisor_prescale4X1000;
				closest_baud_prescale4X = ((port->uartclk/divisor)/4)/closest_divisor_prescale4X1000;
				closest_baud_prescale4X *= 1000;
				closest_baud_prescale4X += ((((port->uartclk/divisor)/4)%closest_divisor_prescale4X1000)*1000)/closest_divisor_prescale4X1000;
				// find out what prescaler works best
				if( AbsDiff( closest_baud_prescale4X, baud ) < AbsDiff( closest_baud, baud ) ){
					// prescaler of 4 had best result
					prescaler_value = 4;
					closest_divisorX1000 = closest_divisor_prescale4X1000;
					if( (AbsDiff( closest_baud_prescale4X, baud)*1000)/baud > 20 ){
						// it was the best but more thatn 2% error so log it
						printk(KERN_INFO "CTIserial: BlueStorm: set baud - no good solution for baud but going with closest at 4X(%d,%lu)\n", baud, closest_baud_prescale4X);
					}
				}
			}
			else {
				// greater than 2% error and no fractional support!
				// should return an error but there is no way to so set the closest baud
				if( (AbsDiff( closest_baud, baud )*1000)/baud > 20 )
				printk(KERN_INFO "CTIserial: BlueStorm: set baud error(%d,%lu)\n", baud, (unsigned long)closest_baud);
			}
		}

		*quot = closest_divisorX1000/1000;
		*frac = (RoundX1000((closest_divisorX1000 - TruncX1000(closest_divisorX1000))*16)/1000);
		*prescaler = prescaler_value;
		*sample_rate = (unsigned char)divisor; 
	}
}


static void serial8250_get_divisorf(struct uart_port *port, unsigned int baud, unsigned int *quot, unsigned int *frac, unsigned char *prescaler, unsigned char *sample_rate)
{
	*frac = 0;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		*quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		 baud == (port->uartclk/8))
		*quot = 0x8002;
	else {
		cti_uart_get_divisorf(port, baud, quot, frac, prescaler, sample_rate);
	}
}

static unsigned int serial8250_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		 baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = cti_uart_get_divisor(port, baud);

	return quot;
}

void
cti_serial8250_do_set_termios(struct uart_port *port, struct ktermios *termios,
		          struct ktermios *old)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot, max_baud, frac;
 	unsigned char prescaler, sample_rate;

	int fifo_bug = 0;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB) {
		cval |= UART_LCR_PARITY;
		if (up->bugs & UART_BUG_PARITY)
			fifo_bug = 1;
	}
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif

	/*
	 * Ask the core to calculate the divisor for us.
	 */
 	max_baud = (up->port.type == PORT_16C950 ? port->uartclk/4 : port->uartclk/16);
  	baud = cti_uart_get_baud_rate(port, termios, old, 0, max_baud);
 	if (up->port.use_frac_div) {
 		serial8250_get_divisorf(port, baud, &quot, &frac, &prescaler, &sample_rate);
	}
 	else {
 		quot = serial8250_get_divisor(port, baud);
	}

	/*
	 * Oxford Semi 952 rev B workaround
	 */
	if (up->bugs & UART_BUG_QUOT && (quot & 0xff) == 0)
		quot++;

	if (up->capabilities & UART_CAP_FIFO && port->fifosize > 1) {
		fcr = uart_config[port->type].fcr;
		if (baud < 2400 || fifo_bug) {
			fcr &= ~UART_FCR_TRIGGER_MASK;
			fcr |= UART_FCR_TRIGGER_1;
		}
	}

	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	if (up->capabilities & UART_CAP_AFE && port->fifosize >= 32) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&port->lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	cti_uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (!(up->bugs & UART_BUG_NOMSR) &&
			UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	if (up->capabilities & UART_CAP_UUE)
		up->ier |= UART_IER_UUE;
	if (up->capabilities & UART_CAP_RTOIE)
		up->ier |= UART_IER_RTOIE;

	serial_port_out(port, UART_IER, up->ier);

	if (up->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
		 */
		if (termios->c_cflag & CRTSCTS)
		{	
			if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X) ) 
			{
				efr |= UART_EFR_RTS | UART_EFR_CTS;
				//serial_outp(up, UART_MCR, serial_inp(up, UART_MCR)|UART_MCR_RTS);
				up->port.mctrl|=UART_MCR_RTS;
				serial8250_set_mctrl(&up->port, up->port.mctrl);
			}
			else
			{
				if((up->port.lmode != TIOCSER485HALFDUPLEX) && (up->port.lmode != TIOCSER485SLAVEMULTIPLEX))
					serial_outp(up, UART_FLAT_EFR, serial_inp(up, UART_FLAT_EFR) | (UART_EFR_RTS | UART_EFR_CTS));
			}
		}
		else
		{
			 if ( (up->port.type == PORT_XR17XX5X) || (up->port.type == PORT_XR17V35X) )
				serial_outp(up, UART_FLAT_EFR, serial_inp(up, UART_FLAT_EFR) & ~(UART_EFR_RTS | UART_EFR_CTS));
		}
		if ( (up->port.type != PORT_XR17XX5X) && (up->port.type != PORT_XR17V35X) ) 
		{
			serial_outp(up, UART_LCR, 0xBF);
			serial_outp(up, UART_EFR, efr);
			serial_outp(up, UART_LCR, 0);
		}
	}

	/* Workaround to enable 115200 baud on OMAP1510 internal ports */
	if (is_omap1510_8250(up)) {
		if (baud == 115200) {
			quot = 1;
			serial_port_out(port, UART_OMAP_OSC_12M_SEL, 1);
		} else
			serial_port_out(port, UART_OMAP_OSC_12M_SEL, 0);
	}

	/*
	 * For NatSemi, switch to bank 2 not bank 1, to avoid resetting EXCR2,
	 * otherwise just set DLAB
	 */
	if (up->capabilities & UART_NATSEMI)
		serial_port_out(port, UART_LCR, 0xe0);
	else
		serial_port_out(port, UART_LCR, cval | UART_LCR_DLAB);

	if (up->port.use_frac_div)
		serial_dlf_write(up, quot, frac,prescaler,sample_rate);
	else
		serial_dl_write(up, quot);

	/*
	 * LCR DLAB must be set to enable 64-byte FIFO mode. If the FCR
	 * is written without DLAB set, this mode will be disabled.
	 */
	if (port->type == PORT_16750)
		serial_port_out(port, UART_FCR, fcr);

	serial_port_out(port, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	if (port->type != PORT_16750) {
		/* emulated UARTs (Lucent Venus 167x) need two steps */
		if (fcr & UART_FCR_ENABLE_FIFO)
			serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_port_out(port, UART_FCR, fcr);		/* set fcr */
	}
	serial8250_set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}
EXPORT_SYMBOL(cti_serial8250_do_set_termios);

static void
serial8250_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	if (port->set_termios)
		port->set_termios(port, termios, old);
	else
		cti_serial8250_do_set_termios(port, termios, old);
}

static void
serial8250_set_ldisc(struct uart_port *port, int new)
{
	if (new == N_PPS) {
		port->flags |= UPF_HARDPPS_CD;
		serial8250_enable_ms(port);
	} else
		port->flags &= ~UPF_HARDPPS_CD;
}


void cti_serial8250_do_pm(struct uart_port *port, unsigned int state,
		      unsigned int oldstate)
{
	struct uart_8250_port *p =
		container_of(port, struct uart_8250_port, port);

	serial8250_set_sleep(p, state != 0);
}
EXPORT_SYMBOL(cti_serial8250_do_pm);

static void
serial8250_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	if (port->pm)
		port->pm(port, state, oldstate);
	else
		cti_serial8250_do_pm(port, state, oldstate);
}

static unsigned int serial8250_port_size(struct uart_8250_port *pt)
{
	if (pt->port.iotype == UPIO_AU)
		return 0x1000;
	if (is_omap1_8250(pt))
		return 0x16 << pt->port.regshift;

	return 8 << pt->port.regshift;
}

/*
 * Resource handling.
 */
static int serial8250_request_std_resource(struct uart_8250_port *up)
{
	unsigned int size = serial8250_port_size(up);
	struct uart_port *port = &up->port;
	int ret = 0;

	switch (port->iotype) {
	case UPIO_AU:
	case UPIO_TSI:
	case UPIO_MEM32:
	case UPIO_MEM:
	case UPIO_DWAPB:
	case UPIO_DWAPB32:
		if (!port->mapbase)
			break;

		if (!request_mem_region(port->mapbase, size, "serial")) {
			ret = -EBUSY;
			break;
		}

		if (port->flags & UPF_IOREMAP) {
			port->membase = ioremap_nocache(port->mapbase, size);
			if (!port->membase) {
				release_mem_region(port->mapbase, size);
				ret = -ENOMEM;
			}
		}
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		if (!request_region(port->iobase, size, "serial")){
			ret = -EBUSY;
        }
		break;
	}
	return ret;
}

static void serial8250_release_std_resource(struct uart_8250_port *up)
{
	unsigned int size = serial8250_port_size(up);
	struct uart_port *port = &up->port;

	switch (port->iotype) {
	case UPIO_AU:
	case UPIO_TSI:
	case UPIO_MEM32:
	case UPIO_MEM:
	case UPIO_DWAPB:
	case UPIO_DWAPB32:
		if (!port->mapbase)
			break;

		if (port->flags & UPF_IOREMAP) {
			iounmap(port->membase);
			port->membase = NULL;
		}

		release_mem_region(port->mapbase, size);
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		release_region(port->iobase, size);
		break;
	}
}

static int serial8250_request_rsa_resource(struct uart_8250_port *up)
{
	unsigned long start = UART_RSA_BASE << up->port.regshift;
	unsigned int size = 8 << up->port.regshift;
	struct uart_port *port = &up->port;
	int ret = -EINVAL;

	switch (port->iotype) {
	case UPIO_HUB6:
	case UPIO_PORT:
		start += port->iobase;
		if (request_region(start, size, "serial-rsa"))
			ret = 0;
		else
			ret = -EBUSY;
		break;
	}

	return ret;
}

static void serial8250_release_rsa_resource(struct uart_8250_port *up)
{
	unsigned long offset = UART_RSA_BASE << up->port.regshift;
	unsigned int size = 8 << up->port.regshift;
	struct uart_port *port = &up->port;

	switch (port->iotype) {
	case UPIO_HUB6:
	case UPIO_PORT:
		release_region(port->iobase + offset, size);
		break;
	}
}

static void serial8250_release_port(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	serial8250_release_std_resource(up);
	if (port->type == PORT_RSA)
		serial8250_release_rsa_resource(up);
}

static int serial8250_request_port(struct uart_port *port)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	int ret;

	if (port->type == PORT_8250_CIR)
		return -ENODEV;

	ret = serial8250_request_std_resource(up);
	if (ret == 0 && port->type == PORT_RSA) {
		ret = serial8250_request_rsa_resource(up);
		if (ret < 0)
			serial8250_release_std_resource(up);
	}

	return ret;
}

static void serial8250_config_port(struct uart_port *port, int flags)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);
	int probeflags = PROBE_ANY;
	int ret;


	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = serial8250_request_std_resource(up);
	if (ret < 0)
		return;

	if (port->iotype != up->cur_iotype)
		set_io_from_upio(port);
  
	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);
	else
    {
		/* we already have been told the type */
		printk(KERN_INFO "CTISerial: skipping autoconfig (type=%u)\n", up->port.type);
		up->port.fifosize = uart_config[up->port.type].fifo_size;
		up->capabilities = uart_config[up->port.type].flags;
		up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

    }

	/* if access method is AU, it is a 16550 with a quirk */
	//if (port->type == PORT_16550A && port->iotype == UPIO_AU)
	//	up->bugs |= UART_BUG_NOMSR;

	if (port->type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(up);

	if (port->type == PORT_UNKNOWN)
		serial8250_release_std_resource(up);

	/* Fixme: probably not the best place for this */
	/*if ((port->type == PORT_XR17V35X))
		port->handle_irq = exar_handle_irq;*/
}

static int
serial8250_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= nr_irqs || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
	    ser->type >= ARRAY_SIZE(uart_config) || ser->type == PORT_CIRRUS ||
	    ser->type == PORT_STARTECH)
		return -EINVAL;
	return 0;
}

static const char *
serial8250_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static unsigned int serial_asr_read(struct uart_8250_port *up, int offset);
void dump_uart(struct uart_port *port)
{
        unsigned char lcr;
        struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
        struct tty_port *tport = &port->state->port;
    	struct tty_struct *tty = tport->tty;
	while(serial_inp(up, UART_LSR) & 0x01)
	printk("rx %c",serial_inp(up,0));
        lcr = serial_inp(up, UART_LCR);
        serial_outp(up, UART_LCR, lcr | 0x80);
        printk(KERN_INFO "DLL   0x%02X\n",serial_inp(up, UART_DLL));
        printk(KERN_INFO "DLM   0x%02X\n",serial_inp(up, UART_DLM));
        serial_outp(up, UART_LCR, lcr & ~0x80);
        printk(KERN_INFO "IER   0x%02X\n",serial_inp(up, UART_IER));
        printk(KERN_INFO "ISR   0x%02X\n",serial_inp(up, UART_IIR));
        printk(KERN_INFO "FCR   0x%02X\n",serial_inp(up, UART_FCR));
        serial_outp(up, UART_LCR, lcr);
        printk(KERN_INFO "LCR   0x%02X\n",serial_inp(up, UART_LCR));
        printk(KERN_INFO "MCR   0x%02X\n",serial_inp(up, UART_MCR));
        printk(KERN_INFO "LSR   0x%02X\n",serial_inp(up, UART_LSR));
        printk(KERN_INFO "MSR   0x%02X\n",serial_inp(up, UART_MSR));
        printk(KERN_INFO "SPR   0x%02X\n",serial_inp(up, UART_SCR));
        printk(KERN_INFO "FCTR  0x%02X\n",serial_inp(up, UART_FLAT_FCTR));
        serial_outp(up, UART_LCR, 0xBF);
        printk(KERN_INFO "EFR   0x%02X\n",serial_inp(up, UART_EFR));
        serial_outp(up, UART_LCR, lcr );
        printk(KERN_INFO "FLAT_EFR   0x%02X\n",serial_inp(up, UART_FLAT_EFR));
	if(up->port.type == PORT_16C950)
	{
        	printk(KERN_INFO "TXCNT 0x%02X\n", serial_asr_read( up, UART_TFL ));
	        printk(KERN_INFO "RXCNT 0x%02X\n", serial_asr_read( up, UART_RFL ));
	}
	else
	{
        	printk(KERN_INFO "TXCNT 0x%02X\n",serial_inp(up, UART_FLAT_TXCNT));
	        printk(KERN_INFO "RXCNT 0x%02X\n",serial_inp(up, UART_FLAT_RXCNT));
	}
	printk(KERN_INFO "tty->hw_stopped %d tty->stopped %d\n",tty->hw_stopped, tty->stopped);
}


static int
serial8250_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
	switch (cmd) {
		case TIOCSER485GET:
		case TIOCSER485SET:
			if (port->lmode_fn)
				return (port->lmode_fn)(port, cmd, (unsigned int *)arg);
			break;
		case TIOCSOFTAUTO485:
			return do_softauto485(up, (int *)arg);
		case TIOCSENXTREME:
			ret = 0;
			up->use_xtreme = 1;
			break;
		case TIOCSDISMSRINT:
			ret = 0;
			if(up->disable_msr_int)
				up->disable_msr_int = 0;
			else
				up->disable_msr_int = 1;
			break;
		case TIOCSDUMPUART:
			if ( (up->port.type == PORT_XR17XX5X) || (up->port.type == PORT_XR17V35X) ||  (up->port.type == PORT_16C950)) 
			{
				ret = 0;
				dump_uart(port);
			}
			break;
		case TIOCSERPTMGET:
		case TIOCSERPTMSET:
			if (port->ptm_fn)
				return (port->ptm_fn)(port, cmd,
                                                       (unsigned int *)arg);
			/* fall through */
		default:
			break;
	}

	return ret;
}


static struct uart_ops serial8250_pops = {
	.tx_empty	= serial8250_tx_empty,
	.set_mctrl	= serial8250_set_mctrl,
	.get_mctrl	= serial8250_get_mctrl,
	.stop_tx	= serial8250_stop_tx,
	.start_tx	= serial8250_start_tx,
	.stop_rx	= serial8250_stop_rx,
	.enable_ms	= serial8250_enable_ms,
	.break_ctl	= serial8250_break_ctl,
	.startup	= serial8250_startup,
	.shutdown	= serial8250_shutdown,
	.set_termios	= serial8250_set_termios,
	.set_ldisc	= serial8250_set_ldisc,
	.pm		= serial8250_pm,
	.type		= serial8250_type,
	.release_port	= serial8250_release_port,
	.request_port	= serial8250_request_port,
	.config_port	= serial8250_config_port,
	.verify_port	= serial8250_verify_port,
	.ioctl		= serial8250_ioctl,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = serial8250_get_poll_char,
	.poll_put_char = serial8250_put_poll_char,
#endif
};

static struct uart_8250_port serial8250_ports[MAX_PORTS];

static void (*serial8250_isa_config)(int port, struct uart_port *up,
	unsigned short *capabilities);

void cti_serial8250_set_isa_configurator(
	void (*v)(int port, struct uart_port *up, unsigned short *capabilities))
{
	serial8250_isa_config = v;
}
EXPORT_SYMBOL(cti_serial8250_set_isa_configurator);

static void __init serial8250_isa_init_ports(void)
{
	struct uart_8250_port *up;
	static int first = 1;
	int i, irqflag = 0;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];
		struct uart_port *port = &up->port;

		port->line = i;
		spin_lock_init(&port->lock);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0))
		init_timer(&up->timer);
		up->timer.function = serial8250_timeout;
#else
		timer_setup(&up->timer,serial8250_timeout,0);
#endif
		up->cur_iotype = 0xFF;

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		up->mcr_force = ALPHA_KLUDGE_MCR;

		port->ops = &serial8250_pops;
        tasklet_init(&up->tlet, SHRE_poll, (unsigned long)up);
	}

	if (share_irqs)
		irqflag = IRQF_SHARED;

	for (i = 0, up = serial8250_ports; i < nr_uarts; i++, up++) {
		struct uart_port *port = &up->port;

		port->iobase   = old_serial_port.port;
		port->irq      = irq_canonicalize(old_serial_port.irq);
		port->irqflags = old_serial_port.irqflags;
		port->uartclk  = old_serial_port.baud_base * 16;
		port->flags    = old_serial_port.flags;
		port->hub6     = old_serial_port.hub6;
		port->membase  = old_serial_port.iomem_base;
		port->iotype   = old_serial_port.io_type;
		port->regshift = old_serial_port.iomem_reg_shift;
		set_io_from_upio(port);
		port->irqflags |= irqflag;
		if (serial8250_isa_config != NULL)
			serial8250_isa_config(i, &up->port, &up->capabilities);

	}
}

static void
serial8250_init_fixed_type_port(struct uart_8250_port *up, unsigned int type)
{
	up->port.type = type;
	if (!up->port.fifosize)
		up->port.fifosize = uart_config[type].fifo_size;
	if (!up->tx_loadsz)
		up->tx_loadsz = uart_config[type].tx_loadsz;
	if (!up->capabilities)
		up->capabilities = uart_config[type].flags;
}

static void __init
serial8250_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];
		up->cur_iotype = 0xFF;
	}

	serial8250_isa_init_ports();

    for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];

		if (up->port.dev)
			continue;

		up->port.dev = dev;

		if (up->port.flags & UPF_FIXED_TYPE)
			serial8250_init_fixed_type_port(up, up->port.type);

		cti_uart_add_one_port(drv, &up->port);
	}
}

/*
 * Connect Tech boards' 485 line mode support
 *
 * cti485 does the work. _2 and _4 do checking for the hybrid boards.
 * They rely on the fact that the ports will start at a 256 aligned
 * address to calculate which port on the board it is.
 */
static inline int pci_portnum_cti(struct uart_port *port)
{
	return (port->iobase % 0x100) / 0x08;
}

static int get_lmode_from_arg(unsigned int *value){
	
	int lmode;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))

	//use copy_from_user if userspace pointer is valid
	//Kernel 5.0 dropped the type argument for access_ok
	if(access_ok(VERIFY_READ, (void __user *)value, sizeof(int))){ 
		if(copy_from_user(&lmode, (void __user *)value, sizeof(int))){
			return -EFAULT;
		};
		return lmode;
	}
	else{ // not a userspace pointer
		return *value;
	}

#else
	
	//use copy_from_user if userspace pointer is valid
	if(access_ok((void __user *)value, sizeof(int))){ 
		if(copy_from_user(&lmode, (void __user *)value, sizeof(int))){
			return -EFAULT;
		};
		return lmode;
	}
	else{ // not a userspace pointer
		return *value;
	}
	
#endif
}

int pci_cti485(struct uart_port *port, int ioctl, unsigned int *value)
{
    struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
	unsigned char port_offset;
	unsigned char portmask;
	unsigned char txctl_offset;
	unsigned char rxctl_offset;
	unsigned char differ_offset;
	unsigned char bits;
	unsigned long flags;

	if (ioctl == TIOCSER485GET)
		return put_user(up->port.lmode, value);

	port_offset = up->port.iobase % 0x100;
	portmask = 1 << port_offset / 0x08;
	txctl_offset = 0x6c - port_offset;
	rxctl_offset = 0x70 - port_offset;

	spin_lock_irqsave(&up->port.lock, flags);
	if (value){
		up->port.lmode = get_lmode_from_arg(value);
	}
	else if (up->port.lmode == TIOCSER485NOT_INITED) {
		differ_offset = 0x74 - port_offset;
		bits = serial_in(up, differ_offset);
		switch (bits) {
			case 0x03:
				up->port.lmode = TIOCSER485SLAVEMULTIPLEX;
				break;
			default:
				up->port.lmode = TIOCSER485FULLDUPLEX;
				break;
		}
	}
	bits = serial_in(up, txctl_offset);
	switch (up->port.lmode) {
		default:
		case TIOCSER485FULLDUPLEX:
			bits &= ~portmask;
			serial_out(up, txctl_offset, bits);

			bits = serial_in(up, rxctl_offset);
			bits |= portmask;
			serial_out(up, rxctl_offset, bits);

			serial_out(up, UART_LCR, 0xbf);
			bits = serial_in(up, UART_FCTR);
			bits &= ~UART_FCTR_TX_INT;
			serial_out(up, UART_FCTR, bits);
			serial_out(up, UART_LCR, 0);

			spin_unlock_irqrestore(&up->port.lock, flags);
			return put_user(1, value);
		case TIOCSER485HALFDUPLEX:
			bits |= portmask;
			serial_out(up, txctl_offset, bits);

			bits = serial_in(up, rxctl_offset);
			bits &= ~portmask;
			serial_out(up, rxctl_offset, bits);

			serial_out(up, UART_LCR, 0xbf);
			bits = serial_in(up, UART_FCTR);
			bits |= UART_FCTR_TX_INT;
			serial_out(up, UART_FCTR, bits);
			serial_out(up, UART_LCR, 0);

			spin_unlock_irqrestore(&up->port.lock, flags);
			return put_user(1, value);
		case TIOCSER485SLAVEMULTIPLEX:
			bits |= portmask;
			serial_out(up, txctl_offset, bits);

			bits = serial_in(up, rxctl_offset);
			bits |= portmask;
			serial_out(up, rxctl_offset, bits);

			serial_out(up, UART_LCR, 0xbf);
			bits = serial_in(up, UART_FCTR);
			bits |= UART_FCTR_TX_INT;
			serial_out(up, UART_FCTR, bits);
			serial_out(up, UART_LCR, 0);


			spin_unlock_irqrestore(&up->port.lock, flags);
			return put_user(1, value);
	} 

	spin_unlock_irqrestore(&up->port.lock, flags);
	return -EINVAL;
}

int pci_cti485_4(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti(port) < 4)
		return -ENOSYS;

	return pci_cti485(port, ioctl, value);
}

int pci_cti485_2(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti(port) < 2)
		return -ENOSYS;

	return pci_cti485(port, ioctl, value);
}

static inline int pci_portnum_cti_bl(struct uart_port *port)
{
	return (port->mapbase % 0x1000) / 0x200;
}

static inline int pci_portnum_cti_bx(struct uart_port *port)
{
	return (port->mapbase % 0x1000) / 0x400;
}

int pci_cti485_bl(struct uart_port *port, int ioctl, unsigned int *value)
{
	struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
	unsigned char bits;
	unsigned long flags;
	int ret;
	int port_num;

	if (ioctl == TIOCSER485GET){
		return put_user(up->port.lmode, value);
	}

	if (value){
		up->port.lmode = get_lmode_from_arg(value);
	}
	else if (up->port.lmode == TIOCSER485NOT_INITED) {
		/*
		 * Some models support power-on tristate.
		 * In order to accomodate these cards, we cannot
		 * enable the transmitter until after we have been told
		 * which line mode we are supposed to use (full or half duplex).
		 * Therefore, we cannot assume anything and have to wait for
		 * the user to send the appropriate IOCTL.
		 */
		if (!up->port.use_485_3t && !up->port.use_485_3t_mpio)
			up->port.lmode = TIOCSER485FULLDUPLEX;
		else {
			return -EINVAL;
		}
	}
	switch (up->port.lmode) {
		default:
		case TIOCSER485FULLDUPLEX:
			spin_lock_irqsave(&up->port.lock, flags);
			bits = serial_in(up, UART_FLAT_FCTR);
			bits &= ~UART_FLAT_FCTR_TX_INT;
			serial_out(up, UART_FLAT_FCTR, bits);

			spin_unlock_irqrestore(&up->port.lock, flags);
			ret = put_user(1, value);
			break;
		case TIOCSER485HALFDUPLEX:
		case TIOCSER485SLAVEMULTIPLEX:
			/* new rs485 RTS delay */
			spin_lock_irqsave(&up->port.lock, flags);
			serial_out(up, UART_MSR, 0x20);

			bits = serial_in(up, UART_FLAT_FCTR);
			bits |= UART_FLAT_FCTR_TX_INT;
			serial_out(up, UART_FLAT_FCTR, bits);
			
			spin_unlock_irqrestore(&up->port.lock, flags);
			ret = put_user(1, value);
			break;
	}

	if (up->port.use_485_3t_mpio) {
		int dcr_port_shift;
		dcr_port_shift = ( up->port.index_on_board >= 8 ) ? 6 : 0;
		port_num = up->port.index_on_dev;
		/* enable the transceiver
		 * MPIOSEL[port_num] = 0
		 * MPIO3T[port_num] = 0
		 * MPIOLVL[port_num] = 1
		 */
		spin_lock_irqsave(&up->port.lock, flags);
		bits = serial_dcr_in(up, dcr_port_shift + 0x13);
		serial_dcr_out(up,  dcr_port_shift + 0x13, bits & ~(1<<port_num));
		bits = serial_dcr_in(up, dcr_port_shift + 0x11);
		serial_dcr_out(up, dcr_port_shift + 0x11, bits & ~(1<<port_num));
		bits = serial_dcr_in(up, dcr_port_shift + 0x10);
		serial_dcr_out(up, dcr_port_shift + 0x10, bits | (1<<port_num));
		spin_unlock_irqrestore(&up->port.lock, flags);
	}
	return ret;
}

int pci_cti485_bl_6(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti_bl(port) < 6)
		return -ENOSYS;

	return pci_cti485_bl(port, ioctl, value);
}

int pci_cti485_bl_4(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti_bl(port) < 4)
		return -ENOSYS;

	return pci_cti485_bl(port, ioctl, value);
}

int pci_cti485_bl_2(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti_bl(port) < 2)
		return -ENOSYS;

	return pci_cti485_bl(port, ioctl, value);
}

int pci_cti485_bl_1(struct uart_port *port, int ioctl, unsigned int *value)
{
	if (pci_portnum_cti_bl(port) < 1)
		return -ENOSYS;

	return pci_cti485_bl(port, ioctl, value);
}

#if 0
#define DEBUG_LMODE(fmt...) printk(fmt)
#else
#define DEBUG_LMODE(fmt...)  do{} while(0)
#endif

int pci_cti485_tn(struct uart_port *port, int ioctl, unsigned int *value)
{
	struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
	unsigned long flags;
	int ret;
	int port_num;

	port_num = serial_icr_read( up, 0x12 );
	DEBUG_LMODE("pci_cti_485_tn(ttyCTI%d:ch%d): %x, %x\n", port->line, port_num, ioctl, (value ? *value : 0 ));

	if (ioctl == TIOCSER485GET)
		return put_user(up->port.lmode, value);

	spin_lock_irqsave(&up->port.lock, flags);
	if (value){
		up->port.lmode = get_lmode_from_arg(value);
	}
	else if (up->port.lmode == TIOCSER485NOT_INITED) {
		/*
		 * Some models support power-on tristate.
		 * In order to accomodate these cards, we cannot
		 * enable the transmitter until after we have been told
		 * which line mode we are supposed to use (full or half duplex).
		 * Therefore, we cannot assume anything and have to wait for
		 * the user to send the appropriate IOCTL.
		 */
		DEBUG_LMODE("pci_cti_485_tn(ttyCTI%d:ch%d): TIOCSER485NOT_INITED\n", port->line, port_num);
		if (!up->port.use_485_3t && !up->port.use_485_3t_mpio)
			up->port.lmode = TIOCSER485FULLDUPLEX;
		else {
			spin_unlock_irqrestore(&up->port.lock, flags);
			return -EINVAL;
		}
	}
	switch (up->port.lmode) {
		default:
		case TIOCSER485FULLDUPLEX:
			DEBUG_LMODE("pci_cti_485_tn(ttyCTI%d:ch%d): TIOCSER485FULLDUPLEX\n", port->line, port_num);
			up->acr &= ~UART_ACR_DTR_MASK;
			serial_icr_write( up, UART_ACR, up->acr );

			ret = put_user(1, value);
			break;
		case TIOCSER485HALFDUPLEX:
		case TIOCSER485SLAVEMULTIPLEX:
			DEBUG_LMODE("pci_cti_485_tn(ttyCTI%d:ch%d): TIOCSER485HALFDUPLEX || TIOCSER485SLAVEMULTIPLEX\n", port->line, port_num);
			/* new rs485 RTS delay */
			serial_icr_write( up, 0x14, 1 );    // RS485_DLYEN
			serial_icr_write( up, 0x15, 0x40 ); // RS485_DLYCNT
			
			up->acr |= UART_ACR_DTR_485_HIGH;
			serial_icr_write( up, UART_ACR, up->acr );
			
			ret = put_user(1, value);
			break;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);
	return ret;
}

EXPORT_SYMBOL(pci_cti485);
EXPORT_SYMBOL(pci_cti485_4);
EXPORT_SYMBOL(pci_cti485_2);
EXPORT_SYMBOL(pci_cti485_bl);
EXPORT_SYMBOL(pci_cti485_bl_6);
EXPORT_SYMBOL(pci_cti485_bl_4);
EXPORT_SYMBOL(pci_cti485_bl_2);
EXPORT_SYMBOL(pci_cti485_bl_1);
EXPORT_SYMBOL(pci_cti485_tn);

/*
 * Connect Tech boards' PTM support
 */
int pci_ctiptm(struct uart_port *port, int ioctl, unsigned int *value)
{
	int ret = -EINVAL;
	struct uart_8250_port *up = container_of(port, struct uart_8250_port, port);
	unsigned char port_offset;
	struct ptm_info_t ptm_info;
	unsigned char ptm_info_reg;

	if(up) {
		port_offset = port->iobase % 0x100;

		if (ioctl == TIOCSERPTMGET) {
			ptm_info_reg = serial_in(up, 0x78 - port_offset);
			ptm_info.mode = (ptm_info_reg & 0x80) ?
				TIOCSERPTM_MODE_PC : TIOCSERPTM_MODE_PT;
			ptm_info.timeout = (ptm_info_reg & 0x3) * 10 + 3;
			ptm_info.relay = (ptm_info_reg & 0x40) ?
				TIOCSERPTM_RELAY_PC : TIOCSERPTM_RELAY_PT;

			if(copy_to_user((void *)value, &ptm_info,
							 sizeof(ptm_info)))
				ret = -EFAULT;
			else
				ret = 0;
		} else if (ioctl == TIOCSERPTMSET) {
			if(copy_from_user(&ptm_info, (void *)value,
							 sizeof(ptm_info)))
				ret = -EFAULT;
			else {
				switch (ptm_info.timeout) {
				case TIOCSERPTM_TIMEOUT_3:
				case TIOCSERPTM_TIMEOUT_13:
				case TIOCSERPTM_TIMEOUT_23:
				case TIOCSERPTM_TIMEOUT_33:
					ptm_info_reg =
						(ptm_info.timeout - 3) / 10;
					break;
				default:
					goto DONE;
				}
				switch (ptm_info.mode) {
				case TIOCSERPTM_MODE_PC:
					ptm_info_reg |= (1 << 7);
				case TIOCSERPTM_MODE_PT:
					break;
				default:
					goto DONE;
				}
				serial_out(up, 0x78 - port_offset,
							ptm_info_reg);
				ret = 0;
			}
		}
	}

DONE:
	return ret;
}

EXPORT_SYMBOL(pci_ctiptm);

#ifdef CONFIG_SERIAL_8250_CONSOLE

static void serial8250_console_putchar(struct uart_port *port, int ch)
{
	struct uart_8250_port *up =
		container_of(port, struct uart_8250_port, port);

	wait_for_xmitr(up, UART_LSR_THRE);
	serial_port_out(port, UART_TX, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial8250_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_8250_port *up = &serial8250_ports[co->index];
	struct uart_port *port = &up->port;
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	touch_nmi_watchdog();

	local_irq_save(flags);
	if (port->sysrq) {
		/* cti_serial8250_handle_irq() already took the lock */
		locked = 0;
	} else if (oops_in_progress) {
		locked = spin_trylock(&port->lock);
	} else
		spin_lock(&port->lock);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_port_in(port, UART_IER);

	if (up->capabilities & UART_CAP_UUE)
		serial_port_out(port, UART_IER, UART_IER_UUE);
	else
		serial_port_out(port, UART_IER, 0);

	cti_uart_console_write(port, s, count, serial8250_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up, BOTH_EMPTY);
	serial_port_out(port, UART_IER, ier);

	/*
	 *	The receive handling will happen properly because the
	 *	receive ready bit will still be set; it is not cleared
	 *	on read.  However, modem control will not, we must
	 *	call it if we have saved something in the saved flags
	 *	while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		cti_serial8250_modem_status(up);

	if (locked)
		spin_unlock(&port->lock);
	local_irq_restore(flags);
}

static int __init serial8250_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= nr_uarts)
		co->index = 0;
	port = &serial8250_ports[co->index].port;
	if (!port->iobase && !port->membase)
		return -ENODEV;

	if (options)
		cti_uart_parse_options(options, &baud, &parity, &bits, &flow);

	return cti_uart_set_options(port, co, baud, parity, bits, flow);
}

//static int serial8250_console_early_setup(void)
//{
//	return serial8250_find_port_for_earlycon();
//}

static struct console serial8250_console = {
	.name		= "ttyCTI",
	.write		= serial8250_console_write,
//	.device		= uart_console_device,
	.setup		= serial8250_console_setup,
//	.early_setup	= serial8250_console_early_setup,
	.flags		= CON_PRINTBUFFER | CON_ANYTIME,
	.index		= -1,
	.data		= &serial8250_reg,
};

//static int __init serial8250_console_init(void)
//{
//	serial8250_isa_init_ports();
//	register_console(&serial8250_console);
//	return 0;
//}
//console_initcall(serial8250_console_init);

int serial8250_find_port(struct uart_port *p)
{
	int line;
	struct uart_port *port;

	for (line = 0; line < nr_uarts; line++) {
		port = &serial8250_ports[line].port;
		if (cti_uart_match_port(p, port))
			return line;
	}
	return -ENODEV;
}

#define SERIAL8250_CONSOLE	&serial8250_console
#else
#define SERIAL8250_CONSOLE	NULL
#endif

#define CTI_TTY_MAJOR 234
static int cti_tty_major = CTI_TTY_MAJOR;

static struct uart_driver serial8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "ctipu",
	.dev_name		= "ttyCTI",
	.major			= CTI_TTY_MAJOR,
	.minor			= CTI_TTY_MINOR,
	.cons			= SERIAL8250_CONSOLE,
};

/*
 * early_serial_setup - early registration for 8250 ports
 *
 * Setup an 8250 port structure prior to console initialisation.  Use
 * after console initialisation will cause undefined behaviour.
 */
int __init early_serial_setup(struct uart_port *port)
{
	struct uart_port *p;

	if (port->line >= ARRAY_SIZE(serial8250_ports))
		return -ENODEV;

	serial8250_isa_init_ports();
	p = &serial8250_ports[port->line].port;
	p->iobase       = port->iobase;
	p->membase      = port->membase;
	p->irq          = port->irq;
	p->irqflags     = port->irqflags;
	p->uartclk      = port->uartclk;
	p->fifosize     = port->fifosize;
	p->regshift     = port->regshift;
	p->iotype       = port->iotype;
	p->flags        = port->flags;
	p->mapbase      = port->mapbase;
	p->private_data = port->private_data;
	p->type		= port->type;
	p->line		= port->line;

	set_io_from_upio(p);
	if (port->serial_in)
		p->serial_in = port->serial_in;
	if (port->serial_out)
		p->serial_out = port->serial_out;
	if (port->handle_irq)
		p->handle_irq = port->handle_irq;
	else
		p->handle_irq = serial8250_default_handle_irq;

	return 0;
}

/**
 *	cti_serial8250_suspend_port - suspend one serial port
 *	@line:  serial line number
 *
 *	Suspend one serial port.
 */
void cti_serial8250_suspend_port(int line)
{
	cti_uart_suspend_port(&serial8250_reg, &serial8250_ports[line].port);
}

/**
 *	cti_serial8250_resume_port - resume one serial port
 *	@line:  serial line number
 *
 *	Resume one serial port.
 */
void cti_serial8250_resume_port(int line)
{
	struct uart_8250_port *up = &serial8250_ports[line];
	struct uart_port *port = &up->port;

	if (up->capabilities & UART_NATSEMI) {
		/* Ensure it's still in high speed mode */
		serial_port_out(port, UART_LCR, 0xE0);

		ns16550a_goto_highspeed(up);

		serial_port_out(port, UART_LCR, 0);
		port->uartclk = 921600*16;
	}
	cti_uart_resume_port(&serial8250_reg, port);
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int serial8250_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev->dev.platform_data;
	struct uart_8250_port uart;
	int ret, i, irqflag = 0;

	memset(&uart, 0, sizeof(uart));

	if (share_irqs)
		irqflag = IRQF_SHARED;

	for (i = 0; p && p->flags != 0; p++, i++) {
		uart.port.iobase	= p->iobase;
		uart.port.membase	= p->membase;
		uart.port.irq		= p->irq;
		uart.port.irqflags	= p->irqflags;
		uart.port.uartclk	= p->uartclk;
		uart.port.regshift	= p->regshift;
		uart.port.iotype	= p->iotype;
		uart.port.flags		= p->flags;
		uart.port.mapbase	= p->mapbase;
		uart.port.hub6		= p->hub6;
		uart.port.private_data	= p->private_data;
		uart.port.type		= p->type;
		uart.port.serial_in	= p->serial_in;
		uart.port.serial_out	= p->serial_out;
		uart.port.handle_irq	= p->handle_irq;
		uart.port.handle_break	= p->handle_break;
		uart.port.set_termios	= p->set_termios;
		uart.port.pm		= p->pm;
		uart.port.dev		= &dev->dev;
		uart.port.irqflags	|= irqflag;
		ret = cti_serial8250_register_8250_port(&uart);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%llx IRQ%d): %d\n", i,
				p->iobase, (unsigned long long)p->mapbase,
				p->irq, ret);
		}
	}
	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int serial8250_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];

		if (up->port.dev == &dev->dev)
			cti_serial8250_unregister_port(i);
	}
	return 0;
}

static int serial8250_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			cti_uart_suspend_port(&serial8250_reg, &up->port);
	}

	return 0;
}

static int serial8250_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			cti_serial8250_resume_port(i);
	}

	return 0;
}

static struct platform_driver serial8250_isa_driver = {
	.probe		= serial8250_probe,
	.remove		= serial8250_remove,
	.suspend	= serial8250_suspend,
	.resume		= serial8250_resume,
	.driver		= {
		.name	= "ctiserial8250",
		.owner	= THIS_MODULE,
	},
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
static struct platform_device *serial8250_isa_devs;

/*
 * cti_serial8250_register_8250_port and cti_serial8250_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DEFINE_MUTEX(serial_mutex);

static struct uart_8250_port *serial8250_find_match_or_unused(struct uart_port *port)
{
	int i;
	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (cti_uart_match_port(&serial8250_ports[i].port, port))
			return &serial8250_ports[i];
	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < nr_uarts; i++)
		if (serial8250_ports[i].port.type == PORT_UNKNOWN &&
		    serial8250_ports[i].port.iobase == 0){
			return &serial8250_ports[i];
		}
	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < nr_uarts; i++)
		if (serial8250_ports[i].port.type == PORT_UNKNOWN)
			return &serial8250_ports[i];
	return NULL;
}

/**
 *	cti_serial8250_register_8250_port - register a serial port
 *	@up: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int cti_serial8250_register_8250_port(struct uart_8250_port *up)
{
	struct uart_8250_port *uart;
	int ret = -ENOSPC;
	if (up->port.uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);
	
	uart = serial8250_find_match_or_unused(&up->port);
	if (uart && uart->port.type != PORT_8250_CIR) {
		if (uart->port.dev)
			cti_uart_remove_one_port(&serial8250_reg, &uart->port);
		
		uart->port.type         = up->port.type;
		uart->port.iobase       = up->port.iobase;
		uart->port.membase      = up->port.membase;
		uart->port.irq          = up->port.irq;
		uart->port.irqflags     = up->port.irqflags;
		uart->port.uartclk      = up->port.uartclk;
		uart->port.fifosize     = up->port.fifosize;
		uart->port.regshift     = up->port.regshift;
		uart->port.iotype       = up->port.iotype;
		uart->port.flags        = up->port.flags | UPF_BOOT_AUTOCONF;
		uart->bugs		= up->bugs;
		uart->port.mapbase      = up->port.mapbase;
		uart->port.private_data = up->port.private_data;
		uart->tx_loadsz		= up->tx_loadsz;
		uart->capabilities	= up->capabilities;
		if (up->port.dev)
			uart->port.dev = up->port.dev;


        /* Take tx_loadsz from fifosize if it wasn't set separately */
        if (uart->port.fifosize && !uart->tx_loadsz)
            uart->tx_loadsz = uart->port.fifosize;

		

		uart->port.has_dcr         = up->port.has_dcr;
		uart->port.dcr_iobase      = up->port.dcr_iobase;
		uart->port.dcr_membase     = up->port.dcr_membase;
		uart->port.use_plx_fix     = up->port.use_plx_fix;
		uart->port.plx_fix_applied = 0;
		uart->port.use_frac_div    = up->port.use_frac_div;
		uart->port.index_on_board  = up->port.index_on_board;
		uart->port.index_on_dev    = up->port.index_on_dev;
		uart->port.pci_dev_id      = up->port.pci_dev_id;
       
		set_io_from_upio(&uart->port);
        if(uart->port.use_plx_fix) {
			
			/* config MPIO[0] as an output
			 * MPIOSEL[0] = 0
			 */
			serial_dcr_out(uart, 0x13, 0xfe);
			/* ensure MPIO[0] is not tristated
			 * MPIO3T[0] = 0
			 */
			serial_dcr_out(uart, 0x11, 0xfe);
			/* enable interrupt generation
			 * MPIOLVL[0] = 0
			 */
			serial_dcr_out(uart, 0x10, 0xfe);
		}
		uart->port.use_485_3t       = up->port.use_485_3t;
		uart->port.use_485_3t_mpio  = up->port.use_485_3t_mpio;
		uart->port.use_clr_rts 	   = up->port.use_clr_rts;
		uart->port.use_mpio_input  = up->port.use_mpio_input;
		if (up->port.lmode_fn) {
			uart->port.lmode_fn = up->port.lmode_fn;
			uart->port.lmode_fn((struct uart_port *)uart,
							TIOCSER485SET, NULL);
		}
		if (up->port.ptm_fn)
			uart->port.ptm_fn = up->port.ptm_fn;
		
		if (up->port.flags & UPF_FIXED_TYPE)
			serial8250_init_fixed_type_port(uart, up->port.type);

		/* Possibly override default I/O functions.  */
	/*	if (up->port.serial_in)
			uart->port.serial_in = up->port.serial_in;
		if (up->port.serial_out)
			uart->port.serial_out = up->port.serial_out;
		*/
		if (up->port.handle_irq)
			uart->port.handle_irq = up->port.handle_irq;
		/*  Possibly override set_termios call */
		if (up->port.set_termios)
			uart->port.set_termios = up->port.set_termios;
		if (up->port.pm)
			uart->port.pm = up->port.pm;
		if (up->port.handle_break)
			uart->port.handle_break = up->port.handle_break;
		if (up->dl_read)
			uart->dl_read = up->dl_read;
		if (up->dl_write)
			uart->dl_write = up->dl_write;
		if (up->dma)
			uart->dma = up->dma;

		if (serial8250_isa_config != NULL)
			serial8250_isa_config(0, &uart->port,
					&uart->capabilities);

		ret = cti_uart_add_one_port(&serial8250_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;
			
		check_mpio(uart);
	}
	mutex_unlock(&serial_mutex);

	return ret;
}
EXPORT_SYMBOL(cti_serial8250_register_8250_port);

/**
 *	cti_serial8250_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void cti_serial8250_unregister_port(int line)
{
	struct uart_8250_port *uart = &serial8250_ports[line];

	mutex_lock(&serial_mutex);
	cti_uart_remove_one_port(&serial8250_reg, &uart->port);
	if (serial8250_isa_devs) {
		uart->port.flags &= ~UPF_BOOT_AUTOCONF;
		uart->port.type = PORT_UNKNOWN;
		uart->port.dev = &serial8250_isa_devs->dev;
		uart->capabilities = uart_config[uart->port.type].flags;
		cti_uart_add_one_port(&serial8250_reg, &uart->port);
	} else {
		uart->port.dev = NULL;
	}
	mutex_unlock(&serial_mutex);
}
EXPORT_SYMBOL(cti_serial8250_unregister_port);

static int __init serial8250_init(void)
{
	int ret;

	if (nr_uarts > MAX_PORTS)
		nr_uarts = MAX_PORTS;

	printk(KERN_INFO "CTISerial: 8250/16550 driver, "
		"%d ports, IRQ sharing %sabled\n", nr_uarts,
		share_irqs ? "en" : "dis");

	if(disable_msr_int)
		printk(KERN_INFO "MSR interrupt disabled\n");

	serial8250_reg.nr = nr_uarts;
    serial8250_reg.major = cti_tty_major;
	ret = cti_uart_register_driver(&serial8250_reg);

	if (ret)
		goto out;

	serial8250_isa_devs = platform_device_alloc("ctiserial8250",
						    PLAT8250_DEV_LEGACY);
	if (!serial8250_isa_devs) {
		ret = -ENOMEM;
		goto unreg_uart_drv;
	}

	ret = platform_device_add(serial8250_isa_devs);
	if (ret)
		goto put_dev;

	serial8250_register_ports(&serial8250_reg, &serial8250_isa_devs->dev);

	ret = platform_driver_register(&serial8250_isa_driver);
	if (ret == 0)
		goto out;

	platform_device_del(serial8250_isa_devs);
put_dev:
	platform_device_put(serial8250_isa_devs);
unreg_uart_drv:
cti_uart_unregister_driver(&serial8250_reg);
out:
	return ret;
}

static void __exit serial8250_exit(void)
{
	struct platform_device *isa_dev = serial8250_isa_devs;
    int i;

	/*
	 * This tells cti_serial8250_unregister_port() not to re-register
	 * the ports (thereby making serial8250_isa_driver permanently
	 * in use.)
	 */
	serial8250_isa_devs = NULL;

	platform_driver_unregister(&serial8250_isa_driver);
	platform_device_unregister(isa_dev);

   	for (i = 0; i < nr_uarts; i++) {
		struct uart_8250_port *up = &serial8250_ports[i];
		if(up->use_xtreme) {
			/*
			 * Need to take the uart out of sleep mode, otherwise autoconfig
			 * will tend to break if this module is reloaded. Also should
			 * turn off EFR's ECB bit, as we've seen issues with the clock
			 * select bit and autoconfig in the past.
			 */
			serial8250_set_sleep(up, 0);
			if (up->capabilities & UART_CAP_EFR) {
				serial_outp(up, UART_LCR, 0xBF);
				serial_outp(up, UART_EFR, 0);
				serial_outp(up, UART_LCR, 0);
			}
		}
	}

	serial8250_pnp_exit();

#ifdef CONFIG_SPARC
	sunserial_unregister_minors(&serial8250_reg, UART_NR);
#else
	cti_uart_unregister_driver(&serial8250_reg);
#endif
}

module_init(serial8250_init);
module_exit(serial8250_exit);

EXPORT_SYMBOL(cti_serial8250_suspend_port);
EXPORT_SYMBOL(cti_serial8250_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Connect Tech Inc. Generic 8250/16x50 serial driver");

module_param(share_irqs, uint, 0644);
MODULE_PARM_DESC(share_irqs, "Share IRQs with other non-8250/16x50 devices"
	" (unsafe)");

module_param(nr_uarts, uint, 0644);
MODULE_PARM_DESC(nr_uarts, "Maximum number of UARTs supported. (1-" __MODULE_STRING(CONFIG_SERIAL_8250_NR_UARTS) ")");

module_param(skip_txen_test, uint, 0644);
MODULE_PARM_DESC(skip_txen_test, "Skip checking for the TXEN bug at init time");

#ifdef CONFIG_SERIAL_8250_RSA
module_param_array(probe_rsa, ulong, &probe_rsa_count, 0444);
MODULE_PARM_DESC(probe_rsa, "Probe I/O ports for RSA");
#endif
MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);

#ifdef CONFIG_SERIAL_8250_DEPRECATED_OPTIONS
#ifndef MODULE
/* This module was renamed to 8250_core in 3.7.  Keep the old "8250" name
 * working as well for the module options so we don't break people.  We
 * need to keep the names identical and the convenient macros will happily
 * refuse to let us do that by failing the build with redefinition errors
 * of global variables.  So we stick them inside a dummy function to avoid
 * those conflicts.  The options still get parsed, and the redefined
 * MODULE_PARAM_PREFIX lets us keep the "8250." syntax alive.
 *
 * This is hacky.  I'm sorry.
 */
static void __used s8250_options(void)
{
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "8250_core."

	module_param_cb(share_irqs, &param_ops_uint, &share_irqs, 0644);
	module_param_cb(nr_uarts, &param_ops_uint, &nr_uarts, 0644);
	module_param_cb(skip_txen_test, &param_ops_uint, &skip_txen_test, 0644);
#ifdef CONFIG_SERIAL_8250_RSA
	__module_param_call(MODULE_PARAM_PREFIX, probe_rsa,
		&param_array_ops, .arr = &__param_arr_probe_rsa,
		0444, -1);
#endif
}
#else
MODULE_ALIAS("8250_core");
#endif
#endif


