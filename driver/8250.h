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
 */
/*	Copyright (c) 2010,13 CTI, Connect Tech Inc. All Rights Reserved.*/

#ifndef _CTI_LINUX_SERIAL_8250_H
#define _CTI_LINUX_SERIAL_8250_H

#ifndef UART_EXAR_DVID
#define UART_EXAR_DVID		0x8d	/* Device identification */
#endif

//#include <linux/serial_8250.h>
#include <linux/dmaengine.h>

/*
 * This is the platform device platform_data structure
 */
struct plat_serial8250_port {
	unsigned long	iobase;		/* io base address */
	void __iomem	*membase;	/* ioremap cookie or NULL */
	resource_size_t	mapbase;	/* resource base */
	unsigned int	irq;		/* interrupt number */
	unsigned long	irqflags;	/* request_irq flags */
	unsigned int	uartclk;	/* UART clock rate */
	void            *private_data;
	unsigned char	regshift;	/* register shift */
	unsigned char	iotype;		/* UPIO_* */
	unsigned char	hub6;
	upf_t		flags;		/* UPF_* flags */
	unsigned int	type;		/* If UPF_FIXED_TYPE */
	unsigned int	(*serial_in)(struct uart_port *, int);
	void		(*serial_out)(struct uart_port *, int, int);
	void		(*set_termios)(struct uart_port *,
			               struct ktermios *new,
			               struct ktermios *old);
    int             (*handle_irq)(struct uart_port *);
	void		(*pm)(struct uart_port *, unsigned int state,
			      unsigned old);
    void            (*handle_break)(struct uart_port *);
};


struct uart_8250_port {
	struct uart_port	port;
	struct timer_list	timer;		/* "no irq" timer */
	struct list_head	list;		/* ports on this IRQ */
	unsigned short		capabilities;	/* port capabilities */
	unsigned short		bugs;		/* port bugs */
	unsigned int		tx_loadsz;	/* transmit fifo load size */
	unsigned char		acr;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
	unsigned char		cur_iotype;	/* Running I/O type */

	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
	
    struct uart_8250_dma    *dma;

    /* 8250 specific callbacks */
    int                (*dl_read)(struct uart_8250_port *);
    void               (*dl_write)(struct uart_8250_port *, int);

	struct tasklet_struct	tlet;
	int			softauto485;
	unsigned char		saved_mcr;
	int			is_saved_mcr;
	unsigned int		use_xtreme;
	unsigned int		disable_msr_int;

	unsigned int		type;			/* port type */
	int			(*lmode_fn)(struct uart_8250_port *port, int ioctl, unsigned int *value);
	unsigned short		pci_dev_id;

};

struct uart_8250_dma {
	dma_filter_fn		fn;
	void			*rx_param;
	void			*tx_param;

	int			rx_chan_id;
	int			tx_chan_id;

	struct dma_slave_config	rxconf;
	struct dma_slave_config	txconf;

	struct dma_chan		*rxchan;
	struct dma_chan		*txchan;

	dma_addr_t		rx_addr;
	dma_addr_t		tx_addr;

	dma_cookie_t		rx_cookie;
	dma_cookie_t		tx_cookie;

	void			*rx_buf;

	size_t			rx_size;
	size_t			tx_size;

	unsigned char		tx_running:1;
};

struct old_serial_port {
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
};

struct serial8250_config {
	const char	*name;
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned int	flags;
};

/*
 * Allocate 8250 platform device IDs.  Nothing is implied by
 * the numbering here, except for the legacy entry being -1.
 */
enum {
	PLAT8250_DEV_LEGACY = -1,
	PLAT8250_DEV_PLATFORM,
	PLAT8250_DEV_PLATFORM1,
	PLAT8250_DEV_PLATFORM2,
	PLAT8250_DEV_FOURPORT,
	PLAT8250_DEV_ACCENT,
	PLAT8250_DEV_BOCA,
	PLAT8250_DEV_EXAR_ST16C554,
	PLAT8250_DEV_HUB6,
	PLAT8250_DEV_MCA,
	PLAT8250_DEV_AU1X00,
	PLAT8250_DEV_SM501,
};


#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */
#define UART_CAP_RTOIE	(1 << 13)	/* UART needs IER bit 4 set (Xscale, Tegra) */
#define UART_CAP_HFIFO	(1 << 14)	/* UART has a "hidden" FIFO */

#define UART_BUG_QUOT	(1 << 0)	/* UART has buggy quot LSB */
#define UART_BUG_TXEN	(1 << 1)	/* UART has buggy TX IIR status */
#define UART_BUG_NOMSR	(1 << 2)	/* UART has buggy MSR status bits (Au1x00) */
#define UART_BUG_THRE	(1 << 3)	/* UART has buggy THRE reassertion */
#define UART_BUG_PARITY	(1 << 4)	/* UART mishandles parity if FIFO enabled */

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)

#ifdef CONFIG_SERIAL_8250_SHARE_IRQ
#define SERIAL8250_SHARE_IRQS 1
#else
#define SERIAL8250_SHARE_IRQS 0
#endif

static inline int serial_in(struct uart_8250_port *up, int offset)
{
        printk("up=%p\n",up);	
	return up->port.serial_in(&up->port, offset);
}

static inline void serial_out(struct uart_8250_port *up, int offset, int value)
{
	up->port.serial_out(&up->port, offset, value);
}

void cti_serial8250_clear_and_reinit_fifos(struct uart_8250_port *p);

static inline int serial_dl_read(struct uart_8250_port *up)
{
	return up->dl_read(up);
}

static inline void serial_dl_write(struct uart_8250_port *up, int value)
{
	up->dl_write(up, value);
}

#if defined(__alpha__) && !defined(CONFIG_PCI)
/*
 * Digital did something really horribly wrong with the OUT1 and OUT2
 * lines on at least some ALPHA's.  The failure mode is that if either
 * is cleared, the machine locks up with endless interrupts.
 */
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#elif defined(CONFIG_SBC8560)
/*
 * WindRiver did something similarly broken on their SBC8560 board. The
 * UART tristates its IRQ output while OUT2 is clear, but they pulled
 * the interrupt line _up_ instead of down, so if we register the IRQ
 * while the UART is in that state, we die in an IRQ storm. */
#define ALPHA_KLUDGE_MCR (UART_MCR_OUT2)
#else
#define ALPHA_KLUDGE_MCR 0
#endif

#ifdef CONFIG_SERIAL_8250_PNP
int serial8250_pnp_init(void);
void serial8250_pnp_exit(void);
#else
static inline int serial8250_pnp_init(void) { return 0; }
static inline void serial8250_pnp_exit(void) { }
#endif

#ifdef CONFIG_ARCH_OMAP1
static inline int is_omap1_8250(struct uart_8250_port *pt)
{
	int res;

	switch (pt->port.mapbase) {
	case OMAP1_UART1_BASE:
	case OMAP1_UART2_BASE:
	case OMAP1_UART3_BASE:
		res = 1;
		break;
	default:
		res = 0;
		break;
	}

	return res;
}

static inline int is_omap1510_8250(struct uart_8250_port *pt)
{
	if (!cpu_is_omap1510())
		return 0;

	return is_omap1_8250(pt);
}
#else
static inline int is_omap1_8250(struct uart_8250_port *pt)
{
	return 0;
}
static inline int is_omap1510_8250(struct uart_8250_port *pt)
{
	return 0;
}
#endif

static inline int serial8250_tx_dma(struct uart_8250_port *p)
{
	return -1;
}
static inline int serial8250_rx_dma(struct uart_8250_port *p, unsigned int iir)
{
	return -1;
}
static inline int serial8250_request_dma(struct uart_8250_port *p)
{
	return -1;
}
static inline void serial8250_release_dma(struct uart_8250_port *p) { }

#endif /* _CTI_LINUX_SERIAL_8250_H */
