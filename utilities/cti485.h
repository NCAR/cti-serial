/*
*      cti485.h -- CTI 485 IOCTLs
*
*      Copyright (C) 2010  Connect Tech Inc.
*
*      This program is free software; you can redistribute it and/or modify
*      it under the terms of the GNU General Public License as published by
*      the Free Software Foundation; either version 2 of the License, or
*      (at your option) any later version.
*
*      This program is distributed in the hope that it will be useful,
*      but WITHOUT ANY WARRANTY; without even the implied warranty of
*      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*      GNU General Public License for more details.
*
*      You should have received a copy of the GNU General Public License
*      along with this program; if not, write to the Free Software
*      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#ifndef _CTI485_H
#define _CTI485_H


/*
 * Ioctls for Connect Tech's 485 capable boards
 *
 * ioctl(ser_port, TIOCSER485GET, &cur_mode);
 * if (cur_mode != TIOCSER485HALFDUPLEX) {
 *   cur_mode = TIOCSER485HALFDUPLEX;
 *   ioctl(ser_port, TIOCSER485SET, &cur_mode);
 * }
 */
#define TIOCSER485GET           0x54A0  /* Get the 485 line mode */
#define TIOCSER485SET           0x54A1  /* Set the 485 line mode */
#define TIOCSERCTIBAUDSET       0x54A2  /* Set an arbitrary baud */
#define TIOCSOFTAUTO485         0x54A3  /* Turn on/off software auto485 signalling for Xtreme*/
#define TIOCSENXTREME           0x54A4  /* flag for Xtreme port initialization */
#define TIOCSDISMSRINT          0x54A5  /* flag to disable modem signal interrupt */
#define TIOCSDUMPUART           0x54A6  /* flag to dump UART registers to dmesg */

#define TIOCSER485NOT_INITED            0
#define TIOCSER485FULLDUPLEX            1
#define TIOCSER485HALFDUPLEX            2
#define TIOCSER485SLAVEMULTIPLEX        3


#endif  /* _CTI485_H */

