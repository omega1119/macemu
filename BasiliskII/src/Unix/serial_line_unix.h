/*
 *  serial_line_unix.h - Host serial line configuration shared by the
 *                       Serial Driver and the SCC register emulation
 *
 *  Basilisk II (C) Christian Bauer
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef SERIAL_LINE_UNIX_H
#define SERIAL_LINE_UNIX_H

#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif

/*
 *  A serial pref may be prefixed with "emuser:" to talk to an EMuSer
 *  USB-RS422 adapter (E-mu Emax/Emulator II). Its firmware cannot be set to
 *  31250 or 500000 baud directly and instead maps magic host settings:
 *    38400 baud 8N1 -> 31250 baud (MIDI speed)
 *    50 baud 8O1    -> 500000 baud (synchronous fast transfer)
 */

static const char EMUSER_PREFIX[] = "emuser:";

static inline const char *serial_parse_path(const char *pref, bool *emuser)
{
	*emuser = false;
	if (pref && strncmp(pref, EMUSER_PREFIX, sizeof(EMUSER_PREFIX) - 1) == 0) {
		*emuser = true;
		return pref + sizeof(EMUSER_PREFIX) - 1;
	}
	return pref;
}

#ifndef __APPLE__
static speed_t serial_speed_const(uint32 baud)
{
	static const struct { uint32 rate; speed_t code; } table[] = {
		{50, B50}, {75, B75}, {110, B110}, {134, B134}, {150, B150}, {200, B200},
		{300, B300}, {600, B600}, {1200, B1200}, {1800, B1800}, {2400, B2400},
		{4800, B4800}, {9600, B9600}, {19200, B19200}, {38400, B38400},
		{57600, B57600}, {115200, B115200}, {230400, B230400}
	};
	// No arbitrary rates here: use the next higher (or equal) standard rate
	for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); i++)
		if (baud <= table[i].rate)
			return table[i].code;
	return B230400;
}
#endif

/*
 *  tcsetattr() for a port that may run at a non-standard rate set with
 *  IOSSIOSPEED (macOS); custom_speed is 0 for standard rates. The cached
 *  termios then holds a standard placeholder speed, and the custom rate has
 *  to be re-applied because tcsetattr() resets it.
 */

static int serial_set_mode(int fd, const struct termios *mode, uint32 custom_speed)
{
	int res = tcsetattr(fd, TCSANOW, mode);
#ifdef __APPLE__
	if (res == 0 && custom_speed) {
		speed_t speed = custom_speed;
		res = ioctl(fd, IOSSIOSPEED, &speed);
	}
#endif
	return res;
}

/*
 *  Apply baud rate and character format to a host serial device.
 *  parity: 0 = none, 1 = odd, 2 = even. Returns the effective host baud rate.
 *  *custom_speed is set to the rate if it had to be set with IOSSIOSPEED.
 */

static uint32 serial_apply_line(int fd, struct termios *mode, uint32 baud, int data_bits, int parity, int stop_bits, bool emuser, uint32 *custom_speed)
{
	*custom_speed = 0;
	if (emuser) {
		if (baud >= 250000) {
			baud = 50;
			parity = 1;
		} else {
			baud = 38400;
			parity = 0;
		}
		data_bits = 8;
		stop_bits = 1;
	}

	mode->c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
	switch (data_bits) {
		case 5: mode->c_cflag |= CS5; break;
		case 6: mode->c_cflag |= CS6; break;
		case 7: mode->c_cflag |= CS7; break;
		default: mode->c_cflag |= CS8; break;
	}
	if (parity) {
		mode->c_cflag |= PARENB;
		if (parity == 1)
			mode->c_cflag |= PARODD;
		mode->c_iflag |= INPCK;
	} else
		mode->c_iflag &= ~INPCK;
	if (stop_bits >= 2)
		mode->c_cflag |= CSTOPB;

#ifdef __APPLE__
	// speed_t is the numeric rate on Darwin; PTYs accept any rate, real
	// devices may need IOSSIOSPEED for non-standard ones
	cfsetispeed(mode, baud);
	cfsetospeed(mode, baud);
	if (tcsetattr(fd, TCSANOW, mode) < 0) {
		cfsetispeed(mode, B9600);
		cfsetospeed(mode, B9600);
		*custom_speed = baud;
		serial_set_mode(fd, mode, baud);
	}
#else
	cfsetispeed(mode, serial_speed_const(baud));
	cfsetospeed(mode, serial_speed_const(baud));
	tcsetattr(fd, TCSANOW, mode);
#endif
	return baud;
}

#endif
