/*
 *  scc.cpp - Minimal Z8530 SCC register emulation
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

#include "sysdeps.h"

#include <stdio.h>
#include <string.h>
#include <chrono>

#include "scc.h"

#ifndef SCC_UNIT_TEST
#include "cpu_emulation.h"
#include "main.h"
#include "prefs.h"
#endif

#define DEBUG 0
#include "debug.h"


// Register window (size 0 = SCC emulation disabled)
uint32 SCCEmuBase = 0;
uint32 SCCEmuSize = 0;

#ifndef SCC_UNIT_TEST
static SCCChip *the_scc = NULL;
#endif

// RR0 bits
enum {
	RR0_RX_AVAIL = 0x01,
	RR0_TX_EMPTY = 0x04,
	RR0_DCD = 0x08,
	RR0_CTS = 0x20,
	RR0_TX_UNDERRUN = 0x40
};

static const char chan_name[2] = {'A', 'B'};


/*
 *  Derive line settings from the write registers
 */

static uint32 snap_baud(double rate)
{
	static const uint32 std_rates[] = {
		50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 3600, 4800, 7200,
		9600, 14400, 19200, 28800, 31250, 38400, 57600, 76800, 115200, 230400,
		250000, 500000, 1000000
	};
	for (unsigned i = 0; i < sizeof(std_rates) / sizeof(std_rates[0]); i++) {
		double r = std_rates[i];
		if (rate > r * 0.97 && rate < r * 1.03)
			return std_rates[i];
	}
	return (uint32)(rate + 0.5);
}

SCCLineSettings SCCComputeLineSettings(const uint8 *wr, uint32 ext_clock_hz)
{
	static const int clock_mult[4] = {1, 16, 32, 64};
	static const int bits_map[4] = {5, 7, 6, 8};

	SCCLineSettings s;
	memset(&s, 0, sizeof(s));

	// Prefer the transmitter clock source when the transmitter is enabled
	int src = (wr[5] & 0x08) ? ((wr[11] >> 3) & 3) : ((wr[11] >> 5) & 3);
	double clock;
	switch (src) {
		case 0:		// RTxC pin (3.6864 MHz crystal on the Mac)
			clock = SCC_RTXC_HZ;
			break;
		case 1:		// TRxC pin, driven from HSKi by external devices
			clock = ext_clock_hz;
			s.external_clock = true;
			break;
		default: {	// Baud rate generator (DPLL treated alike)
			double brg_src = (wr[14] & 0x02) ? SCC_PCLK_HZ : SCC_RTXC_HZ;
			uint32 tc = (wr[13] << 8) | wr[12];
			clock = brg_src / (2.0 * (tc + 2));
			break;
		}
	}
	s.baud = snap_baud(clock / clock_mult[wr[4] >> 6]);

	s.data_bits = bits_map[(wr[5] & 0x08) ? ((wr[5] >> 5) & 3) : ((wr[3] >> 6) & 3)];
	s.parity = (wr[4] & 0x01) ? ((wr[4] & 0x02) ? 2 : 1) : 0;
	switch ((wr[4] >> 2) & 3) {
		case 0: s.stop_bits = 1; s.synchronous = true; break;
		case 1: s.stop_bits = 1; break;
		default: s.stop_bits = 2; break;
	}
	return s;
}


/*
 *  Chip model
 */

SCCChip::SCCChip()
{
	ext_clock_hz = SCC_EXT_CLOCK_HZ;
	poll_delay_us = 5;
	debug = false;
	for (int c = 0; c < 2; c++) {
		memset(&ch[c], 0, sizeof(ch[c]));
		ch[c].host = NULL;
	}
	reset();
}

SCCChip::~SCCChip()
{
	for (int c = 0; c < 2; c++) {
		delete ch[c].host;
		ch[c].host = NULL;
	}
}

void SCCChip::attach(int channel, SCCHostChannel *host)
{
	delete ch[channel].host;
	ch[channel].host = host;
	ch[channel].host_failed = false;
}

void SCCChip::reset(void)
{
	reset_channel(0, true);
	reset_channel(1, true);
}

void SCCChip::reset_channel(int c, bool hardware)
{
	Channel &p = ch[c];
	if (hardware) {
		memset(p.wr, 0, sizeof(p.wr));
		p.wr[4] = 0x04;
		p.wr[11] = 0x08;
		p.wr[14] = 0x30;
		p.wr[15] = 0xf8;
	} else {
		p.wr[0] = 0;
		p.wr[1] &= 0x24;
		p.wr[3] &= 0xfe;
		p.wr[4] |= 0x04;
		p.wr[5] &= 0x61;
		p.wr[14] = (p.wr[14] & 0xc3) | 0x20;
		p.wr[15] = 0xf8;
	}
	p.reg_ptr = 0;

	// Data received while nobody was listening would have been lost on real hardware
	if (p.host && p.host->is_open())
		p.host->flush_rx();
}

bool SCCChip::ensure_host(int c)
{
	Channel &p = ch[c];
	if (p.host == NULL)
		return false;
	if (p.host->is_open())
		return true;
	if (p.host_failed)
		return false;
	if (!p.host->open()) {
		p.host_failed = true;
		if (debug)
			fprintf(stderr, "SCC %c: cannot open host device\n", chan_name[c]);
		return false;
	}
	if (debug)
		fprintf(stderr, "SCC %c: host device opened\n", chan_name[c]);
	if (p.line_valid)
		p.host->set_line(p.line);
	return true;
}

void SCCChip::update_line(int c)
{
	Channel &p = ch[c];
	if (!(p.wr[3] & 0x01) && !(p.wr[5] & 0x08))
		return;		// Receiver and transmitter disabled

	// A newly enabled port gets another chance to open its host device
	p.host_failed = false;

	SCCLineSettings s = SCCComputeLineSettings(p.wr, ext_clock_hz);
	if (p.line_valid && s == p.line)
		return;
	p.line = s;
	p.line_valid = true;
	if (debug)
		fprintf(stderr, "SCC %c: line %u baud %d%c%d%s%s\n", chan_name[c], s.baud, s.data_bits,
			"NOE"[s.parity], s.stop_bits, s.external_clock ? " ext-clock" : "", s.synchronous ? " sync" : "");
	// ensure_host() applies the line itself when it opens the device
	bool was_open = p.host && p.host->is_open();
	if (ensure_host(c) && was_open)
		p.host->set_line(s);
}

uint8 SCCChip::rr0(int c)
{
	Channel &p = ch[c];
	uint8 v = RR0_TX_EMPTY | RR0_TX_UNDERRUN | RR0_CTS | RR0_DCD;
	if ((p.wr[3] & 0x01) || (p.wr[5] & 0x08))
		ensure_host(c);
	if (p.host && p.host->is_open()) {
		if (p.host->rx_available())
			v |= RR0_RX_AVAIL;
		if (!p.host->tx_ready())
			v &= ~RR0_TX_EMPTY;
	}
	return v;
}

// Make a polled read of an empty receiver take roughly as long as on a real
// Mac, so that software timeout loops keep their original duration
void SCCChip::wait_poll_delay(int c)
{
	Channel &p = ch[c];
	if (poll_delay_us == 0 || !(p.wr[3] & 0x01) || p.host == NULL || !p.host->is_open())
		return;
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now() + std::chrono::microseconds(poll_delay_us);
	while (!p.host->rx_available() && std::chrono::steady_clock::now() < end)
		;
}

uint8 SCCChip::read_control(int c)
{
	Channel &p = ch[c];
	int reg = p.reg_ptr;
	p.reg_ptr = 0;

	switch (reg) {
		case 0: case 4: {
			uint8 v = rr0(c);
			if (!(v & RR0_RX_AVAIL)) {
				wait_poll_delay(c);
				v = rr0(c);
			}
			return v;
		}
		case 1: case 5:
			return 0x07;	// All sent, 8 bit residue code
		case 2: case 6:
			if (c == 0)
				return p.wr[2];
			// Channel B returns the vector modified with "no interrupt pending" status
			if (p.wr[9] & 0x01) {
				if (p.wr[9] & 0x10)
					return (p.wr[2] & 0x8f) | (0x03 << 4);
				return (p.wr[2] & 0xf1) | (0x03 << 1);
			}
			return p.wr[2];
		case 3: case 7:
			return 0;		// No interrupts pending
		case 8:
			return read_data(c);
		case 10: case 14:
			return 0;
		case 12:
			return p.wr[12];
		case 13: case 9:
			return p.wr[13];
		case 15: case 11:
			return p.wr[15] & 0xfa;
		default:
			return 0;
	}
}

void SCCChip::write_control(int c, uint8 value)
{
	Channel &p = ch[c];
	int reg = p.reg_ptr;
	p.reg_ptr = 0;

	if (reg == 0) {
		p.reg_ptr = value & 7;
		if (((value >> 3) & 7) == 1)
			p.reg_ptr |= 8;		// Point high
		p.wr[0] = value;
		return;
	}

	if (debug)
		fprintf(stderr, "SCC %c: WR%d = $%02x\n", chan_name[c], reg, value);

	switch (reg) {
		case 2:		// Interrupt vector is shared
			ch[0].wr[2] = ch[1].wr[2] = value;
			break;
		case 9:		// Master interrupt control is shared
			switch (value & 0xc0) {
				case 0x40: reset_channel(1, false); break;
				case 0x80: reset_channel(0, false); break;
				case 0xc0: reset_channel(0, true); reset_channel(1, true); break;
			}
			ch[0].wr[9] = ch[1].wr[9] = value & 0x3f;
			break;
		case 3: case 4: case 5: case 11: case 12: case 13: case 14:
			p.wr[reg] = value;
			update_line(c);
			break;
		default:
			p.wr[reg] = value;
			break;
	}
}

uint8 SCCChip::read_data(int c)
{
	Channel &p = ch[c];
	if (p.host && p.host->is_open()) {
		int b = p.host->read_byte();
		if (b >= 0) {
			p.last_rx = b;
			if (debug)
				fprintf(stderr, "SCC %c: rx %02x\n", chan_name[c], b);
		}
	}
	return p.last_rx;
}

void SCCChip::write_data(int c, uint8 value)
{
	if (debug)
		fprintf(stderr, "SCC %c: tx %02x\n", chan_name[c], value);
	if (ensure_host(c))
		ch[c].host->write_byte(value);
}

uint8 SCCChip::read(uint32 offset)
{
	int c = (offset & 2) ? 0 : 1;
	return (offset & 4) ? read_data(c) : read_control(c);
}

void SCCChip::write(uint32 offset, uint8 value)
{
	int c = (offset & 2) ? 0 : 1;
	if (offset & 4)
		write_data(c, value);
	else
		write_control(c, value);
}


#ifndef SCC_UNIT_TEST

/*
 *  Global glue
 */

void SCCInit(void)
{
	SCCEmuSize = 0;
	if (!PrefsFindBool("sccemu"))
		return;

#if SCC_EMULATION
	// Use otherwise unused memory in the low half of the scratch area, so that
	// stray accesses (e.g. from JIT compiled code) can never fault
	extern uint8 *ScratchMem;
	SCCEmuBase = Host2MacAddr(ScratchMem) - 0x4000;
	SCCEmuSize = 8;

	the_scc = new SCCChip;
	the_scc->debug = PrefsFindBool("serialdebug");
	int32 ext_clock = PrefsFindInt32("sccextclock");
	if (ext_clock > 0)
		the_scc->ext_clock_hz = ext_clock;
	int32 poll_delay = PrefsFindInt32("sccpolldelay");
	if (poll_delay >= 0)
		the_scc->poll_delay_us = poll_delay;
	the_scc->attach(0, SCCCreateHostChannel(0, PrefsFindString("seriala")));
	the_scc->attach(1, SCCCreateHostChannel(1, PrefsFindString("serialb")));

	printf("SCC register emulation enabled at %08x\n", SCCEmuBase);
#if USE_JIT
	if (PrefsFindBool("jit"))
		printf("WARNING: SCC register emulation does not work with the JIT compiler, set \"jit false\"\n");
#endif
#else
	printf("WARNING: SCC register emulation is not supported in this build\n");
#endif
}

void SCCExit(void)
{
	SCCEmuSize = 0;
	delete the_scc;
	the_scc = NULL;
}

void SCCReset(void)
{
	if (the_scc)
		the_scc->reset();
}

// Point SCCRd/SCCWr at the emulated register window
void SCCPatchLowMem(void)
{
	if (SCCEmuSize == 0)
		return;
	WriteMacInt32(0x1d8, SCCEmuBase);
	WriteMacInt32(0x1dc, SCCEmuBase);
	if (the_scc && the_scc->debug)
		fprintf(stderr, "SCC: SCCRd/SCCWr -> %08x\n", SCCEmuBase);
}

uint32 SCCEmuReadByte(uint32 addr)
{
	if (the_scc == NULL)
		return 0;
	return the_scc->read(addr - SCCEmuBase);
}

void SCCEmuWriteByte(uint32 addr, uint32 b)
{
	if (the_scc)
		the_scc->write(addr - SCCEmuBase, b);
}

#endif
