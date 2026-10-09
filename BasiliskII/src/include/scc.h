/*
 *  scc.h - Minimal Z8530 SCC register emulation
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

#ifndef SCC_H
#define SCC_H

/*
 *  Some classic Mac programs (e.g. Digidesign TurboSynth, Alchemy, MIDI
 *  software) bypass the Serial Driver and program the SCC directly through
 *  the SCCRd/SCCWr low-memory globals. When the "sccemu" pref is set, these
 *  globals point to a small register window that is intercepted by the CPU
 *  emulator's byte accessors and served by this module. Data is exchanged
 *  with the host devices named by the "seriala" (channel A, modem port) and
 *  "serialb" (channel B, printer port) prefs.
 *
 *  Only polled I/O is supported (no SCC interrupts).
 *
 *  Register window layout (same as SCCRd/SCCWr on real hardware):
 *    +0 channel B control, +2 channel A control,
 *    +4 channel B data,    +6 channel A data
 */

// Mac address and size of the emulated register window (size 0 = disabled)
extern uint32 SCCEmuBase;
extern uint32 SCCEmuSize;

extern void SCCInit(void);
extern void SCCExit(void);
extern void SCCReset(void);
extern void SCCPatchLowMem(void);

extern uint32 SCCEmuReadByte(uint32 addr);
extern void SCCEmuWriteByte(uint32 addr, uint32 b);

// Serial line settings derived from the SCC write registers
struct SCCLineSettings {
	uint32 baud;			// Bits per second (0 = unknown)
	int data_bits;			// 5..8
	int parity;				// 0 = none, 1 = odd, 2 = even
	int stop_bits;			// 1 or 2 (1.5 is reported as 2)
	bool external_clock;	// Clocked from the TRxC (HSKi) or RTxC pin
	bool synchronous;		// WR4 selects a synchronous mode

	bool operator==(const SCCLineSettings &o) const
	{
		return baud == o.baud && data_bits == o.data_bits && parity == o.parity
			&& stop_bits == o.stop_bits && external_clock == o.external_clock
			&& synchronous == o.synchronous;
	}
	bool operator!=(const SCCLineSettings &o) const { return !(*this == o); }
};

// Clock sources of the Mac SCC
enum {
	SCC_PCLK_HZ = 3672000,		// PCLK (C8M / 2 class machines)
	SCC_RTXC_HZ = 3686400,		// Crystal on RTxC
	SCC_EXT_CLOCK_HZ = 1000000	// Default HSKi clock (MIDI interface)
};

extern SCCLineSettings SCCComputeLineSettings(const uint8 *wr, uint32 ext_clock_hz);

// Host side of one SCC channel, implemented by the platform
class SCCHostChannel {
public:
	virtual ~SCCHostChannel() {}
	virtual bool open(void) = 0;				// Returns false if the host device is unavailable
	virtual bool is_open(void) = 0;
	virtual bool rx_available(void) = 0;
	virtual int read_byte(void) = 0;			// -1 if no data
	virtual bool tx_ready(void) = 0;
	virtual void write_byte(uint8 b) = 0;
	virtual void set_line(const SCCLineSettings &s) = 0;
	virtual void flush_rx(void) = 0;
};

// Platform factory; path is the "seriala"/"serialb" pref value (may be NULL)
extern SCCHostChannel *SCCCreateHostChannel(int channel, const char *path);

// Core chip model, exposed for unit tests
class SCCChip {
public:
	SCCChip();
	~SCCChip();

	void attach(int channel, SCCHostChannel *host);
	void reset(void);
	uint8 read(uint32 offset);
	void write(uint32 offset, uint8 value);

	uint32 ext_clock_hz;		// Frequency on TRxC (HSKi) when used as clock input
	uint32 poll_delay_us;		// Time a polled empty RR0 read takes on real hardware
	bool debug;

	// Per-channel state, 0 = channel A (modem), 1 = channel B (printer)
	struct Channel {
		uint8 wr[16];
		uint8 reg_ptr;
		uint8 last_rx;
		bool line_valid;
		SCCLineSettings line;
		SCCHostChannel *host;
		bool host_failed;
		uint64 tx_free_ns;		// When the transmitter finishes the last byte (monotonic ns)
	} ch[2];

	bool tx_pacing;				// Transmit no faster than the line rate, like a real SCC

private:
	void reset_channel(int c, bool hardware);
	uint8 read_control(int c);
	void write_control(int c, uint8 value);
	uint8 read_data(int c);
	void write_data(int c, uint8 value);
	uint8 rr0(int c);
	bool ensure_host(int c);
	void update_line(int c);
	void wait_poll_delay(int c);
	void pace_tx(int c);
};

// Time one character takes on the line in nanoseconds (0 if unknown)
extern uint64 SCCByteTimeNs(const SCCLineSettings &s);

#endif
