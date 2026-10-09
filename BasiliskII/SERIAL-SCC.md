# Serial chip (SCC) emulation and serial fixes

This fork adds emulation of the Macintosh serial chip, the Zilog Z8530 SCC, to
Basilisk II. It also fixes the Unix/macOS serial driver. Together these let
classic Mac programs that talk to external hardware over the serial ports work
under Basilisk II, with a USB serial adapter standing in for the Mac's port.

It was built for **Digidesign TurboSynth 2.0** with an **E-mu Emax II**
sampler over RS-422. Sample **download and upload both work**, including the
Emax's 500 kbaud fast transfer mode. The changes are generic and should help
any program that drives the serial ports directly: sample editors, MIDI
software, terminal programs that bypass the Serial Driver, and so on.

- [Why this was needed](#why-this-was-needed)
- [What was changed](#what-was-changed)
- [Using it](#using-it)
- [How it works](#how-it-works)
- [Limitations](#limitations)
- [Files](#files)

## Why this was needed

### 1. Programs bypass the Serial Driver

On a real Mac, software can reach a serial port in two ways:

- through the **Serial Driver** (`.AIn`/`.AOut`, `.BIn`/`.BOut`) with
  Device Manager calls, or
- by **programming the SCC chip directly**, using the addresses in the
  low-memory globals `SCCRd` ($1D8) and `SCCWr` ($1DC).

Many music programs from the late 1980s use the second way. They need exact
timing, external clocking (a MIDI interface supplies a 1 MHz clock on the
HSKi pin) or speeds the driver didn't offer. TurboSynth 2.0 is a typical
example; this is visible from disassembling its `CODE` resources:

- It opens `.AIn`/`.AOut` only to *claim* the port, using Apple's
  `RAMSDOpen` glue, which checks `SPConfig` and AppleTalk.
- Its `CODE 48 'Driver'` segment masks interrupts (`ori.w #$700,sr`),
  loads `SCCRd`/`SCCWr` and resets the channel (WR9 = $80/$40). It then
  writes a register table: WR4 = $84 (×32 clock, 1 stop bit), WR11 = $28
  (receive and transmit clocks from the TRxC pin, i.e. the external HSKi
  clock), WR3 = $C1, WR5 = $6A.
- It then polls RR0 bit 0 (receive character available) and bit 2 (transmit
  buffer empty), and reads and writes the data register directly.
- For Emax sample transfers it switches to ×1 clock with odd parity. That is
  the Emax's synchronous 500 kbaud mode, clocked by the sampler.

Basilisk II only replaced the Serial *Driver* with host code. It does not
emulate the SCC: the ROM patches skip SCC initialisation and point every
hardware base address, `SCCRd`/`SCCWr` included, at scratch memory. Programs
like TurboSynth therefore read and wrote meaningless memory and could never
reach the host's serial port.

### 2. macOS builds had no serial support

Until October 2025 (upstream issue #289) the macOS Xcode project compiled
`serial_dummy.cpp`, so even driver-based programs could not use a serial port
there. Upstream now uses `serial_unix.cpp`. Its first use on macOS exposed the
driver bugs listed below.

### 3. Modern serial hardware doesn't speak the old speeds

The Mac reaches 31,250 baud (MIDI) or 500 kbaud (Emax fast mode) with clocks
supplied by external hardware. macOS USB serial drivers usually accept only
standard rates. The **EMuSer** USB-to-RS-422 adapter for E-mu samplers solves
this in firmware: the host selects "magic" settings and the adapter runs the
real speed.

| Host setting | EMuSer RS-422 link |
|---|---|
| 38400 baud 8N1 | 31,250 baud (MIDI speed) |
| 50 baud 8O1 | 500,000 baud synchronous (fast transfer) |

Basilisk II can now drive such an adapter itself (the `emuser:` prefix
below), so no extra software is needed between the emulator and the sampler.

## What was changed

### SCC register emulation (`sccemu`)

- An emulated Z8530 with two channels. Channel A (modem port) is connected to
  the `seriala` host device and channel B (printer port) to `serialb`.
- `SCCRd`/`SCCWr` (ROM decoder table entries and low memory) point to an
  8-byte register window that the CPU core's byte accessors divert to the
  emulation.
- Supported:
  - Register pointer and "point high" (WR0), channel and hardware reset (WR9).
  - RR0, RR1 (all sent), RR2 (vector), RR3, RR12/13, RR15.
  - The data registers.
- The line settings the program writes become a real serial configuration on
  the host:
  - **Clock:** the WR11 source (baud rate generator via WR12/13/14, the
    3.6864 MHz RTxC crystal, or the external HSKi clock on TRxC) and the WR4
    clock mode (×1/16/32/64) give the speed.
  - **Format:** WR3/WR5 give the data bits, WR4 the parity and stop bits.
  - **Example:** TurboSynth's MIDI set-up gives exactly 31,250 baud 8N1.
- **Real transmit timing.** Like the chip, the emulation holds one byte while
  the previous one is being sent, so data leaves at the programmed line rate
  (320 µs per byte at 31,250 baud). Writes wait rather than reporting a busy
  transmitter, so a program's timeout loop can never drop a byte.
- **Real polling timing.** Programs use busy-wait loops as timeouts, e.g.
  TurboSynth polls RR0 0x7FFF times per byte and 0x7FFFFF times for a reply.
  An empty receive poll takes `sccpolldelay` µs, so those loops last about
  as long as on a real Mac instead of a few microseconds.
- The host device opens when the program enables the receiver or
  transmitter. An I/O thread with lock-free ring buffers keeps the CPU
  thread from blocking on the host.

### Serial Driver fixes (`Unix/serial_unix.cpp`, also used by SheepShaver)

- **Parity never worked:** it was set in `c_oflag` instead of `c_cflag`.
- **Short reads:** reads finished with whatever had arrived. Like the Mac
  Serial Driver they now complete only when all requested bytes are in.
- **Hangs:** KillIO and closing a port could hang the emulator while a read
  was blocked. Reads now use `poll()` with a timeout; a disconnected PTY
  peer no longer causes a busy loop.
- **Partial writes** are retried.
- **Devices are opened with `O_NOCTTY`**, so a PTY can't become the
  emulator's controlling terminal.
- **Modem-line status** is reported sensibly on PTYs.
- **All Mac baud codes** (3600, 7200, 14400, 28800) are supported.
- **External clocking:** `kSERDClockMIDI` and the `kOptionClockX1CTS`
  external clock option (Control 16, bit 6) are handled.
- **Non-standard rates on macOS:** rates such as 31250 work via numeric
  `speed_t` or `IOSSIOSPEED`. The custom rate is re-applied after every
  `tcsetattr()`.
- **Debug output** that was unconditional now only appears with
  `serialdebug true`.

### EMuSer support (`emuser:` prefix)

A serial pref of the form `emuser:/dev/cu.usbmodem…` translates every
requested rate to the adapter's settings, for both the SCC emulation and the
Serial Driver:

- **≥ 250,000 baud** becomes 50 baud 8O1 (fast mode).
- **Anything else** becomes 38400 baud 8N1 (MIDI speed).

## Using it

### Building (macOS)

Build as described in the top-level `README.md`. Install the SDL2 framework
in `/Library/Frameworks` and the GMP/MPFR static libraries in `/usr/local/lib`.
Recent Xcode versions reject the project's 10.13 deployment target, so
override it:

```sh
cd BasiliskII/src/MacOSX
xcodebuild build -project BasiliskII.xcodeproj -scheme BasiliskII \
    -configuration Release MACOSX_DEPLOYMENT_TARGET=12.0
```

On Linux the SCC emulation is part of the normal autotools build.

### Preferences

Add these to `~/.basilisk_ii_prefs`:

```
seriala emuser:/dev/cu.usbmodem454D55534552101
serialb /dev/null
sccemu true
sccpolldelay 1
jit false
```

| Pref | Meaning |
|---|---|
| `seriala`, `serialb` | Host device for the modem (A) and printer (B) port. Prefix `emuser:` for an EMuSer adapter; plain device paths and PTYs work too (a PTY receives the exact rate, e.g. 31250). Don't point both ports at the same device. |
| `sccemu` | `true` enables SCC register emulation. Off by default; without it nothing changes for programs that use the Serial Driver. |
| `sccpolldelay` | µs an empty receive poll takes (default 5 ≈ Mac Plus; 1 ≈ Mac II). With 1, TurboSynth waits about 8 s instead of 42 s if the sampler ever ignores a message. |
| `sccextclock` | Frequency of the external clock on HSKi in Hz (default 1000000, a MIDI interface; ×32 gives 31,250 baud). |
| `serialdebug` | `true` logs Serial Driver calls, SCC register writes, line changes and every byte to stderr. |
| `jit` | Must be `false` on x86_64 when using `sccemu`: compiled code bypasses the register window. Apple Silicon builds have no JIT. |

Basilisk prints `SCC register emulation enabled at …` at start-up when
`sccemu` is active.

### TurboSynth 2.0 with an Emax II

1. Connect the EMuSer to the Emax's RS-422 port and set the prefs above.
2. Start Basilisk II and run **TurboSynth 2.0**. Version 2.21 has no sampler
   transfer code.
3. In TurboSynth choose the Emax on the **Modem** port. If AppleTalk is on,
   put it on the printer port or turn it off.
4. Download or upload samples. Commands run at 31,250 baud; sample data
   switches to 500 kbaud automatically and back afterwards.

With `serialdebug true` you can follow it:

```
SCC: SCCRd/SCCWr -> 04104000
SCC A: using /dev/cu.usbmodem454D55534552101 (EMuSer baud translation)
SCC A: line 31250 baud 8N1 ext-clock
SCC A: 31250 baud requested, EMuSer set to 38400
SCC A: line 1000000 baud 8O1 ext-clock
SCC A: 1000000 baud requested, EMuSer set to 50
```

## How it works

```
68k code: move.b (a0),d0   where a0 = SCCRd
        │
        ▼
get_byte()/put_byte() in uae_cpu*/memory.h
        │  address inside [SCCEmuBase, SCCEmuBase+8)?
        ▼
SCCChip (scc.cpp): Z8530 registers, line settings, transmit/poll timing
        │
        ▼
UnixSCCChannel (Unix/scc_unix.cpp): I/O thread, ring buffers, termios
        │  serial_apply_line(): exact rate or emuser: translation
        ▼
host device (seriala/serialb): USB serial adapter, EMuSer, PTY, ...
```

- **Register window.** It sits in otherwise unused scratch memory (16 KB below
  the hardware-base scratch address), so any other access to it, including
  from JIT code, is harmless. The 32-bit ROM's decoder table maps entries 3,
  4 and 17 to `SCCRd`/`SCCWr`; only those are redirected, everything else
  still uses scratch memory. `InstallDrivers` writes the low-memory globals
  again for safety.
- **Address layout.** Same as on the Mac: +0 channel B control, +2 channel A
  control, +4 channel B data, +6 channel A data.
- **Speed of the hook.** The CPU hook is a single unsigned compare per byte
  access and is compiled only on Unix-like builds (`SCC_EMULATION` in
  `Unix/sysdeps.h`). Other platforms are unchanged.
- **Channel reset** discards received data that nobody read, as a real chip
  would have lost it.
- **No sampler-specific code.** Nothing in the emulation knows about E-mu or
  TurboSynth; it emulates the chip and maps line settings. EMuSer support is
  only the rate mapping selected by the `emuser:` prefix.

## Limitations

- **Polled I/O only:** no SCC interrupts. Programs that use interrupt-driven
  SCC I/O (e.g. Apple's MIDI Driver for the MIDI Manager) are not served yet.
- **DCD and CTS** always read as asserted; break and sync/hunt aren't
  emulated.
- **Basilisk II on Unix/macOS only;** SheepShaver gets the Serial Driver
  fixes but not the SCC emulation.
- **Linux rates:** non-standard rates such as 31250 are rounded to the next
  standard rate unless `emuser:` is used.
- **Other programs untested:** Alchemy 3.0, Sound Designer II and other
  sampler editors look like they use the same technique but haven't been
  tested yet. Reports welcome.

## Files

| File | Purpose |
|---|---|
| `src/scc.cpp`, `src/include/scc.h` | Z8530 model, line-setting derivation, timing, glue |
| `src/Unix/scc_unix.cpp` | Host channel: device, I/O thread, ring buffers |
| `src/Unix/serial_line_unix.h` | termios set-up shared with the Serial Driver; `emuser:` mapping |
| `src/Unix/serial_unix.cpp` | Serial Driver (fixes above) |
| `src/uae_cpu/memory.h`, `src/uae_cpu_2021/memory.h` | Byte-access hook for the register window |
| `src/rom_patches.cpp`, `src/emul_op.cpp`, `src/main.cpp` | `SCCRd`/`SCCWr` redirection, reset, init/exit |
| `src/prefs_items.cpp`, `README` | New prefs and their documentation |
