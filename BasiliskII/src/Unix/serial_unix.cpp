/*
 *  serial_unix.cpp - Serial device driver, Unix specific stuff
 *
 *  Basilisk II (C) 1997-2008 Christian Bauer
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

#include <sys/ioctl.h>

#ifdef HAVE_SYS_FILIO_H
#include <sys/filio.h>
#endif

#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include <termios.h>
#include <errno.h>

#ifdef __linux__
#include <linux/lp.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#endif

#include "cpu_emulation.h"
#include "main.h"
#include "macos_util.h"
#include "prefs.h"
#include "serial.h"
#include "serial_defs.h"
#include "serial_line_unix.h"

extern "C" {
#include "sshpty.h"
}


#define DEBUG 0
#include "debug.h"

#define MONITOR 0

// Runtime logging, enabled with the "serialdebug" pref
static bool serial_debug = false;
#define SERIAL_LOG(...) do { if (serial_debug) fprintf(stderr, __VA_ARGS__); } while (0)

// Timeout for blocking reads, so KillIO and close never wait forever
const int INPUT_POLL_TIMEOUT_MS = 100;


// IRIX missing or unsupported defines
#ifdef sgi
#ifndef CRTSCTS
#define CRTSCTS CNEW_RTSCTS
#endif
#ifndef B230400
#define B230400 B115200
#endif
#endif


// Missing functions
#ifndef HAVE_CFMAKERAW
static int cfmakeraw(struct termios *termios_p)
{
	termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	termios_p->c_oflag &= ~OPOST;
	termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	termios_p->c_cflag &= ~(CSIZE|PARENB);
	termios_p->c_cflag |= CS8;
	return 0;
}
#endif


// Driver private variables
class XSERDPort : public SERDPort {
public:
	XSERDPort(const char *dev)
	{
		device_name = serial_parse_path(dev, &emuser);
		protocol = serial;
		fd = -1;
		pid = 0;
		input_thread_active = output_thread_active = false;
		baud = 9600;
		data_bits = 8;
		parity = 0;
		stop_bits = 1;
		ext_clock = false;
		custom_speed = 0;

		Set_pthread_attr(&thread_attr, 2);
	}

	virtual ~XSERDPort()
	{
		if (input_thread_active) {
			input_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
			pthread_cancel(input_thread);
#endif
			pthread_join(input_thread, NULL);
			sem_destroy(&input_signal);
			input_thread_active = false;
		}
		if (output_thread_active) {
			output_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
			pthread_cancel(output_thread);
#endif
			pthread_join(output_thread, NULL);
			sem_destroy(&output_signal);
			output_thread_active = false;
		}
	}

	virtual int16 open(uint16 config);
	virtual int16 prime_in(uint32 pb, uint32 dce);
	virtual int16 prime_out(uint32 pb, uint32 dce);
	virtual int16 control(uint32 pb, uint32 dce, uint16 code);
	virtual int16 status(uint32 pb, uint32 dce, uint16 code);
	virtual int16 close(void);

private:
	bool open_pty(void);
	bool configure(uint16 config);
	void apply_line(void);
	void set_handshake(uint32 s, bool with_dtr);
	static void *input_func(void *arg);
	static void *output_func(void *arg);

	const char *device_name;			// Device name
	bool emuser;						// Flag: translate rates for an EMuSer adapter
	enum {serial, parallel, pty, midi}
		protocol;						// Type of device
	int fd;								// FD of device
	pid_t pid;							// PID of child process

	bool io_killed;						// Flag: KillIO called, I/O threads must not call deferred tasks
	bool quitting;						// Flag: Quit threads

	pthread_attr_t thread_attr;			// Input/output thread attributes

	bool input_thread_active;			// Flag: Input thread installed
	volatile bool input_thread_cancel;	// Flag: Cancel input thread
	pthread_t input_thread;				// Data input thread
	sem_t input_signal;					// Signal for input thread: execute command
	uint32 input_pb;					// Command parameter for input thread

	bool output_thread_active;			// Flag: Output thread installed
	volatile bool output_thread_cancel;	// Flag: Cancel output thread
	pthread_t output_thread;			// Data output thread
	sem_t output_signal;				// Signal for output thread: execute command
	uint32 output_pb;					// Command parameter for output thread

	struct termios mode;				// Terminal configuration

	uint32 baud;						// Current line settings requested by MacOS
	int data_bits;
	int parity;							// 0 = none, 1 = odd, 2 = even
	int stop_bits;
	bool ext_clock;						// External clock on HSKi (kOptionClockX1CTS)
	uint32 custom_speed;				// Non-standard rate set with IOSSIOSPEED (0 = none)
};


/*
 *  Initialization
 */

void SerialInit(void)
{
	serial_debug = PrefsFindBool("serialdebug");

	// Read serial preferences and create structs for both ports
	const char *seriala = PrefsFindString("seriala");
	const char *serialb = PrefsFindString("serialb");
	SERIAL_LOG("SerialInit: seriala='%s' serialb='%s'\n", seriala ? seriala : "(null)", serialb ? serialb : "(null)");
	the_serd_port[0] = new XSERDPort(seriala);
	the_serd_port[1] = new XSERDPort(serialb);
}


/*
 *  Deinitialization
 */

void SerialExit(void)
{
	delete (XSERDPort *)the_serd_port[0];
	delete (XSERDPort *)the_serd_port[1];
}


/*
 *  Open serial port
 */

int16 XSERDPort::open(uint16 config)
{
	SERIAL_LOG("XSERDPort::open: device='%s' config=%04x\n", device_name ? device_name : "(null)", config);

	// Don't open NULL name devices
	if (device_name == NULL)
		return openErr;

	// Init variables
	io_killed = false;
	quitting = false;
	ext_clock = false;
	custom_speed = 0;

	// Open port, according to the syntax of the path
	if (device_name[0] == '|') {
		// Open a process via ptys
		if (!open_pty())
			goto open_error;
	}
	else if (!strcmp(device_name, "midi")) {
		// MIDI:  not yet implemented
		return openErr;
	}
	else {
		// Device special file (O_NOCTTY: a PTY must not become our controlling terminal)
		fd = ::open(device_name, O_RDWR | O_NOCTTY);
		if (fd < 0) {
			SERIAL_LOG("XSERDPort::open: cannot open '%s': %s\n", device_name, strerror(errno));
			goto open_error;
		}

#if defined(__linux__)
		// Parallel port?
		struct stat st;
		if (fstat(fd, &st) == 0)
			if (S_ISCHR(st.st_mode))
				protocol = ((MAJOR(st.st_rdev) == LP_MAJOR) ? parallel : serial);
#elif defined(__FreeBSD__) || defined(__NetBSD__)
		// Parallel port?
		struct stat st;
		if (fstat(fd, &st) == 0)
			if (S_ISCHR(st.st_mode))
				protocol = (((st.st_rdev >> 16) == 16) ? parallel : serial);
#endif
	}

	// Configure port for raw mode
	if (protocol == serial || protocol == pty) {
		if (tcgetattr(fd, &mode) < 0)
			goto open_error;
		cfmakeraw(&mode);
		mode.c_cflag |= HUPCL;
		mode.c_cc[VMIN] = 1;
		mode.c_cc[VTIME] = 0;
		tcsetattr(fd, TCSAFLUSH, &mode);
	}
	configure(config);

	// Start input/output threads
	input_thread_cancel = false;
	output_thread_cancel = false;
	if (sem_init(&input_signal, 0, 0) < 0)
		goto open_error;
	if (sem_init(&output_signal, 0, 0) < 0)
		goto open_error; 
	input_thread_active = (pthread_create(&input_thread, &thread_attr, input_func, this) == 0);
	output_thread_active = (pthread_create(&output_thread, &thread_attr, output_func, this) == 0);
	if (!input_thread_active || !output_thread_active)
		goto open_error;
	SERIAL_LOG("XSERDPort::open: '%s' open, fd=%d%s\n", device_name, fd, emuser ? " (EMuSer baud translation)" : "");
	return noErr;

open_error:
	if (input_thread_active) {
		input_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
		pthread_cancel(input_thread);
#endif
		pthread_join(input_thread, NULL);
		sem_destroy(&input_signal);
		input_thread_active = false;
	}
	if (output_thread_active) {
		output_thread_cancel = true;
#ifdef HAVE_PTHREAD_CANCEL
		pthread_cancel(output_thread);
#endif
		pthread_join(output_thread, NULL);
		sem_destroy(&output_signal);
		output_thread_active = false;
	}
	if (fd > 0) {
		::close(fd);
		fd = -1;
	}
	return openErr;
}


/*
 *  Read data from port
 */

int16 XSERDPort::prime_in(uint32 pb, uint32 dce)
{
	SERIAL_LOG("XSERDPort::prime_in: %u bytes requested\n", ReadMacInt32(pb + ioReqCount));
	// Send input command to input_thread
	read_done = false;
	read_pending = true;
	input_pb = pb;
	WriteMacInt32(input_dt + serdtDCE, dce);
	sem_post(&input_signal);
	return 1;	// Command in progress
}


/*
 *  Write data to port
 */

int16 XSERDPort::prime_out(uint32 pb, uint32 dce)
{
	SERIAL_LOG("XSERDPort::prime_out: %u bytes\n", ReadMacInt32(pb + ioReqCount));
	// Send output command to output_thread
	write_done = false;
	write_pending = true;
	output_pb = pb;
	WriteMacInt32(output_dt + serdtDCE, dce);
	sem_post(&output_signal);
	return 1;	// Command in progress
}


/*
 *	Control calls
 */

int16 XSERDPort::control(uint32 pb, uint32 dce, uint16 code)
{
	SERIAL_LOG("XSERDPort::control: '%s' csCode %d csParam %04x\n", device_name, code, ReadMacInt16(pb + csParam));
	switch (code) {
		case 1:			// KillIO
			io_killed = true;
			if (protocol == serial)
				tcflush(fd, TCIOFLUSH);
			while (read_pending || write_pending)
				usleep(10000);
			io_killed = false;
			return noErr;

		case kSERDConfiguration:
			if (configure(ReadMacInt16(pb + csParam)))
				return noErr;
			else
				return paramErr;

		case kSERDInputBuffer:
			return noErr;	// Not supported under Unix

		case kSERDSerHShake:
			set_handshake(pb + csParam, false);
			return noErr;

		case kSERDSetBreak:
			if (protocol == serial)
				tcsendbreak(fd, 0);
			return noErr;

		case kSERDClearBreak:
			return noErr;

		case kSERDBaudRate: {
			if (protocol != serial)
				return noErr;
			static const uint16 rates[] = {
				50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 31250, 38400, 57600
			};
			uint16 rate = ReadMacInt16(pb + csParam);
			uint16 actual = 57600;	// Also for rates between 57600 and 65535
			for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
				if (rate <= rates[i]) {
					actual = rates[i];
					break;
				}
			WriteMacInt16(pb + csParam, actual);
			baud = actual;
			apply_line();
			return noErr;
		}

		case kSERDClockMIDI:	// Clock from a MIDI interface (1 MHz on HSKi, x32)
			if (protocol != serial)
				return noErr;
			baud = 31250;
			ext_clock = false;
			apply_line();
			return noErr;

		case kSERDHandshake:
		case kSERDHandshakeRS232:
			set_handshake(pb + csParam, true);
			return noErr;

		case kSERDMiscOptions: {
			if (protocol != serial)
				return noErr;
			uint8 options = ReadMacInt8(pb + csParam);
			if (options & kOptionPreserveDTR)
				mode.c_cflag &= ~HUPCL;
			else
				mode.c_cflag |= HUPCL;
			// Bit 6: the data clock is driven by the external device through HSKi
			// in x1 mode (e.g. E-mu Emax 500 kbaud transfers)
			bool new_ext_clock = (options & kOptionClockX1CTS) != 0;
			if (new_ext_clock != ext_clock) {
				ext_clock = new_ext_clock;
				apply_line();
			} else
				serial_set_mode(fd, &mode, custom_speed);
			return noErr;
		}

		case kSERDAssertDTR: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_DTR;
			ioctl(fd, TIOCMBIS, &status);
			return noErr;
		}

		case kSERDNegateDTR: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_DTR;
			ioctl(fd, TIOCMBIC, &status);
			return noErr;
		}

		case kSERDSetPEChar:
		case kSERDSetPEAltChar:
			return noErr;	// Not supported under Unix

		case kSERDResetChannel:
			if (protocol == serial)
				tcflush(fd, TCIOFLUSH);
			return noErr;

		case kSERDAssertRTS: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_RTS;
			ioctl(fd, TIOCMBIS, &status);
			return noErr;
		}

		case kSERDNegateRTS: {
			if (protocol != serial)
				return noErr;
			unsigned int status = TIOCM_RTS;
			ioctl(fd, TIOCMBIC, &status);
			return noErr;
		}

		case kSERD115KBaud:
			if (protocol != serial)
				return noErr;
			baud = 115200;
			apply_line();
			return noErr;

		case kSERD230KBaud:
		case kSERDSetHighSpeed:
			if (protocol != serial)
				return noErr;
			baud = 230400;
			apply_line();
			return noErr;

		default:
			printf("WARNING: SerialControl(): unimplemented control code %d\n", code);
			return controlErr;
	}
}


/*
 *	Status calls
 */

int16 XSERDPort::status(uint32 pb, uint32 dce, uint16 code)
{
	switch (code) {
		case kSERDInputCount: {
			int num;
			ioctl(fd, FIONREAD, &num);
			WriteMacInt32(pb + csParam, num);
			return noErr;
		}

		case kSERDStatus: {
			uint32 p = pb + csParam;
			WriteMacInt8(p + staCumErrs, cum_errors);
			cum_errors = 0;
			WriteMacInt8(p + staXOffSent, 0);
			WriteMacInt8(p + staXOffHold, 0);
			WriteMacInt8(p + staRdPend, read_pending);
			WriteMacInt8(p + staWrPend, write_pending);
			if (protocol != serial) {
				WriteMacInt8(p + staCtsHold, 0);
				WriteMacInt8(p + staDsrHold, 0);
				WriteMacInt8(p + staModemStatus, dsrEvent | dcdEvent | ctsEvent);
			} else {
				unsigned int status = 0;
				if (ioctl(fd, TIOCMGET, &status) < 0)
					status = TIOCM_CTS | TIOCM_DSR | TIOCM_CD;	// e.g. PTY: report lines as asserted
				WriteMacInt8(p + staCtsHold, status & TIOCM_CTS ? 0 : 1);
				WriteMacInt8(p + staDsrHold, status & TIOCM_DTR ? 0 : 1);
				WriteMacInt8(p + staModemStatus,
					(status & TIOCM_DSR ? dsrEvent : 0)
					| (status & TIOCM_RI ? riEvent : 0)
					| (status & TIOCM_CD ? dcdEvent : 0)
					| (status & TIOCM_CTS ? ctsEvent : 0));
			}
			return noErr;
		}

		default:
			printf("WARNING: SerialStatus(): unimplemented status code %d\n", code);
			return statusErr;
	}
}


/*
 *	Close serial port
 */

int16 XSERDPort::close()
{
	// Kill threads
	if (input_thread_active) {
		quitting = true;
		sem_post(&input_signal);
		pthread_join(input_thread, NULL);
		input_thread_active = false;
		sem_destroy(&input_signal);
	}
	if (output_thread_active) {
		quitting = true;
		sem_post(&output_signal);
		pthread_join(output_thread, NULL);
		output_thread_active = false;
		sem_destroy(&output_signal);
	}

	// Close port
	if (fd > 0)
		::close(fd);
	fd = -1;

	// Wait for the subprocess to exit
	if (pid)
		waitpid(pid, NULL, 0);
	pid = 0;

	return noErr;
}


/*
 * Open a process via ptys
 */

bool XSERDPort::open_pty(void)
{
	// Talk to a process via a pty
	char slave[128];
	int slavefd;

	protocol = pty;
	if (!pty_allocate(&fd, &slavefd, slave, sizeof(slave)))
		return false;
		
	fflush(stdout);
	fflush(stderr);
	switch (pid = fork()) {
	case -1:				// error
		return false;
		break;
	case 0:					// child
		::close(fd);

		/* Make the pseudo tty our controlling tty. */
		pty_make_controlling_tty(&slavefd, slave);

		::close(0); dup(slavefd); // Use the slave fd for stdin,
		::close(1); dup(slavefd); // stdout,
		::close(2); dup(slavefd); // and stderr.

		// <should we be more paranoid about closing unused fds?>
		// <should we drop privileges if running setuid?>

		// Let the shell do the dirty work
		execlp("/bin/sh", "/bin/sh", "-c", ++device_name, (char *)NULL);

		// exec failed!
		printf("serial_open:  could not exec %s: %s\n",
			   "/bin/sh", strerror(errno));
		exit(1);
		break;
	default:				// parent
		// Pid was stored above
		break;
	}

	return true;
}


/*
 *  Configure serial port with MacOS config word
 */

bool XSERDPort::configure(uint16 config)
{
	D(bug(" configure %04x\n", config));
	SERIAL_LOG("XSERDPort::configure: '%s' config %04x\n", device_name, config);
	if (protocol != serial)
		return true;

	// Number of stop bits
	switch (config & 0xc000) {
		case stop10: stop_bits = 1; break;
		case stop20: stop_bits = 2; break;
		default: return false;
	}

	// Parity mode
	switch (config & 0x3000) {
		case noParity: parity = 0; break;
		case oddParity: parity = 1; break;
		case evenParity: parity = 2; break;
		default: return false;
	}

	// Number of data bits
	switch (config & 0x0c00) {
		case data5: data_bits = 5; break;
		case data6: data_bits = 6; break;
		case data7: data_bits = 7; break;
		case data8: data_bits = 8; break;
	}

	// Baud rate
	switch (config & 0x03ff) {
		case baud150: baud = 150; break;
		case baud300: baud = 300; break;
		case baud600: baud = 600; break;
		case baud1200: baud = 1200; break;
		case baud1800: baud = 1800; break;
		case baud2400: baud = 2400; break;
		case baud3600: baud = 3600; break;
		case baud4800: baud = 4800; break;
		case baud7200: baud = 7200; break;
		case baud9600: baud = 9600; break;
		case baud14400: baud = 14400; break;
		case baud19200: baud = 19200; break;
		case baud28800: baud = 28800; break;
		case baud38400: baud = 38400; break;
		case baud57600: baud = 57600; break;
		default:
			return false;
	}
	apply_line();
	return true;
}


/*
 *  Apply the current line settings to the host device
 */

void XSERDPort::apply_line(void)
{
	if (fd < 0 || protocol != serial)
		return;

	// With external clocking the bit rate is set by the device on HSKi (x1 mode)
	uint32 rate = baud;
	if (ext_clock) {
		int32 clock = PrefsFindInt32("sccextclock");
		rate = clock > 0 ? clock : 1000000;
	}
	uint32 host_rate = serial_apply_line(fd, &mode, rate, data_bits, parity, stop_bits, emuser, &custom_speed);
	SERIAL_LOG("XSERDPort::apply_line: '%s' %u baud %d%c%d%s -> host %u baud\n", device_name, rate,
		data_bits, "NOE"[parity], stop_bits, ext_clock ? " (external clock)" : "", host_rate);
}


/*
 *  Set serial handshaking
 */

void XSERDPort::set_handshake(uint32 s, bool with_dtr)
{
	D(bug(" set_handshake %02x %02x %02x %02x %02x %02x %02x %02x\n",
		ReadMacInt8(s + 0), ReadMacInt8(s + 1), ReadMacInt8(s + 2), ReadMacInt8(s + 3),
		ReadMacInt8(s + 4), ReadMacInt8(s + 5), ReadMacInt8(s + 6), ReadMacInt8(s + 7)));
	if (protocol != serial)
		return;

	if (with_dtr) {
		if (ReadMacInt8(s + shkFCTS) || ReadMacInt8(s + shkFDTR))
			mode.c_cflag |= CRTSCTS;
		else
			mode.c_cflag &= ~CRTSCTS;
	} else {
		if (ReadMacInt8(s + shkFCTS))
			mode.c_cflag |= CRTSCTS;
		else
			mode.c_cflag &= ~CRTSCTS;
	}

	D(bug(" %sware flow control\n", mode.c_cflag & CRTSCTS ? "hard" : "soft"));
	serial_set_mode(fd, &mode, custom_speed);
}


/*
 *  Data input thread
 */

void *XSERDPort::input_func(void *arg)
{
	XSERDPort *s = (XSERDPort *)arg;
	while (!s->input_thread_cancel) {

		// Wait for commands
		sem_wait(&s->input_signal);
		if (s->quitting)
			break;

		// Execute command; like the Mac Serial Driver, complete only when all
		// requested bytes have arrived (or on KillIO/close)
		uint8 *buf = (uint8 *)Mac2HostAddr(ReadMacInt32(s->input_pb + ioBuffer));
		uint32 length = ReadMacInt32(s->input_pb + ioReqCount);
		D(bug("input_func waiting for %ld bytes of data...\n", length));
		int32 actual = 0;
		while (actual < (int32)length && !s->io_killed && !s->quitting) {
			struct pollfd pfd;
			pfd.fd = s->fd;
			pfd.events = POLLIN;
			pfd.revents = 0;
			int res = poll(&pfd, 1, INPUT_POLL_TIMEOUT_MS);
			if (res < 0 && errno != EINTR) {
				actual = -1;
				break;
			}
			if (res <= 0)
				continue;
			ssize_t n = 0;
			if (pfd.revents & POLLIN)
				n = read(s->fd, buf + actual, length - actual);
			if (n > 0)
				actual += n;
			else if (n < 0 && errno != EAGAIN && errno != EINTR) {
				actual = -1;
				break;
			} else if ((pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) || ((pfd.revents & POLLIN) && n == 0)) {
				// Hangup/EOF, e.g. PTY peer (proxy) not connected: wait instead of spinning
				usleep(INPUT_POLL_TIMEOUT_MS * 1000);
			}
		}
		D(bug(" %ld bytes received\n", actual));
		if (s->quitting)
			break;
		if (actual > 0 && serial_debug) {
			fprintf(stderr, "XSERDPort: received %d bytes:", actual);
			for (int32 i = 0; i < actual && i < 32; i++)
				fprintf(stderr, " %02x", buf[i]);
			fprintf(stderr, actual > 32 ? " ...\n" : "\n");
		}

#if MONITOR
		bug("Receiving serial data:\n");
		uint8 *adr = (uint8 *)buf;
		for (int i=0; i<actual; i++) {
			bug("%02x ", adr[i]);
		}
		bug("\n");
#endif

		// KillIO called? Then simply return
		if (s->io_killed) {

			WriteMacInt16(s->input_pb + ioResult, uint16(abortErr));
			WriteMacInt32(s->input_pb + ioActCount, 0);
			s->read_pending = s->read_done = false;

		} else {

			// Set error code
			if (actual >= 0) {
				WriteMacInt32(s->input_pb + ioActCount, actual);
				WriteMacInt32(s->input_dt + serdtResult, noErr);
			} else {
				WriteMacInt32(s->input_pb + ioActCount, 0);
				WriteMacInt32(s->input_dt + serdtResult, uint16(readErr));
			}

			// Trigger serial interrupt
			D(bug(" triggering serial interrupt\n"));
			s->read_done = true;
			SetInterruptFlag(INTFLAG_SERIAL);
			TriggerInterrupt();
		}
	}
	return NULL;
}


/*
 *  Data output thread
 */

void *XSERDPort::output_func(void *arg)
{
	XSERDPort *s = (XSERDPort *)arg;
	while (!s->output_thread_cancel) {

		// Wait for commands
		sem_wait(&s->output_signal);
		if (s->quitting)
			break;

		// Execute command
		void *buf = Mac2HostAddr(ReadMacInt32(s->output_pb + ioBuffer));
		uint32 length = ReadMacInt32(s->output_pb + ioReqCount);
		D(bug("output_func transmitting %ld bytes of data...\n", length));

#if MONITOR
		bug("Sending serial data:\n");
		uint8 *adr = (uint8 *)buf;
		for (int i=0; i<length; i++) {
			bug("%02x ", adr[i]);
		}
		bug("\n");
#endif

		int32 actual = 0;
		while (actual < (int32)length && !s->io_killed) {
			ssize_t n = write(s->fd, (uint8 *)buf + actual, length - actual);
			if (n > 0)
				actual += n;
			else if (n < 0 && errno == EINTR)
				continue;
			else {
				if (actual == 0)
					actual = -1;
				break;
			}
		}
		D(bug(" %ld bytes transmitted\n", actual));
		if (actual > 0 && serial_debug) {
			fprintf(stderr, "XSERDPort: sent %d bytes:", actual);
			for (int32 i = 0; i < actual && i < 32; i++)
				fprintf(stderr, " %02x", ((uint8 *)buf)[i]);
			fprintf(stderr, actual > 32 ? " ...\n" : "\n");
		}

		// KillIO called? Then simply return
		if (s->io_killed) {

			WriteMacInt16(s->output_pb + ioResult, uint16(abortErr));
			WriteMacInt32(s->output_pb + ioActCount, 0);
			s->write_pending = s->write_done = false;

		} else {

			// Set error code
			if (actual >= 0) {
				WriteMacInt32(s->output_pb + ioActCount, actual);
				WriteMacInt32(s->output_dt + serdtResult, noErr);
			} else {
				WriteMacInt32(s->output_pb + ioActCount, 0);
				WriteMacInt32(s->output_dt + serdtResult, uint16(writErr));
			}
	
			// Trigger serial interrupt
			D(bug(" triggering serial interrupt\n"));
			s->write_done = true;
			SetInterruptFlag(INTFLAG_SERIAL);
			TriggerInterrupt();
		}
	}
	return NULL;
}
