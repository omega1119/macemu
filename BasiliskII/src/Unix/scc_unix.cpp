/*
 *  scc_unix.cpp - Host side of the SCC register emulation, Unix specific stuff
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

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <atomic>

#include "scc.h"
#include "serial_line_unix.h"


// Single producer / single consumer byte ring
class SCCRing {
public:
	enum { SIZE = 65536 };

	SCCRing() : head(0), tail(0) {}
	bool empty(void) const { return head.load(std::memory_order_acquire) == tail.load(std::memory_order_acquire); }
	bool full(void) const { return ((head.load(std::memory_order_acquire) + 1) & (SIZE - 1)) == tail.load(std::memory_order_acquire); }
	uint32 free_space(void) const { return (tail.load(std::memory_order_acquire) - head.load(std::memory_order_acquire) - 1) & (SIZE - 1); }
	bool put(uint8 b)
	{
		uint32 h = head.load(std::memory_order_relaxed);
		uint32 n = (h + 1) & (SIZE - 1);
		if (n == tail.load(std::memory_order_acquire))
			return false;
		buf[h] = b;
		head.store(n, std::memory_order_release);
		return true;
	}
	int get(void)
	{
		uint32 t = tail.load(std::memory_order_relaxed);
		if (t == head.load(std::memory_order_acquire))
			return -1;
		uint8 b = buf[t];
		tail.store((t + 1) & (SIZE - 1), std::memory_order_release);
		return b;
	}

private:
	uint8 buf[SIZE];
	std::atomic<uint32> head;
	std::atomic<uint32> tail;
};


class UnixSCCChannel : public SCCHostChannel {
public:
	UnixSCCChannel(int channel, const char *pref);
	virtual ~UnixSCCChannel();

	virtual bool open(void);
	virtual bool is_open(void) { return fd >= 0; }
	virtual bool rx_available(void) { return !rx.empty(); }
	virtual int read_byte(void) { return rx.get(); }
	virtual bool tx_ready(void) { return !tx.full(); }
	virtual void write_byte(uint8 b);
	virtual void set_line(const SCCLineSettings &s);
	virtual void flush_rx(void);

private:
	static void *io_func(void *arg);
	void wake(void);

	int channel;
	const char *path;
	bool emuser;
	int fd;
	int wake_pipe[2];
	struct termios mode;

	pthread_t io_thread;
	bool io_thread_active;
	std::atomic<bool> quit;
	std::atomic<bool> flush_request;

	SCCRing rx;		// Host -> Mac (filled by I/O thread)
	SCCRing tx;		// Mac -> host (drained by I/O thread)
};


UnixSCCChannel::UnixSCCChannel(int channel, const char *pref)
	: channel(channel), fd(-1), io_thread_active(false), quit(false), flush_request(false)
{
	path = serial_parse_path(pref, &emuser);
	wake_pipe[0] = wake_pipe[1] = -1;
}

UnixSCCChannel::~UnixSCCChannel()
{
	if (io_thread_active) {
		quit = true;
		wake();
		pthread_join(io_thread, NULL);
		io_thread_active = false;
	}
	if (fd >= 0)
		::close(fd);
	if (wake_pipe[0] >= 0) {
		::close(wake_pipe[0]);
		::close(wake_pipe[1]);
	}
}

bool UnixSCCChannel::open(void)
{
	if (fd >= 0)
		return true;
	if (path == NULL || path[0] == '\0' || path[0] == '|' || strcmp(path, "midi") == 0)
		return false;

	fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		fprintf(stderr, "SCC %c: cannot open %s: %s\n", "AB"[channel], path, strerror(errno));
		return false;
	}

	if (tcgetattr(fd, &mode) == 0) {
		cfmakeraw(&mode);
		mode.c_cflag |= CLOCAL | CREAD;
		mode.c_cflag &= ~(CRTSCTS | HUPCL);
		mode.c_iflag &= ~(IXON | IXOFF | IXANY);
		mode.c_cc[VMIN] = 0;
		mode.c_cc[VTIME] = 0;
		tcsetattr(fd, TCSANOW, &mode);
	}

	if (pipe(wake_pipe) < 0) {
		::close(fd);
		fd = -1;
		return false;
	}
	fcntl(wake_pipe[0], F_SETFL, O_NONBLOCK);
	fcntl(wake_pipe[1], F_SETFL, O_NONBLOCK);

	quit = false;
	io_thread_active = (pthread_create(&io_thread, NULL, io_func, this) == 0);
	if (!io_thread_active) {
		::close(fd);
		fd = -1;
		return false;
	}
	printf("SCC %c: using %s%s\n", "AB"[channel], path, emuser ? " (EMuSer baud translation)" : "");
	return true;
}

void UnixSCCChannel::wake(void)
{
	if (wake_pipe[1] >= 0) {
		char c = 0;
		write(wake_pipe[1], &c, 1);
	}
}

void UnixSCCChannel::write_byte(uint8 b)
{
	bool was_empty = tx.empty();
	if (!tx.put(b))
		return;		// Overrun, the Mac program ignored "Tx buffer empty"
	if (was_empty)
		wake();
}

void UnixSCCChannel::set_line(const SCCLineSettings &s)
{
	if (fd < 0)
		return;
	// The Emax fast transfer mode is synchronous and clocked by the sampler,
	// which the EMuSer firmware handles itself when set to "500000 baud"
	uint32 custom_speed;
	uint32 host_baud = serial_apply_line(fd, &mode, s.baud, s.data_bits, s.parity, s.stop_bits, emuser, &custom_speed);
	if (emuser)
		printf("SCC %c: %u baud requested, EMuSer set to %u\n", "AB"[channel], s.baud, host_baud);
}

void UnixSCCChannel::flush_rx(void)
{
	while (rx.get() >= 0)
		;
	flush_request = true;
	wake();
}

void *UnixSCCChannel::io_func(void *arg)
{
	UnixSCCChannel *s = (UnixSCCChannel *)arg;
	uint8 buf[4096];

	while (!s->quit) {
		if (s->flush_request.exchange(false))
			tcflush(s->fd, TCIFLUSH);

		struct pollfd pfd[2];
		pfd[0].fd = s->fd;
		pfd[0].events = 0;
		if (s->rx.free_space() > 0)
			pfd[0].events |= POLLIN;
		if (!s->tx.empty())
			pfd[0].events |= POLLOUT;
		pfd[0].revents = 0;
		pfd[1].fd = s->wake_pipe[0];
		pfd[1].events = POLLIN;
		pfd[1].revents = 0;

		// Short timeout so a full receive ring is re-checked regularly
		int res = poll(pfd, 2, pfd[0].events & POLLIN ? 100 : 5);
		if (res < 0) {
			if (errno == EINTR)
				continue;
			break;
		}

		if (pfd[1].revents & POLLIN) {
			char dummy[64];
			while (read(s->wake_pipe[0], dummy, sizeof(dummy)) > 0)
				;
		}

		if (pfd[0].revents & POLLIN) {
			uint32 space = s->rx.free_space();
			if (space > sizeof(buf))
				space = sizeof(buf);
			ssize_t n = read(s->fd, buf, space);
			for (ssize_t i = 0; i < n; i++)
				s->rx.put(buf[i]);
		}

		if ((pfd[0].revents & POLLOUT) || (!s->tx.empty() && (pfd[1].revents & POLLIN))) {
			uint32 n = 0;
			int b;
			while (n < sizeof(buf) && (b = s->tx.get()) >= 0)
				buf[n++] = b;
			uint32 done = 0;
			while (done < n) {
				ssize_t w = write(s->fd, buf + done, n - done);
				if (w > 0) {
					done += w;
					continue;
				}
				if (w < 0 && errno == EINTR)
					continue;
				if (w < 0 && errno == EAGAIN) {
					struct pollfd wp;
					wp.fd = s->fd;
					wp.events = POLLOUT;
					wp.revents = 0;
					poll(&wp, 1, 100);
					if (s->quit)
						break;
					continue;
				}
				break;		// Device gone, drop data
			}
		}

		if (pfd[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
			// PTY peer (proxy) not connected; avoid spinning
			usleep(10000);
		}
	}
	return NULL;
}


SCCHostChannel *SCCCreateHostChannel(int channel, const char *path)
{
	return new UnixSCCChannel(channel, path);
}
