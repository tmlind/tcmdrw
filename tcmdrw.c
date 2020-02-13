/*
 * Read and write for Motorola TCMD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <linux/tty.h>

#define DEBUG	0

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof(a[0]))
#define TCMD_REG_MAX_OFFSET	70282	/* May not be correct? */

struct tcmd_data {
	const char *port;
	int baudrate;
	int fd;
	int debug;
	char *buf;
	int buf_sz;
	int read;
};

static int uart_init(struct tcmd_data *d, const char *port, int baudrate)
{
	struct termios t;
	int error;

	d->port = port;
	d->baudrate = baudrate;
	d->debug = DEBUG;

	d->fd = open(d->port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (d->fd < 0) {
		fprintf(stderr, "Could not open %s: %i\n",
			d->port, d->fd);

		return d->fd;
	}

	tcgetattr(d->fd, &t);
	cfmakeraw(&t);
	error = tcflush(d->fd, TCIOFLUSH);
	if (error < 0) {
		fprintf(stderr, "Failed to tcsetattr: %s\n",
			strerror(errno));
		goto close;
	}

	cfsetispeed(&t, d->baudrate);
	cfsetospeed(&t, d->baudrate);
	t.c_iflag &= ~(IXON | IXOFF);
	t.c_lflag |= CRTSCTS;

	error = tcsetattr(d->fd, TCSANOW, &t);
	if (error < 0)
		fprintf(stderr, "Failed to tcsetattr: %s\n",
			strerror(errno));

close:
	if (error < 0) {
		close(d->fd);
		d->fd = -ENODEV;
	}

	return error;
}

static int uart_poll(struct tcmd_data *d, int timeout_ms)
{
	struct pollfd pfd;
	int error;

	pfd.fd = d->fd;
	pfd.events = POLLIN;

	error = poll(&pfd, 1, 1000);
	if (error == 0)
		fprintf(stderr, "Timed out\n");
	else if (error < 0)
		fprintf(stderr, "Poll error: %i\n", error);

	return error;
}

static int uart_read(struct tcmd_data *d)
{
	int error, i;

	memset(d->buf, '\0', d->buf_sz);
	d->read = 0;

	error = read(d->fd, d->buf, d->buf_sz);
	if (error < 0) {
		fprintf(stderr, "Failed to read: %i %s\n",
			error, strerror(errno));

		return error;
	}

	d->read += error;

	if (d->debug) {
		printf("read:  ");
		for (i = 0; i < d->read; i++)
			printf("%02x ", d->buf[i]);
		printf("\n");
	}

	return error;
}

#define TCMD_RESPONSE_LEN	12

enum tcmd_command {
	TCMD_ADDR_H = 12,
	TCMD_ADDR_L = 13,
	TCMD_DATA_START = 20,
};

enum tcmd_response {
	TCMD_RESPONSE_STATUS = 5,
	TCMD_RESPONSE_DATA_TYPE = 11,
};

enum tcmd_data_types {
	TCMD_DATA_TYPE_BYTE = 0x01,
	TCMD_DATA_TYPE_256 = 0x81,
};

enum tcmd_error {
	TCMD_ERROR_0,
	TCMD_ERROR_1,
	TCMD_ERROR_2,
	TCMD_ERROR_3,
	TCMD_ERROR_4,
	TCMD_ERROR_5,
	TCMD_ERROR_NONE,
};

static void tcmd_print_response(struct tcmd_data *d, unsigned int offset,
				int debug)
{
	int i, data_start;

	printf("%05i:0x%05x:", offset, offset);

	if (debug) {
		for (i = 0; i < d->read && i < TCMD_RESPONSE_LEN; i++)
			printf("%02x", d->buf[i]);
		printf(":");
	}

	if (d->read <= TCMD_RESPONSE_LEN) {
		printf("\n");
		return;
	}

	/*
	 * Quirk handling for longer data: First data byte is always zero
	 * and must be ignored as otherwise read values written back will
	 * shift the data by two bytes to right.
	 */
	if (d->buf[TCMD_RESPONSE_DATA_TYPE] == TCMD_DATA_TYPE_256)
		data_start = TCMD_RESPONSE_LEN + 1;
	else
		data_start = TCMD_RESPONSE_LEN;

	for (i = data_start; i < d->read; i++)
		printf("%02x", d->buf[i]);
	printf("\n");
}

/*
 * Command VERSION
 */
static int tcmd_nv_read_revision(struct tcmd_data *d)
{
	const unsigned char cmd[] = {
		0x00, 0x03, 0x00, 0x39, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x02, 0xff, 0xff,
	};
	int i, len = ARRAY_SIZE(cmd);
	int error;

	if (d->debug) {
		printf("write: ");
		for (i = 0; i < len; i++)
			printf("%02x ", cmd[i]);
		printf("\n");
	}

	error = write(d->fd, cmd, len);
	if (error <= 0) {
		fprintf(stderr, "Failed to write: %i %s\n",
			error, strerror(errno));

		return error;
	}

	error = uart_poll(d, 1500);
	if (error <= 0)
		return error;

	error = uart_read(d);

	if (d->debug && d->read > 12 && d->read < d->buf_sz)
		printf("Found modem revision %s\n", d->buf + 12);

	return error;
}

/*
 * Command RDELEM
 */
static int tcmd_nv_read_reg(struct tcmd_data *d, unsigned int offset)
{
	unsigned char cmd[] = {
		0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x08, 0xff, 0xff, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x80,
	};
	int i, len = ARRAY_SIZE(cmd);
	int error;

	cmd[TCMD_ADDR_H] = offset >> 8;
	cmd[TCMD_ADDR_L] = offset & 0xff;

	if (d->debug) {
		printf("write: ");
		for (i = 0; i < len; i++)
			printf("%02x ", cmd[i]);
		printf("\n");
	}

	error = write(d->fd, cmd, len);
	if (error <= 0) {
		fprintf(stderr, "Failed to write: %i %s\n",
			error, strerror(errno));

		return error;
	}

	error = uart_poll(d, 1500);
	if (error <= 0)
		return error;

	error = uart_read(d);

	if (d->read < TCMD_RESPONSE_LEN) {
		fprintf(stderr, "Short response\n");
		tcmd_print_response(d, offset, 1);

		return ENODEV;
	}

	if (d->buf[TCMD_RESPONSE_STATUS] != TCMD_ERROR_NONE) {
		error = -(d->buf[TCMD_RESPONSE_STATUS]);
		tcmd_print_response(d, offset, d->debug);

		return error;
	}

	tcmd_print_response(d, offset, d->debug);

	return error;
}

static int tcmd_dump_all(struct tcmd_data *d)
{
	int nv_len, i;

	for (i = 0; i <= TCMD_REG_MAX_OFFSET; i++) {
		nv_len = tcmd_nv_read_reg(d, i);
		if (nv_len <= TCMD_RESPONSE_LEN)
			return -EINVAL;
	}

	return 0;
}

static int bytes_to_hex(const char *buf)
{
	char tmp[3];

	strncpy(tmp, buf, 2);
	tmp[2] = '\0';

	return strtoul(tmp, NULL, 16);
}

/*
 * Command STELEM
 *
 * Last header byte is size: 0x08 means 0x80 * 2 = 256 bytes
 */
static int tcmd_nv_write_reg(struct tcmd_data *d, const char *val, unsigned int offset)
{
	const unsigned char cmd_header[] = {
		0x00, 0x01, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x10, 0xff, 0xff, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x08,
	};
	unsigned char *cmd;
	int i, error = 0, len, vlen, nv_len, header_len = ARRAY_SIZE(cmd_header);

	nv_len = tcmd_nv_read_reg(d, offset);
	if (nv_len <= TCMD_RESPONSE_LEN)
		return -EINVAL;

	nv_len -= TCMD_RESPONSE_LEN;

	/*
	 * Quirk handling for uninialized NV value. It seems value of 08 is
	 * is uninitialized while 07 is unavailable.
	 */
	 if (nv_len == 1)
		nv_len = 129;

	/* Quirk handling for extra two bytes returned by reads */
	if (d->buf[TCMD_RESPONSE_DATA_TYPE] == TCMD_DATA_TYPE_256)
		nv_len -= 1;

	vlen = strlen(val);
	if (vlen > nv_len * 2) {
		fprintf(stderr, "Wrong length for %05i %05x: %i vs %i\n",
			offset, offset, vlen, nv_len * 2);

		return -EINVAL;
	}

	len = header_len + nv_len;
	cmd = calloc(1, len);
	if (!cmd)
		return -ENOMEM;

	memcpy(cmd, cmd_header, header_len);

	cmd[TCMD_ADDR_H] = offset >> 8;
	cmd[TCMD_ADDR_L] = offset & 0xff;

	for (i = 0; i < nv_len; i++) {
		int offset, hexval;
		const char *cur;

		offset = i * 2;
		cur = val + offset;

		/* Pad with zeros if needed */
		if (offset < vlen)
			hexval = bytes_to_hex(cur);
		else
			hexval = 0;

		cmd[TCMD_DATA_START + i] = hexval;
	}

	if (d->debug) {
		printf("write: ");
		for (i = 0; i < len; i++)
			printf("%02x ", cmd[i]);
		printf("\n");
	}

	error = write(d->fd, cmd, len);
	if (error <= 0) {
		fprintf(stderr, "Failed to write: %i %s\n",
			error, strerror(errno));

		return error;
	}

	error = uart_poll(d, 1500);
	if (error <= 0)
		return error;

	error = uart_read(d);

	if (d->read < TCMD_RESPONSE_LEN) {
		fprintf(stderr, "Short response\n");
		tcmd_print_response(d, offset, 1);

		return ENODEV;
	}

	if (d->buf[TCMD_RESPONSE_STATUS] != TCMD_ERROR_NONE) {
		error = -(d->buf[TCMD_RESPONSE_STATUS]);
		tcmd_print_response(d, offset, d->debug);

		return error;
	}

	tcmd_print_response(d, offset, d->debug);

	free(cmd);

	return error;

}

static int parse_value(char *token)
{
	if ((strlen(token) > 2) && ((token[1] == 'x') || (token[1] == 'X')))
		return strtoul(token, NULL, 16);

	return strtoul(token, NULL, 10);
}

int main(int argc, char *argv[])
{
	struct tcmd_data d;
	const char *uart = "/dev/ttyUSB3";
	const char delimiters[] = { "=", };
	char *running, *token;
	char *val = NULL;
	int offset;
	int error;

	if (argc < 2) {
		printf("usage: %s [--all|offset[+range][=value]]\n", argv[0]);

		return -EINVAL;
	}

	error = uart_init(&d, uart, B9600);
	if (error < 0)
		return error;

	d.buf_sz = 4096;
	d.buf = malloc(d.buf_sz);
	if (!d.buf) {
		error = -ENOMEM;
		goto close;
	}

	running = malloc(strlen(argv[1]) + 1);
	strncpy(running, argv[1], strlen(argv[1]) + 1);

	if (!strncmp("--all", running, 5)) {
		error = tcmd_dump_all(&d);
		goto free;
	}

	token = strsep(&running, delimiters);
	if (token) {
		offset = parse_value(token);

		token = strsep(&running, delimiters);
		if (token) {
			val = malloc(strlen(token));
			if (!val) {
				error = -ENOMEM;
				goto free;
			}
			val = strdup(token);
		}
	} else {
		offset = parse_value(argv[1]);
	}

	error = tcmd_nv_read_revision(&d);
	if (error <= 0)
		goto free;

	if (!val)
		error = tcmd_nv_read_reg(&d, offset);
	else
		error = tcmd_nv_write_reg(&d, val, offset);
	if (error)
		goto free;

free:
	free(val);
	free(d.buf);
	free(running);
close:
	close(d.fd);

	return error;
}
