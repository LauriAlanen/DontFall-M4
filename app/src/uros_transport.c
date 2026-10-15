/*
 * micro-ROS custom transport over rpmsg. See uros_transport.h.
 *
 * rx: the ipc rx thread pushes received bytes into a k_pipe; micro-ROS
 *     reads them from its own thread with a timeout.
 * tx: straight to ipc_send(). Framing writes in small pieces, so every
 *     write fits in one rpmsg buffer.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uros_transport.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <rmw_microros/rmw_microros.h>

#include "ipc.h"

/* room for a few MTU-sized frames while the micro-ROS thread is busy */
#define RX_PIPE_SIZE 2048

K_PIPE_DEFINE(rx_pipe, RX_PIPE_SIZE, 4);

void uros_transport_rx(const uint8_t *data, size_t len)
{
	size_t written;

	/*
	 * Never block the ipc rx thread. A dropped piece breaks a frame's CRC;
	 * XRCE-DDS reliable streams resend it.
	 */
	k_pipe_put(&rx_pipe, data, len, &written, 0, K_NO_WAIT);
	if (written < len) {
		printk("uros: rx pipe full, %u bytes dropped\n", (unsigned int)(len - written));
	}
}

static bool transport_open(struct uxrCustomTransport *transport)
{
	ARG_UNUSED(transport);
	return true;
}

static bool transport_close(struct uxrCustomTransport *transport)
{
	ARG_UNUSED(transport);
	return true;
}

static size_t transport_write(struct uxrCustomTransport *transport, const uint8_t *buf,
			      size_t len, uint8_t *err)
{
	ARG_UNUSED(transport);

	/* fails until Linux has written to the channel once (see header) */
	int ret = ipc_send(IPC_CH_UROS, buf, len);

	if (ret < 0) {
		*err = 1;
		return 0;
	}
	return (size_t)ret;
}

static size_t transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len,
			     int timeout, uint8_t *err)
{
	ARG_UNUSED(transport);
	ARG_UNUSED(err);

	size_t read = 0;

	/* wait up to timeout ms for the first byte, then take what's there */
	k_pipe_get(&rx_pipe, buf, len, &read, 1, K_MSEC(timeout));
	return read;
}

int uros_transport_init(void)
{
	rmw_ret_t ret = rmw_uros_set_custom_transport(true /* framing */, NULL, transport_open,
						      transport_close, transport_write,
						      transport_read);

	return ret == RMW_RET_OK ? 0 : -EIO;
}

void uros_transport_reset(void)
{
	k_pipe_flush(&rx_pipe);
}
