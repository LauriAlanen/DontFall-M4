/*
 * Inter-processor link to Linux on the A7: OpenAMP rpmsg over the IPCC
 * mailbox, exposed to Linux as "rpmsg-tty" channels, one per ipc_channel.
 *
 * The rest of the firmware only sees byte pipes: received messages are
 * handed to the channel's rx callback, data goes out with ipc_send().
 */
#ifndef DONTFALL_M4_IPC_H_
#define DONTFALL_M4_IPC_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Linux numbers the ttys in the order the channels are announced, which is
 * the order below.
 */
enum ipc_channel {
	IPC_CH_CTRL,	/* /dev/ttyRPMSG0: line protocol (protocol.h) */
	IPC_CH_UROS,	/* /dev/ttyRPMSG1: micro-ROS, to the micro-ROS Agent */
	IPC_CH_COUNT,
};

/*
 * Called once per received rpmsg message, from the ipc rx thread (never
 * from an ISR), so it may block briefly and may call ipc_send().
 */
typedef void (*ipc_rx_cb_t)(const uint8_t *data, size_t len);

/*
 * Start the OpenAMP threads. rx_cbs has one callback per ipc_channel.
 * Returns 0, or a negative errno.
 */
int ipc_start(const ipc_rx_cb_t rx_cbs[IPC_CH_COUNT]);

/*
 * Send one message to Linux on a channel. Returns the bytes sent, or a
 * negative error. Fails until the channel is up and Linux has sent on it
 * at least once (rpmsg learns Linux's endpoint address from that).
 */
int ipc_send(enum ipc_channel ch, const void *data, size_t len);

#endif /* DONTFALL_M4_IPC_H_ */
