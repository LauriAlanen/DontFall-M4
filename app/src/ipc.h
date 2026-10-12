/*
 * Inter-processor link to Linux on the A7: OpenAMP rpmsg over the IPCC
 * mailbox, exposed to Linux as the "rpmsg-tty" channel (/dev/ttyRPMSG0).
 *
 * The rest of the firmware only sees a byte pipe: received messages are
 * handed to the rx callback, replies go out with ipc_send().
 */
#ifndef DONTFALL_M4_IPC_H_
#define DONTFALL_M4_IPC_H_

#include <stddef.h>
#include <stdint.h>

/*
 * Called once per received rpmsg message, from the ipc rx thread (never
 * from an ISR), so it may block briefly and may call ipc_send().
 */
typedef void (*ipc_rx_cb_t)(const uint8_t *data, size_t len);

/* Start the OpenAMP threads. Returns 0, or a negative errno. */
int ipc_start(ipc_rx_cb_t rx_cb);

/* Send one message to Linux. Returns the bytes sent, or a negative error. */
int ipc_send(const void *data, size_t len);

#endif /* DONTFALL_M4_IPC_H_ */
