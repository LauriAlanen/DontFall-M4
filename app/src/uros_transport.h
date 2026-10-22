/*
 * micro-ROS custom transport over the IPC_CH_UROS rpmsg channel. On Linux
 * that is /dev/ttyRPMSG0, where the stock micro-ROS Agent runs in serial
 * mode:
 *
 *   micro-ros-agent serial --dev /dev/ttyRPMSG0
 *
 * It is a byte stream with XRCE-DDS framing (HDLC-like, 0x7E-delimited,
 * CRC-checked), the same framing the Agent uses on a UART, so rpmsg message
 * boundaries don't matter and the Agent needs no custom code.
 *
 * The A7 must write to /dev/ttyRPMSG0 once before starting the Agent (any
 * byte, e.g. a newline): rpmsg only learns Linux's endpoint address from a
 * first message, and an XRCE-DDS client always speaks first. Bytes outside
 * a frame are ignored.
 */
#ifndef DONTFALL_M4_UROS_TRANSPORT_H_
#define DONTFALL_M4_UROS_TRANSPORT_H_

#include <stddef.h>
#include <stdint.h>

/* Feed bytes received on IPC_CH_UROS; matches ipc_rx_cb_t. */
void uros_transport_rx(const uint8_t *data, size_t len);

/* Register the transport with micro-ROS. Returns 0, or a negative errno. */
int uros_transport_init(void);

/* Drop buffered rx bytes, e.g. leftovers from a lost Agent. */
void uros_transport_reset(void);

#endif /* DONTFALL_M4_UROS_TRANSPORT_H_ */
