/*
 * A7 <-> M4 command protocol on the rpmsg-tty channel.
 *
 * ASCII lines terminated by '\n' (a trailing '\r' is ignored), so it can
 * be driven by hand with echo/cat on /dev/ttyRPMSG0. Lines may be split
 * across or packed into rpmsg messages; the M4 reassembles them.
 *
 *   A7 -> M4             M4 -> A7
 *   PING <seq>           PONG <seq>      heartbeat, <seq> = 1-10 digits
 *   anything else        ERR unknown
 *   line > 31 chars      ERR too long
 *
 * Interim format: micro-ROS replaces it (see VISION.md).
 */
#ifndef DONTFALL_M4_PROTOCOL_H_
#define DONTFALL_M4_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

/* Feed bytes received from the A7; matches ipc_rx_cb_t. */
void protocol_rx(const uint8_t *data, size_t len);

#endif /* DONTFALL_M4_PROTOCOL_H_ */
