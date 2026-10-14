/*
 * DontFall M4 firmware: OpenAMP remote on the STM32MP157 Cortex-M4.
 *
 * Wires the modules together:
 *  - ipc:       rpmsg-tty channels to Linux (/dev/ttyRPMSG0, /dev/ttyRPMSG1)
 *  - protocol:  line protocol on the first channel (PING/PONG)
 *  - heartbeat: A7 liveness watchdog; LD7 blinks while the A7 is talking
 *  - uros:      micro-ROS node, to the micro-ROS Agent on the second channel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "heartbeat.h"
#include "ipc.h"
#include "protocol.h"
#include "uros.h"
#include "uros_transport.h"

static const ipc_rx_cb_t rx_cbs[IPC_CH_COUNT] = {
	[IPC_CH_CTRL] = protocol_rx,
	[IPC_CH_UROS] = uros_transport_rx,
};

int main(void)
{
	int ret;

	printk("dontfall-m4 up\n");

	ret = heartbeat_init();
	if (ret) {
		/* keep the link up anyway: PONGs still prove the M4 is alive */
		printk("LD7 not available: %d\n", ret);
	}

	ret = ipc_start(rx_cbs);
	if (ret) {
		printk("ipc_start failed: %d\n", ret);
	}

	ret = uros_start();
	if (ret) {
		printk("uros_start failed: %d\n", ret);
	}
	return 0;
}
