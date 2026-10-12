/*
 * DontFall M4 firmware: OpenAMP remote on the STM32MP157 Cortex-M4.
 *
 * Wires the modules together:
 *  - ipc:       rpmsg-tty channel to Linux (/dev/ttyRPMSG0)
 *  - protocol:  line protocol on that channel (PING/PONG)
 *  - heartbeat: A7 liveness watchdog; LD7 blinks while the A7 is talking
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "heartbeat.h"
#include "ipc.h"
#include "protocol.h"

int main(void)
{
	int ret;

	printk("dontfall-m4 up\n");

	ret = heartbeat_init();
	if (ret) {
		/* keep the link up anyway: PONGs still prove the M4 is alive */
		printk("LD7 not available: %d\n", ret);
	}

	ret = ipc_start(protocol_rx);
	if (ret) {
		printk("ipc_start failed: %d\n", ret);
	}
	return 0;
}
