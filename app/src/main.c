/*
 * DontFall M4 firmware: OpenAMP remote on the STM32MP157 Cortex-M4.
 *
 * Wires the modules together:
 *  - ipc:       rpmsg-tty channel to Linux (/dev/ttyRPMSG0)
 *  - link_led:  LD7 blinks while the micro-ROS Agent session is up
 *  - uros:      micro-ROS node, to the micro-ROS Agent on the channel
 *  - imu:       LSM6DSOX on SPI5 (bring-up: data-ready sampling, timing printk)
 *  - control:   the supervisor (arm/disarm), fed by every IMU sample
 *  - wdog:      WWDG1 resets the M4 if it hangs
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "control.h"
#include "imu.h"
#include "ipc.h"
#include "link_led.h"
#include "uros.h"
#include "uros_transport.h"
#include "wdog.h"

static const ipc_rx_cb_t rx_cbs[IPC_CH_COUNT] = {
	[IPC_CH_UROS] = uros_transport_rx,
};

int main(void)
{
	int ret;

	printk("dontfall-m4 up\n");

	ret = link_led_init();
	if (ret) {
		/* only the status LED is missing: run anyway */
		printk("LD7 not available: %d\n", ret);
	}

	ret = ipc_start(rx_cbs);
	if (ret) {
		printk("ipc_start failed: %d\n", ret);
	}

	/* before the IMU: it starts calling control_imu_sample() */
	control_init();

	/* after control_init(): its timer is a check-in source */
	ret = wdog_start();
	if (ret) {
		printk("wdog_start failed: %d\n", ret);
	}

	ret = imu_init(control_imu_sample);
	if (ret) {
		printk("imu_init failed: %d\n", ret);
	}

	ret = uros_start();
	if (ret) {
		printk("uros_start failed: %d\n", ret);
	}
	return 0;
}
