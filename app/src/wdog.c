/*
 * WWDG1 and its kicker. See wdog.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wdog.h"

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

/* WWDG1 counts PCLK1 (~104 MHz) / 4096 / up to 128: at most ~320 ms.
 * 100 ms: well above the kick period and any normal blocking, short
 * enough that a hang is caught before the robot has fallen far.
 */
#define WDOG_TIMEOUT_MS 100
#define WDOG_KICK_MS    20
#define WDOG_STACK_SIZE 512

#define ALL_SOURCES ((1u << WDOG_SOURCE_COUNT) - 1)

K_THREAD_STACK_DEFINE(wdog_stack, WDOG_STACK_SIZE);
static struct k_thread wdog_thread;

static const struct device *const wwdg = DEVICE_DT_GET(DT_NODELABEL(wwdg1));
static atomic_t seen; /* bit per source that checked in since the last kick */

void wdog_checkin(enum wdog_source source)
{
	atomic_or(&seen, BIT(source));
}

static void wdog_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(WDOG_KICK_MS);
		/* take the bits and start collecting again in one step */
		if ((atomic_clear(&seen) & ALL_SOURCES) == ALL_SOURCES) {
			wdt_feed(wwdg, 0);
		}
	}
}

int wdog_start(void)
{
	const struct wdt_timeout_cfg cfg = {
		.window = {.min = 0, .max = WDOG_TIMEOUT_MS},
		.flags = WDT_FLAG_RESET_SOC,
	};
	int ret;

	if (!device_is_ready(wwdg)) {
		return -ENODEV;
	}
	ret = wdt_install_timeout(wwdg, &cfg);
	if (ret < 0) {
		return ret;
	}
	/* no options: pausing it under a debugger would write DBGMCU */
	ret = wdt_setup(wwdg, 0);
	if (ret) {
		return ret;
	}

	k_thread_create(&wdog_thread, wdog_stack, K_THREAD_STACK_SIZEOF(wdog_stack), wdog_task,
			NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&wdog_thread, "wdog");
	printk("wdog: WWDG1 %d ms\n", WDOG_TIMEOUT_MS);
	return 0;
}
