/*
 * LD7 (PH7, Zephyr led0) as the Agent link status. See link_led.h.
 *
 * A periodic k_timer toggles the LED while the link is up, so it blinks
 * at a fixed M4-timed rate whatever the uros thread is doing.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "link_led.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define BLINK_PERIOD_MS 500

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool led_ready;

static void blink_expiry(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	gpio_pin_toggle_dt(&led);
}

static K_TIMER_DEFINE(blink_timer, blink_expiry, NULL);

int link_led_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	if (ret) {
		return ret;
	}
	led_ready = true;
	return 0;
}

void link_led_set(bool up)
{
	if (!led_ready) {
		return;
	}
	if (up) {
		/* first toggle right away: LD7 lights as soon as the link is up */
		k_timer_start(&blink_timer, K_NO_WAIT, K_MSEC(BLINK_PERIOD_MS));
	} else {
		k_timer_stop(&blink_timer);
		gpio_pin_set_dt(&led, 0);
	}
}
