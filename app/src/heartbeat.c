/*
 * A7 liveness watchdog driving LD7 (PH7, Zephyr led0). See heartbeat.h.
 *
 * Two k_timers, both expiring in the system clock ISR:
 *  - watchdog: one-shot, re-armed by every kick; expiry = link lost.
 *  - blink:    periodic, runs only while the link is up.
 * The LED blinks at a fixed M4-timed rate, independent of A7 jitter.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "heartbeat.h"

#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define BLINK_PERIOD_MS 500

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool led_ready;

/* guards link_up against the watchdog ISR */
static struct k_spinlock lock;
static bool link_up;

static void blink_expiry(struct k_timer *timer)
{
	gpio_pin_toggle_dt(&led);
}

static K_TIMER_DEFINE(blink_timer, blink_expiry, NULL);

static void watchdog_expiry(struct k_timer *timer)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool was_up = link_up;

	link_up = false;
	if (was_up && led_ready) {
		k_timer_stop(&blink_timer);
		gpio_pin_set_dt(&led, 0);
	}
	k_spin_unlock(&lock, key);

	if (was_up) {
		printk("heartbeat: A7 link lost\n");
	}
}

static K_TIMER_DEFINE(watchdog_timer, watchdog_expiry, NULL);

int heartbeat_init(void)
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

void heartbeat_kick(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	bool was_up = link_up;

	link_up = true;
	k_timer_start(&watchdog_timer, K_MSEC(HEARTBEAT_TIMEOUT_MS), K_NO_WAIT);
	if (!was_up && led_ready) {
		/* first toggle right away: LD7 lights as soon as the link is up */
		k_timer_start(&blink_timer, K_NO_WAIT, K_MSEC(BLINK_PERIOD_MS));
	}
	k_spin_unlock(&lock, key);

	if (!was_up) {
		printk("heartbeat: A7 link up\n");
	}
}
