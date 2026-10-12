/*
 * DontFall M4 firmware: OpenAMP remote on the STM32MP157 Cortex-M4.
 *
 * Answers every message on the rpmsg-tty channel (Linux: /dev/ttyRPMSG0)
 * with "Moi from M4: <message>". Also toggles the orange user LED LD7
 * (PH7, Zephyr led0) every 500 ms.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "ipc.h"

#define REPLY_PREFIX     "Moi from M4: "
#define REPLY_PREFIX_LEN (sizeof(REPLY_PREFIX) - 1)
/* rpmsg payload limit with the default 512 byte buffers */
#define RPMSG_MAX_PAYLOAD 496

#define BLINK_PERIOD_MS 500

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void blink_expiry(struct k_timer *timer)
{
	gpio_pin_toggle_dt(&led);
}

K_TIMER_DEFINE(blink_timer, blink_expiry, NULL);

static void echo(const uint8_t *data, size_t len)
{
	static unsigned char tx_buff[RPMSG_MAX_PAYLOAD];
	size_t n = MIN(len, sizeof(tx_buff) - REPLY_PREFIX_LEN);

	memcpy(tx_buff, REPLY_PREFIX, REPLY_PREFIX_LEN);
	memcpy(&tx_buff[REPLY_PREFIX_LEN], data, n);
	ipc_send(tx_buff, REPLY_PREFIX_LEN + n);
}

int main(void)
{
	printk("dontfall-m4 up\n");

	if (!gpio_is_ready_dt(&led) ||
	    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE)) {
		printk("LD7 not available\n");
	} else {
		k_timer_start(&blink_timer, K_MSEC(BLINK_PERIOD_MS),
			      K_MSEC(BLINK_PERIOD_MS));
	}

	ipc_start(echo);
	return 0;
}
