/*
 * Line assembly and command dispatch for the A7 <-> M4 protocol (see
 * protocol.h). Runs only in the ipc rx thread, so the line buffer needs
 * no locking.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "protocol.h"

#include <stdbool.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#include "heartbeat.h"
#include "ipc.h"

/* longest line including the terminating NUL */
#define PROTO_LINE_MAX 32
#define SEQ_MAX_LEN    10

static char line[PROTO_LINE_MAX];
static size_t line_len;
static bool line_overflow;

static void reply(const char *s)
{
	int ret = ipc_send(s, strlen(s));

	if (ret < 0) {
		printk("protocol: send failed: %d\n", ret);
	}
}

static bool is_seq(const char *s)
{
	size_t n = 0;

	while (s[n] >= '0' && s[n] <= '9') {
		n++;
	}
	return n > 0 && n <= SEQ_MAX_LEN && s[n] == '\0';
}

static void handle_line(const char *l)
{
	static const char ping[] = "PING ";

	if (strncmp(l, ping, sizeof(ping) - 1) == 0 && is_seq(l + sizeof(ping) - 1)) {
		char out[PROTO_LINE_MAX + 2];

		heartbeat_kick();
		snprintk(out, sizeof(out), "PONG %s\n", l + sizeof(ping) - 1);
		reply(out);
		return;
	}

	printk("protocol: unknown line \"%s\"\n", l);
	reply("ERR unknown\n");
}

void protocol_rx(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		char c = (char)data[i];

		if (c == '\n') {
			if (line_overflow) {
				reply("ERR too long\n");
			} else {
				if (line_len && line[line_len - 1] == '\r') {
					line_len--;
				}
				line[line_len] = '\0';
				if (line_len) {
					handle_line(line);
				}
			}
			line_len = 0;
			line_overflow = false;
		} else if (line_len < PROTO_LINE_MAX - 1) {
			line[line_len++] = c;
		} else {
			/* drop the rest of the line, answer once at its '\n' */
			line_overflow = true;
		}
	}
}
