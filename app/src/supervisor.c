/*
 * Arm/disarm and command timeout. See supervisor.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "supervisor.h"

#include <math.h>

void supervisor_init(struct supervisor *s)
{
	*s = (struct supervisor){
		.state = SUPERVISOR_DISARMED,
		.reason = SUPERVISOR_BOOT,
	};
}

/* Why the inputs don't allow running, SUPERVISOR_OK if they do. The tilt
 * checks are written so a NaN fails them: every comparison with NaN is
 * false.
 */
static enum supervisor_reason check(const struct supervisor_input *in, float tilt_limit)
{
	if (!in->imu_ok) {
		return SUPERVISOR_IMU;
	}
	if (!(fabsf(in->tilt) <= tilt_limit)) {
		return SUPERVISOR_TILT;
	}
	return SUPERVISOR_OK;
}

static bool cmd_fresh(const struct supervisor *s, uint32_t now_us)
{
	/* unsigned difference: right across a wrap of the clock */
	return s->have_cmd && now_us - s->cmd_us <= SUPERVISOR_CMD_TIMEOUT_US;
}

void supervisor_update(struct supervisor *s, const struct supervisor_input *in)
{
	enum supervisor_reason r;

	/* Forget a stale command for good: after a long silence the clock
	 * wraps and the old arrival time would look fresh again.
	 */
	if (!cmd_fresh(s, in->now_us)) {
		s->have_cmd = false;
	}

	if (s->state != SUPERVISOR_ARMED) {
		return;
	}
	r = check(in, SUPERVISOR_DISARM_TILT);
	if (r == SUPERVISOR_OK && in->saturated) {
		if (!s->saturated) {
			s->saturated = true;
			s->saturated_us = in->now_us;
		} else if (in->now_us - s->saturated_us > SUPERVISOR_SATURATED_US) {
			r = SUPERVISOR_SATURATED;
		}
	} else {
		s->saturated = false;
	}
	if (r != SUPERVISOR_OK) {
		s->state = SUPERVISOR_DISARMED;
		s->reason = r;
	}
}

enum supervisor_reason supervisor_arm(struct supervisor *s, const struct supervisor_input *in)
{
	enum supervisor_reason r;

	if (s->state == SUPERVISOR_ARMED) {
		return SUPERVISOR_OK;
	}
	r = check(in, SUPERVISOR_ARM_TILT);
	if (r != SUPERVISOR_OK) {
		return r;
	}
	if (!(fabsf(in->rate) <= SUPERVISOR_ARM_RATE)) {
		return SUPERVISOR_MOVING;
	}
	s->state = SUPERVISOR_ARMED;
	s->reason = SUPERVISOR_OK;
	s->saturated = false;
	/* start from standing still, not from a command sent before */
	s->have_cmd = false;
	return SUPERVISOR_OK;
}

void supervisor_disarm(struct supervisor *s)
{
	if (s->state == SUPERVISOR_ARMED) {
		s->state = SUPERVISOR_DISARMED;
		s->reason = SUPERVISOR_REQUEST;
	}
}

void supervisor_command(struct supervisor *s, uint32_t now_us,
			const struct supervisor_setpoint *cmd)
{
	if (!isfinite(cmd->speed) || !isfinite(cmd->turn)) {
		return;
	}
	s->cmd = *cmd;
	s->cmd_us = now_us;
	s->have_cmd = true;
}

struct supervisor_setpoint supervisor_setpoint(const struct supervisor *s, uint32_t now_us)
{
	if (s->state == SUPERVISOR_ARMED && cmd_fresh(s, now_us)) {
		return s->cmd;
	}
	return (struct supervisor_setpoint){0};
}

const char *supervisor_reason_str(enum supervisor_reason r)
{
	switch (r) {
	case SUPERVISOR_OK:
		return "ok";
	case SUPERVISOR_BOOT:
		return "boot";
	case SUPERVISOR_REQUEST:
		return "request";
	case SUPERVISOR_IMU:
		return "imu";
	case SUPERVISOR_TILT:
		return "tilt";
	case SUPERVISOR_MOVING:
		return "moving";
	case SUPERVISOR_SATURATED:
		return "saturated";
	}
	return "?";
}
