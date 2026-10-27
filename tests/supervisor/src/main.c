/*
 * Tests for the supervisor (app/src/supervisor.c): it decides whether the
 * motors may run, so a missed disarm drives a falling robot and a wrong
 * arm starts the motors when they shouldn't.
 *
 * Each test starts from a healthy IMU held upright, one call every
 * ~1.2 ms (850 Hz), like the sampling path.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "supervisor.h"

#define PERIOD_US 1176
#define DEG       SUPERVISOR_DEG

static struct supervisor s;
static struct supervisor_input in;

static const struct supervisor_setpoint fwd = {.speed = 0.3f, .turn = -0.5f};

/* count samples with the current inputs, next one PERIOD_US later */
static void run(int count)
{
	while (count--) {
		in.now_us += PERIOD_US;
		supervisor_update(&s, &in);
	}
}

static enum supervisor_reason arm(void)
{
	return supervisor_arm(&s, &in);
}

static void command(const struct supervisor_setpoint *c)
{
	supervisor_command(&s, in.now_us, c);
}

static void assert_setpoint(struct supervisor_setpoint want)
{
	struct supervisor_setpoint got = supervisor_setpoint(&s, in.now_us);

	zassert_equal(got.speed, want.speed, "speed %f", (double)got.speed);
	zassert_equal(got.turn, want.turn, "turn %f", (double)got.turn);
}

static void before(void *unused)
{
	ARG_UNUSED(unused);
	in = (struct supervisor_input){.now_us = 0, .imu_ok = true, .tilt = 0.0f};
	supervisor_init(&s);
}

ZTEST(supervisor, test_starts_disarmed)
{
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_BOOT);
	command(&fwd);
	assert_setpoint((struct supervisor_setpoint){0});
}

ZTEST(supervisor, test_arm_when_upright_and_healthy)
{
	in.tilt = 4.9f * DEG;
	zassert_equal(arm(), SUPERVISOR_OK);
	zassert_true(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_OK);
}

/* Each arm condition on its own refuses, and a refusal changes nothing. */
ZTEST(supervisor, test_arm_refused)
{
	in.imu_ok = false;
	zassert_equal(arm(), SUPERVISOR_IMU);
	in.imu_ok = true;

	in.tilt = 5.1f * DEG;
	zassert_equal(arm(), SUPERVISOR_TILT);
	in.tilt = -5.1f * DEG;
	zassert_equal(arm(), SUPERVISOR_TILT);
	in.tilt = NAN;
	zassert_equal(arm(), SUPERVISOR_TILT);
	in.tilt = 0;

	in.rate = 0.21f;
	zassert_equal(arm(), SUPERVISOR_MOVING);
	in.rate = -0.21f;
	zassert_equal(arm(), SUPERVISOR_MOVING);
	in.rate = NAN;
	zassert_equal(arm(), SUPERVISOR_MOVING);

	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_BOOT, "refusal overwrote the reason");
}

/* Held still means under the limit, not exactly 0: a hand trembles. */
ZTEST(supervisor, test_arm_while_held_by_hand)
{
	in.rate = -0.19f;
	zassert_equal(arm(), SUPERVISOR_OK);
	/* once armed, moving is the controller's job */
	in.rate = 3.0f;
	run(10);
	zassert_true(supervisor_armed(&s));
}

/* Saturated for longer than the limit disarms; shorter bursts, as in
 * every catch, don't add up.
 */
ZTEST(supervisor, test_disarm_when_saturated_too_long)
{
	const int limit = SUPERVISOR_SATURATED_US / PERIOD_US;

	zassert_equal(arm(), SUPERVISOR_OK);
	for (int i = 0; i < 5; i++) {
		in.saturated = true;
		run(limit - 1);
		in.saturated = false;
		run(1);
	}
	zassert_true(supervisor_armed(&s), "short bursts added up");

	in.saturated = true;
	run(limit);
	zassert_true(supervisor_armed(&s), "disarmed early");
	run(2);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_SATURATED);
}

/* A saturation left over from before arming doesn't count. */
ZTEST(supervisor, test_saturation_restarts_on_arm)
{
	const int limit = SUPERVISOR_SATURATED_US / PERIOD_US;

	zassert_equal(arm(), SUPERVISOR_OK);
	in.saturated = true;
	run(limit - 10);
	supervisor_disarm(&s);
	run(100);
	zassert_equal(arm(), SUPERVISOR_OK);
	run(20);
	zassert_true(supervisor_armed(&s));
}

ZTEST(supervisor, test_arm_while_armed_is_ok)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	/* past the arm window but inside the disarm limit: stays armed */
	in.tilt = 10.0f * DEG;
	run(10);
	zassert_equal(arm(), SUPERVISOR_OK);
	zassert_true(supervisor_armed(&s));
}

ZTEST(supervisor, test_disarm_on_imu_fault)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	in.imu_ok = false;
	run(1);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_IMU);
}

ZTEST(supervisor, test_disarm_on_tilt)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	in.tilt = 14.9f * DEG;
	run(100);
	zassert_true(supervisor_armed(&s), "disarmed inside the limit");

	in.tilt = -15.1f * DEG;
	run(1);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_TILT);
}

ZTEST(supervisor, test_disarm_on_nan_tilt)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	in.tilt = NAN;
	run(1);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_TILT);
}

/* The cause going away doesn't re-arm: it needs a new request. */
ZTEST(supervisor, test_disarm_latches)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	in.imu_ok = false;
	run(1);
	in.imu_ok = true;
	run(1000);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_IMU, "reason lost");

	zassert_equal(arm(), SUPERVISOR_OK);
	zassert_true(supervisor_armed(&s));
}

ZTEST(supervisor, test_disarm_request)
{
	/* while disarmed: no effect, reason kept */
	supervisor_disarm(&s);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_BOOT);

	zassert_equal(arm(), SUPERVISOR_OK);
	supervisor_disarm(&s);
	zassert_false(supervisor_armed(&s));
	zassert_equal(supervisor_reason(&s), SUPERVISOR_REQUEST);

	/* also beyond the tilt limit and with a fault */
	zassert_equal(arm(), SUPERVISOR_OK);
	in.tilt = 20.0f * DEG;
	run(1);
	supervisor_disarm(&s);
	zassert_false(supervisor_armed(&s));
}

ZTEST(supervisor, test_command_followed_while_armed)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	command(&fwd);
	run(10);
	assert_setpoint(fwd);
	supervisor_disarm(&s);
	assert_setpoint((struct supervisor_setpoint){0});
}

/* A stale command gives zero speed, but the robot stays armed and
 * balances in place.
 */
ZTEST(supervisor, test_command_timeout)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	command(&fwd);
	in.now_us += SUPERVISOR_CMD_TIMEOUT_US;
	supervisor_update(&s, &in);
	assert_setpoint(fwd);

	in.now_us += 1;
	supervisor_update(&s, &in);
	assert_setpoint((struct supervisor_setpoint){0});
	zassert_true(supervisor_armed(&s), "lost commands disarmed");

	/* a new command is followed again */
	command(&fwd);
	assert_setpoint(fwd);
}

/* Arming starts from standing still, not from a command sent earlier. */
ZTEST(supervisor, test_arm_clears_old_command)
{
	command(&fwd);
	run(10);
	zassert_equal(arm(), SUPERVISOR_OK);
	assert_setpoint((struct supervisor_setpoint){0});
}

ZTEST(supervisor, test_non_finite_command_ignored)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	command(&fwd);
	command(&(struct supervisor_setpoint){.speed = NAN, .turn = 0.0f});
	command(&(struct supervisor_setpoint){.speed = 0.0f, .turn = INFINITY});
	assert_setpoint(fwd);
}

ZTEST(supervisor, test_clock_wrap)
{
	in.now_us = UINT32_MAX - 100 * PERIOD_US;
	zassert_equal(arm(), SUPERVISOR_OK);
	command(&fwd);
	run(200);
	zassert_true(in.now_us < 1000000, "clock wrapped");
	assert_setpoint(fwd);
	zassert_true(supervisor_armed(&s));
}

/* After a silence of a whole clock period the old arrival time looks
 * fresh again; the update in between must have forgotten the command.
 */
ZTEST(supervisor, test_stale_command_stays_stale_after_wrap)
{
	zassert_equal(arm(), SUPERVISOR_OK);
	command(&fwd);
	in.now_us += 2 * SUPERVISOR_CMD_TIMEOUT_US;
	supervisor_update(&s, &in);
	in.now_us += UINT32_MAX - 2 * SUPERVISOR_CMD_TIMEOUT_US + 1; /* a full period after the command */
	supervisor_update(&s, &in);
	assert_setpoint((struct supervisor_setpoint){0});
}

ZTEST_SUITE(supervisor, NULL, NULL, before, NULL, NULL);
