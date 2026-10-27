/*
 * Closed-loop tests of the balance controller (app/src/balance.c) on the
 * simulated robot (docs/balance-plan.md step 3). Control logic: a wrong
 * sign spins the wheel up at full torque.
 *
 * The plant is the full nonlinear model with the guessed parameters; the
 * controller sees the true state, no noise or delay (that is step 4).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "balance.h"
#include "plant.h"
#include "sim.h"

/* Time since the run started, so a run printed in parts is one plot. */
static float clock;

static void start(const char *run)
{
	csv_header(run);
	clock = 0;
}

/* Run the closed loop for secs, the controller seeing the tilt off by
 * offset (a crooked IMU), printing it as run (NULL: not). Returns the
 * largest |theta| on the way.
 */
static float fly(const struct balance *b, struct plant_state *s, float secs, float offset,
		 const char *run)
{
	float worst = 0;

	for (int n = 0; n < (int)(secs / DT + 0.5f); n++) {
		float torque = balance_update(b, s->theta + offset, s->theta_dot, s->omega, true);

		zassert_true(fabsf(torque) <= b->t_max, "torque %f", (double)torque);
		csv_row(run, clock, s, torque);
		period(&plant_guess, s, torque);
		clock += DT;
		worst = fmaxf(worst, fabsf(s->theta));
	}
	return worst;
}

static struct balance controller(void)
{
	struct balance b;

	balance_init(&b, plant_guess.t_max);
	return b;
}

/* From 5 deg, held still by hand and let go. It swings through upright
 * to about -1 deg while it spins the wheel back down (the linear design,
 * tests/balance/lqr.py --plot), then settles.
 */
ZTEST(balance_control, test_recovers_from_5_deg)
{
	struct balance b = controller();
	struct plant_state s = {.theta = 5 * DEG};

	start("recover");
	fly(&b, &s, 0.25f, 0, "recover");
	zassert_true(fly(&b, &s, 0.75f, 0, "recover") < 1.5f * DEG, "overshoot");
	zassert_true(fly(&b, &s, 2, 0, "recover") < 0.5f * DEG, "not settled after 1 s");
}

/* A push on the body: it starts turning at 0.5 rad/s while the wheel
 * keeps its speed, so relative to the body the wheel turns back.
 */
ZTEST(balance_control, test_catches_a_push)
{
	struct balance b = controller();
	struct plant_state s = {.theta_dot = 0.5f, .omega = -0.5f};

	start("push");
	zassert_true(fly(&b, &s, 1, 0, "push") < 5 * DEG);
	zassert_true(fly(&b, &s, 2, 0, "push") < 0.5f * DEG);
}

/* Catching a tilt leaves the wheel spinning; the omega term brings it
 * back down instead of leaving it at the next catch's expense.
 */
ZTEST(balance_control, test_wheel_spins_down)
{
	struct balance b = controller();
	struct plant_state s = {.theta = 5 * DEG};
	float peak = 0;

	for (int n = 0; n < (int)(0.5f / DT); n++) {
		fly(&b, &s, DT, 0, NULL);
		peak = fmaxf(peak, fabsf(s.omega));
	}
	zassert_true(peak > 10, "the catch didn't use the wheel: %f", (double)peak);
	fly(&b, &s, 3, 0, NULL);
	zassert_within(s.omega, 0, 1, "wheel still at %f rad/s", (double)s.omega);
}

/* An IMU 0.5 deg off upright. The robot still stands, leaning so that
 * gravity holds the omega term's torque: the wheel settles at a steady
 * k0 offset / k2 = 11 rad/s. Without the omega term the controller holds
 * the wrong angle against gravity and the wheel speeds up until the
 * motor runs out of speed: the reason for the omega term.
 */
ZTEST(balance_control, test_omega_term_holds_an_imu_offset)
{
	const float offset = 0.5f * DEG;
	struct balance b = controller();
	struct plant_state s = {0};
	float settled;

	start("offset");
	fly(&b, &s, 8, offset, "offset");
	settled = s.omega;
	zassert_true(fly(&b, &s, 2, offset, "offset") < 0.1f * DEG);
	zassert_within(s.omega, settled, 0.1f);
	zassert_within(fabsf(s.omega), fabsf(b.k[0] * offset / b.k[2]), 0.5f);

	b.k[2] = 0;
	s = (struct plant_state){0};
	start("offset_no_omega");
	fly(&b, &s, 4, offset, "offset_no_omega");
	zassert_true(fabsf(s.omega) > 30, "wheel at %f rad/s", (double)s.omega);
}

ZTEST(balance_control, test_disarmed_gives_no_torque)
{
	struct balance b = controller();

	zassert_equal(balance_update(&b, 10 * DEG, 1, 20, false), 0.0f);
	zassert_equal(balance_update(&b, -3 * DEG, 0, 0, false), 0.0f);
	zassert_not_equal(balance_update(&b, -3 * DEG, 0, 0, true), 0.0f);
}

ZTEST(balance_control, test_output_within_limit)
{
	struct balance b = controller();

	zassert_equal(balance_update(&b, 90 * DEG, 0, 0, true), b.t_max);
	zassert_equal(balance_update(&b, -90 * DEG, 0, 0, true), -b.t_max);
	zassert_equal(balance_update(&b, 0, -100, 0, true), -b.t_max);
	zassert_equal(balance_update(&b, 0, 0, 1e6f, true), b.t_max);
	zassert_equal(balance_update(&b, NAN, 0, 0, true), 0.0f);
	zassert_equal(balance_update(&b, 0, INFINITY, 0, true), 0.0f);
	zassert_equal(balance_update(&b, 0, 0, -INFINITY, true), 0.0f);
}

ZTEST_SUITE(balance_control, NULL, NULL, NULL, NULL, NULL);
