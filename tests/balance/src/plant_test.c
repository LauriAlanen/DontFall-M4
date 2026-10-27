/*
 * Tests of the simulated balancer itself (docs/balance-plan.md step 1),
 * so the controller tests stand on a model that is known right.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "plant.h"
#include "sim.h"

/* Close to upright the fall is linear: theta0 cosh(t / T) from rest.
 * 1 mrad stays below 1 deg for 3 T, where sin(theta) = theta to 2e-5.
 */
ZTEST(balance_plant, test_fall_from_rest_follows_cosh)
{
	const struct plant_params *p = &plant_guess;
	const float theta0 = 1e-3f, ft = plant_fall_time(p);
	struct plant_state s = {.theta = theta0};
	int n = 0;

	for (int k = 1; k <= 3; k++) {
		int end = (int)(k * ft / DT + 0.5f);

		for (; n < end; n++) {
			period(p, &s, 0);
		}
		float want = theta0 * coshf(n * DT / ft);

		zassert_within(s.theta, want, 0.002f * want, "at %d T: %f, want %f", k,
			       (double)s.theta, (double)want);
	}
}

/* With gravity off nothing outside acts on the robot, so the angular
 * momentum about the pivot stays put while the motor pushes wheel and
 * body against each other.
 */
ZTEST(balance_plant, test_momentum_conserved_without_gravity)
{
	struct plant_params p = plant_guess;
	struct plant_state s = {.theta = 0.1f, .theta_dot = 0.5f, .omega = 10};
	float l0, torque, swing = 0;

	p.g = 0;
	l0 = plant_momentum(&p, &s);
	for (int n = 0; n < (int)(2.0f / DT); n++) {
		torque = p.t_max * sinf(2 * 3.14159265f * 3 * n * DT);
		period(&p, &s, torque);
		zassert_within(plant_momentum(&p, &s), l0, 1e-3f * l0, "at %d", n);
		swing = fmaxf(swing, fabsf(s.omega - 10));
	}
	/* the torque did move the wheel, or this checks nothing */
	zassert_true(swing > 5, "wheel swing %f rad/s", (double)swing);
}

/* Not a check: a fall from 2 deg until it lies down, to plot. */
ZTEST(balance_plant, test_fall_to_plot)
{
	const struct plant_params *p = &plant_guess;
	struct plant_state s = {.theta = 2 * DEG};
	int n = 0;

	csv_header("fall");
	while (s.theta < 90 * DEG && n * DT < 5) {
		csv_row("fall", n * DT, &s, 0);
		period(p, &s, 0);
		n++;
	}
	zassert_true(s.theta >= 90 * DEG, "never fell");
}

ZTEST_SUITE(balance_plant, NULL, NULL, NULL, NULL, NULL);
