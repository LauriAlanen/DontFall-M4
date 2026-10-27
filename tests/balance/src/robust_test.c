/*
 * Robustness of the balance controller (docs/balance-plan.md step 4): the
 * real robot will differ from the model the gains were designed for.
 *
 * Unlike control_test.c the controller doesn't see the true state: an
 * IMU on the body reads gravity, the body's own acceleration, noise and
 * a leftover gyro bias, and the firmware's tilt filter (app/src/tilt.c)
 * turns that into theta. The torque reaches the robot after a delay and
 * through a first-order lag standing in for the FOC current loop.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdint.h>
#include <zephyr/ztest.h>

#include "balance.h"
#include "plant.h"
#include "sim.h"
#include "tilt.h"

#define TILT_TAU  0.5f /* the firmware's filter time constant */
#define MAX_DELAY 64

struct world {
	struct plant_params p; /* the real robot */
	float imu_height;      /* m above the pivot */
	float gyro_bias;       /* rad/s left after calibration */
	float gyro_noise;      /* rad/s std */
	float accel_noise;     /* m/s^2 std */
	int delay;             /* control periods from sensing to torque */
	float motor_lag;       /* s, first order; 0 = none */
};

/* The IMU as measured (docs/archive/imu-plan.md step 5): gyro std 1.5
 * mrad/s, accel std 0.025 m/s^2, bias left after calibration 0.2 mrad/s
 * (5x here). One sample of delay, 2 ms of motor lag.
 */
static struct world measured(void)
{
	return (struct world){
		.p = plant_guess,
		.imu_height = plant_guess.l,
		.gyro_bias = 1e-3f,
		.gyro_noise = 1.5e-3f,
		.accel_noise = 0.025f,
		.delay = 1,
		.motor_lag = 2e-3f,
	};
}

struct run {
	struct world w;
	struct plant_state s;
	struct balance b;
	struct tilt t;
	float queue[MAX_DELAY + 1]; /* commands on their way, oldest first */
	float torque;               /* what the motor gives now */
	uint32_t rng;
	float clock;
	const char *name;
};

/* Normal noise, repeatable: xorshift32 and Box-Muller. */
static float gauss(struct run *r)
{
	float u[2];

	for (int i = 0; i < 2; i++) {
		r->rng ^= r->rng << 13;
		r->rng ^= r->rng >> 17;
		r->rng ^= r->rng << 5;
		u[i] = ((r->rng >> 8) + 0.5f) / (float)(1 << 24);
	}
	return sqrtf(-2 * logf(u[0])) * cosf(2 * 3.14159265f * u[1]);
}

/* What the IMU reads in body axes (x forward, z up, theta is pitch): the
 * specific force at imu_height, gravity plus the tangential and
 * centripetal acceleration of the body turning about the pivot. held: a
 * hand keeps the body still.
 *
 * Free, the body's own acceleration hides most of the tilt from the
 * accelerometer: falling from rest, with the IMU at the centre of mass,
 * it sees only (1 - m l^2 / (I_b + m l^2)) = 19% of it.
 */
static void imu(struct run *r, bool held, float accel[3], float gyro[3])
{
	const struct world *w = &r->w;
	float h = w->imu_height;
	float theta_ddot = held ? 0 : plant_theta_ddot(&w->p, &r->s, r->torque);

	accel[0] = -w->p.g * sinf(r->s.theta) + h * theta_ddot + w->accel_noise * gauss(r);
	accel[1] = w->accel_noise * gauss(r);
	accel[2] = w->p.g * cosf(r->s.theta) - h * r->s.theta_dot * r->s.theta_dot +
		   w->accel_noise * gauss(r);
	gyro[0] = w->gyro_noise * gauss(r);
	gyro[1] = r->s.theta_dot + w->gyro_bias + w->gyro_noise * gauss(r);
	gyro[2] = w->gyro_noise * gauss(r);
}

/* Held still at theta while the tilt filter settles (4 tau), the way it
 * is armed by hand, then let go.
 */
static void start(struct run *r, const struct world *w, float theta, const char *name)
{
	float a[3], g[3];

	*r = (struct run){.w = *w, .s = {.theta = theta}, .rng = 12345, .name = name};
	balance_init(&r->b, w->p.t_max);
	tilt_init(&r->t, TILT_TAU);
	for (int n = 0; n < (int)(4 * TILT_TAU / DT); n++) {
		imu(r, true, a, g);
		tilt_update(&r->t, a, g, DT);
	}
	csv_header(name);
}

/* Run for secs, or until past 25 deg where the supervisor would disarm
 * (SUPERVISOR_DISARM_TILT); returns the largest |theta| on the way.
 */
static float fly(struct run *r, float secs)
{
	const struct world *w = &r->w;
	float worst = 0;

	for (int n = 0; n < (int)(secs / DT + 0.5f); n++) {
		float a[3], g[3], cmd;

		imu(r, false, a, g);
		tilt_update(&r->t, a, g, DT);
		cmd = balance_update(&r->b, r->t.pitch, g[1], r->s.omega, true);

		for (int i = 0; i < w->delay; i++) {
			r->queue[i] = r->queue[i + 1];
		}
		r->queue[w->delay] = cmd;
		cmd = r->queue[0];
		r->torque += w->motor_lag > 0 ? (cmd - r->torque) * (1 - expf(-DT / w->motor_lag))
					      : cmd - r->torque;

		csv_row(r->name, r->clock, &r->s, r->torque);
		period(&w->p, &r->s, r->torque);
		r->clock += DT;
		worst = fmaxf(worst, fabsf(r->s.theta));
		if (worst > 25 * DEG) {
			break;
		}
	}
	return worst;
}

/* Recover from 5 deg, then stand: the largest |theta| over the last
 * stand_secs of settle_secs + stand_secs.
 */
static float recover_and_stand(const struct world *w, float settle_secs, float stand_secs,
			       const char *name)
{
	struct run r;

	start(&r, w, 5 * DEG, name);
	fly(&r, settle_secs);
	return fly(&r, stand_secs);
}

/* Everything as measured on the bench, IMU at the centre of mass. */
ZTEST(balance_robust, test_stands_with_the_measured_imu)
{
	struct world w = measured();
	struct run r;

	start(&r, &w, 5 * DEG, "sensors");
	fly(&r, 1);
	zassert_true(fly(&r, 10) < 0.5f * DEG);
	zassert_within(r.s.omega, 0, 3, "wheel at %f rad/s", (double)r.s.omega);
}

/* At the pivot the IMU only sees gravity; higher up it also feels the
 * body's own acceleration (15 m/s^2 at the top in a 5 deg catch), which
 * the tilt filter's 0.5 s time constant smooths out.
 */
ZTEST(balance_robust, test_imu_anywhere_on_the_body)
{
	const float heights[] = {0, plant_guess.l, 2 * plant_guess.l};
	struct world w = measured();

	ARRAY_FOR_EACH(heights, i) {
		w.imu_height = heights[i];
		zassert_true(recover_and_stand(&w, 1, 5, NULL) < 0.5f * DEG, "IMU at %d mm",
			     (int)(heights[i] * 1000));
	}
}

/* The robot with mass, centre of mass height and wheel inertia each
 * scaled by (m, l, iw) from what the gains were designed for.
 */
static struct world off_by(float m, float l, float iw)
{
	struct world w = measured();

	w.p.m *= m;
	w.p.i_b *= m;
	w.p.l *= l;
	w.p.i_w *= iw;
	w.imu_height = w.p.l;
	return w;
}

/* Every combination 25% off. The worst corner is heavier and taller with
 * a lighter wheel (more gravity torque, less wheel to fight it): its
 * poles go from -9.0 +- 2.9j to -1.1 +- 5.9j /s, a 1 Hz wobble.
 */
ZTEST(balance_robust, test_parameters_25_percent_off)
{
	for (int c = 0; c < 8; c++) {
		float m = c & 1 ? 1.25f : 0.75f, l = c & 2 ? 1.25f : 0.75f;
		float iw = c & 4 ? 1.25f : 0.75f;
		struct world w = off_by(m, l, iw);

		zassert_true(recover_and_stand(&w, 8, 3, NULL) < 0.5f * DEG,
			     "mass x%.2f, height x%.2f, wheel x%.2f", (double)m, (double)l, (double)iw);
	}
}

/* The limit: 30% off in the worst direction it falls. With the true
 * state it still (barely) stands, poles -0.27 +- 6.1j; what the
 * accelerometer feels of the wobble takes the rest. If this starts to
 * pass, the margin got better: update the plan's numbers.
 */
ZTEST(balance_robust, test_falls_30_percent_off_the_worst_way)
{
	struct world w = off_by(1.3f, 1.3f, 0.7f);
	struct run r;

	start(&r, &w, 5 * DEG, "off30");
	zassert_true(fly(&r, 15) > 25 * DEG, "still standing");
}

/* 20 samples (24 ms) of delay and a 20 ms motor lag together, ten times
 * what the loop and a FOC current loop should need. It falls past about
 * 50 ms of delay or 90 ms of lag alone (tests/balance/sandbox.py).
 */
ZTEST(balance_robust, test_delay_and_lag_margin)
{
	struct world w = measured();

	w.delay = 20;
	w.motor_lag = 20e-3f;
	zassert_true(recover_and_stand(&w, 4, 3, "slow") < 0.5f * DEG);
}

ZTEST_SUITE(balance_robust, NULL, NULL, NULL, NULL, NULL);
