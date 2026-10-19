/*
 * Tests for the tilt filter (app/src/tilt.c). The filter is control
 * logic: a wrong angle tips the robot over.
 *
 * Each test simulates a motion: the true roll/pitch, the accelerometer
 * reading gravity at that attitude and the gyro reading the body rates.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include "tilt.h"

#define G    9.80665f
#define TAU  0.5f
#define DT   (1.0f / 850.0f) /* measured ODR, imu-plan.md step 3 */
#define DEG  (3.14159265f / 180.0f)

/* What the accelerometer reads at rest at this attitude (see tilt.c). */
static void gravity(float roll, float pitch, float a[3])
{
	a[0] = -G * sinf(pitch);
	a[1] = G * sinf(roll) * cosf(pitch);
	a[2] = G * cosf(roll) * cosf(pitch);
}

static const float no_rate[3] = {0};

/* Feed the same sample for this many seconds. */
static void hold(struct tilt *t, const float a[3], const float w[3], float s)
{
	for (int n = (int)(s / DT + 0.5f); n > 0; n--) {
		tilt_update(t, a, w, DT);
	}
}

static struct tilt start(float roll, float pitch)
{
	struct tilt t;
	float a[3];

	tilt_init(&t, TAU);
	gravity(roll, pitch, a);
	tilt_update(&t, a, no_rate, DT);
	return t;
}

ZTEST(tilt, test_first_sample_takes_accel_angles)
{
	struct tilt t = start(-10 * DEG, 20 * DEG);

	zassert_within(t.roll, -10 * DEG, 0.01f * DEG);
	zassert_within(t.pitch, 20 * DEG, 0.01f * DEG);
}

/* The accelerometer alone moves the angle like a first-order low pass:
 * 63% of a step after tau, within 1% after 5 tau.
 */
ZTEST(tilt, test_accel_step_settles_with_tau)
{
	struct tilt t = start(0, 0);
	float a[3];

	gravity(0, 10 * DEG, a);
	hold(&t, a, no_rate, TAU);
	zassert_within(t.pitch, 10 * DEG * (1 - expf(-1)), 0.05f * DEG);
	hold(&t, a, no_rate, 4 * TAU);
	zassert_within(t.pitch, 10 * DEG, 0.1f * DEG);
	zassert_within(t.roll, 0, 1e-6f);
}

/* A leftover gyro bias b must not drift: it settles at b * tau. 0.01
 * rad/s is ten times the bias left after calibration (step 5).
 */
ZTEST(tilt, test_gyro_bias_does_not_drift)
{
	const float b = 0.01f;
	const float w[3] = {b, b, 0};
	struct tilt t = start(0, 0);
	float a[3], half;

	gravity(0, 0, a);
	hold(&t, a, w, 30);
	half = t.pitch;
	hold(&t, a, w, 30);
	zassert_within(t.pitch, b * TAU, 0.1f * b * TAU);
	zassert_within(t.roll, b * TAU, 0.1f * b * TAU);
	zassert_within(t.pitch, half, 1e-5f);
}

/* Tipping over at 1 rad/s: the gyro keeps the angle on the true one.
 * Accelerometer alone would lag by rate * tau = 29 deg.
 */
ZTEST(tilt, test_follows_fast_rotation)
{
	const float rate = 1.0f;
	const float w[3] = {0, rate, 0};
	struct tilt t = start(0, 0);
	float a[3], pitch = 0;

	while (pitch < 30 * DEG) {
		pitch += rate * DT;
		gravity(0, pitch, a);
		tilt_update(&t, a, w, DT);
		zassert_within(t.pitch, pitch, 0.1f * DEG, "at %d deg", (int)(pitch / DEG));
	}
}

/* Turning in place while leaning forward: the yaw rate shows up on the
 * gyro's x and z axes, but roll must stay at 0 (Euler rates, not raw
 * body rates).
 */
ZTEST(tilt, test_turning_while_leaning_keeps_roll)
{
	const float pitch = 10 * DEG, yaw_rate = 2.0f;
	const float w[3] = {-yaw_rate * sinf(pitch), 0, yaw_rate * cosf(pitch)};
	struct tilt t = start(0, pitch);
	float a[3];

	gravity(0, pitch, a);
	hold(&t, a, w, 2);
	zassert_within(t.roll, 0, 0.1f * DEG);
	zassert_within(t.pitch, pitch, 0.1f * DEG);
}

/* Upside down (fallen robot), roll crosses +-180 deg: the estimate goes
 * the short way, not back through 0.
 */
ZTEST(tilt, test_roll_wraps_at_180)
{
	struct tilt t = start(179 * DEG, 0);
	float a[3];

	gravity(-179 * DEG, 0, a);
	for (int i = 0; i < 5; i++) {
		hold(&t, a, no_rate, TAU);
		zassert_true(fabsf(t.roll) > 178 * DEG, "roll %d deg", (int)(t.roll / DEG));
	}
	zassert_within(t.roll, -179 * DEG, 0.1f * DEG);
}

ZTEST_SUITE(tilt, NULL, NULL, NULL, NULL, NULL);
