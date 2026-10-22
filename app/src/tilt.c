/*
 * Complementary filter. See tilt.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tilt.h"

#include <math.h>

#define PI_F 3.14159265f

/* Sensor to body axes: body[i] = sum of mount[i][j] * sensor[j]. The only
 * place that knows how the IMU is mounted. Identity until there is a
 * robot (VISION.md, "Bench first"); an axis-aligned mounting is a signed
 * permutation, e.g. sensor y forward, x right: {{0,1,0},{-1,0,0},{0,0,1}}.
 */
static const float mount[3][3] = {
	{1.0f, 0.0f, 0.0f},
	{0.0f, 1.0f, 0.0f},
	{0.0f, 0.0f, 1.0f},
};

static void to_body(const float s[3], float b[3])
{
	for (int i = 0; i < 3; i++) {
		b[i] = mount[i][0] * s[0] + mount[i][1] * s[1] + mount[i][2] * s[2];
	}
}

/* to (-pi, pi], so roll near +-180 deg doesn't jump by a full turn */
static float wrap(float a)
{
	while (a > PI_F) {
		a -= 2.0f * PI_F;
	}
	while (a <= -PI_F) {
		a += 2.0f * PI_F;
	}
	return a;
}

void tilt_init(struct tilt *t, float tau)
{
	*t = (struct tilt){.tau = tau};
}

void tilt_update(struct tilt *t, const float accel[3], const float gyro[3], float dt)
{
	float a[3], w[3];

	to_body(accel, a);
	to_body(gyro, w);

	/* At rest the accelerometer reads +g along world up, which in body
	 * axes is g * (-sin pitch, sin roll cos pitch, cos roll cos pitch).
	 */
	float roll_acc = atan2f(a[1], a[2]);
	float pitch_acc = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));

	if (!t->started) {
		t->roll = roll_acc;
		t->pitch = pitch_acc;
		t->started = true;
		return;
	}

	/* Body rates to Euler angle rates. Roll couples in the yaw rate when
	 * pitched (e.g. turning while leaning); tan(pitch) is singular only
	 * at pitch +-90 deg, where the robot has fallen anyway.
	 */
	float sr = sinf(t->roll), cr = cosf(t->roll);
	float roll_rate = w[0] + (w[1] * sr + w[2] * cr) * tanf(t->pitch);
	float pitch_rate = w[1] * cr - w[2] * sr;

	/* first-order low pass toward the accelerometer, corner 1/tau */
	float k = dt / (t->tau + dt);

	t->roll = wrap(t->roll + roll_rate * dt);
	t->pitch = wrap(t->pitch + pitch_rate * dt);
	t->roll = wrap(t->roll + k * wrap(roll_acc - t->roll));
	t->pitch = wrap(t->pitch + k * wrap(pitch_acc - t->pitch));
}

void tilt_quaternion(float roll, float pitch, float q[4])
{
	/* ZYX Euler to quaternion with yaw = 0 */
	float sr = sinf(roll / 2), cr = cosf(roll / 2);
	float sp = sinf(pitch / 2), cp = cosf(pitch / 2);

	q[0] = sr * cp;
	q[1] = cr * sp;
	q[2] = -sr * sp;
	q[3] = cr * cp;
}

float tilt_from_upright(float roll, float pitch)
{
	/* "up" in body axes is (-sin p, sin r cos p, cos r cos p); atan2
	 * stays accurate near 0 and past 90 deg, unlike acos of the z part
	 */
	float sp = sinf(pitch), cp = cosf(pitch);
	float sr = sinf(roll), cr = cosf(roll);

	return atan2f(sqrtf(sp * sp + sr * sr * cp * cp), cr * cp);
}
