/*
 * Tilt estimate: roll and pitch from the IMU with a complementary filter.
 *
 * Plain C with no Zephyr dependencies, so the same file is built into the
 * firmware and into the host tests (tests/tilt).
 *
 * Body frame (ROS REP 103): x forward, y left, z up. Pitch is the rotation
 * about y (the wheel axle on the robot: positive = nose down), roll about
 * x (positive = left side up). Angles are ZYX Euler angles, yaw is not
 * estimated (nothing measures it).
 *
 * The gyro is accurate over short times but its bias integrates into a
 * drift; the accelerometer gives the angle from gravity with no drift but
 * is noisy and fooled by linear acceleration. The filter integrates the
 * gyro and pulls the angle toward the accelerometer's with time constant
 * tau: slower changes come from the accelerometer, faster ones from the
 * gyro. A constant gyro bias b leaves a constant error of b * tau instead
 * of a growing one.
 */
#ifndef DONTFALL_M4_TILT_H_
#define DONTFALL_M4_TILT_H_

#include <stdbool.h>

struct tilt {
	float tau;        /* s */
	float roll;       /* rad */
	float pitch;      /* rad */
	float pitch_rate; /* rad/s, straight from the gyro, not filtered */
	bool started;
};

/* Start a filter with time constant tau (s). The first tilt_update()
 * takes the accelerometer's angles as they are.
 */
void tilt_init(struct tilt *t, float tau);

/* Feed one sample in sensor axes: accel in m/s^2, gyro in rad/s with the
 * bias removed, dt the time since the previous sample in s (measured, not
 * the nominal 1/ODR).
 */
void tilt_update(struct tilt *t, const float accel[3], const float gyro[3], float dt);

/* Unit quaternion (x, y, z, w) of roll and pitch with yaw 0, as ROS
 * (geometry_msgs/Quaternion) wants it: rotates body vectors into the
 * level frame.
 */
void tilt_quaternion(float roll, float pitch, float q[4]);

/* Angle between the body's z axis and up, 0..pi rad, whichever way it
 * leans: for checks that don't care about the direction.
 */
float tilt_from_upright(float roll, float pitch);

#endif /* DONTFALL_M4_TILT_H_ */
