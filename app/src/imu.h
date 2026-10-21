/*
 * IMU: ST LSM6DSOX on SPI5 (DT node "imu"), through Zephyr's LSM6DSO
 * sensor driver.
 *
 * Bring-up step 3 (DontFall docs/archive/imu-plan.md): the chip's data-ready
 * interrupt (INT1) clocks the sampling at the ODR. Each sample is fetched
 * and timestamped; once a second a summary of the rate and the gaps
 * between samples is printked. imu_latest() hands the newest sample to
 * the micro-ROS publisher (uros.c).
 *
 * Bring-up step 5: the gyro bias is measured at startup while the module
 * is still and subtracted from every sample. No samples are handed out
 * until that has worked.
 *
 * Step 6: every sample also runs the tilt filter (tilt.c); its roll and
 * pitch come with the sample as an orientation quaternion.
 *
 * Step 7: every sample goes through the fault checks (imu_health.h);
 * imu_latest() hands out nothing while there is a fault.
 */
#ifndef DONTFALL_M4_IMU_H_
#define DONTFALL_M4_IMU_H_

#include "imu_health.h"

struct imu_sample {
	double accel[3]; /* m/s^2, sensor axes */
	double gyro[3];  /* rad/s, sensor axes, bias removed */
	/* tilt filter (tilt.h), body axes: unit quaternion (x, y, z, w) of
	 * roll and pitch, yaw 0
	 */
	double orientation[4];
};

/* Check the sensor came up, start sampling and the summary thread.
 * Returns 0, -ENODEV if the driver failed to init (wrong WHO_AM_I, SPI
 * error), or the error from setting the data-ready trigger.
 */
int imu_init(void);

/* Copy the newest sample to *out. Any thread; never blocks on the sensor.
 * Returns IMU_OK, or the fault and leaves *out alone: then the tilt can't
 * be trusted (the balance controller disarms). IMU_FAULT_NO_DATA also
 * while the gyro is still being calibrated.
 */
enum imu_fault imu_latest(struct imu_sample *out);

#endif /* DONTFALL_M4_IMU_H_ */
