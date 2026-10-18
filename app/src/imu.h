/*
 * IMU: ST LSM6DSOX on SPI5 (DT node "imu"), through Zephyr's LSM6DSO
 * sensor driver.
 *
 * Bring-up step 2 (DontFall docs/imu-plan.md): a thread polls accel and
 * gyro a few times a second and printks them in SI units, to check the
 * axes and signs by hand. No consumers yet.
 */
#ifndef DONTFALL_M4_IMU_H_
#define DONTFALL_M4_IMU_H_

/* Check the sensor came up and start the polling thread. Returns 0, or
 * -ENODEV if the driver failed to init (wrong WHO_AM_I, SPI error).
 */
int imu_init(void);

#endif /* DONTFALL_M4_IMU_H_ */
