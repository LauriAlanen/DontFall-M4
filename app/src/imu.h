/*
 * IMU: ST LSM6DSOX on SPI5 (DT node "imu"), through Zephyr's LSM6DSO
 * sensor driver.
 *
 * Bring-up step 3 (DontFall docs/imu-plan.md): the chip's data-ready
 * interrupt (INT1) clocks the sampling at the ODR. Each sample is fetched
 * and timestamped; once a second a summary of the rate and the gaps
 * between samples is printked. No consumers yet.
 */
#ifndef DONTFALL_M4_IMU_H_
#define DONTFALL_M4_IMU_H_

/* Check the sensor came up, start sampling and the summary thread.
 * Returns 0, -ENODEV if the driver failed to init (wrong WHO_AM_I, SPI
 * error), or the error from setting the data-ready trigger.
 */
int imu_init(void);

#endif /* DONTFALL_M4_IMU_H_ */
