/*
 * IMU: ST LSM6DSOX on SPI5 (DT node "imu").
 *
 * Bring-up step 1 (DontFall docs/imu-plan.md): only reads WHO_AM_I over
 * raw SPI, to prove wiring, power and pin muxing before any driver is
 * involved.
 */
#ifndef DONTFALL_M4_IMU_H_
#define DONTFALL_M4_IMU_H_

/* Read WHO_AM_I and printk it. Returns 0 if it is the LSM6DSOX's ID,
 * -EIO for a wrong ID, or another negative errno.
 */
int imu_init(void);

#endif /* DONTFALL_M4_IMU_H_ */
