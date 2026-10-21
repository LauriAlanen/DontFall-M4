/*
 * IMU health: decides from the samples whether the tilt can be trusted.
 *
 * Plain C with no Zephyr dependencies, so the same file is built into the
 * firmware and into the host tests (tests/imu_health). Not thread safe:
 * the caller serialises the calls (imu.c holds its lock). Built into the
 * firmware from step 7b.
 *
 * Faults (DontFall docs/imu-plan.md step 7):
 *  - no data:   no good sample for IMU_HEALTH_TIMEOUT_US (INT1 lost, chip
 *               powered down, the driver drops samples whose status reads
 *               0 because MISO is stuck low)
 *  - read:      sample fetches failing in a row (SPI errors)
 *  - stuck:     every axis exactly the same many samples in a row (MISO
 *               stuck: all 0x00 or 0xFF). A live sensor's noise changes
 *               the low bits every sample.
 *  - range:     a reading at full scale several samples in a row: the
 *               real value is beyond what the sensor measures
 *  - who_am_i:  the periodic re-check read a wrong chip id
 *  - rate:      samples much closer together than the ODR, several in a
 *               row. A live sensor can't; seen with the 3.3 V wire off
 *               (the chip half powered through its I/O pins): the
 *               driver's status read finds data ready every time and it
 *               reads garbage at 15-40 kHz, garbage that changes enough
 *               to pass as live.
 *
 * A fault latches: it clears only after IMU_HEALTH_RECOVER good samples
 * in a row (and, for who_am_i, a good re-check first), so a flapping wire
 * doesn't toggle the flag every sample. The balance controller must still
 * latch its own disarm: health coming back doesn't mean it is safe to
 * drive again.
 */
#ifndef DONTFALL_M4_IMU_HEALTH_H_
#define DONTFALL_M4_IMU_HEALTH_H_

#include <stdbool.h>
#include <stdint.h>

/* ~12 samples at the ~850 Hz ODR: one lost pulse (drdy-pulsed) is normal */
#define IMU_HEALTH_TIMEOUT_US 10000
#define IMU_HEALTH_READ_ERRORS 3
/* ~24 ms; at rest the gyro's noise (~1.5 mrad/s std) is ~5 LSB */
#define IMU_HEALTH_STUCK      20
#define IMU_HEALTH_SATURATED  3
/* ~100 ms */
#define IMU_HEALTH_RECOVER    85
/* under half the ~1176 us period */
#define IMU_HEALTH_MIN_GAP_US 500
#define IMU_HEALTH_FAST       3

enum imu_fault {
	IMU_OK = 0,
	IMU_FAULT_NO_DATA,
	IMU_FAULT_READ,
	IMU_FAULT_STUCK,
	IMU_FAULT_RANGE,
	IMU_FAULT_WHO_AM_I,
	IMU_FAULT_RATE,
};

struct imu_health {
	/* the largest reading the sensor gives (raw 32767 times the
	 * sensitivity), in the units of the samples (um/s^2, urad/s)
	 */
	int32_t accel_fs;
	int32_t gyro_fs;

	enum imu_fault fault; /* latched, IMU_OK when none */
	bool who_am_i_bad;
	bool have_good;
	bool have_last;
	uint32_t last_good_us;
	uint32_t last_us; /* any fetch, good or not */
	uint32_t good_n; /* good samples in a row since the fault */
	uint32_t read_errors;
	uint32_t same_n;
	uint32_t saturated_n;
	uint32_t fast_n;
	int32_t prev[6];
};

/* Start with no data yet: imu_health_check() says IMU_FAULT_NO_DATA
 * until the first good sample.
 */
void imu_health_init(struct imu_health *h, int32_t accel_fs, int32_t gyro_fs);

/* One fetch at now_us (any free-running microsecond clock, may wrap).
 * read_ok false: the fetch failed, accel/gyro are ignored.
 */
void imu_health_sample(struct imu_health *h, uint32_t now_us, bool read_ok,
		       const int32_t accel[3], const int32_t gyro[3]);

/* Result of a WHO_AM_I re-check. */
void imu_health_who_am_i(struct imu_health *h, bool ok);

/* IMU_OK if the latest sample can be trusted at now_us, else the fault.
 * Ask it right before using a sample: it also catches samples that
 * stopped arriving, which imu_health_sample() never sees.
 */
enum imu_fault imu_health_check(const struct imu_health *h, uint32_t now_us);

/* Short name for logs and diagnostics, e.g. "stuck". */
const char *imu_fault_str(enum imu_fault f);

#endif /* DONTFALL_M4_IMU_HEALTH_H_ */
