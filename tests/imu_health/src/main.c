/*
 * Tests for the IMU fault detection (app/src/imu_health.c): the balance
 * controller disarms on it, so a missed fault tips the robot over and a
 * false one stops it for nothing.
 *
 * A simulated sensor at rest: gravity on z plus a little noise, so no two
 * samples in a row are the same, one sample every ~1.2 ms (850 Hz).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

#include "imu_health.h"

#define PERIOD_US 1176
/* the real full scales: 32767 * 0.122 mg (4 g), 32767 * 17.5 mdps (500 dps) */
#define ACCEL_FS  39202800 /* um/s^2 */
#define GYRO_FS   10008000 /* urad/s */

static struct imu_health h;
static uint32_t now;
static int32_t n;

static void live(int32_t a[3], int32_t g[3])
{
	n++;
	a[0] = 1000 * (n % 7);
	a[1] = -1000 * (n % 3);
	a[2] = 9806650 + 1000 * (n % 5);
	g[0] = 300 * (n % 4);
	g[1] = -300 * (n % 5);
	g[2] = 300 * (n % 3);
}

/* count samples, next one PERIOD_US later */
static void good(int count)
{
	int32_t a[3], g[3];

	while (count--) {
		now += PERIOD_US;
		live(a, g);
		imu_health_sample(&h, now, true, a, g);
	}
}

static void same(int count, const int32_t a[3], const int32_t g[3])
{
	while (count--) {
		now += PERIOD_US;
		imu_health_sample(&h, now, true, a, g);
	}
}

static void read_errors(int count)
{
	while (count--) {
		now += PERIOD_US;
		imu_health_sample(&h, now, false, NULL, NULL);
	}
}

static enum imu_fault check(void)
{
	return imu_health_check(&h, now);
}

static void before(void *unused)
{
	ARG_UNUSED(unused);
	now = 0;
	n = 0;
	imu_health_init(&h, ACCEL_FS, GYRO_FS);
}

/* Nothing to trust before the sensor has proven itself. */
ZTEST(imu_health, test_startup_needs_recover_samples)
{
	zassert_equal(check(), IMU_FAULT_NO_DATA);
	good(IMU_HEALTH_RECOVER - 1);
	zassert_equal(check(), IMU_FAULT_NO_DATA);
	good(1);
	zassert_equal(check(), IMU_OK);
}

/* INT1 lost: no sample comes, so only the check at use time can see it.
 * When samples come back the fault latches until they have recovered.
 */
ZTEST(imu_health, test_no_data_timeout)
{
	good(IMU_HEALTH_RECOVER);
	now += IMU_HEALTH_TIMEOUT_US;
	zassert_equal(check(), IMU_OK, "one lost pulse is normal");
	now += 1;
	zassert_equal(check(), IMU_FAULT_NO_DATA);
	good(1);
	zassert_equal(check(), IMU_FAULT_NO_DATA, "latched");
	good(IMU_HEALTH_RECOVER - 1);
	zassert_equal(check(), IMU_OK);
}

/* A single failed fetch is tolerated, several in a row are not. */
ZTEST(imu_health, test_read_errors)
{
	good(IMU_HEALTH_RECOVER);
	read_errors(IMU_HEALTH_READ_ERRORS - 1);
	good(1);
	read_errors(IMU_HEALTH_READ_ERRORS - 1);
	zassert_equal(check(), IMU_OK);
	read_errors(1);
	zassert_equal(check(), IMU_FAULT_READ);
	good(IMU_HEALTH_RECOVER);
	zassert_equal(check(), IMU_OK);
}

/* MISO stuck low: every register reads 0. Stuck high (0xFF) is -1 LSB on
 * every axis, same thing.
 */
ZTEST(imu_health, test_stuck)
{
	const int32_t zero[3] = {0};

	good(IMU_HEALTH_RECOVER);
	same(IMU_HEALTH_STUCK, zero, zero);
	zassert_equal(check(), IMU_OK, "first sample isn't a repeat");
	same(1, zero, zero);
	zassert_equal(check(), IMU_FAULT_STUCK);
	/* and it doesn't age into no data while it keeps coming */
	same(1000, zero, zero);
	zassert_equal(check(), IMU_FAULT_STUCK);
	good(IMU_HEALTH_RECOVER);
	zassert_equal(check(), IMU_OK);
}

/* At full scale the real value is unknown. A short spike is tolerated. */
ZTEST(imu_health, test_range)
{
	const int32_t a[3] = {0, 0, 9806650};
	const int32_t spin[3] = {0, 0, GYRO_FS};
	const int32_t hit[3] = {-ACCEL_FS, 0, 0};
	const int32_t g[3] = {0};

	good(IMU_HEALTH_RECOVER);
	same(IMU_HEALTH_SATURATED - 1, a, spin);
	good(1);
	zassert_equal(check(), IMU_OK);
	same(IMU_HEALTH_SATURATED, a, spin);
	zassert_equal(check(), IMU_FAULT_RANGE, "gyro saturated");
	/* a long run keeps its reason, also on the first good sample after */
	same(100, a, spin);
	good(1);
	zassert_equal(check(), IMU_FAULT_RANGE);
	good(IMU_HEALTH_RECOVER - 1);
	zassert_equal(check(), IMU_OK);
	same(IMU_HEALTH_SATURATED, hit, g);
	zassert_equal(check(), IMU_FAULT_RANGE, "accel saturated");
}

/* A wrong chip id means the data can't be trusted however it looks; only
 * a good re-check, then good samples, clear it.
 */
ZTEST(imu_health, test_who_am_i)
{
	good(IMU_HEALTH_RECOVER);
	imu_health_who_am_i(&h, true);
	zassert_equal(check(), IMU_OK);
	imu_health_who_am_i(&h, false);
	zassert_equal(check(), IMU_FAULT_WHO_AM_I);
	good(10 * IMU_HEALTH_RECOVER);
	zassert_equal(check(), IMU_FAULT_WHO_AM_I);
	imu_health_who_am_i(&h, true);
	good(IMU_HEALTH_RECOVER - 1);
	zassert_equal(check(), IMU_FAULT_WHO_AM_I);
	good(1);
	zassert_equal(check(), IMU_OK);
}

/* Wire off: the driver loops on data ready and reads garbage every
 * ~90 us. A couple of close samples (a late thread) are tolerated.
 */
ZTEST(imu_health, test_rate)
{
	int32_t a[3], g[3];

	good(IMU_HEALTH_RECOVER);
	for (int i = 0; i < IMU_HEALTH_FAST - 1; i++) {
		now += 90;
		live(a, g);
		imu_health_sample(&h, now, true, a, g);
	}
	good(1);
	zassert_equal(check(), IMU_OK);
	for (int i = 0; i < 1000; i++) {
		now += 90;
		live(a, g);
		imu_health_sample(&h, now, true, a, g);
		if (i >= IMU_HEALTH_FAST - 1) {
			zassert_equal(check(), IMU_FAULT_RATE, "sample %d", i);
		}
	}
	good(IMU_HEALTH_RECOVER);
	zassert_equal(check(), IMU_OK);
}

/* Recovery needs good samples in a row: a flapping wire stays bad. */
ZTEST(imu_health, test_recovery_restarts_on_a_bad_sample)
{
	good(IMU_HEALTH_RECOVER);
	read_errors(IMU_HEALTH_READ_ERRORS);
	for (int i = 0; i < 10; i++) {
		good(IMU_HEALTH_RECOVER - 1);
		read_errors(1);
	}
	zassert_not_equal(check(), IMU_OK);
	good(IMU_HEALTH_RECOVER);
	zassert_equal(check(), IMU_OK);
}

/* The microsecond clock wraps after ~71 minutes. */
ZTEST(imu_health, test_clock_wrap)
{
	now = UINT32_MAX - 10 * PERIOD_US;
	good(IMU_HEALTH_RECOVER);
	zassert_true(now < 1000000, "clock wrapped");
	zassert_equal(check(), IMU_OK);
}

ZTEST_SUITE(imu_health, NULL, NULL, before, NULL, NULL);
