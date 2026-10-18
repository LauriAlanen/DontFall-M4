/*
 * IMU on SPI5. See imu.h.
 *
 * The driver (drivers/sensor/st/lsm6dso, on ST's register HAL in hal_st)
 * probes the chip at boot, before main(): it checks WHO_AM_I, resets the
 * chip and writes the ODR, full scale and INT1 setup from the DT node.
 * Here we only use the generic sensor API, so the same code works with
 * any Zephyr IMU driver.
 *
 * On data ready the driver's own thread (CONFIG_LSM6DSO_TRIGGER_OWN_THREAD)
 * calls on_drdy(). It fetches the sample and updates the timing stats.
 * The summary thread swaps the stats out once a second and prints them,
 * so printk never runs in the sampling path.
 *
 * Gyro bias: a MEMS gyro reads a small nonzero rate at rest, and the tilt
 * filter integrates it into a drifting angle. At startup on_drdy()
 * averages CAL_SAMPLES of gyro and takes that as the bias, but only if
 * every axis stayed within CAL_MAX_RANGE (the module was still) and the
 * average is plausible (CAL_MAX_BIAS). Otherwise it starts over, forever:
 * running on an uncalibrated gyro is worse than waiting. Until then there
 * is no sample for imu_latest(), so nothing is published.
 *
 * Samples are kept as integer micro-units (um/s^2, urad/s), so the bias
 * subtraction at the sensor rate is integer maths.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define IMU_STACK_SIZE 1024
/* below uros (5): printing is the least important thing on the M4 */
#define IMU_PRIORITY   K_PRIO_PREEMPT(7)
#define IMU_REPORT_MS  1000

/* ~1 s at the ~850 Hz ODR */
#define CAL_SAMPLES    850
/* Skip ~100 ms first: right after the driver turns the gyro on it reads
 * rad/s-sized garbage for a few tens of ms (turn-on time).
 */
#define CAL_SETTLE     85
/* Max - min per gyro axis over the window. Measured at rest: ~0.01 rad/s
 * when quiet, up to ~0.1 with the desk vibrating (that averages out).
 * Picking the module up or turning it by hand is well above.
 */
#define CAL_MAX_RANGE  100000 /* urad/s, ~5.7 dps */
/* LSM6DSOX zero-rate level is +-1 dps typical; far above that, something
 * is turning the module slowly and steadily.
 */
#define CAL_MAX_BIAS   100000 /* urad/s, ~5.7 dps */

K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread;

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(imu));

/* Timing of the samples in one report window, in hw cycles
 * (k_cycle_get_32, the CPU clock on the M4).
 */
struct imu_stats {
	uint32_t samples;
	uint32_t errors;   /* failed fetches */
	uint32_t gaps;     /* gaps measured (samples, minus the window's first) */
	uint32_t gap_min;
	uint32_t gap_max;
	uint32_t gap_sum;
	uint32_t fetch_max; /* longest sensor_sample_fetch: SPI cost per sample */
};

/* Result of the gyro calibration, for the summary thread to print. */
struct imu_cal {
	bool done;
	uint32_t attempts;    /* windows finished, good or not */
	int32_t bias[3];      /* urad/s, valid once done */
	int32_t range[3];     /* urad/s, max - min in the last window */
};

static struct k_spinlock lock;
static struct imu_stats stats; /* guarded by lock */
static struct imu_cal cal;     /* guarded by lock */
static int32_t accel[3]; /* um/s^2, latest sample, guarded by lock */
static int32_t gyro[3];  /* urad/s, latest sample minus bias, guarded by lock */
static bool have_sample; /* guarded by lock */

/* only touched by on_drdy */
static uint32_t last_cycle;
static bool have_last;
static int32_t bias[3];
static bool calibrated;
static uint32_t settle_n;
static uint32_t cal_n;
static int64_t cal_sum[3];
static int32_t cal_min[3], cal_max[3];

static void stats_reset(struct imu_stats *s)
{
	*s = (struct imu_stats){.gap_min = UINT32_MAX};
}

/* Feed one gyro sample into the calibration window. When the window is
 * full, take its average as the bias if the module was still, else start
 * a new window. Sampling thread only.
 */
static void cal_add(const int32_t g[3])
{
	struct imu_cal result = {0};
	bool still = true;

	if (settle_n < CAL_SETTLE) {
		settle_n++;
		return;
	}
	for (int i = 0; i < 3; i++) {
		cal_sum[i] += g[i];
		cal_min[i] = cal_n ? MIN(cal_min[i], g[i]) : g[i];
		cal_max[i] = cal_n ? MAX(cal_max[i], g[i]) : g[i];
	}
	if (++cal_n < CAL_SAMPLES) {
		return;
	}

	for (int i = 0; i < 3; i++) {
		result.bias[i] = (int32_t)(cal_sum[i] / CAL_SAMPLES);
		result.range[i] = cal_max[i] - cal_min[i];
		if (result.range[i] > CAL_MAX_RANGE ||
		    result.bias[i] > CAL_MAX_BIAS || result.bias[i] < -CAL_MAX_BIAS) {
			still = false;
		}
	}
	cal_n = 0;
	memset(cal_sum, 0, sizeof(cal_sum));

	if (still) {
		memcpy(bias, result.bias, sizeof(bias));
		calibrated = true;
	}
	K_SPINLOCK(&lock) {
		result.done = still;
		result.attempts = cal.attempts + 1;
		cal = result;
	}
}

static void on_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value av[3], gv[3];
	int32_t a[3], g[3];
	uint32_t now = k_cycle_get_32();
	uint32_t fetch;
	int ret;

	ARG_UNUSED(trig);

	/* one fetch reads both sensors; channel_get only converts */
	ret = sensor_sample_fetch(dev);
	fetch = k_cycle_get_32() - now;
	if (ret == 0) {
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, av);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gv);
		for (int i = 0; i < 3; i++) {
			a[i] = (int32_t)sensor_value_to_micro(&av[i]);
			g[i] = (int32_t)sensor_value_to_micro(&gv[i]);
		}
		if (calibrated) {
			for (int i = 0; i < 3; i++) {
				g[i] -= bias[i];
			}
		} else {
			cal_add(g);
		}
	}

	K_SPINLOCK(&lock) {
		stats.samples++;
		stats.fetch_max = MAX(stats.fetch_max, fetch);
		if (have_last) {
			uint32_t gap = now - last_cycle; /* wraps correctly */

			stats.gaps++;
			stats.gap_sum += gap;
			stats.gap_min = MIN(stats.gap_min, gap);
			stats.gap_max = MAX(stats.gap_max, gap);
		}
		if (ret) {
			stats.errors++;
		} else if (calibrated) {
			memcpy(accel, a, sizeof(a));
			memcpy(gyro, g, sizeof(g));
			have_sample = true;
		}
	}
	last_cycle = now;
	have_last = true;
}

/* data ready of the accelerometer; gyro runs at the same ODR in sync */
static const struct sensor_trigger drdy = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

/* printk has no float support (CONFIG_CBPRINTF_FP_SUPPORT costs code
 * size), so print micro-units as fixed point with 4 decimals.
 */
static void print_micro(const char *name, int32_t u)
{
	const char *sign = u < 0 ? "-" : " ";
	/* in units of 1e-4, rounded toward zero */
	uint32_t m = (u < 0 ? -(uint32_t)u : (uint32_t)u) / 100;

	printk(" %s %s%u.%04u", name, sign, m / 10000, m % 10000);
}

static void print_xyz(const char *label, const int32_t v[3])
{
	printk("%s", label);
	print_micro("x", v[0]);
	print_micro("y", v[1]);
	print_micro("z", v[2]);
}

/* Print what the calibration did since the last report. */
static void print_cal(const struct imu_cal *c, uint32_t *printed)
{
	if (c->attempts == *printed) {
		return;
	}
	*printed = c->attempts;
	if (c->done) {
		print_xyz("imu: gyro bias [rad/s]", c->bias);
		printk(" (try %u)\n", c->attempts);
	} else {
		print_xyz("imu: moving, gyro calibration retrying; range [rad/s]", c->range);
		printk("\n");
	}
}

static void imu_task(void *p1, void *p2, void *p3)
{
	struct imu_stats s;
	struct imu_cal c;
	uint32_t cal_printed = 0;
	int32_t a[3], g[3];
	bool ok;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(IMU_REPORT_MS);

		K_SPINLOCK(&lock) {
			s = stats;
			stats_reset(&stats);
			c = cal;
			ok = have_sample;
			memcpy(a, accel, sizeof(a));
			memcpy(g, gyro, sizeof(g));
		}

		print_cal(&c, &cal_printed);

		/* samples counted over ~IMU_REPORT_MS, so samples/s */
		printk("imu: %u Hz, gap us min %u avg %u max %u, fetch us max %u, errors %u\n",
		       s.samples,
		       s.gaps ? k_cyc_to_us_floor32(s.gap_min) : 0,
		       s.gaps ? k_cyc_to_us_floor32(s.gap_sum / s.gaps) : 0,
		       k_cyc_to_us_floor32(s.gap_max),
		       k_cyc_to_us_floor32(s.fetch_max),
		       s.errors);
		if (ok) {
			print_xyz("imu: a[m/s2]", a);
			print_xyz("  g[rad/s]", g);
			printk("\n");
		}
	}
}

bool imu_latest(struct imu_sample *out)
{
	int32_t a[3], g[3];
	bool ok;

	K_SPINLOCK(&lock) {
		ok = have_sample;
		memcpy(a, accel, sizeof(a));
		memcpy(g, gyro, sizeof(g));
	}
	if (!ok) {
		return false;
	}
	/* convert outside the lock: the sampling thread waits on it */
	for (int i = 0; i < 3; i++) {
		out->accel[i] = a[i] * 1e-6;
		out->gyro[i] = g[i] * 1e-6;
	}
	return true;
}

int imu_init(void)
{
	int ret;

	if (!device_is_ready(imu)) {
		printk("imu: %s not ready\n", imu->name);
		return -ENODEV;
	}

	stats_reset(&stats);
	ret = sensor_trigger_set(imu, &drdy, on_drdy);
	if (ret) {
		printk("imu: data-ready trigger failed: %d\n", ret);
		return ret;
	}

	k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_task,
			NULL, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread, "imu");
	return 0;
}
