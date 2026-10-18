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

static struct k_spinlock lock;
static struct imu_stats stats; /* guarded by lock */
static struct sensor_value accel[3]; /* m/s^2, latest sample, guarded by lock */
static struct sensor_value gyro[3];  /* rad/s, latest sample, guarded by lock */
static uint32_t last_cycle;  /* only touched by on_drdy */
static bool have_last;

static void stats_reset(struct imu_stats *s)
{
	*s = (struct imu_stats){.gap_min = UINT32_MAX};
}

static void on_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value a[3], g[3];
	uint32_t now = k_cycle_get_32();
	uint32_t fetch;
	int ret;

	ARG_UNUSED(trig);

	/* one fetch reads both sensors; channel_get only converts */
	ret = sensor_sample_fetch(dev);
	fetch = k_cycle_get_32() - now;
	if (ret == 0) {
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, a);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, g);
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
		} else {
			memcpy(accel, a, sizeof(a));
			memcpy(gyro, g, sizeof(g));
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
 * size), so print sensor_values as fixed point with 3 decimals.
 */
static void print_milli(const char *name, const struct sensor_value *v)
{
	int64_t m = sensor_value_to_milli(v);
	const char *sign = m < 0 ? "-" : " ";

	if (m < 0) {
		m = -m;
	}
	printk(" %s %s%d.%03d", name, sign, (int)(m / 1000), (int)(m % 1000));
}

static void imu_task(void *p1, void *p2, void *p3)
{
	struct imu_stats s;
	struct sensor_value a[3], g[3];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(IMU_REPORT_MS);

		K_SPINLOCK(&lock) {
			s = stats;
			stats_reset(&stats);
			memcpy(a, accel, sizeof(a));
			memcpy(g, gyro, sizeof(g));
		}

		/* samples counted over ~IMU_REPORT_MS, so samples/s */
		printk("imu: %u Hz, gap us min %u avg %u max %u, fetch us max %u, errors %u\n",
		       s.samples,
		       s.gaps ? k_cyc_to_us_floor32(s.gap_min) : 0,
		       s.gaps ? k_cyc_to_us_floor32(s.gap_sum / s.gaps) : 0,
		       k_cyc_to_us_floor32(s.gap_max),
		       k_cyc_to_us_floor32(s.fetch_max),
		       s.errors);
		printk("imu: a[m/s2]");
		print_milli("x", &a[0]);
		print_milli("y", &a[1]);
		print_milli("z", &a[2]);
		printk("  g[rad/s]");
		print_milli("x", &g[0]);
		print_milli("y", &g[1]);
		print_milli("z", &g[2]);
		printk("\n");
	}
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
