/*
 * IMU on SPI5. See imu.h.
 *
 * The driver (drivers/sensor/st/lsm6dso, on ST's register HAL in hal_st)
 * probes the chip at boot, before main(): it checks WHO_AM_I, resets the
 * chip and writes the ODR and full scale from the DT node. Here we only
 * use the generic sensor API, so the same code works with any Zephyr IMU
 * driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu.h"

#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define IMU_STACK_SIZE 1024
/* below uros (5): printing is the least important thing on the M4 */
#define IMU_PRIORITY   K_PRIO_PREEMPT(7)
#define IMU_PERIOD_MS  250

K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread;

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(imu));

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
	struct sensor_value accel[3]; /* m/s^2 */
	struct sensor_value gyro[3];  /* rad/s */
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(IMU_PERIOD_MS);

		/* one fetch reads both sensors; channel_get only converts */
		ret = sensor_sample_fetch(imu);
		if (ret) {
			printk("imu: fetch failed: %d\n", ret);
			continue;
		}
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);

		printk("imu: a[m/s2]");
		print_milli("x", &accel[0]);
		print_milli("y", &accel[1]);
		print_milli("z", &accel[2]);
		printk("  g[rad/s]");
		print_milli("x", &gyro[0]);
		print_milli("y", &gyro[1]);
		print_milli("z", &gyro[2]);
		printk("\n");
	}
}

int imu_init(void)
{
	if (!device_is_ready(imu)) {
		printk("imu: %s not ready\n", imu->name);
		return -ENODEV;
	}

	k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_task,
			NULL, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread, "imu");
	return 0;
}
