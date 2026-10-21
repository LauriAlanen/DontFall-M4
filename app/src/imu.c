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
 * Tilt (step 6b): every calibrated sample also goes through the tilt
 * filter (tilt.c) in the same thread, with the measured time since the
 * previous sample: the chip's ODR is ~2% off nominal.
 *
 * Health (step 7b): every fetch also goes through the fault checks
 * (imu_health.c), and the summary thread re-reads WHO_AM_I once a second
 * with a raw SPI read (the SPI driver serialises it with the sensor
 * driver's transfers). While there is a fault, samples are not handed
 * out, the gyro calibration doesn't use them and the tilt filter starts
 * over, so it takes the accelerometer's angles once the IMU is healthy
 * again instead of integrating garbage.
 *
 * Re-init (step 7c): a chip reset or power loss puts its registers back
 * to defaults, ODR "power down" among them: it still answers WHO_AM_I
 * but never makes data again. Right after boot imu_init() saves the
 * registers the driver set up (cfg_regs); the summary thread compares
 * them once a second and writes the saved values back if they changed.
 * Saving them from the chip instead of writing our own values keeps the
 * register encoding in the driver. Only with a good WHO_AM_I: with a
 * broken bus the reads mean nothing.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu.h"
#include "imu_health.h"
#include "tilt.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
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

/* Tilt filter time constant: a common start for balancing robots, tune
 * on the robot (imu-plan.md step 6).
 */
#define TILT_TAU       0.5f /* s */
#define RAD_TO_UDEG    (180e6f / 3.14159265f)

/* Largest readings, 32767 LSB times the sensitivity of the DT ranges:
 * 0.122 mg at 4 g, 17.5 mdps at 500 dps (so it really reaches 573 dps).
 */
#define ACCEL_FS       39202800 /* um/s^2 */
#define GYRO_FS        10008000 /* urad/s */

#define REG_WHO_AM_I   0x0f
#define WHO_AM_I_LSM6DSOX 0x6c
#define SPI_READ       0x80
#define REG_FUNC_CFG_ACCESS 0x01

K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread;

static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(imu));
/* same bus settings as the driver's (mode 3) */
static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(imu),
	SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
	0);

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
	uint32_t faults;    /* samples that started a fault */
	enum imu_fault last_fault;
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
static struct imu_health health; /* guarded by lock */
static int32_t accel[3]; /* um/s^2, latest sample, guarded by lock */
static int32_t gyro[3];  /* urad/s, latest sample minus bias, guarded by lock */
static float roll, pitch; /* rad, latest tilt estimate, guarded by lock */
static bool have_sample; /* guarded by lock */

/* Registers the driver sets at boot, in the order they are written back:
 * CTRL3_C (BDU, auto-increment) first, the ODRs (CTRL1_XL, CTRL2_G) last
 * so the chip starts sampling only when fully set up. FIFO_CTRL4 (FIFO
 * bypass), COUNTER_BDR_REG1 (pulsed data ready), INT1_CTRL (data ready
 * on INT1), CTRL4_C..CTRL10_C (power modes, I3C off), I3C_BUS_AVB.
 */
static const uint8_t cfg_regs[] = {
	0x12, 0x0a, 0x0b, 0x0d, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x62, 0x10, 0x11,
};
static uint8_t cfg_saved[ARRAY_SIZE(cfg_regs)]; /* summary thread only, after init */

/* only touched by on_drdy */
static uint32_t last_cycle;
static enum imu_fault last_fault = IMU_FAULT_NO_DATA;
static bool have_last;
static int32_t bias[3];
static bool calibrated;
static uint32_t settle_n;
static uint32_t cal_n;
static int64_t cal_sum[3];
static int32_t cal_min[3], cal_max[3];
static struct tilt tilt;

static void stats_reset(struct imu_stats *s)
{
	*s = (struct imu_stats){.gap_min = UINT32_MAX};
}

/* Microseconds for imu_health: wraps at 2^32 us like it expects. The
 * cycle counter can't be used, it wraps at 2^32 cycles (~20 s). Tick
 * resolution (100 us) is plenty for a 10 ms timeout.
 */
static uint32_t now_us(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static int read_reg(uint8_t reg, uint8_t *val)
{
	uint8_t tx[2] = {reg | SPI_READ, 0};
	uint8_t rx[2];
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret = spi_transceive_dt(&spi, &tx_set, &rx_set);

	*val = rx[1];
	return ret;
}

static int write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = {reg, val};
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	return spi_write_dt(&spi, &tx_set);
}

static int cfg_save(void)
{
	for (int i = 0; i < ARRAY_SIZE(cfg_regs); i++) {
		int ret = read_reg(cfg_regs[i], &cfg_saved[i]);

		if (ret) {
			return ret;
		}
	}
	return 0;
}

/* If the chip lost its setup, write it back. */
static void cfg_restore_if_lost(void)
{
	int lost = -1;
	uint8_t v = 0;

	for (int i = 0; i < ARRAY_SIZE(cfg_regs) && lost < 0; i++) {
		if (read_reg(cfg_regs[i], &v) == 0 && v != cfg_saved[i]) {
			lost = i;
		}
	}
	if (lost < 0) {
		return;
	}
	printk("imu: chip setup lost (reg 0x%02x 0x%02x, was 0x%02x), writing it again\n",
	       cfg_regs[lost], v, cfg_saved[lost]);
	/* user register bank, the default after a reset anyway */
	write_reg(REG_FUNC_CFG_ACCESS, 0);
	for (int i = 0; i < ARRAY_SIZE(cfg_regs); i++) {
		write_reg(cfg_regs[i], cfg_saved[i]);
	}
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
	int32_t a[3] = {0}, g[3] = {0};
	uint32_t now = k_cycle_get_32();
	uint32_t t = now_us();
	uint32_t fetch;
	enum imu_fault fault;
	bool use;
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
	}

	K_SPINLOCK(&lock) {
		imu_health_sample(&health, t, ret == 0, a, g);
		fault = imu_health_check(&health, t);
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
		}
		if (fault != IMU_OK && fault != last_fault) {
			stats.faults++;
			stats.last_fault = fault;
		}
	}
	last_fault = fault;

	use = fault == IMU_OK && calibrated;
	if (fault != IMU_OK) {
		/* the next healthy sample takes the accelerometer's angles */
		tilt_init(&tilt, TILT_TAU);
	} else if (calibrated) {
		float af[3], gf[3];

		for (int i = 0; i < 3; i++) {
			g[i] -= bias[i];
			af[i] = a[i] * 1e-6f;
			gf[i] = g[i] * 1e-6f;
		}
		/* the first update ignores dt, so no special case for it */
		tilt_update(&tilt, af, gf,
			    (float)(now - last_cycle) / sys_clock_hw_cycles_per_sec());
	} else {
		cal_add(g);
	}

	if (use) {
		K_SPINLOCK(&lock) {
			memcpy(accel, a, sizeof(a));
			memcpy(gyro, g, sizeof(g));
			roll = tilt.roll;
			pitch = tilt.pitch;
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
	enum imu_fault fault, printed = IMU_OK;
	int32_t a[3], g[3];
	float r, p;
	uint8_t id;
	bool ok, id_ok;
	int ret;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msleep(IMU_REPORT_MS);

		ret = read_reg(REG_WHO_AM_I, &id);
		id_ok = ret == 0 && id == WHO_AM_I_LSM6DSOX;
		if (id_ok) {
			cfg_restore_if_lost();
		}

		K_SPINLOCK(&lock) {
			imu_health_who_am_i(&health, id_ok);
			/* also sees samples that stopped coming */
			fault = imu_health_check(&health, now_us());
			s = stats;
			stats_reset(&stats);
			c = cal;
			ok = have_sample;
			memcpy(a, accel, sizeof(a));
			memcpy(g, gyro, sizeof(g));
			r = roll;
			p = pitch;
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
		if (!id_ok) {
			printk("imu: WHO_AM_I 0x%02x, spi %d\n", id, ret);
		}
		if (s.faults) {
			printk("imu: %u faults, last %s\n", s.faults, imu_fault_str(s.last_fault));
		}
		if (fault != IMU_OK) {
			printk("imu: FAULT %s\n", imu_fault_str(fault));
		} else if (printed != IMU_OK) {
			printk("imu: healthy (was %s)\n", imu_fault_str(printed));
		}
		printed = fault;
		if (ok && fault == IMU_OK) {
			print_xyz("imu: a[m/s2]", a);
			print_xyz("  g[rad/s]", g);
			printk("\nimu: tilt [deg]");
			print_micro("roll", (int32_t)(r * RAD_TO_UDEG));
			print_micro("pitch", (int32_t)(p * RAD_TO_UDEG));
			printk("\n");
		}
	}
}

enum imu_fault imu_latest(struct imu_sample *out)
{
	int32_t a[3], g[3];
	float r, p, q[4];
	enum imu_fault fault;
	bool ok;

	K_SPINLOCK(&lock) {
		fault = imu_health_check(&health, now_us());
		ok = have_sample;
		memcpy(a, accel, sizeof(a));
		memcpy(g, gyro, sizeof(g));
		r = roll;
		p = pitch;
	}
	if (fault != IMU_OK) {
		return fault;
	}
	if (!ok) {
		/* healthy, but the gyro calibration isn't done yet */
		return IMU_FAULT_NO_DATA;
	}
	/* convert outside the lock: the sampling thread waits on it */
	for (int i = 0; i < 3; i++) {
		out->accel[i] = a[i] * 1e-6;
		out->gyro[i] = g[i] * 1e-6;
	}
	tilt_quaternion(r, p, q);
	for (int i = 0; i < 4; i++) {
		out->orientation[i] = q[i];
	}
	return IMU_OK;
}

int imu_init(void)
{
	int ret;

	if (!device_is_ready(imu)) {
		printk("imu: %s not ready\n", imu->name);
		return -ENODEV;
	}

	stats_reset(&stats);
	imu_health_init(&health, ACCEL_FS, GYRO_FS);
	tilt_init(&tilt, TILT_TAU);
	ret = sensor_trigger_set(imu, &drdy, on_drdy);
	if (ret) {
		printk("imu: data-ready trigger failed: %d\n", ret);
		return ret;
	}
	/* after the trigger: it routes data ready to INT1 */
	ret = cfg_save();
	if (ret) {
		printk("imu: reading the chip setup failed: %d\n", ret);
		return ret;
	}

	k_thread_create(&imu_thread, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack), imu_task,
			NULL, NULL, NULL, IMU_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_thread, "imu");
	return 0;
}
