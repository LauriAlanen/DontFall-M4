/*
 * IMU fault detection. See imu_health.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_health.h"

#include <string.h>

void imu_health_init(struct imu_health *h, int32_t accel_fs, int32_t gyro_fs)
{
	*h = (struct imu_health){
		.accel_fs = accel_fs,
		.gyro_fs = gyro_fs,
		.fault = IMU_FAULT_NO_DATA,
	};
}

static void set_fault(struct imu_health *h, enum imu_fault f)
{
	h->fault = f;
	h->good_n = 0;
}

/* The conversion to micro-units rounds, so compare a hair below the
 * largest reading.
 */
static bool at_full_scale(int32_t v, int32_t fs)
{
	int32_t lim = fs - fs / 1000;

	return v >= lim || v <= -lim;
}

void imu_health_sample(struct imu_health *h, uint32_t now_us, bool read_ok,
		       const int32_t accel[3], const int32_t gyro[3])
{
	int32_t v[6];
	bool saturated = false;
	/* unsigned differences: right across a wrap of the clock */
	uint32_t gap = now_us - h->last_us;
	bool fast = h->have_last && gap < IMU_HEALTH_MIN_GAP_US;

	/* Since the last fetch of any kind, not the last good one: a run of
	 * bad samples keeps its own reason.
	 */
	if (h->have_last && gap > IMU_HEALTH_TIMEOUT_US) {
		set_fault(h, IMU_FAULT_NO_DATA);
	}
	h->have_last = true;
	h->last_us = now_us;
	h->fast_n = fast ? h->fast_n + 1 : 0;
	if (h->fast_n >= IMU_HEALTH_FAST) {
		set_fault(h, IMU_FAULT_RATE);
	}
	if (fast) {
		h->good_n = 0;
		return;
	}

	if (!read_ok) {
		h->good_n = 0;
		if (++h->read_errors >= IMU_HEALTH_READ_ERRORS) {
			set_fault(h, IMU_FAULT_READ);
		}
		return;
	}
	h->read_errors = 0;

	memcpy(v, accel, 3 * sizeof(v[0]));
	memcpy(v + 3, gyro, 3 * sizeof(v[0]));
	for (int i = 0; i < 3; i++) {
		saturated |= at_full_scale(accel[i], h->accel_fs) ||
			     at_full_scale(gyro[i], h->gyro_fs);
	}
	h->saturated_n = saturated ? h->saturated_n + 1 : 0;
	h->same_n = memcmp(v, h->prev, sizeof(v)) == 0 ? h->same_n + 1 : 0;
	memcpy(h->prev, v, sizeof(v));

	if (h->same_n >= IMU_HEALTH_STUCK) {
		set_fault(h, IMU_FAULT_STUCK);
	}
	/* last: a sensor pinned at full scale also repeats itself */
	if (h->saturated_n >= IMU_HEALTH_SATURATED) {
		set_fault(h, IMU_FAULT_RANGE);
	}
	if (h->saturated_n || h->same_n >= IMU_HEALTH_STUCK) {
		h->good_n = 0;
		return;
	}

	h->have_good = true;
	h->last_good_us = now_us;
	/* also at startup: the first samples have to prove the chip works */
	if (h->fault != IMU_OK && !h->who_am_i_bad && ++h->good_n >= IMU_HEALTH_RECOVER) {
		h->fault = IMU_OK;
	}
}

void imu_health_who_am_i(struct imu_health *h, bool ok)
{
	if (!ok) {
		h->who_am_i_bad = true;
		set_fault(h, IMU_FAULT_WHO_AM_I);
	} else if (h->who_am_i_bad) {
		/* recover from here on, like from any other fault */
		h->who_am_i_bad = false;
		h->good_n = 0;
	}
}

enum imu_fault imu_health_check(const struct imu_health *h, uint32_t now_us)
{
	if (h->fault != IMU_OK) {
		return h->fault;
	}
	if (!h->have_good || now_us - h->last_good_us > IMU_HEALTH_TIMEOUT_US) {
		return IMU_FAULT_NO_DATA;
	}
	return IMU_OK;
}

const char *imu_fault_str(enum imu_fault f)
{
	switch (f) {
	case IMU_OK:
		return "ok";
	case IMU_FAULT_NO_DATA:
		return "no data";
	case IMU_FAULT_READ:
		return "read";
	case IMU_FAULT_STUCK:
		return "stuck";
	case IMU_FAULT_RANGE:
		return "range";
	case IMU_FAULT_WHO_AM_I:
		return "who_am_i";
	case IMU_FAULT_RATE:
		return "rate";
	}
	return "?";
}
