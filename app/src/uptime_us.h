/*
 * The microsecond clock the plain C modules (imu_health, supervisor) take
 * their times from. One definition, so every caller uses the same clock.
 *
 * Wraps at 2^32 us (~71 min); the modules compare times with unsigned
 * differences, which stay right across the wrap. The cycle counter can't
 * be used, it wraps at 2^32 cycles (~20 s). Tick resolution (100 us) is
 * plenty for millisecond timeouts.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DONTFALL_M4_UPTIME_US_H_
#define DONTFALL_M4_UPTIME_US_H_

#include <stdint.h>
#include <zephyr/kernel.h>

static inline uint32_t uptime_us(void)
{
	return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

#endif /* DONTFALL_M4_UPTIME_US_H_ */
