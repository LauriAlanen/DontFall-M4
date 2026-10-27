/*
 * Shared by the balance tests: the simulated loop's timing and a way to
 * look at a run.
 *
 * The plant is stepped SUBSTEPS times per control period, with the
 * torque held between samples like the real loop at the IMU rate.
 *
 * Build with -DBALANCE_CSV=1 to print each run as "csv,..." lines for
 * tests/balance/plot.py.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SIM_H
#define SIM_H

#include <zephyr/ztest.h>

#include "plant.h"

#define DT       (1.0f / 850.0f) /* measured ODR, docs/archive/imu-plan.md step 3 */
#define SUBSTEPS 4
#define DEG      (3.14159265f / 180.0f)

/* run NULL: not printed */
static inline void csv_header(const char *run)
{
#ifdef BALANCE_CSV
	if (run == NULL) {
		return;
	}
	printk("csv,%s,t,theta,theta_dot,omega,torque\n", run);
#else
	ARG_UNUSED(run);
#endif
}

static inline void csv_row(const char *run, float t, const struct plant_state *s, float torque)
{
#ifdef BALANCE_CSV
	if (run == NULL) {
		return;
	}
	printk("csv,%s,%.5f,%.6f,%.6f,%.6f,%.6f\n", run, (double)t, (double)s->theta,
	       (double)s->theta_dot, (double)s->omega, (double)torque);
#else
	ARG_UNUSED(run);
	ARG_UNUSED(t);
	ARG_UNUSED(s);
	ARG_UNUSED(torque);
#endif
}

/* One control period with the torque held. */
static inline void period(const struct plant_params *p, struct plant_state *s, float torque)
{
	for (int i = 0; i < SUBSTEPS; i++) {
		plant_step(p, s, torque, DT / SUBSTEPS);
	}
}

#endif
