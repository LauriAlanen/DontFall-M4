/*
 * Supervisor glue. See control.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>

#include "balance.h"
#include "imu_health.h"
#include "tilt.h"
#include "uptime_us.h"
#include "wdog.h"

/* how often to look for samples that stopped */
#define CONTROL_TICK_MS 10
/* the IMU's own no-data limit: ~12 samples at 850 Hz */
#define SAMPLE_TIMEOUT_US IMU_HEALTH_TIMEOUT_US
/* N m, a GUESS like the simulation's (plant_guess.t_max) until the
 * GM4108H's datasheet numbers are in
 */
#define TORQUE_MAX 0.2f
/* no encoder yet: the controller sees the wheel standing still */
#define WHEEL_SPEED 0.0f

static struct k_spinlock lock;
static struct supervisor sup;      /* guarded by lock */
static uint32_t last_sample_us;    /* guarded by lock */
static struct supervisor_input last_in; /* guarded by lock, imu_ok false until a sample */
static struct control_status shown; /* guarded by lock: last one printed */
static struct balance bal;          /* constant after init */
static float torque;                /* guarded by lock; the motor stub */
static uint32_t loop_max_cycles;    /* guarded by lock */

static struct k_timer tick;
static struct k_work print_work;

/* Lock held. Print from the work queue if the state or reason moved. */
static void note_change(void)
{
	if (sup.state != shown.state || sup.reason != shown.reason) {
		shown.state = sup.state;
		shown.reason = sup.reason;
		k_work_submit(&print_work);
	}
}

void control_imu_sample(uint32_t now_us, bool ok, float roll, float pitch, float pitch_rate)
{
	uint32_t start = k_cycle_get_32();
	struct supervisor_input in = {
		.now_us = now_us,
		.imu_ok = ok,
		.tilt = ok ? tilt_from_upright(roll, pitch) : NAN,
		.rate = ok ? pitch_rate : NAN,
	};

	K_SPINLOCK(&lock) {
		bool armed;

		/* saturated: the previous sample's output, one sample late */
		in.saturated = fabsf(torque) >= bal.t_max;
		last_sample_us = now_us;
		last_in = in;
		supervisor_update(&sup, &in);
		note_change();
		armed = ok && supervisor_armed(&sup);
		torque = balance_update(&bal, pitch, pitch_rate, WHEEL_SPEED, armed);
		loop_max_cycles = MAX(loop_max_cycles, k_cycle_get_32() - start);
	}
}

/* ISR context. */
static void on_tick(struct k_timer *timer)
{
	const struct supervisor_input in = {
		.now_us = uptime_us(),
		.imu_ok = false,
		.tilt = NAN,
	};

	ARG_UNUSED(timer);

	wdog_checkin(WDOG_CONTROL_TICK);
	K_SPINLOCK(&lock) {
		if (in.now_us - last_sample_us > SAMPLE_TIMEOUT_US) {
			supervisor_update(&sup, &in);
			note_change();
			torque = 0.0f;
		}
	}
}

/* Prints the state as it is now: two quick changes print once. */
static void print_state(struct k_work *work)
{
	struct control_status s;

	ARG_UNUSED(work);

	control_status(&s);
	if (s.state == SUPERVISOR_ARMED) {
		printk("control: ARMED (loop max %u ns)\n", s.loop_max_ns);
	} else {
		printk("control: DISARMED (%s, loop max %u ns)\n", supervisor_reason_str(s.reason),
		       s.loop_max_ns);
	}
}

void control_init(void)
{
	supervisor_init(&sup);
	balance_init(&bal, TORQUE_MAX);
	shown = (struct control_status){.state = sup.state, .reason = sup.reason};
	k_work_init(&print_work, print_state);
	k_timer_init(&tick, on_tick, NULL);
	k_timer_start(&tick, K_MSEC(CONTROL_TICK_MS), K_MSEC(CONTROL_TICK_MS));
}

enum supervisor_reason control_arm(void)
{
	enum supervisor_reason r;

	K_SPINLOCK(&lock) {
		struct supervisor_input in = last_in;

		in.now_us = uptime_us();
		/* the timer may not have run yet since the samples stopped */
		if (in.now_us - last_sample_us > SAMPLE_TIMEOUT_US) {
			in.imu_ok = false;
		}
		r = supervisor_arm(&sup, &in);
		note_change();
	}
	return r;
}

void control_disarm(void)
{
	K_SPINLOCK(&lock) {
		supervisor_disarm(&sup);
		note_change();
		torque = 0.0f;
	}
}

void control_command(float speed, float turn)
{
	const struct supervisor_setpoint cmd = {.speed = speed, .turn = turn};

	K_SPINLOCK(&lock) {
		supervisor_command(&sup, uptime_us(), &cmd);
	}
}

void control_status(struct control_status *out)
{
	K_SPINLOCK(&lock) {
		out->state = sup.state;
		out->reason = sup.reason;
		out->setpoint = supervisor_setpoint(&sup, uptime_us());
		out->torque = torque;
		out->loop_max_ns = (uint32_t)k_cyc_to_ns_floor64(loop_max_cycles);
	}
}
