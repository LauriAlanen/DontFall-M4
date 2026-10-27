/*
 * Supervisor: owns "may the motors run?" (DontFall
 * docs/archive/failsafe-plan.md). The balance controller computes, the
 * supervisor decides whether its output is used; nothing else turns the
 * motors on.
 *
 * Plain C with no Zephyr dependencies, so the same file is built into the
 * firmware and into the host tests (tests/supervisor). Not thread safe:
 * one caller (the IMU sampling path) makes all the calls; requests from
 * other threads are posted to it.
 *
 * States:
 *  - DISARMED: motors off. Leaves on an arm request when all the arm
 *    conditions hold.
 *  - ARMED: controller output used. Leaves on a disarm request or any
 *    disarm condition.
 *
 * A disarm latches: the supervisor never re-arms by itself, also when the
 * cause goes away. The reason for the last disarm is kept for reporting.
 *
 * Lost commands don't disarm: a command older than
 * SUPERVISOR_CMD_TIMEOUT_US gives a zero setpoint (balance in place).
 * Disarming is for when balancing isn't possible, not for a quiet A7.
 *
 * Times are when things happened on the M4's own clock, never message
 * stamps (the A7 and M4 clocks differ).
 */
#ifndef DONTFALL_M4_SUPERVISOR_H_
#define DONTFALL_M4_SUPERVISOR_H_

#include <stdbool.h>
#include <stdint.h>

#define SUPERVISOR_DEG (3.14159265f / 180.0f)
/* arm only while held near upright by hand */
#define SUPERVISOR_ARM_TILT     (5.0f * SUPERVISOR_DEG)
/* past what can still be caught (8.5 deg in the balance simulation with
 * the guessed robot and motor, DontFall docs/balance-plan.md step 4b),
 * with room for a better flywheel
 */
#define SUPERVISOR_DISARM_TILT  (15.0f * SUPERVISOR_DEG)
/* the usual cmd_vel timeout */
#define SUPERVISOR_CMD_TIMEOUT_US 500000

enum supervisor_state {
	SUPERVISOR_DISARMED = 0,
	SUPERVISOR_ARMED,
};

/* Why it is disarmed, or why an arm request was refused. */
enum supervisor_reason {
	SUPERVISOR_OK = 0, /* armed, or an arm request accepted */
	SUPERVISOR_BOOT,   /* not armed since start */
	SUPERVISOR_REQUEST,
	SUPERVISOR_IMU,
	SUPERVISOR_TILT,
};

/* What the supervisor looks at, fresh for every call. New conditions
 * (motor driver fault, controller saturated) become new fields here.
 */
struct supervisor_input {
	uint32_t now_us; /* any free-running microsecond clock, may wrap */
	bool imu_ok;
	float tilt; /* rad from upright, in any direction (0..pi) */
};

/* Wanted motion: forward speed (m/s) and turn rate (rad/s), REP 103. */
struct supervisor_setpoint {
	float speed;
	float turn;
};

struct supervisor {
	enum supervisor_state state;
	enum supervisor_reason reason;
	bool have_cmd;
	uint32_t cmd_us;
	struct supervisor_setpoint cmd;
};

/* Start DISARMED, reason SUPERVISOR_BOOT. */
void supervisor_init(struct supervisor *s);

/* Check the disarm conditions; call it for every IMU sample. */
void supervisor_update(struct supervisor *s, const struct supervisor_input *in);

/* Arm if the arm conditions hold. Returns SUPERVISOR_OK when armed (also
 * when it already was), else why not. A refusal doesn't change the
 * reason kept for the last disarm.
 */
enum supervisor_reason supervisor_arm(struct supervisor *s, const struct supervisor_input *in);

/* Always works, also while already disarmed. */
void supervisor_disarm(struct supervisor *s);

/* A motion command that arrived at now_us. Non-finite values are
 * ignored, so a bad message can't turn into a setpoint.
 */
void supervisor_command(struct supervisor *s, uint32_t now_us,
			const struct supervisor_setpoint *cmd);

/* The setpoint to use at now_us: the latest command while armed and the
 * command is fresh, else zero.
 */
struct supervisor_setpoint supervisor_setpoint(const struct supervisor *s, uint32_t now_us);

static inline bool supervisor_armed(const struct supervisor *s)
{
	return s->state == SUPERVISOR_ARMED;
}

/* SUPERVISOR_OK while armed, else why it disarmed. */
static inline enum supervisor_reason supervisor_reason(const struct supervisor *s)
{
	return s->reason;
}

/* Short name for logs and the status topic, e.g. "tilt". */
const char *supervisor_reason_str(enum supervisor_reason r);

#endif /* DONTFALL_M4_SUPERVISOR_H_ */
