/*
 * Control: what runs on every IMU sample after the IMU's own work. Owns
 * the supervisor (supervisor.h) now; the balance controller joins it later
 * (DontFall docs/balance-plan.md).
 *
 * The supervisor is fed from three places, serialised by one lock:
 *  - every IMU sample (control_imu_sample, the IMU's sample callback),
 *  - a kernel timer every CONTROL_TICK_MS that disarms if the samples
 *    stopped: a lost data-ready interrupt never calls the callback, so
 *    the sampling path alone would never notice,
 *  - arm and disarm requests (control_arm, control_disarm), checked
 *    against the latest sample, so the caller gets the answer at once.
 *
 * State changes are printed from the system work queue, never in the
 * sampling path.
 */
#ifndef DONTFALL_M4_CONTROL_H_
#define DONTFALL_M4_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "supervisor.h"

struct control_status {
	enum supervisor_state state;
	enum supervisor_reason reason;
};

/* Start the supervisor DISARMED and the no-sample timer. Call before the
 * IMU delivers samples.
 */
void control_init(void);

/* The IMU's sample callback (imu_sample_cb_t). */
void control_imu_sample(uint32_t now_us, bool ok, float roll, float pitch);

/* Arm if the latest sample allows it (supervisor_arm()): SUPERVISOR_OK,
 * or why not. Any thread.
 */
enum supervisor_reason control_arm(void);

/* Always works. Any thread. */
void control_disarm(void);

/* Snapshot for telemetry. Any thread. */
void control_status(struct control_status *out);

#endif /* DONTFALL_M4_CONTROL_H_ */
