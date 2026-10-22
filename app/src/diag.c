/*
 * M4 status message. See diag.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "diag.h"

#include <string.h>
#include <zephyr/sys/printk.h>

#include "control.h"
#include "imu.h"
#include "ros_str.h"

enum { DIAG_IMU, DIAG_SUPERVISOR, DIAG_COUNT };

#define LEVEL_OK    diagnostic_msgs__msg__DiagnosticStatus__OK
#define LEVEL_WARN  diagnostic_msgs__msg__DiagnosticStatus__WARN
#define LEVEL_ERROR diagnostic_msgs__msg__DiagnosticStatus__ERROR

static diagnostic_msgs__msg__DiagnosticStatus status[DIAG_COUNT];
static char supervisor_message[32];
/* setpoint in use, under the supervisor status */
enum { VALUE_SPEED, VALUE_TURN, VALUE_COUNT };
static diagnostic_msgs__msg__KeyValue values[VALUE_COUNT];
static char value_buf[VALUE_COUNT][16];

/* printk has no float support: fixed point with 3 decimals, clamped so
 * the conversion can't overflow (the value is only for reading).
 */
static void set_milli(int i, float v)
{
	float c = v > 1e6f ? 1e6f : (v < -1e6f ? -1e6f : v);
	int32_t m = (int32_t)(c * 1000.0f + (c < 0 ? -0.5f : 0.5f));
	uint32_t a = m < 0 ? -(uint32_t)m : (uint32_t)m;

	snprintk(value_buf[i], sizeof(value_buf[i]), "%s%u.%03u", m < 0 ? "-" : "", a / 1000,
		 a % 1000);
	ros_str_set(&values[i].value, value_buf[i]);
}

void diag_init(diagnostic_msgs__msg__DiagnosticArray *msg)
{
	memset(msg, 0, sizeof(*msg));
	memset(status, 0, sizeof(status));
	ros_str_set(&msg->header.frame_id, "");
	msg->status.data = status;
	msg->status.size = DIAG_COUNT;
	msg->status.capacity = DIAG_COUNT;

	ros_str_set(&status[DIAG_IMU].name, "dontfall_m4: imu");
	ros_str_set(&status[DIAG_IMU].hardware_id, "lsm6dsox");
	ros_str_set(&status[DIAG_IMU].message, "");
	ros_str_set(&status[DIAG_SUPERVISOR].name, "dontfall_m4: supervisor");
	ros_str_set(&status[DIAG_SUPERVISOR].hardware_id, "");
	ros_str_set(&status[DIAG_SUPERVISOR].message, "");
	ros_str_set(&values[VALUE_SPEED].key, "speed");
	ros_str_set(&values[VALUE_TURN].key, "turn");
	set_milli(VALUE_SPEED, 0.0f);
	set_milli(VALUE_TURN, 0.0f);
	status[DIAG_SUPERVISOR].values.data = values;
	status[DIAG_SUPERVISOR].values.size = VALUE_COUNT;
	status[DIAG_SUPERVISOR].values.capacity = VALUE_COUNT;
}

void diag_fill(diagnostic_msgs__msg__DiagnosticArray *msg, int64_t stamp_ns)
{
	struct imu_sample unused;
	enum imu_fault fault = imu_latest(&unused);
	struct control_status c;

	control_status(&c);

	msg->header.stamp.sec = (int32_t)(stamp_ns / 1000000000);
	msg->header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000);

	status[DIAG_IMU].level = fault == IMU_OK ? LEVEL_OK : LEVEL_ERROR;
	ros_str_set(&status[DIAG_IMU].message, imu_fault_str(fault));

	if (c.state == SUPERVISOR_ARMED) {
		status[DIAG_SUPERVISOR].level = LEVEL_OK;
		ros_str_set(&status[DIAG_SUPERVISOR].message, "armed");
	} else {
		bool asked = c.reason == SUPERVISOR_BOOT || c.reason == SUPERVISOR_REQUEST;

		status[DIAG_SUPERVISOR].level = asked ? LEVEL_OK : LEVEL_WARN;
		snprintk(supervisor_message, sizeof(supervisor_message), "disarmed (%s)",
			 supervisor_reason_str(c.reason));
		ros_str_set(&status[DIAG_SUPERVISOR].message, supervisor_message);
	}
	set_milli(VALUE_SPEED, c.setpoint.speed);
	set_milli(VALUE_TURN, c.setpoint.turn);
}
