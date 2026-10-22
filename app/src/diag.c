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
}
