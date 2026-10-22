/*
 * The M4's status as diagnostic_msgs/DiagnosticArray, the standard ROS
 * type for health reports (rqt_robot_monitor, diagnostic_aggregator read
 * it). One status per part:
 *  - "dontfall_m4: imu": OK, or ERROR with the fault (imu_health.h)
 *  - "dontfall_m4: supervisor": "armed" or "disarmed (reason)"; WARN when
 *    a fault disarmed it, OK when it was asked to or never armed
 *
 * Builds the message only; uros.c publishes it.
 */
#ifndef DONTFALL_M4_DIAG_H_
#define DONTFALL_M4_DIAG_H_

#include <stdint.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>

/* Point the message at static storage and set the fields that never
 * change. Once, before diag_fill().
 */
void diag_init(diagnostic_msgs__msg__DiagnosticArray *msg);

/* Fill in the current state, stamped stamp_ns (ns since the epoch). */
void diag_fill(diagnostic_msgs__msg__DiagnosticArray *msg, int64_t stamp_ns);

#endif /* DONTFALL_M4_DIAG_H_ */
