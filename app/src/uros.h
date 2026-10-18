/*
 * The M4 as a micro-ROS node.
 *
 * Node "dontfall_m4" publishes the raw IMU as sensor_msgs/Imu on
 * /dontfall_m4/imu at 50 Hz (best effort, no orientation yet). It waits
 * for the micro-ROS Agent on the A7 before creating anything, pings it once a
 * second while connected, and starts over (new session, all entities
 * created again) when a ping goes unanswered, e.g. after an Agent restart.
 */
#ifndef DONTFALL_M4_UROS_H_
#define DONTFALL_M4_UROS_H_

/* Start the micro-ROS thread. Returns 0, or a negative errno. */
int uros_start(void);

#endif /* DONTFALL_M4_UROS_H_ */
