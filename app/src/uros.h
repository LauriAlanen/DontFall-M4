/*
 * The M4 as a micro-ROS node.
 *
 * Hello world for now: node "dontfall_m4" publishes a std_msgs/Int32
 * counter on /dontfall_m4/counter once a second. It waits for the
 * micro-ROS Agent on the A7 before creating anything, pings it once a
 * second while connected, and starts over (new session, all entities
 * created again) when a ping goes unanswered, e.g. after an Agent restart.
 */
#ifndef DONTFALL_M4_UROS_H_
#define DONTFALL_M4_UROS_H_

/* Start the micro-ROS thread. Returns 0, or a negative errno. */
int uros_start(void);

#endif /* DONTFALL_M4_UROS_H_ */
