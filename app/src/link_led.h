/*
 * Link LED: the orange LED LD7 blinks while the micro-ROS session with the
 * Agent on the A7 is up, and is off otherwise. uros.c sets it.
 *
 * A blinking LD7 proves the chain the M4 depends on: remoteproc loaded
 * and started it, rpmsg is up, and the Agent on Linux answers.
 */
#ifndef DONTFALL_M4_LINK_LED_H_
#define DONTFALL_M4_LINK_LED_H_

#include <stdbool.h>

/* Configure LD7 (off). Returns 0, or a negative errno. */
int link_led_init(void);

/* The Agent session came up or went away. Thread context. */
void link_led_set(bool up);

#endif /* DONTFALL_M4_LINK_LED_H_ */
