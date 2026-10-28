/*
 * Balance controller for the reaction wheel balancer (DontFall
 * docs/archive/balance-plan.md): full-state feedback
 *
 *     torque = -(k[0] theta + k[1] theta_dot + k[2] omega)
 *
 * with the LQR gains from balance_gains.h, clamped to the motor's limit.
 * theta is the body's tilt from upright, theta_dot its rate straight from
 * the gyro (bias removed, not from the tilt filter), omega the wheel's
 * speed relative to the body. Positive torque speeds the wheel up the
 * positive way and pushes the body the negative way.
 *
 * The supervisor decides whether the output is used; the controller also
 * gives zero when told it is disarmed, or on an input that isn't a number.
 *
 * Plain C with no Zephyr dependencies, so the same file is built into the
 * firmware and into the host tests (tests/balance).
 */
#ifndef DONTFALL_M4_BALANCE_H_
#define DONTFALL_M4_BALANCE_H_

#include <stdbool.h>

struct balance {
	float k[3];  /* N m per rad, per rad/s, per rad/s */
	float t_max; /* N m */
};

/* The LQR gains and a torque limit t_max (N m, > 0). */
void balance_init(struct balance *b, float t_max);

/* Torque for one sample, in -t_max..t_max N m. */
float balance_update(const struct balance *b, float theta, float theta_dot, float omega,
		     bool armed);

#endif /* DONTFALL_M4_BALANCE_H_ */
