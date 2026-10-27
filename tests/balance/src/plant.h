/*
 * Simulated reaction wheel balancer (docs/balance-plan.md, "The model").
 * Test code only, never in the firmware.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PLANT_H
#define PLANT_H

struct plant_params {
	float m;     /* kg, whole robot (body, motor, wheel) */
	float l;     /* m, pivot to centre of mass */
	float i_b;   /* kg m^2, body about its centre of mass */
	float i_w;   /* kg m^2, wheel about its axle */
	float g;     /* m/s^2, 0 for the momentum test */
	float t_max; /* N m, motor torque limit */
};

/* Rough guesses until there is a mechanical build. */
extern const struct plant_params plant_guess;

struct plant_state {
	float theta;     /* rad, body tilt from upright */
	float theta_dot; /* rad/s */
	float omega;     /* rad/s, wheel speed relative to the body */
};

/* One RK4 step of dt with the motor torque held at torque. */
void plant_step(const struct plant_params *p, struct plant_state *s, float torque, float dt);

/* The body's angular acceleration at this state and torque. */
float plant_theta_ddot(const struct plant_params *p, const struct plant_state *s, float torque);

/* Angular momentum about the pivot: constant when g = 0. */
float plant_momentum(const struct plant_params *p, const struct plant_state *s);

/* Time constant of the fall from upright with no torque. */
float plant_fall_time(const struct plant_params *p);

#endif
