/*
 * Simulated reaction wheel balancer: a body pivoting on its bottom edge
 * with a motor driven wheel at the top.
 *
 *     (I_b + m l^2) theta'' = m g l sin(theta) - torque
 *     I_w (theta'' + omega') = torque
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include "plant.h"

/* GUESSES, no build yet: a 20 cm tall 0.6 kg body with its centre of
 * mass at 12 cm, and a 150 g ring of 6 cm radius as the wheel. t_max is
 * a placeholder until it is checked against the GM4108H datasheet and
 * the SimpleFOCMini's current limit.
 */
const struct plant_params plant_guess = {
	.m = 0.6f,
	.l = 0.12f,
	.i_b = 0.6f * 0.2f * 0.2f / 12.0f,
	.i_w = 0.15f * 0.06f * 0.06f,
	.g = 9.80665f,
	.t_max = 0.2f,
};

struct rates {
	float theta, theta_dot, omega;
};

float plant_theta_ddot(const struct plant_params *p, const struct plant_state *s, float torque)
{
	float j = p->i_b + p->m * p->l * p->l;

	return (p->m * p->g * p->l * sinf(s->theta) - torque) / j;
}

static struct rates deriv(const struct plant_params *p, const struct plant_state *s, float torque)
{
	float theta_ddot = plant_theta_ddot(p, s, torque);

	return (struct rates){
		.theta = s->theta_dot,
		.theta_dot = theta_ddot,
		.omega = torque / p->i_w - theta_ddot,
	};
}

static struct plant_state add(const struct plant_state *s, const struct rates *r, float h)
{
	return (struct plant_state){
		.theta = s->theta + h * r->theta,
		.theta_dot = s->theta_dot + h * r->theta_dot,
		.omega = s->omega + h * r->omega,
	};
}

void plant_step(const struct plant_params *p, struct plant_state *s, float torque, float dt)
{
	struct plant_state tmp;
	struct rates k1, k2, k3, k4;

	k1 = deriv(p, s, torque);
	tmp = add(s, &k1, dt / 2);
	k2 = deriv(p, &tmp, torque);
	tmp = add(s, &k2, dt / 2);
	k3 = deriv(p, &tmp, torque);
	tmp = add(s, &k3, dt);
	k4 = deriv(p, &tmp, torque);

	s->theta += dt / 6 * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
	s->theta_dot += dt / 6 * (k1.theta_dot + 2 * k2.theta_dot + 2 * k3.theta_dot + k4.theta_dot);
	s->omega += dt / 6 * (k1.omega + 2 * k2.omega + 2 * k3.omega + k4.omega);
}

float plant_momentum(const struct plant_params *p, const struct plant_state *s)
{
	float j = p->i_b + p->m * p->l * p->l;

	return j * s->theta_dot + p->i_w * (s->theta_dot + s->omega);
}

float plant_fall_time(const struct plant_params *p)
{
	return sqrtf((p->i_b + p->m * p->l * p->l) / (p->m * p->g * p->l));
}
