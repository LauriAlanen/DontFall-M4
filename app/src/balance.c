/*
 * Balance controller. See balance.h.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "balance.h"

#include <math.h>

#include "balance_gains.h"

void balance_init(struct balance *b, float t_max)
{
	b->k[0] = BALANCE_K_THETA;
	b->k[1] = BALANCE_K_THETA_DOT;
	b->k[2] = BALANCE_K_OMEGA;
	b->t_max = t_max;
}

float balance_update(const struct balance *b, float theta, float theta_dot, float omega,
		     bool armed)
{
	float t;

	if (!armed) {
		return 0.0f;
	}
	t = -(b->k[0] * theta + b->k[1] * theta_dot + b->k[2] * omega);
	/* NaN in, NaN out: never pass that to a motor */
	if (!isfinite(t)) {
		return 0.0f;
	}
	return fminf(fmaxf(t, -b->t_max), b->t_max);
}
