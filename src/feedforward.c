#include "feedforward.h"
#include "inverse_clarke.h"
#include "inverse_park.h"

void calc_feedforward(CONTROL_ALGO *obj, real_t angle_pu)
{
	DQ dq;
	ALPHA_BETA alpha_beta;
	ABC phase;
	real_t max_value;
	real_t min_value;
	real_t middle;

	dq.d = obj->config.feedforward;
	dq.q = 0.0f;
	alpha_beta = inverse_park(dq, angle_pu);
	phase = inverse_clarke(alpha_beta, 0.0f);

	max_value = phase.a > phase.b ? phase.a : phase.b;
	max_value = max_value > phase.c ? max_value : phase.c;
	min_value = phase.a < phase.b ? phase.a : phase.b;
	min_value = min_value < phase.c ? min_value : phase.c;
	middle = 0.5f * (max_value + min_value);

	obj->inter.feedforward.a = phase.a - middle;
	obj->inter.feedforward.b = phase.b - middle;
	obj->inter.feedforward.c = phase.c - middle;
}
