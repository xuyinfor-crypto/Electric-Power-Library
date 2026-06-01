#include "open_loop.h"
#include "inverse_clarke.h"
#include "inverse_park.h"
#include "math.h"

void run_open_loop(CONTROL_ALGO *obj)
{
	ALPHA_BETA alpha_beta;
	ABC phase;
	real_t angle;

	if(obj == NULL) {
		return;
	}

	if(obj->cmd_word.bits.run_open_loop) {
		obj->inter.ramp_angle = wrap_unit(
			obj->inter.ramp_angle + obj->config.base_frequency_hz * obj->config.sample_period_s);
		angle = obj->inter.ramp_angle;
	}
	else {
		angle = obj->input.angle;
	}

	alpha_beta = inverse_park(obj->config.open_loop_dq, angle);
	phase = inverse_clarke(alpha_beta, 0.0f);

	obj->output.phase.a = sat(phase.a, obj->config.duty_limit, -obj->config.duty_limit);
	obj->output.phase.b = sat(phase.b, obj->config.duty_limit, -obj->config.duty_limit);
	obj->output.phase.c = sat(phase.c, obj->config.duty_limit, -obj->config.duty_limit);
	obj->output.alpha_beta = alpha_beta;
	obj->output.pll_angle = angle;
	obj->st_word.bits.open_loop = 1;
}
