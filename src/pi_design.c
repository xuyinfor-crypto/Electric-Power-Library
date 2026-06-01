#include <math.h>
#include "pi_design.h"
#include "math.h"

#define PI_DESIGN_DEG_TO_RAD (PI / 180.0f)
#define PI_DESIGN_MAX_ITER 64
#define PI_DESIGN_EPS 1.0e-6f
#define PI_DESIGN_MAX_WC 1.0e30f
#define PI_DESIGN_MAX_REAL 1.0e30f

static int pi_design_real_is_valid(real_t value)
{
	return value == value
		&& value < PI_DESIGN_MAX_REAL
		&& value > -PI_DESIGN_MAX_REAL;
}

static void pi_design_reset_output(PI_DESIGN *obj, PI_DESIGN_STATUS status)
{
	obj->inter.phase_margin_rad = 0.0f;
	obj->inter.target_phase_rad = 0.0f;
	obj->inter.crossover_rad_s = 0.0f;
	obj->inter.zero_rad_s = 0.0f;
	obj->output.kp = 0.0f;
	obj->output.ki_pid = 0.0f;
	obj->output.ki_cont = 0.0f;
	obj->output.crossover_rad_s = 0.0f;
	obj->output.zero_rad_s = 0.0f;
	obj->output.status = status;
}

static int pi_design_common_is_valid(const PI_DESIGN_CONFIG *config)
{
	return config != NULL
		&& pi_design_real_is_valid(config->delay_s)
		&& pi_design_real_is_valid(config->sample_period_s)
		&& pi_design_real_is_valid(config->zero_ratio_n)
		&& config->delay_s >= 0.0f
		&& config->sample_period_s > 0.0f
		&& config->zero_ratio_n > 0.0f;
}

static int pi_design_plant_is_valid(const PI_DESIGN_PLANT *plant)
{
	if(plant == NULL || !pi_design_real_is_valid(plant->a) || plant->a <= 0.0f) {
		return 0;
	}

	if(plant->type == PI_DESIGN_PLANT_INTEGRATOR) {
		return 1;
	}

	if(plant->type == PI_DESIGN_PLANT_FIRST_ORDER) {
		return pi_design_real_is_valid(plant->b)
			&& pi_design_real_is_valid(plant->c)
			&& plant->b > 0.0f
			&& plant->c > 0.0f;
	}

	return 0;
}

static real_t pi_design_first_order_phase(const PI_DESIGN_CONFIG *config, real_t wc)
{
	return (real_t)atan(config->plant.a * wc / config->plant.b) + config->delay_s * wc;
}

static PI_DESIGN_STATUS pi_design_solve_first_order_wc(const PI_DESIGN_CONFIG *config, real_t target_phase_rad, real_t *wc)
{
	real_t low;
	real_t high;
	real_t mid;
	real_t value;
	int i;

	if(config->delay_s <= 0.0f) {
		if(target_phase_rad <= 0.0f || target_phase_rad >= (PI * 0.5f)) {
			return PI_DESIGN_NO_CROSSOVER;
		}
		*wc = (config->plant.b / config->plant.a) * (real_t)tan(target_phase_rad);
		return *wc > 0.0f && pi_design_real_is_valid(*wc)
			? PI_DESIGN_OK
			: PI_DESIGN_NO_CROSSOVER;
	}

	low = 0.0f;
	high = 1.0f;
	while(pi_design_first_order_phase(config, high) < target_phase_rad) {
		high *= 2.0f;
		if(high > PI_DESIGN_MAX_WC) {
			return PI_DESIGN_NO_CROSSOVER;
		}
	}

	for(i = 0; i < PI_DESIGN_MAX_ITER; i++) {
		mid = (low + high) * 0.5f;
		value = pi_design_first_order_phase(config, mid);
		if(value < target_phase_rad) {
			low = mid;
		} else {
			high = mid;
		}
		if((high - low) <= PI_DESIGN_EPS * (1.0f + high)) {
			*wc = (low + high) * 0.5f;
			return *wc > 0.0f && pi_design_real_is_valid(*wc)
				? PI_DESIGN_OK
				: PI_DESIGN_NO_CROSSOVER;
		}
	}

	*wc = (low + high) * 0.5f;
	return PI_DESIGN_ITERATION_FAILED;
}

void pi_design_init(PI_DESIGN *obj, const PI_DESIGN_CONFIG *config)
{
	if(obj == NULL) {
		return;
	}

	obj->input.phase_margin_deg = 0.0f;
	obj->config.plant.type = PI_DESIGN_PLANT_INTEGRATOR;
	obj->config.plant.a = 0.0f;
	obj->config.plant.b = 0.0f;
	obj->config.plant.c = 0.0f;
	obj->config.delay_s = 0.0f;
	obj->config.sample_period_s = 0.0f;
	obj->config.zero_ratio_n = 0.0f;
	if(config != NULL) {
		obj->config = *config;
	}
	pi_design_reset_output(obj, PI_DESIGN_OK);
}

PI_DESIGN_STATUS pi_design_calc(PI_DESIGN *obj)
{
	const PI_DESIGN_CONFIG *config;
	real_t pm_rad;
	real_t wc;
	real_t wz;
	real_t n;
	real_t n_gain;
	real_t target_phase_rad;
	PI_DESIGN_STATUS solve_status;

	if(obj == NULL) {
		return PI_DESIGN_NULL_OBJECT;
	}

	config = &obj->config;
	if(!pi_design_common_is_valid(config) || !pi_design_plant_is_valid(&config->plant)) {
		pi_design_reset_output(obj, PI_DESIGN_BAD_CONFIG);
		return obj->output.status;
	}

	if(!pi_design_real_is_valid(obj->input.phase_margin_deg)
		|| obj->input.phase_margin_deg <= 0.0f
		|| obj->input.phase_margin_deg >= 180.0f) {
		pi_design_reset_output(obj, PI_DESIGN_BAD_PHASE_MARGIN);
		return obj->output.status;
	}

	pm_rad = obj->input.phase_margin_deg * PI_DESIGN_DEG_TO_RAD;
	n = config->zero_ratio_n;
	n_gain = (real_t)sqrt(1.0f + 1.0f / (n * n));

	if(config->plant.type == PI_DESIGN_PLANT_INTEGRATOR) {
		target_phase_rad = (real_t)atan(n) - pm_rad;
		if(config->delay_s <= 0.0f || target_phase_rad <= 0.0f) {
			pi_design_reset_output(obj, PI_DESIGN_NO_CROSSOVER);
			return obj->output.status;
		}
		wc = target_phase_rad / config->delay_s;
		wz = wc / n;
		obj->output.kp = config->plant.a * wc / n_gain;
	} else {
		target_phase_rad = (PI * 0.5f) + (real_t)atan(n) - pm_rad;
		if(target_phase_rad <= 0.0f) {
			pi_design_reset_output(obj, PI_DESIGN_NO_CROSSOVER);
			return obj->output.status;
		}
		solve_status = pi_design_solve_first_order_wc(config, target_phase_rad, &wc);
		if(solve_status != PI_DESIGN_OK) {
			pi_design_reset_output(obj, solve_status);
			return obj->output.status;
		}
		wz = wc / n;
		obj->output.kp = (real_t)sqrt(config->plant.a * config->plant.a * wc * wc + config->plant.b * config->plant.b) / (config->plant.c * n_gain);
	}

	obj->output.ki_cont = obj->output.kp * wz;
	obj->output.ki_pid = wz * config->sample_period_s;
	if(!pi_design_real_is_valid(obj->output.kp)
		|| !pi_design_real_is_valid(obj->output.ki_pid)
		|| !pi_design_real_is_valid(obj->output.ki_cont)) {
		pi_design_reset_output(obj, PI_DESIGN_ITERATION_FAILED);
		return obj->output.status;
	}

	obj->inter.phase_margin_rad = pm_rad;
	obj->inter.target_phase_rad = target_phase_rad;
	obj->inter.crossover_rad_s = wc;
	obj->inter.zero_rad_s = wz;
	obj->output.crossover_rad_s = wc;
	obj->output.zero_rad_s = wz;
	obj->output.status = PI_DESIGN_OK;
	return obj->output.status;
}
