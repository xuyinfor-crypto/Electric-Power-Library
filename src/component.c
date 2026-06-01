#include "component.h"
#include "clarke.h"
#include "inverse_clarke.h"
#include "inverse_park.h"
#include "lpf.h"
#include "math.h"
#include "park.h"

/*
 * 模块：旋转相序输入映射
 * 设计目的：在 Clarke/Park 变换前，按序方向交换 b/c 相，建立对应旋转坐标系。
 * 设计原理：负序时 b/c 交换，正序时保持不变。
 * 输入：phase - 原始三相量；sequence - 目标序方向。
 * 输出：映射后的三相量。
 * 输出给谁用：通用旋转分量提取流程。
 */
static ABC sequence_input_phase(ABC phase, SEQUENCE sequence)
{
	ABC result = phase;

	if(sequence == SEQUENCE_NEGATIVE) {
		result.b = phase.c;
		result.c = phase.b;
	}

	return result;
}

/*
 * 模块：旋转相序输出映射
 * 设计目的：在 dq 到 abc 反变换后，按序方向交换 b/c 相，恢复原始相序。
 * 设计原理：负序时 b/c 交换，正序时保持不变，与输入映射对称。
 * 输入：phase - 反变换后的三相量；sequence - 目标序方向。
 * 输出：映射后的三相量。
 * 输出给谁用：通用旋转分量反变换流程。
 */
static ABC sequence_output_phase(ABC phase, SEQUENCE sequence)
{
	ABC result = phase;

	if(sequence == SEQUENCE_NEGATIVE) {
		result.b = phase.c;
		result.c = phase.b;
	}

	return result;
}

static int component_order(const COMPONENT_CONFIG *config)
{
	if(config == 0 || config->order <= 0) {
		return 1;
	}
	return config->order;
}

ABC rotate_dq_to_abc(DQ dq, const COMPONENT_CONFIG *config, real_t angle_pu)
{
	ALPHA_BETA alpha_beta;
	ABC phase;
	real_t rotate_angle;

	if(config == NULL) {
		rotate_angle = angle_pu;
	}
	else {
		rotate_angle = (real_t)component_order(config) * angle_pu;
		rotate_angle += config->phase_offset;
	}

	alpha_beta = inverse_park(dq, rotate_angle);
	phase = inverse_clarke(alpha_beta, 0.0f);

	if(config != NULL) {
		phase = sequence_output_phase(phase, config->sequence);
	}

	return phase;
}

void extract_component(COMPONENT *obj, const COMPONENT_INPUT *input)
{
	ABC source;
	ALPHA_BETA alpha_beta;
	DQ detected;
	DQ limited;
	real_t zero;
	real_t rotate_angle;
	real_t power_scale;

	if(obj == NULL || input == NULL) {
		return;
	}

	if(!input->enabled) {
		obj->detected_dq.d = 0.0f;
		obj->detected_dq.q = 0.0f;
		obj->limited_dq.d = 0.0f;
		obj->limited_dq.q = 0.0f;
		obj->phase_ref.a = 0.0f;
		obj->phase_ref.b = 0.0f;
		obj->phase_ref.c = 0.0f;
		obj->power_square = 0.0f;
		return;
	}

	source = sequence_input_phase(input->phase, obj->config.sequence);
	alpha_beta = clarke(source, &zero);
	rotate_angle = (real_t)component_order(&obj->config) * input->angle;
	detected = park(alpha_beta, rotate_angle);

	if(obj->d_lpf.alpha > 0.0f) {
		lowpass_filter(&obj->d_lpf, detected.d);
		obj->detected_dq.d = obj->d_lpf.average;
	}
	else {
		obj->detected_dq.d = detected.d;
	}

	if(obj->q_lpf.alpha > 0.0f) {
		lowpass_filter(&obj->q_lpf, detected.q);
		obj->detected_dq.q = obj->q_lpf.average;
	}
	else {
		obj->detected_dq.q = detected.q;
	}
	limited.d = obj->detected_dq.d;
	limited.q = obj->detected_dq.q;
	if(obj->config.dq_limit > 0.0f) {
		limited.d = sat(limited.d, obj->config.dq_limit, -obj->config.dq_limit);
		limited.q = sat(limited.q, obj->config.dq_limit, -obj->config.dq_limit);
	}

	obj->limited_dq.d = obj->config.gain * limited.d;
	obj->limited_dq.q = obj->config.gain * limited.q;
	obj->phase_ref = rotate_dq_to_abc(obj->limited_dq, &obj->config, input->angle);
	power_scale = obj->config.power_scale == 0.0f ? 1.0f : obj->config.power_scale;
	obj->power_square = power_scale * power_scale
		* (obj->detected_dq.d * obj->detected_dq.d + obj->detected_dq.q * obj->detected_dq.q);
}
