#include "sequence.h"
#include "component.h"
#include "math.h"

void extract_sequence(SEQUENCE_EXTRACTOR *obj, const COMPONENT_INPUT *input)
{
	if(obj == NULL || input == NULL) {
		return;
	}

	extract_component(&obj->positive, input);
	extract_component(&obj->negative, input);

	obj->positive_ref = obj->positive.phase_ref;
	obj->negative_ref = obj->negative.phase_ref;
	obj->phase_ref.a = obj->positive_ref.a + obj->negative_ref.a;
	obj->phase_ref.b = obj->positive_ref.b + obj->negative_ref.b;
	obj->phase_ref.c = obj->positive_ref.c + obj->negative_ref.c;
	obj->power_square = obj->positive.power_square + obj->negative.power_square;
}

void run_sequence_control(SEQUENCE_CONTROLLER *obj, real_t angle)
{
	ABC phase;

	if(obj == NULL) {
		return;
	}

	phase = rotate_dq_to_abc(obj->dq_ref, &obj->config, angle);
	if(obj->output_limit > 0.0f) {
		phase.a = sat(phase.a, obj->output_limit, -obj->output_limit);
		phase.b = sat(phase.b, obj->output_limit, -obj->output_limit);
		phase.c = sat(phase.c, obj->output_limit, -obj->output_limit);
	}

	obj->phase_ref = phase;
}
