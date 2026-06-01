#include "dq_control.h"
#include "math.h"
#include "pid.h"

void run_dq_control(DQ_CONTROLLER *obj, const DQ_CONTROL_INPUT *input)
{
	if(obj == NULL || input == NULL) {
		return;
	}

	obj->d_pid.ref = input->ref.d;
	obj->d_pid.fdb = input->fdb.d;
	obj->q_pid.ref = input->ref.q;
	obj->q_pid.fdb = input->fdb.q;

	pid_calc(&obj->d_pid);
	pid_calc(&obj->q_pid);

	obj->pi_out.d = obj->d_pid.out;
	obj->pi_out.q = obj->q_pid.out;
	obj->dq_out.d = obj->pi_out.d + input->feedforward.d;
	obj->dq_out.q = obj->pi_out.q + input->feedforward.q;
	if(obj->output_limit > 0.0f) {
		obj->dq_out.d = sat(obj->dq_out.d, obj->output_limit, -obj->output_limit);
		obj->dq_out.q = sat(obj->dq_out.q, obj->output_limit, -obj->output_limit);
	}
}
