#include "pid.h"
#include "math.h"

void pid_calc(PID *obj)
{
	if(obj == NULL) {
		return;
	}

	obj->e = obj->ref - obj->fdb;
	obj->up = obj->kp * obj->e;
	obj->ui = obj->ui + obj->ki * obj->up + obj->kc * obj->saterr;
	obj->ud = obj->kd * (obj->up - obj->up1);
	obj->uprsat = obj->up + obj->ui + obj->ud;
	obj->out = sat(obj->uprsat, obj->out_max, obj->out_min);
	obj->saterr = obj->out - obj->uprsat;
	obj->up1 = obj->up;
}
