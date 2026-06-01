#include "mean.h"

void calc_mean(MEAN *obj)
{
	if(obj == NULL || obj->sample_count <= 0) {
		return;
	}

	++obj->cnt;
	obj->cnt %= obj->sample_count;
	obj->vacc += obj->v;
	obj->iacc += obj->i;

	if(obj->cnt == 0) {
		obj->vo = obj->vacc / (real_t)obj->sample_count;
		obj->io = obj->iacc / (real_t)obj->sample_count;
		obj->vacc = 0.0f;
		obj->iacc = 0.0f;
	}
}
