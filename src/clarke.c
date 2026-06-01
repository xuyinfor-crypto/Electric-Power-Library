#include "clarke.h"
#include "math.h"

ALPHA_BETA clarke(ABC phase, real_t *zero)
{
	ALPHA_BETA result;

	result.alpha = (2.0f * phase.a - phase.b - phase.c) / 3.0f;
	result.beta = (phase.b - phase.c) / SQRT3;
	if(zero != NULL) {
		*zero = (phase.a + phase.b + phase.c) / 3.0f;
	}

	return result;
}
