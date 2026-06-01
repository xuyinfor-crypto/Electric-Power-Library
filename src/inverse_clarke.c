#include "inverse_clarke.h"
#include "math.h"

ABC inverse_clarke(ALPHA_BETA alpha_beta, real_t zero)
{
	ABC phase;

	phase.a = alpha_beta.alpha + zero;
	phase.b = -0.5f * alpha_beta.alpha + 0.5f * SQRT3 * alpha_beta.beta + zero;
	phase.c = -0.5f * alpha_beta.alpha - 0.5f * SQRT3 * alpha_beta.beta + zero;

	return phase;
}
