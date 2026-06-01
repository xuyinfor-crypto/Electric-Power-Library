#include "park.h"
#include "math.h"

DQ park(ALPHA_BETA alpha_beta, real_t angle_pu)
{
	DQ dq;
	real_t cos_ang = cos_pu(angle_pu);
	real_t sin_ang = sin_pu(angle_pu);

	dq.d = alpha_beta.alpha * cos_ang + alpha_beta.beta * sin_ang;
	dq.q = -alpha_beta.alpha * sin_ang + alpha_beta.beta * cos_ang;

	return dq;
}
