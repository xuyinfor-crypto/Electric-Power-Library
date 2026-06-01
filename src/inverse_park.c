#include "inverse_park.h"
#include "math.h"

ALPHA_BETA inverse_park(DQ dq, real_t angle_pu)
{
	ALPHA_BETA alpha_beta;
	real_t cos_ang = cos_pu(angle_pu);
	real_t sin_ang = sin_pu(angle_pu);

	alpha_beta.alpha = dq.d * cos_ang - dq.q * sin_ang;
	alpha_beta.beta = dq.d * sin_ang + dq.q * cos_ang;

	return alpha_beta;
}
