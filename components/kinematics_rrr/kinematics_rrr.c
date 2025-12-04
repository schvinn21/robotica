#include "kinematics_rrr.h"
#include <math.h>

void kin_forward(const kin_rrr_params_t *p, float t1, float t2, float t3,
                 float *x, float *y, float *z)
{
    float t23 = t2 + t3;
    float r_planar = p->L1 * cosf(t2) + p->L2 * cosf(t23);
    float z_local  = p->L1 * sinf(t2) + p->L2 * sinf(t23);

    *x = r_planar * cosf(t1);
    *y = r_planar * sinf(t1);
    *z = p->base_height + z_local;
}

bool kin_inverse(const kin_rrr_params_t *p, float x, float y, float z,
                 float *t1, float *t2, float *t3)
{
    float r_xy = sqrtf(x*x + y*y);
    float z_eff = z - p->base_height;

    *t1 = atan2f(y, x);

    float r = sqrtf(r_xy*r_xy + z_eff*z_eff);

    float r_min = fabsf(p->L1 - p->L2);
    float r_max = p->L1 + p->L2;
    if (r < r_min || r > r_max) return false;

    float cos_t3 = (r*r - p->L1*p->L1 - p->L2*p->L2) / (2.0f * p->L1 * p->L2);
    if (cos_t3 > 1.0f) cos_t3 = 1.0f;
    if (cos_t3 < -1.0f) cos_t3 = -1.0f;

    *t3 = acosf(cos_t3); // “cotovelo pra cima” (igual seu .ino)

    float k1 = p->L1 + p->L2 * cosf(*t3);
    float k2 = p->L2 * sinf(*t3);

    *t2 = atan2f(z_eff, r_xy) - atan2f(k2, k1);

    if (*t1 < p->j1_min || *t1 > p->j1_max ||
        *t2 < p->j2_min || *t2 > p->j2_max ||
        *t3 < p->j3_min || *t3 > p->j3_max) {
        return false;
    }

    return true;
}
