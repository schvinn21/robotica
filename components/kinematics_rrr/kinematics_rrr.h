#pragma once
#include <stdbool.h>

typedef struct {
    float base_height;   // BASE_HEIGHT
    float L1;            // ombro-cotovelo
    float L2;            // cotovelo-ferramenta
    float j1_min, j1_max;
    float j2_min, j2_max;
    float j3_min, j3_max;
} kin_rrr_params_t;

void kin_forward(const kin_rrr_params_t *p, float t1, float t2, float t3,
                 float *x, float *y, float *z);

bool kin_inverse(const kin_rrr_params_t *p, float x, float y, float z,
                 float *t1, float *t2, float *t3);
