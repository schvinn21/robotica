#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "tb6600_stepper.h"
#include "kinematics_rrr.h"
#include "sensor_angulo_magnetico.h"   // as5600_t, as5600_read_angle_deg

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    tb6600_stepper_t m;
    float angle_rad;   // estimativa open-loop (por passos)
    float min_rad;
    float max_rad;
    float gear;        // redução (ex: 4.0 -> motor gira 4x para 1x na junta)
} joint_t;

typedef struct {
    kin_rrr_params_t kin;

    long  steps_per_rev;       // passos por volta do MOTOR (microstep já incluso)
    float rad_per_step_j[3];   // rad/step na JUNTA (considerando gear)

    joint_t j[3];

    // ===== AS5600 (opcional) =====
    const as5600_t *as5600;      // ponteiro do sensor inicializado externamente
    float as5600_zero_deg;       // offset (definido com comando "zero")
    bool  as5600_zero_valid;

    int   as5600_joint_idx;      // qual junta o sensor mede (0..2)
    float as5600_joint_ratio;    // converte leitura do sensor para graus da junta
                                // 1.0 se o sensor mede direto a junta
                                // se sensor mede o motor antes da redução: use 1/gear
    bool  as5600_invert;         // inverte sentido do sensor (se estiver ao contrário)
} robot_arm_t;

void     robot_arm_init(robot_arm_t *r);
void     robot_arm_move_joint_to(robot_arm_t *r, int idx, float target_rad);
bool     robot_arm_move_to_xyz(robot_arm_t *r, float x, float y, float z);

void     robot_arm_attach_as5600(robot_arm_t *r, const as5600_t *dev);
esp_err_t robot_arm_as5600_set_zero(robot_arm_t *r);
esp_err_t robot_arm_as5600_read_deg(const robot_arm_t *r, float *deg_out);

#ifdef __cplusplus
}
#endif
