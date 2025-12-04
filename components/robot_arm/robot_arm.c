#include "robot_arm.h"

#include <math.h>
#include <stdlib.h>

#include "esp_log.h"

static const char *TAG = "robot_arm";

/**
 * Ajuste fino:
 * - START_US maior = começo/fim mais lento e suave
 * - MIN_US menor  = maior velocidade no meio (se ficar agressivo, aumente)
 */
#define RAMP_START_US   5000
#define RAMP_MIN_US     1800
#define RAMP_FRAC_DIV   5     // rampa = n/5 (20% acel + 20% desac)

// ===== AS5600 / malha fechada =====
#define AS5600_TOL_DEG                 5.0f
#define AS5600_CLOSED_LOOP_ENABLE      1
#define AS5600_MAX_CORR_ITERS          30
#define AS5600_MAX_STEPS_PER_CORR      2200   // limite por iteração (segurança)
#define AS5600_CORR_DELAY_US           1500  // velocidade na correção (ajuste fino)
#define AS5600_LOG_EVERY_STEPS         50    // log durante movimento (não floodar)

static float wrap360(float a)
{
    while (a < 0.0f)   a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// erro no menor caminho: -180..+180
static float shortest_error_deg(float target, float current)
{
    float t = wrap360(target);
    float c = wrap360(current);
    float e = t - c;
    if (e > 180.0f)  e -= 360.0f;
    if (e < -180.0f) e += 360.0f;
    return e;
}

static float rad_per_step_joint(long steps_per_rev_motor, float gear)
{
    // motor: 2*pi por volta; junta: motor/gear
    // gear = (motor/junta), ex: 4.0 => junta anda 1/4 do motor
    if (steps_per_rev_motor <= 0) steps_per_rev_motor = 1;
    if (gear <= 0.0f) gear = 1.0f;
    return (2.0f * (float)M_PI) / ((float)steps_per_rev_motor * gear);
}

static uint32_t ramp_delay_us(uint32_t i, uint32_t n, uint32_t start_us, uint32_t min_us)
{
    if (n < 2) return start_us;

    uint32_t ramp = n / RAMP_FRAC_DIV;
    if (ramp < 1) ramp = 1;

    if (i < ramp) {
        float t = (float)i / (float)ramp;
        return (uint32_t)(start_us - t * (start_us - min_us));
    }
    if (i > (n - ramp)) {
        float t = (float)(n - i) / (float)ramp;
        return (uint32_t)(start_us - t * (start_us - min_us));
    }
    return min_us;
}

// ---------- AS5600 API ----------
void robot_arm_attach_as5600(robot_arm_t *r, const as5600_t *dev)
{
    r->as5600 = dev;
    r->as5600_zero_valid = false;
    r->as5600_zero_deg = 0.0f;

    // defaults (ajuste conforme sua mecânica)
    r->as5600_joint_idx   = 0;     // assume junta 1
    r->as5600_joint_ratio = 1.0f / 4.0f;  // assume sensor mede a junta diretamenteR.as5600_joint_ratio = 

    r->as5600_invert      = false;
}

esp_err_t robot_arm_as5600_set_zero(robot_arm_t *r)
{
    if (!r || !r->as5600) return ESP_ERR_INVALID_STATE;

    float deg = 0.0f;
    esp_err_t err = as5600_read_angle_deg(r->as5600, &deg);
    if (err != ESP_OK) return err;

    r->as5600_zero_deg = wrap360(deg);
    r->as5600_zero_valid = true;

    ESP_LOGI(TAG, "AS5600 zero setado em %.2f° (raw sensor)", (double)r->as5600_zero_deg);
    return ESP_OK;
}

// lê o AS5600 e devolve em graus "na junta" (aplicando zero/ratio/invert)
esp_err_t robot_arm_as5600_read_deg(const robot_arm_t *r, float *deg_out)
{
    if (!r || !deg_out || !r->as5600) return ESP_ERR_INVALID_STATE;

    float deg = 0.0f;
    esp_err_t err = as5600_read_angle_deg(r->as5600, &deg);
    if (err != ESP_OK) return err;

    deg = wrap360(deg);

    // offset do "zero"
    if (r->as5600_zero_valid) {
        deg = wrap360(deg - r->as5600_zero_deg);
    }

    // inverter sentido
    if (r->as5600_invert) {
        deg = wrap360(-deg);
    }

    // converter leitura do sensor para graus da JUNTA
    // ex: sensor no motor antes da redução: ratio = 1/gear
    deg = wrap360(deg * r->as5600_joint_ratio);

    *deg_out = deg;
    return ESP_OK;
}

// ---------- Core ----------
void robot_arm_init(robot_arm_t *r)
{
    r->rad_per_step_j[0] = rad_per_step_joint(r->steps_per_rev, r->j[0].gear);
    r->rad_per_step_j[1] = rad_per_step_joint(r->steps_per_rev, r->j[1].gear);
    r->rad_per_step_j[2] = rad_per_step_joint(r->steps_per_rev, r->j[2].gear);

    for (int i = 0; i < 3; ++i) {
        tb6600_init(&r->j[i].m);
        r->j[i].angle_rad = 0.0f;
    }

    // defaults AS5600 (desanexado)
    r->as5600 = NULL;
    r->as5600_zero_valid = false;
    r->as5600_zero_deg = 0.0f;
    r->as5600_joint_idx = 0;
    r->as5600_joint_ratio = 1.0f;
    r->as5600_invert = false;

    ESP_LOGI(TAG, "Init OK. rad/step (J1,J2,J3) = %.9f %.9f %.9f",
             r->rad_per_step_j[0], r->rad_per_step_j[1], r->rad_per_step_j[2]);
}

void robot_arm_move_joint_to(robot_arm_t *r, int idx, float target_rad)
{
    if (!r || idx < 0 || idx > 2) return;

    joint_t *J = &r->j[idx];

    float delta = target_rad - J->angle_rad;
    long steps = lroundf(delta / r->rad_per_step_j[idx]);
    if (steps == 0) return;

    int dir = (steps >= 0) ? 1 : 0;
    uint32_t steps_to_do = (uint32_t)labs(steps);

    tb6600_set_dir(&J->m, dir);

    float target_deg_joint = wrap360(target_rad * 180.0f / (float)M_PI);
    ESP_LOGI(TAG, "J%d: %.1f° -> %.1f° (%lu passos)",
             idx + 1,
             J->angle_rad * 180.0f / (float)M_PI,
             target_rad * 180.0f / (float)M_PI,
             (unsigned long)steps_to_do);

    const float dtheta = (dir ? r->rad_per_step_j[idx] : -r->rad_per_step_j[idx]);

    // ===== Movimento open-loop =====
    for (uint32_t s = 0; s < steps_to_do; ++s) {
        uint32_t us = ramp_delay_us(s, steps_to_do, RAMP_START_US, RAMP_MIN_US);

        tb6600_step_us(&J->m, us);
        J->angle_rad += dtheta;

        if (s == (steps_to_do - 1) || (s % 50 == 0)) {
            float x, y, z;
            kin_forward(&r->kin, r->j[0].angle_rad, r->j[1].angle_rad, r->j[2].angle_rad, &x, &y, &z);
            ESP_LOGI(TAG, "Pose aprox: x=%.3f y=%.3f z=%.3f | delay=%luus",
                     x, y, z, (unsigned long)us);
        }

        // log do AS5600 durante o movimento (se for a junta medida)
        if (r->as5600 && (idx == r->as5600_joint_idx) &&
            (s == (steps_to_do - 1) || (s % AS5600_LOG_EVERY_STEPS == 0)))
        {
            float measured = 0.0f;
            if (robot_arm_as5600_read_deg(r, &measured) == ESP_OK) {
                float err_deg = shortest_error_deg(target_deg_joint, measured);
                ESP_LOGI(TAG, "AS5600 | alvo=%.2f°  atual=%.2f°  erro=%.2f°",
                         (double)target_deg_joint, (double)measured, (double)err_deg);
            } else {
                ESP_LOGW(TAG, "AS5600: falha na leitura");
            }
        }
    }

#if AS5600_CLOSED_LOOP_ENABLE
    // ===== Correção em malha fechada (somente se AS5600 mede esta junta) =====
    if (r->as5600 && (idx == r->as5600_joint_idx)) {

        // quantos graus a junta “anda” por step (estimativa aberta)
        const float deg_per_step = (r->rad_per_step_j[idx] * 180.0f / (float)M_PI);
        if (fabsf(deg_per_step) < 1e-6f) {
            ESP_LOGW(TAG, "deg_per_step muito pequeno; pulando correção");
            return;
        }

        for (int it = 0; it < AS5600_MAX_CORR_ITERS; ++it) {
            float measured = 0.0f;
            if (robot_arm_as5600_read_deg(r, &measured) != ESP_OK) {
                ESP_LOGW(TAG, "AS5600: falha na leitura na correção");
                break;
            }

            float err_deg = shortest_error_deg(target_deg_joint, measured);
            bool ok = (fabsf(err_deg) <= AS5600_TOL_DEG);

            ESP_LOGI(TAG, "AS5600 CORR[%d] | alvo=%.2f° atual=%.2f° erro=%.2f° [%s ±%.0f°]",
                     it, (double)target_deg_joint, (double)measured, (double)err_deg,
                     ok ? "OK" : "FORA", (double)AS5600_TOL_DEG);

            if (ok) break;

            // passos necessários para reduzir erro (aprox)
            long steps_corr = lroundf(err_deg / deg_per_step);

            // limita correção por iteração
            if (steps_corr >  AS5600_MAX_STEPS_PER_CORR) steps_corr =  AS5600_MAX_STEPS_PER_CORR;
            if (steps_corr < -AS5600_MAX_STEPS_PER_CORR) steps_corr = -AS5600_MAX_STEPS_PER_CORR;

            if (steps_corr == 0) break;

            int dir_corr = (steps_corr >= 0) ? 1 : 0;
            uint32_t n = (uint32_t)labs(steps_corr);

            tb6600_set_dir(&J->m, dir_corr);

            const float dtheta_corr = (dir_corr ? r->rad_per_step_j[idx] : -r->rad_per_step_j[idx]);

            for (uint32_t s2 = 0; s2 < n; ++s2) {
                tb6600_step_us(&J->m, AS5600_CORR_DELAY_US);
                J->angle_rad += dtheta_corr;
            }
        }
    }
#endif
}

bool robot_arm_move_to_xyz(robot_arm_t *r, float x, float y, float z)
{
    if (!r) return false;

    float t1, t2, t3;
    if (!kin_inverse(&r->kin, x, y, z, &t1, &t2, &t3)) {
        ESP_LOGW(TAG, "IK falhou p/ x=%.3f y=%.3f z=%.3f", x, y, z);
        return false;
    }

    robot_arm_move_joint_to(r, 0, t1);
    robot_arm_move_joint_to(r, 1, t2);
    robot_arm_move_joint_to(r, 2, t3);
    return true;
}
