#pragma once
#include <stdint.h>
#include "driver/gpio.h"

typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    uint32_t pulse_us;   // largura base do pulso (us)
} tb6600_stepper_t;

void tb6600_init(tb6600_stepper_t *m);
void tb6600_set_dir(tb6600_stepper_t *m, int dir);     // dir: 1 ou 0
void tb6600_step(tb6600_stepper_t *m);                 // 1 passo usando m->pulse_us
void tb6600_step_us(tb6600_stepper_t *m, uint32_t us); // 1 passo usando delay "us"
void tb6600_step_n(tb6600_stepper_t *m, uint32_t n);   // n passos usando m->pulse_us
