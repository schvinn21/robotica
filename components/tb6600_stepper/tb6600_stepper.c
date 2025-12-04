#include "tb6600_stepper.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us

void tb6600_init(tb6600_stepper_t *m)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << m->step_pin) | (1ULL << m->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(m->step_pin, 0);
    gpio_set_level(m->dir_pin, 0);
}

void tb6600_set_dir(tb6600_stepper_t *m, int dir)
{
    gpio_set_level(m->dir_pin, dir ? 1 : 0);
}

void tb6600_step_us(tb6600_stepper_t *m, uint32_t us)
{
    gpio_set_level(m->step_pin, 1);
    esp_rom_delay_us(us);
    gpio_set_level(m->step_pin, 0);
    esp_rom_delay_us(us);
}

void tb6600_step(tb6600_stepper_t *m)
{
    tb6600_step_us(m, m->pulse_us);
}

void tb6600_step_n(tb6600_stepper_t *m, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i) tb6600_step(m);
}
