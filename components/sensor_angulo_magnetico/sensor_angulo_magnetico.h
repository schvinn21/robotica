#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t port;
    uint8_t addr; // padrão 0x36
} as5600_t;

esp_err_t as5600_init(as5600_t *dev, i2c_port_t port, uint8_t addr);
esp_err_t as5600_read_raw_angle(const as5600_t *dev, uint16_t *raw_out);
esp_err_t as5600_read_angle_deg(const as5600_t *dev, float *deg_out);

#ifdef __cplusplus
}
#endif
