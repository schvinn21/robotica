#include "sensor_angulo_magnetico.h"

#define AS5600_DEFAULT_ADDR     0x36
#define AS5600_REG_RAW_ANGLE_H  0x0C

static esp_err_t as5600_read_u16(const as5600_t *dev, uint8_t reg, uint16_t *out)
{
    if (!dev || !out) return ESP_ERR_INVALID_ARG;

    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_write_read_device(
        dev->port,
        dev->addr,
        &reg, 1,
        data, 2,
        pdMS_TO_TICKS(50)
    );
    if (err != ESP_OK) return err;

    *out = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t as5600_init(as5600_t *dev, i2c_port_t port, uint8_t addr)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->port = port;
    dev->addr = (addr == 0) ? AS5600_DEFAULT_ADDR : addr;
    return ESP_OK;
}

esp_err_t as5600_read_raw_angle(const as5600_t *dev, uint16_t *raw_out)
{
    uint16_t v = 0;
    esp_err_t err = as5600_read_u16(dev, AS5600_REG_RAW_ANGLE_H, &v);
    if (err != ESP_OK) return err;

    *raw_out = v & 0x0FFF; // 12 bits
    return ESP_OK;
}

esp_err_t as5600_read_angle_deg(const as5600_t *dev, float *deg_out)
{
    if (!deg_out) return ESP_ERR_INVALID_ARG;

    uint16_t raw = 0;
    esp_err_t err = as5600_read_raw_angle(dev, &raw);
    if (err != ESP_OK) return err;

    *deg_out = (raw * 360.0f) / 4096.0f;
    return ESP_OK;
}
