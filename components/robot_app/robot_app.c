#include "robot_app.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#define AS5600_TOL_DEG   30.0f
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"

#include "robot_arm.h"
#include "sensor_angulo_magnetico.h"

static const char *TAG = "robot_app";

static robot_arm_t R;   // instância global do app
static as5600_t   AS;   // sensor global

// ===== I2C (AJUSTE PARA SEUS PINOS) =====
#define I2C_PORT     I2C_NUM_0
#define I2C_SDA_PIN  9
#define I2C_SCL_PIN  10
#define I2C_FREQ_HZ  400000

static void console_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
}

static void i2c_master_init_local(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // ideal: pull-up externo 4.7k
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

static void robot_init_all(void)
{
    // ====== PARAMS (ajuste igual ao seu projeto) ======
    R.steps_per_rev = 1600;

    // PINS (exemplo) - ajuste pro seu ESP32-S3
    R.j[0].m.step_pin = 4;  R.j[0].m.dir_pin = 15; R.j[0].m.pulse_us = 500;
    R.j[1].m.step_pin = 5;  R.j[1].m.dir_pin = 16; R.j[1].m.pulse_us = 500;
    R.j[2].m.step_pin = 6;  R.j[2].m.dir_pin = 17; R.j[2].m.pulse_us = 500;

    R.j[0].gear = 4.0f; R.j[1].gear = 4.0f; R.j[2].gear = 4.0f;

    R.kin.base_height = 0.10f;
    R.kin.L1 = 0.20f;
    R.kin.L2 = 0.20f;

    R.kin.j1_min = -150.0f * (float)M_PI/180.0f; R.kin.j1_max = +150.0f * (float)M_PI/180.0f;
    R.kin.j2_min =  -90.0f * (float)M_PI/180.0f; R.kin.j2_max =  +90.0f * (float)M_PI/180.0f;
    R.kin.j3_min =    0.0f * (float)M_PI/180.0f; R.kin.j3_max = +135.0f * (float)M_PI/180.0f;

    robot_arm_init(&R);

    // I2C + AS5600
    i2c_master_init_local();
    ESP_ERROR_CHECK(as5600_init(&AS, I2C_PORT, 0x36));
    robot_arm_attach_as5600(&R, &AS);

    ESP_LOGI(TAG, "AS5600 OK em 0x36 | I2C SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
}

static void task_robot_cli(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG,
        "CLI pronta.\n"
        "Comandos:\n"
        "  x y z            (ex: 0.20 0.00 0.15)\n"
        "  j <1|2|3> <deg>  (ex: j 1 90)\n"
        "  zero             (zera o AS5600 no ponto atual)\n"
    );

    char line[128];
    while (1) {
        if (fgets(line, sizeof(line), stdin)) {

            // remove \r\n
            line[strcspn(line, "\r\n")] = 0;

            if (strncmp(line, "zero", 4) == 0) {
                esp_err_t err = robot_arm_as5600_set_zero(&R);
                ESP_LOGI(TAG, "zero -> %s", esp_err_to_name(err));
                continue;
            }

            int jidx = 0;
            float deg = 0.0f;
            if (sscanf(line, "j %d %f", &jidx, &deg) == 2) {
                if (jidx >= 1 && jidx <= 3) {
                    float rad = deg * (float)M_PI / 180.0f;
                    robot_arm_move_joint_to(&R, jidx - 1, rad);
                } else {
                    ESP_LOGW(TAG, "Junta invalida. Use 1..3");
                }
                continue;
            }

            float x, y, z;
            if (sscanf(line, "%f %f %f", &x, &y, &z) == 3) {
                robot_arm_move_to_xyz(&R, x, y, z);
            } else {
                ESP_LOGW(TAG, "Formato invalido. Use: x y z  OU  j <1..3> <deg>  OU  zero");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void robot_app_start(void)
{
    console_uart_init();
    robot_init_all();

    xTaskCreate(task_robot_cli, "robot_cli", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tasks iniciadas.");
}
