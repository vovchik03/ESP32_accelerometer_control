#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "board_config.h"
#include "mpu6050.h"
#include "servo.h"
#include "tilt_control.h"

static const char *TAG = "app";

#define TILT_INPUT_MIN_DEG   (-60.0f)
#define TILT_INPUT_MAX_DEG   ( 60.0f)
#define FILTER_ALPHA          0.98f

static i2c_master_bus_handle_t init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = BOARD_I2C_PORT,
        .sda_io_num                   = BOARD_I2C_SDA_GPIO,
        .scl_io_num                   = BOARD_I2C_SCL_GPIO,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    return bus;
}

void app_main(void)
{
    /* --- Ресурси створює main і роздає драйверам --- */
    i2c_master_bus_handle_t bus = init_i2c_bus();

    mpu6050_config_t imu_cfg = MPU6050_CONFIG_DEFAULT(BOARD_MPU6050_ADDR);
    imu_cfg.scl_speed_hz = BOARD_I2C_FREQ_HZ;

    mpu6050_t *imu = NULL;
    ESP_ERROR_CHECK(mpu6050_create(bus, &imu_cfg, &imu));

    servo_config_t cfg_x = SERVO_CONFIG_SG90(BOARD_SERVO_X_GPIO, 0);
    servo_config_t cfg_y = SERVO_CONFIG_SG90(BOARD_SERVO_Y_GPIO, 1);

    servo_t *servo_x = NULL;
    servo_t *servo_y = NULL;
    ESP_ERROR_CHECK(servo_create(&cfg_x, &servo_x));
    ESP_ERROR_CHECK(servo_create(&cfg_y, &servo_y));

    ESP_LOGI(TAG, "init done, entering control loop");

    // controll unit
    const float dt_s = BOARD_CONTROL_PERIOD_MS / 1000.0f;
    tilt_angles_t state = { 0.0f, 0.0f };

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        mpu6050_vec3_t accel, gyro;

        if (mpu6050_read(imu, &accel, &gyro) == ESP_OK) {
            state = tilt_filter_update(state, accel, gyro, dt_s, FILTER_ALPHA);

            float angle_x = tilt_map(state.roll_deg,
                                     TILT_INPUT_MIN_DEG, TILT_INPUT_MAX_DEG,
                                     cfg_x.min_angle_deg, cfg_x.max_angle_deg);
            float angle_y = tilt_map(state.pitch_deg,
                                     TILT_INPUT_MIN_DEG, TILT_INPUT_MAX_DEG,
                                     cfg_y.min_angle_deg, cfg_y.max_angle_deg);

            servo_set_angle(servo_x, angle_x);
            servo_set_angle(servo_y, angle_y);
        } else {
            ESP_LOGW(TAG, "IMU read failed");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BOARD_CONTROL_PERIOD_MS));
    }
}