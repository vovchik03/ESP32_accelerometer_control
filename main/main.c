#include <stdio.h>

void app_main(void)
{
    i2c_master_bus_handle_t bus; 
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    mpu6050_t *imu;
    ESP_ERROR_CHECK(mpu6050_create(bus, BOARD_MPU6050_ADDR, &imu));

    servo_t *servo_x, *servo_y;
    servo_config_t cfg_x = { .gpio = BOARD_SERVO_X_GPIO, .ledc_channel = 0, /* ... */ };
    servo_config_t cfg_y = { .gpio = BOARD_SERVO_Y_GPIO, .ledc_channel = 1, /* ... */ };
    ESP_ERROR_CHECK(servo_create(&cfg_x, &servo_x));
    ESP_ERROR_CHECK(servo_create(&cfg_y, &servo_y));

    
    tilt_angles_t state = {0};
    while (1) {
        mpu6050_vec3_t a, g;
        mpu6050_read_accel(imu, &a);
        mpu6050_read_gyro(imu, &g);

        state = tilt_filter_update(state, /* ... */, 0.01f, 0.98f);

        servo_set_angle(servo_x, tilt_map_to_servo(state.roll_deg,  0, 180));
        servo_set_angle(servo_y, tilt_map_to_servo(state.pitch_deg, 0, 180));

        vTaskDelay(pdMS_TO_TICKS(10));   // 100 Hz
    }
}
