#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct mpu6050_t mpu6050_t;

typedef struct { float x, y, z; } mpu6050_vec3_t;

esp_err_t mpu6050_create(i2c_master_bus_handle_t bus, uint8_t dev_addr, mpu6050_t **out);
esp_err_t mpu6050_read_accel(mpu6050_t *dev, mpu6050_vec3_t *accel_g);
esp_err_t mpu6050_read_gyro(mpu6050_t *dev, mpu6050_vec3_t *gyro_dps);
void      mpu6050_delete(mpu6050_t *dev);