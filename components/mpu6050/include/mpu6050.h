#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mpu6050_t mpu6050_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_vec3_t;

typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,
    MPU6050_ACCEL_FS_4G  = 1,
    MPU6050_ACCEL_FS_8G  = 2,
    MPU6050_ACCEL_FS_16G = 3,
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,
    MPU6050_GYRO_FS_500DPS  = 1,
    MPU6050_GYRO_FS_1000DPS = 2,
    MPU6050_GYRO_FS_2000DPS = 3,
} mpu6050_gyro_fs_t;

typedef struct {
    uint8_t            dev_addr;      /* 0x68 або 0x69 */
    uint32_t           scl_speed_hz;  /* 400000 */
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    uint8_t            dlpf_cfg;      /* 0..6, 3 = ~44 Гц, добре проти вібрацій */
} mpu6050_config_t;

#define MPU6050_CONFIG_DEFAULT(addr) (mpu6050_config_t){ \
    .dev_addr     = (addr),                              \
    .scl_speed_hz = 400000,                              \
    .accel_fs     = MPU6050_ACCEL_FS_2G,                 \
    .gyro_fs      = MPU6050_GYRO_FS_250DPS,              \
    .dlpf_cfg     = 3,                                   \
}

esp_err_t mpu6050_create(i2c_master_bus_handle_t bus,
                         const mpu6050_config_t *cfg,
                         mpu6050_t **out_dev);


esp_err_t mpu6050_read(mpu6050_t *dev,
                       mpu6050_vec3_t *accel_g,
                       mpu6050_vec3_t *gyro_dps);

void mpu6050_delete(mpu6050_t *dev);

#ifdef __cplusplus
}
#endif