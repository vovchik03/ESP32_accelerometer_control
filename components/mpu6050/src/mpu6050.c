#include <stdlib.h>
#include <string.h>

#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

#define MPU6050_WHO_AM_I_VALUE    0x68
#define MPU6050_TIMEOUT_MS        100

static const char *TAG = "mpu6050";

/* LSB на одиницю для кожного діапазону */
static const float s_accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float s_gyro_lsb_per_dps[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };

struct mpu6050_t {
    i2c_master_dev_handle_t dev;
    float accel_scale;   /* сирий LSB * accel_scale = g   */
    float gyro_scale;    /* сирий LSB * gyro_scale  = °/с */
};

static esp_err_t mpu_write_reg(mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(dev->dev, buf, sizeof(buf), MPU6050_TIMEOUT_MS);
}

static esp_err_t mpu_read_regs(mpu6050_t *dev, uint8_t reg,
                               uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev->dev, &reg, 1,
                                       data, len, MPU6050_TIMEOUT_MS);
}

esp_err_t mpu6050_create(i2c_master_bus_handle_t bus,
                         const mpu6050_config_t *cfg,
                         mpu6050_t **out_dev)
{
    if (bus == NULL || cfg == NULL || out_dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_t *dev = calloc(1, sizeof(mpu6050_t));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->dev_addr,
        .scl_speed_hz    = cfg->scl_speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device failed: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }

    /* Перевірка, що на шині дійсно MPU-6050 */
    uint8_t who = 0;
    err = mpu_read_regs(dev, MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no response at 0x%02X: %s", cfg->dev_addr,
                 esp_err_to_name(err));
        goto fail;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I: 0x%02X", who);
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    /* Скидання і вихід зі сну */
    err = mpu_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x80);   /* DEVICE_RESET */
    if (err != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(100));

    /* CLKSEL = 1: тактування від гіроскопа по X, стабільніше за внутрішній RC */
    err = mpu_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) goto fail;

    err = mpu_write_reg(dev, MPU6050_REG_CONFIG, cfg->dlpf_cfg & 0x07);
    if (err != ESP_OK) goto fail;

    err = mpu_write_reg(dev, MPU6050_REG_SMPLRT_DIV, 9);      /* 1 кГц / 10 = 100 Гц */
    if (err != ESP_OK) goto fail;

    err = mpu_write_reg(dev, MPU6050_REG_GYRO_CONFIG,
                        (uint8_t)(cfg->gyro_fs << 3));
    if (err != ESP_OK) goto fail;

    err = mpu_write_reg(dev, MPU6050_REG_ACCEL_CONFIG,
                        (uint8_t)(cfg->accel_fs << 3));
    if (err != ESP_OK) goto fail;

    dev->accel_scale = 1.0f / s_accel_lsb_per_g[cfg->accel_fs];
    dev->gyro_scale  = 1.0f / s_gyro_lsb_per_dps[cfg->gyro_fs];

    ESP_LOGI(TAG, "MPU-6050 ready at 0x%02X", cfg->dev_addr);
    *out_dev = dev;
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(dev->dev);
    free(dev);
    return err;
}

esp_err_t mpu6050_read(mpu6050_t *dev,
                       mpu6050_vec3_t *accel_g,
                       mpu6050_vec3_t *gyro_dps)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 14 байтів: accel XYZ, temp, gyro XYZ — усе одним читанням */
    uint8_t raw[14];
    esp_err_t err = mpu_read_regs(dev, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    if (accel_g != NULL) {
        accel_g->x = (int16_t)((raw[0]  << 8) | raw[1])  * dev->accel_scale;
        accel_g->y = (int16_t)((raw[2]  << 8) | raw[3])  * dev->accel_scale;
        accel_g->z = (int16_t)((raw[4]  << 8) | raw[5])  * dev->accel_scale;
    }
    if (gyro_dps != NULL) {
        gyro_dps->x = (int16_t)((raw[8]  << 8) | raw[9])  * dev->gyro_scale;
        gyro_dps->y = (int16_t)((raw[10] << 8) | raw[11]) * dev->gyro_scale;
        gyro_dps->z = (int16_t)((raw[12] << 8) | raw[13]) * dev->gyro_scale;
    }

    return ESP_OK;
}

void mpu6050_delete(mpu6050_t *dev)
{
    if (dev == NULL) {
        return;
    }
    i2c_master_bus_rm_device(dev->dev);
    free(dev);
}