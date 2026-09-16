#pragma once

/* I2C — MPU-6050 */
#define BOARD_I2C_PORT          I2C_NUM_0
#define BOARD_I2C_SDA_GPIO      21
#define BOARD_I2C_SCL_GPIO      22
#define BOARD_I2C_FREQ_HZ       400000
#define BOARD_MPU6050_ADDR      0x68

/* Серво (LEDC, PWM 50 Гц). GPIO 34..39 — input-only, для PWM не підходять */
#define BOARD_SERVO_X_GPIO      13
#define BOARD_SERVO_Y_GPIO      12

#define BOARD_CONTROL_PERIOD_MS 10 /* 100 Гц */

/* SPI-дисплей (зарезервовано, поки не використовується) */
#define BOARD_DISP_SPI_MOSI     23
#define BOARD_DISP_SPI_SCLK     18
#define BOARD_DISP_SPI_CS       5
#define BOARD_DISP_DC_GPIO      4
#define BOARD_DISP_RST_GPIO     2
