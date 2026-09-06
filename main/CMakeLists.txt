idf_component_register(
    SRCS    "main.c"
    INCLUDE_DIRS "."
    REQUIRES mpu6050 servo tilt_control esp_driver_i2c)
