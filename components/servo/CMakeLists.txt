idf_component_register(
    SRCS "src/servo.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_ledc)