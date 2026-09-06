#pragma once 
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct servo_t servo_t; // непрозорий хендл 

typedef struct {
    int gpio;
    int ledc_channel;
    uint16_t min_pulse_us;    // 500  
    uint16_t max_pulse_us;    // 2500
    float    min_angle_deg;   // 0
    float    max_angle_deg;   // 180
} servo_config_t;

#define SERVO_CONFIG_SG90(pin, ch) (servo_config_t){
    .gpio = (pin),
    .ledc_channel = (ch),
    .min_pulse_us = 500,
    .max_pulse_us = 2500,
    .min_angle_deg = 0.0f,
    .max_angle_deg = 180.0f,
}

esp_err_t servo_create(const servo_config_t *cfg, servo_t **out);
esp_err_t servo_set_angle(servo_t *servo, float angle_deg);
float servo_get_angle(const servo_t *servo);
void servo_delete(servo_t *servo);

#ifdef __cplusplus
}
#endif