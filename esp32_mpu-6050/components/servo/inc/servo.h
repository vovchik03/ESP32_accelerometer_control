#pragma once 
#include "esp_err.h"

typedef struct servo_t servo_t; // непрозорий хендл 

typedef struct {
    int gpio;
    int ledc_channel;
    uint16_t min_pulse_us;    // 500  
    uint16_t max_pulse_us;    // 2500
    float    min_angle_deg;   // 0
    float    max_angle_deg;   // 180
} servo_config_t;

esp_err_t servo_create(const servo_config_t *cfg, servo_t **out);
esp_err_t servo_set_angle(servo_t *servo, float angle_deg);
void servo_delete(servo_t *servo);