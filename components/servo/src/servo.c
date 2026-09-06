#include <stdlib.h>
#include <string.h>

#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_MODE LEDC_LOW_SPEED_MODE
#define SERVO_FREQ_HZ 50
#define SERVO_DUTY_RES     LEDC_TIMER_16_BIT
#define SERVO_PERIOD_US    (1000000 / SERVO_FREQ_HZ) // 20 000 мкс

static const char *TAG = "servo";

static bool s_timer_ready = false;

struct servo_t{
    servo_config_t cfg;
    float angle_deg;
};

static esp_err_t servo_timer_init(void){
    if(s_timer_ready){
        return ESP_OK;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .duty_resolution = SERVO_DUTY_RES,
        .timer_num       = SERVO_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    }

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }
    s_timer_ready = true;
    return ESP_OK; 
}

static uint32_t pulse_us_to_duty(uint32_t pulse_us){
    const uint32_t max_duty = (1u << SERVO_DUTY_RES) - 1u;
    return (uint32_t)(((uint64_t)pulse_us * max_duty) / SERVO_PERIOD_US);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

esp_err_t servo_create(const servo_config_t *cfg, servo_t **out_servo)
{
    if (cfg == NULL || out_servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->max_angle_deg <= cfg->min_angle_deg ||
        cfg->max_pulse_us  <= cfg->min_pulse_us) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = servo_timer_init();
    if (err != ESP_OK) {
        return err;
    }

    servo_t *servo = calloc(1, sizeof(servo_t));
    if (servo == NULL) {
        return ESP_ERR_NO_MEM;
    }
    servo->cfg = *cfg;

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = cfg->gpio,
        .speed_mode = SERVO_MODE,
        .channel    = (ledc_channel_t)cfg->ledc_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };

    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        free(servo);
        return err;
    }

    /* Стартова позиція — середина діапазону */
    float mid = (cfg->min_angle_deg + cfg->max_angle_deg) * 0.5f;
    err = servo_set_angle(servo, mid);
    if (err != ESP_OK) {
        free(servo);
        return err;
    }

    ESP_LOGI(TAG, "servo on GPIO %d, LEDC channel %d",
             cfg->gpio, cfg->ledc_channel);

    *out_servo = servo;
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_t *servo, float angle_deg)
{
    if (servo == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const servo_config_t *c = &servo->cfg;
    angle_deg = clampf(angle_deg, c->min_angle_deg, c->max_angle_deg);

    float k = (angle_deg - c->min_angle_deg) /
              (c->max_angle_deg - c->min_angle_deg);          /* 0.0 .. 1.0 */
    uint32_t pulse_us = c->min_pulse_us +
                        (uint32_t)(k * (c->max_pulse_us - c->min_pulse_us));

    esp_err_t err = ledc_set_duty(SERVO_MODE,
                                  (ledc_channel_t)c->ledc_channel,
                                  pulse_us_to_duty(pulse_us));
    if (err != ESP_OK) {
        return err;
    }

    err = ledc_update_duty(SERVO_MODE, (ledc_channel_t)c->ledc_channel);
    if (err != ESP_OK) {
        return err;
    }

    servo->angle_deg = angle_deg;
    return ESP_OK;
}

float servo_get_angle(const servo_t *servo)
{
    return (servo != NULL) ? servo->angle_deg : 0.0f;
}

void servo_delete(servo_t *servo)
{
    if (servo == NULL) {
        return;
    }
    ledc_stop(SERVO_MODE, (ledc_channel_t)servo->cfg.ledc_channel, 0);
    free(servo);
}