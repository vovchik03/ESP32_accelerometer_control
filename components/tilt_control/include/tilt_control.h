#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} tilt_vec3_t;

typedef struct {
    float roll_deg;    // поворот навколо X 
    float pitch_deg;   // поворот навколо Y 
} tilt_angles_t;

// Кути нахилу з вектора прискорення (у g)
tilt_angles_t tilt_from_accel(tilt_vec3_t accel_g);

/*
 * Комплементарний фільтр.
 * alpha — вага гіроскопа, типово 0.95..0.99.
 * dt_s  — час з попереднього виклику, секунди.
 */
tilt_angles_t tilt_filter_update(tilt_angles_t prev,
                                 tilt_vec3_t   accel_g,
                                 tilt_vec3_t   gyro_dps,
                                 float         dt_s,
                                 float         alpha);

// Лінійне відображення діапазону в діапазон з обрізанням по краях. 
float tilt_map(float value,
               float in_min,  float in_max,
               float out_min, float out_max);

#ifdef __cplusplus
}
#endif