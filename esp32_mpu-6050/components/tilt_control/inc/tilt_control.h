#pragma once

typedef struct { float x, y, z; } tilt_vec3_t;
typedef struct { float roll_deg, pitch_deg; } tilt_angles_t;

// Кути нахилу з вектора прискорення
tilt_angles_t tilt_from_accel(tilt_vec3_t accel_g);

// Комплементарний фільтр: змішує акселерометр з гіроскопом
tilt_angles_t tilt_filter_update(tilt_angles_t prev, tilt_vec3_t accel_g, tilt_vec3_t gyro_dps, float dt_s, float alpha);

// Кут нахилу → кут серво з обмеженням діапазону
float tilt_map_to_servo(float tilt_deg, float out_min, float out_max);