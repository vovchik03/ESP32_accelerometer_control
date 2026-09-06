#include <math.h>
#include "tilt_control.h"

#define RAD_TO_DEG (57.29577951308232f)

tilt_angles_t tilt_from_accel(tilt_vec3_t a)
{
    tilt_angles_t out;
    out.roll_deg  = atan2f(a.y, a.z) * RAD_TO_DEG;
    out.pitch_deg = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z)) * RAD_TO_DEG;
    return out;
}

tilt_angles_t tilt_filter_update(tilt_angles_t prev,
                                 tilt_vec3_t   accel_g,
                                 tilt_vec3_t   gyro_dps,
                                 float         dt_s,
                                 float         alpha)
{
    tilt_angles_t from_accel = tilt_from_accel(accel_g);

    tilt_angles_t out;
    out.roll_deg  = alpha * (prev.roll_deg  + gyro_dps.x * dt_s) +
                    (1.0f - alpha) * from_accel.roll_deg;
    out.pitch_deg = alpha * (prev.pitch_deg + gyro_dps.y * dt_s) +
                    (1.0f - alpha) * from_accel.pitch_deg;
    return out;
}

float tilt_map(float value,
               float in_min,  float in_max,
               float out_min, float out_max)
{
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;

    float k = (value - in_min) / (in_max - in_min);
    return out_min + k * (out_max - out_min);
}