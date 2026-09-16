/* Хостові тести tilt_control: gcc + -lm, без ESP-IDF */
#include <math.h>
#include <stdio.h>
#include "tilt_control.h"

static int s_failed = 0;

#define ASSERT_NEAR(actual, expected, eps)                                   \
    do {                                                                     \
        float a_ = (actual), e_ = (expected);                                \
        if (fabsf(a_ - e_) > (eps)) {                                        \
            printf("FAIL %s:%d: %s = %f, expected %f\n",                     \
                   __FILE__, __LINE__, #actual, (double)a_, (double)e_);     \
            s_failed++;                                                      \
        }                                                                    \
    } while (0)

static void test_from_accel_level(void)
{
    tilt_angles_t t = tilt_from_accel((tilt_vec3_t){ 0.0f, 0.0f, 1.0f });
    ASSERT_NEAR(t.roll_deg,  0.0f, 0.01f);
    ASSERT_NEAR(t.pitch_deg, 0.0f, 0.01f);
}

static void test_from_accel_roll_45(void)
{
    float c = 0.70710678f;
    tilt_angles_t t = tilt_from_accel((tilt_vec3_t){ 0.0f, c, c });
    ASSERT_NEAR(t.roll_deg,  45.0f, 0.01f);
    ASSERT_NEAR(t.pitch_deg,  0.0f, 0.01f);
}

static void test_from_accel_pitch_minus_45(void)
{
    float c = 0.70710678f;
    tilt_angles_t t = tilt_from_accel((tilt_vec3_t){ c, 0.0f, c });
    ASSERT_NEAR(t.pitch_deg, -45.0f, 0.01f);
}

static void test_filter_converges_to_accel(void)
{
    tilt_angles_t s = { 0.0f, 0.0f };
    tilt_vec3_t a = { 0.0f, 0.5f, 0.8660254f };   /* roll = 30° */
    tilt_vec3_t g = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < 2000; i++) {
        s = tilt_filter_update(s, a, g, 0.01f, 0.98f);
    }
    ASSERT_NEAR(s.roll_deg, 30.0f, 0.1f);
}

static void test_filter_integrates_gyro(void)
{
    tilt_angles_t s = { 0.0f, 0.0f };
    tilt_vec3_t a = { 0.0f, 0.0f, 1.0f };
    tilt_vec3_t g = { 100.0f, 0.0f, 0.0f };      /* 100 °/с по X */
    s = tilt_filter_update(s, a, g, 0.01f, 1.0f); /* alpha=1: лише гіроскоп */
    ASSERT_NEAR(s.roll_deg, 1.0f, 0.001f);
}

static void test_map(void)
{
    ASSERT_NEAR(tilt_map(  0.0f, -60.0f, 60.0f, 0.0f, 180.0f),  90.0f, 0.001f);
    ASSERT_NEAR(tilt_map(-60.0f, -60.0f, 60.0f, 0.0f, 180.0f),   0.0f, 0.001f);
    ASSERT_NEAR(tilt_map( 60.0f, -60.0f, 60.0f, 0.0f, 180.0f), 180.0f, 0.001f);
    ASSERT_NEAR(tilt_map( 90.0f, -60.0f, 60.0f, 0.0f, 180.0f), 180.0f, 0.001f); /* clamp */
    ASSERT_NEAR(tilt_map(-90.0f, -60.0f, 60.0f, 0.0f, 180.0f),   0.0f, 0.001f); /* clamp */
}

int main(void)
{
    test_from_accel_level();
    test_from_accel_roll_45();
    test_from_accel_pitch_minus_45();
    test_filter_converges_to_accel();
    test_filter_integrates_gyro();
    test_map();

    if (s_failed) {
        printf("%d assertion(s) failed\n", s_failed);
        return 1;
    }
    printf("all tilt_control tests passed\n");
    return 0;
}
