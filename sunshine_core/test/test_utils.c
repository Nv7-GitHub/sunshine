/* test/test_utils.c */
#include "test_runner.h"
#include "sunshine_core.h"
#include <string.h>

int main(void) {
    /* float16 round-trip */
    float vals[] = {0.0f, 1.0f, -1.0f, 3.14f, 0.001f, 65000.0f, -300.5f};
    for (int i = 0; i < 7; i++) {
        float rt = sunshine_f16_to_f32(sunshine_f32_to_f16(vals[i]));
        ASSERT_NEAR(rt, vals[i], fabsf(vals[i]) * 0.005f + 1e-5f,
                    "float16 round-trip within 0.5%");
    }
    ASSERT_EQ(sunshine_f32_to_f16(0.0f), 0, "zero encodes to 0");

    /* Unit conversions */
    ASSERT_NEAR(sunshine_accel_to_ms2(0),    0.0f,   0.001f, "accel 0 → 0 m/s²");
    ASSERT_NEAR(sunshine_accel_to_ms2(100),  100.0f * ADXL_SCALE_MS2, 0.001f, "accel scale");
    ASSERT_NEAR(sunshine_mag_to_ut(1000),    1000.0f * MAG_SCALE_UT,  0.001f, "mag scale");
    ASSERT_NEAR(sunshine_batt_to_v(0),       7.6f,   0.001f, "batt offset 0 → 7.6V");
    ASSERT_NEAR(sunshine_batt_to_v(127),     7.6f + 127*0.0205f, 0.001f, "batt max");
    ASSERT_NEAR(sunshine_batt_to_v(-127),    7.6f - 127*0.0205f, 0.001f, "batt min");

    TEST_RESULTS();
}
