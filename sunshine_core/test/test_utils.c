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

    /* NaN is the eRPM "ESC not commutating, unmeasurable" sentinel (design §4.1),
     * so it has to survive float16 encoding instead of collapsing to +inf. */
    ASSERT(isnan(sunshine_f16_to_f32(sunshine_f32_to_f16(NAN))), "NaN round-trips as NaN");
    uint16_t nan_enc = sunshine_f32_to_f16(NAN);
    ASSERT((nan_enc & 0x7C00u) == 0x7C00u && (nan_enc & 0x03FFu) != 0,
           "NaN encodes to a half NaN pattern (exp=31, mant!=0), not inf");
    /* KEY REGRESSION: a NaN whose payload lives ONLY in the low 13 bits.  The naive
     * fix (OR the shifted-down mantissa through) drops those bits, yields mant==0 and
     * emits 0x7C00 = +inf here.  Must still be NaN. */
    uint32_t snan_bits = 0x7F800001u; float snan; memcpy(&snan, &snan_bits, 4);
    ASSERT(isnan(sunshine_f16_to_f32(sunshine_f32_to_f16(snan))),
           "NaN with payload only in the low 13 mantissa bits survives");
    ASSERT(isnan(sunshine_f16_to_f32(sunshine_f32_to_f16(-NAN))), "-NaN round-trips as NaN");

    /* Infinity must stay infinity, and finite overflow must SATURATE to infinity —
     * an over-broad "preserve NaN" fix that turns 1e30f into NaN is also wrong. */
    ASSERT_EQ(sunshine_f32_to_f16(INFINITY),  0x7C00, "+inf encodes to 0x7C00");
    ASSERT_EQ(sunshine_f32_to_f16(-INFINITY), 0xFC00, "-inf encodes to 0xFC00");
    float inf_rt = sunshine_f16_to_f32(sunshine_f32_to_f16(INFINITY));
    ASSERT(isinf(inf_rt) && inf_rt > 0, "+inf round-trips as +inf");
    float ninf_rt = sunshine_f16_to_f32(sunshine_f32_to_f16(-INFINITY));
    ASSERT(isinf(ninf_rt) && ninf_rt < 0, "-inf round-trips as -inf");
    ASSERT_EQ(sunshine_f32_to_f16(1e30f),  0x7C00, "finite overflow still saturates to +inf, not NaN");
    ASSERT_EQ(sunshine_f32_to_f16(-1e30f), 0xFC00, "finite -overflow still saturates to -inf, not NaN");
    ASSERT_EQ(sunshine_f32_to_f16(sunshine_f16_to_f32(0x7E00)), 0x7E00,
              "quiet-NaN half is idempotent through a second round trip");

    /* Bit-identity guard: the NaN fix must not perturb ANY finite value, so sweep
     * every normal f16 code rather than sampling a few.  exp==0 is skipped on purpose
     * (f16 subnormals decode to floats the encoder flushes to zero — pre-existing
     * behaviour, out of scope) and exp==31 is skipped because the NaN/inf cases above
     * cover it.  Do not "fix" these bounds.  One counter, one ASSERT, readable output. */
    int fin_bad = 0;
    for (int h = 0; h <= 0xFFFF; h++) {
        int e = (h >> 10) & 0x1F;
        if (e == 0 || e == 31) continue;
        if (sunshine_f32_to_f16(sunshine_f16_to_f32((uint16_t)h)) != (uint16_t)h) fin_bad++;
    }
    ASSERT_EQ(fin_bad, 0, "all 61440 normal f16 codes still round-trip bit-exactly");

    /* Unit conversions */
    ASSERT_NEAR(sunshine_accel_to_ms2(0),    0.0f,   0.001f, "accel 0 → 0 m/s²");
    ASSERT_NEAR(sunshine_accel_to_ms2(100),  100.0f * ADXL_SCALE_MS2, 0.001f, "accel scale");
    ASSERT_NEAR(sunshine_mag_to_ut(1000),    1000.0f * MAG_SCALE_UT,  0.001f, "mag scale");
    /* Battery is int16 @ BATT_SCALE_V V/LSB relative to 7.6 V (schema v5). */
    ASSERT_NEAR(sunshine_batt_to_v(0),       BATT_OFFSET_REF_V,   0.001f, "batt offset 0 → 7.6V");
    ASSERT_NEAR(sunshine_batt_to_v(2600),    BATT_OFFSET_REF_V + 2600*BATT_SCALE_V, 0.001f, "batt +2600 → ~10.2V");
    ASSERT_NEAR(sunshine_batt_to_v(-2600),   BATT_OFFSET_REF_V - 2600*BATT_SCALE_V, 0.001f, "batt -2600 → ~5.0V");
    /* Resolution is now ADC-limited (~2.4 mV), not the old 20.5 mV int8 step. */
    ASSERT_NEAR(sunshine_batt_to_v(1) - sunshine_batt_to_v(0), BATT_SCALE_V, 1e-6f, "batt LSB = 1 mV");

    TEST_RESULTS();
}
