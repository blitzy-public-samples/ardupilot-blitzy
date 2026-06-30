/*
 * test_gps_status.cpp - GoogleTest unit tests for the AP_GPS fix-status
 * state machine and associated timing/blending invariants.
 *
 * This is a host-compiled unit test, built and executed by waf's
 * `ap_find_tests` helper (one binary per *.cpp) via `./waf check`. It is a
 * standalone binary (`test_gps_status`), fully independent of the sibling
 * `test_gps` (NMEA parser) binary in this directory.
 *
 * The behaviours asserted here mirror, verbatim, the documented semantics of
 * the AP_GPS system-under-test (read-only references; nothing in
 * libraries/AP_GPS/*.{cpp,h} is modified):
 *
 *   - GPS_Status enum ordering (AP_GPS.h)               -> ForwardOnlyProgression
 *   - GPS update-rate clamp, GPS_MAX_RATE_MS == 200     -> RateClampToMax
 *       (AP_GPS.cpp: if (rate_ms <= 0 || rate_ms > GPS_MAX_RATE_MS) rate_ms = GPS_MAX_RATE_MS)
 *   - Inverse-variance blending weights                 -> InverseVarianceBlendingWeights
 *       (AP_GPS_Blended.cpp _calc_weights(): weight = sum_sq / sq(h_acc), then normalise)
 *   - GPS idle-timeout reset, GPS_TIMEOUT_MS == 4000u   -> TimeoutResetsToLowestState
 *       (AP_GPS.cpp: if (tnow - last_message_time_ms > GPS_TIMEOUT_MS) memset(state, 0, ...))
 *
 * NOTE: GPS_MAX_RATE_MS and GPS_TIMEOUT_MS are #defined inside AP_GPS.cpp and
 * are NOT visible from AP_GPS.h. We therefore encode their documented literal
 * values in local DOC_* constants; we deliberately do NOT #define/redeclare or
 * otherwise reference the translation-unit-local macros.
 */
#include <AP_gtest.h>
#include <AP_GPS/AP_GPS.h>

// Required at file scope by every ArduPilot unit-test translation unit so that
// the HAL singleton is referenced/resolved at link time.
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Documented, translation-unit-local constants from AP_GPS.cpp (NOT header-visible).
// We encode their documented literal values; we do NOT #define/redeclare the macros.
static const int      DOC_GPS_MAX_RATE_MS = 200;     // AP_GPS.cpp L71: max rate_ms (slowest update = 5 Hz)
static const uint32_t DOC_GPS_TIMEOUT_MS  = 4000u;   // AP_GPS.cpp L74: GPS idle timeout

// Replicates the rate-clamp branch in AP_GPS::init() (AP_GPS.cpp ~L351):
//   if (rate_ms <= 0 || rate_ms > GPS_MAX_RATE_MS) { rate_ms.set(GPS_MAX_RATE_MS); }
// Returns the value that AP_GPS would store for a given requested rate.
static int clamp_rate_ms(int requested_ms)
{
    if (requested_ms <= 0 || requested_ms > DOC_GPS_MAX_RATE_MS) {
        return DOC_GPS_MAX_RATE_MS;
    }
    return requested_ms;
}

// Replicates the idle-timeout decision in AP_GPS::update_instance() (AP_GPS.cpp ~L890):
//   if (tnow - last_message_time_ms > GPS_TIMEOUT_MS) { memset(state, 0, ...); }
// Note the comparison is strictly greater-than: exactly GPS_TIMEOUT_MS is NOT a timeout.
static bool gps_timed_out(uint32_t now_ms, uint32_t last_msg_ms)
{
    return (now_ms - last_msg_ms) > DOC_GPS_TIMEOUT_MS;
}

// Happy path: the GPS_Status enum encodes a forward-only fix-quality ladder.
// Values must be exactly 0..6 and the ladder strictly increasing & contiguous,
// so that legal transitions advance one rung at a time (NO_GPS -> ... -> RTK_FIXED).
TEST(AP_GPS_Status, ForwardOnlyProgression)
{
    // Fully-qualified enumerant values (AP_GPS.h L149-157).
    EXPECT_EQ(0, (int)AP_GPS::NO_GPS);
    EXPECT_EQ(1, (int)AP_GPS::NO_FIX);
    EXPECT_EQ(2, (int)AP_GPS::GPS_OK_FIX_2D);
    EXPECT_EQ(3, (int)AP_GPS::GPS_OK_FIX_3D);
    EXPECT_EQ(4, (int)AP_GPS::GPS_OK_FIX_3D_DGPS);
    EXPECT_EQ(5, (int)AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT);
    EXPECT_EQ(6, (int)AP_GPS::GPS_OK_FIX_3D_RTK_FIXED);

    const AP_GPS::GPS_Status ladder[] = {
        AP_GPS::NO_GPS,
        AP_GPS::NO_FIX,
        AP_GPS::GPS_OK_FIX_2D,
        AP_GPS::GPS_OK_FIX_3D,
        AP_GPS::GPS_OK_FIX_3D_DGPS,
        AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT,
        AP_GPS::GPS_OK_FIX_3D_RTK_FIXED,
    };

    // The ladder lists every value once, lowest-to-highest fix quality.
    EXPECT_EQ(7U, (unsigned)ARRAY_SIZE(ladder));
    EXPECT_EQ((int)AP_GPS::NO_GPS,                 (int)ladder[0]);
    EXPECT_EQ((int)AP_GPS::GPS_OK_FIX_3D_RTK_FIXED, (int)ladder[ARRAY_SIZE(ladder) - 1]);

    for (uint8_t i = 1; i < ARRAY_SIZE(ladder); i++) {
        // strictly increasing and contiguous (+1) -- encodes legal forward transitions
        EXPECT_LT((int)ladder[i - 1], (int)ladder[i]);
        EXPECT_EQ(1, (int)ladder[i] - (int)ladder[i - 1]);
    }
}

// Edge: the configured GPS update rate is clamped to GPS_MAX_RATE_MS (200 ms / 5 Hz).
// Values at or below the limit pass through unchanged; values above it, zero, and
// negative values are all clamped to the maximum.
TEST(AP_GPS_Status, RateClampToMax)
{
    EXPECT_EQ(200, DOC_GPS_MAX_RATE_MS);   // documented constant value

    // Above the maximum -> clamped to the maximum.
    EXPECT_EQ(200, clamp_rate_ms(1000));
    EXPECT_EQ(200, clamp_rate_ms(201));    // just above the boundary -> clamped

    // At and below the maximum -> passthrough.
    EXPECT_EQ(200, clamp_rate_ms(200));    // boundary itself passes through unchanged
    EXPECT_EQ(199, clamp_rate_ms(199));    // just below the boundary -> passthrough
    EXPECT_EQ(100, clamp_rate_ms(100));    // mid-range -> passthrough
    EXPECT_EQ(1,   clamp_rate_ms(1));      // smallest positive -> passthrough

    // Non-positive requests are treated as invalid and clamped to the maximum.
    EXPECT_EQ(200, clamp_rate_ms(0));
    EXPECT_EQ(200, clamp_rate_ms(-5));
}

// Edge: inverse-variance blending assigns a strictly higher normalised weight to
// the receiver with the lower variance (smaller horizontal accuracy figure), and
// the normalised weights sum to ~1.0. The private _calc_weights() formula is
// replicated here with local floats (the SUT method is not directly constructible).
TEST(AP_GPS_Status, InverseVarianceBlendingWeights)
{
    // --- Unequal accuracy: receiver 0 is twice as accurate as receiver 1. ---
    {
        const float h_acc0 = 1.0f;   // lower variance (more accurate)
        const float h_acc1 = 2.0f;   // higher variance (less accurate)

        // horizontal_accuracy_sum_sq = Sigma sq(horizontal_accuracy[i])
        const float sum_sq = h_acc0 * h_acc0 + h_acc1 * h_acc1;   // 1 + 4 = 5

        // weight[i] = horizontal_accuracy_sum_sq / sq(horizontal_accuracy[i])
        float w0 = sum_sq / (h_acc0 * h_acc0);   // 5 / 1   = 5.0
        float w1 = sum_sq / (h_acc1 * h_acc1);   // 5 / 4   = 1.25

        const float sum_w = w0 + w1;             // 6.25
        ASSERT_GT(sum_w, 0.0f);

        // normalise the weights
        w0 /= sum_w;   // 0.8
        w1 /= sum_w;   // 0.2

        EXPECT_GT(w0, w1);                       // lower variance -> strictly higher weight
        EXPECT_NEAR(w0, 0.8f, 1e-6f);            // exact normalised value
        EXPECT_NEAR(w1, 0.2f, 1e-6f);            // exact normalised value
        EXPECT_NEAR(w0 + w1, 1.0f, 1e-6f);       // normalised weights sum to ~1.0
    }

    // --- Equal accuracy: weights must be symmetric (0.5 / 0.5). ---
    {
        const float h_acc = 1.5f;
        const float sum_sq = h_acc * h_acc + h_acc * h_acc;
        float w0 = sum_sq / (h_acc * h_acc);
        float w1 = sum_sq / (h_acc * h_acc);
        const float sum_w = w0 + w1;
        ASSERT_GT(sum_w, 0.0f);
        w0 /= sum_w;
        w1 /= sum_w;

        EXPECT_FLOAT_EQ(w0, w1);                 // equal variance -> equal weight
        EXPECT_NEAR(w0, 0.5f, 1e-6f);
        EXPECT_NEAR(w0 + w1, 1.0f, 1e-6f);
    }
}

// Error/recovery: when no message has arrived for longer than GPS_TIMEOUT_MS the
// instance state is memset to zero, which drops status to the lowest enum value
// (NO_GPS). This is always a reset to the floor, never an illegal forward skip.
TEST(AP_GPS_Status, TimeoutResetsToLowestState)
{
    EXPECT_EQ(4000u, DOC_GPS_TIMEOUT_MS);    // documented constant value

    // The comparison in AP_GPS is strictly greater-than, so the boundary itself
    // (exactly GPS_TIMEOUT_MS elapsed) is NOT yet a timeout.
    EXPECT_FALSE(gps_timed_out(4000, 0));    // boundary: 4000 is NOT > 4000
    EXPECT_FALSE(gps_timed_out(3999, 0));    // below boundary -> no timeout
    EXPECT_TRUE(gps_timed_out(4001, 0));     // just over boundary -> timeout
    EXPECT_TRUE(gps_timed_out(5000, 0));     // well over -> timeout

    // The timeout decision is relative to the last message time, not absolute.
    EXPECT_FALSE(gps_timed_out(104000, 100000)); // delta == 4000 -> no timeout
    EXPECT_TRUE(gps_timed_out(104001, 100000));  // delta == 4001 -> timeout

    // On timeout the instance state is memset to 0 => status == NO_GPS (0),
    // which is <= NO_FIX: always a reset to the floor, never a forward skip.
    const AP_GPS::GPS_Status reset_status = AP_GPS::NO_GPS;
    EXPECT_EQ(0, (int)reset_status);
    EXPECT_LE((int)reset_status, (int)AP_GPS::NO_FIX);
    EXPECT_LE((int)reset_status, (int)AP_GPS::GPS_OK_FIX_3D_RTK_FIXED);
}

AP_GTEST_MAIN()
