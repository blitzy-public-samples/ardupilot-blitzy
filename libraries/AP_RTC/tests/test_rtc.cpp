/*
 * GoogleTest unit tests for AP_RTC and JitterCorrection.
 *
 * System Under Test (READ-ONLY - never modified by these tests):
 *   libraries/AP_RTC/AP_RTC.{h,cpp}            - UTC source arbitration and
 *                                                time-sanity (sanity-floor) logic.
 *   libraries/AP_RTC/JitterCorrection.{h,cpp}  - offboard-timestamp transport
 *                                                jitter compensation.
 *
 * Anatomy follows the repository convention established by
 * libraries/AP_Common/tests/test_location.cpp together with the harness macros
 * in tests/AP_gtest.h.  Each test translation unit is host-compiled by waf and
 * linked into its own standalone binary, executed by `./waf check` and
 * `./waf check-all`.
 *
 * IMPORTANT - zero-initialization requirement (the #1 correctness pitfall):
 *   AP_RTC::rtc_shift has no in-class initializer and is not set by the
 *   constructor.  Likewise, four JitterCorrection members - link_offset_usec,
 *   min_sample_us, initialised and min_sample_counter - have no in-class
 *   initializers and are not set by the constructor (only the const members
 *   max_lag_ms and convergence_loops are initialized).  A stack/local instance
 *   would therefore leave those members indeterminate, producing undefined
 *   behavior in the arbitration / jitter / convergence logic.
 *
 *   REMEDY: every AP_RTC and JitterCorrection object below is declared with
 *   static storage duration (a function-local static inside each TEST), which
 *   guarantees zero-initialization before construction and gives each test a
 *   clean, deterministic starting state.  This mirrors the proven pattern in
 *   libraries/AP_RTC/examples/RTC_test/RTC_Test.cpp (`static AP_RTC _rtc;`).
 *   All method calls go through the local-static instance directly - never via
 *   the AP::rtc() singleton - so each test is independent of singleton state.
 */

#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_RTC/JitterCorrection.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Mandatory for linkage: AP_RTC::set_utc_usec() calls the static
// GCS_MAVLINK::update_signing_timestamp() (compiled because HAL_GCS_ENABLED).
// A file-scope GCS_Dummy satisfies the GCS singleton/linkage requirement and
// is runtime-safe with no initialized MAVLink channels.  This mirrors
// test_location.cpp and examples/RTC_test/RTC_Test.cpp.
GCS_Dummy _gcs;

// 2022-01-01 00:00:00 UTC sanity floor - mirrors the SUT constant
// `oldest_acceptable_date_us` in AP_RTC.cpp exactly.  Timestamps strictly
// below this value are rejected by set_utc_usec(); a value equal to it is
// accepted (boundary behavior is exercised by SubFloorTimestampRejected).
static const uint64_t FLOOR_US = 1640995200ULL * 1000 * 1000;

// ---------------------------------------------------------------------------
// Suite: AP_RTC - UTC source arbitration and time-sanity behavior.
//
// Arbitration contract (verified against AP_RTC.cpp):
//   source priority (lower enum value = higher priority):
//     SOURCE_GPS(0) > SOURCE_MAVLINK_SYSTEM_TIME(1) > SOURCE_HW(2) > SOURCE_NONE(3)
//   While the vehicle is DISARMED (the default in the unit-test HAL) a set is
//   rejected when `type > rtc_source_type` (strict).  It must then pass the
//   allowed-types bitmask gate, the sanity floor, and the no-backwards check.
//
// There is no public getter for rtc_source_type, so the "winning" source is
// inferred indirectly from the value returned by get_utc_usec().  Timestamps
// are separated by large, unambiguous margins (~1000 s) and assertions use
// generous +/-5 s / +/-900 s windows so they hold regardless of host speed.
// ---------------------------------------------------------------------------

// Happy path: a valid epoch set from the GPS source is accepted and round-trips
// through get_utc_usec().  Uses the default allowed_types (== 1, GPS only).
TEST(AP_RTC, ValidEpochRoundTrip)
{
    static AP_RTC rtc;
    const uint64_t t = FLOOR_US + 60ULL * 1000 * 1000;   // 60 s above the floor
    rtc.set_utc_usec(t, AP_RTC::SOURCE_GPS);
    uint64_t got = 0;
    ASSERT_TRUE(rtc.get_utc_usec(got));
    EXPECT_GE(got, t);                                   // monotonic clock only moves forward
    EXPECT_LT(got - t, 5000000ULL);                      // within ~5 s of the set value
}

// Happy path: a higher-priority source (GPS) carrying an equal-or-later
// timestamp overwrites a previously accepted lower-priority source (MAVLINK).
// allowed_types is widened to 7 so MAVLINK is permitted by the bitmask gate.
TEST(AP_RTC, GpsOverridesMavlink)
{
    static AP_RTC rtc;
    rtc.allowed_types.set(7);                            // GPS | MAVLINK | HW (bits 0,1,2)
    const uint64_t t_mav = FLOOR_US + 10ULL * 1000 * 1000;   // floor + 10 s
    rtc.set_utc_usec(t_mav, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);
    uint64_t got1 = 0;
    ASSERT_TRUE(rtc.get_utc_usec(got1));                 // ~= t_mav
    const uint64_t t_gps = t_mav + 1000ULL * 1000 * 1000;    // +1000 s (later AND higher priority)
    rtc.set_utc_usec(t_gps, AP_RTC::SOURCE_GPS);
    uint64_t got2 = 0;
    ASSERT_TRUE(rtc.get_utc_usec(got2));                 // ~= t_gps
    EXPECT_GT(got2, got1 + 900000000ULL);                // proves GPS (~+1000 s) overwrote MAVLINK
}

// Error/boundary: the 2022-01-01 sanity floor rejects timestamps strictly
// below it, while a timestamp exactly equal to the floor is accepted.  Also
// confirms get_utc_usec() returns false while no source has been accepted.
TEST(AP_RTC, SubFloorTimestampRejected)
{
    static AP_RTC rtc;
    uint64_t usec = 0;
    EXPECT_FALSE(rtc.get_utc_usec(usec));                // SOURCE_NONE initially -> false
    rtc.set_utc_usec(FLOOR_US - 1, AP_RTC::SOURCE_GPS);  // below floor -> rejected
    EXPECT_FALSE(rtc.get_utc_usec(usec));                // still unset
    rtc.set_utc_usec(FLOOR_US, AP_RTC::SOURCE_GPS);      // == floor -> accepted (boundary)
    EXPECT_TRUE(rtc.get_utc_usec(usec));
}

// Error: a lower-priority source (MAVLINK) cannot overwrite a higher-priority
// source (GPS) even with a much later timestamp - it is rejected at the
// arbitration step before the timestamp is ever considered.
TEST(AP_RTC, MavlinkDoesNotOverrideGps)
{
    static AP_RTC rtc;
    rtc.allowed_types.set(7);
    const uint64_t t_gps = FLOOR_US + 10ULL * 1000 * 1000;
    rtc.set_utc_usec(t_gps, AP_RTC::SOURCE_GPS);
    uint64_t got1 = 0;
    ASSERT_TRUE(rtc.get_utc_usec(got1));                 // ~= t_gps
    const uint64_t t_mav = t_gps + 1000ULL * 1000 * 1000;    // later, but lower priority
    rtc.set_utc_usec(t_mav, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);  // rejected at arbitration
    uint64_t got2 = 0;
    ASSERT_TRUE(rtc.get_utc_usec(got2));
    EXPECT_LT(got2 - got1, 5000000ULL);                  // still reflects t_gps, NOT t_mav (~+1000 s)
}

// Error: SOURCE_NONE is never selected.  Even with allowed_types widened to 7,
// bit 3 (SOURCE_NONE) is deliberately not set, so a set with SOURCE_NONE is
// rejected at the allowed-types gate and get_utc_usec() stays false.
TEST(AP_RTC, SourceNoneNeverSelected)
{
    static AP_RTC rtc;
    rtc.allowed_types.set(7);                            // bit 3 (SOURCE_NONE) deliberately NOT set
    uint64_t usec = 0;
    EXPECT_FALSE(rtc.get_utc_usec(usec));
    rtc.set_utc_usec(FLOOR_US + 5ULL * 1000 * 1000, AP_RTC::SOURCE_NONE);  // rejected (allowed-types gate)
    EXPECT_FALSE(rtc.get_utc_usec(usec));                // never selected
}

// ---------------------------------------------------------------------------
// Suite: JitterCorrection - offboard-timestamp jitter compensation.
//
// Algorithm contract (verified against JitterCorrection.cpp):
//   diff_us = local - offboard;
//   if (!initialised || diff_us < link_offset_usec) link_offset_usec = diff_us;
//   estimate_us = offboard + link_offset_usec;
//   if (estimate_us + max_lag_ms*1000 < local)        // clamp to maximum lag
//       estimate_us = local - max_lag_ms*1000;
//   <running-minimum convergence over `convergence_loops` samples>
//   return estimate_us;
//
// All expected values below were verified by simulating the exact algorithm.
// ---------------------------------------------------------------------------

// Edge: a small apparent lag (1 ms, well under the 500 ms max) is reflected
// directly in both the returned estimate and the link offset; no clamping.
TEST(JitterCorrection, SmallLagReflectsOffset)
{
    static JitterCorrection jc;                          // defaults: max_lag_ms=500, convergence_loops=100
    EXPECT_EQ(jc.correct_offboard_timestamp_usec(0, 1000), 1000U);  // 1 ms lag < 500 ms
    EXPECT_EQ(jc.get_link_offset_usec(), 1000);
}

// Edge: an apparent lag far above max_lag (~1 s vs the 500 ms cap) is clamped,
// so the estimate is exactly local - max_lag_ms*1000.
TEST(JitterCorrection, LargeLagClampedToMaxLag)
{
    static JitterCorrection jc;
    jc.correct_offboard_timestamp_usec(0, 1000);                          // seed the minimum offset
    const uint64_t est = jc.correct_offboard_timestamp_usec(0, 1000000);  // ~1 s apparent lag >= 500 ms
    EXPECT_EQ(est, 500000U);                             // clamped: local - max_lag_ms*1000
    EXPECT_EQ(1000000U - est, 500000U);                  // == max_lag_ms*1000 (500 ms)
}

// Edge: a custom max_lag (200 ms) changes the clamp point, demonstrating the
// constructor parameter is honored: estimate == local - 200 ms.
TEST(JitterCorrection, CustomParams)
{
    static JitterCorrection jc(200, 50);
    jc.correct_offboard_timestamp_usec(0, 1000);                          // seed the minimum offset
    const uint64_t est = jc.correct_offboard_timestamp_usec(0, 1000000);
    EXPECT_EQ(est, 800000U);                             // clamp at 200 ms -> 1000000 - 200000
}

// Edge: after exactly `convergence_loops` samples, the algorithm promotes the
// running-minimum transport lag back into link_offset_usec.  Here the true
// minimum (1 ms, from the first sample) is restored on the 100th call even
// though intervening clamped samples transiently raised the offset.
TEST(JitterCorrection, ConvergenceWithinLoops)
{
    static JitterCorrection jc(500, 100);                // max_lag_ms=500, convergence_loops=100
    // Call #1 carries the true minimum transport lag (1 ms):
    EXPECT_EQ(jc.correct_offboard_timestamp_usec(0, 1000), 1000U);
    EXPECT_EQ(jc.get_link_offset_usec(), 1000);
    // Calls #2..#100: 99 large-lag samples (~1 s).  Each is clamped to max_lag,
    // which transiently pushes link_offset up to 1,500,000 us during the window.
    for (uint16_t i = 0; i < 99; i++) {
        jc.correct_offboard_timestamp_usec(0, 2000000);
    }
    // The 100th total call (1 initial + 99 in the loop) completes the
    // convergence window: min_sample_counter reaches convergence_loops, so
    // link_offset is reset to the running minimum sample (1000), then the
    // counter resets to 0.  This proves genuine convergence.
    EXPECT_EQ(jc.get_link_offset_usec(), 1000);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
