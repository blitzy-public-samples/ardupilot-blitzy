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

// ---------------------------------------------------------------------------
// Suite: AP_RTC calendar/clock surface.
//
// The following tests exercise the deterministic date-math and clock-getter
// methods of AP_RTC that are independent of the arbitration path above:
//   - date_fields_to_clock_s()  -> drives the in-tree timegm() (_timegm + the
//                                   leap-year helper _is_leap).
//   - clock_s_to_date_fields()  -> gmtime_r()-based inverse.
//   - get_system_clock_utc() / get_local_time() / get_date_and_time_utc() /
//     get_time_utc()            -> become reachable once a UTC time has been
//                                   accepted by set_utc_usec().
//
// All values are chosen so the assertions are robust against the small,
// unavoidable amount of wall-clock time that elapses between set_utc_usec()
// and the subsequent getter call (the SITL/linux HAL advances micros64()).
// ---------------------------------------------------------------------------

// date_fields_to_clock_s() and clock_s_to_date_fields() are exact inverses over
// valid Gregorian dates.  A non-leap anchor (2022-01-01) pins the numeric epoch
// and the weekday; a leap-day (2024-02-29) forces the leap branch of _is_leap()
// and the February column of the ndays[] table inside _timegm().  A structurally
// invalid month (13, i.e. tm_mon > 12) drives the timegm() rejection path.
TEST(AP_RTC, DateFieldConversions)
{
    static AP_RTC rtc;

    // 2022-01-01 00:00:00 UTC is exactly 1640995200 s past the 1970 epoch
    // (18993 days * 86400).  This is also the RTC sanity floor / 1e6.
    EXPECT_EQ(rtc.date_fields_to_clock_s(2022, 0, 1, 0, 0, 0), 1640995200U);

    // Inverse (gmtime_r) must recover the same calendar fields, including the
    // weekday: 2022-01-01 was a Saturday -> tm_wday == 6.
    uint16_t year;
    uint8_t month, day, hour, minute, sec, wday;
    ASSERT_TRUE(rtc.clock_s_to_date_fields(1640995200U, year, month, day,
                                           hour, minute, sec, wday));
    EXPECT_EQ(year, 2022U);
    EXPECT_EQ(month, 0U);      // January, [0-11]
    EXPECT_EQ(day, 1U);
    EXPECT_EQ(hour, 0U);
    EXPECT_EQ(minute, 0U);
    EXPECT_EQ(sec, 0U);
    EXPECT_EQ(wday, 6U);       // Saturday

    // Leap-day round-trip: exercises _is_leap()==true and the 29-day February.
    const uint32_t leap_s = rtc.date_fields_to_clock_s(2024, 1, 29, 12, 0, 0);
    ASSERT_GT(leap_s, 1640995200U);
    ASSERT_TRUE(rtc.clock_s_to_date_fields(leap_s, year, month, day,
                                           hour, minute, sec, wday));
    EXPECT_EQ(year, 2024U);
    EXPECT_EQ(month, 1U);      // February
    EXPECT_EQ(day, 29U);
    EXPECT_EQ(hour, 12U);

    // Structurally invalid tm (month index 13 > 12) -> _timegm() rejects it.
    EXPECT_EQ(rtc.date_fields_to_clock_s(2022, 13, 1, 0, 0, 0), 0U);
}

// The clock getters report unavailable until a source is accepted, then break
// the accepted time into fields.  get_local_time() applies the tz_min offset.
TEST(AP_RTC, ClockGettersReflectAcceptedTime)
{
    static AP_RTC rtc;

    uint8_t hour, minute, sec;
    uint16_t ms;

    // Before any source: getters report unavailable (get_utc_usec() == false).
    EXPECT_FALSE(rtc.get_system_clock_utc(hour, minute, sec, ms));
    EXPECT_FALSE(rtc.get_local_time(hour, minute, sec, ms));
    uint16_t year;
    uint8_t month, day, dhour, dmin, dsec;
    uint16_t dms;
    EXPECT_FALSE(rtc.get_date_and_time_utc(year, month, day, dhour, dmin, dsec, dms));

    // Accept 2022-01-01 00:30:15.000 UTC via GPS (allowed by the default mask).
    const uint64_t t_us = (1640995200ULL + 30ULL * 60ULL + 15ULL) * 1000ULL * 1000ULL;
    rtc.set_utc_usec(t_us, AP_RTC::SOURCE_GPS);

    // System (UTC) clock: hour/minute are stable across the sub-second test
    // runtime; the second may advance slightly past 15 so it is only bounded.
    ASSERT_TRUE(rtc.get_system_clock_utc(hour, minute, sec, ms));
    EXPECT_EQ(hour, 0U);
    EXPECT_EQ(minute, 30U);
    EXPECT_LT(sec, 60U);

    // Calendar decomposition of the same instant.
    ASSERT_TRUE(rtc.get_date_and_time_utc(year, month, day, dhour, dmin, dsec, dms));
    EXPECT_EQ(year, 2022U);
    EXPECT_EQ(month, 0U);
    EXPECT_EQ(day, 1U);
    EXPECT_EQ(dhour, 0U);
    EXPECT_EQ(dmin, 30U);

    // A +60-minute timezone offset shifts local time to 01:30 without altering
    // the underlying UTC clock.
    rtc.tz_min.set(60);
    uint8_t lhour, lminute, lsec;
    uint16_t lms;
    ASSERT_TRUE(rtc.get_local_time(lhour, lminute, lsec, lms));
    EXPECT_EQ(lhour, 1U);
    EXPECT_EQ(lminute, 30U);
}

// get_time_utc() returns the milliseconds until the next occurrence of a target
// time-of-day, ignoring leading -1 sentinels.  This covers every "largest
// element" branch and both the forward and negative-wrap paths (the latter by
// choosing a target earlier than the current time-of-day).
TEST(AP_RTC, TimeUntilTargetOfDay)
{
    static AP_RTC rtc;

    // No element specified -> immediate 0 (early return before clock lookup).
    EXPECT_EQ(rtc.get_time_utc(-1, -1, -1, -1), 0U);

    // With a target but no valid clock yet, get_system_clock_utc() fails -> 0.
    EXPECT_EQ(rtc.get_time_utc(-1, -1, -1, 500), 0U);

    // Accept a known 12:30:15 UTC time so each element has a definite "now".
    const uint64_t t_us =
        (1640995200ULL + 12ULL * 3600ULL + 30ULL * 60ULL + 15ULL) * 1000ULL * 1000ULL;
    rtc.set_utc_usec(t_us, AP_RTC::SOURCE_GPS);

    // Each result is always within one full period of its largest element, in
    // both the forward (target above now) and wrap (target below now) cases.
    EXPECT_LT(rtc.get_time_utc(-1, -1, -1, 999), 1000U);              // ms
    EXPECT_LT(rtc.get_time_utc(-1, -1, -1, 0), 1000U);               // ms wrap
    EXPECT_LT(rtc.get_time_utc(-1, -1, 59, -1), 60UL * 1000UL);      // sec fwd (59>15)
    EXPECT_LT(rtc.get_time_utc(-1, -1, 0, -1), 60UL * 1000UL);       // sec wrap (0<15)
    EXPECT_LT(rtc.get_time_utc(-1, 59, -1, -1), 60UL * 60UL * 1000UL);   // min fwd
    EXPECT_LT(rtc.get_time_utc(-1, 0, -1, -1), 60UL * 60UL * 1000UL);    // min wrap
    EXPECT_LT(rtc.get_time_utc(23, -1, -1, -1), 24UL * 60UL * 60UL * 1000UL);  // hour fwd
    EXPECT_LT(rtc.get_time_utc(0, -1, -1, -1), 24UL * 60UL * 60UL * 1000UL);   // hour wrap
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
