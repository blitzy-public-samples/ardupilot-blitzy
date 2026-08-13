#pragma once

// AP_PNTFreshness - a configurable, vehicle-level position/navigation/timing
// (PNT) DELIVERY-CADENCE gate.
//
// It answers exactly one question: how long ago was a usable PNT solution last
// delivered to the vehicle?  That age ("staleness") is compared against a
// vehicle-owned threshold parameter - FS_PNT_FRESH_MS, injected into update()
// by AP_Arming rather than owned here - in order to gate arming, and it is
// published continuously at low rate on its own NAMED_VALUE_FLOAT telemetry
// channel, GPSFresh, while that threshold is non-zero.  A threshold of zero
// disables the pre-arm gate and the telemetry publication: nothing is gated and
// nothing is published, so a default-configured vehicle stays indistinguishable
// from pre-feature firmware in behaviour, in telemetry bandwidth and in log
// volume.  Freshness tracking itself continues either way - update() always
// maintains the latch and the age below - so enabling the threshold at runtime
// yields an immediately meaningful age rather than a spurious time-since-boot
// spike.
//
// Note carefully what this measures.  It measures the RECENCY of delivery, not
// the goodness of the solution delivered.  The two mechanisms it most resembles
// are both something else, and the distinction is the point of the feature:
//
// This is NOT AP_GPS's driver timeout.
// AP_GPS carries a FIXED four-second timeout - "#define GPS_TIMEOUT_MS 4000u"
// in AP_GPS.cpp - whose expiry zeroes the instance state and flips the instance
// status to NO_FIX, or to NO_GPS after freeing the driver so that detection
// runs again.  That timeout is not configurable, is not exposed as a
// vehicle-level arming surface, and is not telemetered as a freshness value.
// This class is deliberately all three of those things: operator-settable,
// arming-gating, and telemetered.
//
// This is NOT a solution-quality check.
// The existing position-related pre-arms test the QUALITY of an estimate rather
// than the CADENCE of its delivery: "Need Position Estimate", gated on the
// vehicle's own position_ok() predicate, and "GPS glitching", driven by the
// EKF's nav_filter_status gps_glitching flag - both raised from the vehicle
// arming subclass and both backed by the EKF variance machinery in
// ArduCopter/ekf_check.cpp.  The two measures are orthogonal: a
// well-conditioned solution can still be arbitrarily old, and a freshly
// delivered one can be poor.  This gate is therefore additional to those
// checks, never a replacement for any of them.
//
// Why the latch is driven by GPS STATUS and never by a message timestamp.
// Deriving staleness from millis() - AP::gps().last_message_time_ms() is
// forbidden.  When AP_GPS detects its own timeout it RE-ARMS ITS OWN TIMER,
// assigning the current time straight back into last_message_time_ms.  The
// resulting delta therefore saw-tooths forever - climbing to roughly 4000ms,
// snapping back toward zero, and repeating - so it can never express a
// staleness greater than the driver's own timeout, and a permanently dead
// receiver reads as intermittently healthy.  AP_GPS::last_message_time_ms()
// must not be used for this purpose.
//
// The status carries no such reset.  _last_good_ms is the only latch here, and
// it simply stops advancing for as long as the status stays below
// GPS_OK_FIX_2D; it makes no difference whether the driver holds the status at
// NO_FIX or moves it between NO_GPS and NO_FIX while re-detection runs, because
// every value below GPS_OK_FIX_2D means "no usable PNT was delivered".  Under
// sustained starvation the reported staleness therefore keeps growing for as
// long as delivery stays broken.  That monotonic growth is precisely the
// property the SITL GPS-starvation test asserts, and it is what catches any
// regression back to the forbidden timestamp derivation.
//
// Monotonicity is guaranteed, not merely typical, and that guarantee is a
// safety property rather than a cosmetic one.  The age is held in a uint32_t
// millisecond count, so the modular difference from the latch would fold back
// through zero once a usable fix had been absent for a whole 2^32ms (49.7-day)
// period - or had never arrived on a vehicle powered that long.  An age that
// dropped from nearly UINT32_MAX to nearly nothing would make is_stale() report
// false for up to the threshold duration and let a vehicle with no PNT
// whatsoever pass the very gate that exists to stop it.  update() therefore
// only ever adopts a larger age while PNT is undelivered, so the reading
// SATURATES just below UINT32_MAX instead of wrapping, and the only event that
// can lower it is a usable GPS status re-arming the latch.

#include <stdint.h>

class AP_PNTFreshness
{
public:
    // periodic update function: latch the last usable fix, recompute the
    // staleness, and publish the freshness telemetry.  threshold_ms is the
    // operator's configured limit, injected by the caller because the parameter
    // is vehicle-owned; a value of 0 disables the gate and the publication
    // while the latch and the staleness keep being maintained.
    // call at least 1Hz
    void update(uint32_t threshold_ms);

    // accessor for the age of the last usable PNT solution, in milliseconds
    uint32_t staleness_ms() const { return _staleness_ms; }

    // accessor for the threshold last injected by the caller, in milliseconds
    // (0 == gate and publication disabled)
    uint32_t threshold_ms() const { return _threshold_ms; }

    // adopt the operator's threshold without touching the latch, the reported
    // age or the telemetry.  update() calls this, so it is the single place the
    // threshold is stored; the pre-arm check calls it too, because arming and
    // "run prearm checks" commands are serviced between the 1Hz updates and
    // must be judged against the parameter value in force at that moment, not
    // against the one cached up to a second earlier.
    void set_threshold_ms(uint32_t threshold_ms) { _threshold_ms = threshold_ms; }

    // accessor for the enable state: true once a non-zero threshold has been
    // configured
    bool enabled() const { return _threshold_ms > 0; }

    // accessor for the gate itself: true when the feature is enabled and the
    // last usable PNT solution is older than the configured threshold.  Folding
    // enabled() in here is what keeps the gate inert at a zero threshold, no
    // matter how large the staleness grows.
    bool is_stale() const { return enabled() && _staleness_ms > _threshold_ms; }

private:
    // THE latch, and the only timer this class owns: advanced to "now" only
    // while the GPS status reports a usable fix.  Zero-initialised, so before
    // the first usable fix the staleness reads as the time since boot, which is
    // the correct reading - no usable PNT has ever been delivered.
    uint32_t _last_good_ms = 0;

    // the threshold most recently injected by the caller, in milliseconds
    uint32_t _threshold_ms = 0;

    // the reported age of the last usable PNT solution: now - _last_good_ms,
    // evaluated in unsigned 32-bit arithmetic so that it stays wrap-safe across
    // the 49.7-day millisecond rollover of the clock, and never allowed to fall
    // while PNT is undelivered - so it saturates just below UINT32_MAX rather
    // than folding back through zero after a full period without a usable fix.
    // A usable GPS status re-arming the latch is the only event that lowers it.
    uint32_t _staleness_ms = 0;
};
