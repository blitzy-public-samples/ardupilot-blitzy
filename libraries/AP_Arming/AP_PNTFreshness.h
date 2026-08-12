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
// disables the object entirely: nothing is gated and nothing is published, so a
// default-configured vehicle stays indistinguishable from pre-feature firmware
// in behaviour, in telemetry bandwidth and in log volume.
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
// The status, by contrast, latches rather than resetting: under sustained
// starvation it stays at NO_FIX/NO_GPS, so _last_good_ms stops advancing and
// the reported staleness grows without bound.  That unbounded, monotonic growth
// is precisely the property the SITL GPS-starvation test asserts, and it is
// what catches any regression back to the forbidden timestamp derivation.

#include <stdint.h>

class AP_PNTFreshness
{
public:
    // periodic update function: latch the last usable fix, recompute the
    // staleness, and publish the freshness telemetry.  threshold_ms is the
    // operator's configured limit, injected by the caller because the parameter
    // is vehicle-owned; a value of 0 disables the feature.
    // call at least 1Hz
    void update(uint32_t threshold_ms);

    // accessor for the age of the last usable PNT solution, in milliseconds
    uint32_t staleness_ms() const { return _staleness_ms; }

    // accessor for the threshold last injected into update(), in milliseconds
    // (0 == feature disabled)
    uint32_t threshold_ms() const { return _threshold_ms; }

    // accessor for the enable state: true once a non-zero threshold has been
    // configured
    bool enabled() const { return _threshold_ms > 0; }

    // accessor for the gate itself: true when the feature is enabled and the
    // last usable PNT solution is older than the configured threshold.  Folding
    // enabled() in here is what keeps a zero threshold completely inert, no
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

    // now - _last_good_ms, evaluated in unsigned 32-bit arithmetic so that it
    // stays wrap-safe across the 49.7-day millisecond rollover
    uint32_t _staleness_ms = 0;
};
