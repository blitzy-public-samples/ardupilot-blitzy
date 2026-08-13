#include "Copter.h"

/**
 *
 * PNT (position/navigation/timing) data-delivery freshness monitor.
 *
 * This gate measures data-delivery cadence, not estimate quality.  It answers
 * "is the receiver still producing fixes?", not "is the resulting position
 * solution good?".  The monitor keeps one timestamp latch holding the moment
 * the GPS last reported a usable fix, derives from it the elapsed time since
 * delivery was last seen, and publishes that duration continuously as the
 * NAMED_VALUE_FLOAT "GPSFresh" so a ground station can watch it live.  The
 * pre-arm gate built on the measurement lives in
 * AP_Arming_Copter::pnt_freshness_checks() and does nothing at all unless the
 * operator sets FS_PNT_FRESH_MS to a non-zero millisecond threshold.
 *
 * Why the latch is owned here, and why the staleness is deliberately NOT
 * derived as millis() - AP_GPS::last_message_time_ms():
 *
 * The AP_GPS driver defines its own delivery timeout as
 * "#define GPS_TIMEOUT_MS 4000u" (libraries/AP_GPS/AP_GPS.cpp:74) and enters
 * its starved-receiver branch when
 * "tnow - timing[instance].last_message_time_ms > GPS_TIMEOUT_MS"
 * (libraries/AP_GPS/AP_GPS.cpp:890).  Inside that same branch the driver
 * re-stamps its own timestamp with
 * "timing[instance].last_message_time_ms = tnow"
 * (libraries/AP_GPS/AP_GPS.cpp:895), before setting the fix status to NO_FIX
 * for MAVLink and UAVCAN backed instances
 * (libraries/AP_GPS/AP_GPS.cpp:902), or to NO_GPS for serial instances whose
 * driver is then deleted so that detection runs again
 * (libraries/AP_GPS/AP_GPS.cpp:908).
 *
 * The consequence is a saw-tooth.  While the receiver is starved,
 * millis() - last_message_time_ms() climbs to roughly 4000 ms, snaps back to
 * roughly zero, and repeats indefinitely - it can never exceed the driver
 * timeout.  A gate thresholded above roughly four seconds would therefore
 * never fire at all, and a gate thresholded below it would fire and then
 * spuriously self-clear every four seconds.  Either outcome is a safety
 * defect masquerading as a working feature.  The latch below escapes this
 * because it belongs to the monitor: nothing inside AP_GPS can reach it, so
 * it only ever advances on a good sample and is otherwise frozen, and the
 * age derived from it therefore grows without bound while delivery is absent
 * and collapses within one task period once fixes resume.
 *
 * Three pre-existing mechanisms answer the estimate-QUALITY question instead.
 * This monitor neither duplicates nor replaces any of them, and it is not a
 * fourth variant of them:
 *
 *  - the AP_GPS driver timeout itself (GPS_TIMEOUT_MS, applied at
 *    libraries/AP_GPS/AP_GPS.cpp:890-908) - it only records whether a message
 *    arrived within the last four seconds, then forgets it ever asked.  It is
 *    self-resetting and not operator-configurable, and it is the source of the
 *    saw-tooth described above rather than a usable duration;
 *
 *  - the EKF variance failsafe, Copter::ekf_check() and
 *    Copter::ekf_over_threshold() in ekf_check.cpp, which count filter
 *    innovation variances over EKF_CHECK_ITERATIONS_MAX iterations against
 *    g.fs_ekf_thresh - a quality metric on the fused estimate.  A starved GPS
 *    can still leave those variances briefly acceptable;
 *
 *  - the position-estimate pre-arms "Need Position Estimate" and
 *    "GPS glitching", both inside AP_Arming_Copter::mandatory_gps_checks() -
 *    boolean and instantaneous.  Neither expresses HOW LONG delivery has been
 *    absent, and neither is operator-tunable.
 *
 */

// pnt_health_update - latches the time of the most recent usable GPS fix and
// publishes the resulting data-delivery gap to the ground station
// should be called at 10hz
void Copter::pnt_health_update()
{
    // sample the clock exactly once, so that the latch written below and the
    // age derived from it cannot disagree with one another
    const uint32_t now_ms = AP_HAL::millis();

    // Advance the latch only while the receiver is delivering a usable fix.
    // The fix status is consulted rather than any driver timestamp, for the
    // reason given in the file comment above.  AP_GPS::GPS_Status is ordered
    // by increasing quality, so ">=" also admits DGPS and both RTK grades and
    // stays correct if higher grades are added upstream; the two values it
    // excludes are exactly the two the driver assigns to a starved receiver.
    // The accessor also reports NO_FIX while the GPS is force-disabled, so
    // that case is honoured here without a line of code.
    //
    // The latch is maintained whether or not the gate is enabled.  It costs
    // one comparison and one store, and it keeps the measurement warm so that
    // enabling FS_PNT_FRESH_MS mid-session cannot produce a spurious spike.
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        pnt_last_good_ms = now_ms;
    }

    // The latch starts at zero and there is deliberately no "has ever had a
    // fix" flag, so until the first usable fix the measured age equals uptime
    // and an enabled gate refuses arming on a receiver that has never locked.
    // That cold-start behaviour is intentional and must not be "corrected":
    // this is a data-delivery gate, and a receiver which has never delivered
    // anything must not satisfy it.  Seeding the latch to boot time, or adding
    // a first-fix flag so a never-locked receiver passes, were both considered
    // and rejected - each adds state and weakens the gate.

    // A threshold of zero disables the gate, and with it the measurement:
    // the published value is then frozen at a benign zero so that a vehicle
    // which has not opted in behaves observationally as it did before this
    // monitor existed, while the field itself stays continuously present.
    const int32_t threshold_ms = g2.fs_pnt_fresh_ms.get();
    const float published_ms = (threshold_ms > 0) ? (float)(now_ms - pnt_last_good_ms) : 0.0f;

    // Published unconditionally on every invocation, in milliseconds so the
    // live value can be compared against FS_PNT_FRESH_MS with no conversion.
    // NAMED_VALUE_FLOAT is not stream-rate managed, so this task's rate is
    // also the on-wire rate; equating the two is precisely what lets the
    // monitor own a single timestamp and no second interval timer.  The name
    // is eight characters against a ten byte MAVLink field, and a longer one
    // would be silently truncated rather than rejected.
    gcs().send_named_float("GPSFresh", published_ms);
}

// pnt_data_age_ms - returns the elapsed time in milliseconds since the GPS last
// reported a usable fix, as latched by pnt_health_update()
uint32_t Copter::pnt_data_age_ms() const
{
    // Unsigned arithmetic on the 32-bit millisecond counter is correct across
    // its roughly 49.7 day wrap for any true interval below 2^32 ms, and is
    // the same idiom AP_GPS itself uses at libraries/AP_GPS/AP_GPS.cpp:890.
    // It must not be "protected" with a signed cast, a lower clamp or a
    // wrap-detection branch, any of which would break it.
    return AP_HAL::millis() - pnt_last_good_ms;
}
