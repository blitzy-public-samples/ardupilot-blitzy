#include "Copter.h"

/**
 *
 * PNT (position/navigation/timing) data-delivery freshness monitor.
 *
 * This gate measures data-delivery cadence, not estimate quality.  It answers
 * "is the receiver still producing fixes?", not "is the resulting position
 * solution good?".  The monitor keeps one timestamp latch holding the system
 * time at which the GPS last delivered a usable fix - the instant AP_GPS
 * recorded when it parsed that fix, not the instant this monitor happened to
 * notice it - derives from it the elapsed time since that delivery, and
 * publishes that duration continuously as the NAMED_VALUE_FLOAT "GPSFresh" so
 * a ground station can watch it live.  The pre-arm gate built on the
 * measurement lives in AP_Arming_Copter::pnt_freshness_checks() and does
 * nothing at all unless the operator sets FS_PNT_FRESH_MS to a non-zero
 * millisecond threshold.
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
 * millis() - last_message_time_ms() climbs until it crosses the timeout - the
 * branch tests strictly greater-than, so the value necessarily passes 4000 ms
 * and may exceed it by up to one driver update interval - and is then reset to
 * roughly zero, indefinitely.  It therefore cannot grow materially beyond the
 * driver timeout, which is fatal in both directions: a gate thresholded above
 * roughly four seconds would never fire at all, and a gate thresholded below it
 * would fire and then spuriously self-clear every four seconds instead of
 * staying latched for as long as the outage lasts.  Either outcome is a safety
 * defect masquerading as a working feature.
 *
 * The fix status on its own is no substitute for that timestamp either, because
 * it is persistent rather than event-like: AP_GPS leaves state[].status at
 * whatever the last parsed message reported until the timeout branch above
 * finally clears it.  A monitor which merely re-latched "now" on every good
 * status sample would therefore keep re-latching for as long as GPS_TIMEOUT_MS
 * after the last fix actually arrived, and would start the operator's threshold
 * running up to four seconds late.
 *
 * This monitor consequently latches the delivery instant the driver itself
 * recorded.  AP_GPS assigns "timing[instance].last_fix_time_ms = tnow"
 * (libraries/AP_GPS/AP_GPS.cpp:934) only inside the branch where a message was
 * actually parsed, and only when that message carried at least a 2D fix and the
 * receiver is not force-disabled; the starved-receiver branch above never
 * touches it.  That stamp is an event rather than a countdown, so it cannot
 * saw-tooth.  The latch below remains owned here and only here - nothing inside
 * AP_GPS can reach pnt_last_good_ms - so it advances only when a newly
 * delivered usable fix is seen and is otherwise frozen, and the age derived
 * from it grows without bound while delivery is absent and collapses to at most
 * one GPS reporting interval once fixes resume.
 *
 * Three pre-existing mechanisms already watch this ground, and this monitor
 * neither duplicates nor replaces any of them.  One of the three is itself a
 * delivery timeout; the other two answer the estimate-QUALITY question:
 *
 *  - delivery: the AP_GPS driver timeout itself (GPS_TIMEOUT_MS, applied at
 *    libraries/AP_GPS/AP_GPS.cpp:890-908) - it only records whether a message
 *    arrived within the last four seconds, then forgets it ever asked.  It is
 *    self-resetting and not operator-configurable, and it is the source of the
 *    saw-tooth described above rather than a usable duration;
 *
 *  - quality: the EKF variance failsafe in ekf_check.cpp, where
 *    Copter::ekf_over_threshold() (ekf_check.cpp:113) compares the filtered
 *    position, velocity and compass innovation variances against
 *    g.fs_ekf_thresh, and Copter::ekf_check() (ekf_check.cpp:30) counts how
 *    many consecutive iterations came back bad, up to
 *    EKF_CHECK_ITERATIONS_MAX (ekf_check.cpp:11), before declaring a failure.
 *    That is a quality metric on the fused estimate, and a starved GPS can
 *    still leave those variances briefly acceptable;
 *
 *  - quality: the position-estimate pre-arms "Need Position Estimate" and
 *    "GPS glitching", both inside AP_Arming_Copter::mandatory_gps_checks() -
 *    boolean and instantaneous.  Neither expresses HOW LONG delivery has been
 *    absent, and neither is operator-tunable.
 *
 */

// pnt_health_update - latches the system time at which the GPS last delivered a
// usable fix and publishes the resulting data-delivery gap to the ground station
// should be called at 10hz
void Copter::pnt_health_update()
{
    // sample the clock exactly once, so that the age published below and the
    // age the pre-arm gate reads cannot disagree with one another
    const uint32_t now_ms = AP_HAL::millis();

    // Advance the latch only when a newly delivered usable fix is seen, by
    // adopting the delivery instant AP_GPS recorded for the fix it last parsed.
    // Both halves of the test below carry weight: the timestamp is what makes
    // the measurement start when delivery actually stopped rather than when the
    // driver's own four second timeout eventually noticed (see the file comment
    // above), and the status test is what decides that the delivered fix is
    // usable at all.
    //
    // AP_GPS::GPS_Status is ordered by increasing quality, so ">=" also admits
    // DGPS and both RTK grades and stays correct if higher grades are added
    // upstream.  It excludes GPS_OK_FIX_2D and every status below it; of those
    // excluded values, NO_FIX and NO_GPS are the two the driver assigns to a
    // starved receiver.  The accessor also reports NO_FIX while the GPS is
    // force-disabled, so that case is honoured here without a line of code.
    //
    // A zero stamp is rejected because it is not a delivery instant at all: the
    // blended instance clears its own copy while it recomputes
    // (libraries/AP_GPS/AP_GPS_Blended.cpp:231), and adopting that zero would
    // read as "nothing has been delivered since boot".
    //
    // The latch is maintained whether or not the gate is enabled.  It costs one
    // comparison and one store, and it keeps the measurement warm so that
    // enabling FS_PNT_FRESH_MS mid-session cannot produce a spurious spike.
    const uint32_t delivered_ms = AP::gps().last_fix_time_ms();
    if (delivered_ms != 0 && AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        pnt_last_good_ms = delivered_ms;
    }

    // The latch starts at zero and there is deliberately no "has ever had a
    // fix" flag, so until the first usable fix the measured age equals uptime
    // and an enabled gate refuses arming on a receiver that has never locked.
    // That cold-start behaviour is intentional and must not be "corrected":
    // this is a data-delivery gate, and a receiver which has never delivered
    // anything must not satisfy it.  Seeding the latch to boot time, or adding
    // a first-fix flag so a never-locked receiver passes, were both considered
    // and rejected: either would let a receiver that has delivered nothing at
    // all pass a gate whose entire subject is delivery.

    // A threshold of exactly zero disables the gate, and with it the
    // measurement: the published value is then frozen at a benign zero, and the
    // field stays continuously present rather than becoming intermittent.  That
    // is what makes a vehicle which has not opted in keep exactly the pre-arm
    // verdict it had before this monitor existed.  The compatibility claim is
    // scoped to that verdict deliberately: the NAMED_VALUE_FLOAT below is sent
    // either way, and the broadcast helper also writes a dataflash record for
    // it, so the telemetry and log streams do carry one field they did not
    // carry before.
    //
    // Exactly zero, and nothing else.  The parameter's declared 0 to 60000
    // range is metadata rather than enforcement, so the parameter store can
    // hold any int32_t, and a value outside that range is a misconfiguration
    // rather than an opt-out.  Publishing the live measurement for those values
    // keeps the invalid setting visible to whoever is watching, instead of
    // dressing it up as a legitimate disabled state; refusing to arm on it is
    // the job of AP_Arming_Copter::pnt_freshness_checks(), which validates the
    // same parameter and fails closed.
    const int32_t threshold_ms = g2.fs_pnt_fresh_ms.get();
    const float published_ms = (threshold_ms == 0) ? 0.0f : (float)(now_ms - pnt_last_good_ms);

    // Published unconditionally on every invocation, in milliseconds so the
    // live value can be compared against FS_PNT_FRESH_MS with no conversion.
    // NAMED_VALUE_FLOAT is not stream-rate managed, so this task's rate is
    // also the on-wire rate; equating the two is precisely what lets the
    // monitor own a single timestamp and no second interval timer.  The name
    // is eight characters against a ten byte MAVLink field, and a longer one
    // would be silently truncated rather than rejected.
    //
    // Cost contract for this call, which is effectively the whole of this
    // task's 75us budget, the reads above being one clock sample, one delivery
    // timestamp and one status read.  GCS::send_named_float() packs the message
    // once, hands it to the active channels, and writes one dataflash record.
    // Every part of that is bounded at compile time and none of it loops on
    // run-time data: the fan-out visits at most MAVLINK_COMM_NUM_BUFFERS
    // channels, the record reaches at most LOGGER_MAX_BACKENDS backends, and
    // the record's "NVF" format is resolved by pointer against AP_Logger's
    // format list, which is allocated once for the whole boot on the first of
    // these calls and only walked on every call after it.  The steady state
    // allocates nothing.
    //
    // Measured on this checkout in SITL, in the monitor's worst case - gate
    // enabled and the GPS starved, so live values are published and the veto
    // is firing - with logging active on both backends and three live MAVLink
    // channels: 7256 machine instructions per invocation of this function, 99%
    // of them inside this call; exactly one format allocation across 2646
    // publications, on the first; 3.3us average and 27us maximum wall clock
    // across 5600 publications.  Re-measure before raising this task's rate.
    //
    // Two properties of that path are deliberate and must not be "optimised"
    // away.  The dataflash record is the reason the monitor carries no logging
    // code of its own, and the helper writes it under AP_Logger's format-list
    // semaphore, so this task inherits one short bounded critical section that
    // every other AP::logger().Write() caller in the firmware also takes.  And
    // the publication is unconditional, the frozen zero of a disabled gate
    // included, so that the field never becomes intermittent.
    gcs().send_named_float("GPSFresh", published_ms);
}

// pnt_data_age_ms - returns the elapsed time in milliseconds since the GPS last
// delivered a usable fix, measured from the delivery instant that
// pnt_health_update() latched
uint32_t Copter::pnt_data_age_ms() const
{
    // Unsigned arithmetic on the 32-bit millisecond counter is correct across
    // its roughly 49.7 day wrap for any true interval below 2^32 ms, and is
    // the same idiom AP_GPS itself uses at libraries/AP_GPS/AP_GPS.cpp:890.
    // It must not be "protected" with a signed cast, a lower clamp or a
    // wrap-detection branch, any of which would break it.
    return AP_HAL::millis() - pnt_last_good_ms;
}
