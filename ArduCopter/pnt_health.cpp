#include "Copter.h"

/**
 *
 * PNT (position/navigation/timing) data-delivery freshness monitor.
 *
 * This gate measures data-delivery cadence, not estimate quality: it answers
 * "is the receiver still producing fixes?", not "is the resulting position
 * solution good?".  The monitor keeps one timestamp latch holding the system time
 * at which it last saw the GPS reporting a usable fix, derives the elapsed time
 * since then, and publishes that duration continuously as the NAMED_VALUE_FLOAT
 * "GPSFresh" so a ground station can watch it live.  The pre-arm gate built on
 * the measurement lives in AP_Arming_Copter::pnt_freshness_checks() and does
 * nothing at all unless the operator sets FS_PNT_FRESH_MS to a non-zero
 * millisecond threshold.
 *
 * The measurement is deliberately NOT derived as
 * millis() - AP_GPS::last_message_time_ms().  AP_GPS re-stamps that timestamp
 * inside the starved-receiver branch its own GPS_TIMEOUT_MS opens, before
 * clearing the fix status, so any derivation from it saw-tooths between roughly
 * zero and that timeout instead of describing the outage: a threshold above the
 * timeout would never fire at all, and one below it would fire and then
 * spuriously self-clear on every driver cycle.
 *
 * What this monitor samples instead is the reported fix status, which is the same
 * quality vocabulary the gate is defined against: AP_GPS::GPS_Status is ordered
 * by increasing quality, and the two values that starved-receiver branch assigns
 * - NO_FIX and NO_GPS - both sit below the GPS_OK_FIX_3D floor required here.
 * The latch is written from this monitor's own clock sample, on this monitor's own
 * reading of that status, and by nothing else: nothing inside AP_GPS can reach
 * pnt_last_good_ms, so it advances only while the receiver is reporting a usable
 * fix and is otherwise frozen.
 *
 * AP_GPS::last_fix_time_ms() is deliberately not adopted either, even though -
 * unlike last_message_time_ms() - the starved-receiver branch leaves it alone.
 * It is stamped for any fix of at least 2D quality, so it does not describe the
 * 3D floor applied here; it carries the corrected receiver timestamp rather than
 * this monitor's own clock sample whenever a backend supplies one; and the
 * virtual blended instance recomputes it as a weighted average of the
 * per-receiver stamps, clearing it while it does so.  That last one also breaks
 * the pairing such a measurement would depend on, because the no-argument
 * accessors both answer for the primary instance and the primary is that virtual
 * instance whenever the operator selects GPS blending: its status is the HIGHEST
 * status across the receivers while its stamp is weight-AVERAGED over every
 * receiver carrying blend weight, and horizontal-accuracy weighting admits a
 * receiver at 2D.  A receiver still emitting 2D fixes would then drag the stamp
 * forward while a starved sibling went on supplying the 3D status, advancing the
 * latch on a delivery which was never usable and postponing the operator's veto.
 *
 * One consequence of sampling a status is worth knowing rather than discovering.
 * AP_GPS holds whatever the last parsed message reported until its own timeout
 * clears it, so a receiver which goes silent altogether is still reported as
 * fixed until that happens, and the measured age then begins up to
 * GPS_TIMEOUT_MS after delivery actually stopped; a receiver which keeps talking
 * while losing lock - the common real case, and the one SITL reproduces when
 * SIM_GPS1_ENABLE is cleared - drops its reported status within a reporting
 * interval instead.  Either way the offset is bounded by that timeout, it is
 * small against a threshold an operator sets in whole seconds, and it delays the
 * gate rather than defeating it, because the age goes on growing for as long as
 * the outage lasts once the latch freezes.  That is exactly the property the
 * forbidden derivation above lacks.
 *
 * None of this duplicates or replaces the mechanisms already watching this
 * ground.  One of them is itself a delivery timeout - the AP_GPS driver timeout,
 * which is self-resetting, not operator-configurable and the source of the
 * saw-tooth above.  The other two answer the estimate-quality question: the EKF
 * variance failsafe in ekf_check.cpp, where ekf_over_threshold() compares the
 * filtered innovation variances against g.fs_ekf_thresh and ekf_check() counts
 * consecutive bad iterations before declaring a failure, and the "Need Position
 * Estimate" and "GPS glitching" pre-arms inside
 * AP_Arming_Copter::mandatory_gps_checks(), which are boolean and instantaneous.
 * None of the three expresses how long delivery has been absent, and none is
 * operator-tunable.
 *
 * Two limits of a cadence measurement are worth stating so that nothing is read
 * into it which is not there.  It says nothing about whether the fixes being
 * delivered are genuine: a fabricated or spoofed fix reporting at least 3D
 * quality refreshes the latch exactly as a real one does, so this monitor stays
 * silent through it.  Detecting that is a different problem, and so is judging
 * the resulting solution, which is what the quality mechanisms named above are
 * for.  And because the gate keys on the configured threshold rather than on the
 * flight mode, an operator who opts in is refused arming in modes which do not
 * themselves need GPS whenever the receiver is stale or has never locked - unlike
 * AP_Arming_Copter::gps_checks(), which consults the mode first.  That follows
 * from what the gate is for and is not a defect, but it is worth knowing before
 * setting FS_PNT_FRESH_MS on a vehicle flown in Stabilize or Acro.
 *
 */

// pnt_health_update - latches the system time at which the GPS was last seen
// reporting a usable fix and publishes the resulting data-delivery gap to the
// ground station
// should be called at 10hz
void Copter::pnt_health_update()
{
    // one clock sample drives the latch written below and the value published from
    // it; the pre-arm gate reads the age through pnt_data_age_ms(), which samples
    // the clock independently
    const uint32_t now_ms = AP_HAL::millis();

    // Advance the latch while, and only while, the receiver is reporting a usable
    // fix, and advance it to this monitor's own clock sample - never to a timestamp
    // AP_GPS owns, for the reasons given in the file comment above.  GPS_Status is
    // ordered by increasing quality, so ">=" also admits DGPS and both RTK grades
    // and stays correct if higher grades are added upstream; it excludes
    // GPS_OK_FIX_2D and every status below it, and of those excluded values NO_FIX
    // and NO_GPS are the two the driver assigns to a starved receiver.  The
    // accessor also reports NO_FIX while the GPS is force-disabled, so that case is
    // honoured here without a line of code.  The latch is maintained whether or not
    // the gate is enabled, which keeps the measurement warm so that enabling
    // FS_PNT_FRESH_MS mid-session cannot produce a spurious spike.
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        pnt_last_good_ms = now_ms;
    }

    // The latch starts at zero and there is deliberately no "has ever had a fix"
    // flag, so until the first usable fix the measured age is the uptime and an
    // enabled gate refuses arming on a receiver which has never locked.  That is
    // the intended reading of a data-delivery gate - a receiver which has delivered
    // nothing must not satisfy it - and must not be "corrected".

    // A threshold of zero disables the veto and freezes the published value at a
    // benign zero, while the latch above and pnt_data_age_ms() keep tracking as
    // usual; the field therefore stays continuously present rather than becoming
    // intermittent.  The threshold is tested the same way the gate itself tests it
    // in AP_Arming_Copter::pnt_freshness_checks() - at or below zero means "not
    // opted in", since the declared 0 to 60000 range is metadata rather than
    // enforcement and a raw PARAM_SET can store a negative value the operator was
    // never offered.  Sharing one rule between the two readers is what makes the
    // frozen zero mean exactly "the gate is inert" and nothing else - which is
    // also why the disabled reading cannot be told from a perfectly fresh one by
    // looking at the field: on a vehicle which has not opted in, what GPSFresh
    // reports is the state of the gate rather than the state of the receiver.
    const int32_t threshold_ms = g2.fs_pnt_fresh_ms.get();
    const float published_ms = (threshold_ms <= 0) ? 0.0f : (float)(now_ms - pnt_last_good_ms);

    // Published unconditionally on every invocation - the frozen zero of a disabled
    // gate included, so the field never becomes intermittent - in milliseconds so
    // the live value can be compared against FS_PNT_FRESH_MS with no conversion.
    // NAMED_VALUE_FLOAT is not stream-rate managed, so this task's rate is also the
    // on-wire rate; equating the two is what lets the monitor own a single
    // timestamp and no second interval timer.
    //
    // The broadcast helper is effectively the whole of this task's 75us budget,
    // everything above it being one clock sample, one status read, one parameter
    // read and one subtraction.  It packs the message once, hands it to the active
    // channels and writes one dataflash record - which is why this monitor carries
    // no logging code of its own.  Resolving that record's format walks the
    // logger's list of Write() names, which is bounded by the fixed set of such
    // names the firmware contains and is appended to only on the first call for
    // each name in a boot, so the steady state allocates nothing; the short
    // critical section that walk takes is the same one every other logging caller
    // in the firmware takes.  Measure this call before raising the task's rate,
    // since that rate is also the on-wire and on-disk rate.
    //
    // The name is eight characters against the message's ten byte field, checked at
    // compile time rather than trusted, because MAVLink truncates an over-long name
    // silently instead of rejecting it.
    static_assert(sizeof("GPSFresh") - 1 <= MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN,
                  "GPSFresh must fit the NAMED_VALUE_FLOAT name field");
    gcs().send_named_float("GPSFresh", published_ms);
}

// pnt_data_age_ms - returns the elapsed time in milliseconds since the GPS was
// last seen reporting a usable fix, measured from the latch that
// pnt_health_update() maintains
uint32_t Copter::pnt_data_age_ms() const
{
    // Unsigned arithmetic on the 32-bit millisecond counter is correct across its
    // roughly 49.7 day wrap for any true interval below 2^32 ms, and is the same
    // idiom AP_GPS itself uses.  It must not be "protected" with a signed cast, a
    // lower clamp or a wrap-detection branch, any of which would break it.
    return AP_HAL::millis() - pnt_last_good_ms;
}
