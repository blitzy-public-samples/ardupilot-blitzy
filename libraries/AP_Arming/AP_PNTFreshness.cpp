#include "AP_PNTFreshness.h"

#include "AP_Arming_config.h"

#if AP_ARMING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

/*
  periodic update: latch the last usable PNT delivery, recompute the staleness
  from that latch, and publish the result.

  This is the whole of the monitor's behaviour, and it is deliberately tiny: one
  clock read, one status read, one subtraction, two comparisons and - only while
  the feature is enabled - one telemetry send.  It allocates nothing, takes no
  semaphore, owns no timer other than the single _last_good_ms latch, and keeps
  no history, so it is safe to drive from the existing 1Hz arming task.

  threshold_ms is the operator's configured limit in milliseconds, injected by
  the caller because the parameter is vehicle-owned rather than owned here.  A
  value of 0 disables the gate and the publication - nothing is gated and
  nothing is published - while the latch and the staleness are still maintained.

  Call at least 1Hz.  Because the in-tree caller runs at exactly 1Hz, the
  reported staleness has one-second resolution and the telemetry arrives once
  per second; thresholds below roughly 2000ms therefore cannot be resolved
  meaningfully.  That is an intentional trade for reusing an existing scheduler
  slot, and it suits a gate whose purpose is catching sustained delivery
  failure rather than sub-second jitter, so no faster internal timer is added
  here to work around it.
 */
void AP_PNTFreshness::update(uint32_t threshold_ms)
{
    // read the clock exactly once, so the latch and the subtraction below can
    // never disagree by a scheduling delay
    const uint32_t now_ms = AP_HAL::millis();

    // store the injected threshold first, before anything that could skip work:
    // the pre-arm check runs later in this same tick and reads the threshold
    // back through threshold_ms()/enabled()/is_stale(), so it must always see
    // the operator's current value.  The check re-synchronises it as well, for
    // the callers that arrive between ticks
    set_threshold_ms(threshold_ms);

    // THE status test, and the correctness crux of this monitor: the latch below
    // is advanced from the GPS *status*, never from the GPS message timestamp.
    // Subtracting the GPS driver's own last-message time accessor from now is
    // forbidden here, because AP_GPS re-arms that very timer when its own fixed
    // four-second timeout expires, assigning the current time straight back into
    // it.  The resulting delta therefore saw-tooths forever, climbing to roughly
    // 4000ms and snapping back toward zero, so it could never express a
    // staleness beyond the driver's own timeout and a permanently dead receiver
    // would read as intermittently healthy.  The status is never re-armed that
    // way: while delivery stays dead it remains below GPS_OK_FIX_2D - at NO_FIX,
    // or at NO_GPS while re-detection runs - so the test below simply stops
    // advancing _last_good_ms and the age computed further down keeps growing.
    // See AP_PNTFreshness.h for the full rationale.
    //
    // Note equally what is absent: no hdop, num_sats, variance or healthy()
    // qualifier.  Any of those would turn this delivery-cadence measure into a
    // solution-quality measure, which is what the EKF-backed position pre-arms
    // already provide.  GPS_OK_FIX_2D is the first usable-fix value of
    // AP_GPS::GPS_Status - the two states below it, NO_GPS and NO_FIX, are
    // exactly the ones that mean "no usable PNT was delivered".
    bool pnt_delivered = false;
#if AP_GPS_ENABLED
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        pnt_delivered = true;
    }
#endif  // AP_GPS_ENABLED

    if (pnt_delivered) {
        // a usable PNT solution is being delivered right now: re-arm THE latch
        // and report an age of zero.  A usable status is the ONLY event allowed
        // to lower the reported age, which is what makes recovery immediate -
        // no hysteresis, no filtering and no minimum hold time delay it - while
        // still keeping the reported age monotonic for as long as delivery is
        // actually broken.
        _last_good_ms = now_ms;
        _staleness_ms = 0;
    } else {
        // no usable PNT this tick, so the latch stayed put and the age is the
        // elapsed time since it last moved, in unsigned 32-bit arithmetic - the
        // same wrap-safe idiom the GPS driver itself uses, with no signed cast,
        // no widening and no "now < last" special case.  Before the very first
        // usable fix _last_good_ms is still zero, so this reads as the time
        // since boot, which is the correct answer: no usable PNT has ever been
        // delivered.
        const uint32_t age_ms = now_ms - _last_good_ms;

        // Adopt the new age only while it is still growing.  That is a no-op for
        // every interval shorter than one full 2^32ms (49.7-day) period - the
        // ordinary case, where this is exactly the plain subtraction - and it is
        // what stops the pathological case from failing the gate OPEN: once a
        // usable fix has been absent for a whole period (or was never delivered
        // at all and the vehicle has been powered that long), the modular
        // difference folds back through zero, and a reported age that dropped
        // from nearly UINT32_MAX to nearly nothing would make is_stale() return
        // false for up to the threshold duration and let a vehicle with no PNT
        // whatsoever pass a freshness gate that exists precisely to stop it.
        // Retaining the previous, larger value instead saturates the reading
        // just below UINT32_MAX and holds the gate shut, which is the correct
        // reading in every sense that matters: the true age is at least that
        // large, and an age measured in weeks is already unbounded for the
        // purpose of a delivery-cadence check.
        if (age_ms > _staleness_ms) {
            _staleness_ms = age_ms;
        }
    }

#if HAL_GCS_ENABLED
    // publish the freshness on its own NAMED_VALUE_FLOAT channel.  The helper
    // broadcasts to every active MAVLink channel over that existing generic
    // named-value transport, so no message-specific ground-station support is
    // needed to receive the value; whether and how a particular station then
    // surfaces it is that station's own affair, and support differs between
    // them.  On builds where logging is enabled the same helper also writes the
    // value out as a free NVF dataflash row.
    //
    // The send is gated on the feature being enabled, and that gate is
    // mandatory: at the shipped threshold default of 0 a vehicle must stay
    // indistinguishable from pre-feature firmware in behaviour, in telemetry
    // bandwidth and in log volume, and unconditional 1Hz traffic plus its NVF
    // rows would itself be an observable change.  Only the publication is
    // gated - the latch above keeps tracking regardless, so enabling the
    // parameter in flight yields a meaningful value immediately rather than a
    // spurious time-since-boot spike.
    //
    // "GPSFresh" is eight characters, inside the ten-character
    // NAMED_VALUE_FLOAT name field that both send paths enforce by silent
    // truncation, and it is the exact literal the SITL tests subscribe to.
    if (enabled()) {
        gcs().send_named_float("GPSFresh", (float)_staleness_ms);
    }
#endif  // HAL_GCS_ENABLED
}

#endif  // AP_ARMING_ENABLED
