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
  status read, one conditional assignment, one subtraction and - only while the
  feature is enabled - one telemetry send.  It allocates nothing, takes no
  semaphore, owns no timer other than the single _last_good_ms latch, and keeps
  no history, so it is safe to drive from the existing 1Hz arming task.

  threshold_ms is the operator's configured limit in milliseconds, injected by
  the caller because the parameter is vehicle-owned rather than owned here.  A
  value of 0 disables the feature: nothing is gated and nothing is published.

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
    // the operator's current value
    _threshold_ms = threshold_ms;

#if AP_GPS_ENABLED
    // THE latch, and the correctness crux of this monitor: it is advanced from
    // the GPS *status*, never from the GPS message timestamp.  Subtracting the
    // GPS driver's own last-message time accessor from now is forbidden here,
    // because AP_GPS re-arms that very timer when its own fixed four-second
    // timeout expires, assigning the current time straight back into it.  The
    // resulting delta therefore saw-tooths forever, climbing to roughly 4000ms
    // and snapping back toward zero, so it could never express a staleness
    // beyond the driver's own timeout and a permanently dead receiver would
    // read as intermittently healthy.  The status does not reset that way:
    // under sustained starvation it stays at NO_FIX/NO_GPS, so the latch simply
    // stops advancing and the age computed below grows without bound.  See
    // AP_PNTFreshness.h for the full rationale.
    //
    // Note equally what is absent: no hdop, num_sats, variance or healthy()
    // qualifier.  Any of those would turn this delivery-cadence measure into a
    // solution-quality measure, which is what the EKF-backed position pre-arms
    // already provide.  GPS_OK_FIX_2D is the first usable-fix value of
    // AP_GPS::GPS_Status - the two states below it, NO_GPS and NO_FIX, are
    // exactly the ones that mean "no usable PNT was delivered".
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        _last_good_ms = now_ms;
    }
#endif  // AP_GPS_ENABLED

    // unsigned 32-bit arithmetic, which is wrap-safe across the 49.7-day
    // millisecond rollover; this is the same idiom the GPS driver itself uses.
    // Deliberately unclamped, unfiltered and free of any "now < last" special
    // case.  Two consequences are intended rather than tolerated: before the
    // first usable fix _last_good_ms is still zero, so the staleness reads as
    // the time since boot - the correct answer, since no usable PNT has ever
    // been delivered - and when a fix returns the latch jumps forward on the
    // next tick, collapsing the staleness to nearly zero with no hysteresis or
    // minimum hold time to delay recovery.
    _staleness_ms = now_ms - _last_good_ms;

#if HAL_GCS_ENABLED
    // publish the freshness on its own NAMED_VALUE_FLOAT channel, so any ground
    // station can watch it through a generic named-value view with no special
    // support, and so it lands in the dataflash log as a free NVF row.
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
