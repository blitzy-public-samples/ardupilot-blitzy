#include "AP_PNTFreshness.h"

#include "AP_Arming_config.h"

#if AP_ARMING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

void AP_PNTFreshness::update(uint32_t threshold_ms)
{
    // read the clock exactly once, so the latch and the subtraction below can
    // never disagree by a scheduling delay
    const uint32_t now_ms = AP_HAL::millis();

    // Advance the latch from primary-GPS fix status, not last_message_time_ms():
    // AP_GPS resets its message timer on timeout, so a delta taken from it
    // saw-tooths around 4000ms instead of growing.  Status may remain usable
    // until that fixed timeout expires, so this is age since usable status, not
    // exact message-delivery age.  No hdop/num_sats/variance qualifier belongs
    // here - EKF checks own solution-quality criteria.  GPS_OK_FIX_2D is the
    // first usable value of AP_GPS::GPS_Status; NO_GPS and NO_FIX are below it.
    bool pnt_delivered = false;
#if AP_GPS_ENABLED
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        pnt_delivered = true;
    }
#endif  // AP_GPS_ENABLED

    // the age this tick reports, computed in a local and published below with a
    // single store, so a reader on another thread never sees an intermediate value
    uint32_t age_ms;

    if (pnt_delivered) {
        _last_good_ms = now_ms;
        age_ms = 0;
    } else {
        // _last_good_ms is still zero before the first usable status, so this
        // reads as the time since boot: no usable PNT has ever been observed
        age_ms = now_ms - _last_good_ms;

        // max-hold: after a whole 2^32ms (49.7-day) period without usable
        // status the modular difference folds back through zero, and an age
        // that dropped to nearly nothing would reopen the gate on a vehicle
        // with no PNT at all.  Retaining the larger value saturates instead.
        // Reading the published word back is sound: this is its only writer
        const uint32_t published_ms = staleness_ms();
        if (age_ms < published_ms) {
            age_ms = published_ms;
        }
    }

    // publish the age: one relaxed store per tick, per the header's contract
    _staleness_ms.store(age_ms, std::memory_order_relaxed);

#if HAL_GCS_ENABLED
    // the existing helper broadcasts GPSFresh to every active channel and, where
    // logging is enabled, writes an NVF row.  Publishing only while enabled is
    // what keeps a default-configured vehicle unchanged in telemetry bandwidth
    // and log volume; the latch above keeps tracking either way.  Both the gate
    // and the value come from this call, so the send reads no shared state
    if (enabled(threshold_ms)) {
        gcs().send_named_float("GPSFresh", (float)age_ms);
    }
#endif  // HAL_GCS_ENABLED
}

#endif  // AP_ARMING_ENABLED
