#pragma once

// AP_PNTFreshness - tracks the time since the primary GPS status last indicated
// a usable position/navigation/timing (PNT) fix, for the FS_PNT_FRESH_MS arming
// gate and the GPSFresh NAMED_VALUE_FLOAT telemetry.  The threshold parameter
// and the arming check itself belong to the vehicles and to AP_Arming.
//
// This is a delivery-cadence proxy, not an EKF/position-estimate quality check.
// AP_GPS may retain a usable status until its own fixed four-second timeout
// expires, so a dead receiver is detected up to that timeout late; unlike that
// driver timeout this gate is configurable, gates arming and is telemetered.
// Do not derive the age from AP_GPS::last_message_time_ms(): AP_GPS resets that
// timer on timeout, so the delta saw-tooths around 4000ms instead of growing.
//
// A zero threshold disables gating and publication while tracking continues.
// The age is max-held across uint32_t rollover until usable status returns.
//
// THREADING CONTRACT - read this before adding state here.  update() is the
// single writer and runs only on the main thread, at 1Hz from AP_Arming; it owns
// _last_good_ms outright and publishes the age with exactly one relaxed atomic
// store per tick.  The reads are safe from any thread because the arming checks
// that consult them are not confined to the main thread: pre_arm_checks() is
// reachable from the DDS worker thread and from the Lua arming:pre_arm_checks()
// binding.  Relaxed ordering suffices - the age is a self-contained scalar that
// publishes nothing alongside it - and no semaphore is taken, which would only
// have to be held across update()'s GCS send.  The threshold is deliberately not
// stored: it is passed in by whichever thread is asking, so the check path
// writes no shared state.  A stored threshold would be written from two threads
// and could read back as 0, i.e. disabled, passing a vehicle with stale PNT.
// Keep it an argument; never latch it into a member and never add a setter.

#include <stdint.h>

// for the one word this class publishes across threads; the same
// std::atomic<uint32_t> idiom AP_HAL's ring buffers already rely on
#include <atomic>

class AP_PNTFreshness
{
public:
    // update the status latch and publish GPSFresh when threshold_ms is non-zero
    // call at least 1Hz, from the main thread only - it is the single writer
    void update(uint32_t threshold_ms);

    // safe from any thread: one atomic load of the word update() publishes
    uint32_t staleness_ms() const { return _staleness_ms.load(std::memory_order_relaxed); }

    // the enable rule, in one place: zero disables the gate and the publication
    static bool enabled(uint32_t threshold_ms) { return threshold_ms > 0; }

    // threshold_ms is the caller's own value, never stored here
    bool is_stale(uint32_t threshold_ms) const
    {
        return enabled(threshold_ms) && staleness_ms() > threshold_ms;
    }

private:
    // advanced only while the primary GPS status is usable; zero means no
    // usable status has been observed.  Owned exclusively by update()
    uint32_t _last_good_ms = 0;

    // max-held while the status stays unusable, so it saturates rather than
    // folding back through zero on uint32_t rollover; only a usable status
    // resets it.  The single datum crossing threads, hence the atomic
    std::atomic<uint32_t> _staleness_ms { 0 };
};
