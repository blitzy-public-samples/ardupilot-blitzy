// ============================================================================
//  test_afsim_l1.cpp -- Unit test suite for the AfsimL1 reusable L1 guidance
//                       service (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  ROLE (AAP 0.5.1 -- "unit test for the facade"):
//    Verifies all three layers of the extracted service plus the additive
//    AP_L1_Control timing seam:
//
//      Group 1  AfsimL1_AHRS_Shim  -- the Adapter: the six AP_AHRS read APIs,
//                                    the four injection setters and the
//                                    North/East <-> East/North conventions.
//      Group 2  AfsimL1Behavior    -- the Facade: leg/state injection, execute(),
//                                    the two output getters and their exact
//                                    relationship to the wrapped controller.
//      Group 3  AP_L1_Control seam -- BEHAVIOR PRESERVATION of the timing seam:
//                                    the injected dt is used, the seam is
//                                    default-off, the dt>1 s integrator reset
//                                    and the dt>0.1 s cap are unchanged, invalid
//                                    host dt is rejected, and the loiter path
//                                    consumes the injected timebase.
//      Group 4  extern "C" ABI     -- the Boundary: handle lifecycle, NULL /
//                                    bogus / stale-handle safety and end-to-end
//                                    guidance through the flat C entry points.
//
//  DEPENDENCIES: none beyond the service itself. AAP 0.5.2 mandates "no new
//    third-party dependency", so this file deliberately does NOT use GoogleTest;
//    it carries a small, self-contained assertion harness instead. It is built
//    and run by the sibling CMakeLists.txt:
//
//        mkdir build && cd build && cmake .. && make && ./afsim_l1_tests
//        # or, equivalently:  ctest --output-on-failure
//
//  DETERMINISM: every assertion is derived from fixed injected state and a
//    host-supplied dt, so there is no dependence on wall-clock timing anywhere
//    except the two checks that deliberately exercise the default (un-injected)
//    AP_HAL::micros() path, and those are written as inequalities with generous
//    margins so they hold on any clock implementation.
//
//  EXIT STATUS: 0 only when every check passes; the number of failures is
//    printed and returned as a non-zero status otherwise, so the binary works
//    directly as a CI gate.
// ============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>     // nanosleep -- deliberate delay for the default-off clock proof
#include <string>

#include "AfsimL1Behavior.h"
#include "AfsimL1_AHRS_Shim.h"
#include "l1_c_api.h"

// ---------------------------------------------------------------------------
// Minimal assertion harness
// ---------------------------------------------------------------------------
namespace {

int g_checks = 0;
int g_failures = 0;
const char *g_group = "";

void begin_group(const char *name)
{
    g_group = name;
    std::printf("\n--- %s ---\n", name);
}

void report(bool ok, const std::string &what, const std::string &detail)
{
    g_checks++;
    if (ok) {
        std::printf("  ok   : %s\n", what.c_str());
        return;
    }
    g_failures++;
    std::printf("  FAIL : %s [%s] (%s)\n", what.c_str(), g_group, detail.c_str());
}

void check(bool cond, const char *what)
{
    report(cond, what, cond ? "" : "condition is false");
}

std::string fmt(double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return std::string(buf);
}

void check_near(double actual, double expected, double tol, const char *what)
{
    const bool ok = std::isfinite(actual) && std::fabs(actual - expected) <= tol;
    report(ok, what, "actual=" + fmt(actual) + " expected=" + fmt(expected) +
                         " tol=" + fmt(tol));
}

void check_exact(double actual, double expected, const char *what)
{
    const bool ok = (actual == expected);
    report(ok, what, "actual=" + fmt(actual) + " expected=" + fmt(expected));
}

void check_finite(double v, const char *what)
{
    report(std::isfinite(v), what, "value=" + fmt(v));
}

void check_gt(double actual, double bound, const char *what)
{
    const bool ok = std::isfinite(actual) && actual > bound;
    report(ok, what, "actual=" + fmt(actual) + " must be > " + fmt(bound));
}

void check_lt(double actual, double bound, const char *what)
{
    const bool ok = std::isfinite(actual) && actual < bound;
    report(ok, what, "actual=" + fmt(actual) + " must be < " + fmt(bound));
}

// ---------------------------------------------------------------------------
// Shared fixtures
// ---------------------------------------------------------------------------

// A leg running due North for 500 m from the service datum.
void north_leg(Location &prev, Location &next)
{
    prev = Location();
    next = Location();
    next.offset(500.0f, 0.0f);
}

// A loiter centre at the datum.
Location datum_location()
{
    return Location();
}

// Drive a controller for `steps` iterations at a fixed injected dt while the
// vehicle sits at a fixed state, and return the cross-track integrator.
float wind_up_integrator(AP_L1_Control &l1, AfsimL1_AHRS_Shim &ahrs, float dt,
                         int steps, float north_m, float east_m)
{
    Location prev, next;
    north_leg(prev, next);
    ahrs.set_location_NE(north_m, east_m);
    ahrs.set_velocity_EN(0.0f, 20.0f);   // 20 m/s due North
    ahrs.set_yaw_cd(0.0f);
    ahrs.set_pitch_rad(0.0f);
    for (int i = 0; i < steps; i++) {
        l1.set_update_dt(dt);
        l1.update_waypoint(prev, next);
    }
    return l1.crosstrack_error_integrator();
}

// ===========================================================================
// GROUP 1 -- AfsimL1_AHRS_Shim (Adapter layer)
// ===========================================================================
void test_shim()
{
    begin_group("Group 1: AfsimL1_AHRS_Shim adapter");

    AfsimL1_AHRS_Shim ahrs;

    // 1.1 Defaults: a freshly constructed shim must present a well-defined,
    // all-zero state with a unity airspeed ratio so the controller can never
    // read indeterminate values before the host injects anything.
    Location loc;
    check(ahrs.get_location(loc), "get_location() reports success");
    check_exact(ahrs.get_yaw_rad(), 0.0, "default yaw_rad == 0");
    check_exact(ahrs.get_pitch_rad(), 0.0, "default pitch_rad == 0");
    check_exact(ahrs.get_EAS2TAS(), 1.0, "default EAS2TAS == 1");
    check_exact(ahrs.yaw_sensor, 0.0, "default yaw_sensor == 0");
    check_exact(ahrs.groundspeed_vector().x, 0.0, "default groundspeed .x == 0");
    check_exact(ahrs.groundspeed_vector().y, 0.0, "default groundspeed .y == 0");

    // 1.2 Position injection: the synthesised Location must sit exactly at the
    // injected North/East offset from the datum. Verified through the real
    // ArduPilot geometry (Location::get_distance_NE), i.e. round-tripping
    // metres -> lat/lng -> metres.
    const Location datum = datum_location();
    ahrs.set_location_NE(123.0f, -45.0f);
    check(ahrs.get_location(loc), "get_location() after set_location_NE");
    Vector2f ne = datum.get_distance_NE(loc);
    check_near(ne.x, 123.0, 0.05, "injected North offset round-trips (m)");
    check_near(ne.y, -45.0, 0.05, "injected East offset round-trips (m)");

    // 1.3 Velocity convention: set_velocity_EN takes (East, North) -- matching
    // the user example's argument order -- while AP_AHRS::groundspeed_vector()
    // returns (North, East). Getting this backwards is a silent 90-degree
    // guidance error, so it is asserted explicitly.
    ahrs.set_velocity_EN(7.0f, 24.0f);
    check_exact(ahrs.groundspeed_vector().x, 24.0, "groundspeed .x carries NORTH");
    check_exact(ahrs.groundspeed_vector().y, 7.0, "groundspeed .y carries EAST");

    // 1.4 Yaw injection in centidegrees, exposed both as radians and as the
    // centidegree AP_AHRS::yaw_sensor field the heading-hold path reads.
    ahrs.set_yaw_cd(9000.0f);
    check_near(ahrs.get_yaw_rad(), M_PI / 2.0, 1e-6, "9000 cd == pi/2 rad");
    check_exact(ahrs.yaw_sensor, 9000.0, "yaw_sensor == 9000 cd");

    // 1.5 Yaw wrapping into [0, 36000) cd for negative and over-range inputs.
    ahrs.set_yaw_cd(-9000.0f);
    check_exact(ahrs.yaw_sensor, 27000.0, "-9000 cd wraps to 27000 cd");
    check_near(ahrs.get_yaw_rad(), 3.0 * M_PI / 2.0, 1e-6, "-9000 cd == 3pi/2 rad");
    ahrs.set_yaw_cd(45000.0f);
    check_exact(ahrs.yaw_sensor, 9000.0, "45000 cd wraps to 9000 cd");

    // 1.6 Non-finite yaw is rejected at the adapter boundary (CWE-20): a NaN
    // heading would otherwise propagate through every trigonometric term.
    ahrs.set_yaw_cd(std::nanf(""));
    check_exact(ahrs.yaw_sensor, 0.0, "NaN yaw_cd falls back to 0 cd");
    check_exact(ahrs.get_yaw_rad(), 0.0, "NaN yaw_cd falls back to 0 rad");

    // 1.7 Pitch injection round-trip.
    ahrs.set_pitch_rad(0.25f);
    check_near(ahrs.get_pitch_rad(), 0.25, 1e-7, "pitch_rad round-trips");

    // 1.8 Repeated injection overwrites rather than accumulates.
    ahrs.set_location_NE(10.0f, 20.0f);
    ahrs.set_location_NE(30.0f, 40.0f);
    check(ahrs.get_location(loc), "get_location() after re-injection");
    ne = datum.get_distance_NE(loc);
    check_near(ne.x, 30.0, 0.05, "position overwrites (North)");
    check_near(ne.y, 40.0, 0.05, "position overwrites (East)");
}

// ===========================================================================
// GROUP 2 -- AfsimL1Behavior (Facade layer)
// ===========================================================================
void test_facade()
{
    begin_group("Group 2: AfsimL1Behavior facade");

    // 2.1 Construction and init() must leave the service in a usable state and
    // produce finite outputs before any host state has been injected.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.execute(0.02);
        check_finite(svc.get_roll_deg(), "roll finite after init()+execute()");
        check_finite(svc.get_lat_accel(), "lat_accel finite after init()+execute()");
    }

    // 2.2 On-track geometry: sitting exactly on a North leg while tracking
    // North must demand (essentially) zero bank.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.set_state_ne(100.0, 0.0, 0.0, 20.0, 0.0, 0.0);
        svc.execute(0.02);
        check_near(svc.get_roll_deg(), 0.0, 0.05, "on-track roll ~ 0 deg");
        check_near(svc.get_lat_accel(), 0.0, 0.05, "on-track lat_accel ~ 0");
    }

    // 2.3 Symmetry: mirrored cross-track offsets must produce exactly equal and
    // opposite commands. This is the sharpest available check that host state
    // really reaches the controller and that no sign convention is inverted.
    {
        AfsimL1Behavior east;
        AfsimL1Behavior west;
        east.init();
        west.init();
        east.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        west.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        east.set_state_ne(0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
        west.set_state_ne(0.0, -50.0, 0.0, 20.0, 0.0, 0.0);
        east.execute(0.02);
        west.execute(0.02);

        check_gt(std::fabs(east.get_roll_deg()), 1.0,
                 "50 m cross-track produces material bank");
        check_lt(east.get_roll_deg(), 0.0,
                 "East of a North leg banks left (negative roll)");
        check_near(east.get_roll_deg() + west.get_roll_deg(), 0.0, 1e-6,
                   "mirrored rolls are equal and opposite");
        check_near(east.get_lat_accel() + west.get_lat_accel(), 0.0, 1e-6,
                   "mirrored lat_accels are equal and opposite");
    }

    // 2.4 The facade getters are exactly nav_roll_cd()/100 and
    // lateral_acceleration(). Verified against an independently driven
    // AP_L1_Control fed identical inputs, so any unit slip (centidegrees vs
    // degrees) or wrong accessor is caught.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.set_state_ne(0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
        svc.execute(0.02);

        AfsimL1_AHRS_Shim ahrs;
        AP_L1_Control l1(ahrs, nullptr);
        l1.set_default_period(17.0f);
        Location prev, next;
        north_leg(prev, next);
        ahrs.set_location_NE(0.0f, 50.0f);
        ahrs.set_velocity_EN(0.0f, 20.0f);
        ahrs.set_yaw_cd(0.0f);
        ahrs.set_pitch_rad(0.0f);
        l1.set_update_dt(0.02f);
        l1.update_waypoint(prev, next);

        check_exact(svc.get_roll_deg(), (double)(l1.nav_roll_cd() / 100.0f),
                    "get_roll_deg() == nav_roll_cd()/100");
        check_exact(svc.get_lat_accel(), (double)l1.lateral_acceleration(),
                    "get_lat_accel() == lateral_acceleration()");
    }

    // 2.5 set_leg_ne() genuinely re-targets: reversing the leg direction while
    // holding the state fixed must flip the commanded bank.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_state_ne(0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.execute(0.02);
        const double roll_fwd = svc.get_roll_deg();

        AfsimL1Behavior rev;
        rev.init();
        rev.set_state_ne(0.0, 50.0, 0.0, -20.0, 18000.0, 0.0);
        rev.set_leg_ne(500.0, 0.0, 0.0, 0.0);
        rev.execute(0.02);
        const double roll_rev = rev.get_roll_deg();

        check_gt(std::fabs(roll_fwd), 1.0, "forward leg banks");
        check_gt(std::fabs(roll_rev), 1.0, "reversed leg banks");
        check_lt(roll_fwd * roll_rev, 0.0, "reversing the leg flips the bank sign");
    }

    // 2.6 CWE-20: non-finite and absurd host inputs must never yield a
    // non-finite command. to_safe_float() substitutes 0 for NaN/Inf and clamps
    // magnitudes, so the guidance stays defined.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.set_state_ne(0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
        svc.execute(0.02);
        const double good = svc.get_roll_deg();
        check_gt(std::fabs(good), 1.0, "baseline before hostile input");

        const double nan_v = std::nan("");
        const double inf_v = HUGE_VAL;
        svc.set_state_ne(nan_v, inf_v, -inf_v, nan_v, nan_v, nan_v);
        svc.execute(nan_v);
        check_finite(svc.get_roll_deg(), "roll finite after NaN/Inf state + dt");
        check_finite(svc.get_lat_accel(), "lat_accel finite after NaN/Inf state + dt");

        svc.set_leg_ne(nan_v, inf_v, nan_v, -inf_v);
        svc.execute(0.02);
        check_finite(svc.get_roll_deg(), "roll finite after NaN/Inf leg");

        svc.set_state_ne(1e300, -1e300, 1e300, -1e300, 1e300, 1e300);
        svc.execute(0.02);
        check_finite(svc.get_roll_deg(), "roll finite after 1e300 state");
    }

    // 2.7 A negative dt is neutralised by the facade before it reaches the
    // controller, so guidance never integrates backwards in time.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.set_state_ne(0.0, 2.0, 0.0, 20.0, 0.0, 0.0);
        svc.execute(-5.0);
        check_finite(svc.get_roll_deg(), "roll finite for negative dt");
    }

    // 2.8 init() is idempotent and re-seeds the default leg, so a host may call
    // L1_Init() again to restart a run.
    {
        AfsimL1Behavior svc;
        svc.init();
        svc.set_leg_ne(0.0, 0.0, 500.0, 0.0);
        svc.set_state_ne(0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
        svc.execute(0.02);
        const double before = svc.get_roll_deg();
        svc.init();                    // back to the default (0,0) -> (0,500) leg
        svc.execute(0.02);
        const double after = svc.get_roll_deg();
        check_finite(after, "roll finite after re-init()");
        check(before != after, "re-init() re-seeds the default leg");
    }
}

// ===========================================================================
// GROUP 3 -- AP_L1_Control timing seam (behavior preservation)
// ===========================================================================
void test_timing_seam()
{
    begin_group("Group 3: AP_L1_Control timing seam");

    // The cross-track integrator is publicly observable through
    // AP_Navigation::crosstrack_error_integrator(), and with a fixed state it
    // accumulates exactly Nu1 * XTRACK_I * dt per step. That makes every claim
    // about the dt path directly measurable instead of merely plausible.
    //
    // 2 m of cross-track keeps |Nu1| below the 5-degree gate that enables
    // integration, and 100 steps stays far below the +-0.1 integrator clamp.
    const float kNorth = 0.0f;
    const float kEast = 2.0f;
    const int kSteps = 100;

    // 3.1 Test A -- the injected dt is genuinely used, and scales the
    // integration exactly linearly: 5x the dt must give 5x the accumulation.
    {
        AfsimL1_AHRS_Shim a_ahrs;
        AP_L1_Control a(a_ahrs, nullptr);
        a.set_default_period(17.0f);
        const float i_small = wind_up_integrator(a, a_ahrs, 0.02f, kSteps, kNorth, kEast);

        AfsimL1_AHRS_Shim b_ahrs;
        AP_L1_Control b(b_ahrs, nullptr);
        b.set_default_period(17.0f);
        const float i_large = wind_up_integrator(b, b_ahrs, 0.10f, kSteps, kNorth, kEast);

        check_gt(std::fabs(i_small), 1e-9, "dt=0.02 accumulates the integrator");
        check_near(i_large / i_small, 5.0, 1e-3,
                   "integrator scales exactly with the injected dt (5x)");
    }

    // 3.2 Test A' -- determinism: identical injected dt and identical state must
    // reproduce bit-identical outputs across independent instances.
    {
        AfsimL1_AHRS_Shim a_ahrs, b_ahrs;
        AP_L1_Control a(a_ahrs, nullptr), b(b_ahrs, nullptr);
        a.set_default_period(17.0f);
        b.set_default_period(17.0f);
        const float ia = wind_up_integrator(a, a_ahrs, 0.02f, kSteps, kNorth, kEast);
        const float ib = wind_up_integrator(b, b_ahrs, 0.02f, kSteps, kNorth, kEast);
        check_exact(ia, ib, "injected-dt integration is bit-identical");
        check_exact(a.nav_roll_cd(), b.nav_roll_cd(), "injected-dt roll is bit-identical");
        check_exact(a.lateral_acceleration(), b.lateral_acceleration(),
                    "injected-dt lat_accel is bit-identical");
    }

    // 3.3 Test B -- DEFAULT-OFF. A controller that never calls set_update_dt()
    // must keep using the internal AP_HAL::micros() delta, exactly as stock.
    //
    // The legacy-path controllers below are deliberately given STATIC storage
    // duration. Vehicles reach this path through a controller that lives inside
    // a global vehicle object (ArduPlane's `Plane`), so its storage is
    // zero-initialised before construction -- and AP_L1_Control's constructor
    // only runs AP_Param::setup_object_defaults(), which does not touch plain
    // members such as _last_update_waypoint_us. Using statics here reproduces
    // that real, stock configuration faithfully; an automatic-storage instance
    // would instead have the legacy branch read an indeterminate
    // _last_update_waypoint_us on its very first call, which is a property of
    // stock upstream ArduPilot and not of this refactor. (The AfsimL1 service
    // itself never reaches that branch: AfsimL1Behavior::execute() always calls
    // set_update_dt() before update_waypoint().)
    {
        static AfsimL1_AHRS_Shim legacy_ahrs;
        static AP_L1_Control legacy(legacy_ahrs, nullptr);
        legacy.set_default_period(17.0f);
        Location prev, next;
        north_leg(prev, next);
        legacy_ahrs.set_location_NE(kNorth, kEast);
        legacy_ahrs.set_velocity_EN(0.0f, 20.0f);
        legacy_ahrs.set_yaw_cd(0.0f);
        legacy_ahrs.set_pitch_rad(0.0f);
        for (int i = 0; i < kSteps; i++) {
            legacy.update_waypoint(prev, next);   // NO set_update_dt() -- ever
        }
        const float i_legacy = legacy.crosstrack_error_integrator();

        AfsimL1_AHRS_Shim zero_ahrs;
        AP_L1_Control zero(zero_ahrs, nullptr);
        zero.set_default_period(17.0f);
        const float i_zero = wind_up_integrator(zero, zero_ahrs, 0.0f, kSteps, kNorth, kEast);

        AfsimL1_AHRS_Shim ref_ahrs;
        AP_L1_Control ref(ref_ahrs, nullptr);
        ref.set_default_period(17.0f);
        const float i_ref = wind_up_integrator(ref, ref_ahrs, 0.02f, kSteps, kNorth, kEast);

        check_exact(i_zero, 0.0f, "an injected dt of 0 accumulates nothing");
        check_finite(legacy.nav_roll_cd(), "legacy micros() path produces finite roll");
        // 100 back-to-back calls span far less than 100 x 20 ms of wall clock, so
        // the legacy accumulation must be a small fraction of the injected-20 ms
        // reference. Expressed as a ratio so the check cannot become
        // machine-speed dependent (a debugger or valgrind slows every iteration
        // down but never anywhere near 20 ms per call).
        check_lt(std::fabs(i_legacy), std::fabs(i_ref) * 0.25,
                 "default-off path integrates far less than an injected 20 ms step");
    }

    // 3.4 Test B' -- positive proof that the default-off path reads a REAL clock
    // rather than silently integrating zero: sleep a known interval between two
    // legacy updates and confirm the accumulated integrator matches an
    // explicitly injected dt of the same length. Bounded as a ratio with a wide
    // window so scheduler jitter cannot make it flaky, while still being
    // impossible to satisfy if the legacy dt were 0 or a fixed injected value.
    {
        const float kSleepSeconds = 0.060f;

        static AfsimL1_AHRS_Shim clock_ahrs;
        static AP_L1_Control clock_l1(clock_ahrs, nullptr);
        clock_l1.set_default_period(17.0f);
        Location prev, next;
        north_leg(prev, next);
        clock_ahrs.set_location_NE(kNorth, kEast);
        clock_ahrs.set_velocity_EN(0.0f, 20.0f);
        clock_ahrs.set_yaw_cd(0.0f);
        clock_ahrs.set_pitch_rad(0.0f);

        // First call primes _last_update_waypoint_us and performs the
        // "XTRACK_I gain changed" reset, so the integrator starts from zero.
        clock_l1.update_waypoint(prev, next);
        check_exact(clock_l1.crosstrack_error_integrator(), 0.0f,
                    "legacy path starts from a zeroed integrator");

        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = (long)(kSleepSeconds * 1.0e9f);
        while (nanosleep(&ts, &ts) == -1) {
            // restart on EINTR with the remaining time reported back in ts
        }
        clock_l1.update_waypoint(prev, next);
        const float i_clock = clock_l1.crosstrack_error_integrator();

        AfsimL1_AHRS_Shim ref_ahrs;
        AP_L1_Control ref(ref_ahrs, nullptr);
        ref.set_default_period(17.0f);
        const float i_ref = wind_up_integrator(ref, ref_ahrs, kSleepSeconds, 2,
                                               kNorth, kEast);

        check_gt(std::fabs(i_ref), 1e-9, "injected 60 ms reference accumulated");
        check_gt(std::fabs(i_clock), 1e-9,
                 "legacy path accumulated after a real 60 ms delay");
        const double ratio = (double)i_clock / (double)i_ref;
        check_gt(ratio, 0.4, "legacy dt is at least 40% of the slept interval");
        check_lt(ratio, 3.0, "legacy dt is at most 3x the slept interval");
    }

    // 3.5 Test C -- the dt > 1 s reinitialisation is preserved through the
    // injected path. A wound-up controller given a 1.5 s step must end up in
    // exactly the state a freshly constructed controller reaches from a single
    // 1.5 s step, proving _L1_xtrack_i really was zeroed.
    {
        AfsimL1_AHRS_Shim w_ahrs;
        AP_L1_Control wound(w_ahrs, nullptr);
        wound.set_default_period(17.0f);
        const float i_before = wind_up_integrator(wound, w_ahrs, 0.02f, kSteps, kNorth, kEast);
        check_gt(std::fabs(i_before), 1e-9, "integrator wound up before the long step");

        Location prev, next;
        north_leg(prev, next);
        wound.set_update_dt(1.5f);
        wound.update_waypoint(prev, next);
        const float i_after_reset = wound.crosstrack_error_integrator();

        AfsimL1_AHRS_Shim f_ahrs;
        AP_L1_Control fresh(f_ahrs, nullptr);
        fresh.set_default_period(17.0f);
        f_ahrs.set_location_NE(kNorth, kEast);
        f_ahrs.set_velocity_EN(0.0f, 20.0f);
        f_ahrs.set_yaw_cd(0.0f);
        f_ahrs.set_pitch_rad(0.0f);
        // Prime the fresh instance so its XTRACK_I "gain changed" reset has
        // already fired, matching the wound-up instance's bookkeeping.
        fresh.set_update_dt(0.02f);
        fresh.update_waypoint(prev, next);
        const float i_primed = fresh.crosstrack_error_integrator();
        fresh.set_update_dt(1.5f);
        fresh.update_waypoint(prev, next);
        const float i_fresh_after = fresh.crosstrack_error_integrator();

        check_exact(i_primed, 0.0f, "first waypoint update leaves the integrator at 0");
        check_exact(i_after_reset, i_fresh_after,
                    "dt>1 s reset makes a wound-up controller match a fresh one");
        check_lt(std::fabs(i_after_reset), std::fabs(i_before),
                 "dt>1 s reset discards the accumulated integrator");
    }

    // 3.6 The dt > 0.1 s cap is preserved: any injected step above 0.1 s (but
    // at or below 1 s, so no reset fires) must integrate exactly as 0.1 s does.
    {
        AfsimL1_AHRS_Shim c_ahrs, d_ahrs;
        AP_L1_Control capped(c_ahrs, nullptr), exact(d_ahrs, nullptr);
        capped.set_default_period(17.0f);
        exact.set_default_period(17.0f);
        const float i_capped = wind_up_integrator(capped, c_ahrs, 0.5f, kSteps, kNorth, kEast);
        const float i_exact = wind_up_integrator(exact, d_ahrs, 0.1f, kSteps, kNorth, kEast);
        check_exact(i_capped, i_exact, "dt=0.5 s integrates exactly as dt=0.1 s (cap)");

        AfsimL1_AHRS_Shim e_ahrs;
        AP_L1_Control uncapped(e_ahrs, nullptr);
        uncapped.set_default_period(17.0f);
        const float i_uncapped =
            wind_up_integrator(uncapped, e_ahrs, 0.09f, kSteps, kNorth, kEast);
        check(i_uncapped != i_exact, "dt=0.09 s is below the cap and differs");
    }

    // 3.7 Invalid host dt is rejected, so a bad value can neither poison an
    // already-latched timebase nor latch one of its own (CWE-20).
    //
    // The primary formulation is deliberately clock-free and EXACT: latch a
    // known-good 20 ms step, then hammer the setter with the invalid value
    // before every update. Because a rejected value must leave _override_dt
    // untouched, the accumulated integrator has to match the pure-20 ms
    // reference bit for bit -- if NaN/Inf/negative had been stored, the
    // accumulation would become NaN or stop entirely.
    {
        AfsimL1_AHRS_Shim ref_ahrs;
        AP_L1_Control ref(ref_ahrs, nullptr);
        ref.set_default_period(17.0f);
        const float i_ref = wind_up_integrator(ref, ref_ahrs, 0.02f, kSteps, kNorth, kEast);
        check_gt(std::fabs(i_ref), 1e-9, "20 ms reference accumulated for the dt-rejection tests");

        const float bad[] = {std::nanf(""), HUGE_VALF, -HUGE_VALF, -0.02f, -1.0f};
        for (unsigned k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
            AfsimL1_AHRS_Shim ahrs;
            AP_L1_Control l1(ahrs, nullptr);
            l1.set_default_period(17.0f);
            Location prev, next;
            north_leg(prev, next);
            ahrs.set_location_NE(kNorth, kEast);
            ahrs.set_velocity_EN(0.0f, 20.0f);
            ahrs.set_yaw_cd(0.0f);
            ahrs.set_pitch_rad(0.0f);

            l1.set_update_dt(0.02f);          // latch a known-good step first
            for (int i = 0; i < kSteps; i++) {
                l1.set_update_dt(bad[k]);     // must be rejected outright
                l1.update_waypoint(prev, next);
            }

            char what[112];
            std::snprintf(what, sizeof(what),
                          "invalid dt #%u rejected: roll stays finite", k);
            check_finite(l1.nav_roll_cd(), what);
            std::snprintf(what, sizeof(what),
                          "invalid dt #%u rejected: integrator stays finite", k);
            check_finite(l1.crosstrack_error_integrator(), what);
            std::snprintf(what, sizeof(what),
                          "invalid dt #%u rejected: latched 20 ms step is preserved exactly", k);
            check_exact(l1.crosstrack_error_integrator(), i_ref, what);
        }

        // Secondary formulation: an invalid value on its own must not latch the
        // override at all, leaving the controller on the stock micros() path.
        // Static storage for the same reason as Test B (this instance reaches
        // the legacy branch, exactly as a vehicle's controller does).
        static AfsimL1_AHRS_Shim nolatch_ahrs;
        static AP_L1_Control nolatch(nolatch_ahrs, nullptr);
        nolatch.set_default_period(17.0f);
        Location prev, next;
        north_leg(prev, next);
        nolatch_ahrs.set_location_NE(kNorth, kEast);
        nolatch_ahrs.set_velocity_EN(0.0f, 20.0f);
        nolatch_ahrs.set_yaw_cd(0.0f);
        nolatch_ahrs.set_pitch_rad(0.0f);
        for (int i = 0; i < kSteps; i++) {
            nolatch.set_update_dt(std::nanf(""));
            nolatch.update_waypoint(prev, next);
        }
        check_finite(nolatch.crosstrack_error_integrator(),
                     "a rejected dt leaves the integrator finite");
        check_lt(std::fabs(nolatch.crosstrack_error_integrator()),
                 std::fabs(i_ref) * 0.25,
                 "a rejected dt never latches the override (stays on micros())");
    }

    // 3.8 dt == 0 is a VALID host value (a paused simulation) and must latch the
    // override rather than fall through to the hardware clock.
    {
        AfsimL1_AHRS_Shim ahrs;
        AP_L1_Control l1(ahrs, nullptr);
        l1.set_default_period(17.0f);
        const float i0 = wind_up_integrator(l1, ahrs, 0.0f, kSteps, kNorth, kEast);
        check_exact(i0, 0.0f, "dt=0 latches and accumulates nothing");
        check_finite(l1.nav_roll_cd(), "dt=0 still produces a finite roll command");
    }

    // 3.9 The loiter path consumes the INJECTED millisecond timebase
    // (AAP 0.6.1 "Loiter/heading clock -> Injected time for the loiter path").
    // reached_loiter_target() is sticky for 200 ms after leaving the circle, so
    // advancing only the injected clock must be able to expire that window.
    {
        AfsimL1_AHRS_Shim ahrs;
        AP_L1_Control l1(ahrs, nullptr);
        l1.set_default_period(17.0f);
        const Location centre = datum_location();
        const float radius = 100.0f;

        ahrs.set_velocity_EN(20.0f, 0.0f);   // 20 m/s due East
        ahrs.set_yaw_cd(9000.0f);
        ahrs.set_pitch_rad(0.0f);

        // Inside the circle -> the "circle" branch latches _WPcircle and stamps
        // reached_loiter_target_ms with the injected time.
        ahrs.set_location_NE(10.0f, 0.0f);
        l1.set_update_dt(0.02f);             // injected clock -> 20 ms
        l1.update_loiter(centre, radius, 1);
        check(l1.reached_loiter_target(), "inside the circle: loiter target reached");

        // Far outside -> the "capture" branch runs, but only 20 ms of injected
        // time has passed, so the decision must remain sticky.
        ahrs.set_location_NE(1000.0f, 0.0f);
        l1.set_update_dt(0.02f);             // injected clock -> 40 ms
        l1.update_loiter(centre, radius, 1);
        check(l1.reached_loiter_target(),
              "capture within the injected 200 ms window stays sticky");

        // Advance the injected clock past 200 ms (set_update_dt caps each step
        // at 0.1 s, so three steps add 300 ms) and the window must expire.
        for (int i = 0; i < 3; i++) {
            l1.set_update_dt(0.1f);
        }
        l1.update_loiter(centre, radius, 1);
        check(!l1.reached_loiter_target(),
              "injected clock past 200 ms expires the loiter window");
        check_finite(l1.nav_roll_cd(), "loiter path produces a finite roll command");
    }

    // 3.10 The remaining AP_Navigation entry points stay functional under the
    // seam, so the whole controller -- not just update_waypoint -- is usable by
    // the service.
    {
        AfsimL1_AHRS_Shim ahrs;
        AP_L1_Control l1(ahrs, nullptr);
        l1.set_default_period(17.0f);
        ahrs.set_location_NE(0.0f, 0.0f);
        ahrs.set_velocity_EN(0.0f, 20.0f);
        ahrs.set_yaw_cd(0.0f);
        ahrs.set_pitch_rad(0.0f);

        l1.set_update_dt(0.02f);
        l1.update_heading_hold(9000);
        check_finite(l1.nav_roll_cd(), "update_heading_hold produces finite roll");
        check_gt(std::fabs((double)l1.nav_roll_cd()), 0.0,
                 "90 deg heading error commands a turn");

        l1.update_level_flight();
        check_exact(l1.nav_roll_cd(), 0.0, "update_level_flight commands zero roll");
        check_exact(l1.lateral_acceleration(), 0.0,
                    "update_level_flight commands zero lateral acceleration");

        l1.set_data_is_stale();
        check(l1.data_is_stale(), "set_data_is_stale() is observable");
        Location prev, next;
        north_leg(prev, next);
        l1.set_update_dt(0.02f);
        l1.update_waypoint(prev, next);
        check(!l1.data_is_stale(), "a waypoint update clears the stale flag");
    }
}

// ===========================================================================
// GROUP 4 -- extern "C" ABI (Boundary layer)
// ===========================================================================
void test_c_abi()
{
    begin_group("Group 4: extern \"C\" ABI boundary");

    // 4.1 Handle lifecycle.
    void *h = L1_Create();
    check(h != nullptr, "L1_Create() returns a handle");
    void *h2 = L1_Create();
    check(h2 != nullptr && h2 != h, "handles are distinct");

    // 4.2 End-to-end guidance through the flat C entry points.
    L1_Init(h);
    L1_SetLegNE(h, 0.0, 0.0, 500.0, 0.0);
    L1_SetStateNE(h, 0.0, 50.0, 0.0, 20.0, 0.0, 0.0);
    L1_Execute(h, 0.02);
    const double roll = L1_GetRollDeg(h);
    const double lat = L1_GetLatAccel(h);
    check_finite(roll, "L1_GetRollDeg() is finite");
    check_finite(lat, "L1_GetLatAccel() is finite");
    check_gt(std::fabs(roll), 1.0, "C ABI reports a material bank for 50 m offset");

    // 4.3 Instances are independent: driving one must not disturb the other.
    L1_Init(h2);
    L1_SetLegNE(h2, 0.0, 0.0, 500.0, 0.0);
    L1_SetStateNE(h2, 0.0, -50.0, 0.0, 20.0, 0.0, 0.0);
    L1_Execute(h2, 0.02);
    check_near(L1_GetRollDeg(h) + L1_GetRollDeg(h2), 0.0, 1e-6,
               "independent handles give mirrored results");
    check_exact(L1_GetRollDeg(h), roll, "the first handle is undisturbed");

    // 4.4 NULL-handle safety on every entry point.
    L1_Destroy(nullptr);
    L1_Init(nullptr);
    L1_Execute(nullptr, 0.02);
    L1_SetLegNE(nullptr, 0.0, 0.0, 1.0, 1.0);
    L1_SetStateNE(nullptr, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    check_exact(L1_GetRollDeg(nullptr), 0.0, "L1_GetRollDeg(NULL) == 0");
    check_exact(L1_GetLatAccel(nullptr), 0.0, "L1_GetLatAccel(NULL) == 0");
    check(true, "every entry point survives a NULL handle");

    // 4.5 An unregistered pointer must be rejected by the live-handle registry
    // and must NOT be freed.
    {
        char scratch[64];
        std::memset(scratch, 0, sizeof(scratch));
        void *bogus = static_cast<void *>(scratch);
        L1_Init(bogus);
        L1_Execute(bogus, 0.02);
        L1_SetLegNE(bogus, 0.0, 0.0, 1.0, 1.0);
        L1_SetStateNE(bogus, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
        check_exact(L1_GetRollDeg(bogus), 0.0, "L1_GetRollDeg(bogus) == 0");
        check_exact(L1_GetLatAccel(bogus), 0.0, "L1_GetLatAccel(bogus) == 0");
        L1_Destroy(bogus);
        check(true, "L1_Destroy(bogus) is a safe no-op");
    }

    // 4.6 Stale (destroyed) handles are rejected, and a double destroy is safe.
    L1_Destroy(h2);
    L1_Init(h2);
    L1_Execute(h2, 0.02);
    L1_SetLegNE(h2, 0.0, 0.0, 1.0, 1.0);
    L1_SetStateNE(h2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    check_exact(L1_GetRollDeg(h2), 0.0, "stale L1_GetRollDeg == 0");
    check_exact(L1_GetLatAccel(h2), 0.0, "stale L1_GetLatAccel == 0");
    L1_Destroy(h2);
    check(true, "double L1_Destroy is a safe no-op");

    // 4.7 A surviving handle is unaffected by another handle's destruction.
    L1_Execute(h, 0.02);
    check_finite(L1_GetRollDeg(h), "surviving handle still works after a destroy");
    L1_Destroy(h);

    // 4.8 Create/destroy churn must not corrupt the registry.
    for (int i = 0; i < 64; i++) {
        void *t = L1_Create();
        if (t == nullptr) {
            check(false, "L1_Create() failed during churn");
            break;
        }
        L1_Init(t);
        L1_SetLegNE(t, 0.0, 0.0, 500.0, 0.0);
        L1_SetStateNE(t, 0.0, 25.0, 0.0, 20.0, 0.0, 0.0);
        L1_Execute(t, 0.02);
        if (!std::isfinite(L1_GetRollDeg(t))) {
            check(false, "churn iteration produced a non-finite roll");
            L1_Destroy(t);
            break;
        }
        L1_Destroy(t);
    }
    check(true, "64 create/execute/destroy cycles completed cleanly");
}

}  // namespace

int main()
{
    std::printf("=== AfsimL1 unit test suite ===\n");

    test_shim();
    test_facade();
    test_timing_seam();
    test_c_abi();

    std::printf("\n=== AfsimL1 unit tests: %d checks, %d failures ===\n",
                g_checks, g_failures);
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
