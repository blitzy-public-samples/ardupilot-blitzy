/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// ============================================================================
//  main.cpp  --  AfsimL1 service demo: "initialize a simple leg"
//                (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  WHAT THIS IS (AAP 0.2.1, 0.4.1, 0.7.2):
//    A minimal, self-contained demonstration driver that exercises the
//    AfsimL1 reusable navigation service end-to-end:
//
//        set a simple leg  ->  execute one guidance step with a host-supplied
//        dt  ->  read back the roll & lateral-acceleration commands.
//
//    It is the runnable proof that the L1 lateral guidance extracted out of
//    the ArduPilot vehicle flight loop can be driven entirely from OUTSIDE
//    that loop -- exactly the way an external host such as the AFSIM
//    simulation environment would drive it. The flow mirrors the user
//    "initialize a simple leg" example one-to-one (AAP 0.7.2): the simple leg
//    prev = (0, 0) -> next = (0, 500) in North/East metres, a representative
//    platform state, one 50 Hz step, and the two output commands.
//
//  DRIVE PATH -- PATH B (the C Application Binary Interface):
//    This demo consumes the service through its stable extern "C" boundary
//    (l1_c_api.h): the eight flat L1_* entry points operating on an opaque
//    void* handle. That is precisely how AFSIM loads and drives
//    libafsim_l1.so at run time -- no C++ type crosses the boundary, so the
//    host is insulated from the C++ ABI (name mangling, v-table layout, RTTI,
//    exception representation) of the wrapped AP_L1_Control controller
//    (AAP 0.3.3, 0.6.4). The equally-valid Path A (including
//    "AfsimL1Behavior.h" and driving the facade object directly) is documented
//    in README.md; Path B is used here because it demonstrates the external-
//    consumer contract that is the whole point of the extraction.
//
//  SHAPE (AAP 0.7.2, and libraries/SITL/examples/JSON/C++/minimal.cpp):
//    A plain standalone `int main()` with a GPL header and standard-library
//    includes -- NOT the ArduPilot setup()/loop()/AP_HAL_MAIN() firmware
//    example harness -- because the demo is built as/with the standalone
//    shared library (see CMakeLists.txt), not linked into a firmware image.
//
//  SCOPE / CONSTRAINTS (AAP 0.7.1):
//    - Uses ONLY the public service surface (the eight C-ABI functions).
//      It never touches AP_L1_Control internals, and includes no vehicle
//      firmware and no new third-party dependency.
//    - Behavior-preserving: this file performs no guidance mathematics of its
//      own. It is pure wiring + printing; the L1 control law lives untouched
//      inside AP_L1_Control, behind the service facade.
// ============================================================================

#include <cstdio>    // std::printf   -- report the guidance outputs
#include <cstdlib>   // EXIT_SUCCESS / EXIT_FAILURE -- self-checking exit status
#include <cmath>     // std::isfinite -- smoke-test guard on the outputs

#include "l1_c_api.h" // The AfsimL1 service seen exactly as an external host sees it.

/// Demonstrate one full lifecycle of the AfsimL1 service over the C ABI.
///
/// The sequence is the complete external-consumer contract, in order:
///   L1_Create -> L1_Init -> L1_SetLegNE -> L1_SetStateNE -> L1_Execute
///   -> L1_GetRollDeg / L1_GetLatAccel -> L1_Destroy
/// The opaque handle is created once and released exactly once on every exit
/// path, so it is never leaked.
///
/// @return EXIT_SUCCESS when the service was created and produced finite
///         guidance outputs; EXIT_FAILURE if the handle could not be created
///         or either output was non-finite (a useful smoke test for CI).
int main()
{
    // ------------------------------------------------------------------
    // 1. Create the service instance.
    //    L1_Create() allocates the opaque context together with the
    //    underlying guidance facade and returns an opaque handle. A host
    //    holds nothing but this void*; it never sees a C++ type. The handle
    //    must eventually be released with L1_Destroy() to avoid leaking.
    // ------------------------------------------------------------------
    void* h = L1_Create();
    if (h == nullptr) {
        // Allocation failed: report and bail out before using the handle.
        std::fprintf(stderr, "AfsimL1 demo: L1_Create() failed to allocate the service.\n");
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // 2. Initialise the instance.
    //    L1_Init() seeds a default leg inside the facade -- prev = (0, 0),
    //    next = (0, 500) in North/East metres -- so the service is ready to
    //    execute even before any explicit setter is called.
    // ------------------------------------------------------------------
    L1_Init(h);

    // ------------------------------------------------------------------
    // 3. Set the active leg explicitly (demonstrates the leg setter).
    //    Arguments are (prevN, prevE, nextN, nextE) in metres relative to the
    //    service datum, reproducing the user example's simple leg:
    //        prev = (N=0, E=0)  ->  next = (N=0, E=500)
    //    i.e. a 500 m leg from the datum. Legs are host-supplied here rather
    //    than pulled from the vehicle mission code.
    // ------------------------------------------------------------------
    L1_SetLegNE(h, /*prevN=*/0.0, /*prevE=*/0.0, /*nextN=*/0.0, /*nextE=*/500.0);

    // ------------------------------------------------------------------
    // 4. Inject a representative platform state.
    //    These are the position, velocity, and attitude values the vehicle
    //    loop would normally read from AHRS sensor fusion; here the host
    //    supplies them directly. Arguments are
    //        (n, e, velE, velN, yaw_cd, pitch_rad)
    //    = at the leg start (n=0, e=0) with a 20 m/s ground velocity
    //    (velE=0, velN=20), wings-level (yaw_cd=0, pitch_rad=0). This gives
    //    the guidance non-trivial inputs to act on.
    // ------------------------------------------------------------------
    L1_SetStateNE(h, /*n=*/0.0, /*e=*/0.0, /*velE=*/0.0, /*velN=*/20.0, /*yaw_cd=*/0.0, /*pitch_rad=*/0.0);

    // ------------------------------------------------------------------
    // 5. Execute one guidance step with a host-supplied dt.
    //    The host owns the timebase: 0.02 s == a 50 Hz control step. This dt
    //    is injected straight into the controller instead of it computing its
    //    own delta from the hardware clock -- the timing decoupling that lets
    //    AFSIM drive the guidance cadence.
    // ------------------------------------------------------------------
    L1_Execute(h, /*dt_seconds=*/0.02);

    // ------------------------------------------------------------------
    // 6. Read back the guidance commands produced by the step.
    //    get_roll_deg  == AP_L1_Control::nav_roll_cd() / 100  (degrees)
    //    get_lat_accel == AP_L1_Control::lateral_acceleration() (m/s^2)
    // ------------------------------------------------------------------
    const double roll_deg  = L1_GetRollDeg(h);
    const double lat_accel = L1_GetLatAccel(h);

    std::printf("roll_deg = %f, lat_accel = %f\n", roll_deg, lat_accel);

    // ------------------------------------------------------------------
    // 7. Smoke-test guard: the extraction must reproduce finite guidance
    //    outputs for this leg. Reporting the result through the exit status
    //    lets the demo double as a lightweight self-check. This is output
    //    validation only -- it performs no guidance mathematics.
    // ------------------------------------------------------------------
    const bool outputs_finite = std::isfinite(roll_deg) && std::isfinite(lat_accel);
    if (!outputs_finite) {
        std::fprintf(stderr, "AfsimL1 demo: guidance produced a non-finite output.\n");
    }

    // ------------------------------------------------------------------
    // 8. Destroy the service instance.
    //    Releases the underlying facade and the opaque context. Reached on
    //    every success path, so the handle created in step 1 is never leaked.
    // ------------------------------------------------------------------
    L1_Destroy(h);

    return outputs_finite ? EXIT_SUCCESS : EXIT_FAILURE;
}
