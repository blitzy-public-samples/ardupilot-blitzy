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

/// @file    main.cpp
/// @brief   AfsimL1 service demonstration driver -- "initialize a simple leg".
///
/// This is the runnable proof that ArduPilot's L1 lateral-navigation guidance
/// (AP_L1_Control), once extracted into the reusable AfsimL1 service, can be
/// driven entirely from OUTSIDE the vehicle flight loop. It reproduces the
/// user-provided "initialize a simple leg" example (Agent Action Plan 0.7.2)
/// end-to-end:
///
///     set a simple leg  ->  inject platform state  ->  execute ONE guidance
///     step with a host-supplied dt  ->  read back the commanded roll and
///     lateral-acceleration outputs.
///
/// -----------------------------------------------------------------------------
/// Why this drives the service through the C ABI ("Path B")
/// -----------------------------------------------------------------------------
/// The demo deliberately consumes the service ONLY through its stable
/// `extern "C"` boundary (l1_c_api.h): the eight flat `L1_*` entry points plus
/// the opaque `void*` handle. This is exactly how an external host such as the
/// AFSIM simulation environment binds `libafsim_l1.so` at runtime -- no C++ type,
/// reference, or class crosses the boundary, so the host stays decoupled from the
/// C++ ABI (name mangling, v-table layout, RTTI, exception representation). The
/// alternative "Path A" (including AfsimL1Behavior.h and instantiating the facade
/// directly) is equivalent for a same-toolchain caller, but Path B is the
/// contract an out-of-tree consumer actually links against, so it is the more
/// faithful demonstration.
///
/// This file follows the STANDALONE example shape (a plain `int main()` with only
/// standard-library includes, mirroring
/// libraries/SITL/examples/JSON/C++/minimal.cpp) rather than the ArduPilot
/// firmware-example harness (`void setup()/void loop()` + `AP_HAL_MAIN()`),
/// because the AfsimL1 demo is built as/with the standalone shared library.
///
/// -----------------------------------------------------------------------------
/// Scenario exercised
/// -----------------------------------------------------------------------------
///   * Leg   : prev = (0 N, 0 E)  ->  next = (0 N, 500 E) metres, i.e. a 500 m
///             leg lying due East of the datum. These are the exact endpoints of
///             the user example's "simple leg" (prev=(0,0), next=(0,500)).
///   * State : platform at the datum (0 N, 0 E) with a 20 m/s ground velocity
///             (velN = 20, velE = 0), yaw = 0 cd, pitch = 0 rad.
///   * Timing: a single 50 Hz control step, dt = 0.02 s. The host owns the
///             timebase; dt is handed straight to the controller instead of being
///             derived from a hardware micro-clock.
///
/// Because the platform is moving North while the target leg runs East, the L1
/// guidance produces a non-trivial (non-zero) turn command -- a meaningful, finite
/// roll / lateral-acceleration pair -- which the demo prints.
///
/// Build: this translation unit is compiled together with the AfsimL1 service
/// sources (l1_c_api.cpp, AfsimL1Behavior.cpp, AfsimL1_AHRS_Shim.cpp and the
/// composed AP_L1_Control) -- via the sibling CMakeLists.txt (standalone) or the
/// in-tree waf example (wscript) -- and links against `libafsim_l1.so`.

#include <cstdio>        // std::printf / std::fprintf -- output only; no algorithm logic here.

#include "l1_c_api.h"    // The AfsimL1 service's stable extern "C" boundary: the ONLY
                         // service header this demo includes. It declares the eight
                         // L1_* entry points and the opaque handle typedef. Being a pure
                         // C ABI header (no C++ service types), it keeps this driver a
                         // faithful stand-in for an external, differently-compiled host.

/// Program entry point.
///
/// Walks the full service lifecycle in order --
///   L1_Create -> L1_Init -> L1_SetLegNE -> L1_SetStateNE -> L1_Execute
///             -> L1_GetRollDeg / L1_GetLatAccel -> L1_Destroy
/// -- guaranteeing the opaque handle is released before returning (no leak).
///
/// @return 0 on success; 1 if the service instance could not be created.
int main()
{
    // 1. Create the service instance. L1_Create() returns an opaque handle
    //    (a `void*`); the concrete C++ type behind it is never exposed. A NULL
    //    return indicates allocation failure -- handle it defensively so the demo
    //    fails loudly instead of dereferencing a null handle downstream.
    void* h = L1_Create();
    if (h == nullptr) {
        std::fprintf(stderr, "AfsimL1 demo: L1_Create() failed (out of memory?)\n");
        return 1;
    }

    // 2. Initialize the service. init() seeds a default "simple leg"
    //    (prev = 0 N/0 E, next = 0 N/500 E) inside the facade, mirroring the
    //    user example's constructor-time setup.
    L1_Init(h);

    // 3. Set the active navigation leg explicitly to demonstrate the setter and
    //    to pin the exact endpoints from the user example. Argument order is
    //    (prevN, prevE, nextN, nextE) in metres relative to the service datum:
    //    prev = (0, 0) -> next = (0, 500), i.e. next lies 500 m East of the
    //    datum. This matches the user example's next = (0, 500) exactly.
    L1_SetLegNE(h, /*prevN=*/0.0, /*prevE=*/0.0, /*nextN=*/0.0, /*nextE=*/500.0);

    // 4. Inject a representative platform state so the guidance has non-trivial
    //    inputs. Argument order is (n, e, velE, velN, yaw_cd, pitch_rad):
    //    the platform sits at the datum (0 N, 0 E) with a 20 m/s ground velocity
    //    directed North (velN = 20, velE = 0), heading 0 centidegrees, level
    //    pitch (0 rad).
    L1_SetStateNE(h, /*n=*/0.0, /*e=*/0.0, /*velE=*/0.0, /*velN=*/20.0,
                  /*yaw_cd=*/0.0, /*pitch_rad=*/0.0);

    // 5. Advance the L1 guidance by one control step. The host supplies dt
    //    directly (0.02 s == a 50 Hz loop); the service forwards it to the
    //    controller's additive timing seam instead of reading a hardware clock.
    L1_Execute(h, /*dt_seconds=*/0.02);

    // 6. Read back the commanded outputs the external host would act on:
    //    the bank/roll angle (degrees) and the lateral acceleration (m/s^2).
    const double roll_deg  = L1_GetRollDeg(h);
    const double lat_accel = L1_GetLatAccel(h);

    // 7. Report the results. For the 500 m East leg above, these are finite,
    //    non-trivial guidance commands.
    std::printf("AfsimL1 demo: simple leg prev=(0,0) -> next=(0,500) m, "
                "dt=0.02 s\n");
    std::printf("roll_deg = %f, lat_accel = %f\n", roll_deg, lat_accel);

    // 8. Release the service instance. After this call the handle is invalid and
    //    must not be reused; destroying it here guarantees the demo leaks nothing.
    L1_Destroy(h);

    return 0;
}
