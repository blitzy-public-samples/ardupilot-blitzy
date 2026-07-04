# AfsimL1 — Reusable L1 Lateral-Navigation Guidance Service

This module packages ArduPilot's `AP_L1_Control` L1 lateral-navigation guidance as a
**reusable, modular service** exposed behind a **stable `extern "C"` ABI** in a shared
library, **`libafsim_l1.so`**. An external host — for example the
[AFSIM](https://dsiac.dtic.mil/models/afsim/) simulation environment — can drive the guidance
controller through this library **without linking the ArduPilot firmware image**. Legs,
platform state, and timing are pushed in by the host; the commanded roll and lateral
acceleration are read back out.

The extraction is strictly **behavior-preserving**. The L1 guidance mathematics and their
numerical outputs are **identical** to the vehicle build; nothing in the control law,
gains, or geometry is changed. Only the *input/output boundaries* are re-plumbed: the
legs, state, and control-step `dt` that the vehicle flight loop used to supply implicitly
are now injected explicitly, and the roll / lateral-acceleration outputs are surfaced
through the C ABI. The wrapped `AP_L1_Control` receives a single additive, **default-off**
`set_update_dt(float)` timing seam so a host can own the timebase; when that seam is
unused the controller behaves exactly as it does in firmware.

## Architecture

The service is composed of three thin layers, each in its own translation unit, so the
concerns of *task API*, *state adaptation*, and *ABI isolation* stay cleanly separated:

- **Facade / Service layer — `AfsimL1Behavior` (`AfsimL1Behavior.h` / `.cpp`).**
  Presents a small, task-oriented API (`set_leg_ne`, `set_state_ne`, `execute`,
  `get_roll_deg`, `get_lat_accel`) over the richer `AP_L1_Control` controller. It *owns*
  an `AP_L1_Control` instance, the AHRS state adapter, and the current `prev`/`next` leg.
- **Adapter layer — `AfsimL1_AHRS_Shim` (`AfsimL1_AHRS_Shim.h` / `.cpp`).**
  A lightweight, host-driven stand-in that supplies the six `AP_AHRS` reads the controller
  consumes (`get_location`, `groundspeed_vector`, `get_yaw_rad`, `yaw_sensor`,
  `get_pitch_rad`, `get_EAS2TAS`) from injected state instead of from live sensor fusion.
- **ABI-boundary layer — `l1_c_api` (`l1_c_api.h` / `.cpp`).**
  A flat, name-mangling-free `extern "C"` surface over an opaque handle. This is the layer
  an external host binds at runtime.

Data flow for one control step:

```text
host ── L1_SetLegNE  ─┐
host ── L1_SetStateNE ┼─► AfsimL1Behavior ─► AfsimL1_AHRS_Shim ─┐
host ── L1_Execute(dt)┘        (facade)                          ├─► AP_L1_Control
                                                                 │   (reused, unchanged)
host ◄─ L1_GetRollDeg ◄────────┤                                 │
host ◄─ L1_GetLatAccel ◄───────┴─────────────────────────────────┘
```

### Files

| File | Role |
|------|------|
| `AfsimL1Behavior.h` / `.cpp` | Facade / service layer — the task API and composition root. |
| `AfsimL1_AHRS_Shim.h` / `.cpp` | Adapter — host-injected replacement for the `AP_AHRS` read surface. |
| `l1_c_api.h` / `.cpp` | `extern "C"` ABI boundary — the eight exported entry points + opaque handle. |
| `main.cpp` | Runnable "initialize a simple leg" demonstration driver (consumes the C ABI). |
| `CMakeLists.txt` | Standalone shared-library build producing `libafsim_l1.so`. |
| `wscript` | In-tree `waf` example build (`bld.ap_example(use='ap')`). |

## C ABI reference

The boundary is `extern "C"` with an **opaque `void*` handle** (internally a
`struct L1_Context` that wraps an `AfsimL1Behavior*`, defined only in `l1_c_api.cpp`).
Only plain `double` and `void*` scalars ever cross the boundary — **no C++ type,
reference, or class is exposed** — so an external host can load `libafsim_l1.so`
regardless of its own compiler or C++ standard library. Every entry point is **null-safe**:
the `void`-returning functions ignore a `NULL` handle, and the getters return `0.0`.

| Entry point | Description |
|-------------|-------------|
| `void*  L1_Create(void)` | Allocate a service instance; returns the opaque handle (`NULL` on allocation failure). Release it with `L1_Destroy`. |
| `void   L1_Destroy(void* h)` | Free an instance previously returned by `L1_Create`. A `NULL` handle is ignored. |
| `void   L1_Init(void* h)` | Seed a default simple leg / initial state. A `NULL` handle is ignored. |
| `void   L1_SetLegNE(void* h, double prevN, double prevE, double nextN, double nextE)` | Set the active leg from the previous and next waypoints, in metres North/East from the datum. |
| `void   L1_SetStateNE(void* h, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad)` | Inject platform state: position N/E (m), velocity **East then North** (m/s), yaw (centidegrees), pitch (radians). |
| `void   L1_Execute(void* h, double dt_seconds)` | Run one guidance step using the host-supplied `dt` (the host owns the timebase). |
| `double L1_GetRollDeg(void* h)` | Commanded roll (bank) angle in **degrees** (`0.0` if `h` is `NULL`). |
| `double L1_GetLatAccel(void* h)` | Demanded lateral acceleration in **m/s²** (`0.0` if `h` is `NULL`). |

The declarations are the authoritative contract; see `l1_c_api.h`.

### Input validation

Every scalar the host supplies crosses an untrusted boundary, so the service validates all
inputs **before** they mutate any controller or shim state. Each value is checked for
finiteness (`NaN` / `Inf` are rejected) and against a conservative per-quantity magnitude
bound; a rejected call is a **safe no-op** that leaves the previously accepted state
intact, so guidance never consumes a corrupted, half-updated, or out-of-range value. The
`extern "C"` setters additionally re-check finiteness at the boundary (defence in depth)
before delegating to the facade.

| Entry point | Accepted input | On invalid input |
|-------------|----------------|------------------|
| `L1_SetLegNE` | each coordinate finite, within `±2.0e7` m | ignored; the previously set leg is preserved |
| `L1_SetStateNE` | position within `±2.0e7` m, velocity `±1.0e4` m/s, yaw `±3.6e6` cd, pitch `±1.0e2` rad, all finite | ignored atomically; the previously injected state is preserved (no partial update) |
| `L1_Execute` | `dt_seconds` finite and `> 0` | step skipped; controller state and last outputs are unchanged |

The magnitude bounds are deliberately generous — they fence off values that would overflow
the downstream fixed-point latitude/longitude and heading conversions (undefined
`float`→`int32` behaviour) rather than impose a flight envelope. An **accepted** `dt` is
still subject to the controller's own preserved clamp (`dt > 1 s` reinitialises the
cross-track integrator; `dt > 0.1 s` is capped at `0.1 s`), so the injected-timing path is
numerically identical to the firmware path. The current ABI has no error-return channel, so
invalid updates are silently ignored by design; `L1_GetRollDeg` / `L1_GetLatAccel` continue
to report the last well-defined command (and return `0.0` before the first successful
`L1_Execute`).

## Dependency-injection model

In the vehicle build the controller *pulls* its inputs: position, velocity, and attitude
come from live `AP_AHRS` sensor fusion, and the control-step `dt` is derived from the HAL
micro-clock (`AP_HAL::micros()`). This service **inverts** that dependency — the host
*pushes* everything in:

- **Legs** via `L1_SetLegNE` — the previous and next waypoints of the active leg.
- **State** via `L1_SetStateNE` — position, ground velocity, yaw, and pitch, routed through
  the AHRS shim so the controller reads exactly the values the host supplied.
- **Timing** via `L1_Execute(h, dt_seconds)` — the host-supplied `dt` overrides the internal
  `AP_HAL::micros()` delta through the additive, **default-off** `set_update_dt` seam on
  `AP_L1_Control`. The existing clamp semantics are preserved, so numerical behavior is
  unchanged; the host simply owns the timebase.

### Units and conventions

| Quantity | Direction | Unit / convention |
|----------|-----------|-------------------|
| Position `n`, `e` | in | metres North / East, relative to a fixed datum |
| Velocity `velE`, `velN` | in | m/s — **East first, then North** (matching the setter argument order) |
| Yaw `yaw_cd` | in | centidegrees (e.g. `4500` = 45°) |
| Pitch `pitch_rad` | in | radians |
| Leg `prevN, prevE, nextN, nextE` | in | metres North / East from the datum |
| Timing `dt_seconds` | in | seconds (host-owned) |
| Roll (`L1_GetRollDeg`) | out | degrees |
| Lateral acceleration (`L1_GetLatAccel`) | out | m/s² (positive to the right) |

## Building

The service builds as a standalone shared library with `cmake`:

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

This produces **`libafsim_l1.so`** — and nothing else, since the demonstration driver is an
opt-in target (see below).

### Verifying the exported symbols

The library is compiled with `-fvisibility=hidden`, so only the eight C-ABI entry points
are exported. Confirm this with:

```bash
$ nm -D --defined-only libafsim_l1.so
```

The output should list **exactly** these eight `L1_*` symbols and nothing else:

```text
L1_Create  L1_Destroy  L1_Init  L1_Execute
L1_SetLegNE  L1_SetStateNE  L1_GetRollDeg  L1_GetLatAccel
```

### Building the demonstration driver (optional)

The runnable `main.cpp` demo is a separate C-ABI consumer, gated behind an opt-in switch
(default `OFF`) so the default build stays limited to the library. Enable it to build both
`libafsim_l1.so` and the `afsim_demo` executable:

```bash
$ cmake -DAFSIM_L1_BUILD_DEMO=ON ..
$ make
$ ./afsim_demo
```

The executable's `RPATH` already points at the co-located `.so`, so it runs in place from
the build directory without setting `LD_LIBRARY_PATH`.

### In-tree `waf` alternative

The sibling `wscript` builds the same sources as an ArduPilot example via
`bld.ap_example(use='ap')`. From the repository root:

```bash
$ ./waf configure --board sitl
$ ./waf --targets examples/AfsimL1
```

## Usage

A minimal C host binds only `l1_c_api.h` and the eight exported symbols. The sequence below
mirrors the demonstration driver in `main.cpp`:

```c
#include "l1_c_api.h"

void* h = L1_Create();          /* opaque handle; NULL on failure   */
L1_Init(h);                      /* seed a default simple leg        */

/* prev=(0,0) -> next=(0,500): a 500 m leg due East of the datum */
L1_SetLegNE(h, 0, 0, 0, 500);

/* at the datum; 20 m/s ground velocity due North (velE=0, velN=20) */
L1_SetStateNE(h, 0, 0, 0, 20, 0, 0);

L1_Execute(h, 0.02);             /* advance one 50 Hz control step   */

double roll_deg  = L1_GetRollDeg(h);   /* commanded bank angle, degrees */
double lat_accel = L1_GetLatAccel(h);  /* lateral acceleration, m/s^2   */

L1_Destroy(h);                   /* release the instance             */
```

The runnable version of this exact sequence lives in **`main.cpp`**, which drives the
service entirely through the `extern "C"` boundary — exactly as an external host such as
AFSIM would consume `libafsim_l1.so`.
