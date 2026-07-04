# AfsimL1 — Reusable L1 Lateral-Navigation Guidance Service

`AfsimL1` packages ArduPilot's `AP_L1_Control` L1 lateral-navigation guidance as a
**reusable, modular service** exposed behind a **stable `extern "C"` ABI** in a shared
library named **`libafsim_l1.so`**. It lets an external host — for example the AFSIM
simulation environment — drive the L1 guidance law directly, **without linking the
ArduPilot firmware image** and without pulling in the vehicle flight loop, the EKF/DCM
sensor-fusion stack, or the scheduler.

The extraction is strictly **behavior-preserving**: the L1 guidance mathematics and the
numerical roll / lateral-acceleration outputs are **identical** to the vehicle build —
nothing in the control law is rewritten. Only the *input* and *output boundaries* are
re-plumbed. In the firmware the controller *pulls* position, velocity, and attitude from
live `AP_AHRS` sensor fusion and derives its control-step `dt` from the hardware
micro-clock; here the host instead *pushes* in the **legs**, the **state**, and the
**timing**, and reads the **roll** and **lateral-acceleration** commands back — all across
the same flat, mangling-free C boundary. Because only these seams change, the guidance is
the real `AP_L1_Control` computing the same result it computes inside a vehicle.

## Architecture

The service is a three-layer composition (a Facade over an Adapter behind an ABI boundary).
Each layer is small and single-purpose, so the guidance controller it wraps stays
untouched:

| Layer | File(s) | Responsibility |
|-------|---------|----------------|
| **Facade** | `AfsimL1Behavior.{h,cpp}` | Presents the small, task-oriented service API (`set_leg_ne`, `set_state_ne`, `execute`, `get_roll_deg`, `get_lat_accel`) over the richer `AP_L1_Control`; owns the controller instance and the injected legs. |
| **Adapter** | `AfsimL1_AHRS_Shim.{h,cpp}` | Supplies the six AHRS reads the controller consumes (`get_location`, `groundspeed_vector`, `get_yaw_rad`, `yaw_sensor`, `get_pitch_rad`, `get_EAS2TAS`) from the host-injected North/East position, East/North velocity, yaw, and pitch. |
| **ABI boundary** | `l1_c_api.{h,cpp}` | The `extern "C"` entry points operating on an opaque `void*` handle — the only surface an external host ever sees. |

The wrapped guidance controller — `AP_L1_Control` — and the geometry libraries it depends
on (`AP_Math`, `AP_Common/Location`) are compiled **unchanged** into the shared library, so
the extracted service produces the same numbers the vehicle firmware does.

## C ABI

The public boundary is declared in [`l1_c_api.h`](l1_c_api.h) as a flat set of `extern "C"`
functions that operate on an **opaque handle**. The concrete type behind the handle
(`struct L1_Context`, which wraps an `AfsimL1Behavior*`) is defined only in `l1_c_api.cpp`;
callers hold nothing but a `void*` and never see a C++ type. Only plain C scalar types —
`void*` and `double` — cross the boundary.

Keeping the interface flat and C-only is what makes it **ABI-stable**: C++ name mangling,
virtual-table layout, RTTI, and exception representation all differ across compilers and
compiler versions, whereas the C ABI is a stable superset that every toolchain agrees upon.
As a result the host can load `libafsim_l1.so` **regardless of the compiler or C++ standard
library it was itself built with**.

All eight entry points are exported with `__attribute__((visibility("default")))`; every
other symbol in the library is hidden (the library is built with `-fvisibility=hidden`).

| Entry point | Purpose |
|-------------|---------|
| `void*  L1_Create(void)` | Allocate a service instance; returns the opaque handle (non-`NULL` on success, `NULL` if allocation failed). Release it later with `L1_Destroy`. |
| `void   L1_Destroy(void* h)` | Free an instance previously returned by `L1_Create`. Passing `NULL` is a safe no-op. |
| `void   L1_Init(void* h)` | Seed a default simple leg / initial state so the instance is ready to execute. Passing `NULL` is a safe no-op. |
| `void   L1_SetLegNE(void* h, double prevN, double prevE, double nextN, double nextE)` | Set the active navigation leg from the previous and next waypoints, as North/East offsets in **metres** relative to the service datum. |
| `void   L1_SetStateNE(void* h, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad)` | Inject the current platform state the controller consumes: position `n`/`e` (**metres** North/East), ground velocity `velE`/`velN` (**m/s**, East then North), `yaw_cd` (**centidegrees**), `pitch_rad` (**radians**). |
| `void   L1_Execute(void* h, double dt_seconds)` | Run one guidance step using a host-supplied time delta (the host owns the timebase, e.g. `0.02` for 50 Hz). Passing `NULL` is a safe no-op. |
| `double L1_GetRollDeg(void* h)` | Read the commanded roll from the most recent `L1_Execute`, in **degrees** (returns `0.0` if `h` is `NULL`). |
| `double L1_GetLatAccel(void* h)` | Read the demanded lateral acceleration from the most recent `L1_Execute`, in **m/s²** (returns `0.0` if `h` is `NULL`). |

## Dependency-injection model

Inside a vehicle, `AP_L1_Control` obtains everything it needs implicitly: it reads position,
velocity, and attitude from live `AP_AHRS` sensor fusion and derives its control-step `dt`
from the `AP_HAL::micros()` hardware clock. This service **inverts** that dependency — the
host pushes each input in explicitly:

- **Legs** are supplied with `L1_SetLegNE(h, prevN, prevE, nextN, nextE)` instead of being
  fed from the vehicle mission/navigation code.
- **State** (position, velocity, attitude) is supplied with `L1_SetStateNE(...)` and routed
  through the AHRS shim, instead of being read from live sensor fusion.
- **Timing** is supplied as the `dt_seconds` argument to `L1_Execute(...)`. The host owns
  the timebase; the injected `dt` overrides the internal `AP_HAL::micros()` delta through
  the **additive, default-off** `set_update_dt` seam on `AP_L1_Control`. The controller's
  existing clamp semantics are preserved unchanged, so numerical behavior is identical.
  Because the seam is default-off, existing vehicle callers that never call `set_update_dt`
  keep using the hardware-clock path exactly as before.

### Units and conventions

Getting the units and argument order right at the boundary is essential, because only bare
`double`s cross it:

| Quantity | Where | Units / convention |
|----------|-------|--------------------|
| Waypoint / position offsets | `L1_SetLegNE`, `L1_SetStateNE` (`n`, `e`) | **metres**, North / East, relative to the service datum |
| Ground velocity | `L1_SetStateNE` (`velE`, `velN`) | **m/s**, components ordered **East then North** (matching the setter's argument order) |
| Yaw / heading | `L1_SetStateNE` (`yaw_cd`) | **centidegrees** |
| Pitch | `L1_SetStateNE` (`pitch_rad`) | **radians** |
| Control-step `dt` | `L1_Execute` (`dt_seconds`) | **seconds** |
| Roll command (output) | `L1_GetRollDeg` | **degrees** |
| Lateral acceleration (output) | `L1_GetLatAccel` | **m/s²** |

## Building

### Standalone shared library (CMake)

The service builds as a self-contained shared library with CMake — the target an external
host binds at run time. From this directory:

```bash
$ mkdir build && cd build
$ cmake ..
$ make
```

This produces **`libafsim_l1.so`**. The build links only in-tree ArduPilot sources plus the
local service sources; it introduces no new third-party dependency.

### Verifying the exported symbols

The library is compiled with `-fvisibility=hidden`, so only the eight C-ABI entry points are
exported and everything else stays internal. Confirm the exported surface with:

```bash
$ nm -D --defined-only libafsim_l1.so
```

The output should list **exactly** these eight `L1_*` symbols and no others:

```text
L1_Create
L1_Destroy
L1_Init
L1_Execute
L1_SetLegNE
L1_SetStateNE
L1_GetRollDeg
L1_GetLatAccel
```

### In-tree ArduPilot example (waf)

As an alternative to the standalone CMake build, the sibling `wscript` builds the same
service as an in-tree ArduPilot example via waf (`bld.ap_example(use='ap')`). From the
repository root:

```bash
$ ./waf configure --board sitl
$ ./waf build --targets examples/AfsimL1
```

## Usage

There are two equivalent ways to drive the service: through the C ABI (how an external host
such as AFSIM consumes it), or by using the C++ facade directly (convenient when the host is
itself C++ and links the sources in-tree).

### Path B — through the C ABI (`l1_c_api.h`)

This is the external-consumer contract: include only `l1_c_api.h` and call the flat `L1_*`
functions on an opaque handle. No C++ type is involved.

```c
#include "l1_c_api.h"

void* h = L1_Create();                  /* allocate the service instance      */
L1_Init(h);                             /* seed a default leg / state         */
L1_SetLegNE(h, 0, 0, 500, 0);           /* 500 m leg due North (next N=500)   */
L1_SetStateNE(h, 0, 0, 0, 20, 0, 0);    /* 20 m/s North, on track, wings level */
L1_Execute(h, 0.02);                    /* one 50 Hz guidance step            */

double roll_deg  = L1_GetRollDeg(h);    /* commanded roll,      degrees       */
double lat_accel = L1_GetLatAccel(h);   /* demanded lateral accel, m/s^2      */

L1_Destroy(h);                          /* release the instance               */
```

A complete, runnable version of this sequence — including a cross-track state that produces
a materially non-zero roll — is provided in [`main.cpp`](main.cpp), which is the demo driver
built alongside the library.

### Path A — the C++ facade directly (`AfsimL1Behavior.h`)

If the host is C++ and compiles the service in-tree, it can drive the `AfsimL1Behavior`
facade object directly. The facade methods map one-to-one onto the C ABI:

```cpp
#include "AfsimL1Behavior.h"

AfsimL1Behavior l1;
l1.init();
l1.set_leg_ne(0, 0, 500, 0);            // 500 m leg due North
l1.set_state_ne(0, 0, 0, 20, 0, 0);     // n, e, velE, velN, yaw_cd, pitch_rad
l1.execute(0.02);                        // one 50 Hz guidance step

double roll_deg  = l1.get_roll_deg();    // degrees
double lat_accel = l1.get_lat_accel();   // m/s^2
```

## Behavior preservation

This module is a **structural extraction, not an algorithm change**. The L1 control law,
its gains, and its geometry live untouched inside `AP_L1_Control`; the facade, the AHRS
shim, and the C ABI only move the input/output boundaries. The single edit to the wrapped
controller — the additive, default-off `set_update_dt` timing seam — leaves every existing
`AP_L1_Control` signature intact and, when unused, reproduces the stock hardware-clock
behavior exactly. Vehicle firmware (ArduPlane and the other vehicles) is unaffected and
continues to use the controller exactly as before.

