# AfsimL1 — Reusable L1 Lateral-Navigation Guidance Service

`AfsimL1` packages ArduPilot's `AP_L1_Control` L1 lateral-navigation guidance as a
**reusable, modular service** exposed behind a **stable `extern "C"` ABI** in a shared
library named **`libafsim_l1.so`**. It lets an external host — for example the AFSIM
simulation environment — drive the L1 guidance law directly, **without linking the
ArduPilot firmware image** and without pulling in the vehicle flight loop or the scheduler.
As built by the standalone CMake target — the recommended path, which uses the compile-time
AHRS include seam described under **AHRS state decoupling** below — the library also stays
clear of the EKF/DCM sensor-fusion stack. (The alternative *Option A* fidelity build links
the real `AP_AHRS`, and with it the EKF/DCM graph; see that section.)

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

## AHRS state decoupling

`AP_L1_Control` binds a concrete `AP_AHRS &` and reads six accessors from it
(`get_location`, `groundspeed_vector`, `get_yaw_rad`, `yaw_sensor`, `get_pitch_rad`,
`get_EAS2TAS`). Those accessors are **non-virtual**, so a plain subclass cannot override
them — the service must decouple the state source another way. Two options exist (AAP
§0.6.2); this module **implements Option B**:

- **Option B — compile-time include seam (implemented, recommended).** `AP_L1_Control`
  transitively includes `<AP_AHRS/AP_AHRS.h>`. A small shim directory placed **first** on
  the include path intercepts that include so the token `AP_AHRS` resolves to the
  lightweight, injectable `AfsimL1_AHRS_Shim`. The controller therefore binds the shim and
  reads exactly the host-injected state, while the guidance **math** remains the real
  `AP_L1_Control` compiled against the real `AP_Math` / `AP_Common/Location` geometry and
  the real `AP_Param` defaults — so the numbers are preserved bit-for-bit. This keeps the
  `.so` small and free of the EKF/DCM stack. **Both shipped build files set this up for
  you** — the standalone `CMakeLists.txt` and the in-tree `wscript` each generate the shim
  tree, put it first on the include path, and define the `AFSIML1_L1_USES_SHIM_AHRS` macro
  that signals the seam is active, which is why the two builds produce identical output.
- **Option A — real `AP_AHRS` (documented fallback, maximum fidelity).** Link the real
  `AP_AHRS` together with its transitive EKF/DCM dependency graph and drive it from an
  external navigation source. Behaviour-identical, but far heavier; retained only as the
  documented alternative and not built by the shipped CMake target.

Because binding the controller to the shim is only correct under the seam, the façade guards
the requirement at compile time: `AfsimL1Behavior.h` opens (before any include) with a hard
`#error` that fires unless `AFSIML1_L1_USES_SHIM_AHRS` is defined. Any build that compiles
the façade **without** the seam therefore fails loudly with a directive to use one of the two
seam-providing build files, rather than silently compiling a service whose controller reads a
never-written state source. Consuming the service through the C ABI (`l1_c_api.h`, Path B
below) never touches that header, so an external host is unaffected by the guard.

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

All eight entry points are exported with `__attribute__((visibility("default")))`, and the
library is built with `-fvisibility=hidden` so that every symbol from the service's own code —
the facade, the AHRS shim, and the wrapped `AP_L1_Control` / `AP_Math` / `AP_Common` sources —
stays internal. To pin the exported surface to *exactly* those eight functions, the shared
library is additionally linked with a GNU-ld **version script** that exports only the eight
`L1_*` entry points and localizes everything else — including the C++ standard-library
vague-linkage/COMDAT template instantiations that `-fvisibility=hidden` alone cannot suppress.
The eight `L1_*` functions are therefore the **only** symbols the library exports; see
**Verifying the exported symbols** below.

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

This produces two artifacts: the shared library **`libafsim_l1.so`** (the deliverable an
external host binds) and **`afsim_l1_demo`**, a small runnable driver built from
[`main.cpp`](main.cpp) and linked against the library. The build links only in-tree
ArduPilot sources plus the local service sources; it introduces no new third-party
dependency. Run the demo — it executes one guidance step and exits `0` on success:

```bash
$ ./afsim_l1_demo
roll_deg = -38.639999, lat_accel = -7.840306
```

### Verifying the exported symbols

The library is compiled with `-fvisibility=hidden` **and** linked with a version script (see
[Building](#standalone-shared-library-cmake) and `CMakeLists.txt`), so all of the service's own
code — the facade, the AHRS shim, and the wrapped `AP_L1_Control` / `AP_Math` / `AP_Common`
sources — stays internal, and the eight C-ABI entry points are the **only** symbols the library
exports. They appear as global text (`T`) symbols; list them with the bare, unfiltered command:

```bash
$ nm -D --defined-only libafsim_l1.so
```

The output lists **exactly** these eight `L1_*` symbols and no others (shown here sorted by
name; each is a global `T` entry):

```text
L1_Create
L1_Destroy
L1_Execute
L1_GetLatAccel
L1_GetRollDeg
L1_Init
L1_SetLegNE
L1_SetStateNE
```

No other symbols appear. The version script localizes everything the service links, including
the C++ standard-library vague-linkage/COMDAT template instantiations (e.g. `std::_Hashtable<…>`
from the internal `std::unordered_set` live-handle registry) that `-fvisibility=hidden` alone
cannot suppress — so the bare `nm -D --defined-only libafsim_l1.so` above is exactly eight lines,
which are the complete, intended C-ABI contract.

### Running the unit tests

The same CMake build also produces **`afsim_l1_tests`**, a self-contained suite (no external
test framework) covering the AHRS shim accessors and setters, the East/North velocity
convention, the `Location`-from-datum construction, every facade method, all eight C-ABI
entry points including `NULL`- and destroyed-handle safety, and — most importantly —
**behavior preservation**: injected-`dt` determinism, the default-off legacy
`AP_HAL::micros()` path, and the `dt > 1 s` cross-track-integrator reset clamp. Run it
directly, or through CTest:

```bash
$ ./afsim_l1_tests
=== AfsimL1 unit tests: 111 checks, 0 failures ===

$ ctest --output-on-failure
100% tests passed, 0 tests failed out of 2
```

CTest registers two cases: `afsim_l1_unit_tests` (the suite above) and
`afsim_l1_demo_smoke` (asserts the demo still runs and exits `0`).

### In-tree ArduPilot example (waf)

The sibling `wscript` registers the module as an in-tree ArduPilot example following the
established convention (`bld.ap_example(use='ap')`, matching the reference
`AP_AHRS/examples/AHRS_Test/wscript`):

```bash
$ ./waf configure --board linux
$ ./waf build --targets examples/AfsimL1
$ ./build/linux/examples/AfsimL1
roll_deg = -38.639999, lat_accel = -7.840306
```

This path builds the **same Option-B seam** the CMake target uses, so it is a genuine
second build of the service rather than a degraded variant: the `wscript` generates the shim
tree into the build directory, places it **first** on the include path, defines
`AFSIML1_L1_USES_SHIM_AHRS`, and compiles the wrapped `AP_L1_Control.cpp` from its
unmodified in-tree location so the controller binds the shim. The resulting binary prints
**byte-identical** guidance output to the CMake `afsim_l1_demo`, which is the practical
proof that both builds compute the same numbers.

The two paths differ only in what they *emit*: the in-tree target produces the runnable
**example executable**, whereas the CMake target additionally produces the **`libafsim_l1.so`
shared library with the pinned eight-symbol export list** — the artifact an external host
binds at run time. **For the `.so`, use the CMake build.**

`--board sitl` builds the example equally well, but currently needs a one-line environment
prefix that relaxes a single warning in vendored third-party code unrelated to this service
(`modules/littlefs` has an unused local that trips the board's `-Werror=unused-variable`):

```bash
$ CFLAGS='-Wno-error=unused-variable' ./waf configure --board sitl
$ CFLAGS='-Wno-error=unused-variable' ./waf build --targets examples/AfsimL1
```

Prefer `--board linux`, which needs no prefix and keeps ArduPilot's full strict-diagnostics
posture (29 explicit `-Werror=` C++ flags) in force over the service sources.

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
a materially non-zero roll — is provided in [`main.cpp`](main.cpp). The CMake build compiles
it into the **`afsim_l1_demo`** executable alongside the library (see **Building** above), so
you can run the full flow end-to-end; the demo self-checks its outputs and exits non-zero on
a state-flow regression.

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

Unlike Path B, Path A compiles the façade header, so it **requires the Option-B compile-time
seam** (see **AHRS state decoupling**): `AFSIML1_L1_USES_SHIM_AHRS` must be defined and the
generated shim directory must be first on the include path — exactly as the CMake target
arranges. Without that setup, `#include "AfsimL1Behavior.h"` fails at the façade's `#error`.
The simplest way to satisfy it is to add the host translation unit to the CMake target's
sources so it inherits the same seam and define. Hosts that merely load the shared library
at run time should prefer **Path B**, which never compiles any C++ header and is unaffected
by the guard.

## Behavior preservation

This module is a **structural extraction, not an algorithm change**. The L1 control law,
its gains, and its geometry live untouched inside `AP_L1_Control`; the facade, the AHRS
shim, and the C ABI only move the input/output boundaries. The single edit to the wrapped
controller — the additive, default-off `set_update_dt` timing seam — leaves every existing
`AP_L1_Control` signature intact and, when unused, reproduces the stock hardware-clock
behavior exactly. Vehicle firmware (ArduPlane and the other vehicles) is unaffected and
continues to use the controller exactly as before.

