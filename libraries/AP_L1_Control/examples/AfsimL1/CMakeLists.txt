# =============================================================================
#  AfsimL1 — standalone shared-library build (libafsim_l1.so)
# =============================================================================
#
#  PURPOSE
#  -------
#  Builds the AfsimL1 Position/Navigation/Timing (PNT) guidance service into a
#  single self-contained shared object, `libafsim_l1.so`, that an EXTERNAL host
#  process (e.g. the AFSIM simulation environment) can load and drive at runtime
#  through the stable `extern "C"` ABI declared in `l1_c_api.h`. This is the
#  "external process that does not link the ArduPilot firmware image" delivery
#  target described in the Agent Action Plan (§0.2.1, §0.3.1, §0.5.1).
#
#  The service reuses the real `AP_L1_Control` lateral-navigation controller
#  unchanged (behaviour-preserving extraction) and re-plumbs its three coupling
#  points — legs, AHRS state, and the control-step `dt` — to host-supplied
#  injection points behind a C ABI. Only plain `double`/`void*` scalars cross
#  the boundary, so AFSIM can bind the library regardless of its own toolchain.
#
#  Patterned on the verified reference standalone build
#  `libraries/SITL/examples/JSON/C++/CMakeLists.txt` (cmake_minimum_required 3.5
#  + CMAKE_CXX_STANDARD 11), adapted from `add_executable` to
#  `add_library(... SHARED ...)`.
#
#  Build & verify:
#      mkdir build && cd build && cmake .. && make
#      nm -D --defined-only libafsim_l1.so   # → exactly 8 L1_* symbols
#
# -----------------------------------------------------------------------------
#  CENTRAL ENGINEERING DECISION — how the AP_AHRS dependency is satisfied
#  (Agent Action Plan §0.6.2)
# -----------------------------------------------------------------------------
#  `AP_L1_Control` binds a concrete `AP_AHRS &` and reads six non-virtual
#  accessors from it (get_location, groundspeed_vector, get_yaw_rad,
#  yaw_sensor, get_pitch_rad, get_EAS2TAS). `AP_L1_Control.h` transitively pulls
#  <AP_AHRS/AP_AHRS.h> via <AP_TECS/AP_TECS.h>, so decoupling the controller
#  from the live sensor-fusion stack requires intercepting that include chain.
#
#    * Option A (max fidelity) — link the real `AP_AHRS` plus its full
#      transitive dependency graph (the EKF/DCM estimator stack, storage,
#      parameter subsystem, HAL backend). Numerically identical to firmware but
#      pulls a very large source graph into a "standalone" .so, defeating the
#      goal of a small, host-loadable library.
#
#    * Option B (RECOMMENDED, implemented here) — bind the lightweight,
#      service-local `AfsimL1_AHRS_Shim` in place of the full `AP_AHRS` via a
#      compile-time include seam, and compile only the minimal, behaviour-
#      preserving set of real in-tree sources the controller actually needs.
#      This yields a lean (~64 KB) .so with an identical L1 guidance result
#      because the shim reproduces the exact read surface the controller
#      consumes and the tuning parameters are seeded to their stock NAVL1_
#      defaults.
#
#  This file implements Option B. The seam is realised entirely inside the build
#  tree (generated below with `file(WRITE ...)`), so the ONLY committed artefact
#  is this CMakeLists.txt — no source file outside this example is modified, and
#  no vehicle firmware is referenced.
# =============================================================================

cmake_minimum_required(VERSION 3.5)
project(afsim_l1 CXX)

# --- Language standard: match ArduPilot's gnu++11 (AAP §0.5.1) ---------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Shared library requires position-independent code -----------------------
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- Symbol-visibility hardening (AAP §0.6.4) --------------------------------
#  Hide every symbol by default so that ONLY the eight
#  `__attribute__((visibility("default")))` C-ABI entry points in `l1_c_api.cpp`
#  are exported. Downstream verifies with:
#      nm -D --defined-only libafsim_l1.so
#  which must list exactly: L1_Create, L1_Destroy, L1_Init, L1_Execute,
#  L1_SetLegNE, L1_SetStateNE, L1_GetRollDeg, L1_GetLatAccel.
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# =============================================================================
#  Path computation
# =============================================================================
#  This file lives at:
#      libraries/AP_L1_Control/examples/AfsimL1/CMakeLists.txt
#  so the repository `libraries/` root is three directories up. Angle-bracket
#  includes such as <AP_L1_Control/AP_L1_Control.h> and <AP_Common/Location.h>
#  resolve against this root.
get_filename_component(AP_LIBRARIES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

#  Generated compile-time include seam (Option B) and support translation unit
#  are emitted into the build tree, never into the source tree.
set(AFSIM_SEAM_DIR    "${CMAKE_CURRENT_BINARY_DIR}/afsim_seam")
set(AFSIM_SUPPORT_DIR "${CMAKE_CURRENT_BINARY_DIR}/afsim_support")

# =============================================================================
#  Generate the compile-time include seam (Option B)
# =============================================================================
#  The seam directory is placed FIRST on the include path (see
#  target_include_directories below) so these headers shadow the real
#  <AP_AHRS/...>, <AP_TECS/...>, <AP_Terrain/...> and <AP_Param/...> headers,
#  while the real <AP_HAL/...>, <AP_Math/...> and <AP_Common/...> headers pass
#  through unshadowed.

#  (1) AP_AHRS seam — alias the concrete `AP_AHRS` type to the service-local
#      shim. The shim is referenced through the libraries root so no absolute
#      path is baked in. The shim provides the byte-identical read surface the
#      controller consumes.
file(WRITE "${AFSIM_SEAM_DIR}/AP_AHRS/AP_AHRS.h" [==[
#pragma once
// Auto-generated by CMakeLists.txt (Option B include seam). DO NOT EDIT.
// Redirects the controller's `AP_AHRS` dependency to the lightweight,
// host-injected AfsimL1_AHRS_Shim instead of the full sensor-fusion stack.
#include <AP_L1_Control/examples/AfsimL1/AfsimL1_AHRS_Shim.h>
using AP_AHRS = AfsimL1_AHRS_Shim;
]==])

#  (2) AP_TECS seam — the controller's constructor accepts a `const AP_TECS *`
#      and only calls `get_target_airspeed()` (guarded by a null check). A
#      minimal stand-in satisfies the type without pulling the energy-control
#      subsystem; the service passes nullptr, so this body is never invoked.
file(WRITE "${AFSIM_SEAM_DIR}/AP_TECS/AP_TECS.h" [==[
#pragma once
// Auto-generated by CMakeLists.txt (Option B include seam). DO NOT EDIT.
#include <AP_AHRS/AP_AHRS.h>
class AP_TECS {
public:
    float get_target_airspeed() const { return 0.0f; }
};
]==])

#  (3) AP_Terrain seam — the real AP_Terrain pulls the generated GCS_MAVLink
#      headers. AfsimL1's relative-navigation frame never uses terrain, and all
#      terrain code in Location.cpp is compiled out via AP_TERRAIN_AVAILABLE=0,
#      so a forward declaration plus the AP::terrain() accessor prototype is
#      sufficient to satisfy the unconditional #include.
file(WRITE "${AFSIM_SEAM_DIR}/AP_Terrain/AP_Terrain.h" [==[
#pragma once
// Auto-generated by CMakeLists.txt (Option B include seam). DO NOT EDIT.
class AP_Terrain;
namespace AP { AP_Terrain *terrain(); }
]==])

#  (4) AP_Param seam — a lightweight, behaviour-equivalent parameter type.
#      The real AP_Param.cpp drags the storage/EEPROM/HAL persistence stack; the
#      controller only needs (a) the `AP_Float` scalar type and (b)
#      `AP_Param::setup_object_defaults()` to seed its NAVL1_ tuning gains from
#      the AP_GROUPINFO table. This seam reproduces exactly that: the generated
#      setup_object_defaults walks the GroupInfo table and applies each stock
#      default (PERIOD=17, DAMPING=0.75, XTRACK_I=0.02, LIM_BANK=0.0) by member
#      offset — numerically identical to firmware defaults.
file(WRITE "${AFSIM_SEAM_DIR}/AP_Param/AP_Param.h" [==[
#pragma once
// Auto-generated by CMakeLists.txt (Option B include seam). DO NOT EDIT.
#include <cstdint>
#include <cstddef>

enum ap_var_type {
    AP_PARAM_NONE = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_GROUP,
};

// Minimal drop-in for AP_Float: stores a single float and exposes the accessor
// surface AP_L1_Control uses (implicit float conversion, get/set, defaults).
class AP_Float {
public:
    AP_Float() : _v(0.0f) {}
    AP_Float(float v) : _v(v) {}
    void set(const float v) { _v = v; }
    void set_default(const float v) { _v = v; }
    void set_and_default(const float v) { _v = v; }
    float get() const { return _v; }
    operator float() const { return _v; }
    AP_Float &operator=(const float v) { _v = v; return *this; }
private:
    float _v;
};

#define AP_PARAMDEF(_t, _s, _pt)  typedef _t AP_##_s
#define AP_PARAMDEFV(_t, _s, _pt) typedef _t AP_##_s

class AP_Param {
public:
    struct GroupInfo {
        const char *name;
        ptrdiff_t   offset;
        float       def_value;
    };
    // Seed each AP_Float member of `obj` from its GroupInfo default value,
    // mirroring the stock AP_Param::setup_object_defaults() contract.
    static void setup_object_defaults(const void *obj, const struct GroupInfo *gi) {
        if (!obj || !gi) {
            return;
        }
        for (const GroupInfo *p = gi; p->name && p->name[0]; ++p) {
            AP_Float *f = reinterpret_cast<AP_Float *>(
                reinterpret_cast<uint8_t *>(const_cast<void *>(obj)) + p->offset);
            f->set(p->def_value);
        }
    }
};

#define AP_VAROFFSET(type, element) (((ptrdiff_t)(&((const type *)1)->element)) - 1)
#define AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags) \
    { name, AP_VAROFFSET(clazz, element), (float)(def) }
#define AP_GROUPINFO(name, idx, clazz, element, def) \
    AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, 0)
#define AP_GROUPEND { "", 0, 0.0f }
]==])

# =============================================================================
#  Generate the support translation unit
# =============================================================================
#  A handful of symbols that AP_L1_Control and the reused geometry sources
#  reference live in heavy ArduPilot translation units we deliberately do NOT
#  compile (the AP_HAL board backend, and AP_Math.cpp — which, compiled whole,
#  drags unused rotation/matrix/internal-error symbols). This support TU
#  supplies exactly those leaf symbols, with implementations byte-faithful to
#  the stock AP_Math semantics:
#     * AP_HAL::micros/millis/micros64/millis64 (monotonic std::chrono clock)
#       and AP_HAL::panic;
#     * wrap_180_cd<T>, wrap_2PI, wrap_PI, constrain_value_line<float>;
#     * is_equal integral + floating overloads (used by Vector2 ==/!=).
file(WRITE "${AFSIM_SUPPORT_DIR}/afsim_l1_support.cpp" [==[
// =============================================================================
//  afsim_l1_support.cpp  —  AUTO-GENERATED by CMakeLists.txt. DO NOT EDIT.
// =============================================================================
//  Satisfies the small, well-defined set of external symbols that
//  AP_L1_Control + the reused geometry sources reference but that live in heavy
//  ArduPilot TUs deliberately excluded from this standalone build (the AP_HAL
//  board backend and the unusable-whole AP_Math.cpp). Every reimplementation is
//  byte-faithful to the stock AP_Math semantics, preserving guidance behaviour.
// =============================================================================
#include <cstdint>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <limits>
#include <type_traits>

#include <AP_Math/AP_Math.h>   // ftype, M_PI, M_2PI

// ---- AP_HAL time + panic ---------------------------------------------------
namespace AP_HAL {
    static uint64_t boot_us() {
        using namespace std::chrono;
        static const auto t0 = steady_clock::now();
        return (uint64_t)duration_cast<microseconds>(steady_clock::now() - t0).count();
    }
    uint32_t micros()   { return (uint32_t)boot_us(); }
    uint32_t millis()   { return (uint32_t)(boot_us() / 1000ULL); }
    uint64_t micros64() { return boot_us(); }
    uint64_t millis64() { return boot_us() / 1000ULL; }
    void panic(const char *fmt, ...) {
        va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
        fprintf(stderr, "\n"); abort();
    }
}

// ---- AP_Math leaf functions (faithful, self-contained reimplementations) ---
template <typename T>
T wrap_180_cd(const T angle) {
    double a = fmod((double)angle, 36000.0);
    if (a < 0) a += 36000.0;         // wrap_360_cd -> [0, 36000)
    if (a > 18000.0) a -= 36000.0;   // fold to (-18000, 18000]
    return (T)a;
}
template float wrap_180_cd<float>(const float);
template int   wrap_180_cd<int>(const int);

ftype wrap_2PI(const ftype radian) {
    ftype res = fmod(radian, M_2PI);
    if (res < 0) res += M_2PI;
    return res;
}
ftype wrap_PI(const ftype radian) {
    ftype res = wrap_2PI(radian);
    if (res > M_PI) res -= M_2PI;
    return res;
}

template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line) {
    (void)line;
    if (std::isnan(amt)) return (low + high) / 2;
    if (amt < low)  return low;
    if (amt > high) return high;
    return amt;
}
template float constrain_value_line<float>(const float, const float, const float, uint32_t);

// is_equal(): integral overload — direct comparison in the common type.
template <typename A1, typename A2>
typename std::enable_if<std::is_integral<typename std::common_type<A1, A2>::type>::value, bool>::type
is_equal(const A1 v_1, const A2 v_2) {
    typedef typename std::common_type<A1, A2>::type common_type;
    return static_cast<common_type>(v_1) == static_cast<common_type>(v_2);
}
// is_equal(): floating overload — epsilon comparison (double-aware on SITL).
template <typename A1, typename A2>
typename std::enable_if<std::is_floating_point<typename std::common_type<A1, A2>::type>::value, bool>::type
is_equal(const A1 v_1, const A2 v_2) {
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
    typedef typename std::common_type<A1, A2>::type common_type;
    typedef typename std::remove_cv<common_type>::type common_type_nonconst;
    if (std::is_same<double, common_type_nonconst>::value) {
        return fabs(v_1 - v_2) < std::numeric_limits<double>::epsilon();
    }
#endif
    return fabsf(v_1 - v_2) < std::numeric_limits<float>::epsilon();
}
template bool is_equal<int>(const int, const int);
template bool is_equal<short>(const short, const short);
template bool is_equal<long>(const long, const long);
template bool is_equal<float>(const float, const float);
template bool is_equal<double>(const double, const double);
]==])

# =============================================================================
#  Sources
# =============================================================================
#  (a) The three service translation units (the façade, the AHRS shim, and the
#      C-ABI boundary that carries the 8 exported symbols).
#  (b) The wrapped guidance controller, reused unchanged.
#  (c) The minimal set of real, behaviour-preserving geometry sources the
#      controller needs (Vector2 math + Location distance/bearing). Compiling
#      these REAL sources — rather than reimplementing them — guarantees the L1
#      navigation result is numerically identical to firmware.
#  (d) The generated support TU above.
#
#  NOTE: main.cpp is INTENTIONALLY EXCLUDED — it defines main() and is the
#  separate C-ABI demonstration driver, not part of the shared library.
set(SOURCES
    # (a) service
    "${CMAKE_CURRENT_SOURCE_DIR}/AfsimL1Behavior.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/AfsimL1_AHRS_Shim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/l1_c_api.cpp"
    # (b) wrapped controller (reused unchanged)
    "${AP_LIBRARIES_ROOT}/AP_L1_Control/AP_L1_Control.cpp"
    # (c) real, behaviour-preserving geometry
    "${AP_LIBRARIES_ROOT}/AP_Math/vector2.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Common/Location.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Math/location.cpp"
    # (d) generated support translation unit
    "${AFSIM_SUPPORT_DIR}/afsim_l1_support.cpp"
)

# =============================================================================
#  Target — the shared library libafsim_l1.so
# =============================================================================
add_library(afsim_l1 SHARED ${SOURCES})

#  Include search order MATTERS: the generated seam directory must come FIRST so
#  that <AP_AHRS/...>, <AP_TECS/...>, <AP_Terrain/...> and <AP_Param/...> resolve
#  to the Option-B seams, while the libraries root resolves every other
#  <AP_.../...> angle include to the real header, and the current directory
#  resolves the service's own "AfsimL1*.h" quote includes.
target_include_directories(afsim_l1 PRIVATE
    "${AFSIM_SEAM_DIR}"
    "${AP_LIBRARIES_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

#  Compile definitions select the SITL board profile and disable the terrain /
#  AHRS-dependent code paths that would otherwise pull heavy subsystems. These
#  mirror the flags the in-tree waf build injects for the SITL board:
#    * CONFIG_HAL_BOARD / _SUBTYPE  — select the SITL board so AP_HAL/AP_Math
#      headers configure themselves for a hosted (double-precision) target.
#    * __AP_LINE__                  — waf injects this as -D__AP_LINE__=__LINE__.
#    * AP_AHRS_ENABLED=0            — compile out live-AHRS branches in the
#      reused geometry (state is injected via the shim instead).
#    * AP_TERRAIN_AVAILABLE=0       — compile out terrain branches in Location.
#    * AP_INTERNALERROR_ENABLED=0   — drop the internal-error reporting path.
#    * ALLOW_DOUBLE_MATH_FUNCTIONS  — matches SITL double-precision math.
target_compile_definitions(afsim_l1 PRIVATE
    CONFIG_HAL_BOARD=HAL_BOARD_SITL
    CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE
    __AP_LINE__=__LINE__
    AP_AHRS_ENABLED=0
    AP_TERRAIN_AVAILABLE=0
    AP_INTERNALERROR_ENABLED=0
    ALLOW_DOUBLE_MATH_FUNCTIONS
)

#  `lib` prefix and `.so` suffix are applied automatically on Linux, yielding
#  libafsim_l1.so.
set_target_properties(afsim_l1 PROPERTIES
    OUTPUT_NAME afsim_l1
    # Fail the link if ANY symbol is unresolved — guarantees a fully-closed
    # symbol graph (no lazy runtime surprises for the external host).
    LINK_FLAGS "-Wl,--no-undefined"
)

#  Link the C math library (trig/sqrt used by the guidance geometry).
target_link_libraries(afsim_l1 PRIVATE m)

# =============================================================================
#  Demonstration driver — afsim_demo (the "external host" stand-in)
# =============================================================================
#  main.cpp is the runnable "initialize a simple leg" demo (AAP §0.2.1, §0.7.2).
#  It is deliberately NOT part of the shared library (it defines main() and is
#  excluded from ${SOURCES} above); it is a SEPARATE consumer that binds the
#  service EXCLUSIVELY through the stable `extern "C"` boundary (l1_c_api.h) and
#  links against libafsim_l1.so — exactly how an external, differently-compiled
#  host such as AFSIM consumes the library (AAP §0.6.4). Building it here as a
#  first-class target proves that ABI decoupling end-to-end: `cmake .. && make`
#  produces BOTH libafsim_l1.so and a runnable ./afsim_demo.
#
#  Critically, AFSIM_L1_WAF_BUILD is NOT defined for this standalone build, so
#  the in-tree waf link glue in main.cpp (the process-global `hal` that libap
#  needs) is compiled out. The demo therefore needs neither <AP_HAL/AP_HAL.h>
#  nor libap — only l1_c_api.h (a pure C header with no includes) and the eight
#  exported L1_* symbols in the .so. CMake automatically bakes the build-tree
#  location of libafsim_l1.so into the executable's RPATH, so ./afsim_demo runs
#  in place from the build directory without setting LD_LIBRARY_PATH.
add_executable(afsim_demo "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

#  main.cpp includes "l1_c_api.h" (a quote include resolved from its own
#  directory) — add the source dir explicitly so the target is robust regardless
#  of the caller's working directory.
target_include_directories(afsim_demo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

#  Link the demo against the service shared library. This resolves the eight
#  L1_* C-ABI symbols from libafsim_l1.so and nothing else — the demo is a pure
#  C-ABI consumer, so it does NOT need -lm or any ArduPilot internal.
target_link_libraries(afsim_demo PRIVATE afsim_l1)

# =============================================================================
#  Post-build validation guidance (informational)
# =============================================================================
#    nm -D --defined-only $<TARGET_FILE:afsim_l1>
#  MUST list exactly the eight C-ABI entry points and nothing else:
#    L1_Create  L1_Destroy  L1_Init  L1_Execute
#    L1_SetLegNE  L1_SetStateNE  L1_GetRollDeg  L1_GetLatAccel
#
#  The demonstration driver is built automatically by the `afsim_demo` target
#  above: `cmake .. && make` produces BOTH libafsim_l1.so and the afsim_demo
#  executable. Run it in place from the build directory (its RPATH already
#  points at the co-located .so):
#    ./afsim_demo
#
#  To build the demo BY HAND instead — e.g. to mimic an out-of-tree host that
#  has only the C-ABI header and the shared object — run the following from the
#  build directory that holds libafsim_l1.so. Note that AFSIM_L1_WAF_BUILD is
#  intentionally NOT defined, so main.cpp's in-tree HAL glue is compiled out and
#  the demo binds purely through the C ABI (no AP_HAL, no libap). `../main.cpp`
#  is the path to the demo source from inside the build directory:
#    g++ -std=gnu++11 -I.. ../main.cpp -L. -lafsim_l1 -Wl,-rpath,. -o afsim_demo
# =============================================================================
