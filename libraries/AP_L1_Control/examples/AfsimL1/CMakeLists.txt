# =============================================================================
#  libraries/AP_L1_Control/examples/AfsimL1/CMakeLists.txt
#
#  Standalone CMake build for the AfsimL1 L1-lateral-navigation guidance
#  service.  Produces a SHARED library:
#
#          libafsim_l1.so
#
#  This is the "external process that does not link the ArduPilot firmware
#  image" build target (AAP 0.2.1 / 0.3.1 / 0.5.1).  An external host such as
#  the AFSIM simulation environment binds this .so at run time and drives the
#  extracted L1 guidance behaviour through the stable, mangling-free C ABI
#  declared in l1_c_api.h.  The build is deliberately self-contained: it links
#  ONLY in-tree ArduPilot sources plus the local service sources -- no new
#  third-party dependency is introduced (AAP 0.5.2).
#
#  Patterned on the verified reference standalone build
#  libraries/SITL/examples/JSON/C++/CMakeLists.txt (which uses
#  cmake_minimum_required(VERSION 3.5), project(), set(CMAKE_CXX_STANDARD 11)
#  and add_executable()).  Here the target is a SHARED LIBRARY, so
#  add_executable() is replaced by add_library(... SHARED ...).
#
#  Build & verify:
#      mkdir build && cd build && cmake .. && make
#      nm -D --defined-only libafsim_l1.so   # exactly 8 L1_* symbols
#
# -----------------------------------------------------------------------------
#  CENTRAL ENGINEERING DECISION -- how the AP_AHRS state dependency is met
#  (AAP 0.6.2).  AP_L1_Control binds a concrete "AP_AHRS &" and reads six
#  NON-VIRTUAL accessors from it (get_location, groundspeed_vector,
#  get_yaw_rad, yaw_sensor, get_pitch_rad, get_EAS2TAS).  Because those
#  accessors are non-virtual, a plain subclass cannot override them, so the
#  service must decouple the state source another way.  Two options exist:
#
#    * Option A (fallback, maximum fidelity) -- link the REAL AP_AHRS together
#      with its full transitive dependency graph (AP_NavEKF2/3, DCM, AP_GPS,
#      AP_Baro, AP_InertialSensor, ...).  This is behaviour-identical but pulls
#      a very large slice of the firmware into the .so and needs the vehicle
#      HAL, so the resulting library is heavy and slow to build.  Retained here
#      only as the documented alternative.
#
#    * Option B (IMPLEMENTED, recommended) -- a compile-time include seam.
#      AP_L1_Control transitively includes <AP_AHRS/AP_AHRS.h> (via AP_TECS.h).
#      By placing a small generated shim directory FIRST on the include path we
#      intercept that include so the L1 translation unit binds the lightweight,
#      injectable AfsimL1_AHRS_Shim instead of the full AP_AHRS.  The guidance
#      MATH is still the REAL AP_L1_Control compiled against the REAL AP_Math /
#      AP_Common/Location geometry and the REAL AP_Param defaults, so the
#      numerical behaviour is preserved bit-for-bit (AAP 0.7.1).  Only the
#      genuine refactor injection points are shimmed: AP_AHRS -> the state
#      shim, AP_TECS -> a null-airspeed stub (the controller only ever uses a
#      nullptr TECS here), and AP_HAL -> a minimal micros()/millis() clock (the
#      host owns timing via set_update_dt()).  This keeps libafsim_l1.so small
#      and free of the EKF/DCM stack.
#
#  The shim headers and the tiny "seam" translation unit are GENERATED into the
#  build tree by this CMakeLists (file(WRITE ...)) so the whole reproducible
#  recipe lives in this single build file; nothing outside examples/AfsimL1/ is
#  modified.
# =============================================================================

# ------------------------------- Phase 1: Preamble ---------------------------
# cmake_minimum_required and CMAKE_CXX_STANDARD match the reference build and
# AAP 0.5.1 (ArduPilot compiles with -std=gnu++11).  CMAKE_CXX_EXTENSIONS is
# left at its default (ON) so the effective standard is gnu++11, exactly what
# the ArduPilot sources expect.
cmake_minimum_required(VERSION 3.5)
project(afsim_l1 CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# A shared library and every object linked into it must be position independent.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------- Phase 2: Symbol-visibility hardening -----------------
# Hide every symbol by default so ONLY the eight C-ABI entry points in
# l1_c_api.cpp -- each marked __attribute__((visibility("default"))) -- are
# exported (AAP 0.6.4).  Downstream verification:
#     nm -D --defined-only libafsim_l1.so
# must list exactly: L1_Create, L1_Destroy, L1_Init, L1_Execute, L1_SetLegNE,
# L1_SetStateNE, L1_GetRollDeg, L1_GetLatAccel.
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# ------------------------- Phase 3: Include-path roots -----------------------
# This file lives at libraries/AP_L1_Control/examples/AfsimL1/CMakeLists.txt, so
# the repository "libraries/" root is three directories up.  Adding it to the
# include path lets angle-bracket includes such as <AP_L1_Control/AP_L1_Control.h>
# and <AP_Common/Location.h> resolve against the in-tree libraries.
get_filename_component(AP_LIBRARIES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

# =============================================================================
#  Option B include seam -- generate the shim tree into the build directory.
#
#  The generated headers shadow the heavyweight ArduPilot headers ONLY for the
#  three injection points named above.  They are written into
#  ${CMAKE_CURRENT_BINARY_DIR}/afsim_shim and that directory is placed FIRST on
#  the include path so it wins the header search for <AP_AHRS/AP_AHRS.h>,
#  <AP_TECS/AP_TECS.h>, <AP_HAL/AP_HAL.h>, <AP_Param/AP_Param.h> and
#  <AP_Terrain/AP_Terrain.h>.  All other ArduPilot headers (AP_Math, AP_Common,
#  AP_Navigation, AP_InternalError, AP_CustomRotations, ...) fall through to the
#  REAL sources under AP_LIBRARIES_ROOT so the guidance math is unchanged.
# =============================================================================
set(AFSIM_SHIM_DIR "${CMAKE_CURRENT_BINARY_DIR}/afsim_shim")
file(MAKE_DIRECTORY "${AFSIM_SHIM_DIR}/AP_Param")
file(MAKE_DIRECTORY "${AFSIM_SHIM_DIR}/AP_HAL")
file(MAKE_DIRECTORY "${AFSIM_SHIM_DIR}/AP_AHRS")
file(MAKE_DIRECTORY "${AFSIM_SHIM_DIR}/AP_TECS")
file(MAKE_DIRECTORY "${AFSIM_SHIM_DIR}/AP_Terrain")

# ---- AP_Param shim ----------------------------------------------------------
# A minimal, storage-free AP_Param.  AP_Float/AP_Int* become tiny value holders
# and setup_object_defaults() writes each AP_GROUPINFO default into the member
# at its offset.  This is what applies the L1 tuning defaults (PERIOD=17,
# DAMPING=0.75, XTRACK_I=0.02, LIM_BANK=0.0) that the guidance math depends on,
# WITHOUT dragging in AP_Param's real EEPROM/StorageManager/AP_HAL graph.
file(WRITE "${AFSIM_SHIM_DIR}/AP_Param/AP_Param.h"
[==[#pragma once
#include <stdint.h>
#include <stddef.h>
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

template <typename T>
class AP_ParamShimT {
public:
    AP_ParamShimT() : _v(T(0)) {}
    AP_ParamShimT(T v) : _v(v) {}
    void set(T v) { _v = v; }
    void set_default(T v) { _v = v; }
    void set_and_default(T v) { _v = v; }
    T get() const { return _v; }
    operator T() const { return _v; }
    AP_ParamShimT<T>& operator=(T v) { _v = v; return *this; }
private:
    T _v;
};
typedef AP_ParamShimT<float>   AP_Float;
typedef AP_ParamShimT<int8_t>  AP_Int8;
typedef AP_ParamShimT<int16_t> AP_Int16;
typedef AP_ParamShimT<int32_t> AP_Int32;

class AP_Param {
public:
    struct GroupInfo {
        const char* name;
        size_t offset;
        float def_value;
    };
    template <class T>
    static void setup_object_defaults(T* object, const GroupInfo* group_info) {
        uint8_t* base = reinterpret_cast<uint8_t*>(object);
        for (uint8_t i = 0; group_info[i].name != nullptr && group_info[i].name[0] != '\0'; i++) {
            AP_Float* f = reinterpret_cast<AP_Float*>(base + group_info[i].offset);
            f->set(group_info[i].def_value);
        }
    }
};

#define AP_GROUPINFO(name, idx, clazz, element, def) { name, offsetof(clazz, element), (float)(def) }
#define AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags) { name, offsetof(clazz, element), (float)(def) }
#define AP_GROUPEND { nullptr, 0, 0.0f }
#define AP_PARAMDEFV(_type, _name, _pt) typedef _type AP_##_name
]==])

# ---- AP_HAL shim ------------------------------------------------------------
# Only the clock primitives the guidance touches are provided.  micros()/millis()
# use a monotonic std::chrono steady_clock; panic() aborts.  The host supplies
# the control-step dt through set_update_dt(), so this clock only backs the
# default (un-injected) timing path.
file(WRITE "${AFSIM_SHIM_DIR}/AP_HAL/AP_HAL.h"
[==[#pragma once
#include <stdint.h>
#include <chrono>
#include <cstdlib>
namespace AP_HAL {
    class HAL;
    inline uint32_t micros() {
        using namespace std::chrono;
        static const auto t0 = steady_clock::now();
        return (uint32_t)duration_cast<microseconds>(steady_clock::now() - t0).count();
    }
    inline uint32_t millis() {
        using namespace std::chrono;
        static const auto t0 = steady_clock::now();
        return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count();
    }
    [[noreturn]] inline void panic(const char*, ...) { abort(); }
}
]==])

# ---- AP_AHRS shim -----------------------------------------------------------
# Alias AP_AHRS to the local, injectable AfsimL1_AHRS_Shim so that every
# "AP_AHRS &" in AP_L1_Control actually refers to the shim.  <AfsimL1_AHRS_Shim.h>
# resolves against CMAKE_CURRENT_SOURCE_DIR (added to the include path below).
# The AP:: home/origin view is a tiny read-only stand-in used only by the
# never-exercised altitude-frame helpers in AP_Common/Location.cpp.
file(WRITE "${AFSIM_SHIM_DIR}/AP_AHRS/AP_AHRS.h"
[==[#pragma once
#include <AfsimL1_AHRS_Shim.h>
using AP_AHRS = AfsimL1_AHRS_Shim;
namespace AP {
    struct AHRS_LocShim {
        bool get_origin(Location &ret) const { (void)ret; return false; }
        bool home_is_set() const { return false; }
        const Location &get_home() const { static const Location h{}; return h; }
    };
    AHRS_LocShim &ahrs();
}
]==])

# ---- AP_TECS shim -----------------------------------------------------------
# AP_L1_Control only ever holds a "const AP_TECS *" and always receives nullptr
# from AfsimL1Behavior, so a trivial declaration with the single accessor the
# header references is sufficient.  Including <AP_AHRS/AP_AHRS.h> mirrors the
# real header's include so the shim chain is complete.
file(WRITE "${AFSIM_SHIM_DIR}/AP_TECS/AP_TECS.h"
[==[#pragma once
#include <AP_AHRS/AP_AHRS.h>
class AP_TECS {
public:
    float get_target_airspeed() const { return 0.0f; }
};
]==])

# ---- AP_Terrain shim --------------------------------------------------------
# The L1 guidance path never queries terrain.  A forward declaration plus a
# nullptr-returning AP::terrain() replaces the full AP_Terrain (which would pull
# GCS_MAVLink / Logger / AP_Scripting); referenced only by Location.cpp's
# altitude-frame helpers, which the guidance happy-path does not reach.
file(WRITE "${AFSIM_SHIM_DIR}/AP_Terrain/AP_Terrain.h"
[==[#pragma once
class AP_Terrain;
namespace AP {
    AP_Terrain *terrain();
}
]==])

# ---- Seam translation unit --------------------------------------------------
# Definitions for the handful of peripheral leaf symbols the reused ArduPilot
# math / Location sources reference but that the L1 guidance happy-path never
# invokes (NaN-guard internal-error reporting, custom-rotation math, the AP::
# accessors).  Providing them here keeps libafsim_l1.so fully self-contained
# (it links cleanly under -Wl,--no-undefined and loads under dlopen RTLD_NOW)
# while remaining behaviour-neutral, because none of these run on the L1 path.
file(WRITE "${AFSIM_SHIM_DIR}/afsim_l1_seam.cpp"
[==[// GENERATED by CMakeLists.txt -- standalone-build seam (do not edit here).
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_CustomRotations/AP_CustomRotations.h>

namespace AP {
    AHRS_LocShim &ahrs() { static AHRS_LocShim s; return s; }
    AP_Terrain *terrain() { return nullptr; }
    AP_InternalError &internalerror() { static AP_InternalError e; return e; }
    AP_CustomRotations &custom_rotations() {
        alignas(AP_CustomRotations) static unsigned char storage[sizeof(AP_CustomRotations)];
        return *reinterpret_cast<AP_CustomRotations*>(storage);
    }
}
// Never called on the L1 path (NaN-guard / vector rotation): inert definitions.
void AP_InternalError::error(const AP_InternalError::error_t, uint16_t) {}
void AP_CustomRotations::rotate(enum Rotation, Vector3f&) {}
void AP_CustomRotations::rotate(enum Rotation, Vector3d&) {}
]==])

# ------------------------- Include-path ordering -----------------------------
# ORDER IS CRITICAL: the shim directory MUST be searched before the real
# libraries root so the generated shims win the header search for the injection
# points, while every other <...> include falls through to the real in-tree
# sources.  CMAKE_CURRENT_SOURCE_DIR carries the local service headers
# (AfsimL1Behavior.h, AfsimL1_AHRS_Shim.h, l1_c_api.h) and is where the shim's
# <AfsimL1_AHRS_Shim.h> resolves.
include_directories(BEFORE "${AFSIM_SHIM_DIR}")
include_directories("${AP_LIBRARIES_ROOT}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# --------------------------- Compile definitions -----------------------------
# CONFIG_HAL_BOARD=HAL_BOARD_EMPTY selects the zero-peripheral board config
# (HAL_PROGRAM_SIZE_LIMIT_KB=2048 -> HAL_WITH_POSTYPE_DOUBLE=1, identical to
# SITL), so Location keeps double precision and the geometry is numerically
# identical to the vehicle build.  __AP_LINE__ backs the constrain_float()
# macro (normally injected by the waf toolchain, see Tools/ardupilotwaf/boards.py).
# add_definitions() is used (not add_compile_definitions) to stay compatible
# with cmake_minimum_required(VERSION 3.5).
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_EMPTY)
add_definitions(-D__AP_LINE__=__LINE__)

# ------------------------ Phase 4: Sources and target ------------------------
# The service sources (the assigned file's depends_on_files) plus the minimal
# REAL in-tree sources required to preserve the L1 guidance behaviour bit-for-bit
# plus the generated seam.  The AP_Math/AP_Common set below is the confirmed
# minimal closure: removing AP_Math/matrix3.cpp or AP_Math/location.cpp breaks
# the link (mul_transpose / check_lat / check_lng).  Under Option A this list
# would instead carry the real AP_AHRS + EKF/DCM stack.
set(SOURCES
    # --- Local AfsimL1 service (facade + AHRS shim + C ABI boundary) ---
    "${CMAKE_CURRENT_SOURCE_DIR}/AfsimL1Behavior.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/AfsimL1_AHRS_Shim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/l1_c_api.cpp"
    # --- Wrapped guidance controller (REAL, unchanged math) ---
    "${AP_LIBRARIES_ROOT}/AP_L1_Control/AP_L1_Control.cpp"
    # --- REAL AP_Math geometry (behaviour-critical) ---
    "${AP_LIBRARIES_ROOT}/AP_Math/AP_Math.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Math/vector2.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Math/vector3.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Math/matrix3.cpp"
    "${AP_LIBRARIES_ROOT}/AP_Math/location.cpp"
    # --- REAL AP_Common Location geometry (behaviour-critical) ---
    "${AP_LIBRARIES_ROOT}/AP_Common/Location.cpp"
    # --- Generated standalone-build seam ---
    "${AFSIM_SHIM_DIR}/afsim_l1_seam.cpp"
)

# Build the SHARED library.  On Linux this yields libafsim_l1.so (the "lib"
# prefix and ".so" suffix are added automatically).  MUST be SHARED, never an
# executable (AAP 0.2.1 / 0.3.1).
add_library(afsim_l1 SHARED ${SOURCES})
set_target_properties(afsim_l1 PROPERTIES OUTPUT_NAME afsim_l1)

# ------------------- Phase 5: Activate the Option B facade branch ------------
# CRITICAL COMPANION TO THE INCLUDE SEAM (AAP 0.6.2 Option B).
#
# The include seam generated above resolves the token `AP_AHRS` to
# AfsimL1_AHRS_Shim for every translation unit in this target, so AP_L1_Control
# is compiled directly against the injectable shim (the minimal-footprint
# standalone path).  The AfsimL1Behavior facade, however, selects its MEMBER
# LAYOUT and its controller-construction (member-initialiser) branch at COMPILE
# TIME via the AFSIML1_L1_USES_SHIM_AHRS macro
# (see AfsimL1Behavior.h: `#if defined(AFSIML1_L1_USES_SHIM_AHRS)`):
#
#   * DEFINED  (Option B, this build): the facade composes the controller against
#     the SAME owned `_ahrs_shim` that set_state_ne() writes, so host-injected
#     position / velocity / yaw / pitch actually reach the guidance controller.
#   * UNDEFINED (Option A fallback): the facade instead declares a SEPARATE
#     `AP_AHRS _ahrs;` member and binds the controller to it -- but under this
#     build's include seam `AP_AHRS` is itself AfsimL1_AHRS_Shim, so that would
#     be a SECOND, never-written shim instance.  set_state_ne() would keep
#     writing `_ahrs_shim`, leaving the controller reading default (origin, zero-
#     velocity, north-heading) state and the guidance INVARIANT to injected
#     state -- silently defeating the entire purpose of the extraction.
#
# The include seam and this define are therefore two halves of ONE decision and
# MUST travel together: defining the macro is what makes the standalone .so honor
# the extraction's core state-injection contract (AAP 0.1.2 / 0.6.1 / 0.6.2 --
# "the host, not AP_AHRS ... supplies position, velocity, attitude").  Verified
# downstream by the state-flow test: three distinct injected states must produce
# three distinct roll commands (aligned -> ~0 deg; symmetric East/West cross-
# track offsets -> equal-and-opposite bank), not an invariant 0.
#
# PRIVATE scope: the macro governs how THIS target composes the facade, so it is
# confined to this target's own sources (the facade + C ABI + wrapped controller
# compiled here) and is not propagated to any consumer.
target_compile_definitions(afsim_l1 PRIVATE AFSIML1_L1_USES_SHIM_AHRS)

