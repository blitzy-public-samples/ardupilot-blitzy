#pragma once

/// @file    ahrs_shim_seam/AP_AHRS/AP_AHRS.h
/// @brief   Compile-time include seam that binds AP_L1_Control's AHRS
///          dependency to the AfsimL1 service's host-driven state shim.
///
/// This header realizes the "Option B" decoupling strategy that the Agent
/// Action Plan (section 0.6.2) selects for the reusable AfsimL1 service, and
/// that the AfsimL1Behavior.cpp implementation is explicitly written against
/// ("the compile-time include seam makes AP_L1_Control's AP_AHRS &_ahrs bind
/// to AfsimL1_AHRS_Shim").
///
/// ---------------------------------------------------------------------------
/// Why this file exists (the problem it solves)
/// ---------------------------------------------------------------------------
/// AP_L1_Control binds a *concrete* reference `AP_AHRS &_ahrs`
/// (AP_L1_Control.h) and calls SIX NON-VIRTUAL accessors on it
/// (get_location, groundspeed_vector, get_yaw_rad, get_pitch_rad,
/// get_EAS2TAS, and the public field yaw_sensor). Because those accessors are
/// non-virtual, a subclass of the real AP_AHRS cannot override them, so the
/// controller's Position/Navigation state cannot be decoupled by ordinary
/// inheritance. The only behavior-preserving way to feed AP_L1_Control from
/// host-injected state -- WITHOUT rewriting a single line of its guidance
/// mathematics -- is to make the name `AP_AHRS` resolve, *inside the L1
/// translation unit*, to the service's lightweight AfsimL1_AHRS_Shim (whose
/// six read methods are name/signature-identical to the AP_AHRS accessors the
/// controller consumes).
///
/// The build arrangement (the AfsimL1 wscript / CMakeLists) places this
/// directory FIRST on the include path, so that when AP_L1_Control.cpp is
/// (re)compiled for the AfsimL1 service, its transitive `#include
/// <AP_AHRS/AP_AHRS.h>` (via AP_L1_Control.h -> AP_TECS.h) resolves to THIS
/// header instead of the real libraries/AP_AHRS/AP_AHRS.h. Every `_ahrs.<x>`
/// call site in AP_L1_Control then binds to the shim, and the EKF/DCM
/// sensor-fusion stack is never pulled in.
///
/// ---------------------------------------------------------------------------
/// Why this is sound (no ODR / no layout hazard)
/// ---------------------------------------------------------------------------
/// AP_L1_Control stores its AHRS as a REFERENCE (`AP_AHRS &_ahrs`), i.e. a
/// pointer-sized member. The size and layout of AP_L1_Control are therefore
/// IDENTICAL whether `AP_AHRS` denotes the real class or the shim, so the
/// AfsimL1-compiled AP_L1_Control object is layout-compatible everywhere it is
/// used within the service. The service links its own AP_L1_Control
/// translation unit (compiled against this seam); nothing else references the
/// real-AP_AHRS AP_L1_Control, so no conflicting definition is pulled and no
/// One-Definition-Rule violation occurs. The `AP_TECS *` the controller also
/// holds is likewise a pointer and is only dereferenced behind an
/// `if (_tecs != nullptr)` guard, so the service's `nullptr` TECS is safe.
///
/// ---------------------------------------------------------------------------
/// Scope of this seam (it affects ONLY the AfsimL1 example build)
/// ---------------------------------------------------------------------------
/// This directory is added to the include path solely by the AfsimL1 example's
/// build files. It does NOT shadow AP_AHRS.h for any vehicle firmware or any
/// other library: the real controller that ArduPlane/ArduCopter/etc. link is
/// compiled, unchanged, against the real AP_AHRS. AP_L1_Control.cpp and
/// AP_L1_Control.h themselves are NOT edited by this refactor; this seam only
/// changes which `AP_AHRS` the SAME source binds to when compiled for the
/// AfsimL1 service (AAP 0.7.1: no vehicle-firmware changes, additive only).

// The real AP_AHRS.h transitively provides a handful of types that headers
// pulled into the L1 translation unit rely on. The one such type actually
// referenced on this compile path is LowPassFilterFloat, used by an inline
// method in AP_TECS.h (which AP_L1_Control.h includes). Supply it here so the
// seam is a faithful, self-sufficient stand-in for the real header without
// dragging in the EKF/DCM stack.
#include <Filter/LowPassFilter.h>

// Bring in the concrete shim class definition (the six AP_AHRS-compatible read
// methods + the host-side injection setters). Kept as its own reviewed,
// separately-compiled unit; this seam merely re-exposes it under the name the
// controller expects.
#include "../../AfsimL1_AHRS_Shim.h"

// The seam itself: make the name `AP_AHRS` denote the service shim for every
// translation unit compiled with this directory ahead of libraries/ on the
// include path. This is what lets AP_L1_Control's `AP_AHRS &_ahrs`, its
// constructor `AP_L1_Control(AP_AHRS&, const AP_TECS*)`, and AfsimL1Behavior's
// `_l1(_ahrs_shim, nullptr)` all type-check and bind to AfsimL1_AHRS_Shim.
using AP_AHRS = AfsimL1_AHRS_Shim;
