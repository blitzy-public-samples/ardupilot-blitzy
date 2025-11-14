/**
 * @file quadplane.cpp
 * @brief QuadPlane hybrid VTOL implementation for ArduPlane
 * 
 * @details This file implements the QuadPlane subsystem, which enables ArduPlane
 *          fixed-wing aircraft to operate as hybrid VTOL (Vertical Takeoff and Landing)
 *          vehicles by adding multicopter-style motors and control systems.
 *          
 *          The QuadPlane system provides:
 *          - Multicopter flight modes for VTOL operations (QSTABILIZE, QLOITER, QHOVER, etc.)
 *          - Transition management between fixed-wing and VTOL flight
 *          - VTOL assist during fixed-wing flight for enhanced safety
 *          - Support for tilt-rotor configurations
 *          - Support for tailsitter configurations
 *          - Hybrid AUTO missions with both FW and VTOL waypoints
 *          
 *          Architecture:
 *          - Motors: Multicopter motor control using AP_Motors library
 *          - Attitude Control: AC_AttitudeControl for stabilization
 *          - Position Control: AC_PosControl for autonomous position hold
 *          - Navigation: AC_WPNav for waypoint navigation
 *          - Transition: State machine managing FW↔VTOL transitions
 *          - Assist: VTOL_Assist monitoring FW flight and engaging motors when needed
 *          
 *          Coordinate Systems:
 *          - Uses NED (North-East-Down) frame for navigation
 *          - Body frame for attitude control
 *          - Tailsitters use ROTATION_PITCH_90 AHRS view
 *          
 * @note This entire subsystem is conditionally compiled with HAL_QUADPLANE_ENABLED
 * @warning QuadPlane operations require careful tuning and testing in SITL before flight
 * 
 * @see ArduPlane/Plane.h
 * @see libraries/AC_AttitudeControl/AC_AttitudeControl_TS.h
 * @see libraries/AP_Motors/AP_Motors.h
 * 
 * Source: ArduPlane/quadplane.cpp
 */

#include "Plane.h"

/**
 * @brief Conditional compilation guard for QuadPlane functionality
 * 
 * @details HAL_QUADPLANE_ENABLED is defined in the board configuration (hwdef)
 *          to enable or disable QuadPlane support at compile time. This allows
 *          boards with limited flash/RAM to exclude QuadPlane code entirely.
 *          
 *          When disabled, all QuadPlane code is removed from the binary,
 *          saving significant flash memory and RAM.
 */
#if HAL_QUADPLANE_ENABLED

#include "AC_AttitudeControl/AC_AttitudeControl_TS.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPVARPTR(motors, "M_", 2, QuadPlane, plane.quadplane.motors_var_info),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: ms
    // @Range: 500 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // 12 ~ 16 were used by position, velocity and acceleration PIDs

    // @Group: P
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: PILOT_SPD_UP
    // @DisplayName: Pilot maximum vertical speed up
    // @Description: The maximum ascending vertical velocity the pilot may request in m/s
    // @Units: m/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_SPD_UP", 18, QuadPlane, pilot_speed_z_max_up, 2.50),
   
    // @Param: PILOT_SPD_DN
    // @DisplayName: Pilot maximum vertical speed down
    // @Description: The maximum vertical velocity the pilot may request in m/s going down. If 0, uses Q_PILOT_SPD_UP value.
    // @Units: m/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_SPD_DN", 60, QuadPlane, pilot_speed_z_max_dn, 0),

     // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: m/s/s
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("PILOT_ACCEL_Z",  19, QuadPlane, pilot_accel_z,  2.5),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    // 22: THR_MIN_PWM

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    // 23: THR_MAX_PWM

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. The default value of 0 disables assistance but will generate a pre-arm failure to encourage users to set this parameter to -1, or a positive, non-zero value. If this is set to -1 then all Q_ASSIST features are disabled except during transitions. A high non-zero,positive value will lead to more false positives which can waste battery. A lower value will result in less false positive, but will result in assistance taking longer to trigger. If unsure then set to 3 m/s below the minimum airspeed you will fly at. If you don't have an airspeed sensor then use 5 m/s below the minimum airspeed you fly at.
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist.speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate for pilot input on rudder stick in degrees/second
    // @Units: deg/s
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard

    // YAW_RATE_MAX index 25

    // @Param: LAND_FINAL_SPD
    // @DisplayName: Land final speed
    // @Description: The descent speed for the final stage of landing in m/s
    // @Units: m/s
    // @Range: 0.3 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_SPD", 26, QuadPlane, land_final_speed, 0.5),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt, 6),

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: deg
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

    // frame class was moved from 30 when consolidating AP_Motors classes

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: Single/Dual, 12:DodecaHexa, 14:Deca, 15:Scripting Matrix, 17:Dynamic Scripting Matrix
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15:I, 16:MOTOR_FRAME_TYPE_NYT_PLUS, 17:MOTOR_FRAME_TYPE_NYT_X, 18: BetaFlightXReversed, 19: Y4
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: The use of this parameter is no longer recommended and has been superseded by a method that works in all VTOL modes with the exception of QAUTOTUNE which is controlled by the Q_FWD_THR_USE parameter. This Q_VFD_GAIN parameter controls use of the forward motor in VTOL modes that use the velocity controller. Set to 0 to disable this function. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use with QLOITER for position hold with the forward motor. 
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // 33 was used by WVANE_GAIN

    // 34 was used by WVANE_MINROLL

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination, VTOL approach: vehicle will RTL at RTL alt and circle with a radius of Q_FW_LND_APR_RAD down to Q_RTL_ALT and then transition into the wind and QRTL, see 'AUTO VTOL Landing', QRTL Always: do a QRTL instead of RTL
    // @Values: 0:Disabled,1:Enabled,2:VTOL approach,3:QRTL Always
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // 37: TILT_MASK
    // 38: TILT_RATE_UP
    // 39: TILT_MAX

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read https://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),
    
    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also positive and non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least Q_ASSIST_DELAY seconds. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist.angle, 30),

    // @Param: ASSIST_OPTIONS
    // @DisplayName: Quadplane assistance options
    // @Description: Options for special QAssist features
    // @Bitmask: 0: Disable force fixed wing controller recovery
    // @Bitmask: 1: Disable quadplane spin recovery
    // @User: Standard
    AP_GROUPINFO("ASSIST_OPTIONS", 47, QuadPlane, assist.options, 0),
    
    // 47: TILT_TYPE // was AP_Int8, re-used by AP_Int16 ASSIST_OPTIONS
    // 48: TAILSIT_ANGLE
    // 61: TAILSIT_ANG_VT
    // 49: TILT_RATE_DN
    // 50: TAILSIT_INPUT
    // 51: TAILSIT_MASK
    // 52: TAILSIT_MASKCH
    // 53: TAILSIT_VFGAIN
    // 54: TAILSIT_VHGAIN
    // 56: TAILSIT_VHPOW

    // @Param: MAV_TYPE
    // @DisplayName: MAVLink type identifier
    // @Description: This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.
    // @Values: 0:AUTO,1:FIXED_WING,2:QUADROTOR,3:COAXIAL,4:HELICOPTER,7:AIRSHIP,8:FREE_BALLOON,9:ROCKET,10:GROUND_ROVER,11:SURFACE_BOAT,12:SUBMARINE,16:FLAPPING_WING,17:KITE,19:VTOL_DUOROTOR,20:VTOL_QUADROTOR,21:VTOL_TILTROTOR
    AP_GROUPINFO("MAV_TYPE", 57, QuadPlane, mav_type, 0),

    // @Param: OPTIONS
    // @DisplayName: quadplane options
    // @Description: See description for each bitmask bit description
    // @Bitmask: 0: Level Transition-keep wings within LEVEL_ROLL_LIMIT and only use forward motor(s) for climb during transition
    // @Bitmask: 1: Allow FW Takeoff-if bit is not set then NAV_TAKEOFF command on quadplanes will instead perform a NAV_VTOL takeoff
    // @Bitmask: 2: Allow FW Land-if bit is not set then NAV_LAND command on quadplanes will instead perform a NAV_VTOL_LAND
    // @Bitmask: 3: Vtol Takeoff Frame-command NAV_VTOL_TAKEOFF alt set by the command's reference frame not above current location
    // @Bitmask: 4: Always use FW spiral approach-always use Use a fixed wing spiral approach for VTOL landings
    // @Bitmask: 5: USE QRTL-instead of QLAND for rc failsafe when in VTOL modes
    // @Bitmask: 6: Use Governor-use ICE Idle Governor in MANUAL for forward motor
    // @Bitmask: 7: Force Qassist-on always
    // @Bitmask: 8: Mtrs_Only_Qassist-in tailsitters only uses VTOL motors and not flying surfaces for QASSIST
    // @Bitmask: 10: Disarmed Yaw Tilt-enable motor tilt for yaw when disarmed
    // @Bitmask: 11: Delay Spoolup-delay VTOL spoolup for 2 seconds after arming
    // @Bitmask: 12: Disable speed based Qassist when using synthetic airspeed estimates
    // @Bitmask: 13: Disable Ground Effect Compensation-on baro altitude reports
    // @Bitmask: 14: Ignore forward flight angle limits-in Qmodes and use Q_ANGLE_MAX exclusively
    // @Bitmask: 15: ThrLandControl-enable throttle stick control of landing rate
    // @Bitmask: 16: DisableApproach-disable use of approach and airbrake stages in VTOL landing
    // @Bitmask: 17: EnableLandResposition-enable pilot controlled repositioning in AUTO land.Descent will pause while repositioning
    // @Bitmask: 18: ARMVTOL-arm only in VTOL modes (or AUTO mode when current nav cmd is VTOL Takeoff)
    // @Bitmask: 19: CompleteTransition-to fixed wing if Q_TRANS_FAIL timer times out instead of QLAND
    // @Bitmask: 20: Force RTL mode-forces RTL mode on rc failsafe in VTOL modes overriding bit 5(USE_QRTL)
    // @Bitmask: 21: Tilt rotor-tilt motors up when disarmed in FW modes (except manual) to prevent ground strikes.
    // @Bitmask: 22: Scale FF by the ratio of VTOL to plane angle P gains in Position 1 phase of transition into VTOL flight as well as reducing VTOL angle P based on airspeed.
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

    // 60 is used above for VELZ_MAX_DN
    // 61 was used above for TAILSIT_ANG_VT

    AP_GROUPEND
};

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo QuadPlane::var_info2[] = {
    // @Param: TRANS_DECEL
    // @DisplayName: Transition deceleration
    // @Description: This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.
    // @Units: m/s/s
    // @Increment: 0.1
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel, 2.0),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    AP_SUBGROUPPTR(loiter_nav, "LOIT_",  2, QuadPlane, AC_Loiter),

    // 3: TAILSIT_GSCMAX

    // @Param: TRIM_PITCH
    // @DisplayName: Quadplane AHRS trim pitch
    // @Description: This sets the compensation for the pitch angle trim difference between calibrated AHRS level and vertical flight pitch. NOTE! this is relative to calibrated AHRS trim, not forward flight trim which includes PTCH_TRIM_DEG. For tailsitters, this is relative to a baseline of 90 degrees in AHRS.
    // @Units: deg
    // @Range: -10 +10
    // @Increment: 0.1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TRIM_PITCH", 4, QuadPlane, ahrs_trim_pitch, 0),

    // 5: TAILSIT_RLL_MX

#if QAUTOTUNE_ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp
    AP_SUBGROUPINFO(qautotune, "AUTOTUNE_",  6, QuadPlane, QAutoTune),
#endif

    // @Param: FW_LND_APR_RAD
    // @DisplayName: Quadplane fixed wing landing approach radius
    // @Description: This provides the radius used, when using a fixed wing landing approach. If set to 0 then the WP_LOITER_RAD will be selected.
    // @Units: m
    // @Range: 0 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("FW_LND_APR_RAD", 7, QuadPlane, fw_land_approach_radius, 0),

    // @Param: TRANS_FAIL
    // @DisplayName: Quadplane transition failure time
    // @Description: Maximum time allowed for forward transitions, exceeding this time will cancel the transition and the aircraft will immediately change to the mode set by Q_TRANS_FAIL_ACT or finish the transition depending on Q_OPTIONS bit 19. 0 for no limit.
    // @Units: s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRANS_FAIL", 8, QuadPlane, transition_failure.timeout, 0),

    // 9: TAILSIT_MOTMX

    // @Param: THROTTLE_EXPO
    // @DisplayName: Throttle expo strength
    // @Description: Amount of curvature in throttle curve: 0 is linear, 1 is cubic
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("THROTTLE_EXPO", 10, QuadPlane, throttle_expo, 0.2),

    // @Param: ACRO_RLL_RATE
    // @DisplayName: QACRO mode roll rate
    // @Description: The maximum roll rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_RLL_RATE", 11, QuadPlane, acro_roll_rate, 360),

    // @Param: ACRO_PIT_RATE
    // @DisplayName: QACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_PIT_RATE", 12, QuadPlane, acro_pitch_rate, 180),

    // @Param: ACRO_YAW_RATE
    // @DisplayName: QACRO mode yaw rate
    // @Description: The maximum yaw rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_YAW_RATE", 13, QuadPlane, acro_yaw_rate, 90),

    // @Param: TKOFF_FAIL_SCL
    // @DisplayName: Takeoff time failure scalar
    // @Description: Scalar for how long past the expected takeoff time a takeoff should be considered as failed and the vehicle will switch to QLAND. If set to 0 there is no limit on takeoff time.
    // @Range: 1.1 5.0
    // @Increment: 5.1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_FAIL_SCL", 14, QuadPlane, takeoff_failure_scalar, 0),

    // @Param: TKOFF_ARSP_LIM
    // @DisplayName: Takeoff airspeed limit
    // @Description: Airspeed limit during takeoff. If the airspeed exceeds this level the vehicle will switch to QLAND. This is useful for ensuring that you don't takeoff into excessively strong wind. If set to 0 there is no limit on airspeed during takeoff.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_ARSP_LIM", 15, QuadPlane, maximum_takeoff_airspeed, 0),

    // @Param: ASSIST_ALT
    // @DisplayName: Quadplane assistance altitude
    // @Description: This is the altitude below which quadplane assistance will be triggered. This acts the same way as Q_ASSIST_ANGLE and Q_ASSIST_SPEED, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altitude is calculated as being above ground level. The height above ground is given from a Lidar used if available and RNGFND_LANDING=1. Otherwise it comes from terrain data if TERRAIN_FOLLOW=1 and comes from height above home otherwise.
    // @Units: m
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ALT", 16, QuadPlane, assist.alt, 0),

    // 17: TAILSIT_GSCMSK
    // 18: TAILSIT_GSCMIN

    // @Param: ASSIST_DELAY
    // @DisplayName: Quadplane assistance delay
    // @Description: This is delay between the assistance thresholds being met and the assistance starting.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_DELAY", 19, QuadPlane, assist.delay, 0.5),
    
    // @Param: FWD_MANTHR_MAX
    // @DisplayName: VTOL manual forward throttle max percent
    // @Description: Maximum value for manual forward throttle; used with RC option FWD_THR (209)
    // @Range: 0 100
    AP_GROUPINFO("FWD_MANTHR_MAX", 20, QuadPlane, fwd_thr_max, 0),

    // 21: TAILSIT_DSKLD
    // 22: TILT_FIX_ANGLE
    // 23: TILT_FIX_GAIN
    // 24: TAILSIT_RAT_FW
    // 25: TAILSIT_RAT_VT

    // @Group: TAILSIT_
    // @Path: tailsitter.cpp
    AP_SUBGROUPINFO(tailsitter, "TAILSIT_", 26, QuadPlane, Tailsitter),

    // @Group: TILT_
    // @Path: tiltrotor.cpp
    AP_SUBGROUPINFO(tiltrotor, "TILT_", 27, QuadPlane, Tiltrotor),

    // @Param: BACKTRANS_MS
    // @DisplayName: SLT and Tiltrotor back transition pitch limit duration
    // @Description: Pitch angle will increase from 0 to angle max over this duration when switching into VTOL flight in a position control mode. 0 Disables.
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("BACKTRANS_MS", 28, QuadPlane, back_trans_pitch_limit_ms, 3000),

    // @Param: TRANS_FAIL_ACT
    // @DisplayName: Quadplane transition failure action
    // @Description: This sets the mode that is changed to when Q_TRANS_FAIL time elapses, if set. See also Q_OPTIONS bit 19: CompleteTransition if Q_TRANS_FAIL
    // @Values: -1:Warn only, 0:QLand, 1:QRTL
    AP_GROUPINFO("TRANS_FAIL_ACT", 29, QuadPlane, transition_failure.action, 0),

    // @Group: WVANE_
    // @Path: ../libraries/AC_AttitudeControl/AC_WeatherVane.cpp
    AP_SUBGROUPPTR(weathervane, "WVANE_", 30, QuadPlane, AC_WeatherVane),

    // @Param: LAND_ALTCHG
    // @DisplayName: Land detection altitude change threshold
    // @Description: The maximum altitude change allowed during land detection. You can raise this value if you find that landing detection takes a long time to complete. It is the maximum change in altitude over a period of 4 seconds for landing to be detected
    // @Units: m
    // @Range: 0.1 0.6
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("LAND_ALTCHG", 31, QuadPlane, landing_detect.detect_alt_change, 0.2),

    // @Param: NAVALT_MIN
    // @DisplayName: Minimum navigation altitude
    // @Description: This is the altitude in meters above which navigation begins in auto takeoff. Below this altitude the target roll and pitch will be zero. A value of zero disables the feature
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("NAVALT_MIN", 32, QuadPlane, takeoff_navalt_min, 0),

    // @Param: PLT_Y_RATE
    // @DisplayName: Pilot controlled yaw rate
    // @Description: Pilot controlled yaw rate max. Used in all pilot controlled modes except QAcro
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard

    // @Param: PLT_Y_EXPO
    // @DisplayName: Pilot controlled yaw expo
    // @Description: Pilot controlled yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced

    // @Param: PLT_Y_RATE_TC
    // @DisplayName: Pilot yaw rate control input time constant
    // @Description: Pilot yaw rate control input time constant. Low numbers lead to sharper response, higher numbers to softer response.
    // @Units: s
    // @Range: 0 1
    // @Increment: 0.01
    // @Values: 0.5:Very Soft, 0.2:Soft, 0.15:Medium, 0.1:Crisp, 0.05:Very Crisp
    // @User: Standard
    AP_SUBGROUPINFO(command_model_pilot, "PLT_Y_", 33, QuadPlane, AC_CommandModel),

    // @Param: RTL_ALT_MIN
    // @DisplayName: QRTL minimum altitude
    // @Description: If VTOL motors are active QRTL mode will VTOL climb to at least this altitude before returning home. If outside 150% the larger of WP_LOITER_RAD and RTL_RADIUS the vehicle will VTOL climb to Q_RTL_ALT. This parameter has no effect if the vehicle is in forward flight. Should be between Q_LAND_FINAL_ALT and Q_RTL_ALT
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT_MIN", 34, QuadPlane, qrtl_alt_min, 10),

    // @Param: FWD_THR_GAIN
    // @DisplayName: Q mode fwd throttle gain
    // @Description: This parameter sets the gain from forward accel/tilt to forward throttle in certain Q modes. The Q modes this feature operates in is controlled by the Q_FWD_THR_USE parameter. Vehicles using separate forward thrust motors, eg quadplanes, should set this parameter to (all up weight) / (maximum combined thrust of forward motors) with a value of 2 being typical. Vehicles that tilt lifting rotors to provide forward thrust should set this parameter to (all up weight) / (weight lifted by tilting rotors) which for most aircraft can be approximated as (total number of lifting rotors) / (number of lifting rotors that tilt). When using this method of forward throttle control, the forward tilt angle limit is controlled by the Q_FWD_PIT_LIM parameter.
    // @Range: 0.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FWD_THR_GAIN", 35, QuadPlane, q_fwd_thr_gain, 2.0f),

    // @Param: FWD_PIT_LIM
    // @DisplayName: Q mode forward pitch limit
    // @Description: When forward throttle is being controlled by the Q_FWD_THR_GAIN parameter in Q modes, the vehicle forward (nose down) pitch rotation will be limited to the value specified by this parameter and the any additional forward acceleration required will be produced by use of the forward thrust motor(s) or tilting of moveable rotors. Larger values allow the vehicle to pitch more nose down. Set initially to the amount of nose down pitch required to remove wing lift.
    // @Units: deg
    // @Range: 0.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FWD_PIT_LIM", 36, QuadPlane, q_fwd_pitch_lim, 3.0f),

    // @Param: FWD_THR_USE
    // @DisplayName: Q mode forward throttle use
    // @Description: This parameter determines when the feature that uses forward throttle instead of forward tilt is used. The amount of forward throttle is controlled by the Q_FWD_THR_GAIN parameter. The maximum amount of forward pitch allowed is controlled by the Q_FWD_PIT_LIM parameter. Q_FWD_THR_USE = 0 disables the feature. Q_FWD_THR_USE = 1 enables the feature in all position controlled modes such as QLOITER, QLAND, QRTL and VTOL TAKEOFF. Q_FWD_THR_USE = 2 enables the feature in all Q modes except QAUTOTUNE and QACRO. When enabling the feature, the legacy method of controlling forward throttle use via velocity controller error should be disabled by setting Q_VFWD_GAIN to 0. Do not use this feature with tailsitters.
    // @Values: 0:Off,1:On in all position controlled Q modes,2:On in all Q modes except QAUTOTUNE and QACRO
    // @User: Standard
    AP_GROUPINFO("FWD_THR_USE", 37, QuadPlane, q_fwd_thr_use, uint8_t(FwdThrUse::OFF)),

    // @Param: BCK_PIT_LIM
    // @DisplayName: Q mode rearward pitch limit
    // @Description: This sets the maximum number of degrees of back or pitch up in Q modes when the airspeed is at AIRSPEED_MIN, and is used to prevent excessive sutructural loads when pitching up decelerate. If airspeed is above or below AIRSPEED_MIN, the pitch up/back will be adjusted according to the formula pitch_limit = Q_BCK_PIT_LIM * (AIRSPEED_MIN / IAS)^2. The backwards/up pitch limit controlled by this parameter is in addition to limiting applied by PTCH_LIM_MAX_DEG and Q_ANGLE_MAX. The BCK_PIT_LIM limit is only applied when Q_FWD_THR_USE is set to 1 or 2 and the vehicle is flying in a mode that uses forward throttle instead of forward tilt to generate forward speed. Set to a non positive value 0 to deactivate this limit.
    // @Units: deg
    // @Range: 0.0 15.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("BCK_PIT_LIM", 38, QuadPlane, q_bck_pitch_lim, 10.0f),

    // @Param: APPROACH_DIST
    // @DisplayName: Q mode approach distance
    // @Description: The minimum distance from the destination to use the fixed wing airbrake and approach code for landing approach. This is useful if you don't want the fixed wing approach logic to be used when you are close to the destination. Set to zero to always use fixed wing approach.
    // @Units: m
    // @Range: 0.0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("APPROACH_DIST", 39, QuadPlane, approach_distance, 0),
    
    AP_GROUPEND
};

/*
  defaults for all quadplanes
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FLTD", 10.0 },
    { "Q_A_RAT_RLL_SMAX", 50.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FLTD", 10.0 },
    { "Q_A_RAT_PIT_SMAX", 50.0 },
    { "Q_A_RAT_YAW_SMAX", 50.0 },
    { "Q_A_RATE_R_MAX",   75.0 },
    { "Q_A_RATE_P_MAX",   75.0 },
    { "Q_A_RATE_Y_MAX",   75.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
    { "Q_WP_SPEED",       500 },
    { "Q_WP_ACCEL",       100 },
    { "Q_P_JERK_XY",      2   },
    // lower rotational accel limits
    { "Q_A_ACCEL_R_MAX", 40000 },
    { "Q_A_ACCEL_P_MAX", 40000 },
    { "Q_A_ACCEL_Y_MAX", 10000 },
};

/*
  conversion table for quadplane parameters
 */
const AP_Param::ConversionInfo q_conversion_table[] = {
    // tailsitter params have moved but retain the same names
    { Parameters::k_param_quadplane, 48,  AP_PARAM_INT8,  "Q_TAILSIT_ANGLE" },
    { Parameters::k_param_quadplane, 61,  AP_PARAM_INT8,  "Q_TAILSIT_ANG_VT" },
    { Parameters::k_param_quadplane, 50,  AP_PARAM_INT8,  "Q_TAILSIT_INPUT" },
    { Parameters::k_param_quadplane, 53,  AP_PARAM_FLOAT, "Q_TAILSIT_VFGAIN" },
    { Parameters::k_param_quadplane, 54,  AP_PARAM_FLOAT, "Q_TAILSIT_VHGAIN" },
    { Parameters::k_param_quadplane, 56,  AP_PARAM_FLOAT, "Q_TAILSIT_VHPOW" },
    { Parameters::k_param_quadplane, 251,   AP_PARAM_FLOAT, "Q_TAILSIT_GSCMAX" },
    { Parameters::k_param_quadplane, 379,   AP_PARAM_FLOAT, "Q_TAILSIT_RLL_MX" },
    { Parameters::k_param_quadplane, 635,   AP_PARAM_INT16, "Q_TAILSIT_MOTMX" },
    { Parameters::k_param_quadplane, 1147,  AP_PARAM_INT16, "Q_TAILSIT_GSCMSK" },
    { Parameters::k_param_quadplane, 1211,  AP_PARAM_FLOAT, "Q_TAILSIT_GSCMIN" },
    { Parameters::k_param_quadplane, 1403,  AP_PARAM_FLOAT, "Q_TAILSIT_DSKLD" },
    { Parameters::k_param_quadplane, 1595,  AP_PARAM_FLOAT, "Q_TAILSIT_RAT_FW" },
    { Parameters::k_param_quadplane, 1659,  AP_PARAM_FLOAT, "Q_TAILSIT_RAT_FW" },

    // tiltrotor params have moved but retain the same names
    { Parameters::k_param_quadplane, 37,  AP_PARAM_INT16,  "Q_TILT_MASK" },
    { Parameters::k_param_quadplane, 38,  AP_PARAM_INT16,  "Q_TILT_RATE_UP" },
    { Parameters::k_param_quadplane, 39,  AP_PARAM_INT8,  "Q_TILT_MAX" },
    { Parameters::k_param_quadplane, 47,  AP_PARAM_INT8,  "Q_TILT_TYPE" },
    { Parameters::k_param_quadplane, 49,  AP_PARAM_INT16,  "Q_TILT_RATE_DN" },
    { Parameters::k_param_quadplane, 55,  AP_PARAM_FLOAT,  "Q_TILT_YAW_ANGLE" },
    { Parameters::k_param_quadplane, 1467,  AP_PARAM_FLOAT,  "Q_TILT_FIX_ANGLE" },
    { Parameters::k_param_quadplane, 1531,  AP_PARAM_FLOAT,  "Q_TILT_FIX_GAIN" },

    // PARAMETER_CONVERSION - Added: Jan-2022
    { Parameters::k_param_quadplane, 33,  AP_PARAM_FLOAT, "Q_WVANE_GAIN" },     // Moved from quadplane to weathervane library
    { Parameters::k_param_quadplane, 34,  AP_PARAM_FLOAT, "Q_WVANE_ANG_MIN" },  // Q_WVANE_MINROLL moved from quadplane to weathervane library

    // PARAMETER_CONVERSION - Added: July-2022
    { Parameters::k_param_quadplane, 25,  AP_PARAM_FLOAT, "Q_PLT_Y_RATE" },   // Moved from quadplane to command model library
};

// PARAMETER_CONVERSION - Added: Oct-2021
const AP_Param::ConversionInfo mot_pwm_conversion_table[] = {
    { Parameters::k_param_quadplane, 22,  AP_PARAM_INT16, "Q_M_PWM_MIN" },
    { Parameters::k_param_quadplane, 23,  AP_PARAM_INT16, "Q_M_PWM_MAX" },
};

QuadPlane::QuadPlane(AP_AHRS &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);

    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Quadplane");
    }
    _singleton = this;
}


// setup default motors for the frame class
/**
 * @brief Set default servo channel assignments for quadplane motors
 * 
 * @details Assigns multicopter motor outputs to servo channels starting at CH_5.
 *          By default, quadplane motors use outputs 5-12, while fixed-wing
 *          controls use outputs 1-4. This can be overridden with SERVOx_FUNCTION
 *          parameters.
 * 
 * @param[in] num_motors Number of motors to assign (4 for quad, 6 for hexa, etc.)
 * 
 * @note Default assignments: motor1→CH_5, motor2→CH_6, motor3→CH_7, motor4→CH_8, etc.
 * @note Tricopter and tailsitter use custom channel assignments
 * 
 * Source: ArduPlane/quadplane.cpp:650-655
 */
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        SRV_Channels::set_aux_channel_default(SRV_Channels::get_motor_function(i), CH_5+i);
    }
}
    

/**
 * @brief Initialize QuadPlane subsystem and allocate dynamic objects
 * 
 * @details Performs one-time initialization of the QuadPlane subsystem including:
 *          1. Memory availability check (requires ~4KB + object sizes)
 *          2. Motor object allocation based on Q_FRAME_CLASS/Q_FRAME_TYPE
 *          3. Attitude control, position control, and navigation setup
 *          4. Default servo channel assignment
 *          5. Tailsitter and tiltrotor configuration
 *          6. Transition state machine initialization
 *          7. Parameter loading from EEPROM
 *          
 *          Dynamic Memory Allocation:
 *          Objects are allocated dynamically (not at compile time) to save memory
 *          on aircraft without quadplane enabled. Allocation failures trigger
 *          AP_BoardConfig::allocation_error().
 *          
 *          Frame Types Supported:
 *          - QUAD: 4 motors
 *          - HEXA: 6 motors
 *          - OCTA/OCTAQUAD: 8 motors
 *          - Y6: 7 motors (6 motors + 1 for yaw servo)
 *          - DECA: 10 motors
 *          - TRI: Tricopter with yaw servo
 *          - TAILSITTER: Duo-motor tailsitter
 *          - SCRIPTING_MATRIX: Lua-controlled motor matrix
 * 
 * @return true if initialization successful, false if already initialized or disabled
 * 
 * @note This function is called repeatedly until initialization succeeds
 * @note Setup is prevented if vehicle is already armed (safety check)
 * @note Memory requirements vary by frame type and enabled features
 * 
 * @warning Insufficient memory will cause config_error and prevent flight
 * @warning Cannot have both tailsitter and tiltrotor enabled simultaneously
 * 
 * @see setup_defaults()
 * @see tailsitter.setup()
 * @see tiltrotor.setup()
 * 
 * Source: ArduPlane/quadplane.cpp:658-839
 */
bool QuadPlane::setup(void)
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }

    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav) + sizeof(*ahrs_view) + sizeof(*loiter_nav) + sizeof(*weathervane)) {
        AP_BoardConfig::config_error("Not enough memory for quadplane");
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    switch ((AP_Motors::motor_frame_class)frame_class) {
    case AP_Motors::MOTOR_FRAME_QUAD:
        setup_default_channels(4);
        break;
    case AP_Motors::MOTOR_FRAME_HEXA:
        setup_default_channels(6);
        break;
    case AP_Motors::MOTOR_FRAME_OCTA:
    case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        setup_default_channels(8);
        break;
    case AP_Motors::MOTOR_FRAME_Y6:
        setup_default_channels(7);
        break;
    case AP_Motors::MOTOR_FRAME_DECA:
        setup_default_channels(10);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        SRV_Channels::set_default_function(CH_5, SRV_Channel::k_motor1);
        SRV_Channels::set_default_function(CH_6, SRV_Channel::k_motor2);
        SRV_Channels::set_default_function(CH_8, SRV_Channel::k_motor4);
        SRV_Channels::set_default_function(CH_11, SRV_Channel::k_motor7);
        AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
    case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
    case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
        break;
    default:
        AP_BoardConfig::config_error("Unsupported Q_FRAME_CLASS %u", (unsigned int)(frame_class.get()));
    }

    // Make sure not both a tailsiter and tiltrotor
    if ((tailsitter.enable > 0) && (tiltrotor.enable > 0)) {
        AP_BoardConfig::config_error("set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
    }

    switch ((AP_Motors::motor_frame_class)frame_class) {
#if AP_MOTORS_TRI_ENABLED
    case AP_Motors::MOTOR_FRAME_TRI:
        motors = NEW_NOTHROW AP_MotorsTri(rc_speed);
        motors_var_info = AP_MotorsTri::var_info;
        break;
#endif  // AP_MOTORS_TRI_ENABLED
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        // this is a duo-motor tailsitter
        tailsitter.tailsitter_motors = NEW_NOTHROW AP_MotorsTailsitter(rc_speed);
        motors = tailsitter.tailsitter_motors;
        motors_var_info = AP_MotorsTailsitter::var_info;
        break;
    case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = NEW_NOTHROW AP_MotorsMatrix_Scripting_Dynamic(plane.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
    default:
        motors = NEW_NOTHROW AP_MotorsMatrix(rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
        break;
    }

    if (!motors) {
        AP_BoardConfig::allocation_error("motors");
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view((tailsitter.enable > 0) ? ROTATION_PITCH_90 : ROTATION_NONE, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("ahrs_view");
    }

    attitude_control = NEW_NOTHROW AC_AttitudeControl_TS(*ahrs_view, aparm, *motors);
    if (!attitude_control) {
        AP_BoardConfig::allocation_error("attitude_control");
    }

    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = NEW_NOTHROW AC_PosControl(*ahrs_view, *motors, *attitude_control);
    if (!pos_control) {
        AP_BoardConfig::allocation_error("pos_control");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = NEW_NOTHROW AC_WPNav(*ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        AP_BoardConfig::allocation_error("wp_nav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = NEW_NOTHROW AC_Loiter(*ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        AP_BoardConfig::allocation_error("loiter_nav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    weathervane = NEW_NOTHROW AC_WeatherVane();
    if (!weathervane) {
        AP_BoardConfig::allocation_error("weathervane");
    }
    AP_Param::load_object_from_eeprom(weathervane, weathervane->var_info);

    motors->init(frame_class, frame_type);
    motors->update_throttle_range();
    motors->set_update_rate(rc_speed);
    attitude_control->parameter_sanity_check();

    // Try to convert mot PWM params, if still invalid force conversion
    AP_Param::convert_old_parameters(&mot_pwm_conversion_table[0], ARRAY_SIZE(mot_pwm_conversion_table));
    if (!motors->check_mot_pwm_params()) {
        AP_Param::convert_old_parameters(&mot_pwm_conversion_table[0], ARRAY_SIZE(mot_pwm_conversion_table), AP_Param::CONVERT_FLAG_FORCE);
    }

    // setup the trim of any motors used by AP_Motors so I/O board
    // failsafe will disable motors
    uint32_t mask = plane.quadplane.motors->get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, plane.quadplane.motors->get_pwm_output_min());

    // default QAssist state as set with Q_OPTIONS
    if (option_is_set(QuadPlane::OPTION::Q_ASSIST_FORCE_ENABLE)) {
        assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED);
    }

    setup_defaults();

    AP_Param::convert_old_parameters(&q_conversion_table[0], ARRAY_SIZE(q_conversion_table));

    // centi-conversions added January 2024
    land_final_speed.convert_centi_parameter(AP_PARAM_INT16);
    pilot_speed_z_max_up.convert_centi_parameter(AP_PARAM_INT16);
    pilot_speed_z_max_dn.convert_centi_parameter(AP_PARAM_INT16);
    pilot_accel_z.convert_centi_parameter(AP_PARAM_INT16);

    // Provisionally assign the SLT thrust type.
    // It will be overwritten by tailsitter or tiltorotor setups.
    thrust_type = ThrustType::SLT;

    tailsitter.setup();

    tiltrotor.setup();

    if (!transition) {
        transition = NEW_NOTHROW SLT_Transition(*this, motors);
    }
    if (!transition) {
        AP_BoardConfig::allocation_error("transition");
    }

    // init wp_nav variables after defaults are setup
    wp_nav->wp_and_spline_init_cm();

    transition->force_transition_complete();

    // param count will have changed
    AP_Param::invalidate_count();

    char frame_and_type_string[30];
    motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised, %s", frame_and_type_string);
    initialised = true;
    return true;
}

/**
 * @brief Apply default parameter values from defaults_table
 * 
 * @details Loads default parameter values that are specific to QuadPlane operation.
 *          This includes setting up default PIDs, navigation parameters, and other
 *          configuration values that differ from standard fixed-wing defaults.
 *          
 *          Also resets ESC calibration parameter (Q_ESC_CAL) to 0 on every boot
 *          as a safety measure to prevent accidental motor runs at full throttle.
 * 
 * @note Called during setup() initialization
 * @note ESC calibration mode is automatically disabled on boot
 * 
 * @see AP_Param::set_defaults_from_table()
 * 
 * Source: ArduPlane/quadplane.cpp:844-857
 */
void QuadPlane::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));

    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
    // Quadplanes need the same level of GPS error checking as Copters do, Plane is more relaxed
    AP_Param::set_default_by_name("EK2_CHECK_SCALE",100);
    AP_Param::set_default_by_name("EK3_CHECK_SCALE",100);

}

/**
 * @brief Execute ESC (Electronic Speed Controller) calibration procedure
 * 
 * @details Passes throttle input directly to motors for ESC calibration.
 *          This allows ESCs to learn their throttle range (min/max PWM values).
 *          
 *          Calibration Modes (Q_ESC_CAL parameter):
 *          - 0: Disabled (normal operation)
 *          - 1: Throttle Input Mode - motor output follows pilot throttle stick
 *          - 2: Full Range Mode - motors output maximum PWM when armed
 *          
 *          Procedure:
 *          1. Set Q_ESC_CAL to 1 or 2
 *          2. Remove all propellers (SAFETY CRITICAL)
 *          3. Switch to QSTABILIZE mode
 *          4. Arm the vehicle
 *          5. For mode 1: Move throttle to full, then to minimum
 *          6. For mode 2: ESCs see full throttle immediately
 *          7. Disarm and set Q_ESC_CAL back to 0
 * 
 * @note Only active in QSTABILIZE mode
 * @note Q_ESC_CAL automatically resets to 0 on boot (safety feature)
 * @note ESC calibration notification is displayed via AP_Notify
 * 
 * @warning REMOVE ALL PROPELLERS before ESC calibration
 * @warning Motors will spin at commanded throttle when armed
 * 
 * Source: ArduPlane/quadplane.cpp:859-880
 */
void QuadPlane::run_esc_calibration(void)
{
    if (!motors->armed()) {
        motors->set_throttle_passthrough_for_esc_calibration(0);
        AP_Notify::flags.esc_calibration = false;
        return;
    }
    if (!AP_Notify::flags.esc_calibration) {
        gcs().send_text(MAV_SEVERITY_INFO, "Starting ESC calibration");
    }
    AP_Notify::flags.esc_calibration = true;
    switch (esc_calibration) {
    case 1:
        // throttle based calibration
        motors->set_throttle_passthrough_for_esc_calibration(plane.get_throttle_input() * 0.01f);
        break;
    case 2:
        // full range calibration
        motors->set_throttle_passthrough_for_esc_calibration(1);
        break;
    }
}

/**
 * @brief Update multicopter attitude rate controller
 * 
 * @details Coordinates attitude control between fixed-wing and VTOL controllers.
 *          In VTOL modes, uses multicopter attitude control exclusively.
 *          During transitions, blends between FW and VTOL control based on
 *          transition state.
 *          
 *          Control Selection Logic:
 *          - Pure VTOL mode: Use multicopter attitude control
 *          - Tailsitter in VTOL transition: Blend FW and VTOL control
 *          - Fixed-wing mode: Use FW attitude control
 *          - Transition with yaw target: Use multicopter control for yaw tracking
 *          
 *          The function handles the handoff between the two control systems
 *          to ensure smooth transitions without attitude upsets.
 * 
 * @param[in] yaw_rate_cds Desired yaw rate in centidegrees/second from pilot or autopilot
 * 
 * @note Called at main loop rate during VTOL operations and transitions
 * @note force_fw_control_recovery flag can override multicopter control
 * @note Tailsitters use special handling during VTOL transitions
 * 
 * @see AC_AttitudeControl::rate_controller_run()
 * @see transition->update_yaw_target()
 * 
 * Source: ArduPlane/quadplane.cpp:886-987
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    bool use_multicopter_control = in_vtol_mode() && !tailsitter.in_vtol_transition() && !force_fw_control_recovery;
    bool use_yaw_target = false;

    float yaw_target_cd = 0.0;
    if (!use_multicopter_control && transition->update_yaw_target(yaw_target_cd) &&
        !force_fw_control_recovery) {
        use_multicopter_control = true;
        use_yaw_target = true;
    }

    // normal control modes for VTOL and FW flight
    // tailsitter in transition to VTOL flight is not really in a VTOL mode yet
    if (use_multicopter_control) {

        // Pilot input, use yaw rate time constant
        set_pilot_yaw_rate_time_constant();

        // tailsitter-only body-frame roll control options
        // Angle mode attitude control for pitch and body-frame roll, rate control for euler yaw.
        if (tailsitter.enabled() &&
            (tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_BF_ROLL)) {

            if (!(tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_PLANE)) {
                // In multicopter input mode, the roll and yaw stick axes are independent of pitch
                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(false,
                                                                                plane.nav_roll_cd,
                                                                                plane.nav_pitch_cd,
                                                                                yaw_rate_cds);
                return;
            } else {
                // In plane input mode, the roll and yaw sticks are swapped
                // and their effective axes rotate from yaw to roll and vice versa
                // as pitch goes from zero to 90.
                // So it is necessary to also rotate their scaling.

                // Get the roll angle and yaw rate limits
                int16_t roll_limit = aparm.angle_max;
                // separate limit for tailsitter roll, if set
                if (tailsitter.max_roll_angle > 0) {
                    roll_limit = tailsitter.max_roll_angle * 100.0f;
                }
                // Prevent a divide by zero
                const float yaw_rate_max = command_model_pilot.get_rate();
                float yaw_rate_limit = ((yaw_rate_max < 1.0f) ? 1 : yaw_rate_max) * 100.0f;
                float yaw2roll_scale = roll_limit / yaw_rate_limit;

                // Rotate as a function of Euler pitch and swap roll/yaw
                float euler_pitch = radians(.01f * plane.nav_pitch_cd);
                float spitch = fabsf(sinf(euler_pitch));
                float y2r_scale = linear_interpolate(1, yaw2roll_scale, spitch, 0, 1);

                float p_yaw_rate = plane.nav_roll_cd / y2r_scale;
                float p_roll_angle = -y2r_scale * yaw_rate_cds;

                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(true,
                                                                                p_roll_angle,
                                                                                plane.nav_pitch_cd,
                                                                                p_yaw_rate);
                return;
            }
        }

        // note this is actually in deg/s for some SID_AXIS values for yaw
        Vector3f offset_deg;

#if AP_PLANE_SYSTEMID_ENABLED
        auto &systemid = plane.g2.systemid;
        systemid.update();
        offset_deg = systemid.get_attitude_offset_deg();
#endif

        if (use_yaw_target) {
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd + offset_deg.x*100,
                                                               plane.nav_pitch_cd + offset_deg.y*100,
                                                               yaw_target_cd + offset_deg.z*100,
                                                               true);
        } else {
            // use euler angle attitude control
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd + offset_deg.x*100,
                                                                          plane.nav_pitch_cd + offset_deg.y*100,
                                                                          yaw_rate_cds + offset_deg.z*100);
        }
    } else {
        // use the fixed wing desired rates
        Vector3f bf_input_cd { plane.rollController.get_pid_info().target * 100.0f,
                               plane.pitchController.get_pid_info().target * 100.0f,
                               yaw_rate_cds };

        // rotate into multicopter attitude reference frame
        ahrs_view->rotate(bf_input_cd);

        // disable yaw time constant for 1:1 match of desired rates
        disable_yaw_rate_time_constant();

        attitude_control->input_rate_bf_roll_pitch_yaw_no_shaping_cds(bf_input_cd.x, bf_input_cd.y, bf_input_cd.z);
    }
}

// hold in stabilize with given throttle
/**
 * @brief Hold altitude and attitude in stabilize mode with manual throttle control
 * 
 * @details Implements stabilize mode attitude hold where pilot controls throttle
 *          directly while attitude controller maintains roll/pitch/yaw stability.
 *          
 *          Control flow:
 *          1. Update attitude rate controller with desired yaw rate
 *          2. If throttle at zero and not in air mode:
 *             - Set motors to GROUND_IDLE spool state
 *             - Zero throttle output
 *             - Relax attitude control
 *          3. If throttle positive or in air mode:
 *             - Set motors to THROTTLE_UNLIMITED
 *             - Apply throttle with angle boost (except tailsitter forward flight)
 *             - Add SystemID throttle offset if enabled
 * 
 * @param[in] throttle_in Manual throttle input from pilot (0.0 to 1.0)
 * 
 * @note Angle boost disabled for tailsitters in assisted forward flight
 * @note Zero throttle triggers ground idle unless air mode is active
 * @note SystemID throttle offset applied when AP_PLANE_SYSTEMID_ENABLED
 * 
 * @see multicopter_attitude_rate_update()
 * @see relax_attitude_control()
 * @see air_mode_active()
 * 
 * Source: ArduPlane/quadplane.cpp:1159-1180
 */
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    if ((throttle_in <= 0) && !air_mode_active()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
    } else {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        bool should_boost = true;
        if (tailsitter.enabled() && assisted_flight) {
            // tailsitters in forward flight should not use angle boost
            should_boost = false;
        }
#if AP_PLANE_SYSTEMID_ENABLED
        throttle_in += plane.g2.systemid.get_throttle_offset();
#endif
        attitude_control->set_throttle_out(throttle_in, should_boost, 0);
    }
}

/**
 * @brief Execute vertical position and velocity controller (Z-axis)
 * 
 * @details Runs the multicopter altitude controller using AC_PosControl
 *          to maintain desired vertical position or velocity.
 *          
 *          Controller logic:
 *          - Only runs when motors at THROTTLE_UNLIMITED spool state
 *          - Disabled during tailsitter VTOL transitions
 *          - Reinitializes if not active for >20ms
 *          - Sets vertical speed/accel limits from parameters
 *          - Tailsitters use no-descent initialization variant
 *          
 *          Timing:
 *          - Tracks last_pidz_active_ms for activity monitoring
 *          - Records last_pidz_init_ms for initialization timing
 * 
 * @note Exits immediately if motors not at THROTTLE_UNLIMITED
 * @note Exits immediately during tailsitter VTOL transitions
 * @note Reinitializes controller if inactive for more than 20ms
 * @note Tailsitter initialization prevents descent on startup
 * 
 * @warning Critical for altitude hold - failure could cause altitude loss
 * 
 * @see AC_PosControl::update_U_controller()
 * @see AC_PosControl::init_U_controller()
 * @see tailsitter.in_vtol_transition()
 * 
 * Source: ArduPlane/quadplane.cpp:1182-1208
 */
void QuadPlane::run_z_controller(void)
{
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED ) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now)) {
        // never run Z controller in tailsitter transition
        return;
    }
    if ((now - last_pidz_active_ms) > 20 || !pos_control->is_active_U()) {
        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_U_cm(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);

        // initialise the vertical position controller
        if (!tailsitter.enabled()) {
            pos_control->init_U_controller();
        } else {
            // initialise the vertical position controller with no descent
            pos_control->init_U_controller_no_descent();
        }
        last_pidz_init_ms = now;
    }
    last_pidz_active_ms = now;
    pos_control->update_U_controller();
}

/**
 * @brief Relax attitude controllers to reduce integrator windup
 * 
 * @details Disables attitude control loops to prevent integrator accumulation
 *          when the vehicle is not actively controlling attitude (e.g., on ground,
 *          zero throttle, or transitioning to fixed wing).
 *          
 *          Tailsitter-specific behavior:
 *          - Vectored tailsitters: Disable roll and yaw, maintain pitch control
 *          - Non-vectored tailsitters: Disable all attitude control axes
 *          
 *          This prevents integrator windup during periods when control surfaces
 *          or motors are not actively stabilizing the aircraft.
 * 
 * @note Tailsitter behavior controlled by tailsitter.relax_pitch()
 * @note Called during ground idle, zero throttle, and transition modes
 * @note Prevents integrator buildup that could cause overshoot on reactivation
 * 
 * @see AC_AttitudeControl::relax_attitude_controllers()
 * @see hold_stabilize()
 * 
 * Source: ArduPlane/quadplane.cpp:1210-1215
 */
void QuadPlane::relax_attitude_control()
{
    // disable roll and yaw control for vectored tailsitters
    // if not a vectored tailsitter completely disable attitude control
    attitude_control->relax_attitude_controllers(!tailsitter.relax_pitch());
}

/**
 * @brief Detect and handle EKF yaw angle resets
 * 
 * @details Monitors the Extended Kalman Filter (EKF) for yaw angle resets
 *          and updates the attitude controller's inertial frame reference
 *          when a reset is detected.
 *          
 *          EKF yaw resets occur when:
 *          - Compass calibration completes
 *          - EKF switches between different yaw sources
 *          - Significant magnetic anomaly detected
 *          - GPS velocity provides new yaw reference
 *          
 *          Response sequence:
 *          1. Query AHRS for last yaw reset timestamp
 *          2. If timestamp changed (new reset detected):
 *             - Reset attitude controller's inertial frame
 *             - Update cached reset timestamp
 *             - Log EKF_YAW_RESET event
 * 
 * @note Only active after QuadPlane initialization complete
 * @note Called regularly from update() loop
 * @note Critical for maintaining attitude reference after EKF resets
 * 
 * @warning Failure to handle yaw resets causes attitude controller divergence
 * 
 * @see AP_AHRS::getLastYawResetAngle()
 * @see AC_AttitudeControl::inertial_frame_reset()
 * 
 * Source: ArduPlane/quadplane.cpp:1217-1230
 */
void QuadPlane::check_yaw_reset(void)
{
    if (!initialised) {
        return;
    }
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        LOGGER_WRITE_EVENT(LogEvent::EKF_YAW_RESET);
    }
}

/**
 * @brief Set desired vertical velocity for position controller
 * 
 * @details Passes target vertical velocity to AC_PosControl's velocity controller
 *          for altitude control in VTOL modes. Uses 'U' (up) axis convention where
 *          positive values are upward motion.
 *          
 *          Parameters to pos_control:
 *          - target_climb_rate_cms: Desired vertical velocity
 *          - 0: No additional acceleration feedforward
 *          - false: Do not reset z-axis velocity controller
 * 
 * @param[in] target_climb_rate_cms Desired climb rate in cm/s (positive = up, negative = down)
 * 
 * @note Called from hold_hover(), assisted flight, and VTOL flight modes
 * @note Actual throttle output computed by run_z_controller()
 * 
 * @see AC_PosControl::input_vel_accel_U_cm()
 * @see run_z_controller()
 * 
 * Source: ArduPlane/quadplane.cpp:1343-1346
 */
void QuadPlane::set_climb_rate_cms(float target_climb_rate_cms)
{
    pos_control->input_vel_accel_U_cm(target_climb_rate_cms, 0, false);
}

/**
 * @brief Maintain hover position with specified vertical velocity
 * 
 * @details Complete hover control loop combining attitude and altitude control.
 *          Maintains level attitude while controlling vertical velocity to target.
 *          
 *          Control sequence:
 *          1. Enable full throttle range (THROTTLE_UNLIMITED spool state)
 *          2. Configure vertical velocity/acceleration limits from parameters:
 *             - Max descent: Q_PILOT_SPD_DN (or Q_PILOT_SPD_UP if zero)
 *             - Max climb: Q_PILOT_SPD_UP
 *             - Acceleration: Q_PILOT_ACCEL_Z
 *          3. Update attitude rate controller with yaw rate
 *          4. Set desired climb rate for position controller
 *          5. Execute vertical position controller
 * 
 * @param[in] target_climb_rate_cms Desired vertical velocity in cm/s
 *                                   (positive = climb, negative = descend)
 * 
 * @note Primary hover control function used by QHOVER and QLOITER modes
 * @note Zero climb rate produces stable hover at current altitude
 * @note Velocity limits prevent pilot from exceeding configured maximums
 * 
 * @see multicopter_attitude_rate_update()
 * @see set_climb_rate_cms()
 * @see run_z_controller()
 * 
 * Source: ArduPlane/quadplane.cpp:1351-1368
 */
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    // motors use full range
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);

    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds(false));

    // call position controller
    set_climb_rate_cms(target_climb_rate_cms);

    run_z_controller();
}

/**
 * @brief Convert pilot throttle stick input to normalized throttle value
 * 
 * @details Processes raw throttle channel input and applies exponential curve
 *          to provide more intuitive throttle control in VTOL modes.
 *          
 *          Processing steps:
 *          1. Read throttle channel control input
 *          2. Normalize to [0.0, 1.0] range
 *          3. If throttle_expo > 0:
 *             - Apply exponential curve centered at hover throttle
 *             - Mid-stick position produces hover throttle output
 *             - Provides finer control near hover
 *          4. If throttle_expo = 0:
 *             - Return linear throttle mapping
 * 
 * @return Normalized throttle value [0.0, 1.0] with expo applied
 * 
 * @note Expo curve centers around motors->get_throttle_hover()
 * @note Linear response when Q_THROTTLE_EXPO parameter is zero
 * @note Exponential response improves hover precision for most pilots
 * 
 * @see throttle_curve()
 * @see AP_Motors::get_throttle_hover()
 * @see Q_THROTTLE_EXPO parameter
 * 
 * Source: ArduPlane/quadplane.cpp:1418-1435
 */
float QuadPlane::get_pilot_throttle()
{
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    if (is_positive(throttle_expo)) {
        // get hover throttle level [0,1]
        float thr_mid = motors->get_throttle_hover();
        float thrust_curve_expo = constrain_float(throttle_expo, 0.0f, 1.0f);

        // this puts mid stick at hover throttle
        return throttle_curve(thr_mid, thrust_curve_expo, throttle_in);
    } else {
        return throttle_in;
    }
}

/**
 * @brief Transform pilot stick inputs into desired lean angles with limits
 * 
 * @details Converts pilot's roll and pitch RC inputs into desired aircraft
 *          lean angles (in centidegrees) with mode-specific angle limiting
 *          and failsafe handling.
 *          
 *          Processing sequence:
 *          1. Failsafe check - zero outputs if RC failsafe or throttle failsafe active
 *          2. Read roll and pitch channel inputs
 *          3. Scale inputs by angle_max_cd (Q_ANGLE_MAX parameter, default 3000 = 30°)
 *          4. Apply mode-specific circular limit (angle_limit_cd)
 *          5. Convert lateral tilt to euler roll for stability
 *          
 *          Limiting behavior:
 *          - angle_max_cd: Maximum lean from Q_ANGLE_MAX parameter
 *          - angle_limit_cd: Mode-specific limit (can be lower than angle_max)
 *          - Circular limit: sqrt(roll² + pitch²) <= angle_limit
 *          - Minimum limit: 10° (1000 centidegrees) always allowed
 * 
 * @param[out] roll_out_cd Desired roll angle in centidegrees (NED body frame)
 * @param[out] pitch_out_cd Desired pitch angle in centidegrees (NED body frame)
 * @param[in] angle_max_cd Maximum lean angle from parameter (typically Q_ANGLE_MAX)
 * @param[in] angle_limit_cd Mode-specific angle limit (may be less than angle_max_cd)
 * 
 * @note Outputs zeroed during RC or throttle failsafe
 * @note Circular limiting prevents total lean exceeding angle_limit_cd
 * @note Euler conversion compensates for pitch angle in roll calculation
 * @note Typical angle_max_cd is 3000 (30°), can be up to 8000 (80°)
 * 
 * @warning Critical safety function - must zero outputs during failsafe
 * 
 * @see plane.failsafe.rc_failsafe
 * @see plane.failsafe.throttle_counter
 * @see Q_ANGLE_MAX parameter
 * 
 * Source: ArduPlane/quadplane.cpp:1437-1470
 */
void QuadPlane::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // failsafe check
    if (plane.failsafe.rc_failsafe || plane.failsafe.throttle_counter > 0) {
        roll_out_cd = 0;
        pitch_out_cd = 0;
        return;
    }

    // fetch roll and pitch inputs
    roll_out_cd = plane.channel_roll->get_control_in();
    pitch_out_cd = plane.channel_pitch->get_control_in();

    // limit max lean angle, always allow for 10 degrees
    angle_limit_cd = constrain_float(angle_limit_cd, 1000.0f, angle_max_cd);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max_cd/4500.0;
    roll_out_cd *= scaler;
    pitch_out_cd *= scaler;

    // apply circular limit
    float total_in = norm(pitch_out_cd, roll_out_cd);
    if (total_in > angle_limit_cd) {
        float ratio = angle_limit_cd / total_in;
        roll_out_cd *= ratio;
        pitch_out_cd *= ratio;
    }

    // apply lateral tilt to euler roll conversion
    roll_out_cd = 100 * degrees(atanf(cosf(cd_to_rad(pitch_out_cd)) * tanf(cd_to_rad(roll_out_cd))));
}

/*
  get pilot throttle in for landing code. Return value on scale of 0 to 1
*/
float QuadPlane::get_pilot_land_throttle(void) const
{
    if (plane.rc_failsafe_active()) {
        // assume zero throttle if lost RC
        return 0;
    }
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    return constrain_float(throttle_in, 0, 1);
}

// helper for is_flying()
/**
 * @brief Detect if aircraft is currently flying in any mode
 * 
 * @details Determines if the quadplane is airborne using multiple indicators:
 *          
 *          - QuadPlane unavailable → not flying
 *          - GUIDED mode with active guided takeoff → flying
 *          - Throttle > 1% and not at lower limit → flying
 *          - Tailsitter in VTOL transition → flying
 *          
 *          This is a general flying detector that works in both VTOL
 *          and fixed-wing modes. More specific checks available in
 *          is_flying_vtol() for VTOL-only detection.
 * 
 * @return true if aircraft is flying, false if on ground
 * 
 * @note Simple heuristic suitable for general flight detection
 * @note Does not use accelerometer-based takeoff detection
 * @note Tailsitter transition always considered as flying
 * 
 * @see is_flying_vtol() For VTOL-specific flying detection
 * @see should_relax() For landing detection
 * 
 * Source: ArduPlane/quadplane.cpp:1335-1349
 */
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (motors->get_throttle() > 0.01f && !motors->limit.throttle_lower) {
        return true;
    }
    if (tailsitter.in_vtol_transition()) {
        return true;
    }
    return false;
}

/**
 * @brief Detect if motors should relax to prevent ground tipover
 * 
 * @details Simple landing detector that triggers motor relaxation after
 *          sustained throttle at lower limit. Prevents vehicle tipover
 *          by disabling aggressive control when detected on ground.
 *          
 *          Detection logic:
 *          1. Check if motors at lower limit (throttle < 1% or limit flag)
 *          2. If at limit, start timer
 *          3. If at limit for > 1 second, trigger relax
 *          4. If not at limit, reset timer
 *          
 *          When should_relax() returns true, attitude control typically
 *          reduces gains or switches to ground idle mode.
 * 
 * @return true if motors should relax (likely on ground), false otherwise
 * 
 * @note 1-second delay prevents false triggers from brief throttle cuts
 * @note Resets immediately when throttle increases above lower limit
 * @note Used primarily in QLAND and landing detection
 * 
 * @warning Crude detector - does not use accelerometer or rangefinder data
 * 
 * @see landing_detect.lower_limit_start_ms Timer for detection
 * @see motors->limit.throttle_lower Throttle limit flag
 * 
 * Source: ArduPlane/quadplane.cpp:1352-1376
 */
bool QuadPlane::should_relax(void)
{
    const uint32_t tnow = millis();

    bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
        landing_detect.land_start_ms = 0;
        return false;
    } else if (landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = tnow;
    }

    return (tnow - landing_detect.lower_limit_start_ms) > 1000;
}

/**
 * @brief Detect if aircraft is flying specifically in VTOL mode
 * 
 * @details More specific flying detector than is_flying() that only
 *          returns true when flying with VTOL motors. Checks motor
 *          spool state to determine if currently airborne in multicopter mode.
 * 
 * @return true if flying in VTOL mode, false otherwise
 * 
 * @note Returns false if quadplane unavailable
 * @note Motor spool state SHUT_DOWN indicates not flying in VTOL
 * 
 * @see is_flying() For general flying detection (any mode)
 * @see motors->get_spool_state() Motor spool state indicator
 * 
 * Source: ArduPlane/quadplane.cpp:1379-1385
 */
bool QuadPlane::is_flying_vtol(void) const
{
    if (!available()) {
        return false;
    }
    if (motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN) {
        // assume that with no motor outputs we're not flying in VTOL mode
        return false;
    }
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode->is_vtol_man_throttle() && air_mode_active()) {
        // in manual throttle modes with airmode on, don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (plane.control_mode->is_vtol_man_mode()) {
        // in manual flight modes only consider aircraft landed when pilot demanded throttle is zero
        return is_positive(get_throttle_input());
    }
    if (in_vtol_mode() && millis() - landing_detect.lower_limit_start_ms > 5000) {
        // use landing detector
        return true;
    }
    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt. 
 */
/**
 * @brief Calculate desired descent rate for VTOL landing based on altitude
 * 
 * @details Computes adaptive descent rate for QLAND and VTOL RTL landing.
 *          Descent rate varies with height to provide fast descent at altitude
 *          and gentle touchdown near ground.
 *          
 *          Descent rate calculation:
 *          1. Check for manual override (valid for 1 second after set)
 *          2. Linear interpolation between:
 *             - At land_final_alt: Use land_final_speed (typically slow, 20-50 cm/s)
 *             - At land_final_alt+6m: Use default descent speed (faster)
 *          3. Optional pilot throttle control (OPTION::THR_LANDING_CONTROL):
 *             - Throttle >60%: Climb at max_climb_speed
 *             - Throttle 40-60%: Hold altitude (zero descent)
 *             - Throttle <40%: Scale descent rate (0-100%)
 *          4. During pilot repositioning: Limit to non-positive rates
 *          
 *          Typical behavior:
 *          - Above 6m + land_final_alt: Fast descent (e.g., 150 cm/s)
 *          - Approaching land_final_alt: Smooth transition to slow final rate
 *          - Below land_final_alt: Constant final descent (gentle touchdown)
 * 
 * @param[in] height_above_ground Current altitude above landing surface in meters
 * 
 * @return Desired descent rate in cm/s (positive = descent, negative = climb)
 * 
 * @note Override descent rate has 1-second timeout for safety
 * @note QPOS_LAND_FINAL state clamps height to land_final_alt maximum
 * @note Throttle control requires THR_LANDING_CONTROL option enabled
 * @note Pilot repositioning prevents descent (safety feature)
 * 
 * @see Q_LAND_FINAL_ALT parameter (typical 6m)
 * @see Q_LAND_FINAL_SPD parameter (typical 50 cm/s)
 * @see OPTION::THR_LANDING_CONTROL option
 * 
 * Source: ArduPlane/quadplane.cpp:1697-1747
 */
float QuadPlane::landing_descent_rate_cms(float height_above_ground)
{
    if (poscontrol.last_override_descent_ms != 0) {
        const uint32_t now = AP_HAL::millis();
        if (now - poscontrol.last_override_descent_ms < 1000) {
            return poscontrol.override_descent_rate*100;
        }
    }

    if (poscontrol.get_state() == QPOS_LAND_FINAL) {
        // when in final use descent rate for final even if alt has climbed again
        height_above_ground = MIN(height_above_ground, land_final_alt);
    }
    const float max_climb_speed = wp_nav->get_default_speed_up_cms();
    float ret = linear_interpolate(land_final_speed*100, wp_nav->get_default_speed_down_cms(),
                                   height_above_ground,
                                   land_final_alt, land_final_alt+6);

    if (option_is_set(QuadPlane::OPTION::THR_LANDING_CONTROL)) {
        // allow throttle control for landing speed
        const float thr_in = get_pilot_land_throttle();
        if (thr_in > THR_CTRL_LAND_THRESH) {
            thr_ctrl_land = true;
        }
        if (thr_ctrl_land) {
            const float dz = 0.1;
            const float thresh1 = 0.5+dz;
            const float thresh2 = 0.5-dz;
            const float scaling = 1.0 / (0.5 - dz);
            if (thr_in > thresh1) {
                // start climbing
                ret = -(thr_in - thresh1)*scaling*max_climb_speed;
            } else if (thr_in > thresh2) {
                // hold height
                ret = 0;
            } else {
                ret *= (thresh2 - thr_in)*scaling;
            }
        }    
    }

    if (poscontrol.pilot_correction_active) {
        // stop descent when repositioning
        ret = MIN(0, ret);
    }

    return ret;
}

/**
 * @brief Get pilot's desired yaw rate from rudder input
 * 
 * @details Converts rudder stick input to yaw rate command with special
 *          handling for disarm detection, stick mixing modes, and tailsitter
 *          configurations.
 *          
 *          Special cases:
 *          1. Rudder disarm detection:
 *             - If throttle at zero, rudder left, and near-zero vertical velocity
 *             - Disable yaw to allow rudder-disarm sequence
 *             - Only when ARMDISARM rudder arming configured
 *          
 *          2. Stick mixing disabled:
 *             - Return zero in QRTL, GUIDED, or VTOL AUTO modes
 *             - Prevents pilot yaw interference in autonomous modes
 *          
 *          3. Tailsitter scaling:
 *             - Scale by RUDD_DT_GAIN in fixed-wing modes
 *             - Prevents excessive yaw rate when hovering preferences applied to FW
 *             - Body-frame roll input tailsitters require non-zero max rate
 *          
 *          Yaw rate calculation:
 *          - Apply exponential curve to rudder input (command_model_pilot.get_expo())
 *          - Scale by maximum yaw rate (command_model_pilot.get_rate())
 *          - Convert to centidegrees per second
 * 
 * @return Pilot commanded yaw rate in centidegrees/second
 * 
 * @note Returns 0 if pilot may be attempting rudder disarm
 * @note Returns 0 in certain auto modes with STICK_MIXING=NONE
 * @note Tailsitter scaling prevents FW yaw saturation in hover
 * @note Air mode overrides disarm detection (allows yaw at zero throttle)
 * 
 * @see command_model_pilot.get_rate() Maximum yaw rate parameter
 * @see command_model_pilot.get_expo() Exponential curve strength
 * @see RUDD_DT_GAIN parameter for tailsitter scaling
 * 
 * Source: ArduPlane/quadplane.cpp:1785-1825
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    const auto rudder_in = plane.channel_rudder->get_control_in();
    bool manual_air_mode = plane.control_mode->is_vtol_man_throttle() && air_mode_active();
    if (!manual_air_mode &&
        !is_positive(get_throttle_input()) &&
        (!plane.control_mode->does_auto_throttle() || motors->limit.throttle_lower) &&
        plane.arming.get_rudder_arming_type() == AP_Arming::RudderArming::ARMDISARM &&
        rudder_in < 0 &&
        fabsf(inertial_nav.get_velocity_z_up_cms()) < 0.5 * get_pilot_velocity_z_max_dn()) {
        // the user may be trying to disarm, disable pilot yaw control
        return 0;
    }

    if ((plane.g.stick_mixing == StickMixing::NONE) &&
        (plane.control_mode == &plane.mode_qrtl ||
         plane.control_mode->is_guided_mode() ||
         in_vtol_auto())) {
        return 0;
    }

    // add in rudder input
    const float yaw_rate_max = command_model_pilot.get_rate();
    float max_rate = yaw_rate_max;
    if (!in_vtol_mode() && tailsitter.enabled()) {
        // scale by RUDD_DT_GAIN when not in a VTOL mode for
        // tailsitters. This allows for flat turns in tailsitters for
        // fixed wing modes if you want them, but prevents crazy yaw
        // rate demands in fixed wing based on your preferred yaw rate
        // when hovering
        max_rate *= plane.g2.rudd_dt_gain * 0.01;
    }
    if (tailsitter.enabled() &&
        tailsitter.input_type & Tailsitter::input::TAILSITTER_INPUT_BF_ROLL) {
        // must have a non-zero max yaw rate for scaling to work
        max_rate = (yaw_rate_max < 1.0f) ? 1 : yaw_rate_max;
    }
    return input_expo(rudder_in * (1/4500.0), command_model_pilot.get_expo()) * max_rate * 100.0;
}

/**
 * @brief Calculate total desired yaw rate combining all yaw sources
 * 
 * @details Combines multiple yaw rate commands from different sources into
 *          final commanded yaw rate for the attitude controller.
 *          
 *          Yaw rate sources (additive):
 *          1. Assisted flight coordination:
 *             - During fixed-wing assist, use bank angle to generate coordinated yaw
 *             - Provides coordinated turns when VTOL motors assist forward flight
 *             - Only active when assisted_flight flag set
 *          
 *          2. Pilot rudder input:
 *             - Direct yaw rate command from pilot's rudder stick
 *             - Always included (may be zero in certain modes)
 *          
 *          3. Weathervane compensation:
 *             - Automatic nose-into-wind correction during hover/slow flight
 *             - Only when should_weathervane parameter is true
 *             - Helps maintain heading stability in wind
 * 
 * @param[in] should_weathervane true to enable automatic weathervaning
 * 
 * @return Combined yaw rate command in centidegrees/second
 * 
 * @note Multiple yaw sources are additive (summed together)
 * @note Assisted flight coordination provides coordinated turns in FW+VTOL flight
 * @note Weathervaning typically enabled in hover modes, disabled in forward flight
 * 
 * @see desired_auto_yaw_rate_cds() Coordinated turn yaw from bank angle
 * @see get_pilot_input_yaw_rate_cds() Pilot rudder command
 * @see get_weathervane_yaw_rate_cds() Wind compensation yaw
 * 
 * Source: ArduPlane/quadplane.cpp:1830-1847
 */
float QuadPlane::get_desired_yaw_rate_cds(bool should_weathervane)
{
    float yaw_cds = 0;
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_auto_yaw_rate_cds();
    }

    // add in pilot input
    yaw_cds += get_pilot_input_yaw_rate_cds();

    if (should_weathervane) {
        // add in weathervaning
        yaw_cds += get_weathervane_yaw_rate_cds();
    }
    
    return yaw_cds;
}

/**
 * @brief Convert pilot throttle stick to desired climb rate
 * 
 * @details Translates throttle stick position into vertical velocity command
 *          for VTOL altitude control modes (QHOVER, QLOITER, QSTABILIZE, etc.).
 *          
 *          Throttle interpretation:
 *          - Mid stick (trim position): Zero climb rate (hover)
 *          - Above mid: Climb at proportional rate up to Q_PILOT_SPD_UP
 *          - Below mid: Descend at proportional rate up to Q_PILOT_SPD_DN
 *          - Dead zone around mid stick prevents drift
 *          
 *          Failsafe behavior:
 *          - If no valid RC input, returns -50 cm/s (gentle descent)
 *          - Prevents uncontrolled altitude hold during RC loss
 *          
 *          Stick mapping uses pwm_to_angle_dz_trim() which:
 *          - Centers output at trim (mid-stick)
 *          - Applies dead zone for stable hover
 *          - Returns normalized value [-1.0, +1.0]
 * 
 * @return Desired climb rate in cm/s (positive = climb, negative = descend)
 *         Returns -50 cm/s if no valid RC input
 * 
 * @note Separate limits for climb (Q_PILOT_SPD_UP) and descend (Q_PILOT_SPD_DN)
 * @note Dead zone prevents accidental altitude drift near mid-stick
 * @note Failsafe descent rate of 0.5 m/s prevents crashes during RC loss
 * 
 * @see Q_PILOT_SPD_UP parameter (default 2.5 m/s)
 * @see Q_PILOT_SPD_DN parameter (defaults to Q_PILOT_SPD_UP if zero)
 * @see get_pilot_velocity_z_max_dn()
 * 
 * Source: ArduPlane/quadplane.cpp:1917-1930
 */
float QuadPlane::get_pilot_desired_climb_rate_cms(void) const
{
    if (!rc().has_valid_input()) {
        // no valid input means no sensible pilot desired climb rate.
        // descend at 0.5m/s for now
        return -50;
    }
    uint16_t dead_zone = plane.channel_throttle->get_dead_zone();
    uint16_t trim = (plane.channel_throttle->get_radio_max() + plane.channel_throttle->get_radio_min())/2;
    const float throttle_request = plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) *0.01f;
    return throttle_request * (throttle_request > 0.0f ? pilot_speed_z_max_up*100 : get_pilot_velocity_z_max_dn());
}


/**
 * @brief Initialize throttle_wait flag based on current throttle and flight state
 * 
 * @details Sets the throttle_wait flag to prevent immediate motor spool-up
 *          on arming or mode switch. Throttle wait is cleared when pilot raises
 *          throttle or aircraft is already flying.
 *          
 *          Logic:
 *          - throttle_wait = false if throttle >= 10% OR aircraft flying
 *          - throttle_wait = true otherwise (throttle low and on ground)
 *          
 *          Purpose:
 *          - Prevents motors spinning up immediately on arm
 *          - Requires pilot to explicitly raise throttle before flight
 *          - Safety feature: prevents unintended takeoff
 *          - Allows re-arming after landing without motor surprise
 * 
 * @note Called from set_armed() on arm/disarm (unless air mode active)
 * @note Called when entering VTOL modes
 * @note 10% throttle threshold prevents false triggers from stick noise
 * @note Bypassed if already flying (mode transitions in air)
 * 
 * @see throttle_wait flag
 * @see set_armed()
 * @see get_throttle_input()
 * 
 * Source: ArduPlane/quadplane.cpp:1933-1942
 */
void QuadPlane::init_throttle_wait(void)
{
    if (get_throttle_input() >= 10 ||
        plane.is_flying()) {
        throttle_wait = false;
    } else {
        throttle_wait = true;        
    }
}
    
// set motor arming
/**
 * @brief Set motor arming state for quadplane motors
 * 
 * @details Controls arming/disarming of multicopter motors through the
 *          AP_Motors library. Coordinates motor state with vehicle arming
 *          and initializes throttle wait safety feature on arm/disarm.
 *          
 *          Arming sequence:
 *          1. Check quadplane initialized (early exit if not)
 *          2. Pass armed state to motors library
 *          3. If in GUIDED mode: Set guided_wait_takeoff flag
 *          4. Unless air mode active: Initialize throttle_wait flag
 *          
 *          Throttle wait re-initialization on both arm AND disarm prevents
 *          rudder arming on 2nd flight from causing unexpected yaw control.
 *          This ensures consistent behavior across multiple arm cycles.
 *          
 *          Guided mode handling:
 *          - Sets guided_wait_takeoff flag matching armed state
 *          - Prevents guided takeoff commands until armed
 *          - Allows proper sequencing of guided mission startup
 *          
 *          Air mode bypass:
 *          - If air_mode_active(), throttle_wait NOT initialized
 *          - Allows immediate motor response for in-flight recovery
 *          - Critical for stabilization after mid-air arm
 * 
 * @param[in] armed true to arm motors, false to disarm
 * 
 * @note Does nothing if quadplane not initialized (safety check)
 * @note Throttle wait re-initialized on both arm AND disarm
 * @note Air mode bypasses throttle_wait for immediate response
 * @note Motor library handles actual ESC arming protocol
 * @note Guided mode wait_takeoff synchronized with arm state
 * 
 * @warning Disarming in flight will stop motors (altitude loss)
 * 
 * @see init_throttle_wait()
 * @see air_mode_active()
 * @see AP_Motors::armed()
 * 
 * Source: ArduPlane/quadplane.cpp:2005-2021
 */
void QuadPlane::set_armed(bool armed)
{
    if (!initialised) {
        return;
    }
    motors->armed(armed);

    if (plane.control_mode == &plane.mode_guided) {
        guided_wait_takeoff = armed;
    }

    // re-init throttle wait on arm and disarm, to prevent rudder
    // arming on 2nd flight causing yaw
    if (!air_mode_active()) {
        init_throttle_wait();
    }
}


/**
 * @brief Calculate desired climb rate for VTOL assist mode
 * 
 * @details Estimates the desired vertical climb rate when providing VTOL
 *          assistance to fixed-wing flight. The calculation differs based
 *          on flight mode:
 *          
 *          **Auto-throttle modes** (AUTO, GUIDED, etc.):
 *          - Uses altitude error divided by 10s time constant
 *          - climb_rate = altitude_error * 0.1
 *          
 *          **Manual throttle modes** (MANUAL, STABILIZE, etc.):
 *          - Estimates from pilot input (pitch and throttle)
 *          - climb_rate = FBWB_CLIMB_RATE * (pitch/pitch_max) * throttle
 *          
 *          Climb rate is constrained to waypoint navigation limits and
 *          ramped in over 2 seconds to prevent sudden altitude changes.
 * 
 * @return Desired climb rate in cm/s (positive = up)
 * 
 * @note Constrained to Q_WP_SPEED_UP and Q_WP_SPEED_DN parameters
 * @note 2-second ramp prevents sudden transitions when assist begins
 * @note Called during VTOL assist in fixed-wing modes
 * 
 * @see last_pidz_active_ms For assist timing
 * @see plane.calc_altitude_error_cm() For altitude tracking error
 * 
 * Source: ArduPlane/quadplane.cpp:1572-1596
 */
float QuadPlane::assist_climb_rate_cms(void) const
{
    float climb_rate;
    if (plane.control_mode->does_auto_throttle()) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate = plane.calc_altitude_error_cm() * 0.1f;
    } else {
        // otherwise estimate from pilot input
        climb_rate = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(plane.aparm.pitch_limit_max*100));
        climb_rate *= plane.get_throttle_input();
    }
    climb_rate = constrain_float(climb_rate, -wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms());

    // bring in the demanded climb rate over 2 seconds
    const uint32_t ramp_up_time_ms = 2000;
    const uint32_t dt_since_start = last_pidz_active_ms - last_pidz_init_ms;
    if (dt_since_start < ramp_up_time_ms) {
        climb_rate = linear_interpolate(0, climb_rate, dt_since_start, 0, ramp_up_time_ms);
    }
    
    return climb_rate;
}

/**
 * @brief Calculate desired coordinated turn yaw rate for auto flight
 * 
 * @details Computes the yaw rate required for a coordinated turn at current
 *          bank angle and airspeed using standard turn coordination formula:
 *          
 *          Earth frame: yaw_rate = g * tan(roll) / V
 *          Body frame:  yaw_rate = g * sin(roll) / V
 *          
 *          Where g = gravitational acceleration (9.81 m/s²)
 *                V = airspeed (minimum AIRSPEED_MIN enforced)
 *                roll = nav_roll_cd (commanded roll angle)
 *          
 *          Body frame uses sin() for yaw rate in aircraft frame.
 *          Earth frame uses tan() for turn rate about vertical axis.
 * 
 * @param[in] body_frame true=body frame yaw rate, false=earth frame turn rate
 * 
 * @return Desired yaw rate in centidegrees/second
 * 
 * @note Minimum airspeed of AIRSPEED_MIN used to prevent division by zero
 * @note Absolute minimum of 1 m/s enforced for mathematical stability
 * @note Used during transitions and VTOL assist for coordinated flight
 * 
 * @see plane.nav_roll_cd Current commanded roll angle
 * @see plane.aparm.airspeed_min Minimum airspeed parameter
 * 
 * Source: ArduPlane/quadplane.cpp:1601-1615
 */
float QuadPlane::desired_auto_yaw_rate_cds(bool body_frame) const
{
    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed) || aspeed < plane.aparm.airspeed_min) {
        aspeed = plane.aparm.airspeed_min;
    }
    if (aspeed < 1) {
        aspeed = 1;
    }
    if (body_frame) {
        return degrees(GRAVITY_MSS * sinf(cd_to_rad(plane.nav_roll_cd))/aspeed) * 100;
    }
    return degrees(GRAVITY_MSS * tanf(cd_to_rad(plane.nav_roll_cd))/aspeed) * 100;
}

/*
  update for transition from quadplane to fixed wing mode
 */
void SLT_Transition::update()
{
    const uint32_t now = millis();
    
    if (!plane.arming.is_armed_and_safety_off()) {
        // reset the failure timer if we are disarmed
        transition_start_ms = now;
    }

    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);

    /*
      see if we should provide some assistance
     */
    if (quadplane.assist.should_assist(aspeed, have_airspeed)) {
        // the quad should provide some assistance to the plane
        quadplane.assisted_flight = true;
        // update transition state for vehicles using airspeed wait
        if (!in_forced_transition) {
            const bool show_message = transition_state != TRANSITION_AIRSPEED_WAIT || transition_start_ms == 0;
            if (show_message) {
                gcs().send_text(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
            }
            transition_state = TRANSITION_AIRSPEED_WAIT;
            if (transition_start_ms == 0) {
                transition_start_ms = now;
            }
        }
    } else {
        quadplane.assisted_flight = false;
    }


    // if rotors are fully forward then we are not transitioning,
    // unless we are waiting for airspeed to increase (in which case
    // the tilt will decrease rapidly)
    if (quadplane.tiltrotor.fully_fwd() && transition_state != TRANSITION_AIRSPEED_WAIT) {
        if (transition_state == TRANSITION_TIMER) {
            float throttle;
            if (plane.quadplane.tiltrotor.get_forward_throttle(throttle)) {
                // Reset the TECS minimum throttle to match throttle of forward thrust motors
                // and set the throttle channel slew rate limiter to prevent a sudden drop in throttle
                plane.TECS_controller.set_throttle_min(throttle, true);
                SRV_Channels::set_slew_last_scaled_output(SRV_Channel::k_throttle, throttle * 100);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100);
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        }
        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
    }

    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = now;
        }

        // check if we have failed to transition while in TRANSITION_AIRSPEED_WAIT
        if (transition_start_ms != 0 &&
        (quadplane.transition_failure.timeout > 0) &&
        ((now - transition_start_ms) > ((uint32_t)quadplane.transition_failure.timeout * 1000))) {
            if (!quadplane.transition_failure.warned) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Transition failed, exceeded time limit");
                quadplane.transition_failure.warned = true;
            }
            // if option is set and ground speed> 1/2 AIRSPEED_MIN for non-tiltrotors, then complete transition, otherwise QLAND.
            // tiltrotors will immediately transition
            const bool tiltrotor_with_ground_speed = quadplane.tiltrotor.enabled() && (plane.ahrs.groundspeed() > plane.aparm.airspeed_min * 0.5);
            if (quadplane.option_is_set(QuadPlane::OPTION::TRANS_FAIL_TO_FW) && tiltrotor_with_ground_speed) {
                transition_state = TRANSITION_TIMER;
                in_forced_transition = true;
            } else {
                switch (QuadPlane::TRANS_FAIL::ACTION(quadplane.transition_failure.action)) {
                    case QuadPlane::TRANS_FAIL::ACTION::QLAND:
                        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION);
                        break;

                    case QuadPlane::TRANS_FAIL::ACTION::QRTL:
                        plane.set_mode(plane.mode_qrtl, ModeReason::VTOL_FAILED_TRANSITION);
                        quadplane.poscontrol.set_state(QuadPlane::QPOS_POSITION1);
                        break;

                    default:
                        break;
                }
            }
        } else {
            quadplane.transition_failure.warned = false;
        }

        transition_low_airspeed_ms = now;
        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !quadplane.assisted_flight) {
            transition_state = TRANSITION_TIMER;
            airspeed_reached_tilt = quadplane.tiltrotor.current_tilt;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        quadplane.assisted_flight = true;

        // do not allow a climb on the quad motors during transition a
        // climb would add load to the airframe, and prolongs the
        // transition. We don't limit the climb rate on tilt rotors as
        // otherwise the plane can end up in high-alpha flight with
        // low VTOL thrust and may not complete a transition
        float climb_rate_cms = quadplane.assist_climb_rate_cms();
        if (quadplane.option_is_set(QuadPlane::OPTION::LEVEL_TRANSITION) && !quadplane.tiltrotor.enabled()) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
        }
        quadplane.hold_hover(climb_rate_cms);

        if (!quadplane.tiltrotor.is_vectored()) {
            // set desired yaw rate to a coordinated turn
            quadplane.attitude_control->reset_yaw_target_and_rate();
            quadplane.attitude_control->rate_bf_yaw_target(quadplane.desired_auto_yaw_rate_cds(true));
        }
        if (quadplane.tiltrotor.enabled() && !quadplane.tiltrotor.has_fw_motor()) {
            // tilt rotors without dedicated fw motors do not have forward throttle output in this stage
            // prevent throttle I wind up
            plane.TECS_controller.reset_throttle_I();
        }

        last_throttle = motors->get_throttle();

        // reset integrators while we are below target airspeed as we
        // may build up too much while still primarily under
        // multicopter control
        plane.pitchController.reset_I();
        plane.rollController.reset_I();

        // give full authority to attitude control
        quadplane.attitude_control->set_throttle_mix_max(1.0f);
        break;
    }
        
    case TRANSITION_TIMER: {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the transition time, but continue
        // to stabilize and wait for any required forward tilt to complete and the timer to expire
        const uint32_t transition_timer_ms = now - transition_low_airspeed_ms;
        const float trans_time_ms = constrain_float(quadplane.transition_time_ms,500,30000);
        const bool tilt_fwd_complete = !quadplane.tiltrotor.enabled() || quadplane.tiltrotor.tilt_angle_achieved();
        if (transition_timer_ms > unsigned(trans_time_ms) && tilt_fwd_complete) {
            transition_state = TRANSITION_DONE;
            in_forced_transition = false;
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
            float throttle;
            if (plane.quadplane.tiltrotor.get_forward_throttle(throttle)) {
                // Reset the TECS minimum throttle to match throttle of forward thrust motors
                // and set the throttle channel slew rate limiter to prevent a sudden drop in throttle
                plane.TECS_controller.set_throttle_min(throttle, true);
                SRV_Channels::set_slew_last_scaled_output(SRV_Channel::k_throttle, throttle * 100);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100);
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
        }

        float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
        float throttle_scaled = last_throttle * transition_scale;

        // set zero throttle mix, to give full authority to
        // throttle. This ensures that the fixed wing controllers get
        // a chance to learn the right integrators during the transition
        quadplane.attitude_control->set_throttle_mix_value(0.5*transition_scale);

        if (throttle_scaled < 0.01) {
            // ensure we don't drop all the way to zero or the motors
            // will stop stabilizing
            throttle_scaled = 0.01;
        }
        if (quadplane.tiltrotor.enabled() && !quadplane.tiltrotor.has_vtol_motor() && !quadplane.tiltrotor.has_fw_motor()) {
            // All motors tilting, Use a combination of vertical and forward throttle based on current tilt angle
            // scale from all VTOL throttle at airspeed_reached_tilt to all forward throttle at fully forward tilt
            // this removes a step change in throttle once assistance is stopped
            const float ratio = (constrain_float(quadplane.tiltrotor.current_tilt, airspeed_reached_tilt, quadplane.tiltrotor.get_fully_forward_tilt()) - airspeed_reached_tilt) / (quadplane.tiltrotor.get_fully_forward_tilt() - airspeed_reached_tilt);
            const float fw_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),0) * 0.01;
            throttle_scaled = constrain_float(throttle_scaled * (1.0-ratio) + fw_throttle * ratio, 0.0, 1.0);
        }
        quadplane.assisted_flight = true;
        quadplane.hold_stabilize(throttle_scaled);

        if (!quadplane.tiltrotor.is_vectored()) {
            // set desired yaw rate to a coordinated turn
            quadplane.attitude_control->reset_yaw_target_and_rate();
            quadplane.attitude_control->rate_bf_yaw_target(quadplane.desired_auto_yaw_rate_cds(true));
        }
        break;
    }

    case TRANSITION_DONE:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        set_last_fw_pitch();
        in_forced_transition = false;
        return;
    }

    quadplane.motors_output();

    set_last_fw_pitch();
}

void SLT_Transition::VTOL_update()
{
    /*
      setup the transition state appropriately for next time we go into a non-VTOL mode
    */
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
    if (quadplane.throttle_wait && !plane.is_flying()) {
        in_forced_transition = false;
        transition_state = TRANSITION_DONE;
    } else {
        /*
          setup for airspeed wait for later
        */
        transition_state = TRANSITION_AIRSPEED_WAIT;
    }
    last_throttle = motors->get_throttle();

    // Keep assistance reset while not checking
    quadplane.assist.reset();
}

/**
 * @brief Main QuadPlane update function called every main loop iteration
 * 
 * @details Central update function for all QuadPlane operations. Called at main loop
 *          rate (typically 400Hz for multicopter control, 50Hz minimum for FW).
 *          
 *          Update Sequence:
 *          1. Ensure setup() has completed
 *          2. Update motor interlock status based on E-stop
 *          3. Update AHRS view pitch trim if changed
 *          4. Handle advanced failsafe termination if active
 *          5. Handle motor test mode if running
 *          6. Reset controllers when disarmed
 *          7. Branch based on flight mode:
 *             - VTOL mode: Run motors_output() and VTOL_update()
 *             - FW mode: Run transition->update() and check for assist
 *          8. Update tiltrotor mechanisms
 *          9. Update forward throttle in VTOL modes
 *          10. Log motor and attitude data
 *          
 *          Mode-Specific Behavior:
 *          - Manual/Acro/Training FW modes: Motors shut down (except tailsitters)
 *          - Other FW modes: Transition management and assist monitoring
 *          - VTOL modes: Full multicopter control active
 *          - Airbrake: VTOL motors assist deceleration
 *          
 *          Safety Features:
 *          - throttle_wait disabled when throttle rises above 10%
 *          - Emergency stop resets rate controller I-terms
 *          - Disarmed state resets all controllers
 *          - Advanced failsafe can force motor shutdown
 * 
 * @note Called every loop iteration when QuadPlane is enabled
 * @note Logging occurs at varied rates (ANG at loop rate, QTUN at 25Hz, MOTB at 10Hz)
 * @note Motor output may be suppressed by update_throttle_suppression()
 * 
 * @warning Do not call motors->output() directly - use motors_output() instead
 * 
 * @see motors_output()
 * @see transition->update()
 * @see VTOL_Assist::update()
 * @see tiltrotor.update()
 * 
 * Source: ArduPlane/quadplane.cpp:1685-1811
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }

    // keep motors interlock state upto date with E-stop
    motors->set_interlock(!SRV_Channels::get_emergency_stop());

    if ((ahrs_view != NULL) && !is_equal(_last_ahrs_trim_pitch, ahrs_trim_pitch.get())) {
        _last_ahrs_trim_pitch = ahrs_trim_pitch.get();
        ahrs_view->set_pitch_trim(_last_ahrs_trim_pitch);
    }

#if AP_ADVANCEDFAILSAFE_ENABLED
    if (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
#endif
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }

    if (SRV_Channels::get_emergency_stop()) {
        attitude_control->reset_rate_controller_I_terms();
    }

    if (!plane.arming.is_armed_and_safety_off()) {
        /*
          make sure we don't have any residual control from previous flight stages
         */
        if (tailsitter.enabled()) {
            // tailsitters only relax I terms, to make ground testing easier
            attitude_control->reset_rate_controller_I_terms();
        } else {
            // otherwise full relax
            attitude_control->relax_attitude_controllers();
        }
        // todo: do you want to set the throttle at this point?
        pos_control->relax_U_controller(0);
    }

    const uint32_t now = AP_HAL::millis();
    if (!in_vtol_mode() && !in_vtol_airbrake()) {
        // we're in a fixed wing mode, cope with transitions and check
        // for assistance needed
        if (plane.control_mode == &plane.mode_manual ||
            plane.control_mode == &plane.mode_acro ||
            plane.control_mode == &plane.mode_training) {
            // in manual modes quad motors are always off
            if (!tailsitter.enabled()) {
                set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
                motors->output();
            }
            transition->force_transition_complete();
            assisted_flight = false;
        } else {
            transition->update();
        }

    } else {

        assisted_flight = in_vtol_airbrake();

        // output to motors
        motors_output();

        transition->VTOL_update();

    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.get_throttle_input() > 10 ||
         !rc().has_valid_input())) {
        throttle_wait = false;
    }

    tiltrotor.update();

    if (in_vtol_mode()) {
        // if enabled output forward throttle else 0
        float fwd_thr = 0;
        if (allow_forward_throttle_in_vtol_mode()) {
            fwd_thr = forward_throttle_pct();
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, fwd_thr);
    }

#if HAL_LOGGING_ENABLED
    // motors logging
    if (motors->armed()) {
        const bool motors_active = in_vtol_mode() || assisted_flight;
        if (motors_active && (motors->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN)) {
            // log ANG at main loop rate
            bool sysid_running = false;
#if AP_PLANE_SYSTEMID_ENABLED
            sysid_running = plane.g2.systemid.is_running();
#endif
            if (!sysid_running) {
                if (show_vtol_view()) {
                    attitude_control->Write_ANG();
                }
                // log RATE at main loop rate
                attitude_control->Write_Rate(*pos_control);
            }

            // log MOTB at 10 Hz
            if (now - last_motb_log_ms > 100) {
                last_motb_log_ms = now;
                motors->Log_Write();
            }
        }
        // log QTUN at 25 Hz if motors are active, or have been active in the last quarter second
        if ((motors_active || (now - last_motors_active_ms < 250)) && (now - last_qtun_log_ms > 40)) {
            last_qtun_log_ms = now;
            Log_Write_QControl_Tuning();
        }
    }
#else
    (void)now;
#endif  // HAL_LOGGING_ENABLED
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DesiredSpoolState::SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
/**
 * @brief Suppress VTOL motors to prevent unintended ground operation
 * 
 * @details Implements safety logic to prevent quadplane motors from spinning
 *          when vehicle is on ground but armed. Motors are set to GROUND_IDLE
 *          unless specific conditions indicate intentional flight.
 *          
 *          Motor suppression logic (all conditions checked):
 *          1. Motors active within last 2 seconds → Allow (hysteresis)
 *          2. Motors already at/below GROUND_IDLE → Skip (already suppressed)
 *          3. Guided mode waiting for takeoff → Suppress
 *          4. Throttle above zero (with arming check or manual mode) → Allow
 *          5. VTOL manual mode with air mode active → Allow
 *          6. Auto throttle mode with unsuppressed throttle → Allow
 *          7. Vertical velocity > 1 m/s → Allow (already flying)
 *          8. Altitude > 5m above home → Allow (already flying)
 *          9. Auto mode VTOL takeoff command → Allow
 *          
 *          If none of above allow conditions met → Set GROUND_IDLE
 *          
 *          Purpose:
 *          - Prevents motors spinning up on level ground after arm
 *          - Allows motors in flight or during intentional takeoff
 *          - Provides 2-second hysteresis to prevent oscillation
 *          - Respects air mode for immediate response capability
 * 
 * @note Called at high rate from update() loop
 * @note 2-second hysteresis prevents rapid on/off cycling
 * @note Air mode bypasses suppression for aerobatic capability
 * @note Sprung throttle (no arming throttle check) requires manual mode or actual stick movement
 * @note 1 m/s vertical velocity threshold detects actual flight
 * @note 5m altitude threshold ensures flight detection
 * 
 * @warning Suppression failure could cause unexpected motor spin-up on ground
 * 
 * @see last_motors_active_ms
 * @see guided_wait_takeoff
 * @see air_mode_active()
 * 
 * Source: ArduPlane/quadplane.cpp:2582-2641
 */
void QuadPlane::update_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        return;
    }

    if (guided_wait_takeoff) {
        goto idle_state;
    }

    /* if the users throttle is above zero then allow motors to run

       if the user has unset the "check throttle zero when arming"
       then the RC controller has a sprung throttle and we should not
       consider non-zero throttle to mean that pilot is commanding
       takeoff unless in a manual throttle mode
    */
    if (!is_zero(get_throttle_input()) &&
        (rc().arming_check_throttle() ||
         plane.control_mode->is_vtol_man_throttle() ||
         plane.channel_throttle->norm_input_dz() > 0)) {
        return;
    }

    // if in a VTOL manual throttle mode and air_mode is on then allow motors to run
    if (plane.control_mode->is_vtol_man_throttle() && air_mode_active()) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed) {
        return;
    }

    // if our vertical velocity is greater than 1m/s then allow motors to run
    if (fabsf(inertial_nav.get_velocity_z_up_cms()) > 100) {
        return;
    }

    // if we are more than 5m from home altitude then allow motors to run
    if (plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING) > 5) {
        return;
    }

    // allow for takeoff
    if (plane.control_mode == &plane.mode_auto && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return;
    }

idle_state:
    // motors should be in the spin when armed state to warn user they could become active
    set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

/**
 * @brief Update estimated throttle required to hover (adaptive learning)
 * 
 * @details Continuously learns the throttle level needed for neutral hover
 *          by observing actual throttle used during stable hover conditions.
 *          This learned value improves altitude hold and position control.
 *          
 *          Update requirements (all must be met):
 *          1. QuadPlane available and initialized
 *          2. Motors armed AND vehicle flying (not on ground)
 *          3. No vertical velocity command (zero desired climb rate)
 *          4. Forward motor not running (no wing lift assistance)
 *             - Tailsitters: Always allowed
 *             - Others: Forward throttle < THR_MIN + 10%
 *          5. Z controller active within last 20ms
 *          6. Stable hover conditions:
 *             - Throttle > 0
 *             - Vertical velocity < 60 cm/s (0.6 m/s)
 *             - Roll < 5° (500 centidegrees)
 *             - Pitch < 5° (500 centidegrees)
 *             - Airspeed < 30% of min airspeed
 *          
 *          Learning algorithm:
 *          - Uses exponential moving average with 0.01 time constant
 *          - Slowly adapts to battery voltage changes
 *          - Compensates for vehicle weight changes
 *          - Improves over multiple flights
 *          
 *          Gyro FFT integration:
 *          - If enabled, also updates hover frequency for FFT analysis
 *          - Helps identify vibration issues at hover
 * 
 * @note Called at 100 Hz from main update loop
 * @note Learning only occurs during stable hover (strict conditions)
 * @note Forward motor check prevents wing lift from biasing estimate
 * @note 20ms Z controller timeout ensures recent altitude control
 * @note 60 cm/s vertical velocity threshold defines "stable"
 * @note 5° attitude threshold prevents dynamic maneuver contamination
 * @note Low airspeed requirement ensures pure hover (no translational lift)
 * 
 * @see motors->update_throttle_hover() - Performs exponential average
 * @see Q_M_THST_HOVER parameter - Learned hover throttle value
 * 
 * Source: ArduPlane/quadplane.cpp:2687-2722
 */
void QuadPlane::update_throttle_hover()
{
    if (!available()) {
        return;
    }
    
    // if not armed or landed exit
    if (!motors->armed() || !is_flying_vtol()) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_NEU_cms().z)) {
        return;
    }

    // do not update if quadplane forward motor is running (wing may be generating lift)
    // we use the THR_MIN value to account for petrol motors idling at THR_MIN
    if (!tailsitter.enabled() && (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > MAX(0,plane.aparm.throttle_min+10))) {
        return;
    }

    // don't update if Z controller not running
    const uint32_t now = AP_HAL::millis();
    if (now - last_pidz_active_ms > 20) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    float aspeed;
    // calc average throttle if we are in a level hover and low airspeed
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        labs(ahrs_view->roll_sensor) < 500 && labs(ahrs_view->pitch_sensor) < 500 &&
        ahrs.airspeed_estimate(aspeed) && aspeed < plane.aparm.airspeed_min*0.3) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        plane.gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}
/**
 * @brief Output commands to multicopter motors with safety checks
 * 
 * @details Executes motor output with comprehensive safety interlocks and
 *          conditional rate controller execution. This is the safe way to
 *          command motor outputs in VTOL modes.
 *          
 *          Safety Checks (in order):
 *          1. OPTION_DELAY_ARMING: Delay motor spool-up after arming
 *          2. OPTION_DISARMED_TILT: Allow tilt motors to return to vertical
 *          3. Armed and safety off verification
 *          4. Emergency stop status
 *          5. Advanced failsafe termination check
 *          6. ESC calibration mode bypass
 *          7. Tailsitter transition state check
 *          
 *          Rate Controller Execution:
 *          When run_rate_controller=true:
 *          - Updates loop timing (dt) for motors, attitude, and position controllers
 *          - Runs attitude_control->rate_controller_run() for stabilization
 *          - Checks for VTOL recovery assist
 *          - Resets rate controller targets after execution
 *          
 *          Motor Output:
 *          - Calls motors->output() to send PWM signals to ESCs
 *          - Updates last_motors_active_ms timestamp for throttle suppression
 *          - Applies throttle suppression if safety conditions met
 * 
 * @param[in] run_rate_controller If true, execute rate controller before motor output.
 *                                 Set false when rate controller already ran this loop.
 * 
 * @note Called from update() during VTOL operations
 * @note Tracks last_motors_active_ms for 2-second throttle suppression window
 * @note Relax attitude control if >100ms since last controller execution
 * 
 * @warning Never call motors->output() directly - always use this function
 * @warning Tailsitter transitions bypass this function (handled by FW stabilization)
 * 
 * @see update_throttle_suppression()
 * @see AC_AttitudeControl::rate_controller_run()
 * @see VTOL_Assist::check_VTOL_recovery()
 * 
 * Source: ArduPlane/quadplane.cpp:1932-2003
 */
void QuadPlane::motors_output(bool run_rate_controller)
{
    /* Delay for ARMING_DELAY_MS after arming before allowing props to spin:
       1) for safety (OPTION_DELAY_ARMING)
       2) to allow motors to return to vertical (OPTION_DISARMED_TILT)
     */
    if (option_is_set(QuadPlane::OPTION::DISARMED_TILT) || option_is_set(QuadPlane::OPTION::DELAY_ARMING)) {
        if (plane.arming.get_delay_arming()) {
            // delay motor start after arming
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            return;
        }
    }

#if AP_ADVANCEDFAILSAFE_ENABLED
    if (!plane.arming.is_armed_and_safety_off() ||
        (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) ||
         SRV_Channels::get_emergency_stop()) {
#else
    if (!plane.arming.is_armed_and_safety_off() || SRV_Channels::get_emergency_stop()) {
#endif
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == &plane.mode_qstabilize) {
        // output is direct from run_esc_calibration()
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now) && !assisted_flight) {
        /*
          don't run the motor outputs while in tailsitter->vtol
          transition. That is taken care of by the fixed wing
          stabilisation code
         */
        return;
    }

    if (run_rate_controller) {
        if (now - last_att_control_ms > 100) {
            // relax if have been inactive
            relax_attitude_control();
        }

        // see if we need to be in VTOL recovery
        assist.check_VTOL_recovery();

        // run low level rate controllers that only require IMU data and set loop time
        const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
        motors->set_dt_s(last_loop_time_s);
        attitude_control->set_dt_s(last_loop_time_s);
        pos_control->set_dt_s(last_loop_time_s);
        attitude_control->rate_controller_run();
        // reset sysid and other temporary inputs
        attitude_control->rate_controller_target_reset();
        last_att_control_ms = now;
    }

    // see if motors should be shut down
    update_throttle_suppression();

    motors->output();

    // remember when motors were last active for throttle suppression
    if (motors->get_throttle() > 0.01f || tiltrotor.motors_active()) {
        last_motors_active_ms = now;
    }

}

/**
 * @brief Handle MAVLink DO_VTOL_TRANSITION command
 * 
 * @details Processes MAV_CMD_DO_VTOL_TRANSITION mission commands to change
 *          between fixed-wing and VTOL flight modes. This command is used
 *          in AUTO missions to explicitly trigger transitions.
 *          
 *          Supported Transition States:
 *          - MAV_VTOL_STATE_MC: Transition to multicopter (VTOL) mode
 *          - MAV_VTOL_STATE_FW: Transition to fixed-wing mode
 *          
 *          The transition state machine manages the actual transition sequence,
 *          including motor spool-up, control handoff, and airspeed checks.
 * 
 * @param[in] state Desired VTOL state (MAV_VTOL_STATE_MC or MAV_VTOL_STATE_FW)
 * 
 * @return true if transition command accepted, false if QuadPlane not available
 * 
 * @note Used in AUTO missions with DO_VTOL_TRANSITION commands
 * @note Transition may take several seconds to complete
 * @note Transition progress monitored by transition state machine
 * 
 * @see MAV_CMD_DO_VTOL_TRANSITION
 * @see transition->update()
 * 
 * Source: ArduPlane/quadplane.cpp:2008-2048
 */
/**
 * @brief Handle MAV_CMD_DO_VTOL_TRANSITION mission command
 * 
 * @details Processes mission commands to transition between VTOL (multicopter)
 *          and fixed-wing flight modes during AUTO missions. Validates preconditions,
 *          updates vehicle state, and initializes appropriate mode parameters.
 *          
 *          **Transition Command Format:**
 *          MAV_CMD_DO_VTOL_TRANSITION with parameter:
 *          - MAV_VTOL_STATE_MC: Transition to multicopter (VTOL) mode
 *          - MAV_VTOL_STATE_FW: Transition to fixed-wing mode
 *          
 *          **Precondition Validation:**
 *          
 *          **1. QuadPlane Availability Check:**
 *          - !available() - QuadPlane not configured or disabled
 *          - GCS message: "VTOL not available"
 *          - Return false (command rejected)
 *          
 *          Reasons for unavailable:
 *          - Q_ENABLE = 0 (quadplane disabled)
 *          - Motor setup failed during initialization
 *          - Critical hardware not detected
 *          
 *          **2. Mode Validation:**
 *          - plane.control_mode != &plane.mode_auto
 *          - Transition commands only valid in AUTO mode
 *          - GCS message: "VTOL transition only in AUTO"
 *          - Return false (command rejected)
 *          
 *          Rationale:
 *          - Manual mode transitions use mode changes (QLOITER, QHOVER, etc.)
 *          - AUTO mode requires explicit transition commands in mission
 *          - Prevents accidental transitions outside mission execution
 *          
 *          **State Transition Handling:**
 *          
 *          **Case: MAV_VTOL_STATE_MC (Enter VTOL Mode)**
 *          
 *          Purpose: Transition from fixed-wing to multicopter flight
 *          
 *          Status message (if not already in VTOL):
 *          - if (!plane.auto_state.vtol_mode):
 *            * GCS message: "Entered VTOL mode" (MAV_SEVERITY_NOTICE)
 *          - Informs operator of mode change
 *          
 *          State update:
 *          - plane.auto_state.vtol_mode = true
 *          - Marks AUTO mission as VTOL segment
 *          - Affects waypoint handling and controller selection
 *          
 *          **Forward Thrust Reset (Precautionary):**
 *          - q_fwd_throttle = 0.0 (no forward thrust)
 *          - q_fwd_pitch_lim_cd = 100 * q_fwd_pitch_lim (base pitch limit)
 *          
 *          Rationale:
 *          - Should be handled by mode_enter(), but defensive programming
 *          - Ensures clean state for VTOL flight
 *          - Prevents residual forward thrust in VTOL mode
 *          - Critical for tilt-rotor aircraft (motors must be vertical)
 *          
 *          Return: true (transition command accepted)
 *          
 *          **Case: MAV_VTOL_STATE_FW (Enter Fixed-Wing Mode)**
 *          
 *          Purpose: Transition from multicopter to fixed-wing flight
 *          
 *          Status message (if not already in FW):
 *          - if (plane.auto_state.vtol_mode):
 *            * GCS message: "Exited VTOL mode" (MAV_SEVERITY_NOTICE)
 *          - Informs operator of mode change
 *          
 *          State update:
 *          - plane.auto_state.vtol_mode = false
 *          - Marks AUTO mission as fixed-wing segment
 *          - Switches to fixed-wing controllers and navigation
 *          
 *          Return: true (transition command accepted)
 *          
 *          **Case: Default (Invalid State)**
 *          - GCS message: "Invalid VTOL mode"
 *          - Return false (command rejected)
 *          
 *          **Mission Integration:**
 *          
 *          Typical mission usage:
 *          1. MAV_CMD_NAV_VTOL_TAKEOFF - Vertical takeoff
 *          2. MAV_CMD_NAV_WAYPOINT - VTOL waypoints
 *          3. MAV_CMD_DO_VTOL_TRANSITION(FW) - Switch to fixed-wing
 *          4. MAV_CMD_NAV_WAYPOINT - Fixed-wing cruise waypoints
 *          5. MAV_CMD_DO_VTOL_TRANSITION(MC) - Switch to VTOL
 *          6. MAV_CMD_NAV_VTOL_LAND - Vertical landing
 *          
 *          **vtol_mode Flag Effects:**
 *          - Controls waypoint approach logic (straight vs. loiter)
 *          - Determines which attitude/position controllers active
 *          - Affects speed targets and acceleration limits
 *          - Modifies navigation behavior for VTOL vs FW
 *          
 *          **Safety Considerations:**
 *          - Transition at appropriate altitude and airspeed
 *          - Mission planner responsible for safe transition points
 *          - Vehicle must have sufficient altitude for FW→VTOL
 *          - Airspeed must be adequate for VTOL→FW
 *          - Forward thrust reset prevents unexpected behavior
 * 
 * @param[in] state Desired VTOL state (MAV_VTOL_STATE_MC or MAV_VTOL_STATE_FW)
 * 
 * @return true if transition command accepted and executed
 * @return false if command rejected (not available, wrong mode, invalid state)
 * 
 * @note Only valid in AUTO mode
 * @note GCS messages inform operator of transition status
 * @note Forward thrust parameters reset as precaution during MC entry
 * @note vtol_mode flag affects navigation and control behavior
 * @note Mission planner responsible for safe transition altitude/speed
 * @note Duplicate status messages suppressed (only on actual state change)
 * 
 * @warning SAFETY-CRITICAL: Controls flight mode transitions
 * @warning Transitions at inappropriate altitude/speed can cause crash
 * @warning Mission must ensure adequate conditions for transitions
 * @warning Forward thrust must be zero before entering VTOL mode
 * 
 * @see MAV_CMD_DO_VTOL_TRANSITION - Mission command definition
 * @see MAV_VTOL_STATE - State enumeration
 * @see plane.auto_state.vtol_mode - Current VTOL/FW state flag
 * @see available() - QuadPlane availability check
 * @see mode_enter() - Mode entry initialization
 * @see q_fwd_throttle - Forward thrust output
 * @see q_fwd_pitch_lim - Forward pitch limit parameter
 * 
 * Source: ArduPlane/quadplane.cpp:2919-2957
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state) const
{
    if (!available()) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL not available");
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return false;
    }
    switch (state) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        // This is a precaution. It should be looked after by the call to QuadPlane::mode_enter(void) on mode entry.
        plane.quadplane.q_fwd_throttle = 0.0f;
        plane.quadplane.q_fwd_pitch_lim_cd = 100.0f * plane.quadplane.q_fwd_pitch_lim;
        return true;
        
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;

        return true;

    default:
        break;
    }

    gcs().send_text(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return false;
}

/*
  are we in a VTOL auto state?
 */
/**
 * @brief Check if currently in VTOL AUTO mode
 * 
 * @details Determines if the aircraft is in AUTO mode and executing a
 *          VTOL mission command. This is more comprehensive than just
 *          checking auto_state.vtol_mode flag - it also examines the
 *          current navigation command type.
 *          
 *          Returns true for these mission commands:
 *          - MAV_CMD_NAV_VTOL_TAKEOFF
 *          - MAV_CMD_NAV_LOITER_* (if vtol_loiter flag set)
 *          - MAV_CMD_NAV_TAKEOFF (if configured as VTOL takeoff)
 *          - MAV_CMD_NAV_VTOL_LAND, NAV_LAND, PAYLOAD_PLACE (if VTOL land)
 * 
 * @return true if in VTOL AUTO mode, false otherwise
 * 
 * @note More comprehensive than checking vtol_mode flag alone
 * @note Used to determine mission waypoint handling behavior
 * @note Examines current navigation command to determine VTOL status
 * 
 * @see in_vtol_mode() For general VTOL mode detection
 * @see is_vtol_takeoff() For takeoff type detection
 * @see is_vtol_land() For land type detection
 * 
 * Source: ArduPlane/quadplane.cpp:2448-2474
 */
bool QuadPlane::in_vtol_auto(void) const
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.auto_state.vtol_mode) {
        return true;
    }
    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return plane.auto_state.vtol_loiter;
    case MAV_CMD_NAV_TAKEOFF:
        return is_vtol_takeoff(id);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return is_vtol_land(id);
    default:
        return false;
    }
}

/**
 * @brief Check if currently in any VTOL flight mode
 * 
 * @details Primary mode detection function that determines if the aircraft
 *          is flying in a mode using vertical velocity control and VTOL motors.
 *          This is used to decide if transition handling code should run.
 *          
 *          Returns true when:
 *          - In explicit VTOL modes (QHOVER, QLOITER, QLAND, QRTL, etc.)
 *          - In VTOL land sequence (except APPROACH and AIRBRAKE phases)
 *          - In GUIDED mode with vtol_loiter or guided_takeoff active
 *          - In VTOL AUTO mode (beyond AIRBRAKE phase)
 *          
 *          **Important**: AIRBRAKE phase is NOT considered in_vtol_mode
 *          even though VTOL motors are running. This allows transition
 *          logic to manage the switch from fixed-wing to VTOL.
 * 
 * @return true if in VTOL mode with vertical control active, false otherwise
 * 
 * @note QPOS_AIRBRAKE is NOT considered VTOL mode (transition management)
 * @note Used to determine which controller (VTOL vs FW) is active
 * @note Determines whether to run transition handling code
 * 
 * @see in_vtol_auto() For AUTO-specific detection
 * @see in_vtol_land_sequence() For landing phase detection
 * @see plane.control_mode->is_vtol_mode()
 * 
 * Source: ArduPlane/quadplane.cpp:2476-2515
 */
bool QuadPlane::in_vtol_mode(void) const
{
    if (!available()) {
        return false;
    }
    if (in_vtol_land_sequence()) {
        return poscontrol.get_state() != QPOS_APPROACH && poscontrol.get_state() != QPOS_AIRBRAKE;
    }
    if (plane.control_mode->is_vtol_mode()) {
        return true;
    }
    if (plane.control_mode->is_guided_mode()
        && plane.auto_state.vtol_loiter &&
        poscontrol.get_state() > QPOS_APPROACH) {
        return true;
    }
    if (plane.control_mode == &plane.mode_guided &&
        guided_takeoff) {
        return true;
    }
    if (in_vtol_auto()) {
        if (!plane.auto_state.vtol_loiter || poscontrol.get_state() > QPOS_AIRBRAKE) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Check if in VTOL mode requiring position and velocity estimates
 * 
 * @details Determines if current VTOL mode needs active position and
 *          velocity estimation from EKF. Some VTOL modes (like QSTABILIZE)
 *          don't require position estimates, while others (QLOITER, QLAND)
 *          depend on accurate position/velocity for control.
 * 
 * @return true if mode needs position/velocity estimates, false otherwise
 * 
 * @note Used to determine if EKF position estimates must be valid
 * @note QSTABILIZE and QACRO don't require position estimates
 * 
 * @see in_vtol_mode() For general VTOL mode detection
 * 
 * Source: ArduPlane/quadplane.cpp:2517-2521
 */
bool QuadPlane::in_vtol_posvel_mode(void) const
{
    if (!available()) {
        return false;
    }
    return (plane.control_mode == &plane.mode_qloiter ||
            plane.control_mode == &plane.mode_qland ||
            plane.control_mode == &plane.mode_qrtl ||
#if QAUTOTUNE_ENABLED
            plane.control_mode == &plane.mode_qautotune ||
#endif
            (plane.control_mode->is_guided_mode() &&
            plane.auto_state.vtol_loiter &&
             poscontrol.get_state() > QPOS_APPROACH) ||
            in_vtol_auto());
}

/*
  update landing positioning offset
 */
/**
 * @brief Allow pilot to reposition aircraft during VTOL landing approach
 * 
 * @details Enables pilot to make small position corrections during QLAND or
 *          VTOL RTL approach using roll/pitch stick inputs. This allows final
 *          positioning adjustments for precise touchdown location.
 *          
 *          Requires OPTION::REPOSITION_LANDING enabled (Q_OPTIONS bit 18).
 *          
 *          Stick-to-velocity mapping:
 *          - Roll stick right → Move right (east in NED)
 *          - Pitch stick forward → Move forward (north in NED)
 *          - Full stick deflection (±4500) → ±1.0 normalized input
 *          - Maximum speed limited to 0.5 * waypoint acceleration
 *            (provides 0.5s stopping time constant for safety)
 *          
 *          Control flow:
 *          1. Check if REPOSITION_LANDING option enabled
 *          2. Read roll/pitch stick inputs, normalize to [-1, 1]
 *          3. Scale by max correction speed (0.5 * accel for 0.5s stop time)
 *          4. Rotate velocity from body frame to earth frame (yaw compensation)
 *          5. Integrate correction to xy_correction position offset
 *          6. Set pilot_correction_active flag if any stick input present
 *          7. Set pilot_correction_done flag on first correction (latching)
 *          
 *          Safety features:
 *          - Limited max speed prevents aggressive repositioning
 *          - Stopping time constant ensures controllable deceleration
 *          - Correction integrated over time (smooth position adjustment)
 *          - Zero stick input clears velocity but maintains correction
 * 
 * @note Only active when OPTION::REPOSITION_LANDING enabled
 * @note Maximum correction speed = 0.5 * Q_WP_ACCEL_CMSS
 * @note Velocity rotated by vehicle yaw (earth-frame correction)
 * @note pilot_correction_done flag latches true once correction made
 * @note Called at main loop rate during QLAND/VTOL RTL
 * 
 * @see Q_OPTIONS parameter bit 18 (REPOSITION_LANDING)
 * @see Q_WP_ACCEL parameter (typical 100-250 cm/s/s)
 * @see poscontrol.xy_correction - Accumulated position offset
 * @see poscontrol.pilot_correction_active - Current correction status
 * 
 * Source: ArduPlane/quadplane.cpp:3110-3133
 */
void QuadPlane::update_land_positioning(void)
{
    if (!option_is_set(QuadPlane::OPTION::REPOSITION_LANDING)) {
        // not enabled
        poscontrol.pilot_correction_active = false;
        poscontrol.target_vel_cms.zero();
        return;
    }
    const float scale = 1.0 / 4500;
    float roll_in = plane.channel_roll->get_control_in() * scale;
    float pitch_in = plane.channel_pitch->get_control_in() * scale;

    // limit correction speed to accel with stopping time constant of 0.5s
    const float speed_max_cms = wp_nav->get_wp_acceleration_cmss() * 0.5;
    const float dt = plane.scheduler.get_loop_period_s();

    poscontrol.target_vel_cms = Vector3f(-pitch_in, roll_in, 0) * speed_max_cms;
    poscontrol.target_vel_cms.rotate_xy(ahrs_view->yaw);

    // integrate our corrected position
    poscontrol.xy_correction += poscontrol.target_vel_cms.xy() * dt * 0.01;

    poscontrol.pilot_correction_active = (!is_zero(roll_in) || !is_zero(pitch_in));
    if (poscontrol.pilot_correction_active) {
        poscontrol.pilot_correction_done = true;
    }
}

/**
 * @brief Run horizontal position controller (North-East axis control)
 * 
 * @details Executes the AC_PosControl position and velocity controller for
 *          horizontal (North-East) axes. Initializes controller if not active,
 *          configures speed/acceleration limits, and updates control output.
 *          
 *          Configuration sequence:
 *          1. Determine acceleration limit:
 *             - Use waypoint acceleration from Q_WP_ACCEL
 *             - If accel_limit provided and positive, use MAX(wp_accel, accel_limit*100)
 *          2. Set maximum speed from Q_WP_SPEED_NE (default horizontal speed)
 *          3. Configure position controller max speed and acceleration
 *          4. Configure correction speed and acceleration (same as max)
 *          5. Initialize NE controller if not active
 *          6. Set lean angle limit:
 *             - Clamp to 45° maximum (4500 centidegrees)
 *             - Use MAX of accel-derived angle and Q_ANGLE_MAX
 *          7. Handle forward throttle saturation (anti-windup)
 *          8. Execute controller update
 *          
 *          Anti-windup protection:
 *          - If forward throttle > 95%, marks controller as externally limited
 *          - Prevents I-term windup when forward thrust saturated
 *          - Critical for transition flight and high-speed VTOL
 *          
 *          Acceleration to angle conversion:
 *          - Larger accelerations require larger lean angles
 *          - Controller respects both accel limit and angle limit
 *          - 45° hard limit for safety
 * 
 * @param[in] accel_limit Optional acceleration limit override in m/s/s
 *                        If positive, overrides minimum acceleration
 *                        If zero/negative, uses waypoint acceleration only
 * 
 * @note Called from position control modes (QLOITER, QLAND, AUTO, etc.)
 * @note Initializes controller if not previously active
 * @note 95% forward throttle triggers anti-windup protection
 * @note Lean angle clamped to 45° maximum for safety
 * @note Both max and correction speeds set identically
 * 
 * @see AC_PosControl::update_NE_controller()
 * @see Q_WP_SPEED_NE parameter (typical 500-1500 cm/s)
 * @see Q_WP_ACCEL parameter (typical 100-250 cm/s/s)
 * @see Q_ANGLE_MAX parameter (typical 30° = 3000 cd)
 * 
 * Source: ArduPlane/quadplane.cpp:3138-3156
 */
void QuadPlane::run_xy_controller(float accel_limit)
{
    float accel_cmss = wp_nav->get_wp_acceleration_cmss();
    if (is_positive(accel_limit)) {
        // allow for accel limit override
        accel_cmss = MAX(accel_cmss, accel_limit*100);
    }
    const float speed_cms = wp_nav->get_default_speed_NE_cms();
    pos_control->set_max_speed_accel_NE_cm(speed_cms, accel_cmss);
    pos_control->set_correction_speed_accel_NE_cm(speed_cms, accel_cmss);
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }
    pos_control->set_lean_angle_max_cd(MIN(4500, MAX(accel_mss_to_angle_deg(accel_limit)*100, aparm.angle_max)));
    if (q_fwd_throttle > 0.95f) {
        // prevent wind up of the velocity controller I term due to a saturated forward throttle
        pos_control->set_externally_limited_NE();
    }
    pos_control->update_NE_controller();
}

/**
 * @brief Initialize VTOL approach state machine
 * 
 * @details Determines initial position control state when transitioning to
 *          VTOL for landing or waypoint arrival. Selects between full approach
 *          sequence, airbrake, or direct position control based on distance
 *          and configuration.
 *          
 *          State selection logic:
 *          
 *          **1. Skip Approach (QPOS_POSITION1 direct):**
 *          Conditions:
 *          - Q_OPTIONS bit DISABLE_APPROACH is set, OR
 *          - Distance < Q_APPROACH_DIST (and Q_APPROACH_DIST > 0)
 *          Actions:
 *          - Set state to QPOS_POSITION1 (position tracking)
 *          - Send "VTOL Position1" telemetry message
 *          - Skip airbrake and approach phases
 *          
 *          **2. Close Proximity (< transition_threshold):**
 *          
 *          Tailsitter OR motors already spooled:
 *          - Set state to QPOS_POSITION1
 *          - transition->set_last_fw_pitch() (capture pitch for blend)
 *          - Send "VTOL Position1" message
 *          
 *          Normal multicopter:
 *          - Set state to QPOS_AIRBRAKE
 *          - Send telemetry with groundspeed, distance, stopping distance, height
 *          - Aggressively slow down before position control
 *          
 *          **3. Far Distance (>= transition_threshold):**
 *          - Set state to QPOS_APPROACH
 *          - Send "VTOL approach" message with distance
 *          - Begin full approach sequence from afar
 *          
 *          Transition threshold:
 *          - Default: stopping_distance() based on current groundspeed
 *          - Determines when to begin deceleration
 *          - Ensures adequate room for controlled stop
 * 
 * @note Called from guided_start(), control_auto(), waypoint_controller()
 * @note Distance comparison uses 2D horizontal distance (not 3D slant)
 * @note Tailsitters skip airbrake (immediate hover capability)
 * @note THROTTLE_UNLIMITED check detects if already in powered flight
 * @note Telemetry messages aid in flight log analysis
 * @note State machine progresses: APPROACH → AIRBRAKE → POSITION1 → POSITION2
 * 
 * @see poscontrol.set_state() - Position control state setter
 * @see QPOS_APPROACH, QPOS_AIRBRAKE, QPOS_POSITION1 - State definitions
 * @see transition_threshold() - Distance to begin transition
 * @see stopping_distance() - Required distance for safe deceleration
 * @see Q_APPROACH_DIST parameter - Minimum distance for full approach
 * @see Q_OPTIONS parameter - Behavior modification flags
 * 
 * Source: ArduPlane/quadplane.cpp:3254-3290
 */
void QuadPlane::poscontrol_init_approach(void)
{
    const float dist = plane.current_loc.get_distance(plane.next_WP_loc);
    if (option_is_set(QuadPlane::OPTION::DISABLE_APPROACH) ||
        (is_positive(approach_distance) && dist < approach_distance)) {
        // go straight to QPOS_POSITION1
        poscontrol.set_state(QPOS_POSITION1);
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL Position1 d=%.1f", dist);
    } else if (poscontrol.get_state() != QPOS_APPROACH) {
        // check if we are close to the destination. We don't want to
        // do a full approach when very close
        if (dist < transition_threshold()) {
            if (tailsitter.enabled() || motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL Position1 d=%.1f", dist);
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL airbrake v=%.1f d=%.0f sd=%.0f h=%.1f",
                                plane.ahrs.groundspeed(),
                                dist,
                                stopping_distance(),
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_AIRBRAKE);
            }
        } else {
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL approach d=%.1f", dist);
            poscontrol.set_state(QPOS_APPROACH);
        }
        poscontrol.thrust_loss_start_ms = 0;
    }
    poscontrol.pilot_correction_done = false;
    poscontrol.xy_correction.zero();
    poscontrol.slow_descent = false;
}

#if HAL_LOGGING_ENABLED
/*
  log the QPOS message
 */
void QuadPlane::log_QPOS(void)
{
// @LoggerMessage: QPOS
// @Description: Quadplane position data
// @Field: TimeUS: Time since system startup
// @Field: State: Position control state
// @FieldValueEnum: State: QuadPlane::position_control_state
// @Field: Dist: Distance to next waypoint
// @Field: TSpd: Target speed
// @Field: TAcc: Target acceleration
// @Field: OShoot: True if landing point is overshot or heading off by more than 60 degrees

    AP::logger().WriteStreaming("QPOS", "TimeUS,State,Dist,TSpd,TAcc,OShoot", "QBfffB",
                                AP_HAL::micros64(),
                                poscontrol.get_state(),
                                plane.auto_state.wp_distance,
                                poscontrol.target_speed,
                                poscontrol.target_accel,
                                poscontrol.overshoot);
}
#endif

/*
  change position control state
 */
void QuadPlane::PosControlState::set_state(enum position_control_state s)
{
    const uint32_t now = AP_HAL::millis();
    if (state != s) {
        auto &qp = plane.quadplane;
        pilot_correction_done = false;
        // handle resets needed for when the state changes
        if (s == QPOS_POSITION1) {
            reached_wp_speed = false;
            // never do a rate reset, if attitude control is not active it will be automatically reset before running, see: last_att_control_ms
            // if it is active then the rate control should not be reset at all
            qp.attitude_control->reset_yaw_target_and_rate(false);
            pos1_speed_limit = plane.ahrs.groundspeed_vector().length();
            done_accel_init = false;
        } else if (s == QPOS_AIRBRAKE) {
            // start with zero integrator on vertical throttle
            qp.pos_control->get_accel_U_pid().set_integrator(0);
        } else if (s == QPOS_LAND_DESCEND) {
            // reset throttle descent control
            qp.thr_ctrl_land = false;
            qp.land_descend_start_alt = plane.current_loc.alt*0.01;
            last_override_descent_ms = 0;
        } else if (s == QPOS_LAND_ABORT) {
            // reset throttle descent control
            qp.thr_ctrl_land = false;
        } else if (s == QPOS_LAND_FINAL) {
            // remember last pos reset to handle GPS glitch in LAND_FINAL
            Vector2f rpos;
            last_pos_reset_ms = plane.ahrs.getLastPosNorthEastReset(rpos);
            qp.landing_detect.land_start_ms = 0;
            qp.landing_detect.lower_limit_start_ms = 0;
        }
        // double log to capture the state change
#if HAL_LOGGING_ENABLED
        qp.log_QPOS();
#endif
        state = s;
#if HAL_LOGGING_ENABLED
        qp.log_QPOS();
#endif
        last_log_ms = now;
        overshoot = false;
    }
    last_state_change_ms = now;

    // we consider setting the state to be equivalent to running to
    // prevent code from overriding the state as stale
    last_run_ms = now;
}

/**
 * @brief Main VTOL position controller for landing and waypoint navigation
 * 
 * @details This is the primary position control state machine for QuadPlane
 *          VTOL operations. Controls complete landing sequence from fixed-wing
 *          flight through transition to final touchdown, plus waypoint navigation.
 *          
 *          **State Machine Overview:**
 *          ```
 *          APPROACH → AIRBRAKE → POSITION1 → POSITION2 → 
 *          LAND_DESCEND → LAND_FINAL → LAND_COMPLETE
 *          
 *          Special states: LAND_ABORT (emergency climb)
 *          ```
 *          
 *          **State Descriptions:**
 *          
 *          **QPOS_NONE:**
 *          - Uninitialized state (error condition)
 *          - Immediately transitions to QPOS_POSITION1
 *          - Logs internal error
 *          
 *          **QPOS_APPROACH:**
 *          - Fixed-wing flight toward waypoint/landing
 *          - Uses plane nav controller and TECS
 *          - Monitors distance and speed for airbrake entry
 *          - Checks for premature VTOL mode (forces POSITION1)
 *          - Transitions to AIRBRAKE at stopping_distance() + 2s margin
 *          - Tailsitters skip directly to POSITION1
 *          
 *          **QPOS_AIRBRAKE:**
 *          - Active deceleration using VTOL motors
 *          - Fixed-wing control surfaces + VTOL motors for drag
 *          - TECS controls pitch and throttle
 *          - Nav controller provides roll guidance
 *          - Prevents negative pitch (no dive in airbrake)
 *          - Tiltrotor special case: Low throttle stabilize if tilted forward
 *          - Transitions to POSITION1 when:
 *            * Speed < airspeed threshold (ARSPD_MIN-2 or Q_ASSIST_SPEED)
 *            * Distance < position2 threshold (10m) if slow enough
 *          - Emergency transition if position/velocity error too large
 *          
 *          **QPOS_POSITION1:**
 *          - Primary VTOL position hold and navigation
 *          - Full 3D position controller active
 *          - Weathervaning if enabled (yaw into wind)
 *          - Speed limiting during initial entry (prevents overshoot)
 *          - Acceleration initialization for smooth control
 *          - Transitions to POSITION2 when:
 *            * Distance < position2_dist_threshold (10m)
 *            * Speed < position2_target_speed (3 m/s)
 *          - Can transition to LAND_DESCEND if at descent location
 *          
 *          **QPOS_POSITION2:**
 *          - Final approach precision position control
 *          - Tighter tolerances than POSITION1
 *          - Full weathervaning and wind correction
 *          - Smooth XY velocity transitions
 *          - Loiter: Maintains position indefinitely
 *          - Landing: Transitions to LAND_DESCEND when ready
 *          
 *          **QPOS_LAND_DESCEND:**
 *          - Controlled descent to surface
 *          - Descent rate from landing_descent_rate_cms()
 *          - Pilot can override descent (climb stick input)
 *          - Horizontal position tracking continues
 *          - Transitions to LAND_FINAL at low altitude
 *          - GPS glitch handling (position reset detection)
 *          - Can transition to LAND_ABORT if emergency climb needed
 *          
 *          **QPOS_LAND_FINAL:**
 *          - Final touchdown phase
 *          - Very low altitude (below rangefinder threshold)
 *          - Reduced descent rate (gentle touchdown)
 *          - Land detector monitors for ground contact
 *          - GPS glitch protection (freezes horizontal position)
 *          - Transitions to LAND_COMPLETE when landed
 *          
 *          **QPOS_LAND_COMPLETE:**
 *          - Touchdown detected, disarmed or ready for disarm
 *          - Motors at ground idle or shut down
 *          - Position controller idle
 *          
 *          **QPOS_LAND_ABORT:**
 *          - Emergency climb from landing sequence
 *          - Triggered by pilot stick input or obstacle detection
 *          - Maximum climb rate applied
 *          - Exits landing sequence
 *          - Can return to POSITION1 or other modes
 *          
 *          **Key Features:**
 *          
 *          **Speed Management:**
 *          - Approach: Fixed-wing speeds
 *          - Airbrake: Active deceleration to VTOL speeds
 *          - Position1: Speed limiting during entry, full speed when stable
 *          - Position2: Low speed precision control (3 m/s target)
 *          
 *          **Wind Handling:**
 *          - Weathervaning in position modes (yaw into wind)
 *          - Forward motor thrust compensation
 *          - Velocity matching for moving landing targets
 *          
 *          **Safety Features:**
 *          - Land abort detection (pilot override)
 *          - GPS glitch handling (position reset detection)
 *          - Rangefinder failure protection
 *          - Emergency transition on large position errors
 *          - Tilt angle monitoring (tiltrotor safety)
 *          
 *          **Special Vehicle Handling:**
 *          - Tailsitters: Skip airbrake, different transitions
 *          - Tiltrotors: Tilt angle-based throttle suppression
 *          - Standard multicopters: Full state sequence
 *          
 *          **Altitude Control:**
 *          - Z controller runs unless suppressed
 *          - Suppression during airbrake with excessive tilt
 *          - Integration with hold_hover() for altitude hold
 *          - pos_control->land_at_climb_rate_cm() for descent
 *          
 *          **Logging and Telemetry:**
 *          - QPOS logging at 25 Hz
 *          - State change messages to GCS
 *          - Distance, speed, and altitude telemetry
 *          - Debug information for analysis
 * 
 * @note Called from mode_auto.cpp, mode_guided.cpp, mode_rtl.cpp, mode_qrtl.cpp
 * @note Runs at main loop rate during VTOL operations
 * @note State machine may skip states based on conditions (e.g., close proximity)
 * @note Z controller runs at end unless explicitly suppressed by state logic
 * @note Coordinates with transition code for fixed-wing ↔ VTOL switchover
 * @note Uses plane.next_WP_loc as target position throughout
 * 
 * @warning SAFETY-CRITICAL: Landing state machine controls touchdown
 * @warning State transitions must be carefully validated for all aircraft types
 * @warning GPS glitch during landing can cause position errors
 * @warning Rangefinder failures affect descent rate calculations
 * 
 * @see poscontrol.set_state() - State machine transitions
 * @see run_z_controller() - Altitude control
 * @see run_xy_controller() - Horizontal position control
 * @see landing_descent_rate_cms() - Descent rate calculation
 * @see land_detector() - Ground contact detection
 * @see stopping_distance() - Required deceleration distance
 * @see transition_threshold() - Transition initiation distance
 * @see weathervane_controller() - Yaw into wind control
 * 
 * Source: ArduPlane/quadplane.cpp:3425-3989 (564 lines - complex state machine)
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    const Location &loc = plane.next_WP_loc;
    uint32_t now_ms = AP_HAL::millis();

    // distance that we switch to QPOS_POSITION2
    const float position2_dist_threshold = 10.0;

    // target speed when we reach position2 threshold
    const float position2_target_speed = 3.0;

    if (plane.arming.is_armed_and_safety_off()) {
        poscontrol.last_run_ms = now_ms;
    }

    // avoid running the z controller in approach and airbrake if we're not already running it
    // and tilt is more than tilt max
    bool suppress_z_controller = false;

    Vector2f landing_velocity;
    if (now_ms - poscontrol.last_velocity_match_ms < 1000) {
        landing_velocity = poscontrol.velocity_match;
    }

    // horizontal position control
    switch (poscontrol.get_state()) {

    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
        if (in_vtol_mode()) {
            // this means we're not running transition update code and
            // thus not doing qassist checking, force POSITION1 mode
            // now. We don't expect this to trigger, it is a failsafe
            // for a logic error
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 nvtol");
            poscontrol.set_state(QPOS_POSITION1);
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        FALLTHROUGH;

    case QPOS_AIRBRAKE: {
        float aspeed;
        const Vector2f closing_vel = landing_closing_velocity();
        const Vector2f desired_closing_vel = landing_desired_closing_velocity();
        const float groundspeed = plane.ahrs.groundspeed();
        const float distance = plane.auto_state.wp_distance;
        const float closing_speed = closing_vel.length();
        const float desired_closing_speed = desired_closing_vel.length();
        if (!plane.ahrs.airspeed_estimate(aspeed)) {
            aspeed = groundspeed;
        }

        if (tiltrotor.enabled() && poscontrol.get_state() == QPOS_AIRBRAKE) {
            if ((now_ms - last_pidz_active_ms > 2000 && tiltrotor.tilt_over_max_angle()) ||
                tiltrotor.current_tilt >= tiltrotor.get_fully_forward_tilt()) {
                // use low throttle stabilization when airbraking on a
                // tiltrotor. We don't want quite zero throttle as we
                // want some drag, but don't want to run the Z
                // controller which can result in high throttle on
                // motors that are tilted forward, thus increasing
                // speed
                suppress_z_controller = true;
                hold_stabilize(0.01);
            }
        }
        
        // speed for crossover to POSITION1 controller
        const float aspeed_threshold = MAX(plane.aparm.airspeed_min-2, assist.speed);

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.auto_state.crosstrack ? plane.prev_WP_loc : plane.current_loc, loc);

        // use TECS for throttle
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.TECS_controller.get_throttle_demand());

        // use TECS for pitch
        int32_t commanded_pitch = plane.TECS_controller.get_pitch_demand();
        plane.nav_pitch_cd = constrain_int32(commanded_pitch, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
        if (poscontrol.get_state() == QPOS_AIRBRAKE) {
            // don't allow down pitch in airbrake
            plane.nav_pitch_cd = MAX(plane.nav_pitch_cd, 0);
        }

        // use nav controller roll
        plane.calc_nav_roll();

        // work out the point to enter airbrake mode. We want enough
        // distance to stop, plus some margin for the time it takes to
        // change the accel (jerk limit) plus the min time in airbrake
        // mode. For simplicity we assume 2 seconds margin
        const float stop_distance = stopping_distance() + 2*closing_speed;

        if (!suppress_z_controller && poscontrol.get_state() == QPOS_AIRBRAKE) {
            hold_hover(0);
            // don't run Z controller again in this loop
            suppress_z_controller = true;
        }

        /*
          see if we should start airbraking stage. For non-tailsitters
          we can use the VTOL motors as airbrakes by firing them up
          before we transition. This gives a smoother transition and
          gives us a nice lot of deceleration
         */
        if (poscontrol.get_state() == QPOS_APPROACH && distance < stop_distance) {
            if (tailsitter.enabled() || motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                // tailsitters don't use airbrake stage for landing
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed,
                                plane.auto_state.wp_distance,
                                stop_distance,
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL airbrake v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed,
                                distance,
                                stop_distance,
                                plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
                poscontrol.set_state(QPOS_AIRBRAKE);
            }
        }

        /*
          we must switch to POSITION1 if our airspeed drops below the
          assist speed. We additionally switch to POSITION1 if we are
          too far above our desired velocity profile, or our attitude
          has deviated too much
         */
        const int32_t attitude_error_threshold_cd = 1000;

        // use at least 1s of airbrake time to ensure motors have a chance to
        // properly spin up
        const uint32_t min_airbrake_ms = 1000;
        if (poscontrol.get_state() == QPOS_AIRBRAKE &&
            poscontrol.time_since_state_start_ms() > min_airbrake_ms &&
            (aspeed < aspeed_threshold || // too low airspeed
             fabsf(degrees(closing_vel.angle(desired_closing_vel))) > 60 || // wrong direction
             closing_speed > MAX(desired_closing_speed*1.2, desired_closing_speed+2) || // too fast
             closing_speed < desired_closing_speed*0.5 || // too slow ground speed
             labs(plane.ahrs.roll_sensor - plane.nav_roll_cd) > attitude_error_threshold_cd || // bad attitude
             labs(plane.ahrs.pitch_sensor - plane.nav_pitch_cd) > attitude_error_threshold_cd)) {
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.1f h=%.1f dc=%.1f",
                            (double)groundspeed,
                            (double)plane.auto_state.wp_distance,
                            plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING),
                            desired_closing_speed);
            poscontrol.set_state(QPOS_POSITION1);
            transition->set_last_fw_pitch();

            // switch to vfwd for throttle control
            vel_forward.integrator = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            // adjust the initial forward throttle based on our desired and actual closing speed
            // this allows for significant initial forward throttle
            // when we have a strong headwind, but low throttle in the usual case where
            // we want to slow down ready for POSITION2
            vel_forward.integrator = linear_interpolate(0, vel_forward.integrator,
                                                        closing_speed,
                                                        1.2*desired_closing_speed, 0.5*desired_closing_speed);

            // limit our initial forward throttle in POSITION1 to be 0.5 of cruise throttle
            vel_forward.integrator = constrain_float(vel_forward.integrator, 0, plane.aparm.throttle_cruise*0.5);
            
            vel_forward.last_ms = now_ms;
        }

        if (!tiltrotor.enabled() && !tailsitter.enabled()) {
            /*
              cope with fwd motor thrust loss during approach. We detect
              this by looking for the fwd throttle saturating. This only
              applies to separate lift-thrust vehicles
            */
            bool throttle_saturated = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) >= plane.aparm.throttle_max;
            if (throttle_saturated &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
                plane.auto_state.sink_rate > 0.2 && aspeed < aspeed_threshold+4) {
                if (poscontrol.thrust_loss_start_ms == 0) {
                    poscontrol.thrust_loss_start_ms = now_ms;
                }
                if (now_ms - poscontrol.thrust_loss_start_ms > 5000) {
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 thrust loss as=%.1f at=%.1f",
                                    aspeed, aspeed_threshold);
                    poscontrol.set_state(QPOS_POSITION1);
                    transition->set_last_fw_pitch();
                }
            } else {
                poscontrol.thrust_loss_start_ms = 0;
            }

            // handle loss of forward thrust in approach based on low airspeed detection
            if (poscontrol.get_state() == QPOS_APPROACH && aspeed < aspeed_threshold &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 low speed as=%.1f at=%.1f",
                                aspeed, aspeed_threshold);
                poscontrol.set_state(QPOS_POSITION1);
                transition->set_last_fw_pitch();
            }
        }

        if (poscontrol.get_state() == QPOS_APPROACH) {
            poscontrol_init_approach();
        }
        break;
    }

    case QPOS_POSITION1: {
        setup_target_position();

        if (tailsitter.enabled() && tailsitter.in_vtol_transition(now_ms)) {
            break;
        }

        const Vector2f diff_wp = plane.current_loc.get_distance_NE(loc);
        const float distance = diff_wp.length();
        const Vector2f rel_groundspeed_vector = landing_closing_velocity();
        const float rel_groundspeed_sq = rel_groundspeed_vector.length_squared();
        float closing_groundspeed = 0;

        if (distance > 0.1) {
            closing_groundspeed = rel_groundspeed_vector * diff_wp.normalized();
        }

        // calculate speed we should be at to reach the position2
        // target speed at the position2 distance threshold, assuming
        // Q_TRANS_DECEL is correct
        const float stopping_speed = safe_sqrt(MAX(0, distance-position2_dist_threshold) * 2 * transition_decel + sq(position2_target_speed));

        float target_speed = stopping_speed;

        // maximum configured VTOL speed
        const float wp_speed = MAX(1.0, wp_nav->get_default_speed_NE_cms() * 0.01);
        const float scaled_wp_speed = get_scaled_wp_speed(degrees(diff_wp.angle()));

        // limit target speed to a the pos1 speed limit, which starts out at the initial speed
        // but is adjusted if we start putting our nose down. We always allow at least twice
        // the WP speed
        target_speed = MIN(MAX(poscontrol.pos1_speed_limit, 2*wp_speed), target_speed);

        if (poscontrol.reached_wp_speed ||
            rel_groundspeed_sq < sq(wp_speed) ||
            wp_speed > 1.35*scaled_wp_speed) {
            // once we get below the Q_WP_SPEED then we don't want to
            // speed up again. At that point we should fly within the
            // limits of the configured VTOL controller we also apply
            // this limit when we are more than 45 degrees off the
            // target in yaw, which is when we start to become
            // unstable
            target_speed = MIN(target_speed, scaled_wp_speed);
            poscontrol.reached_wp_speed = true;
        }

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        Vector2f target_speed_xy_cms;
        Vector2f target_accel_cms;
        bool have_target_yaw = false;
        float target_yaw_deg;
        const float target_accel = MIN(accel_needed(distance, sq(closing_groundspeed)), transition_decel*2);
        if (distance > 0.1) {
            Vector2f diff_wp_norm = diff_wp.normalized();
            target_speed_xy_cms = diff_wp_norm * target_speed * 100;
            target_accel_cms = diff_wp_norm * (-target_accel*100);
            target_yaw_deg = degrees(diff_wp_norm.angle());
            const float yaw_err_deg = wrap_180(target_yaw_deg - degrees(plane.ahrs.get_yaw_rad()));
            bool overshoot = (closing_groundspeed < 0 || fabsf(yaw_err_deg) > 60);
            if (overshoot && !poscontrol.overshoot) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL Overshoot d=%.1f cs=%.1f yerr=%.1f",
                                distance, closing_groundspeed, yaw_err_deg);
                poscontrol.overshoot = true;
                pos_control->set_accel_desired_NE_cmss(Vector2f());
            }
            if (poscontrol.overshoot) {
                /* we have overshot the landing point or our nose is
                   off by more than 60 degrees. Zero target accel and
                   point nose at the landing point. Set target speed
                   to our position2 threshold speed
                */
                target_accel_cms.zero();

                // allow up to the WP speed when we are further away, slowing to the pos2 target speed
                // when we are close
                target_speed = linear_interpolate(position2_target_speed, wp_speed,
                                                  distance,
                                                  position2_dist_threshold*1.5,
                                                  2*position2_dist_threshold + stopping_distance(rel_groundspeed_sq));

                target_speed_xy_cms = diff_wp_norm * target_speed * 100;
                have_target_yaw = true;

                // adjust target yaw angle for wind. We calculate yaw based on the target speed
                // we want assuming no speed scaling due to direction
                const Vector2f wind = plane.ahrs.wind_estimate().xy();
                const float gnd_speed = plane.ahrs.groundspeed();
                Vector2f target_speed_xy = landing_velocity + diff_wp_norm * gnd_speed - wind;
                target_yaw_deg = degrees(target_speed_xy.angle());
            }
        }
        const float target_speed_ms = target_speed_xy_cms.length() * 0.01;

        target_speed_xy_cms += landing_velocity * 100;
        poscontrol.target_speed = target_speed_ms;
        poscontrol.target_accel = target_accel;

        if (!poscontrol.reached_wp_speed &&
            rel_groundspeed_sq < sq(target_speed_ms) &&
            rel_groundspeed_sq > sq(2*wp_speed) &&
            plane.nav_pitch_cd < 0) {
            // we have slowed down more than expected, likely due to
            // drag from the props and we're starting to put our nose
            // down as a result. We want to accept the slowdown and
            // re-calculate the target speed profile
            poscontrol.pos1_speed_limit = sqrtf(rel_groundspeed_sq);
        }

        // use input shaping and abide by accel and jerk limits
        pos_control->input_vel_accel_NE_cm(target_speed_xy_cms, target_accel_cms);

        // run horizontal velocity controller
        run_xy_controller(MAX(target_accel, transition_decel)*1.5);

        if (!poscontrol.done_accel_init) {
            /*
              the pos controller init assumes zero accel, we need to
              override that so that we can start decelerating more
              quickly at the start of POSITION1
             */
            poscontrol.done_accel_init = true;
            pos_control->set_accel_desired_NE_cmss(target_accel_cms);
        }
        
        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
            pos_control->set_externally_limited_NE();
        }

        // call attitude controller
        disable_yaw_rate_time_constant();

        // setup scaling of roll and pitch angle P gains to match fixed wing gains
        setup_rp_fw_angle_gains();

        if (have_target_yaw) {
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd,
                                                               plane.nav_pitch_cd,
                                                               target_yaw_deg*100, true);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        }
        if ((plane.auto_state.wp_distance < position2_dist_threshold) && tiltrotor.tilt_angle_achieved() &&
            fabsf(rel_groundspeed_sq) < sq(3*position2_target_speed)) {
            // if continuous tiltrotor only advance to position 2 once tilts have finished moving
            poscontrol.set_state(QPOS_POSITION2);
            poscontrol.pilot_correction_done = false;
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f h=%.1f",
                            (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance,
                            plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING));
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_ABORT:
    case QPOS_LAND_DESCEND: {
        setup_target_position();
        /*
          for final land repositioning and descent we run the position controller
         */
        Vector2f zero;
        Vector2f vel_cms = poscontrol.target_vel_cms.xy() + landing_velocity*100;
        pos_control->input_pos_vel_accel_NE_cm(poscontrol.target_cm.xy(), vel_cms, zero);

        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        update_land_positioning();

        run_xy_controller(transition_decel*1.5);

        // nav roll and pitch are controlled by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
            pos_control->set_externally_limited_NE();
        }

        // call attitude controller
        set_pilot_yaw_rate_time_constant();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;
    }

    case QPOS_LAND_FINAL:
        update_land_positioning();

        // relax when close to the ground
        if (should_relax()) {
            pos_control->relax_velocity_controller_NE();
        } else {
            Vector2f zero;
            Vector2f vel_cms = poscontrol.target_vel_cms.xy() + landing_velocity*100;
            Vector2f rpos;
            const uint32_t last_reset_ms = plane.ahrs.getLastPosNorthEastReset(rpos);
            /* we use velocity control when we may be touching the
              ground or if we've had a position reset from AHRS. This
              helps us handle a GPS glitch in the final land phase,
              and also prevents trying to reposition after touchdown
            */
            if (motors->limit.throttle_lower ||
                motors->get_throttle() < 0.5*motors->get_throttle_hover() ||
                last_reset_ms != poscontrol.last_pos_reset_ms) {
                pos_control->input_vel_accel_NE_cm(vel_cms, zero);
            } else {
                // otherwise use full pos control
                pos_control->input_pos_vel_accel_NE_cm(poscontrol.target_cm.xy(), vel_cms, zero);
            }
        }

        run_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();

        // call attitude controller
        set_pilot_yaw_rate_time_constant();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;

    case QPOS_LAND_COMPLETE:
        // nothing to do
        break;
    }

    // now height control
    switch (poscontrol.get_state()) {
    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
    case QPOS_AIRBRAKE:
        // we just want stability from the VTOL controller in these
        // phases of landing, so relax the Z controller, unless we are
        // providing assistance
        if (transition->complete()) {
            pos_control->relax_U_controller(0);
        }
        break;
    case QPOS_POSITION1:
        if (tailsitter.in_vtol_transition(now_ms)) {
            pos_control->relax_U_controller(0);
            break;
        }
        FALLTHROUGH;
    case QPOS_POSITION2: {
        bool vtol_loiter_auto = false;
        if (plane.control_mode == &plane.mode_auto) {
            switch (plane.mission.get_current_nav_cmd().id) {
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_LOITER_TIME:
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_LOITER_TO_ALT:
                vtol_loiter_auto = true;
                break;
            }
        }
        if (plane.control_mode == &plane.mode_guided || vtol_loiter_auto) {
            plane.ahrs.get_location(plane.current_loc);
            int32_t target_altitude_cm;
            if (!plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,target_altitude_cm)) {
                break;
            }
            if (poscontrol.slow_descent &&
                plane.prev_WP_loc.get_distance(plane.next_WP_loc) > 50) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = plane.current_loc.line_path_proportion(plane.prev_WP_loc, plane.next_WP_loc);
                int32_t prev_alt;
                if (plane.prev_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,prev_alt)) {
                    target_altitude_cm = linear_interpolate(prev_alt,
                                                         target_altitude_cm,
                                                         plane.auto_state.wp_proportion,
                                                         0, 1);
                }
            }
            float zero = 0;
            float target_z = target_altitude_cm;
            pos_control->input_pos_vel_accel_U_cm(target_z, zero, 0);
        } else if (plane.control_mode == &plane.mode_qrtl) {
            Location loc2 = loc;
            loc2.change_alt_frame(Location::AltFrame::ABOVE_ORIGIN);
            float target_z = loc2.alt;
            float zero = 0;
            pos_control->input_pos_vel_accel_U_cm(target_z, zero, 0);
        } else {
            set_climb_rate_cms(0);
        }
        break;
    }

    case QPOS_LAND_DESCEND:
    case QPOS_LAND_ABORT:
    case QPOS_LAND_FINAL: {
        float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        if (poscontrol.get_state() == QPOS_LAND_FINAL) {
            if (!option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
                ahrs.set_touchdown_expected(true);
            }
        }
        if (poscontrol.get_state() == QPOS_LAND_ABORT) {
            set_climb_rate_cms(wp_nav->get_default_speed_up_cms());
            break;
        }
        const float descent_rate_cms = landing_descent_rate_cms(height_above_ground);
        pos_control->land_at_climb_rate_cm(-descent_rate_cms, descent_rate_cms>0);
        break;
    }

    case QPOS_LAND_COMPLETE:
        break;
    }

    /*
      run the z controller unless something has already run it or set a target throttle
     */
    if (!suppress_z_controller) {
        // otherwise run z controller
        run_z_controller();
    }

#if HAL_LOGGING_ENABLED
    if (now_ms - poscontrol.last_log_ms >= 40) {
        // log poscontrol at 25Hz
        poscontrol.last_log_ms = now_ms;
        log_QPOS();
    }
#endif
}

/**
 * @brief Determine which forward throttle control method is active
 * 
 * @details Selects between three forward throttle control approaches based
 *          on configuration parameters, flight state, and special conditions.
 *          Forward throttle uses a pusher/puller motor to provide forward
 *          thrust during VTOL flight, improving efficiency and control.
 *          
 *          **Method Selection Priority (highest to lowest):**
 *          
 *          **1. Disabled Conditions (return NONE):**
 *          - Internal combustion engine not running (if ICE enabled)
 *          - QAUTOTUNE mode active (needs pure multicopter control)
 *          - No forward throttle gain parameters configured
 *          
 *          **2. NEW Method (return ActiveFwdThr::NEW):**
 *          Prerequisites: Q_FWD_THR_GAIN > 0
 *          Activated when:
 *          - AUX function switch enabled vfwd_enable_active, OR
 *          - Q_FWD_THR_USE == ALL (always use new method), OR
 *          - Q_FWD_THR_USE == POSCTRL AND position controller active
 *          
 *          NEW method characteristics:
 *          - Forward throttle replaces forward pitch command
 *          - Direct throttle control (not velocity-based)
 *          - Calculated by position controller
 *          - Recommended for new aircraft
 *          
 *          **3. OLD Method (return ActiveFwdThr::OLD):**
 *          Prerequisites: Q_VFWD_GAIN > 0 AND position controller active
 *          Activated when:
 *          - NEW method not active
 *          - Legacy velocity-based controller enabled
 *          
 *          OLD method characteristics:
 *          - PI controller tracking forward velocity
 *          - Runs at 10 Hz update rate
 *          - Legacy system (deprecated)
 *          - Maintained for backward compatibility
 *          
 *          **4. No Forward Throttle (return NONE):**
 *          - No parameters configured
 *          - Position controller not active
 *          - Special modes that disable forward throttle
 *          
 *          **Special Conditions:**
 *          
 *          ICE (Internal Combustion Engine):
 *          - Forward motor may be ICE-powered
 *          - Requires engine running state
 *          - Disabled if engine not running
 *          
 *          QAUTOTUNE mode:
 *          - Requires pure multicopter response
 *          - Disables forward throttle for tuning accuracy
 *          - Prevents forward motor from affecting PID tuning
 *          
 *          AUX function override:
 *          - Pilot can enable NEW method via switch
 *          - Overrides Q_FWD_THR_USE parameter
 *          - Allows in-flight testing
 * 
 * @return ActiveFwdThr::NONE if forward throttle disabled
 * @return ActiveFwdThr::NEW if using new position-based method
 * @return ActiveFwdThr::OLD if using legacy velocity-based method
 * 
 * @note NEW method preferred over OLD for all new aircraft
 * @note Method selection can change in flight based on conditions
 * @note Called from forward_throttle_pct() to determine throttle calculation
 * @note Position controller state affects method availability
 * 
 * @see Q_FWD_THR_GAIN parameter - NEW method gain (recommended)
 * @see Q_VFWD_GAIN parameter - OLD method gain (legacy)
 * @see Q_FWD_THR_USE parameter - When to use NEW method (0/1/2)
 * @see forward_throttle_pct() - Calculates actual throttle value
 * @see pos_control->is_active_NE() - Position controller state
 * 
 * Source: ArduPlane/quadplane.cpp:4136-4173
 */
QuadPlane::ActiveFwdThr QuadPlane::get_vfwd_method(void) const
{
    const bool have_fwd_thr_gain = is_positive(q_fwd_thr_gain);
    const bool have_vfwd_gain = is_positive(vel_forward.gain);

#if AP_ICENGINE_ENABLED
    const auto ice_state = plane.g2.ice_control.get_state();
    if (ice_state != AP_ICEngine::ICE_DISABLED && ice_state != AP_ICEngine::ICE_RUNNING) {
        // we need the engine running for fwd throttle
        return ActiveFwdThr::NONE;
    }
#endif

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        return ActiveFwdThr::NONE;
    }
#endif

    if (have_fwd_thr_gain) {
        if (vfwd_enable_active) {
            // user has used AUX function to activate new method
            return ActiveFwdThr::NEW;
        }
        if (q_fwd_thr_use == FwdThrUse::ALL) {
            return ActiveFwdThr::NEW;
        }
        if (q_fwd_thr_use == FwdThrUse::POSCTRL && pos_control->is_active_NE()) {
            return ActiveFwdThr::NEW;
        }
    }
    if (have_vfwd_gain && pos_control->is_active_NE()) {
        return ActiveFwdThr::OLD;
    }
    return ActiveFwdThr::NONE;
}

/*
  map from pitch tilt to fwd throttle when enabled
 */
/**
 * @brief Calculate forward throttle and pitch limits for hybrid thrust systems
 * 
 * @details Manages forward throttle output and pitch angle limits for quadplanes
 *          with forward thrust motors or tilt-rotors during VTOL flight. Prevents
 *          wing overload during high-speed braking, prevents prop strikes near
 *          ground, and adapts pitch limits based on motor health and airspeed.
 *          
 *          **Forward Thrust System Types:**
 *          - Forward thrust motors: Separate fixed motors providing forward thrust
 *          - Tilt-rotors: VTOL motors that tilt forward for cruise flight
 *          - Traditional tilt-wings: Full wing tilts for transition
 *          
 *          **Algorithm Overview:**
 *          
 *          **1. Forward Thrust Method Check:**
 *          - get_vfwd_method() - Determine if forward thrust is active
 *          - Returns: NEW method (modern forward thrust system) or other
 *          
 *          If NOT NEW method:
 *          - q_fwd_throttle = 0.0 (no forward thrust)
 *          - q_fwd_pitch_lim_cd = 100 * q_fwd_pitch_lim (base limit)
 *          - Return early (feature disabled)
 *          
 *          **2. Forward Throttle Calculation:**
 *          Purpose: Convert desired pitch acceleration to forward motor thrust
 *          
 *          Pitch angle extraction:
 *          - fwd_tilt_rad = radians(constrain(-0.01 * nav_pitch_cd, 0°, 45°))
 *          - nav_pitch_cd: Desired navigation pitch (centidegrees)
 *          - Constrain to 0-45 degrees (forward pitch only)
 *          - Negative sign: Nose down pitch is negative, thrust is positive
 *          
 *          Throttle mapping:
 *          - q_fwd_throttle = MIN(q_fwd_thr_gain * tan(fwd_tilt_rad), 1.0)
 *          - q_fwd_thr_gain: Tuning parameter for thrust response
 *          - tan(angle): Converts tilt to horizontal thrust requirement
 *          - Capped at 1.0 (100% throttle maximum)
 *          
 *          Rationale:
 *          - Quadplane limits forward pitch to prevent negative wing lift
 *          - Forward motor provides pitch-limited horizontal acceleration
 *          - tan(angle) gives proper thrust-to-pitch relationship
 *          
 *          **3. Forward Pitch Limit Adaptation (Non-Tiltrotor Only):**
 *          Purpose: Detect forward motor failure and relax pitch limit
 *          
 *          Condition: !tiltrotor.enabled()
 *          - Only for separate forward thrust motors
 *          - Tilt-rotors always rely on forward thrust, no failure mode
 *          
 *          Pitch limit range calculation:
 *          - fwd_tilt_range_cd = angle_max - (100 * q_fwd_pitch_lim)
 *          - Range from conservative limit to full lean angle
 *          
 *          If positive range available:
 *          
 *          **Motor Health Detection:**
 *          - fwd_limited = pos_control active AND forward pitch limited
 *          - Position controller saturating forward = motor may be failed
 *          
 *          Target selection:
 *          - If motor suspected failed: fwd_pitch_lim_cd_tgt = angle_max (full lean)
 *          - If motor healthy: fwd_pitch_lim_cd_tgt = 100 * q_fwd_pitch_lim (conservative)
 *          
 *          **Rate Limiting (10-second slew):**
 *          - delta_max = 0.1 * fwd_tilt_range_cd * G_Dt
 *          - Slew takes 10 seconds to transition between limits
 *          - Prevents abrupt limit changes
 *          
 *          - q_fwd_pitch_lim_cd += constrain(target - current, -delta_max, +delta_max)
 *          - Gradual adaptation to motor health
 *          
 *          **Clamp to Demanded Pitch:**
 *          - q_fwd_pitch_lim_cd = MIN(limit, MAX(-nav_pitch_cd, base_limit))
 *          - Don't open limit more than current pitch demand
 *          - Prevents unnecessary exposure to high pitch angles
 *          
 *          **4. High-Speed Braking Protection:**
 *          Purpose: Prevent wing overload during rapid VTOL deceleration
 *          
 *          Condition: q_bck_pitch_lim > 0 (back transition limit enabled)
 *          
 *          **Dynamic Pitch Limit Calculation:**
 *          - reference_speed = MAX(airspeed_min, MIN_AIRSPEED_MIN)
 *          - speed_scaler = (reference_speed / current_airspeed)²
 *          - nav_pitch_upper_limit_cd = q_bck_pitch_lim * speed_scaler
 *          - Capped at angle_max
 *          
 *          Scaling rationale:
 *          - Wing lift proportional to velocity²
 *          - High speed = more lift = less pitch needed for deceleration
 *          - Low speed = less lift = more pitch allowed
 *          - Square law matches aerodynamic forces
 *          
 *          **Low-Pass Filtering (0.5s time constant):**
 *          - tconst = 0.5 seconds
 *          - dt = elapsed time since last update
 *          - coef = dt / (dt + tconst) - First-order filter coefficient
 *          - q_bck_pitch_lim_cd = (1 - coef) * old + coef * new
 *          - Smooth transitions prevent jerky limit changes
 *          
 *          **Apply Upper Pitch Limit:**
 *          - plane.nav_pitch_cd = MIN(nav_pitch_cd, q_bck_pitch_lim_cd)
 *          - Prevent excessive pitch during high-speed braking
 *          
 *          **QBRK Logging:**
 *          - SpdScaler: Speed-based scaling factor
 *          - NPULCD: Navigation pitch upper limit
 *          - QBPLCD: Quadplane back transition pitch limit
 *          - NPCD: Actual demanded navigation pitch
 *          
 *          **5. Ground Proximity Throttle Reduction:**
 *          Purpose: Prevent forward motor prop strike near ground
 *          
 *          Normal operation (!in_vtol_land_approach):
 *          - alt_cutoff = MAX(0, vel_forward_alt_cutoff)
 *          - height_above_ground = relative_ground_altitude(TAKEOFF_LANDING)
 *          - fwd_thr_scaler = linear_interpolate(0.0, 1.0, height, cutoff, cutoff+2)
 *          
 *          Scaling behavior:
 *          - Below alt_cutoff: fwd_thr_scaler = 0.0 (no forward thrust)
 *          - Between cutoff and cutoff+2m: Linear ramp 0.0 to 1.0
 *          - Above cutoff+2m: fwd_thr_scaler = 1.0 (full forward thrust)
 *          
 *          VTOL land approach exception:
 *          - fwd_thr_scaler = 1.0 (always allow forward thrust)
 *          - Ensures horizontal position control capability
 *          - Prevents bad height estimate from blocking approach
 *          
 *          Apply scaler:
 *          - q_fwd_throttle *= fwd_thr_scaler
 *          - Reduces thrust near ground to prevent prop strike
 *          
 *          **6. Lower Pitch Limit Relaxation:**
 *          Purpose: Maintain forward acceleration when throttle reduced
 *          
 *          Calculation:
 *          - nav_pitch_lower_limit_cd = -(angle_max * (1 - scaler) + q_fwd_pitch_lim_cd * scaler)
 *          
 *          Behavior:
 *          - fwd_thr_scaler = 1.0: Lower limit = -q_fwd_pitch_lim_cd (conservative)
 *          - fwd_thr_scaler = 0.0: Lower limit = -angle_max (full lean allowed)
 *          - Interpolates between based on throttle availability
 *          
 *          Rationale:
 *          - Forward throttle reduced = need more pitch for acceleration
 *          - Allows pitch compensation when thrust unavailable
 *          - Maintains forward capability near ground
 *          
 *          **FWDT Logging:**
 *          - fts: Forward throttle scaler
 *          - qfplcd: Quadplane forward pitch limit
 *          - npllcd: Navigation pitch lower limit
 *          - npcd: Demanded navigation pitch
 *          - qft: Quadplane forward throttle
 *          - npulcd: Navigation pitch upper limit
 *          
 *          **7. Apply Lower Pitch Limit:**
 *          - plane.nav_pitch_cd = MAX(nav_pitch_cd, nav_pitch_lower_limit_cd)
 *          - Prevent excessive nose-up pitch
 *          - Ensures forward thrust system effectiveness
 * 
 * @note Called during VTOL flight modes with forward thrust active
 * @note Rate-limited pitch adaptation: 10-second slew time
 * @note Braking protection: Dynamic limit based on airspeed squared
 * @note Ground proximity: 2-meter ramp zone for prop strike prevention
 * @note Tilt-rotor systems do not adapt pitch limits (always rely on tilt)
 * @note Forward motor failure detection via position controller saturation
 * @note Updates q_fwd_throttle (0.0-1.0 range)
 * @note Updates q_fwd_pitch_lim_cd (pitch limit in centidegrees)
 * @note Updates q_bck_pitch_lim_cd (back transition limit in centidegrees)
 * @note Modifies plane.nav_pitch_cd (bounded to computed limits)
 * 
 * @warning SAFETY-CRITICAL: Controls pitch limits and forward thrust
 * @warning Improper limits can cause wing overload or loss of control
 * @warning Prop strike prevention essential for forward motor safety
 * @warning High-speed braking limits prevent structural damage
 * @warning Motor failure detection allows continued control authority
 * 
 * @see get_vfwd_method() - Determine forward thrust method
 * @see in_vtol_land_approach() - Check if in landing approach
 * @see relative_ground_altitude() - Get height above ground
 * @see q_fwd_thr_gain parameter - Forward throttle gain
 * @see q_fwd_pitch_lim parameter - Forward pitch limit (degrees)
 * @see q_bck_pitch_lim parameter - Back transition pitch limit (degrees)
 * @see vel_forward_alt_cutoff parameter - Ground proximity cutoff altitude
 * @see aparm.angle_max - Maximum lean angle in VTOL modes
 * @see tiltrotor.enabled() - Check if tilt-rotor configured
 * 
 * Source: ArduPlane/quadplane.cpp:4257-4369
 */
void QuadPlane::assign_tilt_to_fwd_thr(void)
{

    const auto fwd_thr_active = get_vfwd_method();
    if (fwd_thr_active != ActiveFwdThr::NEW) {
        q_fwd_throttle = 0.0f;
        q_fwd_pitch_lim_cd = 100.0f * q_fwd_pitch_lim;
        return;
    }
    // Handle the case where we are limiting the forward pitch angle to prevent negative wing lift
    // and are using the forward thrust motor or tilting rotors to provide the forward acceleration
    float fwd_tilt_rad = radians(constrain_float(-0.01f * (float)plane.nav_pitch_cd, 0.0f, 45.0f));
    q_fwd_throttle = MIN(q_fwd_thr_gain * tanf(fwd_tilt_rad), 1.0f);

    // Relax forward tilt limit if the position controller is saturating in the forward direction because
    // the forward thrust motor could be failed. Do not do this with tilt rotors because they do not rely on
    // forward throttle during VTOL flight
    if (!tiltrotor.enabled()) {
        const float fwd_tilt_range_cd = (float)aparm.angle_max - 100.0f * q_fwd_pitch_lim;
        if (is_positive(fwd_tilt_range_cd)) {
            // rate limit the forward tilt change to slew between the motor good and motor failed
            // value over 10 seconds
            const bool fwd_limited = plane.quadplane.pos_control->is_active_NE() and plane.quadplane.pos_control->get_fwd_pitch_is_limited();
            const float fwd_pitch_lim_cd_tgt = fwd_limited ? (float)aparm.angle_max : 100.0f * q_fwd_pitch_lim;
            const float delta_max = 0.1f * fwd_tilt_range_cd * plane.G_Dt;
            q_fwd_pitch_lim_cd += constrain_float((fwd_pitch_lim_cd_tgt - q_fwd_pitch_lim_cd), -delta_max, delta_max);
            // Don't let the forward pitch limit be more than the forward pitch demand before limiting to
            // avoid opening up the limit more than necessary
            q_fwd_pitch_lim_cd = MIN(q_fwd_pitch_lim_cd, MAX(-(float)plane.nav_pitch_cd, 100.0f * q_fwd_pitch_lim));
        } else {
            // take the lesser of the two limits
            q_fwd_pitch_lim_cd = (float)aparm.angle_max;
        }
    }

    // Prevent the wing from being overloaded when braking from high speed in a VTOL mode
    float nav_pitch_upper_limit_cd = 100.0f * q_bck_pitch_lim;
    float aspeed;
    if (is_positive(q_bck_pitch_lim) && ahrs.airspeed_estimate(aspeed)) {
        const float reference_speed = MAX(plane.aparm.airspeed_min, MIN_AIRSPEED_MIN);
        float speed_scaler = sq(reference_speed / MAX(aspeed, 0.1f));
        nav_pitch_upper_limit_cd *= speed_scaler;
        nav_pitch_upper_limit_cd = MIN(nav_pitch_upper_limit_cd, (float)aparm.angle_max);

        const float tconst = 0.5f;
        const float dt = AP_HAL::millis() - q_pitch_limit_update_ms;
        q_pitch_limit_update_ms = AP_HAL::millis();
        if (is_positive(dt)) {
            const float coef = dt / (dt + tconst);
            q_bck_pitch_lim_cd = (1.0f - coef) * q_bck_pitch_lim_cd + coef * nav_pitch_upper_limit_cd;
        }

        plane.nav_pitch_cd = MIN(plane.nav_pitch_cd, (int32_t)q_bck_pitch_lim_cd);

#if HAL_LOGGING_ENABLED
        // @LoggerMessage: QBRK
        // @Description: Quadplane Braking
        // @Field: TimeUS: Time since system startup
        // @Field: SpdScaler: braking speed scaler
        // @Field: NPULCD: upper limit for navigation pitch
        // @Field: QBPLCD: upper limit for back transition pitch
        // @Field: NPCD: demanded navigation pitch
        AP::logger().WriteStreaming("QBRK",
                                "TimeUS,SpdScaler,NPULCD,QBPLCD,NPCD",  // labels
                                "Qffii",    // fmt
                                AP_HAL::micros64(),
                                (double)speed_scaler,
                                (double)nav_pitch_upper_limit_cd,
                                (int32_t)q_bck_pitch_lim_cd,
                                (int32_t)plane.nav_pitch_cd);
#endif
    }

    float fwd_thr_scaler;
    if (!in_vtol_land_approach()) {
        // To prevent forward motor prop strike, reduce throttle to zero when close to ground.
        float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
        float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        fwd_thr_scaler = linear_interpolate(0.0f, 1.0f, height_above_ground, alt_cutoff, alt_cutoff+2);
    } else {
        // When we are doing horizontal positioning in a VTOL land we always allow the fwd motor
        // to run. Otherwise a bad height above landing point estimate could cause the aircraft
        // not to be able to approach the landing point
        fwd_thr_scaler = 1.0f;
    }
    q_fwd_throttle *= fwd_thr_scaler;

    // When reducing forward throttle use, relax lower pitch limit to maintain forward
    // acceleration capability.
    const float nav_pitch_lower_limit_cd = - (int32_t)((float)aparm.angle_max * (1.0f - fwd_thr_scaler) + q_fwd_pitch_lim_cd * fwd_thr_scaler);

#if HAL_LOGGING_ENABLED
    // Diagnostics logging - remove when feature is fully flight tested.
    // @LoggerMessage: FWDT
    // @Description: Forward Throttle calculations
    // @Field: TimeUS: Time since system startup
    // @Field: fts: forward throttle scaler
    // @Field: qfplcd: quadplane forward pitch limit
    // @Field: npllcd: navigation pitch lower limit
    // @Field: npcd: demanded navigation pitch
    // @Field: qft: quadplane forward throttle
    // @Field: npulcd: upper limit for navigation pitch
    AP::logger().WriteStreaming("FWDT",
                                "TimeUS,fts,qfplcd,npllcd,npcd,qft,npulcd",  // labels
                                "Qffffff",    // fmt
                                AP_HAL::micros64(),
                                (double)fwd_thr_scaler,
                                (double)q_fwd_pitch_lim_cd,
                                (double)nav_pitch_lower_limit_cd,
                                (double)plane.nav_pitch_cd,
                                (double)q_fwd_throttle,
                                (float)nav_pitch_upper_limit_cd);
#endif

    plane.nav_pitch_cd = MAX(plane.nav_pitch_cd, (int32_t)nav_pitch_lower_limit_cd);
}

/*
  we want to limit WP speed to a lower speed when more than 20 degrees
  off pointing at the destination. quadplanes are often
  unstable when flying sideways or backwards
*/
float QuadPlane::get_scaled_wp_speed(float target_bearing_deg) const
{
    const float yaw_difference = fabsf(wrap_180(degrees(plane.ahrs.get_yaw_rad()) - target_bearing_deg));
    const float wp_speed = wp_nav->get_default_speed_NE_cms() * 0.01;
    if (yaw_difference > 20) {
        // this gives a factor of 2x reduction in max speed when
        // off by 90 degrees, and 3x when off by 180 degrees
        const float speed_reduction = linear_interpolate(1, 3,
                                                         yaw_difference,
                                                         20, 160);
        return wp_speed / speed_reduction;
    }
    return wp_speed;
}

/*
  setup the target position based on plane.next_WP_loc
 */
/**
 * @brief Set position controller target from waypoint location
 * 
 * @details Converts plane.next_WP_loc from global GPS coordinates to local
 *          North-East-Up (NEU) frame in centimeters for position controller.
 *          Also configures vertical velocity and acceleration limits.
 *          
 *          Coordinate transformation:
 *          1. Get EKF origin (home location in global frame)
 *          2. Calculate NE offset from origin to target waypoint (meters)
 *          3. Apply XY correction offset (poscontrol.xy_correction)
 *          4. Convert to centimeters for position controller
 *          5. Calculate Up (altitude) as difference from origin altitude
 *          
 *          Motor spool state management:
 *          - Not in land approach: Set THROTTLE_UNLIMITED (full power available)
 *          - In land approach but past QPOS_APPROACH: Set THROTTLE_UNLIMITED
 *          - In land approach during QPOS_APPROACH: Leave spool state unchanged
 *          
 *          Vertical limits configuration:
 *          - Max speed down: get_pilot_velocity_z_max_dn() (Q_PILOT_SPD_DN)
 *          - Max speed up: pilot_speed_z_max_up * 100 cm/s (Q_PILOT_SPD_UP)
 *          - Acceleration: pilot_accel_z * 100 cm/s/s (Q_PILOT_ACCEL_Z)
 *          - Applied to both speed/accel and correction speed/accel
 *          
 *          XY correction offset:
 *          - Accumulated position corrections from external sources
 *          - Allows offset from nominal waypoint
 *          - Used for precision landing adjustments
 *          
 *          Coordinate system:
 *          - Input: GPS latitude/longitude/altitude (global)
 *          - Output: NEU frame centimeters (local, relative to origin)
 *          - North: poscontrol.target_cm.x
 *          - East: poscontrol.target_cm.y
 *          - Up: poscontrol.target_cm.z (altitude above origin)
 * 
 * @note Called before every position controller update
 * @note Origin must be set by EKF (after GPS lock)
 * @note If origin unavailable, uses zero origin (emergency fallback)
 * @note Target updates even if position hasn't changed (for corrections)
 * @note Vertical limits affect all climb/descent operations
 * 
 * @see plane.next_WP_loc - Target waypoint location
 * @see poscontrol.target_cm - Output target in NEU frame (cm)
 * @see poscontrol.xy_correction - External position offset
 * @see pos_control->set_max_speed_accel_U_cm() - Vertical limits
 * @see Q_PILOT_SPD_UP, Q_PILOT_SPD_DN, Q_PILOT_ACCEL_Z parameters
 * 
 * Source: ArduPlane/quadplane.cpp:4397-4417
 */
void QuadPlane::setup_target_position(void)
{
    const Location &loc = plane.next_WP_loc;
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    if (!in_vtol_land_approach() || poscontrol.get_state() > QPOS_APPROACH) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    Vector2f diff2d = origin.get_distance_NE(loc);
    diff2d += poscontrol.xy_correction;
    poscontrol.target_cm.x = diff2d.x * 100;
    poscontrol.target_cm.y = diff2d.y * 100;
    poscontrol.target_cm.z = plane.next_WP_loc.alt - origin.alt;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);
}

/**
 * @brief Vertical takeoff controller for VTOL ascent
 * 
 * @details Controls vertical VTOL takeoff from ground to target altitude.
 *          Manages motor spool-up, navigation enabling, position hold, and
 *          climb rate. Handles special cases for rudder arming, tiltrotor
 *          positioning, and navigation suppression during early climb.
 *          
 *          **Takeoff Sequence:**
 *          
 *          **1. Precondition Checks:**
 *          - Reset fixed-wing nav outputs (roll_cd, pitch_cd = 0)
 *          - Return if not armed and safety off
 *          - Check for tiltrotor tilt-up completion (guided mode)
 *          - Wait for rudder neutral after rudder arming
 *          
 *          **2. Rudder Arming Safety:**
 *          - If armed via rudder AND rudder not neutral:
 *            * Keep motors at GROUND_IDLE (visible armed indication)
 *            * Do not climb until rudder released
 *            * Send warning message every TAKEOFF_RUDDER_WARNING_TIMEOUT
 *            * Reset takeoff_start_time_ms (restart timer)
 *            * Prevents accidental takeoff with stick input
 *          
 *          **3. Tiltrotor Wait:**
 *          - Guided mode + guided_takeoff + tiltrotor enabled:
 *            * Wait for tilts to reach fully up position
 *            * Do not climb until tilt complete
 *            * Reset takeoff_start_time_ms while waiting
 *            * Required for proper tiltrotor takeoff attitude
 *          
 *          **4. Position Controller Setup:**
 *          - setup_target_position() - Convert waypoint to NEU frame
 *          - Configure target position from plane.next_WP_loc
 *          - Set vertical speed/accel limits
 *          
 *          **5. Velocity Matching (if recent):**
 *          - If velocity_match updated within 1 second:
 *            * Use velocity_match vector for XY velocity target
 *            * Supports moving platform takeoffs
 *            * Convert from m/s to cm/s (*100)
 *          - Otherwise: Zero XY velocity target
 *          
 *          **6. Navigation Suppression (Q_NAVALT_MIN):**
 *          - If Q_NAVALT_MIN > 0 AND altitude gain < Q_NAVALT_MIN:
 *            * Disable horizontal navigation (no_navigation = true)
 *            * Relax NE velocity controller (zero stick response)
 *            * Helps with poor GPS velocity data on ground
 *            * Altitude measured from takeoff_start_alt
 *            * Start altitude captured on first run or after 1s gap
 *          - Once above threshold: Enable full navigation
 *          
 *          **7. Horizontal Control:**
 *          Navigation suppressed:
 *          - relax_velocity_controller_NE() - Zero horizontal control
 *          - Aircraft drifts with initial conditions
 *          
 *          Navigation active:
 *          - input_vel_accel_NE_cm() - Track velocity target
 *          - Get nav roll/pitch from position controller
 *          - assign_tilt_to_fwd_thr() - Forward motor if configured
 *          - run_xy_controller() - Update horizontal position control
 *          
 *          **8. Attitude Control:**
 *          - set_pilot_yaw_rate_time_constant() - Yaw rate filtering
 *          - input_euler_angle_roll_pitch_euler_rate_yaw_cd():
 *            * Roll angle from position controller (or 0)
 *            * Pitch angle from position controller (or 0)
 *            * Yaw rate = pilot input + weathervane
 *          
 *          **9. Vertical Control:**
 *          Guided takeoff mode:
 *          - Target specific altitude with zero velocity at top
 *          - pos_control->input_pos_vel_accel_U_cm(pos_z, 0, 0)
 *          - pos_z = target altitude + 5cm margin
 *          - Smooth stop at target altitude
 *          
 *          Normal takeoff:
 *          - Constant climb rate = wp_nav->get_default_speed_up_cms()
 *          - set_climb_rate_cms(vel_z)
 *          - Climbs until mode change or altitude reached
 *          
 *          **10. Execute Climb:**
 *          - run_z_controller() - Execute vertical climb control
 * 
 * @note Called from mode_qhover, mode_qloiter, mode_guided during takeoff
 * @note Runs at main loop rate during takeoff sequence
 * @note takeoff_last_run_ms tracks continuity (resets if >1s gap)
 * @note Fixed-wing nav outputs zeroed (pure VTOL control)
 * @note Weathervaning active during takeoff (yaw into wind)
 * 
 * @see setup_target_position() - Configure position target
 * @see run_xy_controller() - Horizontal position control
 * @see run_z_controller() - Vertical climb control
 * @see get_pilot_input_yaw_rate_cds() - Pilot yaw input
 * @see get_weathervane_yaw_rate_cds() - Wind heading correction
 * @see Q_NAVALT_MIN parameter - Navigation enable altitude
 * @see poscontrol.velocity_match - Moving platform velocity
 * 
 * Source: ArduPlane/quadplane.cpp:4422-4528
 */
void QuadPlane::takeoff_controller(void)
{
    // reset fixed wing controller to neutral as base output
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    if (!plane.arming.is_armed_and_safety_off()) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    const auto spool_state = motors->get_desired_spool_state();
    if (plane.control_mode == &plane.mode_guided && guided_takeoff
        && tiltrotor.enabled() && !tiltrotor.fully_up() &&
        spool_state != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // waiting for motors to tilt up
        takeoff_start_time_ms = now;
        return;
    }

    // don't takeoff up until rudder is re-centered after rudder arming
    if (plane.arming.last_arm_method() == AP_Arming::Method::RUDDER &&
        (takeoff_last_run_ms == 0 ||
         now - takeoff_last_run_ms > 1000) &&
        !rc().seen_neutral_rudder() &&
        spool_state <= AP_Motors::DesiredSpoolState::GROUND_IDLE) {
        // start motor spinning if not spinning already so user sees it is armed
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        takeoff_start_time_ms = now;
        if (now - plane.takeoff_state.rudder_takeoff_warn_ms > TAKEOFF_RUDDER_WARNING_TIMEOUT) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff waiting for rudder release");
            plane.takeoff_state.rudder_takeoff_warn_ms = now;
        }
        return;
    }


    /*
      for takeoff we use the position controller
    */
    setup_target_position();

    // set position control target and update

    Vector2f vel, zero;
    if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
        vel = poscontrol.velocity_match * 100;
    }

    /*
      support zeroing roll/pitch during early part of takeoff. This
      can help particularly with poor GPS velocity data
     */
    bool no_navigation = false;
    if (takeoff_navalt_min > 0) {
        const float alt = plane.current_loc.alt*0.01;
        if (takeoff_last_run_ms == 0 ||
            now - takeoff_last_run_ms > 1000) {
            takeoff_start_alt = alt;
        }
        if (alt - takeoff_start_alt < takeoff_navalt_min) {
            no_navigation = true;
        }
    }
    takeoff_last_run_ms = now;

    if (no_navigation) {
        pos_control->relax_velocity_controller_NE();
    } else {
        pos_control->input_vel_accel_NE_cm(vel, zero);

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        assign_tilt_to_fwd_thr();
    }

    run_xy_controller();

    set_pilot_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());

    float vel_z = wp_nav->get_default_speed_up_cms();
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        // for guided takeoff we aim for a specific height with zero
        // velocity at that height
        Location origin;
        if (ahrs.get_origin(origin)) {
            // a small margin to ensure we do move to the next takeoff
            // stage
            const int32_t margin_cm = 5;
            float pos_z = margin_cm + plane.next_WP_loc.alt - origin.alt;
            vel_z = 0;
            pos_control->input_pos_vel_accel_U_cm(pos_z, vel_z, 0);
        } else {
            set_climb_rate_cms(vel_z);
        }
    } else {
        set_climb_rate_cms(vel_z);
    }

    run_z_controller();
}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
/**
 * @brief VTOL waypoint navigation controller for auto missions
 * 
 * @details Full multicopter-style waypoint navigation for AUTO mode when
 *          in VTOL flight. Uses AC_WPNav library for smooth trajectory
 *          generation and tracking. Manages horizontal and vertical control
 *          for precise 3D waypoint following.
 *          
 *          **Control Flow:**
 *          
 *          **1. Position Target Setup:**
 *          - setup_target_position() converts plane.next_WP_loc to NEU frame
 *          - Waypoint in centimeters relative to EKF origin
 *          
 *          **2. Waypoint Update Detection:**
 *          Check if waypoint changed:
 *          - Different location than last_auto_target, OR
 *          - More than 500ms since last update
 *          
 *          If waypoint changed:
 *          - wp_nav->set_wp_destination_NEU_cm() - Set new target
 *          - Update last_auto_target cache
 *          - Triggers trajectory recalculation
 *          
 *          Update last_loiter_ms timestamp (tracks update frequency)
 *          
 *          **3. Trajectory Controller:**
 *          - wp_nav->update_wpnav() - Main waypoint nav update
 *          - Generates smooth S-curve trajectory
 *          - Calculates velocity and acceleration profiles
 *          - Computes desired roll/pitch for path tracking
 *          - Outputs: nav_roll_cd, nav_pitch_cd, yaw angle
 *          
 *          **4. Attitude Command Extraction:**
 *          - plane.nav_roll_cd = wp_nav->get_roll()
 *          - plane.nav_pitch_cd = wp_nav->get_pitch()
 *          - These drive fixed-wing nav calculations (for display/logging)
 *          
 *          **5. Forward Motor Assignment (Tiltrotor/Tailsitter):**
 *          - assign_tilt_to_fwd_thr() - Compute forward thrust component
 *          - Distributes thrust between vertical and forward motors
 *          - Based on tilt angles or tailsitter configuration
 *          
 *          **6. Transition Roll/Pitch Limiting:**
 *          - transition->set_VTOL_roll_pitch_limit() - Apply angle limits
 *          - Constrains roll/pitch during transition states
 *          - If limited: pos_control->set_externally_limited_NE()
 *          - Informs position controller of external constraints
 *          
 *          **7. Attitude Control:**
 *          - disable_yaw_rate_time_constant() - Direct yaw control
 *          - attitude_control->input_euler_angle_roll_pitch_yaw_cd():
 *            * Roll: From waypoint controller
 *            * Pitch: From waypoint controller
 *            * Yaw: From waypoint controller (heading to target)
 *            * true flag: Enables feedforward for smooth tracking
 *          
 *          **8. Vertical Control:**
 *          - set_climb_rate_cms(assist_climb_rate_cms())
 *          - assist_climb_rate_cms() computes climb rate from altitude error
 *          - run_z_controller() - Execute altitude control
 *          
 *          **Key Features:**
 *          - Smooth S-curve trajectories (no abrupt velocity changes)
 *          - Continuous waypoint tracking (updates at main loop rate)
 *          - Coordinate with transition state (angle limiting)
 *          - Altitude control separate from horizontal navigation
 *          - Caching prevents unnecessary trajectory recalculation
 *          - 500ms update timeout ensures fresh commands
 * 
 * @note Called from control_auto() when in VTOL auto mode
 * @note Runs at main loop rate during waypoint navigation
 * @note Different from vtol_position_controller (which handles landing)
 * @note wp_nav library handles all trajectory smoothing
 * @note Altitude controlled by assist_climb_rate_cms() + Z controller
 * @note Yaw points toward next waypoint (not weathervaning)
 * 
 * @see setup_target_position() - Convert waypoint to NEU frame
 * @see wp_nav->update_wpnav() - Waypoint trajectory controller
 * @see assign_tilt_to_fwd_thr() - Forward motor thrust distribution
 * @see assist_climb_rate_cms() - Compute climb rate from altitude error
 * @see run_z_controller() - Execute altitude control
 * @see AC_WPNav library - Waypoint navigation implementation
 * 
 * Source: ArduPlane/quadplane.cpp:4681-4723
 */
void QuadPlane::waypoint_controller(void)
{
    setup_target_position();

    const Location &loc = plane.next_WP_loc;
    const uint32_t now = AP_HAL::millis();
    if (!loc.same_loc_as(last_auto_target) ||
        now - last_loiter_ms > 500) {
        wp_nav->set_wp_destination_NEU_cm(poscontrol.target_cm.tofloat());
        last_auto_target = loc;
    }
    last_loiter_ms = now;

    /*
      this is full copter control of auto flight
    */
    // run wpnav controller
    wp_nav->update_wpnav();

    // nav roll and pitch are controller by waypoint controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    assign_tilt_to_fwd_thr();

    if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_NE();
    }

    // call attitude controller
    disable_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(plane.nav_roll_cd,
                                                       plane.nav_pitch_cd,
                                                       wp_nav->get_yaw(),
                                                       true);

    // climb based on altitude error
    set_climb_rate_cms(assist_climb_rate_cms());
    run_z_controller();
}


/**
 * @brief Main AUTO mode controller when in VTOL flight
 * 
 * @details Top-level controller for AUTO mode when auto_state.vtol_mode is true.
 *          Dispatches to appropriate sub-controller based on current mission
 *          command (takeoff, landing, loiter, or waypoint). Manages motor spool
 *          state and handles special cases like arming delays and payload operations.
 *          
 *          **Control Flow:**
 *          
 *          **1. Setup Check:**
 *          - Call setup() to initialize VTOL systems
 *          - Return immediately if setup fails
 *          - Ensures motors, controllers, and sensors ready
 *          
 *          **2. Motor Spool State Management:**
 *          Conditions for motor control:
 *          - Only if poscontrol.get_state() > QPOS_APPROACH (in VTOL flight)
 *          - Default: should_run_motors = false
 *          
 *          Motor suppression cases:
 *          - Arming delay active (plane.arming.get_delay_arming())
 *          - Payload place wait state:
 *            * motors->get_desired_spool_state() == SHUT_DOWN
 *            * Currently in MAV_CMD_NAV_PAYLOAD_PLACE mission
 *            * poscontrol state == QPOS_LAND_COMPLETE
 *            * Prevents motors spinning during payload operations
 *          
 *          If not suppressed:
 *          - set_desired_spool_state(THROTTLE_UNLIMITED)
 *          - Enables full motor control for VTOL flight
 *          
 *          **3. Mission Command Dispatch:**
 *          Get current navigation command: plane.mission.get_current_nav_cmd().id
 *          
 *          **Takeoff Commands:**
 *          - MAV_CMD_NAV_VTOL_TAKEOFF
 *          - MAV_CMD_NAV_TAKEOFF (if is_vtol_takeoff())
 *          Controller: takeoff_controller()
 *          - Vertical VTOL ascent
 *          - Position hold during climb
 *          
 *          **Landing Commands:**
 *          - MAV_CMD_NAV_VTOL_LAND
 *          - MAV_CMD_NAV_PAYLOAD_PLACE
 *          - MAV_CMD_NAV_LAND (if is_vtol_land())
 *          Controller: vtol_position_controller()
 *          - Full landing state machine
 *          - Approach → Airbrake → Position → Descend → Final → Complete
 *          
 *          **Loiter Commands:**
 *          - MAV_CMD_NAV_LOITER_UNLIM (unlimited loiter)
 *          - MAV_CMD_NAV_LOITER_TIME (timed loiter)
 *          - MAV_CMD_NAV_LOITER_TURNS (turn-based loiter)
 *          - MAV_CMD_NAV_LOITER_TO_ALT (loiter to altitude)
 *          
 *          Loiter logic:
 *          - Check if poscontrol.last_run_ms > 100ms ago
 *          - If stale: Reset to QPOS_POSITION1 (ensure clean state)
 *          - Controller: vtol_position_controller()
 *          - Holds position at waypoint location
 *          
 *          **Waypoint Commands (default case):**
 *          - All other navigation commands
 *          - MAV_CMD_NAV_WAYPOINT
 *          - Mission spline waypoints
 *          - Other navigation commands
 *          Controller: waypoint_controller()
 *          - Smooth trajectory following
 *          - S-curve speed profiles
 *          - Full 3D position control
 *          
 *          **Special Considerations:**
 *          
 *          **Payload Place:**
 *          - Motors shut down after touchdown
 *          - Prevents prop wash affecting payload
 *          - Vehicle remains armed but motors idle
 *          - Must complete manually or via timeout
 *          
 *          **Loiter Reset:**
 *          - 100ms timeout ensures fresh position control
 *          - Prevents stale state from previous operations
 *          - QPOS_POSITION1 = normal position tracking state
 *          
 *          **Takeoff/Landing Detection:**
 *          - is_vtol_takeoff() checks if takeoff should be VTOL
 *          - is_vtol_land() checks if landing should be VTOL
 *          - Supports mixed-mode AUTO missions (FW + VTOL segments)
 * 
 * @note Called from mode_auto.cpp at main loop rate
 * @note Only active when auto_state.vtol_mode == true
 * @note Switches between fixed-wing and VTOL controllers automatically
 * @note Each sub-controller handles its own attitude and throttle
 * @note Position control state persists across controller switches
 * 
 * @see takeoff_controller() - VTOL takeoff control
 * @see vtol_position_controller() - Landing and loiter control
 * @see waypoint_controller() - Waypoint navigation
 * @see setup() - VTOL system initialization
 * @see is_vtol_takeoff() - Determine if takeoff is VTOL
 * @see is_vtol_land() - Determine if landing is VTOL
 * 
 * Source: ArduPlane/quadplane.cpp:4778-4857
 */
void QuadPlane::control_auto(void)
{
    if (!setup()) {
        return;
    }

    if (poscontrol.get_state() > QPOS_APPROACH) {
        bool should_run_motors = false;

        // don't run the motors if in an arming delay
        if (plane.arming.get_delay_arming()) {
            should_run_motors = false;
        }

        // don't run motors if we are in the wait state for payload place
        if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::SHUT_DOWN &&
            plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
            poscontrol.get_state() == QPOS_LAND_COMPLETE) {
            should_run_motors = false;
        }
        
        if (should_run_motors) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
    }

    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        if (is_vtol_takeoff(id)) {
            takeoff_controller();
        }
        break;
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
    case MAV_CMD_NAV_LAND:
        if (is_vtol_land(id)) {
            vtol_position_controller();
        }
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT: {
        const uint32_t now = AP_HAL::millis();
        if (now - poscontrol.last_run_ms > 100) {
            // ensure that poscontrol is reset
            poscontrol.set_state(QPOS_POSITION1);
        }
        vtol_position_controller();
    }
        break;
    default:
        waypoint_controller();
        break;
    }
}

/**
 * @brief Initialize VTOL takeoff from mission command
 * 
 * @details Processes MAV_CMD_NAV_VTOL_TAKEOFF mission command and configures
 *          quadplane for vertical takeoff. Sets target altitude, initializes
 *          position controllers, and calculates takeoff timeout based on
 *          climb performance.
 *          
 *          **Takeoff Configuration:**
 *          
 *          **1. System Initialization:**
 *          - Call setup() to initialize VTOL systems
 *          - Return false if setup fails (not ready)
 *          
 *          **2. Position Setup:**
 *          XY Position:
 *          - Always uses current location for horizontal position
 *          - Sets loc.lat = 0, loc.lng = 0 (local takeoff)
 *          - plane.set_next_WP(loc) configures target
 *          
 *          Z Position (altitude):
 *          
 *          **Option A: RESPECT_TAKEOFF_FRAME enabled (Q_OPTIONS bit):**
 *          - Respects mission altitude frame (RELATIVE, ABSOLUTE, etc.)
 *          - change_alt_frame(ABSOLUTE) converts to absolute altitude
 *          - If already above target altitude: Return false (no takeoff)
 *          - Allows precise altitude control in missions
 *          
 *          **Option B: Normal mode (default):**
 *          - Altitude is RELATIVE to current position
 *          - next_WP_loc.alt = current_loc.alt + cmd.content.location.alt
 *          - Frame set to ABSOLUTE for internal processing
 *          - Simple relative climb from current height
 *          
 *          **3. Throttle Configuration:**
 *          - throttle_wait = false (disable throttle wait state)
 *          - Allows immediate motor spool-up
 *          
 *          **4. Vertical Controller Limits:**
 *          - Max speed down: get_pilot_velocity_z_max_dn() (cm/s)
 *          - Max speed up: pilot_speed_z_max_up * 100 cm/s
 *          - Acceleration: pilot_accel_z * 100 cm/s/s
 *          - Applied to position controller and correction controller
 *          
 *          **5. Position Controller Initialization:**
 *          - pos_control->init_U_controller() - Reset Z controller integrators
 *          - Ensures clean start (no accumulated errors)
 *          
 *          **6. Navigation Controller Update:**
 *          - plane.nav_controller->update_waypoint()
 *          - For status display and logging (fixed-wing nav system)
 *          - Shows takeoff waypoint in ground station
 *          
 *          **7. Takeoff Time Calculation:**
 *          Physics-based time estimate for takeoff completion:
 *          
 *          Variables:
 *          - d_total: Total climb distance (meters)
 *          - accel_m_s_s: Vertical acceleration (Q_PILOT_ACCEL_Z)
 *          - vel_max: Maximum climb rate (Q_PILOT_SPD_UP)
 *          - vel_z: Current vertical velocity (from inertial nav)
 *          
 *          Calculations:
 *          - t_accel = (vel_max - vel_z) / accel_m_s_s
 *          - d_accel = vel_z * t_accel + 0.5 * accel_m_s_s * t_accel²
 *          - d_remaining = d_total - d_accel
 *          - t_constant = d_remaining / vel_max
 *          - travel_time = max(t_accel, 0) + max(t_constant, 0)
 *          
 *          Phases:
 *          1. Acceleration phase: Accelerate from current speed to max
 *          2. Constant velocity phase: Climb at max speed to target
 *          
 *          **8. Takeoff Timeout Configuration:**
 *          - takeoff_start_time_ms = millis() (capture start time)
 *          - takeoff_time_limit_ms = travel_time * takeoff_failure_scalar * 1000
 *          - Minimum timeout: 5000ms (5 seconds)
 *          - takeoff_failure_scalar provides safety margin (typically 2x)
 *          - Prevents infinite climb attempts if something fails
 * 
 * @param[in] cmd Mission command with takeoff parameters
 * 
 * @return true if takeoff initialized successfully
 * @return false if setup failed or already above target altitude
 * 
 * @note Called from mode_auto.cpp when processing mission commands
 * @note Takeoff occurs at current XY position (no horizontal movement)
 * @note Timeout scaled by takeoff_failure_scalar for safety margin
 * @note Minimum 0.1 m/s and 0.1 m/s² used to prevent divide-by-zero
 * @note Physics equations assume constant acceleration to max velocity
 * 
 * @see takeoff_controller() - Executes the vertical climb
 * @see Q_PILOT_SPD_UP parameter - Maximum climb rate
 * @see Q_PILOT_ACCEL_Z parameter - Vertical acceleration
 * @see Q_OPTIONS parameter - RESPECT_TAKEOFF_FRAME flag
 * @see takeoff_failure_scalar parameter - Timeout safety factor
 * 
 * Source: ArduPlane/quadplane.cpp:4976-5038
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    // we always use the current location in XY for takeoff. The altitude defaults
    // to relative to current height, but if Q_OPTIONS is set to respect takeoff frame
    // then it will use normal frame handling for height
    Location loc = cmd.content.location;
    loc.lat = 0;
    loc.lng = 0;
    plane.set_next_WP(loc);
    if (option_is_set(QuadPlane::OPTION::RESPECT_TAKEOFF_FRAME)) {
        // convert to absolute frame for takeoff
        if (!plane.next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE) ||
            plane.current_loc.alt >= plane.next_WP_loc.alt) {
            // we are above the takeoff already, no need to do anything
            return false;
        }
    } else {
        plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + cmd.content.location.alt,
                                     Location::AltFrame::ABSOLUTE);
    }
    throttle_wait = false;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_velocity_z_max_dn(), pilot_speed_z_max_up*100, pilot_accel_z*100);

    // initialise the vertical position controller
    pos_control->init_U_controller();

    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.current_loc, plane.next_WP_loc);

    // calculate the time required to complete a takeoff
    // this may be conservative and accept extra time due to clamping
    // derived from the following latex equations if you want a nicely formatted view
    // t_{accel} = \frac{V_max - V_z}{a}
    // d_{accel} = V_z*t_{accel} + \frac{1}{2}*a*t_{accel}^2
    // d_{remaining} = d_{total} - d_{accel}
    // t_{constant} = \frac{d_{remaining}}{V_z}
    // t = max(t_{accel}, 0) + max(t_{constant}, 0)
    const float d_total = (plane.next_WP_loc.alt - plane.current_loc.alt) * 0.01f;
    const float accel_m_s_s = MAX(0.1, pilot_accel_z);
    const float vel_max = MAX(0.1, pilot_speed_z_max_up);
    const float vel_z = inertial_nav.get_velocity_z_up_cms() * 0.01f;
    const float t_accel = (vel_max - vel_z) / accel_m_s_s;
    const float d_accel = vel_z * t_accel + 0.5f * accel_m_s_s * sq(t_accel);
    const float d_remaining = d_total - d_accel;
    const float t_constant = d_remaining / vel_max;
    const float travel_time = MAX(t_accel, 0) + MAX(t_constant, 0);

    // setup the takeoff failure handling code
    takeoff_start_time_ms = millis();
    takeoff_time_limit_ms = MAX(travel_time * takeoff_failure_scalar * 1000, 5000); // minimum time 5 seconds

    return true;
}


/**
 * @brief Initialize VTOL landing from mission command
 * 
 * @details Processes MAV_CMD_NAV_VTOL_LAND or MAV_CMD_NAV_PAYLOAD_PLACE mission
 *          command and configures quadplane for controlled VTOL descent and
 *          landing. Sets target location, initializes position controllers,
 *          and prepares landing state machine.
 *          
 *          **Landing Configuration:**
 *          
 *          **1. System Initialization:**
 *          - Call setup() to initialize VTOL systems
 *          - Return false if setup fails (not ready)
 *          
 *          **2. Target Position Setup:**
 *          Horizontal (XY):
 *          - plane.set_next_WP(cmd.content.location)
 *          - Uses latitude/longitude from mission command
 *          - Sets final touchdown location
 *          
 *          Vertical (Z):
 *          - Initially set to current altitude (plane.current_loc.alt)
 *          - Frame: ABSOLUTE
 *          - Landing controller will manage final descent
 *          - Starting at current altitude allows approach phase
 *          
 *          Rationale for current altitude:
 *          - Allows controlled approach from current position
 *          - Landing state machine handles descent sequencing
 *          - QPOS_APPROACH → AIRBRAKE → POSITION → DESCEND → FINAL
 *          
 *          **3. Position Controller Initialization:**
 *          - pos_control->init_NE_controller() - Reset XY position integrators
 *          - pos_control->init_U_controller() - Reset Z position integrators
 *          - Ensures clean state (no accumulated errors from previous modes)
 *          
 *          **4. State Reset:**
 *          Throttle configuration:
 *          - throttle_wait = false (enable immediate motor control)
 *          
 *          Landing detection reset:
 *          - landing_detect.lower_limit_start_ms = 0 (reset throttle limit timer)
 *          - landing_detect.land_start_ms = 0 (reset touchdown timer)
 *          - Prepares land detector for new landing sequence
 *          
 *          Crash state:
 *          - plane.crash_state.is_crashed = false
 *          - Clear any previous crash detection
 *          - Allows fresh landing attempt
 *          
 *          **5. Navigation Controller Update:**
 *          - plane.nav_controller->update_waypoint()
 *          - Start waypoint: plane.prev_WP_loc (if crosstrack) or current_loc
 *          - End waypoint: plane.next_WP_loc (landing location)
 *          - Crosstrack: Follow path from previous waypoint vs direct approach
 *          - Updates fixed-wing nav system for status/logging
 *          
 *          **6. Landing State Machine Initialization:**
 *          - poscontrol_init_approach() - Initialize landing approach state
 *          - Selects initial state based on distance and conditions:
 *            * QPOS_APPROACH: Far from landing point (use fixed-wing approach)
 *            * QPOS_AIRBRAKE: Close but fast (active deceleration)
 *            * QPOS_POSITION1: Close and slow (direct position control)
 *          - State selection considers:
 *            * Distance to landing point
 *            * Current groundspeed
 *            * Aircraft type (tailsitter/tiltrotor/multicopter)
 *            * Q_APPROACH_DIST parameter
 *            * Q_OPTIONS flags (DISABLE_APPROACH)
 *          
 *          **Post-Initialization:**
 *          After this function returns, vtol_position_controller() executes
 *          the landing state machine through to touchdown.
 * 
 * @param[in] cmd Mission command with landing location
 * 
 * @return true if landing initialized successfully
 * @return false if setup failed
 * 
 * @note Called from mode_auto.cpp when processing landing mission commands
 * @note Initial altitude set to current altitude (descent managed by controller)
 * @note Landing state machine progresses automatically once initialized
 * @note Supports precision landing if configured
 * @note Supports payload place operations (motors shut down after landing)
 * 
 * @see vtol_position_controller() - Executes landing state machine
 * @see poscontrol_init_approach() - Select initial landing state
 * @see land_detector() - Ground contact detection
 * @see Q_APPROACH_DIST parameter - Approach distance threshold
 * @see Q_OPTIONS parameter - Landing behavior flags
 * 
 * Source: ArduPlane/quadplane.cpp:5136-5167
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.set_alt_cm(plane.current_loc.alt,
                                 Location::AltFrame::ABSOLUTE);

    // initialise the position controller
    pos_control->init_NE_controller();
    pos_control->init_U_controller();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    landing_detect.land_start_ms = 0;

    plane.crash_state.is_crashed = false;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.auto_state.crosstrack ? plane.prev_WP_loc : plane.current_loc,
                                          plane.next_WP_loc);

    poscontrol_init_approach();
    return true;
}

/**
 * @brief Verify VTOL takeoff completion and handle failures
 * 
 * @details Monitors VTOL takeoff progress and detects completion or failure
 *          conditions. Called repeatedly during takeoff to check altitude
 *          progress, timeout, and wind conditions. Transitions to next mission
 *          item when takeoff complete, or initiates emergency landing on failure.
 *          
 *          **Verification Flow:**
 *          
 *          **1. Availability Check:**
 *          - If !available(): Return true (skip takeoff, not configured)
 *          - Allows mission to continue if quadplane disabled
 *          
 *          **2. Arming Check:**
 *          - If not armed and safety off:
 *            * Re-initialize takeoff: do_vtol_takeoff(cmd)
 *            * Return false (not complete)
 *            * Handles case where vehicle disarmed during takeoff
 *          
 *          **3. Ground Effect Compensation:**
 *          - First 3 seconds of takeoff (now - takeoff_start_time_ms < 3000)
 *          - If DISABLE_GROUND_EFFECT_COMP option NOT set:
 *            * ahrs.set_takeoff_expected(true)
 *            * Signals EKF to expect ground effect turbulence
 *            * Improves altitude estimation near ground
 *          
 *          **4. Timeout Failure Detection:**
 *          Conditions:
 *          - takeoff_failure_scalar > 0 (timeout enabled)
 *          - now - takeoff_start_time_ms > takeoff_time_limit_ms
 *          
 *          Actions:
 *          - Send critical GCS message: "Failed to complete takeoff within time limit"
 *          - Force mode to QLAND (emergency VTOL landing)
 *          - Reason: VTOL_FAILED_TAKEOFF
 *          - Return false (takeoff failed)
 *          
 *          Timeout calculation (from do_vtol_takeoff):
 *          - Based on climb distance, acceleration, max velocity
 *          - Multiplied by takeoff_failure_scalar (safety margin)
 *          - Minimum 5 seconds
 *          
 *          **5. Excessive Wind Failure (if AP_AIRSPEED_ENABLED):**
 *          Conditions:
 *          - maximum_takeoff_airspeed > 0 (limit enabled)
 *          - plane.airspeed.get_airspeed() > maximum_takeoff_airspeed
 *          
 *          Actions:
 *          - Send critical GCS message: "Failed to complete takeoff, excessive wind"
 *          - Force mode to QLAND (emergency landing)
 *          - Reason: VTOL_FAILED_TAKEOFF
 *          - Return false (takeoff aborted)
 *          
 *          Rationale:
 *          - High airspeed during VTOL takeoff indicates strong winds
 *          - May exceed vehicle capability
 *          - Better to land than continue in unsafe conditions
 *          
 *          **6. Altitude Completion Check:**
 *          - If current_loc.alt < next_WP_loc.alt: Return false (still climbing)
 *          - Target altitude reached, proceed with completion
 *          
 *          **7. Transition Preparation:**
 *          - transition->restart() - Reset transition state machine
 *          - Prepares for potential transition to fixed-wing flight
 *          - TECS_controller.set_pitch_max(transition_pitch_max)
 *          - TECS_controller.set_pitch_min(-transition_pitch_max)
 *          - Configure pitch limits for transition or forward flight
 *          
 *          **8. Altitude Target:**
 *          - set_alt_target_current() - Lock current altitude as target
 *          - Prevents altitude drift after takeoff complete
 *          - TODO comment suggests this may be removed in future
 *          
 *          **9. Fence Auto-Enable (if AP_FENCE_ENABLED):**
 *          - plane.fence.auto_enable_fence_after_takeoff()
 *          - Activates geofence after successful takeoff
 *          - Safety feature to prevent flyaway
 *          
 *          **10. TECS Reset (AUTO mode only):**
 *          - plane.TECS_controller.reset()
 *          - Clears altitude rate filters
 *          - Prevents initial takeoff climb rates from affecting subsequent flight
 *          - Only in AUTO (not in other modes that use VTOL takeoff)
 *          
 *          **11. Crosstrack Disable:**
 *          - plane.auto_state.next_wp_crosstrack = false
 *          - Next waypoint uses direct approach (no crosstrack correction)
 *          - Immediate heading to next waypoint after takeoff
 *          
 *          **12. Completion:**
 *          - Return true (takeoff verified complete)
 *          - Mission advances to next item
 * 
 * @param[in] cmd Takeoff mission command (for re-initialization if needed)
 * 
 * @return true if takeoff complete (altitude reached)
 * @return false if still climbing, failed, or not armed
 * 
 * @note Called repeatedly at main loop rate during takeoff
 * @note Failure conditions force QLAND mode (emergency landing)
 * @note Timeout scaled by takeoff_failure_scalar parameter
 * @note Wind check requires airspeed sensor (compile-time optional)
 * @note Ground effect compensation improves EKF performance
 * 
 * @warning SAFETY-CRITICAL: Failure detection prevents continued unsafe climb
 * @warning Timeout detection prevents infinite climb attempts
 * @warning Wind detection prevents takeoff in excessive wind
 * 
 * @see do_vtol_takeoff() - Initialize takeoff
 * @see takeoff_controller() - Execute climb
 * @see takeoff_failure_scalar parameter - Timeout safety factor
 * @see maximum_takeoff_airspeed parameter - Wind abort threshold
 * @see Q_OPTIONS parameter - DISABLE_GROUND_EFFECT_COMP flag
 * 
 * Source: ArduPlane/quadplane.cpp:5256-5321
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }

    const uint32_t now = millis();

    // reset takeoff if we aren't armed
    if (!plane.arming.is_armed_and_safety_off()) {
        do_vtol_takeoff(cmd);
        return false;
    }

    if (now - takeoff_start_time_ms < 3000 &&
        !option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
        ahrs.set_takeoff_expected(true);
    }
    
    // check for failure conditions
    if (is_positive(takeoff_failure_scalar) && ((now - takeoff_start_time_ms) > takeoff_time_limit_ms)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff within time limit");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }

#if AP_AIRSPEED_ENABLED
    if (is_positive(maximum_takeoff_airspeed) && (plane.airspeed.get_airspeed() > maximum_takeoff_airspeed)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff, excessive wind");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }
#endif

    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition->restart();
    plane.TECS_controller.set_pitch_max(transition_pitch_max);
    plane.TECS_controller.set_pitch_min(-transition_pitch_max);

    // todo: why are you doing this, I want to delete it.
    set_alt_target_current();

#if AP_FENCE_ENABLED
    plane.fence.auto_enable_fence_after_takeoff();
#endif

    if (plane.control_mode == &plane.mode_auto) {
        // we reset TECS so that the target height filter is not
        // constrained by the climb and sink rates from the initial
        // takeoff height.
        plane.TECS_controller.reset();
    }

    // don't crosstrack on next WP
    plane.auto_state.next_wp_crosstrack = false;

    return true;
}

/**
 * @brief Landing detection based on sustained ground contact indicators
 * 
 * @details Detects touchdown by monitoring throttle state and altitude stability
 *          over a configurable timeout period. Uses multiple criteria to confirm
 *          landing and reject false positives from brief throttle reductions or
 *          momentary altitude stability.
 *          
 *          **Detection Criteria:**
 *          
 *          **1. Preliminary Landing Conditions:**
 *          - should_relax() returns true:
 *            * Throttle output at lower limit (motors near minimum)
 *            * Indicates vehicle on ground or attempting to land
 *          - !poscontrol.pilot_correction_active:
 *            * No active manual position corrections
 *            * Ensures detection not triggered during manual adjustments
 *          
 *          If either condition false:
 *          - Reset landing_detect.land_start_ms = 0 (clear detection)
 *          - Return false (not landed)
 *          - Prevents detection during flight or manual control
 *          
 *          **2. Detection Timer Initialization:**
 *          If land_start_ms == 0 (first detection call):
 *          - landing_detect.land_start_ms = now (start timeout timer)
 *          - landing_detect.vpos_start_m = current vertical position
 *          - Begin monitoring altitude stability
 *          
 *          **3. Altitude Stability Check:**
 *          - height = inertial_nav vertical position (meters, up positive)
 *          - altitude_change = abs(height - vpos_start_m)
 *          - Threshold: landing_detect.detect_alt_change (default 0.2m = 20cm)
 *          
 *          If altitude changed > threshold:
 *          - landing_detect.land_start_ms = 0 (reset detection)
 *          - Return false (not stable, still descending or moving)
 *          
 *          Rationale:
 *          - Vehicle on ground should not move vertically
 *          - 20cm threshold accommodates:
 *            * Suspension compression
 *            * Altitude estimation noise
 *            * Soft/uneven terrain settling
 *          - Larger changes indicate continued descent or liftoff
 *          
 *          **4. Timeout Verification:**
 *          Two separate timers must both expire:
 *          
 *          Timer A: Altitude stability duration
 *          - (now - land_start_ms) >= timeout_ms
 *          - Altitude stable for requested timeout
 *          - Confirms vertical position not changing
 *          
 *          Timer B: Throttle at lower limit duration
 *          - (now - lower_limit_start_ms) >= (timeout_ms + 1000)
 *          - Motors at minimum for timeout + 1 second
 *          - Extra 1-second margin ensures sustained low throttle
 *          
 *          If either timer not expired:
 *          - Return false (not landed yet)
 *          - Continue monitoring
 *          
 *          **5. Landing Confirmed:**
 *          Both conditions met for full timeout:
 *          - Return true (vehicle landed)
 *          - Allows landing completion actions
 *          
 *          **Detection Timeline Example (4-second timeout):**
 *          - T+0.0s: Throttle reaches lower limit (lower_limit_start_ms set)
 *          - T+0.5s: Altitude stable, land_start_ms set
 *          - T+4.5s: Altitude timer expires (4s from land_start_ms)
 *          - T+5.0s: Throttle timer expires (5s from lower_limit_start_ms)
 *          - T+5.0s: Landing confirmed (return true)
 *          
 *          **False Positive Prevention:**
 *          - Altitude stability requirement: Rejects brief ground contacts (bounces)
 *          - Throttle duration requirement: Rejects momentary throttle cuts
 *          - Pilot correction check: Rejects manual landing attempts
 *          - Dual timer requirement: Requires sustained conditions
 *          
 *          **Timeout Selection Guidelines:**
 *          - Short timeout (1-2s): Aggressive landing detection, faster disarm
 *          - Medium timeout (4s): Balanced, default for most operations
 *          - Long timeout (6s): Conservative, backup detector in descent phase
 * 
 * @param[in] timeout_ms Required duration in milliseconds (altitude & throttle)
 * 
 * @return true if landing detected (sustained ground contact)
 * @return false if not landed (still descending or conditions not met)
 * 
 * @note Called from check_land_complete(), check_land_final(), other detectors
 * @note Runs at main loop rate during landing operations
 * @note Altitude threshold: 20cm (landing_detect.detect_alt_change)
 * @note Throttle timer offset: +1000ms beyond altitude timer
 * @note Requires both altitude stability AND low throttle
 * @note Pilot corrections reset detection (safety feature)
 * @note State persists in landing_detect structure
 * 
 * @warning SAFETY-CRITICAL: Controls landing completion and disarm
 * @warning False positive would cause premature disarm during descent
 * @warning False negative would delay disarm after landing
 * @warning Altitude noise can cause detection resets
 * 
 * @see check_land_complete() - Uses 4-second timeout
 * @see check_land_final() - Uses 6-second timeout
 * @see should_relax() - Throttle lower limit detection
 * @see landing_detect.land_start_ms - Altitude stability timer
 * @see landing_detect.lower_limit_start_ms - Throttle limit timer
 * @see landing_detect.detect_alt_change - Altitude threshold (0.2m)
 * 
 * Source: ArduPlane/quadplane.cpp:5433-5472
 */
bool QuadPlane::land_detector(uint32_t timeout_ms)
{
    bool might_be_landed = should_relax() && !poscontrol.pilot_correction_active;
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    float height = inertial_nav.get_position_z_up_cm() * 0.01;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height;
    }

    // we only consider the vehicle landed when the motors have been
    // at minimum for timeout_ms+1000 and the vertical position estimate has not
    // changed by more than 20cm for timeout_ms
    if (fabsf(height - landing_detect.vpos_start_m) > landing_detect.detect_alt_change) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return false;
    }
           
    if ((now - landing_detect.land_start_ms) < timeout_ms ||
        (now - landing_detect.lower_limit_start_ms) < (timeout_ms+1000)) {
        // not landed yet
        return false;
    }

    return true;
}

/*
  check if a landing is complete
 */
/**
 * @brief Check if landing is complete and handle post-landing actions
 * 
 * @details Detects final touchdown using land_detector and manages post-landing
 *          actions including motor shutdown, disarming, or mission continuation.
 *          Handles special case of payload place operations where motors shut down
 *          but vehicle remains armed for scripted actions.
 *          
 *          **Completion Detection Flow:**
 *          
 *          **1. State Validation:**
 *          - Only active during QPOS_LAND_FINAL (final descent phase)
 *          - Return false if in any other state
 *          - Prevents false positives during approach or descent
 *          
 *          **2. Touchdown Detection:**
 *          - land_detector(4000) - Check for sustained ground contact
 *          - 4000ms timeout = 4 seconds of ground indicators
 *          - Criteria checked by land_detector:
 *            * Throttle at lower limit
 *            * Low vertical velocity
 *            * No vertical acceleration
 *            * Motors near minimum output
 *          - If not detected: Return false (still landing)
 *          
 *          **3. Landing Complete Actions:**
 *          If land_detector returns true:
 *          
 *          State transition:
 *          - poscontrol.set_state(QPOS_LAND_COMPLETE)
 *          - Marks landing state machine complete
 *          
 *          Status message:
 *          - GCS message: "Land complete" (MAV_SEVERITY_INFO)
 *          - Confirms successful landing to operator
 *          
 *          **4. Payload Place Special Handling:**
 *          
 *          Check: plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)
 *          
 *          Actions for payload place:
 *          - set_desired_spool_state(SHUT_DOWN) - Stop motors completely
 *          - Return false (do NOT mark mission complete yet)
 *          - Vehicle remains ARMED but motors stopped
 *          
 *          Rationale:
 *          - Lua script controls payload operations after touchdown
 *          - Script may trigger landing abort (climb away with payload)
 *          - Script may command disarm after payload released
 *          - Motors must stop to prevent prop wash on payload
 *          - Armed state allows scripted motor restart if needed
 *          
 *          **5. Normal Landing Post-Actions:**
 *          
 *          Not payload place - Check disarm conditions:
 *          
 *          Condition A: Not in AUTO mode
 *          - plane.control_mode != &plane.mode_auto
 *          - Disarm immediately (manual landing modes)
 *          
 *          Condition B: Mission does not continue
 *          - !plane.mission.continue_after_land()
 *          - MIS_OPTIONS parameter controls this behavior
 *          - Disarm if mission ends at landing
 *          
 *          Disarm action:
 *          - plane.arming.disarm(AP_Arming::Method::LANDED)
 *          - Clean shutdown of all systems
 *          - Safety feature prevents accidental takeoff
 *          
 *          Mission continuation:
 *          - If AUTO mode AND continue_after_land enabled:
 *            * Do not disarm
 *            * Allow mission to proceed to next command
 *            * Enables ground-based mission operations
 *          
 *          **6. Return Value:**
 *          - Return true if landing complete (normal landing)
 *          - Return false if payload place (not complete yet)
 *          
 *          **Key Features:**
 *          - 4-second ground contact confirmation prevents false positives
 *          - Payload place keeps motors stopped but system armed
 *          - AUTO mode with MIS_OPTIONS allows continued armed operations
 *          - Manual modes always disarm for safety
 *          - Mission-end landings disarm automatically
 * 
 * @return true if landing complete and mission can continue
 * @return false if not yet complete or payload place waiting
 * 
 * @note Called from verify_vtol_land() during landing verification
 * @note Only active during QPOS_LAND_FINAL phase
 * @note 4-second land detector timeout (4000ms parameter)
 * @note Payload place requires script or manual intervention
 * @note Disarm behavior controlled by mode and MIS_OPTIONS
 * @note Motors shut down for payload place but remain armed
 * 
 * @warning SAFETY-CRITICAL: Controls motor shutdown and disarm
 * @warning Payload place motors must stop to prevent damage
 * @warning Disarm logic prevents accidental armed state on ground
 * 
 * @see land_detector() - Ground contact detection
 * @see verify_vtol_land() - Landing verification controller
 * @see QPOS_LAND_FINAL - Final descent state
 * @see QPOS_LAND_COMPLETE - Landing complete state
 * @see MAV_CMD_NAV_PAYLOAD_PLACE - Payload place mission command
 * @see MIS_OPTIONS parameter - Mission continuation options
 * 
 * Source: ArduPlane/quadplane.cpp:5473-5502
 */
bool QuadPlane::check_land_complete(void)
{
    if (poscontrol.get_state() != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return false;
    }
    if (land_detector(4000)) {
        poscontrol.set_state(QPOS_LAND_COMPLETE);
        gcs().send_text(MAV_SEVERITY_INFO,"Land complete");

        if (plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)) {
            // for payload place with full landing we shutdown motors
            // and wait for the lua script to trigger a climb (using
            // landing abort) or disarm
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            return false;
        }

        if (plane.control_mode != &plane.mode_auto ||
            !plane.mission.continue_after_land()) {
            // disarm on land unless we have MIS_OPTIONS setup to
            // continue after land in AUTO
            plane.arming.disarm(AP_Arming::Method::LANDED);
        }
        return true;
    }
    return false;
}


/**
 * @brief Check if aircraft should transition to final landing phase
 * 
 * @details Determines when to switch from QPOS_LAND_DESCEND to QPOS_LAND_FINAL
 *          based on altitude above ground. Uses rangefinder data with noise
 *          filtering to prevent false triggers. Also applies landing detector
 *          as backup in case vehicle touches down during descent phase.
 *          
 *          **Final Phase Detection Logic:**
 *          
 *          **1. Altitude Measurement:**
 *          - height_above_ground = relative_ground_altitude(TAKEOFF_LANDING)
 *          - Uses rangefinder if available and healthy
 *          - Falls back to barometer + terrain database if no rangefinder
 *          - Falls back to home-relative altitude if no terrain data
 *          
 *          **2. Altitude Threshold Check:**
 *          Primary condition: height_above_ground < land_final_alt
 *          - land_final_alt parameter defines final phase altitude (meters)
 *          - Typically 5-10 meters above ground
 *          - Close enough for aggressive final descent
 *          
 *          **3. Noise Filtering (Glitch Rejection):**
 *          Stability requirement:
 *          - abs(height_above_ground - last_land_final_agl) < 5.0m
 *          - Consecutive readings must agree within 5 meters
 *          - Prevents high-altitude glitches triggering early final
 *          
 *          Rationale:
 *          - Rangefinder can give erroneous high readings (clouds, birds)
 *          - Requires 2 consecutive readings at ~10Hz (100ms apart)
 *          - 5m max_change threshold catches most noise
 *          - Prevents premature transition to aggressive descent
 *          
 *          If both conditions met:
 *          - Return true (transition to QPOS_LAND_FINAL)
 *          - Begin final descent phase
 *          
 *          If not met:
 *          - Update last_land_final_agl = current height
 *          - Continue descent phase
 *          
 *          **4. Backup Landing Detection:**
 *          Fallback condition:
 *          - land_detector(6000) - 6 second touchdown detection
 *          - Detects if vehicle lands during descent phase
 *          - Prevents getting stuck in descent state
 *          
 *          Criteria (from land_detector):
 *          - Throttle at lower limit for 6 seconds
 *          - Low vertical velocity
 *          - No vertical acceleration
 *          - Motors near minimum output
 *          
 *          If detected:
 *          - Return true (already landed, go to final state)
 *          - Allows proper landing completion handling
 *          
 *          **Altitude Source Priority:**
 *          1. Rangefinder (if available, healthy, in range)
 *          2. Barometer + terrain database
 *          3. Home-relative altitude
 *          
 *          **Why Two Detection Methods:**
 *          - Altitude method: Normal case, controlled transition
 *          - Landing detector: Backup if touchdown happens early
 *          - Early touchdown can occur due to:
 *            * Rapid altitude loss (downdraft)
 *            * Terrain uncertainty
 *            * Manual throttle reduction
 *            * Obstacle on ground
 * 
 * @return true if should transition to QPOS_LAND_FINAL
 * @return false if should continue QPOS_LAND_DESCEND
 * 
 * @note Called from verify_vtol_land() during descent phase
 * @note Runs at main loop rate (~400Hz for quadplanes)
 * @note 5m noise filter prevents false positives from sensor glitches
 * @note 6-second land detector timeout (backup detection)
 * @note Height measured using rangefinder, terrain, or home-relative
 * @note last_land_final_agl updated every call for filtering
 * 
 * @warning SAFETY-CRITICAL: Controls transition to final descent
 * @warning False positive would trigger aggressive descent at high altitude
 * @warning Noise filtering essential for safe operation
 * @warning Backup detector prevents getting stuck if early touchdown
 * 
 * @see verify_vtol_land() - Landing state machine controller
 * @see land_detector() - Ground contact detection
 * @see relative_ground_altitude() - AGL measurement
 * @see land_final_alt parameter - Final phase trigger altitude
 * @see QPOS_LAND_DESCEND - Controlled descent state
 * @see QPOS_LAND_FINAL - Final descent state
 * 
 * Source: ArduPlane/quadplane.cpp:5506-5527
 */
bool QuadPlane::check_land_final(void)
{
    float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    // we require 2 readings at 10Hz to be within 5m of each other to
    // trigger the switch to land final. This prevents a short term
    // glitch at high altitude from triggering land final
    const float max_change = 5;
    if (height_above_ground < land_final_alt &&
        fabsf(height_above_ground - last_land_final_agl) < max_change) {
        return true;
    }
    last_land_final_agl = height_above_ground;

    /*
      also apply landing detector, in case we have landed in descent
      phase. Use a longer threshold
     */
    return land_detector(6000);
}

/**
 * @brief Verify VTOL landing progress and detect completion
 * 
 * @details Monitors landing state machine progress and handles transitions
 *          between landing phases. Detects when to start descent, final landing,
 *          and landing completion. Handles landing abort conditions for payload
 *          place operations. Called repeatedly during landing to advance state
 *          machine and check completion criteria.
 *          
 *          **Verification Flow:**
 *          
 *          **1. Availability Check:**
 *          - If !available(): Return true (skip landing, not configured)
 *          - Allows mission to continue if quadplane disabled
 *          
 *          **2. Position Hold to Descend Transition (QPOS_POSITION2):**
 *          
 *          Conditions for transition to descent:
 *          - Current state == QPOS_POSITION2 (precision position hold)
 *          
 *          **Position Reached Check:**
 *          If pilot correction complete (poscontrol.pilot_correction_done):
 *          - reached_position = !pilot_correction_active
 *          - Uses operator manual correction state
 *          
 *          Otherwise (automatic):
 *          - dist = distance from target XY (meters)
 *          - reached_position = dist < 2.0m (descend_dist_threshold)
 *          - Position error must be under 2 meters
 *          
 *          **Velocity Matching:**
 *          - If velocity_match updated within 1 second:
 *            * target_vel = poscontrol.velocity_match (moving platform)
 *          - Otherwise: target_vel = zero (stationary landing)
 *          
 *          **Velocity Check:**
 *          - Get current NED velocity from AHRS
 *          - vel_error = (vel_ned.xy() - target_vel).length()
 *          - Velocity matched = vel_error < 3.0 m/s (descend_speed_threshold)
 *          
 *          **Transition Actions (if position AND velocity matched):**
 *          - poscontrol.set_state(QPOS_LAND_DESCEND)
 *          - poscontrol.pilot_correction_done = false (reset for next phase)
 *          - pos_control->set_lean_angle_max_cd(0) (vertical descent, no tilt)
 *          - poscontrol.xy_correction.zero() (clear precision corrections)
 *          - Deploy landing gear (if AP_LANDINGGEAR_ENABLED)
 *          - Record altitude: last_land_final_agl (for final phase detection)
 *          - GCS message: "Land descend started"
 *          
 *          **Target Altitude Setup:**
 *          AUTO mode:
 *          - set_next_WP(mission height) - Use mission waypoint altitude
 *          - Allows rangefinder-less final phase trigger
 *          
 *          Other modes (QLAND, QRTL):
 *          - set_next_WP(next_WP_loc) - Keep existing target
 *          - Set altitude to home altitude (ABSOLUTE frame)
 *          - Descend to home altitude level
 *          
 *          **3. Descend to Final Transition (QPOS_LAND_DESCEND):**
 *          
 *          Check: check_land_final() returns true
 *          - Altitude reached land_final_alt, OR
 *          - Significant altitude loss detected (rangefinder), OR
 *          - Timeout conditions met
 *          
 *          Transition actions:
 *          - poscontrol.set_state(QPOS_LAND_FINAL)
 *          - Cut IC engine (if land_icengine_cut != 0 and AP_ICENGINE_ENABLED)
 *          - GCS message: "Land final started"
 *          - Begin aggressive final descent
 *          
 *          **4. Land Abort Handling (QPOS_LAND_ABORT):**
 *          
 *          Abort recovery check:
 *          - If current altitude >= land_descend_start_alt:
 *            * Successfully climbed back to safe altitude
 *            * Return true (landing aborted, continue mission)
 *          
 *          Used when:
 *          - Payload place minimum altitude violated
 *          - Manual abort commanded
 *          - Obstacle detected during descent
 *          
 *          **5. Payload Place Abort Detection:**
 *          
 *          Conditions:
 *          - In MAV_CMD_NAV_PAYLOAD_PLACE mission
 *          - State is QPOS_LAND_DESCEND or QPOS_LAND_FINAL
 *          - Mission param p1 > 0 (minimum clearance altitude defined)
 *          - Current altitude < (land_descend_start_alt - p1)
 *          
 *          Actions if altitude too low:
 *          - GCS message: "Payload place aborted"
 *          - poscontrol.set_state(QPOS_LAND_ABORT)
 *          - Initiates climb back to safe altitude
 *          - Prevents payload striking ground prematurely
 *          
 *          Rationale:
 *          - Payload place requires minimum ground clearance
 *          - Terrain variation or GPS error could violate clearance
 *          - Abort and climb to retry or continue mission
 *          
 *          **6. Landing Completion Check:**
 *          
 *          Conditions:
 *          - check_land_complete() returns true:
 *            * Motors at ground idle
 *            * Low throttle detected for duration
 *            * Vehicle stationary on ground
 *          - plane.mission.continue_after_land() returns true:
 *            * Mission configured to continue after landing
 *            * Not end of mission
 *          
 *          Actions if complete:
 *          - GCS message: "Mission continue"
 *          - Return true (landing verified complete)
 *          - Mission advances to next command
 *          
 *          **7. Default Return:**
 *          - Return false (landing not complete, continue)
 *          - State machine continues execution
 * 
 * @return true if landing complete or aborted (continue mission)
 * @return false if still landing (continue current operation)
 * 
 * @note Called repeatedly at main loop rate during landing
 * @note Manages transitions through landing state machine
 * @note 2m position threshold for descend start (descend_dist_threshold)
 * @note 3m/s velocity threshold for descend start (descend_speed_threshold)
 * @note Supports moving platform landing via velocity_match
 * @note Lean angle zeroed during final descent (vertical only)
 * @note Landing gear deployed at start of descent phase
 * @note IC engine cut at final phase for safety
 * @note Payload place abort prevents ground strike
 * 
 * @warning SAFETY-CRITICAL: Controls landing phase transitions
 * @warning Abort logic prevents payload damage and mission failure
 * @warning Velocity matching essential for moving platform safety
 * 
 * @see vtol_position_controller() - Execute landing state machine
 * @see check_land_final() - Detect final phase altitude
 * @see check_land_complete() - Detect touchdown completion
 * @see do_vtol_land() - Initialize landing
 * @see QPOS_POSITION2 - Precision position hold state
 * @see QPOS_LAND_DESCEND - Controlled descent state
 * @see QPOS_LAND_FINAL - Final descent state
 * @see QPOS_LAND_ABORT - Abort climb state
 * 
 * Source: ArduPlane/quadplane.cpp:5525-5618
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }

    if (poscontrol.get_state() == QPOS_POSITION2) {
        // see if we should move onto the descend stage of landing
        const float descend_dist_threshold = 2.0;
        const float descend_speed_threshold = 3.0;
        bool reached_position = false;
        if (poscontrol.pilot_correction_done) {
            reached_position = !poscontrol.pilot_correction_active;
        } else {
            const float dist = (inertial_nav.get_position_neu_cm().topostype() - poscontrol.target_cm).xy().length() * 0.01;
            reached_position = dist < descend_dist_threshold;
        }
        Vector2f target_vel;
        if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
            target_vel = poscontrol.velocity_match;
        }
        Vector3f vel_ned;
        UNUSED_RESULT(plane.ahrs.get_velocity_NED(vel_ned));
        
        if (reached_position &&
            (vel_ned.xy() - target_vel).length() < descend_speed_threshold) {
            poscontrol.set_state(QPOS_LAND_DESCEND);
            poscontrol.pilot_correction_done = false;
            pos_control->set_lean_angle_max_cd(0);
            poscontrol.xy_correction.zero();
#if AP_LANDINGGEAR_ENABLED
            plane.g2.landing_gear.deploy_for_landing();
#endif
            last_land_final_agl = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
            gcs().send_text(MAV_SEVERITY_INFO,"Land descend started");
            if (plane.control_mode == &plane.mode_auto) {
                // set height to mission height, so we can use the mission
                // WP height for triggering land final if no rangefinder
                // available
                plane.set_next_WP(plane.mission.get_current_nav_cmd().content.location);
            } else {
                plane.set_next_WP(plane.next_WP_loc);
                plane.next_WP_loc.set_alt_cm(ahrs.get_home().alt,
                                             Location::AltFrame::ABSOLUTE);
            }
        }
    }

    // at land_final_alt begin final landing
    if (poscontrol.get_state() == QPOS_LAND_DESCEND && check_land_final()) {
        poscontrol.set_state(QPOS_LAND_FINAL);

#if AP_ICENGINE_ENABLED
        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0, false);
        }
#endif  // AP_ICENGINE_ENABLED
        gcs().send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    // at land_final_alt begin final landing
    if (poscontrol.get_state() == QPOS_LAND_ABORT &&
        plane.current_loc.alt*0.01 >= land_descend_start_alt) {
        // continue to next WP, if there is one
        return true;
    }

    if (plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
        (poscontrol.get_state() == QPOS_LAND_DESCEND ||
         poscontrol.get_state() == QPOS_LAND_FINAL)) {
        const auto &cmd = plane.mission.get_current_nav_cmd();
        if (cmd.p1 > 0 && plane.current_loc.alt*0.01 < land_descend_start_alt - cmd.p1*0.01) {
            gcs().send_text(MAV_SEVERITY_INFO,"Payload place aborted");
            poscontrol.set_state(QPOS_LAND_ABORT);
        }
    }
    
    if (check_land_complete() && plane.mission.continue_after_land()) {
        gcs().send_text(MAV_SEVERITY_INFO,"Mission continue");
        return true;
    }
    return false;
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Write QTUN (QuadPlane Tuning) log message for control analysis
 * 
 * @details Logs comprehensive quadplane control state including throttle,
 *          altitude, climb rate, transition state, and assistance flags.
 *          Essential for post-flight analysis, tuning, and debugging of
 *          VTOL control performance.
 *          
 *          **Logged Data Categories:**
 *          
 *          **1. Throttle Information:**
 *          - throttle_in: Input from attitude controller (0-1)
 *          - angle_boost: Throttle boost for maintaining altitude during turns
 *          - throttle_out: Actual motor throttle output (0-1)
 *          - throttle_hover: Learned hover throttle (0-1)
 *          - throttle_mix: Attitude/altitude throttle mixing ratio
 *          
 *          **2. Altitude Control:**
 *          - desired_alt: Target altitude in meters (from position controller)
 *          - inav_alt: Inertial nav estimated altitude in meters
 *          - baro_alt: Barometer altitude in centimeters
 *          - Set to 0 in QSTABILIZE (no altitude control)
 *          
 *          **3. Vertical Velocity:**
 *          - target_climb_rate: Desired climb rate in cm/s
 *          - climb_rate: Actual climb rate from inertial nav in cm/s
 *          - Set to 0 in QSTABILIZE (no altitude control)
 *          
 *          **4. Transition State:**
 *          - transition_state: Current transition phase (VTOL/transition/FW)
 *          - From transition->get_log_transition_state()
 *          
 *          **5. Assistance Flags Bitmask:**
 *          Bit field indicating active assist modes:
 *          - Bit 0: in_assisted_flight (VTOL assist active)
 *          - Bit 1: forced (assistance forced by parameter)
 *          - Bit 2: speed (assisting due to low airspeed)
 *          - Bit 3: alt (assisting due to low altitude)
 *          - Bit 4: angle (assisting due to attitude error)
 *          - Bit 5: fw_force (forcing fixed-wing controllers)
 *          - Bit 6: spin_recovery (recovering from spin)
 *          
 *          **Assistance Flag Assembly:**
 *          - assisted_flight: General assist active flag
 *          - assist.in_force_assist(): Forced assist by Q_ASSIST_ANGLE = -1
 *          - assist.in_speed_assist(): Airspeed below Q_ASSIST_SPEED
 *          - assist.in_alt_assist(): Altitude below Q_ASSIST_ALT
 *          - assist.in_angle_assist(): Attitude error exceeds Q_ASSIST_ANGLE
 *          - force_fw_control_recovery: Emergency FW controller override
 *          - in_spin_recovery: Active spin recovery maneuver
 *          
 *          **Additional Logged Data:**
 *          - pos_control->write_log(): Position controller internal state (PSCN)
 *          - tiltrotor.write_log(): Tilt-rotor actuator angles (if configured)
 *          
 *          **Log Message Format:**
 *          - Message name: QTUN (Quadplane Tuning)
 *          - Logged at main loop rate when HAL_LOGGING_ENABLED
 *          - Binary log format for efficient storage
 *          - Structure defined in log_QControl_Tuning
 *          
 *          **Usage in Analysis:**
 *          - Throttle tuning: Compare throttle_hover to actual hover requirement
 *          - Altitude tracking: Compare desired_alt to inav_alt and baro_alt
 *          - Climb rate: Compare target_climb_rate to actual climb_rate
 *          - Transition analysis: Correlate transition_state with other parameters
 *          - Assist analysis: Identify when and why assists triggered
 *          - Position control: Review PSCN logs alongside QTUN
 *          - Tilt-rotor: Monitor actuator positions during transitions
 *          
 *          **Common Tuning Scenarios:**
 *          
 *          **Hover throttle incorrect:**
 *          - Compare throttle_hover to throttle_out during steady hover
 *          - Should converge over time to actual hover requirement
 *          
 *          **Altitude oscillation:**
 *          - Check target_climb_rate vs climb_rate correlation
 *          - Review throttle_mix for attitude/altitude competition
 *          - Examine angle_boost during turns
 *          
 *          **Assist triggering unexpectedly:**
 *          - Review assist bitmask to identify trigger cause
 *          - Check thresholds: Q_ASSIST_SPEED, Q_ASSIST_ALT, Q_ASSIST_ANGLE
 *          - Correlate with attitude and airspeed data
 * 
 * @note Only compiled when HAL_LOGGING_ENABLED defined
 * @note Called at main loop rate during VTOL operations
 * @note QSTABILIZE mode logs zeros for altitude/climb rate (no controller)
 * @note Assistance flags indicate active assist reasons (multiple may be active)
 * @note Position controller and tilt-rotor also log separately
 * @note Log message identifier: LOG_QTUN_MSG
 * @note Structure: log_QControl_Tuning in AP_Logger
 * 
 * @see AP_Logger - Data logging system
 * @see pos_control->write_log() - Position controller PSCN log
 * @see tiltrotor.write_log() - Tilt actuator log
 * @see Q_ASSIST_* parameters - Assistance trigger thresholds
 * @see attitude_control - Attitude controller state
 * @see pos_control - Position controller state
 * @see transition - Transition state machine
 * @see log_assistance_flags - Assistance bitmask definition
 * 
 * Source: ArduPlane/quadplane.cpp:6402-6469
 */
void QuadPlane::Log_Write_QControl_Tuning()
{
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (plane.control_mode != &plane.mode_qstabilize) {
        des_alt_m = pos_control->get_pos_desired_U_cm() * 0.01f;
        target_climb_rate_cms = pos_control->get_vel_target_U_cms();
    }

    // Assemble assistance bitmask, definition here is used to generate log documentation
    enum class log_assistance_flags {
        in_assisted_flight = 1U<<0, // true if VTOL assist is active
        forced             = 1U<<1, // true if assistance is forced
        speed              = 1U<<2, // true if assistance due to low airspeed
        alt                = 1U<<3, // true if assistance due to low altitude
        angle              = 1U<<4, // true if assistance due to attitude error
        fw_force           = 1U<<5, // true if forcing use of fixed wing controllers
        spin_recovery      = 1U<<6, // true if recovering from a spin
    };

    uint8_t assist_flags = 0;
    if (assisted_flight) {
        assist_flags |= (uint8_t)log_assistance_flags::in_assisted_flight;
    }
    if (assist.in_force_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::forced;
    }
    if (assist.in_speed_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::speed;
    }
    if (assist.in_alt_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::alt;
    }
    if (assist.in_angle_assist()) {
        assist_flags |= (uint8_t)log_assistance_flags::angle;
    }
    if (force_fw_control_recovery) {
        assist_flags |= (uint8_t)log_assistance_flags::fw_force;
    }
    if (in_spin_recovery) {
        assist_flags |= (uint8_t)log_assistance_flags::spin_recovery;
    }

    struct log_QControl_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_QTUN_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : int32_t(plane.barometer.get_altitude() * 100),
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z_up_cms()),
        throttle_mix        : attitude_control->get_throttle_mix(),
        transition_state    : transition->get_log_transition_state(),
        assist              : assist_flags,
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));

    // write multicopter position control message
    pos_control->write_log();

    // Write tiltrotor tilt angle log
    tiltrotor.write_log();
}
#endif


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
/**
 * @brief Calculate forward motor throttle percentage for VTOL modes
 * 
 * @details Computes commanded throttle for forward-facing pusher/puller motor
 *          during VTOL flight. Forward thrust counters wind and reduces pitch
 *          angle needed for forward flight, improving efficiency and control.
 *          
 *          Three operational modes:
 *          
 *          **1. NEW Method (Q_FWD_THR_USE == 1 or 2):**
 *          - Uses q_fwd_throttle calculated by position controller
 *          - Forward throttle instead of forward pitch
 *          - Returns pre-calculated value directly
 *          
 *          **2. Manual modes (QACRO, QSTABILIZE, QHOVER):**
 *          - No automatic forward throttle (keeps modes simple for recovery)
 *          - If Q_FWD_THR_CH assigned: Use manual RC channel control
 *          - Manual control scaled by Q_FWD_THR_MAX parameter
 *          - Otherwise returns 0 (drift with wind acceptable)
 *          
 *          **3. OLD Method - Automatic velocity controller:**
 *          - Requires Q_VFWD_GAIN > 0 (legacy forward velocity controller)
 *          - PI controller running at 10 Hz
 *          - Tracks horizontal velocity error in body-frame forward direction
 *          - Velocity error normalized by max airspeed
 *          - Pitch demand compensation (moves toward zero pitch)
 *          - Integrator provides persistent thrust
 *          - Constrained to [THR_MIN, THR_CRUISE] range
 *          
 *          Special handling:
 *          - Landing approach: Always allow forward motor (lidar failure protection)
 *          - Landing final + throttle low: Disable forward motor (settling phase)
 *          - Rangefinder out-of-range-low: Disable forward motor
 *          - Below alt_cutoff: Linear fade-out based on altitude
 *          - Zero output: Decay integrator to prevent windup
 *          - Low velocity + should_relax: Zero error and decay integrator (landed detection)
 * 
 * @return Forward motor throttle percentage [0, 100]
 *         - 0 = Off or minimum
 *         - 100 = Maximum forward thrust
 * 
 * @note NEW method bypasses all legacy logic (recommended for new aircraft)
 * @note Manual modes require RC channel assignment for forward throttle
 * @note OLD method runs at 10 Hz (100ms update rate)
 * @note Integrator prevents reverse thrust unless reverse enabled
 * @note Landing phases have specific throttle cutoff logic
 * @note Altitude-based fade prevents ground strikes during low altitude flight
 * 
 * @warning Do not use OLD method (Q_VFWD_GAIN) with tailsitters
 * @warning Forward motor active during landing approach (intentional)
 * 
 * @see Q_FWD_THR_USE parameter (0=old, 1=position modes, 2=all modes)
 * @see Q_FWD_THR_CH parameter (RC channel for manual control)
 * @see Q_FWD_THR_MAX parameter (maximum forward throttle %)
 * @see Q_VFWD_GAIN parameter (legacy velocity controller gain)
 * @see Q_VFWD_ALT_CUTOFF parameter (altitude fade threshold)
 * @see get_vfwd_method()
 * 
 * Source: ArduPlane/quadplane.cpp:4777-4892
 */
float QuadPlane::forward_throttle_pct()
{
    // handle special case where forward thrust motor is used instead of forward pitch.
    if (get_vfwd_method() == ActiveFwdThr::NEW) {
        return 100.0f * q_fwd_throttle;
    }

    /*
      Unless an RC channel is assigned for manual forward throttle control,
      we don't use forward throttle in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind.
    */
    if (plane.control_mode == &plane.mode_qacro ||
        plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover) {

        if (rc_fwd_thr_ch == nullptr) {
            return 0;
        } else {
            // calculate fwd throttle demand from manual input
            float fwd_thr = rc_fwd_thr_ch->percent_input();

            // set forward throttle to fwd_thr_max * (manual input + mix): range [0,100]
            fwd_thr *= 0.01f * constrain_float(fwd_thr_max, 0, 100);
            return fwd_thr;
        }
    }

    /*
      see if the controller should be active
    */
    if (get_vfwd_method() != ActiveFwdThr::OLD) {
        return 0;
    }

    /*
      in modes with a velocity controller
    */
    float deltat = (AP_HAL::millis() - vel_forward.last_ms) * 0.001f;
    if (deltat > 1 || deltat < 0) {
        vel_forward.integrator = 0;
        deltat = 0.1;
    }
    if (deltat < 0.1) {
        // run at 10Hz
        return vel_forward.last_pct;
    }
    vel_forward.last_ms = AP_HAL::millis();
    
    // work out the desired speed in forward direction
    Vector3f desired_velocity_cms = pos_control->get_vel_desired_NEU_cms();

    // convert to NED m/s
    desired_velocity_cms.z *= -1;

    Vector3f vel_ned;
    if (!plane.ahrs.get_velocity_NED(vel_ned)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    // get component of velocity error in fwd body frame direction
    Vector3f vel_error_body = ahrs.get_rotation_body_to_ned().transposed() * ((desired_velocity_cms*0.01f) - vel_ned);

    float fwd_vel_error = vel_error_body.x;

    // scale forward velocity error by maximum airspeed
    fwd_vel_error /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error -= (wp_nav->get_default_speed_NE_cms() * 0.01f) * plane.nav_pitch_cd / (plane.aparm.pitch_limit_max*100);

    if (should_relax() && vel_ned.length() < 1) {
        // we may be landed
        fwd_vel_error = 0;
        vel_forward.integrator *= 0.95f;
    }
    
    // integrator as throttle percentage (-100 to 100)
    vel_forward.integrator += fwd_vel_error * deltat * vel_forward.gain * 100;

    // inhibit reverse throttle and allow petrol engines with min > 0
    int8_t fwd_throttle_min = plane.have_reverse_thrust() ? 0 : plane.aparm.throttle_min;
    vel_forward.integrator = constrain_float(vel_forward.integrator, fwd_throttle_min, plane.aparm.throttle_cruise);

    if (in_vtol_land_approach()) {
        // when we are doing horizontal positioning in a VTOL land
        // we always allow the fwd motor to run. Otherwise a bad
        // lidar could cause the aircraft not to be able to
        // approach the landing point when landing below the takeoff point
        vel_forward.last_pct = vel_forward.integrator;
    } else if ((in_vtol_land_final() && motors->limit.throttle_lower) ||
#if AP_RANGEFINDER_ENABLED
               (plane.rangefinder_use(RangeFinderUse::TAKEOFF_LANDING) &&
                (plane.rangefinder.status_orient(plane.rangefinder_orientation()) == RangeFinder::Status::OutOfRangeLow))) {
#else
              false) {
#endif
        // we're in the settling phase of landing or using a rangefinder that is out of range low, disable fwd motor
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
    } else {
        // If we are below alt_cutoff then scale down the effect until
        // it turns off at alt_cutoff and decay the integrator
        float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
        float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);

        vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                                  height_above_ground, alt_cutoff, alt_cutoff+2);
    }
    if (is_zero(vel_forward.last_pct)) {
        // if the percent is 0 then decay the integrator
        vel_forward.integrator *= 0.95f;
    }

    return vel_forward.last_pct;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    /*
      we only do weathervaning in modes where we are doing VTOL
      position control.
    */
    if (!in_vtol_mode() ||
        !transition->allow_weathervane() ||
        !motors->armed() || (motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) ||
        plane.control_mode == &plane.mode_qstabilize ||
#if QAUTOTUNE_ENABLED
        plane.control_mode == &plane.mode_qautotune ||
#endif
        plane.control_mode == &plane.mode_qhover ||
        should_relax()
        ) {
        // Ensure the weathervane controller is reset to prevent weathervaning from happening outside of the timer
        weathervane->reset();
        return 0.0;
    }

    const bool is_takeoff = in_vtol_auto() && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id);
    float wv_output;
    if (weathervane->get_yaw_out(wv_output,
                                     plane.channel_rudder->get_control_in(),
                                     plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING),
                                     pos_control->get_roll_cd(),
                                     pos_control->get_pitch_cd(),
                                     is_takeoff,
                                     in_vtol_land_sequence())) {
        return constrain_float(wv_output * (1/45.0), -100.0, 100.0) * command_model_pilot.get_rate() * 0.5;
    }

    return 0.0;
}

/*
  start guided mode control
 */
/**
 * @brief Initialize VTOL guided mode
 * 
 * @details Sets up QuadPlane for VTOL guided mode operation. Initializes
 *          position controller target and determines if this is a descent
 *          or ascent profile for approach logic.
 *          
 *          Initialization sequence:
 *          1. Clear guided_takeoff flag (not a takeoff operation)
 *          2. setup_target_position() - Initialize position target to current location
 *          3. poscontrol_init_approach() - Initialize approach state machine
 *          4. Determine slow_descent flag based on altitude change
 *          
 *          Slow descent determination:
 *          - Preferred: Use ABSOLUTE altitude frame for accurate comparison
 *          - Fallback: Use relative altitude if ABSOLUTE unavailable
 *          - slow_descent = true if descending (from_alt > to_alt)
 *          - slow_descent = false if ascending or level
 *          
 *          The slow_descent flag affects approach behavior:
 *          - Descent: Gradual deceleration, early flare preparation
 *          - Ascent: Standard climb profile
 * 
 * @note Called when entering GUIDED mode with VTOL
 * @note Clears guided_takeoff to start in position control (not takeoff)
 * @note ABSOLUTE altitude frame preferred over relative for accuracy
 * @note Fallback to relative altitude for backward compatibility
 * 
 * @see setup_target_position() - Initialize target to current position
 * @see poscontrol_init_approach() - Initialize approach state
 * @see guided_update() - Main guided mode update loop
 * @see guided_mode_enabled() - Check if guided mode should use VTOL
 * 
 * Source: ArduPlane/quadplane.cpp:5001-5014
 */
void QuadPlane::guided_start(void)
{
    guided_takeoff = false;
    setup_target_position();
    int32_t from_alt;
    int32_t to_alt;
    poscontrol_init_approach();
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
    } else {
        // default back to old method
        poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    }
}

/**
 * @brief Update VTOL guided mode control
 * 
 * @details Main update function for QuadPlane guided mode. Handles both
 *          guided takeoff sequences and normal guided position control.
 *          Called from mode_guided.cpp during guided mode execution.
 *          
 *          Dual mode operation:
 *          
 *          **Guided Takeoff Mode:**
 *          Conditions: mode_guided AND guided_takeoff flag AND below target altitude
 *          - Clear throttle_wait (allow immediate motor response)
 *          - Set motors to THROTTLE_UNLIMITED (full power available)
 *          - Run takeoff_controller() for climb control
 *          - Continues until reaching target altitude
 *          
 *          **Normal Guided Position Control:**
 *          Conditions: Not in takeoff OR reached altitude OR not in guided mode
 *          - If transitioning from takeoff: Set state to QPOS_POSITION2
 *          - Clear guided_takeoff flag (return to position control)
 *          - Run vtol_position_controller() for 3D position tracking
 *          
 *          Takeoff completion detection:
 *          - Monitors current_loc.alt vs next_WP_loc.alt
 *          - Transition occurs when alt >= target_alt
 *          - Automatic switchover to position controller
 *          
 *          State machine interaction:
 *          - QPOS_POSITION2 = Approach state for position tracking
 *          - Ensures smooth transition from takeoff to position hold
 * 
 * @note Called at main loop rate during guided mode
 * @note Altitude comparison uses current altitude frame (relative or absolute)
 * @note guided_takeoff flag set externally via DO_VTOL_TRANSITION or takeoff command
 * @note Throttle unlimited during takeoff for maximum climb performance
 * @note Position controller active for waypoint tracking and loiter
 * 
 * @see guided_start() - Initialize guided mode
 * @see guided_mode_enabled() - Check if guided uses VTOL
 * @see takeoff_controller() - Vertical takeoff control
 * @see vtol_position_controller() - 3D position control
 * @see set_desired_spool_state() - Motor spool state management
 * 
 * Source: ArduPlane/quadplane.cpp:5019-5032
 */
void QuadPlane::guided_update(void)
{
    if (plane.control_mode == &plane.mode_guided && guided_takeoff && plane.current_loc.alt < plane.next_WP_loc.alt) {
        throttle_wait = false;
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        takeoff_controller();
    } else {
        if (guided_takeoff) {
            poscontrol.set_state(QPOS_POSITION2);
        }
        guided_takeoff = false;
        // run VTOL position controller
        vtol_position_controller();
    }
}

/**
 * @brief Emergency motor shutdown for Advanced Failsafe termination
 * 
 * @details Immediately shuts down all VTOL motors in response to Advanced
 *          Failsafe (AFS) termination command. This is a safety-critical
 *          function that provides emergency motor cutoff capability.
 *          
 *          Termination sequence:
 *          1. Check if QuadPlane available (enabled and initialized)
 *          2. Set motor spool state to SHUT_DOWN (emergency stop)
 *          3. Call motors->output() to immediately apply zero output
 *          
 *          AFS Termination triggers:
 *          - GPS failure beyond recovery threshold
 *          - Geofence breach with terminate action
 *          - Loss of GCS control with terminate configured
 *          - Battery critical with terminate action
 *          - External termination command (RC or GCS)
 *          
 *          Motor behavior:
 *          - SHUT_DOWN state forces immediate zero throttle
 *          - Bypasses all normal motor control logic
 *          - Output pulse sent to ESCs immediately
 *          - No gradual spool down (emergency action)
 *          
 *          Safety considerations:
 *          - Aircraft will fall from altitude
 *          - Only use for catastrophic failure scenarios
 *          - Configured via AFS_ENABLE and AFS_TERMINATE parameters
 *          - Last resort action when continued flight unsafe
 * 
 * @note Only executes if QuadPlane available (enabled and setup complete)
 * @note Does nothing if QuadPlane not enabled (fixed-wing only aircraft)
 * @note Called from Advanced Failsafe system (AP_AdvancedFailsafe)
 * @note Motor output immediate (no delay or gradual shutdown)
 * 
 * @warning SAFETY-CRITICAL: Aircraft will lose VTOL capability immediately
 * @warning Results in immediate loss of altitude and controlled flight
 * @warning Only for emergency termination scenarios
 * @warning Cannot be reversed once executed (requires reboot)
 * 
 * @see AP_AdvancedFailsafe - Main AFS implementation
 * @see set_desired_spool_state() - Motor spool state control
 * @see available() - Check QuadPlane enabled and initialized
 * @see AFS_ENABLE parameter - Enable Advanced Failsafe
 * @see AFS_TERMINATE parameter - Configure termination actions
 * 
 * Source: ArduPlane/quadplane.cpp:5034-5039
 */
void QuadPlane::afs_terminate(void)
{
    if (available()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
    }
}

/**
 * @brief Check if guided mode should use VTOL motors
 * 
 * @details Determines whether guided mode operation should use QuadPlane
 *          VTOL motors or fixed-wing control surfaces. Used to decide
 *          between VTOL position control and fixed-wing navigation.
 *          
 *          Validation checks (all must pass):
 *          
 *          1. **QuadPlane Available:**
 *             - Q_ENABLE parameter set (QuadPlane enabled)
 *             - Motors initialized and ready
 *             - Returns false if not available
 *          
 *          2. **Mode Compatibility:**
 *             - Must be in GUIDED or AUTO mode
 *             - Other modes use their own control logic
 *             - Returns false for other modes (FBWA, CRUISE, etc.)
 *          
 *          3. **Mission Command Compatibility:**
 *             - In AUTO: Check current navigation command
 *             - MAV_CMD_NAV_LOITER_TURNS: Force fixed-wing (return false)
 *             - Loiter turns requires fixed-wing aerodynamics
 *             - All other AUTO commands: Check guided_mode parameter
 *          
 *          4. **Q_GUIDED_MODE Parameter:**
 *             - 0 = Disabled (use fixed-wing guided)
 *             - Non-zero = Enabled (use VTOL guided)
 *             - Returns guided_mode != 0
 *          
 *          Use cases:
 *          - GUIDED mode from GCS: Use VTOL if guided_mode enabled
 *          - AUTO mode waypoints: Use VTOL per guided_mode setting
 *          - AUTO mode loiter turns: Always use fixed-wing
 *          - Precision landing: Enable VTOL guided for final approach
 * 
 * @return true if guided mode should use VTOL motors and position control
 * @return false if guided mode should use fixed-wing control surfaces
 * 
 * @note Called frequently during AUTO and GUIDED mode execution
 * @note Loiter turns explicitly excluded (requires fixed-wing flight)
 * @note guided_mode parameter can be changed in flight
 * @note Does not check if currently in VTOL mode (only checks if should be)
 * 
 * @see Q_GUIDED_MODE parameter - Enable VTOL guided mode
 * @see guided_start() - Initialize VTOL guided
 * @see guided_update() - Update VTOL guided control
 * @see available() - Check QuadPlane availability
 * @see MAV_CMD_NAV_LOITER_TURNS - Fixed-wing only command
 * 
 * Source: ArduPlane/quadplane.cpp:5044-5058
 */
bool QuadPlane::guided_mode_enabled(void)
{
    if (!available()) {
        return false;
    }
    // only use quadplane guided when in AUTO or GUIDED mode
    if (plane.control_mode != &plane.mode_guided && plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.control_mode == &plane.mode_auto &&
        plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_TURNS) {
        // loiter turns is a fixed wing only operation
        return false;
    }
    return guided_mode != 0;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{
    pos_control->set_pos_desired_U_cm(inertial_nav.get_position_z_up_cm());
}

/**
 * @brief Initiate user-commanded vertical takeoff in GUIDED mode
 * 
 * @details Processes MAVLink VTOL_TAKEOFF command or similar user takeoff request
 *          in GUIDED mode. Validates preconditions, sets up waypoint for vertical
 *          climb, and initializes guided mode VTOL takeoff sequence.
 *          
 *          **Command Context:**
 *          - MAVLink: MAV_CMD_NAV_VTOL_TAKEOFF command in GUIDED mode
 *          - GCS: User-initiated takeoff button/command
 *          - API: External navigation system takeoff command
 *          - Companion computer: Autonomous takeoff request
 *          
 *          **Precondition Validation:**
 *          
 *          **1. Mode Check:**
 *          - plane.control_mode != &plane.mode_guided
 *          - GCS message: "User Takeoff only in GUIDED mode"
 *          - Return false (command rejected)
 *          
 *          Rationale:
 *          - User takeoff distinct from mission takeoff (AUTO mode)
 *          - Prevents confusion between autonomous and user commands
 *          - GUIDED mode indicates external control authority
 *          
 *          **2. Arming Check:**
 *          - !plane.arming.is_armed_and_safety_off()
 *          - GCS message: "Must be armed for takeoff"
 *          - Return false (command rejected)
 *          
 *          Safety verification:
 *          - Motors must be armed for flight
 *          - Safety switch must be off (if equipped)
 *          - Prevents accidental motor start
 *          
 *          **3. Already Flying Check:**
 *          - is_flying() returns true
 *          - GCS message: "Already flying - no takeoff"
 *          - Return false (command rejected)
 *          
 *          Prevents:
 *          - Re-triggering takeoff sequence mid-flight
 *          - Confusion about vehicle state
 *          - Unnecessary waypoint resets
 *          
 *          **Takeoff Initialization (if validation passes):**
 *          
 *          **1. Loiter Flag:**
 *          - plane.auto_state.vtol_loiter = true
 *          - Indicates VTOL position hold behavior
 *          - Affects navigation controller selection
 *          
 *          **2. Waypoint Setup:**
 *          - plane.prev_WP_loc = plane.current_loc (start position)
 *          - plane.next_WP_loc = plane.current_loc (initial target)
 *          - plane.next_WP_loc.alt += takeoff_altitude * 100 (target altitude)
 *          
 *          Altitude conversion:
 *          - takeoff_altitude parameter in meters
 *          - Multiply by 100 for centimeter storage
 *          - Target altitude = current + requested climb
 *          
 *          **3. Motor Spool State:**
 *          - set_desired_spool_state(THROTTLE_UNLIMITED)
 *          - Motors spool up to full power availability
 *          - Ready for vertical climb thrust
 *          
 *          **4. Guided Mode Start:**
 *          - guided_start()
 *          - Initialize guided mode controllers
 *          - Set up position and attitude targets
 *          
 *          **5. Takeoff Flags:**
 *          - guided_takeoff = true
 *          - Marks takeoff sequence active
 *          - Affects throttle and navigation logic
 *          
 *          - guided_wait_takeoff = false
 *          - Not waiting for takeoff (immediately active)
 *          - Climb starts immediately
 *          
 *          **6. Ground Effect Compensation:**
 *          If !option_is_set(OPTION::DISABLE_GROUND_EFFECT_COMP):
 *          - ahrs.set_takeoff_expected(true)
 *          - EKF adjusts for ground effect during takeoff
 *          - Improves altitude estimate near ground
 *          - Enhanced barometer and accelerometer fusion
 *          
 *          Otherwise (ground effect comp disabled):
 *          - Skip AHRS takeoff notification
 *          - Standard EKF behavior throughout takeoff
 *          
 *          **7. Return Success:**
 *          - Return true
 *          - Takeoff sequence initiated
 *          - Vehicle begins vertical climb
 *          
 *          **Takeoff Execution:**
 *          - guided_update() called at main loop rate
 *          - vtol_position_controller() manages climb
 *          - Climbs at configured Q_WP_SPEED_UP rate
 *          - Maintains horizontal position (loiter)
 *          - Completes when reaching target altitude
 *          
 *          **Altitude Reached Behavior:**
 *          - Transitions to position hold at target altitude
 *          - Awaits next GUIDED command or mode change
 *          - Position maintained until new target
 *          
 *          **Typical Usage Sequence:**
 *          1. Operator switches to GUIDED mode
 *          2. Operator arms vehicle
 *          3. GCS sends VTOL_TAKEOFF command with altitude
 *          4. This function validates and initiates takeoff
 *          5. Vehicle climbs vertically to altitude
 *          6. Vehicle holds position awaiting next command
 *          7. Operator issues waypoint or other navigation command
 * 
 * @param[in] takeoff_altitude Desired climb altitude in meters above current position
 * 
 * @return true if takeoff initiated successfully
 * @return false if validation failed (wrong mode, not armed, already flying)
 * 
 * @note Only valid in GUIDED mode
 * @note Requires vehicle armed with safety off
 * @note Rejects if already flying
 * @note Vertical climb only (horizontal position held)
 * @note Altitude specified relative to current position
 * @note Ground effect compensation enabled by default (can disable with option)
 * @note GCS messages inform operator of rejection reasons
 * @note Climb rate determined by Q_WP_SPEED_UP parameter
 * @note Position hold after reaching altitude
 * 
 * @warning SAFETY-CRITICAL: Initiates motor spool and vertical climb
 * @warning Ensure clear airspace above vehicle before commanding
 * @warning Vehicle will climb immediately after command acceptance
 * @warning Altitude must be sufficient for safe operation
 * 
 * @see guided_start() - Initialize guided mode
 * @see guided_update() - Execute guided mode control
 * @see is_flying() - Flying state detection
 * @see set_desired_spool_state() - Motor spool control
 * @see MAV_CMD_NAV_VTOL_TAKEOFF - MAVLink command
 * @see Q_WP_SPEED_UP - Climb rate parameter
 * @see OPTION::DISABLE_GROUND_EFFECT_COMP - Ground effect option
 * @see vtol_position_controller() - Position control execution
 * 
 * Source: ArduPlane/quadplane.cpp:7051-7079
 */
bool QuadPlane::do_user_takeoff(float takeoff_altitude)
{
    if (plane.control_mode != &plane.mode_guided) {
        gcs().send_text(MAV_SEVERITY_INFO, "User Takeoff only in GUIDED mode");
        return false;
    }
    if (!plane.arming.is_armed_and_safety_off()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be armed for takeoff");
        return false;
    }
    if (is_flying()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
        return false;
    }
    plane.auto_state.vtol_loiter = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.current_loc;
    plane.next_WP_loc.alt += takeoff_altitude*100;
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    guided_start();
    guided_takeoff = true;
    guided_wait_takeoff = false;
    if (!option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
        ahrs.set_takeoff_expected(true);
    }
    return true;
}

/**
 * @brief Check if waypoint navigation controller is actively being updated
 * 
 * @details Determines if the AC_WPNav waypoint navigation controller is
 *          currently in use for position control. The waypoint navigation
 *          controller provides smooth trajectory generation, acceleration
 *          limiting, and waypoint sequencing for autonomous navigation.
 *          
 *          **Waypoint Navigation Controller:**
 *          - Instance: wp_nav (AC_WPNav object)
 *          - Purpose: Autonomous waypoint navigation with smooth paths
 *          - Features: S-curve acceleration, corner cutting, terrain following
 *          - Used for: Loiter, autonomous waypoints, land approach
 *          
 *          **Modes Using WP_NAV:**
 *          
 *          **QLOITER Mode:**
 *          - Purpose: Position hold with pilot override capability
 *          - WP_NAV usage: Maintains loiter position target
 *          - Updates: Processes pilot stick inputs to adjust target
 *          - Behavior: Returns to loiter point when sticks centered
 *          - Controller: wp_nav->update_loiter()
 *          
 *          **QLAND Mode:**
 *          - Purpose: Autonomous vertical landing
 *          - WP_NAV usage: Maintains horizontal position during descent
 *          - Updates: Holds landing target position
 *          - Behavior: Vertical descent with position hold
 *          - Controller: wp_nav combined with altitude control
 *          
 *          **Modes NOT Using WP_NAV:**
 *          
 *          **QHOVER:**
 *          - Uses pos_control directly (no wp_nav)
 *          - Simple position hold without trajectory generation
 *          - Faster response, less smooth
 *          
 *          **QSTABILIZE/QACRO:**
 *          - No position control (manual flight)
 *          - Only attitude control active
 *          
 *          **GUIDED:**
 *          - May use wp_nav for waypoint guidance
 *          - Uses pos_control for position targets
 *          - Context-dependent (not always wp_nav)
 *          
 *          **AUTO:**
 *          - Uses wp_nav for VTOL waypoints
 *          - Not checked by this function (checks specific AUTO state instead)
 *          - Context-dependent within AUTO mode
 *          
 *          **Why This Check Matters:**
 *          
 *          **Controller Selection:**
 *          - wp_nav provides smoother trajectories than direct pos_control
 *          - Determines which update function to call
 *          - Affects navigation behavior and response
 *          
 *          **Parameter Application:**
 *          - WP_NAV has separate parameters (Q_WP_SPEED, Q_WP_RADIUS, etc.)
 *          - Different tuning than direct position control
 *          - Affects performance characteristics
 *          
 *          **Feature Availability:**
 *          - Corner cutting only available with wp_nav
 *          - S-curve acceleration only with wp_nav
 *          - Terrain following integration with wp_nav
 *          
 *          **Update Path Determination:**
 *          - If using_wp_nav() == true: Call wp_nav->update()
 *          - If using_wp_nav() == false: Call pos_control->update()
 *          - Ensures correct controller runs each loop
 * 
 * @return true if wp_nav controller should be updated (QLOITER or QLAND)
 * @return false if wp_nav not in use (other modes or unavailable)
 * 
 * @note QLOITER and QLAND are the only modes checked
 * @note GUIDED mode may use wp_nav but returns false from this check
 * @note AUTO mode has separate wp_nav usage logic
 * @note wp_nav provides smoother trajectories than direct pos_control
 * @note Does not check if quadplane is available()
 * 
 * @see wp_nav - AC_WPNav waypoint navigation object
 * @see pos_control - AC_PosControl position controller object
 * @see AC_WPNav::update_loiter() - Loiter update function
 * @see Q_WP_SPEED - Waypoint speed parameter
 * @see Q_WP_RADIUS - Waypoint radius parameter
 * @see Q_WP_SPEED_UP - Climb speed parameter
 * @see Q_WP_SPEED_DN - Descent speed parameter
 * 
 * Source: ArduPlane/quadplane.cpp:7228-7235
 */
bool QuadPlane::using_wp_nav(void) const
{
    if (plane.control_mode == &plane.mode_qloiter || plane.control_mode == &plane.mode_qland) {
        return true;
    }
    return false;
}

/*
  return mav_type for heartbeat
 */
MAV_TYPE QuadPlane::get_mav_type(void) const
{
    if (mav_type.get() == 0) {
        return MAV_TYPE_FIXED_WING;
    }
    return MAV_TYPE(mav_type.get());
}

/**
 * @brief Determine if mission command is a VTOL takeoff
 * 
 * @details Checks if a mission command ID represents a vertical takeoff operation.
 *          Handles explicit VTOL takeoff commands and conditionally treats
 *          fixed-wing takeoff commands as VTOL takeoffs based on configuration.
 *          
 *          **Command Type Detection:**
 *          
 *          **1. Explicit VTOL Takeoff:**
 *          - id == MAV_CMD_NAV_VTOL_TAKEOFF
 *          - Mission planner explicitly requested VTOL takeoff
 *          - Always returns true
 *          - Unambiguous vertical takeoff
 *          
 *          **2. Fixed-Wing Takeoff (Conditional):**
 *          Conditions ALL must be true:
 *          - id == MAV_CMD_NAV_TAKEOFF (standard takeoff command)
 *          - available() == true (QuadPlane enabled and configured)
 *          - !option_is_set(OPTION::ALLOW_FW_TAKEOFF) (FW takeoff not allowed)
 *          
 *          When conditions met:
 *          - Treat as VTOL takeoff
 *          - Return true
 *          - Use vertical takeoff instead of fixed-wing
 *          
 *          Rationale:
 *          - QuadPlane configured but FW takeoff not enabled
 *          - Mission may have been created for pure fixed-wing
 *          - Safe to substitute VTOL takeoff for compatibility
 *          - Prevents failed takeoff attempt
 *          
 *          **3. Allow Fixed-Wing Takeoff Option:**
 *          If option_is_set(OPTION::ALLOW_FW_TAKEOFF):
 *          - MAV_CMD_NAV_TAKEOFF executed as fixed-wing takeoff
 *          - Returns false (not a VTOL takeoff)
 *          - Aircraft uses conventional runway takeoff
 *          - Requires suitable runway and conditions
 *          
 *          **4. Default:**
 *          - id is neither VTOL nor eligible FW takeoff
 *          - Return false
 *          - Not a takeoff command or wrong type
 *          
 *          **Mission Command Handling:**
 *          
 *          **MAV_CMD_NAV_VTOL_TAKEOFF Parameters:**
 *          - param1: Empty
 *          - param2: Front transition heading (if applicable)
 *          - param3: Empty
 *          - param4: Yaw angle (degrees, NaN for current heading)
 *          - param5: Latitude (or 0 for current location)
 *          - param6: Longitude (or 0 for current location)
 *          - param7: Altitude (AMSL or relative)
 *          
 *          **MAV_CMD_NAV_TAKEOFF Parameters (when treated as VTOL):**
 *          - param1: Minimum pitch (ignored for VTOL)
 *          - param2-6: Standard waypoint parameters
 *          - param7: Altitude
 *          
 *          **Usage in Mission Logic:**
 *          
 *          Called during mission command verification:
 *          1. Mission command loaded from mission list
 *          2. is_vtol_takeoff(cmd.id) checks command type
 *          3. If true: Execute do_vtol_takeoff() initialization
 *          4. If false: Execute standard fixed-wing takeoff
 *          
 *          **Configuration Scenarios:**
 *          
 *          **Scenario A: Pure VTOL Operation**
 *          - Q_ENABLE = 1 (QuadPlane enabled)
 *          - ALLOW_FW_TAKEOFF = 0 (default, FW takeoff disabled)
 *          - Result: Both MAV_CMD_NAV_VTOL_TAKEOFF and MAV_CMD_NAV_TAKEOFF use VTOL
 *          - Use case: No runway, vertical takeoff only
 *          
 *          **Scenario B: Hybrid Operation with FW Takeoff**
 *          - Q_ENABLE = 1
 *          - ALLOW_FW_TAKEOFF = 1 (FW takeoff enabled)
 *          - Result: MAV_CMD_NAV_TAKEOFF uses fixed-wing, MAV_CMD_NAV_VTOL_TAKEOFF uses VTOL
 *          - Use case: Runway available, mission planner chooses takeoff type
 *          
 *          **Scenario C: Pure Fixed-Wing**
 *          - Q_ENABLE = 0 (QuadPlane disabled)
 *          - Result: All takeoffs are fixed-wing (this function returns false)
 *          - Use case: Traditional fixed-wing aircraft
 * 
 * @param[in] id MAVLink mission command ID to check
 * 
 * @return true if command should be executed as VTOL takeoff
 * @return false if command is not a VTOL takeoff
 * 
 * @note MAV_CMD_NAV_VTOL_TAKEOFF always returns true
 * @note MAV_CMD_NAV_TAKEOFF conditionally treated as VTOL based on options
 * @note OPTION::ALLOW_FW_TAKEOFF controls fixed-wing takeoff permission
 * @note Used during mission command type determination
 * @note Does not execute takeoff, only identifies command type
 * 
 * @see do_vtol_takeoff() - Execute VTOL takeoff initialization
 * @see MAV_CMD_NAV_VTOL_TAKEOFF - Explicit VTOL takeoff command
 * @see MAV_CMD_NAV_TAKEOFF - Standard takeoff command
 * @see OPTION::ALLOW_FW_TAKEOFF - Enable fixed-wing takeoff option
 * @see available() - QuadPlane availability check
 * @see is_vtol_land() - Corresponding landing type check
 * 
 * Source: ArduPlane/quadplane.cpp:7339-7352
 */
bool QuadPlane::is_vtol_takeoff(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return true;
    }
    if (id == MAV_CMD_NAV_TAKEOFF && available() && !option_is_set(QuadPlane::OPTION::ALLOW_FW_TAKEOFF)) {
        // treat fixed wing takeoff as VTOL takeoff
        return true;
    }
    return false;
}

/**
 * @brief Determine if mission command is a VTOL landing
 * 
 * @details Checks if a mission command ID represents a vertical landing operation.
 *          Handles explicit VTOL landing commands, payload placement, and
 *          conditionally treats fixed-wing landing as VTOL based on configuration.
 *          Accounts for fixed-wing spiral approach transition to VTOL landing.
 *          
 *          **Command Type Detection:**
 *          
 *          **1. Explicit VTOL Landing Commands:**
 *          - id == MAV_CMD_NAV_VTOL_LAND (VTOL landing)
 *          - id == MAV_CMD_NAV_PAYLOAD_PLACE (payload delivery with landing)
 *          
 *          **Fixed-Wing Spiral Approach Check:**
 *          If landing_with_fixed_wing_spiral_approach():
 *          - Aircraft uses fixed-wing spiral descent to approach altitude
 *          - Then transitions to VTOL for final landing phase
 *          
 *          Stage-dependent return:
 *          - plane.vtol_approach_s.approach_stage == Stage::VTOL_LANDING
 *          - Return true ONLY if in VTOL landing stage
 *          - Return false if still in fixed-wing spiral approach
 *          
 *          Stages:
 *          - APPROACH_LINE: Initial fixed-wing approach
 *          - VTOL_LANDING: Transitioned to VTOL, vertical descent
 *          
 *          Rationale:
 *          - During spiral approach, use fixed-wing controllers
 *          - After transition, use VTOL controllers
 *          - Stage determines controller selection
 *          
 *          **Standard VTOL Landing (no spiral):**
 *          If !landing_with_fixed_wing_spiral_approach():
 *          - Return true immediately
 *          - Pure VTOL landing from start
 *          - No fixed-wing approach phase
 *          
 *          **2. Fixed-Wing Landing (Conditional):**
 *          Conditions ALL must be true:
 *          - id == MAV_CMD_NAV_LAND (standard landing command)
 *          - available() == true (QuadPlane enabled)
 *          - !option_is_set(OPTION::ALLOW_FW_LAND) (FW landing not allowed)
 *          
 *          When conditions met:
 *          - Treat as VTOL landing
 *          - Return true
 *          - Use vertical landing instead of fixed-wing
 *          
 *          Rationale:
 *          - QuadPlane configured but FW landing disabled
 *          - Mission may have been created for pure fixed-wing
 *          - Safe to substitute VTOL landing for compatibility
 *          - Prevents unsafe fixed-wing landing attempt
 *          
 *          **3. Allow Fixed-Wing Landing Option:**
 *          If option_is_set(OPTION::ALLOW_FW_LAND):
 *          - MAV_CMD_NAV_LAND executed as fixed-wing landing
 *          - Returns false (not a VTOL landing)
 *          - Aircraft uses conventional runway landing
 *          - Requires suitable runway and conditions
 *          
 *          **4. Default:**
 *          - id is neither VTOL nor eligible FW landing
 *          - Return false
 *          - Not a landing command or wrong type
 *          
 *          **Mission Command Details:**
 *          
 *          **MAV_CMD_NAV_VTOL_LAND Parameters:**
 *          - param1: Empty
 *          - param2: Empty
 *          - param3: Approach altitude (optional pre-land loiter altitude)
 *          - param4: Yaw angle (degrees, NaN for current heading)
 *          - param5: Latitude
 *          - param6: Longitude
 *          - param7: Landing altitude (AMSL)
 *          
 *          **MAV_CMD_NAV_PAYLOAD_PLACE Parameters:**
 *          - param1: Maximum acceptable horizontal distance from target (m)
 *          - param2: Maximum acceptable vertical distance from target (m)
 *          - param3-4: Empty
 *          - param5: Latitude
 *          - param6: Longitude
 *          - param7: Landing altitude (AMSL)
 *          
 *          Additional payload place behavior:
 *          - Precision position hold before descent
 *          - Abort if minimum clearance violated
 *          - Mission continues after placement
 *          
 *          **MAV_CMD_NAV_LAND Parameters (when treated as VTOL):**
 *          - param1: Abort altitude (meters)
 *          - param2: Landing mode (precision land if supported)
 *          - param3: Empty
 *          - param4: Yaw angle
 *          - param5-7: Latitude, longitude, altitude
 *          
 *          **Fixed-Wing Spiral Approach:**
 *          
 *          **Purpose:** Safe altitude loss in confined area
 *          
 *          **Sequence:**
 *          1. Command: MAV_CMD_NAV_VTOL_LAND with spiral approach enabled
 *          2. Stage::APPROACH_LINE - Fixed-wing spiral descent
 *          3. Reach target altitude/position
 *          4. Transition to VTOL
 *          5. Stage::VTOL_LANDING - Vertical descent and landing
 *          6. is_vtol_land() returns false during spiral, true during VTOL
 *          
 *          **Configuration Scenarios:**
 *          
 *          **Scenario A: Pure VTOL Landing**
 *          - Q_ENABLE = 1
 *          - ALLOW_FW_LAND = 0 (default)
 *          - Result: All landings are VTOL
 *          - Use case: No runway, vertical landing only
 *          
 *          **Scenario B: Hybrid with FW Landing**
 *          - Q_ENABLE = 1
 *          - ALLOW_FW_LAND = 1
 *          - Result: MAV_CMD_NAV_LAND uses FW, MAV_CMD_NAV_VTOL_LAND uses VTOL
 *          - Use case: Mission planner chooses landing type
 *          
 *          **Scenario C: Spiral Approach Landing**
 *          - Q_ENABLE = 1
 *          - Q_OPTIONS includes VTOL_LAND_FW_APPROACH
 *          - Result: MAV_CMD_NAV_VTOL_LAND with spiral, then VTOL
 *          - Use case: Altitude loss in confined area
 * 
 * @param[in] id MAVLink mission command ID to check
 * 
 * @return true if command should be executed as VTOL landing (or in VTOL phase)
 * @return false if command is not a VTOL landing or still in FW approach
 * 
 * @note MAV_CMD_NAV_VTOL_LAND and MAV_CMD_NAV_PAYLOAD_PLACE handled
 * @note Spiral approach returns false until VTOL_LANDING stage
 * @note MAV_CMD_NAV_LAND conditionally treated as VTOL based on options
 * @note OPTION::ALLOW_FW_LAND controls fixed-wing landing permission
 * @note Used for controller selection during landing
 * @note Does not execute landing, only identifies command type and phase
 * 
 * @see do_vtol_land() - Execute VTOL landing initialization
 * @see verify_vtol_land() - Verify landing progress
 * @see landing_with_fixed_wing_spiral_approach() - Check spiral approach config
 * @see MAV_CMD_NAV_VTOL_LAND - Explicit VTOL landing command
 * @see MAV_CMD_NAV_PAYLOAD_PLACE - Payload placement command
 * @see MAV_CMD_NAV_LAND - Standard landing command
 * @see OPTION::ALLOW_FW_LAND - Enable fixed-wing landing option
 * @see plane.vtol_approach_s.approach_stage - Spiral approach stage
 * @see is_vtol_takeoff() - Corresponding takeoff type check
 * 
 * Source: ArduPlane/quadplane.cpp:7436-7449
 */
bool QuadPlane::is_vtol_land(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_LAND || id == MAV_CMD_NAV_PAYLOAD_PLACE) {
        if (landing_with_fixed_wing_spiral_approach()) {
            return plane.vtol_approach_s.approach_stage == Plane::VTOLApproach::Stage::VTOL_LANDING;
        } else {
            return true;
        }
    }
    if (id == MAV_CMD_NAV_LAND && available() && !option_is_set(QuadPlane::OPTION::ALLOW_FW_LAND)) {
        // treat fixed wing land as VTOL land
        return true;
    }
    return false;
}

/*
  return true if we are in a transition to fwd flight from hover
 */
/**
 * @brief Check if aircraft is in forward transition (VTOL→FW)
 * 
 * @details Returns true when transitioning from multicopter flight to
 *          fixed-wing flight. During this phase, VTOL motors gradually
 *          reduce thrust while forward motor increases, and control
 *          transitions from multicopter to fixed-wing algorithms.
 * 
 * @return true if actively transitioning from VTOL to fixed-wing
 * 
 * @note Returns false if QuadPlane not available or not transitioning
 * @note Complement of in_vtol_mode() during transition phase
 * 
 * @see transition->active_frwd()
 * @see in_vtol_mode()
 * 
 * Source: ArduPlane/quadplane.cpp:4035-4038
 */
/**
 * @brief Check if currently in forward transition (VTOL to fixed-wing)
 * 
 * @details Determines if the aircraft is actively transitioning from
 *          multicopter (VTOL) mode to fixed-wing flight. This involves
 *          accelerating to fixed-wing flying speed while gradually
 *          transitioning control from multicopter to fixed-wing surfaces.
 *          
 *          **Transition Phases:**
 *          - Forward transition: VTOL → Fixed-wing (this function)
 *          - Back transition: Fixed-wing → VTOL (see in_back_transition)
 *          
 *          **Detection Logic:**
 *          1. available() - QuadPlane configured and enabled
 *          2. transition->active_frwd() - Forward transition state active
 *          
 *          **Forward Transition Characteristics:**
 *          - Increase airspeed from hover to cruise speed
 *          - Gradually reduce VTOL motor thrust
 *          - Increase fixed-wing control surface authority
 *          - May involve tilting motors/rotors forward
 *          - Maintain altitude and attitude control throughout
 *          
 *          **Typical Forward Transition Sequence:**
 *          1. Command forward transition (MAV_CMD_DO_VTOL_TRANSITION)
 *          2. Aircraft pitches forward for acceleration
 *          3. Fixed-wing thrust increases
 *          4. Airspeed builds to transition complete speed
 *          5. VTOL motors throttle back or tilt to cruise position
 *          6. Transition complete, pure fixed-wing flight
 *          
 *          **When Forward Transition Occurs:**
 *          - AUTO mission with DO_VTOL_TRANSITION(FW) command
 *          - QRTL mode transitioning to fixed-wing RTL
 *          - Manual mode change from QHOVER/QLOITER to FBWA/CRUISE
 *          - Triggered by airspeed in assisted flight modes
 *          
 *          **Uses of This Check:**
 *          - Blend VTOL and fixed-wing control outputs
 *          - Modify navigation behavior during transition
 *          - Adjust throttle and attitude limits
 *          - Control tilt-rotor or tilt-wing positions
 *          - Display transition status to operator
 * 
 * @return true if in forward transition to fixed-wing
 * @return false if not transitioning or unavailable
 * 
 * @note Forward transition typically takes 5-10 seconds
 * @note Transition duration depends on Q_TRANSITION_MS parameter
 * @note Tilt-rotor aircraft may transition motors during this phase
 * @note Control blending from VTOL to FW occurs during transition
 * @note Returns false if QuadPlane not available
 * 
 * @see in_back_transition() - Check for FW to VTOL transition
 * @see available() - QuadPlane availability check
 * @see transition->active_frwd() - Forward transition state
 * @see Q_TRANSITION_MS parameter - Transition duration
 * @see MAV_CMD_DO_VTOL_TRANSITION - Mission transition command
 * 
 * Source: ArduPlane/quadplane.cpp:7051-7054
 */
bool QuadPlane::in_frwd_transition(void) const
{
    return available() && transition->active_frwd();
}

/**
 * @brief Calculate stopping distance for current or specified ground speed
 * 
 * @details Estimates the distance required to decelerate from current speed
 *          to zero using the configured transition deceleration rate (Q_TRANS_DECEL).
 *          
 *          Uses simplified kinematic equation: distance = v²/(2*a)
 *          where v = ground speed, a = deceleration rate
 *          
 *          This is an approximation as actual drag varies with pitch attitude
 *          and airspeed, but provides reasonable transition distance planning.
 * 
 * @param[in] ground_speed_squared Ground speed squared in (m/s)²
 * 
 * @return Estimated stopping distance in meters
 * 
 * @note Used for transition distance planning in AUTO missions
 * @note Deceleration rate set by Q_TRANS_DECEL parameter (default 2.0 m/s²)
 * @note Actual stopping distance varies with wind and aircraft drag
 * 
 * @see transition_threshold()
 * @see Q_TRANS_DECEL parameter
 * 
 * Source: ArduPlane/quadplane.cpp:4043-4049
 */
float QuadPlane::stopping_distance(float ground_speed_squared) const
{
    // use v^2/(2*accel). This is only quite approximate as the drag
    // varies with pitch, but it gives something for the user to
    // control the transition distance in a reasonable way
    return ground_speed_squared / (2 * transition_decel);
}

/**
 * @brief Calculate required deceleration to stop within given distance
 * 
 * @details Inverse calculation of stopping_distance() - computes the
 *          deceleration rate needed to stop within a target distance
 *          from current speed.
 *          
 *          Uses kinematic equation: a = v²/(2*d)
 *          where v = current speed, d = available stopping distance
 * 
 * @param[in] stop_distance Available distance for stopping in meters
 * @param[in] ground_speed_squared Current ground speed squared in (m/s)²
 * 
 * @return Required deceleration in m/s²
 * 
 * @note Minimum stop_distance of 1 meter enforced to prevent division by zero
 * @note Used for transition planning when approaching waypoints
 * 
 * Source: ArduPlane/quadplane.cpp:4054-4057
 */
float QuadPlane::accel_needed(float stop_distance, float ground_speed_squared) const
{
    return ground_speed_squared / (2 * MAX(1,stop_distance));
}

/**
 * @brief Calculate current stopping distance based on actual ground speed
 * 
 * @details Convenience wrapper for stopping_distance(ground_speed_squared)
 *          that uses current ground speed from AHRS.
 * 
 * @return Current stopping distance in meters
 * 
 * @note Uses current ground speed from velocity vector
 * @note Updates continuously as ground speed changes
 * 
 * @see stopping_distance(float ground_speed_squared)
 * 
 * Source: ArduPlane/quadplane.cpp:4062-4065
 */
float QuadPlane::stopping_distance(void)
{
    return stopping_distance(plane.ahrs.groundspeed_vector().length_squared());
}

/**
 * @brief Calculate transition initiation distance threshold
 * 
 * @details Computes the distance from destination at which VTOL landing
 *          approach should begin, based on stopping distance at cruise airspeed.
 *          
 *          Uses 1.5x safety factor:
 *          threshold = 1.5 * stopping_distance(airspeed_cruise²)
 *          
 *          This ensures adequate distance to decelerate from cruise speed,
 *          perform any required circling, and transition to VTOL for landing.
 * 
 * @return Distance threshold in meters for transition initiation
 * 
 * @note Used to determine when to begin VTOL landing approach in AUTO missions
 * @note Based on AIRSPEED_CRUISE parameter (default 15 m/s)
 * @note 1.5x factor provides safety margin for wind and approach alignment
 * 
 * @see stopping_distance()
 * @see AIRSPEED_CRUISE parameter
 * 
 * Source: ArduPlane/quadplane.cpp:4071-4075
 */
float QuadPlane::transition_threshold(void)
{
    // 1.5 times stopping distance for cruise speed
    return 1.5 * stopping_distance(sq(plane.aparm.airspeed_cruise));
}

#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity

/**
 * @brief Update attitude/altitude throttle mixing ratio based on flight conditions
 * 
 * @details Dynamically adjusts how throttle is split between attitude control
 *          (maintaining vehicle orientation) and altitude control (maintaining
 *          height). This balance is critical for stable VTOL flight across
 *          different flight phases and conditions.
 *          
 *          **Throttle Mix Concept:**
 *          
 *          Multicopter control requires throttle for two purposes:
 *          1. **Altitude Control**: Vertical thrust to maintain or change altitude
 *          2. **Attitude Control**: Differential thrust to rotate the vehicle
 *          
 *          The throttle mix determines priority:
 *          - **Mix Min (0.0)**: Prioritize altitude, limit attitude corrections
 *          - **Mix Max (1.0)**: Prioritize attitude, use full throttle range
 *          
 *          **Acceleration Filtering:**
 *          
 *          First, filter earth-frame acceleration for noise rejection:
 *          ```
 *          accel_ef = ahrs.get_accel_ef()
 *          accel_ef.z += GRAVITY_MSS  // Remove gravity component
 *          throttle_mix_accel_ef_filter.apply(accel_ef, dt)
 *          ```
 *          
 *          Purpose: Smooth acceleration measurement to detect sustained
 *          acceleration/turbulence rather than transient vibration.
 *          
 *          **Transition Override:**
 *          
 *          ```
 *          if (!transition->allow_update_throttle_mix())
 *              return  // Transition manages its own mix
 *          ```
 *          
 *          During forward/back transitions, transition controller directly
 *          manages throttle mix for coordinated fixed-wing/VTOL blending.
 *          This function defers to transition logic when active.
 *          
 *          **Disarmed/Landed State:**
 *          
 *          ```
 *          if (!motors->armed())
 *              set_throttle_mix_min()  // Prioritize altitude when disarmed
 *          ```
 *          
 *          When motors not armed, altitude control has priority (though
 *          both are inactive). Ensures consistent state on arming.
 *          
 *          **Manual Throttle Modes:**
 *          
 *          For modes where pilot directly controls throttle (QSTABILIZE, QACRO):
 *          
 *          **If throttle zero and not in air mode:**
 *          ```
 *          if (!is_positive(get_throttle_input()) && !air_mode_active())
 *              set_throttle_mix_min()  // Altitude priority (allows descent)
 *          ```
 *          
 *          Rationale: Zero throttle indicates pilot wants descent. Prioritize
 *          altitude (descent) over aggressive attitude corrections.
 *          
 *          **If throttle active or air mode:**
 *          ```
 *          else
 *              set_throttle_mix_man()  // Manual throttle mix
 *          ```
 *          
 *          Use manual throttle mixing strategy. Air mode maintains attitude
 *          authority even at zero throttle (for acrobatics).
 *          
 *          **Autopilot Throttle Modes:**
 *          
 *          For modes where autopilot controls throttle (QHOVER, QLOITER, AUTO, etc.),
 *          use condition-based logic to determine mix:
 *          
 *          **Condition 1: Large Angle Request**
 *          ```
 *          angle_target = attitude_control->get_att_target_euler_cd()
 *          large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD
 *          ```
 *          - LAND_CHECK_LARGE_ANGLE_CD = 1500 (15 degrees)
 *          - Vector length: sqrt(roll² + pitch²)
 *          - True if combined roll/pitch target exceeds 15°
 *          - Indicates aggressive maneuvering (navigation, avoidance)
 *          - Requires full attitude authority → use_mix_max = true
 *          
 *          **Condition 2: Large Angle Error**
 *          ```
 *          angle_error = attitude_control->get_att_error_angle_deg()
 *          large_angle_error = angle_error > LAND_CHECK_ANGLE_ERROR_DEG
 *          ```
 *          - LAND_CHECK_ANGLE_ERROR_DEG = 30.0 degrees
 *          - Error between target and actual attitude
 *          - True if error exceeds 30°
 *          - Indicates external disturbance (wind gust, collision)
 *          - Requires aggressive correction → use_mix_max = true
 *          
 *          **Condition 3: High Acceleration**
 *          ```
 *          accel_moving = throttle_mix_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING
 *          ```
 *          - LAND_CHECK_ACCEL_MOVING = 3.0 m/s²
 *          - Filtered acceleration magnitude (gravity removed)
 *          - True if experiencing sustained acceleration > 3 m/s²
 *          - Indicates falling, turbulence, or rapid maneuvering
 *          - Requires attitude authority to recover → use_mix_max = true
 *          
 *          **Condition 4: Not Descending**
 *          ```
 *          descent_not_demanded = pos_control->get_vel_desired_NEU_cms().z >= 0.0f
 *          ```
 *          - NEU frame: North-East-Up (positive Z = up)
 *          - True if desired vertical velocity >= 0 (hover or climb)
 *          - False if desired descent (negative Z velocity)
 *          - Not descending: May need attitude authority → use_mix_max = true
 *          - Descending: Prioritize altitude control for smooth landing
 *          
 *          **Combined Logic:**
 *          ```
 *          use_mix_max = large_angle_request || large_angle_error || 
 *                        accel_moving || descent_not_demanded
 *          ```
 *          
 *          If ANY condition true: Use max mix (attitude priority)
 *          If ALL conditions false: Use min mix (altitude priority)
 *          
 *          **Landing Exception:**
 *          
 *          During VTOL landing sequence, override the normal logic:
 *          ```
 *          if (in_vtol_land_sequence())
 *              use_mix_max = !in_vtol_land_final()
 *          ```
 *          
 *          - **Landing approach**: use_mix_max = true (maintain attitude control)
 *          - **Final landing**: use_mix_max = false (prioritize gentle descent)
 *          
 *          Rationale: Approach phase requires position/attitude authority for
 *          precision landing. Final phase prioritizes smooth vertical descent
 *          over aggressive attitude corrections.
 *          
 *          **Apply Mix Setting:**
 *          ```
 *          if (use_mix_max)
 *              attitude_control->set_throttle_mix_max(1.0)  // Full attitude authority
 *          else
 *              attitude_control->set_throttle_mix_min()     // Altitude priority
 *          ```
 *          
 *          **Effect on Controllers:**
 *          
 *          The throttle mix setting affects AC_AttitudeControl behavior:
 *          
 *          **Mix Max (1.0) - Attitude Priority:**
 *          - Allows full throttle range for attitude corrections
 *          - Can use throttle from 0% to 100% for vehicle rotation
 *          - Better attitude tracking during maneuvers
 *          - May sacrifice altitude accuracy temporarily
 *          - Used during: Aggressive flight, disturbances, climbing
 *          
 *          **Mix Min (0.0) - Altitude Priority:**
 *          - Limits throttle range available for attitude
 *          - Reserves most throttle authority for altitude control
 *          - Better altitude tracking during descent
 *          - May limit attitude correction aggressiveness
 *          - Used during: Gentle descent, landing final, zero throttle
 *          
 *          **Typical Scenarios:**
 *          
 *          **Scenario 1: Aggressive Navigation**
 *          - Waypoint with sharp turn required
 *          - angle_target > 15° → large_angle_request = true
 *          - use_mix_max = true → Full attitude authority
 *          - Result: Crisp turns, may bob vertically
 *          
 *          **Scenario 2: Wind Gust**
 *          - Strong wind tilts vehicle
 *          - angle_error > 30° → large_angle_error = true
 *          - use_mix_max = true → Aggressive correction
 *          - Result: Rapid attitude recovery
 *          
 *          **Scenario 3: Gentle Landing**
 *          - in_vtol_land_final() = true
 *          - use_mix_max = false (landing override)
 *          - set_throttle_mix_min() → Altitude priority
 *          - Result: Smooth vertical descent, less attitude correction
 *          
 *          **Scenario 4: Stable Hover**
 *          - Small angle targets (< 15°)
 *          - Low angle error (< 30°)
 *          - Low acceleration (< 3 m/s²)
 *          - Not descending
 *          - descent_not_demanded = true → use_mix_max = true
 *          - Result: Maintains attitude authority for stability
 *          
 *          **Scenario 5: Controlled Descent**
 *          - Descending at commanded rate
 *          - descent_not_demanded = false
 *          - All other conditions false
 *          - use_mix_max = false → Altitude priority
 *          - Result: Smooth descent rate, less aggressive attitude
 * 
 * @note Called at main loop rate (typically 400 Hz)
 * @note Transition controller may override and manage mix directly
 * @note Landing sequence has special logic for approach vs final descent
 * @note Manual throttle modes use different mix strategy than autopilot modes
 * @note Acceleration filtered to reject vibration and transient noise
 * @note Mix setting affects trade-off between attitude and altitude tracking
 * @note Constants defined: LAND_CHECK_ANGLE_ERROR_DEG (30°), LAND_CHECK_LARGE_ANGLE_CD (1500 = 15°), LAND_CHECK_ACCEL_MOVING (3.0 m/s²)
 * 
 * @warning SAFETY-CRITICAL: Affects vehicle stability and landing behavior
 * @warning Incorrect mix can cause altitude loss or attitude instability
 * @warning Landing mix timing critical for safe touchdown
 * 
 * @see AC_AttitudeControl::set_throttle_mix_max() - Set attitude priority
 * @see AC_AttitudeControl::set_throttle_mix_min() - Set altitude priority
 * @see AC_AttitudeControl::set_throttle_mix_man() - Manual throttle mix
 * @see transition->allow_update_throttle_mix() - Transition override check
 * @see in_vtol_land_sequence() - Landing sequence detection
 * @see in_vtol_land_final() - Final landing phase detection
 * @see air_mode_active() - Air mode status (maintains authority at zero throttle)
 * 
 * Source: ArduPlane/quadplane.cpp:7827-7886
 */
void QuadPlane::update_throttle_mix(void)
{
    // update filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    throttle_mix_accel_ef_filter.apply(accel_ef, plane.scheduler.get_loop_period_s());

    // transition will directly manage the mix
    if (!transition->allow_update_throttle_mix()) {
        return;
    }

    // if disarmed or landed prioritise throttle
    if (!motors->armed()) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (plane.control_mode->is_vtol_man_throttle()) {
        // manual throttle
        if (!is_positive(get_throttle_input()) && !air_mode_active()) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        bool accel_moving = (throttle_mix_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_NEU_cms().z >= 0.0f;

        bool use_mix_max = large_angle_request || large_angle_error || accel_moving || descent_not_demanded;

        /*
          special case for auto landing, we want a high degree of
          attitude control until LAND_FINAL
         */
        if (in_vtol_land_sequence()) {
            use_mix_max = !in_vtol_land_final();
        }

        if (use_mix_max) {
            attitude_control->set_throttle_mix_max(1.0);
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
}

/*
  see if we are in the approach phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_approach(void) const
{
    if (plane.control_mode == &plane.mode_qrtl &&
        poscontrol.get_state() <= QPOS_POSITION2) {
        return true;
    }
    if (in_vtol_auto()) {
        if (is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
            (poscontrol.get_state() == QPOS_APPROACH ||
             poscontrol.get_state() == QPOS_AIRBRAKE ||
             poscontrol.get_state() == QPOS_POSITION1 ||
             poscontrol.get_state() == QPOS_POSITION2)) {
            return true;
        }
    }
    return false;
}

/*
  see if we are in the descent phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_descent(void) const
{
    const auto state = poscontrol.get_state();
    if (plane.control_mode == &plane.mode_qrtl &&
        (state == QPOS_LAND_DESCEND || state == QPOS_LAND_FINAL || state == QPOS_LAND_ABORT)) {
        return true;
    }
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        (state == QPOS_LAND_DESCEND || state == QPOS_LAND_FINAL || state == QPOS_LAND_ABORT)) {
        return true;
    }
    return false;
}

/*
  see if we are in the final phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_final(void) const
{
    return in_vtol_land_descent() && poscontrol.get_state() == QPOS_LAND_FINAL;
}

/*
  see if we are in any of the phases of a VTOL landing
 */
bool QuadPlane::in_vtol_land_sequence(void) const
{
    return plane.control_mode == &plane.mode_qrtl || in_vtol_land_approach() || in_vtol_land_descent() || in_vtol_land_final();
}

/*
  see if we are in the VTOL position control phase of a landing
 */
bool QuadPlane::in_vtol_land_poscontrol(void) const
{
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        poscontrol.get_state() >= QPOS_POSITION1) {
        return true;
    }
    return false;
}

/*
  see if we are in the airbrake phase of a VTOL landing
 */
bool QuadPlane::in_vtol_airbrake(void) const
{
    if (plane.control_mode == &plane.mode_qrtl &&
        poscontrol.get_state() == QPOS_AIRBRAKE) {
        return true;
    }
    if (plane.control_mode == &plane.mode_auto &&
        is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        poscontrol.get_state() == QPOS_AIRBRAKE) {
        return true;
    }
    return false;
}

// return true if we should show VTOL view
bool QuadPlane::show_vtol_view() const
{
    return available() && transition->show_vtol_view() && !force_fw_control_recovery;
}

// return true if we should show VTOL view
bool SLT_Transition::show_vtol_view() const
{

    return quadplane.in_vtol_mode();
}

/*
  return the PILOT_VELZ_MAX_DN value if non zero, otherwise returns the PILOT_VELZ_MAX value.
  return is in cm/s
*/
uint16_t QuadPlane::get_pilot_velocity_z_max_dn() const
{
    if (is_zero(pilot_speed_z_max_dn)) {
        return abs(pilot_speed_z_max_up*100);
    }
    return abs(pilot_speed_z_max_dn*100);
}

/*
  should we use the fixed wing attitude controllers for roll/pitch control
 */
bool QuadPlane::use_fw_attitude_controllers(void) const
{
    if (available() &&
        motors->armed() &&
        motors->get_desired_spool_state() >= AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
        !tailsitter.enabled() &&
        poscontrol.get_state() != QPOS_AIRBRAKE &&
        !force_fw_control_recovery) {

        if (in_vtol_mode()) {
            // in VTOL modes always slave fixed wing to VTOL rate control
            return false;
        }

        if (transition->use_multirotor_control_in_fwd_transition()) {
            /*
              special case for vectored yaw tiltrotors in forward
              transition, keep multicopter control until we reach
              target transition airspeed. This can result in loss of
              yaw control on some tilt-vectored airframes without
              strong VTOL yaw control
            */
            return false;
        }
    }

    return true;
}

/*
  calculate our closing velocity vector on the landing point, taking
  into account target velocity
*/
Vector2f QuadPlane::landing_closing_velocity()
{
    Vector2f landing_velocity;
    if (AP_HAL::millis() - poscontrol.last_velocity_match_ms < 1000) {
        landing_velocity = poscontrol.velocity_match;
    }
    return ahrs.groundspeed_vector() - landing_velocity;
}

/*
  calculate our desired closing velocity vector on the landing point.
*/
Vector2f QuadPlane::landing_desired_closing_velocity()
{
    if (poscontrol.get_state() >= QPOS_LAND_DESCEND) {
        return Vector2f(0,0);
    }
    const Vector2f diff_wp = plane.current_loc.get_distance_NE(plane.next_WP_loc);
    float dist = diff_wp.length();
    if (dist < 1) {
        return Vector2f(0,0);
    }

    // base target speed based on sqrt of distance
    float target_speed = safe_sqrt(2*transition_decel*dist);

    // don't let the target speed go above landing approach speed
    const float eas2tas = plane.ahrs.get_EAS2TAS();
    float land_speed = plane.aparm.airspeed_cruise;
    float tecs_land_airspeed = plane.TECS_controller.get_land_airspeed();
    if (is_positive(tecs_land_airspeed)) {
        land_speed = tecs_land_airspeed;
    } else {
        // use half way between min airspeed and cruise if
        // TECS_LAND_AIRSPEED not set
        land_speed = 0.5*(land_speed+plane.aparm.airspeed_min);
    }
    target_speed = MIN(target_speed, eas2tas * land_speed);

    Vector2f target_speed_xy = diff_wp.normalized() * target_speed;

    return target_speed_xy;
}

/*
  get target airspeed for landing, for use by TECS
*/
float QuadPlane::get_land_airspeed(void)
{
    const auto qstate = poscontrol.get_state();
    if (qstate == QPOS_APPROACH ||
        plane.control_mode == &plane.mode_rtl) {
        const float cruise_speed = plane.aparm.airspeed_cruise;
        // assume cruise speed, but try to do better:
        float approach_speed = cruise_speed;
        float tecs_land_airspeed = plane.TECS_controller.get_land_airspeed();
        if (is_positive(tecs_land_airspeed)) {
            approach_speed = tecs_land_airspeed;
        } else if (qstate == QPOS_APPROACH) {
            // default to half way between min airspeed and cruise
            // airspeed when on the approach
            approach_speed = 0.5*(cruise_speed+plane.aparm.airspeed_min);
        }
        const float time_to_pos1 = (plane.auto_state.wp_distance - stopping_distance(sq(approach_speed))) / MAX(approach_speed, 5);
        /*
          slow down to landing approach speed as we get closer to landing
        */
        approach_speed = linear_interpolate(approach_speed, cruise_speed,
                                            time_to_pos1,
                                            20, 60);
        return approach_speed;
    }

    if (qstate == QPOS_AIRBRAKE) {
        // during airbraking ask TECS to slow us to stall speed
        return plane.aparm.airspeed_min;
    }
    
    // calculate speed based on landing desired velocity
    Vector2f vel = landing_desired_closing_velocity();
    const Vector2f wind = plane.ahrs.wind_estimate().xy();
    const float eas2tas = plane.ahrs.get_EAS2TAS();
    vel -= wind;
    vel /= eas2tas;
    return vel.length();
}

/**
 * @brief Set desired motor spool state with immediate shutdown on request
 * 
 * @details Manages motor spool state transitions (ground idle, spooling up,
 *          throttle unlimited, spooling down, shutdown) with special handling
 *          for emergency shutdowns to avoid slow ramp-down delays.
 *          
 *          **Motor Spool States:**
 *          
 *          **SHUT_DOWN:**
 *          - Motors completely off, no PWM output
 *          - Used when: Disarmed, emergency termination, failsafe action
 *          - Transition from any state immediately stops motors
 *          
 *          **GROUND_IDLE:**
 *          - Motors at minimum throttle (spin slowly)
 *          - Used when: Armed on ground, zero pilot throttle (non-air mode)
 *          - Keeps motors ready for immediate response
 *          - PWM output at Q_M_SPIN_MIN (typically 5-10% throttle)
 *          
 *          **SPOOLING_UP:**
 *          - Transitioning from idle to flight throttle
 *          - Gradual ramp-up to avoid sudden acceleration
 *          - Happens during: Arming delay completion, takeoff initiation
 *          - Ramp time determined by motor library configuration
 *          
 *          **THROTTLE_UNLIMITED:**
 *          - Full throttle range available for flight
 *          - Normal flight state
 *          - Allows 0-100% throttle output based on controller demands
 *          
 *          **SPOOLING_DOWN:**
 *          - Transitioning from flight throttle to idle
 *          - Gradual ramp-down for smooth landing
 *          - Happens during: Landing detection, mode exit
 *          - Slow ramp prevents hard touchdown
 *          
 *          **State Change Logic:**
 *          
 *          ```
 *          if (motors->get_desired_spool_state() != state)
 *          ```
 *          
 *          Only apply change if state is actually different. Avoids:
 *          - Redundant motor library calls
 *          - Unnecessary state machine resets
 *          - Repeated zero-throttle commands
 *          
 *          **Emergency Shutdown Handling:**
 *          
 *          When requesting SHUT_DOWN state:
 *          ```
 *          if (state == AP_Motors::DesiredSpoolState::SHUT_DOWN)
 *              motors->set_roll(0)
 *              motors->set_pitch(0)
 *              motors->set_yaw(0)
 *              motors->set_throttle(0)
 *          ```
 *          
 *          **Rationale:**
 *          
 *          Normal motor spool-down has a gradual ramp for smooth landings.
 *          However, SHUT_DOWN is requested in emergencies:
 *          - Crash detection
 *          - Geofence breach with terminate action
 *          - Advanced failsafe activation
 *          - Manual kill switch
 *          
 *          In these cases, the slow ramp-down is dangerous:
 *          - Delays motor shutdown (vehicle still producing thrust)
 *          - Could cause continued flight after crash
 *          - May worsen collision or runaway
 *          
 *          **Immediate Shutdown Mechanism:**
 *          
 *          Before setting SHUT_DOWN state, zero all control inputs:
 *          - **set_roll(0)**: No roll corrections
 *          - **set_pitch(0)**: No pitch corrections
 *          - **set_yaw(0)**: No yaw corrections
 *          - **set_throttle(0)**: Zero throttle demand
 *          
 *          This forces the motor mixer to calculate zero or minimal output
 *          for all motors immediately, bypassing the normal spool-down ramp.
 *          
 *          Then, set_desired_spool_state(SHUT_DOWN) commands the motor
 *          library to disable outputs completely.
 *          
 *          **Effect:**
 *          - Motors stop within milliseconds instead of 1-2 seconds
 *          - Critical for emergency situations
 *          - Prevents additional unwanted vehicle movement
 *          
 *          **Normal State Transitions (non-SHUT_DOWN):**
 *          
 *          For other state transitions, simply call motor library:
 *          ```
 *          motors->set_desired_spool_state(state)
 *          ```
 *          
 *          The motor library handles normal spool up/down ramps internally.
 *          Examples:
 *          - GROUND_IDLE → THROTTLE_UNLIMITED: Gradual spool up
 *          - THROTTLE_UNLIMITED → GROUND_IDLE: Gradual spool down
 *          
 *          **Common Usage Scenarios:**
 *          
 *          **Scenario 1: Arming Sequence**
 *          - set_desired_spool_state(GROUND_IDLE)
 *          - Motors spin at minimum
 *          - After ARMING_DELAY_MS: SPOOLING_UP
 *          - Then: THROTTLE_UNLIMITED
 *          
 *          **Scenario 2: Takeoff**
 *          - set_desired_spool_state(THROTTLE_UNLIMITED)
 *          - Full throttle range available
 *          - Controllers command vertical climb
 *          
 *          **Scenario 3: Landing**
 *          - During descent: THROTTLE_UNLIMITED
 *          - Ground contact detected: SPOOLING_DOWN
 *          - After stabilization: GROUND_IDLE
 *          - After disarm: SHUT_DOWN
 *          
 *          **Scenario 4: Emergency Termination**
 *          - Crash detected or terminate command
 *          - set_desired_spool_state(SHUT_DOWN)
 *          - Zero all control inputs first
 *          - Motors stop immediately
 *          
 *          **Scenario 5: Manual Throttle Zero (no air mode)**
 *          - Pilot releases throttle stick
 *          - set_desired_spool_state(GROUND_IDLE)
 *          - Motors spin at minimum
 *          - Ready for immediate response
 *          
 *          **Integration with Motor Library:**
 *          
 *          The AP_Motors library manages actual spool state machine:
 *          - Tracks current vs desired state
 *          - Handles ramp timing
 *          - Applies throttle curves
 *          - Manages motor mixing
 *          
 *          This function provides QuadPlane-specific policy:
 *          - When to change states
 *          - Emergency shutdown acceleration
 *          - State persistence across modes
 * 
 * @param[in] state Desired motor spool state
 * 
 * @note Only applies change if state differs from current state
 * @note SHUT_DOWN bypasses normal ramp-down for emergency situations
 * @note Zeros all control inputs before SHUT_DOWN to force immediate stop
 * @note Other state transitions use motor library's internal ramp logic
 * @note Called throughout QuadPlane code during state transitions
 * 
 * @warning SAFETY-CRITICAL: Controls motor behavior during emergencies
 * @warning SHUT_DOWN must stop motors immediately for crash safety
 * @warning Incorrect state can cause loss of control or hard landing
 * 
 * @see AP_Motors::set_desired_spool_state() - Motor library state machine
 * @see AP_Motors::DesiredSpoolState - Spool state enumeration
 * @see motors_output() - Calls this function during normal operation
 * @see afs_terminate() - Emergency termination using SHUT_DOWN
 * @see Q_M_SPIN_MIN - Minimum motor spin parameter (GROUND_IDLE PWM)
 * 
 * Source: ArduPlane/quadplane.cpp:8345-8357
 */
void QuadPlane::set_desired_spool_state(AP_Motors::DesiredSpoolState state)
{
    if (motors->get_desired_spool_state() != state) {
        if (state == AP_Motors::DesiredSpoolState::SHUT_DOWN) {
            // also request zero throttle, so we avoid the slow ramp down
            motors->set_roll(0);
            motors->set_pitch(0);
            motors->set_yaw(0);
            motors->set_throttle(0);
        }
        motors->set_desired_spool_state(state);
    }
}

bool QuadPlane::air_mode_active() const
{
    if ((air_mode == AirMode::ON) || ((air_mode == AirMode::ASSISTED_FLIGHT_ONLY) && assisted_flight)) {
        return true;
    }
    return false;
}

/*
  return scaling factor for tilting rotors in forward flight throttle
  we want to scale back tilt angle for roll/pitch by throttle in forward flight
 */
float QuadPlane::FW_vector_throttle_scaling()
{
    const float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01;
    // scale relative to a fixed 0.5 mid throttle so that changes in TRIM_THROTTLE in missions don't change
    // the scaling of tilt
    const float mid_throttle = 0.5;
    return mid_throttle / constrain_float(throttle, 0.1, 1.0);
}

QuadPlane *QuadPlane::_singleton = nullptr;

bool SLT_Transition::set_FW_roll_limit(int32_t& roll_limit_cd)
{
    if (quadplane.assisted_flight && (transition_state == TRANSITION_AIRSPEED_WAIT || transition_state == TRANSITION_TIMER) &&
        quadplane.option_is_set(QuadPlane::OPTION::LEVEL_TRANSITION)) {
        // the user wants transitions to be kept level to within LEVEL_ROLL_LIMIT
        roll_limit_cd = MIN(roll_limit_cd, plane.g.level_roll_limit*100);
        return true;
    }
    return false;
}

bool SLT_Transition::allow_update_throttle_mix() const
{
    // transition is directly managing throttle mix in these cases
    return !(quadplane.assisted_flight && (transition_state == TRANSITION_AIRSPEED_WAIT || transition_state == TRANSITION_TIMER));
}

bool SLT_Transition::active_frwd() const
{
    // We need to be in assisted flight...
    if (!quadplane.assisted_flight) {
        return false;
    }
    // ... and a transition must be active...
    if (!((transition_state == TRANSITION_AIRSPEED_WAIT) || (transition_state == TRANSITION_TIMER))) {
        return false;
    }
    // ... but not executing a QPOS_AIRBRAKE maneuver during an automated landing.
    if (quadplane.in_vtol_airbrake()) {
        return false;
    }
    
    return true;
}

/*
  limit VTOL roll/pitch in POSITION1, POSITION2 and waypoint controller. This serves three roles:
   1) an expanding envelope limit on pitch to prevent sudden pitch at the start of a back transition

   2) limiting roll and pitch down to the Q_ANGLE_MAX, as the accel limits may push us beyond that for pitch up.
      This is needed as the position controller doesn't have separate limits for pitch and roll

   3) preventing us pitching up a lot when our airspeed may be low
      enough that the real airspeed may be negative, which would result
      in reversed control surfaces
 */
bool SLT_Transition::set_VTOL_roll_pitch_limit(int32_t& roll_cd, int32_t& pitch_cd)
{
    bool ret = false;
    const int16_t angle_max = quadplane.aparm.angle_max;

    /*
      we always limit roll to Q_ANGLE_MAX
     */
    int32_t new_roll_cd = constrain_int32(roll_cd, -angle_max, angle_max);
    if (new_roll_cd != roll_cd) {
        roll_cd = new_roll_cd;
        ret = true;
    }

    /*
      always limit pitch down to Q_ANGLE_MAX. We need to do this as
      the position controller accel limits may exceed this limit
     */
    if (pitch_cd < -angle_max) {
        pitch_cd = -angle_max;
        ret = true;
    }

    /*
      prevent trying to fly backwards (negative airspeed) at high
      pitch angles, which can result in a high degree of instability
      in SLT aircraft. This can happen with a tailwind in a back
      transition, where the position controller (which is unaware of
      airspeed) demands high pitch to hit the desired landing point
     */
    float airspeed;
    if (pitch_cd > angle_max &&
        plane.ahrs.airspeed_estimate(airspeed) && airspeed < 0.5 * plane.aparm.airspeed_min) {
        const float max_limit_cd = linear_interpolate(angle_max, 4500,
                                                      airspeed,
                                                      0, 0.5 * plane.aparm.airspeed_min);
        if (pitch_cd > max_limit_cd) {
            pitch_cd = max_limit_cd;
            ret = true;
        }
    }

    if (quadplane.back_trans_pitch_limit_ms <= 0) {
        // time based pitch envelope disabled
        return ret;
    }

    const uint32_t limit_time_ms = quadplane.back_trans_pitch_limit_ms;

    const uint32_t dt = AP_HAL::millis() - last_fw_mode_ms;
    if (last_fw_mode_ms == 0 || dt > limit_time_ms) {
        // we are beyond the time limit, don't apply envelope
        last_fw_mode_ms = 0;
        return ret;
    }

    // we limit pitch during initial transition
    const float max_limit_cd = linear_interpolate(MAX(last_fw_nav_pitch_cd,0), MIN(angle_max,plane.aparm.pitch_limit_max*100),
                                            dt,
                                            0, limit_time_ms);

    if (pitch_cd > max_limit_cd) {
        pitch_cd = max_limit_cd;
        return true;
    }

    /*
        limit the pitch down with an expanding envelope. This
        prevents the velocity controller demanding nose down during
        the initial slowdown if the target velocity curve is higher
        than the actual velocity curve (for a high drag
        aircraft). Nose down will cause a lot of downforce on the
        wings which will draw a lot of current and also cause the
        aircraft to lose altitude rapidly.pitch limit varies also with speed
        to prevent inability to progress to position if moving from a loiter
        to landing
    */
    const float min_limit_cd = linear_interpolate(MIN(last_fw_nav_pitch_cd,0), MAX(-angle_max,plane.aparm.pitch_limit_min*100),
                                                  dt,
                                                  0, limit_time_ms);

    if (plane.nav_pitch_cd < min_limit_cd) {
        plane.nav_pitch_cd = min_limit_cd;
        return true;
    }

    return ret;
}

/*
  remember last fixed wing pitch for pitch envelope in back transition
 */
void SLT_Transition::set_last_fw_pitch()
{
    last_fw_mode_ms = AP_HAL::millis();
    last_fw_nav_pitch_cd = plane.nav_pitch_cd;
}

void SLT_Transition::force_transition_complete()
{
    transition_state = TRANSITION_DONE;
    in_forced_transition = false;
    transition_start_ms = 0;
    transition_low_airspeed_ms = 0;
    set_last_fw_pitch();

    // Keep assistance reset while not checking
    quadplane.assist.reset();
}

MAV_VTOL_STATE SLT_Transition::get_mav_vtol_state() const
{
    if (quadplane.in_vtol_mode()) {
        QuadPlane::position_control_state state = quadplane.poscontrol.get_state();
        if ((state == QuadPlane::position_control_state::QPOS_AIRBRAKE) || (state == QuadPlane::position_control_state::QPOS_POSITION1)) {
            return MAV_VTOL_STATE_TRANSITION_TO_MC;
        }
        return MAV_VTOL_STATE_MC;
    }

    switch (transition_state) {
        case TRANSITION_AIRSPEED_WAIT:
        case TRANSITION_TIMER:
            // we enter this state during assisted flight, not just
            // during a forward transition.
            return MAV_VTOL_STATE_TRANSITION_TO_FW;

        case TRANSITION_DONE:
            return MAV_VTOL_STATE_FW;
    }

    return MAV_VTOL_STATE_UNDEFINED;
}

// Set FW roll and pitch limits and keep TECS informed
void SLT_Transition::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_airbrake()) {
        // not in FW flight
        return;
    }

    if (transition_state == TRANSITION_DONE) {
        // transition complete, nothing to do
        return;
    }

    if (!plane.control_mode->does_auto_throttle()) {
        // don't limit pitch when in manually controlled modes like FBWA, ACRO
        return;
    }

    float max_pitch;
    if (transition_state < TRANSITION_TIMER) {
        if (plane.ahrs.groundspeed() < 3.0) {
            // until we have some ground speed limit to zero pitch
            max_pitch = 0.0;
        } else {
            max_pitch = quadplane.transition_pitch_max;
        }
    } else {
        max_pitch = (quadplane.transition_pitch_max+1.0)*2.0;
    }

    // set a single loop pitch limit in TECS
    plane.TECS_controller.set_pitch_max(max_pitch);
    plane.TECS_controller.set_pitch_min(-max_pitch);

    // ensure pitch is constrained to limit
    nav_pitch_cd = constrain_int32(nav_pitch_cd, -max_pitch*100.0, max_pitch*100.0);
}

/*
  see if we are in a VTOL takeoff
 */
bool QuadPlane::in_vtol_takeoff(void) const
{
    if (in_vtol_auto() && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return true;
    }
    return false;
}

/**
 * @brief Reset QuadPlane state on mode change (called for ALL modes, not just VTOL)
 * 
 * @details Performs comprehensive state reset when vehicle changes flight mode.
 *          Called by Plane mode exit/enter logic for ANY mode change, including
 *          transitions between fixed-wing modes, VTOL modes, or mixed modes.
 *          Ensures clean state for new mode without lingering artifacts from
 *          previous mode's operation.
 *          
 *          **Design Rationale:**
 *          
 *          Mode changes can occur in many scenarios:
 *          - Pilot manual mode switch (RC transmitter)
 *          - GCS commanded mode change (MAVLink)
 *          - Failsafe mode transition (e.g., RTL)
 *          - Mission mode transitions (AUTO segments)
 *          - Mode-specific exit conditions (land complete, takeoff complete)
 *          
 *          Each mode may have configured QuadPlane parameters that shouldn't
 *          persist into the next mode. Without reset:
 *          - Position corrections could apply to wrong mode
 *          - Velocity matching could affect transitions
 *          - Guided takeoff state could confuse other modes
 *          - Forward throttle could interfere with pure VTOL
 *          
 *          **State Reset Operations:**
 *          
 *          **1. Position Control Angle Limits:**
 *          ```
 *          if (available())
 *              pos_control->set_lean_angle_max_cd(0)
 *          ```
 *          
 *          Reset to zero (will be set by new mode's controller):
 *          - Prevents old mode's angle limits affecting new mode
 *          - Each mode sets appropriate limit (LOITER vs AUTO vs GUIDED)
 *          - Zero = use system defaults until mode sets specific value
 *          - Only reset if QuadPlane available (Q_ENABLE set)
 *          
 *          **2. Horizontal Position Corrections:**
 *          ```
 *          poscontrol.xy_correction.zero()
 *          ```
 *          
 *          Clear accumulated position error corrections:
 *          - xy_correction tracks position error integration
 *          - Old position targets irrelevant to new mode
 *          - Prevents jump in position controller output
 *          - Starts new mode with clean horizontal tracking
 *          
 *          **3. Velocity Matching State:**
 *          ```
 *          poscontrol.velocity_match.zero()
 *          poscontrol.last_velocity_match_ms = 0
 *          ```
 *          
 *          Clear velocity matching for position transitions:
 *          - velocity_match tracks desired velocity for smooth transitions
 *          - Used when switching from waypoint nav to loiter
 *          - Old velocities don't apply to new mode
 *          - Timestamp reset prevents stale data usage
 *          
 *          **4. Position Control State Machine:**
 *          ```
 *          poscontrol.set_state(QuadPlane::QPOS_NONE)
 *          ```
 *          
 *          Reset position controller state to NONE:
 *          - QPOS_NONE = position controller inactive
 *          - Mode will set appropriate state (QPOS_POSITION, QPOS_LAND_*, etc.)
 *          - Prevents old state machine state affecting new mode
 *          - Clean entry to new mode's position control logic
 *          
 *          **5. Pilot Repositioning Corrections:**
 *          ```
 *          poscontrol.pilot_correction_done = false
 *          poscontrol.pilot_correction_active = false
 *          poscontrol.target_vel_cms.zero()
 *          ```
 *          
 *          Clear pilot stick repositioning state:
 *          - pilot_correction_active: Pilot currently adjusting position
 *          - pilot_correction_done: Position adjustment completed
 *          - target_vel_cms: Desired velocity from pilot stick input
 *          
 *          Used in QLAND with Q_OPTIONS bit REPOSITION_LANDING:
 *          - Allows pilot to adjust landing position during descent
 *          - Should not persist between modes
 *          - New mode gets fresh pilot input processing
 *          
 *          **6. Guided Takeoff State:**
 *          ```
 *          guided_wait_takeoff_on_mode_enter = guided_wait_takeoff
 *          guided_wait_takeoff = false
 *          ```
 *          
 *          Clear guided takeoff wait flag with state preservation:
 *          - guided_wait_takeoff: Waiting for takeoff command in GUIDED
 *          - guided_wait_takeoff_on_mode_enter: Remembers state for logic
 *          
 *          Purpose: Detect if mode was entered with takeoff pending
 *          - Allows special behavior for mode switches during takeoff prep
 *          - Prevents takeoff state from affecting non-GUIDED modes
 *          - GUIDED mode can check on_mode_enter to resume takeoff logic
 *          
 *          **7. Forward Throttle State:**
 *          ```
 *          q_fwd_throttle = 0.0f
 *          q_fwd_pitch_lim_cd = 100.0f * q_fwd_pitch_lim
 *          ```
 *          
 *          Reset forward assist throttle and pitch limits:
 *          - q_fwd_throttle: Forward motor throttle (0-1), zero = no assist
 *          - q_fwd_pitch_lim_cd: Pitch limit in centidegrees
 *          
 *          Forward throttle assist used during:
 *          - VTOL forward transitions
 *          - Assisted forward flight
 *          - Tilt-rotor configurations
 *          
 *          Reset prevents:
 *          - Unwanted forward thrust in hover modes
 *          - Pitch limit interference in pure VTOL
 *          - Transition state carrying over to incompatible mode
 *          
 *          Pitch limit reset to default (Q_FW_PITCH_LIM parameter):
 *          - Will be adjusted by transition/assist logic as needed
 *          - Prevents restrictive limit affecting new mode
 *          
 *          **8. Fixed-Wing Recovery Flags:**
 *          ```
 *          force_fw_control_recovery = false
 *          in_spin_recovery = false
 *          ```
 *          
 *          Clear emergency recovery state flags:
 *          
 *          **force_fw_control_recovery:**
 *          - Forced fixed-wing control during VTOL upset
 *          - Used when VTOL controllers unable to recover
 *          - Should not persist across mode changes
 *          - New mode gets fresh recovery assessment
 *          
 *          **in_spin_recovery:**
 *          - Active spin recovery maneuver
 *          - Special control logic during spin
 *          - Must not carry over to new mode
 *          - New mode should detect spin independently
 *          
 *          **Mode Entry Sequence:**
 *          
 *          1. Old mode calls mode_exit()
 *          2. Plane framework calls this QuadPlane::mode_enter()
 *          3. New mode calls mode_enter()
 *          4. New mode initializes its specific state
 *          5. First update() call executes new mode logic
 *          
 *          **Important Characteristics:**
 *          
 *          **Called for ALL modes:**
 *          - Fixed-wing modes: MANUAL, STABILIZE, FBWA, CRUISE, etc.
 *          - VTOL modes: QSTABILIZE, QHOVER, QLOITER, etc.
 *          - Auto modes: AUTO, GUIDED, RTL (both FW and VTOL segments)
 *          
 *          **Why universal reset:**
 *          - Ensures consistency regardless of mode transition
 *          - Prevents "hidden state" bugs from rare mode combinations
 *          - Single location for all QuadPlane mode entry reset
 *          - Simplifies mode-specific logic (clean slate guaranteed)
 *          
 *          **Thread Safety:**
 *          - Called from main loop (single-threaded)
 *          - No concurrent access to reset state variables
 *          - No semaphore protection needed
 * 
 * @note Called for EVERY mode change, not just VTOL modes
 * @note Executed after old mode exit, before new mode enters
 * @note Provides clean slate for new mode's QuadPlane state
 * @note Does not reset vehicle state (position, velocity, attitude)
 * @note Does not affect motor outputs directly (only controller state)
 * @note Guided takeoff state preserved in guided_wait_takeoff_on_mode_enter
 * @note Forward throttle and pitch limits reset to safe defaults
 * @note Position controller state machine reset to QPOS_NONE
 * @note All pilot corrections and velocity matching cleared
 * 
 * @warning Do not add operations with side effects beyond state reset
 * @warning Motor outputs should remain unchanged by this function
 * @warning Do not initiate control actions, only clear old state
 * 
 * @see Plane::set_mode() - Main mode change orchestration
 * @see Mode::enter() - Individual mode entry (called after this)
 * @see Q_OPTIONS - QuadPlane option bits (REPOSITION_LANDING, etc.)
 * @see poscontrol.set_state() - Position control state machine
 * @see Q_FW_PITCH_LIM - Forward flight pitch limit parameter
 * 
 * Source: ArduPlane/quadplane.cpp:8780-8802
 */
void QuadPlane::mode_enter(void)
{
    if (available()) {
        pos_control->set_lean_angle_max_cd(0);
    }
    poscontrol.xy_correction.zero();
    poscontrol.velocity_match.zero();
    poscontrol.last_velocity_match_ms = 0;
    poscontrol.set_state(QuadPlane::QPOS_NONE);

    // Clear any pilot corrections
    poscontrol.pilot_correction_done = false;
    poscontrol.pilot_correction_active = false;
    poscontrol.target_vel_cms.zero();

    // clear guided takeoff wait on any mode change, but remember the
    // state for special behaviour
    guided_wait_takeoff_on_mode_enter = guided_wait_takeoff;
    guided_wait_takeoff = false;

    q_fwd_throttle = 0.0f;
    q_fwd_pitch_lim_cd = 100.0f * q_fwd_pitch_lim;

    force_fw_control_recovery = false;
    in_spin_recovery = false;
}

/**
 * @brief Set attitude control yaw rate time constant to pilot input command model value
 * 
 * @details Configures yaw rate controller time constant to match pilot input
 *          command model, providing consistent response feel across control axes.
 *          The time constant determines how quickly yaw rate tracks commanded
 *          rate, affecting yaw responsiveness and "feel" during manual flight.
 *          
 *          **Time Constant Concept:**
 *          
 *          Time constant (TC) in control systems represents the time for response
 *          to reach 63.2% of final value in first-order system:
 *          ```
 *          output(t) = final_value * (1 - e^(-t/TC))
 *          ```
 *          
 *          **Smaller TC (e.g., 0.1s):**
 *          - Faster response to pilot input
 *          - More aggressive, "twitchy" feel
 *          - Better for acrobatic flight
 *          - Can amplify pilot input noise
 *          
 *          **Larger TC (e.g., 0.5s):**
 *          - Slower, smoother response
 *          - More "damped" feel
 *          - Better for stable filming/photography
 *          - Filters out rapid stick movements
 *          
 *          **Command Model Integration:**
 *          
 *          ```
 *          attitude_control->set_yaw_rate_tc(command_model_pilot.get_rate_tc())
 *          ```
 *          
 *          Sources time constant from pilot command model:
 *          - command_model_pilot: Pilot input shaping/filtering model
 *          - get_rate_tc(): Returns configured time constant
 *          - Applied to yaw axis for consistency
 *          
 *          **Why Match Command Model:**
 *          
 *          The command model shapes pilot stick inputs for all axes (roll, pitch, yaw).
 *          Using the same time constant for yaw rate control ensures:
 *          - Consistent response feel across all axes
 *          - Predictable vehicle behavior during combined maneuvers
 *          - Uniform "sluggishness" or "snappiness" setting
 *          
 *          Example: If pilot prefers smooth, cinematic flight:
 *          - Command model TC set to 0.4s
 *          - This function applies 0.4s to yaw rate controller
 *          - Result: All axes respond smoothly and consistently
 *          
 *          **Parameter Source:**
 *          
 *          Typically controlled by tuning parameters:
 *          - Q_RC_EXPO_TC or similar (command model time constant)
 *          - Affects how stick inputs are shaped
 *          - Range usually 0.1s to 0.5s
 *          
 *          **Usage Context:**
 *          
 *          Called when:
 *          - Entering manual VTOL modes (QSTABILIZE, QACRO, QLOITER)
 *          - Pilot has direct yaw rate control
 *          - Want consistent response with command model
 *          
 *          Not called when:
 *          - Autonomous yaw control (AUTO, GUIDED waypoints)
 *          - Yaw rate commanded by navigation controllers
 *          - disable_yaw_rate_time_constant() used instead
 *          
 *          **Effect on Control:**
 *          
 *          With time constant enabled (this function):
 *          - Pilot yaw stick → command model → shaped yaw rate
 *          - Yaw rate controller tracks with TC filtering
 *          - Smooth, consistent response matching other axes
 *          
 *          With time constant disabled (0.0):
 *          - Commanded yaw rate tracked aggressively
 *          - No additional filtering/smoothing
 *          - Used for autonomous, pre-filtered commands
 * 
 * @note Only affects yaw rate control, not angle control
 * @note Time constant from command_model_pilot (pilot input shaping)
 * @note Provides consistent feel across roll/pitch/yaw axes
 * @note Called when pilot has direct yaw rate authority
 * @note Typically in range 0.1s to 0.5s
 * 
 * @see AC_AttitudeControl::set_yaw_rate_tc() - Set yaw rate time constant
 * @see command_model_pilot.get_rate_tc() - Get pilot command model TC
 * @see disable_yaw_rate_time_constant() - Disable TC for autonomous control
 * 
 * Source: ArduPlane/quadplane.cpp:9004-9007
 */
void QuadPlane::set_pilot_yaw_rate_time_constant()
{
    attitude_control->set_yaw_rate_tc(command_model_pilot.get_rate_tc());
}

/**
 * @brief Disable attitude control yaw rate time constant for autonomous control
 * 
 * @details Sets yaw rate controller time constant to zero, disabling response
 *          filtering for aggressive tracking of autonomous yaw rate commands.
 *          Used when yaw rate is commanded by navigation or guidance controllers
 *          rather than pilot stick input.
 *          
 *          **Zero Time Constant Behavior:**
 *          
 *          Setting TC = 0.0 effectively disables first-order filtering:
 *          ```
 *          output(t) = commanded_rate  // Immediate tracking
 *          ```
 *          
 *          Result:
 *          - Yaw rate controller tracks commanded rate aggressively
 *          - No smoothing or damping of rate commands
 *          - Fastest possible response to yaw rate changes
 *          - Suitable for pre-filtered autonomous commands
 *          
 *          **Rationale for Disabling:**
 *          
 *          **Autonomous Navigation:**
 *          Navigation controllers (waypoint following, orbit, loiter) generate
 *          smooth, filtered yaw rate commands internally:
 *          - Already optimized for vehicle dynamics
 *          - Don't need additional filtering
 *          - Time constant would add unnecessary lag
 *          
 *          **Precision Maneuvers:**
 *          Some maneuvers require precise yaw rate tracking:
 *          - Coordinated turns during waypoint nav
 *          - Yaw-to-target in GUIDED mode
 *          - Transition yaw coordination
 *          - Additional filtering reduces accuracy
 *          
 *          **Pre-Computed Trajectories:**
 *          When yaw rate is part of trajectory plan:
 *          - Time constant would deviate from plan
 *          - Could cause coordination errors
 *          - Direct tracking maintains trajectory fidelity
 *          
 *          **Contrast with Pilot Input:**
 *          
 *          **Pilot Yaw (TC enabled):**
 *          - Stick input can be noisy, rapid
 *          - Benefit from smoothing
 *          - Time constant improves feel
 *          - Prevents overcorrection
 *          
 *          **Autonomous Yaw (TC disabled, this function):**
 *          - Commands already smooth and optimized
 *          - No noise to filter
 *          - Want precise, immediate tracking
 *          - Lag would reduce performance
 *          
 *          **Usage Context:**
 *          
 *          Called when entering or during:
 *          - AUTO mode waypoint navigation
 *          - GUIDED mode position/velocity/angle commands
 *          - RTL and other autonomous return modes
 *          - Any mode where navigation controller commands yaw
 *          
 *          Not called during:
 *          - Manual VTOL modes (QSTABILIZE, QACRO)
 *          - QLOITER when pilot controls yaw
 *          - set_pilot_yaw_rate_time_constant() used instead
 *          
 *          **Impact on Performance:**
 *          
 *          **With TC (pilot modes):**
 *          ```
 *          Pilot stick → Command model → Yaw rate cmd → TC filter → Controller
 *          Result: Smooth, natural feel, reduced sensitivity
 *          ```
 *          
 *          **Without TC (autonomous, this function):**
 *          ```
 *          Nav controller → Yaw rate cmd → Controller (no filter)
 *          Result: Precise tracking, immediate response
 *          ```
 *          
 *          **Example Scenario:**
 *          
 *          **Waypoint Navigation:**
 *          1. Vehicle approaching waypoint at 45° to right
 *          2. Navigation controller commands yaw rate to align with next waypoint
 *          3. This function called (TC = 0.0)
 *          4. Yaw rate controller tracks command aggressively
 *          5. Vehicle rotates smoothly through calculated trajectory
 *          6. No lag between commanded and actual yaw rate
 *          7. Precise heading at waypoint arrival
 *          
 *          **Contrast with Pilot Yaw:**
 *          If TC was enabled (pilot value 0.3s):
 *          1. Navigation yaw rate command issued
 *          2. TC filtering adds 0.3s lag
 *          3. Yaw rate lags behind command
 *          4. Vehicle arrives at waypoint with heading error
 *          5. Position controller must compensate
 *          6. Less efficient navigation
 * 
 * @note Sets yaw rate time constant to exactly 0.0 (disabled)
 * @note Used for autonomous navigation and guidance modes
 * @note Enables aggressive tracking of smooth autonomous commands
 * @note Opposite of set_pilot_yaw_rate_time_constant()
 * @note Navigation controllers generate pre-filtered yaw rates
 * @note No additional smoothing or lag desired for autonomous yaw
 * 
 * @see AC_AttitudeControl::set_yaw_rate_tc() - Set yaw rate time constant
 * @see set_pilot_yaw_rate_time_constant() - Enable TC for pilot control
 * 
 * Source: ArduPlane/quadplane.cpp:9009-9012
 */
void QuadPlane::disable_yaw_rate_time_constant()
{
    attitude_control->set_yaw_rate_tc(0.0);
}

// Check if servo auto trim is allowed, only if countrol surfaces are fully in use
bool QuadPlane::allow_servo_auto_trim()
{
    if (!available()) {
        // Quadplane disabled, auto trim always allowed
        return true;
    }
    if (in_vtol_mode()) {
        // VTOL motors active in VTOL modes
        return false;
    }
    if (!in_assisted_flight()) {
        // In forward flight and VTOL motors not active
        return true;
    }
    if (tailsitter.enabled() && option_is_set(QuadPlane::OPTION::TAILSIT_Q_ASSIST_MOTORS_ONLY)) {
        // Tailsitter in forward flight, motors providing active stabalisation with motors only option
        // Control surfaces are running as normal with I term active, motor I term is zeroed
        return true;
    }
    // In forward flight with active VTOL motors
    return false;
}

bool QuadPlane::landing_with_fixed_wing_spiral_approach(void) const
{
    const AP_Mission::Mission_Command cmd = plane.mission.get_current_nav_cmd();

    if (cmd.id == MAV_CMD_NAV_PAYLOAD_PLACE &&
        option_is_set(QuadPlane::OPTION::MISSION_LAND_FW_APPROACH)) {
        return true;
    }
    
    return ((cmd.id == MAV_CMD_NAV_VTOL_LAND) &&
            (option_is_set(QuadPlane::OPTION::MISSION_LAND_FW_APPROACH) ||
             cmd.p1 == NAV_VTOL_LAND_OPTIONS_FW_SPIRAL_APPROACH));
}

/**
 * @brief Scale VTOL angle P gains to match fixed-wing gains at high airspeed
 * 
 * @details Dynamically adjusts multicopter attitude controller roll and pitch
 *          angle P gains based on airspeed to provide smooth control authority
 *          transition between VTOL motors and fixed-wing control surfaces.
 *          Critical for hybrid flight modes where both control methods active.
 *          
 *          **Design Rationale:**
 *          
 *          In hybrid QuadPlane configurations (especially tailsitters, tiltrotors,
 *          and VTOL+wing vehicles), control authority comes from two sources:
 *          
 *          **Low Speed (VTOL regime):**
 *          - Multicopter motors provide roll/pitch control via differential thrust
 *          - Fixed-wing surfaces ineffective (insufficient airflow)
 *          - Use standard multicopter angle P gains (Q_A_RAT_*_P parameters)
 *          
 *          **High Speed (Fixed-wing regime):**
 *          - Fixed-wing surfaces (ailerons, elevator) dominate control authority
 *          - Multicopter motor differential thrust still applied
 *          - If MC gains unchanged, combined authority too high → oscillations
 *          - Need to reduce MC gains to match FW surface effectiveness
 *          
 *          **Problem Without Scaling:**
 *          ```
 *          Low speed:  MC motors provide roll control (appropriate gain)
 *          High speed: MC motors + FW ailerons both provide roll control
 *                      → Double control authority
 *                      → Overcorrection, oscillations, instability
 *          ```
 *          
 *          **Solution:**
 *          Scale down MC angle P gains at high speed to compensate for FW
 *          surface contribution, maintaining consistent total control authority.
 *          
 *          **Gain Extraction:**
 *          
 *          **1. Get Multicopter Angle P Gains:**
 *          ```cpp
 *          mc_angR = attitude_control->get_angle_roll_p().kP()
 *          mc_angP = attitude_control->get_angle_pitch_p().kP()
 *          ```
 *          
 *          - Q_A_RAT_RLL_P: Roll angle P gain (typically 4.5-6.0)
 *          - Q_A_RAT_PIT_P: Pitch angle P gain (typically 4.5-6.0)
 *          - Base gains tuned for pure VTOL flight
 *          - Determines aggressiveness of angle corrections via motor mixing
 *          
 *          **2. Calculate Fixed-Wing Equivalent Angle P Gains:**
 *          ```cpp
 *          fw_angR = 1.0 / plane.rollController.tau()
 *          fw_angP = 1.0 / plane.pitchController.tau()
 *          ```
 *          
 *          **Time Constant to P Gain Conversion:**
 *          
 *          Fixed-wing controllers use time constants (τ, tau) not P gains:
 *          - tau: Time constant for first-order response (seconds)
 *          - Smaller tau → faster response → higher equivalent P gain
 *          - Relationship: P_gain ≈ 1/tau for first-order systems
 *          
 *          Example:
 *          - PTCH_RATE_I/PTCH_RATE_D determine tau_pitch
 *          - Typical tau = 0.2 to 0.5 seconds
 *          - Equivalent P gain = 1/0.3 = 3.3 (approximately)
 *          
 *          This conversion allows comparing FW and MC gain magnitudes.
 *          
 *          **3. Validate Configuration:**
 *          ```cpp
 *          if (!is_positive(mc_angR) || !is_positive(mc_angP))
 *              return  // Bad configuration, no scaling
 *          ```
 *          
 *          Ensure MC gains are positive and valid:
 *          - Zero or negative gains indicate configuration error
 *          - Scaling would be meaningless or dangerous
 *          - Exit without modifying gains
 *          
 *          **Airspeed-Based Scaling:**
 *          
 *          **1. Get Current Airspeed:**
 *          ```cpp
 *          if (!ahrs.airspeed_estimate(aspeed))
 *              return  // Can't get airspeed, no scaling
 *          ```
 *          
 *          Airspeed sources (in priority order):
 *          - Airspeed sensor (if available and healthy)
 *          - Synthetic airspeed from GPS + wind estimate
 *          - EKF airspeed estimate
 *          
 *          If no airspeed available:
 *          - Unable to determine control regime
 *          - Keep default MC gains (safe conservative approach)
 *          - Better to have full MC authority than risk insufficient control
 *          
 *          **2. Define Interpolation Bounds:**
 *          ```cpp
 *          low_airspeed = 3.0  // m/s (10.8 km/h, 6.7 mph)
 *          ```
 *          
 *          **Low Airspeed Threshold (3.0 m/s):**
 *          - Below this: Pure VTOL control (no FW surface effectiveness)
 *          - Use 100% MC gains, 0% FW scaling
 *          - Rationale: 3 m/s insufficient for meaningful aerodynamic control
 *          
 *          **High Airspeed Threshold (AIRSPEED_MIN parameter):**
 *          - Above this: FW surfaces fully effective
 *          - Scale MC gains to match FW gains
 *          - Typically 10-15 m/s for most aircraft
 *          - Parameter: ARSPD_FBW_MIN
 *          
 *          **3. Check Airspeed Validity:**
 *          ```cpp
 *          if (aspeed <= low_airspeed || plane.aparm.airspeed_min <= low_airspeed)
 *              return  // No scaling needed
 *          ```
 *          
 *          Exit without scaling if:
 *          - Current airspeed ≤ 3 m/s: In pure VTOL regime
 *          - AIRSPEED_MIN ≤ 3 m/s: Invalid configuration (too low for FW flight)
 *          
 *          **4. Calculate Scale Factors:**
 *          ```cpp
 *          angR_scale = linear_interpolate(mc_angR, fw_angR, 
 *                                          aspeed,
 *                                          low_airspeed, airspeed_min) / mc_angR
 *          ```
 *          
 *          **Linear Interpolation Formula:**
 *          ```
 *          result = mc_angR + (fw_angR - mc_angR) * (aspeed - 3.0) / (airspeed_min - 3.0)
 *          ```
 *          
 *          **Interpolation Behavior:**
 *          
 *          At aspeed = 3.0 m/s (low end):
 *          ```
 *          result = mc_angR + (fw_angR - mc_angR) * 0 = mc_angR
 *          scale = mc_angR / mc_angR = 1.0  → No scaling
 *          ```
 *          
 *          At aspeed = airspeed_min (high end):
 *          ```
 *          result = mc_angR + (fw_angR - mc_angR) * 1.0 = fw_angR
 *          scale = fw_angR / mc_angR  → Match FW gain
 *          ```
 *          
 *          At aspeed = midpoint:
 *          ```
 *          result = (mc_angR + fw_angR) / 2
 *          scale = result / mc_angR  → Blend both gains
 *          ```
 *          
 *          **Example Calculation:**
 *          
 *          Given:
 *          - mc_angR = 6.0 (Q_A_RAT_RLL_P)
 *          - fw_angR = 1/0.3 = 3.33 (from roll tau)
 *          - low_airspeed = 3.0 m/s
 *          - airspeed_min = 13.0 m/s
 *          - aspeed = 10.0 m/s (current)
 *          
 *          Calculation:
 *          ```
 *          interpolated = 6.0 + (3.33 - 6.0) * (10 - 3) / (13 - 3)
 *                       = 6.0 + (-2.67) * (7 / 10)
 *                       = 6.0 - 1.87 = 4.13
 *          angR_scale = 4.13 / 6.0 = 0.688
 *          ```
 *          
 *          Result: Roll gain scaled to 68.8% at 10 m/s
 *          
 *          **5. Apply Gain Scaling:**
 *          ```cpp
 *          gain_scale = {angR_scale, angP_scale, 1.0}
 *          attitude_control->set_angle_P_scale(gain_scale)
 *          ```
 *          
 *          **Vector Components:**
 *          - X (roll): angR_scale (airspeed-dependent)
 *          - Y (pitch): angP_scale (airspeed-dependent)
 *          - Z (yaw): 1.0 (unchanged, no FW yaw surfaces or less relevant)
 *          
 *          **Why Yaw = 1.0:**
 *          - Fixed-wing yaw control (rudder) less directly comparable to MC yaw
 *          - Yaw dynamics very different between VTOL and FW regimes
 *          - Yaw typically not as critical for stability (weathervaning acceptable)
 *          - Keep full MC yaw authority at all speeds
 *          
 *          **Effect on Attitude Control:**
 *          
 *          The attitude controller applies scale to angle P gains internally:
 *          ```
 *          effective_roll_P = Q_A_RAT_RLL_P * angR_scale
 *          effective_pitch_P = Q_A_RAT_PIT_P * angP_scale
 *          ```
 *          
 *          Scaled gains used to calculate rate targets from angle errors:
 *          ```
 *          desired_roll_rate = angle_error_roll * effective_roll_P
 *          ```
 *          
 *          **Flight Regime Examples:**
 *          
 *          **Scenario 1: Hover (2 m/s airspeed)**
 *          - aspeed < low_airspeed → function returns early
 *          - Scale = {1.0, 1.0, 1.0} (unchanged)
 *          - Full MC angle P gains active
 *          - Result: Normal VTOL stability
 *          
 *          **Scenario 2: Transition (8 m/s airspeed)**
 *          - Between 3 m/s and airspeed_min (13 m/s)
 *          - Interpolation: ~50% toward FW gains
 *          - Scale ≈ {0.78, 0.78, 1.0}
 *          - MC gains reduced by 22%
 *          - Result: Balanced control authority, smooth transition
 *          
 *          **Scenario 3: Fast Forward Flight (18 m/s airspeed)**
 *          - Above airspeed_min (13 m/s)
 *          - Extrapolation continues beyond airspeed_min
 *          - Scale ≈ {0.55, 0.55, 1.0} (example)
 *          - MC gains significantly reduced
 *          - Result: FW surfaces dominate, MC provides minor corrections
 *          
 *          **Scenario 4: Configuration Error**
 *          - mc_angP = 0 (invalid parameter)
 *          - Function returns early
 *          - Scale unchanged from previous call
 *          - Result: Safe degraded operation
 *          
 *          **Scenario 5: No Airspeed Sensor**
 *          - ahrs.airspeed_estimate() returns false
 *          - Function returns early
 *          - Scale = {1.0, 1.0, 1.0} (full MC gains)
 *          - Result: Conservative control (full VTOL authority)
 *          
 *          **Integration with Control Pipeline:**
 *          
 *          ```
 *          Main Loop (400 Hz):
 *              1. Read sensors (airspeed, attitude)
 *              2. Mode update() called
 *              3. If hybrid flight mode:
 *                  setup_rp_fw_angle_gains()  ← This function
 *              4. Attitude controller calculates corrections
 *                  - Uses scaled angle P gains
 *                  - Converts angle error → rate targets
 *              5. Rate controller generates motor commands
 *              6. Motor mixer outputs to motors
 *              7. Fixed-wing controller outputs to surfaces
 *          ```
 *          
 *          **When This Function Called:**
 *          
 *          - During assisted fixed-wing flight (Q_ASSIST active)
 *          - During forward transitions
 *          - In VTOL modes with forward flight (QLOITER moving)
 *          - Any mode where both VTOL and FW control active
 *          
 *          **Not Called During:**
 *          - Pure VTOL hover modes (no FW control surfaces active)
 *          - Pure fixed-wing modes (VTOL motors not active)
 *          - Transitions where scaling handled differently
 *          
 *          **Tuning Implications:**
 *          
 *          **Multicopter Angle P Gains (Q_A_RAT_RLL_P, Q_A_RAT_PIT_P):**
 *          - Tuned for pure VTOL flight
 *          - Higher values → more aggressive angle corrections
 *          - This function scales them down at speed
 *          
 *          **Fixed-Wing Time Constants (PTCH/RLL tau):**
 *          - Affect equivalent FW gains (1/tau)
 *          - Larger tau → slower FW response → less MC scaling needed
 *          - Smaller tau → faster FW response → more MC scaling needed
 *          
 *          **Transition Tuning:**
 *          - ARSPD_FBW_MIN sets upper bound of scaling region
 *          - Lower ARSPD_FBW_MIN → scaling starts at lower speed
 *          - Higher ARSPD_FBW_MIN → longer transition region
 *          
 *          **Common Issues:**
 *          
 *          **Oscillations at High Speed:**
 *          - Insufficient scaling (MC gains still too high)
 *          - Increase FW tau (decrease 1/tau) to scale MC more
 *          - Or: Reduce MC angle P gains overall
 *          
 *          **Poor Control at Moderate Speed:**
 *          - Excessive scaling (MC gains too low)
 *          - Decrease FW tau (increase 1/tau) to scale MC less
 *          - Or: Adjust ARSPD_FBW_MIN to shift interpolation region
 *          
 *          **Abrupt Behavior at Transition:**
 *          - ARSPD_FBW_MIN too close to low_airspeed (3 m/s)
 *          - Increase ARSPD_FBW_MIN for gentler interpolation slope
 *          - Typical recommendation: ARSPD_FBW_MIN ≥ 10 m/s
 * 
 * @note Called at main loop rate during hybrid flight modes
 * @note Scales only roll and pitch angle P gains, not yaw
 * @note Interpolates linearly between 3 m/s and ARSPD_FBW_MIN
 * @note No scaling applied if airspeed unavailable (safe default)
 * @note No scaling applied if airspeed < 3 m/s (pure VTOL)
 * @note FW gains calculated as 1/tau from rate controllers
 * @note Scaling provides smooth control authority transition
 * @note Critical for stability in assisted and transition flight
 * @note Scale factors: {angR_scale, angP_scale, 1.0} vector
 * 
 * @warning SAFETY-CRITICAL: Affects attitude control stability
 * @warning Incorrect scaling can cause oscillations or loss of control
 * @warning Requires proper tuning of both MC and FW gains
 * @warning Invalid gain configuration silently ignored (returns early)
 * 
 * @see AC_AttitudeControl::set_angle_P_scale() - Apply gain scaling vector
 * @see AC_AttitudeControl::get_angle_roll_p() - Get roll angle P gain
 * @see AC_AttitudeControl::get_angle_pitch_p() - Get pitch angle P gain
 * @see plane.rollController.tau() - FW roll time constant
 * @see plane.pitchController.tau() - FW pitch time constant
 * @see ahrs.airspeed_estimate() - Get current airspeed estimate
 * @see linear_interpolate() - Interpolation utility function
 * @see Q_A_RAT_RLL_P - MC roll angle P gain parameter
 * @see Q_A_RAT_PIT_P - MC pitch angle P gain parameter
 * @see ARSPD_FBW_MIN - Minimum fixed-wing airspeed parameter
 * @see Q_ASSIST_SPEED - Speed at which FW assist begins
 * 
 * Source: ArduPlane/quadplane.cpp:9266-9303
 */
void QuadPlane::setup_rp_fw_angle_gains(void)
{
    const float mc_angR = attitude_control->get_angle_roll_p().kP();
    const float mc_angP = attitude_control->get_angle_pitch_p().kP();
    const float fw_angR = 1.0/plane.rollController.tau();
    const float fw_angP = 1.0/plane.pitchController.tau();

    if (!is_positive(mc_angR) || !is_positive(mc_angP)) {
        // bad configuration, don't scale
        return;
    }

    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed)) {
        // can't get airspeed, no scaling of VTOL angle gains
        return;
    }

    const float low_airspeed = 3.0;
    if (aspeed <= low_airspeed || plane.aparm.airspeed_min <= low_airspeed) {
        // no scaling
        return;
    }

    const float angR_scale = linear_interpolate(mc_angR, fw_angR,
                                                aspeed,
                                                low_airspeed, plane.aparm.airspeed_min) / mc_angR;
    const float angP_scale = linear_interpolate(mc_angP, fw_angP,
                                                aspeed,
                                                low_airspeed, plane.aparm.airspeed_min) / mc_angP;
    const Vector3f gain_scale{angR_scale, angP_scale, 1.0};
    attitude_control->set_angle_P_scale(gain_scale);
}

/*
  abort landing, used by scripting for payload place and ship landing abort
  will return false if not in a landing descent
 */
bool QuadPlane::abort_landing(void)
{
    if (poscontrol.get_state() == QPOS_LAND_ABORT ||
        !(plane.control_mode == &plane.mode_auto)) {
        // already aborted or not in AUTO?
        return false;
    }

    // special case for payload place with full landing
    const bool payload_place_landed =
        plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE) &&
        poscontrol.get_state() == QPOS_LAND_COMPLETE;

    if (!payload_place_landed && !in_vtol_land_descent()) {
        return false;
    }
    poscontrol.set_state(QuadPlane::QPOS_LAND_ABORT);
    return true;
}

// Should we allow stick mixing from the pilot
bool QuadPlane::allow_stick_mixing() const
{
    if (!available()) {
        // Quadplane not enabled
        return true;
    }
    // Ask transition logic
    return transition->allow_stick_mixing();
}

/*
  return true if we should disable TECS in the current flight state
  this ensures that TECS resets when we change height in a VTOL mode
 */
bool QuadPlane::should_disable_TECS() const
{
    if (in_vtol_land_descent()) {
        return true;
    }
    if (plane.control_mode == &plane.mode_guided &&
        plane.auto_state.vtol_loiter) {
        return true;
    }
    return false;
}

/**
 * @brief Get pilot throttle input with deadzone and reverse handling for QuadPlane
 * 
 * @details Retrieves pilot throttle stick position as percentage with deadzone applied.
 *          Re-implementation of Plane::get_throttle_input specific to QuadPlane needs.
 *          Simplified version that doesn't require RC validity check since failsafe
 *          handling sets safe control values automatically.
 *          
 *          **Input Range:**
 *          - 0-100: Throttle stick percentage
 *          - Deadzone applied by RC library (typically ±5%)
 *          - Center stick = ~50% (varies with deadzone)
 *          
 *          **Failsafe Behavior:**
 *          - **CRITICAL**: Returns 50% throttle during failsafe
 *          - Plane::control_failsafe sets control_in to 50% automatically
 *          - Ensures motors at mid-throttle during RC loss
 *          - No explicit validity check needed (handled upstream)
 *          
 *          **Rationale for 50% Failsafe:**
 *          - Below 50%: Motors may spin down, losing altitude
 *          - Above 50%: May climb aggressively, wasting battery
 *          - 50%: Maintains approximate hover, buys time for recovery
 *          - Mode-specific failsafe actions then engage (RTL, QLAND)
 *          
 *          **Reverse Throttle Handling:**
 *          ```cpp
 *          if (plane.reversed_throttle)
 *              ret = -ret
 *          ```
 *          
 *          RC option allows throttle stick reversal:
 *          - Stick up = throttle down (reversed)
 *          - Stick down = throttle up (reversed)
 *          - ret becomes negative when stick is "up" in reversed mode
 *          
 *          **Why Reverse Throttle:**
 *          - Some pilots prefer stick-down for throttle-up (Mode 1/3 style)
 *          - Consistency with fixed-wing throttle conventions
 *          - Personal preference/ergonomics
 *          - Applied in software, doesn't change RC PWM interpretation
 *          
 *          **Simplified vs Plane Implementation:**
 *          
 *          **Plane::get_throttle_input:**
 *          - Checks RC validity explicitly
 *          - Handles no_deadzone parameter
 *          - More complex failsafe logic
 *          
 *          **QuadPlane::get_throttle_input (this function):**
 *          - Assumes RC already validated or failsafed
 *          - Always uses deadzone
 *          - Simpler, relies on Plane failsafe handling
 *          - Sufficient for VTOL operations
 *          
 *          **Rationale for Simplification:**
 *          - QuadPlane calls occur after Plane failsafe processing
 *          - control_in value guaranteed valid or failsafed
 *          - No need to re-check RC validity
 *          - Reduces code duplication
 *          - Maintains consistency with Plane's failsafe decisions
 *          
 *          **Usage Context:**
 *          
 *          Called from:
 *          - init_throttle_wait(): Check if pilot demanding throttle
 *          - Manual VTOL modes: QSTABILIZE, QACRO throttle input
 *          - QLOITER/QHOVER: Altitude hold with pilot override
 *          - Throttle suppression logic
 *          
 *          **Example Values:**
 *          
 *          **Normal Operation (not reversed):**
 *          - Stick full down: 0%
 *          - Stick centered: ~50%
 *          - Stick full up: 100%
 *          
 *          **Reversed Throttle:**
 *          - Stick full down: -100% (full throttle in reversed interpretation)
 *          - Stick centered: ~-50%
 *          - Stick full up: 0% (zero throttle in reversed interpretation)
 *          
 *          **Failsafe (RC loss):**
 *          - Returns: 50% (regardless of actual stick position)
 *          - Provides hover throttle estimate
 *          - Mode then transitions to failsafe action
 *          
 *          **Integration with Control Flow:**
 *          ```
 *          Main Loop:
 *              1. Read RC inputs
 *              2. Plane::control_failsafe checks RC validity
 *                  → If invalid: set control_in = 50%
 *                  → If valid: use actual stick position
 *              3. QuadPlane code calls get_throttle_input()
 *              4. Returns control_in (already safe)
 *              5. VTOL mode uses throttle input
 *          ```
 * 
 * @return float Throttle stick position as percentage (0-100 normally, negative if reversed)
 * 
 * @note Returns 50% during RC failsafe (set by Plane::control_failsafe)
 * @note Deadzone always applied by RC library
 * @note No explicit RC validity check (handled by Plane failsafe)
 * @note Simpler than Plane::get_throttle_input (QuadPlane-specific needs)
 * @note Reverse throttle option honored (RC option REVERSED_THROTTLE)
 * 
 * @warning Returns 50% in failsafe - modes must handle this appropriately
 * @warning Do not assume this checks RC validity - already handled upstream
 * 
 * @see Plane::control_failsafe() - Sets control_in to 50% on RC loss
 * @see plane.channel_throttle->get_control_in() - Get processed RC input
 * @see plane.reversed_throttle - RC option for throttle reversal
 * @see init_throttle_wait() - Uses this to check pilot throttle demand
 * 
 * Source: ArduPlane/quadplane.cpp:9679-9688
 */
float QuadPlane::get_throttle_input() const
{
    float ret = plane.channel_throttle->get_control_in();
    if (plane.reversed_throttle) {
        // RC option for reverse throttle has been set
        ret = -ret;
    }
    return ret;
}

/**
 * @brief Check if forward throttle is permitted in current VTOL flight mode
 * 
 * @details Determines whether forward-assist motors or pusher propellers can
 *          be engaged in the current VTOL mode. Used for QuadPlane configurations
 *          with dedicated forward thrust (tiltrotors, tail-sitters, pusher props).
 *          Prevents forward thrust during unsafe conditions (disarmed, shutting down).
 *          
 *          **Forward Throttle Concept:**
 *          
 *          Some QuadPlane configurations have separate forward propulsion:
 *          - **Tiltrotors**: VTOL motors tilt for forward thrust
 *          - **Tail-sitters**: Multicopter motors also provide forward thrust
 *          - **Pusher/tractor props**: Separate fixed motor for forward flight
 *          - **VTOL assist**: Forward motors help maintain airspeed in VTOL modes
 *          
 *          This function determines if these forward motors can be activated.
 *          
 *          **Safety Conditions (all must be true):**
 *          
 *          **1. In VTOL Mode:**
 *          ```cpp
 *          in_vtol_mode()
 *          ```
 *          
 *          Vehicle must be in a VTOL flight mode:
 *          - True: QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QAUTOTUNE
 *          - True: AUTO/GUIDED with VTOL sub-mode active
 *          - False: Fixed-wing modes (MANUAL, FBWA, CRUISE, AUTO FW, etc.)
 *          
 *          **Rationale:**
 *          - Forward throttle in FW mode handled by fixed-wing controller
 *          - This function only manages VTOL mode forward assist
 *          - Prevents mode confusion and control conflicts
 *          
 *          **2. Motors Armed:**
 *          ```cpp
 *          motors->armed()
 *          ```
 *          
 *          VTOL motors must be armed:
 *          - Armed: Pre-flight checks passed, motors can spin
 *          - Disarmed: Safety lockout, no motor outputs
 *          
 *          **Rationale:**
 *          - Forward throttle without arming is dangerous
 *          - Could cause unexpected vehicle movement on ground
 *          - Consistent with VTOL motor arming requirements
 *          - Pilot expects all motors locked when disarmed
 *          
 *          **3. Not Shutting Down:**
 *          ```cpp
 *          motors->get_desired_spool_state() != SHUT_DOWN
 *          ```
 *          
 *          Motor spool state must not be SHUT_DOWN:
 *          - SHUT_DOWN: Emergency stop, crash detected, or termination active
 *          - Other states: GROUND_IDLE, SPOOLING_UP, THROTTLE_UNLIMITED, SPOOLING_DOWN
 *          
 *          **Rationale:**
 *          - SHUT_DOWN indicates emergency or crash
 *          - ALL motors should stop (including forward motors)
 *          - Forward thrust during emergency worsens situation
 *          - Consistent total motor shutdown required
 *          
 *          **Spool State Details:**
 *          - **SHUT_DOWN**: Total motor shutdown, no outputs
 *          - **GROUND_IDLE**: Motors at minimum spin (armed on ground)
 *          - **SPOOLING_UP**: Ramping up from idle to flight throttle
 *          - **THROTTLE_UNLIMITED**: Full flight operations
 *          - **SPOOLING_DOWN**: Ramping down from flight to idle
 *          
 *          Forward throttle permitted in all states except SHUT_DOWN.
 *          
 *          **Return Value Logic:**
 *          
 *          ```
 *          return (in_vtol_mode && armed && not_shutting_down)
 *          ```
 *          
 *          **true**: All three conditions met, forward throttle safe to use
 *          **false**: At least one condition failed, block forward throttle
 *          
 *          **Usage Contexts:**
 *          
 *          **1. Forward Motor Control:**
 *          ```cpp
 *          if (allow_forward_throttle_in_vtol_mode())
 *              apply_forward_throttle(q_fwd_throttle)
 *          else
 *              apply_forward_throttle(0.0)
 *          ```
 *          
 *          **2. Tiltrotor Angle Limits:**
 *          - Check before allowing tilt toward forward flight
 *          - Prevents tilt during unsafe conditions
 *          - Ensures motors upright when conditions not met
 *          
 *          **3. Pusher Prop Activation:**
 *          - Gate for separate forward propulsion motor
 *          - Prevents pusher running while disarmed
 *          - Stops pusher during emergency shutdown
 *          
 *          **Scenario Examples:**
 *          
 *          **Scenario 1: Normal QLOITER Flight**
 *          - in_vtol_mode() = true (QLOITER is VTOL mode)
 *          - motors->armed() = true (armed and flying)
 *          - spool_state = THROTTLE_UNLIMITED (flight operations)
 *          - **Return: true** → Forward throttle allowed
 *          - Result: Forward assist can engage if needed
 *          
 *          **Scenario 2: QLOITER But Disarmed**
 *          - in_vtol_mode() = true (QLOITER is VTOL mode)
 *          - motors->armed() = false (on ground, disarmed)
 *          - spool_state = SHUT_DOWN (disarmed)
 *          - **Return: false** → Forward throttle blocked
 *          - Result: No forward thrust on ground (safe)
 *          
 *          **Scenario 3: FBWA Fixed-Wing Mode**
 *          - in_vtol_mode() = false (FBWA is FW mode)
 *          - motors->armed() = true
 *          - spool_state = SHUT_DOWN (VTOL motors not used)
 *          - **Return: false** → Forward throttle blocked by VTOL logic
 *          - Result: FW throttle controller handles forward thrust
 *          
 *          **Scenario 4: Emergency Shutdown**
 *          - in_vtol_mode() = true (was in QHOVER)
 *          - motors->armed() = true (still technically armed)
 *          - spool_state = SHUT_DOWN (crash detected, terminating)
 *          - **Return: false** → Forward throttle blocked
 *          - Result: All motors stop, including forward motors
 *          
 *          **Scenario 5: Landing Approach**
 *          - in_vtol_mode() = true (QLAND mode)
 *          - motors->armed() = true (landing sequence active)
 *          - spool_state = SPOOLING_DOWN (gentle descent)
 *          - **Return: true** → Forward throttle allowed
 *          - Result: Forward assist available during landing approach
 *          
 *          **Scenario 6: Transition to VTOL**
 *          - in_vtol_mode() = true (just entered QSTABILIZE)
 *          - motors->armed() = true (mid-transition)
 *          - spool_state = SPOOLING_UP (VTOL motors starting)
 *          - **Return: true** → Forward throttle allowed
 *          - Result: Forward motors can continue during transition
 *          
 *          **Integration with Forward Thrust Systems:**
 *          
 *          **Tiltrotor:**
 *          ```cpp
 *          if (allow_forward_throttle_in_vtol_mode())
 *              tiltrotor.tilt_angle = calculate_desired_tilt()
 *          else
 *              tiltrotor.tilt_angle = TILT_MAX_ANGLE_DEG  // Vertical
 *          ```
 *          
 *          **Pusher Prop:**
 *          ```cpp
 *          if (allow_forward_throttle_in_vtol_mode())
 *              pusher_throttle = q_fwd_throttle
 *          else
 *              pusher_throttle = 0.0
 *          ```
 *          
 *          **Tail-Sitter:**
 *          ```cpp
 *          if (allow_forward_throttle_in_vtol_mode())
 *              // Allow differential thrust for forward flight
 *          else
 *              // Pure hover thrust only
 *          ```
 *          
 *          **Common Misunderstandings:**
 *          
 *          **"Forward throttle only for transitions":**
 *          - FALSE: Also used for VTOL forward flight assist
 *          - Can be active in QLOITER, QHOVER with forward speed
 *          - Helps maintain airspeed in wind
 *          
 *          **"Only affects dedicated forward motors":**
 *          - FALSE: Also affects tiltrotor angles
 *          - Impacts motor thrust vectoring
 *          - Controls overall forward thrust strategy
 *          
 *          **"Armed check redundant with spool state":**
 *          - FALSE: Spool state can be non-SHUT_DOWN while disarmed briefly
 *          - During arming sequence, states transition
 *          - Belt-and-suspenders safety approach
 * 
 * @return true Forward throttle permitted in current VTOL mode
 * @return false Forward throttle blocked (not VTOL, disarmed, or shutting down)
 * 
 * @note Checks three conditions: VTOL mode, armed, not shutting down
 * @note Only applies to VTOL modes, not fixed-wing modes
 * @note Blocks forward throttle during emergency SHUT_DOWN state
 * @note Used for tiltrotors, tail-sitters, and pusher/puller configurations
 * @note Does not check if forward throttle is actually commanded
 * @note Only verifies if it's safe to apply forward throttle
 * 
 * @warning SAFETY-CRITICAL: Prevents forward thrust during unsafe conditions
 * @warning Forward throttle while disarmed could cause injury or damage
 * @warning Must block forward thrust during emergency shutdown
 * 
 * @see in_vtol_mode() - Check if currently in VTOL flight mode
 * @see motors->armed() - Check if VTOL motors are armed
 * @see motors->get_desired_spool_state() - Get current motor spool state
 * @see AP_Motors::DesiredSpoolState::SHUT_DOWN - Emergency shutdown state
 * @see q_fwd_throttle - Forward throttle amount (if allowed)
 * @see tiltrotor_output() - Applies forward throttle to tilted motors
 * @see forward_throttle_pct() - Calculates forward throttle percentage
 * 
 * Source: ArduPlane/quadplane.cpp:9804-9807
 */
bool QuadPlane::allow_forward_throttle_in_vtol_mode() const
{
    return in_vtol_mode() && motors->armed() && (motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::SHUT_DOWN);
}

/**
 * @brief Write QuadPlane attitude and rate controller data to dataflash log
 * 
 * @details Logs VTOL attitude control and rate controller performance metrics
 *          to onboard dataflash storage for post-flight analysis and tuning.
 *          Provides detailed insight into multicopter controller behavior during
 *          VTOL operations and transitions. Critical for PID tuning, performance
 *          analysis, and troubleshooting control issues.
 *          
 *          **Logged Data Messages:**
 *          
 *          **1. ANG Message (Attitude Angles):**
 *          ```cpp
 *          attitude_control->Write_ANG()
 *          ```
 *          
 *          Logs:
 *          - **Desired attitude**: Target roll/pitch/yaw from mode logic
 *          - **Actual attitude**: Current vehicle attitude from AHRS/EKF
 *          - **Attitude errors**: Difference between desired and actual
 *          - **Control outputs**: Attitude controller output to rate controller
 *          - **Timestamp**: High-resolution time for correlation
 *          
 *          **Purpose:**
 *          - Verify attitude tracking performance
 *          - Tune attitude P gains
 *          - Diagnose attitude oscillations or instability
 *          - Analyze mode transition behavior
 *          - Check input shaping effectiveness
 *          
 *          **2. Rate Message (Angular Rates):**
 *          ```cpp
 *          attitude_control->Write_Rate(*pos_control)
 *          ```
 *          
 *          Logs:
 *          - **Desired rates**: Target roll/pitch/yaw rates (deg/s)
 *          - **Actual rates**: Current gyro measurements (deg/s)
 *          - **Rate errors**: Difference between desired and actual rates
 *          - **PID components**: P, I, D, FF contributions for each axis
 *          - **Motor outputs**: Final mixed motor commands
 *          - **Position controller state**: Passed via pos_control reference
 *          
 *          **Purpose:**
 *          - Tune rate PID loops (most critical tuning)
 *          - Diagnose oscillations or poor tracking
 *          - Verify feedforward effectiveness
 *          - Check integral windup issues
 *          - Analyze motor saturation
 *          - Validate derivative filter settings
 *          
 *          **Logging Frequency:**
 *          
 *          Typically called at fast loop rate (typ. 400Hz for attitude control):
 *          - High-resolution capture of controller dynamics
 *          - Enables frequency domain analysis (FFT)
 *          - Captures high-frequency oscillations
 *          - Sufficient for PID loop analysis
 *          
 *          **Note on Frequency:**
 *          - Actual log rate may be reduced by LOG_RATE_MULT parameter
 *          - Default logs every Nth sample to manage storage
 *          - Higher rates needed for notch filter tuning
 *          - Lower rates acceptable for basic flight review
 *          
 *          **Usage Context:**
 *          
 *          Called from:
 *          - **Main VTOL update loop**: Each attitude control cycle
 *          - **All VTOL modes**: QSTABILIZE, QHOVER, QLOITER, etc.
 *          - **Transitions**: Forward and back transitions
 *          - **AUTO VTOL**: During VTOL waypoint execution
 *          
 *          Not called during:
 *          - Fixed-wing flight (plane uses separate logging)
 *          - Disarmed state (no control active)
 *          - QuadPlane disabled (HAL_QUADPLANE_ENABLED = 0)
 *          
 *          **Log Analysis Workflow:**
 *          
 *          **1. Extract Logs:**
 *          ```bash
 *          mavlogdump.py --types=ANG,RATE dataflash.bin > attrate.log
 *          ```
 *          
 *          **2. Analyze with MAVExplorer:**
 *          ```python
 *          graph ANG.DesRoll ANG.Roll  # Check attitude tracking
 *          graph RATE.RDes RATE.R      # Check rate tracking
 *          graph RATE.RP RATE.RI RATE.RD  # Examine PID components
 *          ```
 *          
 *          **3. Frequency Analysis:**
 *          ```python
 *          fft(RATE.R)  # Identify oscillation frequencies
 *          fft(RATE.P)  # Same for pitch
 *          fft(RATE.Y)  # Same for yaw
 *          ```
 *          
 *          **4. Performance Metrics:**
 *          - Settling time: Time to reach steady state
 *          - Overshoot: Percentage overshoot on step input
 *          - Tracking error RMS: Root mean square error
 *          - Phase margin: Stability indicator
 *          
 *          **Position Control Integration:**
 *          
 *          ```cpp
 *          attitude_control->Write_Rate(*pos_control)
 *          ```
 *          
 *          Pass pos_control reference to rate logger:
 *          - Correlate position errors with rate commands
 *          - Analyze how position loop drives attitude loop
 *          - Verify cascaded control chain: position → velocity → attitude → rate
 *          - Check for position-induced oscillations
 *          
 *          **Example Log Fields:**
 *          
 *          **ANG Message:**
 *          ```
 *          TimeUS: Timestamp (microseconds)
 *          DesRoll: Desired roll angle (centidegrees)
 *          Roll: Actual roll angle (centidegrees)
 *          DesPitch: Desired pitch angle (centidegrees)
 *          Pitch: Actual pitch angle (centidegrees)
 *          DesYaw: Desired yaw angle (centidegrees)
 *          Yaw: Actual yaw angle (centidegrees)
 *          ```
 *          
 *          **RATE Message:**
 *          ```
 *          TimeUS: Timestamp (microseconds)
 *          RDes: Desired roll rate (deg/s)
 *          R: Actual roll rate (deg/s)
 *          ROut: Roll motor output
 *          RP: Roll P term
 *          RI: Roll I term
 *          RD: Roll D term
 *          RFF: Roll feedforward
 *          [Similar for pitch and yaw]
 *          ```
 *          
 *          **Troubleshooting with Logs:**
 *          
 *          **Problem: Roll oscillation at 10Hz**
 *          - Check RATE.R for 10Hz frequency component
 *          - Likely too high P gain or insufficient D term
 *          - Solution: Reduce ATC_RAT_RLL_P or increase ATC_RAT_RLL_D
 *          
 *          **Problem: Slow attitude response**
 *          - Check ANG message for large sustained errors
 *          - Likely too low attitude P gain
 *          - Solution: Increase Q_A_ANGLE_P
 *          
 *          **Problem: Yaw washout in hover**
 *          - Check RATE.Y for accumulating error
 *          - Likely insufficient yaw I term or motor saturation
 *          - Solution: Increase ATC_RAT_YAW_I or check motor margins
 *          
 *          **Problem: Position oscillation causing attitude jitter**
 *          - Cross-reference RATE with PSC (position controller) logs
 *          - pos_control reference enables this correlation
 *          - May need to detune position controller
 *          
 *          **Storage Considerations:**
 *          
 *          At 400Hz logging:
 *          - ANG: ~50 bytes per message
 *          - RATE: ~80 bytes per message
 *          - Total: ~130 bytes * 400 Hz = 52 KB/sec
 *          - 10 minute flight: ~31 MB
 *          
 *          Mitigation:
 *          - LOG_RATE_MULT reduces effective rate (log every Nth sample)
 *          - Typical setting: 4 (logs at 100Hz effective rate)
 *          - Still sufficient for most tuning needs
 *          - Full 400Hz needed only for advanced notch filter tuning
 *          
 *          **Comparison with Fixed-Wing Logging:**
 *          
 *          **QuadPlane VTOL mode** (this function):
 *          - Logs multicopter attitude/rate control
 *          - High-frequency PID analysis
 *          - 3-axis rate control focus
 *          
 *          **Plane fixed-wing mode** (separate logging):
 *          - Logs fixed-wing controllers (TECS, L1)
 *          - Lower frequency (typically 50Hz)
 *          - Airspeed/altitude focus
 *          
 *          **During transitions:**
 *          - Both logging systems may be active
 *          - Allows analysis of transition control handoff
 *          - Critical for transition tuning
 *          
 *          **Developer Notes:**
 *          
 *          **Implementation location:**
 *          - AC_AttitudeControl::Write_ANG() in attitude_control library
 *          - AC_AttitudeControl::Write_Rate() in attitude_control library
 *          - Logging format defined in LogStructure.h
 *          
 *          **Why separate ANG and Rate messages:**
 *          - Different update rates (attitude can be slower than rate)
 *          - Separate analysis of attitude vs rate loop performance
 *          - Allows independent logging rate configuration
 *          
 *          **Why pass pos_control reference:**
 *          - Rate controller needs position control state for context
 *          - Enables correlation in log analysis
 *          - Avoids duplicating position data in rate message
 *          - Position controller can be nullptr (checked internally)
 * 
 * @note Called at fast loop rate (typically 400Hz) during VTOL operations
 * @note Actual log rate may be reduced by LOG_RATE_MULT parameter
 * @note Not called during fixed-wing flight (separate logging)
 * @note Logs written to onboard dataflash for post-flight analysis
 * @note Essential for PID tuning and performance troubleshooting
 * @note High log rate can consume significant storage space
 * 
 * @see AC_AttitudeControl::Write_ANG() - Log attitude angles and errors
 * @see AC_AttitudeControl::Write_Rate() - Log rate control and PID terms
 * @see attitude_control - QuadPlane attitude controller instance
 * @see pos_control - Position controller (passed for correlation)
 * @see LOG_RATE_MULT - Parameter controlling log decimation
 * @see LogStructure.h - Defines ANG and RATE message formats
 * 
 * Source: ArduPlane/quadplane.cpp:10023-10027
 */
void QuadPlane::Log_Write_AttRate()
{
    attitude_control->Write_ANG();
    attitude_control->Write_Rate(*pos_control);

}

#endif  // HAL_QUADPLANE_ENABLED
