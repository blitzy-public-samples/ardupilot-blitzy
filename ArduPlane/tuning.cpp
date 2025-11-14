/**
 * @file tuning.cpp
 * @brief In-flight parameter tuning support for ArduPlane
 * 
 * @details This file implements real-time parameter adjustment capabilities for ArduPlane,
 *          allowing pilots to tune control parameters during flight via RC transmitter
 *          channels. This functionality is critical for:
 *          - Initial vehicle tuning and setup
 *          - Fine-tuning control response in various flight conditions
 *          - Quick adjustments without landing or ground station connection
 *          - Testing parameter ranges safely while maintaining manual control
 * 
 *          The implementation integrates with the AP_Tuning library to provide:
 *          - Transmitter-based tuning parameter selection via TUNE_PARAM
 *          - Real-time parameter updates while flying
 *          - Parameter reversion on channel reset
 *          - Support for both individual parameters and parameter sets
 *          - Separate tuning support for fixed-wing and QuadPlane VTOL modes
 * 
 *          Tuning Architecture:
 *          - AP_Tuning_Plane extends AP_Tuning base class with vehicle-specific mapping
 *          - tuning_sets[] defines logical groups of related parameters
 *          - tuning_names[] provides human-readable parameter names
 *          - get_param_pointer() maps tuning IDs to actual parameter objects
 *          - Parameter values are preserved in FRAM for reversion capability
 * 
 *          Conditional Compilation:
 *          This entire file is conditionally compiled based on AP_TUNING_ENABLED.
 *          When AP_TUNING_ENABLED is 0, all tuning functionality is excluded from
 *          the build to save flash memory on space-constrained boards.
 * 
 * @note This feature requires a transmitter with at least one available tuning channel
 * @warning In-flight tuning can destabilize the aircraft if extreme values are set.
 *          Always test with caution and maintain ability to revert to safe values.
 * 
 * @see AP_Tuning base class in libraries/AP_Tuning/
 * @see TUNE_PARAM documentation for available tuning parameters
 * 
 * Source: ArduPlane/tuning.cpp
 */

#include <AP_Tuning/AP_Tuning_config.h>

#if AP_TUNING_ENABLED

#include "Plane.h"

/**
 * @brief Parameter group definition for ArduPlane tuning system
 * 
 * @details The vehicle class has its own var_info table for TUNE_PARAM so it can
 *          have separate parameter documentation for the list of available parameters.
 *          This allows ArduPlane to expose fixed-wing specific tuning options alongside
 *          QuadPlane VTOL parameters when HAL_QUADPLANE_ENABLED is true.
 * 
 *          TUNE_PARAM determines which parameter or parameter set is adjusted by the
 *          tuning channel. Values less than 50 are QuadPlane-only, values 50-99 are
 *          fixed-wing specific, and values greater than 100 represent parameter sets
 *          that adjust multiple related parameters simultaneously.
 */
const AP_Param::GroupInfo AP_Tuning_Plane::var_info[] = {
    // @Param: PARAM
    // @DisplayName: Transmitter tuning parameter or set of parameters
    // @Description: This sets which parameter or set of parameters will be tuned. Values greater than 100 indicate a set of parameters rather than a single parameter. Parameters less than 50 are for QuadPlane vertical lift motors only.
    // @Values: 0:None,1:RateRollPI,2:RateRollP,3:RateRollI,4:RateRollD,5:RatePitchPI,6:RatePitchP,7:RatePitchI,8:RatePitchD,9:RateYawPI,10:RateYawP,11:RateYawI,12:RateYawD,13:AngleRollP,14:AnglePitchP,15:AngleYawP,16:PosXYP,17:PosZP,18:VelXYP,19:VelXYI,20:VelZP,21:AccelZP,22:AccelZI,23:AccelZD,24:RatePitchFF,25:RateRollFF,26:RateYawFF,50:FixedWingRollP,51:FixedWingRollI,52:FixedWingRollD,53:FixedWingRollFF,54:FixedWingPitchP,55:FixedWingPitchI,56:FixedWingPitchD,57:FixedWingPitchFF,101:Set_RateRollPitch,102:Set_RateRoll,103:Set_RatePitch,104:Set_RateYaw,105:Set_AngleRollPitch,106:Set_VelXY,107:Set_AccelZ,108:Set_RatePitchDP,109:Set_RateRollDP,110:Set_RateYawDP
    // @User: Standard
    AP_GROUPINFO("PARAM", 1, AP_Tuning_Plane, parmset, 0),

    // the rest of the parameters are from AP_Tuning
    AP_NESTEDGROUPINFO(AP_Tuning, 0),

    AP_GROUPEND
};

/**
 * @brief Tuning set definitions for related parameter groups
 * 
 * @details These arrays define logical groupings of tuning parameters that are
 *          commonly adjusted together. When a tuning set is selected via TUNE_PARAM,
 *          the tuning channel simultaneously adjusts all parameters in the set,
 *          maintaining their relative ratios while scaling the overall values.
 * 
 *          This approach is useful for:
 *          - Adjusting P+I+D gains together while preserving their relationships
 *          - Tuning roll and pitch axes simultaneously for symmetric response
 *          - Quick overall responsiveness adjustment without individual tuning
 * 
 * @note Each array is referenced by the tuning_sets[] structure below
 * @note Array sizes are automatically calculated using the TUNING_ARRAY macro
 */
// QuadPlane VTOL rate controller tuning sets (parameters < 50)
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll_pitch[] =  { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI,
                                                                 TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI};
const uint8_t AP_Tuning_Plane::tuning_set_rate_roll[] =        { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_pitch[] =       { TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_PI };
const uint8_t AP_Tuning_Plane::tuning_set_rate_yaw[] =         { TUNING_RATE_YAW_P, TUNING_RATE_YAW_I, TUNING_RATE_YAW_D };
const uint8_t AP_Tuning_Plane::tuning_set_ang_roll_pitch[] =   { TUNING_ANG_ROLL_P, TUNING_ANG_PITCH_P };
const uint8_t AP_Tuning_Plane::tuning_set_vxy[] =              { TUNING_VXY_P, TUNING_VXY_I };
const uint8_t AP_Tuning_Plane::tuning_set_az[] =               { TUNING_AZ_P, TUNING_AZ_I, TUNING_AZ_D };
const uint8_t AP_Tuning_Plane::tuning_set_rate_pitchDP[]=      { TUNING_RATE_PITCH_D, TUNING_RATE_PITCH_P };
const uint8_t AP_Tuning_Plane::tuning_set_rate_rollDP[]=       { TUNING_RATE_ROLL_D, TUNING_RATE_ROLL_P };
const uint8_t AP_Tuning_Plane::tuning_set_rate_yawDP[]=        { TUNING_RATE_YAW_D, TUNING_RATE_YAW_P };

// Fixed-wing controller tuning sets (parameters >= 50)
const uint8_t AP_Tuning_Plane::tuning_set_dp_roll_pitch[] =    { TUNING_RLL_D, TUNING_RLL_P, TUNING_PIT_D, TUNING_PIT_P };
const uint8_t AP_Tuning_Plane::tuning_set_pidff_roll[] =       { TUNING_RLL_P, TUNING_RLL_I, TUNING_RLL_D, TUNING_RLL_FF };
const uint8_t AP_Tuning_Plane::tuning_set_pidff_pitch[] =      { TUNING_PIT_P, TUNING_PIT_I, TUNING_PIT_D, TUNING_PIT_FF };

/**
 * @brief Helper macro to safely get array size and pointer for tuning sets
 * @param v Array variable name
 * @return Array size followed by array pointer for use in tuning_set initialization
 * @note This macro prevents array length errors by automatically calculating size
 */
#define TUNING_ARRAY(v) ARRAY_SIZE(v), v

/**
 * @brief Master list mapping tuning set IDs to parameter arrays
 * 
 * @details This array maps TUNE_PARAM values (>100) to their corresponding parameter
 *          arrays. The AP_Tuning base class uses this mapping to determine which
 *          parameters to adjust when a tuning set is selected.
 * 
 *          Each entry contains:
 *          - set_id: TUNE_PARAM value that selects this set (e.g., TUNING_SET_RATE_ROLL_PITCH = 101)
 *          - count: Number of parameters in the set
 *          - members: Pointer to array of tuning parameter IDs
 * 
 *          The list is terminated with a zero-initialized entry.
 * 
 * @note Tuning set IDs start at 101 to distinguish from individual parameters
 */
const AP_Tuning_Plane::tuning_set AP_Tuning_Plane::tuning_sets[] = {
    { TUNING_SET_RATE_ROLL_PITCH,       TUNING_ARRAY(tuning_set_rate_roll_pitch) },
    { TUNING_SET_RATE_ROLL,             TUNING_ARRAY(tuning_set_rate_roll) },
    { TUNING_SET_RATE_PITCH,            TUNING_ARRAY(tuning_set_rate_pitch) },
    { TUNING_SET_RATE_YAW,              TUNING_ARRAY(tuning_set_rate_yaw) },
    { TUNING_SET_ANG_ROLL_PITCH,        TUNING_ARRAY(tuning_set_ang_roll_pitch) },
    { TUNING_SET_VXY,                   TUNING_ARRAY(tuning_set_vxy) },
    { TUNING_SET_AZ,                    TUNING_ARRAY(tuning_set_az) },
    { TUNING_SET_RATE_PITCHDP,          TUNING_ARRAY(tuning_set_rate_pitchDP) },
    { TUNING_SET_RATE_ROLLDP,           TUNING_ARRAY(tuning_set_rate_rollDP) },
    { TUNING_SET_RATE_YAWDP,            TUNING_ARRAY(tuning_set_rate_yawDP) },
    { TUNING_SET_DP_ROLL_PITCH,         TUNING_ARRAY(tuning_set_dp_roll_pitch) },
    { TUNING_SET_PIDFF_ROLL,            TUNING_ARRAY(tuning_set_pidff_roll) },
    { TUNING_SET_PIDFF_PITCH,           TUNING_ARRAY(tuning_set_pidff_pitch) },
    { 0, 0, nullptr }
};

/**
 * @brief Human-readable names for tuning parameters
 * 
 * @details This table maps tuning parameter IDs to descriptive string names used for:
 *          - Logging and telemetry reporting
 *          - Ground station display
 *          - Debugging and parameter identification
 * 
 *          The names correspond to the actual controller gains being adjusted:
 *          - Rate* parameters: Rate controller PID gains (degrees/second tracking)
 *          - Ang* parameters: Angle controller P gains (attitude tracking)
 *          - P/V/A parameters: Position/Velocity/Acceleration controller gains (QuadPlane)
 *          - Roll/Pitch/Yaw parameters: Fixed-wing controller gains
 * 
 *          Parameter ID ranges:
 *          - 1-49: QuadPlane VTOL parameters (require HAL_QUADPLANE_ENABLED)
 *          - 50-99: Fixed-wing parameters (always available)
 *          - 100+: Parameter sets (not included in names table)
 * 
 * @note The list is terminated with TUNING_NONE and nullptr
 * @note Names match the parameter descriptions in var_info[] above
 */
const AP_Tuning_Plane::tuning_name AP_Tuning_Plane::tuning_names[] = {
    { TUNING_RATE_ROLL_PI, "RateRollPI" },
    { TUNING_RATE_ROLL_P,  "RateRollP" },
    { TUNING_RATE_ROLL_I,  "RateRollI" },
    { TUNING_RATE_ROLL_D,  "RateRollD" },
    { TUNING_RATE_PITCH_PI,"RatePitchPI" },
    { TUNING_RATE_PITCH_P, "RatePitchP" },
    { TUNING_RATE_PITCH_I, "RatePitchI" },
    { TUNING_RATE_PITCH_D, "RatePitchD" },
    { TUNING_RATE_YAW_PI,  "RateYawPI" },
    { TUNING_RATE_YAW_P,   "RateYawP" },
    { TUNING_RATE_YAW_I,   "RateYawI" },
    { TUNING_RATE_YAW_D,   "RateYawD" },
    { TUNING_ANG_ROLL_P,   "AngRollP" },
    { TUNING_ANG_PITCH_P,  "AngPitchP" },
    { TUNING_ANG_YAW_P,    "AngYawP" },
    { TUNING_RATE_PITCH_FF, "RatePitchFF" },
    { TUNING_RATE_ROLL_FF, "RateRollFF" },
    { TUNING_RATE_YAW_FF, "RateYawFF" },
    { TUNING_PXY_P,        "PXY_P" },
    { TUNING_PZ_P,         "PZ_P" },
    { TUNING_VXY_P,        "VXY_P" },
    { TUNING_VXY_I,        "VXY_I" },
    { TUNING_VZ_P,         "VZ_P" },
    { TUNING_AZ_P,         "RateAZ_P" },
    { TUNING_AZ_I,         "RateAZ_I" },
    { TUNING_AZ_D,         "RateAZ_D" },
    { TUNING_RLL_P,        "RollP" },
    { TUNING_RLL_I,        "RollI" },
    { TUNING_RLL_D,        "RollD" },
    { TUNING_RLL_FF,       "RollFF" },
    { TUNING_PIT_P,        "PitchP" },
    { TUNING_PIT_I,        "PitchI" },
    { TUNING_PIT_D,        "PitchD" },
    { TUNING_PIT_FF,       "PitchFF" },
    { TUNING_Q_FWD_THR,    "QModeFwdThr" },
    { TUNING_NONE, nullptr }
};

/**
 * @brief Get pointer to the AP_Float parameter object for a given tuning ID
 * 
 * @details This function maps tuning parameter IDs (from TUNE_PARAM) to the actual
 *          AP_Float objects in the controller classes. This is the core mapping function
 *          that enables the tuning system to access and modify real-time controller gains.
 * 
 *          The function handles three categories of parameters:
 *          1. QuadPlane VTOL parameters (1-49): Access attitude_control and pos_control PIDs
 *          2. Fixed-wing parameters (50-99): Access rollController and pitchController gains
 *          3. Invalid or disabled parameters: Return nullptr
 * 
 *          QuadPlane Parameter Access:
 *          - Requires HAL_QUADPLANE_ENABLED and quadplane.available()
 *          - Accesses AC_AttitudeControl PID objects for rate/angle control
 *          - Accesses AC_PosControl PID objects for position/velocity/acceleration
 * 
 *          Fixed-Wing Parameter Access:
 *          - Accesses APM_Control rollController and pitchController objects
 *          - Provides P, I, D, and FF gain tuning for fixed-wing flight
 * 
 * @param[in] parm Tuning parameter ID (1-99) from TUNE_PARAM selection
 * 
 * @return Pointer to AP_Float parameter object if valid, nullptr if:
 *         - Parameter ID is invalid or unrecognized
 *         - QuadPlane parameter requested but QuadPlane not available
 *         - Parameter not supported on current hardware configuration
 * 
 * @note This function is called frequently during active tuning (typically 10-50Hz)
 * @note Returning nullptr disables tuning for that parameter selection
 * 
 * @see AP_Tuning::check_input() for tuning channel processing
 * @see set_value() for how parameter values are applied
 */
AP_Float *AP_Tuning_Plane::get_param_pointer(uint8_t parm)
{
#if HAL_QUADPLANE_ENABLED
    // Verify QuadPlane availability for VTOL parameters (IDs < 50)
    if (parm < TUNING_FIXED_WING_BASE && !plane.quadplane.available()) {
        // QuadPlane tuning options not available - QuadPlane not enabled or initialized
        return nullptr;
    }
#endif

    switch(parm) {

#if HAL_QUADPLANE_ENABLED
    // QuadPlane VTOL rate controller parameters (attitude rate tracking)
    // These adjust the AC_AttitudeControl rate PID controllers used in VTOL modes
    
    case TUNING_RATE_ROLL_PI:
        // use P for initial value when tuning PI
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_P:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kP();

    case TUNING_RATE_ROLL_I:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kI();

    case TUNING_RATE_ROLL_D:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().kD();

    case TUNING_RATE_PITCH_PI:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_P:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kP();

    case TUNING_RATE_PITCH_I:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kI();

    case TUNING_RATE_PITCH_D:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().kD();

    case TUNING_RATE_YAW_PI:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_P:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kP();

    case TUNING_RATE_YAW_I:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kI();

    case TUNING_RATE_YAW_D:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().kD();

    case TUNING_ANG_ROLL_P:
        return &plane.quadplane.attitude_control->get_angle_roll_p().kP();

    case TUNING_ANG_PITCH_P:
        return &plane.quadplane.attitude_control->get_angle_pitch_p().kP();

    case TUNING_ANG_YAW_P:
        return &plane.quadplane.attitude_control->get_angle_yaw_p().kP();

    case TUNING_PXY_P:
        return &plane.quadplane.pos_control->get_pos_NE_p().kP();

    case TUNING_PZ_P:
        return &plane.quadplane.pos_control->get_pos_U_p().kP();

    case TUNING_VXY_P:
        return &plane.quadplane.pos_control->get_vel_NE_pid().kP();

    case TUNING_VXY_I:
        return &plane.quadplane.pos_control->get_vel_NE_pid().kI();

    case TUNING_VZ_P:
        return &plane.quadplane.pos_control->get_vel_U_pid().kP();

    case TUNING_AZ_P:
        return &plane.quadplane.pos_control->get_accel_U_pid().kP();

    case TUNING_AZ_I:
        return &plane.quadplane.pos_control->get_accel_U_pid().kI();

    case TUNING_AZ_D:
        return &plane.quadplane.pos_control->get_accel_U_pid().kD();

    case TUNING_RATE_PITCH_FF:
        return &plane.quadplane.attitude_control->get_rate_pitch_pid().ff();

    case TUNING_RATE_ROLL_FF:
        return &plane.quadplane.attitude_control->get_rate_roll_pid().ff();

    case TUNING_RATE_YAW_FF:
        return &plane.quadplane.attitude_control->get_rate_yaw_pid().ff();

    case TUNING_Q_FWD_THR:
        return &plane.quadplane.q_fwd_thr_gain;
#endif // HAL_QUADPLANE_ENABLED

    // Fixed-wing controller parameters (parameters >= 50)
    // These adjust the APM_Control roll and pitch controllers used in forward flight
    
    case TUNING_RLL_P:
        return &plane.rollController.kP();

    case TUNING_RLL_I:
        return &plane.rollController.kI();

    case TUNING_RLL_D:
        return &plane.rollController.kD();

    case TUNING_RLL_FF:
        return &plane.rollController.kFF();

    case TUNING_PIT_P:
        return &plane.pitchController.kP();

    case TUNING_PIT_I:
        return &plane.pitchController.kI();

    case TUNING_PIT_D:
        return &plane.pitchController.kD();

    case TUNING_PIT_FF:
        return &plane.pitchController.kFF();
    }
    return nullptr;
}

/**
 * @brief Save a tuning parameter value to persistent storage
 * 
 * @details This function saves the current value of a tuning parameter to EEPROM/FRAM
 *          for persistence across reboots. Called when the pilot requests to save
 *          tuned values via the tuning system interface (typically via MAVLink or
 *          RC channel action).
 * 
 *          Special handling for combined PI parameters:
 *          - TUNING_RATE_ROLL_PI saves both P and I components separately
 *          - TUNING_RATE_PITCH_PI saves both P and I components separately
 * 
 *          This allows pilots to:
 *          - Test parameter values in flight
 *          - Save successful tuning results permanently
 *          - Revert to saved values if needed
 * 
 * @param[in] parm Tuning parameter ID to save
 * 
 * @note Parameter is saved to non-volatile storage (EEPROM/FRAM)
 * @note Write cycles are limited on EEPROM - avoid excessive saves
 * @note If parameter pointer is nullptr (invalid/unavailable), no action taken
 * 
 * @see AP_Float::save() for underlying storage mechanism
 * @see reload_value() for restoring saved parameters
 */
void AP_Tuning_Plane::save_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        save_value(TUNING_RATE_ROLL_P);
        save_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        save_value(TUNING_RATE_PITCH_P);
        save_value(TUNING_RATE_PITCH_I);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            f->save();
        }
        break;
    }
}

/**
 * @brief Set a tuning parameter to a new value with reversion support
 * 
 * @details This function updates a tuning parameter's value in real-time during flight.
 *          It is called continuously as the pilot adjusts the tuning channel, providing
 *          immediate feedback on parameter changes.
 * 
 *          Reversion Mechanism:
 *          On first adjustment of a parameter (tracked via have_set bitmap):
 *          1. Attempts to load last saved value from FRAM as reversion point
 *          2. If no saved value exists, saves current value for reversion
 *          3. Marks parameter as "have been set" to avoid repeated FRAM operations
 * 
 *          This enables safe in-flight tuning with ability to revert to known-good values
 *          if the pilot encounters instability.
 * 
 *          Special handling for combined PI parameters:
 *          - TUNING_RATE_ROLL_PI sets both P and I to the same value
 *          - TUNING_RATE_PITCH_PI sets both P and I to the same value
 * 
 * @param[in] parm  Tuning parameter ID to modify
 * @param[in] value New parameter value (controller-specific units, typically gain values)
 * 
 * @note Parameter change is applied immediately via set_and_notify()
 * @note Controllers receive parameter change notifications for real-time updates
 * @note have_set bitmap tracks which parameters have reversion points saved
 * @note If parameter pointer is nullptr (invalid/unavailable), no action taken
 * 
 * @warning Extreme parameter values can cause instability - tuning ranges should be limited
 * 
 * @see get_param_pointer() for parameter object access
 * @see reload_value() for reverting to saved values
 * @see AP_Float::set_and_notify() for parameter update mechanism
 */
void AP_Tuning_Plane::set_value(uint8_t parm, float value)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        set_value(TUNING_RATE_ROLL_P, value);
        set_value(TUNING_RATE_ROLL_I, value);
        break;
    case TUNING_RATE_PITCH_PI:
        set_value(TUNING_RATE_PITCH_P, value);
        set_value(TUNING_RATE_PITCH_I, value);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            if (!(param_bit & have_set)) {
                // first time this param has been set by tuning. We
                // need to see if a reversion value is available in
                // FRAM, and if not then save one
                float current_value = f->get();
                if (!f->load()) {
                    // there is no value in FRAM, set one
                    f->set_and_save(current_value);
                }
                have_set |= param_bit;
            }
            f->set_and_notify(value);
        }
        break;
    }
}

/**
 * @brief Reload a tuning parameter from persistent storage (revert to saved value)
 * 
 * @details This function restores a parameter to its previously saved value from FRAM,
 *          allowing the pilot to revert changes if the tuned values cause instability
 *          or undesirable behavior.
 * 
 *          Reversion Safety:
 *          - Only reloads parameters that have been modified during this session (have_set bitmap)
 *          - Prevents accidentally loading uninitialized FRAM values
 *          - Provides immediate return to last known-good values during flight
 * 
 *          Typical usage scenarios:
 *          - Pilot finds tuned values too aggressive and wants to revert
 *          - Quick return to baseline after experimental tuning
 *          - Recovery from parameter values that cause oscillation
 * 
 *          Special handling for combined PI parameters:
 *          - TUNING_RATE_ROLL_PI reloads both P and I components
 *          - TUNING_RATE_PITCH_PI reloads both P and I components
 * 
 * @param[in] parm Tuning parameter ID to reload from storage
 * 
 * @note Only reloads parameters that have been set during current session (have_set check)
 * @note Parameter is loaded from FRAM reversion point established by set_value()
 * @note If parameter pointer is nullptr (invalid/unavailable), no action taken
 * @note If parameter has never been set via tuning (not in have_set), no reload occurs
 * 
 * @see set_value() for how reversion points are established
 * @see save_value() for permanently storing parameter values
 * @see AP_Float::load() for underlying FRAM read mechanism
 */
void AP_Tuning_Plane::reload_value(uint8_t parm)
{
    switch(parm) {
    // special handling of dual-parameters
    case TUNING_RATE_ROLL_PI:
        reload_value(TUNING_RATE_ROLL_P);
        reload_value(TUNING_RATE_ROLL_I);
        break;
    case TUNING_RATE_PITCH_PI:
        reload_value(TUNING_RATE_PITCH_P);
        reload_value(TUNING_RATE_PITCH_I);
        break;
    default:
        AP_Float *f = get_param_pointer(parm);
        if (f != nullptr) {
            uint64_t param_bit = (1ULL << parm);
            // only reload if we have set this parameter at some point
            if (param_bit & have_set) {
                f->load();
            }
        }
        break;
    }
}

/**
 * End of conditional compilation block for AP_TUNING_ENABLED
 * 
 * When AP_TUNING_ENABLED is 0 (disabled), the entire contents of this file are excluded
 * from compilation, saving flash memory on boards with limited storage. This is controlled
 * via AP_Tuning_config.h based on board capabilities and feature selections.
 * 
 * Boards that typically disable tuning:
 * - Very small flash (< 1MB) where every byte counts
 * - Production vehicles with locked-down configurations
 * - Safety-critical applications where in-flight tuning is not permitted
 */
#endif  // AP_TUNING_ENABLED
