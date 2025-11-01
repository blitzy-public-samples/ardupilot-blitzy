/**
 * @file tuning.h
 * @brief ArduPlane in-flight tuning interface header
 * 
 * @details This file defines the AP_Tuning_Plane class which provides
 *          in-flight parameter tuning capabilities for ArduPlane fixed-wing
 *          and quadplane vehicles. The tuning system allows pilots to adjust
 *          control parameters via transmitter knobs or switches during flight,
 *          enabling real-time PID tuning and controller optimization without
 *          landing the aircraft.
 *          
 *          The tuning interface supports:
 *          - Individual parameter tuning (single PID gains)
 *          - Tuning sets (multiple related parameters adjusted together)
 *          - Both quadplane VTOL parameters and fixed-wing parameters
 *          - Parameter save/reload functionality
 *          - Transmitter channel mapping for tuning control
 *          
 *          This is a vehicle-specific implementation of the AP_Tuning base
 *          class, providing parameter selection and access methods specific
 *          to ArduPlane's control architecture.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * @see AP_Tuning base class in libraries/AP_Tuning/
 * @see tuning.cpp for implementation details
 */

#pragma once

#include <AP_Tuning/AP_Tuning_config.h>

#if AP_TUNING_ENABLED

#include <AP_Tuning/AP_Tuning.h>

/**
 * @class AP_Tuning_Plane
 * @brief ArduPlane-specific implementation of in-flight transmitter tuning
 * 
 * @details This class extends AP_Tuning to provide in-flight parameter adjustment
 *          capabilities specific to ArduPlane vehicles. It manages the mapping between
 *          tuning channel inputs and adjustable parameters for both fixed-wing and
 *          quadplane flight modes.
 *          
 *          Key responsibilities:
 *          - Define tunable parameters for Plane (PID gains, feedforward terms)
 *          - Provide parameter pointer access for the base tuning system
 *          - Implement parameter save/reload/set operations
 *          - Track which parameters have been modified during flight
 *          - Support both individual parameter tuning and parameter sets
 *          
 *          Architecture:
 *          - Inherits from AP_Tuning (libraries/AP_Tuning/)
 *          - Uses tuning_func enum to identify individual parameters
 *          - Uses tuning_sets enum to group related parameters
 *          - Maintains static arrays defining parameter groupings
 *          - Tracks modifications via have_set bitmask
 *          
 *          Typical usage:
 *          1. Configure TUNE_CHAN to specify transmitter channel
 *          2. Set TUNE_SELECTOR to choose parameter or parameter set
 *          3. Adjust transmitter knob to modify parameter value
 *          4. Parameter changes applied in real-time to control loops
 *          5. Save to EEPROM if desired using TUNE_SAVE parameter
 * 
 * @note This class is only compiled when AP_TUNING_ENABLED is defined
 * @note Parameter modifications during flight should be done cautiously
 * @warning Improper tuning values can result in unstable flight or loss of control
 * 
 * @see AP_Tuning base class for tuning framework
 * @see libraries/AP_Tuning/AP_Tuning.h for interface definition
 */
class AP_Tuning_Plane : public AP_Tuning
{
private:
    /**
     * @brief Table of tuning parameter sets
     * 
     * @details Static array defining groups of related parameters that can be
     *          tuned together. Each set contains multiple tuning_func values
     *          that are adjusted simultaneously when a tuning set is selected.
     *          Defined in tuning.cpp.
     */
    static const tuning_set tuning_sets[];

    /**
     * @brief Table of tuning parameter names for telemetry reporting
     * 
     * @details Static array mapping tuning_func enum values to human-readable
     *          parameter names for ground station display and logging.
     *          Defined in tuning.cpp.
     */
    static const tuning_name tuning_names[];
    
public:
    /**
     * @brief Constructor for AP_Tuning_Plane
     * 
     * @details Initializes the Plane tuning system by passing the tuning_sets
     *          and tuning_names tables to the base AP_Tuning class constructor.
     *          No additional initialization required for Plane-specific tuning.
     */
    AP_Tuning_Plane(void) : AP_Tuning(tuning_sets, tuning_names) {}

    /**
     * @brief Parameter metadata for the tuning system
     * 
     * @details AP_Param group information table defining the tuning parameters
     *          that can be configured (TUNE_CHAN, TUNE_SELECTOR, TUNE_RANGE, etc.).
     *          Used by the parameter system for storage and ground station access.
     */
    static const struct AP_Param::GroupInfo  var_info[];
    
private:

    /**
     * @enum tuning_func
     * @brief Enumeration of individual tunable parameters
     * 
     * @details Defines all individual parameters that can be adjusted via the
     *          in-flight tuning system. Parameters are organized into two main
     *          categories:
     *          
     *          **Quadplane Parameters (1-26)**: PID gains and feedforward terms
     *          for VTOL flight modes, including:
     *          - Rate controllers (roll, pitch, yaw) - inner loop angular rate control
     *          - Angle controllers (roll, pitch, yaw) - outer loop attitude control
     *          - Position controllers (PXY, PZ) - horizontal and vertical position
     *          - Velocity controllers (VXY, VZ) - velocity control loops
     *          - Acceleration controllers (AZ) - vertical acceleration control
     *          
     *          **Fixed-Wing Parameters (50+)**: PID gains for conventional flight,
     *          including:
     *          - Roll (RLL) control parameters
     *          - Pitch (PIT) control parameters
     *          - Q_FWD_THR - Quadplane forward throttle in transitions
     *          
     *          Each enum value corresponds to a specific controller gain that can
     *          be accessed and modified through the tuning interface. The base
     *          value (50) for fixed-wing parameters allows clear separation between
     *          quadplane and fixed-wing tuning modes.
     * 
     * @note Parameter numbers must remain stable for compatibility with saved configurations
     * @note Values 0 and 100+ are reserved for special purposes (NONE and parameter sets)
     * 
     * @see tuning_sets enum for grouped parameter tuning
     * @see tuning.cpp for implementation of parameter access
     */
    enum tuning_func {
        TUNING_NONE =                          0,  ///< No tuning active

        // Quadplane VTOL tuning parameters (1-26)
        // Rate controller parameters - inner loop angular rate control
        TUNING_RATE_ROLL_PI =                  1,  ///< Roll rate P and I gains together
        TUNING_RATE_ROLL_P =                   2,  ///< Roll rate P gain only
        TUNING_RATE_ROLL_I =                   3,  ///< Roll rate I gain only
        TUNING_RATE_ROLL_D =                   4,  ///< Roll rate D gain only

        TUNING_RATE_PITCH_PI =                 5,  ///< Pitch rate P and I gains together
        TUNING_RATE_PITCH_P =                  6,  ///< Pitch rate P gain only
        TUNING_RATE_PITCH_I =                  7,  ///< Pitch rate I gain only
        TUNING_RATE_PITCH_D =                  8,  ///< Pitch rate D gain only

        TUNING_RATE_YAW_PI =                   9,  ///< Yaw rate P and I gains together
        TUNING_RATE_YAW_P =                   10,  ///< Yaw rate P gain only
        TUNING_RATE_YAW_I =                   11,  ///< Yaw rate I gain only
        TUNING_RATE_YAW_D =                   12,  ///< Yaw rate D gain only

        // Angle controller parameters - outer loop attitude control
        TUNING_ANG_ROLL_P =                   13,  ///< Roll angle P gain (attitude to rate)
        TUNING_ANG_PITCH_P =                  14,  ///< Pitch angle P gain (attitude to rate)
        TUNING_ANG_YAW_P =                    15,  ///< Yaw angle P gain (attitude to rate)

        // Position controller parameters
        TUNING_PXY_P =                        16,  ///< Horizontal position P gain (position to velocity)
        TUNING_PZ_P  =                        17,  ///< Vertical position P gain (altitude to climb rate)

        // Velocity controller parameters
        TUNING_VXY_P =                        18,  ///< Horizontal velocity P gain
        TUNING_VXY_I =                        19,  ///< Horizontal velocity I gain
        TUNING_VZ_P  =                        20,  ///< Vertical velocity P gain

        // Acceleration controller parameters
        TUNING_AZ_P =                         21,  ///< Vertical acceleration P gain (throttle control)
        TUNING_AZ_I =                         22,  ///< Vertical acceleration I gain
        TUNING_AZ_D  =                        23,  ///< Vertical acceleration D gain

        // Feedforward terms for rate controllers
        TUNING_RATE_PITCH_FF =         24,  ///< Pitch rate feedforward gain
        TUNING_RATE_ROLL_FF =         25,  ///< Roll rate feedforward gain
        TUNING_RATE_YAW_FF =         26,  ///< Yaw rate feedforward gain

        // Fixed-wing tuning parameters (50+)
        TUNING_FIXED_WING_BASE =              50,  ///< Base offset for fixed-wing parameters
        
        // Roll control parameters for fixed-wing flight
        TUNING_RLL_P =                        50,  ///< Roll P gain (fixed-wing)
        TUNING_RLL_I =                        51,  ///< Roll I gain (fixed-wing)
        TUNING_RLL_D =                        52,  ///< Roll D gain (fixed-wing)
        TUNING_RLL_FF =                       53,  ///< Roll feedforward gain (fixed-wing)

        // Pitch control parameters for fixed-wing flight
        TUNING_PIT_P =                        54,  ///< Pitch P gain (fixed-wing)
        TUNING_PIT_I =                        55,  ///< Pitch I gain (fixed-wing)
        TUNING_PIT_D =                        56,  ///< Pitch D gain (fixed-wing)
        TUNING_PIT_FF =                       57,  ///< Pitch feedforward gain (fixed-wing)

        // Quadplane transition parameters
        TUNING_Q_FWD_THR =                    58,  ///< Quadplane forward throttle in transitions
    };

    /**
     * @enum tuning_sets
     * @brief Enumeration of tuning parameter sets for grouped parameter adjustment
     * 
     * @details Defines predefined groups of related parameters that are tuned together
     *          as a set. When a tuning set is selected (via TUNE_PARMSET parameter with
     *          values > 100), all parameters in that set are adjusted proportionally
     *          by the tuning input channel.
     *          
     *          This allows efficient tuning workflows where multiple related gains
     *          need to be adjusted together, such as all PID terms for a single axis
     *          or all rate controller gains for roll and pitch simultaneously.
     *          
     *          Each tuning set corresponds to a static array (defined in tuning.cpp)
     *          that lists the tuning_func values included in that set.
     *          
     *          Common tuning workflows:
     *          - Use TUNING_SET_RATE_ROLL_PITCH for initial rate controller tuning
     *          - Use TUNING_SET_PIDFF_* sets for comprehensive axis tuning
     *          - Use TUNING_SET_DP_ROLL_PITCH for dual-parameter roll/pitch tuning
     * 
     * @note Set selection uses TUNE_PARMSET parameter with values > 100
     * @note Individual parameter selection uses values < 100
     * 
     * @see tuning_func enum for individual parameter definitions
     * @see tuning.cpp for set composition arrays
     */
    enum tuning_sets {
        TUNING_SET_RATE_ROLL_PITCH =         1,  ///< Rate controller PI gains for roll and pitch together
        TUNING_SET_RATE_ROLL =               2,  ///< Rate controller PI gains for roll only
        TUNING_SET_RATE_PITCH =              3,  ///< Rate controller PI gains for pitch only
        TUNING_SET_RATE_YAW =                4,  ///< Rate controller PI gains for yaw only
        TUNING_SET_ANG_ROLL_PITCH =          5,  ///< Angle controller P gains for roll and pitch
        TUNING_SET_VXY =                     6,  ///< Horizontal velocity controller PI gains
        TUNING_SET_AZ =                      7,  ///< Vertical acceleration controller PID gains
        TUNING_SET_RATE_PITCHDP =            8,  ///< Pitch rate controller with D and P gains
        TUNING_SET_RATE_ROLLDP =             9,  ///< Roll rate controller with D and P gains
        TUNING_SET_RATE_YAWDP =             10,  ///< Yaw rate controller with D and P gains
        TUNING_SET_DP_ROLL_PITCH =          11,  ///< D and P gains for roll and pitch rate controllers
        TUNING_SET_PIDFF_ROLL =             12,  ///< Complete PID+FF set for roll rate controller
        TUNING_SET_PIDFF_PITCH =            13,  ///< Complete PID+FF set for pitch rate controller
    };

    /**
     * @brief Get pointer to the AP_Float parameter object for a tuning parameter
     * 
     * @details Resolves a tuning_func enum value to the actual AP_Float* pointer
     *          for the corresponding controller parameter. This allows the base
     *          tuning system to directly access and modify parameter values.
     *          
     *          Handles parameter lookup for both quadplane and fixed-wing modes,
     *          accessing the appropriate controller objects (AC_AttitudeControl,
     *          AC_PosControl, APM_Control, etc.) based on the parameter ID.
     * 
     * @param[in] parm Tuning parameter ID from tuning_func enum
     * 
     * @return Pointer to AP_Float parameter object, or nullptr if parameter invalid
     * 
     * @note This is a virtual override of AP_Tuning::get_param_pointer()
     * @warning Returned pointer is only valid while the controller objects exist
     * 
     * @see tuning_func enum for parameter ID definitions
     */
    AP_Float *get_param_pointer(uint8_t parm) override;
    
    /**
     * @brief Save current value of a tuning parameter to EEPROM
     * 
     * @details Saves the current value of the specified parameter to non-volatile
     *          storage. This persists parameter changes made during flight so they
     *          are retained after reboot. Uses the AP_Param system for storage.
     * 
     * @param[in] parm Tuning parameter ID from tuning_func enum
     * 
     * @note This is a virtual override of AP_Tuning::save_value()
     * @note EEPROM writes are rate-limited to prevent excessive wear
     * 
     * @see AP_Param::save() for underlying save mechanism
     */
    void save_value(uint8_t parm) override;
    
    /**
     * @brief Reload a tuning parameter from EEPROM to restore saved value
     * 
     * @details Reloads the parameter value from non-volatile storage, discarding
     *          any unsaved changes made during the current flight. This allows
     *          reverting to the last saved configuration.
     * 
     * @param[in] parm Tuning parameter ID from tuning_func enum
     * 
     * @note This is a virtual override of AP_Tuning::reload_value()
     * 
     * @see AP_Param::load() for underlying load mechanism
     */
    void reload_value(uint8_t parm) override;
    
    /**
     * @brief Set a tuning parameter to a specific value
     * 
     * @details Sets the specified parameter to the given value and marks it as
     *          having been modified in the have_set bitmask. This is the primary
     *          method called when the tuning input channel changes, applying the
     *          scaled value to the parameter.
     *          
     *          The value is applied immediately to the controller, affecting
     *          flight behavior in real-time without requiring a reboot or mode change.
     * 
     * @param[in] parm  Tuning parameter ID from tuning_func enum
     * @param[in] value New parameter value to apply (already scaled by base class)
     * 
     * @note This is a virtual override of AP_Tuning::set_value()
     * @note Updates have_set bitmask to track modified parameters
     * @warning Parameter changes take effect immediately and affect flight control
     * 
     * @see get_param_pointer() for parameter resolution
     */
    void set_value(uint8_t parm, float value) override;

    /**
     * @brief Static arrays defining tuning parameter set compositions
     * 
     * @details Each array contains a list of tuning_func values that comprise
     *          a tuning set. Arrays are null-terminated to indicate end of set.
     *          Defined in tuning.cpp with specific parameter combinations for
     *          each tuning workflow.
     */
    static const uint8_t tuning_set_rate_roll_pitch[];  ///< Roll and pitch rate PI gains
    static const uint8_t tuning_set_rate_roll[];        ///< Roll rate PI gains only
    static const uint8_t tuning_set_rate_pitch[];       ///< Pitch rate PI gains only
    static const uint8_t tuning_set_rate_yaw[];         ///< Yaw rate PI gains only
    static const uint8_t tuning_set_ang_roll_pitch[];   ///< Roll and pitch angle P gains
    static const uint8_t tuning_set_vxy[];              ///< Horizontal velocity PI gains
    static const uint8_t tuning_set_az[];               ///< Vertical acceleration PID gains
    static const uint8_t tuning_set_rate_pitchDP[];     ///< Pitch rate D and P gains
    static const uint8_t tuning_set_rate_rollDP[];      ///< Roll rate D and P gains
    static const uint8_t tuning_set_rate_yawDP[];       ///< Yaw rate D and P gains
    static const uint8_t tuning_set_dp_roll_pitch[];    ///< Roll and pitch rate D and P gains
    static const uint8_t tuning_set_pidff_roll[];       ///< Roll rate full PID+FF set
    static const uint8_t tuning_set_pidff_pitch[];      ///< Pitch rate full PID+FF set

    /**
     * @brief Bitmask tracking which parameters have been modified during flight
     * 
     * @details 64-bit bitmask where each bit corresponds to a tuning_func parameter.
     *          When a parameter is modified via set_value(), the corresponding bit
     *          is set to indicate the parameter has been changed from its default.
     *          
     *          This tracking allows:
     *          - Identifying which parameters were tuned during a flight session
     *          - Selective parameter save operations
     *          - Telemetry reporting of modified parameters
     *          - Reset detection when parameters are reloaded
     * 
     * @note Bit position corresponds to tuning_func enum value
     * @note Limited to tracking first 64 parameters (sufficient for current set)
     * 
     * @see set_value() which sets bits in this mask
     */
    uint64_t have_set;
};

#endif  // AP_TUNING_ENABLED
