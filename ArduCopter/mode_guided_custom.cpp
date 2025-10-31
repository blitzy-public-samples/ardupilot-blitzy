/**
 * @file mode_guided_custom.cpp
 * @brief Custom guided mode implementation for script-controlled navigation
 * 
 * @details This file implements ModeGuidedCustom, an extended variant of the standard
 *          guided mode that supports custom navigation behaviors defined by Lua scripts.
 *          This mode is only available when both guided mode and scripting are enabled.
 *          
 *          Custom guided modes allow scripts to define application-specific or experimental
 *          guided control patterns that are not covered by the standard guided mode sub-types
 *          (e.g., position, velocity, angle, heading). Scripts can register custom modes with
 *          unique identifiers and names, and control when entry to these modes is permitted.
 *          
 *          Key characteristics:
 *          - Inherits from ModeGuided, leveraging standard guided mode infrastructure
 *          - Entry controlled by scripting layer via state.allow_entry flag
 *          - Supports custom mode numbering and naming for ground station display
 *          - Enables advanced automation and mission-specific behaviors
 *          
 *          Use cases include:
 *          - Research and development of novel flight behaviors
 *          - Application-specific navigation patterns (inspection, surveying, etc.)
 *          - Integration with external control systems via scripting
 *          - Rapid prototyping of new guidance algorithms without C++ changes
 * 
 * @note This mode requires AP_SCRIPTING_ENABLED and MODE_GUIDED_ENABLED to be defined
 * @warning Custom guided modes should be thoroughly tested in SITL before hardware deployment
 * 
 * @see ModeGuided - Base guided mode implementation
 * @see libraries/AP_Scripting/ - Lua scripting interface documentation
 * 
 * Source: ArduCopter/mode_guided_custom.cpp
 */

#include "Copter.h"

#if MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED

/**
 * @brief Constructor for custom guided mode variant
 * 
 * @details Initializes a custom guided mode instance with script-defined identification.
 *          This constructor is called by the scripting layer when registering a new
 *          custom mode. The mode number and names are stored for identification by
 *          ground control stations and internal mode switching logic.
 *          
 *          The mode number must be unique and is typically assigned from a reserved
 *          range for custom modes. The full name and short name are used for:
 *          - Ground station display and selection
 *          - MAVLink mode reporting
 *          - Logging and telemetry identification
 * 
 * @param[in] _number Mode number identifier (must be unique, typically in custom mode range)
 * @param[in] _full_name Long descriptive name for the custom mode (e.g., "Custom Inspection")
 * @param[in] _short_name Short name for displays with limited space (e.g., "C_INSP")
 * 
 * @note The constructor only stores identification; actual mode behavior is implemented
 *       through the scripting layer
 * @note Mode names should be descriptive and avoid conflicts with standard mode names
 * 
 * @see AP_Scripting custom mode registration API
 */
ModeGuidedCustom::ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}

/**
 * @brief Initialize custom guided mode and perform entry checks
 * 
 * @details This method is called when the vehicle attempts to enter this custom guided mode.
 *          It performs a two-stage validation process:
 *          
 *          1. Script-level validation: Checks if the controlling script permits entry via
 *             the state.allow_entry flag. Scripts can dynamically control mode availability
 *             based on vehicle state, mission phase, or external conditions.
 *          
 *          2. Standard guided checks: If script permits entry, delegates to ModeGuided::init()
 *             to perform standard pre-arm, sensor health, and flight condition checks.
 *          
 *          This design allows scripts to add application-specific entry conditions while
 *          maintaining all standard ArduPilot safety checks. For example, a script might
 *          only allow entry when a specific sensor is available or during certain mission phases.
 * 
 * @param[in] ignore_checks If true, bypass some standard checks (e.g., during failsafe recovery)
 *                          Note: Script-level check (state.allow_entry) is never bypassed
 * 
 * @return true if mode initialization successful and vehicle can enter custom guided mode
 * @return false if script blocks entry or standard guided mode checks fail
 * 
 * @note Script must set state.allow_entry = true for mode entry to succeed
 * @note Standard guided mode entry checks include: arming state, GPS fix, position estimate,
 *       EKF health, and other safety-critical conditions
 * @warning Mode entry failure during autonomous operations may trigger failsafe behavior
 * 
 * @see ModeGuided::init() - Base guided mode initialization and safety checks
 * @see state.allow_entry - Script-controlled entry permission flag
 */
bool ModeGuidedCustom::init(bool ignore_checks)
{
    // Script can block entry to custom guided mode based on application-specific conditions
    // This provides a gate for the scripting layer to control when this mode is available
    if (!state.allow_entry) {
        return false;
    }

    // Delegate to base ModeGuided to perform standard guided mode entry checks
    // This ensures all ArduPilot safety requirements are met (GPS, EKF, arming, etc.)
    return ModeGuided::init(ignore_checks);
}

#endif
