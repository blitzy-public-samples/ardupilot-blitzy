/**
 * @file UserParameters.cpp
 * @brief User-customizable parameter definitions for ArduCopter
 * 
 * @details This file provides a mechanism for users to define custom parameters
 *          that can be accessed via the ground control station and stored persistently
 *          in EEPROM. User parameters are integrated into ArduPilot's AP_Param system
 *          and follow the same persistence, serialization, and MAVLink access patterns
 *          as built-in parameters.
 * 
 *          USER PARAMETER NAMING CONVENTION:
 *          All user parameters are prefixed with "USR" (3 characters), leaving up to
 *          13 additional characters for the parameter name. Total maximum name length
 *          is 16 characters including the prefix.
 *          
 *          Example: USR_MY_PARAM would appear as "USR_MY_PARAM" in ground station
 * 
 *          PARAMETER TYPE SUPPORT:
 *          The AP_Param system supports multiple parameter types through specialized
 *          classes defined in AP_Param.h:
 *          - AP_Int8:   8-bit signed integer (-128 to 127)
 *          - AP_Int16:  16-bit signed integer (-32768 to 32767)
 *          - AP_Int32:  32-bit signed integer
 *          - AP_Float:  32-bit floating point
 *          
 *          INTEGRATION WITH MAIN PARAMETER SYSTEM:
 *          The UserParameters class is instantiated as a nested object within the
 *          main Copter::Parameters class (see Parameters.cpp and Parameters.h).
 *          The var_info table defined here is registered with AP_Param during
 *          system initialization, making these parameters accessible through:
 *          - MAVLink PARAM_REQUEST_LIST and PARAM_SET messages
 *          - Ground control station parameter editors
 *          - CLI param commands
 *          - Lua scripting via param:get() and param:set()
 * 
 *          PARAMETER PERSISTENCE:
 *          User parameters are automatically saved to EEPROM when modified and
 *          restored on boot. The AP_Param system handles serialization, wear
 *          leveling, and version migration automatically.
 * 
 *          USAGE WORKFLOW:
 *          1. Define parameter member variables in UserParameters.h
 *          2. Register parameters in var_info[] array below with AP_GROUPINFO
 *          3. Assign unique index numbers (0-N) to each parameter
 *          4. Set appropriate default values
 *          5. Rebuild firmware and upload to vehicle
 *          6. Parameters appear in ground station with "USR" prefix
 * 
 *          EXAMPLE CUSTOMIZATION:
 *          To add a new user parameter for custom tuning:
 *          1. Add to UserParameters.h: AP_Float _my_gain;
 *          2. Add to var_info below: AP_GROUPINFO("_MY_GAIN", 3, UserParameters, _my_gain, 1.0),
 *          3. Access in code: copter.user_parameters._my_gain
 * 
 * @note This feature is controlled by USER_PARAMS_ENABLED in config.h
 * @note Parameter indices must be unique and sequential within this group
 * @note Changing parameter indices after deployment will reset stored values
 * @warning Modifying var_info structure requires understanding of AP_Param system
 * 
 * @see AP_Param.h for parameter type definitions and API
 * @see Parameters.h for main parameter system integration
 * @see Parameters.cpp for UserParameters object instantiation
 * 
 * Source: ArduCopter/UserParameters.cpp
 */

#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED

/**
 * @brief Parameter registration table for user-defined parameters
 * 
 * @details This var_info array is the core of the AP_Param registration system.
 *          Each entry in the table defines one parameter with the following information:
 *          - Parameter name suffix (appended to "USR" prefix)
 *          - Unique index for storage identification
 *          - Member variable reference
 *          - Default value
 * 
 *          AP_GROUPINFO MACRO SYNTAX:
 *          AP_GROUPINFO(name, index, class, variable, default)
 *          
 *          @param name     Parameter name suffix (string), max 13 chars
 *          @param index    Unique index number for this parameter (0-255)
 *          @param class    Class name containing the parameter (UserParameters)
 *          @param variable Member variable name in class
 *          @param default  Default value assigned on first boot
 * 
 *          PARAMETER INDEX ASSIGNMENT:
 *          - Indices must be unique within this var_info table
 *          - Indices should be assigned sequentially starting from 0
 *          - Once deployed, changing an index will cause parameter reset
 *          - Gaps in index sequence are allowed for future expansion
 * 
 *          PARAMETER NAMING CONSTRAINTS:
 *          - Total name = "USR" + name parameter = max 16 characters
 *          - Name parameter can be up to 13 characters
 *          - Use uppercase for consistency with ArduPilot conventions
 *          - Use underscores to separate words
 *          - Prefix with underscore by convention (e.g., "_INT8", "_FLOAT")
 * 
 *          EXAMPLE PARAMETER TYPES DEMONSTRATED BELOW:
 *          - AP_Int8:  8-bit integer for small integer values or flags
 *          - AP_Int16: 16-bit integer for medium-range values
 *          - AP_Float: Floating point for precise tuning values
 * 
 * @note The AP_GROUPEND marker must always be the last entry in the table
 * @note Adding new parameters requires rebuilding and uploading firmware
 * @note Parameter changes take effect after reboot
 * 
 * @see AP_Param::GroupInfo for structure definition
 * @see AP_Param.h for available parameter types and macros
 */

// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    
    /**
     * Example 8-bit integer parameter: USR_INT8
     * 
     * Demonstrates AP_Int8 usage for small integer values or boolean flags.
     * Range: -128 to 127 (signed 8-bit)
     * Storage: 1 byte in EEPROM
     * Access: copter.user_parameters._int8
     * 
     * Use case: Flags, small counts, or enumeration values
     * Default value: 0
     * Index: 0 (first user parameter)
     */
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    
    /**
     * Example 16-bit integer parameter: USR_INT16
     * 
     * Demonstrates AP_Int16 usage for medium-range integer values.
     * Range: -32768 to 32767 (signed 16-bit)
     * Storage: 2 bytes in EEPROM
     * Access: copter.user_parameters._int16
     * 
     * Use case: PWM values, medium-range measurements, scaled integers
     * Default value: 0
     * Index: 1 (second user parameter)
     */
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    
    /**
     * Example floating point parameter: USR_FLOAT
     * 
     * Demonstrates AP_Float usage for precise decimal values.
     * Range: ±3.4e38 (32-bit IEEE 754 floating point)
     * Storage: 4 bytes in EEPROM
     * Access: copter.user_parameters._float
     * 
     * Use case: Tuning gains, scaling factors, measurements requiring precision
     * Default value: 0.0
     * Index: 2 (third user parameter)
     * 
     * Note: For parameters requiring units, document units in comments or parameter
     *       description metadata (future enhancement).
     */
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),

    /**
     * Parameter table terminator - MUST be last entry
     * 
     * AP_GROUPEND marks the end of the var_info array. The AP_Param system
     * uses this sentinel value to determine table length during registration.
     * Do not remove or modify this entry.
     */
    AP_GROUPEND
};

/**
 * @brief Constructor for UserParameters class
 * 
 * @details Initializes the UserParameters object and registers parameter defaults
 *          with the AP_Param system. This constructor is called during Copter object
 *          initialization before parameter values are loaded from EEPROM.
 * 
 *          INITIALIZATION SEQUENCE:
 *          1. Constructor called during Copter object construction
 *          2. setup_object_defaults() registers var_info table with AP_Param
 *          3. Default values from var_info are applied to member variables
 *          4. AP_Param::load_all() later restores saved values from EEPROM
 *          5. If no saved value exists, default from var_info is retained
 * 
 *          INTEGRATION WITH NESTED OBJECT PATTERN:
 *          UserParameters is a nested object within Copter::Parameters, which is
 *          itself nested within the main Copter class. This nested structure allows
 *          logical grouping of parameters while maintaining a flat parameter namespace
 *          for MAVLink and ground station access.
 * 
 *          Structure hierarchy:
 *          Copter
 *            └── Parameters (g)
 *                  └── UserParameters
 * 
 *          AP_Param::setup_object_defaults() FUNCTIONALITY:
 *          - Iterates through var_info table entries
 *          - Sets each member variable to its default value
 *          - Registers parameter metadata (name, type, storage location)
 *          - Enables parameter discovery via MAVLink PARAM_REQUEST_LIST
 * 
 * @note This constructor runs early in system initialization before EEPROM access
 * @note Parameter values are not persistent until after load_all() completes
 * @note Member variable addresses must remain stable after construction
 * 
 * @see AP_Param::setup_object_defaults() for registration implementation
 * @see AP_Param::load_all() for EEPROM restoration
 * @see Parameters.cpp for UserParameters object instantiation
 */
UserParameters::UserParameters()
{
    // Register this object's parameters with the AP_Param system
    // This call associates each member variable with its var_info table entry,
    // applies default values, and enables parameter persistence and MAVLink access
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // USER_PARAMS_ENABLED
