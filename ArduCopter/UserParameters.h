/**
 * @file UserParameters.h
 * @brief User-defined custom parameters for ArduCopter
 * 
 * @details This file provides a framework for users to define their own custom parameters
 *          in ArduCopter without modifying the core parameter system. The UserParameters
 *          class allows users to add application-specific parameters that persist across
 *          reboots and can be accessed via MAVLink ground control stations.
 * 
 *          The UserParameters system integrates with the AP_Param parameter storage
 *          framework, providing automatic persistence, MAVLink parameter protocol support,
 *          and ground station configuration capabilities.
 * 
 * ## Purpose
 * 
 * This system enables users to:
 * - Define custom parameters for specific mission requirements
 * - Store user-specific configuration data persistently
 * - Access parameters from Lua scripts via the scripting API
 * - Modify parameters through ground control stations
 * - Maintain separation between core and user parameters
 * 
 * ## Integration with AP_Param Framework
 * 
 * UserParameters leverages the AP_Param framework which provides:
 * - Persistent storage in EEPROM/flash
 * - MAVLink parameter protocol support (PARAM_REQUEST_LIST, PARAM_SET, etc.)
 * - Type-safe parameter access
 * - Parameter validation and range checking
 * - Default value management
 * 
 * ## Usage Pattern
 * 
 * To add custom parameters:
 * 
 * 1. **Define parameter variables** in the private section:
 *    @code
 *    AP_Int16 _my_custom_value;    // Custom integer parameter
 *    AP_Float _my_threshold;        // Custom float parameter
 *    @endcode
 * 
 * 2. **Add entries to var_info array** in UserParameters.cpp:
 *    @code
 *    const AP_Param::GroupInfo UserParameters::var_info[] = {
 *        // @Param: MY_VALUE
 *        // @DisplayName: My Custom Value
 *        // @Description: Description of what this parameter controls
 *        // @Range: 0 1000
 *        // @User: Standard
 *        AP_GROUPINFO("MY_VALUE", 1, UserParameters, _my_custom_value, 100),
 *        
 *        AP_GROUPEND
 *    };
 *    @endcode
 * 
 * 3. **Create accessor methods** for type-safe access:
 *    @code
 *    int16_t get_my_custom_value() const { return _my_custom_value; }
 *    void set_my_custom_value(int16_t val) { _my_custom_value.set(val); }
 *    @endcode
 * 
 * 4. **Access from vehicle code**:
 *    @code
 *    // In ArduCopter code:
 *    int16_t value = copter.g2.user_parameters.get_my_custom_value();
 *    @endcode
 * 
 * ## Parameter Naming Convention
 * 
 * User parameters appear in ground stations with the prefix configured in
 * Parameters.cpp when this object is registered. Typically they appear as:
 * - USER_INT8
 * - USER_INT16
 * - USER_FLOAT
 * 
 * The "USER_" prefix distinguishes them from core ArduCopter parameters.
 * 
 * ## Available Parameter Types
 * 
 * The AP_Param framework supports the following types:
 * - AP_Int8: 8-bit signed integer (-128 to 127)
 * - AP_Int16: 16-bit signed integer (-32768 to 32767)
 * - AP_Int32: 32-bit signed integer
 * - AP_Float: Single-precision floating point
 * 
 * ## Access from Lua Scripts
 * 
 * Custom user parameters can be accessed from Lua scripts using the parameter API:
 * @code{.lua}
 * -- Read user parameter
 * local value = param:get('USER_INT16')
 * 
 * -- Write user parameter
 * param:set('USER_FLOAT', 3.14)
 * @endcode
 * 
 * ## Memory Considerations
 * 
 * Each parameter consumes storage space:
 * - AP_Int8: 1 byte + overhead
 * - AP_Int16: 2 bytes + overhead
 * - AP_Int32: 4 bytes + overhead
 * - AP_Float: 4 bytes + overhead
 * 
 * The AP_Param system uses EEPROM or flash storage, which has limited
 * write cycles. Avoid frequent parameter updates in flight-critical loops.
 * 
 * ## Thread Safety
 * 
 * Parameter reads are thread-safe. Parameter writes from multiple threads
 * should be avoided or protected with appropriate locking mechanisms.
 * 
 * @note This is a template/example implementation. Users should modify this
 *       file to add their specific parameters based on their requirements.
 * 
 * @warning Do not add parameters that control flight-critical functions
 *          without thorough testing and understanding of safety implications.
 * 
 * @see AP_Param for the underlying parameter storage framework
 * @see Parameters.cpp for integration into the main parameter system
 * @see libraries/AP_Param/AP_Param.h for AP_Param API details
 * 
 * Source: ArduCopter/UserParameters.h
 */

#pragma once

#include <AP_Param/AP_Param.h>

/**
 * @class UserParameters
 * @brief Container for user-defined custom parameters
 * 
 * @details This class provides a mechanism for users to define custom parameters
 *          that are stored persistently and accessible via MAVLink. It integrates
 *          with ArduCopter's parameter system through the AP_Param framework.
 * 
 *          The UserParameters object is instantiated as part of the Parameters_g2
 *          structure (copter.g2.user_parameters) and is accessible throughout
 *          the ArduCopter codebase.
 * 
 * ## Design Pattern
 * 
 * This class follows the AP_Param group pattern used throughout ArduPilot:
 * - Private member variables store the actual parameter data
 * - Public accessor methods provide controlled access
 * - Static var_info array defines parameter metadata
 * - Integration with main parameter system via Parameters.cpp
 * 
 * ## Lifecycle
 * 
 * 1. **Construction**: UserParameters() constructor is called during system init
 * 2. **Load**: AP_Param::load_object() loads values from persistent storage
 * 3. **Runtime**: Parameters accessed via accessor methods
 * 4. **Save**: Changes automatically persisted by AP_Param framework
 * 
 * ## Example Expansion
 * 
 * To add a new parameter for mission-specific logic:
 * 
 * @code
 * // In UserParameters.h private section:
 * AP_Int16 _mission_mode;      // Custom mission mode selector
 * AP_Float _detection_threshold; // Object detection threshold
 * 
 * // In UserParameters.h public section:
 * int16_t get_mission_mode() const { return _mission_mode; }
 * float get_detection_threshold() const { return _detection_threshold; }
 * 
 * // In UserParameters.cpp var_info:
 * // @Param: MISSION_MODE
 * // @DisplayName: Mission Mode
 * // @Description: Selects which mission profile to use
 * // @Values: 0:Standard, 1:Survey, 2:Inspection
 * // @User: Standard
 * AP_GROUPINFO("MISSION_MODE", 4, UserParameters, _mission_mode, 0),
 * 
 * // @Param: DETECT_THR
 * // @DisplayName: Detection Threshold
 * // @Description: Threshold for object detection algorithm
 * // @Range: 0.0 1.0
 * // @Increment: 0.01
 * // @User: Advanced
 * AP_GROUPINFO("DETECT_THR", 5, UserParameters, _detection_threshold, 0.5),
 * @endcode
 * 
 * @note The var_info array index (4, 5 in example) must be unique and never reused
 *       to maintain parameter storage compatibility across firmware versions.
 * 
 * @warning Changing parameter types or removing parameters can cause issues with
 *          saved parameter values. Use careful versioning and migration strategies.
 */
class UserParameters {

public:
    /**
     * @brief Default constructor for UserParameters
     * 
     * @details Initializes the UserParameters object. The actual parameter values
     *          are loaded from persistent storage by the AP_Param framework after
     *          construction via AP_Param::load_object().
     * 
     *          Default values for parameters are specified in the var_info array
     *          in UserParameters.cpp and are used when parameters have not been
     *          previously saved or after EEPROM/parameter reset.
     * 
     * @note This constructor is called during ArduCopter initialization before
     *       parameters are loaded from storage.
     */
    UserParameters();
    
    /**
     * @brief Parameter metadata table for AP_Param framework
     * 
     * @details This static array defines all user parameters, their storage locations,
     *          default values, and metadata. It is used by the AP_Param system to:
     *          - Identify and load parameters from persistent storage
     *          - Provide parameter information to ground control stations
     *          - Validate parameter values and types
     *          - Generate parameter documentation
     * 
     *          The var_info array must be defined in UserParameters.cpp and should
     *          follow this pattern:
     * 
     *          @code
     *          const AP_Param::GroupInfo UserParameters::var_info[] = {
     *              // @Param: PARAM_NAME
     *              // @DisplayName: Human Readable Name
     *              // @Description: Detailed description of parameter function
     *              // @Range: min max
     *              // @Units: unit_symbol
     *              // @Increment: step_value
     *              // @User: Standard|Advanced|Developer
     *              AP_GROUPINFO("PARAM_NAME", index, UserParameters, _variable, default_value),
     *              
     *              AP_GROUPEND  // Required terminator
     *          };
     *          @endcode
     * 
     *          Each AP_GROUPINFO entry includes:
     *          - Parameter name as it appears in ground stations
     *          - Unique index for storage (never reuse indices)
     *          - Class name (UserParameters)
     *          - Member variable name
     *          - Default value
     * 
     * @note The index in AP_GROUPINFO must be unique within this parameter group
     *       and should never be changed or reused to maintain storage compatibility.
     * 
     * @warning The array must end with AP_GROUPEND marker for proper AP_Param operation.
     * 
     * @see UserParameters.cpp for the actual var_info implementation
     * @see libraries/AP_Param/AP_Param.h for AP_GROUPINFO macro details
     */
    static const struct AP_Param::GroupInfo var_info[];

    // Accessor Methods
    // Put accessors to your parameter variables here
    
    /**
     * @brief Get the 8-bit integer example parameter value
     * 
     * @details Provides read-only access to the example AP_Int8 parameter.
     *          This demonstrates the pattern for creating type-safe parameter
     *          accessors that can be used throughout the ArduCopter codebase.
     * 
     * ## Usage Example
     * @code
     * // Access from vehicle code:
     * int8_t value = copter.g2.user_parameters.get_int8Param();
     * 
     * // Use in conditional logic:
     * if (copter.g2.user_parameters.get_int8Param() > 50) {
     *     // Custom behavior based on user parameter
     * }
     * @endcode
     * 
     * @return The current value of the INT8 user parameter
     * 
     * @note This is an example accessor. Replace with meaningful names and
     *       documentation for your specific parameters.
     */
    AP_Int8 get_int8Param() const { return _int8; }
    
    /**
     * @brief Get the 16-bit integer example parameter value
     * 
     * @details Provides read-only access to the example AP_Int16 parameter.
     *          AP_Int16 is suitable for parameters requiring larger integer
     *          ranges than AP_Int8 (-32768 to 32767).
     * 
     * ## Usage Example
     * @code
     * // Access from Lua script via scripting bindings:
     * local user_val = param:get('USER_INT16')
     * 
     * // Access from C++ code:
     * int16_t threshold = copter.g2.user_parameters.get_int16Param();
     * @endcode
     * 
     * @return The current value of the INT16 user parameter
     * 
     * @note This is an example accessor. Modify to match your parameter purpose.
     */
    AP_Int16 get_int16Param() const { return _int16; }
    
    /**
     * @brief Get the floating-point example parameter value
     * 
     * @details Provides read-only access to the example AP_Float parameter.
     *          AP_Float is suitable for parameters requiring decimal precision,
     *          such as thresholds, scaling factors, or measurement values.
     * 
     * ## Usage Example
     * @code
     * // Use in calculations:
     * float scale = copter.g2.user_parameters.get_floatParam();
     * float adjusted_value = raw_value * scale;
     * 
     * // Compare with threshold:
     * if (sensor_reading > copter.g2.user_parameters.get_floatParam()) {
     *     trigger_action();
     * }
     * @endcode
     * 
     * @return The current value of the FLOAT user parameter
     * 
     * @note This is an example accessor. Replace with appropriate name and logic
     *       for your specific floating-point parameter needs.
     * 
     * @warning Floating-point comparisons should account for precision limits.
     *          Use appropriate epsilon values for equality checks.
     */
    AP_Float get_floatParam() const { return _float; }

private:
    // Parameter Variable Definitions
    // Put your parameter variable definitions here
    
    /**
     * @brief Example 8-bit integer parameter
     * 
     * @details This is a template parameter demonstrating AP_Int8 usage.
     *          AP_Int8 is appropriate for parameters with small integer ranges
     *          (-128 to 127), such as mode selectors, enable flags, or small counts.
     * 
     *          Replace this with your own parameters following this pattern:
     *          - Use descriptive variable names (e.g., _detection_mode, _retry_count)
     *          - Add corresponding entry in var_info array in UserParameters.cpp
     *          - Create public accessor method(s) for controlled access
     *          - Document the parameter's purpose and valid range
     * 
     * ## Storage
     * Stored persistently via AP_Param framework. Value survives reboots and
     * parameter resets (unless EEPROM is erased).
     * 
     * ## Access Pattern
     * - Read: via get_int8Param() accessor or direct access in member methods
     * - Write: via AP_Int8::set() method or MAVLink parameter set
     * - From GCS: Appears as USER_INT8 (or custom name defined in var_info)
     * 
     * @note This is an example. Delete or rename for your application.
     */
    AP_Int8 _int8;
    
    /**
     * @brief Example 16-bit integer parameter
     * 
     * @details This is a template parameter demonstrating AP_Int16 usage.
     *          AP_Int16 provides larger integer range (-32768 to 32767) suitable
     *          for parameters like:
     *          - Altitude thresholds (in centimeters or meters)
     *          - Speed limits (in cm/s)
     *          - Timeout values (in milliseconds)
     *          - Waypoint indices or mission counts
     * 
     *          Replace this with your own parameters following this pattern:
     *          - Use descriptive names (e.g., _altitude_limit, _mission_index)
     *          - Define valid range in var_info with @Range tag
     *          - Document units clearly (meters, cm, seconds, ms, etc.)
     *          - Provide sensible default value in var_info
     * 
     * ## Memory Footprint
     * Consumes 2 bytes of RAM plus AP_Param storage overhead.
     * 
     * ## Thread Safety
     * Parameter reads are atomic. Writes should be coordinated if accessed
     * from multiple execution contexts.
     * 
     * @note This is an example. Delete or rename for your application.
     */
    AP_Int16 _int16;
    
    /**
     * @brief Example floating-point parameter
     * 
     * @details This is a template parameter demonstrating AP_Float usage.
     *          AP_Float provides single-precision floating-point storage suitable
     *          for parameters requiring decimal precision:
     *          - Scaling factors (0.5, 1.5, 2.0)
     *          - Thresholds with fractional values
     *          - Physical measurements (temperature, voltage, current)
     *          - Tuning gains (proportional, derivative coefficients)
     * 
     *          Replace this with your own parameters following this pattern:
     *          - Use descriptive names (e.g., _gain_multiplier, _temp_threshold)
     *          - Specify range and increment in var_info for GCS slider control
     *          - Document units and typical values
     *          - Consider precision requirements (float = ~7 decimal digits)
     * 
     * ## Precision Considerations
     * Single-precision floats have limited precision (~7 significant digits).
     * For calculations requiring higher precision, consider using double in
     * your code and only storing as float.
     * 
     * ## Memory Footprint
     * Consumes 4 bytes of RAM plus AP_Param storage overhead.
     * 
     * ## Default Value
     * Specified in var_info array. Should represent a safe, typical starting value.
     * 
     * @note This is an example. Delete or rename for your application.
     * 
     * @warning When using floats in comparisons, account for floating-point
     *          precision limitations. Use epsilon-based comparisons for equality.
     */
    AP_Float _float;
};
