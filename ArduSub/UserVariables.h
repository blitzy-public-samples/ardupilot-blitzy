/**
 * @file UserVariables.h
 * @brief User-defined variables header for custom ArduSub extensions
 * 
 * @details This file provides a designated location for users to declare custom variables
 *          that are needed across multiple functions in their UserCode.cpp implementations.
 *          Variables declared here become members of the Sub class and are accessible
 *          throughout the vehicle code when USERHOOK_VARIABLES is defined.
 * 
 *          The USERHOOK system provides a safe extension mechanism that allows users to
 *          add custom functionality without modifying core ArduSub code. This approach
 *          ensures that user customizations can be easily maintained across ArduSub updates.
 * 
 * @section usage Usage Pattern
 * 
 * To use this file:
 * 1. Enable USERHOOK_VARIABLES in your build configuration or APM_Config.h
 * 2. Declare your custom variables within the #ifdef USERHOOK_VARIABLES block
 * 3. Initialize and use these variables in UserCode.cpp hook functions
 * 4. Access variables as sub.your_variable_name from within Sub class methods
 * 
 * @section example Example Usage
 * 
 * @code{.cpp}
 * #ifdef USERHOOK_VARIABLES
 * // Custom sensor data
 * float my_sensor_value;
 * uint32_t my_last_update_ms;
 * 
 * // Custom state tracking
 * bool my_feature_enabled;
 * int16_t my_custom_mode;
 * #endif
 * @endcode
 * 
 * @note Variables declared here become part of the Sub class and persist for the lifetime
 *       of the vehicle. Initialize them appropriately in userhook_init() in UserCode.cpp.
 * 
 * @note Avoid using variable names that conflict with existing Sub class member variables.
 *       Use a distinctive prefix (e.g., "user_", "custom_", or your project name) to
 *       minimize the risk of naming conflicts with core ArduSub variables.
 * 
 * @note For simple local variables only needed within a single UserCode.cpp function,
 *       consider declaring them as static variables within that function instead of
 *       adding them here as class members.
 * 
 * @warning Changes to this file require recompilation. The USERHOOK_VARIABLES define
 *          must be set before including this file for the variables to be declared.
 * 
 * @warning User variables consume RAM. Be mindful of memory constraints, especially
 *          on boards with limited memory. Each variable declared here increases the
 *          size of the Sub class instance.
 * 
 * @see UserCode.cpp for the implementation hooks where these variables can be used
 * @see APM_Config.h for enabling USERHOOK_VARIABLES and other user hook defines
 * 
 * Source: ArduSub/UserVariables.h:1-18
 */

// Example variables used in Wii camera testing - replace with your own variables
// This example demonstrates the proper format for declaring user-defined variables
#ifdef USERHOOK_VARIABLES

// Example: Wii Camera Integration Variables
// The following variables demonstrate proper declaration patterns for custom sensors
#if WII_CAMERA == 1
    WiiCamera           ircam;              ///< Wii camera object for IR tracking
    int                 WiiRange=0;         ///< Detected range from IR camera (implementation-specific units)
    int                 WiiRotation=0;      ///< Detected rotation angle from IR camera
    int                 WiiDisplacementX=0; ///< Horizontal displacement from camera center
    int                 WiiDisplacementY=0; ///< Vertical displacement from camera center
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


