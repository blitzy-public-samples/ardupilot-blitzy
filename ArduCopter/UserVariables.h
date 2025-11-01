/**
 * @file UserVariables.h
 * @brief User-defined variable extensibility system for ArduPilot Copter
 * 
 * @details This file provides a mechanism for users to add custom member variables
 *          to the Copter class without modifying core ArduPilot source files. This
 *          extensibility system allows developers and researchers to implement custom
 *          functionality, experimental features, or vehicle-specific behaviors while
 *          maintaining compatibility with upstream ArduPilot updates.
 * 
 * ## Purpose and Design
 * 
 * The USERHOOK_VARIABLES system enables users to:
 * - Add custom member variables to the main Copter class
 * - Store persistent state for custom algorithms
 * - Interface with external hardware or sensors
 * - Implement experimental control features
 * - Maintain vehicle-specific customizations
 * 
 * Variables defined here become members of the Copter class and can be accessed
 * from custom code in UserCode.cpp, allowing complete integration with the
 * vehicle's main loop and control systems.
 * 
 * ## How to Use This File
 * 
 * To add your own custom variables:
 * 
 * 1. **Enable the user hook system** by defining USERHOOK_VARIABLES in your
 *    build configuration (typically in APM_Config.h or via build flags):
 *    ```cpp
 *    #define USERHOOK_VARIABLES 1
 *    ```
 * 
 * 2. **Add your variable declarations** inside the USERHOOK_VARIABLES block:
 *    ```cpp
 *    #ifdef USERHOOK_VARIABLES
 *        // Custom sensor interface
 *        MyCustomSensor custom_sensor;
 *        
 *        // State tracking variables
 *        float my_altitude_offset_cm;
 *        uint32_t my_last_update_ms;
 *        bool my_feature_enabled;
 *        
 *        // Custom control variables
 *        Vector3f my_position_target;
 *    #endif
 *    ```
 * 
 * 3. **Initialize and use variables** in UserCode.cpp:
 *    ```cpp
 *    void Copter::userhook_init()
 *    {
 *        // Initialize your variables
 *        my_altitude_offset_cm = 0.0f;
 *        my_last_update_ms = 0;
 *        my_feature_enabled = false;
 *        custom_sensor.init();
 *    }
 *    
 *    void Copter::userhook_FastLoop()
 *    {
 *        // Access variables at 400Hz main loop rate
 *        if (my_feature_enabled) {
 *            custom_sensor.update();
 *            my_last_update_ms = AP_HAL::millis();
 *        }
 *    }
 *    ```
 * 
 * ## Integration with ArduPilot Extensibility System
 * 
 * This file is part of the complete user hook system:
 * 
 * - **UserVariables.h** (this file): Declares custom member variables for Copter class
 * - **UserCode.cpp**: Implements custom code using the variables (init, loops, hooks)
 * - **UserParameters.h**: Defines custom parameters exposed to ground stations
 * - **UserParameters.cpp**: Implements custom parameter handling and persistence
 * 
 * Together, these files allow complete custom functionality without modifying core files.
 * 
 * ## Variable Naming Conventions
 * 
 * To avoid conflicts with future ArduPilot development:
 * - Use descriptive, project-specific prefixes (e.g., `myproject_`, `custom_`)
 * - Avoid generic names like `temp`, `data`, `value`
 * - Document the purpose and units of each variable
 * - Use appropriate types (prefer fixed-width types like uint32_t over int)
 * 
 * ## Memory and Performance Considerations
 * 
 * Variables declared here are allocated as part of the Copter class:
 * - They consume RAM for the lifetime of the vehicle
 * - Large objects (arrays, buffers) should be allocated carefully
 * - Consider using dynamic allocation for optional features
 * - Be mindful of total RAM usage on memory-constrained boards
 * 
 * ## Coordinate Systems and Units
 * 
 * When adding navigation or control variables:
 * - Use NED (North-East-Down) frame for earth-referenced positions/velocities
 * - Use body frame for vehicle-relative vectors
 * - Follow ArduPilot conventions: angles in centidegrees or radians (document clearly)
 * - Distances in centimeters or meters (document clearly)
 * - Velocities in cm/s or m/s (document clearly)
 * - Always document your coordinate frame and units explicitly
 * 
 * ## Example Use Cases
 * 
 * ### Example 1: Custom Sensor Integration
 * ```cpp
 * #ifdef USERHOOK_VARIABLES
 *     // Custom LIDAR sensor for terrain following
 *     MyLidarSensor terrain_lidar;
 *     float terrain_height_cm;          // Measured terrain height in cm
 *     uint32_t terrain_last_update_ms;  // Last valid measurement timestamp
 * #endif
 * ```
 * 
 * ### Example 2: Experimental Control Algorithm
 * ```cpp
 * #ifdef USERHOOK_VARIABLES
 *     // Experimental aggressive descent controller
 *     float descent_rate_target_cms;    // Target descent rate in cm/s
 *     float descent_altitude_start_cm;  // Altitude where aggressive descent began
 *     bool aggressive_descent_active;   // True when custom controller is active
 * #endif
 * ```
 * 
 * ### Example 3: Competition-Specific Variables
 * ```cpp
 * #ifdef USERHOOK_VARIABLES
 *     // Autonomous racing waypoint tracking
 *     Vector3f race_waypoint_ned[10];   // Race course waypoints in NED frame
 *     uint8_t race_current_waypoint;    // Current target waypoint index
 *     uint32_t race_start_time_ms;      // Race start timestamp
 *     uint32_t race_lap_times_ms[3];    // Lap time recording
 * #endif
 * ```
 * 
 * ## Safety Considerations
 * 
 * @warning Custom variables can affect flight safety if used in control algorithms.
 *          Always test custom functionality thoroughly in SITL simulation before
 *          flying on real hardware.
 * 
 * @warning Ensure proper initialization of all variables in userhook_init() to avoid
 *          undefined behavior. Uninitialized variables can cause unpredictable flight
 *          characteristics or crashes.
 * 
 * @warning Be cautious when integrating custom code into the main loop (FastLoop).
 *          Excessive computation time can affect the 400Hz control loop timing and
 *          degrade flight performance or stability.
 * 
 * ## Testing Recommendations
 * 
 * 1. **SITL Testing**: Always test custom functionality in Software-In-The-Loop simulation:
 *    ```bash
 *    sim_vehicle.py -v ArduCopter --console --map
 *    ```
 * 
 * 2. **Parameter Validation**: Verify custom parameters load and save correctly
 * 
 * 3. **Failsafe Testing**: Ensure custom code doesn't interfere with failsafe mechanisms
 * 
 * 4. **Performance Monitoring**: Monitor CPU usage and loop timing with custom code active
 * 
 * 5. **Hardware-in-the-Loop**: Test on actual flight controller hardware before flying
 * 
 * ## Maintaining Compatibility with ArduPilot Updates
 * 
 * This file is specifically designed to be user-modified. To maintain compatibility:
 * - Keep all custom code within the USERHOOK_VARIABLES conditional block
 * - Do not modify code outside this file unless necessary
 * - When updating ArduPilot, preserve your UserVariables.h changes
 * - Test thoroughly after each ArduPilot update
 * - Consider using git branches for custom modifications
 * 
 * ## Additional Resources
 * 
 * - UserCode.cpp: Implementation of custom code hooks
 * - UserParameters.h/cpp: Custom parameter definition system
 * - ArduCopter/Copter.h: Main Copter class definition
 * - https://ardupilot.org/dev/: ArduPilot developer documentation
 * 
 * @note This file is included directly into the Copter class definition in Copter.h,
 *       making all variables here member variables of the main vehicle class.
 * 
 * @see UserCode.cpp for custom code implementation
 * @see UserParameters.h for custom parameter definitions
 * @see Copter.h for the main Copter class definition
 * 
 * Source: ArduCopter/UserVariables.h
 */

// Example variables used in Wii camera testing - replace with your own variables
// These serve as a reference implementation showing the basic usage pattern
#ifdef USERHOOK_VARIABLES

/**
 * Example: Wii Camera Integration Variables
 * 
 * This section demonstrates the user variable system with a real-world example
 * from Wii IR camera optical tracking experiments. Replace these with your own
 * custom variables for your specific application.
 * 
 * The Wii camera example shows:
 * - Hardware interface object (ircam)
 * - Measurement storage variables (range, rotation, displacement)
 * - Proper variable naming and organization
 * - Integration with conditional compilation (WII_CAMERA flag)
 * 
 * To use this example as a template:
 * 1. Replace WiiCamera with your hardware/sensor class
 * 2. Replace measurement variables with your data storage needs
 * 3. Initialize in userhook_init() in UserCode.cpp
 * 4. Update values in userhook_FastLoop() or other loop hooks
 */
#if WII_CAMERA == 1
WiiCamera           ircam;              ///< Wii IR camera interface object
int                 WiiRange=0;         ///< Measured range from IR camera (units: implementation-defined)
int                 WiiRotation=0;      ///< Rotation angle from IR tracking (units: implementation-defined)
int                 WiiDisplacementX=0; ///< Horizontal displacement in image coordinates (pixels or normalized)
int                 WiiDisplacementY=0; ///< Vertical displacement in image coordinates (pixels or normalized)
#endif  // WII_CAMERA

#endif  // USERHOOK_VARIABLES


