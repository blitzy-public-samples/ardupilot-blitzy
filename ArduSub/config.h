/**
 * @file config.h
 * @brief ArduSub build-time configuration settings and default parameter values
 * 
 * @details This header file defines compile-time configuration options and default values
 *          for the ArduSub underwater vehicle firmware. These settings control:
 *          - Hardware timing parameters (main loop rate, PWM frequencies)
 *          - Default flight mode behavior and control gains
 *          - Feature flags for optional subsystems (circle nav, avoidance, etc.)
 *          - Sensor configuration (rangefinder, RC input)
 *          - Navigation and control parameters
 *          - Logging defaults
 * 
 *          All definitions use #ifndef guards allowing them to be overridden at build time
 *          through board-specific hwdef files or external configuration.
 * 
 * @note To customize these settings for a specific build, define the desired values
 *       in your board's hwdef.dat file or via compiler flags before this file is included.
 * 
 * @note Changing these values affects the compiled binary behavior. Extensive testing
 *       in SITL and controlled environments is required before deploying to real hardware.
 * 
 * @warning Many of these parameters affect underwater vehicle stability and safety.
 *          Modification should only be done by developers who understand the implications
 *          for buoyancy control, thruster allocation, and underwater navigation.
 * 
 * @see APM_Config.h for additional vehicle configuration options
 * @see defines.h for ArduSub-specific type and constant definitions
 * 
 * Source: ArduSub/config.h
 */

#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Hardware abstraction layer board definition requirement
 * 
 * @details CONFIG_HAL_BOARD must be defined by the build system to specify
 *          the target hardware platform (ChibiOS, Linux, SITL, etc.)
 */
#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduSub
#endif

/**
 * @brief Main scheduler loop execution rate in Hz
 * 
 * @details ArduSub runs its main control loop at 400Hz on all hardware platforms.
 *          This rate determines the frequency of:
 *          - Attitude controller updates
 *          - Sensor data processing
 *          - Motor output updates
 *          - Mode logic execution
 * 
 * @note This is a fixed rate (not configurable) to ensure consistent control
 *       performance across different hardware platforms.
 * 
 * @warning Changing this rate requires extensive retuning of all control loops
 *          and careful validation of timing-dependent code.
 */
# define MAIN_LOOP_RATE    400

/**
 * @brief Default depth threshold for considering vehicle surfaced
 * 
 * @details Pressure sensor reading indicating 10cm depth means the craft is
 *          considered at the surface. Negative value follows ArduSub convention
 *          where negative depths are above the zero reference point.
 * 
 * @note Units: centimeters (negative values = above reference)
 * @note Can be overridden via build configuration for different surface detection needs
 */
#ifndef SURFACE_DEPTH_DEFAULT
# define SURFACE_DEPTH_DEFAULT -10.0f // pressure sensor reading 10cm depth means craft is considered surfaced
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default PWM output frequency for motor/servo channels
 * 
 * @details Sets the default update rate for PWM outputs to thrusters and servos.
 *          200Hz provides a balance between command responsiveness and ESC compatibility.
 * 
 * @note Units: Hz (cycles per second)
 * @note Individual channels can override this rate via SERVOx_RATE parameters
 * @note Many marine ESCs operate optimally at 200-400Hz PWM rates
 */
#ifndef RC_SPEED_DEFAULT
#   define RC_SPEED_DEFAULT 200
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle Nav parameters
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enable Circle navigation mode support
 * 
 * @details When enabled (1), includes Circle mode functionality allowing the vehicle
 *          to maintain a circular path around a point. When disabled (0), removes
 *          Circle mode code to reduce firmware size.
 * 
 * @note Set to 0 to disable if Circle mode is not needed and flash space is constrained
 * @note Disabling saves approximately 2-3KB of flash memory
 */
#ifndef CIRCLE_NAV_ENABLED
# define CIRCLE_NAV_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// RC (Radio Control Input)
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enable RC input processing for ArduSub
 * 
 * @details Controls whether RC receiver input is processed. When enabled (1),
 *          the vehicle can accept pilot commands from RC transmitter/receiver.
 *          When disabled (0), RC input is ignored (useful for fully autonomous
 *          or companion computer-controlled vehicles).
 * 
 * @note Disabling RC requires alternative control method (MAVLink, scripting, etc.)
 */
#ifndef AP_SUB_RC_ENABLED
# define AP_SUB_RC_ENABLED 1
#endif

/**
 * @brief Enable RC channel mapping functionality
 * 
 * @details Automatically inherits the AP_SUB_RC_ENABLED setting. When enabled,
 *          allows remapping of RC input channels to different functions via
 *          RCMAP parameters (RCMAP_ROLL, RCMAP_PITCH, etc.).
 * 
 * @note Only relevant when RC input is enabled
 */
#ifndef RCMAP_ENABLED
# define RCMAP_ENABLED AP_SUB_RC_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default RC throttle PWM value indicating failsafe condition
 * 
 * @details RC throttle PWM values at or below this threshold trigger throttle
 *          failsafe actions. Typical RC receivers output ~1000μs when signal
 *          is lost, so 975μs provides margin to detect loss of signal.
 * 
 * @note Units: microseconds (PWM pulse width)
 * @note Can be adjusted via FS_THR_VALUE parameter for different RC systems
 * @note Values below this trigger failsafe actions (disarm, surface, hold position)
 * 
 * @warning Setting this too high may cause false failsafe triggers from valid
 *          low throttle commands. Setting too low may fail to detect signal loss.
 */
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif


//////////////////////////////////////////////////////////////////////////////
// Rangefinder (Distance Sensor Configuration)
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Number of consecutive good readings required for healthy rangefinder status
 * 
 * @details The rangefinder must provide this many consecutive valid measurements
 *          before being considered healthy and suitable for navigation use.
 *          This prevents spurious readings from triggering navigation responses.
 * 
 * @note Higher values increase confidence but slow initial acquisition
 */
#ifndef RANGEFINDER_HEALTH_MAX
# define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

/**
 * @brief Timeout period for rangefinder altitude validity
 * 
 * @details If no valid rangefinder reading is received within this time period,
 *          the desired rangefinder altitude resets to the current measured altitude.
 *          This prevents the vehicle from acting on stale rangefinder data.
 * 
 * @note Units: milliseconds
 * @note 1000ms (1 second) provides balance between responsiveness and timeout protection
 */
#ifndef RANGEFINDER_TIMEOUT_MS
# define RANGEFINDER_TIMEOUT_MS  1000      // desired rangefinder alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

/**
 * @brief Low-pass filter cutoff frequency for rangefinder data in waypoint navigation
 * 
 * @details Filters rangefinder altitude measurements before providing them to the
 *          waypoint navigation controller. Low frequency (0.25Hz) heavily smooths
 *          the data, reducing response to transient obstacles or noise.
 * 
 * @note Units: Hz (cycles per second)
 * @note Lower frequencies = more smoothing but slower response to real altitude changes
 */
#ifndef RANGEFINDER_WPNAV_FILT_HZ
# define RANGEFINDER_WPNAV_FILT_HZ   0.25f // filter frequency for rangefinder altitude provided to waypoint navigation class
#endif

/**
 * @brief Enable/disable tilt correction for rangefinder data used by EKF
 * 
 * @details When disabled (0), rangefinder data is used by EKF without correcting
 *          for vehicle tilt. This is appropriate for downward-facing rangefinders
 *          on underwater vehicles where tilt angles are typically small.
 * 
 * @note Set to 1 to enable tilt correction if rangefinder mounting or vehicle
 *       orientation results in significant tilt-induced measurement errors
 */
#ifndef RANGEFINDER_TILT_CORRECTION        // by disable tilt correction for use of range finder data by EKF
# define RANGEFINDER_TILT_CORRECTION 0
#endif

/**
 * @brief Minimum signal quality threshold for accepting rangefinder readings
 * 
 * @details Rangefinder measurements with signal quality below this value are
 *          rejected as unreliable. Quality is sensor-specific but typically
 *          ranges from 0 (poor) to 100 (excellent).
 * 
 * @note Units: Signal quality metric (0-100 scale for most sensors)
 * @note Value of 90 requires high-quality readings, filtering out marginal data
 */
#ifndef RANGEFINDER_SIGNAL_MIN_DEFAULT
# define RANGEFINDER_SIGNAL_MIN_DEFAULT 90 // rangefinder readings with signal quality below this value are ignored
#endif

/**
 * @brief Default target depth for surface tracking mode
 * 
 * @details In surface tracking (SURFTRAK) mode, the vehicle attempts to maintain
 *          this depth below the water surface using rangefinder feedback.
 *          Negative value follows ArduSub depth convention (negative = depth below surface).
 * 
 * @note Units: centimeters (negative values indicate depth below surface)
 * @note -50.0cm (0.5 meters) keeps vehicle safely submerged while tracking surface
 * @note Can be adjusted via SURFTRAK_DEPTH parameter for different operational needs
 */
#ifndef SURFTRAK_DEPTH_DEFAULT
# define SURFTRAK_DEPTH_DEFAULT -50.0f     // surftrak will try to keep the sub below this depth
#endif

/**
 * @brief Enable obstacle avoidance subsystem
 * 
 * @details Controls compilation of obstacle avoidance features using proximity sensors.
 *          When enabled (1), the vehicle can detect and avoid obstacles. Requires
 *          proximity sensor hardware and depends on Fence library.
 * 
 * @note Disabled by default (0) as it requires specific hardware (proximity sensors)
 * @note When enabled, automatically enables FENCE_ENABLED as avoidance relies on fence library
 * @note Enabling adds significant code size; only enable if obstacle avoidance is needed
 */
#ifndef AVOIDANCE_ENABLED
# define AVOIDANCE_ENABLED 0
#endif

/**
 * @brief Conditional fence enablement when avoidance is active
 * 
 * @details Automatically enables fence functionality when avoidance is enabled,
 *          as the avoidance library depends on the fence infrastructure for
 *          boundary management and safety enforcement.
 * 
 * @note This is not user-configurable; it's automatically set based on AVOIDANCE_ENABLED
 */
#if AVOIDANCE_ENABLED // Avoidance Library relies on Fence
# define FENCE_ENABLED 1
#endif

/**
 * @brief Default MAVLink system ID for this vehicle
 * 
 * @details Unique identifier for this vehicle in MAVLink network. Must be unique
 *          among all MAVLink devices in the same network to prevent message routing
 *          conflicts. Ground stations use this ID to address commands to this vehicle.
 * 
 * @note Valid range: 1-255
 * @note Can be changed via SYSID_THISMAV parameter
 * @note Should be set uniquely for each vehicle in multi-vehicle operations
 */
#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - External Navigation Control
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enable Guided mode for external navigation computer control
 * 
 * @details When enabled (1), allows external navigation systems (companion computers,
 *          ground stations) to control the vehicle via MAVLink GUIDED mode commands.
 *          This is essential for autonomous missions driven by external computers.
 * 
 * @note Enabled by default (1) as this is a common operational mode for ROVs
 * @note Disabling (0) removes Guided mode functionality and saves minimal flash space
 * @note Required for companion computer control and many advanced autonomy features
 */
#ifndef NAV_GUIDED
# define NAV_GUIDED    1
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Acro Mode - Acrobatic/Rate Control Mode
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Acro mode roll/pitch proportional gain
 * 
 * @details Proportional gain converting pilot stick input to desired roll/pitch rate
 *          in Acro mode. Higher values result in faster rotation rates for given
 *          stick deflection. This is a rate-to-rate gain, not an attitude controller.
 * 
 * @note Units: (deg/s) / (stick input percent)
 * @note Can be adjusted via ACRO_RP_P parameter for different pilot preferences
 */
#ifndef ACRO_RP_P
# define ACRO_RP_P                 4.5f
#endif

/**
 * @brief Acro mode yaw proportional gain
 * 
 * @details Proportional gain converting pilot yaw stick input to desired yaw rate.
 *          Default value (3.375) is 75% of roll/pitch gain, providing slightly
 *          less aggressive yaw response which most pilots find more controllable.
 * 
 * @note Units: (deg/s) / (stick input percent)
 * @note Can be adjusted via ACRO_YAW_P parameter
 */
#ifndef ACRO_YAW_P
# define ACRO_YAW_P                3.375f
#endif

/**
 * @brief Maximum lean angle for Acro leveling feature
 * 
 * @details When Acro leveling is active (pilot sticks centered), limits the
 *          maximum roll/pitch angle the vehicle will maintain. Prevents excessive
 *          tilt during automatic leveling.
 * 
 * @note Units: centidegrees (3000 = 30 degrees)
 * @note Can be adjusted via ACRO_LEVEL_MAX parameter
 */
#ifndef ACRO_LEVEL_MAX_ANGLE
# define ACRO_LEVEL_MAX_ANGLE      3000
#endif

/**
 * @brief Roll axis balance factor for asymmetric vehicles
 * 
 * @details Compensates for roll axis asymmetry in thruster placement or vehicle
 *          mass distribution. Value of 1.0 means symmetric (no compensation).
 *          Values != 1.0 scale the roll response to balance the vehicle.
 * 
 * @note Default 1.0 assumes symmetric vehicle configuration
 * @note Adjust if vehicle tends to roll faster in one direction
 */
#ifndef ACRO_BALANCE_ROLL
#define ACRO_BALANCE_ROLL          1.0f
#endif

/**
 * @brief Pitch axis balance factor for asymmetric vehicles
 * 
 * @details Compensates for pitch axis asymmetry in thruster placement or vehicle
 *          mass distribution. Value of 1.0 means symmetric (no compensation).
 * 
 * @note Default 1.0 assumes symmetric vehicle configuration
 * @note Adjust if vehicle tends to pitch faster in one direction (e.g., forward vs back)
 */
#ifndef ACRO_BALANCE_PITCH
#define ACRO_BALANCE_PITCH         1.0f
#endif

/**
 * @brief Default exponential curve factor for Acro mode stick response
 * 
 * @details Controls the shape of the stick response curve. 0.0 = linear response,
 *          higher values create more stick sensitivity near center (fine control)
 *          with reduced sensitivity near endpoints (coarse control).
 * 
 * @note Range: 0.0 (linear) to 1.0 (maximum expo)
 * @note Default 0.3 provides moderate expo for balanced control feel
 * @note Can be adjusted via ACRO_EXPO parameter for pilot preference
 */
#ifndef ACRO_EXPO_DEFAULT
#define ACRO_EXPO_DEFAULT          0.3f
#endif

//////////////////////////////////////////////////////////////////////////////
// AUTO Mode - Autonomous Mission Execution
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default yaw behavior during waypoint navigation
 * 
 * @details Determines how the vehicle orients its heading when navigating between
 *          waypoints in AUTO mode. WP_YAW_BEHAVIOR_CORRECT_XTRACK means the vehicle
 *          points toward the next waypoint, correcting for cross-track error.
 * 
 * @note Other options include pointing along track, maintaining current heading, etc.
 * @note Can be changed via WP_YAW_BEHAVIOR parameter
 * @note Affects vehicle orientation during autonomous missions
 */
#ifndef WP_YAW_BEHAVIOR_DEFAULT
# define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_CORRECT_XTRACK
#endif

/**
 * @brief Maximum yaw rotation rate during autonomous navigation
 * 
 * @details Limits how quickly the vehicle rotates in yaw when changing heading
 *          during AUTO mode. Prevents aggressive yaw maneuvers that could disturb
 *          vehicle stability or sensor operations (e.g., camera, sonar).
 * 
 * @note Units: degrees per second
 * @note 60 deg/s provides smooth but reasonably responsive heading changes
 * @note Can be adjusted via ATC_SLEW_YAW parameter for different operational needs
 */
#ifndef AUTO_YAW_SLEW_RATE
# define AUTO_YAW_SLEW_RATE    60              // degrees/sec
#endif

/**
 * @brief Minimum speed threshold for look-ahead yaw pointing
 * 
 * @details Below this speed, the vehicle maintains its current heading rather than
 *          pointing along its ground track. Prevents erratic yaw behavior when
 *          moving slowly or holding position in current.
 * 
 * @note Units: centimeters per second (100 cm/s = 1 m/s)
 * @note At speeds below this threshold, current heading is maintained
 * @note Helps prevent yaw oscillations during slow-speed maneuvering
 */
#ifndef YAW_LOOK_AHEAD_MIN_SPEED
# define YAW_LOOK_AHEAD_MIN_SPEED  100             // minimum ground speed in cm/s required before vehicle is aimed at ground course
#endif

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Maximum roll/pitch stick input range
 * 
 * @details Defines the maximum value for roll and pitch pilot inputs in stabilize
 *          mode. Input is scaled from -ROLL_PITCH_INPUT_MAX to +ROLL_PITCH_INPUT_MAX
 *          corresponding to full stick deflection.
 * 
 * @note Units: centidegrees (4500 = 45 degrees)
 * @note This is an input scaling value, not a physical angle limit
 */
#ifndef ROLL_PITCH_INPUT_MAX
# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
#endif

/**
 * @brief Default maximum lean angle for stabilize mode
 * 
 * @details Maximum roll and pitch angles the vehicle will attempt to achieve
 *          in stabilize mode when pilot commands full stick deflection.
 *          Limits vehicle tilt for safety and controllability.
 * 
 * @note Units: centidegrees (4500 = 45 degrees)
 * @note Can be adjusted via ANGLE_MAX parameter (typical range: 1000-8000 centidegrees)
 * @note Lower values improve stability, higher values allow more aggressive maneuvering
 * 
 * @warning Excessive lean angles can cause loss of depth control and vehicle instability
 *          in underwater operations due to thruster effectiveness reduction at high tilts
 */
#ifndef DEFAULT_ANGLE_MAX
# define DEFAULT_ANGLE_MAX         4500            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter position control gains
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Horizontal position controller proportional gain
 * 
 * @details Proportional gain for horizontal (XY) position control in loiter and
 *          position hold modes. Converts position error (in meters) to desired
 *          velocity command. Higher values result in more aggressive position correction.
 * 
 * @note Units: (m/s) / meter of position error
 * @note Can be tuned via PSC_POSXY_P parameter
 * @note Affects how quickly vehicle returns to target position when disturbed
 */
#ifndef POS_XY_P
# define POS_XY_P               1.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PosHold parameter defaults
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enable PosHold flight mode
 * 
 * @details Controls whether PosHold mode is compiled into the firmware. PosHold
 *          mode maintains position using GPS or other position sensors, allowing
 *          pilot to override position with stick inputs then return to autonomous
 *          position hold when sticks are released.
 * 
 * @note Enabled by default (1) as this is a commonly used mode
 * @note Disabling (0) saves minimal flash space but removes PosHold functionality
 * @note Requires position estimation (GPS, visual odometry, etc.) to function
 */
#ifndef POSHOLD_ENABLED
# define POSHOLD_ENABLED               1 // PosHold flight mode enabled by default
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Throttle stick deadzone for altitude hold modes
 * 
 * @details Defines a deadzone range around mid-throttle where no vertical velocity
 *          is commanded in altitude hold or loiter modes. This prevents small stick
 *          movements or trim offsets from causing unwanted vertical motion.
 * 
 * @note Units: PWM microseconds deviation from mid-stick (typically 1500μs)
 * @note 100μs deadzone = ±100μs around mid-throttle is considered "neutral"
 * @note Can be adjusted via THR_DZ parameter for different pilot preferences
 */
#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

/**
 * @brief Default maximum pilot-commanded vertical velocity
 * 
 * @details Maximum vertical (depth) velocity the pilot can command with full
 *          throttle stick deflection in depth-hold modes. Limits ascent/descent
 *          rate for safety and controllability.
 * 
 * @note Units: centimeters per second (500 cm/s = 5 m/s)
 * @note Can be adjusted via PILOT_VELZ_MAX parameter
 * @note Conservative default prevents rapid depth changes that could damage vehicle
 * 
 * @warning Excessive vertical velocities can cause barotrauma to vehicle components,
 *          loss of buoyancy control, or exceed vehicle structural limits
 */
#ifndef PILOT_VELZ_MAX
# define PILOT_VELZ_MAX    500     // maximum vertical velocity in cm/s
#endif

/**
 * @brief Default maximum pilot-commanded horizontal velocity
 * 
 * @details Maximum horizontal velocity the pilot can command with full stick
 *          deflection in position-controlled modes (loiter, guided, etc.).
 * 
 * @note Units: centimeters per second (200 cm/s = 2 m/s)
 * @note Can be adjusted via PILOT_SPEED_UP parameter
 * @note Affects responsiveness in position hold modes
 */
#ifndef PILOT_SPEED_DEFAULT
# define PILOT_SPEED_DEFAULT 200 // maximum horizontal velocity in cm/s while under pilot control
#endif

/**
 * @brief Default vertical acceleration limit for pilot control
 * 
 * @details Maximum vertical acceleration applied when pilot commands depth changes.
 *          Limits rate of vertical velocity change to prevent abrupt depth transitions
 *          that could destabilize the vehicle or trigger depth sensor errors.
 * 
 * @note Units: centimeters per second squared (100 cm/s² = 1 m/s²)
 * @note Can be adjusted via PILOT_ACCEL_Z parameter
 * @note Smoother depth changes improve control stability and sensor fusion
 */
#ifndef PILOT_ACCEL_Z_DEFAULT
# define PILOT_ACCEL_Z_DEFAULT 100 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

/**
 * @brief Delay before automatic disarming when landed
 * 
 * @details Time delay before the vehicle automatically disarms after detecting
 *          landed/surfaced condition. Value of 0 disables automatic disarming,
 *          requiring manual disarm by pilot.
 * 
 * @note Units: seconds (0 = disabled)
 * @note Disabled by default (0) for ArduSub as underwater vehicles often rest
 *       on bottom or surface while still armed for mission operations
 * @note Can be changed via DISARM_DELAY parameter if auto-disarm is desired
 */
#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  0
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default logging bitmask - specifies which data streams to log
 * 
 * @details Defines the default set of data streams that are logged to the SD card
 *          or dataflash. Each bit corresponds to a different log message type.
 *          This comprehensive default captures essential data for post-mission
 *          analysis and troubleshooting:
 *          - MASK_LOG_ATTITUDE_MED: Medium-rate attitude data (roll, pitch, yaw)
 *          - MASK_LOG_GPS: GPS position, velocity, and status
 *          - MASK_LOG_PM: Performance monitoring (CPU load, loop timing)
 *          - MASK_LOG_CTUN: Control tuning data (controller outputs and errors)
 *          - MASK_LOG_NTUN: Navigation tuning (position/velocity errors)
 *          - MASK_LOG_RCIN: RC input channels
 *          - MASK_LOG_IMU: Inertial measurement unit data (gyro, accel)
 *          - MASK_LOG_CMD: Mission commands and events
 *          - MASK_LOG_CURRENT: Battery current and voltage
 *          - MASK_LOG_RCOUT: Motor/servo output channels
 *          - MASK_LOG_OPTFLOW: Optical flow sensor data
 *          - MASK_LOG_COMPASS: Magnetometer data
 *          - MASK_LOG_CAMERA: Camera trigger events
 *          - MASK_LOG_MOTBATT: Motor battery monitoring
 * 
 * @note Can be customized via LOG_BITMASK parameter to enable/disable specific logs
 * @note More logging increases SD card usage but provides better diagnostic capability
 * @note High-rate logging (all messages) can impact performance on slower SD cards
 * 
 * @see AP_Logger library for complete list of available log message types
 */
#ifndef DEFAULT_LOG_BITMASK
# define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//////////////////////////////////////////////////////////////////////////////
// Default flight modes
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default mode for flight mode switch position 1
 * 
 * @details MANUAL mode provides direct pilot control with no stabilization.
 *          Pilot inputs directly control thruster outputs. Used for testing
 *          and situations where direct control is needed.
 * 
 * @note Can be changed via FLTMODE1 parameter
 * @note Mode assignments can be customized per vehicle configuration
 */
#ifndef FLIGHT_MODE_1
# define FLIGHT_MODE_1 Mode::Number::MANUAL
#endif

/**
 * @brief Default mode for flight mode switch position 2
 * 
 * @details MANUAL mode (repeated for convenience on multi-position switches).
 *          Provides redundant access to direct control mode.
 * 
 * @note Can be changed via FLTMODE2 parameter
 */
#ifndef FLIGHT_MODE_2
# define FLIGHT_MODE_2 Mode::Number::MANUAL
#endif

/**
 * @brief Default mode for flight mode switch position 3
 * 
 * @details STABILIZE mode provides attitude stabilization. Vehicle maintains
 *          level attitude when pilot inputs are centered. Most commonly used
 *          mode for normal ROV operations.
 * 
 * @note Can be changed via FLTMODE3 parameter
 * @note Recommended as primary operational mode for most missions
 */
#ifndef FLIGHT_MODE_3
# define FLIGHT_MODE_3 Mode::Number::STABILIZE
#endif

/**
 * @brief Default mode for flight mode switch position 4
 * 
 * @details STABILIZE mode (repeated for convenience). Provides redundant access
 *          to stabilized control mode.
 * 
 * @note Can be changed via FLTMODE4 parameter
 */
#ifndef FLIGHT_MODE_4
# define FLIGHT_MODE_4 Mode::Number::STABILIZE
#endif

/**
 * @brief Default mode for flight mode switch position 5
 * 
 * @details SURFACE mode automatically drives the vehicle to the surface and
 *          maintains position there. Emergency ascent mode for safety.
 * 
 * @note Can be changed via FLTMODE5 parameter
 * @note Useful as emergency mode assignment for quick surface access
 */
#ifndef FLIGHT_MODE_5
# define FLIGHT_MODE_5 Mode::Number::SURFACE
#endif

/**
 * @brief Default mode for flight mode switch position 6
 * 
 * @details SURFACE mode (repeated for convenience). Provides redundant access
 *          to emergency surface mode.
 * 
 * @note Can be changed via FLTMODE6 parameter
 * @note Having multiple switch positions assigned to SURFACE ensures easy emergency access
 */
#ifndef FLIGHT_MODE_6
# define FLIGHT_MODE_6 Mode::Number::SURFACE
#endif

