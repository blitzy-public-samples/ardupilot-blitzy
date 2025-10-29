/**
 * @file AP_NavEKF3_feature.h
 * @brief Compile-time feature selection system for Extended Kalman Filter 3 (EKF3)
 * 
 * @details This header file implements a conditional compilation framework that enables or disables
 *          specific EKF3 fusion algorithms and sensor integrations based on:
 *          - Board memory constraints (flash size limits)
 *          - Vehicle type (Copter, Plane, Rover, Sub, etc.)
 *          - Build type (Replay, DAL Standalone, production)
 *          - Availability of external sensor libraries
 * 
 *          The feature flag system optimizes binary size for memory-constrained boards while
 *          enabling full functionality on boards with sufficient flash memory. Each feature can
 *          be individually overridden at build time by defining the flag before including this header.
 * 
 * Feature Selection Mechanism:
 * - Boards with >1024KB flash (HAL_PROGRAM_SIZE_LIMIT_KB > 1024) enable advanced features by default
 * - Boards with <=1024KB flash disable memory-intensive features to fit within space constraints
 * - Vehicle-specific builds enable features relevant to that platform (e.g., body odometry on Rover)
 * - Replay and DAL Standalone builds enable all features for comprehensive log analysis
 * - Features dependent on external sensors only compile when those sensor libraries are available
 * 
 * Memory Impact:
 * - Each enabled feature adds ~1-5KB of code depending on complexity
 * - Sensor fusion features also add per-instance state (~100-500 bytes per EKF core)
 * - On 1MB boards, selective feature compilation prevents flash overflow
 * 
 * @note Feature flags can be overridden in hwdef-bl.dat or via compiler defines (-DEK3_FEATURE_X=0)
 * @warning Disabling features removes sensor fusion capabilities - ensure vehicle configuration
 *          matches available EKF features to prevent navigation failures
 * 
 * @see AP_NavEKF3_core.h for feature usage in EKF algorithms
 * @see libraries/AP_HAL_ChibiOS/hwdef/README.md for board-specific feature configuration
 * 
 * Source: libraries/AP_NavEKF3/AP_NavEKF3_feature.h:1-49
 */

#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Beacon/AP_Beacon_config.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow_config.h>

/**
 * @brief Master feature flag enabling all EKF3 capabilities for analysis and replay builds
 * 
 * @details EK3_FEATURE_ALL evaluates to true for AP_DAL_Standalone and Replay build types,
 *          which require complete sensor fusion functionality for log analysis and algorithm
 *          validation. These builds are not constrained by board flash size limits.
 * 
 *          Build Types Enabling All Features:
 *          - APM_BUILD_AP_DAL_Standalone: Data Abstraction Layer standalone test builds
 *          - APM_BUILD_Replay: Log replay tool for offline algorithm analysis
 * 
 *          This flag is used as a condition in other feature definitions to unconditionally
 *          enable features in analysis contexts regardless of board or vehicle type.
 * 
 * Usage Pattern:
 * - Other features use: EK3_FEATURE_ALL || <specific_conditions>
 * - Ensures replay and analysis tools can process all logged sensor data
 * - Allows offline algorithm testing with all fusion modes enabled
 * 
 * @note This does NOT affect production vehicle builds - only development/analysis tools
 * @see Tools/Replay/ for log replay implementation
 */
#define EK3_FEATURE_ALL APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) || APM_BUILD_TYPE(APM_BUILD_Replay)

/**
 * @brief Enable body frame odometry fusion including wheel encoder integration
 * 
 * @details Body odometry fusion allows EKF3 to incorporate velocity measurements from wheel encoders,
 *          visual odometry systems, or other body-frame velocity sensors. This feature improves
 *          position estimation in GPS-denied environments, particularly for ground vehicles.
 * 
 *          Enablement Conditions (any true enables feature):
 *          - EK3_FEATURE_ALL: Analysis/replay builds
 *          - APM_BUILD_Rover: All Rover vehicle builds (primary use case)
 *          - HAL_PROGRAM_SIZE_LIMIT_KB > 1024: Boards with >1MB flash (2MB boards)
 * 
 *          Fusion Algorithm:
 *          - Integrates body-frame velocity measurements into EKF velocity states
 *          - Supports wheel encoder delta measurements with individual wheel velocities
 *          - Handles sensor timing jitter and dropout
 *          - Applies body-to-earth frame transformations using current attitude estimate
 * 
 *          Use Cases:
 *          - Ground vehicles with wheel encoders (primary: Rover)
 *          - Visual odometry systems providing body-frame velocity
 *          - Indoor navigation with optical flow and height sensors
 * 
 *          Memory Impact: ~3KB code, ~200 bytes state per EKF core
 * 
 * @note Can be disabled on Rover builds for space-constrained boards by defining EK3_FEATURE_BODY_ODOM=0
 * @warning Disabling this on Rover removes wheel encoder fusion capability, reducing GPS-denied navigation accuracy
 * @see AP_NavEKF3_core::FuseBodyVel() for fusion implementation
 */
#ifndef EK3_FEATURE_BODY_ODOM
#define EK3_FEATURE_BODY_ODOM EK3_FEATURE_ALL || APM_BUILD_TYPE(APM_BUILD_Rover) || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable external navigation system fusion (motion capture, UWB positioning, SLAM)
 * 
 * @details External navigation fusion allows EKF3 to integrate position and velocity measurements
 *          from external navigation systems such as:
 *          - Motion capture systems (Vicon, OptiTrack, Qualisys)
 *          - Ultra-wideband (UWB) positioning systems
 *          - Visual-Inertial Odometry (VIO) systems (T265, ZED)
 *          - External SLAM algorithms
 * 
 *          Enablement Conditions (any true enables feature):
 *          - EK3_FEATURE_ALL: Analysis/replay builds
 *          - HAL_PROGRAM_SIZE_LIMIT_KB > 1024: Boards with >1MB flash only (2MB boards)
 * 
 *          Fusion Capabilities:
 *          - Position measurements in NED (North-East-Down) frame
 *          - Velocity measurements in NED frame
 *          - Attitude measurements from external systems (optional)
 *          - Configurable measurement delays and noise parameters
 *          - Health monitoring and fallback to internal sensors
 * 
 *          Data Sources:
 *          - MAVLink VISION_POSITION_ESTIMATE messages
 *          - MAVLink ATT_POS_MOCAP messages
 *          - Custom external navigation drivers via AP_ExternalNav interface
 * 
 *          Memory Impact: ~4KB code, ~300 bytes state per EKF core
 * 
 * @note Disabled by default on 1MB boards due to flash constraints
 * @note Requires accurate synchronization between external system and autopilot clocks
 * @warning External nav fusion requires careful sensor frame alignment and timing configuration
 * @see AP_NavEKF3_core::FuseVelPosNED() for external position/velocity fusion
 * @see libraries/AP_ExternalAHRS/ for external navigation system integration
 */
#ifndef EK3_FEATURE_EXTERNAL_NAV
#define EK3_FEATURE_EXTERNAL_NAV EK3_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable aerodynamic drag-specific force fusion for improved wind estimation
 * 
 * @details Drag fusion improves wind velocity estimation by modeling the relationship between
 *          airspeed, groundspeed, and aerodynamic drag forces. This feature uses accelerometer
 *          measurements corrected for gravitational and rotational effects to estimate the
 *          component of acceleration due to aerodynamic drag.
 * 
 *          Enablement Conditions (any true enables feature):
 *          - EK3_FEATURE_ALL: Analysis/replay builds
 *          - HAL_PROGRAM_SIZE_LIMIT_KB > 1024: Boards with >1MB flash only (2MB boards)
 * 
 *          Algorithm:
 *          - Models drag force as proportional to square of airspeed: F_drag = 0.5 * ρ * Cd * A * V²
 *          - Estimates ballistic coefficient from accelerometer measurements
 *          - Fuses drag-specific force to correct wind velocity states
 *          - Improves convergence of wind estimates, particularly during turns
 * 
 *          Benefits:
 *          - Faster wind velocity estimation convergence
 *          - More accurate wind estimates in dynamic conditions
 *          - Improved position accuracy when using airspeed sensor
 *          - Better handling of wind gusts and turbulence
 * 
 *          Primary Use Cases:
 *          - Fixed-wing aircraft with airspeed sensors
 *          - Any vehicle operating in significant wind conditions
 * 
 *          Memory Impact: ~2.5KB code, ~150 bytes state per EKF core
 * 
 * @note Requires airspeed sensor for effective operation
 * @note Disabled by default on 1MB boards due to flash constraints
 * @warning Drag fusion accuracy depends on consistent vehicle aerodynamic properties
 * @see AP_NavEKF3_core::FuseDragForces() for fusion implementation
 */
#ifndef EK3_FEATURE_DRAG_FUSION
#define EK3_FEATURE_DRAG_FUSION EK3_FEATURE_ALL || HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable beacon distance measurement fusion for indoor positioning
 * 
 * @details Beacon fusion integrates distance measurements from fixed ground-based beacons to
 *          provide accurate position estimation in GPS-denied environments. The system uses
 *          trilateration from multiple beacon ranges to constrain position estimates.
 * 
 *          Enablement Condition:
 *          - AP_BEACON_ENABLED: Beacon library must be available and compiled in
 * 
 *          Supported Beacon Systems:
 *          - Marvelmind Indoor GPS
 *          - Pozyx UWB positioning
 *          - Custom beacon systems via AP_Beacon interface
 * 
 *          Fusion Algorithm:
 *          - Fuses range measurements to known beacon positions
 *          - Handles beacon position uncertainty and measurement noise
 *          - Requires minimum 3 beacons for 2D position, 4 for 3D position
 *          - Automatically disables when beacon signal quality degrades
 * 
 *          Configuration:
 *          - Beacon positions configured via parameters (BCN_POS_X/Y/Z)
 *          - Measurement noise adjusted via BCN_ALT and BCN_ORIENT_YAW parameters
 *          - Fusion enabled when beacon distance measurements available
 * 
 *          Use Cases:
 *          - Indoor flight and navigation
 *          - Warehouse operations
 *          - GPS-denied positioning with pre-surveyed beacon infrastructure
 * 
 *          Memory Impact: ~3KB code (only if AP_Beacon library compiled), ~250 bytes state per EKF core
 * 
 * @note Feature automatically disabled if AP_Beacon library not available (saves flash space)
 * @note Requires accurate beacon position survey and configuration
 * @warning Beacon fusion requires stable beacon signal reception - dropout causes position drift
 * @see AP_NavEKF3_core::FuseRngBcn() for beacon range fusion implementation
 * @see libraries/AP_Beacon/ for beacon driver implementation
 */
#ifndef EK3_FEATURE_BEACON_FUSION
#define EK3_FEATURE_BEACON_FUSION AP_BEACON_ENABLED
#endif

/**
 * @brief Enable position state reset capability for handling large position corrections
 * 
 * @details Position reset allows the EKF to handle discontinuous position updates by resetting
 *          the position state rather than attempting to converge through normal fusion. This
 *          is necessary when:
 *          - Switching between different position sources (GPS, beacon, external nav)
 *          - Large position corrections exceed innovation gate thresholds
 *          - Recovering from prolonged GPS outages
 * 
 *          Enablement Conditions (any true enables feature):
 *          - EK3_FEATURE_ALL: Analysis/replay builds
 *          - AP_AHRS_POSITION_RESET_ENABLED: Position reset enabled in AHRS configuration
 * 
 *          Reset Mechanism:
 *          - Instantaneous adjustment of position states
 *          - Preserves velocity, attitude, and other states
 *          - Maintains covariance consistency through proper reset handling
 *          - Triggers origin relocation if necessary
 * 
 *          Use Cases:
 *          - Initializing position from external navigation after GPS outage
 *          - Switching position reference during flight
 *          - Correcting accumulated drift after long GPS denial periods
 *          - Handling large GPS position steps (anti-spoofing, constellation changes)
 * 
 *          Safety Considerations:
 *          - Position resets can cause sudden position reference changes
 *          - May trigger fence breaches if reset crosses fence boundary
 *          - Logged as position reset events for post-flight analysis
 * 
 *          Memory Impact: ~1.5KB code, minimal state overhead
 * 
 * @note Position reset is NOT the same as EKF reset (full state reset)
 * @warning Position resets can cause discontinuities in position-dependent control modes
 * @see AP_NavEKF3_core::ResetPosition() for reset implementation
 * @see AP_NavEKF3_core::realignYawGPS() for yaw reset with position reset
 */
#ifndef EK3_FEATURE_POSITION_RESET
#define EK3_FEATURE_POSITION_RESET EK3_FEATURE_ALL || AP_AHRS_POSITION_RESET_ENABLED
#endif

/**
 * @brief Enable rangefinder (lidar/sonar) height measurement fusion
 * 
 * @details Rangefinder fusion integrates distance-to-ground measurements from lidar or sonar
 *          sensors to improve altitude estimation, particularly in terrain-following and
 *          precision landing scenarios. Provides direct height-above-ground measurements
 *          independent of barometric pressure changes.
 * 
 *          Enablement Condition:
 *          - AP_RANGEFINDER_ENABLED: Rangefinder library must be available and compiled in
 * 
 *          Supported Rangefinder Types:
 *          - Lidar: Lightware, Benewake, Garmin, TeraRanger, etc.
 *          - Sonar: MaxBotix, HC-SR04
 *          - Radar: Ainstein US-D1
 * 
 *          Fusion Behavior:
 *          - Primary altitude source when close to ground (configurable threshold)
 *          - Blends with barometric altitude based on terrain slope and signal quality
 *          - Handles terrain following with tilt compensation
 *          - Rejects outliers using innovation consistency checks
 * 
 *          Terrain Handling:
 *          - Compensates for vehicle tilt when measuring sloped terrain
 *          - Validates measurements against terrain database (if available)
 *          - Rejects measurements during aggressive maneuvers
 * 
 *          Use Cases:
 *          - Precision landing and takeoff
 *          - Low-altitude terrain following
 *          - Indoor flight altitude hold
 *          - Ground effect altitude stabilization
 * 
 *          Configuration:
 *          - RNGFND parameters control sensor setup and thresholds
 *          - EK3_RNG_USE_HGT controls when rangefinder used for height estimation
 *          - EK3_RNG_USE_SPD controls speed threshold for rangefinder fusion
 * 
 *          Memory Impact: ~2KB code (only if AP_RangeFinder library compiled), ~100 bytes state per EKF core
 * 
 * @note Feature automatically disabled if AP_RangeFinder library not available
 * @note Rangefinder accuracy degrades over non-flat terrain or low-reflectivity surfaces
 * @warning Rapid altitude changes during rangefinder transitions can affect altitude hold performance
 * @see AP_NavEKF3_core::FuseRngBcnStatic() for static rangefinder fusion
 * @see libraries/AP_RangeFinder/ for rangefinder driver implementations
 */
#ifndef EK3_FEATURE_RANGEFINDER_MEASUREMENTS
#define EK3_FEATURE_RANGEFINDER_MEASUREMENTS AP_RANGEFINDER_ENABLED
#endif

/**
 * @brief Enable optical flow sensor fusion for velocity estimation without GPS
 * 
 * @details Optical flow fusion integrates image-based velocity measurements from downward-facing
 *          cameras to estimate vehicle velocity in GPS-denied environments. Combined with a
 *          rangefinder for height, optical flow enables stable position hold without GPS.
 * 
 *          Enablement Conditions (all must be true):
 *          - HAL_NAVEKF3_AVAILABLE: NavEKF3 library available on platform
 *          - AP_OPTICALFLOW_ENABLED: Optical flow library must be available and compiled in
 * 
 *          Supported Optical Flow Sensors:
 *          - PX4Flow: Integrated camera + sonar
 *          - Cheerson CxOf: Low-cost optical flow sensor
 *          - MAVLink optical flow (custom sensors)
 *          - CXOF, UPFLOW: Various commercial sensors
 * 
 *          Fusion Algorithm:
 *          - Converts flow rate (rad/s) to body-frame velocity using height-above-ground
 *          - Requires rangefinder or known altitude for velocity calculation: V = flow_rate * height
 *          - Compensates for vehicle rotation (gyro-corrected flow)
 *          - Rejects flow during rapid altitude changes or high tilt angles
 * 
 *          Quality Requirements:
 *          - Adequate surface texture (rejects flow over blank/repetitive surfaces)
 *          - Sufficient lighting (some sensors include illumination)
 *          - Valid height measurement (typically from rangefinder)
 *          - Vehicle tilt within acceptable range (<30° typically)
 * 
 *          Use Cases:
 *          - Indoor position hold without GPS
 *          - GPS-denied navigation with known altitude
 *          - Velocity estimation backup during GPS dropouts
 *          - Precision hover over landing targets
 * 
 *          Configuration:
 *          - FLOW_TYPE: Optical flow sensor type selection
 *          - EK3_FLOW_USE: Enable/disable optical flow fusion
 *          - EK3_FLOW_DELAY: Flow measurement delay compensation
 *          - EK3_FLOW_I_GATE: Innovation gate for outlier rejection
 * 
 *          Memory Impact: ~4KB code (only if AP_OpticalFlow library compiled), ~200 bytes state per EKF core
 * 
 * @note Requires both optical flow sensor AND rangefinder for effective velocity estimation
 * @note Flow quality degrades over featureless terrain, water, or in low light conditions
 * @warning Optical flow alone cannot provide absolute position - only velocity and relative position
 * @warning Flow fusion disabled when height-above-ground unknown or quality metrics poor
 * @see AP_NavEKF3_core::FuseOptFlow() for optical flow fusion implementation
 * @see libraries/AP_OpticalFlow/ for optical flow driver implementations
 */
#ifndef EK3_FEATURE_OPTFLOW_FUSION
#define EK3_FEATURE_OPTFLOW_FUSION HAL_NAVEKF3_AVAILABLE && AP_OPTICALFLOW_ENABLED
#endif
