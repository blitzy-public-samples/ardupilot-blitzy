/**
 * @file defines.h
 * @brief Core constants, enumerations, and macros for ArduPlane fixed-wing autopilot
 * 
 * @details This file defines fundamental constants, enumerations, and configuration values
 *          used throughout the ArduPlane codebase. It includes:
 *          - Control system constants (servo limits, speed thresholds)
 *          - Failsafe state definitions and action enumerations
 *          - Flight mode behavior flags and options
 *          - Logging configuration bitmasks
 *          - Guidance and navigation state definitions
 *          - Safety system enumerations (fence, crash detection)
 *          
 *          These definitions establish the core behavioral parameters and state machines
 *          for fixed-wing vehicle control, safety systems, and operational modes.
 *          
 * @note This file contains internal defines that should not be modified without
 *       understanding their system-wide implications. Many constants are tuned for
 *       specific flight characteristics and safety margins.
 *       
 * @warning Modifying values in this file can significantly affect flight behavior,
 *          safety margins, and system stability. Changes should be thoroughly tested
 *          in SITL simulation before deployment to actual hardware.
 * 
 * @see plane.h for main Plane class
 * @see mode.h for flight mode implementations
 * @see AP_Arming_Plane.cpp for arming check usage of these constants
 * 
 * Source: ArduPlane/defines.h
 */

#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

/**
 * @brief Maximum servo travel representation in centidegrees
 * 
 * @details This value represents 45 degrees (4500 centidegrees) and serves as an
 *          arbitrary representation of maximum servo travel for control surface scaling.
 *          Used throughout the control system for normalizing servo commands.
 * 
 * @note Value is in centidegrees (1 degree = 100 centidegrees)
 */
#define SERVO_MAX 4500.0  // This value represents 45 degrees and is just an
                        // arbitrary representation of servo max travel.

/**
 * @brief Minimum allowable airspeed for arming and speed scaling
 * 
 * @details Defines the minimum airspeed threshold (5 m/s) used for:
 *          - Pre-arming airspeed sensor validation checks
 *          - Speed-based control scaling to prevent division by zero
 *          - Minimum dynamic pressure calculations
 * 
 * @note Value in meters per second (m/s)
 * @warning Setting this too low may allow arming with faulty airspeed sensors
 */
#define MIN_AIRSPEED_MIN 5 // m/s, used for arming check and speed scaling

/**
 * @brief Timeout for takeoff rudder neutral warning messages
 * 
 * @details Duration in milliseconds (3000ms = 3 seconds) that the GCS warning about
 *          rudder not returning to neutral during takeoff sequence will repeat.
 *          Prevents warning message spam while still alerting pilot to potential issue.
 * 
 * @note Value in milliseconds (ms)
 */
#define TAKEOFF_RUDDER_WARNING_TIMEOUT 3000 //ms that GCS warning about not returning arming rudder to neutral repeats

/**
 * @brief Minimum GPS ground speed for course capture and heading decisions
 * 
 * @details Minimum ground speed threshold (5 m/s) used to determine when:
 *          - Initial direction/heading is captured for navigation
 *          - Heading lock is engaged in CRUISE mode
 *          - Steering state hold_course_cd is established
 *          
 *          Below this speed, GPS heading is unreliable due to low signal-to-noise ratio.
 * 
 * @note Value in meters per second (m/s)
 * @warning Below this speed, GPS course over ground becomes unreliable for navigation
 */
#define GPS_GND_CRS_MIN_SPD 5 // m/s, used to set when intial_direction.heading is captured,deciding to heading lock in cruise mode, or steer_state.hold_course_cd

/**
 * @brief Failsafe state enumeration for radio control link monitoring
 * 
 * @details Defines the current failsafe state based on duration of RC signal loss
 *          or other failsafe trigger conditions. State progression:
 *          NONE → SHORT → LONG as signal loss duration increases.
 *          
 *          The failsafe state machine triggers different actions at each level,
 *          providing escalating responses to maintain vehicle safety.
 * 
 * @note Used throughout failsafe.cpp and mode switching logic
 * @warning Critical safety system - modifications affect emergency response behavior
 * 
 * @see Plane::failsafe_check() for state machine implementation
 * @see failsafe_action_short and failsafe_action_long for action definitions
 */
enum failsafe_state {
    FAILSAFE_NONE=0,    ///< No failsafe active, normal operation with valid RC link
    FAILSAFE_SHORT=1,   ///< Short-term failsafe triggered (typically <5 seconds of signal loss)
    FAILSAFE_LONG=2,    ///< Long-term failsafe triggered (extended signal loss, action based on FS_LONG_ACTN)
    FAILSAFE_GCS=3      ///< GCS (Ground Control Station) failsafe triggered via MAVLink
};


/**
 * @brief GCS (Ground Control Station) failsafe trigger types
 * 
 * @details Defines when GCS failsafe is triggered based on MAVLink heartbeat loss
 *          or telemetry radio signal strength. Used by FS_GCS_ENABL parameter to
 *          configure GCS failsafe behavior.
 *          
 *          GCS failsafe provides protection against loss of ground control station
 *          communication, with options for different trigger conditions and modes.
 * 
 * @note Configured via FS_GCS_ENABL parameter
 * @warning GCS_FAILSAFE_HB_AUTO only triggers in AUTO mode, other modes unprotected
 * 
 * @see GCS_MAVLINK::failsafe_check() for heartbeat monitoring
 * @see Parameters.h for FS_GCS_ENABL parameter definition
 */
enum gcs_failsafe {
    GCS_FAILSAFE_OFF        = 0, ///< GCS failsafe disabled - no action on heartbeat loss
    GCS_FAILSAFE_HEARTBEAT  = 1, ///< Trigger failsafe if MAVLink heartbeat not received within timeout
    GCS_FAILSAFE_HB_RSSI    = 2, ///< Trigger failsafe on heartbeat loss OR if RADIO.remrssi drops to 0
    GCS_FAILSAFE_HB_AUTO    = 3  ///< Trigger failsafe on heartbeat loss only while in AUTO mode
};

/**
 * @brief Short-term failsafe action options
 * 
 * @details Defines the vehicle response when short-term failsafe is triggered
 *          (typically after 1-5 seconds of RC signal loss). Actions are designed
 *          to maintain vehicle control while awaiting signal recovery or long-term
 *          failsafe escalation.
 *          
 *          Configured via FS_SHORT_ACTN parameter.
 * 
 * @note Short failsafe is typically brief and may recover without escalation
 * @warning FS_ACTION_SHORT_DISABLED removes short-term failsafe protection
 * 
 * @see Plane::failsafe_short_on_event() for action implementation
 * @see Parameters.h for FS_SHORT_ACTN parameter definition
 */
enum failsafe_action_short {
    FS_ACTION_SHORT_BESTGUESS = 0,  ///< Intelligent mode selection: CIRCLE or continue if in AUTO/GUIDED/LOITER
    FS_ACTION_SHORT_CIRCLE = 1,     ///< Enter CIRCLE mode to loiter at current location
    FS_ACTION_SHORT_FBWA = 2,       ///< Enter FLY BY WIRE_A mode (stabilized with altitude hold)
    FS_ACTION_SHORT_DISABLED = 3,   ///< Short-term failsafe disabled, wait for long failsafe
    FS_ACTION_SHORT_FBWB = 4,       ///< Enter FLY BY WIRE_B mode (stabilized with altitude and speed hold)
};

/**
 * @brief Long-term failsafe action options
 * 
 * @details Defines the vehicle response when long-term failsafe is triggered
 *          (after extended RC signal loss, typically >5 seconds). Actions are designed
 *          for safe vehicle recovery or controlled termination when RC link is
 *          presumed lost for extended duration.
 *          
 *          Configured via FS_LONG_ACTN parameter. Long failsafe represents escalation
 *          from short failsafe when signal recovery doesn't occur.
 * 
 * @note Long failsafe assumes extended or permanent RC link loss
 * @warning FS_ACTION_LONG_PARACHUTE deploys parachute - irreversible action
 * 
 * @see Plane::failsafe_long_on_event() for action implementation
 * @see Parameters.h for FS_LONG_ACTN parameter definition
 */
enum failsafe_action_long {
    FS_ACTION_LONG_CONTINUE = 0,    ///< Continue current mode/mission (useful in AUTO missions)
    FS_ACTION_LONG_RTL = 1,         ///< Return To Launch - navigate back to home and loiter or land
    FS_ACTION_LONG_GLIDE = 2,       ///< Controlled glide descent with motors off (for gliders)
    FS_ACTION_LONG_PARACHUTE = 3,   ///< Deploy parachute if installed (emergency termination)
    FS_ACTION_LONG_AUTO = 4,        ///< Enter AUTO mode and continue mission
    FS_ACTION_LONG_AUTOLAND = 5,    ///< Initiate automatic landing sequence at current location
};

/**
 * @brief Stick mixing mode enumeration for autonomous flight modes
 * 
 * @details Defines how pilot stick inputs are mixed with autonomous flight control
 *          commands during AUTO, GUIDED, and other autonomous modes. Stick mixing
 *          allows pilot override while maintaining some autonomous navigation.
 *          
 *          Different mixing modes provide varying levels of pilot authority over
 *          autonomous flight, from full control to no mixing.
 * 
 * @note Configured via STICK_MIXING parameter
 * @warning VTOL_YAW is specifically for quadplane yaw control during VTOL flight
 * 
 * @see Plane::stick_mix_channel() for mixing implementation
 * @see mode.cpp for mode-specific stick mixing behavior
 */
enum class StickMixing {
    NONE     = 0,           ///< No stick mixing - pilot inputs ignored in autonomous modes
    FBW      = 1,           ///< FBW-style mixing - pilot can override autonomous commands
    DIRECT_REMOVED = 2,     ///< Legacy direct mixing mode (removed/deprecated)
    VTOL_YAW = 3,           ///< VTOL yaw mixing for quadplane hover modes
    FBW_NO_PITCH = 4,       ///< FBW mixing but without pitch axis (roll/yaw only)
};

/**
 * @brief RTL (Return To Launch) automatic landing behavior options
 * 
 * @details Defines the landing behavior when RTL mode is activated. Controls whether
 *          the vehicle automatically proceeds to landing after reaching home location,
 *          and the specific landing pattern used.
 *          
 *          Configured via RTL_AUTOLAND parameter. Affects behavior at end of RTL
 *          sequence, providing options from simple loiter to fully automated landing.
 * 
 * @note RTL first navigates to home position, then applies autoland behavior
 * @warning Autoland requires properly configured DO_LAND_START mission items
 * 
 * @see mode_rtl.cpp for RTL mode implementation
 * @see Parameters.h for RTL_AUTOLAND parameter definition
 */
enum class RtlAutoland {
    RTL_DISABLE = 0,                ///< RTL without autoland - loiter at home location indefinitely
    RTL_THEN_DO_LAND_START = 1,     ///< After reaching home, execute DO_LAND_START mission sequence
    RTL_IMMEDIATE_DO_LAND_START = 2,///< Immediately execute DO_LAND_START, skip normal RTL navigation
    NO_RTL_GO_AROUND = 3,           ///< Like RTL_DISABLE but initiates go-around pattern if already landing
    DO_RETURN_PATH_START = 4,       ///< Execute DO_RETURN_PATH_START mission item sequence
};

/**
 * @brief PID tuning data broadcast bitmask flags
 * 
 * @details Bitmask values for selecting which PID controllers broadcast their tuning
 *          data via MAVLink telemetry. Used for real-time PID tuning and debugging
 *          during flight tests.
 *          
 *          Multiple PIDs can be broadcast simultaneously by OR-ing bitmask values.
 *          Reduces telemetry bandwidth by allowing selective PID data streaming.
 * 
 * @note Configured via TUNE_SELECTOR parameter for telemetry streaming
 * @warning Broadcasting multiple PIDs simultaneously increases telemetry bandwidth
 * 
 * @see GCS_MAVLink::send_pid_tuning() for broadcast implementation
 * @see PID_TUNING MAVLink message definition
 */
enum tuning_pid_bits {
    TUNING_BITS_ROLL  = (1 <<  0),  ///< Broadcast roll axis PID tuning data
    TUNING_BITS_PITCH = (1 <<  1),  ///< Broadcast pitch axis PID tuning data
    TUNING_BITS_YAW   = (1 <<  2),  ///< Broadcast yaw axis PID tuning data
    TUNING_BITS_STEER = (1 <<  3),  ///< Broadcast steering PID tuning data (ground steering)
    TUNING_BITS_LAND  = (1 <<  4),  ///< Broadcast landing PID tuning data
    TUNING_BITS_ACCZ  = (1 <<  5),  ///< Broadcast vertical acceleration PID tuning data
    TUNING_BITS_END // dummy just used for static checking
};

/**
 * @brief Compile-time check that tuning bitmask fits in MAVLink message field
 * 
 * @details Ensures TUNING_BITS_END doesn't exceed the 24-bit limit imposed by
 *          MAVLink PID_TUNING message format, preventing buffer overflow or
 *          protocol violations.
 */
static_assert(TUNING_BITS_END <= (1 << 24) + 1, "Tuning bit mask is too large to be set by MAVLink");

/**
 * @brief ArduPlane-specific logging message type identifiers
 * 
 * @details Enumeration of custom log message types specific to ArduPlane fixed-wing
 *          vehicle operation. These supplement the common AP_Logger message types
 *          with plane-specific control, navigation, and quadplane data.
 *          
 *          Only 32 vehicle-specific message types are available (limitation of
 *          AP_Logger architecture). Each message type corresponds to a specific
 *          data structure logged to onboard storage.
 * 
 * @note Message structures defined in Log.cpp
 * @warning Limited to 32 vehicle-specific messages maximum
 * 
 * @see Log.cpp for message structure definitions
 * @see AP_Logger for logging system implementation
 */
enum log_messages {
    LOG_CTUN_MSG,       ///< Control tuning message (elevator, aileron, rudder demands)
    LOG_NTUN_MSG,       ///< Navigation tuning message (L1 controller, navigation targets)
    LOG_STATUS_MSG,     ///< Vehicle status message (mode, arming state, flight stage)
    LOG_QTUN_MSG,       ///< Quadplane tuning message (VTOL motor outputs, hover control)
    LOG_PIQR_MSG,       ///< Quadplane roll PID message
    LOG_PIQP_MSG,       ///< Quadplane pitch PID message
    LOG_PIQY_MSG,       ///< Quadplane yaw PID message
    LOG_PIQA_MSG,       ///< Quadplane throttle/altitude PID message
    LOG_PIDG_MSG,       ///< Ground steering PID message
    LOG_AETR_MSG,       ///< Aileron/Elevator/Throttle/Rudder pilot input message
    LOG_OFG_MSG,        ///< Optical flow and ground distance message
    LOG_TSIT_MSG,       ///< Transition state message for quadplane VTOL transitions
    LOG_TILT_MSG,       ///< Tiltrotor servo position and transition message
};

/**
 * @brief Logging enable bitmask - Attitude logging at fast rate (25Hz for planes)
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_ATTITUDE_FAST          (1<<0)

/**
 * @brief Logging enable bitmask - Attitude logging at medium rate (10Hz)
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_ATTITUDE_MED           (1<<1)

/**
 * @brief Logging enable bitmask - GPS position, velocity, and status
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_GPS                    (1<<2)

/**
 * @brief Logging enable bitmask - Performance monitoring (CPU load, timing, memory)
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_PM                     (1<<3)

/**
 * @brief Logging enable bitmask - Control tuning (CTUN message)
 * @note Used in LOG_BITMASK parameter
 * @see LOG_CTUN_MSG for message structure
 */
#define MASK_LOG_CTUN                   (1<<4)

/**
 * @brief Logging enable bitmask - Navigation tuning (NTUN message)
 * @note Used in LOG_BITMASK parameter
 * @see LOG_NTUN_MSG for message structure
 */
#define MASK_LOG_NTUN                   (1<<5)

//#define MASK_LOG_MODE                 (1<<6) // no longer used

/**
 * @brief Logging enable bitmask - IMU sensor data (gyro, accelerometer)
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_IMU                    (1<<7)

/**
 * @brief Logging enable bitmask - Mission command execution
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_CMD                    (1<<8)

/**
 * @brief Logging enable bitmask - Battery current and voltage
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_CURRENT                (1<<9)

/**
 * @brief Logging enable bitmask - Compass/magnetometer data
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_COMPASS                (1<<10)

/**
 * @brief Logging enable bitmask - TECS (Total Energy Control System) tuning
 * @note Used in LOG_BITMASK parameter
 * @see AP_TECS for energy management algorithm
 */
#define MASK_LOG_TECS                   (1<<11)

/**
 * @brief Logging enable bitmask - Camera trigger events
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_CAMERA                 (1<<12)

/**
 * @brief Logging enable bitmask - RC input channels (pilot stick positions)
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_RC                     (1<<13)

/**
 * @brief Logging enable bitmask - Rangefinder/sonar distance measurements
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_SONAR                  (1<<14)

// #define MASK_LOG_ARM_DISARM             (1<<15)

/**
 * @brief Logging enable bitmask - Raw IMU sensor data (unfiltered)
 * @note Used in LOG_BITMASK parameter, high data rate
 */
#define MASK_LOG_IMU_RAW                (1UL<<19)

/**
 * @brief Logging enable bitmask - Attitude at full rate (400Hz typical)
 * @note Used in LOG_BITMASK parameter
 * @warning High data rate, can fill SD card quickly
 */
#define MASK_LOG_ATTITUDE_FULLRATE      (1U<<20)

/**
 * @brief Logging enable bitmask - Video stabilization/gimbal data
 * @note Used in LOG_BITMASK parameter
 */
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<21)

/**
 * @brief Logging enable bitmask - Notch filter data at full rate
 * @note Used in LOG_BITMASK parameter for harmonic notch debugging
 * @warning High data rate
 */
#define MASK_LOG_NOTCH_FULLRATE         (1UL<<22)

/**
 * @brief Crash detection action bitmask flags
 * 
 * @details Defines actions taken when crash is detected during flight or ground
 *          operations. Actions can be combined using bitwise OR to enable multiple
 *          responses simultaneously.
 *          
 *          Crash detection monitors for abnormal attitudes, impact accelerations,
 *          or loss of control during flight or ground operations.
 * 
 * @note Configured via CRASH_DETECT parameter
 * @warning DISARM action immediately cuts motor power - ensure appropriate for vehicle type
 * 
 * @see Plane::crash_detection_update() for detection algorithm
 */
enum {
    CRASH_DETECT_ACTION_BITMASK_DISABLED = 0,       ///< Crash detection disabled, no actions taken
    CRASH_DETECT_ACTION_BITMASK_DISARM = (1<<0),    ///< Automatically disarm vehicle when crash detected
    // note: next enum will be (1<<1), then (1<<2), then (1<<3)
};

/**
 * @brief Reverse thrust enable bitmask for flight modes
 * 
 * @details Bitmask flags defining which flight modes are allowed to use reverse thrust
 *          (negative throttle) for speed control. Reverse thrust can be used for rapid
 *          deceleration during landing approaches or speed reduction in various modes.
 *          
 *          Multiple modes can be enabled simultaneously by OR-ing bitmask values.
 *          Requires ESC/motor configuration supporting reverse thrust or airbrakes.
 * 
 * @note Configured via USE_REV_THRUST parameter
 * @warning Requires ESC capable of reverse thrust and proper configuration
 * 
 * @see SRV_Channel::set_output_scaled() for reverse thrust application
 * @see Parameters.h for USE_REV_THRUST parameter definition
 */
enum class UseReverseThrust {
    AUTO_ALWAYS          = (1<<0),  ///< Enable reverse thrust for all AUTO mode segments
    AUTO_LAND_APPROACH   = (1<<1),  ///< Enable reverse thrust during AUTO landing approach phase
    AUTO_LOITER_TO_ALT   = (1<<2),  ///< Enable reverse thrust in AUTO LOITER_TO_ALT waypoint command
    AUTO_LOITER_ALL      = (1<<3),  ///< Enable reverse thrust in all AUTO loiter operations
    AUTO_WAYPOINT        = (1<<4),  ///< Enable reverse thrust during AUTO waypoint navigation
    LOITER               = (1<<5),  ///< Enable reverse thrust in LOITER mode
    RTL                  = (1<<6),  ///< Enable reverse thrust in RTL (Return To Launch) mode
    CIRCLE               = (1<<7),  ///< Enable reverse thrust in CIRCLE mode
    CRUISE               = (1<<8),  ///< Enable reverse thrust in CRUISE mode
    FBWB                 = (1<<9),  ///< Enable reverse thrust in FLY BY WIRE_B mode
    GUIDED               = (1<<10), ///< Enable reverse thrust in GUIDED mode
    AUTO_LANDING_PATTERN = (1<<11), ///< Enable reverse thrust during AUTO landing pattern execution
    FBWA                 = (1<<12), ///< Enable reverse thrust in FLY BY WIRE_A mode
    ACRO                 = (1<<13), ///< Enable reverse thrust in ACRO mode
    STABILIZE            = (1<<14), ///< Enable reverse thrust in STABILIZE mode
    THERMAL              = (1<<15), ///< Enable reverse thrust in THERMAL soaring mode
};

/**
 * @brief Flight behavior option flags bitmask
 * 
 * @details Bitmask flags for enabling various optional flight behaviors and control
 *          modifications. These options allow fine-tuning of vehicle behavior for
 *          specific airframe configurations, operational requirements, or pilot
 *          preferences.
 *          
 *          Multiple options can be enabled simultaneously by OR-ing bitmask values.
 *          Configured via FLIGHT_OPTIONS parameter.
 * 
 * @note Configured via FLIGHT_OPTIONS parameter
 * @warning Some options significantly alter flight behavior - test in SITL first
 * 
 * @see Parameters.h for FLIGHT_OPTIONS parameter definition
 * @see Plane class methods for option implementation
 */
enum FlightOptions {
    DIRECT_RUDDER_ONLY   = (1 << 0),    ///< Use rudder stick for direct rudder control only (no yaw damper)
    CRUISE_TRIM_THROTTLE = (1 << 1),    ///< Trim throttle to maintain altitude in CRUISE mode
    DISABLE_TOFF_ATTITUDE_CHK = (1 << 2), ///< Disable attitude checks during takeoff (for catapult/hand launch)
    CRUISE_TRIM_AIRSPEED = (1 << 3),    ///< Trim throttle to maintain airspeed in CRUISE mode
    CLIMB_BEFORE_TURN = (1 << 4),       ///< Climb to waypoint altitude before initiating turn
    ACRO_YAW_DAMPER = (1 << 5),         ///< Enable yaw damper in ACRO mode for coordinated turns
    SURPRESS_TKOFF_SCALING = (1<<6),    ///< Suppress control surface scaling during takeoff
    ENABLE_DEFAULT_AIRSPEED = (1<<7),   ///< Use default airspeed if airspeed sensor fails
    GCS_REMOVE_TRIM_PITCH = (1 << 8),   ///< Remove pitch trim from GCS attitude display
    OSD_REMOVE_TRIM_PITCH = (1 << 9),   ///< Remove pitch trim from OSD attitude display
    CENTER_THROTTLE_TRIM = (1<<10),     ///< Center throttle stick position is trim (not zero)
    DISABLE_GROUND_PID_SUPPRESSION = (1<<11), ///< Don't suppress I-term accumulation on ground
    ENABLE_LOITER_ALT_CONTROL = (1<<12),///< Enable altitude control in LOITER mode
    INDICATE_WAITING_FOR_RUDDER_NEUTRAL = (1<<13), ///< Flash LEDs when waiting for rudder neutral before arm
    IMMEDIATE_CLIMB_IN_AUTO = (1<<14),  ///< Begin climb immediately in AUTO mode (no level flight first)
    FLAP_ACTUAL_SPEED = (1<<15),        ///< Use actual speed instead of target for automatic flap deployment
};

/**
 * @brief Crow flap configuration option flags
 * 
 * @details Bitmask flags for configuring crow flap (spoiler-flap) behavior on flying
 *          wings and gliders. Crow flaps deploy elevons differentially upward to
 *          increase drag for landing approaches while maintaining roll control.
 *          
 *          Different configurations suit different airframe types and control surface
 *          arrangements.
 * 
 * @note Configured via CROW_FLAP_OPT parameter
 * @warning FLYINGWING option required for flying wing airframes
 * 
 * @see RC_Channel::set_output_scaled() for crow flap mixing
 * @see Parameters.h for CROW_FLAP_OPT parameter definition
 */
enum CrowFlapOptions {
    FLYINGWING       = (1 << 0),    ///< Enable crow flaps on flying wing airframe (elevon mixing)
    FULLSPAN         = (1 << 1),    ///< Use full span elevons for crow (not just inboard)
    PROGRESSIVE_CROW = (1 << 2),    ///< Progressive crow deployment proportional to flap input
}; 


/**
 * @brief GUIDED mode heading control type enumeration
 * 
 * @details Defines the heading control behavior when in GUIDED mode. Determines
 *          whether the vehicle maintains a specific heading, follows course over
 *          ground, or has no heading constraint during GUIDED navigation.
 *          
 *          Used when GCS or companion computer sends GUIDED mode commands.
 * 
 * @note Set via MAVLink GUIDED mode commands
 * @see mode_guided.cpp for GUIDED mode implementation
 * @see GCS_MAVLink for GUIDED command processing
 */
enum guided_heading_type_t {
    GUIDED_HEADING_NONE = 0,    ///< No heading tracking - free to yaw during GUIDED navigation
    GUIDED_HEADING_COG,         ///< Maintain course over ground (track ground path)
    GUIDED_HEADING_HEADING,     ///< Maintain specific compass heading
};


/**
 * @brief Airmode configuration enumeration
 * 
 * @details Defines airmode behavior - when enabled, airmode maintains full attitude
 *          authority even at zero throttle by allowing motor/throttle output to go
 *          negative (for thrust vectoring or reverse thrust configurations).
 *          
 *          Primarily used in quadplane VTOL modes and 3D aerobatic aircraft.
 *          Allows aggressive maneuvers and attitude recovery at low throttle.
 * 
 * @note Configured via Q_AIRMODE parameter (quadplane) or AIRMODE parameter
 * @warning Requires motors/ESCs capable of bidirectional control or thrust vectoring
 * 
 * @see QuadPlane::airmode_active() for airmode activation logic
 * @see Parameters.h for Q_AIRMODE parameter definition
 */
enum class AirMode {
    OFF,                    ///< Airmode disabled - traditional throttle/attitude mixing
    ON,                     ///< Airmode always enabled when armed
    ASSISTED_FLIGHT_ONLY,   ///< Airmode only in assisted flight modes (not manual)
};

/**
 * @brief Geofence automatic enable/disable options
 * 
 * @details Defines when geofence boundaries are automatically enabled or disabled
 *          based on vehicle state (takeoff, landing, arming). Provides options for
 *          automatic fence management to prevent ground-level fence breaches or
 *          ensure protection during flight.
 * 
 * @note Configured via FENCE_AUTOENABLE parameter
 * @warning Auto-disable options may leave vehicle unprotected during certain phases
 * 
 * @see AC_Fence for geofence implementation
 * @see Parameters.h for FENCE_AUTOENABLE parameter definition
 */
enum class FenceAutoEnable : uint8_t {
    OFF=0,                      ///< Fence auto-enable disabled - manual control only
    Auto=1,                     ///< Auto-enable on takeoff, auto-disable on landing
    AutoDisableFloorOnly=2,     ///< Auto-disable altitude floor only on landing (keep boundaries)
    WhenArmed=3                 ///< Enable fence when armed, disable when disarmed
};

/**
 * @brief Rangefinder usage bitmask options
 * 
 * @details Bitmask flags defining when rangefinder (lidar/sonar) data is used for
 *          altitude control and terrain following. Allows selective rangefinder use
 *          during different flight phases to improve terrain following accuracy while
 *          avoiding false readings during inappropriate phases.
 *          
 *          Multiple options can be enabled simultaneously by OR-ing bitmask values.
 * 
 * @note Configured via RNGFND_LANDING parameter
 * @warning Rangefinder requires clear line-of-sight to ground and appropriate terrain
 * 
 * @see AP_RangeFinder for rangefinder implementation
 * @see Parameters.h for RNGFND_LANDING parameter definition
 */
enum class RangeFinderUse : uint8_t {
    NONE    = 0U,           ///< Rangefinder disabled for altitude control
    ALL     = (1U<<0),      ///< Use rangefinder for all altitude control (terrain following)
    TAKEOFF_LANDING = (1U<<1), ///< Use rangefinder during takeoff and landing phases only
    ASSIST  = (1U<<2),      ///< Use rangefinder to assist barometric altitude (blending)
    CLIMB   = (1U<<3),      ///< Use rangefinder during climb phases
};

