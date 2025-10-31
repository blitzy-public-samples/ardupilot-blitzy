/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/*
 *   The AP_Mission library:
 *   - responsible for managing a list of commands made up of "nav", "do" and "conditional" commands
 *   - reads and writes the mission commands to storage.
 *   - provides easy access to current, previous and upcoming waypoints
 *   - calls main program's command execution and verify functions.
 *   - accounts for the DO_JUMP command
 *
 */
#pragma once

#include "AP_Mission_config.h"

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Common/float16.h>

// definitions
#define AP_MISSION_EEPROM_VERSION           0x65AE  // version number stored in first four bytes of eeprom.  increment this by one when eeprom format is changed
#define AP_MISSION_EEPROM_COMMAND_SIZE      15      // size in bytes of all mission commands

#ifndef AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS
#if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 100     // allow up to 100 do-jump commands
#else
#define AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS 15      // allow up to 15 do-jump commands
#endif
#endif

#define AP_MISSION_JUMP_REPEAT_FOREVER      -1      // when do-jump command's repeat count is -1 this means endless repeat

#define AP_MISSION_CMD_ID_NONE              0       // mavlink cmd id of zero means invalid or missing command
#define AP_MISSION_CMD_INDEX_NONE           65535   // command index of 65535 means invalid or missing command
#define AP_MISSION_JUMP_TIMES_MAX           32767   // maximum number of times a jump can be executed.  Used when jump tracking fails (i.e. when too many jumps in mission)

#define AP_MISSION_FIRST_REAL_COMMAND       1       // command #0 reserved to hold home position

#define AP_MISSION_RESTART_DEFAULT          0       // resume the mission from the last command run by default

#define AP_MISSION_OPTIONS_DEFAULT          0       // Do not clear the mission when rebooting

#define AP_MISSION_MAX_WP_HISTORY           7       // The maximum number of previous wp commands that will be stored from the active missions history
#define LAST_WP_PASSED (AP_MISSION_MAX_WP_HISTORY-2)

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_MISSION_SDCARD_FILENAME "APM/mission.stg"
#else
#define AP_MISSION_SDCARD_FILENAME "mission.stg"
#endif

union PackedContent;

/**
 * @class AP_Mission
 * @brief Mission management singleton for ArduPilot autopilot system
 * 
 * @details The AP_Mission library manages the complete mission command list for autonomous
 *          vehicle operations. It provides:
 *          - Storage and retrieval of mission commands to/from EEPROM or SD card
 *          - Sequential execution of navigation, do, and conditional commands
 *          - Support for complex mission logic including DO_JUMP, conditional delays, and tags
 *          - Integration with vehicle-specific command execution via callback functions
 *          - MAVLink mission protocol implementation for ground station communication
 * 
 * Architecture:
 * - Mission commands stored at index 0 (home) through num_commands-1
 * - Commands divided into: NAV (waypoints, loiter), DO (immediate actions), CONDITIONAL (delays/conditions)
 * - Two command queues: _nav_cmd (current navigation target) and _do_cmd (current do command)
 * - State machine: STOPPED → RUNNING → COMPLETE
 * - Callback-based integration: vehicle provides start/verify/complete functions
 * 
 * Usage Pattern:
 * 1. Construction: Provide cmd_start_fn, cmd_verify_fn, mission_complete_fn callbacks
 * 2. init(): Initialize storage and load mission count
 * 3. start()/resume(): Begin mission execution
 * 4. update(): Call at ≥10Hz to advance mission and execute commands
 * 5. Vehicle callbacks handle actual command execution
 * 
 * Thread Safety:
 * - Use get_semaphore() to lock when performing multi-command operations
 * - Storage access is protected by internal semaphore
 * 
 * @note Mission command 0 is always reserved for home position
 * @note Maximum mission size determined by storage capacity (EEPROM or SD card)
 * 
 * @warning Modifying active mission commands requires mission restart to take effect
 * @warning DO_JUMP commands limited to prevent infinite loops (see AP_MISSION_JUMP_TIMES_MAX)
 * @warning EEPROM has limited write cycles - avoid excessive mission updates
 * 
 * Source: libraries/AP_Mission/AP_Mission.h:60-971, libraries/AP_Mission/AP_Mission.cpp:1-150
 */
class AP_Mission
{

public:
    /**
     * @struct Jump_Command
     * @brief DO_JUMP command payload for mission branching
     * 
     * @details Allows mission to jump to a different command index, optionally repeating.
     *          Infinite loops prevented by AP_MISSION_JUMP_TIMES_MAX limit.
     */
    struct PACKED Jump_Command {
        uint16_t target;        ///< Target command index (0-based mission position)
        int16_t num_times;      ///< Number of times to repeat jump. -1 = repeat forever (AP_MISSION_JUMP_REPEAT_FOREVER)
    };

    /**
     * @struct Conditional_Delay_Command
     * @brief CONDITION_DELAY command payload - delays next DO command execution
     * 
     * @details Postpones execution of subsequent DO commands for specified duration.
     *          Navigation continues during delay.
     */
    struct PACKED Conditional_Delay_Command {
        float seconds;          ///< Delay period in seconds (s)
    };

    /**
     * @struct Conditional_Distance_Command
     * @brief CONDITION_DISTANCE command payload - delays until distance from waypoint
     * 
     * @details Postpones DO command execution until vehicle is within specified distance
     *          of the next navigation waypoint.
     */
    struct PACKED Conditional_Distance_Command {
        float meters;           ///< Distance from next waypoint in meters (m). DO commands execute when closer than this.
    };

    /**
     * @struct Yaw_Command
     * @brief CONDITION_YAW and DO_SET_YAW command payload - yaw control
     * 
     * @details Controls vehicle heading with specified angle and turn rate.
     *          Used for pointing vehicle before actions (e.g., camera trigger).
     */
    struct PACKED Yaw_Command {
        float angle_deg;        ///< Target yaw angle in degrees (deg). 0=North, 90=East, 180=South, 270=West
        float turn_rate_dps;    ///< Turn rate in degrees per second (deg/s). 0 = use vehicle default rate
        int8_t direction;       ///< Turn direction: -1 = counter-clockwise, +1 = clockwise
        uint8_t relative_angle; ///< Angle mode: 0 = absolute angle (North-referenced), 1 = relative to current heading
    };

    /**
     * @struct Change_Speed_Command
     * @brief DO_CHANGE_SPEED command payload - modify vehicle speed
     * 
     * @details Changes target speed during mission execution. Affects subsequent waypoints
     *          until another speed change command or mission completion.
     */
    struct PACKED Change_Speed_Command {
        uint8_t speed_type;     ///< Speed type: 0=airspeed (Plane/Copter), 1=ground speed
        float target_ms;        ///< Target speed in meters per second (m/s). -1 = no change
        float throttle_pct;     ///< Throttle percentage (0-100%). 0 = no change. Plane only.
    };

    /**
     * @struct Set_Relay_Command
     * @brief DO_SET_RELAY command payload - control relay outputs
     * 
     * @details Sets relay on or off. Relay voltage depends on board hardware
     *          (typically 3.3V or 5V when on).
     */
    struct PACKED Set_Relay_Command {
        uint8_t num;            ///< Relay number (typically 0-3, hardware dependent)
        uint8_t state;          ///< Relay state: 0=off (0V), 1=on (3.3V or 5V depending on board)
    };

    /**
     * @struct Repeat_Relay_Command
     * @brief DO_REPEAT_RELAY command payload - toggle relay repeatedly
     * 
     * @details Cycles relay on and off for specified number of times with given period.
     *          Useful for triggering camera shutters or other pulsed devices.
     */
    struct PACKED Repeat_Relay_Command {
        uint8_t num;            ///< Relay number (typically 0-3, hardware dependent)
        int16_t repeat_count;   ///< Number of on/off cycles to execute
        float cycle_time;       ///< Cycle period in seconds (s). Total on+off time for one cycle.
    };

    /**
     * @struct Set_Servo_Command
     * @brief DO_SET_SERVO command payload - position servo output
     * 
     * @details Sets a servo channel to specified PWM value. Used for gimbals, grippers,
     *          camera triggers, or any PWM-controlled device.
     */
    struct PACKED Set_Servo_Command {
        uint8_t channel;        ///< Servo output channel number (hardware dependent, typically 1-16)
        uint16_t pwm;           ///< PWM pulse width in microseconds (μs). Typically 1000-2000μs.
    };

    /**
     * @struct Repeat_Servo_Command
     * @brief DO_REPEAT_SERVO command payload - cycle servo position repeatedly
     * 
     * @details Moves servo to PWM position, returns to trim, repeats for count times.
     *          Useful for dropping multiple items or pulsing mechanisms.
     */
    struct PACKED Repeat_Servo_Command {
        uint8_t channel;        ///< Servo output channel number (hardware dependent, typically 1-16)
        uint16_t pwm;           ///< PWM pulse width in microseconds (μs). Typically 1000-2000μs.
        int16_t repeat_count;   ///< Number of move-to-PWM and return-to-trim cycles
        float cycle_time;       ///< Cycle period in seconds (s). Time for complete move-and-return cycle.
    };

    /**
     * @struct Mount_Control
     * @brief DO_MOUNT_CONTROL command payload - position camera gimbal
     * 
     * @details Controls gimbal/mount pointing direction. Angles are stabilized
     *          (earth-frame referenced) unless mount is in RC targeting mode.
     */
    struct PACKED Mount_Control {
        float pitch;            ///< Pitch angle in degrees (deg). Positive = up, negative = down
        float roll;             ///< Roll angle in degrees (deg). Positive = right, negative = left
        float yaw;              ///< Yaw angle in degrees (deg), relative to vehicle heading. 0 = forward
    };

    /**
     * @struct Digicam_Configure
     * @brief DO_DIGICAM_CONFIGURE command payload - configure camera settings
     * 
     * @details Sets camera shooting parameters. Support depends on camera type.
     *          Used with MAVLink camera protocol.
     */
    struct PACKED Digicam_Configure {
        uint8_t shooting_mode;  ///< Shooting mode: 1=ProgramAuto, 2=AV, 3=TV, 4=Manual, 5=IntelligentAuto, 6=SuperiorAuto
        uint16_t shutter_speed; ///< Shutter speed (format camera-dependent)
        uint8_t aperture;       ///< Aperture as F-stop × 10 (e.g., 28 = F2.8)
        uint16_t ISO;           ///< ISO sensitivity (e.g., 100, 200, 400, 800)
        uint8_t exposure_type;  ///< Exposure compensation type
        uint8_t cmd_id;         ///< Command ID for camera (camera-dependent)
        float engine_cutoff_time;   ///< Engine cutoff time in seconds (s) before shot (Plane only)
    };

    /**
     * @struct Digicam_Control
     * @brief DO_DIGICAM_CONTROL command payload - trigger camera actions
     * 
     * @details Controls camera operation (shoot, zoom, focus). Support depends on camera.
     */
    struct PACKED Digicam_Control {
        uint8_t session;        ///< Session control: 0=off, 1=on
        uint8_t zoom_pos;       ///< Zoom position (camera-dependent scale)
        int8_t zoom_step;       ///< Zoom step: -1=zoom out, 0=no change, +1=zoom in
        uint8_t focus_lock;     ///< Focus lock: 0=unlock, 1=lock
        uint8_t shooting_cmd;   ///< Shooting command (camera-dependent)
        uint8_t cmd_id;         ///< Command ID for camera (camera-dependent)
    };

    /**
     * @struct Cam_Trigg_Distance
     * @brief DO_SET_CAM_TRIGG_DIST command payload - distance-based camera triggering
     * 
     * @details Automatically triggers camera every X meters of travel. Used for photogrammetry
     *          and mapping missions. Set meters=0 to disable distance-based triggering.
     */
    struct PACKED Cam_Trigg_Distance {
        float meters;           ///< Distance between triggers in meters (m). 0 = disable distance triggering
        uint8_t trigger;        ///< Immediate trigger: 1=trigger one shot immediately, 0=no immediate shot
    };

    /**
     * @struct Gripper_Command
     * @brief DO_GRIPPER command payload - control cargo gripper
     * 
     * @details Opens or closes gripper for cargo operations (package delivery, etc.).
     */
    struct PACKED Gripper_Command {
        uint8_t num;            ///< Gripper number (for vehicles with multiple grippers)
        uint8_t action;         ///< Action: 0=release/open, 1=grab/close
    };

    /**
     * @struct AuxFunction
     * @brief DO_AUX_FUNCTION command payload - trigger RC auxiliary function
     * 
     * @details Activates an RC auxiliary function as if triggered by an RC switch.
     *          Allows mission control of features normally controlled by RC switches.
     */
    struct PACKED AuxFunction {
        uint16_t function;  ///< Auxiliary function ID (see RC_Channel::AUX_FUNC enum)
        uint8_t switchpos;  ///< Switch position (see RC_Channel::AuxSwitchPos enum: LOW/MIDDLE/HIGH)
    };

    /**
     * @struct Altitude_Wait
     * @brief NAV_ALTITUDE_WAIT command payload - wait for altitude condition (high-altitude balloons)
     * 
     * @details Loiters until reaching target altitude with specified descent rate and optional
     *          wiggle maneuver. Specialized for high-altitude balloon operations.
     */
    struct PACKED Altitude_Wait {
        float altitude;         ///< Target altitude in meters (m) above home
        float descent_rate;     ///< Required descent rate in meters per second (m/s)
        uint8_t wiggle_time;    ///< Wiggle maneuver duration in seconds (s). 0=no wiggle
    };

    /**
     * @struct Guided_Limits_Command
     * @brief NAV_GUIDED_ENABLE command payload - set limits for guided mode
     * 
     * @details Defines spatial and temporal limits for guided mode operation.
     *          Mission aborts if vehicle exceeds limits. Used for safe autonomous operations.
     * 
     * @note Maximum time limit is stored in Mission_Command.p1 field (not in this structure)
     */
    struct PACKED Guided_Limits_Command {
        float alt_min;          ///< Minimum altitude in meters (m). Command aborts if below. 0=no lower limit
        float alt_max;          ///< Maximum altitude in meters (m). Command aborts if above. 0=no upper limit
        float horiz_max;        ///< Maximum horizontal distance in meters (m) from start. 0=no limit
    };

    /**
     * @struct Do_VTOL_Transition
     * @brief DO_VTOL_TRANSITION command payload - switch between fixed-wing and multicopter modes
     * 
     * @details Commands QuadPlane to transition between multicopter and fixed-wing flight.
     *          Used in QuadPlane missions for efficient long-range flight.
     */
    struct PACKED Do_VTOL_Transition {
        uint8_t target_state;   ///< Target state: 3=MC mode (multicopter), 4=FW mode (fixed-wing)
    };

    /**
     * @struct Navigation_Delay_Command
     * @brief NAV_DELAY command payload - delay mission progression
     * 
     * @details Delays progression to next navigation command either for specified duration
     *          or until absolute UTC time. Vehicle continues current navigation during delay.
     * 
     * @note Use seconds for relative delay, or hour_utc/min_utc/sec_utc for absolute time
     */
    struct PACKED Navigation_Delay_Command {
        float seconds;          ///< Relative delay in seconds (s). -1=use absolute time instead
        int8_t hour_utc;        ///< Absolute time hour in UTC (0-23). -1=use relative delay
        int8_t min_utc;         ///< Absolute time minute in UTC (0-59)
        int8_t sec_utc;         ///< Absolute time second in UTC (0-59)
    };

    /**
     * @struct Do_Engine_Control
     * @brief DO_ENGINE_CONTROL command payload - start/stop internal combustion engine
     * 
     * @details Controls engine starting/stopping for vehicles with ICE (Internal Combustion Engine).
     *          Supports cold start procedures and altitude-delayed starts.
     */
    struct PACKED Do_Engine_Control {
        bool start_control;     ///< Engine control: true=start engine, false=stop engine
        bool cold_start;        ///< Cold start: true=use cold start procedure, false=normal start
        uint16_t height_delay_cm; ///< Height delay in centimeters (cm). Start engine after reaching this altitude
        bool allow_disarmed_start; ///< Safety: true=allow engine start while disarmed (use with caution!)
    };

    /**
     * @struct Set_Yaw_Speed
     * @brief NAV_SET_YAW_SPEED command payload - move at speed while facing direction (Rover)
     * 
     * @details Commands vehicle to move at specified speed while maintaining heading.
     *          Used for Rover to drive in specific direction. Continues until next nav command.
     */
    struct PACKED Set_Yaw_Speed {
        float angle_deg;        ///< Target heading in degrees (deg). 0=North, 90=East
        float speed;            ///< Travel speed in meters per second (m/s)
        uint8_t relative_angle; ///< Angle mode: 0=absolute (North-referenced), 1=relative to current heading
    };

    /**
     * @struct Winch_Command
     * @brief DO_WINCH command payload - control winch for cargo operations
     * 
     * @details Controls winch deployment/retraction for cargo delivery, sampling, or
     *          tethered operations. Multiple control modes available.
     */
    struct PACKED Winch_Command {
        uint8_t num;            ///< Winch number (for vehicles with multiple winches)
        uint8_t action;         ///< Action: 0=relax, 1=length control (absolute), 2=rate control
        float release_length;   ///< Cable length in meters (m). Positive=unwind, negative=wind in
        float release_rate;     ///< Release rate in meters per second (m/s) for rate control mode
    };

    /**
     * @struct scripting_Command
     * @brief DO_SCRIPTING command payload - pass parameters to Lua script
     * 
     * @details Provides three float parameters to running Lua script.
     *          Script checks for and handles these parameters. Meaning depends on script.
     * 
     * @note Requires AP_SCRIPTING_ENABLED
     */
    struct PACKED scripting_Command {
        float p1;               ///< Parameter 1 (script-defined meaning)
        float p2;               ///< Parameter 2 (script-defined meaning)
        float p3;               ///< Parameter 3 (script-defined meaning)
    };

#if AP_SCRIPTING_ENABLED
    /**
     * @struct nav_script_time_Command_tag0
     * @brief NAV_SCRIPT_TIME command payload (legacy storage format)
     * 
     * @details Old storage format for script-controlled navigation.
     *          Retained for backward compatibility with existing missions.
     * 
     * @deprecated Use nav_script_time_Command (new format) for new missions
     */
    struct PACKED nav_script_time_Command_tag0 {
        uint8_t command;        ///< Script command ID
        uint8_t timeout_s;      ///< Timeout in seconds (s)
        float arg1;             ///< Script argument 1 (full float precision)
        float arg2;             ///< Script argument 2 (full float precision)
    };

    /**
     * @struct nav_script_time_Command
     * @brief NAV_SCRIPT_TIME command payload - script-controlled navigation waypoint
     * 
     * @details Allows Lua script to control vehicle navigation for duration/until complete.
     *          Script provides verify function. Command times out if script doesn't complete.
     *          New storage format uses Float16 for better packing.
     * 
     * @note Requires AP_SCRIPTING_ENABLED and running Lua script to handle command
     */
    struct PACKED nav_script_time_Command {
        uint8_t command;        ///< Script-defined command ID (script determines meaning)
        uint8_t timeout_s;      ///< Timeout in seconds (s). Command completes if script doesn't finish in time
        Float16_t arg1;         ///< Script argument 1 (16-bit float, script-defined meaning)
        Float16_t arg2;         ///< Script argument 2 (16-bit float, script-defined meaning)
        int16_t arg3;           ///< Script argument 3 (integer, script-defined meaning)
        int16_t arg4;           ///< Script argument 4 (integer, script-defined meaning)
    };
#endif

    /**
     * @struct nav_attitude_time_Command
     * @brief NAV_ATTITUDE_TIME command payload - hold attitude for duration
     * 
     * @details Commands vehicle to hold specified attitude (roll/pitch/yaw) and climb rate
     *          for given time. Used for aerobatic maneuvers or test sequences.
     */
    struct PACKED nav_attitude_time_Command {
        uint16_t time_sec;      ///< Duration to hold attitude in seconds (s)
        int16_t roll_deg;       ///< Target roll angle in degrees (deg)
        int8_t pitch_deg;       ///< Target pitch angle in degrees (deg)
        int16_t yaw_deg;        ///< Target yaw angle in degrees (deg)
        int16_t climb_rate;     ///< Climb rate in centimeters per second (cm/s)
    };

    /**
     * @struct gimbal_manager_pitchyaw_Command
     * @brief DO_GIMBAL_MANAGER_PITCHYAW command payload - advanced gimbal control
     * 
     * @details Controls gimbal using gimbal manager protocol with angle and rate control.
     *          Provides finer control than older mount control commands.
     */
    struct PACKED gimbal_manager_pitchyaw_Command {
        int8_t pitch_angle_deg; ///< Pitch angle in degrees (deg)
        int16_t yaw_angle_deg;  ///< Yaw angle in degrees (deg)
        int8_t pitch_rate_degs; ///< Pitch rate in degrees per second (deg/s)
        int8_t yaw_rate_degs;   ///< Yaw rate in degrees per second (deg/s)
        uint8_t flags;          ///< Control flags (see GIMBAL_MANAGER_FLAGS)
        uint8_t gimbal_id;      ///< Gimbal device ID (for multi-gimbal systems)
    };

    /**
     * @struct image_start_capture_Command
     * @brief IMAGE_START_CAPTURE command payload - start time-lapse image capture
     * 
     * @details Starts automatic image capture at specified interval. Used for photogrammetry
     *          and mapping. Continues until IMAGE_STOP_CAPTURE or mission end.
     */
    struct PACKED image_start_capture_Command {
        uint8_t instance;       ///< Camera instance (for multi-camera systems)
        float interval_s;       ///< Interval between captures in seconds (s). 0=capture as fast as possible
        uint16_t total_num_images; ///< Total number of images to capture. 0=capture indefinitely
        uint16_t start_seq_number; ///< Starting sequence number for image numbering
    };

    /**
     * @struct set_camera_zoom_Command
     * @brief SET_CAMERA_ZOOM command payload - control camera zoom level
     * 
     * @details Controls camera optical or digital zoom. Support depends on camera.
     */
    struct PACKED set_camera_zoom_Command {
        uint8_t zoom_type;      ///< Zoom type: 0=step, 1=continuous, 2=range, 3=focal length
        float zoom_value;       ///< Zoom value (meaning depends on zoom_type)
    };

    /**
     * @struct set_camera_focus_Command
     * @brief SET_CAMERA_FOCUS command payload - control camera focus
     * 
     * @details Controls camera focus distance or mode. Support depends on camera.
     */
    struct PACKED set_camera_focus_Command {
        uint8_t focus_type;     ///< Focus type: 0=step, 1=continuous, 2=range, 3=meters, 4=auto, 5=auto single, 6=auto continuous
        float focus_value;      ///< Focus value (meaning depends on focus_type)
    };

    /**
     * @struct set_camera_source_Command
     * @brief SET_CAMERA_SOURCE command payload - select camera video source
     * 
     * @details Selects video source for cameras with multiple inputs (e.g., thermal + optical).
     */
    struct PACKED set_camera_source_Command {
        uint8_t instance;       ///< Camera instance (for multi-camera systems)
        uint8_t primary_source; ///< Primary video source ID
        uint8_t secondary_source; ///< Secondary video source ID (for picture-in-picture)
    };

    /**
     * @struct video_start_capture_Command
     * @brief VIDEO_START_CAPTURE command payload - start video recording
     * 
     * @details Starts video recording on specified stream. Continues until VIDEO_STOP_CAPTURE.
     */
    struct PACKED video_start_capture_Command {
        uint8_t video_stream_id; ///< Video stream ID to start recording
    };

    /**
     * @struct video_stop_capture_Command
     * @brief VIDEO_STOP_CAPTURE command payload - stop video recording
     * 
     * @details Stops video recording on specified stream.
     */
    struct PACKED video_stop_capture_Command {
        uint8_t video_stream_id; ///< Video stream ID to stop recording
    };

    /**
     * @union Content
     * @brief Command-specific payload union - stores parameters for different command types
     * 
     * @details Mission commands use this union to store command-specific parameters efficiently.
     *          Only one member is active based on the command ID. For location-based commands
     *          (waypoints, loiter, etc.), the 'location' member is used. For action commands,
     *          the appropriate command structure is used.
     * 
     * @note Union size is determined by largest member (Location)
     * @note Total Mission_Command size is 15 bytes when packed for storage
     */
    union Content {
        Jump_Command jump;                          ///< DO_JUMP: Branch to another command index
        Conditional_Delay_Command delay;            ///< CONDITION_DELAY: Time-based delay
        Conditional_Distance_Command distance;      ///< CONDITION_DISTANCE: Distance-based trigger
        Yaw_Command yaw;                            ///< CONDITION_YAW, DO_SET_YAW: Yaw control
        Change_Speed_Command speed;                 ///< DO_CHANGE_SPEED: Speed adjustment
        Set_Relay_Command relay;                    ///< DO_SET_RELAY: Relay control
        Repeat_Relay_Command repeat_relay;          ///< DO_REPEAT_RELAY: Relay cycling
        Set_Servo_Command servo;                    ///< DO_SET_SERVO: Servo positioning
        Repeat_Servo_Command repeat_servo;          ///< DO_REPEAT_SERVO: Servo cycling
        Mount_Control mount_control;                ///< DO_MOUNT_CONTROL: Gimbal positioning
        Digicam_Configure digicam_configure;        ///< DO_DIGICAM_CONFIGURE: Camera settings
        Digicam_Control digicam_control;            ///< DO_DIGICAM_CONTROL: Camera actions
        Cam_Trigg_Distance cam_trigg_dist;          ///< DO_SET_CAM_TRIGG_DIST: Distance-based camera trigger
        Gripper_Command gripper;                    ///< DO_GRIPPER: Gripper control
        AuxFunction auxfunction;                    ///< DO_AUX_FUNCTION: Auxiliary function trigger
        Guided_Limits_Command guided_limits;        ///< NAV_GUIDED_ENABLE: Guided mode limits
        Altitude_Wait altitude_wait;                ///< NAV_ALTITUDE_WAIT: Altitude condition wait
        Do_VTOL_Transition do_vtol_transition;      ///< DO_VTOL_TRANSITION: QuadPlane mode switch
        Do_Engine_Control do_engine_control;        ///< DO_ENGINE_CONTROL: ICE engine start/stop
        Navigation_Delay_Command nav_delay;         ///< NAV_DELAY: Navigation delay
        Set_Yaw_Speed set_yaw_speed;                ///< NAV_SET_YAW_SPEED: Rover yaw+speed control
        Winch_Command winch;                        ///< DO_WINCH: Winch control
        scripting_Command scripting;                ///< DO_SCRIPTING: Lua script parameters
#if AP_SCRIPTING_ENABLED
        nav_script_time_Command nav_script_time;    ///< NAV_SCRIPT_TIME: Script navigation
#endif
        nav_attitude_time_Command nav_attitude_time; ///< NAV_ATTITUDE_TIME: Attitude hold
        gimbal_manager_pitchyaw_Command gimbal_manager_pitchyaw; ///< DO_GIMBAL_MANAGER_PITCHYAW: Advanced gimbal control
        image_start_capture_Command image_start_capture; ///< IMAGE_START_CAPTURE: Start time-lapse capture
        set_camera_zoom_Command set_camera_zoom;    ///< SET_CAMERA_ZOOM: Camera zoom control
        set_camera_focus_Command set_camera_focus;  ///< SET_CAMERA_FOCUS: Camera focus control
        set_camera_source_Command set_camera_source; ///< SET_CAMERA_SOURCE: Camera video source select
        video_start_capture_Command video_start_capture; ///< VIDEO_START_CAPTURE: Start video recording
        video_stop_capture_Command video_stop_capture; ///< VIDEO_STOP_CAPTURE: Stop video recording
        Location location{};                        ///< Location for waypoint commands (WAYPOINT, LOITER, LAND, etc.)
    };

    /**
     * @struct Mission_Command
     * @brief Complete mission command containing ID, parameters, and payload
     * 
     * @details Represents a single mission item that can be executed by the autopilot.
     *          Commands are stored sequentially in storage (EEPROM or SD card) with
     *          index 0 reserved for home position. Each command is 15 bytes when packed.
     * 
     * Command Types:
     * - NAV commands: Define vehicle navigation targets (WAYPOINT, LOITER, LAND, etc.)
     * - DO commands: Immediate actions executed once (SET_SERVO, GRIPPER, etc.)
     * - CONDITIONAL commands: Time or distance-based triggers (DELAY, DISTANCE)
     * 
     * Storage Format:
     * - index: 2 bytes - Command position in mission (0 = home, 1+ = mission items)
     * - id: 2 bytes - MAVLink MAV_CMD enumeration value
     * - p1: 2 bytes - General parameter, meaning varies by command type
     * - content: 8 bytes - Command-specific payload (union)
     * - type_specific_bits: 1 byte - Additional storage for location commands
     * 
     * @note Total size: 15 bytes packed (AP_MISSION_EEPROM_COMMAND_SIZE)
     * @note Index 0 is always home position, first real mission command is index 1
     */
    struct Mission_Command {
        /**
         * @brief Command position in mission list
         * 
         * @details Zero-based index with special meaning:
         *          - 0: Home position (reserved, set via write_home_to_storage())
         *          - 1 to num_commands-1: Mission commands
         *          - AP_MISSION_CMD_INDEX_NONE (65535): Invalid/unset
         * 
         * @note First real mission command starts at index 1 (AP_MISSION_FIRST_REAL_COMMAND)
         */
        uint16_t index;
        
        /**
         * @brief MAVLink command ID (MAV_CMD enumeration)
         * 
         * @details Standard MAVLink command identifier defining command type and behavior.
         *          Examples: MAV_CMD_NAV_WAYPOINT (16), MAV_CMD_DO_JUMP (177),
         *                    MAV_CMD_CONDITION_DELAY (112)
         * 
         * @note AP_MISSION_CMD_ID_NONE (0) indicates invalid or missing command
         * @see https://mavlink.io/en/messages/common.html#mav_commands
         */
        uint16_t id;
        
        /**
         * @brief General purpose parameter 1
         * 
         * @details Meaning varies by command type:
         * - NAV_WAYPOINT: Hold time (seconds, 0 = no hold)
         * - LOITER_TIME: Loiter time (seconds)
         * - LOITER_TURNS: Number of turns (use get_loiter_turns() for fractional)
         * - DO_JUMP: Not used (reserved)
         * - CONDITION_DELAY: Not used (time in content.delay.seconds)
         * 
         * @note Units vary by command - always check MAVLink specification
         */
        uint16_t p1;
        
        /**
         * @brief Command-specific payload
         * 
         * @details Union containing command parameters. Active member determined by command ID.
         * For location commands (waypoints, loiter), use content.location.
         * For action commands, use appropriate command structure.
         * 
         * @note Only one union member is valid at a time
         */
        Content content;

        /**
         * @brief Additional storage bits for location commands
         * 
         * @details Bitmask providing extra storage for commands that use Location.
         * - Bit 0: Reserved
         * - Bit 1: LOITER_TURNS fractional mode (turns stored as value/256)
         * - Bits 2-7: Available for future use
         * 
         * @note Only meaningful for commands storing location data
         */
        uint8_t type_specific_bits;

        /**
         * @brief Get human-readable command type name
         * 
         * @return String describing command type (e.g., "NAV_WAYPOINT", "DO_JUMP")
         * @note Useful for logging and debugging
         */
        const char *type() const;

        /**
         * @brief Equality comparison operator
         * 
         * @param b Command to compare against
         * @return true if all bytes match (including unused padding)
         * 
         * @warning Compares entire structure including potential padding bytes
         */
        bool operator ==(const Mission_Command &b) const { return (memcmp(this, &b, sizeof(Mission_Command)) == 0); }
        
        /**
         * @brief Inequality comparison operator
         * 
         * @param b Command to compare against
         * @return true if any bytes differ
         */
        bool operator !=(const Mission_Command &b) const { return !operator==(b); }

        /**
         * @brief Get number of turns for LOITER_TURNS command
         * 
         * @details Handles special storage format for loiter turns that allows fractional values.
         * Normal mode: p1 low byte = integer turns
         * Fractional mode: p1 low byte / 256.0 = fractional turns (when type_specific_bits bit 1 set)
         * 
         * @return Number of loiter turns (may be fractional)
         * 
         * @note Only valid for MAV_CMD_NAV_LOITER_TURNS commands
         * @note Fractional turns enabled when type_specific_bits & 0x02 is true
         */
        float get_loiter_turns(void) const {
            float turns = LOWBYTE(p1);
            if (type_specific_bits & (1U<<1)) {
                // special storage handling allows for fractional turns
                turns *= (1.0/256.0);
            }
            return turns;
        }
    };


    /**
     * @typedef mission_cmd_fn_t
     * @brief Callback function type for command execution
     * 
     * @param cmd Mission command to execute or verify
     * @return true if command started successfully or verification passed, false otherwise
     * 
     * @details Vehicle code provides two callbacks of this type:
     * - cmd_start_fn: Called once when command becomes active, initializes command execution
     * - cmd_verify_fn: Called repeatedly (typically 10Hz+) to check if command is complete
     */
    FUNCTOR_TYPEDEF(mission_cmd_fn_t, bool, const Mission_Command&);
    
    /**
     * @typedef mission_complete_fn_t
     * @brief Callback function type for mission completion notification
     * 
     * @details Called once when mission reaches the end or is explicitly completed.
     *          Vehicle code should handle post-mission behavior (e.g., loiter, land, disarm).
     */
    FUNCTOR_TYPEDEF(mission_complete_fn_t, void);

    /**
     * @brief AP_Mission constructor - initializes mission management system
     * 
     * @param cmd_start_fn Callback to start a new mission command
     * @param cmd_verify_fn Callback to verify command completion
     * @param mission_complete_fn Callback when mission completes
     * 
     * @details Sets up mission singleton and registers vehicle-specific callbacks.
     *          The callbacks integrate mission logic with vehicle-specific execution:
     *          - cmd_start_fn: Vehicle initializes command execution (e.g., set waypoint target)
     *          - cmd_verify_fn: Vehicle checks if command complete (e.g., reached waypoint)
     *          - mission_complete_fn: Vehicle handles mission end (e.g., loiter or land)
     * 
     * @note Constructor enforces singleton pattern in SITL builds
     * @note Loads parameter defaults and clears command queues
     */
    AP_Mission(mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
        _cmd_start_fn(cmd_start_fn),
        _cmd_verify_fn(cmd_verify_fn),
        _mission_complete_fn(mission_complete_fn),
        _prev_nav_cmd_id(AP_MISSION_CMD_ID_NONE),
        _prev_nav_cmd_index(AP_MISSION_CMD_INDEX_NONE),
        _prev_nav_cmd_wp_index(AP_MISSION_CMD_INDEX_NONE)
    {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (_singleton != nullptr) {
            AP_HAL::panic("Mission must be singleton");
        }
#endif
        _singleton = this;

        // load parameter defaults
        AP_Param::setup_object_defaults(this, var_info);

        // clear commands
        _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
        _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    }

    /**
     * @brief Get the AP_Mission singleton instance
     * 
     * @return Pointer to the singleton AP_Mission object, or nullptr if not constructed
     * 
     * @details The mission object must be constructed before calling this.
     *          Typically accessed via AP::mission() wrapper function.
     * 
     * @note Singleton pattern ensures only one mission manager exists
     */
    static AP_Mission *get_singleton()
    {
        return _singleton;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mission);

    /**
     * @enum mission_state
     * @brief Mission execution state machine states
     * 
     * @details Tracks current mission execution status. State transitions:
     * - STOPPED → RUNNING: via start() or resume()
     * - RUNNING → COMPLETE: when mission reaches end or explicit completion
     * - RUNNING → STOPPED: via stop()
     * - COMPLETE → RUNNING: via start() (restart from beginning)
     * - Any state → STOPPED: via stop() or clear()
     */
    enum mission_state {
        MISSION_STOPPED=0,   ///< Mission not executing, update() has no effect. Initial state or stopped via stop().
        MISSION_RUNNING=1,   ///< Mission actively executing, update() advances commands and calls vehicle callbacks.
        MISSION_COMPLETE=2   ///< Mission finished, all commands executed. Calls mission_complete_fn callback.
    };

    ///
    /// public mission methods
    ///

    /**
     * @brief Initialize mission system and verify storage
     * 
     * @details Performs mission startup sequence:
     * - Checks EEPROM version matches library version
     * - Clears mission if version mismatch detected
     * - Loads mission command count from storage
     * - Initializes jump tracking arrays
     * - Sets up storage backend (EEPROM or SD card)
     * 
     * @note Must be called once during vehicle initialization before using mission
     * @warning Mission cleared if storage version incompatible (prevents corruption)
     */
    void init();

    /**
     * @brief Get current mission execution state
     * 
     * @return Current mission_state (STOPPED, RUNNING, or COMPLETE)
     * 
     * @details Query mission status to determine if:
     * - Mission is actively executing (RUNNING)
     * - Mission stopped or not started (STOPPED)
     * - Mission reached end successfully (COMPLETE)
     */
    mission_state state() const
    {
        return _flags.state;
    }

    /**
     * @brief Get total number of commands in mission
     * 
     * @return Number of commands including home (index 0)
     * 
     * @details Count includes:
     * - Index 0: Home position (always present if mission loaded)
     * - Index 1 to num_commands-1: Mission commands
     * 
     * @note Empty mission returns 0, mission with only home returns 1
     * @note Use present() to check if actual mission commands exist
     */
    uint16_t num_commands() const
    {
        return _cmd_total;
    }

    /**
     * @brief Get maximum number of commands that can be stored
     * 
     * @return Maximum mission capacity based on available storage
     * 
     * @details Capacity determined by:
     * - EEPROM: StorageManager allocated space / 15 bytes per command
     * - SD card: Larger capacity (platform dependent)
     * 
     * @note Attempting to add commands beyond this limit will fail
     */
    uint16_t num_commands_max() const {
        return _commands_max;
    }

    /**
     * @brief Check if mission has actual commands (beyond home)
     * 
     * @return true if mission has commands beyond index 0 (home)
     * 
     * @details Useful to check if there's an actual mission to execute.
     * Returns false if only home position loaded or mission empty.
     */
    bool present() const { return _cmd_total > 1; }

    /**
     * @brief Start mission from beginning
     * 
     * @details Resets mission to first command (index 1) and transitions to RUNNING state.
     * Current navigation and do commands cleared and reloaded from mission start.
     * All jump counters reset to allow fresh execution.
     * 
     * @note Ignores current mission progress - always restarts from command 1
     * @note Does not validate mission - invalid commands may cause issues during execution
     * @see start_or_resume() for conditional restart based on MIS_RESTART parameter
     */
    void start();

    /**
     * @brief Stop mission execution
     * 
     * @details Transitions mission to STOPPED state. Subsequent update() calls have no effect
     * until mission restarted via start() or resumed via resume(). Current command execution
     * interrupted but mission position preserved for resume().
     * 
     * @note Vehicle continues current mode but mission logic suspended
     * @note Mission position saved - resume() will continue from current command
     */
    void stop();

    /**
     * @brief Resume mission from last position
     * 
     * @details Continues mission execution from where it was stopped or interrupted.
     * Transitions to RUNNING state and re-initializes currently active commands.
     * May rewind to previous waypoint if configured via MIS_RESTART_DIST.
     * 
     * @note Running commands re-executed (start callbacks called again)
     * @note Jump counters preserved from previous execution
     * @see set_force_resume() to override MIS_RESTART behavior
     */
    void resume();

    /**
     * @brief Start or resume mission based on MIS_RESTART parameter
     * 
     * @details Calls start() if MIS_RESTART=1 (auto-restart), otherwise calls resume().
     * Provides consistent mission entry point that respects user preference.
     * 
     * @note MIS_RESTART=0: Resume from last position (default)
     * @note MIS_RESTART=1: Always restart from beginning
     * @note Can be overridden by set_force_resume(true)
     */
    void start_or_resume();

    /**
     * @brief Check if mission starts with takeoff command
     * 
     * @return true if first real command (index 1) is takeoff type
     * 
     * @details Checks for: MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_VTOL_TAKEOFF
     * Useful for pre-flight validation and auto mode entry logic.
     * 
     * @note Only checks first command, doesn't validate entire mission
     */
    bool starts_with_takeoff_cmd();

    /**
     * @brief Reset mission to first command without clearing
     * 
     * @details Sets current command index to beginning (index 1) but maintains STOPPED state.
     * Unlike start(), doesn't transition to RUNNING. Unlike clear(), doesn't erase mission.
     * 
     * @note Mission remains stopped - call start() or resume() to begin execution
     */
    void reset();

    /**
     * @brief Clear entire mission from storage
     * 
     * @return true if successfully cleared
     * 
     * @details Erases all mission commands from EEPROM/SD card. Home position (index 0) 
     * preserved. Mission count reset to 1. Updates _last_change_time_ms.
     * 
     * @warning Writes to EEPROM - avoid excessive calls due to write cycle limits
     * @warning Cannot be undone - mission data permanently lost
     * 
     * @note Mission transitions to STOPPED state
     * @note Ground station should be notified of mission clear
     */
    bool clear();

    /**
     * @brief Remove mission commands beyond specified index
     * 
     * @param index Commands beyond this index will be removed
     * 
     * @details Shortens mission by removing commands from index+1 to end.
     * Updates command count. Useful for dynamic mission modification.
     * 
     * @warning Cannot truncate home (index 0) - minimum mission size is 1
     * @warning Writes to EEPROM for each removed command
     * 
     * @note If current command beyond truncation point, mission may stop
     */
    void truncate(uint16_t index);

    /**
     * @brief Update mission execution - must be called regularly
     * 
     * @details Core mission update loop that:
     * - Loads next navigation command if current complete
     * - Loads next do/conditional commands
     * - Calls vehicle cmd_verify_fn to check command progress
     * - Calls vehicle cmd_start_fn when new commands activated
     * - Handles DO_JUMP logic and jump counters
     * - Advances mission when commands complete
     * - Triggers mission_complete_fn when mission ends
     * 
     * @note Should be called at ≥10Hz for smooth mission execution
     * @note Has no effect when mission in STOPPED state
     * @note Vehicle callbacks may be invoked during this call
     * 
     * @warning Missing update() calls causes mission to stall
     * @warning Do not call from interrupt context (uses semaphore)
     */
    void update();

    ///
    /// public command methods
    ///

    /**
     * @brief Add command to end of mission
     * 
     * @param[in,out] cmd Command to add (cmd.index will be updated with new position)
     * @return true if successfully added and written to storage
     * @return false if mission full or storage write failed
     * 
     * @details Appends command to mission list and writes to EEPROM/SD card.
     * The cmd.index field is automatically updated with the new command position.
     * Updates mission count and last change timestamp.
     * 
     * @warning Writes to EEPROM - avoid excessive calls (limited write cycles)
     * @warning Does not affect currently running mission until restart/resume
     * 
     * @note Command added at position _cmd_total (after current last command)
     * @note Ground station should be notified of mission change
     * @note Use get_semaphore() if adding multiple commands atomically
     */
    bool add_cmd(Mission_Command& cmd);

    /**
     * @brief Replace existing command at specified index
     * 
     * @param[in] index Position in mission to replace (0 = home, 1+ = mission commands)
     * @param[in] cmd New command to write at this position
     * @return true if successfully replaced and written to storage
     * @return false if index invalid or storage write failed
     * 
     * @details Overwrites command at specified index with new command.
     * Updates last change timestamp. Does not affect mission count.
     * 
     * @warning Replacing active command has no effect until mission restarted
     * @warning Cannot replace index beyond current mission size
     * @warning Writes to EEPROM - limited write cycles
     * 
     * @note To modify active mission, must call restart_current_nav_cmd() or restart mission
     * @note Ground station should be notified of mission change
     */
    bool replace_cmd(uint16_t index, const Mission_Command& cmd);

    /**
     * @brief Check if command is navigation type
     * 
     * @param[in] cmd Command to check
     * @return true if NAV command (waypoint, loiter, land, etc.)
     * @return false if DO or CONDITIONAL command
     * 
     * @details Navigation commands define vehicle flight path and are the primary
     * mission progression markers. Only one NAV command active at a time.
     * DO and CONDITIONAL commands execute alongside NAV commands.
     * 
     * Examples:
     * - NAV commands: WAYPOINT, LOITER_UNLIM, LAND, TAKEOFF, RTL
     * - DO commands: SET_SERVO, GRIPPER, CHANGE_SPEED
     * - CONDITIONAL: DELAY, DISTANCE, YAW
     * 
     * @note Static method - can be called without AP_Mission instance
     */
    static bool is_nav_cmd(const Mission_Command& cmd);

    /**
     * @brief Get current navigation command
     * 
     * @return Reference to current NAV command being executed
     * 
     * @details Returns the active navigation command that defines current flight path.
     * This is the waypoint, loiter, or other NAV command the vehicle is currently
     * executing or navigating toward.
     * 
     * @note Returns command with index=AP_MISSION_CMD_INDEX_NONE if no NAV command loaded
     * @note Check _nav_cmd.index != AP_MISSION_CMD_INDEX_NONE before using
     */
    const Mission_Command& get_current_nav_cmd() const
    {
        return _nav_cmd;
    }

    /**
     * @brief Get current navigation command index
     * 
     * @return Current NAV command index, or 0 if no command active
     * 
     * @details Returns mission index of currently executing NAV command.
     * Returns 0 (home) if no command loaded - this is for MAVLink compatibility
     * where reporting 0 indicates home/no active command.
     * 
     * @note Returns 0 for both "at home" and "no mission" cases
     * @note Use get_current_nav_cmd() to distinguish between cases
     */
    uint16_t get_current_nav_index() const
    {
        return _nav_cmd.index==AP_MISSION_CMD_INDEX_NONE?0:_nav_cmd.index;
    }

    /**
     * @brief Get current navigation command ID
     * 
     * @return MAVLink command ID (MAV_CMD) of current NAV command
     * 
     * @details Returns the MAV_CMD enumeration value of the active navigation command.
     * Examples: MAV_CMD_NAV_WAYPOINT (16), MAV_CMD_NAV_LOITER_UNLIM (17)
     * 
     * @note Returns AP_MISSION_CMD_ID_NONE (0) if no command active
     */
    uint16_t get_current_nav_id() const
    {
        return _nav_cmd.id;
    }

    /**
     * @brief Get previous navigation command ID
     * 
     * @return MAV_CMD ID of previous NAV command, or AP_MISSION_CMD_ID_NONE
     * 
     * @details Returns ID of the last completed navigation command.
     * Useful for determining what waypoint was just passed.
     * Only ID returned (not full command) to conserve RAM.
     * 
     * @note Returns AP_MISSION_CMD_ID_NONE (0) if no previous NAV command
     * @note Previous command is the last NAV command that was active before current
     */
    uint16_t get_prev_nav_cmd_id() const
    {
        return _prev_nav_cmd_id;
    }

    /**
     * @brief Get previous navigation command index
     * 
     * @return Mission index of previous NAV command, or AP_MISSION_CMD_INDEX_NONE
     * 
     * @details Returns mission position of last completed navigation command.
     * Only index returned (not full command) to conserve RAM.
     * 
     * @note Returns AP_MISSION_CMD_INDEX_NONE (65535) if no previous NAV command
     * @note Useful for mission resume logic and waypoint history
     */
    uint16_t get_prev_nav_cmd_index() const
    {
        return _prev_nav_cmd_index;
    }

    /**
     * @brief Get previous waypoint-type navigation command index
     * 
     * @return Index of previous NAV command containing waypoint, or AP_MISSION_CMD_INDEX_NONE
     * 
     * @details Returns index of last NAV command that contained a Location (waypoint).
     * Excludes NAV commands without locations (like NAV_GUIDED_ENABLE).
     * Useful for calculating heading/distance from last waypoint.
     * Only index returned to conserve RAM.
     * 
     * @note Returns AP_MISSION_CMD_INDEX_NONE (65535) if no previous waypoint
     * @note Waypoint commands include: WAYPOINT, LOITER, LAND, TAKEOFF, etc.
     */
    uint16_t get_prev_nav_cmd_with_wp_index() const
    {
        return _prev_nav_cmd_wp_index;
    }

    /**
     * @brief Get next navigation command in mission
     * 
     * @param[in] start_index Index to begin search from (typically current command + 1)
     * @param[out] cmd Filled with next NAV command if found
     * @return true if NAV command found
     * @return false if no more NAV commands (mission end reached)
     * 
     * @details Searches mission from start_index forward for next navigation command.
     * Accounts for DO_JUMP commands - will follow jumps to find actual next NAV.
     * Skips over DO and CONDITIONAL commands automatically.
     * 
     * @note Does not modify current mission state
     * @note Jump counters may be checked but not incremented
     * @note Useful for look-ahead logic (next waypoint heading, distance, etc.)
     */
    bool get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd);

    /**
     * @brief Get ground course to next waypoint
     * 
     * @param[in] default_angle Angle in centidegrees to return if next leg unknown
     * @return Ground course in centidegrees (0-35999, 0=north, 9000=east)
     * 
     * @details Calculates bearing from current position to next waypoint in mission.
     * Useful for anticipating turns and planning smooth transitions.
     * Returns default_angle if next navigation leg cannot be determined.
     * 
     * @note Returns value in centidegrees (degrees * 100): 0-35999
     * @note 0 = north, 9000 = east, 18000 = south, 27000 = west
     * @note May return default_angle if: mission ending, no location in next command
     */
    int32_t get_next_ground_course_cd(int32_t default_angle);

    /**
     * @brief Get current do/conditional command
     * 
     * @return Reference to current DO or CONDITIONAL command being executed
     * 
     * @details Returns the active do or conditional command running alongside the
     * navigation command. Multiple DO commands may execute in sequence while
     * a single NAV command is active.
     * 
     * @note Returns command with index=AP_MISSION_CMD_INDEX_NONE if no DO command loaded
     * @note DO commands execute once, CONDITIONAL commands wait for conditions
     */
    const Mission_Command& get_current_do_cmd() const
    {
        return _do_cmd;
    }

    /**
     * @brief Get current do/conditional command ID
     * 
     * @return MAVLink command ID of current DO/CONDITIONAL command
     * 
     * @details Returns MAV_CMD enumeration of active do or conditional command.
     * Examples: MAV_CMD_DO_SET_SERVO (183), MAV_CMD_CONDITION_DELAY (112)
     * 
     * @note Returns AP_MISSION_CMD_ID_NONE (0) if no DO command active
     */
    uint16_t get_current_do_cmd_id() const
    {
        return _do_cmd.id;
    }

    /**
     * @brief Jump to specific mission command by index
     * 
     * @param[in] index Mission command index to jump to (0=home, 1+=mission)
     * @return true if successfully jumped to command
     * @return false if index invalid or beyond mission bounds
     * 
     * @details Changes current mission position to specified index.
     * Loads new NAV and DO commands starting from index.
     * Calls vehicle's command start callbacks for new commands.
     * Updates mission state and resets jump tracking for new position.
     * 
     * @warning Immediately changes active mission - vehicle will navigate to new target
     * @warning Mission must be in RUNNING state for jump to execute properly
     * @warning Jump counters reset - affects DO_JUMP behavior
     * 
     * @note Commonly used for: RTL landing sequences, failsafe actions, scripting control
     * @note Index 0 (home) is valid but unusual as a jump target
     * @note Does not affect mission stored in EEPROM, only runtime state
     */
    bool set_current_cmd(uint16_t index);

    /**
     * @brief Restart current navigation command
     * 
     * @return true if successfully restarted
     * @return false if current NAV command deleted or invalid
     * 
     * @details Re-initializes currently active navigation command by calling
     * vehicle's start callback again. Used when mission modified externally
     * (e.g., scripting changes waypoint location) to re-evaluate command.
     * 
     * @note Does not advance to next command - re-runs current command
     * @note Useful after replace_cmd() to activate the replacement
     * @note Reloads command from storage to get latest version
     * 
     * @warning Will fail if current command was deleted from mission
     */
    bool restart_current_nav_cmd();

    /**
     * @brief Read command from storage
     * 
     * @param[in] index Mission command index to read (0=home, 1+=mission)
     * @param[out] cmd Filled with command from storage if successful
     * @return true if successfully read
     * @return false if index invalid or storage read failed
     * 
     * @details Reads mission command from EEPROM or SD card storage.
     * Does not affect current mission execution state.
     * Performs format conversion if older storage format detected.
     * 
     * @note Index 0 always contains home position
     * @note const method - thread-safe for reading
     * @note Use get_semaphore() for atomic multi-command reads
     * 
     * @warning Index beyond num_commands() will return false
     */
    bool read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const;

    /**
     * @brief Write current home position to storage
     * 
     * @details Writes home location (mission index 0) to EEPROM/SD card.
     * Home position taken from current AHRS position estimate.
     * Called automatically during init and when home is set.
     * 
     * @note Home is mission command 0 - special reserved position
     * @note Home altitude typically set to current altitude at arming
     * @note Writes to EEPROM - limited write cycles
     * 
     * @warning Should only be called when vehicle has valid position fix
     */
    void write_home_to_storage();

    /**
     * @brief Convert MISSION_ITEM to MISSION_ITEM_INT format
     * 
     * @param[in] mission_item MAVLink MISSION_ITEM (float coordinates)
     * @param[out] mission_item_int MAVLink MISSION_ITEM_INT (integer coordinates)
     * @return MAV_MISSION_ACCEPTED on success, error code on failure
     * 
     * @details Converts legacy float-based mission item format to integer format.
     * Integer format uses 1e7 scaled lat/lon for better precision.
     * All ArduPilot code uses MISSION_ITEM_INT internally.
     * 
     * @note Static method - no instance required
     * @note MISSION_ITEM_INT preferred over MISSION_ITEM for precision
     * @note Return value must be checked (WARN_IF_UNUSED attribute)
     */
    static MAV_MISSION_RESULT convert_MISSION_ITEM_to_MISSION_ITEM_INT(const mavlink_mission_item_t &mission_item,
            mavlink_mission_item_int_t &mission_item_int) WARN_IF_UNUSED;

    /**
     * @brief Convert MISSION_ITEM_INT to MISSION_ITEM format
     * 
     * @param[in] mission_item_int MAVLink MISSION_ITEM_INT (integer coordinates)
     * @param[out] mission_item MAVLink MISSION_ITEM (float coordinates)
     * @return MAV_MISSION_ACCEPTED on success, error code on failure
     * 
     * @details Converts integer mission item format to legacy float format.
     * Used for compatibility with ground stations that don't support MISSION_ITEM_INT.
     * Precision loss possible in coordinate conversion.
     * 
     * @note Static method - no instance required
     * @note MISSION_ITEM_INT should be preferred when possible
     * @note Return value must be checked (WARN_IF_UNUSED attribute)
     * 
     * @warning May lose precision converting int32 coordinates to float
     */
    static MAV_MISSION_RESULT convert_MISSION_ITEM_INT_to_MISSION_ITEM(const mavlink_mission_item_int_t &mission_item_int,
            mavlink_mission_item_t &mission_item) WARN_IF_UNUSED;

    /**
     * @brief Convert MAVLink message to mission command
     * 
     * @param[in] packet MAVLink MISSION_ITEM_INT message from ground station
     * @param[out] cmd ArduPilot Mission_Command for storage
     * @return MAV_MISSION_ACCEPTED on success
     * @return MAV_MISSION_RESULT error code on failure
     * 
     * @details Converts MAVLink mission item protocol message to internal
     * Mission_Command format for EEPROM storage. Validates parameters,
     * checks supported command types, and performs coordinate scaling.
     * Used during mission upload from ground station.
     * 
     * @note Static method - no instance required
     * @note Handles both location-based and parameter-based commands
     * @note Performs sanity checking on coordinates and parameters
     * 
     * @warning Returns error code if: unsupported command, invalid params, NaN detected
     * @warning Some MAVLink commands not supported by ArduPilot
     */
    static MAV_MISSION_RESULT mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd);

    /**
     * @brief Convert mission command to MAVLink message
     * 
     * @param[in] cmd ArduPilot Mission_Command from storage
     * @param[out] packet MAVLink MISSION_ITEM_INT message for ground station
     * @return true on success
     * @return false on failure (invalid command structure)
     * 
     * @details Converts internal Mission_Command to MAVLink protocol format
     * for transmission to ground station. Used during mission download
     * and mission item requests.
     * 
     * @note Static method - no instance required
     * @note Always uses MISSION_ITEM_INT format (integer coordinates)
     * @note Handles coordinate descaling (internal to MAVLink format)
     */
    static bool mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet);

    /**
     * @brief Get last mission modification timestamp
     * 
     * @return Timestamp in milliseconds of last mission change
     * 
     * @details Returns system time (AP_HAL::millis()) when mission was last modified.
     * Changes include: add_cmd, replace_cmd, clear, truncate.
     * Used to detect mission updates and invalidate cached calculations.
     * 
     * @note Time wraps at ~49 days (uint32_t milliseconds)
     * @note Mission loads/reads do not update timestamp
     * @note Only modifications that could affect mission execution update timestamp
     */
    uint32_t last_change_time_ms(void) const
    {
        return _last_change_time_ms;
    }

    /**
     * @brief Find nearest landing sequence start point
     * 
     * @param[in] current_loc Vehicle's current location
     * @return Index of nearest DO_LAND_START command, or 0 if none found
     * 
     * @details Searches mission for DO_LAND_START commands and returns index
     * of the one nearest to current position. Used for emergency landing
     * and failsafe landing sequence entry. Accounts for DO_JUMP commands
     * in distance calculations.
     * 
     * @note Returns 0 (home) if no DO_LAND_START found in mission
     * @note Distance calculated as straight-line horizontal distance in meters
     * @note Does not account for altitude differences or terrain
     * 
     * @warning Does not validate that landing sequence is safe from current position
     */
    uint16_t get_landing_sequence_start(const Location &current_loc);

    /**
     * @brief Jump to nearest landing sequence
     * 
     * @param[in] current_loc Vehicle's current location
     * @return true if jumped to landing sequence
     * @return false if no DO_LAND_START available
     * 
     * @details Finds nearest DO_LAND_START command and jumps mission to that point.
     * Used for emergency landing and failsafe responses. Sets in_landing_sequence flag.
     * Immediately changes active mission - vehicle begins landing approach.
     * 
     * @note Wrapper around get_landing_sequence_start() + set_current_cmd()
     * @note Sets _flags.in_landing_sequence = true
     * 
     * @warning Vehicle immediately begins flying landing approach
     * @warning No validation that approach is safe from current position
     * @warning May result in sharp turns if far from landing sequence start
     */
    bool jump_to_landing_sequence(const Location &current_loc);

    /**
     * @brief Jump to closest landing abort sequence
     * 
     * @param[in] current_loc Vehicle's current location
     * @return true if jumped to abort landing sequence
     * @return false if no valid abort landing available
     * 
     * @details Searches for DO_LAND_START commands marked as abort sequences
     * and jumps to the nearest one. Used when primary landing fails or
     * is unsafe. Abort landing sequences are alternate landing sites.
     * 
     * @note Abort sequences are DO_LAND_START commands with specific parameter
     * @note Sets in_landing_sequence flag when jump occurs
     * 
     * @warning Limited to finding explicitly marked abort sequences
     * @warning Does not validate abort sequence is safer than continuing
     */
    bool jump_to_abort_landing_sequence(const Location &current_loc);

    /**
     * @brief Jump to landing sequence (scripting helper)
     * 
     * @return true if jumped to landing sequence
     * @return false if no landing sequence available
     * 
     * @details Scripting helper that uses current AHRS position to find
     * and jump to nearest landing sequence. Wrapper for jump_to_landing_sequence(Location).
     * Allows Lua scripts to initiate landing without providing location.
     * 
     * @note Only available when AP_SCRIPTING_ENABLED
     * @note Uses current vehicle position from AHRS
     * 
     * @warning Vehicle position must be valid (GPS fix)
     */
#if AP_SCRIPTING_ENABLED
    bool jump_to_landing_sequence(void);

    /**
     * @brief Jump to abort landing sequence (scripting helper)
     * 
     * @return true if jumped to abort sequence
     * @return false if no abort sequence available
     * 
     * @details Scripting helper that uses current AHRS position to find
     * and jump to nearest abort landing sequence. Wrapper for 
     * jump_to_abort_landing_sequence(Location).
     * 
     * @note Only available when AP_SCRIPTING_ENABLED
     * @note Uses current vehicle position from AHRS
     * 
     * @warning Vehicle position must be valid (GPS fix)
     */
    bool jump_to_abort_landing_sequence(void);
#endif

    /**
     * @brief Jump to closest point on return path
     * 
     * @param[in] current_loc Vehicle's current location
     * @return true if jumped to return path
     * @return false if no return path found or jump failed
     * 
     * @details Finds closest mission waypoint between DO_RETURN_PATH_START
     * and DO_LAND_START (or mission landing). Used to rejoin planned
     * return path after diversion. Calculates actual distance accounting
     * for DO_JUMP commands. Sets in_return_path flag.
     * 
     * @note Searches for closest waypoint by straight-line distance (m)
     * @note Only considers waypoints in return path section
     * @note Sets _flags.in_return_path = true
     * 
     * @warning May result in sharp turn if far from return path
     * @warning Does not validate rejoining path is safe
     * @warning Return path must be explicitly marked with DO_RETURN_PATH_START
     */
    bool jump_to_closest_mission_leg(const Location &current_loc);

    /**
     * @brief Check if landing sequence is shortest route home
     * 
     * @param[in] current_loc Vehicle's current location
     * @return true if jumping to landing sequence is shorter than continuing mission
     * @return false if continuing current mission is shorter route
     * 
     * @details Compares distance to land via DO_LAND_START vs continuing
     * current mission to its natural landing. Used for intelligent
     * failsafe decisions - choose fastest way home. Accounts for
     * DO_JUMP commands in distance calculations.
     * 
     * @note Distance calculations include all waypoints, not straight-line
     * @note Used by failsafe logic when FAILSAFE_TO_BEST_LANDING option set
     * @note Returns false if no landing sequence available
     * 
     * @warning Distance approximation - actual flight distance may differ
     * @warning Does not account for wind, terrain, or obstacles
     */
    bool is_best_land_sequence(const Location &current_loc);

    /**
     * @brief Set landing sequence flag
     * 
     * @param[in] flag true if in landing sequence, false otherwise
     * 
     * @details Sets internal flag indicating mission is executing a landing
     * sequence (entered via DO_LAND_START). Used by vehicle code to
     * determine if currently in landing approach vs normal mission.
     * Affects behavior of some failsafes and mode changes.
     * 
     * @note Automatically set by jump_to_landing_sequence()
     * @note Vehicle code may set manually for non-mission landings
     */
    void set_in_landing_sequence_flag(bool flag)
    {
        _flags.in_landing_sequence = flag;
    }

    /**
     * @brief Get landing sequence flag
     * 
     * @return true if mission is executing landing sequence
     * @return false if in normal mission or not in landing
     * 
     * @details Returns whether mission entered a landing sequence via
     * DO_LAND_START command. Used to modify vehicle behavior during
     * landing approaches.
     * 
     * @note Does not indicate vehicle is landing, only that mission is in landing section
     * @note Flag persists until mission stopped or reset
     */
    bool get_in_landing_sequence_flag() const {
        return _flags.in_landing_sequence;
    }

    /**
     * @brief Get return path flag
     * 
     * @return true if mission passed DO_RETURN_PATH_START
     * @return false if not in return path section
     * 
     * @details Returns whether mission has passed a DO_RETURN_PATH_START
     * waypoint, either naturally during mission progression or via
     * jump_to_closest_mission_leg() call. Indicates vehicle is on
     * planned return route.
     * 
     * @note Set automatically when mission passes DO_RETURN_PATH_START
     * @note Set by jump_to_closest_mission_leg() when rejoining return path
     */
    bool get_in_return_path_flag() const {
        return _flags.in_return_path;
    }

    /**
     * @brief Force mission resume on next start
     * 
     * @param[in] force_resume true to force resume, false to respect MIS_RESTART param
     * 
     * @details Overrides MIS_RESTART parameter for next start_or_resume() call.
     * Used by vehicle code to ensure mission resumes from last position
     * (e.g., after brief mode change). Cleared after start_or_resume() called.
     * 
     * @note Temporary override - only affects next start_or_resume()
     * @note Useful for maintaining mission progress across mode switches
     * @note Does not modify MIS_RESTART parameter value
     */
    void set_force_resume(bool force_resume)
    {
        _force_resume = force_resume;
    }

    /**
     * @brief Check if mission will resume on start
     * 
     * @return true if mission will resume from last command
     * @return false if mission will restart from beginning
     * 
     * @details Returns whether next start_or_resume() will resume from
     * last position (true) or restart from beginning (false).
     * Based on MIS_RESTART parameter and force_resume flag.
     * 
     * @note MIS_RESTART=0 means resume, MIS_RESTART=1 means restart
     * @note force_resume flag overrides parameter
     */
    bool is_resume() const { return _restart == 0 || _force_resume; }

    /**
     * @brief Get mission storage semaphore
     * 
     * @return Reference to HAL_Semaphore protecting mission storage
     * 
     * @details Provides access to semaphore for thread-safe multi-command
     * operations. Must be locked when performing atomic operations on
     * multiple mission commands (e.g., reading sequence of waypoints).
     * Prevents conflicts with mission updates from GCS or other threads.
     * 
     * Usage example:
     * @code
     * WITH_SEMAPHORE(mission.get_semaphore());
     * // Read multiple commands atomically
     * mission.read_cmd_from_storage(1, cmd1);
     * mission.read_cmd_from_storage(2, cmd2);
     * @endcode
     * 
     * @note Required for multi-command atomic operations
     * @note Single-command operations already internally protected
     * @note Always use WITH_SEMAPHORE macro for proper RAII locking
     * 
     * @warning Holding semaphore too long can block mission execution
     * @warning Never call mission update() while holding semaphore
     */
    HAL_Semaphore &get_semaphore(void)
    {
        return _rsem;
    }

    /**
     * @brief Check if mission contains specific command type
     * 
     * @param[in] command MAV_CMD to search for in mission
     * @return true if mission contains at least one instance of command
     * @return false if command not found in mission
     * 
     * @details Searches entire mission for specified MAVLink command type.
     * Used to check mission capabilities (e.g., has camera commands,
     * has landing sequence). Does not search beyond DO_JUMPs.
     * 
     * @note Searches all commands from index 0 to num_commands-1
     * @note Returns false if mission empty or command parameter invalid
     * @note Performance: O(n) search through mission
     * 
     * Example:
     * @code
     * if (mission.contains_item(MAV_CMD_DO_LAND_START)) {
     *     // Mission has landing sequence
     * }
     * @endcode
     */
    bool contains_item(MAV_CMD command) const;

    /**
     * @brief Check if mission has terrain-relative altitude items
     * 
     * @return true if any command uses terrain-relative altitude
     * @return false if all altitudes are absolute or relative to home
     * 
     * @details Searches mission for commands with terrain-relative altitude frame.
     * Result is cached and recalculated only when mission changes.
     * Used to determine if terrain database required for mission.
     * 
     * @note Result cached for performance - only recalculates on mission change
     * @note Terrain-relative frame = MAV_FRAME_GLOBAL_TERRAIN_ALT
     * @note Requires terrain database if returns true
     * 
     * @warning Mission execution may fail if terrain data unavailable
     */
    bool contains_terrain_alt_items(void);
    
    /**
     * @brief Check if command type contains location
     * 
     * @param[in] command MAV_CMD ID to check
     * @return true if command includes Location (lat/lon/alt)
     * @return false if command is parameter-only
     * 
     * @details Determines if given command type stores location data vs
     * only parameters. Used for storage format selection and waypoint
     * extraction. Location commands stored differently than parameter commands.
     * 
     * Examples:
     * - Location commands: WAYPOINT, LOITER, LAND, TAKEOFF
     * - Non-location: DO_SET_SERVO, DO_JUMP, CONDITION_DELAY
     * 
     * @note Static method - no instance required
     * @note Based on MAVLink command definition
     */
    static bool cmd_has_location(const uint16_t command);

    /**
     * @brief Reset waypoint history tracking
     * 
     * @details Clears internal history of previous waypoints. Prevents
     * resume logic from using stale waypoint history when mission
     * restarted. Called automatically on mission start/clear.
     * 
     * @note Clears _wp_index_history array
     * @note History used for mission resume and RTL path calculations
     * @note Automatically called by start() and clear()
     * 
     * @warning Only call if intentionally discarding mission progress history
     */
    void reset_wp_history(void);

    /**
     * @enum Option
     * @brief Mission behavior option flags (MIS_OPTIONS bitmask parameter)
     * 
     * @details Configuration flags that modify mission execution behavior.
     * Multiple options can be combined using bitwise OR in MIS_OPTIONS parameter.
     * 
     * @note These are bit flags in MIS_OPTIONS parameter
     * @note Default is 0 (all options disabled)
     */
    enum class Option {
        CLEAR_ON_BOOT            = (1U<<0), ///< Clear mission from EEPROM on vehicle boot (bit 0)
        FAILSAFE_TO_BEST_LANDING = (1U<<1), ///< On failsafe, choose fastest route to landing (bit 1)
        CONTINUE_AFTER_LAND      = (1U<<2), ///< Continue mission after landing if next is takeoff (bit 2)
        DONT_ZERO_COUNTER        = (1U<<3), ///< Don't reset mission counter on completion (bit 3)
    };

    /**
     * @brief Check if mission option flag is set
     * 
     * @param[in] option Option flag to check
     * @return true if option bit set in MIS_OPTIONS parameter
     * @return false if option bit clear
     * 
     * @details Tests if specific option flag is enabled in MIS_OPTIONS bitmask.
     * Options modify mission execution behavior.
     * 
     * Example:
     * @code
     * if (mission.option_is_set(AP_Mission::Option::FAILSAFE_TO_BEST_LANDING)) {
     *     // Use intelligent failsafe landing selection
     * }
     * @endcode
     */
    bool option_is_set(Option option) const {
        return (_options.get() & (uint16_t)option) != 0;
    }

    /**
     * @brief Check if next command after land is takeoff
     * 
     * @return true if next command after current landing is takeoff
     * @return false otherwise
     * 
     * @details Used with CONTINUE_AFTER_LAND option to determine if mission
     * should continue after landing. Searches forward from current position
     * for next command after land to see if it's a takeoff.
     * 
     * @note Only relevant when CONTINUE_AFTER_LAND option set
     * @note Skips DO and CONDITIONAL commands to find next NAV
     */
    bool continue_after_land_check_for_takeoff(void);

    /**
     * @brief Check if mission continues after landing
     * 
     * @return true if CONTINUE_AFTER_LAND option enabled
     * @return false if vehicle should disarm after landing
     * 
     * @details Returns whether mission should continue after landing
     * (if next command is takeoff) vs stopping and disarming.
     * Based on CONTINUE_AFTER_LAND option flag.
     * 
     * @note Default behavior is to stop mission after landing
     * @note With option set, mission continues if next command is takeoff
     * @note Useful for multi-leg missions with intermediate landings
     */
    bool continue_after_land(void) const {
        return option_is_set(Option::CONTINUE_AFTER_LAND);
    }

    /**
     * @brief Mission parameter definitions
     * 
     * @details AP_Param group info for mission parameters:
     * - MIS_TOTAL: Total number of mission commands
     * - MIS_RESTART: Mission restart behavior (0=resume, 1=restart)
     * - MIS_OPTIONS: Mission behavior option flags (bitmask)
     * 
     * @note Parameters stored in EEPROM
     * @note MIS_TOTAL automatically updated as mission modified
     * 
     * @warning Do not modify parameter names/types - breaks ground station compatibility
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get mission item in MAVLink format (Lua/scripting)
     * 
     * @param[in] index Mission command index to retrieve (0=home, 1+=mission)
     * @param[out] result MAVLink MISSION_ITEM_INT filled with command data
     * @return true if successfully retrieved
     * @return false if index invalid or read failed
     * 
     * @details Lua scripting helper to read mission commands in MAVLink format.
     * Allows scripts to inspect mission without understanding internal storage.
     * Converts internal Mission_Command to mavlink_mission_item_int_t.
     * 
     * @note Thread-safe for reading
     * @note Returns command in MISSION_ITEM_INT format (integer coordinates)
     * @note Index 0 returns home position
     * 
     * @warning Index beyond mission bounds returns false
     */
    bool get_item(uint16_t index, mavlink_mission_item_int_t& result) const ;

    /**
     * @brief Set mission item from MAVLink format (Lua/scripting)
     * 
     * @param[in] index Mission command index to write (0=home, 1+=mission)
     * @param[in] source MAVLink MISSION_ITEM_INT with command data
     * @return true if successfully written
     * @return false if index invalid, conversion failed, or write failed
     * 
     * @details Lua scripting helper to modify mission commands using MAVLink format.
     * Allows scripts to create/modify mission without understanding internal storage.
     * Converts mavlink_mission_item_int_t to internal Mission_Command and writes to storage.
     * 
     * @note Validates command before writing
     * @note Writing to index >= num_commands extends mission
     * @note Updates _last_change_time_ms on successful write
     * 
     * @warning Modifying active command requires mission restart to take effect
     * @warning Writes to EEPROM - limited write cycles
     * @warning No automatic validation of mission safety/feasibility
     */
    bool set_item(uint16_t index, mavlink_mission_item_int_t& source) ;

    /**
     * @brief Get most recent jump tag and age
     * 
     * @param[out] tag Tag number of most recent JUMP_TAG command executed
     * @param[out] age Number of NAV commands since tag (1=currently executing tag)
     * @return true if valid jump tag exists
     * @return false if no jump tag has been executed
     * 
     * @details Returns information about last JUMP_TAG command encountered.
     * Age increments each time a NAV command is executed:
     * - Age 1: Currently executing the JUMP_TAG
     * - Age 2: One NAV command after JUMP_TAG
     * - Age 3: Two NAV commands after JUMP_TAG, etc.
     * 
     * Used by scripts/vehicle code to track mission progress relative to tags.
     * 
     * @note Only most recent tag remembered (single tag tracking)
     * @note Age resets when new tag encountered
     * @note Age 0 means no tag ever seen
     * @note DO and CONDITIONAL commands don't increment age, only NAV commands
     */
    bool get_last_jump_tag(uint16_t &tag, uint16_t &age) const;

    /**
     * @brief Jump mission to first matching tag
     * 
     * @param[in] tag JUMP_TAG number to search for and jump to
     * @return true if successfully jumped to tag
     * @return false if tag not found or jump failed
     * 
     * @details Searches mission for first JUMP_TAG command with matching tag number
     * and jumps mission execution to that point. Immediately changes active mission.
     * Records tag as most recent and sets age to 1.
     * 
     * Used for mission branching based on events or conditions.
     * 
     * Example mission flow:
     * - Tag 100: Start of survey pattern
     * - Tag 200: Start of emergency procedure
     * - Script can call jump_to_tag(200) to activate emergency
     * 
     * @note Searches forward from beginning of mission
     * @note Takes first match if multiple tags with same number
     * @note Vehicle immediately begins executing from tag location
     * 
     * @warning No validation that jump destination is safe
     * @warning Mission state changes immediately
     */
    bool jump_to_tag(const uint16_t tag);

    /**
     * @brief Find index of first matching jump tag
     * 
     * @param[in] tag JUMP_TAG number to search for
     * @return Index of first matching JUMP_TAG command
     * @return 0 if tag not found
     * 
     * @details Searches mission for JUMP_TAG with matching tag number.
     * Returns mission index without jumping. Used to check if tag exists
     * before attempting jump, or to calculate distances to tags.
     * 
     * @note Returns 0 (home) if not found - check against actual mission structure
     * @note Does not modify mission state
     * @note Searches entire mission from index 0 to end
     */
    uint16_t get_index_of_jump_tag(const uint16_t tag) const;

    /**
     * @brief Validate mission index
     * 
     * @param[in] index Mission index to validate
     * @return true if index is valid (0 to num_commands-1)
     * @return false if index beyond mission bounds
     * 
     * @details Quick validation that index is within current mission.
     * Does not check if command at index is valid, only that index in range.
     * 
     * @note Index 0 (home) is always valid if mission initialized
     * @note Valid range: 0 to _cmd_total-1
     */
    bool is_valid_index(const uint16_t index) const { return index < _cmd_total; }

#if AP_SDCARD_STORAGE_ENABLED
    /**
     * @brief Check if SD card mission storage failed
     * 
     * @return true if SD card mission storage has failed
     * @return false if SD card storage working or not in use
     * 
     * @details When AP_SDCARD_STORAGE_ENABLED, missions can be stored on SD card
     * instead of EEPROM. This returns whether SD card storage has encountered errors.
     * 
     * SD card storage allows much larger missions than EEPROM.
     * Failure may indicate card removed, corrupted, or full.
     * 
     * @note Only available when AP_SDCARD_STORAGE_ENABLED compiled in
     * @note SD card file: APM/mission.stg (ChibiOS) or mission.stg (other)
     * 
     * @warning SD card failure causes mission to stop loading
     */
    bool failed_sdcard_storage(void) const {
        return _failed_sdcard_storage;
    }
#endif

#if HAL_LOGGING_ENABLED
    /**
     * @brief Set LOG_BITMASK bit for mission item logging
     * 
     * @param[in] bit Bit position in LOG_BITMASK to trigger mission logging
     * 
     * @details Configures which LOG_BITMASK bit controls whether mission items
     * are logged to dataflash when started. Allows vehicle code to integrate
     * mission logging with standard logging configuration.
     * 
     * When configured bit is set in LOG_BITMASK parameter, AP_Mission will
     * log each mission command as it begins execution.
     * 
     * @note Only available when HAL_LOGGING_ENABLED compiled in
     * @note Default is -1 (logging not tied to LOG_BITMASK)
     * @note Vehicle typically calls this during initialization
     */
    void set_log_start_mission_item_bit(uint32_t bit) { log_start_mission_item_bit = bit; }
#endif

private:
    static AP_Mission *_singleton;

    static StorageAccess _storage;

    static bool stored_in_location(uint16_t id);

    struct {
        uint16_t age;   // a value of 0 means we have never seen a tag. Once a tag is seen, age will increment every time the mission index changes.
        uint16_t tag;   // most recent tag that was successfully jumped to. Only valid if age > 0
    } _jump_tag;

    struct Mission_Flags {
        mission_state state;
        bool nav_cmd_loaded;         // true if a "navigation" command has been loaded into _nav_cmd
        bool do_cmd_loaded;          // true if a "do"/"conditional" command has been loaded into _do_cmd
        bool do_cmd_all_done;        // true if all "do"/"conditional" commands have been completed (stops unnecessary searching through eeprom for do commands)
        bool in_landing_sequence;   // true if the mission has jumped to a landing
        bool resuming_mission;      // true if the mission is resuming and set false once the aircraft attains the interrupted WP
        bool in_return_path;        // true if the mission has passed a DO_RETURN_PATH_START waypoint either in the course of the mission or via a `jump_to_closest_mission_leg` call
    } _flags;

    // mission WP resume history
    uint16_t _wp_index_history[AP_MISSION_MAX_WP_HISTORY]; // storing the nav_cmd index for the last 6 WPs

    ///
    /// private methods
    ///

    /// write_cmd_to_storage - write a command to storage
    ///     cmd.index is used to calculate the storage location
    ///     true is returned if successful
    bool write_cmd_to_storage(uint16_t index, const Mission_Command& cmd);

    /// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
    void complete();

    bool verify_command(const Mission_Command& cmd);
    bool start_command(const Mission_Command& cmd);

    /// advance_current_nav_cmd - moves current nav command forward
    //      starting_index is used to set the index from which searching will begin, leave as 0 to search from the current navigation target
    ///     do command will also be loaded
    ///     accounts for do-jump commands
    //      returns true if command is advanced, false if failed (i.e. mission completed)
    bool advance_current_nav_cmd(uint16_t starting_index = 0);

    /// advance_current_do_cmd - moves current do command forward
    ///     accounts for do-jump commands
    ///     returns true if successfully advanced (can it ever be unsuccessful?)
    void advance_current_do_cmd();

    /// get_next_cmd - gets next command found at or after start_index
    ///     returns true if found, false if not found (i.e. mission complete)
    ///     accounts for do_jump commands
    ///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
    bool get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found, bool send_gcs_msg = true);

    /// get_next_do_cmd - gets next "do" or "conditional" command after start_index
    ///     returns true if found, false if not found
    ///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
    ///     accounts for do_jump commands but never increments the jump's num_times_run (get_next_nav_cmd is responsible for this)
    bool get_next_do_cmd(uint16_t start_index, Mission_Command& cmd);

    ///
    /// jump handling methods
    ///
    // init_jump_tracking - initialise jump_tracking variables
    void init_jump_tracking();

    /// get_jump_times_run - returns number of times the jump command has been run
    ///     return is signed to be consistent with do-jump cmd's repeat count which can be -1 (to signify to repeat forever)
    int16_t get_jump_times_run(const Mission_Command& cmd);

    /// increment_jump_times_run - increments the recorded number of times the jump command has been run
    void increment_jump_times_run(Mission_Command& cmd, bool send_gcs_msg = true);

    /// check_eeprom_version - checks version of missions stored in eeprom matches this library
    /// command list will be cleared if they do not match
    void check_eeprom_version();

    // check if command is a landing type command.  Asside the obvious, MAV_CMD_DO_PARACHUTE is considered a type of landing
    bool is_landing_type_cmd(uint16_t id) const;

    // check if command is a takeoff type command.
    bool is_takeoff_type_cmd(uint16_t id) const;

    // approximate the distance travelled to get to a landing.  DO_JUMP commands are observed in look forward.
    bool distance_to_landing(uint16_t index, float &tot_distance,Location current_loc);

    // Approximate the distance traveled to return to the mission path. DO_JUMP commands are observed in look forward.
    // Stop searching once reaching a landing or do-land-start
    bool distance_to_mission_leg(uint16_t index, uint16_t &search_remaining, float &rejoin_distance, uint16_t &rejoin_index, const Location& current_loc);

    // calculate the location of a resume cmd wp
    bool calc_rewind_pos(Mission_Command& rewind_cmd);

    // update progress made in mission to store last position in the event of mission exit
    void update_exit_position(void);

    void on_mission_timestamp_change();

    /// sanity checks that the masked fields are not NaN's or infinite
    static MAV_MISSION_RESULT sanity_check_params(const mavlink_mission_item_int_t& packet);

    /// check if the next nav command is a takeoff, skipping delays
    bool is_takeoff_next(uint16_t start_index);

    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes

    // parameters
    AP_Int16                _cmd_total;  // total number of commands in the mission
    AP_Int16                _options;    // bitmask options for missions, currently for mission clearing on reboot but can be expanded as required
    AP_Int8                 _restart;   // controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)

    // internal variables
    bool                    _force_resume;  // when set true it forces mission to resume irrespective of MIS_RESTART param.
    uint16_t                _repeat_dist; // Distance to repeat on mission resume (m), can be set with MAV_CMD_DO_SET_RESUME_REPEAT_DIST
    struct Mission_Command  _nav_cmd;   // current "navigation" command.  It's position in the command list is held in _nav_cmd.index
    struct Mission_Command  _do_cmd;    // current "do" command.  It's position in the command list is held in _do_cmd.index
    struct Mission_Command  _resume_cmd;  // virtual wp command that is used to resume mission if the mission needs to be rewound on resume.
    uint16_t                _prev_nav_cmd_id;       // id of the previous "navigation" command. (WAYPOINT, LOITER_TO_ALT, ect etc)
    uint16_t                _prev_nav_cmd_index;    // index of the previous "navigation" command.  Rarely used which is why we don't store the whole command
    uint16_t                _prev_nav_cmd_wp_index; // index of the previous "navigation" command that contains a waypoint.  Rarely used which is why we don't store the whole command
    Location         _exit_position;  // the position in the mission that the mission was exited

    // jump related variables
    struct jump_tracking_struct {
        uint16_t index;                 // index of do-jump commands in mission
        int16_t num_times_run;          // number of times this jump command has been run
    } _jump_tracking[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];

    // last time that mission changed
    uint32_t _last_change_time_ms;
    uint32_t _last_change_time_prev_ms;

    // maximum number of commands that will fit in storage
    uint16_t _commands_max;

#if AP_SDCARD_STORAGE_ENABLED
    bool _failed_sdcard_storage;
#endif

    // fast call to get command ID of a mission index
    uint16_t get_command_id(uint16_t index) const;

    // memoisation of contains-relative:
    bool _contains_terrain_alt_items;  // true if the mission has terrain-relative items
    uint32_t _last_contains_relative_calculated_ms;  // will be equal to _last_change_time_ms if _contains_terrain_alt_items is up-to-date
    bool calculate_contains_terrain_alt_items(void) const;

    // multi-thread support. This is static so it can be used from
    // const functions
    static HAL_Semaphore _rsem;

    // mission items common to all vehicles:
    bool start_command_do_aux_function(const AP_Mission::Mission_Command& cmd);
    bool start_command_do_gripper(const AP_Mission::Mission_Command& cmd);
    bool start_command_do_servorelayevents(const AP_Mission::Mission_Command& cmd);
    bool start_command_camera(const AP_Mission::Mission_Command& cmd);
    bool start_command_parachute(const AP_Mission::Mission_Command& cmd);
    bool command_do_set_repeat_dist(const AP_Mission::Mission_Command& cmd);

    bool start_command_do_sprayer(const AP_Mission::Mission_Command& cmd);
    bool start_command_do_scripting(const AP_Mission::Mission_Command& cmd);
    bool start_command_do_gimbal_manager_pitchyaw(const AP_Mission::Mission_Command& cmd);
    bool start_command_fence(const AP_Mission::Mission_Command& cmd);

    /*
      handle format conversion of storage format to allow us to update
      format to take advantage of new packing
     */
    void format_conversion(uint8_t tag_byte, const Mission_Command &cmd, PackedContent &packed_content) const;

#if HAL_LOGGING_ENABLED
    // if not -1, this bit in LOG_BITMASK specifies whether to log a message each time we start a command:
    uint32_t log_start_mission_item_bit = -1;
#endif
};

namespace AP
{
AP_Mission *mission();
};
