/**
 * @file AP_Mount.h
 * @brief Mount/gimbal manager singleton for controlling 2 or 3-axis camera gimbals
 * 
 * @details This file defines the AP_Mount singleton class that manages multiple gimbal/mount
 *          backends attached to the vehicle. The mount manager supports up to AP_MOUNT_MAX_INSTANCES
 *          (typically 2) simultaneous gimbals with various control protocols.
 * 
 *          Supported mount types (backends instantiated based on MNTx_TYPE parameters):
 *          - Servo: Direct servo-controlled mounts
 *          - SoloGimbal: 3DR Solo gimbal integration
 *          - Alexmos: Basecam SimpleBGC gimbal controllers
 *          - SToRM32: SToRM32 gimbal controllers (MAVLink and serial protocols)
 *          - Gremsy: Gremsy gimbal using MAVLink v2 Gimbal protocol
 *          - Siyi: Siyi gimbal using custom serial protocol
 *          - Scripting: Lua scripting-controlled gimbals
 *          - Xacti: Xacti DroneCAN gimbal
 *          - Viewpro: Viewpro gimbal using custom serial protocol
 *          - Topotek: Topotek gimbal using custom serial protocol
 *          - CADDX: CADDX gimbal using custom serial protocol
 *          - XFRobot: XFRobot gimbal using custom serial protocol
 * 
 *          The mount manager integrates with the ArduPilot scheduler to provide:
 *          - update() called at ≥10Hz for general gimbal control
 *          - update_fast() called at INS rate for high-rate gyro integration
 * 
 *          MAVLink command handling includes:
 *          - MAV_CMD_DO_MOUNT_CONFIGURE: Configure mount operating mode
 *          - MAV_CMD_DO_MOUNT_CONTROL: Control mount orientation
 *          - MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: Gimbal manager pitch/yaw control
 *          - GIMBAL_MANAGER_SET_ATTITUDE: Set gimbal attitude as quaternion
 *          - GIMBAL_MANAGER_SET_PITCHYAW: Set gimbal pitch/yaw
 * 
 *          The manager supports:
 *          - ROI (Region of Interest) pointing at GPS locations
 *          - Target system ID following
 *          - Camera control integration (trigger, zoom, focus, recording)
 *          - Rangefinder integration for some backends
 * 
 * @note This is a singleton class accessed via AP::mount()
 * @note Backend selection is based on MNTx_TYPE parameter for each instance
 * @note Legacy parameter conversion from old MNTx_ format is performed during init()
 * @note All angles in API are in degrees unless otherwise stated (internally radians)
 * @note Roll/pitch angles are earth-frame, yaw can be earth-frame (locked) or body-frame (follow)
 * 
 * @warning Gimbal update() must be called at minimum 10Hz for proper operation
 * @warning Pre-arm checks must pass before arming (pre_arm_checks())
 * @warning Mode transitions should respect current state to avoid sudden gimbal movements
 * 
 * @see AP_Mount_Backend Base class for all mount backends
 * @see AP_Mount_Params Mount parameter definitions
 * 
 * @author Joe Holdsworth, Ritchie Wilson, Amilcar Lucas, Gregory Fletcher, Randy Mackay
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include <GCS_MAVLink/GCS_config.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Camera/AP_Camera_shareddefs.h>
#include "AP_Mount_Params.h"

// maximum number of mounts
#define AP_MOUNT_MAX_INSTANCES          2

// declare backend classes
class AP_Mount_Backend;
class AP_Mount_Servo;
class AP_Mount_SoloGimbal;
class AP_Mount_Alexmos;
class AP_Mount_SToRM32;
class AP_Mount_SToRM32_serial;
class AP_Mount_Gremsy;
class AP_Mount_Siyi;
class AP_Mount_Scripting;
class AP_Mount_Xacti;
class AP_Mount_Viewpro;
class AP_Mount_Topotek;
class AP_Mount_CADDX;
class AP_Mount_XFRobot;

/*
  This is a workaround to allow the MAVLink backend access to the
  SmallEKF. It would be nice to find a neater solution to this
 */

/**
 * @class AP_Mount
 * @brief Singleton manager for 2/3-axis camera gimbals and mounts
 * 
 * @details AP_Mount manages multiple gimbal/mount backends, providing a unified interface
 *          for controlling camera stabilization and pointing across various hardware types.
 * 
 *          Architecture:
 *          - Singleton pattern accessed via AP::mount() or get_singleton()
 *          - Supports up to AP_MOUNT_MAX_INSTANCES (2) simultaneous gimbals
 *          - Backend instantiation based on MNTx_TYPE parameters during init()
 *          - Primary instance selection based on configuration (_primary)
 * 
 *          Backend Management:
 *          - Each backend inherits from AP_Mount_Backend base class
 *          - Backend type determined by Type enum and MNTx_TYPE parameter
 *          - Backends instantiated in init() and stored in _backends[] array
 *          - Legacy parameter migration from old naming scheme performed automatically
 * 
 *          Scheduler Integration:
 *          - update() called at ≥10Hz (typically 50Hz) for general gimbal control
 *          - update_fast() called at INS rate (400Hz+) for high-rate gyro integration
 *          - Both methods iterate over all instantiated backends
 * 
 *          Control Modes (MAV_MOUNT_MODE):
 *          - RETRACTED: Gimbal in stowed position
 *          - NEUTRAL: Gimbal in neutral/home position
 *          - MAVLINK_TARGETING: Direct angle/rate control via MAVLink
 *          - RC_TARGETING: Control via RC input
 *          - GPS_POINT: Point at GPS location (ROI)
 *          - SYSID_TARGET: Track another vehicle by system ID
 *          - HOME_LOCATION: Point at home location
 * 
 *          MAVLink Command Handling:
 *          - MAV_CMD_DO_MOUNT_CONFIGURE: Set mount operating mode
 *          - MAV_CMD_DO_MOUNT_CONTROL: Control orientation (legacy)
 *          - MAV_CMD_DO_SET_ROI_SYSID: Point at vehicle system ID
 *          - MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: Modern pitch/yaw control
 *          - MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: Configure gimbal manager
 *          - GIMBAL_MANAGER_SET_ATTITUDE: Set attitude as quaternion
 *          - GIMBAL_MANAGER_SET_PITCHYAW: Set pitch/yaw angles
 *          - GIMBAL_DEVICE_INFORMATION: Device capability reporting
 *          - GIMBAL_DEVICE_ATTITUDE_STATUS: Current attitude reporting
 * 
 *          Coordinate Frames:
 *          - Roll/Pitch: Always earth-frame (relative to horizon)
 *          - Yaw: Earth-frame (locked to compass heading) or body-frame (follows vehicle)
 *          - Yaw frame controlled by yaw_is_earth_frame / yaw_lock flags
 * 
 *          Camera Integration:
 *          - Photo trigger (take_picture)
 *          - Video recording start/stop (record_video)
 *          - Zoom control (set_zoom) - rate or percentage
 *          - Focus control (set_focus) - rate, percentage, or auto
 *          - Target tracking (set_tracking) - point or rectangle
 *          - Lens selection (set_lens, set_camera_source)
 *          - Camera information/settings/status reporting via MAVLink
 * 
 *          Rangefinder Support:
 *          - Some backends provide rangefinder distance (get_rangefinder_distance)
 *          - Rangefinder enable/disable control (set_rangefinder_enable)
 * 
 *          Units and Conventions:
 *          - API angles: degrees (converted to radians internally)
 *          - API rates: deg/s (converted to rad/s internally)
 *          - Distances: meters
 *          - Timing: update() ≥10Hz minimum, update_fast() at INS rate
 * 
 * @note Singleton instance accessed via AP::mount() namespace accessor
 * @note Parameter migration from legacy MNTx_ to new format handled automatically
 * @note Instance 0 is typically primary, but _primary member defines actual primary
 * @note Some features (camera control, rangefinder) depend on backend support
 * 
 * @warning update() must be called at minimum 10Hz for stable gimbal operation
 * @warning update_fast() timing critical for gyro integration - called at INS rate
 * @warning Gimbal angle limits enforced by backend - exceeding limits may cause issues
 * @warning Mode transitions should be smooth - abrupt changes can cause gimbal instability
 * @warning Camera control compatibility varies by backend - check backend documentation
 * @warning Pre-arm checks (pre_arm_checks) must pass before vehicle arming
 * 
 * @see AP_Mount_Backend Base class defining backend interface
 * @see AP_Mount_Servo Servo-controlled mount backend
 * @see AP_Mount_SToRM32 SToRM32 gimbal backend
 * @see AP_Mount_Gremsy Gremsy gimbal backend
 * @see AP_Mount_Siyi Siyi gimbal backend
 * @see AP_Mount_Params Mount parameter definitions
 */
class AP_Mount
{
    // declare backends as friends
    friend class AP_Mount_Backend;
    friend class AP_Mount_Servo;
    friend class AP_Mount_SoloGimbal;
    friend class AP_Mount_Alexmos;
    friend class AP_Mount_SToRM32;
    friend class AP_Mount_SToRM32_serial;
    friend class AP_Mount_Gremsy;
    friend class AP_Mount_Siyi;
    friend class AP_Mount_Scripting;
    friend class AP_Mount_Xacti;
    friend class AP_Mount_Viewpro;
    friend class AP_Mount_Topotek;
    friend class AP_Mount_CADDX;

public:
    AP_Mount();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount);

    /**
     * @brief Get singleton instance of AP_Mount
     * 
     * @return Pointer to singleton AP_Mount instance, or nullptr if not instantiated
     * 
     * @note Prefer using AP::mount() namespace accessor over direct get_singleton()
     * @note Singleton created during vehicle initialization
     */
    static AP_Mount *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Mount/gimbal type enumeration
     * 
     * @details Defines all supported mount backend types. Backend selection is based
     *          on the MNTx_TYPE parameter for each instance. Each type corresponds to
     *          a specific hardware protocol and backend implementation.
     * 
     *          Backend types are conditionally compiled based on feature flags:
     *          - HAL_MOUNT_SERVO_ENABLED: Enables Servo and BrushlessPWM types
     *          - HAL_SOLO_GIMBAL_ENABLED: Enables SoloGimbal type
     *          - HAL_MOUNT_ALEXMOS_ENABLED: Enables Alexmos type
     *          - HAL_MOUNT_STORM32MAVLINK_ENABLED: Enables SToRM32 MAVLink type
     *          - HAL_MOUNT_STORM32SERIAL_ENABLED: Enables SToRM32 serial type
     *          - HAL_MOUNT_GREMSY_ENABLED: Enables Gremsy type
     *          - HAL_MOUNT_SIYI_ENABLED: Enables Siyi type
     *          - HAL_MOUNT_SCRIPTING_ENABLED: Enables Scripting type
     *          - HAL_MOUNT_XACTI_ENABLED: Enables Xacti type
     *          - HAL_MOUNT_VIEWPRO_ENABLED: Enables Viewpro type
     *          - HAL_MOUNT_TOPOTEK_ENABLED: Enables Topotek type
     *          - HAL_MOUNT_CADDX_ENABLED: Enables CADDX type
     *          - HAL_MOUNT_XFROBOT_ENABLED: Enables XFRobot type
     * 
     * @note Not all types available on all hardware platforms
     * @note Type selection via MNTx_TYPE parameter (x = instance number 1-2)
     */
    enum class Type {
        None = 0,            ///< No mount configured (disabled)
#if HAL_MOUNT_SERVO_ENABLED
        Servo = 1,           ///< Direct servo-controlled mount using PWM outputs
#endif
#if HAL_SOLO_GIMBAL_ENABLED
        SoloGimbal = 2,      ///< 3DR Solo gimbal with custom protocol
#endif
#if HAL_MOUNT_ALEXMOS_ENABLED
        Alexmos = 3,         ///< Basecam SimpleBGC (Alexmos) gimbal controllers
#endif
#if HAL_MOUNT_STORM32MAVLINK_ENABLED
        SToRM32 = 4,         ///< SToRM32 gimbal using MAVLink protocol
#endif
#if HAL_MOUNT_STORM32SERIAL_ENABLED
        SToRM32_serial = 5,  ///< SToRM32 gimbal using custom serial protocol (STorM32 native)
#endif
#if HAL_MOUNT_GREMSY_ENABLED
        Gremsy = 6,          ///< Gremsy gimbal using MAVLink v2 Gimbal Device protocol
#endif
#if HAL_MOUNT_SERVO_ENABLED
        BrushlessPWM = 7,    ///< Brushless stabilized gimbal using PWM protocol
#endif
#if HAL_MOUNT_SIYI_ENABLED
        Siyi = 8,            ///< Siyi gimbal using custom serial protocol (includes camera control)
#endif
#if HAL_MOUNT_SCRIPTING_ENABLED
        Scripting = 9,       ///< Lua scripting-controlled gimbal backend
#endif
#if HAL_MOUNT_XACTI_ENABLED
        Xacti = 10,          ///< Xacti camera/gimbal using DroneCAN protocol
#endif
#if HAL_MOUNT_VIEWPRO_ENABLED
        Viewpro = 11,        ///< Viewpro gimbal using custom serial protocol
#endif
#if HAL_MOUNT_TOPOTEK_ENABLED
        Topotek = 12,        ///< Topotek gimbal using custom serial protocol
#endif
#if HAL_MOUNT_CADDX_ENABLED
        CADDX = 13,          ///< CADDX gimbal using custom serial protocol
#endif
#if HAL_MOUNT_XFROBOT_ENABLED
        XFRobot = 14,        ///< XFRobot gimbal using custom serial protocol
#endif
    };

    /**
     * @brief Initialize and detect all configured mount instances
     * 
     * @details Performs mount backend enumeration and initialization:
     *          1. Iterates through all possible instances (up to AP_MOUNT_MAX_INSTANCES)
     *          2. Reads MNTx_TYPE parameter for each instance
     *          3. Instantiates appropriate backend based on Type value
     *          4. Performs legacy parameter migration from old MNTx_ naming scheme
     *          5. Allocates backend in _backends[] array
     *          6. Sets _num_instances count
     *          7. Determines _primary instance (typically first valid instance)
     * 
     * @note Called early in vehicle initialization with __INITFUNC__ attribute
     * @note Legacy parameters (old format) automatically converted to new format
     * @note Failed backend initialization leaves nullptr in _backends[] array
     * @note Maximum instances limited by AP_MOUNT_MAX_INSTANCES (typically 2)
     * 
     * @see convert_params() for legacy parameter migration
     */
    __INITFUNC__ void init();

    /**
     * @brief Update all mount backends - main gimbal control loop
     * 
     * @details Called periodically to update gimbal control outputs:
     *          - Iterates through all instantiated backends
     *          - Calls backend update() method for each
     *          - Handles mode-specific control logic
     *          - Updates servo outputs, serial commands, or MAVLink messages
     *          - Processes target tracking (ROI, sysid following)
     * 
     * @note Must be called at ≥10Hz for proper gimbal operation (typically 50Hz)
     * @note Called from vehicle main loop scheduler
     * @note Lower update rates may cause jerky gimbal movement
     * 
     * @warning Update rate below 10Hz can cause instability in some gimbals
     * 
     * @see update_fast() for high-rate gyro integration
     */
    void update();

    /**
     * @brief Fast update for gimbals requiring INS data at full rate
     * 
     * @details Called at INS update rate (typically 400Hz) for gimbals that need
     *          high-rate gyro data integration:
     *          - Iterates through all instantiated backends
     *          - Calls backend update_fast() if implemented
     *          - Provides gyro data for gimbal stabilization
     *          - Only used by backends that require high-rate data
     * 
     * @note Called from INS update task at full rate (400Hz+)
     * @note Not all backends implement update_fast() - only those needing high-rate data
     * @note Critical for proper stabilization in some gimbal types
     * 
     * @see update() for main gimbal control loop
     */
    void update_fast();

    /**
     * @brief Get primary mount instance ID
     * 
     * @return Primary mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @note Primary instance is the default target for single-argument methods
     * @note Typically instance 0, but can be changed via configuration
     */
    uint8_t get_primary_instance() const { return _primary; }

    /**
     * @brief Get mount type for primary instance
     * 
     * @return Type enum value indicating backend type (None, Servo, SToRM32, etc.)
     * 
     * @see Type enum for all possible mount types
     */
    Type get_mount_type() const { return get_mount_type(_primary); }
    
    /**
     * @brief Get mount type for specific instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @return Type enum value indicating backend type, or Type::None if invalid instance
     */
    Type get_mount_type(uint8_t instance) const;

    /**
     * @brief Check if primary mount has yaw/pan control
     * 
     * @return true if mount has yaw control capability, false otherwise
     * 
     * @note Required for multicopter operations where yaw control is essential
     * @note Some 2-axis gimbals only have roll/pitch, no yaw
     */
    bool has_pan_control() const { return has_pan_control(_primary); }
    
    /**
     * @brief Check if specific mount instance has yaw/pan control
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @return true if mount has yaw control capability, false if 2-axis only or invalid
     * 
     * @note Critical for copters - 2-axis mounts may not provide adequate control
     */
    bool has_pan_control(uint8_t instance) const;

    /**
     * @brief Get current operating mode of primary mount
     * 
     * @return MAV_MOUNT_MODE enum value indicating current mode
     * 
     * @note Modes include: RETRACTED, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, SYSID_TARGET, HOME_LOCATION
     * 
     * @see MAV_MOUNT_MODE enum (MAVLink standard)
     */
    enum MAV_MOUNT_MODE get_mode() const { return get_mode(_primary); }
    
    /**
     * @brief Get current operating mode of specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @return MAV_MOUNT_MODE enum value, or mode from backend if valid instance
     */
    enum MAV_MOUNT_MODE get_mode(uint8_t instance) const;

    /**
     * @brief Set operating mode for primary mount
     * 
     * @param[in] mode Desired MAV_MOUNT_MODE (RETRACTED, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, etc.)
     * 
     * @note Mode change is immediate but gimbal motion may take time
     * @note Some modes require additional setup (e.g., GPS_POINT needs set_roi_target)
     * 
     * @warning Abrupt mode changes can cause sudden gimbal movement
     * 
     * @see set_mode_to_default() to restore default mode
     */
    void set_mode(enum MAV_MOUNT_MODE mode) { return set_mode(_primary, mode); }
    
    /**
     * @brief Set operating mode for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] mode Desired MAV_MOUNT_MODE
     */
    void set_mode(uint8_t instance, enum MAV_MOUNT_MODE mode);

    /**
     * @brief Restore primary mount to default mode from MNTx_DEFLT_MODE parameter
     * 
     * @note This operation requires approximately 60μs on Pixhawk/PX4 hardware
     * @note Default mode set via MNT1_DEFLT_MODE parameter (or MNT2_DEFLT_MODE for instance 2)
     */
    void set_mode_to_default() { set_mode_to_default(_primary); }
    
    /**
     * @brief Restore specific mount instance to default mode
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     */
    void set_mode_to_default(uint8_t instance);

    /**
     * @brief Set yaw lock behavior for primary mount in RC_TARGETING mode
     * 
     * @param[in] yaw_lock true = earth-frame yaw (lock to compass heading), false = body-frame yaw (follow vehicle)
     * 
     * @details Yaw lock determines gimbal yaw behavior:
     *          - true (locked): Gimbal maintains earth-frame heading (e.g., points North even as vehicle yaws)
     *          - false (follow): Gimbal yaw rotates with vehicle body frame
     * 
     * @note Only affects RC_TARGETING mode
     * @note Earth-frame yaw useful for keeping camera pointed at fixed heading
     * @note Body-frame yaw useful for pilot control relative to vehicle
     */
    void set_yaw_lock(bool yaw_lock) { set_yaw_lock(_primary, yaw_lock); }
    
    /**
     * @brief Set yaw lock behavior for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] yaw_lock true = earth-frame, false = body-frame
     */
    void set_yaw_lock(uint8_t instance, bool yaw_lock);

    /**
     * @brief Set angle target for primary mount
     * 
     * @param[in] roll_deg Roll angle target in degrees (earth-frame, relative to horizon)
     * @param[in] pitch_deg Pitch angle target in degrees (earth-frame, relative to horizon)
     * @param[in] yaw_deg Yaw angle target in degrees (frame depends on yaw_is_earth_frame)
     * @param[in] yaw_is_earth_frame true = yaw is earth-frame (compass heading), false = yaw is body-frame
     * 
     * @details Sets desired gimbal orientation in angle mode:
     *          - Roll/Pitch: Always earth-frame (0° = level with horizon)
     *          - Yaw: Earth-frame (compass heading) if yaw_is_earth_frame=true, body-frame (relative to vehicle nose) if false
     * 
     * @note Used in MAVLINK_TARGETING mode
     * @note Gimbal will slew to target angles at configured slew rates
     * @note Angles clamped to gimbal mechanical/configured limits
     * 
     * @warning Large angle changes may cause rapid gimbal movement
     */
    void set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame) { set_angle_target(_primary, roll_deg, pitch_deg, yaw_deg, yaw_is_earth_frame); }
    
    /**
     * @brief Set angle target for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] roll_deg Roll angle in degrees (earth-frame)
     * @param[in] pitch_deg Pitch angle in degrees (earth-frame)
     * @param[in] yaw_deg Yaw angle in degrees
     * @param[in] yaw_is_earth_frame true = yaw earth-frame, false = yaw body-frame
     */
    void set_angle_target(uint8_t instance, float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame);

    /**
     * @brief Set rate target for primary mount
     * 
     * @param[in] roll_degs Roll rate in deg/s
     * @param[in] pitch_degs Pitch rate in deg/s
     * @param[in] yaw_degs Yaw rate in deg/s
     * @param[in] yaw_lock true = yaw rate is earth-frame, false = yaw rate is body-frame
     * 
     * @details Sets desired gimbal rotation rates:
     *          - Positive roll rate = roll right
     *          - Positive pitch rate = pitch up
     *          - Positive yaw rate = yaw right (if body-frame) or clockwise (if earth-frame)
     * 
     * @note Used in MAVLINK_TARGETING rate control mode
     * @note Rates continuously applied until changed or mode exits
     * @note Rate limits enforced by gimbal backend
     */
    void set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock) { set_rate_target(_primary, roll_degs, pitch_degs, yaw_degs, yaw_lock); }
    
    /**
     * @brief Set rate target for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] roll_degs Roll rate in deg/s
     * @param[in] pitch_degs Pitch rate in deg/s
     * @param[in] yaw_degs Yaw rate in deg/s
     * @param[in] yaw_lock true = earth-frame, false = body-frame
     */
    void set_rate_target(uint8_t instance, float roll_degs, float pitch_degs, float yaw_degs, bool yaw_lock);

    /**
     * @brief Set Region of Interest (ROI) target location for primary mount
     * 
     * @param[in] target_loc GPS location to point gimbal toward
     * 
     * @details Configures gimbal to continuously point at specified GPS location:
     *          - Gimbal automatically adjusts as vehicle moves
     *          - Requires valid GPS lock
     *          - Typically used in GPS_POINT mode
     * 
     * @note ROI pointing active until cleared with clear_roi_target() or mode changed
     * @note Location altitude used for proper pitch angle calculation
     */
    void set_roi_target(const Location &target_loc) { set_roi_target(_primary,target_loc); }
    
    /**
     * @brief Set ROI target location for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] target_loc GPS location to point gimbal toward
     */
    void set_roi_target(uint8_t instance, const Location &target_loc);

    /**
     * @brief Clear ROI target for primary mount
     * 
     * @note Typically called when exiting GPS_POINT mode
     * @note Gimbal returns to mode-specific default behavior
     */
    void clear_roi_target() { clear_roi_target(_primary); }
    
    /**
     * @brief Clear ROI target for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     */
    void clear_roi_target(uint8_t instance);

    /**
     * @brief Set target vehicle system ID for primary mount to track
     * 
     * @param[in] sysid MAVLink system ID of vehicle to track
     * 
     * @details Configures gimbal to point at another vehicle:
     *          - Requires receiving GLOBAL_POSITION_INT messages from target
     *          - Gimbal points at target vehicle's reported GPS position
     *          - Used in SYSID_TARGET mode
     * 
     * @note Target vehicle must be broadcasting position via MAVLink
     */
    void set_target_sysid(uint8_t sysid) { set_target_sysid(_primary, sysid); }
    
    /**
     * @brief Set target vehicle system ID for specific mount instance
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] sysid MAVLink system ID of vehicle to track
     */
    void set_target_sysid(uint8_t instance, uint8_t sysid);

    /**
     * @brief Handle MAV_CMD_DO_SET_ROI_SYSID command
     * 
     * @param[in] packet MAVLink command packet with system ID parameter
     * 
     * @return MAV_RESULT indicating success (ACCEPTED) or failure reason
     * 
     * @note Configures gimbal to track another vehicle by system ID
     */
    MAV_RESULT handle_command_do_set_roi_sysid(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAVLink commands directed at mount/gimbal
     * 
     * @param[in] packet MAVLink command packet (COMMAND_INT format)
     * @param[in] msg Full MAVLink message for context
     * 
     * @return MAV_RESULT indicating command result (ACCEPTED, DENIED, UNSUPPORTED, etc.)
     * 
     * @details Handles mount-related commands:
     *          - MAV_CMD_DO_MOUNT_CONFIGURE: Set mount operating mode
     *          - MAV_CMD_DO_MOUNT_CONTROL: Control mount orientation (legacy)
     *          - MAV_CMD_DO_SET_ROI_SYSID: Track vehicle by system ID
     *          - MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: Gimbal manager pitch/yaw control
     *          - MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: Configure gimbal manager
     * 
     * @note Routes commands to appropriate handler methods
     */
    MAV_RESULT handle_command(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
    /**
     * @brief Handle PARAM_VALUE messages for mount parameters
     * 
     * @param[in] msg MAVLink PARAM_VALUE message
     * 
     * @note Used for parameter synchronization with some gimbal backends
     * @note Primarily used by SToRM32 and similar gimbals with parameter systems
     */
    void handle_param_value(const mavlink_message_t &msg);
    
    /**
     * @brief Handle MAVLink messages directed at mount/gimbal
     * 
     * @param[in] chan MAVLink channel message received on
     * @param[in] msg MAVLink message to process
     * 
     * @details Routes messages to appropriate handlers:
     *          - GIMBAL_MANAGER_SET_ATTITUDE: Set gimbal attitude as quaternion
     *          - GIMBAL_MANAGER_SET_PITCHYAW: Set gimbal pitch/yaw angles
     *          - GIMBAL_DEVICE_INFORMATION: Receive gimbal capability information
     *          - GIMBAL_DEVICE_ATTITUDE_STATUS: Receive gimbal attitude status
     *          - GLOBAL_POSITION_INT: Track target vehicle position for SYSID_TARGET mode
     *          - GIMBAL_REPORT: Legacy gimbal status (deprecated)
     */
    void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);

    /**
     * @brief Send GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports current gimbal attitude and status:
     *          - Current attitude as quaternion
     *          - Angular velocity
     *          - Failure flags
     *          - Device ID
     * 
     * @note Sent periodically as part of telemetry stream
     * @note Complies with MAVLink Gimbal Protocol v2
     */
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    /**
     * @brief Send GIMBAL_MANAGER_INFORMATION message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports gimbal manager capabilities:
     *          - Supported control modes (angle, rate)
     *          - Roll/pitch/yaw angle limits
     *          - Manager ID and capabilities
     * 
     * @note Sent on request or during gimbal manager discovery
     * @note Defines gimbal control interface capabilities
     */
    void send_gimbal_manager_information(mavlink_channel_t chan);

    /**
     * @brief Send GIMBAL_MANAGER_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports gimbal manager status:
     *          - Current control mode
     *          - Primary and secondary control sysid/compid
     *          - Manager status flags
     * 
     * @note Sent periodically as part of telemetry stream
     * @note Indicates who has gimbal control authority
     */
    void send_gimbal_manager_status(mavlink_channel_t chan);

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    /**
     * @brief Get Point of Interest (POI) information
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] quat Current gimbal attitude as quaternion
     * @param[out] loc Current vehicle GPS location
     * @param[out] poi_loc Calculated POI GPS location based on gimbal pointing
     * 
     * @return true on success, false if POI calculation failed or instance invalid
     * 
     * @details Calculates where gimbal is pointing in GPS coordinates:
     *          - Uses current gimbal attitude and vehicle position
     *          - Projects gimbal pointing vector to ground or specified altitude
     *          - Useful for identifying what camera is viewing
     * 
     * @note Requires valid GPS lock and gimbal attitude
     * @note POI accuracy depends on altitude and gimbal angle accuracy
     */
    bool get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc) const;
#endif

    /**
     * @brief Get mount attitude as quaternion
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] att_quat Gimbal attitude quaternion
     * 
     * @return true on success, false if instance invalid or attitude unavailable
     * 
     * @details Returns earth-frame quaternion with yaw in body-frame:
     *          - Roll/pitch relative to earth (horizon)
     *          - Yaw relative to vehicle body frame
     * 
     * @note Mixed frame convention used for compatibility with control inputs
     */
    bool get_attitude_quaternion(uint8_t instance, Quaternion& att_quat);

    /**
     * @brief Get mount attitude in Euler angles
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] roll_deg Roll angle in degrees (earth-frame, relative to horizon)
     * @param[out] pitch_deg Pitch angle in degrees (earth-frame, relative to horizon)
     * @param[out] yaw_bf_deg Yaw angle in degrees (body-frame, relative to vehicle nose)
     * 
     * @return true on success, false if instance invalid or attitude unavailable
     * 
     * @note Roll: positive = right, pitch: positive = up, yaw: positive = right
     * @note Yaw is body-frame for ease of use in most applications
     */
    bool get_attitude_euler(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_bf_deg);

    /**
     * @brief Run pre-arm checks on all mount instances
     * 
     * @param[out] failure_msg Buffer to receive failure message if check fails
     * @param[in]  failure_msg_len Length of failure_msg buffer
     * 
     * @return true if all checks pass, false if any check fails
     * 
     * @details Performs safety checks before arming:
     *          - Backend initialization status
     *          - Parameter validity
     *          - Hardware communication status
     *          - Gimbal health status
     * 
     * @note Failure message does not include prefix - add "Mount: " if needed
     * @note Called during vehicle pre-arm check sequence
     * 
     * @warning Vehicle should not arm if pre-arm checks fail
     */
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    /**
     * @brief Get current rate target
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] roll_degs Roll rate target in deg/s
     * @param[out] pitch_degs Pitch rate target in deg/s
     * @param[out] yaw_degs Yaw rate target in deg/s
     * @param[out] yaw_is_earth_frame true if yaw rate is earth-frame, false if body-frame
     * 
     * @return true on success, false if instance invalid or no rate target set
     * 
     * @note Used for telemetry and scripting
     */
    bool get_rate_target(uint8_t instance, float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame);

    /**
     * @brief Get current angle target
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] roll_deg Roll angle target in degrees (earth-frame)
     * @param[out] pitch_deg Pitch angle target in degrees (earth-frame)
     * @param[out] yaw_deg Yaw angle target in degrees
     * @param[out] yaw_is_earth_frame true if yaw is earth-frame, false if body-frame
     * 
     * @return true on success, false if instance invalid or no angle target set
     * 
     * @note Used for telemetry and scripting
     */
    bool get_angle_target(uint8_t instance, float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame);

    /**
     * @brief Get current ROI target location
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] target_loc GPS location of current ROI target
     * 
     * @return true if ROI target set, false if no target or instance invalid
     * 
     * @note Used by scripting backends and logging
     */
    bool get_location_target(uint8_t instance, Location& target_loc);
    
    /**
     * @brief Set mount attitude directly in Euler angles (scripting backends)
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] roll_deg Roll angle in degrees (earth-frame)
     * @param[in] pitch_deg Pitch angle in degrees (earth-frame)
     * @param[in] yaw_bf_deg Yaw angle in degrees (body-frame)
     * 
     * @note Primarily used by scripting backends to report actual gimbal attitude
     * @note Also used for logging purposes
     */
    void set_attitude_euler(uint8_t instance, float roll_deg, float pitch_deg, float yaw_bf_deg);

    /**
     * @brief Write mount log packets for all backends
     * 
     * @details Logs mount data for all instantiated backends:
     *          - Current attitude (roll, pitch, yaw)
     *          - Target attitude
     *          - Operating mode
     *          - Timestamp
     * 
     * @note Called periodically from main logging task
     * @note Log message type: GMBL
     */
    void write_log();

    /**
     * @brief Write mount log packet for single backend with specified timestamp
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] timestamp_us Timestamp in microseconds since boot
     * 
     * @note Called by camera library to synchronize gimbal logs with camera events
     * @note Allows precise time correlation between gimbal attitude and photos
     */
    void write_log(uint8_t instance, uint64_t timestamp_us);

    //
    // Camera controls for gimbals that include integrated cameras
    //

    /**
     * @brief Trigger camera to take a picture
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @note Not all backends support camera control
     * @note Backend-specific: Siyi, Xacti, Viewpro, Topotek support photo trigger
     * 
     * @warning Check backend camera support before relying on this feature
     */
    bool take_picture(uint8_t instance);

    /**
     * @brief Start or stop video recording
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] start_recording true = start recording, false = stop recording
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @note Not all backends support video recording control
     * @note Backend-specific: Siyi, Xacti, Viewpro, Topotek support video control
     */
    bool record_video(uint8_t instance, bool start_recording);

    /**
     * @brief Set camera zoom level
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] zoom_type ZoomType enum: RATE (continuous zoom) or PERCENTAGE (absolute position)
     * @param[in] zoom_value Zoom rate (-100 to 100) or percentage (0 to 100)
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @details Zoom control types:
     *          - RATE: Continuous zoom in/out at specified rate (negative = zoom out, positive = zoom in)
     *          - PERCENTAGE: Absolute zoom position (0 = wide angle, 100 = maximum zoom)
     * 
     * @note Backend-specific camera support required (Siyi, Xacti, Viewpro, etc.)
     * @note Zoom range and behavior varies by camera model
     */
    bool set_zoom(uint8_t instance, ZoomType zoom_type, float zoom_value);

    /**
     * @brief Set camera focus
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] focus_type FocusType enum: RATE, PERCENTAGE, AUTO, AUTO_SINGLE, or CONTINUOUS
     * @param[in] focus_value Focus rate (-1 to 1) or percentage (0 to 100), ignored for auto modes
     * 
     * @return SetFocusResult enum: ACCEPTED, FAILED, or UNSUPPORTED
     * 
     * @details Focus control types:
     *          - RATE: Continuous focus adjustment (focus_value: -1 = near, 0 = hold, 1 = far)
     *          - PERCENTAGE: Absolute focus position (0 to 100)
     *          - AUTO: Automatic focus (one-shot)
     *          - AUTO_SINGLE: Single auto-focus operation
     *          - CONTINUOUS: Continuous auto-focus mode
     * 
     * @note Backend-specific camera support required
     * @note Not all cameras support all focus modes
     */
    SetFocusResult set_focus(uint8_t instance, FocusType focus_type, float focus_value);

    /**
     * @brief Set camera target tracking
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] tracking_type TrackingType enum: NONE, POINT, or RECTANGLE
     * @param[in] p1 Primary point: single point for POINT mode, top-left corner for RECTANGLE mode
     * @param[in] p2 Secondary point: unused for POINT mode, bottom-right corner for RECTANGLE mode
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @details Tracking modes:
     *          - NONE: Disable tracking
     *          - POINT: Track single point (only p1 used)
     *          - RECTANGLE: Track region (p1 = top-left, p2 = bottom-right)
     * 
     *          Point coordinates normalized to image frame:
     *          - X: 0.0 = left edge, 1.0 = right edge
     *          - Y: 0.0 = top edge, 1.0 = bottom edge
     * 
     * @note Requires backend with tracking-capable camera (e.g., Siyi with AI tracking)
     * @note Gimbal automatically adjusts to keep tracked object centered
     */
    bool set_tracking(uint8_t instance, TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2);

    /**
     * @brief Set camera lens selection
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] lens Lens number (0 to 5, meaning varies by camera)
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @note For multi-lens cameras (e.g., wide + thermal + zoom)
     * @note Lens numbering and availability is camera-specific
     * @note Some cameras use lens 0=visible, 1=thermal, 2=PIP, etc.
     */
    bool set_lens(uint8_t instance, uint8_t lens);

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    /**
     * @brief Set camera source (advanced lens selection)
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] primary_source Primary camera source (AP_Camera::CameraSource enum as uint8_t)
     * @param[in] secondary_source Secondary camera source for PIP mode
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @details Functionally similar to set_lens() but uses typed camera sources:
     *          - Allows explicit selection of visible, thermal, IR, etc.
     *          - Supports picture-in-picture (PIP) configurations
     *          - Uses AP_Camera::CameraSource enum for type safety
     * 
     * @note Backend-specific support required
     * @note Not all cameras support secondary source (PIP mode)
     */
    bool set_camera_source(uint8_t instance, uint8_t primary_source, uint8_t secondary_source);
#endif

    /**
     * @brief Send CAMERA_INFORMATION message to GCS
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports camera capabilities:
     *          - Vendor and model name
     *          - Firmware version
     *          - Sensor size and resolution
     *          - Lens focal length
     *          - Supported features (video, photo, etc.)
     * 
     * @note Sent on request via MAV_CMD_REQUEST_MESSAGE
     * @note Backend must support camera information reporting
     */
    void send_camera_information(uint8_t instance, mavlink_channel_t chan) const;

    /**
     * @brief Send CAMERA_SETTINGS message to GCS
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports current camera settings:
     *          - Operating mode (photo, video, etc.)
     *          - Zoom level
     *          - Focus mode
     * 
     * @note Sent periodically or on request
     * @note Backend must support camera settings reporting
     */
    void send_camera_settings(uint8_t instance, mavlink_channel_t chan) const;

    /**
     * @brief Send CAMERA_CAPTURE_STATUS message to GCS
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports camera capture status:
     *          - Recording state (idle, recording)
     *          - Available storage
     *          - Image count
     *          - Video time
     * 
     * @note Sent periodically during recording
     * @note Backend must support capture status reporting
     */
    void send_camera_capture_status(uint8_t instance, mavlink_channel_t chan) const;

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Send CAMERA_THERMAL_RANGE message to GCS (thermal cameras)
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Reports thermal camera temperature range:
     *          - Minimum temperature in view
     *          - Maximum temperature in view
     *          - Temperature at center point
     * 
     * @note Only supported by thermal-capable cameras
     * @note Requires backend thermal sensor support
     */
    void send_camera_thermal_range(uint8_t instance, mavlink_channel_t chan) const;
#endif

    /**
     * @brief Change camera settings not normally controlled by autopilot
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] setting CameraSetting enum value (from AP_Camera::Setting)
     * @param[in] value Setting-specific value
     * 
     * @return true if command sent successfully, false if unsupported or instance invalid
     * 
     * @details Allows control of advanced camera settings:
     *          - Exposure compensation
     *          - White balance
     *          - ISO
     *          - Aperture
     *          - Shutter speed
     *          - Image format
     *          - Video format
     * 
     * @note Setting availability depends on camera backend
     * @note Value interpretation is setting-specific
     * @note Not all cameras support all settings
     * 
     * @warning Improper settings may affect image quality
     */
    bool change_setting(uint8_t instance, CameraSetting setting, float value);

    //
    // Rangefinder integration (for backends with ranging capability)
    //

    /**
     * @brief Get rangefinder distance measurement from gimbal
     * 
     * @param[in]  instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[out] distance_m Distance measurement in meters
     * 
     * @return true if distance available, false if backend doesn't support or measurement invalid
     * 
     * @details Some gimbal/camera systems include integrated rangefinders (laser or other):
     *          - Distance measured along gimbal pointing direction
     *          - Can be used for terrain following or object avoidance
     *          - Typical range: 1-1200m depending on sensor
     * 
     * @note Backend-specific support required (e.g., some Siyi models)
     * @note Distance accuracy depends on target surface and environmental conditions
     * @note Not a replacement for dedicated rangefinder in most applications
     */
    bool get_rangefinder_distance(uint8_t instance, float& distance_m) const;

    /**
     * @brief Enable or disable gimbal-integrated rangefinder
     * 
     * @param[in] instance Mount instance number (0 to AP_MOUNT_MAX_INSTANCES-1)
     * @param[in] enable true = enable rangefinder, false = disable
     * 
     * @return true if command sent successfully, false if backend doesn't support or instance invalid
     * 
     * @note Only applicable to backends with integrated rangefinder
     * @note Some gimbals power down rangefinder when disabled to save power
     */
    bool set_rangefinder_enable(uint8_t instance, bool enable);

    /// @brief Parameter table for mount manager
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    /**
     * @brief Singleton instance pointer
     * 
     * @note Accessed via get_singleton() or AP::mount()
     */
    static AP_Mount *_singleton;

    /**
     * @brief Parameter storage for all mount instances
     * 
     * @details Each element contains MNTx_TYPE, MNTx_DEFLT_MODE, angle limits,
     *          rate limits, and backend-specific configuration parameters.
     *          Indexed by instance number (0 to AP_MOUNT_MAX_INSTANCES-1).
     */
    AP_Mount_Params _params[AP_MOUNT_MAX_INSTANCES];

    /**
     * @brief Number of mount instances successfully instantiated
     * 
     * @details Set during init() based on MNTx_TYPE parameters:
     *          - 0 if no mounts configured
     *          - 1-2 for number of active mounts (max AP_MOUNT_MAX_INSTANCES)
     * 
     * @note Used for iteration bounds in update() and other methods
     */
    uint8_t             _num_instances;
    
    /**
     * @brief Index of primary mount instance (0 to AP_MOUNT_MAX_INSTANCES-1)
     * 
     * @details Primary mount receives commands without explicit instance parameter:
     *          - set_mode() routes to set_mode(_primary, mode)
     *          - set_angle_target() routes to set_angle_target(_primary, ...)
     *          - Typically instance 0, but configurable
     * 
     * @note Used as default instance for single-argument public methods
     */
    uint8_t             _primary;
    
    /**
     * @brief Array of backend instance pointers
     * 
     * @details Backends instantiated during init() based on MNTx_TYPE parameter:
     *          - nullptr if instance not configured (Type::None)
     *          - Pointer to AP_Mount_Servo for Type::Servo
     *          - Pointer to AP_Mount_Siyi for Type::Siyi
     *          - Pointer to AP_Mount_Gremsy for Type::Gremsy
     *          - etc. for all backend types
     * 
     * @note Backends allocated with NEW_NOTHROW during init()
     * @note Backends freed in destructor
     * 
     * @see Type enum for all backend types
     */
    AP_Mount_Backend    *_backends[AP_MOUNT_MAX_INSTANCES];

private:
    /**
     * @brief Get primary backend instance (internal helper)
     * 
     * @return Pointer to primary backend, or nullptr if not instantiated
     */
    AP_Mount_Backend *get_primary() const;
    
    /**
     * @brief Get specific backend instance (internal helper)
     * 
     * @param[in] instance Instance number to retrieve
     * 
     * @return Pointer to backend, or nullptr if instance invalid or not instantiated
     */
    AP_Mount_Backend *get_instance(uint8_t instance) const;

    /**
     * @brief Handle GIMBAL_REPORT message (legacy protocol)
     * 
     * @param[in] chan MAVLink channel
     * @param[in] msg GIMBAL_REPORT message
     * 
     * @note Legacy gimbal protocol, deprecated in favor of GIMBAL_DEVICE_ATTITUDE_STATUS
     */
    void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg);

    /**
     * @brief Handle MAV_CMD_DO_MOUNT_CONFIGURE command
     * 
     * @param[in] packet Command packet with mount mode parameter
     * 
     * @return MAV_RESULT (ACCEPTED, DENIED, etc.)
     */
    MAV_RESULT handle_command_do_mount_configure(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_MOUNT_CONTROL command (legacy)
     * 
     * @param[in] packet Command packet with angle parameters
     * 
     * @return MAV_RESULT (ACCEPTED, DENIED, etc.)
     * 
     * @note Legacy mount control, prefer gimbal manager commands for new implementations
     */
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW command
     * 
     * @param[in] packet Command packet with pitch/yaw parameters
     * 
     * @return MAV_RESULT (ACCEPTED, DENIED, etc.)
     */
    MAV_RESULT handle_command_do_gimbal_manager_pitchyaw(const mavlink_command_int_t &packet);
    
    /**
     * @brief Handle MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE command
     * 
     * @param[in] packet Command packet with configuration parameters
     * @param[in] msg Full message for context
     * 
     * @return MAV_RESULT (ACCEPTED, DENIED, etc.)
     */
    MAV_RESULT handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg);
    
    /**
     * @brief Handle GIMBAL_MANAGER_SET_ATTITUDE message
     * 
     * @param[in] msg Message with quaternion attitude target
     */
    void handle_gimbal_manager_set_attitude(const mavlink_message_t &msg);
    
    /**
     * @brief Handle GIMBAL_MANAGER_SET_PITCHYAW message
     * 
     * @param[in] msg Message with pitch/yaw angle targets
     */
    void handle_gimbal_manager_set_pitchyaw(const mavlink_message_t &msg);
    
    /**
     * @brief Handle GLOBAL_POSITION_INT message for target tracking
     * 
     * @param[in] msg Position message from target vehicle (for SYSID_TARGET mode)
     */
    void handle_global_position_int(const mavlink_message_t &msg);
    
    /**
     * @brief Handle GIMBAL_DEVICE_INFORMATION message
     * 
     * @param[in] msg Message with gimbal capability information from device
     */
    void handle_gimbal_device_information(const mavlink_message_t &msg);
    
    /**
     * @brief Handle GIMBAL_DEVICE_ATTITUDE_STATUS message
     * 
     * @param[in] msg Message with gimbal attitude status from device
     */
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg);

    /**
     * @brief Convert legacy mount parameters to new format
     * 
     * @details Migrates parameters from old MNTx_ naming to new format:
     *          - Called during init() before backend instantiation
     *          - One-time conversion preserves user settings
     *          - Old parameters cleared after successful conversion
     * 
     * @note Only runs if legacy parameters detected
     */
    void convert_params();
};

/**
 * @brief AP namespace accessor functions
 */
namespace AP {
    /**
     * @brief Get AP_Mount singleton instance via namespace accessor
     * 
     * @return Pointer to AP_Mount singleton, or nullptr if not instantiated
     * 
     * @note Preferred method to access mount manager over get_singleton()
     * @note Example usage: AP::mount()->set_angle_target(0, 0, 90, false);
     */
    AP_Mount *mount();
};

#endif // HAL_MOUNT_ENABLED
