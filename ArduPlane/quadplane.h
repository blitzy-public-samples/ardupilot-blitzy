/**
 * @file quadplane.h
 * @brief QuadPlane VTOL and hybrid flight control subsystem for ArduPlane
 * 
 * @details This file defines the QuadPlane class which implements VTOL (Vertical Takeoff and Landing)
 *          and hybrid fixed-wing + multicopter flight capabilities for ArduPlane. The QuadPlane
 *          subsystem enables aircraft to combine the efficiency of fixed-wing flight with the
 *          versatility of multicopter VTOL operations.
 *          
 *          The QuadPlane class manages:
 *          - VTOL flight modes (QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO, QAUTOTUNE)
 *          - Transitions between fixed-wing and VTOL flight
 *          - Motor control for multicopter operations
 *          - Attitude and position control in VTOL modes
 *          - VTOL assist for fixed-wing flight stability
 *          - Support for multiple aircraft configurations (standard quadplane, tilt-rotor, tailsitter)
 * 
 * @note Requires Q_ENABLE parameter set to 1 to activate QuadPlane functionality
 * @note This subsystem is only compiled if HAL_QUADPLANE_ENABLED is defined
 * 
 * Source: ArduPlane/quadplane.h:1-749
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// HAL_QUADPLANE_ENABLED guards the entire QuadPlane subsystem compilation
// When disabled, this reduces firmware size for pure fixed-wing configurations
#ifndef HAL_QUADPLANE_ENABLED
#define HAL_QUADPLANE_ENABLED 1
#endif

#if HAL_QUADPLANE_ENABLED

#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h> // Attitude control library
#include <AC_AttitudeControl/AC_CommandModel.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AC_AttitudeControl/AC_WeatherVane.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AC_WPNav/AC_Loiter.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Proximity/AP_Proximity.h>
#include "qautotune.h"
#include "defines.h"
#include "tailsitter.h"
#include "tiltrotor.h"
#include "transition.h"
#include "VTOL_Assist.h"

/**
 * @class QuadPlane
 * @brief VTOL and hybrid flight control subsystem for fixed-wing aircraft with multicopter capabilities
 * 
 * @details The QuadPlane class provides comprehensive VTOL (Vertical Takeoff and Landing) functionality
 *          for ArduPlane, enabling aircraft to operate as both fixed-wing planes and multicopters.
 *          This hybrid capability combines the flight efficiency of fixed-wing with the operational
 *          flexibility of VTOL.
 * 
 * **VTOL Flight Capabilities**:
 * - **Q-Modes**: Dedicated VTOL flight modes including:
 *   - QSTABILIZE: Manual VTOL flight with attitude stabilization
 *   - QHOVER: Position hold using multicopter motors
 *   - QLOITER: GPS position hold with altitude control
 *   - QLAND: Autonomous VTOL landing
 *   - QRTL: VTOL Return-To-Launch
 *   - QACRO: Acrobatic VTOL mode with rate control
 *   - QAUTOTUNE: Automatic PID tuning for VTOL flight
 * - Vertical takeoff and landing using multicopter motors
 * - Hover and precision position hold capabilities
 * - Waypoint navigation in VTOL mode
 * - Precision landing support
 * 
 * **Transition Management** (Safety-Critical):
 * - **Forward Transition**: Smooth transition from VTOL to fixed-wing flight
 *   - Airspeed-based transition initiation
 *   - Gradual throttle transfer from VTOL motors to forward motor
 *   - Tilt-rotor servo coordination (if applicable)
 *   - Pitch attitude control during acceleration
 * - **Back Transition**: Transition from fixed-wing to VTOL mode
 *   - Deceleration to safe VTOL speed
 *   - Motor spool-up and control authority transfer
 *   - Altitude hold during transition
 * - Configurable transition timing and behavior via Q_TRANSITION_MS parameter
 * - Transition failure detection and recovery (Q_TRANS_FAIL_* parameters)
 * 
 * **Aircraft Configuration Support**:
 * - **Standard QuadPlane** (ThrustType::SLT): 
 *   Fixed multicopter motors + separate forward thrust motor
 *   Independent VTOL lift and forward flight propulsion
 * - **Tilt-Rotor** (ThrustType::TILTROTOR):
 *   Motors tilt between vertical and horizontal positions
 *   Provides both lift and forward thrust with same motors
 *   Requires tilt servo configuration
 * - **Tailsitter** (ThrustType::TAILSITTER):
 *   Entire aircraft rotates between vertical and horizontal flight
 *   Motors fixed to airframe, aircraft attitude changes for flight mode
 *   Requires special control algorithms for attitude transitions
 * - Frame class selection via Q_FRAME_CLASS (quad, hexa, octa, etc.)
 * - Frame type selection via Q_FRAME_TYPE (X, +, H, V, etc.)
 * 
 * **VTOL Assisted Flight**:
 * - **Assistance Functions**:
 *   - Angle limit enforcement to prevent stalls in fixed-wing flight
 *   - Altitude loss recovery using VTOL motors
 *   - Attitude upset and spin recovery assistance
 *   - Automatic engagement when fixed-wing flight stability threatened
 * - Configurable via Q_ASSIST_SPEED, Q_ASSIST_ANGLE, Q_ASSIST_ALT parameters
 * - Can be force-enabled or disabled via Q_OPTIONS bitmask
 * 
 * **Motor Control and Mixing**:
 * - Integration with AP_Motors library for multicopter motor control
 * - Support for various frame configurations (quad, hexa, octa, Y6, etc.)
 * - Motor output allocation and mixing
 * - ESC calibration support (Q_ESC_CAL parameter)
 * - ESC telemetry integration
 * - Tilt servo coordination for tilt-rotor configurations
 * - Motor output limiting and thrust scaling
 * 
 * **Position and Attitude Control**:
 * - **AC_PosControl**: 3D position controller for VTOL modes
 *   - XY position hold and waypoint navigation
 *   - Altitude hold with configurable climb/descend rates
 *   - Velocity control and acceleration limiting
 * - **AC_AttitudeControl_Multi**: Multicopter attitude stabilization
 *   - Rate controllers for roll, pitch, yaw
 *   - Angle controllers for attitude hold
 *   - Input shaping and rate limiting
 * - **AC_WPNav**: Waypoint navigation for VTOL autonomous modes
 * - **AC_Loiter**: Loiter position hold implementation
 * - Uses multicopter PID controllers (AC_PID) for rate control
 * 
 * **Safety-Critical Features**:
 * - Pre-arm checks for VTOL configuration validity
 * - Failsafe behavior in VTOL modes (Q_RTL_MODE, Q_OPTIONS:FS_QRTL)
 * - Motor arming/disarming interlocks
 * - Transition failure detection and recovery
 * - Land detection algorithms
 * - Throttle failsafe handling in VTOL modes
 * - Geofencing support in VTOL flight
 * 
 * **Integration with Plane**:
 * - Coordinates with main Plane class for hybrid control
 * - Shares AHRS, GPS, battery, and sensor data with fixed-wing systems
 * - Mode switching between fixed-wing and VTOL modes
 * - Parameter storage and management via AP_Param
 * - MAVLink integration for ground station communication
 * 
 * **Configuration Parameters**:
 * Key parameters controlling QuadPlane behavior:
 * - Q_ENABLE: Master enable (must be 1 to activate QuadPlane)
 * - Q_FRAME_CLASS: Motor frame configuration (quad, hexa, octa, etc.)
 * - Q_FRAME_TYPE: Motor layout (X, +, H, etc.)
 * - Q_TRANSITION_MS: Transition duration in milliseconds
 * - Q_ASSIST_SPEED: Airspeed below which VTOL assist engages
 * - Q_ASSIST_ANGLE: Attitude angle limit for assist activation
 * - Q_RTL_MODE: VTOL RTL behavior configuration
 * - Q_OPTIONS: Bitmask for various behavioral options
 * - Q_THR_MIN_PWM, Q_THR_MAX_PWM: Motor output ranges
 * 
 * @note QuadPlane requires Q_ENABLE=1 parameter to activate. All Q_* parameters are ignored if disabled.
 * @note Uses multicopter control libraries (AC_AttitudeControl, AC_PosControl, AP_Motors)
 * @note Coordinates with Plane class for hybrid fixed-wing + VTOL control
 * @note Motor configuration must match Q_FRAME_CLASS and Q_FRAME_TYPE parameters exactly
 * 
 * @warning VTOL transitions are safety-critical operations requiring careful parameter tuning
 * @warning Incorrect motor configuration can lead to loss of control during VTOL flight
 * @warning Transition parameters must be validated in SITL before flight testing
 * @warning Always test VTOL assist functions at safe altitude before relying on them
 * 
 * @see Plane Main fixed-wing flight control class
 * @see AP_Motors Multicopter motor control library
 * @see AC_AttitudeControl_Multi Multicopter attitude controller
 * @see AC_PosControl Position controller for VTOL modes
 * @see Transition Transition state machine implementation
 * @see Tailsitter Tailsitter-specific control logic
 * @see Tiltrotor Tilt-rotor mechanism control
 * @see VTOL_Assist VTOL assistance system for fixed-wing flight
 */
class QuadPlane
{
public:
    friend class Plane;
    friend class AP_Tuning_Plane;
    friend class GCS_MAVLINK_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class QAutoTune;
    friend class AP_Arming_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channel;
    friend class Tailsitter;
    friend class Tiltrotor;
    friend class SLT_Transition;
    friend class Tailsitter_Transition;
    friend class VTOL_Assist;

    friend class Mode;
    friend class ModeManual;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeQHover;
    friend class ModeQLand;
    friend class ModeQLoiter;
    friend class ModeQRTL;
    friend class ModeQStabilize;
    friend class ModeQAutotune;
    friend class ModeQAcro;
    friend class ModeLoiterAltQLand;
    friend class ModeAutoLand;
    friend class AP_SystemID;

    QuadPlane(AP_AHRS &_ahrs);

    static QuadPlane *get_singleton() {
        return _singleton;
    }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info2[];

    /**
     * @brief Run AUTO mode flight control for QuadPlane
     * 
     * @details Handles waypoint navigation and mission commands in AUTO mode when
     *          QuadPlane is active, including VTOL takeoffs, landings, and transitions.
     */
    void control_auto(void);
    
    /**
     * @brief Initialize QuadPlane subsystem, motors, and controllers
     * 
     * @details Performs one-time initialization of the QuadPlane subsystem including:
     *          - Motor library instantiation based on Q_FRAME_CLASS
     *          - Attitude controller setup (AC_AttitudeControl_Multi)
     *          - Position controller setup (AC_PosControl)
     *          - Waypoint navigation setup (AC_WPNav)
     *          - Tailsitter/tiltrotor initialization if applicable
     *          - Parameter validation and default setup
     * 
     * @return true if initialization successful, false if QuadPlane cannot be activated
     * 
     * @note Called during Plane::setup() if Q_ENABLE=1
     * @note Failure typically indicates invalid Q_FRAME_CLASS or missing parameters
     */
    bool setup(void);

    /**
     * @enum ThrustType
     * @brief QuadPlane aircraft configuration types defining motor thrust orientation
     * 
     * @details Defines the fundamental mechanical configuration of the QuadPlane aircraft,
     *          determining how motors provide lift and forward thrust. This affects
     *          transition behavior, control algorithms, and motor mixing.
     */
    enum class ThrustType : uint8_t {
        SLT=0,        ///< Standard Lift + Thrust: Traditional quadplane with separate vertical lift motors and forward pusher/puller motor
        TAILSITTER,   ///< Tailsitter: Aircraft sits on tail for takeoff, entire vehicle rotates between vertical and horizontal flight
        TILTROTOR,    ///< Tilt-Rotor: Motors tilt from vertical to horizontal orientation, providing both lift and forward thrust
    };
    
    /**
     * @brief Get the current thrust type configuration
     * @return Current ThrustType (SLT, TAILSITTER, or TILTROTOR)
     */
    ThrustType get_thrust_type(void) {return thrust_type;}

    void vtol_position_controller(void);
    void setup_target_position(void);
    void takeoff_controller(void);
    void waypoint_controller(void);
    void update_land_positioning(void);

    void update_throttle_mix(void);
    
    /**
     * @brief Main QuadPlane update called from scheduler
     * 
     * @details Primary update function called at regular intervals from the main scheduler.
     *          Handles transition state machine updates, VTOL assist logic, motor outputs,
     *          and coordination between fixed-wing and VTOL control systems.
     * 
     * @note Called at main loop rate (typically 400Hz) from Plane::update()
     * @note Timing-critical function - must complete within scheduler time budget
     */
    void update(void);

    /**
     * @brief Set motor arming state for VTOL motors
     * 
     * @details Arms or disarms the multicopter motors, enabling or preventing motor output.
     *          Coordinates with main Plane arming state and applies QuadPlane-specific
     *          arming interlocks and safety checks.
     * 
     * @param[in] armed true to arm motors, false to disarm
     * 
     * @note Does not arm if pre-arm checks fail
     * @note Respects Q_OPTIONS:DELAY_ARMING if configured
     */
    void set_armed(bool armed);

    /**
     * @brief Check if VTOL subsystem is available and initialized
     * 
     * @return true if QuadPlane initialized successfully and ready for use
     * 
     * @note Returns false if Q_ENABLE=0 or setup() failed
     */
    bool available(void) const {
        return initialised;
    }

    /**
     * @brief Check if VTOL motors are currently assisting fixed-wing flight
     * 
     * @details VTOL assist engages multicopter motors during fixed-wing flight when:
     *          - Airspeed drops below Q_ASSIST_SPEED threshold
     *          - Attitude error exceeds Q_ASSIST_ANGLE limit
     *          - Altitude loss exceeds configured threshold
     * 
     * @return true if VTOL assist is currently active
     * 
     * @see VTOL_Assist for assist logic implementation
     */
    bool in_assisted_flight(void) const {
        return available() && assisted_flight;
    }

    // abort landing, only valid when in a VTOL landing descent
    bool abort_landing(void);

    bool in_frwd_transition(void) const;

    bool handle_do_vtol_transition(enum MAV_VTOL_STATE state) const;

    bool do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);
    bool do_vtol_land(const AP_Mission::Mission_Command& cmd);
    bool verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd);
    bool verify_vtol_land(void);
    /**
     * @brief Check if currently in AUTO mode with VTOL operations
     * @return true if in AUTO mode executing VTOL mission items
     */
    bool in_vtol_auto(void) const;
    
    /**
     * @brief Check if currently in a VTOL flight mode
     * 
     * @details Returns true for Q-modes: QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO, QAUTOTUNE
     * 
     * @return true if in any VTOL mode (Q-mode)
     * @return false if in fixed-wing mode or transitioning
     */
    bool in_vtol_mode(void) const;
    
    /**
     * @brief Check if executing VTOL takeoff
     * @return true if performing vertical takeoff using VTOL motors
     */
    bool in_vtol_takeoff(void) const;
    
    /**
     * @brief Check if in a VTOL mode requiring position/velocity control
     * 
     * @details Returns true for modes that use AC_PosControl (QHOVER, QLOITER, QLAND, etc.)
     * 
     * @return true if position/velocity controller should be active
     */
    bool in_vtol_posvel_mode(void) const;
    void update_throttle_hover();
    bool show_vtol_view() const;

    // vtol help for is_flying()
    bool is_flying(void);

    // return desired forward throttle percentage
    float forward_throttle_pct();
    float get_weathervane_yaw_rate_cds(void);

    // see if we are flying from vtol point of view
    bool is_flying_vtol(void) const;

    // user initiated takeoff for guided mode
    bool do_user_takeoff(float takeoff_altitude);

    // return true if the wp_nav controller is being updated
    bool using_wp_nav(void) const;

    // return true if the user has set ENABLE
    bool enabled(void) const { return enable != 0; }
    
    // is throttle controlled landing descent active?
    bool thr_ctrl_land;

    uint16_t get_pilot_velocity_z_max_dn() const;
    
    /**
     * @struct log_QControl_Tuning
     * @brief Binary log message structure for QuadPlane control tuning data (QTUN)
     * 
     * @details Records key VTOL control parameters for analysis and tuning.
     *          Logged periodically during VTOL operations to track controller
     *          performance and diagnose issues.
     * 
     * @note Log message name: QTUN
     * @note Logging rate typically 25Hz during VTOL flight
     */
    struct PACKED log_QControl_Tuning {
        LOG_PACKET_HEADER;
        uint64_t time_us;               ///< Timestamp in microseconds
        float    throttle_in;           ///< Input throttle (0.0 to 1.0)
        float    angle_boost;           ///< Angle boost throttle compensation
        float    throttle_out;          ///< Output throttle to motors (0.0 to 1.0)
        float    throttle_hover;        ///< Hover throttle estimate (0.0 to 1.0)
        float    desired_alt;           ///< Desired altitude in meters
        float    inav_alt;              ///< Inertial navigation altitude estimate in meters
        int32_t  baro_alt;              ///< Barometric altitude in centimeters
        int16_t  target_climb_rate;     ///< Target climb rate in cm/s
        int16_t  climb_rate;            ///< Actual climb rate in cm/s
        float    throttle_mix;          ///< Throttle mix value for attitude/altitude blending
        uint8_t  transition_state;      ///< Current transition state
        uint8_t  assist;                ///< VTOL assist active flag
    };

    MAV_TYPE get_mav_type(void) const;

    // called when we change mode (for any mode, not just Q modes)
    void mode_enter(void);

    // Check if servo auto trim is allowed
    bool allow_servo_auto_trim();

    /*
      are we in the descent phase of a VTOL landing?
     */
    bool in_vtol_land_descent(void) const;

    // Should we allow stick mixing from the pilot
    bool allow_stick_mixing() const;

    /*
      should we disable the TECS controller?
      only called when in an auto-throttle mode
     */
    bool should_disable_TECS() const;

    // Get pilot throttle input with deadzone, this will return 50% throttle in failsafe!
    float get_throttle_input() const;

    void Log_Write_AttRate();

private:
    AP_AHRS &ahrs;

    // key aircraft parameters passed to multiple libraries
    AP_MultiCopter aparm;

    AP_InertialNav inertial_nav{ahrs};

    AP_Enum<AP_Motors::motor_frame_class> frame_class;
    AP_Enum<AP_Motors::motor_frame_type> frame_type;

    // Types of different "quadplane" configurations.
    ThrustType thrust_type;

    /**
     * @brief AP_Motors instance for multicopter motor control and mixing
     * 
     * @details Dynamically allocated pointer to motor controller, type determined by Q_FRAME_CLASS:
     *          - AP_MotorsMatrix for standard multicopter frames (quad, hexa, octa)
     *          - AP_MotorsTri for tricopter with tail servo
     *          - AP_MotorsSingle for single rotor helicopters
     *          - AP_MotorsCoax for coaxial helicopters
     * 
     * @note Allocated during setup() based on frame class
     * @note nullptr if QuadPlane not initialized or Q_ENABLE=0
     */
    AP_MotorsMulticopter *motors = nullptr;
    
    /// Parameter group info for motors configuration
    const struct AP_Param::GroupInfo *motors_var_info;

    /**
     * @brief Multicopter attitude controller for VTOL stabilization
     * 
     * @details Provides rate and angle control for VTOL flight modes:
     *          - Rate controllers (roll_rate, pitch_rate, yaw_rate)
     *          - Angle controllers (roll_angle, pitch_angle)
     *          - Input shaping and feedforward
     *          - PID loop management
     * 
     * @note Dynamically allocated during setup()
     * @see AC_AttitudeControl_Multi for implementation
     */
    AC_AttitudeControl_Multi *attitude_control;
    
    /**
     * @brief 3D position controller for VTOL position hold and navigation
     * 
     * @details Implements position and velocity control for VTOL modes:
     *          - XY position hold (lateral positioning)
     *          - Z position/altitude control
     *          - Velocity control in all axes
     *          - Acceleration limiting
     *          - Jerk limiting for smooth motion
     * 
     * @note Used by QHOVER, QLOITER, QLAND, and waypoint navigation
     * @see AC_PosControl for implementation
     */
    AC_PosControl *pos_control;
    
    /**
     * @brief Waypoint navigation controller for AUTO mode VTOL operations
     * @see AC_WPNav for implementation
     */
    AC_WPNav *wp_nav;
    
    /**
     * @brief Loiter position hold implementation
     * @see AC_Loiter for implementation
     */
    AC_Loiter *loiter_nav;
    
    // maximum vertical velocity the pilot may request
    AP_Float pilot_speed_z_max_up;
    AP_Float pilot_speed_z_max_dn;

    // vertical acceleration the pilot may request
    AP_Float pilot_accel_z;

    // air mode state: OFF, ON, ASSISTED_FLIGHT_ONLY
    AirMode air_mode;

    // Command model parameter class
    // Default max rate, default expo, default time constant
    AC_CommandModel command_model_pilot{100.0, 0.25, 0.25};
    // helper functions to set and disable time constant from command model
    void set_pilot_yaw_rate_time_constant();
    void disable_yaw_rate_time_constant();

    // return true if airmode should be active
    bool air_mode_active() const;

    // check for an EKF yaw reset
    void check_yaw_reset(void);
    
    // hold hover (for transition)
    void hold_hover(float target_climb_rate_cms);

    // hold stabilize (for transition)
    void hold_stabilize(float throttle_in);

    // set climb rate in position controller
    void set_climb_rate_cms(float target_climb_rate_cms);

    // get pilot desired yaw rate in cd/s
    float get_pilot_input_yaw_rate_cds(void) const;

    // get overall desired yaw rate in cd/s
    float get_desired_yaw_rate_cds(bool weathervane=true);
    
    // get desired climb rate in cm/s
    float get_pilot_desired_climb_rate_cms(void) const;

    // get pilot lean angle
    void get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const;

    // get pilot throttle in for landing code. Return value on scale of 0 to 1
    float get_pilot_land_throttle(void) const;

    /**
     * @brief Initialize throttle wait state for safe motor startup
     * 
     * @details Sets the throttle_wait flag which prevents motors from spinning up until
     *          the pilot moves the throttle stick to a safe position. This safety feature
     *          prevents unexpected motor startup when switching into VTOL modes.
     * 
     * @note Called when entering VTOL modes that require pilot throttle control
     * @note Motor output is suppressed until throttle stick moved to low position
     */
    void init_throttle_wait();

    // use multicopter rate controller
    void multicopter_attitude_rate_update(float yaw_rate_cds);

    float get_pilot_throttle(void);
    void control_hover(void);
    void relax_attitude_control();

    bool check_land_complete(void);
    bool land_detector(uint32_t timeout_ms);
    bool check_land_final(void);

    float assist_climb_rate_cms(void) const;

    // calculate desired yaw rate for assistance
    float desired_auto_yaw_rate_cds(bool body_frame = false) const;

    bool should_relax(void);
    void motors_output(bool run_rate_controller = true);
    void Log_Write_QControl_Tuning();
    void log_QPOS(void);
    float landing_descent_rate_cms(float height_above_ground);
    
    // setup correct aux channels for frame class
    void setup_default_channels(uint8_t num_motors);

    void guided_start(void);
    void guided_update(void);

    void update_throttle_suppression(void);

    void run_z_controller(void);
    void run_xy_controller(float accel_limit=0.0);

    void setup_defaults(void);

    // calculate a stopping distance for fixed-wing to vtol transitions
    float stopping_distance(float ground_speed_squared) const;
    float accel_needed(float stop_distance, float ground_speed_squared) const;
    float stopping_distance(void);

    // distance below which we don't do approach, based on stopping
    // distance for cruise speed
    float transition_threshold(void);

    AP_Int16 transition_time_ms;
    AP_Int16 back_trans_pitch_limit_ms;

    // transition deceleration, m/s/s
    AP_Float transition_decel;

    // transition failure handling
    struct TRANS_FAIL {
        enum ACTION {
            QLAND,
            QRTL
        };
        AP_Int16 timeout;
        AP_Enum<ACTION> action;
        bool warned;
    } transition_failure;


    // Quadplane trim, degrees
    AP_Float ahrs_trim_pitch;
    float _last_ahrs_trim_pitch;

    // fw landing approach radius
    AP_Float fw_land_approach_radius;

    AP_Int16 rc_speed;

    /**
     * @brief VTOL assistance system for fixed-wing flight stability
     * 
     * @details Manages automatic engagement of VTOL motors to assist fixed-wing flight when:
     *          - Airspeed drops below safe threshold (Q_ASSIST_SPEED)
     *          - Attitude error exceeds limits (Q_ASSIST_ANGLE)
     *          - Excessive altitude loss detected
     *          - Spin recovery needed
     * 
     * @see VTOL_Assist for implementation details
     * @see in_assisted_flight() to check if assist currently active
     */
    VTOL_Assist assist {*this};

    // landing speed in m/s
    AP_Float land_final_speed;

    // QRTL start altitude, meters
    AP_Int16 qrtl_alt;
    AP_Int16 qrtl_alt_min;
    
    // alt to switch to QLAND_FINAL
    AP_Float land_final_alt;
    AP_Float vel_forward_alt_cutoff;
    
    AP_Int8 enable;
    AP_Int8 transition_pitch_max;

    /**
     * @brief Q_RTL_MODE parameter: Controls when VTOL RTL is used instead of fixed-wing RTL
     * 
     * @details Determines under what conditions Return-To-Launch uses VTOL mode (QRTL)
     *          instead of fixed-wing approach and landing.
     */
    AP_Int8 rtl_mode;
    
    /**
     * @enum RTL_MODE
     * @brief Options for Q_RTL_MODE parameter controlling VTOL vs fixed-wing RTL behavior
     */
    enum RTL_MODE{
        NONE,                   ///< Always use fixed-wing RTL, never switch to QRTL
        SWITCH_QRTL,            ///< Switch to QRTL when close to home (within Q_RTL_MODE distance)
        VTOL_APPROACH_QRTL,     ///< Use fixed-wing approach, then QRTL for final landing
        QRTL_ALWAYS,            ///< Always use QRTL (pure VTOL return and landing)
    };

    // control if a VTOL GUIDED will be used
    AP_Int8 guided_mode;

    // control ESC throttle calibration
    AP_Int8 esc_calibration;
    void run_esc_calibration(void);

    // ICEngine control on landing
    AP_Int8 land_icengine_cut;

    // HEARTBEAT mav_type override
    AP_Int8 mav_type;

    // manual throttle curve expo strength
    AP_Float throttle_expo;

    // manual forward throttle input
    AP_Float fwd_thr_max;
    RC_Channel *rc_fwd_thr_ch;

    // QACRO mode max roll/pitch/yaw rates
    AP_Float acro_roll_rate;
    AP_Float acro_pitch_rate;
    AP_Float acro_yaw_rate;

    // gain from forward acceleration to forward throttle
    AP_Float q_fwd_thr_gain;

    // limit applied to forward pitch to prevent wing producing negative lift
    AP_Float q_fwd_pitch_lim;

    // limit applied to back pitch to prevent wing producing excessive lift
    AP_Float q_bck_pitch_lim;

    /**
     * @enum ActiveFwdThr
     * @brief Active forward throttle handling method for VTOL modes
     * 
     * @details Determines which algorithm is used for forward throttle control
     *          when using forward motor assistance in VTOL modes.
     */
    enum class ActiveFwdThr : uint8_t {
        NONE = 0,   ///< No forward throttle in VTOL mode
        OLD  = 1,   ///< Legacy forward throttle algorithm
        NEW  = 2,   ///< New forward throttle algorithm with improved control
    };
    
    /**
     * @brief Auxiliary function override for forward throttle enable
     * 
     * @details Can be toggled via RC auxiliary function to enable/disable
     *          forward throttle in VTOL modes dynamically.
     */
    bool vfwd_enable_active;
    
    /**
     * @enum FwdThrUse
     * @brief Q_FWD_THR_USE parameter: When to use forward throttle in VTOL modes
     * 
     * @details Controls when forward throttle (from pusher/puller motor) is used
     *          during VTOL flight, controlled by Q_FWD_THR_GAIN parameter.
     */
    enum class FwdThrUse : uint8_t {
        OFF     = 0,    ///< Forward throttle disabled in VTOL modes
        POSCTRL = 1,    ///< Use forward throttle only in position control modes (QHOVER, QLOITER)
        ALL     = 2,    ///< Use forward throttle in all VTOL modes
    };
    
    /// Storage for Q_FWD_THR_USE parameter value
    AP_Enum<FwdThrUse> q_fwd_thr_use;

    // return which vfwd method to use
    ActiveFwdThr get_vfwd_method(void) const;

    // time we last got an EKF yaw reset
    uint32_t ekfYawReset_ms;

    struct {
        AP_Float gain;
        float integrator;
        uint32_t last_ms;
        float last_pct;
    } vel_forward;

    AC_WeatherVane *weathervane;

    bool initialised;

    Location last_auto_target;

    float q_fwd_throttle; // forward throttle used in q modes
    float q_fwd_pitch_lim_cd; // forward pitch limit applied when using q_fwd_throttle
    float q_bck_pitch_lim_cd; // backward pitch limit applied when using Q_BCK_PIT_LIM
    uint32_t q_pitch_limit_update_ms; // last time the backward pitch limit was updated

    // when did we last run the attitude controller?
    uint32_t last_att_control_ms;

    /**
     * @brief Forward/back transition state machine controller
     * 
     * @details Manages the transition between fixed-wing and VTOL flight modes.
     *          Implements state machine for:
     *          - Forward transition: VTOL → Fixed-wing
     *          - Back transition: Fixed-wing → VTOL
     *          - Transition timing and airspeed monitoring
     *          - Motor and control surface blending during transitions
     *          - Transition failure detection
     * 
     * @note Dynamically allocated as either SLT_Transition or Tailsitter_Transition
     *       based on thrust_type configuration
     * 
     * @see Transition for base transition logic
     * @see SLT_Transition for standard quadplane transitions
     * @see Tailsitter_Transition for tailsitter-specific transitions
     */
    Transition *transition = nullptr;

    // true when waiting for pilot throttle
    bool throttle_wait;

    // true when quad is assisting a fixed wing mode
    bool assisted_flight;

    // are we in a guided takeoff?
    bool guided_takeoff;

    /* if we arm in guided mode when we arm then go into a "waiting
       for takeoff command" state. In this state we are waiting for
       one of the following:

       1) disarm
       2) guided takeoff command
       3) change to AUTO with a takeoff waypoint as first nav waypoint
       4) change to another mode

       while in this state we don't go to throttle unlimited, and will
       refuse a change to AUTO mode if the first waypoint is not a
       takeoff. If we try to switch to RTL then we will instead use
       QLAND

       This state is needed to cope with the takeoff sequence used
       by QGC on common controllers such as the MX16, which do this on a "takeoff" swipe:

          - changes mode to GUIDED
          - arms
          - changes mode to AUTO
    */
    bool guided_wait_takeoff;
    bool guided_wait_takeoff_on_mode_enter;

    struct {
        // time when motors reached lower limit
        uint32_t lower_limit_start_ms;
        uint32_t land_start_ms;
        float vpos_start_m;

        // landing detection threshold in meters
        AP_Float detect_alt_change;
    } landing_detect;

    // throttle mix acceleration filter
    LowPassFilterVector3f throttle_mix_accel_ef_filter{1.0};

    // time we last set the loiter target
    uint32_t last_loiter_ms;

    /**
     * @enum position_control_state
     * @brief State machine phases for VTOL landing sequence
     * 
     * @details Defines the progression of states during a VTOL landing operation.
     *          Each state represents a distinct phase with specific control objectives.
     */
    enum position_control_state {
        QPOS_NONE = 0,          ///< Not in position control (normal flight)
        QPOS_APPROACH,          ///< Approach phase: Navigate to landing position, decelerate to VTOL speed
        QPOS_AIRBRAKE,          ///< Airbrake phase: Rapid deceleration using VTOL drag and back-transition
        QPOS_POSITION1,         ///< Position hold phase 1: Stabilize at landing position, prepare for descent
        QPOS_POSITION2,         ///< Position hold phase 2: Final position refinement before descent
        QPOS_LAND_DESCEND,      ///< Landing descent: Controlled descent to ground with position hold
        QPOS_LAND_ABORT,        ///< Landing abort: Abort landing and climb to safe altitude
        QPOS_LAND_FINAL,        ///< Final landing phase: Ground contact detection and motor disarm
        QPOS_LAND_COMPLETE      ///< Landing complete: On ground, motors disarmed
    };
    class PosControlState {
    public:
        enum position_control_state get_state() const {
            return state;
        }
        void set_state(enum position_control_state s);
        uint32_t time_since_state_start_ms() const {
            return AP_HAL::millis() - last_state_change_ms;
        }
        Vector3p target_cm;
        Vector2f xy_correction;
        Vector3f target_vel_cms;
        bool slow_descent:1;
        bool pilot_correction_active;
        bool pilot_correction_done;
        uint32_t thrust_loss_start_ms;
        uint32_t last_log_ms;
        bool reached_wp_speed;
        uint32_t last_run_ms;
        float pos1_speed_limit;
        bool done_accel_init;
        Vector2f velocity_match;
        uint32_t last_velocity_match_ms;
        float target_speed;
        float target_accel;
        uint32_t last_pos_reset_ms;
        bool overshoot;

        float override_descent_rate;
        uint32_t last_override_descent_ms;
    private:
        uint32_t last_state_change_ms;
        enum position_control_state state;
    } poscontrol;

    struct {
        bool running;
        uint32_t start_ms;            // system time the motor test began
        uint32_t timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
        uint8_t seq = 0;              // motor sequence number of motor being tested
        uint8_t throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
        uint16_t throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type
        uint8_t motor_count;          // number of motors to cycle
    } motor_test;

    // time of last MOTB log message
    uint32_t last_motb_log_ms;

    // time of last QTUN log message
    uint32_t last_qtun_log_ms;

    /**
     * @brief Tilt-rotor mechanism control subsystem
     * 
     * @details Manages tilt servo control for tilt-rotor QuadPlane configurations.
     *          Controls the transition of motor orientation from vertical (VTOL)
     *          to horizontal (forward flight) positions.
     * 
     * @note Only active when thrust_type == ThrustType::TILTROTOR
     * @see Tiltrotor for tilt mechanism implementation
     */
    Tiltrotor tiltrotor{*this, motors};

    /**
     * @brief Tailsitter-specific configuration and control
     * 
     * @details Manages tailsitter aircraft behavior including:
     *          - Transition between vertical and horizontal flight attitudes
     *          - Control surface mixing for both orientations
     *          - Hover attitude control
     *          - Transition gain scheduling
     * 
     * @note Only active when thrust_type == ThrustType::TAILSITTER
     * @see Tailsitter for tailsitter-specific control logic
     */
    Tailsitter tailsitter{*this, motors};

    // the attitude view of the VTOL attitude controller
    AP_AHRS_View *ahrs_view;

    // time when motors were last active
    uint32_t last_motors_active_ms;

    // time when we last ran the vertical accel controller
    uint32_t last_pidz_active_ms;
    uint32_t last_pidz_init_ms;

    // throttle scailing for vectored motors in FW flighy
    float FW_vector_throttle_scaling(void);

    void afs_terminate(void);
    bool guided_mode_enabled(void);

    // set altitude target to current altitude
    void set_alt_target_current(void);

    /**
     * @brief Q_OPTIONS parameter bitmask value storage
     * 
     * @details Stores the Q_OPTIONS parameter which controls various QuadPlane behavioral options
     *          through a bitmask. Each bit enables/disables a specific feature or behavior.
     */
    AP_Int32 options;
    
    /**
     * @enum OPTION
     * @brief Bitmask options for Q_OPTIONS parameter controlling QuadPlane behavior
     * 
     * @details Each enum value represents a bit in the Q_OPTIONS parameter bitmask.
     *          Multiple options can be combined by setting multiple bits.
     *          Use option_is_set() to check if a specific option is enabled.
     */
    enum class OPTION {
        LEVEL_TRANSITION=(1<<0),                    ///< Force level attitude during transitions instead of climbing attitude
        ALLOW_FW_TAKEOFF=(1<<1),                    ///< Allow fixed-wing takeoff instead of requiring VTOL takeoff
        ALLOW_FW_LAND=(1<<2),                       ///< Allow fixed-wing landing instead of requiring VTOL landing
        RESPECT_TAKEOFF_FRAME=(1<<3),               ///< Respect takeoff command frame type (MAVLink DO_VTOL_TRANSITION)
        MISSION_LAND_FW_APPROACH=(1<<4),            ///< Use fixed-wing approach for mission landings
        FS_QRTL=(1<<5),                             ///< Use QRTL instead of RTL for failsafe
        IDLE_GOV_MANUAL=(1<<6),                     ///< Use manual throttle governor in VTOL idle
        Q_ASSIST_FORCE_ENABLE=(1<<7),               ///< Force-enable VTOL assist (always active)
        TAILSIT_Q_ASSIST_MOTORS_ONLY=(1<<8),        ///< For tailsitters, only use motors for Q_ASSIST (not control surfaces)
        AIRMODE_UNUSED=(1<<9),                      ///< Unused bit (previously AIRMODE option, now deprecated)
        DISARMED_TILT=(1<<10),                      ///< Allow tilt servos to move when disarmed
        DELAY_ARMING=(1<<11),                       ///< Delay motor spin-up after arming for safety
        DISABLE_SYNTHETIC_AIRSPEED_ASSIST=(1<<12),  ///< Disable synthetic airspeed for Q_ASSIST decisions
        DISABLE_GROUND_EFFECT_COMP=(1<<13),         ///< Disable ground effect compensation during landing
        INGORE_FW_ANGLE_LIMITS_IN_Q_MODES=(1<<14),  ///< Ignore fixed-wing angle limits when in Q-modes
        THR_LANDING_CONTROL=(1<<15),                ///< Use throttle for landing descent control
        DISABLE_APPROACH=(1<<16),                   ///< Disable approach phase of VTOL landing
        REPOSITION_LANDING=(1<<17),                 ///< Allow repositioning during VTOL landing descent
        ONLY_ARM_IN_QMODE_OR_AUTO=(1<<18),          ///< Restrict arming to Q-modes or AUTO mode only
        TRANS_FAIL_TO_FW=(1<<19),                   ///< On transition timeout, continue in fixed-wing instead of QLAND/QRTL
        FS_RTL=(1<<20),                             ///< Use standard RTL instead of QRTL for failsafe
        DISARMED_TILT_UP=(1<<21),                   ///< Tilt motors up when disarmed (for tilt-rotors)
        SCALE_FF_ANGLE_P=(1<<22),                   ///< Scale angle P gain with feedforward for improved attitude response
    };
    
    /**
     * @brief Check if a specific Q_OPTIONS bit is set
     * 
     * @param[in] option The OPTION enum value to check
     * @return true if the specified option bit is set in Q_OPTIONS parameter
     * @return false if the option bit is not set
     * 
     * @note This is the recommended way to check Q_OPTIONS flags throughout the codebase
     */
    bool option_is_set(OPTION option) const {
        return (options.get() & int32_t(option)) != 0;
    }

    // minimum distance to be from destination to use approach logic
    AP_Float approach_distance;

    AP_Float takeoff_failure_scalar;
    AP_Float maximum_takeoff_airspeed;
    uint32_t takeoff_start_time_ms;
    uint32_t takeoff_time_limit_ms;

    float last_land_final_agl;

    // AHRS alt for land abort and package place, meters
    float land_descend_start_alt;

    // min alt for navigation in takeoff
    AP_Float takeoff_navalt_min;
    uint32_t takeoff_last_run_ms;
    float takeoff_start_alt;

    // oneshot with duration ARMING_DELAY_MS used by quadplane to delay spoolup after arming:
    // ignored unless OPTION_DELAY_ARMING or OPTION_TILT_DISARMED is set
    bool delay_arming;

    // should we force use of fixed wing controller for attitude upset recovery?
    bool force_fw_control_recovery;

    // are we in spin recovery?
    bool in_spin_recovery;

    /*
      return true if current mission item is a vtol takeoff
     */
    bool is_vtol_takeoff(uint16_t id) const;

    /*
      return true if current mission item is a vtol landing
     */
    bool is_vtol_land(uint16_t id) const;

#if QAUTOTUNE_ENABLED
    // qautotune mode
    QAutoTune qautotune;
#endif

    /*
      are we in the approach phase of a VTOL landing?
     */
    bool in_vtol_land_approach(void) const;

    /*
      are we in the final landing phase of a VTOL landing?
     */
    bool in_vtol_land_final(void) const;

    /**
     * @brief Check if executing VTOL landing sequence
     * 
     * @details Returns true during any phase of a VTOL landing including:
     *          - Approach phase (deceleration to landing position)
     *          - Airbrake phase (rapid deceleration using VTOL drag)
     *          - Position hold phases (POSITION1, POSITION2)
     *          - Descent phase (controlled descent to ground)
     *          - Final landing phase (ground contact detection)
     * 
     * @return true if in any VTOL landing phase
     * @return false if not landing or in fixed-wing flight
     * 
     * @see in_vtol_land_approach() for approach phase check
     * @see in_vtol_land_descent() for descent phase check
     * @see in_vtol_land_final() for final landing phase check
     */
    bool in_vtol_land_sequence(void) const;

    /*
      see if we are in the VTOL position control phase of a landing
    */
    bool in_vtol_land_poscontrol(void) const;

    /*
      are we in the airbrake phase of a VTOL landing?
     */
    bool in_vtol_airbrake(void) const;

    // returns true if the vehicle should currently be doing a spiral landing
    bool landing_with_fixed_wing_spiral_approach(void) const;

    /*
      return true if we should use the fixed wing attitude control loop
     */
    bool use_fw_attitude_controllers(void) const;

    /*
      get the airspeed for landing approach
     */
    float get_land_airspeed(void);

    /*
      setup for landing approach
     */
    void poscontrol_init_approach(void);

    /*
      calculate our closing velocity vector on the landing
      point. Takes account of the landing point having a velocity
     */
    Vector2f landing_closing_velocity();

    /*
      calculate our desired closing velocity vector on the landing point.
    */
    Vector2f landing_desired_closing_velocity();

    /*
      change spool state, providing easy hook for catching changes in debug
     */
    void set_desired_spool_state(AP_Motors::DesiredSpoolState state);

    /*
      limit forward pitch demand if using rotor tilt or forward flight motor to provide forward acceleration.
     */
    void assign_tilt_to_fwd_thr(void);

    /*
      get a scaled Q_WP_SPEED based on direction of movement
     */
    float get_scaled_wp_speed(float target_bearing_deg) const;

    /*
      setup scaling of roll and pitch angle P gains to match fixed wing gains
     */
    void setup_rp_fw_angle_gains(void);

    /*
      return true if forward throttle from forward_throttle_pct() should be used
     */
    bool allow_forward_throttle_in_vtol_mode() const;

public:
    void motor_test_output();
    MAV_RESULT mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type,
                                        uint16_t throttle_value, float timeout_sec,
                                        uint8_t motor_count);
private:
    void motor_test_stop();

    static QuadPlane *_singleton;
};

#endif  // HAL_QUADPLANE_ENABLED
