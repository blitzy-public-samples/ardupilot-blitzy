/**
 * @file mode.h
 * @brief ArduSub flight mode base class and mode framework definitions
 * 
 * @details This header defines the Mode base class that all ArduSub flight modes
 *          inherit from, providing a common interface for mode initialization,
 *          execution, and capability queries. The Mode class implements the flight
 *          mode framework with virtual methods that derived mode classes override
 *          to implement specific behaviors.
 *          
 *          The mode framework supports the full range of ArduSub capabilities:
 *          - Manual control modes (MANUAL, ACRO, STABILIZE)
 *          - Assisted modes (ALT_HOLD, POSHOLD, SURFTRAK)
 *          - Fully autonomous modes (AUTO, GUIDED, CIRCLE)
 *          - Special purpose modes (SURFACE, MOTOR_DETECT)
 *          
 *          Each mode controls the underwater vehicle through the attitude controller
 *          (AC_AttitudeControl_Sub), position controller (AC_PosControl), and motor
 *          output system (AP_Motors6DOF).
 *          
 *          Mode Lifecycle:
 *          1. Mode::init() - Called once when entering the mode
 *          2. Mode::run() - Called repeatedly at scheduler rate (typically 50Hz for Sub)
 *          3. Mode exit - Cleanup when switching to another mode
 * 
 * @note This is a foundational header for the entire ArduSub flight mode system.
 *       All flight mode implementations inherit from the Mode base class.
 * 
 * @see Sub.h for the main vehicle class
 * @see mode_*.cpp for individual mode implementations
 * 
 * Source: ArduSub/mode.h:1-end
 */

#pragma once

#include "Sub.h"
class Parameters;
class ParametersG2;

class GCS_Sub;

/**
 * @enum GuidedSubMode
 * @brief Sub-modes within the GUIDED flight mode for different control types
 * 
 * @details The GUIDED mode can accept different types of commands from the ground
 *          control station or companion computer. This enum distinguishes between
 *          waypoint navigation, velocity control, position-velocity hybrid control,
 *          and attitude angle control within the GUIDED mode.
 */
enum GuidedSubMode {
    Guided_WP,          ///< Guided waypoint mode - navigate to a specific 3D position
    Guided_Velocity,    ///< Velocity control mode - maintain specified velocity vector
    Guided_PosVel,      ///< Position and velocity control - combined position target with velocity feedforward
    Guided_Angle,       ///< Attitude control mode - maintain specified attitude angles with climb rate
};

/**
 * @enum AutoSubMode
 * @brief Sub-modes within the AUTO flight mode for mission execution
 * 
 * @details AUTO mode executes mission commands from the onboard mission list.
 *          During mission execution, AUTO mode transitions between these sub-modes
 *          based on the current mission command type. Each sub-mode implements
 *          a specific autonomous behavior required for mission execution.
 */
enum AutoSubMode {
    Auto_WP,                  ///< Waypoint navigation - fly to mission waypoint position
    Auto_CircleMoveToEdge,    ///< Moving to circle edge - transition phase when starting circular loiter
    Auto_Circle,              ///< Circular loiter - maintain circular flight pattern around center point
    Auto_NavGuided,           ///< Nav-guided mode - external navigation control during mission
    Auto_Loiter,              ///< Loiter mode - maintain position at current location
    Auto_TerrainRecover       ///< Terrain recovery - regain terrain following after loss
};

/**
 * @enum RTLState
 * @brief State machine states for Return to Launch (RTL) mode execution
 * 
 * @details RTL mode returns the vehicle to the launch position through a sequence
 *          of states. The vehicle progresses through these states to safely return
 *          home, handling altitude changes and final descent/landing.
 * 
 * @note For underwater vehicles, "climb" and "descent" relate to depth changes,
 *       with "climb" meaning moving toward the surface (decreasing depth).
 */
enum RTLState {
    RTL_InitialClimb,    ///< Initial climb - ascend to RTL altitude before returning home
    RTL_ReturnHome,      ///< Return home - navigate horizontally toward launch position
    RTL_LoiterAtHome,    ///< Loiter at home - hold position above launch point
    RTL_FinalDescent,    ///< Final descent - descend to landing altitude
    RTL_Land             ///< Land - final landing sequence (for Sub, typically surface operations)
};

/**
 * @class Mode
 * @brief Base class for all ArduSub flight modes
 * 
 * @details The Mode class provides the common interface and infrastructure for all
 *          ArduSub flight modes. Each flight mode inherits from this base class and
 *          overrides virtual methods to implement mode-specific behavior.
 *          
 *          Mode Inheritance Pattern:
 *          - All modes inherit from Mode base class
 *          - Some modes inherit from other mode classes (e.g., ModeAuto inherits from ModeGuided)
 *          - Virtual methods define the mode interface contract
 *          
 *          Mode Lifecycle:
 *          1. init(bool ignore_checks) - Called when entering the mode
 *             - Returns true if initialization successful, false to prevent mode change
 *             - ignore_checks parameter allows bypassing safety checks
 *          2. run() - Called repeatedly at main loop rate (typically 50Hz for ArduSub)
 *             - Implements the mode's control logic
 *             - Updates attitude controller, position controller, motors
 *          3. Exit - No explicit exit method; cleanup in next mode's init()
 *          
 *          Subsystem Accessor Pattern:
 *          - Mode class provides convenience references to vehicle subsystems
 *          - Reduces code churn during refactoring
 *          - Direct access to: attitude_control, position_control, motors, ahrs, etc.
 *          
 *          Mode Capabilities:
 *          - requires_GPS() - Whether mode needs GPS for operation
 *          - requires_altitude() - Whether mode needs altitude/depth sensor
 *          - allows_arming() - Whether vehicle can be armed in this mode
 *          - is_autopilot() - Whether mode is fully autonomous
 *          - in_guided_mode() - Whether mode accepts external guidance commands
 * 
 * @note All flight mode implementations must inherit from this class and implement
 *       the pure virtual methods (run(), requires_GPS(), requires_altitude(),
 *       allows_arming(), name(), name4(), number()).
 * 
 * @warning Derived mode classes must implement thread-safe run() methods as they
 *          are called from the main scheduler loop at high frequency.
 * 
 * @see Sub.h for the main vehicle class that manages mode switching
 * @see AC_AttitudeControl_Sub for attitude control interface
 * @see AC_PosControl for position control interface
 * @see AP_Motors6DOF for motor output interface
 * 
 * Source: ArduSub/mode.h:36-122
 */
class Mode
{

public:

    /**
     * @enum Number
     * @brief Enumeration of all available flight modes in ArduSub
     * 
     * @details Each flight mode has a unique number used for mode identification,
     *          parameter storage, MAVLink communication, and ground station display.
     *          Mode numbers are permanent and must not be changed as they are stored
     *          in parameters and used by ground control stations.
     * 
     * @note Mode numbers are not necessarily sequential. Gaps exist for historical
     *       reasons and compatibility with other ArduPilot vehicle types.
     */
    enum class Number : uint8_t {
        STABILIZE =     0,  ///< Manual angle control with manual depth/throttle - primary control mode, stabilizes vehicle attitude based on pilot input
        ACRO =          1,  ///< Manual body-frame angular rate control with manual depth/throttle - for advanced pilots, direct rate control
        ALT_HOLD =      2,  ///< Manual angle control with automatic depth/throttle - holds depth automatically while pilot controls horizontal movement
        AUTO =          3,  ///< Fully automatic waypoint control using mission commands - executes pre-programmed mission from onboard mission list
        GUIDED =        4,  ///< Fully automatic control via GCS/companion computer commands - accepts real-time position/velocity commands from external sources
        CIRCLE =        7,  ///< Automatic circular flight with automatic throttle - maintains circular flight pattern around center point
        SURFACE =       9,  ///< Automatically return to surface, pilot maintains horizontal control - ascends to surface while allowing horizontal pilot input
        POSHOLD =      16,  ///< Automatic position hold with manual override and automatic throttle - holds 3D position until pilot moves sticks
        MANUAL =       19,  ///< Pass-through input with no stabilization - direct motor control, no attitude stabilization (for experienced pilots only)
        MOTOR_DETECT = 20,  ///< Automatically detect motor orientation - test mode to determine correct motor configuration and direction
        SURFTRAK =     21   ///< Track distance above seafloor (hold range) - maintains constant altitude above bottom using rangefinder
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    /**
     * @brief Mode constructor - initializes mode base class
     * 
     * @details Constructs a Mode object and initializes convenience references
     *          to vehicle subsystems. Called automatically when mode objects
     *          are instantiated.
     */
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    /**
     * @brief Initialize the mode when entering it
     * 
     * @details Called once when the vehicle enters this mode. Derived classes
     *          override this to perform mode-specific initialization such as
     *          resetting controllers, setting initial targets, or checking
     *          preconditions. Base class implementation returns true.
     * 
     * @param[in] ignore_checks If true, skip safety checks that might prevent mode entry
     * 
     * @return true if mode initialization successful, false to prevent mode change
     * 
     * @note This is called at mode switch time, not at high frequency
     */
    virtual bool init(bool ignore_checks) { return true; }
    
    /**
     * @brief Main mode execution loop - called repeatedly while mode is active
     * 
     * @details Pure virtual function that must be implemented by all derived mode
     *          classes. This is called at the main scheduler rate (typically 50Hz
     *          for ArduSub) and implements the mode's control logic. Should update
     *          attitude targets, position targets, and motor outputs as appropriate
     *          for the mode.
     * 
     * @note This is a pure virtual function (=0) and MUST be overridden
     * @note Called at high frequency (50Hz) - keep execution time minimal
     * 
     * @warning Must be implemented by derived classes. Failing to implement
     *          will result in compilation error.
     */
    virtual void run() = 0;
    
    /**
     * @brief Check if mode requires GPS for operation
     * 
     * @details Pure virtual function indicating whether this mode needs GPS
     *          position information to function. Used for pre-arm checks and
     *          mode switching validation.
     * 
     * @return true if mode requires GPS lock, false otherwise
     * 
     * @note Modes returning true cannot be entered without valid GPS
     */
    virtual bool requires_GPS() const = 0;
    
    /**
     * @brief Check if mode requires altitude/depth sensor for operation
     * 
     * @details Pure virtual function indicating whether this mode needs altitude
     *          or depth sensor (barometer or depth sensor) to function properly.
     *          Used for pre-arm checks and mode switching validation.
     * 
     * @return true if mode requires altitude/depth sensor, false otherwise
     * 
     * @note For underwater vehicles, this typically means depth sensor requirement
     */
    virtual bool requires_altitude() const = 0;
    
    /**
     * @brief Check if vehicle can be armed in this mode
     * 
     * @details Pure virtual function indicating whether the vehicle is allowed
     *          to arm (enable motors) while in this mode. Some modes like MANUAL
     *          may have restrictions on arming.
     * 
     * @param[in] from_gcs true if arming request came from ground control station,
     *                     false if from pilot RC transmitter
     * 
     * @return true if arming is allowed in this mode, false otherwise
     * 
     * @note This is checked during the arming sequence
     */
    virtual bool allows_arming(bool from_gcs) const = 0;
    
    /**
     * @brief Check if mode is fully autonomous (autopilot mode)
     * 
     * @details Indicates whether this mode operates autonomously without pilot
     *          input. Autopilot modes typically control all vehicle axes
     *          automatically. Base class returns false (not autopilot).
     * 
     * @return true if mode is fully autonomous, false if pilot control required
     * 
     * @note Used for flight mode classification and logging
     */
    virtual bool is_autopilot() const { return false; }
    
    /**
     * @brief Check if mode accepts guided commands
     * 
     * @details Indicates whether this mode can accept real-time position,
     *          velocity, or attitude commands from GCS or companion computer.
     *          Base class returns false (not guided mode).
     * 
     * @return true if mode accepts guided commands, false otherwise
     * 
     * @note Only GUIDED and derived modes (like AUTO) return true
     */
    virtual bool in_guided_mode() const { return false; }

    /**
     * @brief Get full mode name string
     * 
     * @details Pure virtual function returning the mode's full name as a
     *          C-string for display and logging.
     * 
     * @return Pointer to mode name string (e.g., "STABILIZE", "ALT_HOLD")
     * 
     * @note String must be static/constant with lifetime exceeding call
     */
    virtual const char *name() const = 0;
    
    /**
     * @brief Get abbreviated 4-character mode name
     * 
     * @details Pure virtual function returning the mode's abbreviated name
     *          for display on limited-space interfaces.
     * 
     * @return Pointer to 4-character mode name string (e.g., "STAB", "ALTH")
     * 
     * @note String must be exactly 4 characters for display compatibility
     */
    virtual const char *name4() const = 0;

    /**
     * @brief Get unique mode number
     * 
     * @details Pure virtual function returning the mode's unique identifying
     *          number from the Mode::Number enum.
     * 
     * @return Mode number from Mode::Number enum
     * 
     * @note Used for parameter storage, MAVLink communication, and logging
     */
    virtual Mode::Number number() const = 0;
  
    /**
     * @brief Convert pilot input to desired angular rates
     * 
     * @details Processes pilot RC input (in centidegrees or RC units) and converts
     *          to desired angular rates (in deg/s) for the attitude controller.
     *          Applies expo curves, scaling, and rate limits configured in parameters.
     * 
     * @param[in]  roll_in   Pilot roll input in centidegrees or RC units
     * @param[in]  pitch_in  Pilot pitch input in centidegrees or RC units  
     * @param[in]  yaw_in    Pilot yaw input in centidegrees or RC units
     * @param[out] roll_out  Desired roll rate in deg/s
     * @param[out] pitch_out Desired pitch rate in deg/s
     * @param[out] yaw_out   Desired yaw rate in deg/s
     * 
     * @note Input scaling and expo curves configured via ACRO_* parameters
     * @note Called by ACRO and other rate-based control modes
     * 
     * Source: ArduSub/mode.h:80
     */
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);


protected:

    /**
     * @brief Run autopilot navigation logic
     * 
     * @details Virtual function for autonomous navigation logic. Base class
     *          provides empty implementation. Autonomous modes override this
     *          to implement waypoint navigation, loiter, or other autonomous
     *          behaviors.
     * 
     * @note Base implementation does nothing; autonomous modes override
     */
    virtual void run_autopilot() {}

    /**
     * @brief Check if vehicle is disarmed or landed
     * 
     * @details Helper function to check vehicle state. Returns true if motors
     *          are disarmed or vehicle has landed. Used by modes to determine
     *          if certain operations (like takeoff) are needed.
     * 
     * @return true if disarmed or landed, false otherwise
     * 
     * @note For underwater vehicles, "landed" typically means on surface or seafloor
     * 
     * Source: ArduSub/mode.h:89
     */
    bool is_disarmed_or_landed() const;

    /**
     * @brief Run horizontal control during landing
     * 
     * @details Controls horizontal position/velocity during landing sequence.
     *          Used by modes that support landing operations to maintain
     *          horizontal stability while descending.
     * 
     * @note For underwater vehicles, used during surface operations
     * 
     * Source: ArduSub/mode.h:93
     */
    void land_run_horizontal_control();
    
    /**
     * @brief Run vertical control during landing
     * 
     * @details Controls vertical descent rate during landing sequence. Manages
     *          descent velocity and can pause descent if requested.
     * 
     * @param[in] pause_descent If true, pause the descent temporarily
     * 
     * @note For underwater vehicles, controls depth/altitude during surface operations
     * 
     * Source: ArduSub/mode.h:94
     */
    void land_run_vertical_control(bool pause_descent = false);

    // Convenience references to avoid code churn in conversion
    // These provide direct access to vehicle subsystems and reduce refactoring overhead
    
    Parameters &g;                           ///< Reference to main parameter set
    ParametersG2 &g2;                        ///< Reference to second parameter set (newer parameters)
    AP_InertialNav &inertial_nav;            ///< Reference to inertial navigation system (position/velocity estimation)
    AP_AHRS &ahrs;                           ///< Reference to Attitude Heading Reference System (attitude estimation)
    AP_Motors6DOF &motors;                   ///< Reference to 6DOF motor output system for underwater vehicles
    RC_Channel *&channel_roll;               ///< Reference to roll RC input channel
    RC_Channel *&channel_pitch;              ///< Reference to pitch RC input channel
    RC_Channel *&channel_throttle;           ///< Reference to throttle/depth RC input channel
    RC_Channel *&channel_yaw;                ///< Reference to yaw RC input channel
    RC_Channel *&channel_forward;            ///< Reference to forward/backward RC input channel (Sub-specific)
    RC_Channel *&channel_lateral;            ///< Reference to lateral/strafe RC input channel (Sub-specific)
    AC_PosControl *position_control;         ///< Pointer to position controller (3D position/velocity control)
    AC_AttitudeControl_Sub *attitude_control; ///< Pointer to attitude controller (angle/rate control for Sub)
    // TODO: channels
    float &G_Dt;                             ///< Reference to loop delta time in seconds

public:

    /**
     * @brief Set vehicle flight mode
     * 
     * @details Pass-through function to set the vehicle's flight mode. Calls
     *          the main vehicle's set_mode function. This is a candidate for
     *          refactoring into the Mode base class proper.
     * 
     * @param[in] mode   Mode number to switch to
     * @param[in] reason Reason for mode change (for logging and diagnostics)
     * 
     * @return true if mode change successful, false if mode change rejected
     * 
     * @note Pass-through function to reduce code churn during refactoring
     * 
     * Source: ArduSub/mode.h:118
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Get reference to ground control station interface
     * 
     * @details Pass-through function to access the GCS (ground control station)
     *          communication interface. This is a candidate for refactoring into
     *          the Mode base class proper.
     * 
     * @return Reference to GCS_Sub object for MAVLink communication
     * 
     * @note Pass-through function to reduce code churn during refactoring
     * 
     * Source: ArduSub/mode.h:119
     */
    GCS_Sub &gcs();

    // end pass-through functions
};

/**
 * @class ModeManual
 * @brief Manual mode - direct motor control with no stabilization
 * 
 * @details MANUAL mode provides pass-through control where pilot inputs directly
 *          control motor outputs with no attitude stabilization. This is for
 *          experienced pilots only and is useful for testing or emergency situations.
 * 
 * @warning No stabilization provided - vehicle will not self-level
 * 
 * Source: ArduSub/mode.h:124-142
 */
class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;
    virtual void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }
    Mode::Number number() const override { return Mode::Number::MANUAL; }
};


/**
 * @class ModeAcro
 * @brief Acrobatic mode - manual body-frame angular rate control with manual depth
 * 
 * @details ACRO mode provides direct angular rate control in the body frame with
 *          manual depth/throttle control. Pilot inputs command rotation rates rather
 *          than angles, providing more responsive control for experienced pilots.
 *          No self-leveling is provided.
 * 
 * @note Requires skilled pilot - vehicle will not self-level when sticks centered
 * 
 * Source: ArduSub/mode.h:145-165
 */
class ModeAcro : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }
    Mode::Number number() const override { return Mode::Number::ACRO; }
};


/**
 * @class ModeStabilize
 * @brief Stabilize mode - manual angle control with manual depth/throttle
 * 
 * @details STABILIZE is the primary control mode for ArduSub. Pilot inputs command
 *          desired roll/pitch angles and yaw rate, with automatic attitude
 *          stabilization. The vehicle will self-level when pilot inputs are
 *          centered. Depth/throttle is under manual control.
 * 
 * @note This is the recommended mode for most operations
 * 
 * Source: ArduSub/mode.h:168-188
 */
class ModeStabilize : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }
    Mode::Number number() const override { return Mode::Number::STABILIZE; }
};


/**
 * @class ModeAlthold
 * @brief Altitude hold mode - manual angle control with automatic depth/throttle
 * 
 * @details ALT_HOLD mode extends STABILIZE by adding automatic depth control.
 *          Pilot controls horizontal movement (roll/pitch/yaw) while the vehicle
 *          automatically maintains depth. This is the most common mode for
 *          underwater inspection and survey operations.
 * 
 * @note Requires functioning depth sensor (barometer or depth sensor)
 * 
 * Source: ArduSub/mode.h:191-215
 */
class ModeAlthold : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Control depth/altitude in ALT_HOLD mode
     * 
     * @details Runs the depth controller to maintain target depth based on
     *          pilot input. Adjusts vertical velocity to hold depth or respond
     *          to pilot depth change commands.
     * 
     * Source: ArduSub/mode.h:205
     */
    void control_depth();

protected:

    /**
     * @brief Pre-processing for mode run
     * 
     * @details Called before main mode logic, handles common setup
     * 
     * Source: ArduSub/mode.h:209
     */
    void run_pre();
    
    /**
     * @brief Post-processing for mode run
     * 
     * @details Called after main mode logic, handles common cleanup
     * 
     * Source: ArduSub/mode.h:210
     */
    void run_post();

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }
    Mode::Number number() const override { return Mode::Number::ALT_HOLD; }
};


/**
 * @class ModeSurftrak
 * @brief Surface tracking mode - maintains constant altitude above seafloor
 * 
 * @details SURFTRAK mode uses a downward-facing rangefinder to maintain a constant
 *          distance above the seafloor or bottom surface. This is useful for
 *          underwater inspection, survey, and pipeline following operations.
 *          Inherits from ModeAlthold and replaces depth control with range control.
 * 
 * @note Requires functioning downward-facing rangefinder
 * @warning Terrain must be within rangefinder detection range
 * 
 * Source: ArduSub/mode.h:218-248
 */
class ModeSurftrak : public ModeAlthold
{

public:
    /**
     * @brief ModeSurftrak constructor
     * 
     * @details Initializes surface tracking mode with default rangefinder target
     * 
     * Source: ArduSub/mode.h:223
     */
    ModeSurftrak();

    void run() override;

    bool init(bool ignore_checks) override;

    /**
     * @brief Get current rangefinder target distance
     * 
     * @return Target distance above terrain in centimeters
     * 
     * Source: ArduSub/mode.h:229
     */
    float get_rangefinder_target_cm() const WARN_IF_UNUSED { return rangefinder_target_cm; }
    
    /**
     * @brief Set rangefinder target distance
     * 
     * @param[in] target_cm Desired distance above terrain in centimeters
     * 
     * @return true if target accepted, false if invalid
     * 
     * Source: ArduSub/mode.h:230
     */
    bool set_rangefinder_target_cm(float target_cm);

protected:

    const char *name() const override { return "SURFTRAK"; }
    const char *name4() const override { return "STRK"; }
    Mode::Number number() const override { return Mode::Number::SURFTRAK; }

private:

    void reset();                     ///< Reset surface tracking state
    void control_range();             ///< Run rangefinder-based altitude control
    void update_surface_offset();    ///< Update surface offset based on rangefinder

    float rangefinder_target_cm;     ///< Target distance above terrain in cm

    bool pilot_in_control;            ///< True when pilot is manually adjusting altitude
    float pilot_control_start_z_cm;   ///< Altitude when pilot took control (for smooth transition)
};

/**
 * @class ModeGuided
 * @brief Guided mode - accepts real-time commands from GCS/companion computer
 * 
 * @details GUIDED mode allows external control via MAVLink commands from a ground
 *          control station or companion computer. Supports position targets,
 *          velocity commands, and attitude commands. This is the primary mode
 *          for autonomous missions controlled by external systems.
 * 
 * @note Requires GPS and altitude sensor
 * @note Accepts real-time position/velocity/attitude commands
 * 
 * Source: ArduSub/mode.h:250-300
 */
class ModeGuided : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool guided_limit_check();
    void guided_limit_init_time_and_pos();
    void guided_set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void guided_set_angle(const Quaternion&, float);
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location&);
    bool guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_velocity(const Vector3f& velocity);
    void guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    float get_auto_heading();
    void guided_limit_clear();
    void set_auto_yaw_mode(autopilot_yaw_mode yaw_mode);

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }
    Mode::Number number() const override { return Mode::Number::GUIDED; }

    autopilot_yaw_mode get_default_auto_yaw_mode(bool rtl) const;

private:
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_takeoff_run();
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
};



/**
 * @class ModeAuto
 * @brief Autonomous mode - executes pre-programmed missions
 * 
 * @details AUTO mode executes missions stored in the onboard mission list. Supports
 *          waypoint navigation, loiter, circle, and other mission commands.
 *          Inherits from ModeGuided to reuse position/velocity control logic.
 * 
 * @note Requires GPS and altitude sensor
 * @note Executes mission commands from onboard mission storage
 * 
 * Source: ArduSub/mode.h:304-341
 */
class ModeAuto : public ModeGuided
{

public:
    // inherit constructor
    using ModeGuided::ModeGuided;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool auto_loiter_start();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location& dest_loc);
    void auto_circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void auto_circle_start();
    void auto_nav_guided_start();
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    void set_yaw_rate(float turn_rate_dps);
    bool auto_terrain_recover_start();

protected:

    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }
    Mode::Number number() const override { return Mode::Number::AUTO; }

private:
    void auto_wp_run();
    void auto_circle_run();
    void auto_nav_guided_run();
    void auto_loiter_run();
    void auto_terrain_recover_run();
};


/**
 * @class ModePoshold
 * @brief Position hold mode - maintains 3D position with manual override
 * 
 * @details POSHOLD mode automatically holds the vehicle's 3D position (horizontal
 *          position and depth) until the pilot provides input. When pilot moves
 *          sticks, vehicle responds and then holds the new position when sticks
 *          are released. Combines automatic position hold with intuitive manual
 *          control.
 * 
 * @note Requires GPS and altitude sensor
 * @note Inherits depth control from ModeAlthold
 * 
 * Source: ArduSub/mode.h:344-369
 */
class ModePoshold : public ModeAlthold
{

public:
    // inherit constructor
    using ModeAlthold::ModeAlthold;

    virtual void run() override;

    bool init(bool ignore_checks) override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "POSH"; }
    Mode::Number number() const override { return Mode::Number::POSHOLD; }

private:

    /**
     * @brief Control horizontal position hold
     * 
     * @details Runs horizontal position controller to maintain position or
     *          respond to pilot input for position changes
     * 
     * Source: ArduSub/mode.h:368
     */
    void control_horizontal();
};


/**
 * @class ModeCircle
 * @brief Circle mode - maintains circular flight pattern
 * 
 * @details CIRCLE mode flies the vehicle in a circular pattern around a center
 *          point at a configured radius. Useful for inspection, survey, and
 *          video recording operations. Automatically controls position and yaw
 *          to maintain the circular path.
 * 
 * @note Requires GPS and altitude sensor
 * @note Circle radius and rate configured via parameters
 * 
 * Source: ArduSub/mode.h:372-392
 */
class ModeCircle : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }
    Mode::Number number() const override { return Mode::Number::CIRCLE; }
};

/**
 * @class ModeSurface
 * @brief Surface mode - automatically ascends to surface
 * 
 * @details SURFACE mode automatically ascends the vehicle toward the surface
 *          (decreasing depth) while allowing pilot to maintain horizontal control.
 *          Useful for emergency surfacing or returning to surface for recovery.
 *          Can operate with or without barometer/depth sensor.
 * 
 * @note Does not require GPS or altitude sensor
 * @note Pilot maintains horizontal control during ascent
 * 
 * Source: ArduSub/mode.h:394-414
 */
class ModeSurface : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "SURFACE"; }
    const char *name4() const override { return "SURF"; }
    Mode::Number number() const override { return Mode::Number::SURFACE; }
    bool nobaro_mode;  ///< True if operating without barometer (pure ascent)
};


/**
 * @class ModeMotordetect
 * @brief Motor detection mode - automatically detects motor configuration
 * 
 * @details MOTOR_DETECT mode is a test mode that automatically determines the
 *          correct motor orientation and configuration. Runs a sequence of
 *          motor tests to identify which motor is which and verify correct
 *          rotation directions. Used during initial vehicle setup.
 * 
 * @warning Should only be used during initial setup in a safe test environment
 * @note Does not require GPS or altitude sensor
 * @note Vehicle must be secured during motor detection
 * 
 * Source: ArduSub/mode.h:417-437
 */
class ModeMotordetect : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "MOTORDETECT"; }
    const char *name4() const override { return "DETE"; }
    Mode::Number number() const override { return Mode::Number::MOTOR_DETECT; }
};
