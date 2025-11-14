/**
 * @file mode.h
 * @brief Blimp flight mode system architecture and mode implementations
 * 
 * @details This file defines the Mode abstraction for the Blimp vehicle,
 *          which is a lighter-than-air vehicle requiring unique control approaches.
 *          
 *          The mode system provides:
 *          - Mode::Number enum defining all available flight modes (LAND, MANUAL, VELOCITY, LOITER, RTL)
 *          - Mode base class with virtual interface for mode implementations
 *          - Concrete mode subclasses implementing specific flight behaviors
 *          
 *          Mode Lifecycle:
 *          1. Construction: Mode object created during vehicle initialization
 *          2. init(): Called when entering the mode (with optional ignore_checks parameter)
 *          3. run(): Called every control loop iteration while mode is active
 *          4. Capability queries: requires_GPS(), allows_arming(), etc. determine mode behavior
 *          
 *          Each mode subclass overrides the virtual methods to implement mode-specific
 *          control logic appropriate for lighter-than-air vehicle dynamics.
 *          
 * @note Mode number 30 is reserved for "offboard" external/Lua control
 * @see Blimp.h for main vehicle class
 * @see mode.cpp for mode implementations
 * 
 * Source: Blimp/mode.h
 */

#pragma once

#include "Blimp.h"
class Parameters;
class ParametersG2;

class GCS_Blimp;

/**
 * @class Mode
 * @brief Base class for all Blimp flight modes
 * 
 * @details The Mode class provides an abstraction for different flight control behaviors
 *          appropriate for lighter-than-air vehicles. Each mode implements specific control
 *          logic by overriding virtual methods.
 *          
 *          Mode Pattern:
 *          - Pure virtual interface enforces implementation of required methods
 *          - Capability query methods (requires_GPS, allows_arming) define mode characteristics
 *          - run() method called at main loop rate implements mode-specific control
 *          
 *          Mode Responsibilities:
 *          - Process pilot input appropriate for the mode
 *          - Calculate desired vehicle state (position, velocity, attitude)
 *          - Interface with navigation and control systems
 *          - Report mode status to ground control station
 *          
 *          Blimp-Specific Considerations:
 *          - Lighter-than-air dynamics require different control approaches than multirotors
 *          - Buoyancy affects vertical control strategy
 *          - Wind sensitivity requires robust position hold algorithms
 *          
 * @note All mode subclasses must implement: run(), name(), name4(), number(), 
 *       requires_GPS(), has_manual_throttle(), allows_arming()
 * @warning Mode transitions affect vehicle control authority and safety checks
 * 
 * Source: Blimp/mode.h:9-127
 */
class Mode
{

public:

    /**
     * @enum Mode::Number
     * @brief Enumeration of all available Blimp flight modes
     * 
     * @details Each mode number corresponds to a specific flight behavior implementation.
     *          Mode numbers are used for:
     *          - Mode switching via RC transmitter or GCS commands
     *          - Parameter storage (mode-specific parameters)
     *          - Telemetry reporting to ground control station
     *          - Logging and debugging
     *          
     *          Mode Numbering Convention:
     *          - 0-10: Standard flight modes
     *          - 30: Reserved for offboard/external control (Lua scripting)
     *          
     * @note Mode numbers must remain stable for parameter and mission compatibility
     * @warning Changing mode numbers breaks backward compatibility with stored parameters
     */
    enum class Number : uint8_t {
        LAND =          0,  ///< Landing mode - currently just stops moving
        MANUAL =        1,  ///< Manual control - direct pilot control of all axes
        VELOCITY =      2,  ///< Velocity mode - pilot commands velocity, autopilot maintains
        LOITER =        3,  ///< Loiter mode - position hold at current location
        RTL =           4,  ///< Return to launch - autonomous return to home position
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    /**
     * @brief Construct a new Mode object
     * 
     * @details Initializes mode base class with references to vehicle subsystems.
     *          Constructor sets up convenience references to avoid code churn during
     *          conversion from monolithic to mode-based architecture.
     */
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // child classes should override these methods
    
    /**
     * @brief Initialize the mode when entering it
     * 
     * @details Called when switching into this mode. Subclasses override to perform
     *          mode-specific initialization such as:
     *          - Setting target positions or velocities
     *          - Resetting integrators or filters
     *          - Checking preconditions (GPS lock, sensor health)
     *          
     * @param[in] ignore_checks If true, bypass safety checks (used during failsafe)
     * 
     * @return true if initialization successful and mode can be entered
     * @return false if initialization failed (mode switch will be rejected)
     * 
     * @note Default implementation returns true (no initialization required)
     * @note Called at mode switch time, not every loop iteration
     */
    virtual bool init(bool ignore_checks)
    {
        return true;
    }
    
    /**
     * @brief Execute mode-specific control logic
     * 
     * @details Pure virtual method called every control loop iteration while this mode
     *          is active. Subclasses must implement to provide mode-specific behavior.
     *          
     *          Typical responsibilities:
     *          - Read pilot input (if applicable)
     *          - Calculate desired vehicle state
     *          - Send commands to attitude/position controllers
     *          - Update mode-specific state variables
     *          
     * @note Called at main loop rate (typically 50-400 Hz depending on vehicle configuration)
     * @warning Must execute quickly to avoid loop timing issues
     */
    virtual void run() = 0;
    
    /**
     * @brief Check if this mode requires GPS for operation
     * 
     * @return true if mode requires GPS lock (position-based modes)
     * @return false if mode can operate without GPS (manual modes)
     * 
     * @note Used to prevent entering GPS-dependent modes when GPS is unavailable
     * @note Affects failsafe behavior and mode availability
     */
    virtual bool requires_GPS() const = 0;
    
    /**
     * @brief Check if this mode allows manual throttle control
     * 
     * @return true if pilot has direct throttle control
     * @return false if throttle is controlled by autopilot
     * 
     * @note Affects pilot input processing and control allocation
     */
    virtual bool has_manual_throttle() const = 0;
    
    /**
     * @brief Check if vehicle can be armed in this mode
     * 
     * @param[in] from_gcs True if arming request is from ground control station
     * 
     * @return true if arming is allowed in this mode
     * @return false if arming is prohibited (e.g., LAND mode)
     * 
     * @note Part of pre-arm safety check system
     * @warning Modes that prohibit arming enhance safety by preventing unexpected behavior
     */
    virtual bool allows_arming(bool from_gcs) const = 0;
    
    /**
     * @brief Check if this mode is an autopilot mode
     * 
     * @return true if mode performs autonomous navigation
     * @return false if mode requires pilot input
     * 
     * @note Default implementation returns false (manual mode)
     * @note Used for telemetry reporting and failsafe logic
     */
    virtual bool is_autopilot() const
    {
        return false;
    }
    
    /**
     * @brief Check if this mode supports user-initiated takeoff
     * 
     * @param[in] must_navigate True if takeoff must include navigation to a target
     * 
     * @return true if mode supports takeoff command
     * @return false if mode does not support takeoff
     * 
     * @note Default implementation returns false (no takeoff support)
     * @note Relevant for autonomous takeoff sequences
     */
    virtual bool has_user_takeoff(bool must_navigate) const
    {
        return false;
    }
    
    /**
     * @brief Check if this mode is a guided mode
     * 
     * @return true if mode accepts external guidance commands
     * @return false if mode does not accept guidance commands
     * 
     * @note Default implementation returns false (not guided mode)
     * @note Used to determine if GCS or companion computer can send position targets
     */
    virtual bool in_guided_mode() const
    {
        return false;
    }

    /**
     * @brief Get the full name of this flight mode
     * 
     * @return const char* Mode name string (e.g., "MANUAL", "LOITER", "RTL")
     * 
     * @note Used for logging, display, and telemetry reporting
     * @note Name is human-readable identifier for the mode
     */
    virtual const char *name() const = 0;
    
    /**
     * @brief Get the 4-character abbreviated name of this flight mode
     * 
     * @return const char* 4-character mode abbreviation (e.g., "MANU", "LOIT", "RTL")
     * 
     * @note Used for space-constrained displays (OSD, small screens)
     * @note Must be exactly 4 characters for display formatting
     */
    virtual const char *name4() const = 0;

    /**
     * @brief Get the unique mode number identifier
     * 
     * @return Mode::Number Enum value identifying this mode
     * 
     * @note Used for mode switching commands and parameter storage
     * @note Each mode has a unique number defined in Mode::Number enum
     */
    virtual Mode::Number number() const = 0;

    /**
     * @brief Check if the vehicle is currently landing
     * 
     * @return true if mode is actively performing landing sequence
     * @return false if not landing
     * 
     * @note Default implementation returns false (not landing)
     * @note Used for telemetry and safety checks
     */
    virtual bool is_landing() const
    {
        return false;
    }

    /**
     * @brief Check if mode requires terrain data for safe operation
     * 
     * @return true if loss of terrain data should trigger failsafe
     * @return false if terrain data is optional or not used
     * 
     * @note Default implementation returns false (terrain not required)
     * @note Relevant for modes using terrain following or collision avoidance
     */
    virtual bool requires_terrain_failsafe() const
    {
        return false;
    }

    /**
     * @brief Get the current waypoint location for GCS display
     * 
     * @param[out] loc Location object to populate with waypoint position
     * 
     * @return true if mode has an active waypoint to report
     * @return false if no waypoint is available
     * 
     * @note Default implementation returns false (no waypoint)
     * @note Used for GCS map display of active navigation target
     */
    virtual bool get_wp(Location &loc)
    {
        return false;
    };
    
    /**
     * @brief Get bearing to active waypoint
     * 
     * @return int32_t Bearing to waypoint in centidegrees (0-36000)
     * 
     * @note Default implementation returns 0 (no waypoint)
     * @note Used for GCS navigation display
     */
    virtual int32_t wp_bearing() const
    {
        return 0;
    }
    
    /**
     * @brief Get distance to active waypoint
     * 
     * @return float Distance to waypoint in meters
     * 
     * @note Default implementation returns 0.0 (no waypoint)
     * @note Used for GCS navigation display and telemetry
     */
    virtual float wp_distance_m() const
    {
        return 0.0f;
    }
    
    /**
     * @brief Get crosstrack error from desired path
     * 
     * @return float Crosstrack error in meters (positive = right of path)
     * 
     * @note Default implementation returns 0.0 (no path following)
     * @note Used for navigation performance monitoring
     */
    virtual float crosstrack_error() const
    {
        return 0.0f;
    }

    /**
     * @brief Update navigation calculations for the current mode
     * 
     * @details Performs navigation updates common across multiple modes.
     *          Called by mode run() methods as needed.
     *          
     * @note Implementation in mode.cpp
     */
    void update_navigation();

    /**
     * @brief Extract and process pilot input from RC channels
     * 
     * @param[out] pilot Vector3f containing pilot input (right, front, up) in normalized units
     * @param[out] yaw Pilot yaw input in normalized units
     * 
     * @details Reads RC channel inputs and processes them into normalized pilot commands.
     *          Handles:
     *          - RC channel scaling and centering
     *          - Deadband application
     *          - Input filtering
     *          - Coordinate frame transformation
     *          
     * @note Called by mode run() methods to get pilot input
     * @note Input ranges and scaling depend on RC calibration
     * @note Implementation in mode.cpp
     */
    void get_pilot_input(Vector3f &pilot, float &yaw);

protected:

    /**
     * @brief Run autopilot navigation logic
     * 
     * @details Virtual method for autopilot modes to implement autonomous navigation.
     *          Default implementation is empty (no autopilot behavior).
     *          
     * @note Override in autopilot modes (RTL, Loiter, etc.)
     */
    virtual void run_autopilot() {}

    /**
     * @brief Check if vehicle is disarmed or landed
     * 
     * @return true if vehicle is disarmed or has landed
     * @return false if vehicle is armed and flying
     * 
     * @note Helper function for mode logic decisions
     * @note Implementation in mode.cpp
     */
    bool is_disarmed_or_landed() const;

    /**
     * @brief Execute horizontal control during landing
     * 
     * @details Provides horizontal position/velocity control appropriate for landing phase.
     *          Called by landing modes to maintain horizontal stability.
     *          
     * @note Implementation in mode.cpp
     * @note Used by modes that support landing sequences
     */
    void land_run_horizontal_control();
    
    /**
     * @brief Execute vertical control during landing
     * 
     * @param[in] pause_descent If true, hold current altitude instead of descending
     * 
     * @details Provides vertical velocity control for landing descent.
     *          Can pause descent while maintaining horizontal control.
     *          
     * @note Implementation in mode.cpp
     * @note Handles buoyancy compensation appropriate for lighter-than-air vehicles
     */
    void land_run_vertical_control(bool pause_descent = false);

    // Convenience references to avoid code churn in conversion:
    // These provide direct access to vehicle subsystems from mode code
    
    Parameters &g;                  ///< Reference to main parameter set
    ParametersG2 &g2;               ///< Reference to secondary parameter set
    AP_InertialNav &inertial_nav;   ///< Reference to inertial navigation system
    AP_AHRS &ahrs;                  ///< Reference to AHRS (attitude and heading reference)
    Fins *&motors;                  ///< Reference to fin control system (blimp actuators)
    Loiter *&loiter;                ///< Reference to loiter controller
    RC_Channel *&channel_right;     ///< Reference to right/roll RC channel
    RC_Channel *&channel_front;     ///< Reference to front/pitch RC channel
    RC_Channel *&channel_up;        ///< Reference to up/throttle RC channel
    RC_Channel *&channel_yaw;       ///< Reference to yaw RC channel
    float &G_Dt;                    ///< Reference to loop delta time in seconds

public:
    /**
     * @brief Request mode change
     * 
     * @param[in] mode Desired mode number to switch to
     * @param[in] reason Reason for mode change (pilot, GCS, failsafe, etc.)
     * 
     * @return true if mode change successful
     * @return false if mode change rejected (failed preconditions)
     * 
     * @note Pass-through function to reduce code churn during conversion
     * @note Candidate for moving into Mode base class in future refactoring
     * @note Calls vehicle-level set_mode() implementation
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Get reference to GCS interface
     * 
     * @return GCS_Blimp& Reference to ground control station communication interface
     * 
     * @note Pass-through function to reduce code churn during conversion
     * @note Candidate for moving into Mode base class in future refactoring
     * @note Used for sending telemetry and status messages
     */
    GCS_Blimp &gcs();

    // end pass-through functions
};

/**
 * @class ModeManual
 * @brief Manual flight mode - direct pilot control of all axes
 * 
 * @details In Manual mode, pilot has direct control over all vehicle axes:
 *          - Right/roll input controls right/left movement
 *          - Front/pitch input controls forward/backward movement
 *          - Up/throttle input controls vertical movement
 *          - Yaw input controls rotation
 *          
 *          Mode Characteristics:
 *          - No GPS required - can fly without position estimation
 *          - No stabilization - pilot must actively control all axes
 *          - Manual throttle control - pilot directly controls vertical velocity
 *          - Allows arming - safe for initial arming and testing
 *          
 *          Typical Use Cases:
 *          - Initial flight testing and tuning
 *          - Indoor flight without GPS
 *          - Manual control when autopilot is not desired
 *          - Emergency manual control
 *          
 * @note Requires skilled pilot input for stable flight
 * @note No position hold or automatic stabilization
 * 
 * Source: Blimp/mode.h:129-170
 */
class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    /**
     * @brief Execute manual mode control logic
     * 
     * @details Reads pilot input and directly commands vehicle actuators.
     *          No stabilization or position hold is applied.
     *          
     * @note Called every control loop iteration
     * @note Implementation in mode_manual.cpp
     */
    virtual void run() override;

    /**
     * @brief Manual mode does not require GPS
     * @return false - can operate without GPS lock
     */
    bool requires_GPS() const override
    {
        return false;
    }
    
    /**
     * @brief Manual mode provides direct throttle control
     * @return true - pilot controls vertical velocity directly
     */
    bool has_manual_throttle() const override
    {
        return true;
    }
    
    /**
     * @brief Manual mode allows arming
     * @param[in] from_gcs Not used in manual mode
     * @return true - arming is always allowed in manual mode
     */
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    
    /**
     * @brief Manual mode is not an autopilot mode
     * @return false - requires continuous pilot input
     */
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    /**
     * @brief Get full mode name
     * @return "MANUAL"
     */
    const char *name() const override
    {
        return "MANUAL";
    }
    
    /**
     * @brief Get abbreviated mode name
     * @return "MANU"
     */
    const char *name4() const override
    {
        return "MANU";
    }

    /**
     * @brief Get mode number identifier
     * @return Mode::Number::MANUAL
     */
    Mode::Number number() const override { return Mode::Number::MANUAL; }

private:

};

/**
 * @class ModeVelocity
 * @brief Velocity mode - pilot commands velocity, autopilot maintains
 * 
 * @details In Velocity mode, pilot input is interpreted as desired velocity rather
 *          than direct control. The autopilot maintains the commanded velocity using
 *          position feedback.
 *          
 *          Mode Characteristics:
 *          - Requires GPS for velocity estimation
 *          - Autopilot controls throttle to maintain desired velocity
 *          - Pilot commands desired velocity on all axes
 *          - Allows arming for normal flight operations
 *          
 *          Control Behavior:
 *          - Pilot input proportional to desired velocity
 *          - Zero stick input commands zero velocity (holds position)
 *          - Smoother flight than manual mode due to velocity control
 *          - Wind compensation through GPS velocity feedback
 *          
 *          Typical Use Cases:
 *          - Smooth manual flight with velocity control
 *          - Assisted manual flight for less experienced pilots
 *          - Transition mode between manual and full autopilot
 *          
 * @note Requires GPS lock for velocity estimation
 * @todo Clarify is_autopilot() return value (currently returns false)
 * 
 * Source: Blimp/mode.h:172-214
 */
class ModeVelocity : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    /**
     * @brief Execute velocity mode control logic
     * 
     * @details Reads pilot input as velocity commands and uses position controller
     *          to maintain desired velocity.
     *          
     * @note Called every control loop iteration
     * @note Implementation in mode_velocity.cpp
     */
    virtual void run() override;

    /**
     * @brief Velocity mode requires GPS
     * @return true - needs GPS for velocity estimation
     */
    bool requires_GPS() const override
    {
        return true;
    }
    
    /**
     * @brief Velocity mode uses automatic throttle control
     * @return false - autopilot controls throttle based on velocity error
     */
    bool has_manual_throttle() const override
    {
        return false;
    }
    
    /**
     * @brief Velocity mode allows arming
     * @param[in] from_gcs Not used in velocity mode
     * @return true - arming is allowed in velocity mode
     */
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    
    /**
     * @brief Check if velocity mode is autopilot mode
     * @return false - currently considered manual mode with velocity assist
     * @todo Review classification - has autopilot characteristics
     */
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    /**
     * @brief Get full mode name
     * @return "VELOCITY"
     */
    const char *name() const override
    {
        return "VELOCITY";
    }
    
    /**
     * @brief Get abbreviated mode name
     * @return "VELY"
     */
    const char *name4() const override
    {
        return "VELY";
    }

    /**
     * @brief Get mode number identifier
     * @return Mode::Number::VELOCITY
     */
    Mode::Number number() const override { return Mode::Number::VELOCITY; }

private:

};

/**
 * @class ModeLoiter
 * @brief Loiter mode - autonomous position hold at current location
 * 
 * @details In Loiter mode, the vehicle maintains its current position and heading
 *          autonomously using GPS and inertial navigation. Pilot can adjust the
 *          hold position with stick inputs.
 *          
 *          Mode Characteristics:
 *          - Requires GPS for position estimation
 *          - Autonomous position and altitude hold
 *          - Pilot can adjust hold position with stick inputs
 *          - Allows arming for normal operations
 *          
 *          Control Behavior:
 *          - On entry: Captures current position and yaw as target
 *          - During flight: Maintains target position against wind/disturbances
 *          - Pilot input: Temporarily adjusts position, returns to hold when released
 *          - Yaw: Holds heading or allows pilot yaw control
 *          
 *          Wind Compensation:
 *          - Uses GPS velocity feedback to compensate for wind
 *          - Particularly important for lighter-than-air vehicles
 *          - Position controller actively fights drift
 *          
 *          Typical Use Cases:
 *          - Hovering at a fixed location for observation
 *          - Holding position while adjusting camera/sensors
 *          - Stable platform for testing or data collection
 *          
 * @note Requires good GPS lock for stable position hold
 * @todo Clarify is_autopilot() return value (currently returns false)
 * 
 * Source: Blimp/mode.h:216-260
 */
class ModeLoiter : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    /**
     * @brief Initialize loiter mode
     * 
     * @param[in] ignore_checks If true, bypass GPS check (failsafe entry)
     * 
     * @return true if initialization successful (GPS available or checks ignored)
     * @return false if GPS not available and checks not ignored
     * 
     * @details Captures current position and yaw as loiter target.
     *          Checks GPS availability unless ignore_checks is true.
     *          
     * @note Implementation in mode_loiter.cpp
     */
    virtual bool init(bool ignore_checks) override;
    
    /**
     * @brief Execute loiter mode control logic
     * 
     * @details Maintains target position using position controller.
     *          Processes pilot input to adjust hold position.
     *          
     * @note Called every control loop iteration
     * @note Implementation in mode_loiter.cpp
     */
    virtual void run() override;

    /**
     * @brief Loiter mode requires GPS
     * @return true - needs GPS for position hold
     */
    bool requires_GPS() const override
    {
        return true;
    }
    
    /**
     * @brief Loiter mode uses automatic throttle control
     * @return false - autopilot controls all axes
     */
    bool has_manual_throttle() const override
    {
        return false;
    }
    
    /**
     * @brief Loiter mode allows arming
     * @param[in] from_gcs Not used in loiter mode
     * @return true - arming is allowed in loiter mode
     */
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    
    /**
     * @brief Check if loiter mode is autopilot mode
     * @return false - currently considered manual with position hold assist
     * @todo Review classification - has autonomous position control
     */
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    /**
     * @brief Get full mode name
     * @return "LOITER"
     */
    const char *name() const override
    {
        return "LOITER";
    }
    
    /**
     * @brief Get abbreviated mode name
     * @return "LOIT"
     */
    const char *name4() const override
    {
        return "LOIT";
    }

    /**
     * @brief Get mode number identifier
     * @return Mode::Number::LOITER
     */
    Mode::Number number() const override { return Mode::Number::LOITER; }

private:
    Vector3f target_pos;  ///< Target position to hold (NED frame, meters)
    float target_yaw;     ///< Target yaw angle to maintain (radians)
};

/**
 * @class ModeLand
 * @brief Landing mode - stops movement for lighter-than-air vehicle landing
 * 
 * @details In Land mode, the blimp stops all movement to prepare for landing or
 *          shutdown. For lighter-than-air vehicles, "landing" primarily means
 *          stopping motion rather than descending to ground.
 *          
 *          Mode Characteristics:
 *          - Does not require GPS
 *          - Manual throttle control (pilot can control descent)
 *          - Prohibits arming (safety feature - landing is end-of-flight state)
 *          - Not an autopilot mode
 *          
 *          Control Behavior:
 *          - Stops horizontal movement
 *          - Allows manual vertical control for final positioning
 *          - Can be used as emergency stop mode
 *          - Prepares vehicle for safe shutdown
 *          
 *          Safety Features:
 *          - Arming prohibited prevents accidental restart
 *          - Simple control logic reduces failure modes
 *          - Works without GPS for emergency situations
 *          
 *          Typical Use Cases:
 *          - Normal end-of-flight landing sequence
 *          - Emergency stop of all autonomous motion
 *          - Pre-shutdown stabilization
 *          - Failsafe landing mode
 *          
 * @note Currently just stops moving - may be enhanced in future for active landing
 * @warning Arming is prohibited in LAND mode for safety
 * 
 * Source: Blimp/mode.h:262-303
 */
class ModeLand : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    /**
     * @brief Execute land mode control logic
     * 
     * @details Stops vehicle movement while allowing manual throttle control.
     *          Provides stable end-of-flight behavior.
     *          
     * @note Called every control loop iteration
     * @note Implementation in mode_land.cpp
     */
    virtual void run() override;

    /**
     * @brief Land mode does not require GPS
     * @return false - can operate without GPS for emergency landing
     */
    bool requires_GPS() const override
    {
        return false;
    }
    
    /**
     * @brief Land mode provides manual throttle control
     * @return true - pilot can control vertical positioning during landing
     */
    bool has_manual_throttle() const override
    {
        return true;
    }
    
    /**
     * @brief Land mode prohibits arming
     * @param[in] from_gcs Not used - arming always prohibited
     * @return false - arming not allowed in landing mode
     * @warning Landing is end-of-flight state; arming prohibited for safety
     */
    bool allows_arming(bool from_gcs) const override
    {
        return false;
    };
    
    /**
     * @brief Land mode is not an autopilot mode
     * @return false - basic landing behavior without complex autopilot logic
     */
    bool is_autopilot() const override
    {
        return false;
    }

protected:

    /**
     * @brief Get full mode name
     * @return "LAND"
     */
    const char *name() const override
    {
        return "LAND";
    }
    
    /**
     * @brief Get abbreviated mode name
     * @return "LAND"
     */
    const char *name4() const override
    {
        return "LAND";
    }

    /**
     * @brief Get mode number identifier
     * @return Mode::Number::LAND
     */
    Mode::Number number() const override { return Mode::Number::LAND; }

private:

};

/**
 * @class ModeRTL
 * @brief Return to Launch mode - autonomous return to home position
 * 
 * @details RTL (Return To Launch) mode autonomously navigates the vehicle back to
 *          the launch location or a designated home position. This is a critical
 *          failsafe mode for bringing the vehicle back safely.
 *          
 *          Mode Characteristics:
 *          - Requires GPS for navigation to home position
 *          - Fully autonomous navigation (autopilot mode)
 *          - Automatic throttle and position control
 *          - Allows arming (can arm and immediately RTL if needed)
 *          
 *          RTL Sequence:
 *          1. Capture home/launch position on initialization
 *          2. Navigate directly to home position
 *          3. Maintain altitude during transit
 *          4. Arrive at home position
 *          5. May transition to loiter or land depending on configuration
 *          
 *          Navigation Behavior:
 *          - Uses position controller to navigate to home
 *          - Compensates for wind during transit
 *          - Maintains safe altitude above terrain
 *          - Direct path unless obstacle avoidance active
 *          
 *          Failsafe Usage:
 *          - Triggered by RC loss, GCS loss, battery low, etc.
 *          - Provides autonomous return when pilot cannot control
 *          - Critical safety feature for lighter-than-air vehicles
 *          
 *          Typical Use Cases:
 *          - Manual RTL command from pilot or GCS
 *          - Automatic failsafe response to lost link
 *          - Low battery return home
 *          - End of autonomous mission
 *          
 * @note Requires GPS lock and valid home position
 * @todo Clarify is_autopilot() return value (currently returns false, should likely be true)
 * @warning Vehicle will navigate autonomously; ensure clear path to home
 * 
 * Source: Blimp/mode.h:305-346
 */
class ModeRTL : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    /**
     * @brief Initialize RTL mode
     * 
     * @param[in] ignore_checks If true, bypass GPS check (failsafe entry)
     * 
     * @return true if initialization successful (GPS and home position valid)
     * @return false if GPS unavailable or home position not set
     * 
     * @details Sets up navigation to home position.
     *          Verifies GPS lock and home position unless checks ignored.
     *          
     * @note Implementation in mode_rtl.cpp
     */
    virtual bool init(bool ignore_checks) override;
    
    /**
     * @brief Execute RTL mode control logic
     * 
     * @details Navigates autonomously to home position using position controller.
     *          Monitors progress and adjusts path as needed.
     *          
     * @note Called every control loop iteration
     * @note Implementation in mode_rtl.cpp
     */
    virtual void run() override;

    /**
     * @brief RTL mode requires GPS
     * @return true - needs GPS for navigation to home
     */
    bool requires_GPS() const override
    {
        return true;
    }
    
    /**
     * @brief RTL mode uses automatic throttle control
     * @return false - autopilot controls all axes during navigation
     */
    bool has_manual_throttle() const override
    {
        return false;
    }
    
    /**
     * @brief RTL mode allows arming
     * @param[in] from_gcs Not used in RTL mode
     * @return true - arming is allowed (can arm directly into RTL if needed)
     */
    bool allows_arming(bool from_gcs) const override
    {
        return true;
    };
    
    /**
     * @brief Check if RTL mode is autopilot mode
     * @return false - currently returns false
     * @todo Review classification - RTL is autonomous navigation, should likely return true
     */
    bool is_autopilot() const override
    {
        return false;
        //TODO
    }

protected:

    /**
     * @brief Get full mode name
     * @return "RTL"
     */
    const char *name() const override
    {
        return "RTL";
    }
    
    /**
     * @brief Get abbreviated mode name
     * @return "RTL"
     */
    const char *name4() const override
    {
        return "RTL";
    }

    /**
     * @brief Get mode number identifier
     * @return Mode::Number::RTL
     */
    Mode::Number number() const override { return Mode::Number::RTL; }

};
