/**
 * @file mode.h
 * @brief Rover mode base class and mode declarations defining the mode system architecture
 * 
 * @details This file defines the Mode base class and all concrete mode implementations for the ArduPilot Rover.
 *          The mode system provides a polymorphic interface for vehicle control, where each mode implements
 *          its own navigation and control logic through virtual methods.
 *          
 *          Architecture:
 *          - Mode base class defines the interface contract for all modes
 *          - Each concrete mode (Auto, Guided, Manual, etc.) inherits from Mode and implements update()
 *          - Mode switching is handled by Rover::set_mode() which validates transitions and calls enter()/exit()
 *          - Modes are identified by the Number enum and accessed via mode_from_mode_num()
 *          
 *          Mode Categories:
 *          - Manual Control: Manual, Acro, Steering - Direct pilot control with varying levels of stabilization
 *          - Autopilot: Auto, Guided, RTL, SmartRTL, Loiter, Circle, Follow - Autonomous navigation
 *          - Special: Hold - Stops all actuators, Initialising - Boot-up state, Dock - Precision docking
 *          
 *          Thread Safety:
 *          - Mode methods are called from the main scheduler thread (typically 50Hz)
 *          - Mode switching is atomic and protected by the vehicle state machine
 * 
 * @note Mode transitions are validated by checking allows_arming(), requires_position(), requires_velocity()
 *       to ensure the vehicle has necessary capabilities before entering a mode.
 * 
 * Source: Rover/mode.h
 */
#pragma once

#include "Rover.h"

// pre-define ModeRTL so Auto can appear higher in this file
class ModeRTL;

/**
 * @class Mode
 * @brief Base class for all rover control modes providing a polymorphic interface for vehicle behavior
 * 
 * @details Mode is an abstract base class that defines the interface contract for all rover flight modes.
 *          Each mode (Manual, Auto, Guided, etc.) inherits from this class and implements its own control
 *          logic by overriding virtual methods.
 *          
 *          Key Responsibilities:
 *          - Define the interface for mode-specific control logic (update() method)
 *          - Provide mode identification (mode_number(), name4())
 *          - Declare mode capabilities (is_autopilot_mode(), requires_position(), etc.)
 *          - Offer navigation primitives for autopilot modes (navigate_to_waypoint(), calc_steering_to_heading())
 *          - Manage pilot input interpretation (get_pilot_desired_steering_and_throttle(), etc.)
 *          
 *          Lifecycle:
 *          1. Mode::enter() called when switching to this mode - validates entry and calls _enter()
 *          2. Mode::update() called every scheduler cycle (typically 50Hz) - implements mode behavior
 *          3. Mode::exit() called when leaving this mode - cleanup and calls _exit()
 *          
 *          Subclass Implementation Pattern:
 *          - Override mode_number() and name4() for identification
 *          - Override update() to implement per-cycle control logic
 *          - Override _enter() for mode-specific initialization (optional)
 *          - Override _exit() for mode-specific cleanup (optional)
 *          - Override capability queries (is_autopilot_mode(), requires_position(), etc.) as needed
 *          
 *          Thread Safety:
 *          - All mode methods are called from the main scheduler thread
 *          - No explicit locking required as access is single-threaded
 *          
 *          Coordinate Frames:
 *          - Headings are typically in centi-degrees (cd), earth frame, 0=North clockwise
 *          - Positions use Location (GPS coordinates) or Vector2f/Vector2p (meters from EKF origin)
 *          - Steering outputs are in the range -4500 to +4500 (centi-degrees)
 * 
 * @note This is an abstract class - cannot be instantiated directly. Use concrete mode classes.
 * @warning Mode switching during critical maneuvers should validate state transitions carefully to avoid
 *          vehicle instability or loss of control. Always check mode preconditions before switching.
 */
class Mode
{
public:

    /**
     * @enum Number
     * @brief Mode identification numbers mapped to each concrete mode class
     * 
     * @details These numbers identify modes in parameters (MODE1-MODE6), MAVLink messages, and logging.
     *          Numbers are stable across firmware versions for ground station compatibility.
     *          
     *          Mode Number Mapping:
     *          - MANUAL (0): Direct pilot control, no stabilization
     *          - ACRO (1): Rate-controlled manual mode with stabilization
     *          - STEERING (3): Pilot controls steering, speed control automated
     *          - HOLD (4): Stops all actuators immediately
     *          - LOITER (5): Holds current position using position controller
     *          - FOLLOW (6): Follow another vehicle (requires follow library)
     *          - SIMPLE (7): Simplified control with heading-lock
     *          - DOCK (8): Precision docking mode (optional, requires MODE_DOCK_ENABLED)
     *          - CIRCLE (9): Circle around a point at constant radius
     *          - AUTO (10): Execute mission commands from AP_Mission
     *          - RTL (11): Return to launch location
     *          - SMART_RTL (12): Return via recorded path
     *          - GUIDED (15): Accept position/velocity targets from GCS or companion computer
     *          - INITIALISING (16): Boot-up state before EKF initialization
     * 
     * @note Some mode numbers are skipped for historical reasons or reserved for future use
     * @note Mode number 30 is reserved for "offboard" external/Lua script control
     */
    enum class Number : uint8_t {
        MANUAL       = 0,
        ACRO         = 1,
        STEERING     = 3,
        HOLD         = 4,
        LOITER       = 5,
        FOLLOW       = 6,
        SIMPLE       = 7,
#if MODE_DOCK_ENABLED
        DOCK         = 8,
#endif
        CIRCLE       = 9,
        AUTO         = 10,
        RTL          = 11,
        SMART_RTL    = 12,
        GUIDED       = 15,
        INITIALISING = 16,
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    /**
     * @brief Constructor for Mode base class
     * 
     * @details Initializes references to vehicle subsystems (AHRS, parameters, RC channels, etc.)
     *          and sets up initial mode state. Called during vehicle initialization for each mode instance.
     */
    Mode();

    // do not allow copying
    CLASS_NO_COPY(Mode);

    /**
     * @brief Enter this mode, returns false if we failed to enter
     * 
     * @details Called by Rover::set_mode() when switching to this mode. Performs validation
     *          and calls the subclass _enter() method for mode-specific initialization.
     *          Entry can fail if preconditions are not met (e.g., GPS required but not available).
     * 
     * @return true if mode was entered successfully, false if entry failed
     * 
     * @note If entry fails, the vehicle will attempt to enter the previous mode or Hold mode
     * @note Subclasses override _enter() (protected) not this method
     */
    bool enter();

    /**
     * @brief Perform any cleanups required when exiting this mode
     * 
     * @details Called by Rover::set_mode() when leaving this mode. Stops any ongoing
     *          navigation, resets mode-specific state, and calls the subclass _exit() method.
     * 
     * @note Subclasses override _exit() (protected) not this method
     */
    void exit();

    /**
     * @brief Returns a unique number specific to this mode
     * 
     * @details Pure virtual method that each mode must implement to return its Number enum value.
     *          Used for mode identification in parameters, telemetry, and logging.
     * 
     * @return Mode::Number enum value identifying this mode
     * 
     * @note This number must be stable across firmware versions for ground station compatibility
     */
    virtual Number mode_number() const = 0;

    /**
     * @brief Returns short text name (up to 4 bytes)
     * 
     * @details Pure virtual method returning a short string identifier for the mode.
     *          Used in logging and text-based displays. Must be exactly 4 characters.
     * 
     * @return const char* pointing to 4-character mode name (e.g., "AUTO", "GUID", "MANU")
     * 
     * @note Must be exactly 4 characters for log format compatibility
     */
    virtual const char *name4() const = 0;

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    /**
     * @brief Convert user input to targets, implement high level control for this mode
     * 
     * @details Pure virtual method called every scheduler cycle (typically 50Hz) to implement
     *          the mode's control logic. Each mode interprets pilot inputs and sensor data
     *          differently to achieve its desired behavior.
     *          
     *          Typical update() implementation:
     *          1. Read pilot inputs (if manual mode) or navigation targets (if autopilot mode)
     *          2. Calculate desired steering and throttle outputs
     *          3. Send outputs to attitude controller or motor library
     *          
     *          Example implementations:
     *          - Manual: Direct passthrough of pilot RC inputs
     *          - Guided: Navigate to commanded waypoint from GCS
     *          - Auto: Execute current mission command
     *          - Hold: Output zero throttle and steering
     * 
     * @note Called at main loop rate (typically 50Hz) - keep execution time minimal
     * @warning Do not block or delay in update() - will cause scheduler overruns
     */
    virtual void update() = 0;

    //
    // attributes of the mode
    //

    /**
     * @brief Return if in non-manual mode: Auto, Guided, RTL, SmartRTL
     * 
     * @details Indicates whether this mode performs autonomous navigation without requiring
     *          continuous pilot input. Autopilot modes typically use position/velocity controllers
     *          and navigation algorithms to reach target locations.
     * 
     * @return true if mode is autonomous (Auto, Guided, RTL, SmartRTL, Loiter, Circle, Follow, Dock)
     * @return false if mode requires manual pilot control (Manual, Acro, Steering, Hold)
     * 
     * @note Used to determine if stick mixing should be allowed and arming restrictions
     */
    virtual bool is_autopilot_mode() const { return false; }

    /**
     * @brief Return if external control is allowed in this mode (Guided or Guided-within-Auto)
     * 
     * @details Indicates whether the mode accepts position/velocity commands from external sources
     *          such as ground control stations or companion computers via MAVLink.
     * 
     * @return true if mode accepts external navigation commands (Guided mode, or Auto mode in Guided submode)
     * @return false otherwise
     */
    virtual bool in_guided_mode() const { return false; }

    /**
     * @brief Returns true if vehicle can be armed or disarmed from the transmitter in this mode
     * 
     * @details Controls whether pilot can arm/disarm via RC switch in this mode.
     *          Typically disabled for autopilot modes to prevent accidental arming during missions.
     * 
     * @return true if transmitter arming allowed (default for manual modes)
     * @return false if transmitter arming disabled (default for autopilot modes)
     */
    virtual bool allows_arming_from_transmitter() { return !is_autopilot_mode(); }

    /**
     * @brief Returns false if vehicle cannot be armed in this mode
     * 
     * @details Some modes (RTL, SmartRTL, Initialising) prevent arming because they require
     *          the vehicle to be already armed or are transitional states.
     * 
     * @return true if mode allows arming (default)
     * @return false if mode prevents arming
     */
    virtual bool allows_arming() const { return true; }

    /**
     * @brief Returns true if pilot stick inputs should be mixed with autopilot commands
     * 
     * @details In autopilot modes, pilot can nudge vehicle position or override speed using stick inputs.
     * 
     * @return true for autopilot modes (allows pilot override)
     * @return false for manual modes (pilot has full control already)
     */
    bool allows_stick_mixing() const { return is_autopilot_mode(); }

    //
    // attributes for mavlink system status reporting
    //

    /**
     * @brief Returns true if any RC input is used
     * 
     * @details Indicates whether mode uses pilot stick inputs for control.
     *          Used for MAVLink SYS_STATUS reporting and mode classification.
     * 
     * @return true for manual/semi-autonomous modes (Manual, Acro, Steering)
     * @return false for fully autonomous modes (Auto, Guided, RTL)
     */
    virtual bool has_manual_input() const { return false; }

    /**
     * @brief Returns true if heading/attitude is actively controlled
     * 
     * @details Indicates whether mode stabilizes vehicle heading.
     *          Used for MAVLink HEARTBEAT message mode flags.
     * 
     * @return true for most modes (default)
     * @return false for Hold and Manual modes (no active stabilization)
     */
    virtual bool attitude_stabilized() const { return true; }

    /**
     * @brief Returns true if mode requires position estimate from EKF
     * 
     * @details Indicates whether mode needs valid position fix to operate safely.
     *          Used for pre-mode-change validation and MAVLink reporting.
     * 
     * @return true for modes needing position (Auto, Guided, Loiter, etc.) - default
     * @return false for modes that work without position (Manual, Hold, Acro, Steering)
     * 
     * @note Mode change will be rejected if position required but EKF has no position fix
     */
    virtual bool requires_position() const { return true; }

    /**
     * @brief Returns true if mode requires velocity estimate from EKF
     * 
     * @details Indicates whether mode needs valid velocity estimate to operate safely.
     *          Used for pre-mode-change validation and MAVLink reporting.
     * 
     * @return true for modes needing velocity (most modes) - default
     * @return false for modes that work without velocity (Manual, Hold)
     * 
     * @note Mode change will be rejected if velocity required but EKF has no velocity estimate
     */
    virtual bool requires_velocity() const { return true; }

    /**
     * @brief Return waypoint bearing in degrees for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
     * 
     * @details Bearing from vehicle to the next waypoint in the mission or navigation target.
     *          Used primarily for telemetry display on ground control station.
     * 
     * @return Bearing to waypoint in degrees (0-360, 0=North, clockwise)
     * 
     * @note Only meaningful for modes with waypoint navigation (Auto, Guided, RTL)
     */
    virtual float wp_bearing() const;

    /**
     * @brief Return navigation bearing in degrees for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
     * 
     * @details Desired heading the vehicle should follow to track toward target.
     *          May differ from wp_bearing due to cross-track correction or path following.
     * 
     * @return Navigation bearing in degrees (0-360, 0=North, clockwise)
     */
    virtual float nav_bearing() const;

    /**
     * @brief Return cross track error in meters for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
     * 
     * @details Perpendicular distance from vehicle to the desired path or circle edge.
     *          Positive values typically indicate vehicle is right of desired path.
     * 
     * @return Cross track error in meters
     */
    virtual float crosstrack_error() const;

    /**
     * @brief Return desired lateral acceleration in m/s² for reporting to ground station
     * 
     * @details Lateral acceleration command being sent to the steering controller.
     *          Used for telemetry and performance monitoring.
     * 
     * @return Desired lateral acceleration in m/s²
     */
    virtual float get_desired_lat_accel() const;

    /**
     * @brief Get speed error in m/s (not currently supported)
     * 
     * @return 0.0f (speed error tracking not implemented)
     */
    float speed_error() const { return 0.0f; }

    //
    // navigation methods
    //

    /**
     * @brief Return distance in meters to destination
     * 
     * @details Straight-line distance from current vehicle position to navigation target.
     *          Used for triggering waypoint reached conditions and telemetry reporting.
     * 
     * @return Distance to destination in meters (0.0 if no destination)
     */
    virtual float get_distance_to_destination() const { return 0.0f; }

    /**
     * @brief Return desired location (used in Guided, Auto, RTL, etc)
     * 
     * @details Retrieves the target location this mode is navigating toward.
     *          Used by external systems to query mode's current navigation goal.
     * 
     * @param[out] destination Location object to be populated with target coordinates
     * 
     * @return true if destination is valid and was copied to parameter
     * @return false if there is no valid destination for this mode
     * 
     * @note Only autopilot modes with waypoint navigation return true
     */
    virtual bool get_desired_location(Location& destination) const WARN_IF_UNUSED { return false; }

    /**
     * @brief Set desired location (used in Guided, Auto)
     * 
     * @details Commands the mode to navigate to a new target location.
     *          Used by Guided mode to accept GCS commands and by Auto for mission waypoints.
     * 
     * @param[in] destination Target location to navigate to (GPS coordinates)
     * @param[in] next_destination Next location after destination (optional, for path planning)
     *                             If not provided, vehicle will stop at destination
     * 
     * @return true if destination was accepted and navigation started
     * @return false if mode cannot accept destinations
     * 
     * @note Only Guided and Auto modes typically accept set_desired_location calls
     * @note Providing next_destination enables smoother cornering and speed planning
     */
    virtual bool set_desired_location(const Location &destination, Location next_destination = Location()) WARN_IF_UNUSED;

    /**
     * @brief Returns true if vehicle has reached desired location
     * 
     * @details Indicates whether vehicle is within acceptable distance/heading tolerance of target.
     *          Used by mission logic to advance to next waypoint.
     * 
     * @return true if destination reached or no destination (default for non-navigation modes)
     * @return false if still navigating to destination
     * 
     * @note Default returns true to prevent missions from stalling in non-navigation modes
     */
    virtual bool reached_destination() const { return true; }

    /**
     * @brief Get default speed for this mode from parameters (CRUISE_SPEED, WP_SPEED or RTL_SPEED)
     * 
     * @details Returns the configured default speed parameter for this mode.
     *          Different modes use different speed parameters based on their purpose.
     * 
     * @param[in] rtl Set to true if called from RTL or SmartRTL modes (uses RTL_SPEED parameter)
     * 
     * @return Default speed in m/s from appropriate parameter
     * 
     * @note Auto/Guided typically use WP_SPEED, RTL/SmartRTL use RTL_SPEED, others use CRUISE_SPEED
     */
    float get_speed_default(bool rtl = false) const;

    /**
     * @brief Set desired speed in m/s
     * 
     * @details Commands the mode to change its target speed.
     *          Used by scripting, MAVLink commands, and mission speed change commands.
     * 
     * @param[in] speed Desired speed in m/s (positive forward, negative reverse)
     * 
     * @return true if mode accepted the speed command
     * @return false if mode does not support speed changes (default)
     * 
     * @note Only autopilot modes typically accept speed commands
     */
    virtual bool set_desired_speed(float speed) { return false; }

    /**
     * @brief Execute the mission in reverse (i.e. backing up)
     * 
     * @details Sets a flag that causes the vehicle to drive in reverse.
     *          Used for backing out of tight spaces or reverse waypoint navigation.
     * 
     * @param[in] value true to enable reverse driving, false for normal forward driving
     */
    void set_reversed(bool value);

    /**
     * @brief Initialize reversed flag when entering autopilot mode
     * 
     * @details Resets the reverse driving flag to false when entering most autopilot modes,
     *          unless resuming a mission that was already in reverse.
     * 
     * @note Overridden by Auto mode to preserve reverse state when resuming mission
     */
    virtual void init_reversed_flag() { if (is_autopilot_mode()) { set_reversed(false); } }

    /**
     * @brief Handle tacking request (from auxiliary switch) in sailboats
     * 
     * @details Initiates a sailboat tack maneuver when pilot activates tack auxiliary function.
     *          Only functional in modes that support manual tacking (Acro mode).
     * 
     * @note Default implementation does nothing - override in modes supporting tacking
     */
    virtual void handle_tack_request();

protected:

    /**
     * @brief Subclasses override this to perform checks before entering the mode
     * 
     * @details Called by enter() after base class validation. Subclasses implement
     *          mode-specific initialization and precondition checking here.
     * 
     * @return true if mode-specific entry succeeded (default)
     * @return false if entry should be aborted
     * 
     * @note This is where modes initialize navigation targets, reset state, etc.
     */
    virtual bool _enter() { return true; }

    /**
     * @brief Subclasses override this to perform any required cleanup when exiting the mode
     * 
     * @details Called by exit() before base class cleanup. Subclasses implement
     *          mode-specific cleanup and state reset here.
     * 
     * @note Stop any ongoing timers, clear targets, reset flags, etc.
     */
    virtual void _exit() { return; }

    /**
     * @brief Decode pilot steering and throttle inputs
     * 
     * @details Reads RC stick positions and applies deadzone, scaling, and expo curves
     *          to produce steering and throttle commands for the vehicle.
     * 
     * @param[out] steering_out Steering command in range -4500 to +4500 (centi-degrees)
     *                          Positive values = turn right/clockwise
     * @param[out] throttle_out Throttle command in range -100 to +100 (percentage)
     *                          Positive = forward, negative = reverse
     * 
     * @note Applies configured deadzone, expo, and scaling from RC_Channel parameters
     */
    void get_pilot_desired_steering_and_throttle(float &steering_out, float &throttle_out) const;

    /**
     * @brief Decode pilot input steering and return steering and speed commands
     * 
     * @details Similar to get_pilot_desired_steering_and_throttle but converts throttle
     *          stick position to a target speed in m/s based on speed parameters.
     * 
     * @param[out] steering_out Steering command in range -4500 to +4500 (centi-degrees)
     * @param[out] speed_out Target speed in m/s (positive = forward, negative = reverse)
     * 
     * @note Used by modes that implement speed control rather than direct throttle
     */
    void get_pilot_desired_steering_and_speed(float &steering_out, float &speed_out) const;

    /**
     * @brief Decode pilot lateral movement input
     * 
     * @details Reads lateral stick (if configured) for side-to-side movement on omnidirectional rovers.
     * 
     * @param[out] lateral_out Lateral command in range -4500 to +4500 (centi-degrees)
     *                         Positive = right, negative = left
     * 
     * @note Only functional on rovers with lateral movement capability (omni wheels, mecanum)
     */
    void get_pilot_desired_lateral(float &lateral_out) const;

    /**
     * @brief Decode pilot's input and return heading and speed commands
     * 
     * @details Converts pilot stick inputs to a desired heading (yaw stick) and speed (throttle stick).
     *          Used by modes that fly to a heading rather than direct steering control.
     * 
     * @param[out] heading_out Desired heading in centi-degrees (0-36000, 0=North, clockwise)
     * @param[out] speed_out Target speed in m/s
     */
    void get_pilot_desired_heading_and_speed(float &heading_out, float &speed_out) const;

    /**
     * @brief Decode pilot roll and pitch inputs
     * 
     * @details Reads roll and pitch stick positions for vehicles with active suspension or
     *          walking robots that need body attitude control.
     * 
     * @param[out] roll_out Roll command in range -1 to +1 (normalized)
     * @param[out] pitch_out Pitch command in range -1 to +1 (normalized)
     * 
     * @note Only used by rovers with roll/pitch control capability (balance bots, legged robots)
     */
    void get_pilot_desired_roll_and_pitch(float &roll_out, float &pitch_out) const;

    /**
     * @brief Decode pilot height inputs for walking robots
     * 
     * @details Reads height adjustment stick for legged robots with adjustable body height.
     * 
     * @param[out] walking_height_out Height command in range -1 to +1 (normalized)
     * 
     * @note Only used by walking/legged rovers with height adjustment
     */
    void get_pilot_desired_walking_height(float &walking_height_out) const;

    /**
     * @brief High level call to navigate to waypoint using position controller
     * 
     * @details Calculates steering and throttle to navigate from current position to
     *          the active waypoint stored in _destination. Handles path following,
     *          cross-track error correction, and speed control.
     * 
     * @note Requires valid position estimate and destination set in _destination
     * @note Updates _distance_to_destination and _reached_destination member variables
     */
    void navigate_to_waypoint();

    /**
     * @brief Calculate steering output given a desired turn rate
     * 
     * @details Converts a turn rate command to steering output using the attitude controller.
     *          Used for rate-controlled modes and path following.
     * 
     * @param[in] turn_rate Desired turn rate in radians/sec (positive = turn right/clockwise)
     * 
     * @note Steering output is sent directly to attitude controller
     */
    void calc_steering_from_turn_rate(float turn_rate);

    /**
     * @brief Calculate steering angle given a desired lateral acceleration
     * 
     * @details Converts lateral acceleration command to steering output for path tracking.
     *          Uses vehicle speed and turning radius to compute required steering angle.
     * 
     * @param[in] lat_accel Desired lateral acceleration in m/s²
     * @param[in] reversed Set to true if driving in reverse (inverts steering)
     * 
     * @note Higher speeds require less steering for same lateral acceleration
     */
    void calc_steering_from_lateral_acceleration(float lat_accel, bool reversed = false);

    /**
     * @brief Calculate steering output to drive towards desired heading
     * 
     * @details Implements heading controller that calculates steering to reach target heading.
     *          Used for heading hold, guided heading mode, and mission heading commands.
     * 
     * @param[in] desired_heading_cd Target heading in centi-degrees (0-36000, 0=North, clockwise)
     * @param[in] rate_max_degs Maximum turn rate in deg/s (0 = use default from parameters)
     * 
     * @note Uses PID controller to minimize heading error while respecting rate limits
     */
    void calc_steering_to_heading(float desired_heading_cd, float rate_max_degs = 0.0f);

    /**
     * @brief Calculates the amount of throttle that should be output
     * 
     * @details Implements speed controller that calculates throttle to achieve target speed.
     *          Considers proximity to waypoints, cornering speed limits, and obstacle avoidance.
     * 
     * @param[in] target_speed Desired speed in m/s (positive = forward, negative = reverse)
     * @param[in] avoidance_enabled Set to true to reduce speed when obstacles detected
     * 
     * @note May reduce speed below target_speed when approaching waypoints or obstacles
     * @note Sends throttle output directly to motor library
     */
    virtual void calc_throttle(float target_speed, bool avoidance_enabled);

    /**
     * @brief Performs a controlled stop
     * 
     * @details Gradually reduces vehicle speed to zero using deceleration limits.
     *          Maintains heading stability during braking.
     * 
     * @return true once vehicle has stopped (speed < threshold)
     * @return false if still decelerating
     * 
     * @note Call repeatedly in update() until it returns true
     */
    bool stop_vehicle();

    /**
     * @brief Estimate maximum vehicle speed in m/s
     * 
     * @details Calculates theoretical maximum speed based on cruise speed/throttle relationship
     *          and configured speed limits. Used for speed planning and limit enforcement.
     * 
     * @param[in] cruise_speed Cruise speed in m/s from parameter
     * @param[in] cruise_throttle Cruise throttle in range -1 to +1 from parameter
     * 
     * @return Estimated maximum speed in m/s
     */
    float calc_speed_max(float cruise_speed, float cruise_throttle) const;

    /**
     * @brief Calculate pilot input to nudge speed up or down
     * 
     * @details In autopilot modes, allows pilot to temporarily override target speed
     *          using throttle stick. Returns speed adjustment based on stick position.
     * 
     * @param[in] target_speed Current target speed in m/s
     * @param[in] reversed Set to true if vehicle is intentionally backing up
     *                     (allows pilot to increase reverse speed by pulling throttle down)
     * 
     * @return Speed adjustment in m/s to add to target_speed
     * 
     * @note Returns 0.0 if stick is centered (no pilot override)
     */
    float calc_speed_nudge(float target_speed, bool reversed);

protected:

    // decode pilot steering and throttle inputs and return in steer_out and throttle_out arguments
    // steering_out is in the range -4500 ~ +4500 with positive numbers meaning rotate clockwise
    // throttle_out is in the range -100 ~ +100
    void get_pilot_input(float &steering_out, float &throttle_out) const;
    void set_steering(float steering_value);

    // references to avoid code churn:
    class AP_AHRS &ahrs;
    class Parameters &g;
    class ParametersG2 &g2;
    class RC_Channel *&channel_steer;
    class RC_Channel *&channel_throttle;
    class RC_Channel *&channel_lateral;
    class RC_Channel *&channel_roll;
    class RC_Channel *&channel_pitch;
    class RC_Channel *&channel_walking_height;
    class AR_AttitudeControl &attitude_control;

    // private members for waypoint navigation
    float _distance_to_destination; // distance from vehicle to final destination in meters
    bool _reached_destination;  // true once the vehicle has reached the destination
    float _desired_yaw_cd;      // desired yaw in centi-degrees.  used in Auto, Guided and Loiter
};


/**
 * @class ModeAcro
 * @brief Rate-controlled manual mode with stabilization for rovers
 * 
 * @details Acro mode provides manual control with rate stabilization. Pilot stick inputs
 *          command turn rates rather than absolute steering angles, providing smoother
 *          control and the ability to perform coordinated turns. Similar to Acro mode in
 *          multirotors but for ground vehicles.
 *          
 *          Control behavior:
 *          - Steering stick: Commands turn rate (deg/s)
 *          - Throttle stick: Direct throttle control
 *          - Rate controller stabilizes turn rate
 *          - Manual tacking support for sailboats
 *          
 *          Use cases:
 *          - Smooth manual driving with rate stabilization
 *          - Sailboat tacking maneuvers
 *          - Learning platform for new pilots
 * 
 * @note Requires velocity estimate for rate control on non-skid-steer rovers
 */
class ModeAcro : public Mode
{
public:

    Number mode_number() const override { return Number::ACRO; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // acro mode requires a velocity estimate for non skid-steer rovers
    bool requires_position() const override { return false; }
    bool requires_velocity() const override;

    // sailboats in acro mode support user manually initiating tacking from transmitter
    void handle_tack_request() override;
};


/**
 * @class ModeAuto
 * @brief Autonomous mission execution mode following waypoints from AP_Mission
 * 
 * @details Auto mode executes pre-programmed missions stored in AP_Mission. Navigates through
 *          waypoints, performs mission commands (delays, conditionals, camera triggers, etc.),
 *          and can hand over control to scripting or guided mode when commanded.
 *          
 *          Mission execution:
 *          - Reads mission commands from AP_Mission library
 *          - Executes NAV commands (waypoint, RTL, loiter, circle, heading/speed, script time)
 *          - Executes DO commands (set servo, camera trigger, set relay, change speed)
 *          - Supports conditional commands (delay, distance, within)
 *          - Allows Guided-within-Auto for external control during missions
 *          
 *          Submodes:
 *          - WP: Navigate to waypoint
 *          - HeadingAndSpeed: Drive at specified heading and speed
 *          - RTL: Return to launch within mission
 *          - Loiter: Hold position for duration
 *          - Guided: Accept external commands (Guided-within-Auto)
 *          - Stop: Controlled stop
 *          - NavScriptTime: Lua script control during mission
 *          - Circle: Circle around point
 *          
 *          Mission resume:
 *          - Automatically resumes mission from last waypoint after failsafe recovery
 *          - Stores mission progress in AP_Mission
 * 
 * @note Mission commands are defined in mavlink/common.xml (MAV_CMD_*)
 * @warning Ensure mission is validated before flight - invalid missions can cause unexpected behavior
 */
class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name4() const override { return "AUTO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void calc_throttle(float target_speed, bool avoidance_enabled) override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return if external control is allowed in this mode (Guided or Guided-within-Auto)
    bool in_guided_mode() const override { return _submode == SubMode::Guided || _submode == SubMode::NavScriptTime; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override;
    float get_desired_lat_accel() const override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // start RTL (within auto)
    void start_RTL();

    // lua accessors for nav script time support
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);
    void nav_script_time_done(uint16_t id);

    // 
    void init_reversed_flag() override {
        if (!mission.is_resume()) {
            set_reversed(false);
        }
    }

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command&),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command_callback, bool, const AP_Mission::Mission_Command&),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    enum class DoneBehaviour : uint8_t {
        HOLD      = 0,
        LOITER    = 1,
        ACRO      = 2,
        MANUAL    = 3,
    };

protected:

    bool _enter() override;
    void _exit() override;

    enum SubMode: uint8_t {
        WP,                // drive to a given location
        HeadingAndSpeed,   // turn to a given heading
        RTL,               // perform RTL within auto mode
        Loiter,            // perform Loiter within auto mode
        Guided,            // handover control to external navigation system from within auto mode
        Stop,              // stop the vehicle as quickly as possible
        NavScriptTime,     // accept targets from lua scripts while NAV_SCRIPT_TIME commands are executing
        Circle,            // circle a given location
    } _submode;

private:

    bool check_trigger(void);
    bool start_loiter();
    void start_guided(const Location& target_loc);
    void start_stop();
    void send_guided_position_target();

    bool start_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);

    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
    bool do_nav_wp(const AP_Mission::Mission_Command& cmd, bool always_stop_at_destination);
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_nav_set_yaw_speed(const AP_Mission::Mission_Command& cmd);
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_RTL() const;
    bool verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_set_yaw_speed();
    bool do_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    bool verify_wait_delay();
    bool verify_within_distance();
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_set_reverse(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_script_time();
#endif

    bool waiting_to_start;  // true if waiting for EKF origin before starting mission
    bool auto_triggered;        // true when auto has been triggered to start

    // HeadingAndSpeed sub mode variables
    float _desired_speed;   // desired speed in HeadingAndSpeed submode
    bool _reached_heading;  // true when vehicle has reached desired heading in TurnToHeading sub mode

    // Loiter control
    uint16_t loiter_duration;       // How long we should loiter at the nav_waypoint (time in seconds)
    uint32_t loiter_start_time;     // How long have we been loitering - The start time in millis
    bool previously_reached_wp;     // set to true if we have EVER reached the waypoint

    // Guided-within-Auto variables
    struct {
        Location loc;           // location target sent to external navigation
        bool valid;             // true if loc is valid
        uint32_t last_sent_ms;  // system time that target was last sent to offboard navigation
    } guided_target;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;
    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    int32_t condition_start;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands
    uint32_t nav_delay_time_start_ms;

#if AP_SCRIPTING_ENABLED
    // nav_script_time command variables
    struct {
        bool done;          // true once lua script indicates it has completed
        uint16_t id;        // unique id to avoid race conditions between commands and lua scripts
        uint32_t start_ms;  // system time nav_script_time command was received (used for timeout)
        uint8_t command;    // command number provided by mission command
        uint8_t timeout_s;  // timeout (in seconds) provided by mission command
        float arg1;         // 1st argument provided by mission command
        float arg2;         // 2nd argument provided by mission command
        int16_t arg3;       // 3rd argument provided by mission command
        int16_t arg4;       // 4th argument provided by mission command
    } nav_scripting;
#endif

    // Mission change detector
    AP_Mission_ChangeDetector mis_change_detector;
};

/**
 * @class ModeCircle
 * @brief Circle around a point at constant radius and speed
 * 
 * @details Circle mode drives the vehicle in a circular path around a specified center point.
 *          Maintains constant radius and speed while tracking the circle edge. Can be initiated
 *          manually or as part of a mission command.
 *          
 *          Circle behavior:
 *          - Drives to circle edge if starting outside/inside circle
 *          - Maintains constant distance from center point (radius parameter)
 *          - Drives at constant speed (speed parameter)
 *          - Direction: clockwise or counter-clockwise (direction parameter)
 *          - Tracks total angle circled for mission completion conditions
 *          
 *          Configuration parameters:
 *          - CIRCLE_RADIUS: Circle radius in meters
 *          - CIRCLE_SPEED: Speed around circle in m/s (0 = use WP_SPEED)
 *          - CIRCLE_DIR: Direction (0=clockwise, 1=counter-clockwise)
 *          
 *          Use cases:
 *          - Surveillance/monitoring of fixed point
 *          - Search patterns
 *          - Demonstration/testing
 *          - Mission circle commands
 * 
 * @note Automatically limits speed to respect lateral acceleration limits
 * @note Increases radius if smaller than vehicle's minimum turn radius
 */
class ModeCircle : public Mode
{
public:

    // need a constructor for parameters
    ModeCircle();

    // Does not allow copies
    CLASS_NO_COPY(ModeCircle);

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name4() const override { return "CIRC"; }

    // return the distance at which the vehicle is considered to be on track along the circle
    float get_reached_distance() const;

    // initialise with specific center location, radius (in meters) and direction
    // replaces use of _enter when initialised from within Auto mode
    bool set_center(const Location& center_loc, float radius_m, bool dir_ccw);

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override { return dist_to_edge_m; }
    float get_desired_lat_accel() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed_ms) override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return total angle in radians that vehicle has circled
    // fabsf is used so that full rotations in either direction are counted
    float get_angle_total_rad() const { return fabsf(angle_total_rad); }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float radius;        // circle radius in meters
    AP_Float speed;         // vehicle speed in m/s.  If zero uses WP_SPEED
    AP_Int8 direction;      // direction 0:clockwise, 1:counter-clockwise

    // initialise mode
    bool _enter() override;

    // Update position controller targets driving to the circle edge
    void update_drive_to_radius();

    // Update position controller targets while circling
    void update_circling();

    // initialise target_yaw_rad using the vehicle's position and yaw
    // if there is no current position estimate target_yaw_rad is set to vehicle yaw
    void init_target_yaw_rad();

    // limit config speed so that lateral acceleration is within limits
    // outputs warning to user if speed is reduced
    void check_config_speed();

    // ensure config radius is no smaller then vehicle's TURN_RADIUS
    // radius is increased if necessary and warning is output to the user
    void check_config_radius();

    // enum for Direction parameter
    enum class Direction {
        CW = 0,
        CCW = 1
    };

    // local members
    struct {
        Location center_loc;    // circle center as a Location
        Vector2f center_pos;    // circle center as an offset (in meters) from the EKF origin
        float radius;   // circle radius
        float speed;    // desired speed around circle in m/s
        Direction dir;  // direction, 0:clockwise, 1:counter-clockwise
    } config;
    struct {
        float speed;    // vehicle's target speed around circle in m/s
        float yaw_rad;  // earth-frame angle of tarrget point on the circle
        Vector2p pos;   // latest position target sent to position controller
        Vector2f vel;   // latest velocity target sent to position controller
        Vector2f accel; // latest accel target sent to position controller
    } target;
    float angle_total_rad;  // total angle in radians that vehicle has circled
    bool reached_edge;      // true once vehicle has reached edge of circle
    float dist_to_edge_m;   // distance to edge of circle in meters (equivalent to crosstrack error)
    bool tracking_back;     // true if the vehicle is trying to track back onto the circle
};

/**
 * @class ModeGuided
 * @brief Accept position/velocity/heading targets from GCS or companion computer
 * 
 * @details Guided mode enables external control of the vehicle via MAVLink commands from a
 *          ground control station or companion computer. Supports multiple control interfaces
 *          including position targets, velocity targets, heading/speed commands, and attitude
 *          control for sailboats.
 *          
 *          Control types supported:
 *          - Position targets: Drive to specific GPS coordinate or offset from current position
 *          - Velocity targets: Drive at specified velocity vector (NED frame)
 *          - Heading and speed: Drive at constant heading and speed
 *          - Turn rate and speed: Manual-like control via MAVLink
 *          - Attitude (sail): Wind vane angle control for sailboats
 *          
 *          MAVLink command handling:
 *          - SET_POSITION_TARGET_LOCAL_NED: Position or velocity control in NED frame
 *          - SET_POSITION_TARGET_GLOBAL_INT: GPS position targets
 *          - SET_ATTITUDE_TARGET: Attitude control (sailboats)
 *          - NAV_GUIDED_ENABLE: Enable/disable guided control
 *          
 *          Timeout protection:
 *          - Stops vehicle if no new commands received within timeout period
 *          - Timeout configurable via WP_TIMEOUT parameter
 *          
 *          Use cases:
 *          - Companion computer navigation (avoidance, vision-based)
 *          - Dynamic path planning from GCS
 *          - Research and algorithm development
 *          - Follow-me mode from ground station
 *          - ROS2/DDS integration
 * 
 * @note Can be used within Auto missions (Guided-within-Auto) for external control segments
 * @warning Ensure reliable MAVLink connection - command timeout will stop vehicle
 */
class ModeGuided : public Mode
{
public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Rover;
#endif

    Number mode_number() const override { return Number::GUIDED; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return if external control is allowed in this mode (Guided or Guided-within-Auto)
    bool in_guided_mode() const override { return true; }

    // return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override;
    float crosstrack_error() const override;
    float get_desired_lat_accel() const override;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // return true if vehicle has reached destination
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // get or set desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;

    // set desired heading and speed
    void set_desired_heading_and_speed(float yaw_angle_cd, float target_speed);

    // set desired heading-delta, turn-rate and speed
    void set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed);
    void set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed);

    // set steering and throttle (-1 to +1).  Only called from scripts
    void set_steering_and_throttle(float steering, float throttle);

    // vehicle start loiter
    bool start_loiter();

    // start stopping
    void start_stop();

    // guided limits
    void limit_set(uint32_t timeout_ms, float horiz_max);
    void limit_clear();
    void limit_init_time_and_location();
    bool limit_breached() const;

protected:

    enum class SubMode: uint8_t {
        WP,
        HeadingAndSpeed,
        TurnRateAndSpeed,
        Loiter,
        SteeringAndThrottle,
        Stop
    };

    // enum for GUID_OPTIONS parameter
    enum class Options : int32_t {
        SCurvesUsedForNavigation = (1U << 6)
    };

    bool _enter() override;

    // returns true if GUID_OPTIONS bit set to use scurve navigation instead of position controller input shaping
    // scurves provide path planning and object avoidance but cannot handle fast updates to the destination (for fast updates use position controller input shaping)
    bool use_scurves_for_navigation() const;

    SubMode _guided_mode;    // stores which GUIDED mode the vehicle is in

    // attitude control
    bool have_attitude_target;  // true if we have a valid attitude target
    uint32_t _des_att_time_ms;  // system time last call to set_desired_attitude was made (used for timeout)
    float _desired_yaw_rate_cds;// target turn rate centi-degrees per second
    bool send_notification;     // used to send one time notification to ground station
    float _desired_speed;       // desired speed used only in HeadingAndSpeed submode

    // direct steering and throttle control
    bool _have_strthr;          // true if we have a valid direct steering and throttle inputs
    uint32_t _strthr_time_ms;   // system time last call to set_steering_and_throttle was made (used for timeout)
    float _strthr_steering;     // direct steering input in the range -1 to +1
    float _strthr_throttle;     // direct throttle input in the range -1 to +1

    // limits
    struct {
        uint32_t timeout_ms;// timeout from the time that guided is invoked
        float horiz_max;    // horizontal position limit in meters from where guided mode was initiated (0 = no limit)
        uint32_t start_time_ms; // system time in milliseconds that control was handed to the external computer
        Location start_loc; // starting location for checking horiz_max limit
    } limit;
};


/**
 * @class ModeHold
 * @brief Emergency stop mode that immediately stops all actuators
 * 
 * @details Hold mode provides an immediate stop capability by commanding zero steering
 *          and zero throttle. Unlike other modes, it does not attempt controlled braking
 *          or position holding - it simply stops all motor outputs.
 *          
 *          Behavior:
 *          - Immediately commands zero throttle to all motors
 *          - Immediately commands zero steering
 *          - No active stabilization or control
 *          - Vehicle will coast to a stop based on momentum and friction
 *          - Does not require position or velocity estimates
 *          
 *          Typical uses:
 *          - Emergency stop via RC switch
 *          - Failsafe action when other modes cannot function
 *          - Testing and calibration (motors off)
 *          - Stopping vehicle when position estimate lost
 *          
 *          Differences from other stop modes:
 *          - Unlike Loiter: Does not hold position (no thrust to counteract drift/wind)
 *          - Unlike Manual with centered sticks: Completely disables outputs
 *          - Unlike stop_vehicle(): No controlled deceleration
 * 
 * @note This mode does not stabilize attitude - vehicle may drift on slopes or in water/wind
 * @warning Vehicle may roll on slopes, drift in water, or be pushed by wind - not for parking
 */
class ModeHold : public Mode
{
public:

    Number mode_number() const override { return Number::HOLD; }
    const char *name4() const override { return "HOLD"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool attitude_stabilized() const override { return false; }

    // hold mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
};

/**
 * @class ModeLoiter
 * @brief Hold current position using active position controller
 * 
 * @details Loiter mode holds the vehicle at its current GPS position (or last known position
 *          if GPS lost). Unlike Hold mode, Loiter actively controls steering and throttle
 *          to maintain position against external forces like wind, water current, or slopes.
 *          
 *          Position holding:
 *          - Records target position on mode entry (current GPS location)
 *          - Continuously calculates position error and commands corrections
 *          - Uses position controller to drive back to target if pushed away
 *          - Maintains heading toward target position
 *          - Respects WP_RADIUS position tolerance parameter
 *          
 *          Active control:
 *          - Steering: Points vehicle toward loiter target
 *          - Throttle: Drives forward/back to reach target position
 *          - Continuously updates based on position feedback
 *          - Pilot can override with stick inputs (if stick mixing enabled)
 *          
 *          Typical uses:
 *          - Hold position for observation or waiting
 *          - Station keeping in wind or current
 *          - Mission loiter waypoints with duration
 *          - Intermediate waypoint holding in complex missions
 *          
 *          Requirements:
 *          - Valid GPS position estimate required
 *          - Velocity estimate recommended for smooth control
 * 
 * @note More appropriate than Hold for outdoor position holding (resists drift)
 * @note Vehicle will drive to maintain position - ensure area is obstacle-free
 */
class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override { return _desired_yaw_cd * 0.01f; }
    float nav_bearing() const override { return _desired_yaw_cd * 0.01f; }
    float crosstrack_error() const override { return 0.0f; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

protected:

    bool _enter() override;

    Location _destination;      // target location to hold position around
    float _desired_speed;       // desired speed (ramped down from initial speed to zero)
};

/**
 * @class ModeManual
 * @brief Direct manual control with no stabilization or automation
 * 
 * @details Manual mode provides completely direct control where pilot RC stick inputs are
 *          passed through to motors with minimal processing. No attitude stabilization,
 *          no position control, no automation - just raw pilot control.
 *          
 *          Control mapping:
 *          - Steering stick: Directly controls steering servo/motor output
 *          - Throttle stick: Directly controls throttle output to drive motors
 *          - Lateral stick (if equipped): Direct lateral movement on omni/mecanum
 *          - No electronic stability or correction
 *          
 *          Processing applied:
 *          - RC input deadzone (to prevent drift from stick centering errors)
 *          - Exponential curve (if configured) for smoother control response
 *          - Servo/motor output scaling and trimming
 *          - Safety limits (SMAX/SMIN for steering, THR_MIN/THR_MAX for throttle)
 *          
 *          Typical uses:
 *          - Initial vehicle testing and tuning
 *          - Maximum pilot authority for expert operators
 *          - RC-only operation without GPS or sensors
 *          - Emergency control when autopilot systems fail
 *          - Demonstrations of raw vehicle dynamics
 *          
 *          Advantages:
 *          - Minimum latency (no control loop processing)
 *          - Works without position/velocity estimates
 *          - Full pilot authority
 *          - Simplest, most predictable behavior
 *          
 *          Limitations:
 *          - No stability assistance
 *          - Requires skilled pilot
 *          - Vehicle may be difficult to control at high speed
 * 
 * @note This is the safest mode to arm in when GPS or sensors are unavailable
 * @note Recommended for initial vehicle setup and testing before using autopilot modes
 */
class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }

    // manual mode does not require position or velocity estimate
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }

protected:

    void _exit() override;
};


/**
 * @class ModeRTL
 * @brief Return To Launch - navigate back to home/launch location
 * 
 * @details RTL (Return To Launch) mode autonomously navigates the vehicle back to the home
 *          position, which is typically where the vehicle was armed (launch location). This
 *          is a critical safety mode for returning the vehicle when RC link is lost or
 *          when commanded by the pilot.
 *          
 *          RTL navigation sequence:
 *          1. Calculate direct path from current position to home location
 *          2. Navigate to home using position controller
 *          3. Optionally navigate via rally points if configured (closer than home)
 *          4. Slow down as approaching home (WP_RADIUS parameter)
 *          5. Stop at home or loiter if RTL_RADIUS > 0
 *          
 *          Home location:
 *          - Automatically set to arming location (can be reset in flight)
 *          - Can be manually set via MAVLink (DO_SET_HOME command)
 *          - Stored as GPS coordinate with altitude
 *          - Displayed on ground station map
 *          
 *          Rally point integration:
 *          - If rally points configured, may navigate to closest rally point instead of home
 *          - Rally points provide alternative safe landing locations
 *          - Controlled via RALLY_* parameters
 *          
 *          Speed control:
 *          - Uses WP_SPEED parameter for cruise speed
 *          - Pilot can nudge speed up/down with throttle stick
 *          - Automatically slows when approaching home (WP_RADIUS)
 *          
 *          Completion behavior:
 *          - If RTL_RADIUS = 0: Stops at home
 *          - If RTL_RADIUS > 0: Loiters around home at specified radius
 *          
 *          Typical uses:
 *          - Failsafe action on RC signal loss
 *          - Commanded return after mission completion
 *          - Manual activation via mode switch
 *          - Battery failsafe action
 * 
 * @note Requires valid GPS position and home location set
 * @warning Flies direct path - ensure area between current position and home is obstacle-free
 * @warning Cannot be armed in RTL mode - prevents accidental activation on ground
 */
class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name4() const override { return "RTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

protected:

    bool _enter() override;

    bool send_notification; // used to send one time notification to ground station
    bool _loitering;        // true if loitering at end of RTL

};

/**
 * @class ModeSmartRTL
 * @brief Return via recorded path - retrace the path vehicle took to current location
 * 
 * @details Smart RTL (SRTL) records the vehicle's path during flight and returns to home by
 *          retracing this path in reverse. This is safer than direct RTL when obstacles or
 *          difficult terrain lie between current position and home, since the return path is
 *          known to be traversable.
 *          
 *          Path recording:
 *          - Continuously records vehicle position while driving
 *          - Stores path points in AP_SmartRTL library
 *          - Simplifies path to save memory (removes intermediate points on straight segments)
 *          - Limited by SRTL_POINTS parameter (memory allocation)
 *          - Recording occurs in all modes except SmartRTL itself
 *          
 *          Return navigation:
 *          1. Loads recorded path points from AP_SmartRTL
 *          2. Reverses path direction (last point becomes first waypoint)
 *          3. Navigates to each path point in sequence
 *          4. Uses position controller for smooth tracking
 *          5. Stops at home location
 *          
 *          Path simplification:
 *          - Removes points that lie on straight lines (within tolerance)
 *          - Prioritizes recent path points if memory full
 *          - Maintains critical waypoints (corners, obstacles)
 *          
 *          Failover behavior:
 *          - If no path recorded yet: Falls back to regular RTL
 *          - If path recording overflows: Returns via available points then direct to home
 *          - If position estimate lost during return: Stops vehicle (safety)
 *          
 *          Memory management:
 *          - SRTL_POINTS parameter sets maximum path points (default 200)
 *          - Each point consumes ~12 bytes
 *          - Path cleared on disarm or mode change from SRTL
 *          
 *          Advantages over RTL:
 *          - Avoids unknown obstacles between current position and home
 *          - Follows known-good path
 *          - Better for complex environments (urban, forest, rocks)
 *          
 *          Typical uses:
 *          - Exploration missions where return path may have obstacles
 *          - Search and rescue in complex terrain
 *          - Failsafe mode when direct path unsafe
 *          - Off-road navigation with path memory
 * 
 * @note Requires valid GPS position throughout operation
 * @note Path recording starts after first GPS lock and continues until disarm
 * @warning Path memory is limited - long missions may overflow and fall back to RTL
 * @warning Cannot be armed in SmartRTL mode
 */
class ModeSmartRTL : public Mode
{
public:

    Number mode_number() const override { return Number::SMART_RTL; }
    const char *name4() const override { return "SRTL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED;

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }
    bool reached_destination() const override { return smart_rtl_state == SmartRTLState::StopAtHome; }

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

    // save current position for use by the smart_rtl flight mode
    void save_position();

protected:

    // Safe RTL states
    enum class SmartRTLState: uint8_t {
        WaitForPathCleanup,
        PathFollow,
        StopAtHome,
        Failure
    } smart_rtl_state;

    bool _enter() override;
    bool _load_point;
    bool _loitering;        // true if loitering at end of SRTL
};



/**
 * @class ModeSteering
 * @brief Manual steering with automated speed control
 * 
 * @details Steering mode provides a hybrid control scheme where the pilot manually controls
 *          steering direction while the autopilot maintains the target speed. This simplifies
 *          driving by removing the need to constantly adjust throttle.
 *          
 *          Control behavior:
 *          - Steering stick: Pilot directly controls steering (like Manual mode)
 *          - Throttle stick: Sets target speed (autopilot maintains this speed)
 *          - Speed controller automatically adjusts throttle to maintain target
 *          - Pilot can override speed with throttle stick at any time
 *          
 *          Speed control:
 *          - Throttle stick position sets desired speed in m/s
 *          - Centered stick: Stop (zero speed)
 *          - Forward stick: Forward motion (0 to CRUISE_SPEED)
 *          - Back stick: Reverse motion (0 to -CRUISE_SPEED)
 *          - Speed controller compensates for slopes, wind, load changes
 *          
 *          Steering control:
 *          - Direct steering like Manual mode (no stabilization)
 *          - Lateral acceleration limiting for safety (respects TURN_MAX_G)
 *          - Prevents rollovers on high-speed turns
 *          
 *          Typical uses:
 *          - Learning mode between Manual and full autopilot
 *          - Precision speed control during manual driving
 *          - Constant-speed operations (surveys, demonstrations)
 *          - Reduced pilot workload on long manual drives
 *          
 *          Requirements:
 *          - Velocity estimate required for speed control
 *          - Position not required (can work without GPS)
 * 
 * @note Easier to drive than Manual mode on slopes or with varying loads
 * @note Lateral acceleration limiting provides some safety vs pure Manual
 */
class ModeSteering : public Mode
{
public:

    Number mode_number() const override { return Number::STEERING; }
    const char *name4() const override { return "STER"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }

    // steering requires velocity but not position
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return true; }

    // return desired lateral acceleration
    float get_desired_lat_accel() const override { return _desired_lat_accel; }

private:

    float _desired_lat_accel;   // desired lateral acceleration calculated from pilot steering input
};

/**
 * @class ModeInitializing
 * @brief Temporary boot-up mode before EKF initializes
 * 
 * @details Initializing mode is the default mode immediately after power-on before the
 *          Extended Kalman Filter (EKF) has established a position and velocity estimate.
 *          Vehicle remains in this mode until EKF origin is set and navigation estimates
 *          are available.
 *          
 *          Behavior:
 *          - Does nothing (empty update() method)
 *          - Prevents any motor output
 *          - Waits for EKF to initialize
 *          - Automatically transitions to configured startup mode once ready
 *          
 *          EKF initialization requirements:
 *          - GPS lock acquired (typically 6+ satellites)
 *          - IMU calibrated and stable
 *          - Compass calibrated (if used)
 *          - Barometer initialized
 *          - EKF origin set (reference point for local navigation)
 *          
 *          Transition behavior:
 *          - Once EKF ready: Automatically switches to INITIAL_MODE parameter
 *          - If INITIAL_MODE requires position but no GPS: May switch to Manual/Hold
 *          - Typical transition time: 5-30 seconds after power-on
 *          
 *          Restrictions:
 *          - Cannot arm in Initializing mode (no navigation available)
 *          - No motor outputs allowed
 *          - Mode cannot be manually selected by pilot
 *          
 *          Indicators:
 *          - LED patterns show "waiting for GPS" or "EKF initializing"
 *          - GCS displays "Initializing" mode
 *          - Pre-arm checks will fail until initialization complete
 * 
 * @note This is a transient mode - vehicle should exit automatically within seconds
 * @warning If stuck in Initializing mode, check GPS lock and sensor health
 * @warning Cannot be armed - this is intentional for safety
 */
class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name4() const override { return "INIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    // do not allow arming from this mode
    bool allows_arming() const override { return false; }

    // attributes for mavlink system status reporting
    bool has_manual_input() const override { return true; }
    bool attitude_stabilized() const override { return false; }
protected:
    bool _enter() override { return false; };
};

#if MODE_FOLLOW_ENABLED
/**
 * @class ModeFollow
 * @brief Follow another vehicle using position telemetry
 * 
 * @details Follow mode enables the rover to autonomously follow another vehicle (lead vehicle)
 *          by processing position telemetry received via MAVLink. The rover maintains a specified
 *          distance and bearing offset from the lead vehicle.
 *          
 *          Architecture:
 *          - Uses AP_Follow library for lead vehicle tracking
 *          - Processes GLOBAL_POSITION_INT MAVLink messages from lead vehicle
 *          - Calculates desired position relative to lead vehicle
 *          - Uses standard position controller to reach target
 *          
 *          Follow parameters (FOLL_ prefix):
 *          - FOLL_ENABLE: Enable/disable follow mode
 *          - FOLL_SYSID: MAVLink system ID of lead vehicle
 *          - FOLL_DIST_MAX: Maximum distance to follow (meters)
 *          - FOLL_OFS_X/Y/Z: Position offset from lead vehicle (NED frame)
 *          - FOLL_ALT_TYPE: Altitude offset type (relative vs absolute)
 *          - FOLL_YAW_BEHAVE: Yaw behavior (face lead, flight direction, or maintain offset)
 *          
 *          Operational behavior:
 *          - Continuously receives lead vehicle position at ~1-10 Hz
 *          - Calculates target position: lead_position + configured_offset
 *          - Drives to target position using position controller
 *          - Maintains offset as lead vehicle moves
 *          - Stops if telemetry lost (safety failsafe)
 *          
 *          Follow algorithm:
 *          1. Receive GLOBAL_POSITION_INT from lead vehicle
 *          2. Transform lead position to local NED frame
 *          3. Apply configured X/Y offset in lead vehicle's frame or earth frame
 *          4. Calculate bearing and distance to target position
 *          5. Generate throttle and steering commands to reach target
 *          6. Respect speed limits and obstacle avoidance (if configured)
 *          
 *          Yaw behavior modes:
 *          - FACE_LEAD: Rover always points toward lead vehicle
 *          - SAME_AS_LEAD: Rover matches lead vehicle's heading
 *          - DIRECTION_OF_FLIGHT: Rover points in direction of travel
 *          
 *          Safety features:
 *          - Maximum follow distance limit (FOLL_DIST_MAX)
 *          - Telemetry timeout failsafe (stops if no updates for FOLL_TIMEOUT seconds)
 *          - Option to enable avoidance systems while following
 *          - Manual override available via RC stick input (if stick mixing enabled)
 *          
 *          Telemetry requirements:
 *          - Lead vehicle must broadcast GLOBAL_POSITION_INT messages
 *          - Requires MAVLink system ID configuration
 *          - Recommend 2-5 Hz update rate minimum
 *          - Both vehicles need GPS lock for accurate following
 *          
 *          Coordinate frame handling:
 *          - Lead vehicle position received in global lat/lon/alt
 *          - Converted to local NED frame relative to rover
 *          - Offset applied in configured frame (lead vehicle frame or earth frame)
 *          - Target position fed to position controller
 *          
 *          Typical use cases:
 *          - Following a lead rover in convoy operations
 *          - Following a person carrying a transmitter
 *          - Following a boat or vehicle from shore/water
 *          - Automated formation following
 * 
 * @note Requires valid FOLL_ENABLE and FOLL_SYSID configuration
 * @note Lead vehicle must transmit position at sufficient rate (>1 Hz recommended)
 * @warning Stops if telemetry from lead vehicle is lost for FOLL_TIMEOUT seconds
 * @warning Requires both vehicles to have GPS lock for accurate following
 */
class ModeFollow : public Mode
{
public:

    Number mode_number() const override { return Number::FOLLOW; }
    const char *name4() const override { return "FOLL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // attributes of the mode
    bool is_autopilot_mode() const override { return true; }

    // return desired heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
    float wp_bearing() const override;
    float nav_bearing() const override { return wp_bearing(); }
    float crosstrack_error() const override { return 0.0f; }

    // return desired location
    bool get_desired_location(Location& destination) const override WARN_IF_UNUSED { return false; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override;

    // set desired speed in m/s
    bool set_desired_speed(float speed) override;

protected:

    bool _enter() override;
    void _exit() override;

    float _desired_speed;       // desired speed in m/s
};
#endif

/**
 * @class ModeSimple
 * @brief Simplified manual control with heading locked to initial direction
 * 
 * @details Simple mode provides intuitive manual control where the pilot's left/right stick input
 *          controls the vehicle's movement relative to its initial heading direction, rather than
 *          the vehicle's current orientation. This makes the rover easier to control for beginners
 *          and in situations where the rover's orientation is hard to track visually.
 *          
 *          Control concept:
 *          - Forward stick always drives "away" from pilot (based on initial heading)
 *          - Backward stick always drives "toward" pilot
 *          - Left/right stick steers left/right relative to pilot's perspective
 *          - Vehicle orientation becomes irrelevant to pilot input interpretation
 *          
 *          Initial heading lock:
 *          - Initial direction recorded when mode is entered
 *          - Can be reset by switching out and back into Simple mode
 *          - Heading reference remains fixed until mode exit
 *          - Uses compass/EKF heading at mode entry time
 *          
 *          Input transformation:
 *          1. Read pilot stick inputs (throttle and steering channels)
 *          2. Get current vehicle heading from EKF
 *          3. Calculate heading error: current_heading - initial_heading
 *          4. Rotate pilot input vector by -heading_error
 *          5. Apply transformed inputs to steering and throttle controllers
 *          
 *          Mathematical transformation:
 *          - Pilot inputs treated as vector: [throttle_input, steering_input]
 *          - Vector rotated by (initial_heading - current_heading) angle
 *          - Result applied to vehicle control
 *          - Effectively creates "earth frame" control from "body frame" input
 *          
 *          Behavior comparison to Manual mode:
 *          - Manual: Right stick → vehicle turns right regardless of orientation
 *          - Simple: Right stick → vehicle moves to pilot's right relative to initial heading
 *          - Manual: Forward → vehicle drives forward based on its nose direction
 *          - Simple: Forward → vehicle drives away from pilot based on initial heading
 *          
 *          Advantages:
 *          - Easier for new pilots to learn
 *          - Intuitive control when vehicle is far away
 *          - Useful when vehicle orientation is hard to see
 *          - Reduces confusion during complex maneuvers
 *          
 *          Limitations:
 *          - Requires compass for heading reference
 *          - Can be confusing if initial heading not aligned with pilot
 *          - Heading reference does not update during mode (must re-enter to reset)
 *          - Not suitable for precision docking or tight spaces
 *          
 *          Failsafe behavior:
 *          - If compass fails, may revert to Manual mode or trigger failsafe
 *          - RC failsafe applies normally
 *          - Manual override possible by switching modes
 *          
 *          Typical use cases:
 *          - Training new rover pilots
 *          - Long-distance manual driving
 *          - FPV operation where orientation is unclear
 *          - Operating in open spaces without obstacles
 * 
 * @note Requires valid compass/heading estimate for proper operation
 * @note Initial heading locked at mode entry - switch modes to reset reference
 * @warning May be disorienting if entered when vehicle not aligned with pilot's perspective
 */
class ModeSimple : public Mode
{
public:

    Number mode_number() const override { return Number::SIMPLE; }
    const char *name4() const override { return "SMPL"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void init_heading();

    // simple type enum used for SIMPLE_TYPE parameter
    enum simple_type {
        Simple_InitialHeading = 0,
        Simple_CardinalDirections = 1,
    };

private:

    float _initial_heading_cd;  // vehicle heading (in centi-degrees) at moment vehicle was armed
    float _desired_heading_cd;  // latest desired heading (in centi-degrees) from pilot
};

#if MODE_DOCK_ENABLED
/**
 * @class ModeDock
 * @brief Precision autonomous docking using vision/beacon guidance
 * 
 * @details Dock mode provides high-precision autonomous docking capability for rovers that need
 *          to return to a specific location for charging, loading, or parking. The mode uses
 *          external position sources (such as AprilTags via vision systems or IR beacons) to
 *          achieve centimeter-level positioning accuracy at the docking target.
 *          
 *          Docking system architecture:
 *          - External sensor provides docking target position (typically AprilTag detection)
 *          - Position reported via MAVLink LANDING_TARGET message or similar
 *          - Mode calculates approach vector and controls vehicle to dock
 *          - Precision slowdown as vehicle approaches target
 *          - Completion detection when within acceptable error threshold
 *          
 *          Approach algorithm:
 *          1. Receive docking target position from external sensor (vision/beacon)
 *          2. Calculate position error: current_position - dock_target_position
 *          3. Calculate desired heading: align with configured approach direction
 *          4. Apply heading correction if enabled (DOCK_HDG_CORR_EN)
 *          5. Calculate speed: Apply progressive slowdown based on distance
 *          6. Generate throttle and steering commands
 *          7. Stop when within DOCK_STOP_DIST threshold
 *          
 *          Speed control with slowdown:
 *          - Far from dock (>5m): Use full DOCK_SPEED
 *          - Approaching dock (1-5m): Progressive slowdown based on distance and lateral error
 *          - Near dock (<1m): Very slow final approach (crawl speed)
 *          - At dock (<stopping_dist): Stop and mark complete
 *          
 *          Heading correction:
 *          - If DOCK_HDG_CORR_EN enabled, vehicle aligns with DOCK_DIR parameter
 *          - Correction weight controlled by DOCK_HDG_CORR_WT (0-1.0)
 *          - Balance between following dock vector and aligning to desired orientation
 *          - Useful for ensuring correct docking orientation (e.g., charging contacts aligned)
 *          
 *          Position sources:
 *          - AprilTag vision system (most common)
 *          - IR beacon positioning
 *          - Optical tracking system
 *          - Precision GPS with RTK
 *          - Any system capable of providing LANDING_TARGET messages
 *          
 *          Coordinate frame handling:
 *          - Dock position received relative to vehicle or EKF origin
 *          - calc_dock_pos_rel_vehicle_NE() computes dock vector in NE (North-East) frame
 *          - Approach vector calculated in local NE coordinates
 *          - Steering and throttle commands generated from position/heading error
 *          
 *          Two-phase approach strategy:
 *          - Phase 1 (far): Navigate using filtered dock position estimates
 *          - Phase 2 (near): Force use of real-time dock target when <3m away
 *          - Transition triggered by _force_real_target_limit_cm threshold
 *          - Ensures final approach uses most current sensor data
 *          
 *          Lateral error handling:
 *          - Monitors lateral offset from ideal approach line
 *          - Acceptable lateral error: _acceptable_pos_error_cm (20 cm default)
 *          - Slows down more aggressively if lateral error exceeds threshold
 *          - Prevents overshooting or side-swiping dock
 *          
 *          Parameters (DOCK_ prefix):
 *          - DOCK_SPEED: Approach speed in m/s (reduced automatically near dock)
 *          - DOCK_DIR: Desired approach direction in degrees (0-360)
 *          - DOCK_HDG_CORR_EN: Enable heading correction (0/1)
 *          - DOCK_HDG_CORR_WT: Heading correction weight (0.0-1.0)
 *          - DOCK_STOP_DIST: Distance in meters at which to stop (completion threshold)
 *          
 *          Completion detection:
 *          - Sets _docking_complete flag when within DOCK_STOP_DIST
 *          - Vehicle stops all motion
 *          - Can transition to loiter or hold after completion
 *          - Sends completion notification to GCS
 *          
 *          Sensor requirements:
 *          - Must receive regular LANDING_TARGET messages (>1 Hz recommended)
 *          - Position accuracy: <10 cm recommended for reliable docking
 *          - Sensor must track dock throughout approach (no blind spots)
 *          - Reliable detection to at least 10m range recommended
 *          
 *          Safety features:
 *          - Progressive slowdown prevents collision
 *          - Lateral error monitoring prevents side approaches
 *          - Sensor loss detection triggers failsafe (stops or switches mode)
 *          - Manual override available via RC or mode switch
 *          
 *          Typical use cases:
 *          - Autonomous charging station docking
 *          - Precision parking at loading/unloading stations
 *          - Automated garage entry
 *          - Docking with mobile platforms (e.g., trailer, boat)
 *          - Warehouse automation with precise positioning
 *          
 *          Integration with missions:
 *          - Can be triggered from AUTO mode via DO_SET_MODE command
 *          - Often used as final step in mission sequence
 *          - Compatible with DO_LAND_START for landing sequence
 *          
 *          Compilation:
 *          - Enabled via MODE_DOCK_ENABLED feature flag
 *          - May be disabled on low-memory boards
 *          - Check board hwdef for availability
 * 
 * @note Requires external position source providing LANDING_TARGET messages
 * @note Progressive slowdown ensures safe final approach
 * @warning Sensor position accuracy directly affects docking precision
 * @warning Ensure DOCK_DIR aligns with physical dock orientation
 * @warning Test thoroughly in safe environment before operational use
 * @warning Sensor loss will cause docking failure - ensure reliable detection
 */
class ModeDock : public Mode
{
public:

    // need a constructor for parameters
    ModeDock(void);

    // Does not allow copies
    CLASS_NO_COPY(ModeDock);

    Number mode_number() const override { return Number::DOCK; }
    const char *name4() const override { return "DOCK"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool is_autopilot_mode() const override { return true; }

    // return distance (in meters) to destination
    float get_distance_to_destination() const override { return _distance_to_destination; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float speed; // dock mode speed
    AP_Float desired_dir; // desired direction of approach
    AP_Int8 hdg_corr_enable; // enable heading correction
    AP_Float hdg_corr_weight; // heading correction weight
    AP_Float stopping_dist; // how far away from the docking target should we start stopping

    bool _enter() override;

    // return reduced speed of vehicle based on error in position and current distance from the dock
    float apply_slowdown(float desired_speed);

    // calculate position of dock relative to the vehicle
    bool calc_dock_pos_rel_vehicle_NE(Vector2f &dock_pos_rel_vehicle) const;

    // we force the vehicle to use real dock target vector when this much close to the docking station
    const float _force_real_target_limit_cm = 300.0f;
    // acceptable lateral error in vehicle's position with respect to dock. This is used while slowing down the vehicle
    const float _acceptable_pos_error_cm = 20.0f;

    Vector2f _dock_pos_rel_origin_cm;   // position vector towards docking target relative to ekf origin
    Vector2f _desired_heading_NE;       // unit vector in desired direction of docking
    bool _docking_complete = false;     // flag to mark docking complete when we are close enough to the dock
    bool _loitering = false; // true if we are loitering after mission completion
};
#endif
