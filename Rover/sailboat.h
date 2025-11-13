/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file sailboat.h
 * @brief Sailboat-specific control for marine vehicles with wind propulsion
 * 
 * @details This file implements sailboat functionality for Rover, including:
 * - Sail trimming algorithms based on apparent wind angle
 * - Tacking logic for upwind navigation (beating to windward)
 * - Motor assist policy for low wind conditions and slow tacks
 * - Wind angle calculations and no-go zone handling
 * - Velocity Made Good (VMG) calculations
 * - Sail angle control for mainsails, wingsails, and rotating masts
 * - Detection and handling of "in irons" condition (head-to-wind stall)
 * 
 * The sailboat controller integrates with the standard Rover navigation
 * system to provide autonomous sailboat operation. It handles the unique
 * characteristics of wind-powered navigation including tacking through
 * the wind when the destination lies upwind.
 * 
 * @note Requires sailboat-specific frame type configuration and appropriate
 *       parameter setup for sail angle limits, no-go zone, and motor assist.
 * 
 * Source: Rover/sailboat.h
 */

/**
 * @class Sailboat
 * @brief Controller for sailboat-specific navigation and sail management
 * 
 * @details The Sailboat class provides autonomous control for wind-powered
 * marine vehicles. Key responsibilities include:
 * 
 * - **Sail Control**: Automatically trims sails based on apparent wind angle
 *   to optimize thrust while respecting angle limits and heel constraints
 * 
 * - **Tacking Management**: Implements tacking state machine to navigate
 *   upwind by alternating between port and starboard tacks. Detects when
 *   the destination lies within the no-go zone and calculates optimal
 *   tacking angles.
 * 
 * - **Motor Assist**: Manages motor usage according to USE_MOTOR parameter:
 *   - NEVER: Pure sailing mode
 *   - ASSIST: Motor activates for low wind or slow tacks
 *   - ALWAYS: Constant motor thrust
 * 
 * - **Wind Angle Calculations**: Computes apparent wind angle from true wind
 *   and vehicle motion, accounting for the no-go zone where sailing directly
 *   into wind is impossible.
 * 
 * - **VMG Optimization**: Calculates Velocity Made Good (speed toward
 *   destination) to optimize routing when tacking.
 * 
 * The class integrates with RC input for manual sail control and provides
 * autonomous sail management for Auto, Guided, RTL, and other navigation modes.
 * 
 * @note Thread Safety: Methods called from main control loop at 50Hz rate
 * @warning Sail control timing is critical - improper sail angles during
 *          tacking can cause the boat to lose steerage or capsize in strong winds
 * 
 * Source: Rover/sailboat.h:19-119
 */
class Sailboat
{
public:

    /**
     * @brief Constructor for Sailboat controller
     * 
     * @details Initializes sailboat control system with default values.
     * Tacking state is cleared and motor state initialized.
     */
    Sailboat();

    /**
     * @brief Check if sailboat functionality is enabled
     * 
     * @return true if sailboat mode is enabled (enable parameter > 0)
     * @return false if sailboat mode is disabled
     * 
     * @note This is checked before applying sailboat-specific control logic
     */
    bool sail_enabled() const { return enable > 0;}

    /**
     * @brief Check if autonomous tacking (sailboat navigation) is enabled
     * 
     * @details Tacking enables the sailboat to navigate upwind by following
     * an indirect route with alternating port and starboard tacks.
     * 
     * @return true if tacking is enabled and sailboat can navigate upwind
     * @return false if tacking disabled - boat cannot sail into no-go zone
     * 
     * @note When disabled, destinations in the no-go zone are unreachable
     */
    bool tack_enabled() const;

    /**
     * @brief Initialize sailboat controller
     * 
     * @details Sets up sailboat-specific functionality including:
     * - Loading parameters from storage
     * - Initializing RC input channels
     * - Clearing tacking state
     * - Setting initial motor state
     * 
     * @note Called during Rover initialization sequence
     */
    void init();

    /**
     * @brief Initialize RC input channel for mainsail control
     * 
     * @details Locates and configures the RC channel assigned to mainsail
     * control (typically channel 8 or AUX function). Validates channel
     * is available and properly configured.
     * 
     * @note If mainsail channel not found, autonomous sail control still works
     *       but manual override via RC will not be available
     */
    void init_rc_in();

    /**
     * @brief Calculate throttle output and set mainsail position
     * 
     * @details This is the main control function called each cycle. It:
     * 1. Determines if motor assist is needed based on wind speed and tacking state
     * 2. Calculates appropriate throttle output
     * 3. Trims mainsail based on apparent wind angle using sail trimming algorithm
     * 4. Applies sail angle limits and heel angle constraints
     * 
     * The sail trimming algorithm sheets in the sail progressively as apparent
     * wind angle increases from head-to-wind, achieving optimal angle of attack
     * for the sail airfoil while avoiding stall or luffing.
     * 
     * @param[in] desired_speed Target speed in m/s for motor assist calculations
     * @param[out] throttle_out Calculated throttle value (-100 to +100)
     * 
     * @note Called at main loop rate (typically 50Hz)
     * @warning Rapid sail movements during tacking can cause stability issues
     * 
     * Source: Rover/sailboat.cpp
     */
    void get_throttle_and_set_mainsail(float desired_speed, float &throttle_out);

    /**
     * @brief Calculate Velocity Made Good toward destination
     * 
     * @details VMG is the component of boat velocity in the direction of the
     * destination. When tacking upwind, VMG is less than boat speed since
     * the boat sails at an angle to the destination. This metric is used to
     * optimize tacking angles and determine the most efficient route.
     * 
     * Formula: VMG = boat_speed * cos(angle_to_destination)
     * 
     * @return VMG in m/s (positive when making progress, negative when moving away)
     * 
     * @note Critical for upwind performance optimization
     * @see use_indirect_route(), calc_heading()
     */
    float get_VMG() const;

    /**
     * @brief Handle user-initiated tack request in Acro mode
     * 
     * @details In Acro (manual) mode, the pilot can initiate a tack by using
     * the tack input. This initiates the tacking state machine which:
     * 1. Calculates target heading on opposite tack
     * 2. Engages motor assist if configured (to help boat through the wind)
     * 3. Monitors progress through the tack
     * 4. Releases control when tack completes
     * 
     * @note Acro tacks are pilot-initiated but system-assisted
     * @warning Ensure adequate water depth and clearance before tacking
     * 
     * Source: Rover/sailboat.cpp
     */
    void handle_tack_request_acro();

    /**
     * @brief Get target heading during tacking maneuver
     * 
     * @details Returns the heading in radians that the boat should steer
     * toward during an active tack. Only valid when tacking() returns true.
     * 
     * @return Target heading in radians (0 to 2*PI, 0=North, PI/2=East)
     * 
     * @note Only used in Acro mode - autonomous modes use their own navigation
     * @see tacking(), handle_tack_request_acro()
     */
    float get_tack_heading_rad();

    /**
     * @brief Handle user-initiated tack request in autonomous modes
     * 
     * @details In Auto, Guided, RTL, SmartRTL modes, the pilot can manually
     * initiate a tack. This is useful when:
     * - The vehicle is "in irons" (stalled head-to-wind) and needs assistance
     * - The pilot wants to tack earlier than the automatic logic would
     * - Testing or troubleshooting tacking behavior
     * 
     * The system calculates the new tack heading and temporarily overrides
     * the navigation controller until the tack completes.
     * 
     * @note Tack direction (port/starboard) chosen to get closer to destination
     * @warning Manual tack requests can conflict with geofence or mission waypoints
     * 
     * Source: Rover/sailboat.cpp
     */
    void handle_tack_request_auto();

    /**
     * @brief Clear tacking state and resume normal navigation
     * 
     * @details Resets all tacking state variables:
     * - currently_tacking flag cleared
     * - Tack heading reset
     * - Motor assist released
     * - Timestamps cleared
     * 
     * Called automatically when tack completes or can be called to abort a tack.
     * 
     * @note Safe to call even if not currently tacking
     */
    void clear_tack();

    /**
     * @brief Check if boat is currently executing a tack
     * 
     * @details The tacking state is active from when tack is initiated until
     * the boat achieves the target heading on the new tack. During tacking:
     * - Motor assist may be engaged
     * - Special navigation logic applies
     * - Sail control follows tacking sequence
     * 
     * @return true if actively tacking
     * @return false if normal sailing
     * 
     * @see clear_tack(), handle_tack_request_acro(), handle_tack_request_auto()
     */
    bool tacking() const;

    /**
     * @brief Check if destination requires indirect tacking route
     * 
     * @details Determines if the desired heading lies within the no-go zone
     * (directly upwind) where the boat cannot sail. If true, the boat must
     * tack back and forth on port and starboard tacks to make progress upwind.
     * 
     * The no-go zone is typically 45-50 degrees either side of the wind direction,
     * configured by the SAIL_NO_GO parameter.
     * 
     * @param[in] desired_heading_cd Desired heading in centidegrees (0-36000)
     * 
     * @return true if destination is upwind and requires tacking
     * @return false if destination can be reached on single heading
     * 
     * @note Only returns true if tacking is enabled
     * @see tack_enabled(), calc_heading()
     */
    bool use_indirect_route(float desired_heading_cd) const;

    /**
     * @brief Calculate heading to sail when destination is upwind
     * 
     * @details When the destination lies in the no-go zone, this calculates
     * the optimal sailing heading. The algorithm:
     * 1. Determines which tack (port or starboard) provides better VMG
     * 2. Calculates heading at edge of no-go zone on that tack
     * 3. Accounts for wind shifts and boat leeway
     * 
     * The boat will sail this heading until it needs to tack, making
     * a zigzag course toward the upwind destination.
     * 
     * @param[in] desired_heading_cd True heading to destination in centidegrees
     * 
     * @return Achievable sailing heading in centidegrees accounting for no-go zone
     * 
     * @note Returns desired_heading_cd unchanged if not in no-go zone
     * @warning Frequent tacking in tight spaces may not allow enough distance to
     *          build speed between tacks
     * 
     * Source: Rover/sailboat.cpp
     */
    float calc_heading(float desired_heading_cd);

    /**
     * @enum UseMotor
     * @brief Motor usage policy for sailboat operations
     * 
     * @details Controls when the propulsion motor is activated:
     * 
     * - **USE_MOTOR_NEVER** (0): Pure sailing mode. Motor never activates
     *   regardless of wind conditions. Use for racing or pure sailing operation.
     * 
     * - **USE_MOTOR_ASSIST** (1): Motor activates automatically when:
     *   * Wind speed drops below SAIL_WNDSPD_MIN threshold
     *   * Boat is tacking and making slow progress
     *   * Boat is "in irons" (stalled head-to-wind)
     *   Motor disengages automatically when conditions improve.
     * 
     * - **USE_MOTOR_ALWAYS** (2): Motor runs continuously at speed calculated
     *   to achieve desired_speed. Sail still trims automatically. Use for
     *   motor-sailing in light winds or when schedule is critical.
     * 
     * @note Configurable via USE_MOTOR parameter
     * @see set_motor_state(), motor_assist_low_wind(), motor_assist_tack()
     */
    enum class UseMotor {
        USE_MOTOR_NEVER  = 0,
        USE_MOTOR_ASSIST = 1,
        USE_MOTOR_ALWAYS = 2
    };

    /**
     * @brief Set motor usage state
     * 
     * @details Changes the motor state according to UseMotor policy. Can be
     * called to:
     * - Force motor on/off for testing
     * - Respond to parameter changes
     * - Override for emergency situations
     * 
     * State changes are logged and optionally reported to ground stations.
     * 
     * @param[in] state New motor state (NEVER, ASSIST, or ALWAYS)
     * @param[in] report_failure If true, sends MAVLink status message to GCS
     * 
     * @note State persists until changed by another call or parameter update
     * @warning Setting NEVER in zero wind conditions will leave boat adrift
     */
    void set_motor_state(UseMotor state, bool report_failure = true);

    /**
     * @brief Parameter table for sailboat configuration
     * 
     * @details Defines all sailboat-specific parameters including:
     * - SAIL_ENABLE: Enable/disable sailboat functionality
     * - SAIL_ANGLE_MIN: Minimum mainsail angle (fully sheeted in)
     * - SAIL_ANGLE_MAX: Maximum mainsail angle (fully sheeted out)
     * - SAIL_ANGLE_IDEAL: Ideal mainsail angle for close-hauled sailing
     * - SAIL_HEEL_MAX: Maximum heel angle before easing sails
     * - SAIL_NO_GO: No-go zone angle either side of wind (degrees)
     * - SAIL_WNDSPD_MIN: Wind speed threshold for motor assist (m/s)
     * - SAIL_XTRACK_MAX: Maximum cross-track error before tacking (meters)
     * - SAIL_LOIT_RADIUS: Loiter radius for sailboat (meters)
     * 
     * @note Parameters loaded from EEPROM during init()
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get sailboat-specific loiter radius
     * 
     * @details Returns the loiter radius configured for sailboat operation.
     * Typically larger than standard rover loiter radius to account for
     * tacking maneuvers and wind drift.
     * 
     * @return Loiter radius in meters
     * 
     * @note Used by navigation controller for loiter mode
     */
    float get_loiter_radius() const {return loit_radius;}

    /**
     * @brief Set mainsail position from pilot RC input
     * 
     * @details In manual control modes (Acro, Manual), reads the mainsail RC
     * channel and directly controls sail position. Pilot has direct control
     * of sail trim, overriding automatic sail control algorithms.
     * 
     * Sail angle is scaled from RC input (1000-2000 μs) to configured
     * min/max angles, with center stick corresponding to ideal angle.
     * 
     * @note Only active when valid RC input is present on mainsail channel
     * @warning Pilot must understand sail trim - improper angles can cause
     *          poor performance or safety issues
     * 
     * Source: Rover/sailboat.cpp
     */
    void set_pilot_desired_mainsail();

    /**
     * @brief Set mainsail position in autonomous modes
     * 
     * @details Automatically calculates and sets optimal mainsail trim based on:
     * - Apparent wind angle
     * - Desired speed
     * - Heel angle (eases sail if heeling too far)
     * - Sail angle limits
     * 
     * Implements sail trimming algorithm that progressively sheets in the
     * sail as apparent wind angle increases from head-to-wind, optimizing
     * the angle of attack for the sail airfoil. Accounts for point of sail:
     * - Close-hauled: Sail sheeted in tight (small angle)
     * - Beam reach: Sail partially eased
     * - Broad reach/run: Sail fully eased (large angle)
     * 
     * @param[in] desired_speed Target speed in m/s for speed-based trim adjustment
     * 
     * @note Called by get_throttle_and_set_mainsail() in autonomous modes
     * @warning Excessive heel angle will cause sails to ease automatically,
     *          reducing thrust but improving stability
     * 
     * Source: Rover/sailboat.cpp
     */
    void set_auto_mainsail(float desired_speed);

    /**
     * @brief Stop sails from driving the boat
     * 
     * @details Moves sails to positions that minimize thrust:
     * - For traditional sails: Sheet out to maximum angle (luffing)
     * - For wingsails: Feather to neutral position (aligned with wind)
     * 
     * Used in emergency situations, when stopped in a mode, or when
     * transitioning between modes. Prevents unwanted motion while
     * maintaining some control authority.
     * 
     * @note Motor may still provide thrust if motor_state is ALWAYS
     * @see set_motor_state()
     */
    void relax_sails();

private:

    /**
     * @brief Check if motor assist is needed for current tack
     * 
     * @details Determines if the motor should activate to help the boat tack.
     * Motor assistance is triggered when:
     * - Currently tacking (in tacking state machine)
     * - Motor state is ASSIST
     * - Boat is making slow progress through the tack
     * - Risk of getting stuck "in irons" (head-to-wind stall)
     * 
     * Motor typically activates briefly as the boat passes through head-to-wind
     * to maintain steerage and ensure the boat completes the tack.
     * 
     * @return true if motor should activate to assist tacking maneuver
     * @return false if motor assist not needed or not configured
     * 
     * @note Monitored continuously during tacking sequence
     * @see motor_assist_low_wind(), set_motor_state()
     */
    bool motor_assist_tack() const;

    /**
     * @brief Check if motor assist is needed due to low wind
     * 
     * @details Determines if wind speed is below the threshold requiring motor
     * assistance. Compares current wind speed to SAIL_WNDSPD_MIN parameter.
     * When wind is too low:
     * - Boat cannot maintain steerage
     * - Cannot make reasonable progress
     * - Risk of drifting into obstacles
     * 
     * Motor activates automatically if motor_state is ASSIST.
     * 
     * @return true if wind speed below threshold and motor assist configured
     * @return false if wind adequate or motor assist not configured
     * 
     * @note Wind speed obtained from wind vane sensor or EKF wind estimation
     * @see motor_assist_tack(), set_motor_state()
     */
    bool motor_assist_low_wind() const;

    // Parameter storage - see var_info[] for descriptions
    AP_Int8 enable;                 ///< Enable sailboat functionality (0=disabled, 1=enabled)
    AP_Float sail_angle_min;        ///< Minimum mainsail angle in degrees (fully sheeted in, typically 0-10°)
    AP_Float sail_angle_max;        ///< Maximum mainsail angle in degrees (fully eased, typically 80-90°)
    AP_Float sail_angle_ideal;      ///< Ideal close-hauled sail angle in degrees (typically 15-25°)
    AP_Float sail_heel_angle_max;   ///< Maximum heel angle in degrees before easing sails (safety limit, typically 30-40°)
    AP_Float sail_no_go;            ///< No-go zone angle either side of true wind in degrees (typically 45-50°)
    AP_Float sail_windspeed_min;    ///< Minimum wind speed in m/s for sailing without motor assist (typically 1-2 m/s)
    AP_Float xtrack_max;            ///< Maximum cross-track error in meters before initiating tack in auto modes
    AP_Float loit_radius;           ///< Loiter radius in meters for sailboat (larger than standard rover, typically 15-30m)

    RC_Channel *channel_mainsail;   ///< Pointer to RC input channel for manual mainsail control (typically AUX function)
    
    // Tacking state machine variables - manage upwind tacking maneuvers
    bool currently_tacking;         ///< True when actively executing a tacking maneuver (transitioning between port/starboard tacks)
    float tack_heading_rad;         ///< Target heading in radians on new tack (0 to 2*PI, set at tack initiation)
    uint32_t tack_request_ms;       ///< System timestamp in milliseconds when pilot requested manual tack
    uint32_t auto_tack_start_ms;    ///< System timestamp in milliseconds when autonomous tack was initiated
    uint32_t tack_clear_ms;         ///< System timestamp in milliseconds when tack was completed and state cleared
    bool tack_assist;               ///< True if motor should assist current tack (helps prevent "in irons" stall)
    
    UseMotor motor_state;           ///< Current motor usage policy (NEVER=0, ASSIST=1, ALWAYS=2)
};
