/**
 * @file AP_TECS.h
 * @brief Total Energy Control System (TECS) for fixed-wing aircraft
 * 
 * @details This file implements the Total Energy Control System, which provides
 * combined control of airspeed and altitude for fixed-wing aircraft using energy
 * management principles.
 * 
 * **Fundamental Principle:**
 * Total Energy (TE) = Kinetic Energy (KE) + Potential Energy (PE)
 * - Specific Total Energy (STE) = gh + ½v² (energy per unit mass)
 * - Throttle controls the rate of change of total energy: d(STE)/dt
 * - Pitch controls the distribution of energy between kinetic (airspeed) and potential (altitude)
 * 
 * **Energy Management Strategy:**
 * - Throttle adjusts total energy of the aircraft (climb/descent capability)
 * - Pitch angle redistributes energy between altitude and airspeed
 * - Complementary filters provide robust height rate and airspeed rate estimation
 * - Underspeed protection automatically prioritizes airspeed over altitude when necessary
 * 
 * **Operational Modes:**
 * - Normal mode: Full TECS control with airspeed sensor
 * - Synthetic airspeed mode: Fallback when airspeed sensor unavailable (uses GPS + wind estimate)
 * - Underspeed protection: Automatic mode when airspeed drops below safe threshold
 * - Landing flare: Specialized exponential height tracking for touchdown
 * - Gliding mode: Zero-throttle energy management for soaring/engine-out scenarios
 * 
 * **Integration Points:**
 * - AP_AHRS: Provides attitude, position, velocity, and wind estimates
 * - AP_FixedWing: Vehicle parameters including cruise airspeed and control limits
 * - AP_Landing: Coordinates landing flare timing and sink rate management
 * - AP_Soaring: Requests gliding mode for thermal soaring
 * 
 * **Thread Safety:**
 * Designed for single-threaded sequential execution. All methods must be called from
 * the main navigation loop in the correct sequence: update_50hz() before update_pitch_throttle().
 * 
 * **Safety Considerations:**
 * - Incorrect tuning can cause stalls, spins, or dangerous flight attitudes
 * - TECS_TIME_CONST too small causes oscillations and aggressive maneuvering
 * - TECS_THR_DAMP too low causes throttle-induced phugoid oscillations
 * - TECS_CLMB_MAX must not exceed actual aircraft climb performance
 * - Underspeed protection overrides pilot commands to prevent stalls
 * 
 * @author Paul Riseborough (original implementation, 2013)
 * @copyright Copyright (c) 2013-2025 ArduPilot.org
 * 
 * Source: libraries/AP_TECS/AP_TECS.h
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <Filter/AverageFilter.h>

class AP_Landing;

/**
 * @class AP_TECS
 * @brief Total Energy Control System for fixed-wing aircraft energy management
 * 
 * @details AP_TECS implements a sophisticated energy-based flight control system for
 * fixed-wing aircraft. Unlike traditional separate altitude and airspeed controllers,
 * TECS views the aircraft as an energy system where:
 * 
 * **Energy Framework:**
 * - Total Energy = Potential Energy (altitude) + Kinetic Energy (airspeed)
 * - Throttle controls total energy rate (ability to climb/accelerate)
 * - Pitch controls energy balance (trading altitude for speed or vice versa)
 * 
 * **Control Architecture:**
 * 
 * Dual Control Loops:
 * 1. **Total Energy Loop:** Controls d(STE)/dt via throttle
 *    - STE = gh + ½v² (specific total energy)
 *    - Tracks commanded total energy rate to achieve desired climb/descent
 * 
 * 2. **Energy Balance Loop:** Controls energy distribution via pitch
 *    - Balances altitude error vs airspeed error
 *    - Adjustable weighting via TECS_SPDWEIGHT parameter
 * 
 * **State Estimation:**
 * Uses complementary filters combining:
 * - Height rate: Barometric altitude + vertical accelerometer
 * - Airspeed rate: Pitot tube + longitudinal accelerometer
 * - Provides robust estimates in turbulence and during maneuvers
 * 
 * **Underspeed Protection:**
 * Automatically activates when airspeed drops dangerously low:
 * - Forces maximum throttle
 * - Switches to speed-priority mode (allows altitude loss to regain speed)
 * - Overrides pilot altitude commands to prevent stall
 * 
 * **Fallback Modes:**
 * When airspeed sensor fails or is unavailable:
 * - Synthetic airspeed mode using GPS groundspeed and wind estimate
 * - Throttle control degrades to height-rate-based (less optimal but safe)
 * - Automatically selected via use_synthetic_airspeed()
 * 
 * **Landing Flare Integration:**
 * Specialized control during final landing approach:
 * - Exponential height tracking for smooth flare
 * - Coordinated with AP_Landing for touchdown sequencing
 * - Progressive sink rate reduction based on path_proportion
 * 
 * **Gliding Mode:**
 * Zero-throttle flight for:
 * - Thermal soaring (requested by AP_Soaring)
 * - Engine failure scenarios (propulsion_failed flag)
 * - Glider aircraft (THR_MAX=0 configuration)
 * 
 * **Typical Usage Pattern:**
 * ```cpp
 * // Initialization (once)
 * AP_TECS tecs(ahrs, fixed_wing_params, landing, log_bitmask);
 * 
 * // Main loop (called at ≥50Hz)
 * tecs.update_50hz();  // Update state estimators
 * 
 * // Control loop (called at 10-500Hz, typically 50Hz)
 * tecs.update_pitch_throttle(
 *     height_demand_cm,
 *     airspeed_demand_cm,
 *     flight_stage,
 *     distance_beyond_land_wp,
 *     pitch_min_climbout_cd,
 *     throttle_nudge,
 *     height_above_field_m,
 *     load_factor,
 *     pitch_trim_deg
 * );
 * 
 * // Get outputs
 * float throttle_pct = tecs.get_throttle_demand();  // -100 to 100
 * int32_t pitch_cd = tecs.get_pitch_demand();       // -9000 to 9000 centidegrees
 * ```
 * 
 * **Coordinate Frames:**
 * - Heights: Relative to field elevation (ground level), positive up, in meters
 * - Speeds: True airspeed (TAS) and equivalent airspeed (EAS) in m/s
 * - Pitch: Body frame, positive = nose up, in radians internally
 * 
 * **Units Convention:**
 * - Heights/altitudes: meters (m)
 * - Airspeeds: meters/second (m/s)
 * - Accelerations: m/s²
 * - Pitch angles: radians (internal), degrees (setter parameters), centidegrees (getter output)
 * - Throttle: normalized [-1.0, 1.0] internally, percentage [-100, 100] for external interface
 * - Time constants: seconds (s)
 * - Energy rates: m/s (specific energy rate = energy/mass rate)
 * 
 * **Key Parameters (TECS_*):**
 * - TECS_TIME_CONST: Time constant for altitude/airspeed response (seconds)
 * - TECS_THR_DAMP: Throttle damping to prevent oscillations
 * - TECS_PTCH_DAMP: Pitch damping coefficient
 * - TECS_CLMB_MAX: Maximum climb rate capability (m/s)
 * - TECS_SINK_MAX: Maximum sink rate (m/s, positive down)
 * - TECS_SPDWEIGHT: Weighting between speed and height priority [0=height, 2=speed]
 * - TECS_INTEG_GAIN: Integrator gain for steady-state error correction
 * 
 * **Thread Safety:**
 * NOT thread-safe. Must be called sequentially from single thread (main navigation loop).
 * No internal locking mechanisms. Concurrent calls will corrupt state estimators.
 * 
 * @note This is a flight-critical controller. Parameter changes should be tested
 * thoroughly in SITL before flight testing. Incorrect tuning can result in:
 * - Stalls and spins (TECS_TIME_CONST too small, TECS_CLMB_MAX too high)
 * - Oscillations (insufficient damping)
 * - Poor tracking (TECS_TIME_CONST too large)
 * - Throttle hunting (TECS_THR_DAMP too low)
 * 
 * @warning Safety-Critical Component. Modifications require thorough testing and
 * validation to ensure flight safety across all operational scenarios.
 * 
 * Source: libraries/AP_TECS/AP_TECS.h:27-511
 */
class AP_TECS {
public:
    /**
     * @brief Construct TECS controller with required dependencies
     * 
     * @details Creates a new TECS controller instance and initializes parameter defaults.
     * The constructor establishes references to required subsystems but does not initialize
     * state estimators or control loops. Actual initialization occurs on first call to
     * update_50hz() or when reset() is called.
     * 
     * @param[in] ahrs Reference to AP_AHRS for attitude, position, velocity, and wind data
     * @param[in] parms Reference to AP_FixedWing parameters (cruise speeds, control limits)
     * @param[in] landing Reference to AP_Landing for flare coordination and landing configuration
     * @param[in] log_bitmask Bitmask controlling which TECS internal variables are logged
     * 
     * @note The ahrs, parms, and landing references must remain valid for the lifetime
     * of the TECS object (typically static objects with program lifetime).
     * 
     * @note Parameter defaults are loaded from var_info table via AP_Param::setup_object_defaults().
     * User-configured parameters will override defaults after EEPROM load.
     */
    AP_TECS(AP_AHRS &ahrs, const AP_FixedWing &parms, const AP_Landing &landing, const uint32_t log_bitmask)
        : _ahrs(ahrs)
        , aparm(parms)
        , _landing(landing)
        , _log_bitmask(log_bitmask)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_TECS);

    /**
     * @brief Update height and speed estimation filters at 50Hz or faster
     * 
     * @details This method runs the complementary filters that estimate height rate and
     * airspeed rate by fusing barometer/GPS altitude data with accelerometer measurements.
     * These state estimates are required by update_pitch_throttle() to compute control demands.
     * 
     * **Filter Implementation:**
     * - Height rate: Combines barometric altitude rate with vertical accelerometer (NED frame down axis)
     * - Airspeed rate: Combines pitot differential pressure rate with longitudinal accelerometer (body X axis)
     * - Complementary filter design provides high-frequency response from accelerometers,
     *   low-frequency response from slower but drift-free baro/pitot sensors
     * 
     * **Calling Requirements:**
     * - MUST be called at ≥50Hz for proper filter operation
     * - MUST be called BEFORE update_pitch_throttle() each control cycle
     * - Typical call rate: 50-400Hz depending on platform scheduler
     * 
     * **Filter State Updates:**
     * - _height_filter.dd_height: Height second derivative (vertical acceleration filtered)
     * - _height_filter.height: Integrated height estimate
     * - _integDTAS_state: Airspeed rate integrator state
     * - _TAS_state: True airspeed estimate
     * - _vel_dot: Longitudinal acceleration (body X-axis) in m/s²
     * 
     * @note Call rate slower than 50Hz will degrade height and airspeed rate estimation accuracy,
     * potentially causing poor altitude tracking and airspeed regulation.
     * 
     * @warning Failure to call this method before update_pitch_throttle() will result in
     * stale state estimates and degraded control performance. In extreme cases, can cause
     * altitude and airspeed tracking errors leading to unsafe flight conditions.
     * 
     * @warning Thread Safety: Must be called from same thread as update_pitch_throttle(),
     * in sequential order. No concurrent calls allowed.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void update_50hz(void);

    /**
     * @brief Main TECS control loop computing pitch and throttle demands
     * 
     * @details This is the primary TECS control function that implements the total energy
     * control law. It must be called after update_50hz() to use current state estimates.
     * 
     * **Total Energy Control Law:**
     * 
     * Specific Total Energy (STE) = gh + ½v² (energy per unit mass)
     * - g: gravitational acceleration (9.80665 m/s²)
     * - h: height above field elevation (m)
     * - v: true airspeed (m/s)
     * 
     * Control Strategy:
     * 1. **Throttle Controls Total Energy Rate:** d(STE)/dt
     *    - Positive throttle increases total energy (enables climb and/or acceleration)
     *    - Negative throttle (or idle) decreases total energy (descent and/or deceleration)
     * 
     * 2. **Pitch Controls Energy Balance:**
     *    - Nose up: Converts kinetic energy (speed) to potential energy (altitude)
     *    - Nose down: Converts potential energy (altitude) to kinetic energy (speed)
     *    - Balance weighted by TECS_SPDWEIGHT parameter
     * 
     * **Flight Stage Adaptations:**
     * - TAKEOFF: Uses TECS_INTEG_GAIN_TAKEOFF, enforces ptchMinCO_cd climb-out pitch
     * - LAND: Uses TECS_LAND_* parameters, activates flare logic based on path_proportion
     * - NORMAL: Standard TECS control with configured parameters
     * - ABORT_LAND: Transitions from landing to go-around
     * 
     * **Landing Flare Logic:**
     * When path_proportion approaches 1.0 (near touchdown):
     * - Exponential height tracking reduces sink rate progressively
     * - Height demand adjusted for smooth flare entry
     * - Integrators managed to prevent ballooning
     * 
     * **Underspeed Protection:**
     * If airspeed drops below safe threshold:
     * - Automatically sets throttle to maximum
     * - Switches to speed-priority mode (allows altitude loss)
     * - Overrides altitude demand to prevent stall
     * 
     * **Call Rate Constraints:**
     * - Minimum: 10Hz (slower causes sluggish response and poor disturbance rejection)
     * - Maximum: 500Hz (faster can cause numerical issues in integrators)
     * - Recommended: 50-100Hz for optimal performance and computational efficiency
     * 
     * @param[in] hgt_dem_cm Demanded height above field elevation in centimeters (positive up)
     * @param[in] EAS_dem_cm Demanded equivalent airspeed in cm/s (EAS compensates for altitude effects)
     * @param[in] flight_stage Current flight stage (TAKEOFF, NORMAL, LAND, ABORT_LAND, etc.)
     * @param[in] distance_beyond_land_wp Distance past landing waypoint in meters (for flare timing, negative before WP)
     * @param[in] ptchMinCO_cd Minimum pitch during climbout in centidegrees (enforced during TAKEOFF stage)
     * @param[in] throttle_nudge Manual throttle adjustment from pilot in range [-100, 100], typically for thermal soaring
     * @param[in] hgt_afe Height above field elevation in meters (used for flare initiation and ground effect)
     * @param[in] load_factor Aerodynamic load factor from turns (g-loading: 1.0=level, >1.0=turn, affects energy calculations)
     * @param[in] pitch_trim_deg Pitch trim angle in degrees (for airframe-specific trim compensation)
     * 
     * @note Output demands are retrieved via get_throttle_demand() and get_pitch_demand() immediately
     * after this call. Values are valid until next update_pitch_throttle() call.
     * 
     * @note EAS (Equivalent Airspeed) is used for demand to maintain consistent dynamic pressure
     * across altitude changes. Internally converted to TAS (True Airspeed) for energy calculations.
     * 
     * @note load_factor > 1.0 in turns increases kinetic energy requirement (v²/r centripetal acceleration),
     * TECS compensates by increasing total energy demand.
     * 
     * @warning Call rate below 10Hz will result in poor disturbance rejection and sluggish response
     * to altitude/airspeed changes. May cause unsafe tracking errors in gusty conditions.
     * 
     * @warning Call rate above 500Hz can cause numerical instability in integrator calculations,
     * potentially leading to integrator wind-up and control saturation.
     * 
     * @warning During landing flare (path_proportion > 0.9), TECS transitions to exponential height
     * tracking. Incorrect TECS_LAND_SINK or TECS_LAND_ARSPD parameters can cause hard landings or
     * balloon (bounce) on touchdown.
     * 
     * @warning Thread Safety: Must be called from same thread as update_50hz(), after update_50hz()
     * has provided fresh state estimates. No concurrent execution allowed.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void update_pitch_throttle(int32_t hgt_dem_cm,
                               int32_t EAS_dem_cm,
                               enum AP_FixedWing::FlightStage flight_stage,
                               float distance_beyond_land_wp,
                               int32_t ptchMinCO_cd,
                               int16_t throttle_nudge,
                               float hgt_afe,
                               float load_factor,
                               float pitch_trim_deg);

    /**
     * @brief Get computed throttle demand from TECS controller
     * 
     * @details Returns the throttle demand calculated by the most recent update_pitch_throttle()
     * call. This value is the result of the total energy control loop and includes:
     * - Total energy rate tracking (matching climb/descent demand)
     * - Throttle damping (TECS_THR_DAMP) to prevent oscillations
     * - Integrator trim for steady-state errors
     * - Rate limiting for smooth throttle transitions
     * - Saturation limiting to configured throttle range
     * 
     * **Throttle Range:**
     * - Typical range: 0 to 100% (no reverse thrust)
     * - With reverse thrust: -100 to 100% (if THR_MIN < 0 configured)
     * - Zero throttle: Idle/gliding flight
     * - 100% throttle: Maximum available thrust
     * 
     * **Special Conditions:**
     * - Underspeed: Automatically saturates to 100% regardless of altitude error
     * - Gliding mode: Forces to 0% (thermal soaring or engine failure)
     * - Landing: Reduces to TECS_LAND_THR during final approach
     * 
     * @return Throttle demand in percentage [-100, 100]
     *         - Positive values: Forward thrust (typical)
     *         - Zero: Idle/gliding
     *         - Negative values: Reverse thrust (only if reverse thrust enabled via parameter)
     * 
     * @note Call immediately after update_pitch_throttle() for current demand.
     * Value remains valid until next update_pitch_throttle() call.
     * 
     * @note For most aircraft configurations, negative throttle is not supported and return
     * value will be in range [0, 100]. Reverse thrust requires THR_MIN parameter < 0.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:60-62
     */
    float get_throttle_demand(void) {
        return _throttle_dem * 100.0f;
    }

    /**
     * @brief Get computed pitch angle demand from TECS controller
     * 
     * @details Returns the pitch demand calculated by the most recent update_pitch_throttle()
     * call. This value is the result of the energy balance control loop and includes:
     * - Energy balance error correction (altitude vs airspeed weighting via TECS_SPDWEIGHT)
     * - Pitch damping (TECS_PTCH_DAMP) to prevent oscillations
     * - Integrator trim for steady-state pitch errors
     * - Rate limiting for smooth pitch transitions
     * - Saturation to configured pitch range (TECS_PITCH_MIN to TECS_PITCH_MAX)
     * 
     * **Pitch Convention:**
     * - Positive pitch: Nose up (increases altitude, decreases airspeed)
     * - Negative pitch: Nose down (decreases altitude, increases airspeed)
     * - Zero pitch: Level flight (approximately, depends on airframe trim)
     * - Reference frame: Body frame pitch angle
     * 
     * **Special Conditions:**
     * - Underspeed protection: Pitch-for-speed priority (may command nose down despite altitude loss)
     * - Takeoff: Enforces minimum climbout pitch (ptchMinCO_cd parameter)
     * - Landing flare: Progressive nose-up to reduce sink rate at touchdown
     * - Stall protection: Limited by TECS_PITCH_MAX to prevent excessive angle of attack
     * 
     * **Conversion Factor:**
     * 5729.5781 = 180 * 100 / π (radians to centidegrees conversion)
     * - Internal representation: radians
     * - External interface: centidegrees (degrees × 100 for integer precision)
     * 
     * @return Pitch demand in centidegrees, range [-9000, 9000]
     *         - -9000 cd: Maximum nose down (−90 degrees, extreme dive)
     *         - 0 cd: Level flight pitch attitude
     *         - +9000 cd: Maximum nose up (+90 degrees, vertical climb)
     *         - Typical operational range: -2000 cd to +3000 cd (-20° to +30°)
     * 
     * @note Call immediately after update_pitch_throttle() for current demand.
     * Value remains valid until next update_pitch_throttle() call.
     * 
     * @note Extreme pitch demands (±90°) are theoretical limits. Actual aircraft operation
     * constrained by TECS_PITCH_MIN and TECS_PITCH_MAX parameters, typically ±30 degrees
     * for safety and aerodynamic efficiency.
     * 
     * @warning Pitch demand is in body frame. For earth-frame altitude control, attitude
     * controller must convert this to elevator deflection considering current attitude.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:66-68
     */
    int32_t get_pitch_demand(void) {
        return int32_t(_pitch_dem * 5729.5781f);
    }

    /**
     * @brief Get longitudinal acceleration estimate (body X-axis)
     * 
     * @details Returns the estimated rate of change of velocity along the aircraft body
     * X-axis (forward direction). This value is computed by the complementary filter in
     * update_50hz() and represents the longitudinal acceleration used for airspeed rate
     * estimation and energy calculations.
     * 
     * **Measurement Sources:**
     * - IMU accelerometer (body frame X-axis, high-frequency component)
     * - Differentiated airspeed from pitot tube (low-frequency component)
     * - Fused via complementary filter for robust estimate
     * 
     * **Applications:**
     * - Airspeed rate estimation for energy balance calculations
     * - Feedforward compensation in pitch control loop
     * - Thrust-to-weight ratio estimation
     * - Performance monitoring and logging
     * 
     * **Coordinate Frame:**
     * Body X-axis (forward), positive = forward acceleration, negative = deceleration
     * 
     * @return Longitudinal acceleration in m/s²
     *         - Positive: Accelerating forward (thrust > drag)
     *         - Negative: Decelerating (drag > thrust, or climbing)
     *         - Zero: Constant airspeed (equilibrium)
     *         - Typical range: -5 to +5 m/s² for normal flight
     * 
     * @note This value includes effects of:
     * - Engine thrust
     * - Aerodynamic drag
     * - Gravity component along flight path (pitch angle effect)
     * - Wind shear (rapid changes in headwind/tailwind)
     * 
     * @note Updated at update_50hz() rate (≥50Hz). More recent value = more accurate.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:71-73
     */
    float get_VXdot(void) {
        return _vel_dot;
    }

    /**
     * @brief Get current target equivalent airspeed (EAS)
     * 
     * @details Returns the target equivalent airspeed after rate limiting and underspeed
     * protection logic have been applied. This is the actual airspeed target that TECS is
     * tracking, which may differ from the commanded airspeed due to:
     * - Rate limiting (TECS_TIME_CONST controls acceleration/deceleration rate)
     * - Underspeed protection (prevents airspeed from dropping too low)
     * - Aircraft performance limits (cannot exceed maximum or minimum achievable speeds)
     * 
     * **EAS vs TAS:**
     * - EAS (Equivalent Airspeed): Airspeed corrected for altitude, represents dynamic pressure
     * - TAS (True Airspeed): Actual speed through air mass (internally used by TECS)
     * - Conversion: TAS = EAS × √(ρ₀/ρ) where ρ is air density at current altitude
     * - EAS used for external interface to maintain consistent reference across altitude changes
     * 
     * **Target vs Demand:**
     * - Demand: What the autopilot/pilot wants (input to update_pitch_throttle)
     * - Target: What TECS is actively tracking after filtering and protection logic
     * - Target approaches demand gradually based on TECS_TIME_CONST
     * 
     * **Underspeed Protection:**
     * If target would go below safe minimum:
     * - Target automatically increased to prevent stall
     * - Altitude goal sacrificed to maintain safe airspeed
     * - Indicated by underspeed flag in telemetry
     * 
     * @return Target equivalent airspeed in m/s
     *         - Typical range: 15-30 m/s for small UAVs, 30-60 m/s for larger aircraft
     *         - Minimum: ARSPD_FBW_MIN parameter (stall prevention)
     *         - Maximum: ARSPD_FBW_MAX parameter (structural/control limits)
     * 
     * @note This is the EAS value that TECS control loops are attempting to achieve.
     * Actual measured airspeed may differ due to disturbances (wind, turbulence) and
     * control system lag.
     * 
     * @note Use this value (rather than commanded airspeed) for:
     * - Performance monitoring (tracking error calculation)
     * - Telemetry display of active target
     * - Integration with other controllers expecting filtered reference
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:76-78
     */
    float get_target_airspeed(void) const {
        return _TAS_dem_adj / _ahrs.get_EAS2TAS();
    }

    /**
     * @brief Get maximum climb rate achievable with current energy constraints
     * 
     * @details Returns the maximum climb rate that TECS will attempt to achieve given:
     * - Aircraft performance capabilities (TECS_CLMB_MAX parameter)
     * - Current throttle limits (full throttle available?)
     * - Current airspeed and altitude (affects engine and propeller performance)
     * - Aerodynamic constraints (best climb airspeed)
     * 
     * **Scaling Factors:**
     * Value may be scaled from TECS_CLMB_MAX parameter based on:
     * - Throttle saturation (if throttle already at max, reduce expected climb rate)
     * - Airspeed deviations (off-optimum climb speed reduces performance)
     * - Altitude effects (reduced air density at high altitude)
     * 
     * **Usage:**
     * - Mission planning: Calculate climb time and fuel consumption
     * - Geofencing: Determine if altitude constraint can be met
     * - Autopilot logic: Assess if climb demand is achievable
     * - Performance monitoring: Compare actual vs maximum capability
     * 
     * @return Maximum climb rate in m/s (positive up)
     *         - Typical range: 3-10 m/s for small UAVs, 5-20 m/s for larger aircraft
     *         - Based on TECS_CLMB_MAX parameter, potentially scaled by current conditions
     *         - Zero or negative values indicate climb not possible (descending or level only)
     * 
     * @note This is the climb rate capability at current conditions, NOT the demanded
     * climb rate. Actual climb rate demand is constrained by this value to prevent
     * requesting unachievable performance that would cause airspeed loss and potential stall.
     * 
     * @warning If TECS_CLMB_MAX is set higher than actual aircraft capability, TECS will
     * command excessive pitch-up, causing airspeed loss and potential stall. Always tune
     * TECS_CLMB_MAX conservatively based on actual flight test results.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:81-83
     */
    float get_max_climbrate(void) const {
        return _maxClimbRate;
    }

    /**
     * @brief Get maximum sink rate achievable with current energy constraints
     * 
     * @details Returns the maximum sink (descent) rate that TECS will attempt to achieve given:
     * - Aircraft aerodynamic drag (clean configuration vs with spoilers/dive brakes)
     * - Minimum airspeed limits (preventing overspeed in dive)
     * - Structural limits (terminal velocity, Vne - never exceed speed)
     * - Configuration: Normal descent, approach descent, or emergency descent
     * 
     * **Descent Modes:**
     * - Normal: TECS_SINK_MAX parameter (typical descent)
     * - Approach: TECS_SINK_MAX_APPROACH parameter (shallower for landing approach)
     * - Emergency: May exceed normal limits if rapid descent required
     * 
     * **Convention:**
     * Positive value = sink rate (descending). This follows aviation convention where
     * "2000 fpm descent" is a positive sink rate value.
     * 
     * **Limiting Factors:**
     * - Minimum throttle setting (idle or reverse thrust)
     * - Maximum pitch-down angle (structural and control limits)
     * - Maximum allowable airspeed (preventing overspeed)
     * - Drag devices availability (speedbrakes, spoilers)
     * 
     * **Usage:**
     * - Mission planning: Calculate descent time and distance
     * - Approach planning: Verify glide slope is achievable
     * - Emergency procedures: Maximum descent capability for rapid altitude loss
     * - Performance monitoring: Compare actual vs maximum capability
     * 
     * @return Maximum sink rate in m/s (positive = descending, down convention)
     *         - Typical range: 2-5 m/s for small UAVs, 3-10 m/s for larger aircraft
     *         - Based on TECS_SINK_MAX or TECS_SINK_MAX_APPROACH parameters
     *         - Approach mode typically 40-60% of normal sink rate for stable approach
     * 
     * @note During landing approach (flight_stage == LAND), TECS may use
     * TECS_SINK_MAX_APPROACH instead of TECS_SINK_MAX for a more conservative,
     * stable descent profile.
     * 
     * @note In gliding mode (zero throttle), maximum sink rate depends entirely on
     * aerodynamic drag. TECS will pitch down to achieve target glide ratio while
     * respecting airspeed limits.
     * 
     * @warning Excessive sink rate settings (TECS_SINK_MAX too high) can cause:
     * - Overspeed conditions in descent (exceeding Vne)
     * - Loss of control authority at high airspeed
     * - Structural damage from aerodynamic loads
     * Always tune conservatively and validate in SITL before flight testing.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:86-88
     */
    float get_max_sinkrate(void) const {
        return _maxSinkRate;
    }
    
    /**
     * @brief Reset pitch control integrators to zero
     * 
     * @details Resets the pitch (energy balance) control integrators that accumulate
     * steady-state errors in the energy distribution between altitude and airspeed.
     * This includes:
     * - _integSEBdot: Specific Energy Balance rate integrator
     * - _integKE: Kinetic Energy error integrator
     * 
     * **When to Reset:**
     * - Flight mode transitions (e.g., entering AUTO, FBWA, or thermal soaring modes)
     * - After significant pitch discontinuities (e.g., takeoff rotation, landing flare entry)
     * - When pilot manually overrides pitch control
     * - During thermal soaring entry/exit (called by AP_Soaring::SoaringController)
     * - After GPS reset or large position jumps
     * 
     * **Integrator Purpose:**
     * Integrators trim out steady-state errors in pitch control, compensating for:
     * - Airframe asymmetry and trim requirements
     * - Persistent wind shear or vertical air currents
     * - Center-of-gravity variations (fuel burn, payload changes)
     * 
     * **Consequences of Reset:**
     * - Pitch control will rely solely on proportional and derivative terms until integrator rebuilds
     * - May see temporary tracking errors for 2-10 seconds after reset
     * - Prevents integrator wind-up when discontinuities occur
     * 
     * @note Use sparingly. Frequent resets prevent integrators from performing their function
     * of trimming out steady-state errors. Only reset when discontinuities would cause
     * integrator wind-up or when mode transitions invalidate accumulated error.
     * 
     * @note This method is specifically provided for AP_Soaring to reset integrators when
     * entering/exiting thermal soaring mode, where pitch control strategy changes significantly.
     * 
     * @warning Resetting integrators during stable flight will cause temporary tracking errors.
     * Only call when entering/exiting modes or after significant discontinuities.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:91-94
     */
    void reset_pitch_I(void) {
        _integSEBdot = 0.0f;
        _integKE = 0.0f;
    }

    /**
     * @brief Reset throttle control integrator to zero
     * 
     * @details Resets the throttle (total energy) control integrator that accumulates
     * steady-state errors in total energy rate tracking. The integrator (_integTHR_state)
     * provides trim correction for:
     * - Steady-state throttle required for level flight at cruise speed
     * - Propulsion system efficiency variations with airspeed and altitude
     * - Aerodynamic drag variations (gear/flap configuration, airframe variations)
     * - Wind effects and persistent updrafts/downdrafts
     * 
     * **When to Reset:**
     * - Flight mode transitions (entering AUTO, FBWA, CRUISE, etc.)
     * - After pilot manually controls throttle (manual override)
     * - When switching between different power settings (cruise vs climb)
     * - After significant configuration changes (flaps, landing gear deployment)
     * - Following GPS resets or position discontinuities
     * 
     * **Integrator Purpose:**
     * Provides steady-state throttle correction to maintain total energy rate, compensating for:
     * - Throttle curve nonlinearities (different efficiency at different throttle settings)
     * - Altitude-dependent power variations (engine/motor performance changes with density altitude)
     * - Weight changes (fuel burn over flight duration)
     * - Aging propulsion system (reduced efficiency over airframe lifetime)
     * 
     * **Consequences of Reset:**
     * - Throttle control relies on proportional and derivative terms only until integrator rebuilds
     * - May see 2-10 seconds of altitude/airspeed tracking errors as integrator re-converges
     * - Prevents integrator wind-up during large throttle changes or mode transitions
     * 
     * **Rebuild Time:**
     * Integrator typically rebuilds to steady-state value within:
     * - 5-10 seconds in calm conditions
     * - 10-20 seconds in turbulent conditions
     * - Faster rebuild with higher TECS_INTEG_GAIN parameter
     * 
     * @note Typically called automatically by TECS on mode entry. Manual calls usually
     * not required unless implementing custom throttle override logic.
     * 
     * @note Unlike pitch integrator (which has two components), throttle has single
     * integrator state for total energy rate error correction.
     * 
     * @warning Do not call during continuous flight in stable mode. Only reset when
     * throttle discontinuities would cause integrator wind-up. Excessive resets degrade
     * altitude and airspeed tracking performance.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:97-99
     */
    void reset_throttle_I(void) {
        _integTHR_state = 0.0;
    }

    /**
     * @brief Get configured landing sink rate target
     * 
     * @details Returns the TECS_LAND_SINK parameter value, which specifies the target
     * sink rate (descent rate) during final approach and landing flare. This value is
     * used by the landing sequencer to plan the approach profile.
     * 
     * **Landing Sink Rate Purpose:**
     * - Establishes gentle descent rate for stable approach
     * - Progressively reduced during flare based on path_proportion
     * - Prevents hard landings by controlling touchdown vertical velocity
     * 
     * **Typical Values:**
     * - Small UAVs: 0.5-1.0 m/s (relatively flat approach)
     * - Larger aircraft: 1.0-2.5 m/s (steeper approach, more like full-scale)
     * - Precision landing: 0.3-0.7 m/s (very gentle for sensitive payloads)
     * 
     * **Flare Behavior:**
     * During flare (path_proportion approaching 1.0):
     * - Initial sink rate: TECS_LAND_SINK value
     * - Exponentially reduced as touchdown approaches
     * - Final touchdown: Near-zero vertical velocity (ideal)
     * - Flare shape controlled by TECS_LAND_SINK_RATE_CHANGE
     * 
     * @return Landing sink rate in m/s (positive = descending, down convention)
     *         - Typical range: 0.5-2.5 m/s
     *         - Directly from TECS_LAND_SINK parameter
     *         - Used as initial target, progressively reduced in flare
     * 
     * @note This is the START of flare sink rate. Actual touchdown sink rate will be
     * much lower (ideally near zero) after exponential flare reduction.
     * 
     * @note Used by AP_Landing and mission sequencer to:
     * - Plan approach glide slope angle
     * - Calculate distance required for landing
     * - Coordinate with TECS for smooth landing execution
     * 
     * @warning If TECS_LAND_SINK is too high:
     * - Risk of hard landing (high vertical velocity at touchdown)
     * - Insufficient time for flare to arrest descent
     * - Potential damage to airframe or payload
     * 
     * @warning If TECS_LAND_SINK is too low:
     * - Very shallow approach (may overshoot landing zone)
     * - Excessive landing distance required
     * - Difficulty maintaining glide slope in wind
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:102-104
     */
    float get_land_sinkrate(void) const {
        return _land_sink;
    }

    /**
     * @brief Get configured landing airspeed target
     * 
     * @details Returns the TECS_LAND_ARSPD parameter value, which specifies the target
     * airspeed during final approach and landing. This is typically slower than cruise
     * airspeed to reduce touchdown speed and landing distance.
     * 
     * **Landing Airspeed Purpose:**
     * - Reduces kinetic energy for safer touchdown
     * - Decreases landing roll distance
     * - Provides better low-speed control authority
     * - Balances between control effectiveness and stall margin
     * 
     * **Selection Criteria:**
     * Landing airspeed should be:
     * - Above stall speed by comfortable margin (typically stall + 30-50%)
     * - Low enough for safe touchdown and short landing roll
     * - High enough to maintain control authority in wind/turbulence
     * - Compatible with flap configuration if flaps used
     * 
     * **Typical Values:**
     * - Small UAVs: 12-18 m/s (with stall around 8-12 m/s)
     * - Larger aircraft: 18-25 m/s (with stall around 12-17 m/s)
     * - Should be 1.3-1.5× stall speed minimum for safety margin
     * 
     * **Approach Profile:**
     * - Airspeed gradually reduced from cruise to landing speed during descent
     * - Landing airspeed maintained through final approach and flare
     * - TECS prioritizes maintaining this airspeed over altitude hold during landing
     * 
     * @return Landing airspeed in m/s (equivalent airspeed)
     *         - Typical range: 12-25 m/s depending on aircraft size and performance
     *         - Directly from TECS_LAND_ARSPD parameter
     *         - Maintained as target throughout final approach and flare
     * 
     * @note If TECS_LAND_ARSPD is set to -1 (default in some configurations), TECS
     * will use cruise airspeed or autopilot-calculated landing speed instead.
     * 
     * @note Used by AP_Landing to:
     * - Configure approach speed profile
     * - Calculate landing distance required
     * - Coordinate with TECS for airspeed management during landing
     * 
     * @warning If TECS_LAND_ARSPD is too low (close to stall speed):
     * - Risk of stall during approach, especially in turbulence or wind shear
     * - Loss of control authority for flare execution
     * - Unrecoverable sink rate if airspeed lost during final approach
     * 
     * @warning If TECS_LAND_ARSPD is too high:
     * - Excessive touchdown speed (risk of overshoot, hard landing)
     * - Increased landing distance
     * - Difficulty achieving smooth, controlled touchdown
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:107-109
     */
    float get_land_airspeed(void) const {
        return _landAirspeed;
    }

    /**
     * @brief Get current height rate demand being tracked by TECS
     * 
     * @details Returns the height rate (climb/sink rate) demand that TECS is actively
     * tracking after all filtering, limiting, and flare logic have been applied. This
     * is the actual vertical velocity target that the control loops are attempting to
     * achieve.
     * 
     * **Height Rate Sources:**
     * In normal flight:
     * - Derived from altitude error and time constant (TECS_TIME_CONST)
     * - Rate-limited to prevent excessive climb/sink demands
     * - Constrained by TECS_CLMB_MAX and TECS_SINK_MAX
     * 
     * During landing flare:
     * - Exponentially reduced from TECS_LAND_SINK toward zero
     * - Based on path_proportion (progress along landing path)
     * - Shaped by TECS_LAND_SINK_RATE_CHANGE parameter
     * 
     * **Convention:**
     * - Positive: Climbing (ascending)
     * - Negative: Sinking/descending
     * - Zero: Level flight (maintaining altitude)
     * 
     * **Applications:**
     * - Performance monitoring: Compare actual vs demanded climb/sink rate
     * - Telemetry display: Show pilot/ground station what TECS is trying to achieve
     * - Integration with other systems: Terrain following, obstacle avoidance
     * - Logging and analysis: Understand TECS behavior during flight review
     * 
     * @return Height rate demand in m/s
     *         - Positive: Climbing (upward vertical velocity)
     *         - Negative: Descending (downward vertical velocity)
     *         - Zero: Maintaining altitude (level flight)
     *         - Typical range: -5 to +10 m/s during normal operations
     *         - Constrained by aircraft performance limits (TECS_CLMB_MAX, TECS_SINK_MAX)
     * 
     * @note This is the rate-limited, filtered height rate demand, NOT the raw altitude
     * error. Actual demand is always achievable within aircraft performance limits.
     * 
     * @note During landing flare, this value progressively reduces from landing sink rate
     * toward zero as the aircraft approaches touchdown, creating the characteristic
     * flare curvature.
     * 
     * @note Useful for:
     * - Diagnosing altitude tracking issues (compare with actual climb rate from AHRS)
     * - Validating TECS tuning (smooth demand = good tuning)
     * - Understanding energy management decisions (height vs speed priority)
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:112-114
     */
    float get_height_rate_demand(void) const {
        return _hgt_rate_dem;
    }

    /**
     * @brief Set normalized progress along landing path for flare control
     * 
     * @details Sets the path proportion value used to trigger and shape the landing flare.
     * This normalized value represents progress from the start of final approach (0.0)
     * to the touchdown point (1.0). TECS uses this to progressively reduce sink rate
     * and transition from approach to flare to touchdown.
     * 
     * **Path Proportion Definition:**
     * - 0.0: Start of final approach (base of approach glide slope)
     * - 0.5: Midpoint of final approach
     * - 0.9-0.95: Typical flare initiation point
     * - 1.0: Touchdown point (landing waypoint)
     * - >1.0: Past touchdown point (clamped to 1.0)
     * 
     * **Flare Behavior vs Path Proportion:**
     * - [0.0 - 0.9]: Normal approach with constant TECS_LAND_SINK
     * - [0.9 - 1.0]: Progressive flare, exponentially reducing sink rate
     * - At 1.0: Near-zero sink rate for gentle touchdown
     * 
     * **Flare Shape:**
     * Sink rate reduction is exponential, controlled by:
     * - TECS_LAND_SINK: Initial sink rate at flare entry
     * - TECS_LAND_SINK_RATE_CHANGE: Rate of sink rate reduction
     * - _path_proportion: Progress through flare
     * 
     * Formula (approximate): 
     * sink_rate = TECS_LAND_SINK × exp(-k × (path_proportion - 0.9))
     * where k depends on TECS_LAND_SINK_RATE_CHANGE
     * 
     * **Typical Usage:**
     * Called by AP_Landing or mission sequencer during landing phase:
     * ```cpp
     * float progress = distance_travelled / total_approach_distance;
     * tecs.set_path_proportion(progress);
     * ```
     * 
     * @param[in] path_proportion Normalized progress along landing path [0.0, 1.0]
     *            - 0.0 = start of final approach
     *            - 1.0 = touchdown point
     *            - Values outside [0.0, 1.0] are clamped to valid range
     * 
     * @note Must be called each control loop iteration during landing to maintain
     * accurate flare progression. Typically called by AP_Landing::update().
     * 
     * @note Path proportion can be calculated from:
     * - Distance along landing path (linear distance method)
     * - Time since flare initiation (time-based method)
     * - Height above field elevation (altitude-based method)
     * Choice depends on landing sequencer implementation.
     * 
     * @note Value is automatically constrained to [0.0, 1.0] range. Values <0.0 or >1.0
     * are clamped to prevent undefined flare behavior.
     * 
     * @warning Incorrect path_proportion calculation can cause:
     * - Early flare (proportion increases too quickly): Balloon/float, long landing roll
     * - Late flare (proportion increases too slowly): Hard landing, insufficient flare time
     * - Jumped proportion (discontinuity): Abrupt pitch change, altitude bump
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:117-119
     */
    void set_path_proportion(float path_proportion) {
        _path_proportion = constrain_float(path_proportion, 0.0f, 1.0f);
    }

    /**
     * @brief Enable or disable gliding mode for soaring flight
     * 
     * @details Sets the gliding_requested flag to enable zero-throttle gliding flight.
     * This is typically used by the AP_Soaring subsystem for autonomous thermal soaring,
     * where the aircraft uses rising air currents to gain altitude without engine power.
     * 
     * **Gliding Mode Effects:**
     * When gliding_requested = true:
     * - Throttle forced to idle/zero regardless of altitude error
     * - Pitch control prioritizes maintaining safe airspeed (stall prevention)
     * - Energy management focused on maximizing glide ratio
     * - Altitude loss accepted to maintain minimum safe airspeed
     * 
     * **Use Cases:**
     * 1. **Thermal Soaring:** AP_Soaring detects thermal and requests gliding to exploit lift
     * 2. **Glider Aircraft:** Continuous gliding mode (gliding_requested always true)
     * 3. **Energy Conservation:** Glide segments between waypoints to save battery/fuel
     * 4. **Emergency Descent:** Rapid descent without engine power
     * 
     * **Integration with AP_Soaring:**
     * ```cpp
     * if (soaring.is_active() && soaring.is_in_thermal()) {
     *     tecs.set_gliding_requested_flag(true);  // Enable gliding in thermal
     * } else {
     *     tecs.set_gliding_requested_flag(false); // Normal powered flight
     * }
     * ```
     * 
     * **Difference from Propulsion Failure:**
     * - gliding_requested: Intentional, pilot/autopilot wants to glide
     * - propulsion_failed: Emergency, engine/motor has failed unexpectedly
     * Both result in zero throttle, but propulsion failure may trigger additional
     * safety responses (terrain awareness, forced landing planning).
     * 
     * @param[in] gliding_requested true to enable gliding mode, false for normal powered flight
     * 
     * @note Flag must be set each control loop iteration if gliding is to be maintained.
     * Typical pattern: AP_Soaring sets flag true when in thermal, false when cruising.
     * 
     * @note Gliding mode does NOT disable throttle control loops; it forces their output
     * to zero. This allows smooth transition back to powered flight when gliding ends.
     * 
     * @note TECS automatically detects gliding conditions even if flag not set:
     * - THR_MAX parameter set to 0 (glider aircraft configuration)
     * - Propulsion failure detected
     * These automatic detections set internal is_gliding flag.
     * 
     * @warning In gliding mode with insufficient altitude or unfavorable glide ratio,
     * aircraft may not reach intended waypoint. Mission planning must account for
     * glide performance (L/D ratio, sink rate in still air).
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:122-124
     */
    void set_gliding_requested_flag(bool gliding_requested) {
        _flags.gliding_requested = gliding_requested;
    }

    /**
     * @brief Signal propulsion system failure for emergency energy management
     * 
     * @details Sets the propulsion_failed flag to indicate engine/motor failure, triggering
     * emergency energy management strategies. TECS transitions to best-glide configuration
     * and begins planning for forced landing.
     * 
     * **Emergency Responses When Propulsion Failed:**
     * - Throttle output forced to zero (no thrust available)
     * - Pitch commanded to best glide speed for maximum range/endurance
     * - Energy management optimized for reaching forced landing site
     * - Altitude prioritized only to extent needed for forced landing approach
     * - May trigger higher-level autopilot forced landing sequence
     * 
     * **Best Glide Configuration:**
     * TECS automatically targets airspeed that maximizes L/D ratio (glide ratio):
     * - Maximizes distance traveled per altitude lost
     * - Extends time and distance available for finding landing site
     * - Typically near minimum drag speed, often similar to cruise speed
     * 
     * **Detection Integration:**
     * Propulsion failure detection (if implemented) considers:
     * - Throttle commanded high but low RPM/thrust measured
     * - Engine temperature anomalies
     * - ESC telemetry indicating motor failure
     * - Battery voltage collapse under load
     * Currently, detection logic not implemented in TECS; must be set externally.
     * 
     * **Usage Example:**
     * ```cpp
     * if (engine_monitor.has_failed()) {
     *     tecs.set_propulsion_failed_flag(true);
     *     // Trigger forced landing sequence
     *     plane.set_mode(Mode::Number::QLAND, ModeReason::BATTERY_FAILSAFE);
     * }
     * ```
     * 
     * **Difference from Gliding Mode:**
     * - propulsion_failed: Emergency condition, unintended power loss
     * - gliding_requested: Normal operation, intentional zero-throttle flight
     * Both result in zero throttle, but propulsion failure is safety-critical state
     * requiring immediate action and possible emergency landing.
     * 
     * @param[in] propulsion_failed true if engine/motor has failed, false if operating normally
     * 
     * @note This is a safety-critical flag. Setting true should trigger:
     * - Emergency descent planning
     * - Forced landing site selection
     * - Terrain collision avoidance checks
     * - Emergency telemetry notification
     * 
     * @note Once set true, typically remains true for remainder of flight unless
     * propulsion is restored (restart, redundant system activation).
     * 
     * @note TECS does not implement propulsion failure detection internally. External
     * system (AP_ICEngine, AP_Motors, battery monitor) must detect failure and set flag.
     * 
     * @warning Safety-Critical Flag. Incorrect usage can cause:
     * - False positive: Unnecessary emergency landing, mission abort
     * - False negative: Failure to respond to actual propulsion loss, potential crash
     * 
     * @warning When propulsion fails at low altitude, aircraft may not have sufficient
     * energy (altitude) to reach safe landing site. Immediate action required.
     * 
     * @warning Do not use this flag for intentional gliding (thermal soaring, etc.).
     * Use set_gliding_requested_flag() instead for non-emergency gliding operations.
     * 
     * Source: libraries/AP_TECS/AP_TECS.h:127-129
     */
    void set_propulsion_failed_flag(bool propulsion_failed) {
        _flags.propulsion_failed = propulsion_failed;
    }

    /**
     * @brief Override minimum throttle limit for one control cycle
     * 
     * @details Temporarily overrides the minimum throttle limit that TECS will command.
     * This is typically used by external systems to enforce temporary throttle constraints
     * based on specific flight conditions or safety requirements.
     * 
     * **Common Use Cases:**
     * 1. **Takeoff:** Enforce minimum takeoff throttle (e.g., 70%) to ensure climb performance
     * 2. **Flaps Deployment:** Increase minimum throttle when flaps deployed (higher drag)
     * 3. **Icing Conditions:** Maintain higher minimum throttle to prevent engine ice-up
     * 4. **Emergency:** Force minimum throttle for go-around or obstacle avoidance
     * 5. **Servo Protection:** Limit throttle range if servo near limits (called by servos.cpp)
     * 
     * **Override Duration:**
     * - Override persists indefinitely until different value set or reset
     * - Typical usage: Set each loop iteration when constraint active
     * - Automatically cleared when constraint no longer needed (set back to parameter default)
     * 
     * **Slew Rate Limiter Reset:**
     * If reset_output = true:
     * - Throttle rate limiter immediately resets to respect new minimum
     * - Prevents slow ramp-up if current throttle below new minimum
     * - Use when immediate throttle response required (takeoff, go-around)
     * - Rate limiter decay controlled by TECS_THR_ERATE parameter
     * 
     * **Interaction with Parameters:**
     * - Normal minimum: THR_MIN parameter
     * - Override minimum: This function (higher priority)
     * - Actual minimum: max(THR_MIN, thr_min override)
     * 
     * **Range Explanation:**
     * - [-1.0, 1.0]: Full throttle range including reverse thrust
     * - Typical values: [0.0, 1.0] for conventional aircraft (no reverse thrust)
     * - Negative values: Reverse thrust if supported by propulsion system
     * 
     * @param[in] thr_min Minimum throttle limit in range [-1.0, 1.0]
     *            - -1.0 = Full reverse thrust (if supported)
     *            - 0.0 = Idle throttle
     *            - 0.5 = 50% throttle minimum
     *            - 1.0 = Full forward throttle
     * @param[in] reset_output If true, reset throttle slew limiter to immediately respect new minimum
     *            - true: Immediate throttle increase if below new minimum (use for takeoff, go-around)
     *            - false: Gradual ramp to new minimum via slew limiter (use for gentle constraints)
     * 
     * @note Override must be set EACH control loop iteration if persistent constraint needed.
     * Does not automatically reset; last set value remains active until changed.
     * 
     * @note Typically called by:
     * - Takeoff logic (enforce minimum takeoff throttle)
     * - servos.cpp::apply_throttle_limits() (servo safety limits)
     * - Flap control (compensate for additional drag)
     * 
     * @warning If thr_min set higher than maximum achievable throttle (thr_max), TECS
     * behavior is undefined. Ensure thr_min <= thr_max always.
     * 
     * @warning Excessive minimum throttle (e.g., thr_min = 1.0) prevents descent and can
     * cause altitude overshoot. Use conservatively and only when operationally required.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void set_throttle_min(const float thr_min, bool reset_output = false);

    /**
     * @brief Override maximum throttle limit for one control cycle
     * 
     * @details Temporarily overrides the maximum throttle limit that TECS will command.
     * This single-cycle override is typically used to enforce instantaneous throttle
     * constraints based on real-time conditions.
     * 
     * **Common Use Cases:**
     * 1. **Landing Approach:** Limit maximum throttle during final approach for idle descent
     * 2. **Overspeed Protection:** Reduce throttle if approaching maximum safe airspeed
     * 3. **Servo Protection:** Limit throttle if servo near limits (called by servos.cpp)
     * 4. **Thermal Soaring:** Limit throttle when in thermal (gliding with minimal power)
     * 5. **Battery Protection:** Reduce max throttle when battery voltage low
     * 
     * **Single-Cycle Override:**
     * - Override applies for ONE control cycle only
     * - Automatically reverts to parameter default on next update_pitch_throttle() call
     * - Must be called EVERY loop iteration to maintain persistent constraint
     * - Design prevents stuck throttle limits if calling code fails or crashes
     * 
     * **Interaction with Parameters:**
     * - Normal maximum: THR_MAX parameter
     * - Override maximum: This function (higher priority for one cycle)
     * - Actual maximum: min(THR_MAX, thr_max override)
     * 
     * **Range:**
     * - [0.0, 1.0]: Throttle range from idle to full forward thrust
     * - 0.0 = Idle throttle (no thrust, gliding)
     * - 0.5 = 50% maximum throttle
     * - 1.0 = Full throttle (no limitation)
     * 
     * @param[in] thr_max Maximum throttle limit in range [0.0, 1.0]
     *            - 0.0 = Idle only (forced glide)
     *            - 0.5 = Limit to 50% throttle
     *            - 1.0 = No limitation (full throttle available)
     * 
     * @note Override duration: ONE control cycle only. Automatically clears on next
     * update_pitch_throttle() call. Set every loop if persistent limit needed.
     * 
     * @note Typical call pattern:
     * ```cpp
     * if (condition_requires_throttle_limit) {
     *     tecs.set_throttle_max(0.5f);  // Limit to 50% this cycle
     * }
     * tecs.update_pitch_throttle(...);  // Override applies here, then clears
     * ```
     * 
     * @note Called by:
     * - Landing sequencer (limit throttle during approach)
     * - servos.cpp::apply_throttle_limits() (servo safety)
     * - Overspeed protection logic
     * - Thermal soaring controller
     * 
     * @warning If thr_max set lower than minimum throttle (thr_min), TECS behavior
     * undefined. Ensure thr_max >= thr_min always.
     * 
     * @warning Setting thr_max = 0.0 forces gliding flight. If insufficient altitude or
     * adverse glide ratio, aircraft may not reach intended destination. Use with caution.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void set_throttle_max(const float thr_max);

    /**
     * @brief Override minimum pitch limit for one control cycle
     * 
     * @details Temporarily overrides the minimum pitch angle (maximum nose-down) that
     * TECS will command. Single-cycle override typically used to enforce instantaneous
     * pitch constraints based on flight phase or safety requirements.
     * 
     * **Common Use Cases:**
     * 1. **Takeoff:** Enforce positive pitch (e.g., pitch_min = +5°) to ensure climb
     * 2. **Near Ground:** Prevent nose-down when close to terrain (pitch_min = 0°)
     * 3. **Stall Recovery:** Temporarily allow lower pitch for speed recovery
     * 4. **Landing Flare:** Adjust pitch limits during flare transition
     * 5. **Safety Limits:** Prevent excessive dive angle in all flight phases
     * 
     * **Single-Cycle Override:**
     * - Override applies for ONE control cycle only
     * - Automatically reverts to parameter default (TECS_PITCH_MIN) on next call
     * - Must be set EVERY loop iteration to maintain persistent constraint
     * - Prevents stuck pitch limits if calling code fails
     * 
     * **Interaction with Parameters:**
     * - Normal minimum: TECS_PITCH_MIN parameter (degrees)
     * - Override minimum: This function (higher priority for one cycle)
     * - Actual minimum: max(TECS_PITCH_MIN, pitch_min override)
     * 
     * **Pitch Convention:**
     * - Positive pitch: Nose up (climbing, altitude gain)
     * - Negative pitch: Nose down (descending, altitude loss)
     * - pitch_min: Maximum nose-down limit (minimum pitch angle)
     * 
     * **Typical Values:**
     * - Normal flight: -20° to -30° (allows gentle to moderate descent)
     * - Takeoff: 0° to +5° (prevent nose-down during climbout)
     * - Landing: -5° to 0° (prevent excessive dive on approach)
     * - Emergency descent: -45° (steep descent if needed)
     * 
     * @param[in] pitch_min Minimum pitch angle in degrees
     *            - Negative values: Allow nose-down (typical: -20° to -30°)
     *            - Zero: Level pitch minimum (no nose-down allowed)
     *            - Positive values: Require nose-up (used during takeoff)
     *            - Example: pitch_min = -20 means no more than 20° nose-down
     * 
     * @note Override duration: ONE control cycle only. Clears automatically on next
     * update_pitch_throttle() call. Set every loop if persistent limit needed.
     * 
     * @note Typical call pattern:
     * ```cpp
     * if (takeoff_phase) {
     *     tecs.set_pitch_min(5.0f);  // Enforce minimum +5° nose-up
     * }
     * tecs.update_pitch_throttle(...);  // Override applies, then clears
     * ```
     * 
     * @note Called by:
     * - Takeoff logic (enforce climb-out pitch)
     * - Terrain avoidance (prevent nose-down near ground)
     * - Landing sequencer (adjust pitch limits during flare)
     * 
     * @warning If pitch_min set higher than pitch_max, no valid pitch output possible.
     * Ensure pitch_min <= pitch_max always. TECS may saturate at pitch_min if conflict occurs.
     * 
     * @warning Excessively high pitch_min (e.g., +30°) during cruise can cause airspeed
     * loss and potential stall. Use high values only during takeoff or when appropriate.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void set_pitch_min(const float pitch_min);

    /**
     * @brief Override maximum pitch limit for one control cycle
     * 
     * @details Temporarily overrides the maximum pitch angle (maximum nose-up) that TECS
     * will command. Single-cycle override typically used to enforce instantaneous pitch
     * constraints based on flight phase or aerodynamic limits.
     * 
     * **Common Use Cases:**
     * 1. **Stall Prevention:** Limit nose-up pitch to safe angle of attack (e.g., +20°)
     * 2. **Landing Approach:** Reduce max pitch during approach for stable glide slope
     * 3. **High Speed:** Limit pitch-up at high airspeed (structural loads)
     * 4. **Low Altitude:** Prevent excessive pitch-up near ground (tail strike risk)
     * 5. **Wind Conditions:** Adjust pitch limits in strong wind or turbulence
     * 
     * **Single-Cycle Override:**
     * - Override applies for ONE control cycle only
     * - Automatically reverts to parameter default (TECS_PITCH_MAX) on next call
     * - Must be set EVERY loop iteration to maintain persistent constraint
     * - Prevents stuck pitch limits if calling code fails
     * 
     * **Interaction with Parameters:**
     * - Normal maximum: TECS_PITCH_MAX parameter (degrees)
     * - Override maximum: This function (higher priority for one cycle)
     * - Actual maximum: min(TECS_PITCH_MAX, pitch_max override)
     * 
     * **Pitch Convention:**
     * - Positive pitch: Nose up (climbing, altitude gain)
     * - Negative pitch: Nose down (descending, altitude loss)
     * - pitch_max: Maximum nose-up limit (maximum pitch angle)
     * 
     * **Typical Values:**
     * - Normal flight: +15° to +25° (safe climb angle)
     * - High speed: +10° (prevent excessive g-loading)
     * - Landing approach: +10° to +15° (stable approach)
     * - Takeoff: +20° to +30° (maximum climb rate)
     * 
     * @param[in] pitch_max Maximum pitch angle in degrees
     *            - Positive values: Limit nose-up angle (typical: +15° to +25°)
     *            - Zero: No nose-up allowed (forced level or descent only)
     *            - Negative values: Unusual, forces nose-down (emergency descent)
     *            - Example: pitch_max = +20 means no more than 20° nose-up
     * 
     * @note Override duration: ONE control cycle only. Clears automatically on next
     * update_pitch_throttle() call. Set every loop if persistent limit needed.
     * 
     * @note Typical call pattern:
     * ```cpp
     * if (high_airspeed) {
     *     tecs.set_pitch_max(10.0f);  // Limit to +10° nose-up at high speed
     * }
     * tecs.update_pitch_throttle(...);  // Override applies, then clears
     * ```
     * 
     * @note Called by:
     * - Takeoff logic (may adjust max pitch during climbout)
     * - Overspeed protection (limit pitch-up at high airspeed)
     * - Landing sequencer (reduce max pitch during approach)
     * - Stall prevention (limit angle of attack)
     * 
     * @warning If pitch_max set lower than pitch_min, no valid pitch output possible.
     * Ensure pitch_max >= pitch_min always. TECS may saturate at pitch_max if conflict occurs.
     * 
     * @warning Excessively low pitch_max (e.g., +5°) prevents adequate climb performance.
     * Aircraft may be unable to achieve altitude targets or avoid terrain. Use conservatively.
     * 
     * @warning Pitch limits affect stall margin. Setting pitch_max too high (e.g., +35°)
     * can cause stall, spin, or loss of control. Respect aircraft aerodynamic limits.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void set_pitch_max(const float pitch_max);

    /**
     * @brief Force use of synthetic airspeed estimation for one control loop iteration
     * 
     * @details Commands TECS to use synthetic airspeed (GPS groundspeed + wind estimate)
     * instead of pitot tube airspeed sensor for the next control cycle. This is a fallback
     * mode when airspeed sensor data is temporarily invalid or unreliable.
     * 
     * **When to Use:**
     * 1. **Airspeed Sensor Failure:** Detected sensor malfunction or unrealistic readings
     * 2. **Airspeed Sensor Degradation:** Low quality airspeed data (high noise, drift)
     * 3. **Pitot Tube Blockage:** Ice, insects, or debris blocking pressure ports
     * 4. **Sensor Warm-up:** Airspeed sensor not yet initialized after power-on
     * 5. **Manual Override:** Pilot or GCS commands synthetic airspeed mode
     * 
     * **Synthetic Airspeed Calculation:**
     * - True Airspeed = √[(GPS_velocity - Wind_velocity) · (GPS_velocity - Wind_velocity)]
     * - Requires valid GPS velocity and wind estimate from AHRS/EKF
     * - Less accurate than pitot tube, especially in turbulence or wind shear
     * - Wind estimate errors directly affect synthetic airspeed accuracy
     * 
     * **Impact on TECS Control:**
     * - Throttle control degrades to height-rate-based (less optimal)
     * - Energy balance control uses less accurate airspeed estimate
     * - Increased susceptibility to wind gusts and turbulence
     * - May cause altitude and airspeed tracking degradation
     * - Underspeed protection still functional but with reduced accuracy
     * 
     * **Duration:**
     * - Override applies for ONE control loop iteration only
     * - Automatically clears after update_pitch_throttle() execution
     * - Must be called EVERY loop if persistent synthetic airspeed needed
     * - Design prevents accidentally staying in degraded mode
     * 
     * **Parameter Override:**
     * This function overrides TECS_SYNAIRSPEED parameter for one cycle:
     * - TECS_SYNAIRSPEED = 0: Use pitot tube (normal)
     * - TECS_SYNAIRSPEED = 1: Always use synthetic airspeed
     * - use_synthetic_airspeed(): Force synthetic for one cycle regardless of parameter
     * 
     * @note Typical call pattern:
     * ```cpp
     * if (airspeed_sensor_unhealthy()) {
     *     tecs.use_synthetic_airspeed();  // Use synthetic this cycle
     * }
     * tecs.update_pitch_throttle(...);  // Applies synthetic, then reverts
     * ```
     * 
     * @note Called by:
     * - Airspeed sensor health monitoring
     * - Sensor failure detection logic
     * - Manual mode overrides from GCS
     * 
     * @note Performance Degradation:
     * - Altitude tracking: Moderate degradation (±5-10m typical)
     * - Airspeed tracking: Significant degradation (±2-5 m/s typical)
     * - Response to disturbances: Slower, less precise
     * - Fuel efficiency: May decrease due to suboptimal throttle control
     * 
     * @warning Synthetic airspeed mode provides degraded performance. Use only when
     * pitot tube airspeed unavailable or unreliable. Return to normal mode ASAP.
     * 
     * @warning In strong wind or wind shear conditions, synthetic airspeed can have
     * large errors (>10 m/s). Monitor aircraft behavior closely in synthetic mode.
     * 
     * @warning If GPS velocity or wind estimate invalid, synthetic airspeed will be
     * incorrect, potentially causing dangerous control behavior. Ensure GPS and AHRS
     * healthy before relying on synthetic airspeed.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void use_synthetic_airspeed(void) {
        _use_synthetic_airspeed_once = true;
    }

    /**
     * @brief Request TECS reset on next control loop iteration
     * 
     * @details Schedules a complete reinitialization of TECS state estimators, integrators,
     * and control loops on the next call to update_pitch_throttle(). This is required after
     * significant discontinuities or changes that invalidate current state estimates.
     * 
     * **When to Call reset():**
     * 1. **Mode Changes:** Switching between flight modes (AUTO, CRUISE, FBWB, etc.)
     * 2. **GPS Reset/Jump:** Large position correction from GPS (>50m)
     * 3. **Parameter Changes:** TECS tuning parameters modified in-flight
     * 4. **Reference Changes:** Home position or field elevation updated
     * 5. **EKF Reset:** AHRS/EKF reinitialized due to sensor switch or divergence
     * 6. **Manual Override Exit:** Returning to automatic control after manual flight
     * 7. **Failsafe Recovery:** Resuming normal control after failsafe event
     * 8. **Unrealistic State:** Detected filter divergence or unrealistic estimates
     * 
     * **What Gets Reset:**
     * - Height and height rate complementary filter states
     * - Airspeed and airspeed rate complementary filter states
     * - All integrators: throttle (_integTHR_state), pitch (_integSEBdot), kinetic energy (_integKE)
     * - Rate limiters for throttle and pitch commands
     * - Energy estimation variables (SPE, SKE, SPEdot, SKEdot)
     * - Flare state variables (if in landing)
     * - Flag states (underspeed, bad descent, etc.)
     * 
     * **Reset Execution:**
     * - reset() only sets _need_reset flag; actual reset deferred
     * - Reset occurs at beginning of next update_pitch_throttle() call
     * - Smooth reinitialization: States initialized to current sensor values
     * - No discontinuous control outputs during reset
     * 
     * **Reinitialization Process:**
     * Called _initialise_states() function:
     * 1. Height filter: Initialized to current barometric altitude
     * 2. Speed filter: Initialized to current airspeed or GPS groundspeed
     * 3. Integrators: Reset to zero (no residual windup)
     * 4. Rate limiters: Reset to allow immediate response
     * 5. Demanded states: Set to current actual states (no initial error)
     * 
     * **Avoiding Unnecessary Resets:**
     * - Resets cause temporary control degradation (transient response)
     * - Integrators lose accumulated trim (may cause steady-state error temporarily)
     * - Only reset when necessary; prefer continuous operation
     * 
     * **Typical Call Pattern:**
     * ```cpp
     * // On mode change
     * if (mode_changed_to_auto) {
     *     tecs.reset();  // Schedule reset
     * }
     * // Reset executes here on next call
     * tecs.update_pitch_throttle(...);
     * ```
     * 
     * @note Reset is deferred (not immediate) to ensure proper sequencing with other
     * TECS operations. Flag is cleared automatically after reset executes.
     * 
     * @note Multiple calls to reset() before update_pitch_throttle() have no additional
     * effect; only one reset occurs on next update.
     * 
     * @note After reset, control performance may be suboptimal for 1-5 seconds while
     * filters and integrators re-converge to steady-state.
     * 
     * @note Called by:
     * - Flight mode change logic (mode.cpp)
     * - AHRS/EKF reset handlers
     * - Failsafe recovery logic
     * - Parameter update callbacks
     * 
     * @warning Frequent resets (>1 Hz) prevent proper controller convergence and cause
     * poor tracking performance. Avoid calling reset() unnecessarily.
     * 
     * @warning Do NOT call reset() during time-critical phases like landing flare or
     * takeoff rotation unless absolutely necessary. Reset causes temporary performance loss.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp::_initialise_states()
     */
    void reset(void) {
        _need_reset = true;
    }

    /**
     * @brief Apply altitude offset to height demand tracking
     * 
     * @details Applies an instantaneous offset to TECS height demand states to compensate
     * for changes in the altitude reference frame (typically home altitude). This prevents
     * transient altitude errors when reference altitude is updated mid-flight.
     * 
     * **Purpose:**
     * When home altitude changes (GPS altitude reference, barometric pressure correction),
     * all height-related TECS states become incorrect relative to new reference. This
     * function applies a matching offset to maintain continuity without causing transients.
     * 
     * **Common Scenarios:**
     * 1. **Home Position Update:** New home set in-flight, changing altitude datum
     * 2. **Barometer Calibration:** Ground pressure reference updated for QNH change
     * 3. **Altitude Datum Change:** Switch between GPS altitude and barometric altitude
     * 4. **Multi-Home Missions:** Rally point becomes new home with different elevation
     * 5. **Barometer Drift Compensation:** Long flight with significant pressure change
     * 
     * **What Gets Offset:**
     * - _hgt_dem_in_raw: Raw height demand input
     * - _hgt_dem_in: Height demand after rate limiting
     * - _hgt_dem_lpf: Low-pass filtered height demand
     * - _hgt_dem: Final height demand to control loops
     * - _hgt_dem_prev: Previous height demand
     * - _flare_hgt_dem_adj: Flare height demand (if in landing)
     * - _flare_hgt_dem_ideal: Ideal flare height
     * - _post_TO_hgt_offset: Post-takeoff height offset
     * 
     * **What Does NOT Change:**
     * - Current estimated height (_height): Reflects actual altitude
     * - Height rate estimates (_climb_rate): Rate unchanged by offset
     * - Integrator states: Maintained to preserve trim
     * - Airspeed demands: Independent of altitude reference
     * 
     * **Offset Sign Convention:**
     * - Positive alt_offset: New reference altitude higher than old (increase all height demands)
     * - Negative alt_offset: New reference altitude lower than old (decrease all height demands)
     * - Example: If home altitude increased by 10m, call offset_altitude(10.0f)
     * 
     * **Preventing Transients:**
     * Without offset_altitude():
     * - Home altitude increases by 10m
     * - Height demand remains at old value (now 10m too low relative to new reference)
     * - TECS thinks aircraft 10m high, commands descent
     * - Unwanted altitude loss occurs
     * 
     * With offset_altitude(10.0f):
     * - Home altitude increases by 10m
     * - All height demands increased by 10m to match new reference
     * - TECS sees no altitude error, maintains current flight path
     * - No transient response
     * 
     * @param[in] alt_offset Altitude offset to apply in meters
     *            - Positive: Increase height demands (new reference altitude higher)
     *            - Negative: Decrease height demands (new reference altitude lower)
     *            - Typical range: ±10m to ±100m
     *            - Example: alt_offset = 15.5 means add 15.5m to all height demands
     * 
     * @note Called immediately when home altitude or altitude reference changes.
     * Must be called BEFORE next update_pitch_throttle() to prevent transient.
     * 
     * @note Typical call pattern:
     * ```cpp
     * float altitude_change = new_home_alt - old_home_alt;
     * tecs.offset_altitude(altitude_change);
     * // Now TECS height demands match new reference
     * ```
     * 
     * @note Called by:
     * - Home position update handlers
     * - Barometer calibration routines
     * - Altitude reference frame change logic
     * - Rally point home switching
     * 
     * @note This is a pure offset operation, not a reset. Filters and integrators
     * maintain their state; only demand references are shifted.
     * 
     * @warning Large offsets (>100m) may indicate incorrect reference update or GPS error.
     * Verify altitude changes are realistic before applying large offsets.
     * 
     * @warning Do NOT call offset_altitude() for aircraft altitude changes (climbing/descending).
     * Only use for changes in REFERENCE altitude (home, datum). Aircraft motion handled
     * automatically by TECS height tracking.
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (implementation)
     */
    void offset_altitude(const float alt_offset);

    /**
     * @brief Parameter table for user-configurable TECS tuning parameters
     * 
     * @details Static table defining all TECS_* parameters exposed to users via ground
     * control stations for tuning and configuration. This table is used by AP_Param
     * system for parameter storage, retrieval, and EEPROM persistence.
     * 
     * **Parameters Defined (TECS_* namespace):**
     * 
     * **Time Constants and Damping:**
     * - TECS_TIME_CONST: Primary time constant for altitude/airspeed response (seconds)
     *   - Controls how aggressively TECS responds to errors
     *   - Typical: 3-10 seconds (smaller = more aggressive, larger = smoother)
     * - TECS_THR_DAMP: Throttle damping gain [0.1-1.0]
     *   - Prevents throttle-induced phugoid oscillations
     * - TECS_PTCH_DAMP: Pitch damping gain [0.0-1.0]
     *   - Reduces pitch oscillations and overshoot
     * 
     * **Performance Limits:**
     * - TECS_CLMB_MAX: Maximum climb rate in m/s
     *   - Must match or be less than actual aircraft climb capability
     * - TECS_SINK_MIN: Minimum sink rate in m/s (fastest descent at cruise speed)
     * - TECS_SINK_MAX: Maximum sink rate in m/s (fastest descent at any speed)
     * - TECS_PITCH_MAX: Maximum pitch angle in degrees
     * - TECS_PITCH_MIN: Minimum pitch angle in degrees
     * 
     * **Speed/Height Priority:**
     * - TECS_SPDWEIGHT: Speed vs height priority weighting [0.0-2.0]
     *   - 0.0 = Full height priority (altitude rigid, airspeed loose)
     *   - 1.0 = Balanced (default)
     *   - 2.0 = Full speed priority (airspeed rigid, altitude loose)
     * 
     * **Integrator Gains:**
     * - TECS_INTEG_GAIN: Integrator gain for cruise [0.0-0.5]
     * - TECS_INTEG_GAIN_TKOFF: Integrator gain during takeoff
     * - TECS_INTEG_GAIN_LAND: Integrator gain during landing
     * 
     * **Landing-Specific:**
     * - TECS_LAND_ARSPD: Target airspeed for landing in m/s
     * - TECS_LAND_SINK: Target sink rate during landing in m/s
     * - TECS_LAND_SPDWGT: Speed weighting during landing [0.0-2.0]
     * - TECS_LAND_THR: Throttle for landing (percentage)
     * - TECS_LAND_TCONST: Time constant for landing (seconds)
     * - TECS_LAND_PDAMP: Pitch damping for landing
     * - TECS_LAND_TDAMP: Throttle damping for landing
     * - TECS_LAND_SRC: Sink rate change during flare in m/s/s
     * - TECS_FLARE_HGT: Height above ground to start flare in meters
     * 
     * **Advanced:**
     * - TECS_VERT_ACC: Vertical acceleration limit in m/s²
     * - TECS_HGT_OMEGA: Height complementary filter frequency in rad/s
     * - TECS_SPD_OMEGA: Speed complementary filter frequency in rad/s
     * - TECS_RLL_CCOMP: Roll angle compensation factor [0.0-1.0]
     * - TECS_SYNAIRSPEED: Use synthetic airspeed [0=pitot, 1=synthetic]
     * - TECS_OPTIONS: Bitmask for special options
     * - TECS_PITCH_FF_V0: Pitch feedforward at zero airspeed in degrees
     * - TECS_PITCH_FF_K: Pitch feedforward gain per m/s
     * 
     * **Parameter Storage:**
     * - Stored in EEPROM via AP_Param system
     * - Accessible via MAVLink parameter protocol
     * - Configurable via ground control station (Mission Planner, QGroundControl, etc.)
     * - Can be changed in-flight (some may require reset() call)
     * 
     * **Parameter Loading:**
     * Parameters loaded in this sequence:
     * 1. Defaults from var_info table (in AP_TECS.cpp)
     * 2. User values from EEPROM (if previously saved)
     * 3. GCS parameter updates (during flight)
     * 
     * @note This table is defined in AP_TECS.cpp with full parameter metadata including:
     * - Display names and descriptions for GCS
     * - Valid ranges and default values
     * - Units and increment values
     * - Parameter groups and sub-groups
     * 
     * @note Changes to parameter names or types require parameter version migration
     * code to preserve user tuning across firmware updates.
     * 
     * @warning Modifying var_info structure requires understanding of AP_Param system.
     * Incorrect changes can cause parameter corruption or loss of user tuning.
     * 
     * @see AP_Param::GroupInfo for parameter table structure
     * @see libraries/AP_TECS/AP_TECS.cpp for var_info definition
     * 
     * Source: libraries/AP_TECS/AP_TECS.cpp (var_info definition)
     */
    static const struct AP_Param::GroupInfo var_info[];

private:
    // Last time update_50Hz was called
    uint64_t _update_50hz_last_usec;

    // Last time update_pitch_throttle was called
    uint64_t _update_pitch_throttle_last_usec;

    // reference to the AHRS object
    AP_AHRS &_ahrs;

    const AP_FixedWing &aparm;

    // reference to const AP_Landing to access it's params
    const AP_Landing &_landing;

    // Logging  bitmask
    const uint32_t _log_bitmask;

    // TECS tuning parameters
    AP_Float _hgtCompFiltOmega;
    AP_Float _spdCompFiltOmega;
    AP_Float _maxClimbRate;
    AP_Float _minSinkRate;
    AP_Float _maxSinkRate;
    AP_Float _timeConst;
    AP_Float _landTimeConst;
    AP_Float _ptchDamp;
    AP_Float _land_pitch_damp;
    AP_Float _landDamp;
    AP_Float _thrDamp;
    AP_Float _land_throttle_damp;
    AP_Float _integGain;
    AP_Float _integGain_takeoff;
    AP_Float _integGain_land;
    AP_Float _vertAccLim;
    AP_Float _rollComp;
    AP_Float _spdWeight;
    AP_Float _spdWeightLand;
    AP_Float _landThrottle;
    AP_Float _landAirspeed;
    AP_Float _land_sink;
    AP_Float _land_sink_rate_change;
    AP_Int8  _pitch_max;
    AP_Int8  _pitch_min;
    AP_Int8  _land_pitch_max;
    AP_Float _maxSinkRate_approach;
    AP_Int32 _options;
    AP_Float _flare_holdoff_hgt;
    AP_Float _hgt_dem_tconst;

    enum class Option {
        GLIDER_ONLY     = (1<<0),
        DESCENT_SPEEDUP = (1<<1)
    };

    bool option_is_set(const Option option) const {
        return (_options.get() & int32_t(option)) != 0;
    }

    AP_Float _pitch_ff_v0;
    AP_Float _pitch_ff_k;
    AP_Float _accel_gf;
    AP_Int8 _thr_min_pct_ext_rate_lim;

    // current height estimate (above field elevation)
    float _height;

    // throttle demand in the range from -1.0 to 1.0, usually positive unless reverse thrust is enabled via _THRminf < 0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // estimated climb rate (m/s)
    float _climb_rate;

    // climb and sink rate limits
    float _climb_rate_limit;
    float _sink_rate_limit;

    /*
      a filter to estimate climb rate if we don't have it from the EKF
     */
    struct {
        // height filter second derivative
        float dd_height;

        // height integration
        float height;
    } _height_filter;

    // Integrator state 4 - airspeed filter first derivative
    float _integDTAS_state;

    // Integrator state 5 - true airspeed
    float _TAS_state;

    // Integrator state 6 - throttle integrator
    float _integTHR_state;

    // energy balance error integral
    float _integSEBdot;

    // pitch demand kinetic energy error integral
    float _integKE;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;
    float _vel_dot_lpf;

    // True airspeed limits
    float _TASmax;
    float _TASmin;

    // Current true airspeed demand
    float _TAS_dem;

    // Equivalent airspeed demand
    float _EAS_dem;

    // height demands
    float _hgt_dem_in_raw;      // height demand input from autopilot before any modification (m)
    float _hgt_dem_in;          // height demand input from autopilot after unachievable climb or descent limiting (m)
    float _hgt_dem_in_prev;     // previous value of _hgt_dem_in (m)
    float _hgt_dem_lpf;         // height demand after application of low pass filtering (m)
    float _flare_hgt_dem_adj;   // height rate demand during flare adjusted for height tracking offset at flare entry (m)
    float _flare_hgt_dem_ideal; // height we want to fly at during flare (m)
    float _hgt_dem;             // height demand sent to control loops (m)
    float _hgt_dem_prev;        // _hgt_dem from previous frame (m)

    // height rate demands
    float _hgt_dem_rate_ltd;    // height demand after application of the rate limiter (m)
    float _hgt_rate_dem;        // height rate demand sent to control loops

    // offset applied to height demand post takeoff to compensate for height demand filter lag
    float _post_TO_hgt_offset;

    // last lag compensation offset applied to height demand
    float _lag_comp_hgt_offset;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;
    float _TAS_rate_dem_lpf;

    // Total energy rate filter state
    float _STEdotErrLast;

    struct flags {
        // Underspeed condition
        bool underspeed:1;

        // Bad descent condition caused by unachievable airspeed demand
        bool badDescent:1;

        // true when plane is in auto mode and executing a land mission item
        bool is_doing_auto_land:1;

        // true when we have reached target speed in takeoff
        bool reached_speed_takeoff:1;

        // true if the soaring feature has requested gliding flight
        bool gliding_requested:1;

        // true when we are in gliding flight, in one of three situations;
        //   - THR_MAX=0
        //   - gliding has been requested e.g. by soaring feature
        //   - engine failure detected (detection not implemented currently)
        bool is_gliding:1;

        // true if a propulsion failure is detected.
        bool propulsion_failed:1;

        // true when a reset of airspeed and height states to current is performed on this frame
        bool reset:1;

    };
    union {
        struct flags _flags;
        uint8_t _flags_byte;
    };

    // time when underspeed started
    uint32_t _underspeed_start_ms;

    // auto mode flightstage
    enum AP_FixedWing::FlightStage _flight_stage;

    // pitch demand before limiting
    float _pitch_dem_unc;

    // Specific total energy rate limits
    float _STEdot_max;     // Specific total energy rate gain at cruise airspeed & THR_MAX (m/s/s)
    float _STEdot_min;     // Specific total energy rate loss at cruise airspeed & THR_MIN (m/s/s)
    float _STEdot_neg_max; // Specific total energy rate loss at max airspeed & THR_MIN (m/s/s)

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;
    // Maximum and minimum throttle safety limits, set externally, typically by servos.cpp:apply_throttle_limits()
    float _THRmaxf_ext = 1.0f;
    float _THRminf_ext = -1.0f;
    // Maximum and minimum pitch limits, set externally, typically by the takeoff logic.
    float _PITCHmaxf_ext = 90.0f;
    float _PITCHminf_ext = -90.0f;

    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // 1 if throttle is clipping at max value, -1 if clipping at min value, 0 otherwise
    enum class clipStatus  : int8_t {
        MIN  = -1,
        NONE =  0,
        MAX  =  1,
    };
    clipStatus _thr_clip_status;

    // Specific energy quantities
    float _SPE_dem;
    float _SKE_dem;
    float _SPEdot_dem;
    float _SKEdot_dem;
    float _SPE_est;
    float _SKE_est;
    float _SPEdot;
    float _SKEdot;

    // variables used for precision landing pitch control
    float _hgt_at_start_of_flare;
    float _hgt_rate_at_flare_entry;
    float _hgt_afe;
    float _pitch_min_at_flare_entry;

    // used to scale max climb and sink limits to match vehicle ability
    float _max_climb_scaler;
    float _max_sink_scaler;
    float _sink_fraction;

    // Specific energy error quantities
    float _STE_error;

    // 1 when specific energy balance rate demand is clipping in the up direction
    // -1 when specific energy balance rate demand is clipping in the down direction
    // 0 when not clipping
    clipStatus _SEBdot_dem_clip;

    // Time since last update of main TECS loop (seconds)
    float _DT;

    // true when class variables used for flare control have been initialised
    // on flare entry
    bool _flare_initialised;

    // percent traveled along the previous and next waypoints
    float _path_proportion;

    float _distance_beyond_land_wp;

    float _land_pitch_min = -90;

    // need to reset on next loop
    bool _need_reset;
    // Flag if someone else drives throttle externally.
    bool _flag_throttle_forced;

    // Checks if we reset at the beginning of takeoff.
    bool _flag_have_reset_after_takeoff;

    float _SKE_weighting;

    AP_Int8 _use_synthetic_airspeed;
    
    // use synthetic airspeed for next loop
    bool _use_synthetic_airspeed_once;

    // using airspeed in throttle calculation this frame
    bool _using_airspeed_for_throttle;

    // low pass filters used for crossover filter that combines demanded and measured pitch
    // when calculating a pitch to throttle mapping.
    LowPassFilterFloat _pitch_demand_lpf;
    LowPassFilterFloat _pitch_measured_lpf;

    // aerodynamic load factor
    float _load_factor;

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float DT);

    // Update the demanded airspeed
    void _update_speed_demand(void);

    // Update the demanded height
    void _update_height_demand(void);

    // Detect an underspeed condition
    void _detect_underspeed(void);

    // Update Specific Energy Quantities
    void _update_energies(void);

    // Update Demanded Throttle
    void _update_throttle_with_airspeed(void);

    // Update Demanded Throttle Non-Airspeed
    void _update_throttle_without_airspeed(int16_t throttle_nudge, float pitch_trim_deg);

    // Constrain throttle demand and record clipping
    void constrain_throttle();

    // get integral gain which is flight_stage dependent
    float _get_i_gain(void);

    // Detect Bad Descent
    void _detect_bad_descent(void);

    // Update Demanded Pitch Angle
    void _update_pitch(void);

    // Initialise states and variables
    void _initialise_states(float hgt_afe);

    // Calculate specific total energy rate limits
    void _update_STE_rate_lim(void);

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    // current time constant
    float timeConstant(void) const;

    // Update the allowable throttle range.
    void _update_throttle_limits();

    // Update the allowable pitch range.
    void _update_pitch_limits(const int32_t ptchMinCO_cd);
};
