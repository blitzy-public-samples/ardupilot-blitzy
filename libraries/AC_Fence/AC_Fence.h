/**
 * @file AC_Fence.h
 * @brief ArduPilot geofencing system for flight safety boundary enforcement
 * 
 * @details This file implements the AC_Fence library, which provides comprehensive
 *          geofencing capabilities to prevent vehicles from flying outside designated
 *          safe areas. The geofencing system is a critical flight safety feature that
 *          monitors vehicle position and enforces configured boundaries in real-time.
 * 
 *          The fence system supports multiple fence types that can be combined:
 *          - Maximum altitude fence (prevents flying too high)
 *          - Minimum altitude fence (prevents flying too low, aka "floor")
 *          - Circular horizontal fence (cylindrical boundary from home/origin)
 *          - Polygon horizontal fence (arbitrary shaped horizontal boundaries)
 * 
 *          Integration with vehicle control:
 *          - Integrates with vehicle flight modes and failsafe system
 *          - Triggers configurable breach actions (RTL, Land, Brake, etc.)
 *          - Provides pre-arm checks to prevent takeoff outside boundaries
 *          - Supports manual pilot recovery with 10-second grace period
 * 
 *          Coordinate frames used:
 *          - Altitude fences: meters relative to home/origin (NED frame down component)
 *          - Circular fence: meters horizontal distance from EKF origin
 *          - Polygon fence: lat/lon coordinates checked against EKF position
 * 
 *          Update rate: The check() method should be called at approximately 10Hz
 *          for reliable breach detection and timely triggering of recovery actions.
 * 
 *          Thread safety: Polygon fence uses AC_PolyFence_loader which loads fence
 *          points asynchronously from storage. The update() method handles this loading.
 * 
 * @note This is a safety-critical system. All fence checks must complete within
 *       scheduler timing budgets to ensure timely breach detection.
 * 
 * @warning Disabling fences or configuring inappropriate breach actions could result
 *          in loss of vehicle control or flight into hazardous areas.
 * 
 * @see AC_PolyFence_loader for polygon fence storage and loading
 * @see AP_AHRS for vehicle position and altitude estimation
 * 
 * Source: libraries/AC_Fence/AC_Fence.h
 */

#pragma once

#include "AC_Fence_config.h"

#if AP_FENCE_ENABLED

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_Fence/AC_PolyFence_loader.h>

/**
 * @defgroup FenceTypes Fence Type Bitmask Definitions
 * @brief Bit masks for enabled fence types, used for TYPE parameter and enable/disable control
 * @{
 */

/**
 * @brief Maximum altitude fence bit mask
 * 
 * @details When enabled, prevents vehicle from exceeding configured maximum altitude.
 *          Checked continuously during flight against current altitude from EKF.
 *          Typical breach action: RTL (Return to Launch) to bring vehicle back down.
 *          
 *          Units: Compared against altitude in meters relative to home/origin
 *          Check frequency: Every call to check() method (~10Hz)
 * 
 * @note This is one of the "arming fences" - vehicle position must be within this
 *       fence at arming time if pre-arm checking is enabled.
 */
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL

/**
 * @brief Circular horizontal fence bit mask
 * 
 * @details When enabled, prevents vehicle from exceeding configured radius from home/origin.
 *          Creates a cylindrical boundary centered on the EKF origin point.
 *          Checked continuously during flight against horizontal distance from origin.
 *          Typical breach action: RTL (Return to Launch) to bring vehicle back inside.
 *          
 *          Units: Radius in meters from home/origin point
 *          Check frequency: Every call to check() method (~10Hz)
 * 
 * @note This is one of the "arming fences" - vehicle must be within this radius
 *       at arming time if pre-arm checking is enabled.
 * 
 * @warning Ensure fence radius is large enough for typical flight patterns to avoid
 *          nuisance breaches that could interrupt missions unnecessarily.
 */
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)

/**
 * @brief Polygon horizontal fence bit mask
 * 
 * @details When enabled, prevents vehicle from exiting configured polygon boundary(ies).
 *          Supports complex arbitrary-shaped horizontal boundaries defined by lat/lon vertices.
 *          Supports both inclusion zones (must stay inside) and exclusion zones (must stay outside).
 *          Polygon points loaded asynchronously from storage via AC_PolyFence_loader.
 *          Typical breach action: RTL or Brake depending on configuration.
 *          
 *          Check algorithm: Point-in-polygon tests using vehicle's current lat/lon position
 *          Check frequency: Every call to check() method (~10Hz)
 * 
 * @note This is one of the "arming fences" - vehicle position must be within inclusion
 *       polygons and outside exclusion polygons at arming time.
 * 
 * @note Polygon fences require valid EKF position solution. Check will not trigger
 *       breach if position estimate is unavailable.
 */
#define AC_FENCE_TYPE_POLYGON                       4       // polygon horizontal fence

/**
 * @brief Minimum altitude fence bit mask (floor)
 * 
 * @details When enabled, prevents vehicle from descending below configured minimum altitude.
 *          Also called "fence floor" - creates a lower altitude boundary.
 *          Checked continuously during flight against current altitude from EKF.
 *          Typical breach action: Climb back up or maintain altitude.
 *          
 *          Units: Compared against altitude in meters relative to home/origin
 *          Check frequency: Every call to check() method (~10Hz)
 * 
 * @note Unlike other fence types, minimum altitude fence is NOT an "arming fence".
 *       It can be on the ground at arming, so pre-arm check does not validate this fence.
 * 
 * @note Floor fence is often auto-enabled after takeoff and auto-disabled before landing
 *       to allow normal ground operations. See AutoEnable enum for configuration options.
 * 
 * @warning Setting minimum altitude too high can prevent successful landing. Use
 *          auto-disable options or manual override during landing approach.
 */
#define AC_FENCE_TYPE_ALT_MIN                       8       // low alt fence which usually initiates an RTL
/**
 * @brief Bitmask of fences checked during pre-arm validation
 * 
 * @details Combines all fence types that must be satisfied before arming:
 *          - AC_FENCE_TYPE_ALT_MAX (maximum altitude)
 *          - AC_FENCE_TYPE_CIRCLE (circular horizontal)
 *          - AC_FENCE_TYPE_POLYGON (polygon horizontal)
 * 
 *          Note: AC_FENCE_TYPE_ALT_MIN (floor) is NOT included because vehicle is
 *          typically on the ground at arming time.
 */
#define AC_FENCE_ARMING_FENCES  (AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)

/**
 * @brief Bitmask of all possible fence types
 * 
 * @details Combines all fence types supported by the system:
 *          - All arming fences (alt_max, circle, polygon)
 *          - AC_FENCE_TYPE_ALT_MIN (floor fence)
 * 
 *          Used for operations that need to reference or iterate over all fence types.
 */
#define AC_FENCE_ALL_FENCES (AC_FENCE_ARMING_FENCES | AC_FENCE_TYPE_ALT_MIN)

/** @} */ // end of FenceTypes group

/**
 * @brief Distance outside fence at which recovery is abandoned and vehicle lands
 * 
 * @details When vehicle breaches a fence and moves beyond this distance from the boundary,
 *          the vehicle control code should give up on recovery actions (RTL, Brake, etc.)
 *          and execute an immediate landing instead. This prevents prolonged attempts to
 *          return when vehicle has traveled too far outside the safe area.
 * 
 *          Units: meters
 *          Value: 100.0 meters
 * 
 * @note This constant is not used directly by AC_Fence library. It is provided as a
 *       reference value for vehicle-specific fence breach handling code to use when
 *       deciding between continued recovery attempts vs immediate landing.
 * 
 * @warning Setting this distance too small may cause premature landing when vehicle
 *          could have recovered. Setting too large may allow vehicle to travel excessive
 *          distances outside safe area during recovery attempts. Consider typical RTL
 *          distances and fence sizes when evaluating this value.
 * 
 * @warning Landing 100m outside the fence may place vehicle in hazardous or inaccessible
 *          location. Operators should ensure fence boundaries provide adequate buffer
 *          from actual hazards to accommodate this landing distance.
 */
#define AC_FENCE_GIVE_UP_DISTANCE                   100.0f  // distance outside the fence at which we should give up and just land.  Note: this is not used by library directly but is intended to be used by the main code

/**
 * @class AC_Fence
 * @brief ArduPilot geofencing system providing flight safety boundary enforcement
 * 
 * @details The AC_Fence class implements a comprehensive geofencing system that monitors
 *          vehicle position against configured boundaries and triggers recovery actions
 *          when breaches occur. This is a critical flight safety system designed to
 *          prevent vehicles from entering hazardous areas or exceeding safe operating limits.
 * 
 * Architecture Overview:
 * ----------------------
 * The fence system operates as a continuous monitoring loop that:
 * 1. Reads current vehicle position from AHRS/EKF
 * 2. Checks position against each enabled fence type
 * 3. Detects breaches and margin violations
 * 4. Records breach information for vehicle control code
 * 5. Vehicle control code implements configured breach actions
 * 
 * Fence Types Supported:
 * ----------------------
 * - **Maximum Altitude**: Prevents flying above configured ceiling (meters above home)
 * - **Minimum Altitude**: Prevents flying below configured floor (meters above home)
 * - **Circular**: Prevents exceeding horizontal radius from home/origin (meters)
 * - **Polygon**: Prevents exiting arbitrary lat/lon boundary polygons
 * 
 * Breach Detection Flow:
 * ----------------------
 * 1. check() method called at ~10Hz from vehicle scheduler
 * 2. For each enabled fence type:
 *    a. Compare current position against fence boundary
 *    b. Apply configured margin (safety buffer distance)
 *    c. Check for margin breach (warning - inside margin but not fence)
 *    d. Check for fence breach (outside actual boundary)
 * 3. If breach detected:
 *    a. Record breach type, time, and count
 *    b. Calculate breach distance (how far outside fence)
 *    c. Store backup fence values for hysteresis
 *    d. Return bitmask of breached fences to caller
 * 4. Vehicle code checks return value and implements breach action
 * 
 * Breach Recovery:
 * ----------------
 * When breach detected, vehicle implements action specified by _action parameter:
 * - REPORT_ONLY: Log breach but continue current flight mode
 * - RTL_AND_LAND: Return to Launch, then land
 * - ALWAYS_LAND: Immediate landing at current location
 * - SMART_RTL: Use Smart RTL path if available
 * - BRAKE: Stop vehicle movement (multicopter)
 * - GUIDED: Fly to fence return point
 * 
 * Hysteresis via Backup Boundaries:
 * ----------------------------------
 * To prevent rapid re-breaching during recovery, the system uses backup fence
 * boundaries. After initial breach:
 * - Backup values stored at breach + margin distance
 * - Subsequent checks use backup values until vehicle fully inside original fence
 * - Prevents oscillation during recovery maneuvers
 * 
 * Integration with Vehicle Control:
 * ----------------------------------
 * - Pre-arm checks: Validate vehicle position inside fences before arming
 * - Takeoff integration: Auto-enable fences after successful takeoff
 * - Landing integration: Auto-disable floor fence to allow landing
 * - Mode changes: Can prevent mode changes during breach (OPTIONS flag)
 * - Manual recovery: 10-second grace period when pilot takes manual control
 * 
 * Coordinate Frames:
 * ------------------
 * - Altitude fences: Meters relative to home/origin in NED frame (down is positive)
 * - Circle fence: Horizontal meters from EKF origin (x,y plane)
 * - Polygon fence: Lat/lon coordinates in degrees, checked against EKF position
 * 
 * Units Convention:
 * -----------------
 * - All altitudes: meters
 * - Circle radius: meters
 * - Margin: meters
 * - Breach distances: meters (negative = inside fence, positive = outside fence)
 * - Polygon vertices: degrees latitude/longitude
 * 
 * Update Rate Requirements:
 * -------------------------
 * - check() method: Should be called at ~10Hz for reliable breach detection
 * - update() method: Should be called at ~10Hz to handle polygon fence loading from storage
 * - Pre-arm checks: Called once before arming
 * 
 * Thread Safety:
 * --------------
 * - AC_PolyFence_loader loads fence points asynchronously from storage
 * - update() method must be called regularly to process loader state machine
 * - Polygon checks are safe once loader reports points are loaded
 * - Other fence types have no threading concerns (read AHRS position atomically)
 * 
 * @note This is a safety-critical system. All checks must complete within scheduler
 *       timing budgets. Excessive polygon complexity could cause timing overruns.
 * 
 * @warning Modifying fence parameters during flight can cause unexpected breaches.
 *          Parameter changes should be made on the ground when possible.
 * 
 * @warning Disabling all fences removes flight boundary protections. Only disable
 *          fences when necessary and with full awareness of flight area hazards.
 * 
 * @warning Insufficient margin distance can cause fence oscillation during recovery.
 *          Margin should be larger than typical position estimate error and vehicle
 *          control overshoot distances.
 * 
 * @see AC_PolyFence_loader for polygon fence storage and loading implementation
 * @see AP_AHRS for vehicle position and altitude estimation source
 * @see Location class for lat/lon position representation
 * 
 * Source: libraries/AC_Fence/AC_Fence.h, libraries/AC_Fence/AC_Fence.cpp
 */
class AC_Fence
{
public:
    friend class AC_PolyFence_loader;

/**
 * @enum Action
 * @brief Valid recovery actions when a fence boundary is breached
 * 
 * @details Defines the vehicle's response when a fence breach is detected. Action is
 *          configured via the FENCE_ACTION parameter and executed by vehicle-specific
 *          control code when check() method returns a non-zero breach bitmask.
 *          
 *          Vehicle-specific support: Not all vehicles support all action types.
 *          - Copter: Supports all actions
 *          - Plane: Primarily uses RTL, GUIDED, and AUTOLAND options
 *          - Rover: Supports BRAKE, GUIDED, SMART_RTL
 *          - Sub: Limited fence support due to underwater operation
 */
    enum class Action {
        /**
         * @brief Report breach to GCS but take no autonomous action
         * 
         * @details Fence breach is logged and reported via MAVLink to ground control station,
         *          but vehicle continues in current flight mode without autonomous intervention.
         *          Useful for monitoring fence crossings during testing or when pilot prefers
         *          full manual control with breach awareness.
         * 
         * @note Pilot/operator is responsible for recovery when using this action.
         * @warning Vehicle will continue current flight and may travel far outside fence.
         */
        REPORT_ONLY          = 0, // report to GCS that boundary has been breached but take no further action
        
        /**
         * @brief Return to Launch, then land
         * 
         * @details Vehicle will switch to RTL (Return to Launch) mode to fly back to home/origin,
         *          then automatically land once it reaches the return point. If RTL cannot be
         *          initiated (e.g., no position estimate), vehicle will land immediately instead.
         * 
         * @note Most common fence breach action. Provides predictable recovery behavior.
         * @warning Long RTL flights may deplete battery. Ensure sufficient battery reserve.
         */
        RTL_AND_LAND         = 1, // return to launch and, if that fails, land
        
        /**
         * @brief Immediately land at current location
         * 
         * @details Vehicle will transition to LAND mode and descend vertically at current location
         *          without attempting to return to a safe area first. Used when immediate landing
         *          is preferred over attempting to return.
         * 
         * @warning Landing outside fence may place vehicle in hazardous or inaccessible location.
         *          Only use this action when landing anywhere is acceptable or when fence is small
         *          enough that outside landing is still reasonably safe.
         * 
         * @warning For fixed-wing vehicles, immediate landing may not be possible. Consider using
         *          AUTOLAND_OR_RTL instead for planes.
         */
        ALWAYS_LAND          = 2, // always land
        
        /**
         * @brief SmartRTL if available, fallback to RTL, then land
         * 
         * @details Vehicle will attempt SmartRTL (return via recorded path) if path is available.
         *          If SmartRTL not available or fails, falls back to standard RTL. If RTL fails,
         *          lands immediately. SmartRTL avoids obstacles by retracing known-safe flight path.
         * 
         * @note Requires SmartRTL to be configured and path to be recorded during flight.
         * @note SmartRTL path may not be available immediately after takeoff.
         */
        SMART_RTL            = 3, // smartRTL, if that fails, RTL, and if that still fails, land
        
        /**
         * @brief Brake (stop movement) or land if brake fails
         * 
         * @details Vehicle will transition to BRAKE mode to stop all horizontal movement (multicopter).
         *          Vehicle will hover at breach location. If brake mode cannot be initiated or is
         *          not supported (e.g., fixed-wing), vehicle will land instead.
         * 
         * @note Primarily used for multicopters. Fixed-wing vehicles will typically land instead.
         * @note Vehicle remains at breach location until pilot intervenes or battery depletes.
         * @warning Hovering at breach location may leave vehicle outside safe area indefinitely.
         */
        BRAKE                = 4, // brake, if that fails, land
        
        /**
         * @brief SmartRTL if available, otherwise land immediately
         * 
         * @details Vehicle will attempt SmartRTL return via recorded path. If SmartRTL is not
         *          available or fails, vehicle lands immediately at current location without
         *          attempting standard RTL. Faster response than SMART_RTL option.
         * 
         * @note No RTL fallback - goes directly to LAND if SmartRTL unavailable.
         * @warning May result in landing outside safe area if SmartRTL path not available.
         */
        SMART_RTL_OR_LAND    = 5, // SmartRTL, if that fails, Land
        
        /**
         * @brief Switch to Guided mode with fence return point as target
         * 
         * @details Vehicle will switch to GUIDED mode and fly toward the configured fence return
         *          point (or rally point/home if return point not configured). Provides flexible
         *          recovery to a specific safe location rather than home.
         * 
         * @note Requires valid fence return point or rally point configuration.
         * @note Vehicle retains full autonomous control of throttle and attitude in guided mode.
         */
        GUIDED               = 6, // guided mode, with target waypoint as fence return point
        
        /**
         * @brief Guided mode but pilot retains manual throttle control
         * 
         * @details Similar to GUIDED action but pilot retains manual throttle control via RC transmitter.
         *          Vehicle autonomously controls horizontal navigation toward return point while pilot
         *          controls altitude/climb rate. Useful for maintaining pilot altitude authority during recovery.
         * 
         * @note Requires active pilot input on throttle channel during recovery.
         * @warning If pilot does not actively control throttle, vehicle altitude behavior is undefined.
         */
        GUIDED_THROTTLE_PASS = 7, // guided mode, but pilot retains manual throttle control
        
        /**
         * @brief Fixed-wing automatic landing, fallback to RTL if not available
         * 
         * @details For fixed-wing vehicles, initiates automatic landing sequence if enabled and a
         *          suitable landing location is configured. If automatic landing not available or
         *          not applicable for vehicle type, falls back to standard RTL behavior instead.
         * 
         * @note Designed specifically for fixed-wing vehicles with automatic landing capability.
         * @note Requires DO_LAND_START mission item or configured landing sequence.
         */
        AUTOLAND_OR_RTL      = 8, // fixed wing autoland,if enabled, or RTL
    };

    /**
     * @enum AutoEnable
     * @brief Automatic fence enable/disable behavior based on flight state
     * 
     * @details Controls when fences are automatically enabled or disabled during vehicle operation.
     *          Automatic fence control is useful to avoid nuisance breaches during ground operations
     *          while ensuring fences are active during flight. Configured via FENCE_AUTOENABLE parameter.
     */
    enum class AutoEnable : uint8_t
    {
        /**
         * @brief Fences never automatically enabled - manual control only
         * 
         * @details Fences remain in their manually configured state. Pilot/GCS must explicitly
         *          enable or disable fences via parameters or MAVLink commands. Useful when
         *          full manual control of fence state is desired.
         */
        ALWAYS_DISABLED = 0,
        
        /**
         * @brief Enable all configured fences on automatic takeoff
         * 
         * @details Fences are automatically enabled when vehicle completes takeoff sequence in
         *          AUTO mode. Applies to autonomous missions where takeoff is part of mission.
         *          Fences remain enabled until manually disabled or vehicle disarms.
         * 
         * @note Only triggers on AUTO mode takeoff, not manual takeoff.
         * @note Floor fence (ALT_MIN) is included in auto-enable.
         */
        ENABLE_ON_AUTO_TAKEOFF = 1, // enable on auto takeoff
        
        /**
         * @brief Enable fences on takeoff, but disable floor fence before landing
         * 
         * @details All fences enabled after takeoff. Floor fence (ALT_MIN) is automatically disabled
         *          when vehicle begins landing approach to allow normal descent to ground. Other
         *          fences (altitude max, circle, polygon) remain enabled throughout landing.
         *          Prevents floor fence from interfering with landing sequence.
         * 
         * @note Most common auto-enable setting for normal operations.
         * @note Floor disable triggered by landing mode or low altitude during descent.
         */
        ENABLE_DISABLE_FLOOR_ONLY = 2,  // enable on takeoff but disable floor on landing
        
        /**
         * @brief Enable all configured fences when vehicle is armed
         * 
         * @details Fences automatically enabled immediately when vehicle arms, before takeoff.
         *          Fences remain enabled throughout flight until vehicle disarms. Provides
         *          fence protection from the moment vehicle arms rather than waiting for takeoff.
         * 
         * @note Vehicle must already be inside fence boundaries when arming with this option,
         *       otherwise arming may be prevented by pre-arm checks.
         */
        ONLY_WHEN_ARMED = 3 // enable on arming
    };

    /**
     * @enum MavlinkFenceActions
     * @brief MAVLink fence action commands for remote fence control
     * 
     * @details Defines actions that can be commanded via MAVLink protocol from ground control
     *          station or companion computer. Used to enable/disable fences during flight.
     */
    enum class MavlinkFenceActions : uint16_t
    {
        /**
         * @brief Disable all enabled fences
         * 
         * @details Disables all currently active fences. Vehicle can then move outside previous
         *          fence boundaries without triggering breach actions.
         * 
         * @warning Removes all fence boundary protections. Use with caution.
         */
        DISABLE_FENCE = 0,
        
        /**
         * @brief Enable all configured fences
         * 
         * @details Enables all fences that are configured (have non-zero configuration).
         *          Vehicle position is checked immediately - if already outside fence,
         *          breach action may trigger.
         * 
         * @warning Enabling fences when vehicle is outside boundaries causes immediate breach.
         */
        ENABLE_FENCE = 1,
        
        /**
         * @brief Disable only the minimum altitude (floor) fence
         * 
         * @details Disables the floor fence (ALT_MIN) while leaving other fences active.
         *          Useful during landing approach when floor fence would interfere with
         *          normal descent. Other fence types (altitude max, circle, polygon) remain active.
         * 
         * @note Commonly used for manual landing when auto-disable is not configured.
         */
        DISABLE_ALT_MIN_FENCE = 2
    };

    /**
     * @brief Constructor for AC_Fence geofencing system
     * 
     * @details Initializes the fence system with default parameter values and clears
     *          all breach state. Registers fence parameters with AP_Param system for
     *          persistent storage and GCS access.
     */
    AC_Fence();

    /* Do not allow copies */
    CLASS_NO_COPY(AC_Fence);

    /**
     * @brief Initialize the geofencing system
     * 
     * @details Initializes the polygon fence loader subsystem which handles asynchronous
     *          loading of polygon fence vertices from persistent storage (SD card or flash).
     *          Must be called once during vehicle startup after storage systems are initialized.
     * 
     * @note Call this during vehicle setup phase, after AP_Param and storage are initialized.
     * @note Safe to call multiple times - subsequent calls have no effect.
     * 
     * @see AC_PolyFence_loader::init()
     */
    void init() {
        _poly_loader.init();
    }

    /**
     * @brief Get singleton instance of AC_Fence
     * 
     * @details Returns pointer to the single AC_Fence instance for this vehicle.
     *          AC_Fence uses singleton pattern to provide global access point for
     *          fence status queries and control throughout vehicle code.
     * 
     * @return Pointer to AC_Fence singleton instance, or nullptr if not yet constructed
     * 
     * @note Prefer using AP::fence() accessor function rather than calling this directly.
     * 
     * @see AP::fence()
     */
    static AC_Fence *get_singleton() { return _singleton; }

    /**
     * @brief Enable or disable specified fence types
     * 
     * @details Enables or disables one or more fence types specified by bitmask. Changes take
     *          effect immediately. Can be used to selectively enable/disable fences based on
     *          flight conditions (e.g., disable floor fence during landing).
     * 
     * @param[in] value              True to enable fences, false to disable
     * @param[in] fence_types        Bitmask of fence types to enable/disable (AC_FENCE_TYPE_* defines)
     * @param[in] update_auto_mask   If true, updates the auto-enable mask. Default is true.
     * 
     * @return Bitmask of fence types that actually changed state (0 if no changes)
     * 
     * @note If enabling fences and vehicle is already outside boundaries, breach will
     *       be detected on next check() call and breach action will trigger.
     * 
     * @note Disabling fences clears any existing breaches for those fence types.
     * 
     * @warning Enabling fences while vehicle is outside boundaries causes immediate breach
     *          and may trigger unexpected mode changes or landing. Ensure vehicle position
     *          is valid before enabling fences during flight.
     * 
     * @warning Disabling fences removes flight boundary protections. Only disable when
     *          necessary and with full awareness of flight area hazards.
     * 
     * @see enable_configured() for enabling all configured fences
     * @see enable_floor() and disable_floor() for floor fence specific control
     */
    uint8_t enable(bool value, uint8_t fence_types, bool update_auto_mask = true);

    /**
     * @brief Enable or disable all configured fences
     * 
     * @details Convenience method that enables/disables all fence types that have been configured
     *          with non-zero parameters. Equivalent to calling enable() with _configured_fences bitmask.
     * 
     * @param[in] value  True to enable all configured fences, false to disable
     * 
     * @return Bitmask of fence types that actually changed state (0 if no changes)
     * 
     * @note This is the typical method used by auto-enable logic and GCS fence commands.
     * 
     * @warning Same warnings as enable() method apply - enabling while outside boundaries
     *          causes immediate breach.
     * 
     * @see enable()
     */
    uint8_t enable_configured(bool value) { return enable(value, _configured_fences, true); }

    /**
     * @brief Get automatic fence enable/disable mode
     * 
     * @details Returns the configured automatic enable/disable behavior for fences based on
     *          vehicle flight state (arming, takeoff, landing). Configured via FENCE_AUTOENABLE parameter.
     * 
     * @return AutoEnable enum value indicating auto-enable behavior
     * 
     * @see AutoEnable enum for behavior descriptions
     */
    AutoEnable auto_enabled() const { return static_cast<AutoEnable>(_auto_enabled.get()); }

    /**
     * @brief Enable the floor (minimum altitude) fence
     * 
     * @details Enables the ALT_MIN fence type immediately without updating the persistent
     *          FENCE_ENABLE parameter. Used for temporary floor fence enable, typically
     *          by auto-enable logic after takeoff.
     * 
     * @note This does NOT update the EEPROM saved value - floor fence state is temporary
     *       and will be reset on reboot or when FENCE_ENABLE parameter is loaded.
     * 
     * @note If vehicle is already below floor altitude when enabled, breach will be
     *       detected on next check() call.
     * 
     * @see disable_floor()
     * @see auto_enable_fence_floor()
     */
    void enable_floor();

    /**
     * @brief Disable the floor (minimum altitude) fence
     * 
     * @details Disables the ALT_MIN fence type immediately without updating the persistent
     *          FENCE_ENABLE parameter. Used for temporary floor fence disable, typically
     *          by auto-disable logic during landing approach.
     * 
     * @note This does NOT update the EEPROM saved value - floor fence state is temporary
     *       and will be reset on reboot or when FENCE_ENABLE parameter is loaded.
     * 
     * @note Commonly used during landing approach to allow normal descent to ground without
     *       triggering floor fence breach.
     * 
     * @see enable_floor()
     */
    void disable_floor();

    /**
     * @brief Auto-enable fences after takeoff conditions are met
     * 
     * @details Called by vehicle code after successful takeoff when auto-enable is configured
     *          (FENCE_AUTOENABLE = ENABLE_ON_AUTO_TAKEOFF). Enables all configured fences to
     *          protect flight boundaries during autonomous missions.
     * 
     * @note Only effective when FENCE_AUTOENABLE is set to ENABLE_ON_AUTO_TAKEOFF mode.
     * @note Vehicle code is responsible for detecting takeoff completion and calling this method.
     * 
     * @see AutoEnable enum
     */
    void auto_enable_fence_after_takeoff();

    /**
     * @brief Auto-enable fences when vehicle arms
     * 
     * @details Called by vehicle arming code when FENCE_AUTOENABLE = ONLY_WHEN_ARMED. Enables
     *          all configured fences immediately when vehicle arms, providing fence protection
     *          from arming through flight to disarming.
     * 
     * @note Only effective when FENCE_AUTOENABLE is set to ONLY_WHEN_ARMED mode.
     * @note Vehicle must be inside fence boundaries when arming or this may trigger immediate breach.
     * 
     * @see AutoEnable enum
     */
    void auto_enable_fence_on_arming();

    /**
     * @brief Auto-disable fences when vehicle disarms
     * 
     * @details Called by vehicle disarming code to disable fences that should not remain active
     *          when vehicle is disarmed. Which fences are disabled depends on auto-enable configuration.
     * 
     * @note Typically disables fences that were auto-enabled during arming or takeoff.
     * 
     * @see get_auto_disable_fences() for which fences will be disabled
     */
    void auto_disable_fence_on_disarming();

    /**
     * @brief Get bitmask of fences that will be auto-disabled on disarming
     * 
     * @details Returns bitmask indicating which fence types will be automatically disabled
     *          when vehicle disarms, based on current auto-enable configuration.
     * 
     * @return Bitmask of fence types (AC_FENCE_TYPE_*) that auto-disable on disarming
     * 
     * @see auto_disable_fence_on_disarming()
     */
    uint8_t get_auto_disable_fences(void) const;

    /**
     * @brief Auto-enable floor fence after reaching desired altitude
     * 
     * @details Attempts to enable floor fence (ALT_MIN) after vehicle has climbed above the
     *          configured floor altitude plus margin. Called repeatedly by vehicle code during
     *          climb-out after takeoff. Prevents floor fence from being enabled while vehicle
     *          is still near ground.
     * 
     * @return true if floor fence was enabled, false if conditions not met or already enabled
     * 
     * @note Floor fence will only enable if:
     *       - Auto-enable is configured appropriately
     *       - Vehicle has climbed above floor + margin altitude
     *       - Floor fence is configured (ALT_MIN > 0)
     * 
     * @note This is part of ENABLE_DISABLE_FLOOR_ONLY auto-enable behavior.
     */
    bool auto_enable_fence_floor();

    /**
     * @brief Check if any fences are currently enabled
     * 
     * @details Returns true if at least one fence type is currently active. Does not indicate
     *          which specific fences are enabled - use get_enabled_fences() for detailed bitmask.
     * 
     * @return true if any fence is enabled, false if all fences disabled
     * 
     * @see get_enabled_fences() for bitmask of specific enabled fences
     */
    bool enabled() const { return _enabled_fences; }

    /**
     * @brief Check if any fence types are configured (non-zero parameters)
     * 
     * @details Returns bitmask of fence types that have been configured with non-zero parameters,
     *          regardless of whether they are currently enabled. A fence is "present" if it has
     *          configuration that would make it active when enabled.
     * 
     * @return Bitmask of configured fence types (AC_FENCE_TYPE_*)
     * 
     * @note A fence can be present but not enabled. Use enabled() to check active state.
     * @note Used by pre-arm checks and GCS status reporting.
     * 
     * @see enabled()
     * @see get_enabled_fences()
     */
    uint8_t present() const;

    /**
     * @brief Get bitmask of currently enabled fence types
     * 
     * @details Returns bitmask indicating which specific fence types are currently active and
     *          monitoring vehicle position. Each bit corresponds to a fence type from AC_FENCE_TYPE_* defines.
     * 
     * @return Bitmask of enabled fence types (AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | etc.)
     * 
     * @note Return value of 0 means no fences are active.
     * 
     * @see enabled() for simple boolean check
     * @see present() for configured fence types
     */
    uint8_t get_enabled_fences() const;

    /**
     * @brief Update fence system state and handle polygon fence loading
     * 
     * @details Must be called at approximately 10Hz from vehicle scheduler to handle:
     *          - Polygon fence point loading from storage (SD card or flash)
     *          - Parameter change detection and fence reconfiguration
     *          - Auto-enable state machine updates
     * 
     *          The polygon fence loader operates as a state machine that asynchronously reads
     *          fence vertices from persistent storage. This method advances the loader state
     *          machine and handles load completion or failures.
     * 
     * @note MUST be called regularly at ~10Hz for polygon fences to load correctly.
     * @note Call frequency affects polygon load time - lower rates slow loading.
     * @note Safe to call even if no polygon fences are configured.
     * 
     * @see AC_PolyFence_loader for polygon loading state machine details
     */
    void update();

    /**
     * @brief Pre-arm check to validate vehicle position relative to fence boundaries
     * 
     * @details Verifies that vehicle is in a valid position for arming relative to all enabled
     *          "arming fences" (altitude max, circle, polygon). This prevents arming when vehicle
     *          is already outside fence boundaries, which would cause immediate breach on takeoff.
     * 
     *          Checks performed:
     *          - Altitude: Vehicle must be below altitude max fence (if enabled)
     *          - Circle: Vehicle must be within circular fence radius (if enabled)
     *          - Polygon: Vehicle must be inside inclusion polygons and outside exclusion polygons (if enabled)
     * 
     *          Note: Minimum altitude (floor) fence is NOT checked because vehicle is typically on
     *          ground at arming time, which is below any reasonable floor altitude.
     * 
     * @param[out] failure_msg      Buffer to receive human-readable failure message if check fails
     * @param[in]  failure_msg_len  Size of failure_msg buffer in bytes
     * 
     * @return true if all pre-arm checks passed and arming is safe, false if checks failed
     * 
     * @note failure_msg will contain specific failure reason (e.g., "vehicle outside fence")
     *       if return value is false. Message suitable for display to operator.
     * 
     * @note Pre-arm checks can be disabled via vehicle-specific parameters if needed for
     *       special situations, but this removes an important safety verification.
     * 
     * @warning Disabling pre-arm checks allows arming outside fence boundaries, which will
     *          cause immediate fence breach and possible uncommanded mode changes on takeoff.
     * 
     * @see AC_FENCE_ARMING_FENCES for definition of which fences are checked
     */
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

    ///
    /// @name Breach Detection and Recovery Methods
    /// Methods to check vehicle position against boundaries and support recovery
    /// @{
    ///

    /**
     * @brief Check vehicle position against enabled fence boundaries
     * 
     * @details This is the primary fence checking method that should be called at approximately
     *          10Hz from vehicle scheduler. Checks current vehicle position against all enabled
     *          fence types and detects both margin breaches (warning) and fence breaches (boundary violation).
     * 
     * Breach Detection Algorithm Flow:
     * ---------------------------------
     * For each enabled fence type:
     * 
     * 1. **Altitude Max Check** (if AC_FENCE_TYPE_ALT_MAX enabled):
     *    - Get current altitude from AHRS (meters above home)
     *    - Compare: altitude > (_alt_max - _margin) triggers margin breach
     *    - Compare: altitude > _alt_max triggers fence breach
     *    - If breach detected: Store _alt_max_backup = altitude for hysteresis
     * 
     * 2. **Altitude Min (Floor) Check** (if AC_FENCE_TYPE_ALT_MIN enabled):
     *    - Get current altitude from AHRS (meters above home)
     *    - Compare: altitude < (_alt_min + _margin) triggers margin breach
     *    - Compare: altitude < _alt_min triggers fence breach
     *    - If breach detected: Store _alt_min_backup = altitude for hysteresis
     * 
     * 3. **Circle Fence Check** (if AC_FENCE_TYPE_CIRCLE enabled):
     *    - Calculate horizontal distance from home/origin (meters)
     *    - Compare: distance > (_circle_radius - _margin) triggers margin breach
     *    - Compare: distance > _circle_radius triggers fence breach
     *    - If breach detected: Store _circle_radius_backup = distance for hysteresis
     * 
     * 4. **Polygon Fence Check** (if AC_FENCE_TYPE_POLYGON enabled):
     *    - Delegate to AC_PolyFence_loader point-in-polygon checks
     *    - Check against all loaded inclusion and exclusion polygons
     *    - If breach detected: Calculate distance to nearest polygon boundary
     * 
     * Breach Hysteresis via Backup Boundaries:
     * -----------------------------------------
     * To prevent rapid re-breaching during recovery maneuvers:
     * - On initial breach: Backup values stored at current position + margin
     * - Subsequent checks: Use backup values instead of original fence boundaries
     * - Backup values only cleared when vehicle returns fully inside original fence
     * - Prevents oscillation during recovery (e.g., vehicle bouncing off fence boundary)
     * 
     * Breach Recording:
     * -----------------
     * When breach detected:
     * - Updates _breached_fences bitmask with breached fence types
     * - Records _breach_time (milliseconds since boot)
     * - Increments _breach_count
     * - Calculates breach distances (_alt_max_breach_distance, etc.)
     * - Sends breach notification to GCS via MAVLink
     * 
     * Manual Recovery Window:
     * -----------------------
     * If pilot has taken manual control within last 10 seconds (manual_recovery_start()),
     * fence checks are suppressed to allow pilot to recover vehicle position without
     * repeated breach actions triggering.
     * 
     * @param[in] disable_auto_fence  If true, disables automatic fence parameter updates during check.
     *                                Default is false. Used during landing to prevent fence
     *                                state changes from interfering with landing sequence.
     * 
     * @return Bitmask of fence types that are currently breached (AC_FENCE_TYPE_*), or 0 if no breaches
     * 
     * @note Must be called at ~10Hz for reliable breach detection. Lower rates may miss rapid breaches.
     * @note Return value is bitmask - multiple fence types can be breached simultaneously.
     * @note Breach distance calculations are in meters (negative = inside fence, positive = outside).
     * 
     * @warning Disabling auto_fence parameter during landing can prevent proper fence state tracking.
     *          Only use when necessary to prevent fence interference with landing.
     * 
     * @warning This method depends on valid AHRS position estimate. If position is unavailable,
     *          checks may not function correctly (typically treated as no breach to avoid false triggers).
     * 
     * Coordinate Systems:
     * -------------------
     * - Altitude: Meters relative to home/origin in NED frame (down is positive)
     * - Circle: Horizontal meters from EKF origin (x,y plane)
     * - Polygon: Lat/lon in degrees, checked against current EKF position
     * 
     * Units:
     * ------
     * - All distances: meters
     * - Breach distances: meters (negative = inside, positive = outside)
     * - Margin: meters (safety buffer distance)
     * 
     * @see get_breaches() to retrieve breach bitmask
     * @see get_breach_distance() to get distance outside fence
     * @see manual_recovery_start() for pilot recovery mode
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp - check() implementation and check_fence_*() helpers
     */
    uint8_t check(bool disable_auto_fence = false);

    /**
     * @brief Check if a destination location is within fence boundaries
     * 
     * @details Validates whether a given location (typically a waypoint or guided target) is within
     *          all enabled fence boundaries. Used to reject mission waypoints or guided targets that
     *          would command vehicle to fly outside safe area. Checks destination against all enabled
     *          fence types (altitude max/min, circle, polygon).
     * 
     *          This is a lookahead check - validates destination before vehicle attempts to fly there.
     * 
     * @param[in] loc  Location to check (lat/lon/alt) as potential destination
     * 
     * @return true if location is within all enabled fence boundaries, false if any fence would be breached
     * 
     * @note Used by mission validation to reject waypoints outside fence boundaries.
     * @note Used by guided mode to validate commanded target positions.
     * @note If no fences are enabled, always returns true (no restrictions).
     * 
     * @warning Waypoints that fail this check should be rejected or vehicle will breach fence
     *          when attempting to reach them.
     * 
     * @see AP_Mission for waypoint validation usage
     */
    bool check_destination_within_fence(const class Location& loc);

    /// @}


    /**
     * @brief Get bitmask of currently breached fence types
     * 
     * @details Returns bitmask indicating which fence boundaries have been breached (vehicle is
     *          outside boundary). Bits are set when check() detects breach and remain set until
     *          vehicle returns inside fence or fence is disabled.
     * 
     * @return Bitmask of breached fence types (AC_FENCE_TYPE_*), or 0 if no active breaches
     * 
     * @note Multiple fence types can be breached simultaneously (e.g., alt_max + circle).
     * @note Used by vehicle control code to determine when to trigger breach actions.
     * 
     * @see check() for breach detection
     * @see get_breach_time() for when breach occurred
     */
    uint8_t get_breaches() const { return _breached_fences; }

    /**
     * @brief Get time of most recent fence breach
     * 
     * @details Returns system time in milliseconds when last fence breach was detected.
     *          Used to implement breach action delays and determine how long vehicle has
     *          been outside boundaries.
     * 
     * @return Time in milliseconds since boot when last breach occurred, or 0 if never breached
     * 
     * @note Time is in milliseconds from AP_HAL::millis() - wraps every 49.7 days.
     * 
     * @see get_breaches() for which fences are breached
     */
    uint32_t get_breach_time() const { return _breach_time; }

    /**
     * @brief Get total number of fence breaches since boot
     * 
     * @details Returns cumulative count of fence breach events. Incremented each time check()
     *          detects a new breach (transition from inside to outside boundary). Used for
     *          logging and statistics.
     * 
     * @return Total number of breaches since vehicle boot, or since counter was reset
     * 
     * @note Counter does not decrement when vehicle returns inside fence.
     * @note Counter may wrap at 65535 on very long flights with many breaches.
     */
    uint16_t get_breach_count() const { return _breach_count; }

    /**
     * @brief Get bitmask of fence types with margin breaches (warnings)
     * 
     * @details Returns bitmask indicating which fence types have margin breaches - vehicle is
     *          inside the margin safety buffer but has not yet crossed actual boundary. Margin
     *          breaches serve as advance warning that vehicle is approaching fence limit.
     * 
     * @return Bitmask of fence types with margin breaches (AC_FENCE_TYPE_*), or 0 if none
     * 
     * @note Margin breach precedes fence breach as vehicle approaches boundary.
     * @note Margin distance configured via FENCE_MARGIN parameter (meters).
     * @note Can be used to trigger warnings or pre-emptive corrective action.
     * 
     * @see get_margin() for configured margin distance
     * @see get_margin_breach_time() for when margin breach occurred
     */
    uint8_t get_margin_breaches() const { return _breached_fence_margins; }

    /**
     * @brief Get time of most recent fence margin breach
     * 
     * @details Returns system time in milliseconds when last margin breach was detected.
     *          Margin breach is advance warning that vehicle is approaching fence boundary.
     * 
     * @return Time in milliseconds since boot when last margin breach occurred, or 0 if never breached
     * 
     * @note Time is in milliseconds from AP_HAL::millis() - wraps every 49.7 days.
     * 
     * @see get_margin_breaches() for which fences have margin breaches
     */
    uint32_t get_margin_breach_time() const { return _margin_breach_time; }

    /**
     * @brief Get maximum distance outside specified fence boundaries
     * 
     * @details Calculates the greatest distance that vehicle is currently outside the specified
     *          fence types. Useful for determining severity of breach and deciding between
     *          recovery attempts vs immediate landing. Distance is in meters outside boundary.
     * 
     * @param[in] fence_type  Bitmask of fence types to check (AC_FENCE_TYPE_*)
     * 
     * @return Maximum distance in meters outside any of the specified fences
     *         - Positive value: distance outside fence (breach)
     *         - Negative value: distance inside fence (safe)
     *         - Returns distance to most severely breached fence if multiple types specified
     * 
     * @note If vehicle is inside all specified fences, returns negative distance to nearest boundary.
     * @note Useful for comparing against AC_FENCE_GIVE_UP_DISTANCE to decide when to abandon recovery.
     * @note Calculation includes all specified fence types - returns worst-case (maximum) breach distance.
     * 
     * Units: meters
     * 
     * @warning Distance calculation depends on valid position estimate. Returns 0 if position invalid.
     * 
     * @see AC_FENCE_GIVE_UP_DISTANCE for recommended maximum recovery distance
     */
    float get_breach_distance(uint8_t fence_type) const;

    /**
     * @brief Get configured breach action
     * 
     * @details Returns the action that should be taken when fence breach is detected.
     *          Configured via FENCE_ACTION parameter. Vehicle control code is responsible
     *          for implementing the returned action when breach occurs.
     * 
     * @return Action enum value indicating configured breach response
     * 
     * @see Action enum for detailed action descriptions
     */
    Action get_action() const { return _action; }

    /**
     * @brief Get maximum safe altitude (altitude ceiling minus margin)
     * 
     * @details Returns the altitude that vehicle should remain below to avoid margin breach
     *          of altitude max fence. This is the fence boundary minus the safety margin.
     *          Useful for guidance and control to maintain safe distance from altitude ceiling.
     * 
     * @return Maximum safe altitude in meters above home/origin
     * 
     * @note This is _alt_max - _margin (fence boundary minus safety buffer).
     * @note Use this value for trajectory planning to avoid approaching fence boundary.
     * 
     * Units: meters above home
     * 
     * @see get_margin() for margin distance
     */
    float get_safe_alt_max() const { return _alt_max - _margin; }

    /**
     * @brief Get minimum safe altitude (altitude floor plus margin)
     * 
     * @details Returns the altitude that vehicle should remain above to avoid margin breach
     *          of altitude min fence (floor). This is the fence boundary plus the safety margin.
     *          Useful for guidance and control to maintain safe distance from altitude floor.
     * 
     * @return Minimum safe altitude in meters above home/origin
     * 
     * @note This is _alt_min + _margin (fence boundary plus safety buffer).
     * @note Use this value for trajectory planning to avoid approaching floor boundary.
     * 
     * Units: meters above home
     * 
     * @see get_margin() for margin distance
     */
    float get_safe_alt_min() const { return _alt_min + _margin; }

    /**
     * @brief Get circular fence radius
     * 
     * @details Returns the configured horizontal radius for circular fence. Vehicle must remain
     *          within this distance from home/origin point. Configured via FENCE_RADIUS parameter.
     * 
     * @return Fence radius in meters from home/origin
     * 
     * Units: meters
     * 
     * @note This is the fence boundary value, not including margin. Subtract margin for safe distance.
     */
    float get_radius() const { return _circle_radius.get(); }

    /**
     * @brief Get configured fence margin distance
     * 
     * @details Returns the safety buffer distance used for margin breach detection and safe
     *          distance calculations. Margin provides early warning before actual fence breach
     *          and safety buffer for trajectory planning. Configured via FENCE_MARGIN parameter.
     * 
     * @return Margin distance in meters
     * 
     * Units: meters
     * 
     * @warning Insufficient margin can cause fence oscillation during recovery. Margin should
     *          be larger than typical position error plus vehicle control overshoot distance.
     *          Recommended minimum: 2-5 meters for small vehicles, 10+ meters for larger vehicles.
     * 
     * @note Margin is applied to all fence types (altitude, circle, polygon).
     */
    float get_margin() const { return _margin.get(); }

    /**
     * @brief Get return to rally point configuration
     * 
     * @details Returns setting for whether fence breach should return to fence return point
     *          or rally point. Configured via FENCE_RET_RALLY parameter.
     * 
     * @return Return point selection value
     * 
     * @note Interpretation depends on vehicle-specific implementation of breach actions.
     */
    uint8_t get_return_rally() const { return _ret_rally; }
    
    /**
     * @brief Get return altitude for fence breach recovery
     * 
     * @details Returns the altitude that vehicle should climb/descend to during return-to-launch
     *          after fence breach. Configured via FENCE_RET_ALT parameter.
     * 
     * @return Return altitude in meters
     * 
     * Units: meters above home/origin
     * 
     * @note Used by RTL and guided breach actions to determine return flight altitude.
     */
    float get_return_altitude() const { return _ret_altitude; }

    /**
     * @brief Get human-readable names of fence types from bitmask
     * 
     * @details Converts a fence type bitmask into comma-separated human-readable names
     *          suitable for display to operators or logging. Used for user-friendly
     *          fence status reporting and breach messages.
     * 
     * @param[in]  fences  Bitmask of fence types (AC_FENCE_TYPE_*)
     * @param[out] msg     ExpandingString to receive fence type names
     * 
     * Example output: "Alt_Max, Circle, Polygon"
     * 
     * @note This is a static utility method that can be called without AC_Fence instance.
     * @note Used internally by print_fence_message() and externally for logging.
     * 
     * @see print_fence_message() for sending fence status to GCS
     */
    static void get_fence_names(uint8_t fences, ExpandingString& msg);

    /**
     * @brief Print fence status message to ground control station
     * 
     * @details Sends a MAVLink message to GCS with specified text message and fence type names.
     *          Used to notify operators of fence state changes, breaches, and warnings.
     * 
     * @param[in] msg     Text message to send (e.g., "Fence breach:")
     * @param[in] fences  Bitmask of fence types to include in message
     * 
     * Example: "Fence breach: Alt_Max, Circle"
     * 
     * @note Message sent via MAVLink STATUSTEXT protocol to all enabled GCS links.
     * @note Called automatically by check() when breaches occur if notifications enabled.
     * 
     * @see get_fence_names() for fence name formatting
     */
    void print_fence_message(const char* msg, uint8_t fences) const;

    /**
     * @brief Start 10-second manual recovery window for pilot control
     * 
     * @details Signals that pilot has taken manual control of vehicle and fence breach checks
     *          should be suppressed for 10 seconds. This allows pilot to recover vehicle position
     *          without repeated fence breach actions triggering and interfering with manual recovery.
     * 
     *          During the 10-second window:
     *          - Fence checks continue to run but do not trigger new breach actions
     *          - Existing breach flags remain set for status reporting
     *          - Vehicle can be maneuvered back inside fence boundaries without interference
     * 
     *          The window automatically expires after 10 seconds, or can be ended early by
     *          calling this method again with vehicle inside boundaries.
     * 
     * @note Should be called whenever pilot changes flight mode to manual control (e.g., switching
     *       from AUTO to STABILIZE or LOITER).
     * 
     * @note Has no effect if no breaches are currently active - only applies during active breach.
     * 
     * @note 10-second duration is hardcoded and considered appropriate for typical manual recovery.
     * 
     * @warning Pilot must actively recover vehicle position during this window. If vehicle remains
     *          outside fence after 10 seconds, breach actions will resume automatically.
     * 
     * @see check() for breach detection during manual recovery window
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp - manual_recovery_start() implementation
     */
    void manual_recovery_start();

    /**
     * @brief Check if any fence types are present for MAVLink SYS_STATUS reporting
     * 
     * @details Returns whether fence system has any configured fence types. Used by MAVLink
     *          SYS_STATUS message to report fence availability to ground control stations.
     * 
     * @return true if any fence types are configured (present), false if no fences configured
     * 
     * @note Part of MAVLink SYS_STATUS sensor health reporting protocol.
     * 
     * @see present() for detailed fence configuration bitmask
     */
    bool sys_status_present() const;

    /**
     * @brief Check if any fences are enabled for MAVLink SYS_STATUS reporting
     * 
     * @details Returns whether any fence types are currently active/enabled. Used by MAVLink
     *          SYS_STATUS message to report fence enable state to ground control stations.
     * 
     * @return true if any fences are enabled, false if all fences disabled
     * 
     * @note Part of MAVLink SYS_STATUS sensor health reporting protocol.
     * 
     * @see enabled() for basic enabled check
     * @see get_enabled_fences() for detailed enabled fence bitmask
     */
    bool sys_status_enabled() const;

    /**
     * @brief Check if fence system has failures for MAVLink SYS_STATUS reporting
     * 
     * @details Returns whether fence system has detected any failures that prevent proper
     *          operation. Currently reports failure if polygon fence failed to load from storage.
     *          Used by MAVLink SYS_STATUS message to report fence health to ground control stations.
     * 
     * @return true if fence system has failures, false if operating normally
     * 
     * @note Part of MAVLink SYS_STATUS sensor health reporting protocol.
     * @note Polygon load failures are most common failure mode (SD card issues, corrupt data).
     */
    bool sys_status_failed() const;

    /**
     * @brief Get reference to polygon fence loader
     * 
     * @details Provides access to AC_PolyFence_loader for polygon fence operations such as
     *          adding/removing vertices, querying polygon status, or forcing reload from storage.
     * 
     * @return Reference to polygon fence loader instance
     * 
     * @note Used by vehicle code and GCS protocols for polygon fence management.
     * 
     * @see AC_PolyFence_loader for polygon fence storage and loading interface
     */
    AC_PolyFence_loader &polyfence();

    /**
     * @brief Get const reference to polygon fence loader
     * 
     * @details Provides const access to AC_PolyFence_loader for polygon fence queries without
     *          allowing modification. Used for read-only polygon fence status checks.
     * 
     * @return Const reference to polygon fence loader instance
     * 
     * @see AC_PolyFence_loader for polygon fence storage and loading interface
     */
    const AC_PolyFence_loader &polyfence() const;

    /**
     * @enum OPTIONS
     * @brief Fence behavior option flags configured via FENCE_OPTIONS parameter
     * 
     * @details Bitmask flags that modify fence behavior. Multiple options can be combined
     *          by bitwise OR. Configured via FENCE_OPTIONS parameter.
     */
    enum class OPTIONS {
        /**
         * @brief Prevent mode changes during fence breach
         * 
         * @details When enabled, pilot or GCS mode change commands are blocked during active
         *          fence breach. Vehicle must complete breach recovery action or pilot must
         *          use manual_recovery_start() before mode changes are allowed. Prevents pilot
         *          from inadvertently canceling breach recovery by changing modes.
         * 
         * @note Mode changes are still allowed during manual recovery window (10 seconds).
         * @warning This restricts pilot control during breach. Ensure pilots are trained on
         *          manual recovery procedure (typically switching to manual mode triggers recovery window).
         */
        DISABLE_MODE_CHANGE = 1U << 0,
        
        /**
         * @brief Use union of inclusion polygons instead of intersection
         * 
         * @details When multiple inclusion polygon fences are defined, controls whether vehicle
         *          must be inside ALL polygons (intersection, default) or inside ANY polygon (union).
         *          
         *          - Not set (default): Vehicle must be in ALL inclusion zones (intersection)
         *          - Set: Vehicle must be in AT LEAST ONE inclusion zone (union)
         * 
         * @note Only affects behavior when multiple inclusion polygons are configured.
         * @note Exclusion zones are always treated as union (vehicle must be outside ALL exclusion zones).
         */
        INCLUSION_UNION = 1U << 1,
        
        /**
         * @brief Send notifications when margin is breached
         * 
         * @details When enabled, sends MAVLink notification messages when vehicle breaches fence
         *          margin (safety buffer) in addition to notifications for actual fence breaches.
         *          Provides advance warning to operators that vehicle is approaching fence boundary.
         * 
         * @note Margin breach is warning only - no breach action triggered until actual fence crossed.
         * @note Notification rate limited by FENCE_NOTIFY_FREQ parameter to avoid message spam.
         * 
         * @see get_margin_breaches() for margin breach status
         */
        NOTIFY_MARGIN_BREACH = 1U << 2,
    };
    
    /**
     * @brief Check if a fence option flag is enabled (static version)
     * 
     * @details Static utility method to check if a specific option bit is set in options parameter.
     *          Useful for checking options without AC_Fence instance.
     * 
     * @param[in] opt      Option flag to check
     * @param[in] options  Options parameter value
     * 
     * @return true if specified option is enabled, false otherwise
     */
    static bool option_enabled(OPTIONS opt, const AP_Int16 &options) {
        return (options.get() & int16_t(opt)) != 0;
    }
    
    /**
     * @brief Check if a fence option flag is enabled (instance method)
     * 
     * @details Checks if a specific option bit is set in this instance's _options parameter.
     * 
     * @param[in] opt  Option flag to check
     * 
     * @return true if specified option is enabled, false otherwise
     */
    bool option_enabled(OPTIONS opt) const {
        return option_enabled(opt, _options);
    }

    /**
     * @brief ArduPilot parameter table for fence configuration
     * 
     * @details Defines all persistent parameters for fence configuration. Registered with
     *          AP_Param system for persistent storage and GCS access. Parameters include
     *          fence enable, boundaries, margins, actions, and behavior options.
     * 
     * Key parameters:
     * - FENCE_ENABLE: Enable/disable fence types (bitmask)
     * - FENCE_TYPE: Configure which fence types are available (bitmask)
     * - FENCE_ACTION: Breach action (Action enum)
     * - FENCE_ALT_MAX: Maximum altitude in meters
     * - FENCE_ALT_MIN: Minimum altitude (floor) in meters
     * - FENCE_RADIUS: Circle fence radius in meters
     * - FENCE_MARGIN: Safety margin distance in meters
     * - FENCE_TOTAL: Number of polygon vertices
     * - FENCE_OPTIONS: Behavior option flags (OPTIONS enum)
     * - FENCE_AUTOENABLE: Automatic enable/disable mode
     * 
     * @note Parameters are persistent across reboots and accessible via GCS for configuration.
     * 
     * @see AP_Param::GroupInfo for parameter registration details
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp - var_info[] definition
     */
    static const struct AP_Param::GroupInfo var_info[];

#if AP_SDCARD_STORAGE_ENABLED
    /**
     * @brief Check if polygon fence failed to load from SD card storage
     * 
     * @details Returns whether polygon fence loader encountered failure when attempting to
     *          load fence vertices from SD card. Used for diagnostics and pre-arm checks.
     * 
     * @return true if SD card storage failure occurred, false if load succeeded or not attempted
     * 
     * @note Only available when AP_SDCARD_STORAGE_ENABLED is defined (SD card support compiled in).
     * @note Failures typically indicate corrupt fence file, SD card hardware issues, or filesystem errors.
     * 
     * @see AC_PolyFence_loader::failed_sdcard_storage()
     */
    bool failed_sdcard_storage(void) const {
        return _poly_loader.failed_sdcard_storage();
    }
#endif

private:
    /**
     * @name Private Members
     * @{
     * 
     * Internal implementation details for fence checking, breach tracking, and state management.
     * These methods and variables are not part of the public API.
     */
    
    /** @brief Singleton instance pointer */
    static AC_Fence *_singleton;

    /**
     * @brief Check if maximum altitude fence has been newly breached
     * 
     * @details Internal method called by check() to test current altitude against maximum
     *          altitude fence boundary. Compares AHRS altitude against _alt_max parameter
     *          plus margin. Updates breach state and backup boundaries if breach detected.
     * 
     * @return true if altitude max fence was newly breached on this check, false otherwise
     * 
     * @note Implements hysteresis via _alt_max_backup to prevent re-breach during recovery.
     * @note Called at ~10Hz by check() method when ALT_MAX fence is enabled.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool check_fence_alt_max();

    /**
     * @brief Check if minimum altitude fence has been newly breached
     * 
     * @details Internal method called by check() to test current altitude against minimum
     *          altitude (floor) fence boundary. Compares AHRS altitude against _alt_min parameter
     *          minus margin. Updates breach state and backup boundaries if breach detected.
     * 
     * @return true if altitude min fence was newly breached on this check, false otherwise
     * 
     * @note Implements hysteresis via _alt_min_backup to prevent re-breach during recovery.
     * @note Called at ~10Hz by check() method when ALT_MIN fence is enabled.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool check_fence_alt_min();

    /**
     * @brief Check if polygon fence has been newly breached
     * 
     * @details Internal method called by check() to test current position against polygon
     *          fence boundaries. Delegates to AC_PolyFence_loader for point-in-polygon checks
     *          against all loaded inclusion and exclusion polygons. Updates breach state if breach detected.
     * 
     * @return true if polygon fence was newly breached on this check, false otherwise
     * 
     * @note Requires valid EKF position estimate. Returns false (no breach) if position unavailable.
     * @note Polygon vertices must be loaded by _poly_loader before checks can function.
     * @note Called at ~10Hz by check() method when POLYGON fence is enabled.
     * 
     * @see AC_PolyFence_loader for polygon boundary checking implementation
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool check_fence_polygon();

    /**
     * @brief Check if circular fence has been newly breached
     * 
     * @details Internal method called by check() to test current position against circular
     *          fence boundary. Calculates horizontal distance from home/origin and compares
     *          against _circle_radius parameter plus margin. Updates breach state and backup
     *          boundaries if breach detected.
     * 
     * @return true if circle fence was newly breached on this check, false otherwise
     * 
     * @note Implements hysteresis via _circle_radius_backup to prevent re-breach during recovery.
     * @note Called at ~10Hz by check() method when CIRCLE fence is enabled.
     * @note Requires valid EKF position estimate. Returns false (no breach) if position unavailable.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool check_fence_circle();

    /**
     * @brief Record fence breach in tracking variables
     * 
     * @details Internal method to update breach state when fence violation is detected.
     *          Updates _breached_fences bitmask, _breach_time, and _breach_count.
     *          Called by individual fence check methods when breach occurs.
     * 
     * @param[in] fence_type  Bitmask of fence type that was breached (AC_FENCE_TYPE_*)
     * 
     * @note Sets bit in _breached_fences and records time/count for breach tracking.
     * @note Multiple fence types can be breached simultaneously.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    void record_breach(uint8_t fence_type);

    /**
     * @brief Clear fence breach from tracking variables
     * 
     * @details Internal method to clear breach state when vehicle returns inside fence boundary.
     *          Clears bit in _breached_fences bitmask. Called by individual fence check methods
     *          when vehicle returns to safe area.
     * 
     * @param[in] fence_type  Bitmask of fence type to clear (AC_FENCE_TYPE_*)
     * 
     * @note Clears bit in _breached_fences but preserves breach time/count for history.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    void clear_breach(uint8_t fence_type);

    /**
     * @brief Record margin breach in tracking variables
     * 
     * @details Internal method to update margin breach state when vehicle enters margin
     *          safety buffer zone (warning area before actual fence breach). Updates
     *          _breached_fence_margins bitmask. Called by individual fence check methods.
     * 
     * @param[in] fence_type  Bitmask of fence type with margin breach (AC_FENCE_TYPE_*)
     * 
     * @note Margin breach is advance warning that vehicle is approaching fence boundary.
     * @note Sets bit in _breached_fence_margins and records margin breach time.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    void record_margin_breach(uint8_t fence_type);

    /**
     * @brief Clear margin breach from tracking variables
     * 
     * @details Internal method to clear margin breach state when vehicle moves away from
     *          fence boundary and exits margin buffer zone. Clears bit in _breached_fence_margins
     *          bitmask. Called by individual fence check methods.
     * 
     * @param[in] fence_type  Bitmask of fence type to clear margin breach (AC_FENCE_TYPE_*)
     * 
     * @note Clears bit in _breached_fence_margins but preserves margin breach time for history.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    void clear_margin_breach(uint8_t fence_type);

    /**
     * @brief Pre-arm check for polygon fence
     * 
     * @details Internal method to validate vehicle position is inside polygon inclusion zones
     *          and outside exclusion zones before arming. Called by pre_arm_check() when polygon
     *          fence is enabled.
     * 
     * @param[out] failure_msg      Buffer to receive failure message if check fails
     * @param[in]  failure_msg_len  Size of failure_msg buffer in bytes
     * 
     * @return true if vehicle position satisfies polygon fence requirements, false otherwise
     * 
     * @note Requires valid EKF position estimate. Fails check if position unavailable.
     * @note Requires polygon vertices to be loaded. Fails check if load incomplete or failed.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool pre_arm_check_polygon(char *failure_msg, const uint8_t failure_msg_len) const;
    
    /**
     * @brief Pre-arm check for circular fence
     * 
     * @details Internal method to validate vehicle position is inside circular fence radius
     *          before arming. Called by pre_arm_check() when circle fence is enabled.
     * 
     * @param[out] failure_msg      Buffer to receive failure message if check fails
     * @param[in]  failure_msg_len  Size of failure_msg buffer in bytes
     * 
     * @return true if vehicle is within circle fence radius, false otherwise
     * 
     * @note Requires valid EKF position estimate. Fails check if position unavailable.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool pre_arm_check_circle(char *failure_msg, const uint8_t failure_msg_len) const;
    
    /**
     * @brief Pre-arm check for altitude fences
     * 
     * @details Internal method to validate vehicle altitude is below maximum altitude fence
     *          before arming. Called by pre_arm_check() when altitude max fence is enabled.
     *          Does not check minimum altitude (floor) since vehicle is typically on ground.
     * 
     * @param[out] failure_msg      Buffer to receive failure message if check fails
     * @param[in]  failure_msg_len  Size of failure_msg buffer in bytes
     * 
     * @return true if vehicle altitude is below altitude max fence, false otherwise
     * 
     * @note Only checks ALT_MAX fence. ALT_MIN (floor) not checked during pre-arm.
     * 
     * Source: libraries/AC_Fence/AC_Fence.cpp
     */
    bool pre_arm_check_alt(char *failure_msg, const uint8_t failure_msg_len) const;
    
    /**
     * @brief Check if fence floor (minimum altitude) is currently enabled
     * 
     * @details Internal helper to test if ALT_MIN fence type is in enabled fences bitmask.
     * 
     * @return true if floor fence is enabled, false otherwise
     */
    bool floor_enabled() const { return _enabled_fences & AC_FENCE_TYPE_ALT_MIN; }

    /**
     * @name Parameter Variables
     * @{
     * 
     * Persistent configuration parameters stored via AP_Param system. These correspond to
     * FENCE_* parameters accessible via GCS. Modified values persist across reboots.
     */
    
    /** 
     * @brief Currently enabled fence types (runtime state bitmask)
     * 
     * @details Runtime bitmask of which fence types are currently active. Can differ from
     *          _configured_fences due to auto-enable/disable or pilot commands. Bits correspond
     *          to AC_FENCE_TYPE_* defines.
     * 
     * @note This is runtime state, not a persistent parameter. Updated by enable()/disable() methods.
     */
    uint8_t         _enabled_fences;
    
    /** @brief Previous value of _enabled parameter for change detection */
    bool            _last_enabled;
    
    /** 
     * @brief Overall fence feature enable (FENCE_ENABLE parameter)
     * 
     * @details Master enable parameter. Bitmask of which fence types are currently enabled.
     *          Corresponds to FENCE_ENABLE parameter. Bits correspond to AC_FENCE_TYPE_* defines.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     */
    AP_Int8         _enabled;
    
    /** 
     * @brief Auto-enable mode configuration (FENCE_AUTOENABLE parameter)
     * 
     * @details Controls automatic fence enabling during takeoff/arming. See AutoEnable enum
     *          for valid values. Corresponds to FENCE_AUTOENABLE parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @see AutoEnable enum for mode options
     */
    AP_Int8         _auto_enabled;
    
    /** @brief Previous value of _auto_enabled parameter for change detection */
    uint8_t         _last_auto_enabled;
    
    /** 
     * @brief Configured fence types available (FENCE_TYPE parameter)
     * 
     * @details Bitmask of which fence types are configured and available for use.
     *          Corresponds to FENCE_TYPE parameter. Bits correspond to AC_FENCE_TYPE_* defines.
     *          Fence types not in this bitmask cannot be enabled even if requested.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     */
    AP_Int8         _configured_fences;
    
    /** 
     * @brief Breach recovery action (FENCE_ACTION parameter)
     * 
     * @details Specifies what action vehicle takes when fence is breached. See Action enum
     *          for valid actions. Corresponds to FENCE_ACTION parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @see Action enum for available recovery actions
     * @warning Action choice affects vehicle safety. ALWAYS_LAND could strand vehicle far from home.
     */
    AP_Enum<Action> _action;
    
    /** 
     * @brief Maximum altitude limit in meters (FENCE_ALT_MAX parameter)
     * 
     * @details Upper altitude boundary in meters above home/origin. Vehicle triggers breach
     *          action if altitude exceeds this value plus margin. Corresponds to FENCE_ALT_MAX parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Altitude measured relative to EKF origin (typically home position altitude).
     * @note Units: meters
     */
    AP_Float        _alt_max;
    
    /** 
     * @brief Minimum altitude limit in meters (FENCE_ALT_MIN parameter)
     * 
     * @details Lower altitude boundary (floor) in meters above home/origin. Vehicle triggers
     *          breach action if altitude drops below this value minus margin. Corresponds to
     *          FENCE_ALT_MIN parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Altitude measured relative to EKF origin (typically home position altitude).
     * @note Often auto-disabled during landing to prevent breach during descent.
     * @note Units: meters
     */
    AP_Float        _alt_min;
    
    /** 
     * @brief Circle fence radius in meters (FENCE_RADIUS parameter)
     * 
     * @details Horizontal circular boundary radius in meters from home position. Vehicle triggers
     *          breach action if horizontal distance from home exceeds this value plus margin.
     *          Corresponds to FENCE_RADIUS parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Distance measured horizontally in NED frame from EKF origin.
     * @note Units: meters
     */
    AP_Float        _circle_radius;
    
    /** 
     * @brief Safety margin distance in meters (FENCE_MARGIN parameter)
     * 
     * @details Safety buffer distance in meters added to fence boundaries. Autopilot attempts
     *          to maintain at least this distance from fence boundaries. Breach occurs when
     *          vehicle enters margin zone. Corresponds to FENCE_MARGIN parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Margin too small can cause fence oscillation. Recommend at least 2-5 meters.
     * @note Units: meters
     * @warning Insufficient margin can cause repeated breach/recovery cycles.
     */
    AP_Float        _margin;
    
    /** 
     * @brief Number of polygon fence vertices (FENCE_TOTAL parameter)
     * 
     * @details Total number of polygon vertices stored. Used by polygon loader to know how
     *          many points to load. Corresponds to FENCE_TOTAL parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Maximum vertices limited by storage and memory constraints.
     */
    AP_Int8         _total;
    
    /** 
     * @brief Rally point return mode (FENCE_RET_RALLY parameter)
     * 
     * @details Controls whether RTL breach action returns to fence return point or rally point.
     *          0 = return to fence return point, 1 = return to nearest rally point.
     *          Corresponds to FENCE_RET_RALLY parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     */
    AP_Int8         _ret_rally;
    
    /** 
     * @brief Return altitude in meters (FENCE_RET_ALT parameter)
     * 
     * @details Altitude in meters to climb/descend to during RTL breach recovery action.
     *          Corresponds to FENCE_RET_ALT parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Units: meters above home/origin
     */
    AP_Int16        _ret_altitude;
    
    /** 
     * @brief Fence behavior option flags (FENCE_OPTIONS parameter)
     * 
     * @details Bitmask of option flags that modify fence behavior. See OPTIONS enum for bit
     *          definitions. Corresponds to FENCE_OPTIONS parameter.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @see OPTIONS enum for available option flags
     */
    AP_Int16        _options;
    
    /** 
     * @brief Margin breach notification frequency in Hz (FENCE_NOTIFY_FREQ parameter)
     * 
     * @details Maximum rate at which margin breach notifications are sent to GCS. Used to
     *          rate-limit notifications and prevent message spam. Corresponds to FENCE_NOTIFY_FREQ
     *          parameter. Only active if OPTIONS::NOTIFY_MARGIN_BREACH is enabled.
     * 
     * @note Persistent parameter saved to EEPROM/flash.
     * @note Units: Hz (notifications per second)
     * @note Value of 0 disables notification rate limiting.
     */
    AP_Float        _notify_freq;
    
    /** @} */ // End of Parameter Variables group

    /**
     * @name Backup Fence Boundaries for Hysteresis
     * @{
     * 
     * Backup boundary values used to implement hysteresis and prevent re-breach notifications
     * during recovery. When breach is detected, backup is set to current vehicle position
     * relative to fence. Breach is only re-triggered if vehicle continues moving further
     * outside the fence beyond the backup boundary.
     */
    
    /** 
     * @brief Backup altitude upper limit for breach hysteresis
     * 
     * @details Stores altitude at time of max altitude breach. Used to detect if vehicle
     *          continues climbing after initial breach. Prevents repeated breach notifications
     *          during recovery if altitude oscillates near fence boundary.
     * 
     * @note Units: meters above home/origin
     * @note Set to current altitude when ALT_MAX breach first detected.
     */
    float           _alt_max_backup;
    
    /** 
     * @brief Backup altitude lower limit for breach hysteresis
     * 
     * @details Stores altitude at time of min altitude (floor) breach. Used to detect if vehicle
     *          continues descending after initial breach. Prevents repeated breach notifications
     *          during recovery if altitude oscillates near fence boundary.
     * 
     * @note Units: meters above home/origin
     * @note Set to current altitude when ALT_MIN breach first detected.
     */
    float           _alt_min_backup;
    
    /** 
     * @brief Backup circle fence radius for breach hysteresis
     * 
     * @details Stores horizontal distance from home at time of circle fence breach. Used to
     *          detect if vehicle continues moving away after initial breach. Prevents repeated
     *          breach notifications during recovery if position oscillates near fence boundary.
     * 
     * @note Units: meters from home (horizontal distance)
     * @note Set to current distance when CIRCLE breach first detected.
     */
    float           _circle_radius_backup;

    /** @} */ // End of Backup Fence Boundaries group

    /**
     * @name Breach Distance Tracking
     * @{
     * 
     * Distance values indicating how far vehicle is from each fence boundary. Negative values
     * indicate vehicle is inside fence (safe), positive values indicate vehicle is outside
     * fence (breach). Used for breach severity reporting and margin breach detection.
     */
    
    /** 
     * @brief Distance above maximum altitude fence
     * 
     * @details Signed distance in meters above altitude max fence boundary. Positive = above
     *          fence (breach), negative = below fence (safe), zero = exactly at boundary.
     * 
     * @note Units: meters (positive = breach, negative = safe)
     */
    float           _alt_max_breach_distance;
    
    /** 
     * @brief Distance below minimum altitude fence
     * 
     * @details Signed distance in meters below altitude min (floor) fence boundary. Positive =
     *          below fence (breach), negative = above fence (safe), zero = exactly at boundary.
     * 
     * @note Units: meters (positive = breach, negative = safe)
     */
    float           _alt_min_breach_distance;
    
    /** 
     * @brief Distance beyond circular fence boundary
     * 
     * @details Signed distance in meters beyond circle fence radius. Positive = outside fence
     *          (breach), negative = inside fence (safe), zero = exactly at boundary.
     * 
     * @note Units: meters (positive = breach, negative = safe)
     */
    float           _circle_breach_distance;
    
    /** 
     * @brief Distance beyond polygon fence boundary
     * 
     * @details Signed distance in meters beyond polygon fence boundaries. Positive = outside
     *          all inclusion zones or inside any exclusion zone (breach), negative = inside
     *          inclusion zones and outside exclusion zones (safe).
     * 
     * @note Units: meters (positive = breach, negative = safe)
     * @note Distance calculation depends on polygon loader implementation.
     */
    float           _polygon_breach_distance;

    /** @} */ // End of Breach Distance Tracking group

    /**
     * @name Internal State Variables
     * @{
     * 
     * Additional internal state tracking for fence system operation.
     */
    
    /** 
     * @brief Distance from home position in meters
     * 
     * @details Horizontal distance from EKF origin (home). Used for circle fence checks and
     *          distance reporting. Updated during fence checks.
     * 
     * @note Units: meters (horizontal distance in NED frame)
     */
    float           _home_distance;
    
    /** 
     * @brief Distance to nearest fence boundary
     * 
     * @details Minimum distance to any enabled fence boundary. Positive if inside all fences,
     *          negative if outside any fence. Used for distance-to-fence reporting.
     * 
     * @note Units: meters
     */
    float           _fence_distance;

    /** @} */ // End of Internal State Variables group

    /**
     * @name Breach Tracking Variables
     * @{
     * 
     * Variables tracking fence breach history and state. Used for breach logging, recovery
     * actions, and operator notifications.
     */
    
    /** 
     * @brief Bitmask of currently breached fence types
     * 
     * @details Bitmask where each bit represents a breached fence type (AC_FENCE_TYPE_*).
     *          Bit is set when fence breached, cleared when vehicle returns inside fence.
     *          Used to determine if recovery action should be triggered.
     * 
     * @note Zero = no breaches, non-zero = at least one fence breached
     * @note Multiple fences can be breached simultaneously
     */
    uint8_t         _breached_fences;
    
    /** 
     * @brief Bitmask of fence types with margin breaches
     * 
     * @details Bitmask where each bit represents a fence type whose margin (safety buffer)
     *          has been breached. Provides advance warning before actual fence breach occurs.
     * 
     * @note Margin breach does not trigger recovery action, only notifications (if enabled).
     * @see OPTIONS::NOTIFY_MARGIN_BREACH for margin notification control
     */
    uint8_t         _breached_fence_margins;
    
    /** 
     * @brief Time of last fence breach in milliseconds
     * 
     * @details System time (AP_HAL::millis()) when most recent fence breach was recorded.
     *          Used for breach duration calculations and recovery timing.
     * 
     * @note Units: milliseconds since system boot
     */
    uint32_t        _breach_time;
    
    /** 
     * @brief Time of last margin breach in milliseconds
     * 
     * @details System time (AP_HAL::millis()) when most recent fence margin breach was recorded.
     *          Used for margin breach notifications and logging.
     * 
     * @note Units: milliseconds since system boot
     */
    uint32_t        _margin_breach_time;
    
    /** 
     * @brief Total number of fence breaches since boot
     * 
     * @details Counter incremented each time any fence is breached. Used for statistics
     *          and logging. Persists until reboot.
     * 
     * @note Counter does not decrement. Provides total breach count for flight session.
     */
    uint16_t        _breach_count;
    
    /** 
     * @brief Last time fence breach notification was sent to GCS
     * 
     * @details System time when last breach notification message was sent. Used to rate-limit
     *          breach notifications and prevent message spam.
     * 
     * @note Units: milliseconds since system boot
     */
    uint32_t _last_breach_notify_sent_ms;
    
    /** 
     * @brief Last time margin breach notification was sent to GCS
     * 
     * @details System time when last margin breach notification message was sent. Used to
     *          rate-limit margin breach notifications per FENCE_NOTIFY_FREQ parameter.
     * 
     * @note Units: milliseconds since system boot
     * @note Only used if OPTIONS::NOTIFY_MARGIN_BREACH is enabled.
     */
    uint32_t _last_margin_breach_notify_sent_ms;

    /** @} */ // End of Breach Tracking Variables group

    /**
     * @name Manual Recovery State
     * @{
     * 
     * State tracking for pilot manual recovery window. When pilot takes manual control after
     * breach, fences are temporarily disabled for 10 seconds to allow pilot to recover.
     */
    
    /** 
     * @brief System time when pilot initiated manual recovery
     * 
     * @details System time (AP_HAL::millis()) when manual_recovery_start() was called,
     *          indicating pilot took manual control after fence breach. Fences are disabled
     *          for 10 seconds from this time to allow pilot recovery.
     * 
     * @note Units: milliseconds since system boot
     * @note Zero = no manual recovery in progress
     * @note 10-second window is hardcoded in fence check logic
     */
    uint32_t        _manual_recovery_start_ms;

    /** @} */ // End of Manual Recovery State group

    /**
     * @enum MinAltState
     * @brief Minimum altitude fence manual override state
     * 
     * @details Tracks whether altitude floor (ALT_MIN) fence has been manually enabled or
     *          disabled, overriding automatic enable/disable logic. Used to respect pilot
     *          or GCS commands for floor fence control.
     */
    enum class MinAltState
    {
        DEFAULT = 0,            ///< No manual override, use automatic enable/disable logic
        MANUALLY_ENABLED,       ///< Floor fence manually enabled by pilot/GCS command
        MANUALLY_DISABLED       ///< Floor fence manually disabled by pilot/GCS command
    } _min_alt_state;           ///< Current minimum altitude fence manual override state
    

    /**
     * @brief Polygon fence loader and manager
     * 
     * @details Manages loading, storage, and checking of polygon fence vertices. Handles
     *          both inclusion and exclusion polygon fences. Loads vertices from EEPROM/flash
     *          or SD card and performs point-in-polygon boundary checks.
     * 
     * @note Initialized with references to _total (vertex count) and _options (behavior flags).
     * @note Polygon loading is asynchronous via update() method to avoid blocking main loop.
     * 
     * @see AC_PolyFence_loader for polygon fence implementation details
     * 
     * Source: libraries/AC_Fence/AC_PolyFence_loader.h
     */
    AC_PolyFence_loader _poly_loader{_total, _options};
    
    /** @} */ // End of Private Members group
};

/**
 * @namespace AP
 * @brief ArduPilot global namespace for singleton accessors
 */
namespace AP {
    /**
     * @brief Get AC_Fence singleton instance
     * 
     * @details Convenience accessor function for AC_Fence singleton. Provides global access
     *          to the fence system from anywhere in ArduPilot code. Preferred access method
     *          over direct singleton pointer access.
     * 
     * @return Pointer to AC_Fence singleton instance, or nullptr if not instantiated
     * 
     * @note Returns nullptr if fence system is not compiled in (AP_FENCE_ENABLED not defined).
     * @note Always check for nullptr before dereferencing in case fence is not available.
     * 
     * Usage example:
     * @code
     * AC_Fence *fence = AP::fence();
     * if (fence != nullptr && fence->enabled()) {
     *     uint8_t breaches = fence->get_breaches();
     *     // Handle fence breaches...
     * }
     * @endcode
     * 
     * @see AC_Fence::get_singleton() for direct singleton access
     */
    AC_Fence *fence();
};

#endif // AP_FENCE_ENABLED
