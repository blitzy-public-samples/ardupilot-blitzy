/**
 * @file AC_WeatherVane.h
 * @brief Automatic yaw-into-wind control for ArduPilot
 * 
 * @details Implements weathervane functionality that automatically points the vehicle
 *          into the wind during position hold modes, takeoff, and landing. Helps reduce
 *          drift in windy conditions by minimizing lateral wind exposure through automatic
 *          yaw corrections. The system monitors vehicle attitude and automatically adds
 *          yaw commands to align with wind direction based on configured parameters.
 * 
 * Source: libraries/AC_AttitudeControl/AC_WeatherVane.h
 */

#include <AP_Param/AP_Param.h>

/**
 * @class AC_WeatherVane
 * @brief Autonomous yaw-into-wind control system
 * 
 * @details Provides automatic yaw control to point the vehicle into the wind, reducing
 *          drift during hover, loiter, position hold, takeoff, and landing operations.
 *          The weathervane system monitors vehicle attitude errors and automatically
 *          adds corrective yaw commands when enabled.
 * 
 *          Key Features:
 *          - Configurable direction options (nose-in, tail-in, side-in, or automatic)
 *          - Height and velocity thresholds for activation
 *          - Smooth gain-based correction to prevent oscillations
 *          - Separate settings for normal flight, takeoff, and landing
 *          - Pilot override capability - pilot yaw input always takes precedence
 *          - One-time GCS notification on first activation
 * 
 *          Operation:
 *          The weathervane calculates desired yaw corrections based on vehicle attitude
 *          and wind conditions. It only operates above configured height and below maximum
 *          velocity thresholds. When pilot commands yaw, weathervane is temporarily disabled
 *          to ensure full pilot authority.
 * 
 * @note Weathervane is only active when:
 *       - Enabled via parameters (DIRECTION parameter != 0)
 *       - Above minimum height threshold (_min_height)
 *       - Below maximum horizontal velocity (_max_vel_xy)
 *       - Below maximum vertical velocity (_max_vel_z)
 *       - Pilot is not commanding yaw
 *       - System is allowed via allow_weathervaning()
 * 
 * @note GCS notification is sent once on first activation per flight
 * 
 * @warning Incorrect gain settings can cause yaw oscillations or unstable behavior.
 *          Start with low gain values and increase gradually during testing.
 * 
 * @warning Direction parameters affect vehicle orientation relative to wind.
 *          Verify expected behavior in SITL before flight testing.
 * 
 * @see AC_PosControl Uses weathervane for position hold modes
 * @see AC_AttitudeControl Receives weathervane yaw rate corrections
 */
class AC_WeatherVane {
    public:

        /**
         * @brief Constructor for weathervane system
         * 
         * @details Initializes the weathervane control system with default parameter values.
         *          All state variables are initialized to safe defaults.
         */
        AC_WeatherVane(void);

        CLASS_NO_COPY(AC_WeatherVane);

        /**
         * @brief Calculate and return the yaw output to weathervane the vehicle
         * 
         * @details Computes the desired yaw rate correction to point the vehicle into the wind.
         *          This is the main weathervane calculation function, called at the attitude
         *          control loop rate (typically 400Hz for multirotors).
         * 
         *          The function evaluates current flight conditions against configured thresholds
         *          (height, velocity, pilot input) and calculates appropriate yaw corrections
         *          based on vehicle attitude and wind direction. Different behaviors can be
         *          configured for normal flight, takeoff, and landing phases.
         * 
         *          Algorithm:
         *          1. Check if weathervane is enabled and allowed
         *          2. Verify height and velocity thresholds
         *          3. Check for pilot yaw input (overrides weathervane)
         *          4. Select appropriate direction mode (normal/takeoff/landing)
         *          5. Calculate yaw error based on roll and pitch angles
         *          6. Apply gain and rate limiting to output
         *          7. Send GCS notification on first activation
         * 
         * @param[out] yaw_output         Weathervane yaw rate output in centidegrees/s (body frame)
         * @param[in]  pilot_yaw          Pilot yaw input in centidegrees/s (positive = yaw right)
         * @param[in]  hgt                Current height above home in meters
         * @param[in]  roll_cdeg          Current vehicle roll angle in centidegrees (positive = right)
         * @param[in]  pitch_cdeg         Current vehicle pitch angle in centidegrees (positive = nose up)
         * @param[in]  is_takeoff         True if vehicle is in takeoff phase
         * @param[in]  is_landing         True if vehicle is in landing phase
         * 
         * @return true if weathervane is actively modifying yaw (output is valid), false otherwise
         * 
         * @note Returns false (inactive) if:
         *       - Weathervane disabled via parameters
         *       - Not allowed via allow_weathervaning()
         *       - Below minimum height threshold
         *       - Velocity exceeds maximum thresholds
         *       - Pilot is commanding yaw
         * 
         * @note Weathervane output smoothly ramps up over WEATHERVANE_TIMEOUT_MS to prevent
         *       sudden yaw movements on activation
         * 
         * @note First activation sends MAV_CMD_DO_WEATHERVANE notification to GCS
         * 
         * @warning Pilot yaw input always takes precedence - weathervane immediately deactivates
         *          when pilot commands yaw to ensure full manual control authority
         * 
         * @warning Output must be added to final yaw rate command, not used as absolute setpoint
         */
        bool get_yaw_out(float &yaw_output, const int16_t pilot_yaw, const float hgt, const float roll_cdeg, const float pitch_cdeg, const bool is_takeoff, const bool is_landing);

        /**
         * @brief Reset weathervane state to initial conditions
         * 
         * @details Clears all internal state flags and resets tracking variables.
         *          Should be called whenever the weathervaning process is interrupted,
         *          such as during mode transitions, manual control takeover, or when
         *          re-initializing position control.
         * 
         *          Reset operations:
         *          - Clears last_output to zero
         *          - Resets active_msg_sent flag (allows new GCS notification)
         *          - Clears activation timestamp
         *          - Resets check timestamp for rate limiting
         * 
         * @note Call this when transitioning between flight modes that may use weathervane
         * 
         * @note After reset, GCS will receive a new activation notification when weathervane
         *       engages again
         */
        void reset(void);

        /**
         * @brief Enable or disable weathervane operation externally
         * 
         * @details Provides external control over weathervane activation independent of
         *          parameter settings. This allows flight mode logic or other systems to
         *          temporarily prevent weathervane operation without modifying parameters.
         * 
         *          The weathervane will only operate when BOTH:
         *          - Parameter-based enable is active (DIRECTION != 0)
         *          - External allow flag is true (set via this method)
         * 
         * @param[in] allow True to allow weathervane operation, false to prevent it
         * 
         * @note This does not modify parameter values - it provides runtime control
         * 
         * @note Default state is allowed (true) to prevent race conditions during initialization
         * 
         * @note Typically used by flight modes to disable weathervane during specific maneuvers
         *       or when weathervane behavior is not appropriate for the current operation
         */
        void allow_weathervaning(bool allow) { allowed = allow; }

        /**
         * @brief Parameter table for weathervane configuration
         * 
         * @details Defines the AP_Param group structure for weathervane parameters.
         *          Allows configuration via ground control station or parameter files.
         */
        static const struct AP_Param::GroupInfo var_info[];

    private:

        /**
         * @enum Direction
         * @brief Wind-relative orientation options for weathervane control
         * 
         * @details Defines how the vehicle should orient itself relative to wind direction.
         *          Different options optimize for different vehicle types and operational needs.
         *          For example, tailsitters may prefer side-in or tail-in orientations, while
         *          standard multirotors typically use nose-in for reduced drag.
         */
        enum class Direction {
            TAKEOFF_OR_LAND_ONLY = -1, ///< Weathervane only active during takeoff or landing phases
            OFF = 0,                    ///< Weathervane disabled
            NOSE_IN = 1,                ///< Point nose into wind (minimum frontal area for standard multirotors)
            NOSE_OR_TAIL_IN = 2,        ///< Point nose or tail into wind, whichever requires less rotation
            SIDE_IN = 3,                ///< Point side into wind (for copter tailsitters with optimized side profile)
            TAIL_IN = 4,                ///< Point tail into wind (for tailsitters, facilitates easier descent)
        };

        /**
         * @enum Options
         * @brief Bitfield options for weathervane behavior modifications
         * 
         * @details Provides additional configuration flags to enable optional weathervane
         *          features or behavior modifications. Options are combined using bitwise OR.
         */
        enum class Options {
            PITCH_ENABLE = (1<<0),  ///< Enable pitch-based weathervane correction (use pitch angle in addition to roll)
        };
    
        // Parameters
        
        /**
         * @brief Weathervane direction mode for normal flight
         * 
         * @details Selects wind-relative orientation during normal flight operations (loiter,
         *          position hold, etc.). See Direction enum for available options.
         * 
         * @note Parameter: WVANE_DIRECTION
         * @note Range: -1 to 4 (see Direction enum values)
         * @note Default: 0 (OFF)
         */
        AP_Int8 _direction;
        
        /**
         * @brief Weathervane correction gain
         * 
         * @details Controls the aggressiveness of yaw corrections. Higher values produce
         *          faster alignment with wind but may cause oscillations. Lower values
         *          provide smoother operation but slower wind alignment.
         * 
         * @note Parameter: WVANE_GAIN
         * @note Range: [0.0, 1.0]
         * @note Default: Typically 0.2 to 0.5 depending on vehicle
         * 
         * @warning Values above 0.8 may cause yaw oscillations. Test carefully in SITL first.
         */
        AP_Float _gain;
        
        /**
         * @brief Minimum deadzone angle for weathervane activation
         * 
         * @details Weathervane will not activate unless attitude error exceeds this threshold.
         *          Prevents continuous small corrections and reduces control system activity
         *          when vehicle is already approximately aligned with wind.
         * 
         * @note Parameter: WVANE_MINROLL (historical naming, applies to attitude error)
         * @note Units: degrees
         * @note Range: [0, 90]
         * @note Default: Typically 5 to 10 degrees
         */
        AP_Float _min_dz_ang_deg;
        
        /**
         * @brief Minimum height above home for weathervane operation
         * 
         * @details Weathervane is disabled below this height to prevent ground effect
         *          interference and ensure safe low-altitude maneuvering without automatic
         *          yaw corrections.
         * 
         * @note Parameter: WVANE_HGT_MIN
         * @note Units: meters
         * @note Range: [0, 100]
         * @note Default: Typically 3 to 5 meters
         */
        AP_Float _min_height;
        
        /**
         * @brief Maximum horizontal velocity for weathervane operation
         * 
         * @details Weathervane is disabled above this horizontal ground speed. Prevents
         *          weathervane from interfering during aggressive maneuvering or when
         *          strong position corrections are being applied.
         * 
         * @note Parameter: WVANE_VELXY_MAX
         * @note Units: m/s
         * @note Range: [0, 20]
         * @note Default: Typically 1 to 2 m/s
         */
        AP_Float _max_vel_xy;
        
        /**
         * @brief Maximum vertical velocity for weathervane operation
         * 
         * @details Weathervane is disabled when vertical speed exceeds this threshold.
         *          Prevents weathervane interference during aggressive climbs or descents.
         * 
         * @note Parameter: WVANE_VELZ_MAX
         * @note Units: m/s
         * @note Range: [0, 10]
         * @note Default: Typically 0.5 to 1.0 m/s
         */
        AP_Float _max_vel_z;
        
        /**
         * @brief Weathervane direction mode during landing
         * 
         * @details Separate direction setting for landing phase. Allows different orientation
         *          strategy during descent and touchdown. Set to -1 to use normal _direction.
         * 
         * @note Parameter: WVANE_LND_DIR
         * @note Range: -1 to 4 (see Direction enum, -1 = use normal direction)
         * @note Default: -1 (use normal direction setting)
         */
        AP_Int8 _landing_direction;
        
        /**
         * @brief Weathervane direction mode during takeoff
         * 
         * @details Separate direction setting for takeoff phase. Allows different orientation
         *          strategy during ascent from ground. Set to -1 to use normal _direction.
         * 
         * @note Parameter: WVANE_TKOFF_DIR
         * @note Range: -1 to 4 (see Direction enum, -1 = use normal direction)
         * @note Default: -1 (use normal direction setting)
         */
        AP_Int8 _takeoff_direction;
        
        /**
         * @brief Weathervane option flags
         * 
         * @details Bitfield of optional features and behavior modifications.
         *          See Options enum for available flags.
         * 
         * @note Parameter: WVANE_OPTIONS
         * @note Default: 0 (no options enabled)
         */
        AP_Int16 _options;

        /**
         * @brief Previous weathervane yaw rate output
         * 
         * @details Stores the last calculated yaw output for rate limiting and smooth
         *          transitions. Used to implement gradual ramp-up of corrections.
         * 
         * @note Units: centidegrees/s
         */
        float last_output;
        
        /**
         * @brief GCS notification sent flag
         * 
         * @details Set to true after first GCS notification is sent. Prevents repeated
         *          notifications throughout the flight while allowing notification on
         *          subsequent flights (cleared by reset()).
         */
        bool active_msg_sent;
        
        /**
         * @brief Timestamp of first weathervane activation
         * 
         * @details Records system time (milliseconds) when weathervane first activated
         *          during current engagement. Used to implement smooth gain ramp-up over
         *          WEATHERVANE_TIMEOUT_MS period.
         * 
         * @note Units: milliseconds (system time)
         */
        uint32_t first_activate_ms;
        
        /**
         * @brief Timestamp of last weathervane check
         * 
         * @details Records system time (milliseconds) of last get_yaw_out() evaluation.
         *          Used for rate limiting calculations and timeout detection.
         * 
         * @note Units: milliseconds (system time)
         */
        uint32_t last_check_ms;

        /**
         * @brief External allow flag for weathervane operation
         * 
         * @details Permits external systems (flight modes, mission logic) to enable or
         *          disable weathervane independent of parameter settings. Weathervane
         *          operates only when this is true AND parameter-based enable is active.
         * 
         * @note Initialized to true to avoid race conditions during system startup
         *       where RC channels may not be fully initialized yet
         * 
         * @see allow_weathervaning() Method to modify this flag
         */
        bool allowed = true;
};
