/**
 * @file AP_AutoTune.h
 * @brief In-flight PID auto-tuner for fixed-wing aircraft
 * 
 * @details Implements event-driven autotune algorithm that excites each axis (roll, pitch, yaw)
 *          with commanded rate events, measures actuator→rate responses, computes
 *          feedforward/PID gains iteratively with safety limits and convergence checks.
 *          
 *          Algorithm: Issues rate demands → Measures actuator and rate responses →
 *          Estimates FF from single-event actuator/rate ratios → Adjusts P/I/D/IMAX
 *          with slew-limited convergence → Validates with oscillation detection
 *          
 *          Integration: Reads/writes AC_PID gains (ff, kP, kI, kD, kIMAX), uses
 *          AP::scheduler for dt, AP_HAL timing, AP_Logger for 25Hz telemetry
 *          
 * @note Autotune must be run in calm conditions with sufficient altitude
 * @warning User must monitor vehicle and be ready to switch modes if oscillations occur
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <Filter/SlewLimiter.h>

#include <Filter/ModeFilter.h>

/**
 * @class AP_AutoTune
 * @brief Fixed-wing in-flight PID controller autotuner
 * 
 * @details State machine that issues rate commands, measures responses, estimates
 *          feedforward from single-event actuator/rate ratios, and adjusts P/I/D/IMAX
 *          gains with convergence checks and safety limits.
 *          
 *          State Machine: IDLE → DEMAND_POS → DEMAND_NEG → repeat
 *          Each demand cycle measures actuator output and rate response, computes
 *          FF estimate, updates PID gains based on tracking performance
 *          
 *          Integration Points:
 *          - Reads/writes AC_PID gains (ff, kP, kI, kD, kIMAX)
 *          - Uses AP::scheduler()->get_loop_period_s() for dt
 *          - Uses AP_HAL::millis() for state timing
 *          - Logs to AP_Logger at 25Hz (ATRP message)
 *          - Persists gains to EEPROM with wear reduction
 *          
 *          Safety Features:
 *          - Conservative per-loop gain adjustment limits
 *          - Oscillation detection via rate/target correlation
 *          - User-controllable tuning aggressiveness (autotune_level 0-10)
 *          - Original gains restored on mode exit
 *          - Slew-limited P/D updates prevent instability
 *          
 * @note Coordinate System: Body frame rates (deg/s)
 * @warning Autotune should only be run in calm conditions with sufficient altitude.
 *          Pilot must monitor vehicle and be ready to switch modes immediately if
 *          oscillations occur. Not suitable for first flights or untested airframes.
 */
class AP_AutoTune
{
public:
    /**
     * @struct ATGains
     * @brief Persistent tuning parameters for one axis
     * 
     * @details Stores time constant, rate limits, and PID/FF gains for roll, pitch, or yaw axis.
     *          Values are saved to EEPROM when changed during autotune.
     */
    struct ATGains {
        AP_Float tau;         ///< Time constant for response (seconds)
        AP_Int16 rmax_pos;    ///< Maximum positive rate (deg/s)
        AP_Int16 rmax_neg;    ///< Maximum negative rate (deg/s, stored as positive value)
        float FF;             ///< Feedforward gain (dimensionless)
        float P;              ///< Proportional gain (dimensionless)
        float I;              ///< Integral gain (dimensionless)
        float D;              ///< Derivative gain (dimensionless)
        float IMAX;           ///< Integral windup limit (dimensionless, 0-1 range)
        float flt_T;          ///< Target filter frequency coefficient (Hz)
        float flt_E;          ///< Error filter frequency coefficient (Hz)
        float flt_D;          ///< Derivative filter frequency coefficient (Hz)
    };

    /**
     * @enum ATType
     * @brief Axis selection for autotune
     * 
     * @details Identifies which control axis (roll, pitch, yaw) this autotune instance
     *          is tuning. Used for logging and axis-specific string identification.
     */
    enum ATType {
        AUTOTUNE_ROLL  = 0,  ///< Roll axis tuning
        AUTOTUNE_PITCH = 1,  ///< Pitch axis tuning
        AUTOTUNE_YAW = 2,    ///< Yaw axis tuning
    };

    /**
     * @enum Options
     * @brief Autotune option flags to disable specific filter updates
     * 
     * @details Bitmask options controlled by AUTOTUNE_OPTIONS parameter to allow
     *          users to prevent autotune from updating specific filter coefficients.
     */
    enum Options {
        DISABLE_FLTD_UPDATE = 0,  ///< Bit 0: Disable derivative filter (flt_D) updates
        DISABLE_FLTT_UPDATE = 1   ///< Bit 1: Disable target filter (flt_T) updates
    };

    /**
     * @struct log_ATRP
     * @brief Binary log packet structure for autotune telemetry
     * 
     * @details PACKED structure logged at 25Hz during autotune to record state machine
     *          progress, gain evolution, and tuning actions. Used for post-flight
     *          analysis and debugging of autotune performance.
     *          
     * @note Message name: ATRP (AutoTune Rate PID)
     * @note Logging rate: 25Hz (every 40ms)
     */
    struct PACKED log_ATRP {
        LOG_PACKET_HEADER;
        uint64_t time_us;    ///< Timestamp in microseconds
        uint8_t type;        ///< ATType (0=roll, 1=pitch, 2=yaw)
        uint8_t state;       ///< ATState (0=IDLE, 1=DEMAND_POS, 2=DEMAND_NEG)
        float actuator;      ///< Filtered actuator output (normalized, typically -1 to +1)
        float P_slew;        ///< Slew-limited P gain (dimensionless)
        float D_slew;        ///< Slew-limited D gain (dimensionless)
        float FF_single;     ///< Single-event feedforward estimate from last demand (dimensionless)
        float FF;            ///< Current feedforward gain (dimensionless)
        float P;             ///< Current proportional gain (dimensionless)
        float I;             ///< Current integral gain (dimensionless)
        float D;             ///< Current derivative gain (dimensionless)
        uint8_t action;      ///< Action enum (gain adjustment action taken this cycle)
        float rmax;          ///< Current maximum rate limit (deg/s)
        float tau;           ///< Current time constant (seconds)
    };

    /**
     * @brief Constructor for AP_AutoTune instance
     * 
     * @details Initializes autotune for one axis with references to gain storage,
     *          vehicle parameters, and the PID controller to be tuned.
     * 
     * @param[in,out] _gains Reference to ATGains structure to read/write tuned parameters
     * @param[in] type Axis type (AUTOTUNE_ROLL, AUTOTUNE_PITCH, or AUTOTUNE_YAW)
     * @param[in] parms Reference to AP_FixedWing parameters (autotune_level, autotune_options)
     * @param[in,out] rpid Reference to AC_PID rate controller to tune
     * 
     * @note Does not start tuning - call start() when autotune mode is entered
     */
    AP_AutoTune(ATGains &_gains, ATType type, const AP_FixedWing &parms, class AC_PID &rpid);

    /**
     * @brief Start autotune when mode is entered
     * 
     * @details Initializes state machine, saves current gains for restoration,
     *          resets filters and accumulators, sets running flag true.
     *          Reads autotune_level parameter to determine initial tau/rmax targets.
     * 
     * @note Call this once when pilot selects autotune mode
     * @warning Ensure vehicle is in stable flight with sufficient altitude before calling
     */
    void start(void);

    /**
     * @brief Stop autotune and restore original gains
     * 
     * @details Restores gains that were active before start() was called,
     *          resets state machine to IDLE, sets running flag false.
     *          Does NOT save gains to EEPROM - gains are only saved if
     *          autotune completes successfully.
     * 
     * @note Call this when pilot exits autotune mode or if oscillations detected
     */
    void stop(void);

    /**
     * @brief Main autotune update loop - call at vehicle loop rate
     * 
     * @details State machine update that processes rate controller outputs,
     *          measures responses, computes gain adjustments, and logs telemetry.
     *          Typically called at 50-400Hz depending on vehicle type.
     *          
     *          Processing sequence:
     *          1. Read actuator output, rate, and target from pid_info
     *          2. Apply low-pass filtering to measurements
     *          3. Update state machine (IDLE→DEMAND_POS→DEMAND_NEG)
     *          4. Compute FF estimate from actuator/rate ratio
     *          5. Adjust P/D gains based on tracking error
     *          6. Apply slew limits to gain changes
     *          7. Write updated gains to AC_PID controller
     *          8. Log ATRP message at 25Hz
     * 
     * @param[in] pid_info Rate controller state from AC_PID::get_pid_info()
     *                     Contains: target (deg/s), actual (deg/s), error, P/I/D/FF terms
     * @param[in] scaler Airspeed scaling factor for gain adjustment (typically 0.5-2.0)
     *                   Scales gains to account for dynamic pressure variations
     * @param[in] angle_err_deg Current attitude tracking error in degrees
     *                          Used to determine if aircraft is following demands accurately
     * 
     * @note Only processes when running flag is true
     * @note Call at main loop rate (typically 50-400Hz)
     * @warning Large angle errors (>30°) may trigger conservative tuning or abort
     */
    void update(struct AP_PIDInfo &pid_info, float scaler, float angle_err_deg);

    bool running;  ///< True when autotune is active, false otherwise

    /**
     * @brief Convert ATType enum to human-readable string
     * 
     * @param[in] _type Axis type (AUTOTUNE_ROLL, AUTOTUNE_PITCH, or AUTOTUNE_YAW)
     * 
     * @return Pointer to static string "roll", "pitch", or "yaw"
     * 
     * @note Static method - can be called without instance
     * @note Returns "unknown" for invalid type values
     */
    static const char *axis_string(ATType _type);

private:
    ATGains &current;     ///< Reference to current gains being tuned (read/write)
    class AC_PID &rpid;   ///< Reference to rate PID controller being tuned

    ATType type;          ///< Axis type (roll, pitch, or yaw)

    const AP_FixedWing &aparm;  ///< Reference to fixed-wing parameters (autotune_level, autotune_options)

    ATGains restore;      ///< Original gains to restore if autotune is aborted
    ATGains last_save;    ///< Last saved gains for EEPROM wear reduction

    uint32_t last_log_ms; ///< Timestamp of last ATRP log message (milliseconds)

    /**
     * @enum ATState
     * @brief State machine states for autotune algorithm
     * 
     * @details Autotune cycles through states to generate rate demands and measure responses:
     *          IDLE: Waiting for conditions to start next demand cycle
     *          DEMAND_POS: Commanding positive rate, measuring response
     *          DEMAND_NEG: Commanding negative rate, measuring response
     */
    enum class ATState {
        IDLE,        ///< Not actively demanding rates, collecting baseline data
        DEMAND_POS,  ///< Demanding positive rate (e.g., +30 deg/s roll right)
        DEMAND_NEG   ///< Demanding negative rate (e.g., -30 deg/s roll left)
    };
    ATState state;   ///< Current state machine state

    /**
     * @enum Action
     * @brief Gain adjustment actions taken during tuning
     * 
     * @details Identifies what tuning action was taken this update cycle.
     *          Used for logging and understanding gain evolution during autotune.
     */
    enum class Action {
        NONE,            ///< No gain adjustment this cycle
        LOW_RATE,        ///< Rate response too low, adjusting gains upward
        SHORT,           ///< Response duration too short, adjusting time constant
        RAISE_PD,        ///< Raising both P and D gains together
        LOWER_PD,        ///< Lowering both P and D gains together
        IDLE_LOWER_PD,   ///< Lowering PD during idle state (oscillation detected)
        RAISE_D,         ///< Raising D gain only
        RAISE_P,         ///< Raising P gain only
        LOWER_D,         ///< Lowering D gain only
        LOWER_P          ///< Lowering P gain only
    };
    Action action;   ///< Action taken in current update cycle

    uint32_t state_enter_ms;  ///< Timestamp when current state was entered (milliseconds)

    /**
     * @brief Check if current state should exit based on elapsed time
     * 
     * @param[in] state_time_ms Time spent in current state (milliseconds)
     * 
     * @details Evaluates state exit conditions:
     *          - DEMAND states exit after sufficient response time
     *          - Transitions to opposite demand or back to IDLE
     *          - Updates FF estimate and gain adjustments on exit
     */
    void check_state_exit(uint32_t state_time_ms);
    
    /**
     * @brief Save current gains to EEPROM if changed
     * 
     * @details Writes current gain values to EEPROM only if they differ from
     *          last_save values, reducing EEPROM wear. Called periodically
     *          during successful autotune to persist progress.
     */
    void save_gains(void);

    /**
     * @brief Save float parameter to EEPROM only if changed
     * 
     * @param[in,out] v AP_Float parameter to potentially save
     * @param[in] value New value to save
     * 
     * @details Compares value with v.get(), only calls v.save() if different.
     *          Reduces EEPROM wear by avoiding redundant writes.
     */
    void save_float_if_changed(AP_Float &v, float value);
    
    /**
     * @brief Save int16 parameter to EEPROM only if changed
     * 
     * @param[in,out] v AP_Int16 parameter to potentially save
     * @param[in] value New value to save
     * 
     * @details Compares value with v.get(), only calls v.save() if different.
     *          Reduces EEPROM wear by avoiding redundant writes.
     */
    void save_int16_if_changed(AP_Int16 &v, int16_t value);
    
    /**
     * @brief Transition to new state machine state
     * 
     * @param[in] newstate Target ATState to transition to
     * 
     * @details Updates state variable, records state_enter_ms timestamp,
     *          resets accumulators for new state, may adjust rate demands.
     */
    void state_change(ATState newstate);
    
    /**
     * @brief Get axis name string for this autotune instance
     * 
     * @return Pointer to static string "roll", "pitch", or "yaw"
     * 
     * @note Instance method - returns string for this->type
     */
    const char *axis_string(void) const;

    /**
     * @brief Read current gains from AC_PID controller
     * 
     * @return ATGains structure populated with current FF, P, I, D, IMAX, filter values
     * 
     * @details Queries rpid for current gain values and copies them to ATGains struct.
     *          Used to snapshot gains before tuning and to read intermediate values.
     */
    ATGains get_gains(void);
    
    /**
     * @brief Restore gains from backup (restore member)
     * 
     * @details Writes restore gains back to current and rpid, used when
     *          autotune is aborted or user exits mode before completion.
     *          Original gains are preserved from start() call.
     */
    void restore_gains(void);

    /**
     * @brief Update rmax and tau towards target values
     * 
     * @details Gradually adjusts maximum rate limits and time constant towards
     *          targets determined by autotune_level parameter. Uses slew limiting
     *          to prevent abrupt changes that could destabilize vehicle.
     */
    void update_rmax();

    /**
     * @brief Check if specific autotune option flag is set
     * 
     * @param[in] option Options enum value to test
     * 
     * @return true if option bit is set in autotune_options parameter, false otherwise
     * 
     * @details Inline method to test bitmask flags from aparm.autotune_options.
     *          Used to check DISABLE_FLTD_UPDATE and DISABLE_FLTT_UPDATE flags.
     */
    bool has_option(Options option) {
        return (aparm.autotune_options.get() & uint32_t(1<<uint32_t(option))) != 0;
    }

    // Filters for signal conditioning
    ModeFilterFloat_Size5 ff_filter;  ///< 5-point mode filter for FF estimate (rejects outliers)

    LowPassFilterConstDtFloat actuator_filter;  ///< Low-pass filter for actuator output smoothing
    LowPassFilterConstDtFloat rate_filter;      ///< Low-pass filter for rate measurement smoothing
    LowPassFilterConstDtFloat target_filter;    ///< Low-pass filter for target rate smoothing

    // Slew limiters prevent abrupt gain changes
    float slew_limit_max;  ///< Maximum slew rate for gain changes (units/s)
    float slew_limit_tau;  ///< Time constant for slew limiter (seconds)
    SlewLimiter slew_limiter_P{slew_limit_max, slew_limit_tau};  ///< Slew limiter for P gain
    SlewLimiter slew_limiter_D{slew_limit_max, slew_limit_tau};  ///< Slew limiter for D gain

    // Measurement accumulators for current demand cycle
    float max_actuator;  ///< Maximum actuator output observed in current state (normalized)
    float min_actuator;  ///< Minimum actuator output observed in current state (normalized)
    float max_rate;      ///< Maximum rate observed in current state (deg/s)
    float min_rate;      ///< Minimum rate observed in current state (deg/s)
    float max_target;    ///< Maximum target rate in current state (deg/s)
    float min_target;    ///< Minimum target rate in current state (deg/s)
    
    // Gain adjustment tracking
    float max_P;        ///< Maximum P gain reached during tuning (dimensionless)
    float max_D;        ///< Maximum D gain reached during tuning (dimensionless)
    float min_Dmod;     ///< Minimum D gain modification factor (dimensionless)
    float max_Dmod;     ///< Maximum D gain modification factor (dimensionless)
    float max_SRate_P;  ///< Maximum slew rate for P gain (dimensionless/s)
    float max_SRate_D;  ///< Maximum slew rate for D gain (dimensionless/s)
    
    // Feedforward estimation
    float FF_single;    ///< Single-event FF estimate from last demand cycle (dimensionless)
    uint16_t ff_count;  ///< Number of FF estimates accumulated
    
    // Timing and loop control
    float dt;            ///< Loop update period from AP::scheduler (seconds)
    
    // Convergence tracking
    float D_limit;       ///< Current limit for D gain adjustment (dimensionless)
    float P_limit;       ///< Current limit for P gain adjustment (dimensionless)
    uint32_t D_set_ms;   ///< Timestamp when D gain was last adjusted (milliseconds)
    uint32_t P_set_ms;   ///< Timestamp when P gain was last adjusted (milliseconds)
    uint8_t done_count;  ///< Count of successful convergence cycles (tuning complete when threshold reached)
};
