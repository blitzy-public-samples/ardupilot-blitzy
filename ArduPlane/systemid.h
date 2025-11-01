/**
 * @file systemid.h
 * @brief System identification class header for ArduPlane
 * 
 * @details This file defines the AP_SystemID class which implements system identification
 *          capabilities for ArduPlane vehicles. System identification (SysID) is used to
 *          characterize vehicle dynamics by applying known excitation signals (chirp waveforms)
 *          to control inputs and measuring the vehicle response. This data is logged and can
 *          be analyzed offline to tune control parameters or validate aircraft models.
 * 
 *          The implementation supports multiple excitation modes:
 *          - Attitude control axes (roll, pitch, yaw with input/recover modes)
 *          - Rate control axes (angular rates on roll, pitch, yaw)
 *          - Mixer outputs (direct actuator excitation)
 * 
 *          Primary use cases:
 *          - PID tuning and validation in VTOL/QuadPlane configurations
 *          - Model identification for advanced control design
 *          - System characterization and performance analysis
 * 
 *          System identification is typically enabled only in SITL (Software In The Loop)
 *          simulation environments for safety, but can be enabled on hardware via custom builds.
 * 
 * @note System identification should only be performed by experienced operators in safe
 *       environments as it intentionally excites vehicle dynamics.
 * 
 * @warning This feature is safety-critical - improper configuration can lead to unstable
 *          flight behavior. Always test in simulation first.
 * 
 * Source: ArduPlane/systemid.h
 */

#pragma once

#include "quadplane.h"

#ifndef AP_PLANE_SYSTEMID_ENABLED
// enable via custom build server
#define AP_PLANE_SYSTEMID_ENABLED CONFIG_HAL_BOARD == HAL_BOARD_SITL && HAL_QUADPLANE_ENABLED
#endif

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/chirp.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/vector3.h>
#include <AP_Math/vector2.h>

/**
 * @class AP_SystemID
 * @brief System identification implementation for ArduPlane
 * 
 * @details The AP_SystemID class provides automated system identification capabilities
 *          for ArduPlane vehicles, particularly QuadPlane configurations. It generates
 *          chirp waveforms (sine waves with linearly varying frequency) that excite
 *          specific control axes while logging vehicle response data for offline analysis.
 * 
 *          System identification workflow:
 *          1. Configure parameters (axis, magnitude, frequency range, timing)
 *          2. Start identification via start() method or parameter trigger
 *          3. Chirp waveform is automatically generated and applied to selected axis
 *          4. Vehicle response is logged at high rate for analysis
 *          5. Stop identification via stop() method or automatic completion
 * 
 *          The class supports various excitation modes through the AxisType enum:
 *          - INPUT modes: Excite attitude controller input (angle commands)
 *          - RECOVER modes: Excite attitude controller with recovery behavior
 *          - RATE modes: Excite rate controller directly (angular rate commands)
 *          - MIX modes: Excite mixer outputs directly (actuator-level excitation)
 * 
 *          Parameters are managed through the AP_Param system and include:
 *          - axis: Which axis/mode to excite
 *          - waveform_magnitude: Amplitude of excitation signal
 *          - frequency_start/stop: Chirp frequency sweep range
 *          - time_fade_in/record/fade_out: Timing profile
 * 
 *          Integration with ArduPlane:
 *          - Offsets are applied via get_attitude_offset_deg() and get_throttle_offset()
 *          - Vehicle control loops add these offsets to normal control outputs
 *          - update() is called from main vehicle scheduler loop
 * 
 * @note Requires AP_PLANE_SYSTEMID_ENABLED to be defined (default: SITL + QuadPlane only)
 * 
 * @warning System identification intentionally perturbs vehicle control. Only use in
 *          safe environments with adequate altitude/space for recovery.
 * 
 * Source: ArduPlane/systemid.h
 */
class AP_SystemID {

public:
    /**
     * @brief Constructor for AP_SystemID
     * 
     * @details Initializes system identification object with default state.
     *          Parameters are initialized through the AP_Param system.
     *          Initial state is not running (running = false).
     */
    AP_SystemID(void);
    
    /**
     * @brief Start system identification sequence
     * 
     * @details Initiates the system identification test sequence with currently configured
     *          parameters. This method:
     *          - Sets running flag to true
     *          - Initializes chirp waveform generator with configured parameters
     *          - Resets timing references (waveform_time = 0)
     *          - Stores initial controller state for restoration on stop
     *          - Begins logging high-rate data
     * 
     *          The excitation waveform progresses through phases:
     *          1. Fade-in period (time_fade_in): Gradually ramp up amplitude
     *          2. Constant frequency dwell: Brief period at start frequency
     *          3. Chirp sweep (time_record): Linear frequency sweep from start to stop
     *          4. Fade-out period (time_fade_out): Gradually ramp down to zero
     * 
     * @note Can be triggered via parameter change or external command
     * @note Should only be called when vehicle is in stable flight condition
     * 
     * @warning Starting system identification in inappropriate flight modes or conditions
     *          can lead to loss of control. Ensure adequate altitude and safe environment.
     * 
     * @see stop(), update(), AxisType
     */
    void start(void);
    
    /**
     * @brief Stop system identification sequence
     * 
     * @details Terminates the currently running system identification test and restores
     *          normal flight control behavior. This method:
     *          - Sets running flag to false
     *          - Clears all control offsets (attitude_offset_deg and throttle_offset to zero)
     *          - Restores original controller configuration (e.g., feedforward settings)
     *          - Stops high-rate data logging
     * 
     *          Stop can occur through:
     *          - Manual stop command
     *          - Automatic completion when waveform sequence finishes
     *          - Safety abort if vehicle enters unsafe conditions
     * 
     * @note Vehicle returns to normal control immediately after stop
     * @note Logged data should be extracted for offline analysis
     * 
     * @see start(), is_running()
     */
    void stop(void);
    
    /**
     * @brief Update system identification state and generate excitation signals
     * 
     * @details Called from the main vehicle scheduler loop (typically 400Hz for QuadPlane).
     *          This method:
     *          - Updates waveform time based on elapsed time since last call
     *          - Generates current chirp waveform sample based on configured parameters
     *          - Calculates instantaneous frequency of chirp signal
     *          - Applies waveform to appropriate control axis based on AxisType setting
     *          - Updates attitude_offset_deg or throttle_offset for vehicle control loops
     *          - Logs waveform parameters and vehicle response data
     *          - Checks for automatic stop conditions (waveform complete)
     * 
     *          Timing management:
     *          - Tracks waveform_time from start
     *          - Handles fade-in, constant frequency, chirp sweep, and fade-out phases
     *          - Maintains phase continuity for smooth transitions
     * 
     *          Axis-specific behavior:
     *          - INPUT_*/RECOVER_*/RATE_*: Apply to attitude_offset_deg (roll/pitch/yaw)
     *          - MIX_*: Apply to mixer-level offsets
     *          - MIX_THROTTLE: Apply to throttle_offset
     * 
     * @note This function should be called at consistent loop rate for accurate timing
     * @note Only generates non-zero offsets when running == true
     * 
     * @warning Function modifies control offsets - vehicle control loops must properly
     *          integrate these offsets to maintain safe flight.
     * 
     * @see start(), stop(), get_attitude_offset_deg(), get_throttle_offset()
     * 
     * Source: ArduPlane/systemid.cpp (implementation)
     */
    void update();

    /**
     * @brief Parameter table for AP_Param system
     * 
     * @details Defines all configurable parameters for system identification through the
     *          ground control station parameter interface. Parameters include:
     *          - SID_AXIS: Axis/mode selection (AxisType enum)
     *          - SID_MAGNITUDE: Waveform amplitude (units depend on axis type)
     *          - SID_F_START_HZ: Starting frequency of chirp sweep (Hz)
     *          - SID_F_STOP_HZ: Ending frequency of chirp sweep (Hz)
     *          - SID_T_FADE_IN: Fade-in time (seconds)
     *          - SID_T_REC: Recording/sweep duration (seconds)
     *          - SID_T_FADE_OUT: Fade-out time (seconds)
     * 
     * @note Parameter names use SID_ prefix for System IDentification
     * @note Changing parameters while test is running may cause discontinuities
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get current attitude control offset from system identification
     * 
     * @details Returns the current attitude offset (in degrees) being applied by the
     *          system identification routine. This offset is added to normal attitude
     *          commands in the vehicle control loops to inject the excitation signal.
     * 
     *          Vector components (NED body frame):
     *          - x: Roll offset (degrees)
     *          - y: Pitch offset (degrees)
     *          - z: Yaw offset (degrees)
     * 
     * @return Reference to current attitude offset vector (degrees)
     * 
     * @note Returns zero vector when not running or for non-attitude axes
     * @note Vehicle attitude controller adds this offset to pilot/autopilot commands
     * 
     * @see get_throttle_offset(), update()
     */
    const Vector3f &get_attitude_offset_deg(void) const {
        return attitude_offset_deg;
    }
    
    /**
     * @brief Get current throttle offset from system identification
     * 
     * @details Returns the current throttle offset being applied by system identification
     *          for MIX_THROTTLE axis excitation. Offset is in normalized throttle units
     *          (typically -1.0 to 1.0 range depending on mixer configuration).
     * 
     * @return Throttle offset value, or 0.0 if not running
     * 
     * @note Only non-zero when running and axis == MIX_THROTTLE
     * @note Vehicle mixer adds this offset to commanded throttle output
     * 
     * @see get_attitude_offset_deg(), AxisType::MIX_THROTTLE
     */
    float get_throttle_offset(void) const {
        return running ? throttle_offset : 0.0;
    }

    /**
     * @brief Check if system identification is currently active
     * 
     * @details Returns true if a system identification test sequence is currently running.
     *          This flag is set by start() and cleared by stop().
     * 
     * @return true if system identification is active, false otherwise
     * 
     * @note Used by vehicle control loops to determine if offsets should be applied
     * @note Can be checked before starting new test to avoid conflicts
     * 
     * @see start(), stop()
     */
    bool is_running() const {
        return running;
    }

private:
    // Chirp waveform generator used to create frequency-swept excitation signals
    Chirp chirp_input;
    
    // Flag indicating if system identification test is currently active
    bool running;

    /**
     * @enum AxisType
     * @brief Defines which control axis or mode is excited during system identification
     * 
     * @details System identification can excite different levels of the control hierarchy
     *          to characterize different aspects of vehicle dynamics:
     * 
     *          INPUT modes (attitude controller input level):
     *          - Excite desired attitude angle inputs to attitude controller
     *          - Tests complete attitude control loop response
     *          - Useful for tuning attitude PID gains
     * 
     *          RECOVER modes (attitude controller with recovery):
     *          - Similar to INPUT modes but with different controller behavior
     *          - Tests attitude controller recovery characteristics
     * 
     *          RATE modes (rate controller level):
     *          - Excite desired angular rate inputs to rate controller
     *          - Tests rate control loop response directly
     *          - Useful for inner loop tuning and actuator characterization
     * 
     *          MIX modes (mixer output level):
     *          - Excite mixer outputs directly (actuator-level commands)
     *          - Bypasses control loops to characterize vehicle dynamics alone
     *          - Most direct characterization of airframe response
     * 
     * @note Selection affects which control path is excited and what dynamics are measured
     * @note Different modes require different magnitude scaling
     */
    enum class AxisType {
        NONE = 0,               ///< No excitation - system identification disabled
        
        INPUT_ROLL = 1,         ///< Excite roll angle input to attitude controller (degrees)
        INPUT_PITCH = 2,        ///< Excite pitch angle input to attitude controller (degrees)
        INPUT_YAW = 3,          ///< Excite yaw angle input to attitude controller (degrees)
        
        RECOVER_ROLL = 4,       ///< Excite roll axis with recovery mode behavior (degrees)
        RECOVER_PITCH = 5,      ///< Excite pitch axis with recovery mode behavior (degrees)
        RECOVER_YAW = 6,        ///< Excite yaw axis with recovery mode behavior (degrees)
        
        RATE_ROLL = 7,          ///< Excite roll rate input to rate controller (deg/s)
        RATE_PITCH = 8,         ///< Excite pitch rate input to rate controller (deg/s)
        RATE_YAW = 9,           ///< Excite yaw rate input to rate controller (deg/s)
        
        MIX_ROLL = 10,          ///< Excite roll mixer output directly (normalized -1 to 1)
        MIX_PITCH = 11,         ///< Excite pitch mixer output directly (normalized -1 to 1)
        MIX_YAW = 12,           ///< Excite yaw mixer output directly (normalized -1 to 1)
        MIX_THROTTLE = 13,      ///< Excite throttle mixer output directly (normalized)
    };

    /**
     * @brief Set body-frame feedforward flag in attitude controller
     * 
     * @details Temporarily modifies attitude controller feedforward setting during system
     *          identification to ensure consistent response characteristics. Original setting
     *          is restored when test completes.
     * 
     * @param[in] value Feedforward enable/disable state
     * 
     * @note Original state is saved in restore.att_bf_feedforward
     */
    void set_bf_feedforward(bool value);
    
    /**
     * @brief Log system identification data for offline analysis
     * 
     * @details Writes high-rate log messages containing:
     *          - Current waveform parameters (frequency, magnitude, phase)
     *          - Applied control offsets
     *          - Vehicle state (attitude, rates, position)
     *          - Timestamp for synchronization
     * 
     *          Log data is subsampled according to log_subsample parameter to manage
     *          data rate while maintaining sufficient resolution for analysis.
     * 
     * @note Called from update() at main loop rate
     * @note Logged data is post-processed to extract frequency response
     */
    void log_data() const;
    
    // Subsample factor for data logging (logs every Nth sample to reduce data rate)
    int8_t log_subsample;


    // --- Configuration Parameters (AP_Param managed) ---
    
    /**
     * Axis selection parameter - determines which control axis/mode is excited
     * Parameter name: SID_AXIS
     * Set to non-zero to enable and display other system identification parameters
     */
    AP_Enum<AxisType> axis;
    
    /**
     * Waveform magnitude parameter - amplitude of excitation signal
     * Parameter name: SID_MAGNITUDE
     * Units depend on AxisType:
     * - INPUT/RECOVER modes: degrees (typical range 5-20 deg)
     * - RATE modes: deg/s (typical range 50-200 deg/s)
     * - MIX modes: normalized units (typical range 0.1-0.5)
     */
    AP_Float waveform_magnitude;
    
    /**
     * Start frequency parameter - initial frequency of chirp sweep
     * Parameter name: SID_F_START_HZ
     * Units: Hz
     * Typical range: 0.1 - 2.0 Hz (low frequency for large aircraft dynamics)
     */
    AP_Float frequency_start;
    
    /**
     * Stop frequency parameter - final frequency of chirp sweep
     * Parameter name: SID_F_STOP_HZ
     * Units: Hz
     * Typical range: 2.0 - 30.0 Hz (captures actuator and structural modes)
     * Must be greater than frequency_start for forward sweep
     */
    AP_Float frequency_stop;
    
    /**
     * Fade-in time parameter - duration of amplitude ramp-up
     * Parameter name: SID_T_FADE_IN
     * Units: seconds
     * Gradual increase prevents abrupt control transients at test start
     * Typical range: 1-5 seconds
     */
    AP_Float time_fade_in;
    
    /**
     * Recording time parameter - duration of chirp frequency sweep
     * Parameter name: SID_T_REC
     * Units: seconds
     * Longer duration provides finer frequency resolution
     * Typical range: 10-60 seconds depending on frequency range
     */
    AP_Float time_record;
    
    /**
     * Fade-out time parameter - duration of amplitude ramp-down
     * Parameter name: SID_T_FADE_OUT
     * Units: seconds
     * Gradual decrease returns vehicle to normal control smoothly
     * Typical range: 1-5 seconds
     */
    AP_Float time_fade_out;
    
    /**
     * XY control multiplier for VTOL position control modes
     * Parameter name: SID_XY_MUL
     * Scales horizontal control authority during identification
     * Used to maintain position control while exciting attitude
     */
    AP_Float xy_control_mul;

    /**
     * @brief Saved controller settings for restoration after test
     * 
     * @details Stores original controller configuration that may be modified during
     *          system identification. Values are restored by stop() to return vehicle
     *          to normal operation.
     */
    struct {
        bool att_bf_feedforward;    ///< Original attitude_control body-frame feedforward setting
    } restore;

    // --- Waveform Generation State ---
    
    /**
     * Time reference for waveform generation (seconds since test start)
     * Incremented each update() call based on elapsed time
     * Used to determine current phase (fade-in, chirp, fade-out)
     */
    float waveform_time;
    
    /**
     * Current waveform sample value
     * Computed by chirp generator based on current time and frequency
     * Range: [-waveform_magnitude, +waveform_magnitude] during active sweep
     */
    float waveform_sample;
    
    /**
     * Instantaneous waveform frequency (radians per second)
     * Changes linearly during chirp sweep from frequency_start to frequency_stop
     * Used for phase-continuous waveform generation
     */
    float waveform_freq_rads;
    
    /**
     * Duration of constant frequency dwell period before chirp begins (seconds)
     * Allows initial transients to settle before frequency sweep starts
     */
    float time_const_freq;
    
    /**
     * Timestamp of last update() call (milliseconds)
     * Used to calculate elapsed time between updates for accurate waveform timing
     */
    uint32_t last_loop_time_ms;

    // --- Position Control State (for VTOL modes) ---
    
    /**
     * Target velocity vector for position controller (m/s, NED horizontal plane)
     * Used when exciting attitude while maintaining position control
     */
    Vector2f target_vel;
    
    /**
     * Target position for position hold (meters, NED horizontal plane)
     * Reference position maintained during attitude excitation
     */
    Vector2f target_pos;
    
    /**
     * Previous cycle input velocity (m/s, NED horizontal plane)
     * Used for velocity derivative calculation and filtering
     */
    Vector2f input_vel_last;

    // --- Control Output Offsets ---
    
    /**
     * Current attitude offset applied to vehicle control (degrees, NED body frame)
     * Vector components: [roll, pitch, yaw]
     * Added to normal attitude commands by vehicle control loops
     * Non-zero only when running and appropriate AxisType selected
     */
    Vector3f attitude_offset_deg;
    
    /**
     * Current throttle offset applied to mixer output (normalized)
     * Added to commanded throttle when AxisType == MIX_THROTTLE
     * Zero unless running with throttle excitation
     */
    float throttle_offset;

    /**
     * Axis type that was active when test started
     * Used to track test configuration and for proper cleanup on stop
     */
    AxisType start_axis;

};

#endif // AP_PLANE_SYSTEMID_ENABLED
