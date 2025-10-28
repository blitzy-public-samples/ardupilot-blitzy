/**
 * @file AP_OpticalFlow_Calibrator.h
 * @brief Automatic optical flow sensor calibration algorithm
 * 
 * @details This file implements automatic calibration for optical flow sensors
 * to estimate per-axis scaling factors that correct sensor calibration errors.
 * 
 * The calibration algorithm uses least-squares fitting to compare measured flow
 * rates against predicted line-of-sight (LOS) rates computed from vehicle attitude
 * and velocity. This produces optimal scaling factors for the FLOW_FXSCALER and
 * FLOW_FYSCALER parameters.
 * 
 * Algorithm Overview:
 * 1. Collect AP_OPTICALFLOW_CAL_MAX_SAMPLES (50) samples during vehicle motion
 * 2. For each axis, compare measured flow rates to predicted LOS rates
 * 3. Perform least-squares fitting to find optimal scaling factors
 * 4. Validate scaling factors are within bounds [0.20, 4.0]
 * 5. Implement 120-second timeout for calibration completion
 * 6. Log calibration data (OFCA) when HAL_LOGGING_ENABLED
 * 
 * @note Calibration requires vehicle motion with sufficient optical flow and altitude variation
 * @note Scalars are applied to FLOW_FXSCALER and FLOW_FYSCALER parameters
 * 
 * @warning Requires good GPS and altitude reference during calibration for accurate los_pred
 * @warning Poor surface texture or lighting may produce unreliable calibration results
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger_config.h>

/**
 * @brief Maximum number of samples collected per axis before calibration begins
 * 
 * @details Defines the buffer size for calibration data collection. The calibrator
 * collects this many samples for each axis (X and Y) before performing the
 * least-squares fitting algorithm. Larger values provide more robust calibration
 * but require more memory and longer data collection time.
 * 
 * Memory allocation: 2 axes × 50 samples × sizeof(sample_t) ≈ 1200 bytes
 */
#define AP_OPTICALFLOW_CAL_MAX_SAMPLES 50  // number of samples required before calibration begins

/**
 * @class AP_OpticalFlow_Calibrator
 * @brief Automatic optical flow sensor calibration via least-squares estimation
 * 
 * @details This class implements an automatic calibration system for optical flow sensors
 * that estimates per-axis scaling factors to correct for sensor calibration errors.
 * 
 * The calibration process:
 * 1. Collects AP_OPTICALFLOW_CAL_MAX_SAMPLES (50) samples during vehicle motion
 * 2. Compares measured flow rates to predicted line-of-sight (LOS) rates computed
 *    from vehicle attitude and velocity
 * 3. Performs per-axis least-squares fitting to find optimal scaling factors
 * 4. Validates scaling factors are within bounds [0.20, 4.0]
 * 5. Implements 120-second timeout to prevent indefinite calibration
 * 6. Logs calibration data (OFCA) when HAL_LOGGING_ENABLED
 * 
 * State Machine:
 * - NOT_STARTED: Initial state, no calibration active
 * - RUNNING: Actively collecting samples during vehicle motion
 * - READY_TO_CALIBRATE: Buffer full, ready to compute (may wait until disarmed)
 * - SUCCESS: Valid scaling factors computed and available
 * - FAILED: Calibration failed (timeout, bad data, or out-of-bounds scalars)
 * 
 * Units:
 * - Flow rates: rad/s
 * - Body rates: rad/s
 * - Scalars: Dimensionless multipliers (typically 0.8-1.2 for good calibration)
 * - Timestamps: milliseconds (uint32_t)
 * 
 * @note Requires vehicle motion with sufficient optical flow and altitude variation
 * @note 120-second timeout prevents indefinite calibration if insufficient motion
 * 
 * @warning Scalar bounds [0.20, 4.0] prevent unrealistic corrections
 */
class AP_OpticalFlow_Calibrator {
public:
    /**
     * @brief Initialize calibrator state to NOT_STARTED
     * 
     * @details Constructs the calibrator with initial state set to NOT_STARTED.
     * All member variables are zero-initialized by default.
     */
    AP_OpticalFlow_Calibrator() {};

    /**
     * @brief Begin calibration sample collection
     * 
     * @details Initiates the calibration process by:
     * - Resetting sample buffers to empty
     * - Setting state to RUNNING
     * - Recording start timestamp for timeout tracking
     * - Clearing any previous calibration results
     * 
     * The calibrator will begin collecting samples when update() is called
     * with valid flow data during vehicle motion.
     * 
     * @note Vehicle must be in motion with good GPS/altitude reference
     * @note Call update() regularly to collect samples
     * 
     * @see stop(), update()
     */
    void start();
    
    /**
     * @brief Halt calibration process
     * 
     * @details Stops the calibration by:
     * - Transitioning to NOT_STARTED state (unless already in SUCCESS state)
     * - Preserving SUCCESS state and calibration results if present
     * - Clearing RUNNING or READY_TO_CALIBRATE states
     * 
     * If calibration has already succeeded, calling stop() preserves the
     * calibration results while preventing further sample collection.
     * 
     * @see start(), update()
     */
    void stop();

    /**
     * @brief State machine update and calibration computation
     * 
     * @details Updates the calibration state machine. Behavior depends on current state:
     * - RUNNING: Collects samples if buffers not full, transitions to READY_TO_CALIBRATE
     *   when sample buffers reach AP_OPTICALFLOW_CAL_MAX_SAMPLES (50) samples per axis
     * - READY_TO_CALIBRATE: Calls run_calibration() to compute scaling factors
     * - Checks for 120-second timeout in all active states
     * 
     * When calibration completes successfully, state transitions to SUCCESS and this
     * method returns true. The computed scaling factors can then be retrieved via
     * get_scalars().
     * 
     * Timeout handling:
     * - 120-second timeout starts when start() is called
     * - If timeout expires before SUCCESS, state transitions to FAILED
     * - Timeout prevents indefinite calibration with insufficient vehicle motion
     * 
     * @return true if new scaling factors were calculated (state transitioned to SUCCESS)
     *         false otherwise (still collecting data, failed, or not started)
     * 
     * @note Call this method regularly (e.g., at main loop rate) during calibration
     * @note Requires valid flow data to be provided via add_sample()
     * 
     * @see start(), stop(), get_scalars(), run_calibration()
     */
    bool update();

    /**
     * @brief Retrieve calibrated X and Y axis scaling factors
     * 
     * @details Returns the computed scaling factors for both axes after successful
     * calibration. These values should be multiplied by the existing FLOW_FXSCALER
     * and FLOW_FYSCALER parameter values to obtain corrected scaling.
     * 
     * Scaling factor interpretation:
     * - 1.0 = No correction needed (sensor is perfectly calibrated)
     * - < 1.0 = Sensor over-reports flow, scale down
     * - > 1.0 = Sensor under-reports flow, scale up
     * - Typical range: 0.8 to 1.2 for reasonably calibrated sensors
     * - Valid range: 0.20 to 4.0 (enforced by calibration algorithm)
     * 
     * @return Vector2f with X scaling factor in .x and Y scaling factor in .y
     *         Returns (1.0, 1.0) if calibration not yet successful
     * 
     * @note Only valid when update() has returned true (SUCCESS state)
     * @note Apply scalars by multiplying: new_param = old_param × scalar
     * 
     * @see update(), run_calibration()
     */
    Vector2f get_scalars();

private:

    /**
     * @struct sample_t
     * @brief Single-axis calibration data point
     * 
     * @details Stores one sample of optical flow data for calibration.
     * Each sample contains the measured flow rate, compensating body rate,
     * and predicted line-of-sight rate for comparison.
     * 
     * The calibration algorithm uses these three values to compute optimal
     * scaling via least-squares fitting: minimize Σ(flow_rate - los_pred×scalar - body_rate)²
     */
    struct sample_t {
        float flow_rate;    ///< Measured optical flow rate (rad/s)
        float body_rate;    ///< Gyro body rate for compensation (rad/s)
        float los_pred;     ///< Predicted line-of-sight rate from velocity/altitude (rad/s)
    };

    /**
     * @brief Store flow sample in calibration buffer
     * 
     * @details Adds a new sample to the per-axis sample buffers if space is available
     * and the sample is not a duplicate. Samples are stored separately for X and Y axes.
     * 
     * Duplicate detection:
     * - Compares timestamp_ms with _last_sample_timestamp_ms
     * - Ignores samples with identical timestamps to prevent double-counting
     * 
     * Buffer management:
     * - Maximum AP_OPTICALFLOW_CAL_MAX_SAMPLES (50) samples per axis
     * - Stops accepting samples when buffer is full
     * - num_samples counter tracks current buffer fill level
     * 
     * @param[in] timestamp_ms Sample timestamp in milliseconds for duplicate detection
     * @param[in] flow_rate Measured optical flow rates for X and Y axes (rad/s)
     * @param[in] body_rate Gyro body rates for X and Y axes (rad/s)
     * @param[in] los_pred Predicted line-of-sight rates for X and Y axes (rad/s)
     * 
     * @note Only stores samples when state is RUNNING
     * @note Silently ignores samples when buffers are full
     * 
     * @see sample_buffers_full(), update()
     */
    void add_sample(uint32_t timestamp_ms, const Vector2f& flow_rate, const Vector2f& body_rate, const Vector2f& los_pred);

    /**
     * @brief Check if both axis sample buffers are full
     * 
     * @details Determines whether both X and Y axis buffers have collected
     * AP_OPTICALFLOW_CAL_MAX_SAMPLES (50) samples and are ready for calibration.
     * 
     * @return true if both axes have exactly AP_OPTICALFLOW_CAL_MAX_SAMPLES samples
     *         false if either axis still needs more samples
     * 
     * @note This condition triggers transition from RUNNING to READY_TO_CALIBRATE state
     * 
     * @see add_sample(), update()
     */
    bool sample_buffers_full() const;

    /**
     * @brief Execute least-squares calibration algorithm for both axes
     * 
     * @details Performs the complete calibration computation:
     * 1. Calls calc_scalars() for X axis (axis=0) and Y axis (axis=1)
     * 2. Validates computed scalars are within bounds [0.20, 4.0]
     * 3. Updates best_scalar and best_scalar_fitness for each axis
     * 4. Transitions state to SUCCESS if valid scalars found, FAILED otherwise
     * 
     * Scalar validation:
     * - Minimum allowed: 0.20 (sensor over-reports by 5×)
     * - Maximum allowed: 4.0 (sensor under-reports by 4×)
     * - Out-of-bounds results indicate unreliable calibration data
     * 
     * Success criteria:
     * - Both axes produce scalars within [0.20, 4.0]
     * - Fitness values indicate good fit (low RMS residuals)
     * - Sufficient sample quality and vehicle motion
     * 
     * @return true if calibration succeeded and valid scalars computed
     *         false if calibration failed (bad data, out-of-bounds, or computation error)
     * 
     * @note Updates _cal_data[0].best_scalar, _cal_data[0].best_scalar_fitness
     * @note Updates _cal_data[1].best_scalar, _cal_data[1].best_scalar_fitness
     * @note Logs calibration results when HAL_LOGGING_ENABLED
     * 
     * @see calc_scalars(), update()
     */
    bool run_calibration();

    /**
     * @brief Per-axis least-squares fitting algorithm
     * 
     * @details Computes optimal scaling factor for a single axis (X or Y) using
     * iterative optimization that minimizes mean squared residuals across all samples.
     * 
     * Algorithm steps:
     * 1. For each sample, compute single-sample optimal scalar via calc_sample_best_scalar()
     * 2. Evaluate fitness of each candidate scalar via calc_mean_squared_residuals()
     * 3. Select scalar with lowest fitness (minimum RMS error)
     * 4. Return best scalar and its fitness metric
     * 
     * Mathematical formulation:
     * - Minimize: Σ(flow_rate - los_pred×scalar - body_rate)² across all samples
     * - Per-sample optimal: scalar = (flow_rate - body_rate) / los_pred
     * - Aggregate fitness: sqrt(mean(residuals²))
     * 
     * @param[in]  axis Axis to calibrate: 0=X axis, 1=Y axis
     * @param[out] scalar Best-fit scaling factor for the specified axis
     * @param[out] fitness RMS error of best-fit scalar (lower is better)
     * 
     * @return true if valid scalar found with finite fitness
     *         false if computation failed (divide by zero, invalid samples)
     * 
     * @note Uses all AP_OPTICALFLOW_CAL_MAX_SAMPLES samples from _cal_data[axis]
     * @note Fitness units: rad/s (RMS residual)
     * 
     * @see calc_sample_best_scalar(), calc_mean_squared_residuals(), run_calibration()
     */
    bool calc_scalars(uint8_t axis, float& scalar, float& fitness);

    /**
     * @brief Compute single sample error for given scalar
     * 
     * @details Calculates the squared residual error for one sample using the
     * provided scaling factor. This measures how well the scalar predicts the
     * observed flow rate.
     * 
     * Residual formula:
     * residual² = (flow_rate - los_pred×scalar - body_rate)²
     * 
     * Where:
     * - flow_rate: Measured optical flow (rad/s)
     * - los_pred: Predicted flow from velocity/altitude (rad/s)
     * - scalar: Candidate scaling factor
     * - body_rate: Gyro compensation (rad/s)
     * 
     * @param[in] sample Single-axis calibration data point
     * @param[in] scalar Candidate scaling factor to evaluate
     * 
     * @return Squared residual error in (rad/s)²
     * 
     * @note Lower return values indicate better scalar fit for this sample
     * 
     * @see calc_mean_squared_residuals(), calc_scalars()
     */
    float calc_sample_residual(const sample_t& sample, float scalar) const;

    /**
     * @brief Compute analytical optimal scalar for single sample
     * 
     * @details Calculates the scaling factor that minimizes residual for one sample
     * using analytical solution to the least-squares problem.
     * 
     * Optimal scalar derivation:
     * - Minimize: (flow_rate - los_pred×scalar - body_rate)²
     * - Derivative: d/d(scalar) = -2×los_pred×(flow_rate - los_pred×scalar - body_rate)
     * - Set to zero and solve: scalar = (flow_rate - body_rate) / los_pred
     * 
     * Validity checks:
     * - Returns false if los_pred ≈ 0 (divide by zero)
     * - Returns false if result is NaN or infinite
     * 
     * @param[in]  sample Single-axis calibration data point
     * @param[out] best_scalar Optimal scaling factor for this sample
     * 
     * @return true if valid scalar computed (los_pred sufficiently non-zero)
     *         false if computation failed (denominator too small or result invalid)
     * 
     * @note This gives per-sample optimal, not global optimal across all samples
     * 
     * @see calc_scalars(), calc_sample_residual()
     */
    bool calc_sample_best_scalar(const sample_t& sample, float& best_scalar) const;

    /**
     * @brief Aggregate fitness metric across all samples
     * 
     * @details Computes mean squared residual for all samples of one axis given
     * a candidate scaling factor. This is the objective function minimized by
     * the calibration algorithm.
     * 
     * Computation:
     * 1. For each sample in _cal_data[axis].samples[0..num_samples-1]:
     *    - Compute residual² via calc_sample_residual(sample, scalar)
     * 2. Sum all residuals²
     * 3. Return mean: sum / num_samples
     * 
     * Fitness interpretation:
     * - Units: (rad/s)² 
     * - Lower values indicate better scalar fit
     * - Zero means perfect fit (unlikely with real data)
     * - Very high values indicate poor scalar or bad data
     * 
     * @param[in] axis Axis to evaluate: 0=X axis, 1=Y axis
     * @param[in] scalar Candidate scaling factor to evaluate
     * 
     * @return Mean squared residual in (rad/s)²
     * 
     * @note Uses num_samples from _cal_data[axis] (up to AP_OPTICALFLOW_CAL_MAX_SAMPLES)
     * 
     * @see calc_sample_residual(), calc_scalars()
     */
    float calc_mean_squared_residuals(uint8_t axis, float scalar) const;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write calibration sample to dataflash log
     * 
     * @details Logs a single calibration sample to the OFCA (Optical Flow Calibration)
     * dataflash message for post-flight analysis and debugging.
     * 
     * OFCA log message contents:
     * - Axis identifier (0=X, 1=Y)
     * - Sample number (0 to AP_OPTICALFLOW_CAL_MAX_SAMPLES-1)
     * - Measured flow rate (rad/s)
     * - Gyro body rate compensation (rad/s)
     * - Predicted line-of-sight rate (rad/s)
     * 
     * Log analysis usage:
     * - Verify sample quality and range
     * - Check for correlation between flow_rate and los_pred
     * - Identify problematic samples (outliers, noise)
     * - Validate gyro compensation effectiveness
     * 
     * @param[in] axis Axis identifier: 0=X axis, 1=Y axis
     * @param[in] sample_num Sample index in buffer (0 to 49)
     * @param[in] flow_rate Measured optical flow rate (rad/s)
     * @param[in] body_rate Gyro body rate for compensation (rad/s)
     * @param[in] los_pred Predicted line-of-sight rate (rad/s)
     * 
     * @note Only available when HAL_LOGGING_ENABLED is defined
     * @note Called during run_calibration() for each collected sample
     * 
     * @see run_calibration()
     */
    void log_sample(uint8_t axis, uint8_t sample_num, float flow_rate, float body_rate, float los_pred);
#endif

    /**
     * @enum CalState
     * @brief Calibration state machine states
     * 
     * @details Defines the possible states of the calibration process.
     * State transitions are managed by update() and controlled by start()/stop().
     */
    enum class CalState {
        NOT_STARTED = 0,    ///< Initial state, no calibration active
        RUNNING,            ///< Actively collecting samples during vehicle motion
        READY_TO_CALIBRATE, ///< Sample buffer full, ready to compute (may wait until disarmed)
        SUCCESS,            ///< Valid scaling factors computed and available
        FAILED              ///< Calibration failed (timeout, bad data, or out-of-bounds scalars)
    } _cal_state;           ///< Current calibration state

    // Private member variables
    
    /**
     * @brief Calibration start timestamp for timeout tracking
     * 
     * @details System time in milliseconds when start() was called.
     * Used to implement 120-second calibration timeout. If calibration
     * does not complete within 120 seconds, state transitions to FAILED.
     */
    uint32_t _start_time_ms;
    
    /**
     * @struct Per-axis calibration data
     * @brief Sample buffer and results for one axis (X or Y)
     */
    struct {
        sample_t samples[AP_OPTICALFLOW_CAL_MAX_SAMPLES];   ///< Buffer of calibration samples (50 samples max)
        uint8_t num_samples;                                ///< Current number of samples in buffer (0 to 50)
        float best_scalar;                                  ///< Computed optimal scaling factor (1.0 = no correction)
        float best_scalar_fitness;                          ///< RMS residual error for best_scalar (rad/s), lower is better
    } _cal_data[2];                                         ///< Per-axis data: [0]=X axis, [1]=Y axis
    
    /**
     * @brief Timestamp of most recent sample
     * 
     * @details System time in milliseconds of the last sample added via add_sample().
     * Used to filter duplicate samples with identical timestamps and prevent
     * double-counting of the same data.
     */
    uint32_t _last_sample_timestamp_ms;
    
    /**
     * @brief Last status report timestamp
     * 
     * @details System time in milliseconds when the last periodic status message
     * was sent. Used to throttle status reporting and prevent excessive log messages
     * during calibration.
     */
    uint32_t _last_report_ms;
};
