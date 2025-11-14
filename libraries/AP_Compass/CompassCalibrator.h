/**
 * @file CompassCalibrator.h
 * @brief Compass calibration algorithms using sphere and ellipsoid fitting
 * 
 * @details This file implements comprehensive compass calibration using Levenberg-Marquardt
 *          optimization for both sphere fitting (hard-iron correction) and ellipsoid fitting
 *          (soft-iron correction). The calibration process compensates for:
 *          - Hard-iron effects: Constant magnetic offsets from vehicle magnetization
 *          - Soft-iron effects: Field distortions from ferromagnetic materials (diagonals and off-diagonals)
 *          - Sensor scaling errors: Radius compensation for field strength variations
 * 
 *          The calibrator uses a two-step process:
 *          1. STEP ONE: Sphere fit to calculate offsets and radius assuming spherical field distribution
 *          2. STEP TWO: Ellipsoid fit to refine with diagonal and off-diagonal corrections for soft-iron
 * 
 *          Memory optimization: Compressed storage for attitude (int8_t roll/pitch/yaw) and samples
 *          (int16_t x/y/z) reduces memory footprint while maintaining adequate precision.
 * 
 *          Sample distribution validation: Geodesic coverage mask tracks sample collection across
 *          different orientations to ensure adequate angular coverage for robust calibration.
 * 
 * @note All magnetic field measurements are in milligauss (mGauss)
 * @note Requires COMPASS_CAL_NUM_SAMPLES (300) samples before fitting begins
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Compass_config.h"

#if COMPASS_CAL_ENABLED

#include <AP_Math/AP_Math.h>

#define COMPASS_CAL_NUM_SPHERE_PARAMS       4      ///< Sphere fit parameters: radius + 3 offsets
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS    9      ///< Ellipsoid fit parameters: 3 offsets + 3 diagonals + 3 off-diagonals
#define COMPASS_CAL_NUM_SAMPLES             300    ///< Minimum samples required before fitting begins

/**
 * @class CompassCalibrator
 * @brief Compass calibration engine using Levenberg-Marquardt sphere and ellipsoid fitting
 * 
 * @details This class performs comprehensive compass calibration to compensate for hard-iron
 *          and soft-iron magnetic interference. The calibration uses a two-step process:
 * 
 *          **Algorithm Overview:**
 *          1. **Sample Collection**: Accumulates compass readings with associated attitude data
 *             while the vehicle is rotated through various orientations (300 samples minimum)
 *          2. **Sphere Fit (STEP ONE)**: Levenberg-Marquardt optimization solves for:
 *             - Field radius (expected magnetic field strength in mGauss)
 *             - 3D offset vector (hard-iron correction in mGauss)
 *             Assumes ideal spherical field distribution around the corrected center
 *          3. **Ellipsoid Fit (STEP TWO)**: Extended Levenberg-Marquardt optimization solves for:
 *             - 3D offset vector (refined)
 *             - 3D diagonal scaling (primary soft-iron axes)
 *             - 3D off-diagonal scaling (cross-axis soft-iron coupling)
 *             Models field as ellipsoid to correct for ferromagnetic distortions
 * 
 *          **Mathematical Foundation:**
 *          - Sphere fit: Minimize Σ(||sample - offset|| - radius)² across all samples
 *          - Ellipsoid fit: Minimize Σ(||S * (sample - offset)|| - radius)² where S is scaling matrix
 *          - Fitness metric: RMS residuals in milligauss after correction
 * 
 *          **Memory Optimization:**
 *          - AttitudeSample: Compressed int8_t storage for roll/pitch/yaw (±180° → ±127)
 *          - CompassSample: Compressed int16_t storage for x/y/z magnetic field
 *          - Allows 300 samples without excessive RAM consumption
 * 
 *          **Sample Distribution Validation:**
 *          - completion_mask_t: Geodesic coverage mask tracking sample distribution
 *          - Ensures samples cover multiple orientations for robust calibration
 *          - Prevents acceptance of calibration with clustered samples
 * 
 *          **Orientation Detection:**
 *          - Automatic detection of compass mounting orientation (ROTATION_NONE, YAW_90, etc.)
 *          - Compares expected earth field direction against measured field across attitudes
 *          - Corrects for incorrect orientation parameter configuration
 * 
 *          **Typical Usage:**
 *          1. start() - Begin calibration with parameters
 *          2. new_sample() - Feed compass readings during vehicle rotation (300+ samples)
 *          3. update() - Call regularly to advance calibration state machine
 *          4. get_report() - Retrieve results (offsets, diagonals, off-diagonals, fitness)
 * 
 * @note Thread-safe: Uses semaphores for sample collection and state updates
 * @note All field measurements in milligauss (mGauss)
 * @note Levenberg-Marquardt lambda parameters control optimization step size and convergence
 * 
 * @warning offset_max threshold affects calibration acceptance - too low rejects valid calibrations
 * @warning tolerance (fitness) threshold affects quality - default 5.0 mGauss RMS is typical
 * @warning Requires vehicle rotation through diverse orientations for reliable results
 * 
 * @see AP_Compass for integration into compass driver framework
 */
class CompassCalibrator {
public:
    /**
     * @brief Constructor initializes calibrator to NOT_STARTED state
     */
    CompassCalibrator();

    /**
     * @brief Start compass calibration process
     * 
     * @details Initiates the calibration state machine and prepares for sample collection.
     *          Allocates sample buffer, resets state, and configures calibration parameters.
     *          After delay expires, transitions to RUNNING_STEP_ONE to begin sphere fit.
     * 
     * @param[in] retry           If true, automatically restart calibration on failure
     * @param[in] delay           Delay in seconds before calibration starts (allows user setup time)
     * @param[in] offset_max      Maximum acceptable offset magnitude in mGauss (typically 600-1000)
     *                            Calibration fails if calculated offsets exceed this threshold
     * @param[in] compass_idx     Index of compass being calibrated (0, 1, 2 for multi-compass systems)
     * @param[in] tolerance       Maximum acceptable RMS fitness in mGauss (default 5.0)
     *                            Lower values require more accurate fits but may reject valid calibrations
     * 
     * @note Requires COMPASS_CAL_NUM_SAMPLES (300) samples before fitting begins
     * @note Call update() regularly to advance calibration state machine
     * 
     * @warning offset_max threshold must account for actual vehicle magnetic environment
     * @warning tolerance too low may cause false failures; too high accepts poor calibrations
     * 
     * @see update() for state machine progression
     * @see new_sample() for sample collection
     */
    void start(bool retry, float delay, uint16_t offset_max, uint8_t compass_idx, float tolerance);
    
    /**
     * @brief Stop calibration and free resources
     * 
     * @details Halts calibration state machine, frees sample buffer, and resets to NOT_STARTED.
     *          Safe to call at any point during calibration.
     */
    void stop();

    /**
     * @brief Accumulate new compass sample for calibration
     * 
     * @details Adds compass reading to sample buffer during calibration. Samples are accepted
     *          based on distribution criteria to ensure adequate angular coverage. Associated
     *          attitude information (from AHRS) is stored with each sample for orientation detection.
     *          
     *          Sample acceptance criteria:
     *          - Must be sufficiently different from existing samples (avoid clustering)
     *          - Contributes to coverage of geodesic sample distribution mask
     *          - Buffer has space available (300 samples maximum in step one)
     * 
     * @param[in] sample   Compass reading vector in mGauss (body frame, uncorrected)
     * 
     * @note Call during RUNNING_STEP_ONE or RUNNING_STEP_TWO states
     * @note Thread-safe: Uses semaphore for sample buffer access
     * @note Current AHRS attitude is automatically captured with each sample
     * 
     * @see accept_sample() for acceptance criteria
     * @see update_completion_mask() for coverage tracking
     */
    void new_sample(const Vector3f& sample);

    /**
     * @brief Configure compass orientation and automatic correction settings
     * 
     * @details Sets the expected compass mounting orientation and enables automatic orientation
     *          detection if configured. Orientation detection compares expected earth field
     *          direction (based on configured ROTATION) against measured field across multiple
     *          attitudes to identify incorrect orientation parameter.
     * 
     *          If fix_orientation is enabled and actual orientation differs from configured,
     *          calibrator will automatically correct the orientation parameter and provide
     *          the corrected rotation in the calibration report.
     * 
     * @param[in] orientation      Expected compass mounting orientation (ROTATION_NONE, ROTATION_YAW_90, etc.)
     * @param[in] is_external      True if compass is external (mounted away from autopilot)
     *                             External compasses more likely to have non-standard orientations
     * @param[in] fix_orientation  True to enable automatic orientation detection and correction
     * @param[in] always_45_deg    True to consider 45-degree rotations with equal tolerance
     *                             Allows detection of ROTATION_YAW_45, ROTATION_PITCH_45, etc.
     * 
     * @note Orientation detection requires samples across multiple vehicle attitudes
     * @note Only right-angle rotations checked unless always_45_deg is true
     * 
     * @see calculate_orientation() for automatic detection algorithm
     * @see auto_rotation_index() for rotation enumeration
     */
    void set_orientation(enum Rotation orientation, bool is_external, bool fix_orientation, bool always_45_deg);

    /**
     * @brief Check if calibration is actively running
     * 
     * @return true if in RUNNING_STEP_ONE or RUNNING_STEP_TWO (actively fitting parameters)
     * @return false if NOT_STARTED, WAITING_TO_START, SUCCESS, or any failure state
     * 
     * @note Returns true during both sphere fit and ellipsoid fit phases
     */
    bool running();

    /**
     * @brief Check if calibration has failed
     * 
     * @return true if status is FAILED, BAD_ORIENTATION, or BAD_RADIUS
     * @return false for all other states including SUCCESS and in-progress states
     * 
     * @see Status enum for failure conditions
     */
    bool failed();


    /**
     * @brief Update calibration state machine and perform fitting calculations
     * 
     * @details Advances calibration through state transitions and executes fitting algorithms.
     *          Must be called regularly (typically at 10-50 Hz) to progress calibration.
     * 
     *          State machine sequence:
     *          1. NOT_STARTED → WAITING_TO_START: After start() called, waiting for delay
     *          2. WAITING_TO_START → RUNNING_STEP_ONE: Delay expired, begin sample collection
     *          3. RUNNING_STEP_ONE: Collect 300 samples, then run sphere fit (Levenberg-Marquardt)
     *             - Iteratively optimizes radius and offsets
     *             - Checks fit acceptance criteria (fitness < tolerance, offsets < offset_max)
     *             - On success → RUNNING_STEP_TWO, on failure → FAILED or retry
     *          4. RUNNING_STEP_TWO: Thin samples, then run ellipsoid fit (Levenberg-Marquardt)
     *             - Iteratively optimizes offsets, diagonals, and off-diagonals
     *             - Checks fit acceptance criteria
     *             - On success → SUCCESS, on failure → FAILED or retry
     * 
     *          Fitting algorithm (Levenberg-Marquardt):
     *          - Calculates Jacobian matrix for parameter sensitivity
     *          - Applies damping factor (lambda) to balance gradient descent and Gauss-Newton
     *          - Iterates until convergence or maximum iterations reached
     *          - Lambda increases on poor steps (more gradient descent), decreases on good steps
     * 
     * @note Call at regular intervals (10-50 Hz recommended)
     * @note Fitting calculations may take multiple update() calls to converge
     * @note Thread-safe: Uses semaphores for state access
     * 
     * @see run_sphere_fit() for STEP ONE algorithm
     * @see run_ellipsoid_fit() for STEP TWO algorithm
     * @see fit_acceptable() for acceptance criteria
     */
    void update();

    /**
     * @brief Compass calibration state enumeration
     * 
     * @details Status values correspond directly to MAVLink MAG_CAL_STATUS enumeration
     *          for ground control station reporting and monitoring.
     * 
     * @note Values match MAVLink protocol definition - do not modify
     */
    enum class Status {
        NOT_STARTED = 0,       ///< Calibration not yet initiated
        WAITING_TO_START = 1,  ///< Delay period before sample collection begins
        RUNNING_STEP_ONE = 2,  ///< Collecting samples and running sphere fit (hard-iron correction)
        RUNNING_STEP_TWO = 3,  ///< Running ellipsoid fit (soft-iron correction with diagonals/off-diagonals)
        SUCCESS = 4,           ///< Calibration completed successfully, parameters valid
        FAILED = 5,            ///< Calibration failed - fitness exceeded tolerance or other error
        BAD_ORIENTATION = 6,   ///< Detected compass orientation doesn't match configured orientation
        BAD_RADIUS = 7,        ///< Calculated field radius outside acceptable range (magnetic anomaly or sensor issue)
    };

    /**
     * @brief Geodesic coverage mask tracking sample distribution across orientations
     * 
     * @details Bitmask array representing sample coverage across different vehicle orientations.
     *          Each bit corresponds to a specific face/direction in a geodesic subdivision of
     *          the sampling sphere. Ensures samples are well-distributed rather than clustered.
     * 
     * @note Used for MAVLink MAG_CAL_REPORT completion_mask field
     * @note Array of 10 uint8_t provides 80 bits for geodesic face tracking
     */
    typedef uint8_t completion_mask_t[10];

    /**
     * @brief Calibration state structure for real-time status reporting
     * 
     * @details Provides current calibration progress and status information for MAVLink
     *          reporting to ground control stations. Updated regularly during calibration.
     */
    struct State {
        Status status;                    ///< Current calibration state (NOT_STARTED through BAD_RADIUS)
        uint8_t attempt;                  ///< Current attempt number (increments on retry after failure)
        float completion_pct;             ///< Percentage of required samples collected (0.0-100.0)
        completion_mask_t completion_mask; ///< Bitmask of geodesic coverage for sample distribution quality
    } cal_state;

    /**
     * @brief Calibration results structure containing final parameters
     * 
     * @details Contains calibration output parameters and diagnostic information.
     *          Retrieved via get_report() after calibration completes (SUCCESS or failure).
     * 
     * @note All magnetic field values in milligauss (mGauss)
     * @note Offsets applied as: corrected = (raw - ofs)
     * @note Scaling applied as: corrected = diag * (raw - ofs) + offdiag cross terms
     */
    struct Report {
        Status status;                    ///< Final calibration status (SUCCESS, FAILED, BAD_ORIENTATION, BAD_RADIUS)
        float fitness;                    ///< RMS residual error in mGauss (lower is better, typically < 5.0)
        Vector3f ofs;                     ///< Hard-iron offset correction in mGauss (body frame x/y/z)
        Vector3f diag;                    ///< Soft-iron diagonal scaling factors (dimensionless, typically near 1.0)
        Vector3f offdiag;                 ///< Soft-iron off-diagonal cross-coupling corrections (dimensionless)
        float orientation_confidence;     ///< Confidence in automatic orientation detection (0.0-1.0, higher is better)
        Rotation original_orientation;    ///< Orientation configured before calibration
        Rotation orientation;             ///< Detected/corrected orientation after calibration
        float scale_factor;               ///< Sensor scaling correction factor (dimensionless, typically near 1.0)
        bool check_orientation;           ///< True if orientation detection was enabled during calibration
    } cal_report;

    /**
     * @brief Calibration configuration settings
     * 
     * @details Internal structure holding calibration parameters configured via start()
     *          and set_orientation(). Maintains settings throughout calibration process.
     */
    struct Settings {
        float tolerance;                  ///< Maximum acceptable RMS fitness in mGauss (typically 5.0)
        bool check_orientation;           ///< True to enable automatic orientation detection
        enum Rotation orientation;        ///< Current/detected compass mounting orientation
        enum Rotation orig_orientation;   ///< Original configured orientation before auto-detection
        bool is_external;                 ///< True if compass is external (affects orientation detection)
        bool fix_orientation;             ///< True to automatically correct orientation if mismatch detected
        uint16_t offset_max;              ///< Maximum acceptable offset magnitude in mGauss (typically 600-1000)
        uint8_t attempt;                  ///< Current calibration attempt number
        bool retry;                       ///< True to automatically restart calibration on failure
        float delay_start_sec;            ///< Delay in seconds before calibration begins
        uint32_t start_time_ms;           ///< System time in milliseconds when start() was called
        uint8_t compass_idx;              ///< Index of compass being calibrated (0, 1, 2)
        bool always_45_deg;               ///< True to consider 45-degree rotations in orientation detection
    } cal_settings;

    /**
     * @brief Retrieve final calibration results
     * 
     * @return Report structure containing offsets, diagonals, off-diagonals, fitness, and status
     * 
     * @note Call after calibration completes (SUCCESS state) to retrieve parameters
     * @note Thread-safe: Uses semaphore for report structure access
     * 
     * @see Report structure for field descriptions
     */
    const Report get_report();
    
    /**
     * @brief Retrieve current calibration state for progress monitoring
     * 
     * @return State structure with current status, attempt number, completion percentage, and coverage mask
     * 
     * @note Call regularly to monitor calibration progress for user feedback
     * @note Thread-safe: Uses semaphore for state structure access
     * @note Used for MAVLink MAG_CAL_PROGRESS and MAG_CAL_REPORT messages
     * 
     * @see State structure for field descriptions
     */
    const State get_state();

protected:
    /**
     * @brief Convert sequential index to rotation enumeration
     * 
     * @details Maps sequential index (0, 1, 2...) to Rotation enum values, allowing iteration
     *          through rotations while optionally skipping non-right-angle rotations.
     *          Used during automatic orientation detection to test multiple rotation hypotheses.
     * 
     * @param[in] n  Sequential index (0-based)
     * @return Corresponding Rotation enum value
     * 
     * @note Protected visibility allows unit testing (CompassCalibrator_index_test)
     * @note Skips some rotations if always_45_deg is false (only tests right angles)
     * 
     * @see calculate_orientation() for usage in orientation detection
     */
    Rotation auto_rotation_index(uint8_t n) const;

    /**
     * @brief Check if rotation is a right angle (90°, 180°, 270°)
     * 
     * @param[in] r  Rotation to check
     * @return true if rotation is right-angle (multiples of 90°), false for 45° or other angles
     * 
     * @note Used to filter rotations during orientation detection if always_45_deg is false
     */
    bool right_angle_rotation(Rotation r) const;

private:

    /**
     * @brief Calibration parameter container for optimization results
     * 
     * @details Holds current calibration parameters during Levenberg-Marquardt fitting.
     *          Provides pointers to parameter arrays for sphere fit (4 params) and
     *          ellipsoid fit (9 params) optimization routines.
     */
    class param_t {
    public:
        /**
         * @brief Get pointer to sphere fit parameters
         * @return Pointer to 4-element array: [radius, offset.x, offset.y, offset.z]
         */
        float* get_sphere_params() {
            return &radius;
        }

        /**
         * @brief Get pointer to ellipsoid fit parameters
         * @return Pointer to 9-element array: [offset.x, offset.y, offset.z, diag.x, diag.y, diag.z, offdiag.x, offdiag.y, offdiag.z]
         */
        float* get_ellipsoid_params() {
            return &offset.x;
        }

        float radius;       ///< Magnetic field strength in mGauss (expected magnitude after correction)
        Vector3f offset;    ///< Hard-iron offset corrections in mGauss (body frame x/y/z)
        Vector3f diag;      ///< Soft-iron diagonal scaling factors (dimensionless, primary axes)
        Vector3f offdiag;   ///< Soft-iron off-diagonal scaling (dimensionless, cross-coupling)
        float scale_factor; ///< Sensor scaling correction factor to compensate for radius errors
    };

    /**
     * @brief Compressed attitude storage for memory efficiency
     * 
     * @details Stores vehicle attitude in compressed int8_t format rather than float
     *          to minimize RAM usage for 300 samples. Provides adequate precision
     *          (±180° → ±127 → ~1.4° resolution) for orientation detection.
     * 
     * @note Memory savings: 3 bytes vs 12 bytes for float storage
     * @note Roll/pitch/yaw stored as degrees * 127/180 (int8_t range)
     */
    class AttitudeSample {
    public:
        /**
         * @brief Convert compressed attitude to rotation matrix
         * @return 3x3 rotation matrix from body frame to earth frame
         */
        Matrix3f get_rotmat() const;
        
        /**
         * @brief Capture current vehicle attitude from AHRS
         * @note Compresses float attitude (degrees) to int8_t for storage
         */
        void set_from_ahrs();
    private:
        int8_t roll;   ///< Roll angle compressed: degrees * 127/180
        int8_t pitch;  ///< Pitch angle compressed: degrees * 127/180
        int8_t yaw;    ///< Yaw angle compressed: degrees * 127/180
    };

    /**
     * @brief Compressed compass sample storage for memory efficiency
     * 
     * @details Stores compass reading and associated attitude in compressed format
     *          to minimize RAM usage for 300 samples. Uses int16_t for magnetic field
     *          (adequate for milligauss range) and AttitudeSample for attitude.
     * 
     * @note Memory savings: 9 bytes vs 24 bytes for full float storage
     * @note Magnetic field stored as int16_t milligauss (±32767 mGauss range)
     */
    class CompassSample {
    public:
        /**
         * @brief Retrieve compass reading as Vector3f
         * @return Compass reading in mGauss (body frame, uncompressed)
         */
        Vector3f get() const;
        
        /**
         * @brief Store compass reading in compressed format
         * @param[in] in  Compass reading vector in mGauss
         */
        void set(const Vector3f &in);
        
        AttitudeSample att;  ///< Associated vehicle attitude when sample was taken
    private:
        int16_t x;  ///< Compass X-axis reading in mGauss (compressed to int16_t)
        int16_t y;  ///< Compass Y-axis reading in mGauss (compressed to int16_t)
        int16_t z;  ///< Compass Z-axis reading in mGauss (compressed to int16_t)
    };

    /**
     * @brief Set calibration status with required state initialization
     * 
     * @param[in] status  New status to transition to
     * @return true if status change successful, false otherwise
     * 
     * @note Performs state-specific initialization (e.g., allocating buffers, resetting counters)
     */
    bool set_status(Status status);

    /**
     * @brief Retrieve pending sample from intermediate buffer
     * 
     * @details Thread-safe sample retrieval from new_sample() intermediate storage.
     *          Transfers sample to main calibration buffer if acceptance criteria met.
     */
    void pull_sample();

    /**
     * @brief Check if sample should be added to calibration buffer
     * 
     * @details Evaluates sample against acceptance criteria:
     *          - Sufficient angular separation from existing samples
     *          - Contributes to coverage mask improvement
     *          - Not redundant with existing buffer contents
     * 
     * @param[in] sample      Compass reading to evaluate (Vector3f or CompassSample)
     * @param[in] skip_index  Optional index to skip during comparison (UINT16_MAX = no skip)
     * @return true if sample accepted and added to buffer, false if rejected
     * 
     * @note Prevents clustering of samples in similar orientations
     * @note Ensures diverse sample distribution for robust calibration
     */
    bool accept_sample(const Vector3f &sample, uint16_t skip_index = UINT16_MAX);
    bool accept_sample(const CompassSample &sample, uint16_t skip_index = UINT16_MAX);

    /**
     * @brief Check if current fit meets acceptance criteria
     * 
     * @return true if fitness < tolerance AND offsets < offset_max, false otherwise
     * 
     * @note Called after each fit iteration to determine success
     * @see fit_acceptable() criteria: RMS residuals and offset magnitude thresholds
     */
    bool fit_acceptable() const;

    /**
     * @brief Clear sample buffer and reset parameters to defaults
     * 
     * @details Resets sample buffer, completion mask, and parameters to initial state.
     *          Called when starting new calibration or after failure with retry enabled.
     */
    void reset_state();

    /**
     * @brief Initialize fitness tracking before starting Levenberg-Marquardt fit
     * 
     * @details Calculates initial fitness (RMS residuals) and stores as baseline
     *          for detecting fit improvement during optimization iterations.
     */
    void initialize_fit();

    /**
     * @brief Check if actively fitting parameters
     * 
     * @return true if enough samples collected and Levenberg-Marquardt fitting in progress
     * 
     * @note Internal version of running() for use within calibrator
     */
    bool _fitting() const;

    /**
     * @brief Reduce sample count before ellipsoid fit
     * 
     * @details Removes redundant samples after sphere fit to reduce computation time
     *          for ellipsoid fit while preserving sample distribution quality.
     *          Typically reduces from 300 samples to fewer while maintaining coverage.
     * 
     * @note Called between STEP_ONE and STEP_TWO
     * @note Removes samples that are too close to other samples (clustering)
     */
    void thin_samples();

    /**
     * @brief Calculate residual error for single sample
     * 
     * @details Computes difference between measured field magnitude and expected radius
     *          after applying offset and scaling corrections. Residual represents
     *          calibration error for this sample.
     * 
     * @param[in] sample  Compass reading in mGauss (body frame, uncorrected)
     * @param[in] params  Calibration parameters to evaluate (offsets, diagonals, off-diagonals)
     * @return Residual error in mGauss (||corrected_sample|| - radius)
     * 
     * @note Lower residuals indicate better fit
     * @note Corrected sample: S * (sample - offset) where S is scaling matrix
     */
    float calc_residual(const Vector3f& sample, const param_t& params) const;

    /**
     * @brief Calculate fitness metric for parameter set
     * 
     * @details Computes RMS (root mean squared) residuals across all samples in buffer.
     *          This is the primary fitness metric minimized by Levenberg-Marquardt optimization.
     * 
     *          Fitness = sqrt( Σ(residual²) / n_samples )
     * 
     * @param[in] params  Calibration parameters to evaluate
     * @return RMS residual in mGauss, or 1.0e30f if sample buffer empty
     * 
     * @note Lower fitness indicates better calibration
     * @note Typical good fitness: < 5.0 mGauss RMS
     * @note Used in fit_acceptable() to check against tolerance threshold
     */
    float calc_mean_squared_residuals(const param_t& params) const;

    /**
     * @brief Calculate initial offset estimate from sample mean
     * 
     * @details Computes average of all samples as initial offset guess for Levenberg-Marquardt.
     *          Provides reasonable starting point assuming samples roughly centered around true center.
     * 
     * @note Called at beginning of sphere fit to initialize offset parameters
     */
    void calc_initial_offset();

    /**
     * @brief Calculate Jacobian matrix for sphere fit
     * 
     * @details Computes partial derivatives of residual function with respect to sphere
     *          fit parameters (radius, offset.x, offset.y, offset.z). Jacobian used in
     *          Levenberg-Marquardt to determine parameter update direction and magnitude.
     * 
     * @param[in]  sample  Compass reading for which to calculate Jacobian
     * @param[in]  params  Current parameter values
     * @param[out] ret     4-element array to store Jacobian: [∂r/∂radius, ∂r/∂ox, ∂r/∂oy, ∂r/∂oz]
     * 
     * @note Jacobian represents sensitivity of residual to each parameter
     */
    void calc_sphere_jacob(const Vector3f& sample, const param_t& params, float* ret) const;
    
    /**
     * @brief Execute sphere fit using Levenberg-Marquardt optimization
     * 
     * @details Iteratively solves for magnetic field radius and 3D offset vector
     *          by minimizing RMS residuals across all samples. Assumes spherical
     *          field distribution (constant magnitude after offset correction).
     * 
     *          Algorithm:
     *          1. Calculate Jacobian matrix for all samples
     *          2. Form normal equations: (J'J + λI)Δp = -J'r
     *          3. Solve for parameter update Δp
     *          4. If fitness improves: Accept update, decrease λ (more Gauss-Newton)
     *          5. If fitness worsens: Reject update, increase λ (more gradient descent)
     *          6. Repeat until convergence or max iterations
     * 
     * @note Called during RUNNING_STEP_ONE state
     * @note Solves for 4 parameters: radius + 3 offsets
     * @note Lambda (_sphere_lambda) controls optimization behavior
     * 
     * @see calc_sphere_jacob() for Jacobian calculation
     * @see calc_mean_squared_residuals() for fitness evaluation
     */
    void run_sphere_fit();

    /**
     * @brief Calculate Jacobian matrix for ellipsoid fit
     * 
     * @details Computes partial derivatives of residual function with respect to
     *          ellipsoid fit parameters (3 offsets + 3 diagonals + 3 off-diagonals).
     *          Jacobian used in Levenberg-Marquardt for parameter optimization.
     * 
     * @param[in]  sample  Compass reading for which to calculate Jacobian
     * @param[in]  params  Current parameter values
     * @param[out] ret     9-element array for Jacobian: [∂r/∂ox, ∂r/∂oy, ∂r/∂oz, ∂r/∂dx, ∂r/∂dy, ∂r/∂dz, ∂r/∂odx, ∂r/∂ody, ∂r/∂odz]
     * 
     * @note More complex than sphere Jacobian due to scaling matrix S
     */
    void calc_ellipsoid_jacob(const Vector3f& sample, const param_t& params, float* ret) const;
    
    /**
     * @brief Execute ellipsoid fit using Levenberg-Marquardt optimization
     * 
     * @details Iteratively solves for offsets, diagonal scaling, and off-diagonal
     *          scaling to model soft-iron effects. Models field as ellipsoid rather
     *          than sphere to compensate for directionally-dependent distortions.
     * 
     *          Algorithm: Same Levenberg-Marquardt as sphere fit but with 9 parameters
     *          Scaling matrix: S = [diag.x    offdiag.x  offdiag.y ]
     *                              [offdiag.x diag.y     offdiag.z ]
     *                              [offdiag.y offdiag.z  diag.z    ]
     * 
     *          Corrected field: S * (raw - offset)
     *          Fitness: RMS of (||S * (raw - offset)|| - radius)
     * 
     * @note Called during RUNNING_STEP_TWO state
     * @note Solves for 9 parameters: 3 offsets + 3 diagonals + 3 off-diagonals
     * @note Lambda (_ellipsoid_lambda) controls optimization behavior
     * @note More computationally intensive than sphere fit
     * 
     * @see calc_ellipsoid_jacob() for Jacobian calculation
     * @see thin_samples() called before ellipsoid fit to reduce computation
     */
    void run_ellipsoid_fit();

    /**
     * @brief Update coverage mask for single sample
     * 
     * @details Marks geodesic region corresponding to sample's orientation in coverage mask.
     *          Tracks sample distribution across different vehicle attitudes to ensure
     *          adequate angular coverage for robust calibration.
     * 
     * @param[in] sample  Compass reading vector to add to coverage tracking
     * 
     * @note Coverage mask used for completion_pct calculation and quality assessment
     */
    void update_completion_mask(const Vector3f& sample);

    /**
     * @brief Recalculate coverage mask from all samples in buffer
     * 
     * @details Resets and rebuilds completion mask by processing entire sample buffer.
     *          Called after sample thinning or when buffer contents change significantly.
     * 
     * @note More expensive than single-sample update; use only when necessary
     */
    void update_completion_mask();

    /**
     * @brief Calculate expected earth field for sample given rotation hypothesis
     * 
     * @details Rotates measured sample by specified rotation and compares against
     *          expected earth field direction based on vehicle attitude. Used during
     *          automatic orientation detection to test rotation hypotheses.
     * 
     * @param[in] sample  Compass sample with associated attitude
     * @param[in] r       Rotation hypothesis to test
     * @return Expected earth field vector in NED frame for this rotation
     * 
     * @note Earth field direction determined from AHRS attitude and sample
     */
    Vector3f calculate_earth_field(CompassSample &sample, enum Rotation r);
    
    /**
     * @brief Automatically detect compass mounting orientation
     * 
     * @details Tests multiple rotation hypotheses to find best match between measured
     *          field and expected earth field across all samples and attitudes.
     *          Detects when configured orientation parameter is incorrect.
     * 
     *          Algorithm:
     *          1. For each possible rotation in auto_rotation_index()
     *          2. Calculate expected earth field for all samples given this rotation
     *          3. Compare against actual measured earth field direction
     *          4. Score rotation by consistency across samples
     *          5. Select rotation with best consistency score
     *          6. Set orientation_confidence based on score margin
     * 
     * @return true if confident orientation detected, false if ambiguous or failed
     * 
     * @note Requires samples across multiple vehicle attitudes for reliable detection
     * @note Updates _orientation_solution and _orientation_confidence
     * @note Only tests rotations allowed by always_45_deg setting
     * 
     * @see auto_rotation_index() for rotation enumeration
     * @see calculate_earth_field() for field comparison
     */
    bool calculate_orientation();

    /**
     * @brief Apply radius correction to compensate for sensor scaling errors
     * 
     * @details Calculates scale_factor to adjust radius if field strength significantly
     *          differs from expected earth field magnitude. Compensates for magnetometer
     *          sensitivity errors without requiring complete recalibration.
     * 
     * @return true if radius correction successful, false if radius anomalous
     * 
     * @note Called at end of successful calibration
     * @note Sets status to BAD_RADIUS if field strength is far from earth field norm
     */
    bool fix_radius();

    /**
     * @brief Synchronize calibration status to cal_state structure
     * 
     * @details Thread-safe update of cal_state structure for external access via get_state().
     *          Uses semaphore to protect concurrent access during state queries.
     * 
     * @note Called internally during update() to maintain current state for reporting
     */
    inline void update_cal_status();
    
    /**
     * @brief Synchronize calibration results to cal_report structure
     * 
     * @details Thread-safe update of cal_report structure for external access via get_report().
     *          Copies current parameters, fitness, and orientation information.
     * 
     * @note Called after calibration completes (SUCCESS or failure)
     */
    inline void update_cal_report();
    
    /**
     * @brief Synchronize configuration settings to cal_settings structure
     * 
     * @details Thread-safe update of cal_settings structure from internal state.
     *          Maintains configuration visibility for monitoring and diagnostics.
     */
    inline void update_cal_settings();

    /**
     * @brief Internal running state check for thread-safe operation
     * 
     * @return true if actively running calibration (RUNNING_STEP_ONE or RUNNING_STEP_TWO)
     * 
     * @note Thread-safe version of running() for internal use
     */
    bool _running() const;

    // ===== Configuration Parameters =====
    uint8_t _compass_idx;                   ///< Index of compass being calibrated (0, 1, 2 for multi-compass systems)
    Status _status;                         ///< Current calibration state (NOT_STARTED through BAD_RADIUS)

    // Values provided by caller via start()
    float _delay_start_sec;                 ///< Delay in seconds before calibration begins (allows user setup time)
    bool _retry;                            ///< True to automatically restart calibration on failure
    float _tolerance = 5.0;                 ///< Maximum acceptable RMS fitness in mGauss (default 5.0)
    uint16_t _offset_max;                   ///< Maximum acceptable offset magnitude in mGauss (typically 600-1000)

    // ===== Sample Collection State =====
    uint32_t _start_time_ms;                ///< System time in milliseconds when start() was called
    uint8_t _attempt;                       ///< Current calibration attempt number (increments on retry)
    completion_mask_t _completion_mask;     ///< Geodesic coverage bitmask tracking sample distribution
    CompassSample *_sample_buffer;          ///< Dynamic buffer holding up to 300 compass samples with attitudes
    uint16_t _samples_collected;            ///< Number of samples currently in buffer
    uint16_t _samples_thinned;              ///< Number of samples removed by thin_samples() before STEP_TWO

    // ===== Levenberg-Marquardt Fit State =====
    class param_t _params;                  ///< Current calibration parameters (offsets, diagonals, off-diagonals, radius)
    uint16_t _fit_step;                     ///< Current iteration step in Levenberg-Marquardt optimization
    float _fitness;                         ///< Current RMS residual fitness in mGauss
    float _initial_fitness;                 ///< Fitness before current fit iteration (for improvement detection)
    float _sphere_lambda;                   ///< Levenberg-Marquardt damping parameter for sphere fit (controls Gauss-Newton vs gradient descent)
    float _ellipsoid_lambda;                ///< Levenberg-Marquardt damping parameter for ellipsoid fit

    // ===== Orientation Detection State =====
    enum Rotation _orientation;             ///< Currently detected compass mounting orientation
    enum Rotation _orig_orientation;        ///< Original orientation configured before auto-detection
    enum Rotation _orientation_solution;    ///< Best rotation solution from calculate_orientation()
    bool _is_external;                      ///< True if compass is external (away from autopilot)
    bool _check_orientation;                ///< True to enable automatic orientation detection
    bool _fix_orientation;                  ///< True to automatically correct orientation if mismatch detected
    bool _always_45_deg;                    ///< True to consider 45-degree rotations in orientation detection
    float _orientation_confidence;          ///< Confidence score for detected orientation (0.0-1.0, higher is better)
    CompassSample _last_sample;             ///< Most recent sample for state tracking

    // ===== Thread Synchronization =====
    Status _requested_status;               ///< Pending status change requested from external thread
    bool   _status_set_requested;           ///< True if status change is pending
    bool _new_sample;                       ///< True if new sample is pending in intermediate buffer

    HAL_Semaphore state_sem;                ///< Semaphore protecting cal_state, cal_report, cal_settings structures
    HAL_Semaphore sample_sem;               ///< Semaphore protecting sample collection intermediate buffer
};

#endif  // COMPASS_CAL_ENABLED
